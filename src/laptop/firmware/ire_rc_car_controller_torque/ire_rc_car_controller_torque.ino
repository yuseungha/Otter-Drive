#include <Servo.h>

// ============================================================
// 핀 설정: 네 마지막 코드의 실제 배선 기준
// ============================================================
constexpr uint8_t ESC_PIN            = 9;
// Uno에서는 Servo 라이브러리가 Timer1을 사용하므로 D9/D10 PWM을 피해야 함.
// 조향 ENA는 PWM 가능한 D5에 연결한다.
constexpr uint8_t STEER_ENA_PIN      = 5;
constexpr uint8_t STEER_IN1_PIN      = 12;
constexpr uint8_t STEER_IN2_PIN      = 13;
constexpr uint8_t STEER_FEEDBACK_PIN = A5;

// ============================================================
// 조향 캘리브레이션
//
// 2026-08-13 최종 실측값:
// 기계적 왼쪽 끝 = 762, 안전 왼쪽 목표 = 747
// 중앙            = 602
// 기계적 오른쪽 끝 = 447, 안전 오른쪽 목표 = 462
//
// D 조향 +/-1000은 기계적 끝이 아니라 안전 목표에 매핑한다.
// ============================================================
constexpr int STEER_ADC_LEFT   = 747;
constexpr int STEER_ADC_CENTER = 602;
constexpr int STEER_ADC_RIGHT  = 462;

constexpr int STEER_ADC_MECHANICAL_LEFT  = 762;
constexpr int STEER_ADC_MECHANICAL_RIGHT = 447;

constexpr int STEER_DEADBAND = 7;

// 안전 목표를 넘어서 기계적 끝에 도달하면 오류를 래치한다.
constexpr int STEER_ADC_SOFT_MAX = STEER_ADC_MECHANICAL_LEFT;
constexpr int STEER_ADC_SOFT_MIN = STEER_ADC_MECHANICAL_RIGHT;

// 센서 단선 또는 쇼트 검출용
constexpr int STEER_ADC_VALID_MIN = 50;
constexpr int STEER_ADC_VALID_MAX = 973;

// 처음 테스트할 때는 출력을 낮게 잡음
// 정상 확인 후 MAX_PWM을 130~170 정도로 올려도 됨
constexpr int STEER_MIN_PWM = 140;
constexpr int STEER_MAX_PWM = 180;

static_assert(
  STEER_ADC_MECHANICAL_LEFT > STEER_ADC_LEFT &&
  STEER_ADC_LEFT > STEER_ADC_CENTER &&
  STEER_ADC_CENTER > STEER_ADC_RIGHT &&
  STEER_ADC_RIGHT > STEER_ADC_MECHANICAL_RIGHT &&
  STEER_ADC_MECHANICAL_LEFT - STEER_ADC_LEFT == 15 &&
  STEER_ADC_RIGHT - STEER_ADC_MECHANICAL_RIGHT == 15,
  "Steering safety targets must remain inside the measured mechanical ends"
);

// ============================================================
// 중요
//
// true:
// IN1=HIGH, IN2=LOW일 때 ADC 값이 증가해야 함.
//
// 만약 WRONG_DIRECTION 오류가 뜨면 true ↔ false로 바꾸면 됨.
// ============================================================
constexpr bool HIGH_LOW_INCREASES_ADC = false;

// ============================================================
// ESC 설정
// ============================================================
constexpr int ESC_NEUTRAL_US     = 1500;
constexpr int ESC_FORWARD_MAX_US = 1620;
constexpr int ESC_REVERSE_MAX_US = 1350;

constexpr unsigned long ESC_ARM_TIME_MS   = 3000;
// Normalized ROS/serial protocol range. +1000 maps to 1620 us and -1000 maps
// to 1350 us; these are the physically qualified competition limits.
constexpr int THROTTLE_COMMAND_MIN = -1000;
constexpr int THROTTLE_COMMAND_MAX = 1000;
// The installed ESC requires the reverse double-pump sequence. A negative D
// command must remain fresh throughout both stages before reverse is applied.
constexpr unsigned long ESC_REVERSE_ENTRY_PULSE_MS = 250;
constexpr unsigned long ESC_REVERSE_ENTRY_NEUTRAL_MS = 250;
constexpr unsigned long ESC_FORWARD_ENTRY_NEUTRAL_MS = 250;
// Independent Arduino-side fail-safe. ROS and the serial bridge normally send
// at 20 Hz; 300 ms permits five missed frames before forcing neutral.
constexpr unsigned long COMMAND_TIMEOUT_MS = 300;
constexpr char FIRMWARE_CONFIG_BANNER[] =
  "[CONFIG] IRE_TORQUE_20260820_ADC747_602_462_ESC1620_1500_1350_WD300_PWM140_180";

static_assert(
  ESC_REVERSE_MAX_US < ESC_NEUTRAL_US &&
  ESC_NEUTRAL_US < ESC_FORWARD_MAX_US,
  "ESC reverse, neutral, and forward pulses must be strictly ordered"
);

// ============================================================
// 조향 이상동작 감시
// ============================================================
constexpr unsigned long MOTION_CHECK_INTERVAL_MS = 100;

// 반대 방향으로 이 값 이상 움직이면 잘못된 방향으로 판단
constexpr int WRONG_DIRECTION_DELTA = 3;

// 오차가 큰데 이 값 이하로만 움직이면 정지 상태로 판단
constexpr int STALL_DELTA = 1;
constexpr int STALL_ERROR_THRESHOLD = 25;

constexpr uint8_t WRONG_DIRECTION_LIMIT = 2;
constexpr uint8_t STALL_COUNT_LIMIT     = 8;

// ============================================================

Servo esc;

enum class SteeringDrive : uint8_t {
  STOP,
  INCREASE_ADC,
  DECREASE_ADC
};

enum class EscReverseEntryState : uint8_t {
  IDLE,
  REVERSE_PULSE,
  NEUTRAL_WAIT
};

SteeringDrive steeringDrive = SteeringDrive::STOP;
EscReverseEntryState escReverseEntryState = EscReverseEntryState::IDLE;

int targetThrottle = 0;
int appliedThrottle = 0;
int targetSteering = STEER_ADC_CENTER;

int currentSteering = STEER_ADC_CENTER;
int currentError = 0;
int currentSteeringPwm = 0;

bool escArmed = false;
bool steeringEnabled = false;
bool steeringFault = false;

unsigned long lastDebugPrintMs = 0;
unsigned long lastMotionCheckMs = 0;
unsigned long lastCommandMs = 0;
unsigned long escNeutralUntilMs = 0;
unsigned long escReverseEntryDeadlineMs = 0;

bool driveCommandActive = false;

int previousMotionAdc = STEER_ADC_CENTER;
uint8_t wrongDirectionCount = 0;
uint8_t stallCount = 0;

void setEsc(int command);
void stopAllOutputs();

bool deadlinePending(unsigned long now, unsigned long deadline) {
  return static_cast<long>(deadline - now) > 0;
}

// 시리얼 수신 버퍼
char receiveBuffer[48];
uint8_t receiveLength = 0;
bool receiveDiscardUntilNewline = false;

// ============================================================
// 조향 센서값 읽기
// 여러 번 읽어서 평균을 내 노이즈를 줄임
// ============================================================
int readSteeringAdc() {
  long sum = 0;

  // ADC 멀티플렉서 안정화를 위한 첫 값 버림
  analogRead(STEER_FEEDBACK_PIN);

  for (uint8_t i = 0; i < 8; i++) {
    sum += analogRead(STEER_FEEDBACK_PIN);
  }

  return static_cast<int>(sum / 8);
}

// ============================================================
// 조향 정지
// ============================================================
void stopSteering() {
  analogWrite(STEER_ENA_PIN, 0);

  digitalWrite(STEER_IN1_PIN, LOW);
  digitalWrite(STEER_IN2_PIN, LOW);

  steeringDrive = SteeringDrive::STOP;
  currentSteeringPwm = 0;
}

// ============================================================
// 조향 오류 발생
// ============================================================
void triggerSteeringFault(const char* reason) {
  steeringFault = true;
  stopAllOutputs();

  Serial.print(F("[STEERING FAULT] "));
  Serial.println(reason);
}

// ============================================================
// 조향 모터 구동
// ============================================================
void driveSteering(
  SteeringDrive requestedDrive,
  int pwm,
  int measuredAdc
) {
  if (steeringFault || requestedDrive == SteeringDrive::STOP) {
    stopSteering();
    return;
  }

  pwm = constrain(pwm, STEER_MIN_PWM, STEER_MAX_PWM);

  // 반대 방향으로 즉시 전환할 때 짧게 정지
  if (
    steeringDrive != SteeringDrive::STOP &&
    steeringDrive != requestedDrive
  ) {
    analogWrite(STEER_ENA_PIN, 0);
    digitalWrite(STEER_IN1_PIN, LOW);
    digitalWrite(STEER_IN2_PIN, LOW);

    delayMicroseconds(300);
  }

  // 방향이 변경됐다면 감시 기준값 초기화
  if (steeringDrive != requestedDrive) {
    previousMotionAdc = measuredAdc;
    lastMotionCheckMs = millis();

    wrongDirectionCount = 0;
    stallCount = 0;
  }

  bool in1High = false;

  if (requestedDrive == SteeringDrive::INCREASE_ADC) {
    in1High = HIGH_LOW_INCREASES_ADC;
  } else {
    in1High = !HIGH_LOW_INCREASES_ADC;
  }

  if (in1High) {
    digitalWrite(STEER_IN1_PIN, HIGH);
    digitalWrite(STEER_IN2_PIN, LOW);
  } else {
    digitalWrite(STEER_IN1_PIN, LOW);
    digitalWrite(STEER_IN2_PIN, HIGH);
  }

  analogWrite(STEER_ENA_PIN, pwm);

  steeringDrive = requestedDrive;
  currentSteeringPwm = pwm;
}

// ============================================================
// 모터가 목표 반대 방향으로 가는지 감시
// ============================================================
void monitorSteeringMotion(int measuredAdc, int error) {
  if (steeringDrive == SteeringDrive::STOP) {
    return;
  }

  const unsigned long now = millis();

  if (now - lastMotionCheckMs < MOTION_CHECK_INTERVAL_MS) {
    return;
  }

  const int delta = measuredAdc - previousMotionAdc;

  bool movingWrongWay = false;

  if (
    steeringDrive == SteeringDrive::INCREASE_ADC &&
    delta <= -WRONG_DIRECTION_DELTA
  ) {
    movingWrongWay = true;
  }

  if (
    steeringDrive == SteeringDrive::DECREASE_ADC &&
    delta >= WRONG_DIRECTION_DELTA
  ) {
    movingWrongWay = true;
  }

  if (movingWrongWay) {
    wrongDirectionCount++;
  } else if (abs(delta) >= WRONG_DIRECTION_DELTA) {
    wrongDirectionCount = 0;
  }

  // 오차가 큰데 센서값이 거의 안 변하면 끝에 걸렸거나 모터가 멈춘 상태
  if (
    abs(error) >= STALL_ERROR_THRESHOLD &&
    abs(delta) <= STALL_DELTA
  ) {
    stallCount++;
  } else {
    stallCount = 0;
  }

  previousMotionAdc = measuredAdc;
  lastMotionCheckMs = now;

  if (wrongDirectionCount >= WRONG_DIRECTION_LIMIT) {
    triggerSteeringFault("WRONG_DIRECTION");
    return;
  }

  if (stallCount >= STALL_COUNT_LIMIT) {
    triggerSteeringFault("MOTOR_STALLED_OR_ENDPOINT");
  }
}

// ============================================================
// -1000~1000 조향 명령을 ADC 목표값으로 변환
//
// +1000 = 왼쪽
//     0 = 중앙
// -1000 = 오른쪽
// ============================================================
int steeringCommandToAdc(int command) {
  command = constrain(command, -1000, 1000);

  if (command >= 0) {
    return map(
      command,
      0,
      1000,
      STEER_ADC_CENTER,
      STEER_ADC_LEFT
    );
  }

  return map(
    command,
    -1000,
    0,
    STEER_ADC_RIGHT,
    STEER_ADC_CENTER
  );
}

// ============================================================
// 조향 제어
// ============================================================
void updateSteering() {
  currentSteering = readSteeringAdc();
  currentError = targetSteering - currentSteering;

  if (
    currentSteering < STEER_ADC_VALID_MIN ||
    currentSteering > STEER_ADC_VALID_MAX
  ) {
    triggerSteeringFault("INVALID_FEEDBACK_ADC");
    return;
  }

  if (!steeringEnabled || steeringFault) {
    stopSteering();
    return;
  }

  if (abs(currentError) <= STEER_DEADBAND) {
    stopSteering();
    return;
  }

  SteeringDrive requestedDrive;

  if (currentError > 0) {
    requestedDrive = SteeringDrive::INCREASE_ADC;

    if (currentSteering >= STEER_ADC_SOFT_MAX) {
      triggerSteeringFault("LEFT_SOFT_LIMIT");
      return;
    }
  } else {
    requestedDrive = SteeringDrive::DECREASE_ADC;

    if (currentSteering <= STEER_ADC_SOFT_MIN) {
      triggerSteeringFault("RIGHT_SOFT_LIMIT");
      return;
    }
  }

  const int pwm = constrain(
    STEER_MIN_PWM + abs(currentError),
    STEER_MIN_PWM,
    STEER_MAX_PWM
  );

  driveSteering(requestedDrive, pwm, currentSteering);
  // 100 ms 간격으로 진행량을 확인한다. 큰 오차에서 8회 연속 ADC 변화가
  // 1 count 이하이면 약 0.8초 뒤 MOTOR_STALLED_OR_ENDPOINT를 래치하고
  // triggerSteeringFault()가 ESC와 조향을 함께 즉시 정지한다.
  monitorSteeringMotion(currentSteering, currentError);
}

// ============================================================
// ESC 제어
// ============================================================
void setEsc(int command) {
  if (command < THROTTLE_COMMAND_MIN || command > THROTTLE_COMMAND_MAX) {
    esc.writeMicroseconds(ESC_NEUTRAL_US);
    return;
  }

  int pulse = ESC_NEUTRAL_US;

  if (command > 0) {
    pulse = map(
      command,
      0,
      THROTTLE_COMMAND_MAX,
      ESC_NEUTRAL_US,
      ESC_FORWARD_MAX_US
    );
  } else if (command < 0) {
    pulse = map(
      command,
      THROTTLE_COMMAND_MIN,
      0,
      ESC_REVERSE_MAX_US,
      ESC_NEUTRAL_US
    );
  }

  esc.writeMicroseconds(pulse);
}

void cancelReverseEntry() {
  escReverseEntryState = EscReverseEntryState::IDLE;
  escReverseEntryDeadlineMs = 0;
}

void stopAllOutputs() {
  targetThrottle = 0;
  appliedThrottle = 0;
  steeringEnabled = false;
  driveCommandActive = false;
  escNeutralUntilMs = 0;
  cancelReverseEntry();
  setEsc(0);
  stopSteering();
}

void updateEsc() {
  const unsigned long now = millis();

  if (!escArmed || targetThrottle == 0) {
    appliedThrottle = 0;
    cancelReverseEntry();
    setEsc(0);
    return;
  }

  if (targetThrottle < 0) {
    if (escReverseEntryState == EscReverseEntryState::REVERSE_PULSE) {
      if (deadlinePending(now, escReverseEntryDeadlineMs)) {
        // Stage 1: full qualified reverse pulse (1350 us) for 250 ms.
        setEsc(THROTTLE_COMMAND_MIN);
        return;
      }
      escReverseEntryState = EscReverseEntryState::NEUTRAL_WAIT;
      escReverseEntryDeadlineMs = now + ESC_REVERSE_ENTRY_NEUTRAL_MS;
      setEsc(0);
      return;
    }

    if (escReverseEntryState == EscReverseEntryState::NEUTRAL_WAIT) {
      if (deadlinePending(now, escReverseEntryDeadlineMs)) {
        // Stage 2: neutral (1500 us) for 250 ms.
        setEsc(0);
        return;
      }
      cancelReverseEntry();
      appliedThrottle = targetThrottle;
      setEsc(appliedThrottle);
      return;
    }

    if (appliedThrottle < 0) {
      appliedThrottle = targetThrottle;
      setEsc(appliedThrottle);
      return;
    }

    // Every new entry into reverse starts with 1350 us for 250 ms, followed
    // by 1500 us for 250 ms. Only then is the requested reverse value applied.
    appliedThrottle = 0;
    escNeutralUntilMs = 0;
    escReverseEntryState = EscReverseEntryState::REVERSE_PULSE;
    escReverseEntryDeadlineMs = now + ESC_REVERSE_ENTRY_PULSE_MS;
    setEsc(THROTTLE_COMMAND_MIN);
    return;
  }

  // A forward request cancels an unfinished reverse-entry sequence and holds
  // neutral before changing from an already-applied reverse direction.
  if (escReverseEntryState != EscReverseEntryState::IDLE) {
    cancelReverseEntry();
    appliedThrottle = 0;
    escNeutralUntilMs = now + ESC_FORWARD_ENTRY_NEUTRAL_MS;
    setEsc(0);
    return;
  }

  if (appliedThrottle < 0) {
    appliedThrottle = 0;
    escNeutralUntilMs = now + ESC_FORWARD_ENTRY_NEUTRAL_MS;
    setEsc(0);
    return;
  }

  if (deadlinePending(now, escNeutralUntilMs)) {
    setEsc(0);
    return;
  }

  appliedThrottle = targetThrottle;
  setEsc(appliedThrottle);
}

bool isCommandSpace(char value) {
  return value == ' ' || value == '\t';
}

void skipCommandSpaces(const char*& cursor) {
  while (isCommandSpace(*cursor)) {
    cursor++;
  }
}

bool parseBoundedCommandInteger(const char*& cursor, int& value) {
  bool negative = false;
  if (*cursor == '+' || *cursor == '-') {
    negative = *cursor == '-';
    cursor++;
  }
  if (*cursor < '0' || *cursor > '9') {
    return false;
  }

  int magnitude = 0;
  while (*cursor >= '0' && *cursor <= '9') {
    const int digit = *cursor - '0';
    // Reject before multiplication can overflow AVR's 16-bit int. Protocol
    // commands outside +/-1000 are invalid regardless of their exact value.
    if (magnitude > 100 || (magnitude == 100 && digit > 0)) {
      return false;
    }
    magnitude = magnitude * 10 + digit;
    cursor++;
  }
  value = negative ? -magnitude : magnitude;
  return true;
}

bool parseDriveCommand(const char* line, int& throttle, int& steering) {
  if (line[0] != 'D') {
    return false;
  }

  const char* cursor = line + 1;
  skipCommandSpaces(cursor);
  if (!parseBoundedCommandInteger(cursor, throttle)) {
    return false;
  }
  if (!isCommandSpace(*cursor)) {
    return false;
  }
  skipCommandSpaces(cursor);
  if (!parseBoundedCommandInteger(cursor, steering)) {
    return false;
  }
  skipCommandSpaces(cursor);
  return *cursor == '\0';
}

// ============================================================
// 명령 처리
//
// D <스로틀> <조향>   (정확히 두 필드)
// 예:
// D 0 0          ESC 중립 + 조향 중앙
//
// X           즉시 정지
// R           조향 오류 해제 + 정지 유지
// ============================================================
void processCommand(char* line) {
  int throttle;
  int steering;

  if (parseDriveCommand(line, throttle, steering)) {
    if (!escArmed) {
      stopAllOutputs();
      Serial.println(F("[ERROR] ESC not armed; D rejected and outputs stopped"));
      return;
    }
    if (steeringFault) {
      stopAllOutputs();
      Serial.println(F("[ERROR] Steering fault latched; send R while stopped"));
      return;
    }
    if (
      throttle < THROTTLE_COMMAND_MIN ||
      throttle > THROTTLE_COMMAND_MAX ||
      steering < -1000 ||
      steering > 1000
    ) {
      stopAllOutputs();
      Serial.println(F("[ERROR] D values must be -1000..1000; outputs stopped"));
      return;
    }

    targetThrottle = throttle;
    targetSteering = steeringCommandToAdc(steering);

    steeringEnabled = true;
    lastCommandMs = millis();
    driveCommandActive = true;

    Serial.print(F("[COMMAND] throttle="));
    Serial.print(targetThrottle);
    Serial.print(F(" steeringADC="));
    Serial.println(targetSteering);

    return;
  }

  if (line[0] == 'X' && line[1] == '\0') {
    stopAllOutputs();

    Serial.println(F("[STOP]"));
    return;
  }

  if (line[0] == 'R' && line[1] == '\0') {
    // R only clears the steering fault. It first enforces the same immediate
    // stop as X and leaves all motion disabled until a later valid D command.
    stopAllOutputs();
    steeringFault = false;

    wrongDirectionCount = 0;
    stallCount = 0;

    currentSteering = readSteeringAdc();
    targetSteering = currentSteering;

    Serial.println(F("[FAULT RESET; STOPPED]"));
    return;
  }

  stopAllOutputs();
  Serial.println(F("[ERROR] Use: D throttle steering / X / R; outputs stopped"));
}

// ============================================================
// 시리얼 한 줄 수신
// ============================================================
void readCommand() {
  while (Serial.available() > 0) {
    const char received = Serial.read();

    if (received == '\r') {
      continue;
    }

    if (received == '\n') {
      if (receiveDiscardUntilNewline) {
        receiveDiscardUntilNewline = false;
        receiveLength = 0;
        continue;
      }
      receiveBuffer[receiveLength] = '\0';

      if (receiveLength > 0) {
        processCommand(receiveBuffer);
      }

      receiveLength = 0;
      continue;
    }

    if (receiveDiscardUntilNewline) {
      continue;
    }

    if (receiveLength < sizeof(receiveBuffer) - 1) {
      receiveBuffer[receiveLength++] = received;
    } else {
      receiveLength = 0;
      receiveDiscardUntilNewline = true;
      stopAllOutputs();
      Serial.println(F("[ERROR] Command too long; outputs stopped"));
    }
  }
}

// ============================================================
// 디버그 출력
// ============================================================
void printDebugInfo() {
  const unsigned long now = millis();

  if (now - lastDebugPrintMs < 200) {
    return;
  }

  lastDebugPrintMs = now;

  Serial.print(F("[DEBUG] Current="));
  Serial.print(currentSteering);

  Serial.print(F(" Target="));
  Serial.print(targetSteering);

  Serial.print(F(" Error="));
  Serial.print(currentError);

  Serial.print(F(" PWM="));
  Serial.print(currentSteeringPwm);

  Serial.print(F(" Drive="));

  if (steeringDrive == SteeringDrive::INCREASE_ADC) {
    Serial.print(F("ADC_UP"));
  } else if (steeringDrive == SteeringDrive::DECREASE_ADC) {
    Serial.print(F("ADC_DOWN"));
  } else {
    Serial.print(F("STOP"));
  }

  Serial.print(F(" Enabled="));
  Serial.print(steeringEnabled ? F("YES") : F("NO"));

  Serial.print(F(" Fault="));
  Serial.println(steeringFault ? F("YES") : F("NO"));
}

// ============================================================

void setup() {
  pinMode(STEER_ENA_PIN, OUTPUT);
  pinMode(STEER_IN1_PIN, OUTPUT);
  pinMode(STEER_IN2_PIN, OUTPUT);

  stopSteering();

  Serial.begin(115200);

  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(ESC_NEUTRAL_US);

  currentSteering = readSteeringAdc();
  targetSteering = currentSteering;
  previousMotionAdc = currentSteering;

  Serial.println(F("===================================="));
  Serial.println(F("RC car controller started"));
  Serial.println(FIRMWARE_CONFIG_BANNER);
  Serial.println(F("Command: D throttle steering"));
  Serial.println(F("Example: D 0 0"));
  Serial.println(F("Stop: X"));
  Serial.println(F("Reset fault: R"));
  Serial.println(F("===================================="));
}

void loop() {
  readCommand();

  if (
    driveCommandActive &&
    millis() - lastCommandMs >= COMMAND_TIMEOUT_MS
  ) {
    stopAllOutputs();
    Serial.println(F("[WATCHDOG] Command timeout: outputs stopped"));
  }

  if (!escArmed && millis() >= ESC_ARM_TIME_MS) {
    escArmed = true;
    Serial.println(F("[ESC ARMED]"));
  }

  updateEsc();
  updateSteering();
  printDebugInfo();

  delay(5);
}
