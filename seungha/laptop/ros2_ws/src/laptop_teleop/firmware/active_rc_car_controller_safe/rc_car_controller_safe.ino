#include <Servo.h>
#include <stdio.h>

// ============================================================
// 핀 설정: 네 마지막 코드의 실제 배선 기준
// ============================================================
constexpr uint8_t ESC_PIN            = 9;
constexpr uint8_t GEAR_SERVO_PIN     = 10;
// Uno에서는 Servo 라이브러리가 Timer1을 사용하므로 D9/D10 PWM을 피해야 함.
// 조향 ENA는 PWM 가능한 D5에 연결한다.
constexpr uint8_t STEER_ENA_PIN      = 5;
constexpr uint8_t STEER_IN1_PIN      = 12;
constexpr uint8_t STEER_IN2_PIN      = 13;
constexpr uint8_t STEER_FEEDBACK_PIN = A5;

// ============================================================
// 조향 캘리브레이션
//
// 실제 측정값:
// 왼쪽 끝  ≈ 712
// 중앙     ≈ 601
// 오른쪽 끝 ≈ 488
// ============================================================
constexpr int STEER_ADC_LEFT   = 712;
constexpr int STEER_ADC_CENTER = 601;
constexpr int STEER_ADC_RIGHT  = 488;

constexpr int STEER_DEADBAND = 7;

// 실제 끝값보다 약간 바깥으로 나가면 강제 정지
constexpr int STEER_ADC_SOFT_MAX = STEER_ADC_LEFT + 15;
constexpr int STEER_ADC_SOFT_MIN = STEER_ADC_RIGHT - 15;

// 센서 단선 또는 쇼트 검출용
constexpr int STEER_ADC_VALID_MIN = 50;
constexpr int STEER_ADC_VALID_MAX = 973;

// 처음 테스트할 때는 출력을 낮게 잡음
// 정상 확인 후 MAX_PWM을 130~170 정도로 올려도 됨
constexpr int STEER_MIN_PWM = 110;
constexpr int STEER_MAX_PWM = 180;

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
// Full-range output amplitude reduced to 70% around 1500 us neutral.
constexpr int ESC_FORWARD_MAX_US = 1850;
constexpr int ESC_REVERSE_MAX_US = 1150;

constexpr unsigned long ESC_ARM_TIME_MS   = 3000;
// The operator-confirmed throttle scale tops out at 1050.  Keeping this as a
// named limit makes the ROS command value and the ESC mapping agree exactly.
constexpr int THROTTLE_FORWARD_LIMIT = 1050;
constexpr int THROTTLE_REVERSE_LIMIT = 1000;
// Independent Arduino-side fail-safe. ROS and the serial bridge normally send
// at 20 Hz, so 400 ms permits several missed frames before forcing neutral.
constexpr unsigned long COMMAND_TIMEOUT_MS = 400;

// ============================================================
// TRX-4 2단 변속 서보 설정
//
// 모델 미상 서보이므로 링크를 분리한 벤치 시험에서 먼저 확인한다.
// LOW/HIGH 방향이 반대면 두 펄스값을 서로 바꾼다.
// ============================================================
constexpr int GEAR_LOW_COMMAND  = -1;
constexpr int GEAR_HIGH_COMMAND = 1;
constexpr int GEAR_LOW_US       = 1300;
constexpr int GEAR_HIGH_US      = 1700;

static_assert(
  GEAR_LOW_US >= 1000 && GEAR_LOW_US <= 2000 &&
  GEAR_HIGH_US >= 1000 && GEAR_HIGH_US <= 2000 &&
  GEAR_LOW_US != GEAR_HIGH_US,
  "Gear servo pulses must be distinct values in 1000..2000 us"
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
Servo gearServo;

enum class SteeringDrive : uint8_t {
  STOP,
  INCREASE_ADC,
  DECREASE_ADC
};

SteeringDrive steeringDrive = SteeringDrive::STOP;

int targetThrottle = 0;
int targetSteering = STEER_ADC_CENTER;
int requestedGear = GEAR_LOW_COMMAND;
int appliedGear = 0;

int currentSteering = STEER_ADC_CENTER;
int currentError = 0;
int currentSteeringPwm = 0;

bool escArmed = false;
bool steeringEnabled = false;
bool steeringFault = false;
bool gearShiftBlockedLogged = false;

unsigned long lastDebugPrintMs = 0;
unsigned long lastMotionCheckMs = 0;
unsigned long lastCommandMs = 0;

bool driveCommandActive = false;

int previousMotionAdc = STEER_ADC_CENTER;
uint8_t wrongDirectionCount = 0;
uint8_t stallCount = 0;

// 시리얼 수신 버퍼
char receiveBuffer[48];
uint8_t receiveLength = 0;

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
  stopSteering();

  steeringFault = true;
  steeringEnabled = false;
  targetThrottle = 0;

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
  // Keep ADC closed-loop positioning, validity checks, PWM limits and soft
  // endpoints, but do not permanently latch a fault from short-term stall or
  // direction-motion heuristics. Those heuristics produced false positives on
  // the installed steering hardware and blocked all subsequent commands.
}

// ============================================================
// ESC 제어
// ============================================================
void setEsc(int command) {
  command = constrain(
    command, -THROTTLE_REVERSE_LIMIT, THROTTLE_FORWARD_LIMIT);

  int pulse = ESC_NEUTRAL_US;

  if (command > 0) {
    pulse = map(
      command,
      0,
      THROTTLE_FORWARD_LIMIT,
      ESC_NEUTRAL_US,
      ESC_FORWARD_MAX_US
    );
  } else if (command < 0) {
    pulse = map(
      command,
      -THROTTLE_REVERSE_LIMIT,
      0,
      ESC_REVERSE_MAX_US,
      ESC_NEUTRAL_US
    );
  }

  esc.writeMicroseconds(pulse);
}

// ============================================================
// TRX-4 LOW/HIGH 변속
// 스로틀이 0일 때만 새 기어를 적용해 기어 충격을 줄인다.
// ============================================================
void updateGear() {
  if (requestedGear == appliedGear) {
    gearShiftBlockedLogged = false;
    return;
  }

  if (targetThrottle != 0) {
    if (!gearShiftBlockedLogged) {
      Serial.println(F("[GEAR WAIT] Stop throttle before shifting"));
      gearShiftBlockedLogged = true;
    }
    return;
  }

  gearShiftBlockedLogged = false;
  const int pulse = requestedGear == GEAR_LOW_COMMAND
    ? GEAR_LOW_US
    : GEAR_HIGH_US;
  gearServo.writeMicroseconds(pulse);
  appliedGear = requestedGear;

  Serial.print(F("[GEAR] "));
  Serial.print(appliedGear == GEAR_LOW_COMMAND ? F("LOW") : F("HIGH"));
  Serial.print(F(" pulse="));
  Serial.println(pulse);
}

// ============================================================
// 명령 처리
//
// D <스로틀> <조향> <기어>
// 예:
// D 0 0 -1       정지 + LOW
// D 0 0 1        정지 + HIGH
// 이전 D <스로틀> <조향> 형식도 현재 기어를 유지하며 허용
//
// X           즉시 정지
// R           오류 해제
// ============================================================
void processCommand(char* line) {
  int throttle;
  int steering;
  int gear;

  const int driveFields = sscanf(
    line, "D %d %d %d", &throttle, &steering, &gear);
  if (driveFields == 2 || driveFields == 3) {
    if (
      driveFields == 3 &&
      gear != GEAR_LOW_COMMAND &&
      gear != GEAR_HIGH_COMMAND
    ) {
      Serial.println(F("[ERROR] Gear must be -1 (LOW) or 1 (HIGH)"));
      return;
    }

    targetThrottle = constrain(
      throttle, -THROTTLE_REVERSE_LIMIT, THROTTLE_FORWARD_LIMIT);
    targetSteering = steeringCommandToAdc(steering);
    if (driveFields == 3) {
      requestedGear = gear;
    }

    steeringEnabled = true;
    lastCommandMs = millis();
    driveCommandActive = true;

    Serial.print(F("[COMMAND] throttle="));
    Serial.print(targetThrottle);
    Serial.print(F(" steeringADC="));
    Serial.print(targetSteering);
    Serial.print(F(" gear="));
    Serial.println(requestedGear == GEAR_LOW_COMMAND ? F("LOW") : F("HIGH"));

    return;
  }

  if (line[0] == 'X' && line[1] == '\0') {
    targetThrottle = 0;
    steeringEnabled = false;
    driveCommandActive = false;

    setEsc(0);
    stopSteering();

    Serial.println(F("[STOP]"));
    return;
  }

  if (line[0] == 'R' && line[1] == '\0') {
    steeringFault = false;
    steeringEnabled = false;

    wrongDirectionCount = 0;
    stallCount = 0;

    currentSteering = readSteeringAdc();
    targetSteering = currentSteering;

    stopSteering();

    Serial.println(F("[FAULT RESET]"));
    return;
  }

  Serial.println(F("[ERROR] Use: D throttle steering gear / X / R"));
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
      receiveBuffer[receiveLength] = '\0';

      if (receiveLength > 0) {
        processCommand(receiveBuffer);
      }

      receiveLength = 0;
      continue;
    }

    if (receiveLength < sizeof(receiveBuffer) - 1) {
      receiveBuffer[receiveLength++] = received;
    } else {
      receiveLength = 0;
      Serial.println(F("[ERROR] Command too long"));
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
  gearServo.attach(GEAR_SERVO_PIN, 1000, 2000);
  updateGear();

  currentSteering = readSteeringAdc();
  targetSteering = currentSteering;
  previousMotionAdc = currentSteering;

  Serial.println(F("===================================="));
  Serial.println(F("RC car controller started"));
  Serial.println(F("Command: D throttle steering gear"));
  Serial.println(F("Example LOW : D 0 0 -1"));
  Serial.println(F("Example HIGH: D 0 0 1"));
  Serial.println(F("Stop: X"));
  Serial.println(F("Reset fault: R"));
  Serial.println(F("===================================="));
}

void loop() {
  readCommand();

  if (
    driveCommandActive &&
    millis() - lastCommandMs > COMMAND_TIMEOUT_MS
  ) {
    targetThrottle = 0;
    steeringEnabled = false;
    driveCommandActive = false;
    setEsc(0);
    stopSteering();
    Serial.println(F("[WATCHDOG] Command timeout: outputs stopped"));
  }

  if (!escArmed && millis() >= ESC_ARM_TIME_MS) {
    escArmed = true;
    Serial.println(F("[ESC ARMED]"));
  }

  updateGear();
  setEsc(escArmed ? targetThrottle : 0);
  updateSteering();
  printDebugInfo();

  delay(5);
}
