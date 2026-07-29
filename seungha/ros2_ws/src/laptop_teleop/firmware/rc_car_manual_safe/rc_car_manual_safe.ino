#include <Servo.h>
#include <stdio.h>

// ============================================================
// IMPORTANT: keep false until the physical wiring and calibration
// values below have been checked with the wheels off the ground.
// With false, non-zero drive commands are rejected at runtime.
// ============================================================
constexpr bool HARDWARE_PROFILE_VERIFIED = false;

// The connected USB descriptor currently reports PID 0043 (normally Uno R3),
// while the project notes say Mega. The compiler selects the matching block.
#if defined(__AVR_ATmega2560__)
constexpr uint8_t STEER_ENA_PIN = 8;
constexpr uint8_t STEER_IN1_PIN = 22;
constexpr uint8_t STEER_IN2_PIN = 24;
constexpr uint8_t STEER_FEEDBACK_PIN = A0;
const char BOARD_NAME[] = "Arduino Mega 2560";
#elif defined(__AVR_ATmega328P__)
constexpr uint8_t STEER_ENA_PIN = 5;
constexpr uint8_t STEER_IN1_PIN = 12;
constexpr uint8_t STEER_IN2_PIN = 13;
constexpr uint8_t STEER_FEEDBACK_PIN = A5;
const char BOARD_NAME[] = "Arduino Uno R3";
#else
#error "Supported boards: Arduino Mega 2560 or Arduino Uno R3"
#endif

constexpr uint8_t ESC_PIN = 9;

// Latest measured values found on the Jetson. Verify all three again.
constexpr int STEER_ADC_LEFT = 712;
constexpr int STEER_ADC_CENTER = 601;
constexpr int STEER_ADC_RIGHT = 488;
constexpr int STEER_DEADBAND = 7;
constexpr int STEER_ADC_VALID_MIN = 50;
constexpr int STEER_ADC_VALID_MAX = 973;
constexpr int STEER_SOFT_MARGIN = 15;

constexpr int STEER_MIN_PWM = 110;
constexpr int STEER_MAX_PWM = 180;

// true means IN1=HIGH/IN2=LOW must make the feedback ADC increase.
// The latest safe sketch used false. Verify by a very short lifted-wheel test.
constexpr bool HIGH_LOW_INCREASES_ADC = false;

constexpr int ESC_NEUTRAL_US = 1500;
constexpr int ESC_FORWARD_MAX_US = 1560;
constexpr int ESC_REVERSE_MAX_US = 1380;
constexpr int ESC_RAMP_STEP = 10;
constexpr unsigned long ESC_RAMP_INTERVAL_MS = 20;
constexpr unsigned long ESC_REVERSE_GUARD_MS = 400;
constexpr unsigned long ESC_ARM_TIME_MS = 3000;
constexpr unsigned long COMMAND_TIMEOUT_MS = 400;

constexpr unsigned long MOTION_CHECK_INTERVAL_MS = 120;
constexpr int WRONG_DIRECTION_DELTA = 3;
constexpr int STALL_DELTA = 1;
constexpr int STALL_ERROR_THRESHOLD = 25;
constexpr uint8_t WRONG_DIRECTION_LIMIT = 2;
constexpr uint8_t STALL_LIMIT = 8;

constexpr size_t RX_BUFFER_SIZE = 48;

static_assert(
    (STEER_ADC_LEFT > STEER_ADC_CENTER && STEER_ADC_CENTER > STEER_ADC_RIGHT) ||
    (STEER_ADC_LEFT < STEER_ADC_CENTER && STEER_ADC_CENTER < STEER_ADC_RIGHT),
    "STEER_ADC_CENTER must be between LEFT and RIGHT");

Servo esc;

char rxBuffer[RX_BUFFER_SIZE];
size_t rxLength = 0;

int requestedThrottle = 0;
int appliedThrottle = 0;
int targetSteering = STEER_ADC_CENTER;
int currentSteering = STEER_ADC_CENTER;
int currentError = 0;
int steeringPwm = 0;
int steeringDirection = 0;  // +1 increases ADC, -1 decreases ADC, 0 stopped

bool neutralSeen = false;
bool controlEnabled = false;
bool steeringFault = false;
bool estopLatched = false;

unsigned long bootMs = 0;
unsigned long lastCommandMs = 0;
unsigned long lastEscRampMs = 0;
unsigned long escNeutralUntilMs = 0;
unsigned long lastMotionCheckMs = 0;
unsigned long lastDebugMs = 0;
int previousMotionAdc = STEER_ADC_CENTER;
uint8_t wrongDirectionCount = 0;
uint8_t stallCount = 0;

int signOf(int value) {
  return (value > 0) - (value < 0);
}

bool deadlinePending(unsigned long now, unsigned long deadline) {
  return static_cast<long>(deadline - now) > 0;
}

int readSteeringAdc() {
  long sum = 0;
  analogRead(STEER_FEEDBACK_PIN);
  for (uint8_t i = 0; i < 8; ++i) {
    sum += analogRead(STEER_FEEDBACK_PIN);
  }
  return static_cast<int>(sum / 8);
}

void stopSteering() {
  analogWrite(STEER_ENA_PIN, 0);
  digitalWrite(STEER_IN1_PIN, LOW);
  digitalWrite(STEER_IN2_PIN, LOW);
  steeringPwm = 0;
  steeringDirection = 0;
}

void stopAll() {
  requestedThrottle = 0;
  appliedThrottle = 0;
  esc.writeMicroseconds(ESC_NEUTRAL_US);
  controlEnabled = false;
  stopSteering();
}

void triggerFault(const __FlashStringHelper* reason) {
  steeringFault = true;
  neutralSeen = false;
  stopAll();
  Serial.print(F("[FAULT] "));
  Serial.println(reason);
}

int steeringCommandToAdc(int command) {
  command = constrain(command, -1000, 1000);
  if (command >= 0) {
    return map(command, 0, 1000, STEER_ADC_CENTER, STEER_ADC_LEFT);
  }
  return map(command, -1000, 0, STEER_ADC_RIGHT, STEER_ADC_CENTER);
}

void writeEscCommand(int command) {
  command = constrain(command, -1000, 1000);
  int pulse = ESC_NEUTRAL_US;
  if (command > 0) {
    pulse = map(command, 0, 1000, ESC_NEUTRAL_US, ESC_FORWARD_MAX_US);
  } else if (command < 0) {
    pulse = map(command, -1000, 0, ESC_REVERSE_MAX_US, ESC_NEUTRAL_US);
  }
  esc.writeMicroseconds(pulse);
}

void updateEsc() {
  const unsigned long now = millis();
  if (!HARDWARE_PROFILE_VERIFIED || !controlEnabled || estopLatched ||
      now - bootMs < ESC_ARM_TIME_MS) {
    appliedThrottle = 0;
    writeEscCommand(0);
    return;
  }

  if (requestedThrottle == 0) {
    if (appliedThrottle != 0) {
      escNeutralUntilMs = now + ESC_REVERSE_GUARD_MS;
    }
    appliedThrottle = 0;
    writeEscCommand(0);
    return;
  }

  if (appliedThrottle != 0 && signOf(appliedThrottle) != signOf(requestedThrottle)) {
    appliedThrottle = 0;
    escNeutralUntilMs = now + ESC_REVERSE_GUARD_MS;
    writeEscCommand(0);
    return;
  }

  if (deadlinePending(now, escNeutralUntilMs)) {
    writeEscCommand(0);
    return;
  }

  if (now - lastEscRampMs >= ESC_RAMP_INTERVAL_MS) {
    lastEscRampMs = now;
    const int delta = constrain(
        requestedThrottle - appliedThrottle, -ESC_RAMP_STEP, ESC_RAMP_STEP);
    appliedThrottle += delta;
  }
  writeEscCommand(appliedThrottle);
}

void driveSteering(int direction, int pwm) {
  if (direction != steeringDirection) {
    stopSteering();
    delayMicroseconds(300);
    previousMotionAdc = currentSteering;
    lastMotionCheckMs = millis();
    wrongDirectionCount = 0;
    stallCount = 0;
  }

  bool in1High = direction > 0
      ? HIGH_LOW_INCREASES_ADC
      : !HIGH_LOW_INCREASES_ADC;
  digitalWrite(STEER_IN1_PIN, in1High ? HIGH : LOW);
  digitalWrite(STEER_IN2_PIN, in1High ? LOW : HIGH);
  steeringPwm = constrain(pwm, STEER_MIN_PWM, STEER_MAX_PWM);
  analogWrite(STEER_ENA_PIN, steeringPwm);
  steeringDirection = direction;
}

void monitorSteeringMotion() {
  if (steeringDirection == 0) return;
  const unsigned long now = millis();
  if (now - lastMotionCheckMs < MOTION_CHECK_INTERVAL_MS) return;

  const int delta = currentSteering - previousMotionAdc;
  const bool wrong =
      (steeringDirection > 0 && delta <= -WRONG_DIRECTION_DELTA) ||
      (steeringDirection < 0 && delta >= WRONG_DIRECTION_DELTA);
  wrongDirectionCount = wrong ? wrongDirectionCount + 1 : 0;
  stallCount =
      (abs(currentError) >= STALL_ERROR_THRESHOLD && abs(delta) <= STALL_DELTA)
      ? stallCount + 1 : 0;
  previousMotionAdc = currentSteering;
  lastMotionCheckMs = now;

  if (wrongDirectionCount >= WRONG_DIRECTION_LIMIT) {
    triggerFault(F("STEERING_WRONG_DIRECTION"));
  } else if (stallCount >= STALL_LIMIT) {
    triggerFault(F("STEERING_STALLED_OR_ENDPOINT"));
  }
}

void updateSteering() {
  currentSteering = readSteeringAdc();
  currentError = targetSteering - currentSteering;

  if (currentSteering < STEER_ADC_VALID_MIN || currentSteering > STEER_ADC_VALID_MAX) {
    triggerFault(F("INVALID_STEERING_FEEDBACK"));
    return;
  }
  if (!HARDWARE_PROFILE_VERIFIED || !controlEnabled || steeringFault || estopLatched) {
    stopSteering();
    return;
  }
  if (abs(currentError) <= STEER_DEADBAND) {
    stopSteering();
    return;
  }

  const int direction = currentError > 0 ? 1 : -1;
  const int softMax = max(STEER_ADC_LEFT, STEER_ADC_RIGHT) + STEER_SOFT_MARGIN;
  const int softMin = min(STEER_ADC_LEFT, STEER_ADC_RIGHT) - STEER_SOFT_MARGIN;
  if ((direction > 0 && currentSteering >= softMax) ||
      (direction < 0 && currentSteering <= softMin)) {
    triggerFault(F("STEERING_SOFT_LIMIT"));
    return;
  }

  driveSteering(
      direction,
      constrain(STEER_MIN_PWM + abs(currentError), STEER_MIN_PWM, STEER_MAX_PWM));
  monitorSteeringMotion();
}

void processCommand(char* line) {
  if (line[0] == 'X' && line[1] == '\0') {
    estopLatched = true;
    neutralSeen = false;
    stopAll();
    Serial.println(F("[ESTOP LATCHED]"));
    return;
  }
  if (line[0] == 'R' && line[1] == '\0') {
    steeringFault = false;
    estopLatched = false;
    neutralSeen = false;
    wrongDirectionCount = 0;
    stallCount = 0;
    currentSteering = readSteeringAdc();
    targetSteering = currentSteering;
    stopAll();
    Serial.println(F("[RESET - SEND D0 0 BEFORE DRIVING]"));
    return;
  }

  int throttle = 0;
  int steering = 0;
  if (sscanf(line, "D%d %d", &throttle, &steering) != 2 &&
      sscanf(line, "D %d %d", &throttle, &steering) != 2) {
    Serial.println(F("[ERROR] Use D throttle steering / X / R"));
    return;
  }

  throttle = constrain(throttle, -1000, 1000);
  steering = constrain(steering, -1000, 1000);
  if (throttle == 0 && steering == 0) {
    neutralSeen = true;
    requestedThrottle = 0;
    targetSteering = currentSteering;
    controlEnabled = false;
    lastCommandMs = millis();
    stopAll();
    return;
  }
  if (!HARDWARE_PROFILE_VERIFIED) {
    Serial.println(F("[BLOCKED] HARDWARE_PROFILE_VERIFIED is false"));
    stopAll();
    return;
  }
  if (estopLatched || steeringFault || !neutralSeen) {
    Serial.println(F("[BLOCKED] reset faults and send D0 0 first"));
    stopAll();
    return;
  }

  requestedThrottle = throttle;
  targetSteering = steeringCommandToAdc(steering);
  controlEnabled = true;
  lastCommandMs = millis();
}

void readSerial() {
  while (Serial.available() > 0) {
    const char value = Serial.read();
    if (value == '\r') continue;
    if (value == '\n') {
      rxBuffer[rxLength] = '\0';
      if (rxLength > 0) processCommand(rxBuffer);
      rxLength = 0;
      continue;
    }
    if (rxLength < RX_BUFFER_SIZE - 1) {
      rxBuffer[rxLength++] = value;
    } else {
      rxLength = 0;
      Serial.println(F("[ERROR] command too long"));
    }
  }
}

void printDebug() {
  const unsigned long now = millis();
  if (now - lastDebugMs < 200) return;
  lastDebugMs = now;
  Serial.print(F("[DEBUG] Throttle: "));
  Serial.print(appliedThrottle);
  Serial.print(F(" | SteerTarget(ADC): "));
  Serial.print(targetSteering);
  Serial.print(F(" | SteerCurrent(ADC): "));
  Serial.print(currentSteering);
  Serial.print(F(" | Error: "));
  Serial.println(currentError);
}

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
  bootMs = millis();
  lastCommandMs = bootMs;

  Serial.print(F("[READY] "));
  Serial.print(BOARD_NAME);
  Serial.print(F(" profile_verified="));
  Serial.println(HARDWARE_PROFILE_VERIFIED ? F("YES") : F("NO"));
}

void loop() {
  readSerial();
  if (controlEnabled && millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    neutralSeen = false;
    stopAll();
    Serial.println(F("[COMMAND TIMEOUT - STOPPED]"));
  }
  updateEsc();
  updateSteering();
  printDebug();
  delay(5);
}

