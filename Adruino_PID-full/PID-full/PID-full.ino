// ======================================================
// Dual Wheel Closed-loop Speed Control for Robot Vacuum Wheel
// Arduino UNO + L298N
//
// Wiring:
//
// Right wheel counter -> D2
// Left  wheel counter -> D3
//
// L298N Channel A = Right wheel
// ENA -> D5   PWM
// IN1 -> D8
// IN2 -> D9
//
// L298N Channel B = Left wheel
// ENB -> D6   PWM
// IN3 -> D10
// IN4 -> D11
//
// Motor power:
// L298N +12V -> external 12V +
// L298N GND  -> external 12V -
// Arduino GND -> L298N GND
// ======================================================


// ---------------- Encoder pins ----------------
const byte RIGHT_COUNTER_PIN = 2;
const byte LEFT_COUNTER_PIN  = 3;

volatile long rightEncoderCount = 0;
volatile long leftEncoderCount  = 0;


// ---------------- L298N pins ----------------
// Right motor = Channel A
const int RIGHT_ENA = 5;   // PWM
const int RIGHT_IN1 = 8;
const int RIGHT_IN2 = 9;

// Left motor = Channel B
const int LEFT_ENB = 6;    // PWM
const int LEFT_IN3 = 10;
const int LEFT_IN4 = 11;


// ---------------- Wheel parameters ----------------
const float COUNTS_PER_WHEEL_REV = 564.0;
const float WHEEL_DIAMETER_MM = 64.6;
const float MM_PER_COUNT = 3.1415926 * WHEEL_DIAMETER_MM / COUNTS_PER_WHEEL_REV;


// ---------------- Control parameters ----------------
const unsigned long CONTROL_INTERVAL_MS = 100;

// Speed unit: count / 100ms
const float TARGET_MIN = 30.0;
const float TARGET_MAX = 80.0;

// Lowest target the navigation stack (w command) can request.
// Magnitude below this is treated as 0 because the wheels can't track it
// reliably. Calibrate on hardware: lower until the wheel stops turning smoothly.
const float TARGET_DEADBAND = 10.0;

// P controller
float KpRight = 2.5;
float KpLeft  = 2.5;

// PWM limit
const int PWM_MIN = 0;
const int PWM_MAX = 255;

// PWM ramp limit per 100ms
const float PWM_STEP_LIMIT = 15.0;

// Stall protection
const int STALL_PWM_THRESHOLD = 240;
const float STALL_SPEED_RATIO = 0.7;
const int STALL_LIMIT_COUNT = 10;   // 10 * 100ms = 1 second


// ---------------- Runtime variables ----------------
float targetRight = 0.0;   // signed target, positive = forward, negative = reverse
float targetLeft  = 0.0;

float pwmRight = 0.0;
float pwmLeft  = 0.0;

long lastRightEncoderCount = 0;
long lastLeftEncoderCount  = 0;

unsigned long lastControlTime = 0;

int rightStallCounter = 0;
int leftStallCounter  = 0;

bool rightFault = false;
bool leftFault  = false;

String inputString = "";


// ---------------- Square mode ----------------
bool squareMode = false;
int squareStep = 0;
unsigned long squareStepStartTime = 0;

const float SQUARE_FORWARD_SPEED = 35.0;
const float SQUARE_TURN_SPEED = 35.0;
const unsigned long SQUARE_FORWARD_MS = 2500;
const unsigned long SQUARE_TURN_MS = 900;


// ======================================================
// Encoder interrupt functions
// ======================================================
void rightCountPulse() {
  rightEncoderCount++;
}

void leftCountPulse() {
  leftEncoderCount++;
}


// ======================================================
// Base PWM lookup table
// targetAbs unit: count / 100ms
// ======================================================
float getBasePWM(float targetAbs) {
  if (targetAbs <= 0) return 0;

  if (targetAbs <= 12) return 68;   // low-speed feedforward (calibrate on hardware)
  if (targetAbs <= 20) return 73;   // low-speed feedforward (calibrate on hardware)
  if (targetAbs <= 30) return 78;
  if (targetAbs <= 40) return 88;
  if (targetAbs <= 55) return 105;
  if (targetAbs <= 70) return 140;
  if (targetAbs <= 80) return 185;

  return 220;
}


// ======================================================
// Motor control functions
// ======================================================
void setRightMotor(int pwm, int direction) {
  pwm = constrain(pwm, 0, 255);

  if (direction > 0) {
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(RIGHT_ENA, pwm);
  } else if (direction < 0) {
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    analogWrite(RIGHT_ENA, pwm);
  } else {
    analogWrite(RIGHT_ENA, 0);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
  }
}

void setLeftMotor(int pwm, int direction) {
  pwm = constrain(pwm, 0, 255);

  if (direction > 0) {
    digitalWrite(LEFT_IN3, HIGH);
    digitalWrite(LEFT_IN4, LOW);
    analogWrite(LEFT_ENB, pwm);
  } else if (direction < 0) {
    digitalWrite(LEFT_IN3, LOW);
    digitalWrite(LEFT_IN4, HIGH);
    analogWrite(LEFT_ENB, pwm);
  } else {
    analogWrite(LEFT_ENB, 0);
    digitalWrite(LEFT_IN3, LOW);
    digitalWrite(LEFT_IN4, LOW);
  }
}

void stopMotors() {
  targetRight = 0;
  targetLeft = 0;

  pwmRight = 0;
  pwmLeft = 0;

  setRightMotor(0, 0);
  setLeftMotor(0, 0);
}


// ======================================================
// Target speed limit
// ======================================================
float limitSpeed(float speed) {
  if (speed == 0) return 0;

  float sign = speed > 0 ? 1.0 : -1.0;
  float absSpeed = abs(speed);

  if (absSpeed < TARGET_MIN) absSpeed = TARGET_MIN;
  if (absSpeed > TARGET_MAX) absSpeed = TARGET_MAX;

  return sign * absSpeed;
}

void setTargets(float right, float left) {
  targetRight = limitSpeed(right);
  targetLeft  = limitSpeed(left);

  rightFault = false;
  leftFault = false;
  rightStallCounter = 0;
  leftStallCounter = 0;

  pwmRight = getBasePWM(abs(targetRight));
  pwmLeft  = getBasePWM(abs(targetLeft));

  Serial.print("Set targets. Right = ");
  Serial.print(targetRight);
  Serial.print(", Left = ");
  Serial.println(targetLeft);
}


// ======================================================
// Raw signed wheel-speed target (counts/100ms), used by the `w R L` command
// for ROS velocity control. Unlike setTargets() it does NOT force the teleop
// TARGET_MIN floor (so the robot can crawl), and does NOT reset the PWM ramp /
// fault state, because navigation streams this command continuously.
// ======================================================
float clampTarget(float speed) {
  float a = fabs(speed);
  if (a < TARGET_DEADBAND) return 0.0;   // below this the wheel can't track it
  if (a > TARGET_MAX) a = TARGET_MAX;
  return (speed >= 0) ? a : -a;
}

void setTargetsRaw(float right, float left) {
  targetRight = clampTarget(right);
  targetLeft  = clampTarget(left);
}


// ======================================================
// Serial command handling
// Commands:
// f 30   -> forward
// b 30   -> backward
// l 30   -> turn left in place
// r 30   -> turn right in place
// w 30 20 -> set right/left wheel speeds (counts/100ms, signed; ROS cmd_vel)
// s      -> stop
// q      -> square
// clear  -> clear fault
// ======================================================
void processCommand(String cmd) {
  cmd.trim();

  if (cmd.length() == 0) return;

  if (cmd == "s" || cmd == "S") {
    squareMode = false;
    stopMotors();
    Serial.println("Stop.");
    return;
  }

  if (cmd == "clear" || cmd == "CLEAR") {
    rightFault = false;
    leftFault = false;
    rightStallCounter = 0;
    leftStallCounter = 0;
    Serial.println("Fault cleared.");
    return;
  }

  if (cmd == "q" || cmd == "Q") {
    squareMode = true;
    squareStep = 0;
    squareStepStartTime = millis();
    Serial.println("Square mode started.");
    return;
  }

  // w <right> <left>: signed wheel speeds in counts/100ms (ROS velocity control).
  // Enables arbitrary (v, w): different left/right speeds -> arcs, not just
  // forward / in-place turns. Must be parsed before the generic single-arg path
  // below because it takes two (possibly negative) arguments.
  if (cmd.charAt(0) == 'w' || cmd.charAt(0) == 'W') {
    String rest = cmd.substring(1);
    rest.trim();
    int sp = rest.indexOf(' ');
    if (sp <= 0) {
      Serial.println("Use: w <right> <left>");
      return;
    }
    float r = rest.substring(0, sp).toFloat();
    float l = rest.substring(sp + 1).toFloat();
    squareMode = false;
    setTargetsRaw(r, l);
    return;
  }

  char action = cmd.charAt(0);
  float speed = cmd.substring(1).toFloat();

  if (speed <= 0) {
    Serial.println("Invalid speed. Example: f 30");
    return;
  }

  if (action == 'f' || action == 'F') {
    squareMode = false;
    setTargets(speed, speed);
    Serial.println("Forward.");
  } else if (action == 'b' || action == 'B') {
    squareMode = false;
    setTargets(-speed, -speed);
    Serial.println("Backward.");
  } else if (action == 'l' || action == 'L') {
    squareMode = false;
    setTargets(speed, -speed);
    Serial.println("Turn left.");
  } else if (action == 'r' || action == 'R') {
    squareMode = false;
    setTargets(-speed, speed);
    Serial.println("Turn right.");
  } else {
    Serial.println("Unknown command.");
    Serial.println("Use: f 30, b 30, l 30, r 30, s, q, clear");
  }
}

void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) {
        processCommand(inputString);
        inputString = "";
      }
    } else {
      inputString += c;
    }
  }
}


// ======================================================
// Square mode by time
// Step 0: forward
// Step 1: turn right
// Repeat 4 times
// ======================================================
void updateSquareMode() {
  if (!squareMode) return;

  unsigned long now = millis();
  unsigned long elapsed = now - squareStepStartTime;

  int phase = squareStep % 2;

  if (phase == 0) {
    setTargets(SQUARE_FORWARD_SPEED, SQUARE_FORWARD_SPEED);

    if (elapsed >= SQUARE_FORWARD_MS) {
      squareStep++;
      squareStepStartTime = now;
    }
  } else {
    setTargets(-SQUARE_TURN_SPEED, SQUARE_TURN_SPEED);

    if (elapsed >= SQUARE_TURN_MS) {
      squareStep++;
      squareStepStartTime = now;
    }
  }

  if (squareStep >= 8) {
    squareMode = false;
    stopMotors();
    Serial.println("Square mode finished.");
  }
}


// ======================================================
// One wheel controller
// ======================================================
void updateOneWheel(
  const char* name,
  float targetSigned,
  float actualCount,
  float &pwmOutput,
  float Kp,
  int &stallCounter,
  bool &fault,
  bool isRight
) {
  float targetAbs = abs(targetSigned);
  int direction = 0;

  if (targetSigned > 0) direction = 1;
  else if (targetSigned < 0) direction = -1;
  else direction = 0;

  if (targetAbs <= 0) {
    pwmOutput = 0;
    stallCounter = 0;

    if (isRight) setRightMotor(0, 0);
    else setLeftMotor(0, 0);

    return;
  }

  if (fault) {
    pwmOutput = 0;

    if (isRight) setRightMotor(0, 0);
    else setLeftMotor(0, 0);

    return;
  }

  float basePWM = getBasePWM(targetAbs);
  float error = targetAbs - actualCount;

  float targetPWM = basePWM + Kp * error;
  targetPWM = constrain(targetPWM, PWM_MIN, PWM_MAX);

  float diff = targetPWM - pwmOutput;

  if (diff > PWM_STEP_LIMIT) diff = PWM_STEP_LIMIT;
  if (diff < -PWM_STEP_LIMIT) diff = -PWM_STEP_LIMIT;

  pwmOutput += diff;
  pwmOutput = constrain(pwmOutput, PWM_MIN, PWM_MAX);

  if (isRight) setRightMotor((int)pwmOutput, direction);
  else setLeftMotor((int)pwmOutput, direction);

  if (pwmOutput > STALL_PWM_THRESHOLD &&
      actualCount < targetAbs * STALL_SPEED_RATIO) {
    stallCounter++;
  } else {
    stallCounter = 0;
  }

  if (stallCounter >= STALL_LIMIT_COUNT) {
    fault = true;
    pwmOutput = 0;

    if (isRight) setRightMotor(0, 0);
    else setLeftMotor(0, 0);

    Serial.print("STALL FAULT: ");
    Serial.println(name);
  }
}


// ======================================================
// Setup
// ======================================================
void setup() {
  Serial.begin(115200);

  pinMode(RIGHT_COUNTER_PIN, INPUT_PULLUP);
  pinMode(LEFT_COUNTER_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RIGHT_COUNTER_PIN), rightCountPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_COUNTER_PIN), leftCountPulse, RISING);

  pinMode(RIGHT_ENA, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(LEFT_ENB, OUTPUT);
  pinMode(LEFT_IN3, OUTPUT);
  pinMode(LEFT_IN4, OUTPUT);

  stopMotors();

  lastControlTime = millis();

  Serial.println("Dual wheel closed-loop speed control started.");
  Serial.println("Commands:");
  Serial.println("f 30  -> forward");
  Serial.println("b 30  -> backward");
  Serial.println("l 30  -> turn left in place");
  Serial.println("r 30  -> turn right in place");
  Serial.println("w R L -> set wheel speeds counts/100ms (signed, ROS cmd_vel)");
  Serial.println("s     -> stop");
  Serial.println("q     -> square by time");
  Serial.println("clear -> clear fault");
  Serial.println("Speed range: 30~80 count / 100ms");
  Serial.println("Columns:");
  Serial.println("targetR, actualR, pwmR, targetL, actualL, pwmL, speedR_mm_s, speedL_mm_s, faultR, faultL");
}


// ======================================================
// Main loop
// ======================================================
void loop() {
  handleSerialInput();
  updateSquareMode();

  unsigned long now = millis();

  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    float dt = (now - lastControlTime) / 1000.0;

    noInterrupts();
    long currentRightCount = rightEncoderCount;
    long currentLeftCount  = leftEncoderCount;
    interrupts();

    long deltaRight = currentRightCount - lastRightEncoderCount;
    long deltaLeft  = currentLeftCount - lastLeftEncoderCount;

    float actualRight = deltaRight;
    float actualLeft  = deltaLeft;

    updateOneWheel(
      "RIGHT",
      targetRight,
      actualRight,
      pwmRight,
      KpRight,
      rightStallCounter,
      rightFault,
      true
    );

    updateOneWheel(
      "LEFT",
      targetLeft,
      actualLeft,
      pwmLeft,
      KpLeft,
      leftStallCounter,
      leftFault,
      false
    );

    float speedRightMMs = actualRight / dt * MM_PER_COUNT;
    float speedLeftMMs  = actualLeft / dt * MM_PER_COUNT;

    Serial.print(targetRight);
    Serial.print(", ");
    Serial.print(actualRight);
    Serial.print(", ");
    Serial.print(pwmRight);
    Serial.print(", ");

    Serial.print(targetLeft);
    Serial.print(", ");
    Serial.print(actualLeft);
    Serial.print(", ");
    Serial.print(pwmLeft);
    Serial.print(", ");

    Serial.print(speedRightMMs);
    Serial.print(", ");
    Serial.print(speedLeftMMs);
    Serial.print(", ");

    Serial.print(rightFault ? "FAULT" : "OK");
    Serial.print(", ");
    Serial.println(leftFault ? "FAULT" : "OK");

    lastRightEncoderCount = currentRightCount;
    lastLeftEncoderCount  = currentLeftCount;
    lastControlTime = now;
  }
}