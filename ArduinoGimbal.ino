
#include <Wire.h>
#include <Servo.h>
#include <math.h>

// ---------- PINS ----------
const byte MPU_ADDR = 0x68;

const int PITCH_PIN = 9;
const int ROLL_PIN  = 10;

const int JOY_X_PIN = A0;
const int JOY_Y_PIN = A1;
const int JOY_SW_PIN = 2;

// ---------- SERVO SETTINGS ----------
const int SERVO_MIN = 20;
const int SERVO_MAX = 160;
const int SERVO_CENTER = 90;

const bool REVERSE_PITCH = false;
const bool REVERSE_ROLL  = false;

// ---------- SPEED SETTINGS ----------
// Maximum degrees each servo command can change per update.
// Smaller = slower.
const float MAX_STEP = 1.0;

// Time between servo updates in milliseconds.
// Larger = slower.
const unsigned long UPDATE_INTERVAL = 20;

// Automatic-mode filter (kept from your code)
const float FILTER = 0.50;

// ---------- OBJECTS ----------
Servo pitchServo;
Servo rollServo;

// ---------- MODE ----------
enum Mode { AUTOMATIC, MANUAL };
Mode currentMode = AUTOMATIC;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_MS = 40;

// ---------- MPU DATA ----------
int16_t ax, ay, az;
int16_t gx, gy, gz;

float rollAngle = 0;
float pitchAngle = 0;
unsigned long lastTime;

// ---------- SMOOTH SERVO POSITIONS ----------
float currentPitch = SERVO_CENTER;
float currentRoll  = SERVO_CENTER;

int targetPitch = SERVO_CENTER;
int targetRoll  = SERVO_CENTER;

unsigned long lastServoUpdate = 0;

// ---------- READ MPU-6050 ----------
bool readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(MPU_ADDR, (byte)14);

  if (Wire.available() < 14) {
    return false;
  }

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  Wire.read();
  Wire.read(); // Temperature

  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  return true;
}

// ---------- GRADUAL MOVEMENT ----------
float moveGradually(float current, float target) {
  if (target > current + MAX_STEP) {
    return current + MAX_STEP;
  }

  if (target < current - MAX_STEP) {
    return current - MAX_STEP;
  }

  return target;
}

// ---------- BUTTON: CHANGE MODE ----------
void checkModeButton() {
  bool reading = digitalRead(JOY_SW_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
    lastButtonReading = reading;
  }

  if (millis() - lastDebounceTime >= DEBOUNCE_MS) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      if (stableButtonState == LOW) {
        if (currentMode == AUTOMATIC) {
          currentMode = MANUAL;
        } else {
          currentMode = AUTOMATIC;

          rollAngle = atan2((float)ay, (float)az)
                      * 180.0 / PI;

          pitchAngle = atan2(
            -(float)ax,
            sqrt((float)ay * ay + (float)az * az)
          ) * 180.0 / PI;

          lastTime = micros();
        }

        Serial.print("MODE: ");
        Serial.println(
          currentMode == AUTOMATIC ? "AUTOMATIC" : "MANUAL"
        );
      }
    }
  }
}

// ---------- MANUAL JOYSTICK ----------
int joystickToAngle(int pin, bool reverse) {
  int value = analogRead(pin);

  int angle = map(value, 0, 1023, SERVO_MIN, SERVO_MAX);

  if (reverse) {
    angle = SERVO_MIN + SERVO_MAX - angle;
  }

  return constrain(angle, SERVO_MIN, SERVO_MAX);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(9600);

  pinMode(JOY_SW_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  pitchServo.attach(PITCH_PIN);
  rollServo.attach(ROLL_PIN);

  pitchServo.write(SERVO_CENTER);
  rollServo.write(SERVO_CENTER);

  Wire.begin();

  // Wake up MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);

  if (Wire.endTransmission() != 0) {
    Serial.println("MPU ERROR: Check sensor wiring.");

    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  delay(200);

  if (!readMPU()) {
    Serial.println("MPU ERROR: No sensor data.");

    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }

  rollAngle = atan2((float)ay, (float)az) * 180.0 / PI;

  pitchAngle = atan2(
    -(float)ax,
    sqrt((float)ay * ay + (float)az * az)
  ) * 180.0 / PI;

  lastTime = micros();
  lastServoUpdate = millis();

  Serial.println("Gimbal ready.");
  Serial.println("Starts in AUTOMATIC mode.");
  Serial.println("Press joystick button to change mode.");
}

// ---------- MAIN LOOP ----------
void loop() {
  checkModeButton();

  if (!readMPU()) {
    Serial.println("MPU read error!");
    delay(100);
    return;
  }

  // ----- MANUAL MODE -----
  if (currentMode == MANUAL) {
    targetPitch = joystickToAngle(
      JOY_Y_PIN, REVERSE_PITCH
    );

    targetRoll = joystickToAngle(
      JOY_X_PIN, REVERSE_ROLL
    );
  }

  // ----- AUTOMATIC MODE -----
  else {
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;

    if (dt > 0 && dt <= 0.1) {
      float accelRoll = atan2((float)ay, (float)az)
                        * 180.0 / PI;

      float accelPitch = atan2(
        -(float)ax,
        sqrt((float)ay * ay + (float)az * az)
      ) * 180.0 / PI;

      // Original gyro calculations preserved
      float gyroRollRate = gx / 131.0;
      float gyroPitchRate = gy / 131.0;

      rollAngle =
        FILTER * (rollAngle + gyroRollRate * dt)
        + (1.0 - FILTER) * accelRoll;

      pitchAngle =
        FILTER * (pitchAngle + gyroPitchRate * dt)
        + (1.0 - FILTER) * accelPitch;

      targetRoll = SERVO_CENTER - (int)rollAngle;
      targetPitch = SERVO_CENTER - (int)pitchAngle;

      if (REVERSE_ROLL) {
        targetRoll = 2 * SERVO_CENTER - targetRoll;
      }

      if (REVERSE_PITCH) {
        targetPitch = 2 * SERVO_CENTER - targetPitch;
      }

      targetRoll = constrain(
        targetRoll, SERVO_MIN, SERVO_MAX
      );

      targetPitch = constrain(
        targetPitch, SERVO_MIN, SERVO_MAX
      );
    }
  }

  // ----- SLOW, GRADUAL SERVO MOVEMENT -----
  if (millis() - lastServoUpdate >= UPDATE_INTERVAL) {
    lastServoUpdate = millis();

    currentPitch = moveGradually(currentPitch, targetPitch);
    currentRoll = moveGradually(currentRoll, targetRoll);

    pitchServo.write((int)currentPitch);
    rollServo.write((int)currentRoll);
  }

  delay(2);
}
