#include <QTRSensors.h>

#define PWM1      9
#define AIN1      4
#define AIN2      3
#define PWM2      10
#define BIN1      6
#define BIN2      7
#define STDBY     5
#define BTN_START 11

QTRSensors qtr;
const uint8_t SensorCount = 7;
uint16_t sensorValues[SensorCount];

float Kp = 0.0061;
float Kd = 2.49;
float Ki = 0.000;

int speed = 120;
int P, D, I, previousError, PIDvalue, position;
int threshold = 0;

// ── Lost Line Memory ──────────────────────────────────────────
enum LastSide { NONE, LEFT, RIGHT };
LastSide lastSide = RIGHT;   // default recovery direction if line never seen

// ── Motor Helpers ─────────────────────────────────────────────
void motorLeft(int spd) {
  if (spd >= 0) { digitalWrite(AIN1, 1); digitalWrite(AIN2, 0); }
  else          { digitalWrite(AIN1, 0); digitalWrite(AIN2, 1); spd = -spd; }
  analogWrite(PWM1, constrain(spd, 0, 255));
}

void motorRight(int spd) {
  if (spd >= 0) { digitalWrite(BIN1, 1); digitalWrite(BIN2, 0); }
  else          { digitalWrite(BIN1, 0); digitalWrite(BIN2, 1); spd = -spd; }
  analogWrite(PWM2, constrain(spd, 0, 255));
}

void turnRight() { motorLeft(-170); motorRight(170);  }
void turnLeft()  { motorLeft(170);  motorRight(-170); }

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BTN_START, INPUT_PULLUP);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWM1, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWM2, OUTPUT);
  pinMode(STDBY, OUTPUT);
  digitalWrite(STDBY, HIGH);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6}, SensorCount);
  qtr.setEmitterPin(2);

  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("[CALIBRATE]");
  for (uint16_t i = 0; i < 400; i++) qtr.calibrate();

  threshold = 0;
  for (uint8_t i = 0; i < SensorCount; i++)
    threshold += (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2;
  threshold /= SensorCount;

  Serial.print("[THRESHOLD] "); Serial.println(threshold);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("[WAITING] Press button...");
  while (digitalRead(BTN_START) == HIGH) {}
  delay(50);
  Serial.println("[RUNNING]");
}

// ── Main Loop ─────────────────────────────────────────────────
void loop() {
  position = qtr.readLineBlack(sensorValues);

  bool s[7];
  for (int i = 0; i < 7; i++) s[i] = (sensorValues[i] >= threshold);

  // ── Edge memory update — runs every loop ──────────────────
  // s[0] = leftmost sensor, s[6] = rightmost sensor
  if (s[0]) lastSide = LEFT;
  if (s[6]) lastSide = RIGHT;   // RIGHT wins if both fire simultaneously

  // ── Lost line detection ───────────────────────────────────
  bool anyBlack = false;
  for (int i = 0; i < 7; i++) if (s[i]) { anyBlack = true; break; }
  bool lineLost = !anyBlack;

  // ── Turn detection patterns ───────────────────────────────
  bool rightTurn =
    (s[0] && s[1] && s[2] && s[3] && s[4] && !s[5] && !s[6]) ||
    (s[0] && s[1] && s[2] && s[3] && !s[4] && !s[5] && !s[6]) ||
    (s[0] && s[1] && s[2] && s[3] && s[4] && s[5] && s[6])    ||
    (s[0] && s[1] && s[2] && s[3] && s[4] && s[5] && !s[6])   ||
    (s[0] && s[1] && s[2] && !s[3] && !s[4] && !s[5] && !s[6])||
    (s[0] && s[1] && !s[2] && !s[3] && !s[4] && !s[5] && !s[6])||
    (s[0] && s[1] && s[2] && s[3] && s[4] && !s[5] && !s[6])  ||
    (!s[0] && s[1] && s[2] && s[3] && !s[4] && !s[5] && !s[6])||
    (!s[0] && s[1] && s[2] && !s[3] && !s[4] && !s[5] && !s[6])||
    (s[0] && !s[1] && !s[2] && !s[3] && !s[4] && !s[5] && !s[6]);

  bool leftTurn =
    (!s[0] && !s[1] && s[2] && s[3] && !s[4] && s[5] && s[6]) ||
    (!s[0] && !s[1] && s[2] && s[3] && !s[4] && !s[5] && s[6])||
    (!s[0] && !s[1] && s[2] && s[3] && s[4] && !s[5] && s[6]) ||
    (s[6] && s[5] && s[4] && s[3] && s[2] && !s[1] && !s[0])  ||
    (s[6] && s[5] && s[4] && s[3] && s[2] && s[1] && !s[0])   ||
    (s[6] && s[5] && s[4] && s[3] && !s[2] && !s[1] && !s[0]) ||
    (s[6] && s[5] && s[4] && !s[3] && !s[2] && !s[1] && !s[0])||
    (!s[6] && s[5] && s[4] && s[3] && !s[2] && !s[1] && !s[0])||
    (!s[6] && s[5] && s[4] && !s[3] && !s[2] && !s[1] && !s[0])||
    (s[6] && s[5] && !s[4] && !s[3] && !s[2] && !s[1] && !s[0])||
    (s[6] && !s[5] && !s[4] && !s[3] && !s[2] && !s[1] && !s[0]);

  // ── Execute ───────────────────────────────────────────────

  if (lineLost) {
    // ── Recovery — spin toward last known side ──────────────
    Serial.print("[LOST] Recovering toward: ");
    Serial.println(lastSide == RIGHT ? "RIGHT" : "LEFT");

    if (lastSide == RIGHT) turnRight();
    else                   turnLeft();

    // Spin until ANY sensor sees black again
    do {
      qtr.readLineBlack(sensorValues);
    } while (sensorValues[0] < threshold &&
             sensorValues[1] < threshold &&
             sensorValues[2] < threshold &&
             sensorValues[3] < threshold &&
             sensorValues[4] < threshold &&
             sensorValues[5] < threshold &&
             sensorValues[6] < threshold);

    // Reset PID state so re-entry is clean
    previousError = 0;
    I = 0;
    Serial.println("[RECOVERED]");

  } else if (rightTurn) {
    Serial.println("[TURN] Right");
    turnRight();
    do { qtr.readLineBlack(sensorValues); }
    while (sensorValues[3] < threshold);

  } else if (leftTurn) {
    Serial.println("[TURN] Left");
    turnLeft();
    do { qtr.readLineBlack(sensorValues); }
    while (sensorValues[3] < threshold);

  } else {
    lineFollow();
  }
}

// ── PID Line Follow ───────────────────────────────────────────
void lineFollow() {
  int error = position - 3000;

  P = error;
  I = constrain(I + error, -3000, 3000);
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  motorLeft (constrain(speed + PIDvalue, 0, 255));
  motorRight(constrain(speed - PIDvalue, 0, 255));

  Serial.print("E:"); Serial.println(error);
}
