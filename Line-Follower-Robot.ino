#include <QTRSensors.h>
#define PWM1  9
#define AIN1  4
#define AIN2  3
#define PWM2  10
#define BIN1  6
#define BIN2  7
#define STDBY 5
#define BUTTON 11

QTRSensors qtr;
const uint8_t SensorCount = 7;
uint16_t sensorValues[SensorCount];

float Kp = 0.19;//,o.388;//0.32,2.8
float Ki = 0.0;//0.9
float Kd = 0.35;//0.99,0.89,96,35

int   baseSpeed    = 195;//150
int   threshold    = 600;
int   previousError = 0;
long  I = 0;
unsigned long lastTime = 0;   // ← added for dt

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

void stopMotors() {
  motorLeft(0);
  motorRight(0);
}

void setup() {
  Serial.begin(9600);

  pinMode(AIN1,  OUTPUT); pinMode(AIN2,  OUTPUT); pinMode(PWM1, OUTPUT);
  pinMode(BIN1,  OUTPUT); pinMode(BIN2,  OUTPUT); pinMode(PWM2, OUTPUT);
  pinMode(STDBY, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(13, OUTPUT);              // ← LED pin

  digitalWrite(STDBY, HIGH);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  Serial.println("[CALIBRATING — 4 sec]");
  digitalWrite(13, HIGH);           // ← LED ON when calibration starts

  unsigned long calStart = millis();
  while (millis() - calStart < 5000) {
    qtr.calibrate();
  }

  stopMotors();
  digitalWrite(13, LOW);            // ← LED OFF when calibration done
  Serial.println("[CALIBRATION DONE]");

  for (int i = 0; i < SensorCount; i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print(" min="); Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(" max="); Serial.println(qtr.calibrationOn.maximum[i]);
  }

  Serial.println("[WAITING FOR BUTTON on pin 11...]");
  while (digitalRead(BUTTON) == HIGH) { /* wait */ }
  delay(50);

  Serial.println("[GO!]");
  lastTime = micros();
}

void loop() {
  // ── dt ─────────────────────────────────────────────────────────────────────
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0f;   // convert µs → seconds
  if (dt <= 0) dt = 0.001f;                   // guard against zero / rollover
  lastTime = now;

  // ── Sensors ────────────────────────────────────────────────────────────────
  qtr.readCalibrated(sensorValues);

  long weightedSum = 0;
  long activeCount = 0;
  for (int i = 0; i < 7; i++) {
    if (sensorValues[i] > threshold) {
      weightedSum += (long)i * 1000 * sensorValues[i];
      activeCount += sensorValues[i];
    }
  }

 if (activeCount == 0) {
    if (previousError > 0) {
        motorLeft(-baseSpeed); motorRight( baseSpeed); // ← swapped
    } else {
        motorLeft( baseSpeed); motorRight(-baseSpeed); // ← swapped
    }
    return;
}

  // ── PID with dt ────────────────────────────────────────────────────────────
  int position = weightedSum / activeCount;
  int error    = position - 3000;

  I = constrain(I + (long)(error * dt * 1000), -3000, 3000);  // ← scaled by dt
  float D = (error - previousError) ;                      // ← scaled by dt

  float PIDvalue = (Kp * error)  + (Kd * D);
  previousError = error;

  motorLeft (constrain(baseSpeed - PIDvalue, 0, 255));
  motorRight(constrain(baseSpeed + PIDvalue, 0, 255));

 // Serial.print("dt:"); Serial.print(dt * 1000, 2); Serial.print("ms  ");
  //Serial.print("POS:"); Serial.print(position);
  //Serial.print("  ERR:"); Serial.println(error);
}
