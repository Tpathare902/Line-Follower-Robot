/**
 * @file Line-Follower-Robot.ino
 * @brief Professional Line Follower Robot Control System
 * 
 * This firmware implements a complete line-following system with:
 * - Proper PID controller implementation
 * - Time-based control loop (DT correction)
 * - Robust sensor calibration and validation
 * - Comprehensive error handling
 * - Configurable tuning parameters
 * 
 * @author Tpathare902
 * @date 2026-05-08
 */

#include <QTRSensors.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// Motor Control Pins (Motor A - Left)
#define PWM_LEFT    9
#define AIN1        4
#define AIN2        3

// Motor Control Pins (Motor B - Right)
#define PWM_RIGHT   10
#define BIN1        6
#define BIN2        7

// System Pins
#define STANDBY     5
#define BUTTON      11
#define LED_STATUS  13

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

QTRSensors qtr;
const uint8_t SENSOR_COUNT = 7;
const uint8_t EMITTER_PIN = 2;
uint16_t sensorValues[SENSOR_COUNT];

// ============================================================================
// PID CONTROLLER CONFIGURATION
// ============================================================================

struct PIDConfig {
  float Kp;              // Proportional gain
  float Ki;              // Integral gain
  float Kd;              // Derivative gain
  float integralMax;     // Anti-windup limit
  float integralMin;
  float outputMax;       // Output saturation limits
  float outputMin;
};

// Tunable PID parameters
PIDConfig pidConfig = {
  .Kp           = 0.19f,
  .Ki           = 0.02f,    // Small integral for steady-state correction
  .Kd           = 0.35f,
  .integralMax  = 2000.0f,
  .integralMin  = -2000.0f,
  .outputMax    = 255.0f,
  .outputMin    = -255.0f
};

// ============================================================================
// ROBOT CONFIGURATION
// ============================================================================

struct RobotConfig {
  int baseSpeed;
  int sensorThreshold;
  float maxDt;           // Maximum allowed DT
  unsigned long loopInterval;  // Target loop time (microseconds)
};

RobotConfig robotConfig = {
  .baseSpeed        = 195,
  .sensorThreshold  = 600,
  .maxDt            = 0.1f,    // 100ms safety limit
  .loopInterval     = 20000    // 20ms target (50 Hz)
};

// ============================================================================
// RUNTIME STATE VARIABLES
// ============================================================================

struct ControllerState {
  float integral;
  float previousError;
  unsigned long lastTime;
  int lastValidPosition;
  uint8_t lineLostCounter;
};

ControllerState controller = {
  .integral         = 0.0f,
  .previousError    = 0.0f,
  .lastTime         = 0,
  .lastValidPosition = 3000,   // Center position
  .lineLostCounter  = 0
};

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void setupHardware(void);
void setupSensors(void);
void calibrateSensors(void);
void motorLeft(int speed);
void motorRight(int speed);
void stopMotors(void);
void readLinePosition(int& position, bool& isValid);
float computePID(float error, float dt);
void controlMotors(float pidOutput);
void handleLineLost(void);
void printDebugInfo(float dt, int position, float error, float pidOutput);

// ============================================================================
// SETUP & INITIALIZATION
// ============================================================================

void setup() {
  Serial.begin(9600);
  delay(100);
  
  Serial.println(F("\n=== Line Follower Robot - Initialization ===\n"));
  
  setupHardware();
  setupSensors();
  calibrateSensors();
  
  Serial.println(F("[System Ready - Press Button to Start]\n"));
  
  // Wait for button press
  while (digitalRead(BUTTON) == HIGH) {
    delay(10);
  }
  delay(50);
  
  Serial.println(F("[Starting Control Loop]\n"));
  controller.lastTime = micros();
}

// ============================================================================
// MAIN CONTROL LOOP
// ============================================================================

void loop() {
  // Time-based control loop
  unsigned long now = micros();
  float dt = (now - controller.lastTime) / 1000000.0f;
  controller.lastTime = now;
  
  // Safety guard: cap dt at maximum allowed
  if (dt > robotConfig.maxDt) {
    dt = robotConfig.maxDt;
  }
  if (dt <= 0.0f) {
    dt = 0.001f;  // Fallback on timer rollover
  }
  
  // Read sensor data
  int position = 0;
  bool isValid = false;
  readLinePosition(position, isValid);
  
  // Handle line lost condition
  if (!isValid) {
    handleLineLost();
    return;
  }
  
  // Reset line lost counter
  controller.lineLostCounter = 0;
  
  // Update last valid position for recovery
  controller.lastValidPosition = position;
  
  // Compute PID error and output
  int error = position - 3000;  // Target center: position = 3000
  float pidOutput = computePID((float)error, dt);
  
  // Apply motor control
  controlMotors(pidOutput);
  
  // Debug output (uncomment for tuning)
  // printDebugInfo(dt, position, (float)error, pidOutput);
}

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================

void setupHardware(void) {
  // Motor A pins
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  
  // Motor B pins
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  
  // System pins
  pinMode(STANDBY, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED_STATUS, OUTPUT);
  
  // Enable motor driver
  digitalWrite(STANDBY, HIGH);
  stopMotors();
  
  Serial.println(F("[Hardware initialized]"));
}

void setupSensors(void) {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6}, SENSOR_COUNT);
  qtr.setEmitterPin(EMITTER_PIN);
  
  delay(500);
  Serial.println(F("[QTR Sensor initialized]"));
}

void calibrateSensors(void) {
  digitalWrite(LED_STATUS, HIGH);
  Serial.println(F("[Calibrating - 5 seconds - Move robot over line]"));
  
  unsigned long calStart = millis();
  while (millis() - calStart < 5000) {
    qtr.calibrate();
    delay(20);
  }
  
  digitalWrite(LED_STATUS, LOW);
  stopMotors();
  Serial.println(F("[Calibration Complete]\n"));
  
  // Print calibration results
  Serial.println(F("Sensor Calibration Values:"));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(F("  S")); Serial.print(i);
    Serial.print(F(": min=")); Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(F(", max=")); Serial.println(qtr.calibrationOn.maximum[i]);
  }
  Serial.println();
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

/**
 * @brief Control left motor (Motor A)
 * @param speed PWM speed (-255 to +255, negative = reverse)
 */
void motorLeft(int speed) {
  // Clamp to valid range
  speed = constrain(speed, -255, 255);
  
  if (speed > 0) {
    // Forward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWM_LEFT, speed);
  } else if (speed < 0) {
    // Reverse
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWM_LEFT, -speed);
  } else {
    // Stop
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWM_LEFT, 0);
  }
}

/**
 * @brief Control right motor (Motor B)
 * @param speed PWM speed (-255 to +255, negative = reverse)
 */
void motorRight(int speed) {
  // Clamp to valid range
  speed = constrain(speed, -255, 255);
  
  if (speed > 0) {
    // Forward
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWM_RIGHT, speed);
  } else if (speed < 0) {
    // Reverse
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWM_RIGHT, -speed);
  } else {
    // Stop
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWM_RIGHT, 0);
  }
}

void stopMotors(void) {
  motorLeft(0);
  motorRight(0);
}

/**
 * @brief Apply differential drive control
 * @param pidOutput Control signal from PID (-255 to +255)
 */
void controlMotors(float pidOutput) {
  // Differential drive: left = base - pid, right = base + pid
  float leftSpeed = robotConfig.baseSpeed - pidOutput;
  float rightSpeed = robotConfig.baseSpeed + pidOutput;
  
  motorLeft((int)leftSpeed);
  motorRight((int)rightSpeed);
}

// ============================================================================
// SENSOR PROCESSING
// ============================================================================

/**
 * @brief Read and process sensor line position
 * @param position Output: weighted sensor position (0-6000)
 * @param isValid Output: true if line detected
 */
void readLinePosition(int& position, bool& isValid) {
  qtr.readCalibrated(sensorValues);
  
  long weightedSum = 0;
  long activeCount = 0;
  
  // Calculate weighted position
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] > robotConfig.sensorThreshold) {
      weightedSum += (long)i * 1000 * sensorValues[i];
      activeCount += sensorValues[i];
    }
  }
  
  if (activeCount == 0) {
    isValid = false;
    position = controller.lastValidPosition;
  } else {
    isValid = true;
    position = (int)(weightedSum / activeCount);
  }
}

// ============================================================================
// PID CONTROL IMPLEMENTATION
// ============================================================================

/**
 * @brief Compute PID control output
 * @param error Current error from setpoint
 * @param dt Time delta in seconds
 * @return PID output (-255 to +255)
 */
float computePID(float error, float dt) {
  // Proportional term
  float P = pidConfig.Kp * error;
  
  // Integral term (with anti-windup)
  controller.integral += error * dt;
  controller.integral = constrain(
    controller.integral,
    pidConfig.integralMin / pidConfig.Ki,
    pidConfig.integralMax / pidConfig.Ki
  );
  float I = pidConfig.Ki * controller.integral;
  
  // Derivative term (scaled by dt)
  float D = pidConfig.Kd * (error - controller.previousError) / dt;
  controller.previousError = error;
  
  // Sum PID components
  float output = P + I + D;
  
  // Output saturation
  output = constrain(output, pidConfig.outputMin, pidConfig.outputMax);
  
  return output;
}

// ============================================================================
// ERROR HANDLING
// ============================================================================

/**
 * @brief Handle line lost condition
 * Implements recovery strategy to relocate the line
 */
void handleLineLost(void) {
  controller.lineLostCounter++;
  
  // If line lost for extended period, stop
  if (controller.lineLostCounter > 50) {
    stopMotors();
    Serial.println(F("[ERROR: Line lost - stopping]"));
    while (true) { delay(10); }  // Halt
  }
  
  // Gradual turn based on last known error
  if (controller.previousError > 0) {
    // Line was to the right, turn right
    motorLeft(robotConfig.baseSpeed);
    motorRight(robotConfig.baseSpeed / 2);
  } else {
    // Line was to the left, turn left
    motorLeft(robotConfig.baseSpeed / 2);
    motorRight(robotConfig.baseSpeed);
  }
}

// ============================================================================
// DEBUG & DIAGNOSTICS
// ============================================================================

/**
 * @brief Print debug information (for tuning)
 */
void printDebugInfo(float dt, int position, float error, float pidOutput) {
  Serial.print(F("DT:")); Serial.print(dt * 1000, 2); Serial.print(F("ms | "));
  Serial.print(F("POS:")); Serial.print(position); Serial.print(F(" | "));
  Serial.print(F("ERR:")); Serial.print((int)error); Serial.print(F(" | "));
  Serial.print(F("PID:")); Serial.print(pidOutput, 2); Serial.print(F(" | "));
  Serial.print(F("INT:")); Serial.println(controller.integral, 1);
}
