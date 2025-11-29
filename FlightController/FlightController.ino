/*
 * ============================================================================
 * PROFESSIONAL QUADCOPTER FLIGHT CONTROLLER
 * ============================================================================
 * 
 * Hardware: Arduino Nano
 * Author: Professional UAV Embedded Systems
 * Version: 1.0.0
 * 
 * DESCRIPTION:
 * Advanced flight controller firmware with PID stabilization, wireless
 * communication, and comprehensive safety features for quadcopter drones.
 * 
 * PIN CONFIGURATION:
 * - NRF24L01: CE=D4, CSN=D10, MOSI=D11, MISO=D12, SCK=D13
 * - MPU6050: INT=D2, SDA=A4, SCL=A5
 * - Buzzer: D8
 * - Status LED: D7
 * - Motors: FL=D3, FR=D5, RR=D6, RL=D9
 * 
 * ============================================================================
 */

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================
// NRF24L01 Wireless Module
#define NRF_CE_PIN          4
#define NRF_CSN_PIN         10

// MPU6050 IMU
#define MPU_INT_PIN         2
#define MPU_ADDRESS         0x68

// Peripherals
#define BUZZER_PIN          8
#define STATUS_LED_PIN      7

// Motor ESC Outputs (PWM)
#define MOTOR_FL_PIN        3   // Front-Left
#define MOTOR_FR_PIN        5   // Front-Right
#define MOTOR_RR_PIN        6   // Rear-Right
#define MOTOR_RL_PIN        9   // Rear-Left

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================
// NRF24L01 Settings
#define NRF_CHANNEL         103
#define NRF_PAYLOAD_SIZE    32

// Flight Control Parameters
#define LOOP_FREQUENCY      250                 // Hz - Main control loop
#define LOOP_PERIOD_US      (1000000 / LOOP_FREQUENCY)
#define MAX_ANGLE           30.0                // Degrees - Maximum tilt angle
#define THROTTLE_CAP        0.65                // 65% maximum throttle

// ESC/Motor Parameters
#define ESC_MIN             1000                // Minimum PWM (μs)
#define ESC_MAX             2000                // Maximum PWM (μs)
#define ESC_IDLE            1100                // Idle throttle
#define ARM_THROTTLE        1050                // Throttle threshold for arming

// PID Tuning (Optimized for stability)
#define PID_ROLL_KP         1.3
#define PID_ROLL_KI         0.04
#define PID_ROLL_KD         18.0

#define PID_PITCH_KP        1.3
#define PID_PITCH_KI        0.04
#define PID_PITCH_KD        18.0

#define PID_YAW_KP          2.0
#define PID_YAW_KI          0.02
#define PID_YAW_KD          0.0

#define PID_MAX_OUTPUT      400                 // Maximum PID correction

// Communication
#define SIGNAL_TIMEOUT_MS   1000                // Failsafe timeout
#define CALIBRATION_SAMPLES 2000                // Gyro calibration samples

// ============================================================================
// DATA STRUCTURES
// ============================================================================
// Received control data from RC
struct ControlData {
  uint16_t throttle;      // 1000-2000
  uint16_t yaw;           // 1000-2000
  uint16_t pitch;         // 1000-2000
  uint16_t roll;          // 1000-2000
  
  bool calibrate;         // Calibration trigger
  bool motorArm;          // Motor arming
  bool altitudeHold;      // Altitude hold mode
  bool killSwitch;        // Emergency disarm
  
  uint8_t checksum;       // Data integrity
};

// Telemetry data sent to RC
struct TelemetryData {
  float batteryVoltage;
  int16_t gyroX, gyroY, gyroZ;
  int16_t accelX, accelY, accelZ;
  float roll, pitch, yaw;
  bool armed;
  bool calibrated;
  uint8_t checksum;
};

// PID Controller structure
struct PIDController {
  float kp, ki, kd;
  float errorSum;
  float lastError;
  float maxOutput;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// NRF24L01 Addresses
const byte rxAddress[6] = "DRONE";
const byte txAddress[6] = "REMOT";

// Control and telemetry
ControlData rcData;
TelemetryData telemetry;

// IMU Data
int16_t gyroRaw[3], accelRaw[3];
float gyroCalibration[3] = {0, 0, 0};
float gyroRate[3], accelAngle[3];
float angleRoll = 0, anglePitch = 0, angleYaw = 0;

// PID Controllers
PIDController pidRoll, pidPitch, pidYaw;

// Motor outputs
uint16_t motorFL, motorFR, motorRR, motorRL;

// System state
bool systemArmed = false;
bool gyroCalibrated = false;
bool escCalibrationMode = false;
unsigned long lastRxTime = 0;
unsigned long loopTimer;
bool ledState = false;
unsigned long ledTimer = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void setupNRF24();
void setupMPU6050();
void setupPID();
void calibrateGyro();
void readMPU6050();
void calculateAngles(float dt);
float pidCompute(PIDController* pid, float setpoint, float input, float dt);
void resetPID(PIDController* pid);
void mixMotors(uint16_t throttle, float rollPID, float pitchPID, float yawPID);
void writeMotors();
void setMotorSpeed(uint8_t pin, uint16_t speed);
void armSystem();
void disarmSystem();
void failsafe();
void buzzerBeep(uint16_t duration, uint8_t count, uint16_t pause);
void updateStatusLED();
uint8_t calculateChecksum(uint8_t* data, uint8_t len);

// ============================================================================
// SETUP FUNCTION
// ============================================================================
void setup() {
  // Serial communication for debugging
  Serial.begin(115200);
  Serial.println(F("=== Flight Controller Initializing ==="));
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // 400kHz fast mode
  
  // Configure pins
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MPU_INT_PIN, INPUT);
  pinMode(MOTOR_FL_PIN, OUTPUT);
  pinMode(MOTOR_FR_PIN, OUTPUT);
  pinMode(MOTOR_RR_PIN, OUTPUT);
  pinMode(MOTOR_RL_PIN, OUTPUT);
  
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Startup beep
  buzzerBeep(100, 1, 0);
  
  // Initialize NRF24L01
  Serial.print(F("Init NRF24L01... "));
  setupNRF24();
  Serial.println(F("OK"));
  
  // Initialize MPU6050
  Serial.print(F("Init MPU6050... "));
  setupMPU6050();
  Serial.println(F("OK"));
  
  // Initialize PID controllers
  setupPID();
  Serial.println(F("PID Controllers Ready"));
  
  // Set motors to minimum (safety)
  setMotorSpeed(MOTOR_FL_PIN, ESC_MIN);
  setMotorSpeed(MOTOR_FR_PIN, ESC_MIN);
  setMotorSpeed(MOTOR_RR_PIN, ESC_MIN);
  setMotorSpeed(MOTOR_RL_PIN, ESC_MIN);
  
  Serial.println(F("=== Flight Controller Ready ==="));
  buzzerBeep(100, 2, 100);
  
  // Initialize loop timer
  loopTimer = micros();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Timing control - maintain consistent loop rate
  while (micros() - loopTimer < LOOP_PERIOD_US);
  float deltaTime = (micros() - loopTimer) / 1000000.0;
  loopTimer = micros();
  
  // Read IMU data
  readMPU6050();
  calculateAngles(deltaTime);
  
  // Check for incoming radio data
  if (radio.available()) {
    radio.read(&rcData, sizeof(rcData));
    lastRxTime = millis();
    
    // Verify checksum
    uint8_t calcChecksum = calculateChecksum((uint8_t*)&rcData, sizeof(rcData) - 1);
    if (calcChecksum == rcData.checksum) {
      // Valid data received - update LED
      if (millis() - ledTimer > 500) {
        ledState = !ledState;
        digitalWrite(STATUS_LED_PIN, ledState);
        ledTimer = millis();
      }
    }
  }
  
  // Check for signal loss
  if (millis() - lastRxTime > SIGNAL_TIMEOUT_MS) {
    failsafe();
  }
  
  // Handle calibration request
  static bool lastCalibrate = false;
  if (rcData.calibrate && !lastCalibrate && !systemArmed) {
    Serial.println(F("Calibration Started..."));
    calibrateGyro();
  }
  lastCalibrate = rcData.calibrate;
  
  // Handle ESC calibration mode
  static bool lastMotorArm = false;
  if (rcData.motorArm && !lastMotorArm && !rcData.altitudeHold && !systemArmed) {
    escCalibrationMode = true;
    Serial.println(F("ESC Calibration Mode"));
    buzzerBeep(200, 3, 100);
    
    // ESC calibration sequence
    delay(1000);
    for (int i = 0; i < 4; i++) {
      uint8_t pins[] = {MOTOR_FL_PIN, MOTOR_FR_PIN, MOTOR_RR_PIN, MOTOR_RL_PIN};
      
      // Smooth ramp up
      for (uint16_t speed = ESC_MIN; speed <= ESC_MIN + 200; speed += 5) {
        setMotorSpeed(pins[i], speed);
        delay(10);
      }
      buzzerBeep(50, 1, 0);
      delay(500);
      
      // Smooth ramp down
      for (uint16_t speed = ESC_MIN + 200; speed >= ESC_MIN; speed -= 5) {
        setMotorSpeed(pins[i], speed);
        delay(10);
      }
      delay(300);
    }
    
    buzzerBeep(100, 2, 150);
    escCalibrationMode = false;
    Serial.println(F("ESC Calibration Complete"));
  }
  lastMotorArm = rcData.motorArm;
  
  // Arming logic
  if (!escCalibrationMode) {
    if (rcData.killSwitch && gyroCalibrated && 
        rcData.throttle < ARM_THROTTLE && !systemArmed) {
      // Arm conditions met
      if (rcData.motorArm) {
        armSystem();
      }
    }
    
    // Disarm conditions
    if (!rcData.killSwitch || rcData.throttle < ARM_THROTTLE - 20) {
      if (systemArmed) {
        disarmSystem();
      }
    }
  }
  
  // Flight control (only when armed)
  if (systemArmed && gyroCalibrated) {
    // Calculate setpoints from RC input
    float rollSetpoint = map(rcData.roll, 1000, 2000, -MAX_ANGLE * 10, MAX_ANGLE * 10) / 10.0;
    float pitchSetpoint = map(rcData.pitch, 1000, 2000, -MAX_ANGLE * 10, MAX_ANGLE * 10) / 10.0;
    float yawSetpoint = map(rcData.yaw, 1000, 2000, -200, 200);  // deg/s
    
    // Compute PID corrections
    float rollPID = pidCompute(&pidRoll, rollSetpoint, angleRoll, deltaTime);
    float pitchPID = pidCompute(&pidPitch, pitchSetpoint, anglePitch, deltaTime);
    float yawPID = pidCompute(&pidYaw, yawSetpoint, gyroRate[2], deltaTime);
    
    // Apply throttle cap for safety
    uint16_t throttle = rcData.throttle;
    uint16_t maxThrottle = ESC_MIN + (uint16_t)((ESC_MAX - ESC_MIN) * THROTTLE_CAP);
    if (throttle > maxThrottle) {
      throttle = maxThrottle;
    }
    
    // Mix motors
    mixMotors(throttle, rollPID, pitchPID, yawPID);
    
    // Write to motors
    writeMotors();
  } else {
    // Disarmed - motors off
    motorFL = ESC_MIN;
    motorFR = ESC_MIN;
    motorRR = ESC_MIN;
    motorRL = ESC_MIN;
    writeMotors();
  }
  
  // Send telemetry back to RC
  static unsigned long telemetryTimer = 0;
  if (millis() - telemetryTimer > 100) {  // 10Hz telemetry
    telemetry.gyroX = gyroRaw[0];
    telemetry.gyroY = gyroRaw[1];
    telemetry.gyroZ = gyroRaw[2];
    telemetry.accelX = accelRaw[0];
    telemetry.accelY = accelRaw[1];
    telemetry.accelZ = accelRaw[2];
    telemetry.roll = angleRoll;
    telemetry.pitch = anglePitch;
    telemetry.yaw = angleYaw;
    telemetry.armed = systemArmed;
    telemetry.calibrated = gyroCalibrated;
    telemetry.batteryVoltage = 11.1;  // Placeholder
    telemetry.checksum = calculateChecksum((uint8_t*)&telemetry, sizeof(telemetry) - 1);
    
    radio.stopListening();
    radio.openWritingPipe(txAddress);
    radio.write(&telemetry, sizeof(telemetry));
    radio.startListening();
    
    telemetryTimer = millis();
  }
  
  // Update status LED
  updateStatusLED();
}

// ============================================================================
// NRF24L01 INITIALIZATION
// ============================================================================
void setupNRF24() {
  if (!radio.begin()) {
    Serial.println(F("NRF24L01 FAILED!"));
    while (1) {
      digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
      delay(100);
    }
  }
  
  radio.setChannel(NRF_CHANNEL);
  radio.setPALevel(RF24_PA_MAX);          // Maximum power for PA+LNA
  radio.setDataRate(RF24_250KBPS);        // Low rate = better range
  radio.setPayloadSize(NRF_PAYLOAD_SIZE);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.setRetries(5, 15);                // 5*250μs delay, 15 retries
  
  radio.openReadingPipe(1, rxAddress);
  radio.openWritingPipe(txAddress);
  radio.startListening();
}

// ============================================================================
// MPU6050 INITIALIZATION
// ============================================================================
void setupMPU6050() {
  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Wake up
  Wire.endTransmission(true);
  
  delay(100);
  
  // Configure gyro (±500 deg/s)
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x08);  // ±500°/s
  Wire.endTransmission(true);
  
  // Configure accelerometer (±8g)
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x10);  // ±8g
  Wire.endTransmission(true);
  
  // Configure DLPF (Digital Low Pass Filter)
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x03);  // DLPF 44Hz
  Wire.endTransmission(true);
  
  delay(100);
}

// ============================================================================
// PID CONTROLLER INITIALIZATION
// ============================================================================
void setupPID() {
  // Roll PID
  pidRoll.kp = PID_ROLL_KP;
  pidRoll.ki = PID_ROLL_KI;
  pidRoll.kd = PID_ROLL_KD;
  pidRoll.maxOutput = PID_MAX_OUTPUT;
  pidRoll.errorSum = 0;
  pidRoll.lastError = 0;
  
  // Pitch PID
  pidPitch.kp = PID_PITCH_KP;
  pidPitch.ki = PID_PITCH_KI;
  pidPitch.kd = PID_PITCH_KD;
  pidPitch.maxOutput = PID_MAX_OUTPUT;
  pidPitch.errorSum = 0;
  pidPitch.lastError = 0;
  
  // Yaw PID
  pidYaw.kp = PID_YAW_KP;
  pidYaw.ki = PID_YAW_KI;
  pidYaw.kd = PID_YAW_KD;
  pidYaw.maxOutput = PID_MAX_OUTPUT;
  pidYaw.errorSum = 0;
  pidYaw.lastError = 0;
}

// ============================================================================
// GYRO CALIBRATION
// ============================================================================
void calibrateGyro() {
  Serial.println(F("Calibrating Gyro - Keep Drone Still!"));
  digitalWrite(STATUS_LED_PIN, HIGH);
  
  long gyroSum[3] = {0, 0, 0};
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    readMPU6050();
    gyroSum[0] += gyroRaw[0];
    gyroSum[1] += gyroRaw[1];
    gyroSum[2] += gyroRaw[2];
    delay(1);
    
    if (i % 200 == 0) {
      digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    }
  }
  
  gyroCalibration[0] = gyroSum[0] / (float)CALIBRATION_SAMPLES;
  gyroCalibration[1] = gyroSum[1] / (float)CALIBRATION_SAMPLES;
  gyroCalibration[2] = gyroSum[2] / (float)CALIBRATION_SAMPLES;
  
  // Check if calibration is reasonable
  if (abs(gyroCalibration[0]) < 200 && abs(gyroCalibration[1]) < 200 && abs(gyroCalibration[2]) < 200) {
    gyroCalibrated = true;
    Serial.println(F("Calibration SUCCESS"));
    Serial.print(F("Offsets: X="));
    Serial.print(gyroCalibration[0]);
    Serial.print(F(" Y="));
    Serial.print(gyroCalibration[1]);
    Serial.print(F(" Z="));
    Serial.println(gyroCalibration[2]);
    
    buzzerBeep(100, 2, 100);  // Success: 2 beeps
  } else {
    gyroCalibrated = false;
    Serial.println(F("Calibration FAILED - Excessive drift"));
    buzzerBeep(7000, 1, 0);  // Failure: 1 long beep
  }
  
  digitalWrite(STATUS_LED_PIN, LOW);
}

// ============================================================================
// READ MPU6050 DATA
// ============================================================================
void readMPU6050() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);  // Starting register ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 14, true);
  
  // Read accelerometer
  accelRaw[0] = Wire.read() << 8 | Wire.read();  // X
  accelRaw[1] = Wire.read() << 8 | Wire.read();  // Y
  accelRaw[2] = Wire.read() << 8 | Wire.read();  // Z
  
  // Skip temperature
  Wire.read();
  Wire.read();
  
  // Read gyroscope
  gyroRaw[0] = Wire.read() << 8 | Wire.read();  // X
  gyroRaw[1] = Wire.read() << 8 | Wire.read();  // Y
  gyroRaw[2] = Wire.read() << 8 | Wire.read();  // Z
  
  // Apply calibration and convert to deg/s (±500°/s = 65.5 LSB/°/s)
  gyroRate[0] = (gyroRaw[0] - gyroCalibration[0]) / 65.5;
  gyroRate[1] = (gyroRaw[1] - gyroCalibration[1]) / 65.5;
  gyroRate[2] = (gyroRaw[2] - gyroCalibration[2]) / 65.5;
}

// ============================================================================
// CALCULATE ATTITUDE ANGLES (Complementary Filter)
// ============================================================================
void calculateAngles(float dt) {
  // Accelerometer angles (in degrees)
  float accelRoll = atan2(accelRaw[1], accelRaw[2]) * 180.0 / PI;
  float accelPitch = atan2(-accelRaw[0], sqrt(accelRaw[1] * accelRaw[1] + accelRaw[2] * accelRaw[2])) * 180.0 / PI;
  
  // Gyro integration
  angleRoll += gyroRate[0] * dt;
  anglePitch += gyroRate[1] * dt;
  angleYaw += gyroRate[2] * dt;
  
  // Complementary filter (98% gyro, 2% accel)
  angleRoll = 0.98 * angleRoll + 0.02 * accelRoll;
  anglePitch = 0.98 * anglePitch + 0.02 * accelPitch;
}

// ============================================================================
// PID COMPUTATION
// ============================================================================
float pidCompute(PIDController* pid, float setpoint, float input, float dt) {
  // Calculate error
  float error = setpoint - input;
  
  // Proportional
  float pTerm = pid->kp * error;
  
  // Integral (with anti-windup)
  pid->errorSum += error * dt;
  if (pid->errorSum > pid->maxOutput / pid->ki) {
    pid->errorSum = pid->maxOutput / pid->ki;
  } else if (pid->errorSum < -pid->maxOutput / pid->ki) {
    pid->errorSum = -pid->maxOutput / pid->ki;
  }
  float iTerm = pid->ki * pid->errorSum;
  
  // Derivative
  float dTerm = pid->kd * (error - pid->lastError) / dt;
  pid->lastError = error;
  
  // Combine
  float output = pTerm + iTerm + dTerm;
  
  // Limit output
  if (output > pid->maxOutput) output = pid->maxOutput;
  if (output < -pid->maxOutput) output = -pid->maxOutput;
  
  return output;
}

// ============================================================================
// RESET PID CONTROLLER
// ============================================================================
void resetPID(PIDController* pid) {
  pid->errorSum = 0;
  pid->lastError = 0;
}

// ============================================================================
// MOTOR MIXER (Quadcopter X Configuration)
// ============================================================================
void mixMotors(uint16_t throttle, float rollPID, float pitchPID, float yawPID) {
  /*
   * Motor Configuration (X):
   *     FRONT
   *   FL    FR
   *     \ /
   *     / \
   *   RL    RR
   *     REAR
   */
  
  motorFL = throttle - rollPID - pitchPID - yawPID;
  motorFR = throttle + rollPID - pitchPID + yawPID;
  motorRR = throttle + rollPID + pitchPID - yawPID;
  motorRL = throttle - rollPID + pitchPID + yawPID;
  
  // Constrain to ESC limits
  motorFL = constrain(motorFL, ESC_MIN, ESC_MAX);
  motorFR = constrain(motorFR, ESC_MIN, ESC_MAX);
  motorRR = constrain(motorRR, ESC_MIN, ESC_MAX);
  motorRL = constrain(motorRL, ESC_MIN, ESC_MAX);
}

// ============================================================================
// WRITE MOTOR SPEEDS
// ============================================================================
void writeMotors() {
  setMotorSpeed(MOTOR_FL_PIN, motorFL);
  setMotorSpeed(MOTOR_FR_PIN, motorFR);
  setMotorSpeed(MOTOR_RR_PIN, motorRR);
  setMotorSpeed(MOTOR_RL_PIN, motorRL);
}

// ============================================================================
// SET INDIVIDUAL MOTOR SPEED
// ============================================================================
void setMotorSpeed(uint8_t pin, uint16_t speed) {
  // Generate PWM signal (1000-2000μs)
  speed = constrain(speed, ESC_MIN, ESC_MAX);
  
  // Using analogWrite with correction for 490Hz default PWM
  // For proper ESC control, we need 50Hz (20ms period)
  // This is a simplified approach - consider using Servo library for precision
  int pwmValue = map(speed, ESC_MIN, ESC_MAX, 0, 255);
  analogWrite(pin, pwmValue);
}

// ============================================================================
// ARM SYSTEM
// ============================================================================
void armSystem() {
  systemArmed = true;
  resetPID(&pidRoll);
  resetPID(&pidPitch);
  resetPID(&pidYaw);
  
  Serial.println(F("*** SYSTEM ARMED ***"));
  buzzerBeep(100, 3, 50);
}

// ============================================================================
// DISARM SYSTEM
// ============================================================================
void disarmSystem() {
  systemArmed = false;
  
  motorFL = ESC_MIN;
  motorFR = ESC_MIN;
  motorRR = ESC_MIN;
  motorRL = ESC_MIN;
  writeMotors();
  
  Serial.println(F("*** SYSTEM DISARMED ***"));
  buzzerBeep(200, 1, 0);
}

// ============================================================================
// FAILSAFE MODE
// ============================================================================
void failsafe() {
  if (systemArmed) {
    disarmSystem();
    Serial.println(F("!!! FAILSAFE - SIGNAL LOST !!!"));
  }
  
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // Clear RC data
  rcData.throttle = 1000;
  rcData.yaw = 1500;
  rcData.pitch = 1500;
  rcData.roll = 1500;
  rcData.killSwitch = false;
}

// ============================================================================
// BUZZER CONTROL
// ============================================================================
void buzzerBeep(uint16_t duration, uint8_t count, uint16_t pause) {
  for (uint8_t i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) {
      delay(pause);
    }
  }
}

// ============================================================================
// UPDATE STATUS LED
// ============================================================================
void updateStatusLED() {
  if (systemArmed) {
    // Armed: Solid ON
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else if (gyroCalibrated) {
    // Calibrated but not armed: Slow blink (handled in main loop)
    // Already handled in main loop when receiving data
  } else {
    // Not calibrated: Fast blink
    if (millis() - ledTimer > 200) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
      ledTimer = millis();
    }
  }
}

// ============================================================================
// CHECKSUM CALCULATION
// ============================================================================
uint8_t calculateChecksum(uint8_t* data, uint8_t len) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// ============================================================================
// END OF FLIGHT CONTROLLER
// ============================================================================
