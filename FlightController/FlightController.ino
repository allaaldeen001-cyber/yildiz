/*
 * =====================================================================
 * PROFESSIONAL QUADCOPTER DRONE - FLIGHT CONTROLLER
 * =====================================================================
 * 
 * Hardware: Arduino Nano
 * Author: Professional Embedded Systems Engineer
 * Version: 1.0
 * Date: 2025
 * 
 * Description:
 *   Professional flight controller for quadcopter drone with IMU-based
 *   stabilization, PID control, and safety features.
 * 
 * Pin Configuration:
 *   NRF24L01: CE=D4, CSN=D10, SCK=D13, MOSI=D11, MISO=D12
 *   MPU6050: INT=D2, SDA=A4, SCL=A5
 *   Motors: FL=D3, FR=D5, RR=D6, RL=D9
 *   Buzzer: D8
 *   Status LED: D7
 * 
 * Motor Layout:
 *       FRONT
 *     FL ↻  FR ↺
 *       \ X /
 *       / X \
 *     RL ↺  RR ↻
 *       REAR
 * 
 * =====================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <MPU6050.h>

// =====================================================================
// CONFIGURATION
// =====================================================================

// NRF24L01 Configuration
#define NRF_CE_PIN      4
#define NRF_CSN_PIN     10
#define NRF_CHANNEL     103
#define NRF_PA_LEVEL    RF24_PA_MAX
#define NRF_DATA_RATE   RF24_250KBPS

// MPU6050 Configuration
#define MPU_INT_PIN     2

// Motor Pins (PWM)
#define MOTOR_FL        3   // Front Left
#define MOTOR_FR        5   // Front Right
#define MOTOR_RR        6   // Rear Right
#define MOTOR_RL        9   // Rear Left

// Peripheral Pins
#define BUZZER_PIN      8
#define LED_PIN         7

// ESC Settings
#define ESC_MIN         1000  // Minimum PWM
#define ESC_MAX         2000  // Maximum PWM
#define ESC_ARM         1000  // Arming value
#define ESC_IDLE        1100  // Idle throttle

// Safety Limits
#define MAX_ANGLE       30.0  // Maximum tilt angle (degrees)
#define THROTTLE_CAP    0.65  // 65% maximum throttle
#define MIN_ARMED_THROTTLE  1050
#define FAILSAFE_TIMEOUT    1000  // ms

// PID Tuning - Roll & Pitch
#define KP_ANGLE        2.0
#define KI_ANGLE        0.02
#define KD_ANGLE        15.0

// PID Tuning - Yaw
#define KP_YAW          3.0
#define KI_YAW          0.02
#define KD_YAW          0.0

// PID Limits
#define PID_ROLL_MAX    400
#define PID_PITCH_MAX   400
#define PID_YAW_MAX     400

// Rates (deg/s for stick input)
#define MAX_ROLL_RATE   180.0
#define MAX_PITCH_RATE  180.0
#define MAX_YAW_RATE    180.0

// Loop timing
#define LOOP_TIME       4000  // 4ms = 250Hz
#define IMU_SAMPLE_RATE 250   // Hz

// =====================================================================
// DATA STRUCTURES
// =====================================================================

// RC Data packet structure (matches Remote Controller)
struct RC_Data {
  uint16_t throttle;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
  uint8_t armed:1;
  uint8_t altHold:1;
  uint8_t calibrate:1;
  uint8_t escCal:1;
  uint8_t motorTest:1;
  uint8_t reserved:3;
  uint16_t checksum;
};

// Acknowledgment packet structure
struct FC_Ack {
  uint8_t status;
  uint8_t calResult;
  int16_t batteryVoltage;
  uint8_t rssi;
  uint16_t checksum;
};

// PID Controller structure
struct PID {
  float kp, ki, kd;
  float integral;
  float lastError;
  float maxIntegral;
  float maxOutput;
};

// IMU Data structure
struct IMU_Data {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float angleX, angleY;
  float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
};

// Motor speeds
struct Motors {
  int fl, fr, rr, rl;
};

// Flight controller state
enum FC_State {
  STATE_INIT = 0,
  STATE_IDLE,
  STATE_CALIBRATING,
  STATE_ARMED,
  STATE_FLYING,
  STATE_FAILSAFE,
  STATE_ERROR
};

enum Cal_Result {
  CAL_NONE = 0,
  CAL_SUCCESS,
  CAL_FAILED,
  CAL_IN_PROGRESS
};

// =====================================================================
// GLOBAL OBJECTS & VARIABLES
// =====================================================================

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
MPU6050 mpu;

const uint64_t pipeIn = 0xE8E8F0F0E1LL;   // RX address (matches RC TX)
const uint64_t pipeOut = 0xE8E8F0F0E2LL;  // TX address for ACK

RC_Data rcData;
FC_Ack fcAck;

// Flight controller state
FC_State currentState = STATE_INIT;
Cal_Result calibrationResult = CAL_NONE;

// PID Controllers
PID pidRoll, pidPitch, pidYaw;

// IMU data
IMU_Data imuData;

// Motor outputs
Motors motors;

// Timing
unsigned long lastLoopTime = 0;
unsigned long lastRCTime = 0;
bool rcLinkActive = false;

// Calibration
bool calibrationRequested = false;
int calibrationSamples = 0;
const int CALIBRATION_SAMPLES = 2000;  // 8 seconds at 250Hz

// ESC calibration
bool escCalRequested = false;
bool escCalDone = false;

// Motor test
bool motorTestRequested = false;

// Safety flags
bool gyroCalibrated = false;
bool motorsArmed = false;

// Battery monitoring
float batteryVoltage = 11.1;  // Default 3S LiPo

// =====================================================================
// SETUP FUNCTION
// =====================================================================

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  initializePins();
  
  // Startup sequence
  playStartupTone();
  blinkLED(3, 200);
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // 400kHz Fast Mode
  
  // Initialize MPU6050
  if (!initializeMPU()) {
    Serial.println(F("❌ MPU6050 FAILED!"));
    currentState = STATE_ERROR;
    playErrorTone();
    while (1) {
      blinkLED(1, 100);
      delay(500);
    }
  }
  
  // Initialize NRF24L01
  if (!initializeRadio()) {
    Serial.println(F("❌ NRF24L01 FAILED!"));
    currentState = STATE_ERROR;
    playErrorTone();
    while (1) {
      blinkLED(1, 100);
      delay(500);
    }
  }
  
  // Initialize PID controllers
  initializePID();
  
  // Initialize RC data with safe values
  initializeRCData();
  
  // Initialize motors to minimum
  initializeMotors();
  
  Serial.println(F("\n================================"));
  Serial.println(F("  FLIGHT CONTROLLER v1.0"));
  Serial.println(F("================================"));
  Serial.println(F("✓ MPU6050 Initialized"));
  Serial.println(F("✓ NRF24L01 Initialized"));
  Serial.println(F("✓ Motors Initialized"));
  Serial.println(F("⚠️  GYRO CALIBRATION REQUIRED"));
  Serial.println(F("================================\n"));
  
  currentState = STATE_IDLE;
  digitalWrite(LED_PIN, HIGH);
  
  delay(500);
}

// =====================================================================
// MAIN LOOP - 250Hz
// =====================================================================

void loop() {
  unsigned long currentTime = micros();
  
  // Wait for loop time (250Hz = 4000us)
  if (currentTime - lastLoopTime < LOOP_TIME) {
    return;
  }
  
  float deltaTime = (currentTime - lastLoopTime) / 1000000.0;
  lastLoopTime = currentTime;
  
  // Read IMU data
  readIMU();
  
  // Receive RC commands
  receiveRC();
  
  // Check for RC link timeout
  if (millis() - lastRCTime > FAILSAFE_TIMEOUT && motorsArmed) {
    activateFailsafe();
  }
  
  // Process flight controller state
  processState();
  
  // Calculate PID outputs
  if (motorsArmed && currentState == STATE_FLYING) {
    calculatePID(deltaTime);
    mixMotors();
  } else {
    // Motors off or idle
    stopMotors();
  }
  
  // Update motors
  updateMotors();
  
  // Update LED status
  updateStatusLED();
}

// =====================================================================
// INITIALIZATION FUNCTIONS
// =====================================================================

void initializePins() {
  // Motor pins
  pinMode(MOTOR_FL, OUTPUT);
  pinMode(MOTOR_FR, OUTPUT);
  pinMode(MOTOR_RR, OUTPUT);
  pinMode(MOTOR_RL, OUTPUT);
  
  // Peripheral pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MPU_INT_PIN, INPUT);
  
  // Set all outputs to safe state
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}

bool initializeMPU() {
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    return false;
  }
  
  // Configure MPU6050
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);    // ±500°/s
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);     // ±4g
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);                // 42Hz DLPF
  mpu.setRate(3);  // Sample rate divider: 1kHz / (1+3) = 250Hz
  
  // Enable interrupt
  mpu.setIntEnabled(0x01);
  
  delay(100);
  
  return true;
}

bool initializeRadio() {
  if (!radio.begin()) {
    return false;
  }
  
  radio.setChannel(NRF_CHANNEL);
  radio.setPALevel(NRF_PA_LEVEL);
  radio.setDataRate(NRF_DATA_RATE);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  
  radio.openReadingPipe(1, pipeIn);
  radio.openWritingPipe(pipeOut);
  
  radio.startListening();
  
  return true;
}

void initializePID() {
  // Roll PID
  pidRoll.kp = KP_ANGLE;
  pidRoll.ki = KI_ANGLE;
  pidRoll.kd = KD_ANGLE;
  pidRoll.integral = 0;
  pidRoll.lastError = 0;
  pidRoll.maxIntegral = 200;
  pidRoll.maxOutput = PID_ROLL_MAX;
  
  // Pitch PID
  pidPitch.kp = KP_ANGLE;
  pidPitch.ki = KI_ANGLE;
  pidPitch.kd = KD_ANGLE;
  pidPitch.integral = 0;
  pidPitch.lastError = 0;
  pidPitch.maxIntegral = 200;
  pidPitch.maxOutput = PID_PITCH_MAX;
  
  // Yaw PID
  pidYaw.kp = KP_YAW;
  pidYaw.ki = KI_YAW;
  pidYaw.kd = KD_YAW;
  pidYaw.integral = 0;
  pidYaw.lastError = 0;
  pidYaw.maxIntegral = 200;
  pidYaw.maxOutput = PID_YAW_MAX;
}

void initializeRCData() {
  rcData.throttle = 1000;
  rcData.yaw = 1500;
  rcData.pitch = 1500;
  rcData.roll = 1500;
  rcData.armed = 0;
  rcData.altHold = 0;
  rcData.calibrate = 0;
  rcData.escCal = 0;
  rcData.motorTest = 0;
  rcData.checksum = 0;
}

void initializeMotors() {
  // Send minimum PWM to all ESCs
  analogWrite(MOTOR_FL, ESC_MIN / 8);  // Arduino PWM is 0-255
  analogWrite(MOTOR_FR, ESC_MIN / 8);
  analogWrite(MOTOR_RR, ESC_MIN / 8);
  analogWrite(MOTOR_RL, ESC_MIN / 8);
  
  motors.fl = ESC_MIN;
  motors.fr = ESC_MIN;
  motors.rr = ESC_MIN;
  motors.rl = ESC_MIN;
  
  delay(2000);  // Wait for ESC initialization
}

// =====================================================================
// IMU FUNCTIONS
// =====================================================================

void readIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert to physical units
  imuData.accelX = ax / 8192.0;  // ±4g range
  imuData.accelY = ay / 8192.0;
  imuData.accelZ = az / 8192.0;
  
  imuData.gyroX = (gx / 65.5) - imuData.gyroOffsetX;  // ±500°/s range
  imuData.gyroY = (gy / 65.5) - imuData.gyroOffsetY;
  imuData.gyroZ = (gz / 65.5) - imuData.gyroOffsetZ;
  
  // Calculate angles using complementary filter
  float accelAngleX = atan2(imuData.accelY, imuData.accelZ) * 180.0 / PI;
  float accelAngleY = atan2(-imuData.accelX, imuData.accelZ) * 180.0 / PI;
  
  // Complementary filter (98% gyro, 2% accel)
  imuData.angleX = 0.98 * (imuData.angleX + imuData.gyroX * 0.004) + 0.02 * accelAngleX;
  imuData.angleY = 0.98 * (imuData.angleY + imuData.gyroY * 0.004) + 0.02 * accelAngleY;
}

void calibrateGyro() {
  Serial.println(F("Starting gyro calibration..."));
  Serial.println(F("Keep drone stationary!"));
  
  calibrationResult = CAL_IN_PROGRESS;
  currentState = STATE_CALIBRATING;
  
  long gxSum = 0, gySum = 0, gzSum = 0;
  int validSamples = 0;
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    gxSum += gx;
    gySum += gy;
    gzSum += gz;
    validSamples++;
    
    // Visual feedback
    if (i % 250 == 0) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      Serial.print(F("."));
    }
    
    delayMicroseconds(4000);  // 250Hz
  }
  
  Serial.println();
  
  // Calculate offsets
  imuData.gyroOffsetX = (gxSum / (float)validSamples) / 65.5;
  imuData.gyroOffsetY = (gySum / (float)validSamples) / 65.5;
  imuData.gyroOffsetZ = (gzSum / (float)validSamples) / 65.5;
  
  // Reset angles
  imuData.angleX = 0;
  imuData.angleY = 0;
  
  gyroCalibrated = true;
  calibrationResult = CAL_SUCCESS;
  
  Serial.println(F("✓ Gyro calibration complete!"));
  Serial.print(F("Offsets - X: "));
  Serial.print(imuData.gyroOffsetX);
  Serial.print(F(" Y: "));
  Serial.print(imuData.gyroOffsetY);
  Serial.print(F(" Z: "));
  Serial.println(imuData.gyroOffsetZ);
  
  // Success tone
  playCalibrationSuccessTone();
  
  currentState = STATE_IDLE;
  digitalWrite(LED_PIN, HIGH);
}

// =====================================================================
// COMMUNICATION FUNCTIONS
// =====================================================================

void receiveRC() {
  if (radio.available()) {
    radio.read(&rcData, sizeof(RC_Data));
    
    // Verify checksum
    uint16_t calcChecksum = calculateChecksum((uint8_t*)&rcData, sizeof(RC_Data) - 2);
    if (calcChecksum == rcData.checksum) {
      rcLinkActive = true;
      lastRCTime = millis();
      
      // Send acknowledgment with telemetry
      sendAcknowledgment();
      
      // Blink LED to show active link
      static unsigned long lastBlinkTime = 0;
      if (millis() - lastBlinkTime > 1000) {
        lastBlinkTime = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      }
    }
  }
}

void sendAcknowledgment() {
  fcAck.status = currentState;
  fcAck.calResult = calibrationResult;
  fcAck.batteryVoltage = batteryVoltage * 1000;  // Convert to mV
  fcAck.rssi = 85;  // Placeholder
  fcAck.checksum = calculateChecksum((uint8_t*)&fcAck, sizeof(FC_Ack) - 2);
  
  radio.writeAckPayload(1, &fcAck, sizeof(FC_Ack));
}

uint16_t calculateChecksum(uint8_t* data, size_t length) {
  uint16_t sum = 0;
  for (size_t i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum;
}

// =====================================================================
// STATE MACHINE
// =====================================================================

void processState() {
  // Handle calibration request
  if (rcData.calibrate && !calibrationRequested && currentState == STATE_IDLE) {
    calibrationRequested = true;
    calibrateGyro();
    delay(10);
    calibrationRequested = false;
  }
  
  // Handle ESC calibration request
  if (rcData.escCal && !escCalRequested && !motorsArmed) {
    escCalRequested = true;
    performESCCalibration();
    delay(10);
    escCalRequested = false;
  }
  
  // Handle motor test request
  if (rcData.motorTest && !motorTestRequested && gyroCalibrated && !motorsArmed) {
    motorTestRequested = true;
    performMotorTest();
    delay(10);
    motorTestRequested = false;
  }
  
  // Arming logic
  if (rcData.armed && gyroCalibrated && !motorsArmed && rcData.throttle < MIN_ARMED_THROTTLE) {
    // Arm the motors
    motorsArmed = true;
    currentState = STATE_ARMED;
    playArmTone();
    Serial.println(F("✓ MOTORS ARMED"));
    
    // Reset PID
    resetPID();
  } else if (!rcData.armed && motorsArmed) {
    // Disarm the motors
    motorsArmed = false;
    currentState = STATE_IDLE;
    playDisarmTone();
    Serial.println(F("✗ MOTORS DISARMED"));
  }
  
  // Update state based on throttle
  if (motorsArmed) {
    if (rcData.throttle > MIN_ARMED_THROTTLE) {
      currentState = STATE_FLYING;
    } else {
      currentState = STATE_ARMED;
    }
  }
}

void activateFailsafe() {
  Serial.println(F("⚠️  FAILSAFE ACTIVATED!"));
  currentState = STATE_FAILSAFE;
  motorsArmed = false;
  stopMotors();
  playErrorTone();
  rcLinkActive = false;
}

// =====================================================================
// PID CONTROL
// =====================================================================

float computePID(PID* pid, float setpoint, float input, float deltaTime) {
  float error = setpoint - input;
  
  // Proportional
  float pTerm = pid->kp * error;
  
  // Integral
  pid->integral += error * deltaTime;
  pid->integral = constrain(pid->integral, -pid->maxIntegral, pid->maxIntegral);
  float iTerm = pid->ki * pid->integral;
  
  // Derivative
  float dTerm = pid->kd * (error - pid->lastError) / deltaTime;
  pid->lastError = error;
  
  // Output
  float output = pTerm + iTerm + dTerm;
  output = constrain(output, -pid->maxOutput, pid->maxOutput);
  
  return output;
}

void calculatePID(float deltaTime) {
  // Convert RC inputs to desired rates
  float desiredRollRate = map_float(rcData.roll, 1000, 2000, -MAX_ROLL_RATE, MAX_ROLL_RATE);
  float desiredPitchRate = map_float(rcData.pitch, 1000, 2000, -MAX_PITCH_RATE, MAX_PITCH_RATE);
  float desiredYawRate = map_float(rcData.yaw, 1000, 2000, -MAX_YAW_RATE, MAX_YAW_RATE);
  
  // Apply deadband
  if (abs(desiredRollRate) < 5.0) desiredRollRate = 0;
  if (abs(desiredPitchRate) < 5.0) desiredPitchRate = 0;
  if (abs(desiredYawRate) < 5.0) desiredYawRate = 0;
  
  // Compute PID for rate mode (using gyro rates)
  float rollPID = computePID(&pidRoll, desiredRollRate, imuData.gyroX, deltaTime);
  float pitchPID = computePID(&pidPitch, desiredPitchRate, imuData.gyroY, deltaTime);
  float yawPID = computePID(&pidYaw, desiredYawRate, imuData.gyroZ, deltaTime);
  
  // Store for mixing
  pidRoll.lastError = rollPID;
  pidPitch.lastError = pitchPID;
  pidYaw.lastError = yawPID;
}

void resetPID() {
  pidRoll.integral = 0;
  pidRoll.lastError = 0;
  pidPitch.integral = 0;
  pidPitch.lastError = 0;
  pidYaw.integral = 0;
  pidYaw.lastError = 0;
}

// =====================================================================
// MOTOR MIXING
// =====================================================================

void mixMotors() {
  // Get throttle with safety cap
  int throttle = rcData.throttle;
  int maxThrottle = ESC_MIN + (ESC_MAX - ESC_MIN) * THROTTLE_CAP;
  throttle = constrain(throttle, ESC_MIN, maxThrottle);
  
  // Get PID outputs
  float rollPID = pidRoll.lastError;
  float pitchPID = pidPitch.lastError;
  float yawPID = pidYaw.lastError;
  
  // Motor mixing (X configuration)
  motors.fl = throttle + pitchPID + rollPID - yawPID;  // Front Left
  motors.fr = throttle + pitchPID - rollPID + yawPID;  // Front Right
  motors.rr = throttle - pitchPID - rollPID - yawPID;  // Rear Right
  motors.rl = throttle - pitchPID + rollPID + yawPID;  // Rear Left
  
  // Constrain motor outputs
  motors.fl = constrain(motors.fl, ESC_MIN, ESC_MAX);
  motors.fr = constrain(motors.fr, ESC_MIN, ESC_MAX);
  motors.rr = constrain(motors.rr, ESC_MIN, ESC_MAX);
  motors.rl = constrain(motors.rl, ESC_MIN, ESC_MAX);
}

void stopMotors() {
  motors.fl = ESC_MIN;
  motors.fr = ESC_MIN;
  motors.rr = ESC_MIN;
  motors.rl = ESC_MIN;
}

void updateMotors() {
  // Convert 1000-2000 to 0-255 for analogWrite
  analogWrite(MOTOR_FL, map(motors.fl, 1000, 2000, 0, 255));
  analogWrite(MOTOR_FR, map(motors.fr, 1000, 2000, 0, 255));
  analogWrite(MOTOR_RR, map(motors.rr, 1000, 2000, 0, 255));
  analogWrite(MOTOR_RL, map(motors.rl, 1000, 2000, 0, 255));
}

// =====================================================================
// CALIBRATION & TEST FUNCTIONS
// =====================================================================

void performESCCalibration() {
  Serial.println(F("\n=== ESC CALIBRATION ==="));
  Serial.println(F("Starting ESC calibration sequence..."));
  
  playESCCalTone();
  
  // Send high signal
  analogWrite(MOTOR_FL, 255);
  analogWrite(MOTOR_FR, 255);
  analogWrite(MOTOR_RR, 255);
  analogWrite(MOTOR_RL, 255);
  
  Serial.println(F("High signal sent. Waiting 3s..."));
  delay(3000);
  
  // Send low signal
  analogWrite(MOTOR_FL, 0);
  analogWrite(MOTOR_FR, 0);
  analogWrite(MOTOR_RR, 0);
  analogWrite(MOTOR_RL, 0);
  
  Serial.println(F("Low signal sent. Waiting 2s..."));
  delay(2000);
  
  // Spin each motor individually
  Serial.println(F("Testing motors individually..."));
  
  int testSpeed = map(1200, 1000, 2000, 0, 255);
  
  Serial.println(F("FL Motor..."));
  tone(BUZZER_PIN, 800, 200);
  analogWrite(MOTOR_FL, testSpeed);
  delay(1500);
  analogWrite(MOTOR_FL, 0);
  delay(500);
  
  Serial.println(F("FR Motor..."));
  tone(BUZZER_PIN, 900, 200);
  analogWrite(MOTOR_FR, testSpeed);
  delay(1500);
  analogWrite(MOTOR_FR, 0);
  delay(500);
  
  Serial.println(F("RR Motor..."));
  tone(BUZZER_PIN, 1000, 200);
  analogWrite(MOTOR_RR, testSpeed);
  delay(1500);
  analogWrite(MOTOR_RR, 0);
  delay(500);
  
  Serial.println(F("RL Motor..."));
  tone(BUZZER_PIN, 1100, 200);
  analogWrite(MOTOR_RL, testSpeed);
  delay(1500);
  analogWrite(MOTOR_RL, 0);
  delay(500);
  
  Serial.println(F("✓ ESC Calibration Complete!\n"));
  playCalibrationSuccessTone();
  
  escCalDone = true;
}

void performMotorTest() {
  Serial.println(F("\n=== MOTOR TEST ==="));
  Serial.println(F("Spinning all motors at minimum speed..."));
  
  tone(BUZZER_PIN, 1500, 500);
  
  int testSpeed = map(1150, 1000, 2000, 0, 255);
  
  analogWrite(MOTOR_FL, testSpeed);
  analogWrite(MOTOR_FR, testSpeed);
  analogWrite(MOTOR_RR, testSpeed);
  analogWrite(MOTOR_RL, testSpeed);
  
  delay(3000);
  
  analogWrite(MOTOR_FL, 0);
  analogWrite(MOTOR_FR, 0);
  analogWrite(MOTOR_RR, 0);
  analogWrite(MOTOR_RL, 0);
  
  Serial.println(F("✓ Motor Test Complete!\n"));
  tone(BUZZER_PIN, 2000, 300);
}

// =====================================================================
// AUDIO & VISUAL FEEDBACK
// =====================================================================

void playStartupTone() {
  tone(BUZZER_PIN, 1000, 100);
  delay(150);
  tone(BUZZER_PIN, 1500, 100);
  delay(150);
  tone(BUZZER_PIN, 2000, 100);
  delay(150);
}

void playCalibrationSuccessTone() {
  tone(BUZZER_PIN, 1500, 200);
  delay(250);
  tone(BUZZER_PIN, 1800, 200);
  delay(250);
}

void playErrorTone() {
  for (int i = 0; i < 5; i++) {
    tone(BUZZER_PIN, 500, 100);
    delay(150);
  }
}

void playArmTone() {
  tone(BUZZER_PIN, 2000, 300);
}

void playDisarmTone() {
  tone(BUZZER_PIN, 1000, 300);
}

void playESCCalTone() {
  tone(BUZZER_PIN, 1200, 300);
  delay(350);
  tone(BUZZER_PIN, 1400, 300);
  delay(350);
  tone(BUZZER_PIN, 1600, 300);
}

void blinkLED(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_PIN, LOW);
    delay(delayMs);
  }
}

void updateStatusLED() {
  static unsigned long lastToggle = 0;
  unsigned long currentTime = millis();
  
  switch (currentState) {
    case STATE_IDLE:
      digitalWrite(LED_PIN, HIGH);  // Solid on
      break;
      
    case STATE_CALIBRATING:
      if (currentTime - lastToggle > 100) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastToggle = currentTime;
      }
      break;
      
    case STATE_ARMED:
      if (currentTime - lastToggle > 500) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastToggle = currentTime;
      }
      break;
      
    case STATE_FLYING:
      digitalWrite(LED_PIN, HIGH);  // Solid on
      break;
      
    case STATE_FAILSAFE:
    case STATE_ERROR:
      if (currentTime - lastToggle > 100) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        lastToggle = currentTime;
      }
      break;
  }
}

// =====================================================================
// UTILITY FUNCTIONS
// =====================================================================

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// =====================================================================
// END OF FLIGHT CONTROLLER CODE
// =====================================================================
