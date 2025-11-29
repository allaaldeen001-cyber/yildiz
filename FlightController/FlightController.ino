/*
 * Professional Drone Flight Controller
 * Arduino Nano + NRF24L01 + MPU6050 + MS5611
 * 
 * Author: UAV Embedded Systems
 * Description: Advanced flight controller with PID stabilization,
 *              sensor fusion, and altitude hold capability
 */

// ============================================================================
// INCLUDES
// ============================================================================
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
// NRF24L01
#define NRF_CE_PIN        4
#define NRF_CSN_PIN       10

// MPU6050 (I2C: A4-SDA, A5-SCL)
#define MPU_INT_PIN       2

// Peripherals
#define BUZZER_PIN        8
#define LED_PIN           7

// Motors (ESC PWM outputs)
#define MOTOR_FL_PIN      3   // Front Left
#define MOTOR_FR_PIN      5   // Front Right
#define MOTOR_RR_PIN      6   // Rear Right
#define MOTOR_RL_PIN      9   // Rear Left

// ============================================================================
// CONSTANTS
// ============================================================================
#define NRF_CHANNEL       103
#define LOOP_FREQ         250     // Hz (4ms loop time)
#define LOOP_TIME         4000    // microseconds

// Motor limits
#define MOTOR_MIN         1000
#define MOTOR_MAX         2000
#define MOTOR_ARM_LEVEL   1050
#define THROTTLE_SAFE_MAX 1650    // 65% throttle cap for safety

// Angle limits
#define MAX_ANGLE         30.0    // Maximum tilt angle in degrees

// MPU6050 addresses
#define MPU6050_ADDR      0x68
#define MPU6050_PWR_MGMT  0x6B
#define MPU6050_ACCEL     0x3B
#define MPU6050_GYRO      0x43

// MS5611 addresses  
#define MS5611_ADDR       0x77
#define MS5611_CMD_RESET  0x1E
#define MS5611_CMD_D1     0x48    // Pressure conversion
#define MS5611_CMD_D2     0x58    // Temperature conversion
#define MS5611_CMD_ADC    0x00

// ============================================================================
// PID CONSTANTS
// ============================================================================
// Roll PID
#define KP_ROLL           1.5
#define KI_ROLL           0.05
#define KD_ROLL           15.0

// Pitch PID
#define KP_PITCH          1.5
#define KI_PITCH          0.05
#define KD_PITCH          15.0

// Yaw PID
#define KP_YAW            3.0
#define KI_YAW            0.02
#define KD_YAW            0.0

// Altitude PID
#define KP_ALT            2.0
#define KI_ALT            0.1
#define KD_ALT            1.5

// ============================================================================
// DATA STRUCTURES
// ============================================================================
// Data received from Remote Controller
struct RCData {
  uint16_t throttle;    // 1000-2000
  int16_t yaw;          // -500 to +500
  int16_t pitch;        // -500 to +500
  int16_t roll;         // -500 to +500
  bool armSwitch;       // SW_2: Arm/Disarm
  bool altHoldSwitch;   // SW_1: Altitude hold
  bool calibButton;     // BTN_1: Calibration
  bool escButton;       // BTN_2: ESC calibration
  uint8_t checksum;
};

// Telemetry data sent to RC
struct TelemetryData {
  float batteryVoltage;
  float altitude;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  bool armed;
  bool calibrated;
  uint8_t errorCode;
  uint8_t checksum;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
// NRF24L01
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte rxAddress[6] = "DRONE";
const byte txAddress[6] = "REMOT";

// Data packets
RCData rcData;
TelemetryData telemetry;

// Motor objects
Servo motorFL, motorFR, motorRR, motorRL;

// IMU variables
int16_t gyroRaw[3], accelRaw[3];
float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;
float gyroXCal = 0, gyroYCal = 0, gyroZCal = 0;
float angleRoll, anglePitch, angleYaw = 0;
float gyroRollInput, gyroPitchInput, gyroYawInput;

// Barometer variables
uint16_t ms5611_c[8];     // Calibration coefficients
float altitude = 0;
float altitudeStart = 0;
float altitudeSetpoint = 0;
bool altHoldActive = false;

// PID variables
float rollSetpoint = 0, pitchSetpoint = 0, yawSetpoint = 0;
float rollError = 0, pitchError = 0, yawError = 0;
float rollErrorSum = 0, pitchErrorSum = 0, yawErrorSum = 0;
float rollErrorLast = 0, pitchErrorLast = 0, yawErrorLast = 0;
float rollPID = 0, pitchPID = 0, yawPID = 0;
float altError = 0, altErrorSum = 0, altErrorLast = 0, altPID = 0;

// Motor outputs
int motorFLSpeed, motorFRSpeed, motorRRSpeed, motorRLSpeed;

// System state
bool armed = false;
bool calibrated = false;
bool escCalibrationMode = false;
bool connectionActive = false;
unsigned long lastRxTime = 0;
unsigned long loopTimer = 0;
bool ledState = false;

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MPU_INT_PIN, INPUT);
  
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.println(F("=== DRONE FLIGHT CONTROLLER ==="));
  Serial.println(F("Initializing systems..."));
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // 400kHz Fast Mode
  
  // Initialize NRF24L01
  if (!initNRF()) {
    Serial.println(F("ERROR: NRF24L01 init failed!"));
    errorBeep(5);
    while(1);
  }
  Serial.println(F("✓ NRF24L01 initialized"));
  
  // Initialize MPU6050
  if (!initMPU6050()) {
    Serial.println(F("ERROR: MPU6050 init failed!"));
    errorBeep(5);
    while(1);
  }
  Serial.println(F("✓ MPU6050 initialized"));
  
  // Initialize MS5611
  if (!initMS5611()) {
    Serial.println(F("ERROR: MS5611 init failed!"));
    errorBeep(3);
    // Continue without barometer
  } else {
    Serial.println(F("✓ MS5611 initialized"));
  }
  
  // Initialize motors
  initMotors();
  Serial.println(F("✓ Motors initialized"));
  
  // Startup sound
  beep(100);
  delay(100);
  beep(100);
  
  Serial.println(F("System ready. Waiting for RC connection..."));
  loopTimer = micros();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Read RC data
  if (radio.available()) {
    radio.read(&rcData, sizeof(rcData));
    
    if (validateChecksum((uint8_t*)&rcData, sizeof(rcData))) {
      lastRxTime = millis();
      
      // First connection
      if (!connectionActive) {
        connectionActive = true;
        Serial.println(F("✓ RC Connected!"));
        beep(50);
        delay(50);
        beep(50);
      }
      
      // Blink LED to show connection
      if (millis() % 1000 < 500) {
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW);
      }
      
      // Handle calibration request
      if (rcData.calibButton && !calibrated) {
        performCalibration();
      }
      
      // Handle ESC calibration
      if (rcData.escButton && !rcData.altHoldSwitch) {
        performESCCalibration();
      }
      
      // Handle arming
      handleArming();
      
      // Handle altitude hold
      handleAltitudeHold();
    }
  }
  
  // Check for signal loss
  if (millis() - lastRxTime > 1000 && connectionActive) {
    connectionActive = false;
    armed = false;
    digitalWrite(LED_PIN, LOW);
    stopMotors();
    Serial.println(F("⚠ Signal lost - DISARMED"));
  }
  
  // Read sensors
  readMPU6050();
  calculateAngles();
  readMS5611();
  
  // Flight control (only if armed)
  if (armed && connectionActive) {
    calculatePID();
    mixMotors();
    writeMotors();
  } else {
    stopMotors();
  }
  
  // Send telemetry
  sendTelemetry();
  
  // Maintain loop timing
  while (micros() - loopTimer < LOOP_TIME);
  loopTimer = micros();
}

// ============================================================================
// NRF24L01 FUNCTIONS
// ============================================================================
bool initNRF() {
  if (!radio.begin()) {
    return false;
  }
  
  radio.setChannel(NRF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setRetries(5, 15);
  
  radio.openWritingPipe(txAddress);
  radio.openReadingPipe(1, rxAddress);
  radio.startListening();
  
  return true;
}

uint8_t calculateChecksum(uint8_t* data, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len - 1; i++) {
    sum += data[i];
  }
  return sum;
}

bool validateChecksum(uint8_t* data, uint8_t len) {
  return (calculateChecksum(data, len) == data[len - 1]);
}

void sendTelemetry() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 50) {  // 20Hz
    telemetry.batteryVoltage = readBatteryVoltage();
    telemetry.altitude = altitude;
    telemetry.roll = (int16_t)angleRoll;
    telemetry.pitch = (int16_t)anglePitch;
    telemetry.yaw = (int16_t)angleYaw;
    telemetry.armed = armed;
    telemetry.calibrated = calibrated;
    telemetry.errorCode = 0;
    telemetry.checksum = calculateChecksum((uint8_t*)&telemetry, sizeof(telemetry));
    
    radio.stopListening();
    radio.write(&telemetry, sizeof(telemetry));
    radio.startListening();
    
    lastSend = millis();
  }
}

// ============================================================================
// MPU6050 FUNCTIONS
// ============================================================================
bool initMPU6050() {
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_PWR_MGMT);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;
  
  delay(100);
  
  // Configure gyro (±500°/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  
  // Configure accel (±8g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  // Set digital low-pass filter (98Hz)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x02);
  Wire.endTransmission();
  
  return true;
}

void readMPU6050() {
  // Read accel data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  
  accelRaw[0] = Wire.read() << 8 | Wire.read();
  accelRaw[1] = Wire.read() << 8 | Wire.read();
  accelRaw[2] = Wire.read() << 8 | Wire.read();
  
  // Read gyro data
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_GYRO);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  
  gyroRaw[0] = Wire.read() << 8 | Wire.read();
  gyroRaw[1] = Wire.read() << 8 | Wire.read();
  gyroRaw[2] = Wire.read() << 8 | Wire.read();
  
  // Convert to physical units
  gyroX = (gyroRaw[0] / 65.5) - gyroXCal;  // °/s (±500°/s range)
  gyroY = (gyroRaw[1] / 65.5) - gyroYCal;
  gyroZ = (gyroRaw[2] / 65.5) - gyroZCal;
  
  accelX = accelRaw[0] / 4096.0;  // g (±8g range)
  accelY = accelRaw[1] / 4096.0;
  accelZ = accelRaw[2] / 4096.0;
}

void calculateAngles() {
  // Calculate angles from accelerometer
  float accelAngleRoll = atan2(accelY, accelZ) * 57.2958;  // Convert to degrees
  float accelAnglePitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958;
  
  // Complementary filter (gyro 98%, accel 2%)
  angleRoll = 0.98 * (angleRoll + gyroX * (LOOP_TIME / 1000000.0)) + 0.02 * accelAngleRoll;
  anglePitch = 0.98 * (anglePitch + gyroY * (LOOP_TIME / 1000000.0)) + 0.02 * accelAnglePitch;
  
  // Yaw from gyro only
  angleYaw += gyroZ * (LOOP_TIME / 1000000.0);
  
  // Store gyro inputs for PID
  gyroRollInput = gyroX;
  gyroPitchInput = gyroY;
  gyroYawInput = gyroZ;
}

// ============================================================================
// MS5611 FUNCTIONS
// ============================================================================
bool initMS5611() {
  // Reset MS5611
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_CMD_RESET);
  if (Wire.endTransmission() != 0) return false;
  delay(10);
  
  // Read calibration coefficients
  for (uint8_t i = 0; i < 8; i++) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(0xA0 + (i * 2));
    Wire.endTransmission();
    Wire.requestFrom(MS5611_ADDR, 2);
    if (Wire.available() == 2) {
      ms5611_c[i] = Wire.read() << 8 | Wire.read();
    }
  }
  
  return true;
}

void readMS5611() {
  static unsigned long lastRead = 0;
  static uint8_t state = 0;
  static uint32_t d1 = 0, d2 = 0;
  
  if (millis() - lastRead > 20) {  // 50Hz
    if (state == 0) {
      // Start pressure conversion
      Wire.beginTransmission(MS5611_ADDR);
      Wire.write(MS5611_CMD_D1);
      Wire.endTransmission();
      state = 1;
    } else if (state == 1) {
      // Read pressure
      Wire.beginTransmission(MS5611_ADDR);
      Wire.write(MS5611_CMD_ADC);
      Wire.endTransmission();
      Wire.requestFrom(MS5611_ADDR, 3);
      if (Wire.available() == 3) {
        d1 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
      }
      
      // Start temperature conversion
      Wire.beginTransmission(MS5611_ADDR);
      Wire.write(MS5611_CMD_D2);
      Wire.endTransmission();
      state = 2;
    } else {
      // Read temperature
      Wire.beginTransmission(MS5611_ADDR);
      Wire.write(MS5611_CMD_ADC);
      Wire.endTransmission();
      Wire.requestFrom(MS5611_ADDR, 3);
      if (Wire.available() == 3) {
        d2 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
      }
      
      // Calculate temperature and pressure
      int32_t dT = d2 - ((uint32_t)ms5611_c[5] << 8);
      int32_t temp = 2000 + (((int64_t)dT * ms5611_c[6]) >> 23);
      
      int64_t off = ((int64_t)ms5611_c[2] << 16) + (((int64_t)ms5611_c[4] * dT) >> 7);
      int64_t sens = ((int64_t)ms5611_c[1] << 15) + (((int64_t)ms5611_c[3] * dT) >> 8);
      
      int32_t pressure = (((d1 * sens) >> 21) - off) >> 15;
      
      // Calculate altitude (simplified formula)
      altitude = 44330.0 * (1.0 - pow(pressure / 101325.0, 0.1903));
      
      state = 0;
    }
    lastRead = millis();
  }
}

// ============================================================================
// CALIBRATION FUNCTIONS
// ============================================================================
void performCalibration() {
  Serial.println(F("\n=== GYRO CALIBRATION ==="));
  Serial.println(F("Keep drone on level surface..."));
  
  digitalWrite(LED_PIN, HIGH);
  beep(200);
  delay(1000);
  
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 2000;
  
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_GYRO);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    
    gyroRaw[0] = Wire.read() << 8 | Wire.read();
    gyroRaw[1] = Wire.read() << 8 | Wire.read();
    gyroRaw[2] = Wire.read() << 8 | Wire.read();
    
    sumX += gyroRaw[0] / 65.5;
    sumY += gyroRaw[1] / 65.5;
    sumZ += gyroRaw[2] / 65.5;
    
    delay(3);
    
    if (i % 200 == 0) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }
  
  gyroXCal = sumX / samples;
  gyroYCal = sumY / samples;
  gyroZCal = sumZ / samples;
  
  // Read starting altitude
  readMS5611();
  delay(50);
  readMS5611();
  altitudeStart = altitude;
  
  // Check calibration quality
  if (abs(gyroXCal) < 10 && abs(gyroYCal) < 10 && abs(gyroZCal) < 10) {
    calibrated = true;
    Serial.println(F("✓ Calibration SUCCESS!"));
    Serial.print(F("Gyro offsets: X="));
    Serial.print(gyroXCal);
    Serial.print(F(" Y="));
    Serial.print(gyroYCal);
    Serial.print(F(" Z="));
    Serial.println(gyroZCal);
    
    // Success beep (2 short beeps)
    digitalWrite(LED_PIN, HIGH);
    beep(100);
    delay(150);
    beep(100);
    delay(150);
    digitalWrite(LED_PIN, LOW);
  } else {
    calibrated = false;
    Serial.println(F("✗ Calibration FAILED!"));
    Serial.println(F("Gyro drift too high - check sensor"));
    
    // Failure beep (long continuous beep)
    digitalWrite(LED_PIN, HIGH);
    beep(7000);
    digitalWrite(LED_PIN, LOW);
  }
}

void performESCCalibration() {
  Serial.println(F("\n=== ESC CALIBRATION ==="));
  escCalibrationMode = true;
  
  beep(150);
  delay(100);
  beep(150);
  delay(100);
  beep(150);
  
  // Spin each motor one by one
  int testSpeed = 1100;
  
  Serial.println(F("Motor FL..."));
  for (int i = MOTOR_MIN; i <= testSpeed; i += 5) {
    motorFL.writeMicroseconds(i);
    delay(50);
  }
  beep(50);
  delay(500);
  motorFL.writeMicroseconds(MOTOR_MIN);
  delay(1000);
  
  Serial.println(F("Motor FR..."));
  for (int i = MOTOR_MIN; i <= testSpeed; i += 5) {
    motorFR.writeMicroseconds(i);
    delay(50);
  }
  beep(50);
  delay(500);
  motorFR.writeMicroseconds(MOTOR_MIN);
  delay(1000);
  
  Serial.println(F("Motor RR..."));
  for (int i = MOTOR_MIN; i <= testSpeed; i += 5) {
    motorRR.writeMicroseconds(i);
    delay(50);
  }
  beep(50);
  delay(500);
  motorRR.writeMicroseconds(MOTOR_MIN);
  delay(1000);
  
  Serial.println(F("Motor RL..."));
  for (int i = MOTOR_MIN; i <= testSpeed; i += 5) {
    motorRL.writeMicroseconds(i);
    delay(50);
  }
  beep(50);
  delay(500);
  motorRL.writeMicroseconds(MOTOR_MIN);
  
  // Completion beep pattern
  beep(100);
  delay(100);
  beep(100);
  delay(100);
  beep(300);
  
  Serial.println(F("✓ ESC Calibration complete!"));
  escCalibrationMode = false;
}

// ============================================================================
// FLIGHT CONTROL FUNCTIONS
// ============================================================================
void handleArming() {
  static bool lastArmSwitch = false;
  
  // Arm only if: switch ON, calibrated, connected, and low throttle
  if (rcData.armSwitch && !lastArmSwitch && calibrated && connectionActive) {
    if (rcData.throttle < 1100) {  // Throttle must be low
      armed = true;
      angleRoll = 0;  // Reset angles
      anglePitch = 0;
      angleYaw = 0;
      rollErrorSum = 0;  // Reset PID integrals
      pitchErrorSum = 0;
      yawErrorSum = 0;
      beep(50);
      delay(50);
      beep(50);
      Serial.println(F("✓ ARMED"));
    } else {
      beep(1000);  // Warning beep
      Serial.println(F("⚠ Cannot arm - Lower throttle!"));
    }
  }
  
  // Disarm immediately when switch OFF
  if (!rcData.armSwitch && armed) {
    armed = false;
    beep(200);
    Serial.println(F("✓ DISARMED"));
  }
  
  lastArmSwitch = rcData.armSwitch;
}

void handleAltitudeHold() {
  static bool lastAltHoldSwitch = false;
  
  if (rcData.altHoldSwitch && !lastAltHoldSwitch && armed) {
    altHoldActive = true;
    altitudeSetpoint = altitude;
    altErrorSum = 0;
    Serial.print(F("✓ Alt Hold ON @ "));
    Serial.print(altitude - altitudeStart);
    Serial.println(F("m"));
  }
  
  if (!rcData.altHoldSwitch && altHoldActive) {
    altHoldActive = false;
    Serial.println(F("✓ Alt Hold OFF"));
  }
  
  lastAltHoldSwitch = rcData.altHoldSwitch;
}

void calculatePID() {
  // Convert RC inputs to angle setpoints
  rollSetpoint = (rcData.roll / 500.0) * MAX_ANGLE;   // ±30°
  pitchSetpoint = (rcData.pitch / 500.0) * MAX_ANGLE; // ±30°
  yawSetpoint = rcData.yaw / 3.0;  // Rate mode for yaw
  
  // Constrain setpoints
  rollSetpoint = constrain(rollSetpoint, -MAX_ANGLE, MAX_ANGLE);
  pitchSetpoint = constrain(pitchSetpoint, -MAX_ANGLE, MAX_ANGLE);
  
  // Calculate errors
  rollError = rollSetpoint - angleRoll;
  pitchError = pitchSetpoint - anglePitch;
  yawError = yawSetpoint - gyroYawInput;
  
  // Calculate integral (with anti-windup)
  rollErrorSum += rollError * (LOOP_TIME / 1000000.0);
  pitchErrorSum += pitchError * (LOOP_TIME / 1000000.0);
  yawErrorSum += yawError * (LOOP_TIME / 1000000.0);
  
  rollErrorSum = constrain(rollErrorSum, -400, 400);
  pitchErrorSum = constrain(pitchErrorSum, -400, 400);
  yawErrorSum = constrain(yawErrorSum, -400, 400);
  
  // Calculate derivative
  float rollErrorDiff = (rollError - rollErrorLast) / (LOOP_TIME / 1000000.0);
  float pitchErrorDiff = (pitchError - pitchErrorLast) / (LOOP_TIME / 1000000.0);
  float yawErrorDiff = (yawError - yawErrorLast) / (LOOP_TIME / 1000000.0);
  
  // PID outputs
  rollPID = (KP_ROLL * rollError) + (KI_ROLL * rollErrorSum) + (KD_ROLL * rollErrorDiff);
  pitchPID = (KP_PITCH * pitchError) + (KI_PITCH * pitchErrorSum) + (KD_PITCH * pitchErrorDiff);
  yawPID = (KP_YAW * yawError) + (KI_YAW * yawErrorSum) + (KD_YAW * yawErrorDiff);
  
  // Altitude hold PID
  if (altHoldActive) {
    altError = altitudeSetpoint - altitude;
    altErrorSum += altError * (LOOP_TIME / 1000000.0);
    altErrorSum = constrain(altErrorSum, -400, 400);
    float altErrorDiff = (altError - altErrorLast) / (LOOP_TIME / 1000000.0);
    altPID = (KP_ALT * altError) + (KI_ALT * altErrorSum) + (KD_ALT * altErrorDiff);
    altErrorLast = altError;
  } else {
    altPID = 0;
    altErrorSum = 0;
  }
  
  // Store last errors
  rollErrorLast = rollError;
  pitchErrorLast = pitchError;
  yawErrorLast = yawError;
}

void mixMotors() {
  // Get base throttle (cap at 65% for safety)
  int baseThrottle = constrain(rcData.throttle, MOTOR_MIN, THROTTLE_SAFE_MAX);
  
  // Add altitude hold correction
  if (altHoldActive) {
    baseThrottle += (int)altPID;
    baseThrottle = constrain(baseThrottle, MOTOR_ARM_LEVEL, THROTTLE_SAFE_MAX);
  }
  
  // Mix PID outputs with throttle
  // X-configuration motor mixing
  motorFLSpeed = baseThrottle - pitchPID - rollPID + yawPID;
  motorFRSpeed = baseThrottle - pitchPID + rollPID - yawPID;
  motorRRSpeed = baseThrottle + pitchPID + rollPID + yawPID;
  motorRLSpeed = baseThrottle + pitchPID - rollPID - yawPID;
  
  // Constrain motor speeds
  motorFLSpeed = constrain(motorFLSpeed, MOTOR_MIN, MOTOR_MAX);
  motorFRSpeed = constrain(motorFRSpeed, MOTOR_MIN, MOTOR_MAX);
  motorRRSpeed = constrain(motorRRSpeed, MOTOR_MIN, MOTOR_MAX);
  motorRLSpeed = constrain(motorRLSpeed, MOTOR_MIN, MOTOR_MAX);
}

void writeMotors() {
  motorFL.writeMicroseconds(motorFLSpeed);
  motorFR.writeMicroseconds(motorFRSpeed);
  motorRR.writeMicroseconds(motorRRSpeed);
  motorRL.writeMicroseconds(motorRLSpeed);
}

void stopMotors() {
  motorFL.writeMicroseconds(MOTOR_MIN);
  motorFR.writeMicroseconds(MOTOR_MIN);
  motorRR.writeMicroseconds(MOTOR_MIN);
  motorRL.writeMicroseconds(MOTOR_MIN);
  
  // Reset PID integrals
  rollErrorSum = 0;
  pitchErrorSum = 0;
  yawErrorSum = 0;
  altErrorSum = 0;
}

// ============================================================================
// MOTOR INITIALIZATION
// ============================================================================
void initMotors() {
  motorFL.attach(MOTOR_FL_PIN);
  motorFR.attach(MOTOR_FR_PIN);
  motorRR.attach(MOTOR_RR_PIN);
  motorRL.attach(MOTOR_RL_PIN);
  
  // Initialize all motors to minimum
  stopMotors();
  delay(100);
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================
void beep(int duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

void errorBeep(int count) {
  for (int i = 0; i < count; i++) {
    beep(100);
    delay(100);
  }
}

float readBatteryVoltage() {
  // Simplified - would normally use voltage divider on analog pin
  return 11.1;  // Placeholder for 3S LiPo
}
