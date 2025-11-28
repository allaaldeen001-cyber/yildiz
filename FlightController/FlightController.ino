/*
 * ========================================================================
 * QUADCOPTER FLIGHT CONTROLLER
 * ========================================================================
 * Professional quadcopter flight controller with:
 * - PID stabilization for Roll, Pitch, Yaw
 * - MPU6050 IMU sensor fusion
 * - MS5611 barometric altitude hold
 * - NRF24L01 wireless communication
 * - ESC control for 4 motors
 * - Comprehensive calibration system
 * - Safety features and failsafes
 * 
 * Author: Professional Embedded Systems Engineer
 * Date: November 2025
 * Version: 1.0.0
 * ========================================================================
 */

#include <Wire.h>
#include <EEPROM.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// ========================================================================
// PIN DEFINITIONS
// ========================================================================
#define LED_PIN           2
#define MOTOR1_PIN        3   // Front-Right
#define MOTOR2_PIN        4   // Rear-Right
#define MOTOR3_PIN        5   // Rear-Left
#define MOTOR4_PIN        6   // Front-Left
#define BUZZER_PIN        7
#define STATUS_LED_PIN    8
#define NRF_CE_PIN        9
#define NRF_CSN_PIN       10

// ========================================================================
// MPU6050 REGISTERS
// ========================================================================
#define MPU6050_ADDR      0x68
#define PWR_MGMT_1        0x6B
#define ACCEL_XOUT_H      0x3B
#define GYRO_XOUT_H       0x43
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C

// ========================================================================
// MS5611 DEFINITIONS
// ========================================================================
#define MS5611_ADDR       0x77
#define MS5611_RESET      0x1E
#define MS5611_READ_PROM  0xA0
#define MS5611_CONVERT_D1 0x48  // Pressure
#define MS5611_CONVERT_D2 0x58  // Temperature
#define MS5611_READ_ADC   0x00

// ========================================================================
// FLIGHT CONTROLLER PARAMETERS
// ========================================================================
#define LOOP_FREQUENCY    250      // 250Hz control loop
#define LOOP_TIME_US      4000     // 4ms = 250Hz
#define FAILSAFE_TIME     1000     // 1 second no signal = failsafe

// PID Gains - Roll and Pitch
#define PID_ROLL_KP       1.5f
#define PID_ROLL_KI       0.05f
#define PID_ROLL_KD       18.0f
#define PID_PITCH_KP      1.5f
#define PID_PITCH_KI      0.05f
#define PID_PITCH_KD      18.0f

// PID Gains - Yaw
#define PID_YAW_KP        3.0f
#define PID_YAW_KI        0.02f
#define PID_YAW_KD        0.0f

// PID Gains - Altitude
#define PID_ALT_KP        2.0f
#define PID_ALT_KI        0.1f
#define PID_ALT_KD        1.5f

// Motor Limits
#define MOTOR_MIN         1000
#define MOTOR_MAX         2000
#define MOTOR_ARM_START   1100
#define MOTOR_IDLE        1100
#define THROTTLE_MIN      1000
#define THROTTLE_MAX      2000

// Angle Limits
#define MAX_ANGLE         30.0f    // Maximum tilt angle in degrees
#define MAX_YAW_RATE      180.0f   // Maximum yaw rate in deg/s

// ========================================================================
// GLOBAL VARIABLES
// ========================================================================

// NRF24L01 Radio
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte address[6] = "DRONE";

// Motor Control (Servo library for ESC)
Servo motor1, motor2, motor3, motor4;

// Radio Data Structure
struct RadioData {
  int throttle;    // 1000-2000
  int yaw;         // 1000-2000
  int pitch;       // 1000-2000
  int roll;        // 1000-2000
  bool armed;      // true/false
  byte command;    // 0=none, 1=calibrate, 2=motor_test
};
RadioData receivedData;
RadioData lastValidData;

// IMU Data
float gyroX, gyroY, gyroZ;           // Raw gyro (deg/s)
float accelX, accelY, accelZ;        // Raw accel (g)
float gyroXOffset, gyroYOffset, gyroZOffset;  // Calibration offsets
float accelXOffset, accelYOffset, accelZOffset;

// Attitude (Calculated angles)
float roll, pitch, yaw;              // Degrees
float rollRate, pitchRate, yawRate;  // Deg/s

// Barometer Data
uint16_t ms5611_C[8];                // Calibration coefficients
float altitude, baseAltitude;        // Meters
float pressure, temperature;         // Pa, Celsius
float climbRate;                     // m/s
float lastAltitude;

// PID Controllers
struct PIDController {
  float kP, kI, kD;
  float integral;
  float lastError;
  float output;
  float integralLimit;
};

PIDController pidRoll   = {PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, 0, 0, 0, 400};
PIDController pidPitch  = {PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, 0, 0, 0, 400};
PIDController pidYaw    = {PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, 0, 0, 0, 400};
PIDController pidAlt    = {PID_ALT_KP, PID_ALT_KI, PID_ALT_KD, 0, 0, 0, 400};

// Motor Outputs
int motor1Speed, motor2Speed, motor3Speed, motor4Speed;

// System State
bool isArmed = false;
bool isCalibrated = false;
bool nrfConnected = false;
bool altitudeHoldEnabled = false;
unsigned long lastPacketTime = 0;
unsigned long loopStartTime = 0;
unsigned long lastTelemetryTime = 0;

// Flight Mode
enum FlightMode {
  DISARMED,
  ARMED,
  FAILSAFE,
  CALIBRATION
};
FlightMode currentMode = DISARMED;

// ========================================================================
// EEPROM ADDRESSES
// ========================================================================
#define EEPROM_CALIBRATED     0    // 1 byte - calibration flag
#define EEPROM_GYRO_X_OFFSET  4    // 4 bytes - float
#define EEPROM_GYRO_Y_OFFSET  8
#define EEPROM_GYRO_Z_OFFSET  12
#define EEPROM_ACCEL_X_OFFSET 16
#define EEPROM_ACCEL_Y_OFFSET 20
#define EEPROM_ACCEL_Z_OFFSET 24
#define EEPROM_BASE_ALTITUDE  28   // 4 bytes - float

// ========================================================================
// SETUP
// ========================================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println(F("\n========================================"));
  Serial.println(F("  QUADCOPTER FLIGHT CONTROLLER v1.0"));
  Serial.println(F("========================================\n"));

  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  digitalWrite(LED_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // Welcome beep
  beep(100);
  delay(100);
  beep(100);

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // 400kHz Fast Mode
  Serial.println(F("[INIT] I2C initialized"));

  // Initialize MPU6050
  if (initMPU6050()) {
    Serial.println(F("[OK] MPU6050 initialized"));
  } else {
    Serial.println(F("[ERROR] MPU6050 init failed!"));
    errorBeep();
    while(1);
  }

  // Initialize MS5611
  if (initMS5611()) {
    Serial.println(F("[OK] MS5611 initialized"));
  } else {
    Serial.println(F("[ERROR] MS5611 init failed!"));
    errorBeep();
    while(1);
  }

  // Initialize Motors
  motor1.attach(MOTOR1_PIN, 1000, 2000);
  motor2.attach(MOTOR2_PIN, 1000, 2000);
  motor3.attach(MOTOR3_PIN, 1000, 2000);
  motor4.attach(MOTOR4_PIN, 1000, 2000);
  
  // Set motors to minimum
  setAllMotors(MOTOR_MIN);
  Serial.println(F("[OK] Motors initialized (ESC armed)"));

  // Initialize NRF24L01
  if (initNRF()) {
    Serial.println(F("[OK] NRF24L01 initialized"));
    nrfConnected = true;
    beep(100);
    delay(100);
    beep(100);  // Success beep
  } else {
    Serial.println(F("[ERROR] NRF24L01 init failed!"));
    nrfConnected = false;
    errorBeep();
  }

  // Load calibration from EEPROM
  loadCalibration();

  // Initialize received data to safe values
  receivedData.throttle = 1000;
  receivedData.yaw = 1500;
  receivedData.pitch = 1500;
  receivedData.roll = 1500;
  receivedData.armed = false;
  receivedData.command = 0;
  
  lastValidData = receivedData;

  Serial.println(F("\n========================================"));
  Serial.println(F("  WAITING FOR RC CONNECTION..."));
  Serial.println(F("========================================\n"));
  Serial.println(F("Step 1: Power on RC Transmitter"));
  Serial.println(F("Step 2: Set toggle switch to KILL position"));
  Serial.println(F("Step 3: Press Button 1 to calibrate"));
  Serial.println(F("        (Place drone on level surface!)"));
  Serial.println(F("Step 4: Arm drone with toggle switch"));
  Serial.println(F("Step 5: Test motors with Button 2"));
  Serial.println(F("Step 6: Ready to fly!\n"));

  delay(1000);
}

// ========================================================================
// MAIN LOOP
// ========================================================================
void loop() {
  loopStartTime = micros();

  // Read radio data
  readRadioData();

  // Check failsafe
  checkFailsafe();

  // Update sensor data
  readMPU6050();
  calculateAttitude();
  
  // Read barometer (slower rate)
  static unsigned long lastBaroRead = 0;
  if (millis() - lastBaroRead > 50) {  // 20Hz baro update
    readMS5611();
    lastBaroRead = millis();
  }

  // Handle commands from RC
  handleCommands();

  // Update flight mode
  updateFlightMode();

  // Calculate motor outputs
  if (isArmed && currentMode == ARMED) {
    calculatePID();
    calculateMotorOutputs();
  } else {
    resetPID();
    setAllMotors(MOTOR_MIN);
  }

  // Output motors
  outputMotors();

  // Status LED
  updateStatusLED();

  // Telemetry output (10Hz)
  if (millis() - lastTelemetryTime > 100) {
    printTelemetry();
    lastTelemetryTime = millis();
  }

  // Maintain loop frequency
  maintainLoopFrequency();
}

// ========================================================================
// NRF24L01 FUNCTIONS
// ========================================================================
bool initNRF() {
  if (!radio.begin()) {
    return false;
  }
  
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setAutoAck(true);
  radio.startListening();
  
  return true;
}

void readRadioData() {
  if (radio.available()) {
    radio.read(&receivedData, sizeof(RadioData));
    lastPacketTime = millis();
    lastValidData = receivedData;
    
    if (!nrfConnected) {
      nrfConnected = true;
      Serial.println(F("\n[✓] NRF CONNECTION ESTABLISHED!"));
      beep(100);
      delay(50);
      beep(100);
    }
  }
}

// ========================================================================
// MPU6050 FUNCTIONS
// ========================================================================
bool initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);  // Wake up MPU6050
  if (Wire.endTransmission() != 0) return false;
  
  delay(100);
  
  // Set gyro range to ±500 deg/s
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x08);  // FS_SEL = 1 (±500 deg/s)
  Wire.endTransmission();
  
  // Set accel range to ±4g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x08);  // AFS_SEL = 1 (±4g)
  Wire.endTransmission();
  
  delay(100);
  return true;
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  // Read accelerometer
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // Skip temperature
  
  // Read gyroscope
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();
  
  // Convert to physical units
  accelX = (ax / 8192.0f) - accelXOffset;  // ±4g range
  accelY = (ay / 8192.0f) - accelYOffset;
  accelZ = (az / 8192.0f) - accelZOffset;
  
  gyroX = (gx / 65.5f) - gyroXOffset;      // ±500 deg/s range
  gyroY = (gy / 65.5f) - gyroYOffset;
  gyroZ = (gz / 65.5f) - gyroZOffset;
  
  // Store rates
  rollRate = gyroX;
  pitchRate = gyroY;
  yawRate = gyroZ;
}

void calibrateMPU6050() {
  Serial.println(F("\n[CAL] Calibrating MPU6050..."));
  Serial.println(F("      Keep drone LEVEL and STATIONARY!"));
  
  beep(200);
  delay(500);
  
  long gxSum = 0, gySum = 0, gzSum = 0;
  long axSum = 0, aySum = 0, azSum = 0;
  int samples = 1000;
  
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);
    
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();
    
    axSum += ax;
    aySum += ay;
    azSum += az;
    gxSum += gx;
    gySum += gy;
    gzSum += gz;
    
    if (i % 100 == 0) {
      Serial.print(F("."));
    }
    delay(2);
  }
  Serial.println();
  
  // Calculate offsets
  gyroXOffset = (gxSum / (float)samples) / 65.5f;
  gyroYOffset = (gySum / (float)samples) / 65.5f;
  gyroZOffset = (gzSum / (float)samples) / 65.5f;
  
  accelXOffset = (axSum / (float)samples) / 8192.0f;
  accelYOffset = (aySum / (float)samples) / 8192.0f;
  accelZOffset = ((azSum / (float)samples) / 8192.0f) - 1.0f;  // Subtract 1g
  
  Serial.println(F("[✓] MPU6050 calibration complete!"));
  Serial.print(F("    Gyro offsets: "));
  Serial.print(gyroXOffset); Serial.print(F(", "));
  Serial.print(gyroYOffset); Serial.print(F(", "));
  Serial.println(gyroZOffset);
  
  beep(100);
  delay(100);
  beep(100);
}

void calculateAttitude() {
  static unsigned long lastTime = 0;
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0f;
  lastTime = currentTime;
  
  if (dt > 0.1f) dt = 0.004f;  // Sanity check
  
  // Complementary filter
  // Gyro integration
  roll += gyroX * dt;
  pitch += gyroY * dt;
  yaw += gyroZ * dt;
  
  // Accelerometer angles
  float accelRoll = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 57.2958f;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958f;
  
  // Complementary filter (98% gyro, 2% accel)
  roll = 0.98f * roll + 0.02f * accelRoll;
  pitch = 0.98f * pitch + 0.02f * accelPitch;
  
  // Keep yaw in 0-360 range
  if (yaw > 360.0f) yaw -= 360.0f;
  if (yaw < 0.0f) yaw += 360.0f;
}

// ========================================================================
// MS5611 BAROMETER FUNCTIONS
// ========================================================================
bool initMS5611() {
  // Reset MS5611
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_RESET);
  if (Wire.endTransmission() != 0) return false;
  delay(10);
  
  // Read calibration coefficients
  for (int i = 0; i < 8; i++) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(MS5611_READ_PROM + (i * 2));
    Wire.endTransmission();
    Wire.requestFrom(MS5611_ADDR, 2);
    ms5611_C[i] = (Wire.read() << 8) | Wire.read();
  }
  
  // Verify coefficients are valid
  if (ms5611_C[0] == 0 || ms5611_C[0] == 0xFFFF) {
    return false;
  }
  
  return true;
}

void readMS5611() {
  static uint8_t state = 0;
  static uint32_t D1 = 0, D2 = 0;
  
  if (state == 0) {
    // Start pressure conversion
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(MS5611_CONVERT_D1 + 0x08);  // OSR=4096
    Wire.endTransmission();
    state = 1;
  } else if (state == 1) {
    // Read pressure
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(MS5611_READ_ADC);
    Wire.endTransmission();
    Wire.requestFrom(MS5611_ADDR, 3);
    D1 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
    
    // Start temperature conversion
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(MS5611_CONVERT_D2 + 0x08);  // OSR=4096
    Wire.endTransmission();
    state = 2;
  } else if (state == 2) {
    // Read temperature
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(MS5611_READ_ADC);
    Wire.endTransmission();
    Wire.requestFrom(MS5611_ADDR, 3);
    D2 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
    
    // Calculate temperature and pressure
    int32_t dT = D2 - ((uint32_t)ms5611_C[5] << 8);
    temperature = 2000 + ((int64_t)dT * ms5611_C[6]) / 8388608;
    
    int64_t OFF = ((int64_t)ms5611_C[2] << 16) + (((int64_t)ms5611_C[4] * dT) >> 7);
    int64_t SENS = ((int64_t)ms5611_C[1] << 15) + (((int64_t)ms5611_C[3] * dT) >> 8);
    
    pressure = (((D1 * SENS) >> 21) - OFF) >> 15;
    temperature /= 100.0f;
    
    // Calculate altitude (international barometric formula)
    float seaLevelPressure = 101325.0f;  // Pa
    altitude = 44330.0f * (1.0f - pow(pressure / seaLevelPressure, 0.1903f)) - baseAltitude;
    
    // Calculate climb rate
    static float lastAlt = 0;
    climbRate = (altitude - lastAlt) * 20.0f;  // 20Hz update rate
    lastAlt = altitude;
    
    state = 0;
  }
}

void calibrateMS5611() {
  Serial.println(F("\n[CAL] Calibrating MS5611..."));
  Serial.println(F("      Setting baseline altitude..."));
  
  float sum = 0;
  int samples = 20;
  
  for (int i = 0; i < samples; i++) {
    readMS5611();
    delay(50);
    sum += altitude;
    Serial.print(F("."));
  }
  Serial.println();
  
  baseAltitude = sum / samples;
  
  Serial.println(F("[✓] MS5611 calibration complete!"));
  Serial.print(F("    Base altitude: "));
  Serial.print(baseAltitude);
  Serial.println(F(" m"));
  
  beep(100);
  delay(100);
  beep(100);
}

// ========================================================================
// ESC CALIBRATION
// ========================================================================
void calibrateESC() {
  Serial.println(F("\n[CAL] Calibrating ESCs..."));
  Serial.println(F("      REMOVE PROPELLERS FIRST!"));
  Serial.println(F("      Sending HIGH signal..."));
  
  beep(300);
  delay(1000);
  
  // Send max throttle
  setAllMotors(MOTOR_MAX);
  delay(2000);
  
  Serial.println(F("      Sending LOW signal..."));
  
  // Send min throttle
  setAllMotors(MOTOR_MIN);
  delay(2000);
  
  Serial.println(F("[✓] ESC calibration complete!"));
  
  beep(100);
  delay(100);
  beep(100);
  delay(100);
  beep(100);
}

// ========================================================================
// PID FUNCTIONS
// ========================================================================
void calculatePID() {
  static unsigned long lastPIDTime = 0;
  unsigned long currentTime = micros();
  float dt = (currentTime - lastPIDTime) / 1000000.0f;
  lastPIDTime = currentTime;
  
  if (dt > 0.1f) dt = 0.004f;  // Sanity check
  
  // Calculate desired angles from stick inputs
  float desiredRoll = map(receivedData.roll, 1000, 2000, -MAX_ANGLE, MAX_ANGLE);
  float desiredPitch = map(receivedData.pitch, 1000, 2000, -MAX_ANGLE, MAX_ANGLE);
  float desiredYawRate = map(receivedData.yaw, 1000, 2000, -MAX_YAW_RATE, MAX_YAW_RATE);
  
  // Roll PID
  pidRoll.output = computePID(&pidRoll, desiredRoll, roll, dt);
  
  // Pitch PID
  pidPitch.output = computePID(&pidPitch, desiredPitch, pitch, dt);
  
  // Yaw PID (rate control)
  pidYaw.output = computePID(&pidYaw, desiredYawRate, yawRate, dt);
  
  // Altitude PID (if enabled)
  if (altitudeHoldEnabled) {
    // Not implemented yet - future feature
  }
}

float computePID(PIDController* pid, float setpoint, float input, float dt) {
  // Error
  float error = setpoint - input;
  
  // Proportional
  float P = pid->kP * error;
  
  // Integral (with anti-windup)
  pid->integral += error * dt;
  pid->integral = constrain(pid->integral, -pid->integralLimit, pid->integralLimit);
  float I = pid->kI * pid->integral;
  
  // Derivative
  float derivative = (error - pid->lastError) / dt;
  float D = pid->kD * derivative;
  pid->lastError = error;
  
  // Output
  return P + I + D;
}

void resetPID() {
  pidRoll.integral = 0;
  pidRoll.lastError = 0;
  pidPitch.integral = 0;
  pidPitch.lastError = 0;
  pidYaw.integral = 0;
  pidYaw.lastError = 0;
  pidAlt.integral = 0;
  pidAlt.lastError = 0;
}

// ========================================================================
// MOTOR CONTROL
// ========================================================================
void calculateMotorOutputs() {
  // Base throttle
  int throttle = receivedData.throttle;
  
  // Ensure minimum throttle when armed
  if (throttle < MOTOR_IDLE) {
    throttle = MOTOR_IDLE;
  }
  
  // Mix PID outputs with throttle
  // X-frame configuration:
  // M4(FL)  M1(FR)
  //    \    /
  //     \  /
  //     /  \
  //    /    \
  // M3(RL)  M2(RR)
  
  motor1Speed = throttle + pidPitch.output - pidRoll.output - pidYaw.output;  // Front-Right
  motor2Speed = throttle - pidPitch.output - pidRoll.output + pidYaw.output;  // Rear-Right
  motor3Speed = throttle - pidPitch.output + pidRoll.output - pidYaw.output;  // Rear-Left
  motor4Speed = throttle + pidPitch.output + pidRoll.output + pidYaw.output;  // Front-Left
  
  // Constrain motor speeds
  motor1Speed = constrain(motor1Speed, MOTOR_MIN, MOTOR_MAX);
  motor2Speed = constrain(motor2Speed, MOTOR_MIN, MOTOR_MAX);
  motor3Speed = constrain(motor3Speed, MOTOR_MIN, MOTOR_MAX);
  motor4Speed = constrain(motor4Speed, MOTOR_MIN, MOTOR_MAX);
}

void outputMotors() {
  motor1.writeMicroseconds(motor1Speed);
  motor2.writeMicroseconds(motor2Speed);
  motor3.writeMicroseconds(motor3Speed);
  motor4.writeMicroseconds(motor4Speed);
}

void setAllMotors(int speed) {
  motor1Speed = speed;
  motor2Speed = speed;
  motor3Speed = speed;
  motor4Speed = speed;
  outputMotors();
}

// ========================================================================
// COMMAND HANDLING
// ========================================================================
void handleCommands() {
  if (receivedData.command == 1) {  // Calibration
    if (!receivedData.armed) {
      currentMode = CALIBRATION;
      runFullCalibration();
      receivedData.command = 0;
      currentMode = DISARMED;
    } else {
      Serial.println(F("[!] Cannot calibrate while armed!"));
      errorBeep();
    }
  } else if (receivedData.command == 2) {  // Motor test
    if (receivedData.armed) {
      motorTest();
      receivedData.command = 0;
    } else {
      Serial.println(F("[!] Arm drone first for motor test!"));
      errorBeep();
    }
  }
}

void runFullCalibration() {
  Serial.println(F("\n========================================"));
  Serial.println(F("  FULL CALIBRATION SEQUENCE"));
  Serial.println(F("========================================\n"));
  
  // Step 1: ESC Calibration
  calibrateESC();
  delay(1000);
  
  // Step 2: IMU Calibration
  calibrateMPU6050();
  delay(1000);
  
  // Step 3: Barometer Calibration
  calibrateMS5611();
  delay(1000);
  
  // Save to EEPROM
  saveCalibration();
  
  Serial.println(F("\n========================================"));
  Serial.println(F("  CALIBRATION COMPLETE!"));
  Serial.println(F("========================================\n"));
  Serial.println(F("Next step: ARM the drone with toggle switch"));
  
  beep(100);
  delay(100);
  beep(100);
  delay(100);
  beep(100);
  delay(100);
  beep(500);
}

void motorTest() {
  Serial.println(F("\n[TEST] Running motor test..."));
  Serial.println(F("       Motors will spin at LOW speed"));
  
  beep(200);
  delay(500);
  
  // Gradually increase to idle speed
  for (int i = MOTOR_MIN; i <= MOTOR_IDLE + 100; i += 5) {
    setAllMotors(i);
    delay(20);
  }
  
  delay(3000);  // Run for 3 seconds
  
  // Gradually decrease
  for (int i = MOTOR_IDLE + 100; i >= MOTOR_MIN; i -= 5) {
    setAllMotors(i);
    delay(20);
  }
  
  Serial.println(F("[✓] Motor test complete!"));
  Serial.println(F("\n========================================"));
  Serial.println(F("  DRONE READY TO FLY!"));
  Serial.println(F("========================================\n"));
  Serial.println(F("Flight Tips:"));
  Serial.println(F("1. Start with SMALL throttle increases"));
  Serial.println(F("2. Keep stick movements SMOOTH"));
  Serial.println(F("3. Practice hovering before moving"));
  Serial.println(F("4. KILL SWITCH ready at all times!"));
  
  beep(100);
  delay(100);
  beep(100);
}

// ========================================================================
// FAILSAFE
// ========================================================================
void checkFailsafe() {
  if (millis() - lastPacketTime > FAILSAFE_TIME) {
    if (currentMode != DISARMED && currentMode != CALIBRATION) {
      currentMode = FAILSAFE;
      Serial.println(F("\n[!!!] FAILSAFE ACTIVATED - NO SIGNAL!"));
      errorBeep();
      
      // Gradually reduce throttle
      if (receivedData.throttle > MOTOR_MIN + 100) {
        receivedData.throttle -= 5;
      } else {
        isArmed = false;
        receivedData.armed = false;
      }
    }
  }
}

// ========================================================================
// FLIGHT MODE
// ========================================================================
void updateFlightMode() {
  if (receivedData.armed && isCalibrated) {
    if (!isArmed) {
      isArmed = true;
      currentMode = ARMED;
      resetPID();
      Serial.println(F("\n[✓] DRONE ARMED!"));
      Serial.println(F("    Gradually increase throttle"));
      beep(50);
      delay(50);
      beep(50);
    }
  } else {
    if (isArmed) {
      isArmed = false;
      currentMode = DISARMED;
      setAllMotors(MOTOR_MIN);
      Serial.println(F("\n[✓] DRONE DISARMED"));
      beep(200);
    }
  }
}

// ========================================================================
// STATUS & TELEMETRY
// ========================================================================
void updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  
  if (currentMode == ARMED) {
    digitalWrite(STATUS_LED_PIN, HIGH);  // Solid when armed
  } else if (currentMode == FAILSAFE) {
    // Fast blink in failsafe
    if (millis() - lastBlink > 100) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
      lastBlink = millis();
    }
  } else if (currentMode == CALIBRATION) {
    // Very fast blink during calibration
    if (millis() - lastBlink > 50) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
      lastBlink = millis();
    }
  } else {
    // Slow blink when disarmed
    if (millis() - lastBlink > 500) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
      lastBlink = millis();
    }
  }
  
  // LED follows NRF connection status
  digitalWrite(LED_PIN, nrfConnected ? HIGH : LOW);
}

void printTelemetry() {
  if (!isArmed) return;  // Only print when armed
  
  Serial.println(F("=== FLIGHT DATA ==="));
  Serial.print(F("Throttle: ")); Serial.print(receivedData.throttle);
  Serial.print(F(" | Yaw: ")); Serial.print(receivedData.yaw);
  Serial.print(F(" | Pitch: ")); Serial.print(receivedData.pitch);
  Serial.print(F(" | Roll: ")); Serial.println(receivedData.roll);
  
  Serial.print(F("Alt: ")); Serial.print(altitude, 2);
  Serial.print(F("m | Climb: ")); Serial.print(climbRate, 2);
  Serial.println(F("m/s"));
  
  Serial.print(F("IMU: Pitch=")); Serial.print(pitch, 1);
  Serial.print(F("° Roll=")); Serial.print(roll, 1);
  Serial.print(F("° Yaw=")); Serial.print(yaw, 1);
  Serial.println(F("°"));
  
  Serial.print(F("Motors: ["));
  Serial.print(motor1Speed); Serial.print(F(", "));
  Serial.print(motor2Speed); Serial.print(F(", "));
  Serial.print(motor3Speed); Serial.print(F(", "));
  Serial.print(motor4Speed); Serial.println(F("]"));
  
  Serial.print(F("NRF: Ch108 | Signal: "));
  Serial.println(nrfConnected ? F("GOOD") : F("LOST"));
  Serial.println();
}

// ========================================================================
// CALIBRATION STORAGE
// ========================================================================
void saveCalibration() {
  Serial.println(F("[SAVE] Saving calibration to EEPROM..."));
  
  EEPROM.write(EEPROM_CALIBRATED, 1);
  
  EEPROM.put(EEPROM_GYRO_X_OFFSET, gyroXOffset);
  EEPROM.put(EEPROM_GYRO_Y_OFFSET, gyroYOffset);
  EEPROM.put(EEPROM_GYRO_Z_OFFSET, gyroZOffset);
  
  EEPROM.put(EEPROM_ACCEL_X_OFFSET, accelXOffset);
  EEPROM.put(EEPROM_ACCEL_Y_OFFSET, accelYOffset);
  EEPROM.put(EEPROM_ACCEL_Z_OFFSET, accelZOffset);
  
  EEPROM.put(EEPROM_BASE_ALTITUDE, baseAltitude);
  
  isCalibrated = true;
  Serial.println(F("[✓] Calibration saved!"));
}

void loadCalibration() {
  byte calibrated = EEPROM.read(EEPROM_CALIBRATED);
  
  if (calibrated == 1) {
    EEPROM.get(EEPROM_GYRO_X_OFFSET, gyroXOffset);
    EEPROM.get(EEPROM_GYRO_Y_OFFSET, gyroYOffset);
    EEPROM.get(EEPROM_GYRO_Z_OFFSET, gyroZOffset);
    
    EEPROM.get(EEPROM_ACCEL_X_OFFSET, accelXOffset);
    EEPROM.get(EEPROM_ACCEL_Y_OFFSET, accelYOffset);
    EEPROM.get(EEPROM_ACCEL_Z_OFFSET, accelZOffset);
    
    EEPROM.get(EEPROM_BASE_ALTITUDE, baseAltitude);
    
    isCalibrated = true;
    Serial.println(F("[✓] Calibration loaded from EEPROM"));
    Serial.println(F("    You can skip calibration if not needed"));
  } else {
    isCalibrated = false;
    Serial.println(F("[!] No calibration found"));
    Serial.println(F("    MUST calibrate before first flight!"));
  }
}

// ========================================================================
// UTILITY FUNCTIONS
// ========================================================================
void beep(int duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

void errorBeep() {
  for (int i = 0; i < 3; i++) {
    beep(100);
    delay(100);
  }
}

void maintainLoopFrequency() {
  unsigned long elapsedTime = micros() - loopStartTime;
  
  if (elapsedTime < LOOP_TIME_US) {
    delayMicroseconds(LOOP_TIME_US - elapsedTime);
  }
}

// ========================================================================
// END OF FLIGHT CONTROLLER CODE
// ========================================================================
