/*
 * ========================================
 * QUADCOPTER FLIGHT CONTROLLER
 * ========================================
 * 
 * Hardware: Arduino Nano + MPU6050 + MS5611 + nRF24L01+
 * 
 * Features:
 * - PID-based stabilization
 * - Complementary filter for sensor fusion
 * - Dual flight modes (ANGLE/ACRO)
 * - Altitude hold capability
 * - Wireless control via nRF24L01+
 * - Failsafe protection
 * - Auto-disarm on crash detection
 * 
 * Author: DIY Quadcopter Project
 * Version: 1.0
 */

// ========================================
// LIBRARY INCLUDES
// ========================================
#include <Wire.h>           // I2C communication
#include <Servo.h>          // ESC control (uses PWM like servos)
#include <SPI.h>            // SPI for nRF24
#include <RF24.h>           // nRF24L01+ radio
#include <MPU6050.h>        // MPU6050 IMU library
#include <MS5611.h>         // MS5611 barometer

// ========================================
// PIN DEFINITIONS
// ========================================
// Motor pins (PWM capable)
#define MOTOR_FL  3         // Front Left (CCW)
#define MOTOR_FR  5         // Front Right (CW)
#define MOTOR_RR  6         // Rear Right (CCW)
#define MOTOR_RL  9         // Rear Left (CW)

// Radio pins
#define CE_PIN    4
#define CSN_PIN   10

// Status indicators
#define BUZZER_PIN 7
#define LED_PIN    8

// I2C pins (A4=SDA, A5=SCL) used automatically by Wire library

// ========================================
// HARDWARE OBJECTS
// ========================================
MPU6050 mpu;                // IMU sensor
MS5611 ms5611;              // Barometer
RF24 radio(CE_PIN, CSN_PIN); // Radio module

// ESC objects (controlled like servos)
Servo escFL, escFR, escRR, escRL;

// ========================================
// RADIO CONFIGURATION
// ========================================
const byte address[6] = "00001"; // Must match remote controller

// Data structure for receiving commands (must match transmitter)
struct RadioData {
  int throttle;     // 0-1023 (from joystick)
  int yaw;          // 0-1023
  int pitch;        // 0-1023
  int roll;         // 0-1023
  bool armed;       // Armed state
  bool calibrate;   // Calibration command
  bool motorTest;   // Motor test mode
  bool softLand;    // Auto-land command
  bool angleMode;   // true=ANGLE, false=ACRO
};

RadioData receivedData;
unsigned long lastRadioTime = 0;
const unsigned long RADIO_TIMEOUT = 1000; // 1 second failsafe

// ========================================
// IMU VARIABLES
// ========================================
// Raw sensor readings
int16_t ax, ay, az;     // Accelerometer
int16_t gx, gy, gz;     // Gyroscope

// Processed angles (degrees)
float roll = 0, pitch = 0, yaw = 0;
float rollRate = 0, pitchRate = 0, yawRate = 0;

// Calibration offsets
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
float accelXOffset = 0, accelYOffset = 0, accelZOffset = 0;

// Complementary filter coefficient
const float ALPHA = 0.98; // 98% gyro, 2% accelerometer

// ========================================
// ALTITUDE VARIABLES
// ========================================
float currentAltitude = 0;
float targetAltitude = 0;
float baseAltitude = 0;     // Ground level reference
bool altitudeHold = false;

// ========================================
// PID CONTROLLER VARIABLES
// ========================================

// Roll PID
float rollSetpoint = 0;     // Desired roll angle
float rollError = 0;
float rollPrevError = 0;
float rollIntegral = 0;
float rollDerivative = 0;
float rollPID = 0;

// Pitch PID
float pitchSetpoint = 0;
float pitchError = 0;
float pitchPrevError = 0;
float pitchIntegral = 0;
float pitchDerivative = 0;
float pitchPID = 0;

// Yaw PID
float yawSetpoint = 0;
float yawError = 0;
float yawPrevError = 0;
float yawIntegral = 0;
float yawDerivative = 0;
float yawPID = 0;

// Altitude PID
float altError = 0;
float altPrevError = 0;
float altIntegral = 0;
float altDerivative = 0;
float altPID = 0;

// PID Gains - TUNE THESE VALUES FOR YOUR DRONE
// Roll/Pitch gains
float Kp_roll = 1.5;        // Proportional gain
float Ki_roll = 0.05;       // Integral gain
float Kd_roll = 0.8;        // Derivative gain

float Kp_pitch = 1.5;
float Ki_pitch = 0.05;
float Kd_pitch = 0.8;

// Yaw gains (typically higher Kp)
float Kp_yaw = 3.0;
float Ki_yaw = 0.02;
float Kd_yaw = 0.5;

// Altitude gains
float Kp_alt = 2.0;
float Ki_alt = 0.1;
float Kd_alt = 1.5;

// Integral windup limits
const float INTEGRAL_LIMIT = 400;

// ========================================
// FLIGHT STATE VARIABLES
// ========================================
bool armed = false;
bool motorTestMode = false;
bool angleMode = true;      // true=ANGLE (self-level), false=ACRO
bool calibrated = false;

// Motor speeds (1000-2000 microseconds, standard ESC PWM)
int motorFL_speed = 1000;
int motorFR_speed = 1000;
int motorRR_speed = 1000;
int motorRL_speed = 1000;

const int MOTOR_MIN = 1000;  // ESC minimum throttle
const int MOTOR_MAX = 2000;  // ESC maximum throttle
const int MOTOR_ARM = 1000;  // Armed but not spinning

// ========================================
// TIMING VARIABLES
// ========================================
unsigned long currentTime = 0;
unsigned long previousTime = 0;
float deltaTime = 0;

unsigned long loopTimer = 0;
const int LOOP_TIME = 4000;  // 4ms = 250Hz loop rate

// ========================================
// SETUP FUNCTION
// ========================================
void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println(F("=== Flight Controller Starting ==="));
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Startup sequence: LED on, buzzer beep
  digitalWrite(LED_PIN, HIGH);
  beep(100);
  delay(200);
  beep(100);
  delay(200);
  beep(100);
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz fast mode
  
  // Initialize MPU6050
  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println(F("MPU6050 connected!"));
    beep(50);
  } else {
    Serial.println(F("MPU6050 connection failed!"));
    errorBlink();
  }
  
  // Configure MPU6050
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);  // ±500°/s
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);  // ±4g
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);             // 42Hz low-pass filter
  
  // Initialize MS5611
  Serial.println(F("Initializing MS5611..."));
  if (ms5611.begin()) {
    Serial.println(F("MS5611 connected!"));
    beep(50);
  } else {
    Serial.println(F("MS5611 connection failed!"));
    // Continue anyway - altitude not critical
  }
  
  // Initialize radio
  Serial.println(F("Initializing nRF24L01+..."));
  if (radio.begin()) {
    Serial.println(F("Radio initialized!"));
    radio.openReadingPipe(1, address);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.startListening();
    beep(50);
  } else {
    Serial.println(F("Radio initialization failed!"));
    errorBlink();
  }
  
  // Attach ESCs
  escFL.attach(MOTOR_FL, 1000, 2000);
  escFR.attach(MOTOR_FR, 1000, 2000);
  escRR.attach(MOTOR_RR, 1000, 2000);
  escRL.attach(MOTOR_RL, 1000, 2000);
  
  // Initialize ESCs (send minimum throttle)
  Serial.println(F("Initializing ESCs..."));
  setAllMotors(MOTOR_MIN);
  delay(2000); // Wait for ESC initialization
  
  Serial.println(F("=== Ready for Calibration ==="));
  digitalWrite(LED_PIN, LOW);
  
  // Wait for calibration command
  blinkPattern(3, 200);
}

// ========================================
// MAIN LOOP
// ========================================
void loop() {
  // Maintain consistent loop timing
  loopTimer = micros();
  
  // 1. Read radio data
  readRadio();
  
  // 2. Check for special commands
  processCommands();
  
  // 3. Read sensors
  readSensors();
  
  // 4. Calculate angles (complementary filter)
  calculateAngles();
  
  // 5. Calculate altitude
  calculateAltitude();
  
  // 6. Calculate PID corrections
  if (armed && calibrated) {
    calculatePID();
    
    // 7. Mix motor outputs
    mixMotors();
    
    // 8. Apply motor speeds
    applyMotors();
  } else {
    // Disarmed - keep motors off
    setAllMotors(MOTOR_MIN);
    
    // Reset PID integrators when disarmed
    rollIntegral = 0;
    pitchIntegral = 0;
    yawIntegral = 0;
    altIntegral = 0;
  }
  
  // 9. Status updates
  updateStatus();
  
  // 10. Wait for next loop cycle (maintain 250Hz)
  while (micros() - loopTimer < LOOP_TIME);
  
  // Calculate actual loop time for derivatives
  currentTime = micros();
  deltaTime = (currentTime - previousTime) / 1000000.0; // Convert to seconds
  previousTime = currentTime;
}

// ========================================
// RADIO COMMUNICATION
// ========================================
void readRadio() {
  if (radio.available()) {
    radio.read(&receivedData, sizeof(RadioData));
    lastRadioTime = millis();
    
    // Update flight state from radio
    angleMode = receivedData.angleMode;
    
  } else if (millis() - lastRadioTime > RADIO_TIMEOUT) {
    // FAILSAFE: Radio signal lost
    failsafe();
  }
}

void failsafe() {
  // Emergency procedure for signal loss
  armed = false;
  setAllMotors(MOTOR_MIN);
  
  // Alert with buzzer
  static unsigned long lastBeep = 0;
  if (millis() - lastBeep > 500) {
    beep(100);
    lastBeep = millis();
  }
  
  digitalWrite(LED_PIN, (millis() / 200) % 2); // Fast blink
}

// ========================================
// COMMAND PROCESSING
// ========================================
void processCommands() {
  // Calibration command
  if (receivedData.calibrate && !calibrated) {
    calibrateSensors();
  }
  
  // Motor test mode
  if (receivedData.motorTest) {
    motorTest();
  }
  
  // Arming/Disarming
  if (receivedData.armed && !armed && calibrated) {
    // Check throttle is low before arming
    if (receivedData.throttle < 100) {
      armed = true;
      beep(200); // Long beep for armed
      digitalWrite(LED_PIN, HIGH);
      Serial.println(F("ARMED"));
    }
  } else if (!receivedData.armed && armed) {
    armed = false;
    beep(50); // Short beep for disarmed
    beep(50);
    digitalWrite(LED_PIN, LOW);
    Serial.println(F("DISARMED"));
  }
  
  // Soft landing
  if (receivedData.softLand && armed) {
    softLanding();
  }
}

// ========================================
// SENSOR CALIBRATION
// ========================================
void calibrateSensors() {
  Serial.println(F("=== CALIBRATING SENSORS ==="));
  Serial.println(F("Keep drone still on flat surface!"));
  
  beep(100);
  delay(500);
  
  // Blink during calibration
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  // Collect samples
  long gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  long accelXSum = 0, accelYSum = 0, accelZSum = 0;
  const int samples = 1000;
  
  for (int i = 0; i < samples; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    gyroXSum += gx;
    gyroYSum += gy;
    gyroZSum += gz;
    
    accelXSum += ax;
    accelYSum += ay;
    accelZSum += az;
    
    delay(3);
  }
  
  // Calculate averages
  gyroXOffset = gyroXSum / (float)samples;
  gyroYOffset = gyroYSum / (float)samples;
  gyroZOffset = gyroZSum / (float)samples;
  
  accelXOffset = accelXSum / (float)samples;
  accelYOffset = accelYSum / (float)samples;
  accelZOffset = (accelZSum / (float)samples) - 16384; // -1g on Z axis
  
  // Set base altitude
  baseAltitude = ms5611.getAltitude(ms5611.readPressure());
  
  calibrated = true;
  
  Serial.println(F("Calibration complete!"));
  Serial.print(F("Gyro offsets: "));
  Serial.print(gyroXOffset); Serial.print(", ");
  Serial.print(gyroYOffset); Serial.print(", ");
  Serial.println(gyroZOffset);
  
  beep(50);
  delay(100);
  beep(50);
  
  digitalWrite(LED_PIN, HIGH);
}

// ========================================
// SENSOR READING
// ========================================
void readSensors() {
  // Read raw IMU data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Apply calibration offsets
  gx -= gyroXOffset;
  gy -= gyroYOffset;
  gz -= gyroZOffset;
  
  ax -= accelXOffset;
  ay -= accelYOffset;
  az -= accelZOffset;
  
  // Convert to meaningful units
  // Gyro: LSB/°/s = 65.5 for ±500°/s range
  rollRate = gx / 65.5;
  pitchRate = gy / 65.5;
  yawRate = gz / 65.5;
  
  // Accelerometer: LSB/g = 8192 for ±4g range
  float accelX = ax / 8192.0;
  float accelY = ay / 8192.0;
  float accelZ = az / 8192.0;
}

// ========================================
// COMPLEMENTARY FILTER
// ========================================
void calculateAngles() {
  // Convert accelerometer readings to angles
  float accelRoll = atan2(ay, az) * 180.0 / PI;
  float accelPitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
  
  // Integrate gyro rates
  roll += rollRate * deltaTime;
  pitch += pitchRate * deltaTime;
  yaw += yawRate * deltaTime;
  
  // Apply complementary filter
  // High-pass filter on gyro (98%) + Low-pass filter on accel (2%)
  roll = ALPHA * roll + (1.0 - ALPHA) * accelRoll;
  pitch = ALPHA * pitch + (1.0 - ALPHA) * accelPitch;
  
  // Note: Yaw cannot be corrected with accelerometer (no gravity reference)
  // For accurate yaw, magnetometer would be needed
}

// ========================================
// ALTITUDE CALCULATION
// ========================================
void calculateAltitude() {
  currentAltitude = ms5611.getAltitude(ms5611.readPressure()) - baseAltitude;
}

// ========================================
// PID CONTROLLER
// ========================================
void calculatePID() {
  // Convert radio inputs to setpoints
  if (angleMode) {
    // ANGLE MODE: Stick controls desired angle (±30°)
    rollSetpoint = map(receivedData.roll, 0, 1023, -30, 30);
    pitchSetpoint = map(receivedData.pitch, 0, 1023, -30, 30);
  } else {
    // ACRO MODE: Stick controls rotation rate (±200°/s)
    rollSetpoint = map(receivedData.roll, 0, 1023, -200, 200);
    pitchSetpoint = map(receivedData.pitch, 0, 1023, -200, 200);
  }
  
  // Yaw always controls rate (±180°/s)
  yawSetpoint = map(receivedData.yaw, 0, 1023, -180, 180);
  
  // --- ROLL PID ---
  if (angleMode) {
    rollError = rollSetpoint - roll;
  } else {
    rollError = rollSetpoint - rollRate; // Rate mode
  }
  
  rollIntegral += rollError * deltaTime;
  rollIntegral = constrain(rollIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  
  rollDerivative = (rollError - rollPrevError) / deltaTime;
  rollPrevError = rollError;
  
  rollPID = (Kp_roll * rollError) + (Ki_roll * rollIntegral) + (Kd_roll * rollDerivative);
  
  // --- PITCH PID ---
  if (angleMode) {
    pitchError = pitchSetpoint - pitch;
  } else {
    pitchError = pitchSetpoint - pitchRate;
  }
  
  pitchIntegral += pitchError * deltaTime;
  pitchIntegral = constrain(pitchIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  
  pitchDerivative = (pitchError - pitchPrevError) / deltaTime;
  pitchPrevError = pitchError;
  
  pitchPID = (Kp_pitch * pitchError) + (Ki_pitch * pitchIntegral) + (Kd_pitch * pitchDerivative);
  
  // --- YAW PID ---
  yawError = yawSetpoint - yawRate;
  
  yawIntegral += yawError * deltaTime;
  yawIntegral = constrain(yawIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  
  yawDerivative = (yawError - yawPrevError) / deltaTime;
  yawPrevError = yawError;
  
  yawPID = (Kp_yaw * yawError) + (Ki_yaw * yawIntegral) + (Kd_yaw * yawDerivative);
  
  // --- ALTITUDE PID (if enabled) ---
  if (altitudeHold) {
    altError = targetAltitude - currentAltitude;
    
    altIntegral += altError * deltaTime;
    altIntegral = constrain(altIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    
    altDerivative = (altError - altPrevError) / deltaTime;
    altPrevError = altError;
    
    altPID = (Kp_alt * altError) + (Ki_alt * altIntegral) + (Kd_alt * altDerivative);
  } else {
    altPID = 0;
    altIntegral = 0;
  }
}

// ========================================
// MOTOR MIXING
// ========================================
void mixMotors() {
  // Base throttle from radio
  int baseThrottle = map(receivedData.throttle, 0, 1023, MOTOR_MIN, MOTOR_MAX);
  baseThrottle = constrain(baseThrottle, MOTOR_MIN, MOTOR_MAX);
  
  // Add altitude hold correction
  baseThrottle += altPID;
  
  /*
   * X-FRAME MOTOR CONFIGURATION:
   * 
   *      FRONT
   *   M1 ↺   ↻ M2
   *      \ X /
   *      / X \
   *   M4 ↻   ↺ M3
   *      REAR
   * 
   * CONTROL MIXING:
   * - ROLL:  Left/Right tilt
   * - PITCH: Forward/Backward tilt
   * - YAW:   Rotation (CW/CCW motor speed difference)
   */
  
  // Motor 1: Front Left (CCW)
  motorFL_speed = baseThrottle - rollPID - pitchPID + yawPID;
  
  // Motor 2: Front Right (CW)
  motorFR_speed = baseThrottle + rollPID - pitchPID - yawPID;
  
  // Motor 3: Rear Right (CCW)
  motorRR_speed = baseThrottle + rollPID + pitchPID + yawPID;
  
  // Motor 4: Rear Left (CW)
  motorRL_speed = baseThrottle - rollPID + pitchPID - yawPID;
  
  // Constrain all motors to valid range
  motorFL_speed = constrain(motorFL_speed, MOTOR_MIN, MOTOR_MAX);
  motorFR_speed = constrain(motorFR_speed, MOTOR_MIN, MOTOR_MAX);
  motorRR_speed = constrain(motorRR_speed, MOTOR_MIN, MOTOR_MAX);
  motorRL_speed = constrain(motorRL_speed, MOTOR_MIN, MOTOR_MAX);
  
  // Safety: If extreme tilt detected (>45°), disarm immediately
  if (abs(roll) > 45 || abs(pitch) > 45) {
    armed = false;
    Serial.println(F("EMERGENCY DISARM: Extreme tilt!"));
    beep(200);
  }
}

// ========================================
// MOTOR CONTROL
// ========================================
void applyMotors() {
  escFL.writeMicroseconds(motorFL_speed);
  escFR.writeMicroseconds(motorFR_speed);
  escRR.writeMicroseconds(motorRR_speed);
  escRL.writeMicroseconds(motorRL_speed);
}

void setAllMotors(int speed) {
  escFL.writeMicroseconds(speed);
  escFR.writeMicroseconds(speed);
  escRR.writeMicroseconds(speed);
  escRL.writeMicroseconds(speed);
}

// ========================================
// SPECIAL MODES
// ========================================
void motorTest() {
  // Test all motors at low speed (NO PROPELLERS!)
  Serial.println(F("=== MOTOR TEST ==="));
  Serial.println(F("WARNING: Remove all propellers!"));
  
  armed = false; // Safety
  
  beep(100);
  delay(1000);
  
  // Test each motor individually
  Serial.println(F("Testing Front Left..."));
  escFL.writeMicroseconds(1200);
  delay(2000);
  escFL.writeMicroseconds(1000);
  delay(500);
  
  Serial.println(F("Testing Front Right..."));
  escFR.writeMicroseconds(1200);
  delay(2000);
  escFR.writeMicroseconds(1000);
  delay(500);
  
  Serial.println(F("Testing Rear Right..."));
  escRR.writeMicroseconds(1200);
  delay(2000);
  escRR.writeMicroseconds(1000);
  delay(500);
  
  Serial.println(F("Testing Rear Left..."));
  escRL.writeMicroseconds(1200);
  delay(2000);
  escRL.writeMicroseconds(1000);
  
  Serial.println(F("Motor test complete!"));
  beep(50);
  beep(50);
}

void softLanding() {
  // Gradually reduce throttle for smooth landing
  Serial.println(F("Soft landing initiated..."));
  
  int currentThrottle = receivedData.throttle;
  
  while (currentThrottle > 100 && armed) {
    currentThrottle -= 2; // Reduce slowly
    
    // Continue stabilization with reduced throttle
    readSensors();
    calculateAngles();
    
    // Override throttle for landing
    receivedData.throttle = currentThrottle;
    
    calculatePID();
    mixMotors();
    applyMotors();
    
    delay(20); // 50Hz landing rate
  }
  
  // Touch down - disarm
  armed = false;
  setAllMotors(MOTOR_MIN);
  
  Serial.println(F("Landed."));
  beep(50);
  delay(100);
  beep(50);
}

// ========================================
// STATUS & FEEDBACK
// ========================================
void updateStatus() {
  // LED indicates armed state
  if (armed) {
    digitalWrite(LED_PIN, HIGH);
  } else if (calibrated) {
    // Slow blink when ready but disarmed
    digitalWrite(LED_PIN, (millis() / 1000) % 2);
  } else {
    // Fast blink when not calibrated
    digitalWrite(LED_PIN, (millis() / 200) % 2);
  }
  
  // Debug output (reduce frequency to avoid serial bottleneck)
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) { // 2Hz debug output
    Serial.print(F("R:"));
    Serial.print(roll, 1);
    Serial.print(F(" P:"));
    Serial.print(pitch, 1);
    Serial.print(F(" Y:"));
    Serial.print(yaw, 1);
    Serial.print(F(" | Alt:"));
    Serial.print(currentAltitude, 1);
    Serial.print(F(" | Armed:"));
    Serial.print(armed);
    Serial.print(F(" | Mode:"));
    Serial.println(angleMode ? "ANGLE" : "ACRO");
    
    lastDebug = millis();
  }
}

// ========================================
// HELPER FUNCTIONS
// ========================================
void beep(int duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

void blinkPattern(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW);
    delay(duration);
  }
}

void errorBlink() {
  // Continuous error indication
  while (true) {
    digitalWrite(LED_PIN, HIGH);
    beep(100);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

// ========================================
// END OF FLIGHT CONTROLLER CODE
// ========================================
