/*
 * Drone Flight Controller (Receiver)
 * TMRh20 RF24 Library - NO ACK Mode for Low Latency
 * 
 * Hardware:
 * - Arduino Nano/Uno (or Pro Mini for weight savings)
 * - NRF24L01+ module (with PA+LNA recommended)
 * - MPU6050 IMU (Gyro + Accelerometer)
 * - 4x ESCs with motors
 * - LiPo Battery (3S or 4S)
 * 
 * Motor Layout (X configuration):
 *     FRONT
 *   M1     M2
 *     \   /
 *      [X]
 *     /   \
 *   M3     M4
 *     BACK
 * 
 * M1 (Front-Left):  CCW
 * M2 (Front-Right): CW
 * M3 (Back-Left):   CW
 * M4 (Back-Right):  CCW
 * 
 * Wiring:
 * NRF24L01:
 *   VCC  -> 3.3V (add 10-100uF capacitor!)
 *   GND  -> GND
 *   CE   -> Pin 9
 *   CSN  -> Pin 10
 *   SCK  -> Pin 13
 *   MOSI -> Pin 11
 *   MISO -> Pin 12
 * 
 * MPU6050:
 *   VCC  -> 5V
 *   GND  -> GND
 *   SDA  -> A4
 *   SCL  -> A5
 * 
 * ESCs:
 *   M1 -> Pin 3 (PWM)
 *   M2 -> Pin 5 (PWM)
 *   M3 -> Pin 6 (PWM)
 *   M4 -> Pin 11 (PWM) -- Note: conflicts with SPI, use different pin if needed
 */

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>
#include "../common/RF24_Config.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Motor pins (PWM capable)
#define MOTOR1_PIN  3   // Front-Left (CCW)
#define MOTOR2_PIN  5   // Front-Right (CW)
#define MOTOR3_PIN  6   // Back-Left (CW)
#define MOTOR4_PIN  A0  // Back-Right (CCW) - Using A0 as digital to avoid SPI conflict

// Status LED
#define LED_PIN     LED_BUILTIN

// Battery voltage divider (optional)
#define BATTERY_PIN A7

// ============================================================================
// MPU6050 CONFIGURATION
// ============================================================================

#define MPU6050_ADDR  0x68

// Gyro sensitivity: 65.5 LSB/(°/s) for ±500°/s range
#define GYRO_SENSITIVITY  65.5

// Accelerometer calibration (set these after calibration!)
int16_t accXOffset = 0;
int16_t accYOffset = 0;
int16_t accZOffset = 0;

// Gyro calibration (calculated at startup)
float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

// ============================================================================
// PID CONFIGURATION - TUNE THESE FOR YOUR DRONE!
// ============================================================================

// Roll PID
float rollKp = 1.3;
float rollKi = 0.04;
float rollKd = 18.0;

// Pitch PID
float pitchKp = 1.3;
float pitchKi = 0.04;
float pitchKd = 18.0;

// Yaw PID
float yawKp = 4.0;
float yawKi = 0.02;
float yawKd = 0.0;

// PID output limits
#define PID_MAX   400
#define PID_I_MAX 100  // Integral windup limit

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================

#define MOTOR_MIN     1000  // Minimum ESC signal (μs)
#define MOTOR_MAX     2000  // Maximum ESC signal (μs)
#define MOTOR_ARM     1000  // Arming signal (minimum throttle)
#define MOTOR_IDLE    1100  // Idle speed when armed

// Throttle limits
#define THROTTLE_MIN  0
#define THROTTLE_MAX  1000

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// RF24 radio
RF24 radio(RX_CE_PIN, RX_CSN_PIN);

// ESC control using Servo library
Servo motor1, motor2, motor3, motor4;

// Received control data
ControlData controlData;

// Signal status
unsigned long lastReceiveTime = 0;
bool signalLost = false;
bool radioInitialized = false;

// Armed state
bool armed = false;
bool armSwitchPrevious = false;

// IMU data
float gyroRoll, gyroPitch, gyroYaw;   // Gyro rates (°/s)
float accRoll, accPitch;               // Accelerometer angles
float angleRoll, anglePitch;           // Complementary filter angles

// PID variables
float rollError, rollPrevError, rollIntegral, rollDerivative, rollOutput;
float pitchError, pitchPrevError, pitchIntegral, pitchDerivative, pitchOutput;
float yawError, yawPrevError, yawIntegral, yawDerivative, yawOutput;

// Motor outputs
int motor1Speed, motor2Speed, motor3Speed, motor4Speed;

// Timing
unsigned long loopTimer;
unsigned long lastLoopTime;
float dt;  // Delta time in seconds

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println(F("=== Drone Flight Controller ==="));
  Serial.println(F("TMRh20 RF24 Library - NO ACK Mode"));
  
  // Setup LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED on during setup
  
  // Initialize I2C for MPU6050
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C for faster reads
  
  // Initialize MPU6050
  initMPU6050();
  
  // Calibrate gyro (KEEP DRONE STILL!)
  calibrateGyro();
  
  // Initialize motors
  initMotors();
  
  // Initialize radio
  initRadio();
  
  // Initialize control data to safe values
  memset(&controlData, 0, sizeof(ControlData));
  
  // Initialize timing
  lastLoopTime = micros();
  loopTimer = micros();
  
  digitalWrite(LED_PIN, LOW);  // LED off = ready
  
  Serial.println(F("Flight Controller ready!"));
  Serial.println(F("Waiting for transmitter signal..."));
}

// ============================================================================
// RADIO INITIALIZATION
// ============================================================================

void initRadio() {
  Serial.println(F("Initializing NRF24L01..."));
  
  if (!radio.begin()) {
    Serial.println(F("ERROR: Radio not responding!"));
    radioInitialized = false;
    return;
  }
  
  // Configure to match transmitter
  radio.setChannel(RF_CHANNEL);
  radio.setDataRate(RF_DATA_RATE);
  radio.setPALevel(RF_PA_LEVEL);
  
  // Disable auto-acknowledgment (must match transmitter!)
  radio.setAutoAck(false);
  
  // Set payload size
  radio.setPayloadSize(sizeof(ControlData));
  
  // Open reading pipe
  radio.openReadingPipe(1, RADIO_ADDRESS);
  
  // Start listening for data
  radio.startListening();
  
  radioInitialized = true;
  
  Serial.println(F("Radio initialized!"));
  Serial.print(F("Channel: "));
  Serial.println(RF_CHANNEL);
  Serial.println(F("Listening for transmitter..."));
  
  radio.printDetails();
}

// ============================================================================
// MPU6050 INITIALIZATION
// ============================================================================

void initMPU6050() {
  Serial.println(F("Initializing MPU6050..."));
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Wake up
  Wire.endTransmission();
  delay(100);
  
  // Configure gyro range: ±500°/s
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x08);  // ±500°/s
  Wire.endTransmission();
  
  // Configure accelerometer range: ±8g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x10);  // ±8g
  Wire.endTransmission();
  
  // Configure digital low pass filter
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x03);  // DLPF ~43Hz
  Wire.endTransmission();
  
  Serial.println(F("MPU6050 initialized!"));
}

// ============================================================================
// GYRO CALIBRATION
// ============================================================================

void calibrateGyro() {
  Serial.println(F("Calibrating gyro... KEEP DRONE STILL!"));
  
  float sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 2000;
  
  for (int i = 0; i < samples; i++) {
    int16_t gx, gy, gz;
    readGyroRaw(&gx, &gy, &gz);
    
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    
    // Blink LED during calibration
    if (i % 200 == 0) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    
    delayMicroseconds(500);  // ~2 seconds total
  }
  
  gyroXOffset = sumX / samples;
  gyroYOffset = sumY / samples;
  gyroZOffset = sumZ / samples;
  
  Serial.print(F("Gyro offsets - X: "));
  Serial.print(gyroXOffset);
  Serial.print(F(" Y: "));
  Serial.print(gyroYOffset);
  Serial.print(F(" Z: "));
  Serial.println(gyroZOffset);
  
  Serial.println(F("Gyro calibration complete!"));
}

// ============================================================================
// MOTOR INITIALIZATION
// ============================================================================

void initMotors() {
  Serial.println(F("Initializing ESCs..."));
  
  // Attach motors
  motor1.attach(MOTOR1_PIN, MOTOR_MIN, MOTOR_MAX);
  motor2.attach(MOTOR2_PIN, MOTOR_MIN, MOTOR_MAX);
  motor3.attach(MOTOR3_PIN, MOTOR_MIN, MOTOR_MAX);
  motor4.attach(MOTOR4_PIN, MOTOR_MIN, MOTOR_MAX);
  
  // Send minimum throttle to arm ESCs
  motor1.writeMicroseconds(MOTOR_MIN);
  motor2.writeMicroseconds(MOTOR_MIN);
  motor3.writeMicroseconds(MOTOR_MIN);
  motor4.writeMicroseconds(MOTOR_MIN);
  
  delay(2000);  // Wait for ESCs to initialize
  
  Serial.println(F("ESCs initialized!"));
}

// ============================================================================
// MAIN LOOP - 250Hz (4ms cycle)
// ============================================================================

void loop() {
  // Calculate delta time
  unsigned long currentMicros = micros();
  dt = (currentMicros - lastLoopTime) / 1000000.0;  // Convert to seconds
  lastLoopTime = currentMicros;
  
  // 1. Receive control data from transmitter
  receiveData();
  
  // 2. Check for signal loss
  checkSignal();
  
  // 3. Handle arming/disarming
  handleArming();
  
  // 4. Read IMU data
  readIMU();
  
  // 5. Calculate angles (complementary filter)
  calculateAngles();
  
  // 6. Run PID controllers
  calculatePID();
  
  // 7. Calculate motor outputs
  calculateMotorSpeeds();
  
  // 8. Apply motor outputs
  applyMotorSpeeds();
  
  // 9. Debug output (limited rate)
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 200) {
    lastDebug = millis();
    printDebugInfo();
  }
  
  // Maintain loop timing (~250Hz = 4000μs per loop)
  while (micros() - loopTimer < 4000);
  loopTimer = micros();
}

// ============================================================================
// RECEIVE DATA
// ============================================================================

void receiveData() {
  if (!radioInitialized) return;
  
  // Check if data is available
  if (radio.available()) {
    // Read the data
    radio.read(&controlData, sizeof(ControlData));
    
    // Validate checksum
    if (validateControlData(controlData)) {
      lastReceiveTime = millis();
      signalLost = false;
    }
    // If checksum fails, ignore packet and keep old data
  }
}

// ============================================================================
// SIGNAL MONITORING
// ============================================================================

void checkSignal() {
  if (millis() - lastReceiveTime > SIGNAL_TIMEOUT_MS) {
    if (!signalLost) {
      signalLost = true;
      Serial.println(F("WARNING: Signal lost! Activating failsafe."));
    }
    
    // FAILSAFE: Reduce throttle gradually
    if (armed) {
      // Gradual throttle reduction
      if (controlData.throttle > 0) {
        controlData.throttle = max(0, (int)controlData.throttle - 10);
      }
      
      // Zero out control inputs
      controlData.roll = 0;
      controlData.pitch = 0;
      controlData.yaw = 0;
      
      // If throttle reaches zero, disarm after a delay
      if (controlData.throttle == 0) {
        static unsigned long failsafeDisarmTime = 0;
        if (failsafeDisarmTime == 0) {
          failsafeDisarmTime = millis();
        } else if (millis() - failsafeDisarmTime > 2000) {
          armed = false;
          failsafeDisarmTime = 0;
          Serial.println(F("Failsafe: Disarmed"));
        }
      }
    }
  }
}

// ============================================================================
// ARMING / DISARMING
// ============================================================================

void handleArming() {
  // Arm switch is AUX1 (>127 = armed)
  bool armSwitch = controlData.aux1 > 127;
  
  // Detect switch change
  if (armSwitch != armSwitchPrevious) {
    armSwitchPrevious = armSwitch;
    
    if (armSwitch) {
      // Attempt to arm
      // Safety: Only arm if throttle is at minimum
      if (controlData.throttle < 50) {
        armed = true;
        
        // Reset PID integrals
        rollIntegral = 0;
        pitchIntegral = 0;
        yawIntegral = 0;
        
        Serial.println(F(">>> ARMED <<<"));
      } else {
        Serial.println(F("Cannot arm: Throttle not at minimum!"));
      }
    } else {
      // Disarm
      armed = false;
      Serial.println(F(">>> DISARMED <<<"));
    }
  }
}

// ============================================================================
// IMU READING
// ============================================================================

void readGyroRaw(int16_t* gx, int16_t* gy, int16_t* gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);  // Starting with GYRO_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6);
  
  *gx = Wire.read() << 8 | Wire.read();
  *gy = Wire.read() << 8 | Wire.read();
  *gz = Wire.read() << 8 | Wire.read();
}

void readIMU() {
  // Read all 14 bytes (Accel + Temp + Gyro)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // Starting with ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);
  
  int16_t accX = Wire.read() << 8 | Wire.read();
  int16_t accY = Wire.read() << 8 | Wire.read();
  int16_t accZ = Wire.read() << 8 | Wire.read();
  int16_t temp = Wire.read() << 8 | Wire.read();  // Not used
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  
  (void)temp;  // Suppress unused warning
  
  // Apply gyro calibration and convert to °/s
  gyroRoll = (gyroX - gyroXOffset) / GYRO_SENSITIVITY;
  gyroPitch = (gyroY - gyroYOffset) / GYRO_SENSITIVITY;
  gyroYaw = (gyroZ - gyroZOffset) / GYRO_SENSITIVITY;
  
  // Calculate accelerometer angles
  // Note: These formulas assume drone is level in X-Y plane
  accRoll = atan2(accY, accZ) * 180.0 / PI;
  accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
}

// ============================================================================
// COMPLEMENTARY FILTER
// ============================================================================

void calculateAngles() {
  // Complementary filter coefficient
  // Higher = trust gyro more (faster response, more drift)
  // Lower = trust accelerometer more (slower response, more noise)
  const float alpha = 0.98;
  
  // First run initialization
  static bool initialized = false;
  if (!initialized) {
    angleRoll = accRoll;
    anglePitch = accPitch;
    initialized = true;
    return;
  }
  
  // Integrate gyro rates
  angleRoll += gyroRoll * dt;
  anglePitch += gyroPitch * dt;
  
  // Correct drift with accelerometer
  angleRoll = alpha * angleRoll + (1.0 - alpha) * accRoll;
  anglePitch = alpha * anglePitch + (1.0 - alpha) * accPitch;
}

// ============================================================================
// PID CONTROLLER
// ============================================================================

void calculatePID() {
  // Convert control inputs to desired rates (°/s)
  // Stick input is -500 to +500, convert to degrees per second
  float desiredRollRate = controlData.roll * 0.5;   // ±250°/s max
  float desiredPitchRate = controlData.pitch * 0.5;  // ±250°/s max
  float desiredYawRate = controlData.yaw * 0.5;      // ±250°/s max
  
  // For angle mode (self-leveling), use angles instead of rates
  // For rate/acro mode, use gyro rates directly
  bool angleMode = controlData.aux2 < 127;  // AUX2 selects mode
  
  if (angleMode) {
    // ANGLE MODE (self-leveling)
    // Stick input becomes desired angle (-25° to +25°)
    float desiredAngleRoll = controlData.roll * 0.05;   // ±25° max
    float desiredAnglePitch = controlData.pitch * 0.05; // ±25° max
    
    // Calculate error as difference between desired and actual angle
    rollError = desiredAngleRoll - angleRoll;
    pitchError = desiredAnglePitch - anglePitch;
    
    // For yaw, always use rate
    yawError = desiredYawRate - gyroYaw;
  } else {
    // RATE MODE (acro)
    // Error is difference between desired and actual rate
    rollError = desiredRollRate - gyroRoll;
    pitchError = desiredPitchRate - gyroPitch;
    yawError = desiredYawRate - gyroYaw;
  }
  
  // ---- ROLL PID ----
  rollIntegral += rollError * dt;
  rollIntegral = constrain(rollIntegral, -PID_I_MAX, PID_I_MAX);  // Prevent windup
  rollDerivative = (rollError - rollPrevError) / dt;
  rollOutput = rollKp * rollError + rollKi * rollIntegral + rollKd * rollDerivative;
  rollOutput = constrain(rollOutput, -PID_MAX, PID_MAX);
  rollPrevError = rollError;
  
  // ---- PITCH PID ----
  pitchIntegral += pitchError * dt;
  pitchIntegral = constrain(pitchIntegral, -PID_I_MAX, PID_I_MAX);
  pitchDerivative = (pitchError - pitchPrevError) / dt;
  pitchOutput = pitchKp * pitchError + pitchKi * pitchIntegral + pitchKd * pitchDerivative;
  pitchOutput = constrain(pitchOutput, -PID_MAX, PID_MAX);
  pitchPrevError = pitchError;
  
  // ---- YAW PID ----
  yawIntegral += yawError * dt;
  yawIntegral = constrain(yawIntegral, -PID_I_MAX, PID_I_MAX);
  yawDerivative = (yawError - yawPrevError) / dt;
  yawOutput = yawKp * yawError + yawKi * yawIntegral + yawKd * yawDerivative;
  yawOutput = constrain(yawOutput, -PID_MAX, PID_MAX);
  yawPrevError = yawError;
  
  // Reset integrals when not armed (prevents buildup on ground)
  if (!armed) {
    rollIntegral = 0;
    pitchIntegral = 0;
    yawIntegral = 0;
  }
}

// ============================================================================
// MOTOR MIXING
// ============================================================================

void calculateMotorSpeeds() {
  if (!armed) {
    motor1Speed = MOTOR_MIN;
    motor2Speed = MOTOR_MIN;
    motor3Speed = MOTOR_MIN;
    motor4Speed = MOTOR_MIN;
    return;
  }
  
  // Map throttle (0-1000) to motor range with idle offset
  int throttle = map(controlData.throttle, 0, 1000, MOTOR_IDLE, MOTOR_MAX);
  
  // Motor mixing for X configuration
  // M1 (Front-Left, CCW):  + Pitch, + Roll, - Yaw
  // M2 (Front-Right, CW):  + Pitch, - Roll, + Yaw
  // M3 (Back-Left, CW):    - Pitch, + Roll, + Yaw
  // M4 (Back-Right, CCW):  - Pitch, - Roll, - Yaw
  
  motor1Speed = throttle + pitchOutput + rollOutput - yawOutput;
  motor2Speed = throttle + pitchOutput - rollOutput + yawOutput;
  motor3Speed = throttle - pitchOutput + rollOutput + yawOutput;
  motor4Speed = throttle - pitchOutput - rollOutput - yawOutput;
  
  // Constrain to valid ESC range
  motor1Speed = constrain(motor1Speed, MOTOR_IDLE, MOTOR_MAX);
  motor2Speed = constrain(motor2Speed, MOTOR_IDLE, MOTOR_MAX);
  motor3Speed = constrain(motor3Speed, MOTOR_IDLE, MOTOR_MAX);
  motor4Speed = constrain(motor4Speed, MOTOR_IDLE, MOTOR_MAX);
  
  // If throttle is at minimum, stop all motors
  if (controlData.throttle < 50) {
    motor1Speed = MOTOR_MIN;
    motor2Speed = MOTOR_MIN;
    motor3Speed = MOTOR_MIN;
    motor4Speed = MOTOR_MIN;
  }
}

// ============================================================================
// APPLY MOTOR SPEEDS
// ============================================================================

void applyMotorSpeeds() {
  motor1.writeMicroseconds(motor1Speed);
  motor2.writeMicroseconds(motor2Speed);
  motor3.writeMicroseconds(motor3Speed);
  motor4.writeMicroseconds(motor4Speed);
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

void printDebugInfo() {
  Serial.print(armed ? F("ARMED ") : F("DISARM "));
  Serial.print(signalLost ? F("NO-SIG ") : F("OK "));
  
  Serial.print(F("R:"));
  Serial.print(angleRoll, 1);
  Serial.print(F(" P:"));
  Serial.print(anglePitch, 1);
  
  Serial.print(F(" M:"));
  Serial.print(motor1Speed);
  Serial.print(F(","));
  Serial.print(motor2Speed);
  Serial.print(F(","));
  Serial.print(motor3Speed);
  Serial.print(F(","));
  Serial.println(motor4Speed);
}
