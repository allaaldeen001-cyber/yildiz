/*
 * Professional Quadcopter Flight Controller
 * 
 * Hardware Configuration:
 * - Arduino Nano
 * - nRF24L01+ PA (CE:D4, CSN:D10)
 * - MPU6050 (I2C: A4/A5, INT:D2)
 * - Buzzer: D8
 * - Status LED: D7
 * - ESC/Motors: FL:D3, FR:D5, RR:D6, RL:D9
 * 
 * Author: Professional Quadcopter System
 * Version: 2.2 - IMPROVED STABILIZATION
 */

#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

// ============================================
// PIN DEFINITIONS
// ============================================
#define CE_PIN 4
#define CSN_PIN 10
#define MPU_INT_PIN 2
#define BUZZER_PIN 8
#define LED_PIN 7

#define MOTOR_FL 3  // Front Left
#define MOTOR_FR 5  // Front Right
#define MOTOR_RR 6  // Rear Right
#define MOTOR_RL 9  // Rear Left

// ============================================
// MPU6050 REGISTERS
// ============================================
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B

// ============================================
// CONFIGURATION CONSTANTS
// ============================================
#define MOTOR_MIN 1000
#define MOTOR_MAX 2000
#define MOTOR_ARM_VALUE 1000
#define THROTTLE_MIN 1000
#define THROTTLE_MAX 2000

// Flight modes
#define MODE_DISARMED 0
#define MODE_ARMED 1
#define MODE_ANGLE 2      // Auto-level / Stabilize mode
#define MODE_ACRO 3       // Manual / Rate mode
#define MODE_MOTOR_TEST 4 // Motor test mode

// PID Configuration - TUNED FOR STABILITY
// These gains work with the improved PID algorithm!
#define PID_ROLL_KP 1.3      // Proportional: How hard to correct
#define PID_ROLL_KI 0.03     // Integral: Eliminate steady-state error
#define PID_ROLL_KD 15.0     // Derivative: Dampen oscillations

#define PID_PITCH_KP 1.3     // Match roll for symmetric behavior
#define PID_PITCH_KI 0.03
#define PID_PITCH_KD 15.0

#define PID_YAW_KP 3.0       // Yaw needs higher P
#define PID_YAW_KI 0.02
#define PID_YAW_KD 0.0       // No D term for yaw

#define PID_LIMIT 400.0      // Maximum PID output

// Loop timing
#define MAIN_LOOP_TIME 4000  // 4ms = 250Hz
#define FAILSAFE_TIMEOUT 1000  // 1 second

// ============================================
// COMMUNICATION STRUCTURE
// ============================================
struct ControlData {
  int16_t throttle;     // 1000-2000
  int16_t roll;         // -500 to +500
  int16_t pitch;        // -500 to +500
  int16_t yaw;          // -500 to +500
  uint8_t switches;     // Bit field for switches
  uint8_t buttons;      // Bit field for buttons
  uint8_t checksum;     // Data integrity
};

struct TelemetryData {
  float roll;
  float pitch;
  float yaw;
  uint8_t battery;
  uint8_t flightMode;
  uint8_t armed;
};

// ============================================
// GLOBAL VARIABLES
// ============================================
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipeIn = 0xE8E8F0F0E1LL;
const uint64_t pipeOut = 0xE8E8F0F0E2LL;

Servo motorFL, motorFR, motorRR, motorRL;

ControlData rxData;
TelemetryData telemetry;

// IMU data
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float angleRoll, anglePitch, angleYaw;
float gyroRollInput, gyroPitchInput, gyroYawInput;

// Calibration offsets
float gyroRollCal, gyroPitchCal, gyroYawCal;
float accelXCal, accelYCal, accelZCal;

// PID variables
float pidRollSetpoint, pidPitchSetpoint, pidYawSetpoint;
float pidRoll, pidPitch, pidYaw;
float pidRollPrev, pidPitchPrev, pidYawPrev;
float pidRollI, pidPitchI, pidYawI;
float pidRollD, pidPitchD, pidYawD;

// Motor outputs
int motorFLSpeed, motorFRSpeed, motorRRSpeed, motorRLSpeed;

// System state
uint8_t flightMode = MODE_DISARMED;
bool armed = false;
bool calibrated = false;
unsigned long lastReceiveTime = 0;
unsigned long loopTimer;
unsigned long lastDebugTime = 0;

// Button states
bool lastButton1 = false;
bool lastButton2 = false;
bool lastButton3 = false;
bool motorTestActive = false;
bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
int motorTestSpeed = 1000;
bool softLandingActive = false;
unsigned long landingStartTime = 0;

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MPU_INT_PIN, INPUT);
  
  digitalWrite(LED_PIN, LOW);
  
  Serial.println(F("================================="));
  Serial.println(F("Quadcopter Flight Controller v2.2"));
  Serial.println(F("IMPROVED STABILIZATION"));
  Serial.println(F("================================="));
  Serial.println(F(""));
  Serial.println(F("Button Functions from RC:"));
  Serial.println(F("  BTN1 (D4) - Calibrate sensors"));
  Serial.println(F("  BTN2 (D5) - Motor test/direction check"));
  Serial.println(F("  BTN3 (D6) - ARM for flight"));
  Serial.println(F("  BTN4 (D7) - Soft landing mode"));
  Serial.println(F(""));
  
  // Initialize motors
  initMotors();
  
  // Initialize radio
  if (!initRadio()) {
    errorBlink(3);
  }
  
  // Initialize MPU6050
  if (!initMPU6050()) {
    errorBlink(4);
  }
  
  // Calibrate sensors
  calibrateSensors();
  
  // System ready
  Serial.println(F("System Ready!"));
  digitalWrite(LED_PIN, HIGH);
  beep(2, 100);
  
  loopTimer = micros();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // Read radio data
  receiveRadioData();
  
  // Process button commands
  processButtons();
  
  // Read IMU
  readIMU();
  
  // Calculate angles
  calculateAngles();
  
  // Process flight control
  if (motorTestActive) {
    // Motor test mode - smooth ramp
    runMotorTest();
  } else if (armed) {
    calculatePID();
    mixMotors();
  } else {
    stopMotors();
    resetPID();
  }
  
  // Write motor outputs
  updateMotors();
  
  // Check flight mode and arming
  processFlightMode();
  
  // Failsafe check
  checkFailsafe();
  
  // Status indicators
  updateStatusLED();
  
  // Buzzer control
  processBuzzer();
  
  // Debug output (every 100ms)
  printDebugInfo();
  
  // Maintain loop rate (250Hz)
  while (micros() - loopTimer < MAIN_LOOP_TIME);
  loopTimer = micros();
}

// ============================================
// MOTOR INITIALIZATION
// ============================================
void initMotors() {
  motorFL.attach(MOTOR_FL);
  motorFR.attach(MOTOR_FR);
  motorRR.attach(MOTOR_RR);
  motorRL.attach(MOTOR_RL);
  
  motorFL.writeMicroseconds(MOTOR_ARM_VALUE);
  motorFR.writeMicroseconds(MOTOR_ARM_VALUE);
  motorRR.writeMicroseconds(MOTOR_ARM_VALUE);
  motorRL.writeMicroseconds(MOTOR_ARM_VALUE);
  
  Serial.println(F("[OK] Motors initialized"));
  delay(100);
}

// ============================================
// RADIO INITIALIZATION
// ============================================
bool initRadio() {
  if (!radio.begin()) {
    Serial.println(F("[ERROR] Radio hardware not responding!"));
    return false;
  }
  
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  
  radio.openReadingPipe(1, pipeIn);
  radio.openWritingPipe(pipeOut);
  radio.startListening();
  
  Serial.println(F("[OK] Radio initialized"));
  return true;
}

// ============================================
// MPU6050 INITIALIZATION
// ============================================
bool initMPU6050() {
  Wire.begin();
  Wire.setClock(400000);
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("[ERROR] MPU6050 not found!"));
    return false;
  }
  
  // Configure gyro (±500°/s)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_GYRO_CONFIG);
  Wire.write(0x08);
  Wire.endTransmission();
  
  // Configure accelerometer (±8g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_CONFIG);
  Wire.write(0x10);
  Wire.endTransmission();
  
  Serial.println(F("[OK] MPU6050 initialized"));
  return true;
}

// ============================================
// SENSOR CALIBRATION
// ============================================
void calibrateSensors() {
  Serial.println(F("Calibrating sensors..."));
  Serial.println(F("Keep drone level and still!"));
  
  beep(1, 500);
  digitalWrite(LED_PIN, LOW);
  
  float gyroRollSum = 0, gyroPitchSum = 0, gyroYawSum = 0;
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;
  
  for (int i = 0; i < 2000; i++) {
    readIMURaw();
    
    gyroRollSum += gyroX;
    gyroPitchSum += gyroY;
    gyroYawSum += gyroZ;
    accelXSum += accelX;
    accelYSum += accelY;
    accelZSum += accelZ;
    
    if (i % 100 == 0) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    
    delay(3);
  }
  
  gyroRollCal = gyroRollSum / 2000.0;
  gyroPitchCal = gyroPitchSum / 2000.0;
  gyroYawCal = gyroYawSum / 2000.0;
  accelXCal = accelXSum / 2000.0;
  accelYCal = accelYSum / 2000.0;
  accelZCal = (accelZSum / 2000.0) - 4096.0;  // Gravity offset
  
  calibrated = true;
  digitalWrite(LED_PIN, HIGH);
  beep(2, 100);
  
  Serial.println(F("[OK] Calibration complete!"));
  Serial.print(F("Gyro Cal: "));
  Serial.print(gyroRollCal); Serial.print(F(", "));
  Serial.print(gyroPitchCal); Serial.print(F(", "));
  Serial.println(gyroYawCal);
}

// ============================================
// READ IMU RAW DATA
// ============================================
void readIMURaw() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  accelX = (Wire.read() << 8 | Wire.read());
  accelY = (Wire.read() << 8 | Wire.read());
  accelZ = (Wire.read() << 8 | Wire.read());
  int16_t temp = (Wire.read() << 8 | Wire.read());
  gyroX = (Wire.read() << 8 | Wire.read());
  gyroY = (Wire.read() << 8 | Wire.read());
  gyroZ = (Wire.read() << 8 | Wire.read());
}

// ============================================
// READ AND PROCESS IMU DATA
// ============================================
void readIMU() {
  readIMURaw();
  
  // Apply calibration
  gyroX -= gyroRollCal;
  gyroY -= gyroPitchCal;
  gyroZ -= gyroYawCal;
  accelX -= accelXCal;
  accelY -= accelYCal;
  accelZ -= accelZCal;
  
  // Convert to degrees per second (65.5 LSB/°/s for ±500°/s)
  gyroRollInput = gyroX / 65.5;
  gyroPitchInput = gyroY / 65.5;
  gyroYawInput = gyroZ / 65.5;
}

// ============================================
// CALCULATE ANGLES
// ============================================
void calculateAngles() {
  // Calculate angles from accelerometer
  float accelRoll = atan2(accelY, accelZ) * 57.2958;  // Convert to degrees
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958;
  
  // Simplified complementary filter (stronger accel trust for better stability)
  // This is what made TestStabilization work better!
  if (angleRoll == 0 && anglePitch == 0) {
    // First reading, initialize with accel
    angleRoll = accelRoll;
    anglePitch = accelPitch;
  } else {
    // Integrate gyro (4ms loop time = 0.004 seconds)
    angleRoll += gyroRollInput * 0.004;
    anglePitch += gyroPitchInput * 0.004;
    
    // Apply complementary filter (95% gyro, 5% accel for stronger correction)
    angleRoll = angleRoll * 0.95 + accelRoll * 0.05;
    anglePitch = anglePitch * 0.95 + accelPitch * 0.05;
  }
  
  // Yaw tracking (gyro only)
  angleYaw += gyroYawInput * 0.004;
  
  // Keep yaw within -180 to +180
  if (angleYaw > 180) angleYaw -= 360;
  if (angleYaw < -180) angleYaw += 360;
}

// ============================================
// CALCULATE PID
// ============================================
void calculatePID() {
  // Calculate setpoints based on flight mode
  if (flightMode == MODE_ANGLE) {
    // ANGLE MODE: Auto-level, stick input = desired angle
    pidRollSetpoint = rxData.roll / 10.0;   // Max ±50 degrees
    pidPitchSetpoint = rxData.pitch / 10.0;
    pidYawSetpoint = rxData.yaw / 2.0;      // Yaw rate
  } else {
    // ACRO MODE: Manual rate control, stick input = rotation rate
    pidRollSetpoint = rxData.roll / 5.0;    // Max ±100 deg/s
    pidPitchSetpoint = rxData.pitch / 5.0;
    pidYawSetpoint = rxData.yaw / 2.0;
  }
  
  // ========================================
  // ROLL PID - IMPROVED METHOD
  // ========================================
  float rollError;
  if (flightMode == MODE_ANGLE) {
    // Angle mode: Error is angle difference
    rollError = pidRollSetpoint - angleRoll;
  } else {
    // Acro mode: Error is rate difference
    rollError = pidRollSetpoint - gyroRollInput;
  }
  
  // Integral term (anti-windup)
  pidRollI += PID_ROLL_KI * rollError;
  pidRollI = constrain(pidRollI, -PID_LIMIT / 2, PID_LIMIT / 2);
  
  // Derivative term - IMPROVED: use gyro rate directly for smoother response
  // This is what TestStabilization did that worked better!
  float rollDerivative;
  if (flightMode == MODE_ANGLE) {
    // In angle mode, D term fights rotation (use negative gyro rate)
    rollDerivative = -gyroRollInput;
  } else {
    // In acro mode, D term dampens error change
    rollDerivative = rollError - pidRollPrev;
  }
  pidRollD = PID_ROLL_KD * rollDerivative;
  
  // Combine PID terms
  pidRoll = (PID_ROLL_KP * rollError) + pidRollI + pidRollD;
  pidRoll = constrain(pidRoll, -PID_LIMIT, PID_LIMIT);
  pidRollPrev = rollError;
  
  // ========================================
  // PITCH PID - IMPROVED METHOD
  // ========================================
  float pitchError;
  if (flightMode == MODE_ANGLE) {
    // Angle mode: Error is angle difference
    pitchError = pidPitchSetpoint - anglePitch;
  } else {
    // Acro mode: Error is rate difference
    pitchError = pidPitchSetpoint - gyroPitchInput;
  }
  
  // Integral term (anti-windup)
  pidPitchI += PID_PITCH_KI * pitchError;
  pidPitchI = constrain(pidPitchI, -PID_LIMIT / 2, PID_LIMIT / 2);
  
  // Derivative term - IMPROVED: use gyro rate directly
  float pitchDerivative;
  if (flightMode == MODE_ANGLE) {
    // In angle mode, D term fights rotation (use negative gyro rate)
    pitchDerivative = -gyroPitchInput;
  } else {
    // In acro mode, D term dampens error change
    pitchDerivative = pitchError - pidPitchPrev;
  }
  pidPitchD = PID_PITCH_KD * pitchDerivative;
  
  // Combine PID terms
  pidPitch = (PID_PITCH_KP * pitchError) + pidPitchI + pidPitchD;
  pidPitch = constrain(pidPitch, -PID_LIMIT, PID_LIMIT);
  pidPitchPrev = pitchError;
  
  // ========================================
  // YAW PID (Rate control only)
  // ========================================
  float yawError = pidYawSetpoint - gyroYawInput;
  
  pidYawI += PID_YAW_KI * yawError;
  pidYawI = constrain(pidYawI, -PID_LIMIT / 2, PID_LIMIT / 2);
  
  pidYawD = PID_YAW_KD * (yawError - pidYawPrev);
  
  pidYaw = (PID_YAW_KP * yawError) + pidYawI + pidYawD;
  pidYaw = constrain(pidYaw, -PID_LIMIT, PID_LIMIT);
  pidYawPrev = yawError;
}

// ============================================
// MOTOR MIXING
// ============================================
void mixMotors() {
  int throttle = rxData.throttle;
  
  // Ensure minimum throttle for stabilization to work
  // If throttle is too low, set to minimum hover threshold
  if (throttle < 1100) {
    throttle = 1100;  // Minimum for stabilization testing
  }
  
  // Quadcopter X configuration
  // Front Left (CCW): -Pitch +Roll -Yaw
  motorFLSpeed = throttle - pidPitch + pidRoll - pidYaw;
  
  // Front Right (CW): -Pitch -Roll +Yaw
  motorFRSpeed = throttle - pidPitch - pidRoll + pidYaw;
  
  // Rear Right (CCW): +Pitch -Roll -Yaw
  motorRRSpeed = throttle + pidPitch - pidRoll - pidYaw;
  
  // Rear Left (CW): +Pitch +Roll +Yaw
  motorRLSpeed = throttle + pidPitch + pidRoll + pidYaw;
  
  // Constrain motor speeds
  motorFLSpeed = constrain(motorFLSpeed, MOTOR_MIN, MOTOR_MAX);
  motorFRSpeed = constrain(motorFRSpeed, MOTOR_MIN, MOTOR_MAX);
  motorRRSpeed = constrain(motorRRSpeed, MOTOR_MIN, MOTOR_MAX);
  motorRLSpeed = constrain(motorRLSpeed, MOTOR_MIN, MOTOR_MAX);
}

// ============================================
// UPDATE MOTORS
// ============================================
void updateMotors() {
  motorFL.writeMicroseconds(motorFLSpeed);
  motorFR.writeMicroseconds(motorFRSpeed);
  motorRR.writeMicroseconds(motorRRSpeed);
  motorRL.writeMicroseconds(motorRLSpeed);
}

// ============================================
// STOP MOTORS
// ============================================
void stopMotors() {
  motorFLSpeed = MOTOR_ARM_VALUE;
  motorFRSpeed = MOTOR_ARM_VALUE;
  motorRRSpeed = MOTOR_ARM_VALUE;
  motorRLSpeed = MOTOR_ARM_VALUE;
}

// ============================================
// RESET PID
// ============================================
void resetPID() {
  pidRollI = 0;
  pidPitchI = 0;
  pidYawI = 0;
  pidRollPrev = 0;
  pidPitchPrev = 0;
  pidYawPrev = 0;
  angleRoll = 0;
  anglePitch = 0;
  angleYaw = 0;
}

// ============================================
// RECEIVE RADIO DATA
// ============================================
void receiveRadioData() {
  if (radio.available()) {
    radio.read(&rxData, sizeof(ControlData));
    
    // Verify checksum
    uint8_t calcChecksum = (rxData.throttle + rxData.roll + rxData.pitch + rxData.yaw) & 0xFF;
    if (calcChecksum == rxData.checksum) {
      lastReceiveTime = millis();
    }
  } else {
    // If no data received yet, initialize with safe values
    if (lastReceiveTime == 0) {
      rxData.throttle = 1000;
      rxData.roll = 0;
      rxData.pitch = 0;
      rxData.yaw = 0;
      rxData.switches = 0;
      rxData.buttons = 0;
    }
  }
}

// ============================================
// PROCESS BUTTONS
// ============================================
void processButtons() {
  // Button 1 (D4 on remote): CALIBRATION
  // Press to trigger sensor calibration from remote control
  bool button1 = (rxData.buttons & 0x01);
  if (button1 && !lastButton1 && !armed) {
    Serial.println(F("BTN1: Starting sensor calibration from RC..."));
    beep(1, 200);
    calibrateSensors();
    Serial.println(F("BTN1: Calibration complete!"));
  }
  lastButton1 = button1;
  
  // Button 2 (D5 on remote): MOTOR TEST
  // Hold to spin motors and check direction/operation
  bool button2 = (rxData.buttons & 0x02);
  if (button2 && !armed) {
    if (!motorTestActive) {
      motorTestActive = true;
      motorTestSpeed = 1000;
      Serial.println(F("BTN2: Motor test - CHECK DIRECTIONS!"));
      beep(1, 100);
    }
    // Gradually increase speed while held
    if (motorTestSpeed < 1400) {
      motorTestSpeed += 2;  // Slow ramp up
    }
  } else {
    if (motorTestActive) {
      motorTestActive = false;
      motorTestSpeed = 1000;
      stopMotors();
      Serial.println(F("BTN2: Motor test STOP"));
      beep(1, 100);
    }
  }
  lastButton2 = button2;
  
  // Button 3 (D6 on remote): ARM FOR FLIGHT
  // Press to make drone ready to fly (ARM)
  bool button3 = (rxData.buttons & 0x04);
  if (button3 && !lastButton3) {
    if (!armed && rxData.throttle < 1050) {
      // ARM the drone
      armed = true;
      buzzerActive = false;
      digitalWrite(BUZZER_PIN, LOW);
      beep(1, 200);
      Serial.println(F("BTN3: ARMED - Ready to fly! Use joysticks!"));
      Serial.println(F("      MPU6050 stabilization ACTIVE"));
    } else if (armed) {
      // DISARM the drone
      armed = false;
      resetPID();
      beep(2, 100);
      Serial.println(F("BTN3: DISARMED - Safe"));
    } else if (rxData.throttle >= 1050) {
      Serial.println(F("BTN3: Cannot ARM - Throttle too high!"));
      beep(3, 100);
    }
  }
  lastButton3 = button3;
  
  // Button 4 (D7 on remote): SOFT LANDING MODE
  // Toggle soft landing (gradual throttle reduction)
  bool button4 = (rxData.buttons & 0x08);
  static bool lastButton4 = false;
  static bool softLandingActive = false;
  static unsigned long landingStartTime = 0;
  
  if (button4 && !lastButton4 && armed) {
    softLandingActive = !softLandingActive;
    if (softLandingActive) {
      landingStartTime = millis();
      Serial.println(F("BTN4: SOFT LANDING MODE - Throttle reducing slowly"));
      beep(1, 150);
    } else {
      Serial.println(F("BTN4: Soft landing cancelled"));
      beep(2, 100);
    }
  }
  lastButton4 = button4;
  
  // Execute soft landing if active
  if (softLandingActive && armed) {
    unsigned long landingTime = millis() - landingStartTime;
    // Reduce throttle over 5 seconds
    if (landingTime < 5000) {
      int reduction = map(landingTime, 0, 5000, 0, 300);
      rxData.throttle = max(1000, rxData.throttle - reduction);
    } else {
      // After 5 seconds, auto-disarm
      softLandingActive = false;
      armed = false;
      resetPID();
      Serial.println(F("BTN4: Soft landing complete - DISARMED"));
      beep(3, 100);
    }
  }
}

// ============================================
// MOTOR TEST MODE
// ============================================
void runMotorTest() {
  // Spin all motors at same speed for testing
  motorFLSpeed = motorTestSpeed;
  motorFRSpeed = motorTestSpeed;
  motorRRSpeed = motorTestSpeed;
  motorRLSpeed = motorTestSpeed;
}

// ============================================
// BUZZER CONTROL
// ============================================
void processBuzzer() {
  if (buzzerActive) {
    // Beep pattern: 100ms on, 100ms off
    unsigned long elapsed = millis() - buzzerStartTime;
    if ((elapsed % 200) < 100) {
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}

// ============================================
// PROCESS FLIGHT MODE
// ============================================
void processFlightMode() {
  // Don't allow arming in motor test mode
  if (motorTestActive) {
    armed = false;
    return;
  }
  
  // Check arming switch (SW1)
  bool armSwitch = (rxData.switches & 0x01);
  
  if (armSwitch && !armed && rxData.throttle < 1050) {
    // Arm the drone
    armed = true;
    buzzerActive = false;  // Turn off buzzer when arming
    digitalWrite(BUZZER_PIN, LOW);
    beep(1, 200);
    Serial.println(F("ARMED"));
  } else if (!armSwitch && armed) {
    // Disarm the drone
    armed = false;
    resetPID();
    beep(2, 100);
    Serial.println(F("DISARMED"));
  }
  
  // Flight mode selection (SW2)
  // SW2 ON = Angle Mode (auto-level)
  // SW2 OFF = Acro Mode (manual rate control)
  if (armed) {
    if (rxData.switches & 0x02) {
      flightMode = MODE_ANGLE;
    } else {
      flightMode = MODE_ACRO;
    }
  }
  
  telemetry.armed = armed;
  telemetry.flightMode = flightMode;
}

// ============================================
// FAILSAFE CHECK
// ============================================
void checkFailsafe() {
  if (millis() - lastReceiveTime > FAILSAFE_TIMEOUT) {
    if (armed) {
      armed = false;
      stopMotors();
      resetPID();
      Serial.println(F("FAILSAFE TRIGGERED!"));
      beep(3, 100);
    }
  }
}

// ============================================
// STATUS LED
// ============================================
void updateStatusLED() {
  static unsigned long ledTimer = 0;
  static bool ledState = false;
  
  if (armed) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    if (millis() - ledTimer > 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      ledTimer = millis();
    }
  }
}

// ============================================
// BUZZER FUNCTIONS
// ============================================
void beep(int count, int duration) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) delay(duration);
  }
}

void errorBlink(int code) {
  while (true) {
    for (int i = 0; i < code; i++) {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      delay(200);
    }
    delay(1000);
  }
}

// ============================================
// DEBUG OUTPUT
// ============================================
void printDebugInfo() {
  static unsigned long lastPrint = 0;
  
  if (millis() - lastPrint > 100) {  // Print every 100ms
    // Status
    if (motorTestActive) {
      Serial.print(F("MOTOR_TEST "));
    } else if (armed) {
      Serial.print(F("ARMED "));
    } else {
      Serial.print(F("DISARM "));
    }
    
    // Flight mode
    Serial.print(F("| Mode:"));
    if (flightMode == MODE_ANGLE) {
      Serial.print(F("ANGLE"));
    } else if (flightMode == MODE_ACRO) {
      Serial.print(F("ACRO"));
    } else {
      Serial.print(F("---"));
    }
    
    // Angles
    Serial.print(F(" | Ang R:"));
    Serial.print(angleRoll, 1);
    Serial.print(F(" P:"));
    Serial.print(anglePitch, 1);
    
    // Gyro rates
    Serial.print(F(" | Gyro R:"));
    Serial.print(gyroRollInput, 1);
    Serial.print(F(" P:"));
    Serial.print(gyroPitchInput, 1);
    
    // PID outputs
    Serial.print(F(" | PID R:"));
    Serial.print(pidRoll, 0);
    Serial.print(F(" P:"));
    Serial.print(pidPitch, 0);
    
    // Motors
    Serial.print(F(" | Mot FL:"));
    Serial.print(motorFLSpeed);
    Serial.print(F(" FR:"));
    Serial.print(motorFRSpeed);
    Serial.print(F(" RR:"));
    Serial.print(motorRRSpeed);
    Serial.print(F(" RL:"));
    Serial.print(motorRLSpeed);
    
    // Buzzer status
    if (buzzerActive) {
      Serial.print(F(" | BUZZ:ON"));
    }
    
    Serial.println();
    lastPrint = millis();
  }
}
