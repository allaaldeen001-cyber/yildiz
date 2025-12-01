/*
 * Professional Quadcopter Flight Controller v2.3
 * DIRECT MPU → MOTOR RESPONSE (SIMPLIFIED)
 * 
 * This version uses DIRECT motor mixing that MUST work!
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

// Flight modes
#define MODE_DISARMED 0
#define MODE_ARMED 1
#define MODE_ANGLE 2
#define MODE_ACRO 3
#define MODE_MOTOR_TEST 4

// PID Configuration - SIMPLIFIED FOR TESTING
#define PID_KP 1.0      // Start conservative
#define PID_KD 12.0     // Strong damping

#define PID_LIMIT 300.0

// Loop timing
#define MAIN_LOOP_TIME 4000  // 4ms = 250Hz
#define FAILSAFE_TIMEOUT 1000

// ============================================
// COMMUNICATION STRUCTURE
// ============================================
struct ControlData {
  int16_t throttle;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  uint8_t switches;
  uint8_t buttons;
  uint8_t checksum;
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
float angleRoll = 0, anglePitch = 0;
float gyroRollRate, gyroPitchRate, gyroYawRate;

// Calibration offsets
float gyroRollCal = 0, gyroPitchCal = 0, gyroYawCal = 0;

// PID variables
float pidRoll = 0, pidPitch = 0, pidYaw = 0;

// Motor outputs
int motorFLSpeed, motorFRSpeed, motorRRSpeed, motorRLSpeed;

// System state
uint8_t flightMode = MODE_DISARMED;
bool armed = false;
bool calibrated = false;
unsigned long lastReceiveTime = 0;
unsigned long loopTimer;

// Button states
bool lastButton1 = false;
bool lastButton2 = false;
bool lastButton3 = false;
bool motorTestActive = false;
int motorTestSpeed = 1000;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MPU_INT_PIN, INPUT);
  
  digitalWrite(LED_PIN, LOW);
  
  Serial.println(F("\n\n╔═══════════════════════════════════════╗"));
  Serial.println(F("║  FLIGHT CONTROLLER v2.3 - FIXED      ║"));
  Serial.println(F("║  DIRECT MPU → MOTOR RESPONSE         ║"));
  Serial.println(F("╚═══════════════════════════════════════╝\n"));
  
  Serial.println(F("Button Functions:"));
  Serial.println(F("  BTN1 (D4) - Calibrate sensors"));
  Serial.println(F("  BTN2 (D5) - Motor test"));
  Serial.println(F("  BTN3 (D6) - ARM for flight"));
  Serial.println(F("  BTN4 (D7) - Not used"));
  Serial.println();
  
  // Initialize motors
  initMotors();
  
  // Initialize radio
  if (!initRadio()) {
    Serial.println(F("❌ Radio init failed!"));
    errorBlink(3);
  }
  
  // Initialize MPU6050
  if (!initMPU6050()) {
    Serial.println(F("❌ MPU6050 init failed!"));
    errorBlink(4);
  }
  
  // Calibrate sensors
  calibrateSensors();
  
  Serial.println(F("\n✅ System ready! Waiting for RC commands...\n"));
  
  // Initialize loop timer
  loopTimer = micros();
}

void loop() {
  // Maintain 250Hz loop (4ms)
  while (micros() - loopTimer < MAIN_LOOP_TIME);
  loopTimer = micros();
  
  // Read radio data
  receiveRadioData();
  
  // Process buttons
  processButtons();
  
  // Read IMU
  readIMU();
  calculateAngles();
  
  // Flight mode and arming
  processFlightMode();
  
  // Control logic
  if (motorTestActive) {
    runMotorTest();
  } else if (armed) {
    calculatePID();
    mixMotors();
  } else {
    stopMotors();
    resetPID();
  }
  
  // Update motors
  updateMotors();
  
  // Debug output (every 100ms)
  printDebugInfo();
  
  // Send telemetry
  sendTelemetry();
}

// ============================================
// INITIALIZATION FUNCTIONS
// ============================================
void initMotors() {
  motorFL.attach(MOTOR_FL);
  motorFR.attach(MOTOR_FR);
  motorRR.attach(MOTOR_RR);
  motorRL.attach(MOTOR_RL);
  
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorRR.writeMicroseconds(1000);
  motorRL.writeMicroseconds(1000);
  
  delay(1000);
  Serial.println(F("✅ Motors initialized"));
}

bool initRadio() {
  if (!radio.begin()) {
    return false;
  }
  
  radio.openReadingPipe(1, pipeIn);
  radio.openWritingPipe(pipeOut);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.startListening();
  
  Serial.println(F("✅ Radio initialized"));
  return true;
}

bool initMPU6050() {
  Wire.begin();
  Wire.setClock(400000);
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  
  // Configure gyro: ±500°/s
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_GYRO_CONFIG);
  Wire.write(0x08);
  Wire.endTransmission();
  
  // Configure accel: ±8g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_CONFIG);
  Wire.write(0x10);
  Wire.endTransmission();
  
  Serial.println(F("✅ MPU6050 initialized"));
  return true;
}

void calibrateSensors() {
  Serial.println(F("\n⏳ CALIBRATING... Keep drone LEVEL and STILL!"));
  beep(1, 200);
  
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  
  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    
    int16_t gx = (Wire.read() << 8 | Wire.read());
    int16_t gy = (Wire.read() << 8 | Wire.read());
    int16_t gz = (Wire.read() << 8 | Wire.read());
    
    gyroXSum += gx;
    gyroYSum += gy;
    gyroZSum += gz;
    
    delay(3);
  }
  
  gyroRollCal = gyroXSum / 1000.0;
  gyroPitchCal = gyroYSum / 1000.0;
  gyroYawCal = gyroZSum / 1000.0;
  
  calibrated = true;
  
  Serial.println(F("✅ CALIBRATION COMPLETE!"));
  Serial.print(F("   Gyro offsets: X="));
  Serial.print(gyroRollCal, 1);
  Serial.print(F(" Y="));
  Serial.print(gyroPitchCal, 1);
  Serial.print(F(" Z="));
  Serial.println(gyroYawCal, 1);
  
  beep(2, 100);
}

// ============================================
// IMU FUNCTIONS
// ============================================
void readIMU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  accelX = (int16_t)(Wire.read() << 8 | Wire.read());
  accelY = (int16_t)(Wire.read() << 8 | Wire.read());
  accelZ = (int16_t)(Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read();  // Skip temp
  gyroX = (int16_t)(Wire.read() << 8 | Wire.read());
  gyroY = (int16_t)(Wire.read() << 8 | Wire.read());
  gyroZ = (int16_t)(Wire.read() << 8 | Wire.read());
  
  // Apply calibration
  gyroX -= gyroRollCal;
  gyroY -= gyroPitchCal;
  gyroZ -= gyroYawCal;
  
  // Convert to deg/s (65.5 LSB per deg/s at ±500°/s)
  gyroRollRate = gyroX / 65.5;
  gyroPitchRate = gyroY / 65.5;
  gyroYawRate = gyroZ / 65.5;
}

void calculateAngles() {
  // Calculate angles from accelerometer
  float accelRoll = atan2(accelY, accelZ) * 57.2958;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958;
  
  // Simple complementary filter
  angleRoll = accelRoll * 0.05 + (angleRoll + gyroRollRate * 0.004) * 0.95;
  anglePitch = accelPitch * 0.05 + (anglePitch + gyroPitchRate * 0.004) * 0.95;
}

// ============================================
// PID CALCULATION - SIMPLIFIED AND DIRECT
// ============================================
void calculatePID() {
  // Target angle = 0 (level)
  // Error = target - current
  float rollError = 0 - angleRoll;
  float pitchError = 0 - anglePitch;
  
  // Simple PD controller
  // P term: Corrects angle error
  // D term: Dampens rotation (opposes gyro rate)
  pidRoll = (PID_KP * rollError) + (PID_KD * (-gyroRollRate / 65.5));
  pidPitch = (PID_KP * pitchError) + (PID_KD * (-gyroPitchRate / 65.5));
  
  // Yaw uses stick input (rate control)
  float yawTarget = rxData.yaw / 2.0;
  pidYaw = 2.0 * (yawTarget - gyroYawRate);
  
  // Limit outputs
  pidRoll = constrain(pidRoll, -PID_LIMIT, PID_LIMIT);
  pidPitch = constrain(pidPitch, -PID_LIMIT, PID_LIMIT);
  pidYaw = constrain(pidYaw, -PID_LIMIT, PID_LIMIT);
}

// ============================================
// MOTOR MIXING - TRY CONFIGURATION 1 FIRST
// ============================================
void mixMotors() {
  int throttle = rxData.throttle;
  
  // Ensure minimum throttle
  if (throttle < 1150) {
    throttle = 1150;
  }
  
  // ╔═══════════════════════════════════════════════╗
  // ║  MOTOR MIXING CONFIGURATION 1 (Try this first)║
  // ╚═══════════════════════════════════════════════╝
  //
  // X-Configuration:
  //     FRONT
  //   FL     FR
  //     \ X /
  //     / X \
  //   RL     RR
  //     REAR
  //
  // When NOSE DOWN (pitch negative):
  //   - We want REAR motors to speed UP
  //   - We want FRONT motors to slow DOWN
  //   - So: Rear gets +pidPitch, Front gets -pidPitch
  //
  // When TILT RIGHT (roll positive):
  //   - We want LEFT motors to speed UP
  //   - We want RIGHT motors to slow DOWN
  //   - So: Left gets +pidRoll, Right gets -pidRoll
  
  motorFLSpeed = throttle - pidPitch + pidRoll - pidYaw;  // Front Left
  motorFRSpeed = throttle - pidPitch - pidRoll + pidYaw;  // Front Right
  motorRRSpeed = throttle + pidPitch - pidRoll - pidYaw;  // Rear Right
  motorRLSpeed = throttle + pidPitch + pidRoll + pidYaw;  // Rear Left
  
  // Constrain
  motorFLSpeed = constrain(motorFLSpeed, MOTOR_MIN, MOTOR_MAX);
  motorFRSpeed = constrain(motorFRSpeed, MOTOR_MIN, MOTOR_MAX);
  motorRRSpeed = constrain(motorRRSpeed, MOTOR_MIN, MOTOR_MAX);
  motorRLSpeed = constrain(motorRLSpeed, MOTOR_MIN, MOTOR_MAX);
}

/* ╔═══════════════════════════════════════════════════════════════╗
   ║  IF MOTORS RESPOND BACKWARDS, TRY THESE CONFIGURATIONS:      ║
   ╚═══════════════════════════════════════════════════════════════╝

   CONFIGURATION 2: Inverted Pitch
   ─────────────────────────────────────────────────────────────────
   motorFLSpeed = throttle + pidPitch + pidRoll - pidYaw;
   motorFRSpeed = throttle + pidPitch - pidRoll + pidYaw;
   motorRRSpeed = throttle - pidPitch - pidRoll - pidYaw;
   motorRLSpeed = throttle - pidPitch + pidRoll + pidYaw;

   CONFIGURATION 3: Inverted Roll
   ─────────────────────────────────────────────────────────────────
   motorFLSpeed = throttle - pidPitch - pidRoll - pidYaw;
   motorFRSpeed = throttle - pidPitch + pidRoll + pidYaw;
   motorRRSpeed = throttle + pidPitch + pidRoll - pidYaw;
   motorRLSpeed = throttle + pidPitch - pidRoll + pidYaw;

   CONFIGURATION 4: Both Inverted
   ─────────────────────────────────────────────────────────────────
   motorFLSpeed = throttle + pidPitch - pidRoll - pidYaw;
   motorFRSpeed = throttle + pidPitch + pidRoll + pidYaw;
   motorRRSpeed = throttle - pidPitch + pidRoll - pidYaw;
   motorRLSpeed = throttle - pidPitch - pidRoll + pidYaw;

   To try a different configuration:
   1. Comment out the current mixMotors() function
   2. Replace with the configuration you want to test
   3. Upload and test
   ╚═══════════════════════════════════════════════════════════════╝ */

void updateMotors() {
  motorFL.writeMicroseconds(motorFLSpeed);
  motorFR.writeMicroseconds(motorFRSpeed);
  motorRR.writeMicroseconds(motorRRSpeed);
  motorRL.writeMicroseconds(motorRLSpeed);
}

void stopMotors() {
  motorFLSpeed = 1000;
  motorFRSpeed = 1000;
  motorRRSpeed = 1000;
  motorRLSpeed = 1000;
}

void resetPID() {
  pidRoll = 0;
  pidPitch = 0;
  pidYaw = 0;
}

// ============================================
// RADIO FUNCTIONS
// ============================================
void receiveRadioData() {
  if (radio.available()) {
    radio.read(&rxData, sizeof(ControlData));
    lastReceiveTime = millis();
  } else {
    // Failsafe
    if (millis() - lastReceiveTime > FAILSAFE_TIMEOUT && armed) {
      armed = false;
      flightMode = MODE_DISARMED;
      Serial.println(F("⚠️ FAILSAFE! Signal lost!"));
    }
  }
}

void sendTelemetry() {
  static unsigned long lastTelemetry = 0;
  if (millis() - lastTelemetry > 100) {
    telemetry.roll = angleRoll;
    telemetry.pitch = anglePitch;
    telemetry.yaw = 0;
    telemetry.battery = 100;
    telemetry.flightMode = flightMode;
    telemetry.armed = armed ? 1 : 0;
    
    radio.stopListening();
    radio.write(&telemetry, sizeof(TelemetryData));
    radio.startListening();
    
    lastTelemetry = millis();
  }
}

// ============================================
// BUTTON PROCESSING
// ============================================
void processButtons() {
  // Button 1: Calibration
  bool button1 = (rxData.buttons & 0x01);
  if (button1 && !lastButton1 && !armed) {
    calibrateSensors();
  }
  lastButton1 = button1;
  
  // Button 2: Motor Test
  bool button2 = (rxData.buttons & 0x02);
  if (button2 && !armed) {
    if (!motorTestActive) {
      motorTestActive = true;
      motorTestSpeed = 1100;
      Serial.println(F("🔄 Motor test active (hold to increase speed)"));
    }
    motorTestSpeed = constrain(motorTestSpeed + 2, 1100, 1300);
  } else {
    if (motorTestActive) {
      motorTestActive = false;
      motorTestSpeed = 1000;
      Serial.println(F("⏹️ Motor test stopped"));
    }
  }
  lastButton2 = button2;
  
  // Button 3: ARM/DISARM
  bool button3 = (rxData.buttons & 0x04);
  if (button3 && !lastButton3) {
    if (!armed && !motorTestActive) {
      if (calibrated && rxData.throttle < 1050) {
        armed = true;
        flightMode = MODE_ANGLE;
        resetPID();
        Serial.println(F("✅ ARMED - Ready to fly!"));
        beep(1, 100);
      } else {
        Serial.println(F("⚠️ Cannot arm! Check: calibrated + throttle down"));
      }
    } else if (armed) {
      armed = false;
      flightMode = MODE_DISARMED;
      Serial.println(F("🛑 DISARMED"));
      beep(2, 100);
    }
  }
  lastButton3 = button3;
}

void processFlightMode() {
  // Simple: SW2 switches between ANGLE and ACRO
  if (armed) {
    if (rxData.switches & 0x02) {
      flightMode = MODE_ANGLE;
    } else {
      flightMode = MODE_ACRO;
    }
  }
}

void runMotorTest() {
  motorFLSpeed = motorTestSpeed;
  motorFRSpeed = motorTestSpeed;
  motorRRSpeed = motorTestSpeed;
  motorRLSpeed = motorTestSpeed;
}

// ============================================
// DEBUG OUTPUT
// ============================================
void printDebugInfo() {
  static unsigned long lastPrint = 0;
  
  if (millis() - lastPrint > 100) {
    Serial.print(armed ? F("ARM") : F("---"));
    Serial.print(F(" | Ang P:"));
    Serial.print(anglePitch, 1);
    Serial.print(F("° R:"));
    Serial.print(angleRoll, 1);
    Serial.print(F("° | Gyro P:"));
    Serial.print(gyroPitchRate, 0);
    Serial.print(F(" R:"));
    Serial.print(gyroRollRate, 0);
    Serial.print(F(" | PID P:"));
    Serial.print((int)pidPitch);
    Serial.print(F(" R:"));
    Serial.print((int)pidRoll);
    Serial.print(F(" | M: FL:"));
    Serial.print(motorFLSpeed);
    Serial.print(F(" FR:"));
    Serial.print(motorFRSpeed);
    Serial.print(F(" RR:"));
    Serial.print(motorRRSpeed);
    Serial.print(F(" RL:"));
    Serial.print(motorRLSpeed);
    
    // Show tilt direction
    if (anglePitch < -5) Serial.print(F(" [NOSE DOWN]"));
    else if (anglePitch > 5) Serial.print(F(" [NOSE UP]"));
    
    if (angleRoll < -5) Serial.print(F(" [LEFT]"));
    else if (angleRoll > 5) Serial.print(F(" [RIGHT]"));
    
    Serial.println();
    lastPrint = millis();
  }
}

// ============================================
// UTILITY FUNCTIONS
// ============================================
void beep(int count, int duration) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) delay(duration);
  }
}

void errorBlink(int count) {
  while (true) {
    for (int i = 0; i < count; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
    delay(1000);
  }
}
