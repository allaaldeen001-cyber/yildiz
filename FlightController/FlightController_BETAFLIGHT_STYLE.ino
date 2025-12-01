/*
 * ╔═══════════════════════════════════════════════════════════════╗
 * ║  BETAFLIGHT-STYLE FLIGHT CONTROLLER                          ║
 * ║  Like Mamba / SpeedyBee / Professional FC                    ║
 * ╚═══════════════════════════════════════════════════════════════╝
 * 
 * This uses the SAME stabilization method as Betaflight:
 * - Cascaded PID (Angle → Rate)
 * - Direct motor mixing
 * - Proper gyro filtering
 * - Professional defaults
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
#define BUZZER_PIN 8
#define LED_PIN 7

#define MOTOR_FL 3  // Front Left
#define MOTOR_FR 5  // Front Right
#define MOTOR_RR 6  // Rear Right
#define MOTOR_RL 9  // Rear Left

// ============================================
// MPU6050
// ============================================
#define MPU6050_ADDR 0x68

// ============================================
// BETAFLIGHT-STYLE PID GAINS
// ============================================
// These are similar to Betaflight defaults (scaled for Arduino)

// RATE PID (Inner loop - controls rotation rate)
#define RATE_P_ROLL  40      // Betaflight default ~40
#define RATE_I_ROLL  40      // Betaflight default ~40
#define RATE_D_ROLL  30      // Betaflight default ~30

#define RATE_P_PITCH 40
#define RATE_I_PITCH 40
#define RATE_D_PITCH 30

#define RATE_P_YAW   45      // Yaw needs more P
#define RATE_I_YAW   45
#define RATE_D_YAW   0       // No D on yaw

// ANGLE PID (Outer loop - controls angle in ANGLE mode)
#define ANGLE_P      5.0     // Betaflight default ~5.0
#define ANGLE_LIMIT  50.0    // Max angle in degrees

// LIMITS
#define RATE_LIMIT   500.0
#define I_LIMIT      250.0

// RC rates (like Betaflight)
#define RC_RATE      1.0
#define SUPER_RATE   0.7
#define RC_EXPO      0.0

// ============================================
// CONFIGURATION
// ============================================
#define MOTOR_MIN 1000
#define MOTOR_MAX 2000
#define FAILSAFE_TIMEOUT 1000
#define MAIN_LOOP_TIME 4000  // 4ms = 250Hz (like Betaflight)

// ============================================
// STRUCTURES
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
  float roll, pitch, yaw;
  uint8_t battery, flightMode, armed;
};

// ============================================
// GLOBALS
// ============================================
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipeIn = 0xE8E8F0F0E1LL;
const uint64_t pipeOut = 0xE8E8F0F0E2LL;

Servo motorFL, motorFR, motorRR, motorRL;

ControlData rxData;
TelemetryData telemetry;

// IMU
float gyroRate[3] = {0, 0, 0};      // [roll, pitch, yaw] in deg/s
float angle[3] = {0, 0, 0};          // [roll, pitch, yaw] in degrees
float gyroCal[3] = {0, 0, 0};

// PID
float rateSetpoint[3] = {0, 0, 0};  // Desired rotation rate
float angleSetpoint[3] = {0, 0, 0}; // Desired angle (ANGLE mode)

float rateError[3] = {0, 0, 0};
float lastRateError[3] = {0, 0, 0};
float rateErrorSum[3] = {0, 0, 0};  // Integral

float rateP[3], rateI[3], rateD[3];
float ratePID[3] = {0, 0, 0};

// Motors
int motorSpeed[4] = {1000, 1000, 1000, 1000};  // FL, FR, RR, RL

// State
bool armed = false;
bool angleMode = true;  // true = ANGLE, false = ACRO
bool motorTestActive = false;
int motorTestSpeed = 1000;
unsigned long lastReceiveTime = 0;
unsigned long loopTimer;

// Buttons
bool lastButton[4] = {false, false, false, false};

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  Serial.println(F("\n╔════════════════════════════════════════════════╗"));
  Serial.println(F("║  BETAFLIGHT-STYLE FLIGHT CONTROLLER           ║"));
  Serial.println(F("║  Like Mamba / SpeedyBee                       ║"));
  Serial.println(F("╚════════════════════════════════════════════════╝\n"));
  
  Serial.println(F("PID Configuration (Betaflight-style):"));
  Serial.print(F("  RATE: P="));
  Serial.print(RATE_P_ROLL);
  Serial.print(F(" I="));
  Serial.print(RATE_I_ROLL);
  Serial.print(F(" D="));
  Serial.println(RATE_D_ROLL);
  Serial.print(F("  ANGLE: P="));
  Serial.println(ANGLE_P);
  Serial.println();
  
  // Init motors
  motorFL.attach(MOTOR_FL);
  motorFR.attach(MOTOR_FR);
  motorRR.attach(MOTOR_RR);
  motorRL.attach(MOTOR_RL);
  
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorRR.writeMicroseconds(1000);
  motorRL.writeMicroseconds(1000);
  delay(1000);
  
  // Init radio
  if (!radio.begin()) {
    Serial.println(F("❌ Radio FAIL!"));
    while(1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
  }
  radio.openReadingPipe(1, pipeIn);
  radio.openWritingPipe(pipeOut);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.startListening();
  Serial.println(F("✅ Radio OK"));
  
  // Init MPU6050
  Wire.begin();
  Wire.setClock(400000);
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("❌ MPU6050 FAIL!"));
    while(1) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(200); }
  }
  
  // Configure MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);  // ±500°/s
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);  // ±8g
  Wire.endTransmission();
  
  // DLPF = 42Hz (like Betaflight lowpass filter)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  
  Serial.println(F("✅ MPU6050 OK (DLPF=42Hz)"));
  
  // Calibrate gyro
  calibrateGyro();
  
  Serial.println(F("\n✅ READY!"));
  Serial.println(F("   Button 3 = ARM/DISARM"));
  Serial.println(F("   SW2 = ANGLE/ACRO mode\n"));
  
  beep(2, 100);
  
  loopTimer = micros();
}

// ============================================
// MAIN LOOP (250Hz like Betaflight)
// ============================================
void loop() {
  // Maintain 250Hz (4ms loop)
  while (micros() - loopTimer < MAIN_LOOP_TIME);
  loopTimer = micros();
  
  // 1. Read RC
  receiveRC();
  
  // 2. Read IMU
  readGyro();
  calculateAngles();
  
  // 3. Process buttons
  processButtons();
  
  // 4. Calculate PID
  if (armed && !motorTestActive) {
    calculatePID_Betaflight();
    mixMotors_Betaflight();
  } else if (motorTestActive) {
    for (int i = 0; i < 4; i++) motorSpeed[i] = motorTestSpeed;
  } else {
    stopMotors();
  }
  
  // 5. Update motors
  motorFL.writeMicroseconds(motorSpeed[0]);
  motorFR.writeMicroseconds(motorSpeed[1]);
  motorRR.writeMicroseconds(motorSpeed[2]);
  motorRL.writeMicroseconds(motorSpeed[3]);
  
  // 6. Debug output
  printDebug();
  
  // 7. Telemetry
  sendTelemetry();
}

// ============================================
// CALIBRATION
// ============================================
void calibrateGyro() {
  Serial.print(F("⏳ Calibrating gyro... "));
  
  float sum[3] = {0, 0, 0};
  
  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    
    int16_t gx = (Wire.read() << 8 | Wire.read());
    int16_t gy = (Wire.read() << 8 | Wire.read());
    int16_t gz = (Wire.read() << 8 | Wire.read());
    
    sum[0] += gx;
    sum[1] += gy;
    sum[2] += gz;
    
    delay(3);
  }
  
  gyroCal[0] = sum[0] / 1000.0;
  gyroCal[1] = sum[1] / 1000.0;
  gyroCal[2] = sum[2] / 1000.0;
  
  Serial.println(F("DONE"));
}

// ============================================
// IMU
// ============================================
void readGyro() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  int16_t accelX = (Wire.read() << 8 | Wire.read());
  int16_t accelY = (Wire.read() << 8 | Wire.read());
  int16_t accelZ = (Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read();  // Skip temp
  int16_t gyroX = (Wire.read() << 8 | Wire.read());
  int16_t gyroY = (Wire.read() << 8 | Wire.read());
  int16_t gyroZ = (Wire.read() << 8 | Wire.read());
  
  // Apply calibration and convert to deg/s
  gyroRate[0] = (gyroX - gyroCal[0]) / 65.5;  // Roll rate
  gyroRate[1] = (gyroY - gyroCal[1]) / 65.5;  // Pitch rate
  gyroRate[2] = (gyroZ - gyroCal[2]) / 65.5;  // Yaw rate
  
  // Calculate angles for ANGLE mode
  float accelRoll = atan2(accelY, accelZ) * 57.2958;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958;
  
  // Complementary filter (like Betaflight)
  angle[0] = 0.98 * (angle[0] + gyroRate[0] * 0.004) + 0.02 * accelRoll;
  angle[1] = 0.98 * (angle[1] + gyroRate[1] * 0.004) + 0.02 * accelPitch;
}

void calculateAngles() {
  // Already done in readGyro() for efficiency
}

// ============================================
// BETAFLIGHT-STYLE PID
// ============================================
void calculatePID_Betaflight() {
  // ════════════════════════════════════════════
  // CASCADED PID (like Betaflight)
  // ════════════════════════════════════════════
  // 
  // In ANGLE mode:
  //   1. Outer loop: Angle → Rate (ANGLE_P)
  //   2. Inner loop: Rate PID
  // 
  // In ACRO mode:
  //   - Only inner Rate PID
  // ════════════════════════════════════════════
  
  // Convert stick inputs to rate setpoints
  if (angleMode) {
    // ANGLE MODE: Stick controls angle
    angleSetpoint[0] = (rxData.roll / 500.0) * ANGLE_LIMIT;   // ±50°
    angleSetpoint[1] = (rxData.pitch / 500.0) * ANGLE_LIMIT;
    
    // Outer loop: angle error → rate setpoint
    rateSetpoint[0] = (angleSetpoint[0] - angle[0]) * ANGLE_P;
    rateSetpoint[1] = (angleSetpoint[1] - angle[1]) * ANGLE_P;
    
    // Limit rate setpoint
    rateSetpoint[0] = constrain(rateSetpoint[0], -500, 500);
    rateSetpoint[1] = constrain(rateSetpoint[1], -500, 500);
    
  } else {
    // ACRO MODE: Stick directly controls rate
    rateSetpoint[0] = (rxData.roll / 500.0) * 500.0;   // ±500°/s
    rateSetpoint[1] = (rxData.pitch / 500.0) * 500.0;
  }
  
  // Yaw is always rate mode
  rateSetpoint[2] = (rxData.yaw / 500.0) * 400.0;  // ±400°/s
  
  // ════════════════════════════════════════════
  // INNER LOOP: RATE PID
  // ════════════════════════════════════════════
  
  float dtSec = 0.004;  // 4ms = 0.004s
  
  // ROLL
  rateError[0] = rateSetpoint[0] - gyroRate[0];
  rateErrorSum[0] += rateError[0] * dtSec;
  rateErrorSum[0] = constrain(rateErrorSum[0], -I_LIMIT, I_LIMIT);
  
  rateP[0] = RATE_P_ROLL * rateError[0];
  rateI[0] = RATE_I_ROLL * rateErrorSum[0];
  rateD[0] = RATE_D_ROLL * (rateError[0] - lastRateError[0]) / dtSec;
  
  ratePID[0] = rateP[0] + rateI[0] + rateD[0];
  ratePID[0] = constrain(ratePID[0], -RATE_LIMIT, RATE_LIMIT);
  lastRateError[0] = rateError[0];
  
  // PITCH
  rateError[1] = rateSetpoint[1] - gyroRate[1];
  rateErrorSum[1] += rateError[1] * dtSec;
  rateErrorSum[1] = constrain(rateErrorSum[1], -I_LIMIT, I_LIMIT);
  
  rateP[1] = RATE_P_PITCH * rateError[1];
  rateI[1] = RATE_I_PITCH * rateErrorSum[1];
  rateD[1] = RATE_D_PITCH * (rateError[1] - lastRateError[1]) / dtSec;
  
  ratePID[1] = rateP[1] + rateI[1] + rateD[1];
  ratePID[1] = constrain(ratePID[1], -RATE_LIMIT, RATE_LIMIT);
  lastRateError[1] = rateError[1];
  
  // YAW
  rateError[2] = rateSetpoint[2] - gyroRate[2];
  rateErrorSum[2] += rateError[2] * dtSec;
  rateErrorSum[2] = constrain(rateErrorSum[2], -I_LIMIT, I_LIMIT);
  
  rateP[2] = RATE_P_YAW * rateError[2];
  rateI[2] = RATE_I_YAW * rateErrorSum[2];
  rateD[2] = 0;  // No D on yaw
  
  ratePID[2] = rateP[2] + rateI[2];
  ratePID[2] = constrain(ratePID[2], -RATE_LIMIT, RATE_LIMIT);
  lastRateError[2] = rateError[2];
}

// ════════════════════════════════════════════
// BETAFLIGHT-STYLE MOTOR MIXING
// ════════════════════════════════════════════
void mixMotors_Betaflight() {
  int throttle = rxData.throttle;
  
  // Ensure minimum throttle
  if (throttle < 1100) throttle = 1100;
  
  // Scale PID outputs to motor range
  float rollMix = ratePID[0] * 0.5;    // Scale to ±250
  float pitchMix = ratePID[1] * 0.5;
  float yawMix = ratePID[2] * 0.5;
  
  // X-Configuration (standard)
  //     FRONT
  //   FL     FR
  //     \ X /
  //     / X \
  //   RL     RR
  //     REAR
  
  motorSpeed[0] = throttle - pitchMix + rollMix - yawMix;  // FL
  motorSpeed[1] = throttle - pitchMix - rollMix + yawMix;  // FR
  motorSpeed[2] = throttle + pitchMix - rollMix - yawMix;  // RR
  motorSpeed[3] = throttle + pitchMix + rollMix + yawMix;  // RL
  
  // Constrain
  for (int i = 0; i < 4; i++) {
    motorSpeed[i] = constrain(motorSpeed[i], MOTOR_MIN, MOTOR_MAX);
  }
}

void stopMotors() {
  for (int i = 0; i < 4; i++) {
    motorSpeed[i] = 1000;
  }
  // Reset integrators
  for (int i = 0; i < 3; i++) {
    rateErrorSum[i] = 0;
  }
}

// ============================================
// RC & BUTTONS
// ============================================
void receiveRC() {
  if (radio.available()) {
    radio.read(&rxData, sizeof(ControlData));
    lastReceiveTime = millis();
  } else {
    // Failsafe
    if (millis() - lastReceiveTime > FAILSAFE_TIMEOUT && armed) {
      armed = false;
      Serial.println(F("⚠️ FAILSAFE!"));
    }
  }
}

void processButtons() {
  // Button 1: Recalibrate
  bool btn1 = (rxData.buttons & 0x01);
  if (btn1 && !lastButton[0] && !armed) {
    calibrateGyro();
  }
  lastButton[0] = btn1;
  
  // Button 2: Motor test
  bool btn2 = (rxData.buttons & 0x02);
  if (btn2 && !armed) {
    motorTestActive = true;
    motorTestSpeed = constrain(motorTestSpeed + 2, 1100, 1300);
  } else {
    if (motorTestActive) {
      motorTestActive = false;
      motorTestSpeed = 1000;
    }
  }
  lastButton[1] = btn2;
  
  // Button 3: ARM/DISARM
  bool btn3 = (rxData.buttons & 0x04);
  if (btn3 && !lastButton[2]) {
    if (!armed && rxData.throttle < 1050) {
      armed = true;
      Serial.println(F("✅ ARMED"));
      beep(1, 100);
    } else if (armed) {
      armed = false;
      Serial.println(F("🛑 DISARMED"));
      beep(2, 100);
    }
  }
  lastButton[2] = btn3;
  
  // SW2: ANGLE/ACRO mode
  angleMode = (rxData.switches & 0x02);
}

// ============================================
// DEBUG
// ============================================
void printDebug() {
  static unsigned long lastPrint = 0;
  
  if (millis() - lastPrint > 100) {
    Serial.print(armed ? F("ARM ") : F("--- "));
    Serial.print(angleMode ? F("ANG") : F("ACR"));
    Serial.print(F(" | Ang R:"));
    Serial.print(angle[0], 1);
    Serial.print(F(" P:"));
    Serial.print(angle[1], 1);
    Serial.print(F(" | Rate R:"));
    Serial.print((int)gyroRate[0]);
    Serial.print(F(" P:"));
    Serial.print((int)gyroRate[1]);
    Serial.print(F(" | PID R:"));
    Serial.print((int)ratePID[0]);
    Serial.print(F(" P:"));
    Serial.print((int)ratePID[1]);
    Serial.print(F(" | M:"));
    Serial.print(motorSpeed[0]);
    Serial.print(F(" "));
    Serial.print(motorSpeed[1]);
    Serial.print(F(" "));
    Serial.print(motorSpeed[2]);
    Serial.print(F(" "));
    Serial.print(motorSpeed[3]);
    
    if (angle[1] < -5) Serial.print(F(" [▼DOWN]"));
    else if (angle[1] > 5) Serial.print(F(" [▲UP]"));
    if (angle[0] < -5) Serial.print(F(" [◄LEFT]"));
    else if (angle[0] > 5) Serial.print(F(" [►RIGHT]"));
    
    Serial.println();
    lastPrint = millis();
  }
}

// ============================================
// TELEMETRY
// ============================================
void sendTelemetry() {
  static unsigned long lastTx = 0;
  if (millis() - lastTx > 100) {
    telemetry.roll = angle[0];
    telemetry.pitch = angle[1];
    telemetry.yaw = 0;
    telemetry.battery = 100;
    telemetry.flightMode = angleMode ? 2 : 3;
    telemetry.armed = armed ? 1 : 0;
    
    radio.stopListening();
    radio.write(&telemetry, sizeof(TelemetryData));
    radio.startListening();
    
    lastTx = millis();
  }
}

// ============================================
// UTILS
// ============================================
void beep(int count, int duration) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) delay(duration);
  }
}
