/*
 * ╔═══════════════════════════════════════════════════════════════╗
 * ║  BETAFLIGHT-STYLE FC WITH ALTITUDE CONTROL (MS5611)          ║
 * ║  Professional quad with barometric altitude hold             ║
 * ╚═══════════════════════════════════════════════════════════════╝
 * 
 * Features:
 * - Betaflight cascaded PID stabilization
 * - MS5611 barometer for altitude sensing
 * - Altitude Hold mode (SW1/D2)
 * - Smooth Takeoff (Button 4/D7)
 * - Smooth Landing (Button 3/D6)
 * - Ground level calibration
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
// SENSOR ADDRESSES
// ============================================
#define MPU6050_ADDR 0x68
#define MS5611_ADDR  0x77  // MS5611 barometer

// ============================================
// MS5611 COMMANDS
// ============================================
#define MS5611_CMD_RESET    0x1E
#define MS5611_CMD_CONV_D1  0x48  // Pressure conversion OSR=4096
#define MS5611_CMD_CONV_D2  0x58  // Temperature conversion OSR=4096
#define MS5611_CMD_ADC_READ 0x00
#define MS5611_CMD_PROM     0xA0  // PROM read base address

// ============================================
// BETAFLIGHT-STYLE PID GAINS
// ============================================
// Rate PID (inner loop)
#define RATE_P_ROLL  40
#define RATE_I_ROLL  40
#define RATE_D_ROLL  30

#define RATE_P_PITCH 40
#define RATE_I_PITCH 40
#define RATE_D_PITCH 30

#define RATE_P_YAW   45
#define RATE_I_YAW   45
#define RATE_D_YAW   0

// Angle PID (outer loop)
#define ANGLE_P      5.0
#define ANGLE_LIMIT  50.0

// ALTITUDE PID (for altitude hold)
#define ALT_P        50.0     // Strong P for altitude hold
#define ALT_I        10.0     // Integral to eliminate steady error
#define ALT_D        30.0     // Damping for smooth altitude changes
#define ALT_I_LIMIT  200.0

// Limits
#define RATE_LIMIT   500.0
#define I_LIMIT      250.0

// ============================================
// CONFIGURATION
// ============================================
#define MOTOR_MIN 1000
#define MOTOR_MAX 2000
#define FAILSAFE_TIMEOUT 1000
#define MAIN_LOOP_TIME 4000  // 4ms = 250Hz

// Altitude control
#define TAKEOFF_ALTITUDE 150.0   // 150cm = 1.5m takeoff height
#define LANDING_RATE     50.0    // 50cm/s descent rate
#define TAKEOFF_RATE     80.0    // 80cm/s ascent rate

// ============================================
// FLIGHT MODES
// ============================================
#define MODE_ANGLE    0  // Normal stabilized flight
#define MODE_ACRO     1  // Acrobatic mode
#define MODE_ALT_HOLD 2  // Altitude hold mode
#define MODE_TAKEOFF  3  // Smooth takeoff
#define MODE_LANDING  4  // Smooth landing

// ============================================
// STRUCTURES
// ============================================
struct ControlData {
  int16_t throttle;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  uint8_t switches;   // Bit 0: SW1(D2/Alt Hold), Bit 1: SW2(D3/Angle-Acro)
  uint8_t buttons;    // Bit 0-3: Buttons 1-4
  uint8_t checksum;
};

struct TelemetryData {
  float roll, pitch, yaw;
  float altitude;
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
float gyroRate[3] = {0, 0, 0};
float angle[3] = {0, 0, 0};
float gyroCal[3] = {0, 0, 0};

// PID - Attitude
float rateSetpoint[3] = {0, 0, 0};
float angleSetpoint[3] = {0, 0, 0};
float rateError[3] = {0, 0, 0};
float lastRateError[3] = {0, 0, 0};
float rateErrorSum[3] = {0, 0, 0};
float rateP[3], rateI[3], rateD[3];
float ratePID[3] = {0, 0, 0};

// PID - Altitude
float altitudeSetpoint = 0;
float altitudeError = 0;
float lastAltitudeError = 0;
float altitudeErrorSum = 0;
float altitudePID = 0;

// MS5611 Barometer
uint16_t ms5611_prom[8];
float altitude = 0;           // Current altitude in cm
float groundAltitude = 0;     // Ground reference altitude in cm
float verticalVelocity = 0;   // Climb rate in cm/s
float lastAltitude = 0;

// Motors
int motorSpeed[4] = {1000, 1000, 1000, 1000};
int baseThrottle = 0;

// State
bool armed = false;
uint8_t flightMode = MODE_ANGLE;
bool motorTestActive = false;
int motorTestSpeed = 1000;
unsigned long lastReceiveTime = 0;
unsigned long loopTimer;

// Buttons
bool lastButton[4] = {false, false, false, false};

// Takeoff/Landing
unsigned long takeoffStartTime = 0;
unsigned long landingStartTime = 0;
float targetAltitude = 0;

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  Serial.println(F("\n╔════════════════════════════════════════════════╗"));
  Serial.println(F("║  BETAFLIGHT FC + MS5611 ALTITUDE CONTROL      ║"));
  Serial.println(F("║  Professional Quad with Altitude Hold         ║"));
  Serial.println(F("╚════════════════════════════════════════════════╝\n"));
  
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
  Serial.println(F("✅ Motors initialized"));
  
  // Init radio
  if (!radio.begin()) {
    Serial.println(F("❌ Radio FAIL!"));
    errorBlink();
  }
  radio.openReadingPipe(1, pipeIn);
  radio.openWritingPipe(pipeOut);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.startListening();
  Serial.println(F("✅ Radio initialized"));
  
  // Init I2C
  Wire.begin();
  Wire.setClock(400000);
  
  // Init MPU6050
  if (!initMPU6050()) {
    Serial.println(F("❌ MPU6050 FAIL!"));
    errorBlink();
  }
  Serial.println(F("✅ MPU6050 initialized (DLPF=42Hz)"));
  
  // Init MS5611
  if (!initMS5611()) {
    Serial.println(F("❌ MS5611 FAIL!"));
    errorBlink();
  }
  Serial.println(F("✅ MS5611 barometer initialized"));
  
  // Calibrate sensors
  calibrateGyro();
  calibrateAltitude();
  
  Serial.println(F("\n✅ SYSTEM READY!\n"));
  Serial.println(F("Controls:"));
  Serial.println(F("  Button 1 (D4) - Recalibrate sensors"));
  Serial.println(F("  Button 2 (D5) - Motor test"));
  Serial.println(F("  Button 3 (D6) - Smooth Landing"));
  Serial.println(F("  Button 4 (D7) - Smooth Takeoff"));
  Serial.println(F("  SW1 (D2) - Altitude Hold ON/OFF"));
  Serial.println(F("  SW2 (D3) - ANGLE/ACRO mode\n"));
  
  beep(2, 100);
  
  loopTimer = micros();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  // Maintain 250Hz
  while (micros() - loopTimer < MAIN_LOOP_TIME);
  loopTimer = micros();
  
  // 1. Read RC
  receiveRC();
  
  // 2. Read sensors
  readGyro();
  calculateAngles();
  readAltitude();
  
  // 3. Process buttons
  processButtons();
  
  // 4. Calculate control
  if (armed && !motorTestActive) {
    processFlightMode();
    calculatePID_Betaflight();
    
    // Add altitude control if in altitude hold/takeoff/landing
    if (flightMode == MODE_ALT_HOLD || flightMode == MODE_TAKEOFF || flightMode == MODE_LANDING) {
      calculateAltitudePID();
    }
    
    mixMotors_WithAltitude();
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
  
  // 6. Debug & telemetry
  printDebug();
  sendTelemetry();
}

// ============================================
// MPU6050 INIT
// ============================================
bool initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;
  
  // Gyro: ±500°/s
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  
  // Accel: ±8g
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  // DLPF = 42Hz
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  
  return true;
}

// ============================================
// MS5611 INIT
// ============================================
bool initMS5611() {
  // Reset MS5611
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_CMD_RESET);
  if (Wire.endTransmission() != 0) return false;
  delay(10);
  
  // Read PROM (calibration data)
  for (uint8_t i = 0; i < 8; i++) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(MS5611_CMD_PROM + (i * 2));
    Wire.endTransmission();
    
    Wire.requestFrom(MS5611_ADDR, 2);
    if (Wire.available() == 2) {
      ms5611_prom[i] = (Wire.read() << 8) | Wire.read();
    } else {
      return false;
    }
  }
  
  return true;
}

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

void calibrateAltitude() {
  Serial.print(F("⏳ Calibrating altitude (ground level)... "));
  
  float sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += readMS5611Altitude();
    delay(20);
  }
  
  groundAltitude = sum / 50.0;
  altitude = 0;
  
  Serial.print(F("DONE (Ground = "));
  Serial.print(groundAltitude, 1);
  Serial.println(F("cm)"));
}

// ============================================
// MS5611 ALTITUDE READING
// ============================================
float readMS5611Altitude() {
  // Read digital pressure (D1)
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_CMD_CONV_D1);
  Wire.endTransmission();
  delay(10);  // Wait for conversion
  
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_CMD_ADC_READ);
  Wire.endTransmission();
  Wire.requestFrom(MS5611_ADDR, 3);
  
  uint32_t D1 = 0;
  if (Wire.available() == 3) {
    D1 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  }
  
  // Read digital temperature (D2)
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_CMD_CONV_D2);
  Wire.endTransmission();
  delay(10);
  
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(MS5611_CMD_ADC_READ);
  Wire.endTransmission();
  Wire.requestFrom(MS5611_ADDR, 3);
  
  uint32_t D2 = 0;
  if (Wire.available() == 3) {
    D2 = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  }
  
  // Calculate temperature
  int32_t dT = D2 - ((uint32_t)ms5611_prom[5] << 8);
  int32_t TEMP = 2000 + ((int64_t)dT * ms5611_prom[6]) / 8388608;
  
  // Calculate pressure
  int64_t OFF = ((int64_t)ms5611_prom[2] << 16) + (((int64_t)ms5611_prom[4] * dT) / 128);
  int64_t SENS = ((int64_t)ms5611_prom[1] << 15) + (((int64_t)ms5611_prom[3] * dT) / 256);
  
  int32_t P = ((D1 * SENS / 2097152) - OFF) / 32768;
  
  // Convert pressure to altitude (barometric formula)
  // h = 44330 * (1 - (P/P0)^0.1903) where P0 = 101325 Pa
  float altitudeMeters = 44330.0 * (1.0 - pow(P / 101325.0, 0.1903));
  
  return altitudeMeters * 100.0;  // Convert to cm
}

void readAltitude() {
  static unsigned long lastRead = 0;
  
  // Read altitude at 50Hz (every 20ms)
  if (millis() - lastRead >= 20) {
    float rawAlt = readMS5611Altitude();
    
    // Complementary filter for smooth altitude
    altitude = 0.8 * altitude + 0.2 * (rawAlt - groundAltitude);
    
    // Calculate vertical velocity
    verticalVelocity = (altitude - lastAltitude) / 0.02;  // cm/s
    lastAltitude = altitude;
    
    lastRead = millis();
  }
}

// ============================================
// IMU READING
// ============================================
void readGyro() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  int16_t accelX = (Wire.read() << 8 | Wire.read());
  int16_t accelY = (Wire.read() << 8 | Wire.read());
  int16_t accelZ = (Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read();
  int16_t gyroX = (Wire.read() << 8 | Wire.read());
  int16_t gyroY = (Wire.read() << 8 | Wire.read());
  int16_t gyroZ = (Wire.read() << 8 | Wire.read());
  
  gyroRate[0] = (gyroX - gyroCal[0]) / 65.5;
  gyroRate[1] = (gyroY - gyroCal[1]) / 65.5;
  gyroRate[2] = (gyroZ - gyroCal[2]) / 65.5;
  
  float accelRoll = atan2(accelY, accelZ) * 57.2958;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958;
  
  angle[0] = 0.98 * (angle[0] + gyroRate[0] * 0.004) + 0.02 * accelRoll;
  angle[1] = 0.98 * (angle[1] + gyroRate[1] * 0.004) + 0.02 * accelPitch;
}

void calculateAngles() {
  // Already done in readGyro()
}

// ============================================
// FLIGHT MODE PROCESSING
// ============================================
void processFlightMode() {
  // Check for takeoff/landing modes first
  if (flightMode == MODE_TAKEOFF) {
    // Smooth takeoff in progress
    unsigned long elapsed = millis() - takeoffStartTime;
    float progress = elapsed / 1000.0;  // seconds
    
    targetAltitude = progress * TAKEOFF_RATE;  // cm
    
    if (targetAltitude >= TAKEOFF_ALTITUDE) {
      targetAltitude = TAKEOFF_ALTITUDE;
      flightMode = MODE_ALT_HOLD;  // Switch to altitude hold at target
      altitudeSetpoint = targetAltitude;
      Serial.println(F("✅ Takeoff complete, entering ALT HOLD"));
    }
    
    altitudeSetpoint = targetAltitude;
    return;
  }
  
  if (flightMode == MODE_LANDING) {
    // Smooth landing in progress
    unsigned long elapsed = millis() - landingStartTime;
    float progress = elapsed / 1000.0;
    
    targetAltitude = altitudeSetpoint - (progress * LANDING_RATE);
    
    if (targetAltitude <= 10.0 || altitude <= 10.0) {
      // Landed
      armed = false;
      flightMode = MODE_ANGLE;
      Serial.println(F("✅ Landing complete, DISARMED"));
      beep(3, 100);
    }
    
    altitudeSetpoint = targetAltitude;
    return;
  }
  
  // Normal flight modes
  bool altHoldSwitch = (rxData.switches & 0x01);  // SW1 (D2)
  bool acroSwitch = !(rxData.switches & 0x02);     // SW2 (D3)
  
  if (altHoldSwitch) {
    if (flightMode != MODE_ALT_HOLD) {
      // Entering altitude hold - lock current altitude
      flightMode = MODE_ALT_HOLD;
      altitudeSetpoint = altitude;
      altitudeErrorSum = 0;  // Reset integral
      Serial.print(F("🔒 ALT HOLD at "));
      Serial.print(altitude, 0);
      Serial.println(F("cm"));
    }
    // In altitude hold, throttle adjusts setpoint
    altitudeSetpoint += (rxData.throttle - 1500) * 0.02;  // ±10cm/s max adjustment
    altitudeSetpoint = constrain(altitudeSetpoint, 0, 500);  // Max 5m
    
  } else {
    // Normal ANGLE or ACRO mode
    flightMode = acroSwitch ? MODE_ACRO : MODE_ANGLE;
  }
}

// ============================================
// ATTITUDE PID (BETAFLIGHT STYLE)
// ============================================
void calculatePID_Betaflight() {
  // Convert stick inputs
  if (flightMode == MODE_ANGLE || flightMode == MODE_ALT_HOLD || 
      flightMode == MODE_TAKEOFF || flightMode == MODE_LANDING) {
    // ANGLE MODE
    angleSetpoint[0] = (rxData.roll / 500.0) * ANGLE_LIMIT;
    angleSetpoint[1] = (rxData.pitch / 500.0) * ANGLE_LIMIT;
    
    rateSetpoint[0] = (angleSetpoint[0] - angle[0]) * ANGLE_P;
    rateSetpoint[1] = (angleSetpoint[1] - angle[1]) * ANGLE_P;
    
    rateSetpoint[0] = constrain(rateSetpoint[0], -500, 500);
    rateSetpoint[1] = constrain(rateSetpoint[1], -500, 500);
  } else {
    // ACRO MODE
    rateSetpoint[0] = (rxData.roll / 500.0) * 500.0;
    rateSetpoint[1] = (rxData.pitch / 500.0) * 500.0;
  }
  
  rateSetpoint[2] = (rxData.yaw / 500.0) * 400.0;
  
  // RATE PID
  float dtSec = 0.004;
  
  for (int i = 0; i < 3; i++) {
    rateError[i] = rateSetpoint[i] - gyroRate[i];
    rateErrorSum[i] += rateError[i] * dtSec;
    rateErrorSum[i] = constrain(rateErrorSum[i], -I_LIMIT, I_LIMIT);
    
    float kp = (i == 2) ? RATE_P_YAW : (i == 0 ? RATE_P_ROLL : RATE_P_PITCH);
    float ki = (i == 2) ? RATE_I_YAW : (i == 0 ? RATE_I_ROLL : RATE_I_PITCH);
    float kd = (i == 2) ? RATE_D_YAW : (i == 0 ? RATE_D_ROLL : RATE_D_PITCH);
    
    rateP[i] = kp * rateError[i];
    rateI[i] = ki * rateErrorSum[i];
    rateD[i] = kd * (rateError[i] - lastRateError[i]) / dtSec;
    
    ratePID[i] = rateP[i] + rateI[i] + rateD[i];
    ratePID[i] = constrain(ratePID[i], -RATE_LIMIT, RATE_LIMIT);
    lastRateError[i] = rateError[i];
  }
}

// ============================================
// ALTITUDE PID
// ============================================
void calculateAltitudePID() {
  float dtSec = 0.004;
  
  // Error = desired - actual
  altitudeError = altitudeSetpoint - altitude;
  
  // Integral with anti-windup
  altitudeErrorSum += altitudeError * dtSec;
  altitudeErrorSum = constrain(altitudeErrorSum, -ALT_I_LIMIT, ALT_I_LIMIT);
  
  // PID calculation
  float altP = ALT_P * altitudeError;
  float altI = ALT_I * altitudeErrorSum;
  float altD = ALT_D * (altitudeError - lastAltitudeError) / dtSec;
  
  altitudePID = altP + altI + altD;
  
  // Add velocity feedforward (dampening)
  altitudePID -= verticalVelocity * 2.0;
  
  // Limit output
  altitudePID = constrain(altitudePID, -400, 400);
  
  lastAltitudeError = altitudeError;
}

// ============================================
// MOTOR MIXING WITH ALTITUDE
// ============================================
void mixMotors_WithAltitude() {
  int throttle = rxData.throttle;
  
  // In altitude control modes, altitude PID controls base throttle
  if (flightMode == MODE_ALT_HOLD || flightMode == MODE_TAKEOFF || flightMode == MODE_LANDING) {
    // Base throttle from altitude PID
    baseThrottle = 1200 + (int)altitudePID;  // Hover ~1200-1400
    baseThrottle = constrain(baseThrottle, 1100, 1800);
    throttle = baseThrottle;
  } else {
    // Manual throttle
    if (throttle < 1100) throttle = 1100;
    baseThrottle = throttle;
  }
  
  // Scale PID for motor range
  float rollMix = ratePID[0] * 0.5;
  float pitchMix = ratePID[1] * 0.5;
  float yawMix = ratePID[2] * 0.5;
  
  // X-Configuration
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
  for (int i = 0; i < 4; i++) motorSpeed[i] = 1000;
  for (int i = 0; i < 3; i++) rateErrorSum[i] = 0;
  altitudeErrorSum = 0;
}

// ============================================
// BUTTONS & RC
// ============================================
void receiveRC() {
  if (radio.available()) {
    radio.read(&rxData, sizeof(ControlData));
    lastReceiveTime = millis();
  } else {
    if (millis() - lastReceiveTime > FAILSAFE_TIMEOUT && armed) {
      armed = false;
      flightMode = MODE_ANGLE;
      Serial.println(F("⚠️ FAILSAFE!"));
    }
  }
}

void processButtons() {
  // Button 1 (D4): Recalibrate
  bool btn1 = (rxData.buttons & 0x01);
  if (btn1 && !lastButton[0] && !armed) {
    calibrateGyro();
    calibrateAltitude();
  }
  lastButton[0] = btn1;
  
  // Button 2 (D5): Motor test
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
  
  // Button 3 (D6): Smooth Landing
  bool btn3 = (rxData.buttons & 0x04);
  if (btn3 && !lastButton[2] && armed) {
    if (flightMode != MODE_LANDING) {
      flightMode = MODE_LANDING;
      landingStartTime = millis();
      Serial.println(F("🛬 Starting smooth landing..."));
      beep(1, 200);
    }
  }
  lastButton[2] = btn3;
  
  // Button 4 (D7): Smooth Takeoff
  bool btn4 = (rxData.buttons & 0x08);
  if (btn4 && !lastButton[3]) {
    if (!armed && rxData.throttle < 1050) {
      // ARM and start takeoff
      armed = true;
      flightMode = MODE_TAKEOFF;
      takeoffStartTime = millis();
      targetAltitude = 0;
      altitudeSetpoint = 0;
      Serial.println(F("🚁 ARM + Smooth takeoff initiated!"));
      beep(2, 100);
    }
  }
  lastButton[3] = btn4;
}

// ============================================
// DEBUG
// ============================================
void printDebug() {
  static unsigned long lastPrint = 0;
  
  if (millis() - lastPrint > 100) {
    Serial.print(armed ? F("ARM ") : F("--- "));
    
    // Mode indicator
    switch(flightMode) {
      case MODE_ANGLE:    Serial.print(F("ANG")); break;
      case MODE_ACRO:     Serial.print(F("ACR")); break;
      case MODE_ALT_HOLD: Serial.print(F("ALT")); break;
      case MODE_TAKEOFF:  Serial.print(F("T/O")); break;
      case MODE_LANDING:  Serial.print(F("LND")); break;
    }
    
    Serial.print(F(" | Alt:"));
    Serial.print(altitude, 0);
    Serial.print(F("cm"));
    
    if (flightMode == MODE_ALT_HOLD || flightMode == MODE_TAKEOFF || flightMode == MODE_LANDING) {
      Serial.print(F(" → "));
      Serial.print(altitudeSetpoint, 0);
      Serial.print(F("cm | AltPID:"));
      Serial.print((int)altitudePID);
    }
    
    Serial.print(F(" | Ang R:"));
    Serial.print(angle[0], 0);
    Serial.print(F(" P:"));
    Serial.print(angle[1], 0);
    
    Serial.print(F(" | M:"));
    for (int i = 0; i < 4; i++) {
      Serial.print(motorSpeed[i]);
      Serial.print(F(" "));
    }
    
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
    telemetry.altitude = altitude;
    telemetry.battery = 100;
    telemetry.flightMode = flightMode;
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

void errorBlink() {
  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
    delay(200);
  }
}
