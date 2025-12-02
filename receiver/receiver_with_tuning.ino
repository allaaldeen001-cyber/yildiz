/*
 * Drone Flight Controller with Live PID Tuning
 * 
 * This version includes serial PID tuning capability.
 * Use Serial Monitor to adjust PID values in real-time!
 * 
 * Commands:
 *   RP1.5  - Set Roll P to 1.5
 *   RI0.05 - Set Roll I to 0.05
 *   RD20   - Set Roll D to 20
 *   PRINT  - Print all PID values
 *   SAVE   - Print code to copy/paste
 * 
 * ⚠️ WARNING: Live PID tuning is for testing only!
 * Once you find good values, update receiver.ino with permanent values.
 */

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>
#include "../common/RF24_Config.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

#define MOTOR1_PIN  3
#define MOTOR2_PIN  5
#define MOTOR3_PIN  6
#define MOTOR4_PIN  A0

#define LED_PIN     LED_BUILTIN

// ============================================================================
// MPU6050 CONFIGURATION
// ============================================================================

#define MPU6050_ADDR  0x68
#define GYRO_SENSITIVITY  65.5

int16_t accXOffset = 0;
int16_t accYOffset = 0;
int16_t accZOffset = 0;

float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

// ============================================================================
// PID CONFIGURATION - ADJUST THESE VIA SERIAL!
// ============================================================================

float rollKp = 1.3;
float rollKi = 0.04;
float rollKd = 18.0;

float pitchKp = 1.3;
float pitchKi = 0.04;
float pitchKd = 18.0;

float yawKp = 4.0;
float yawKi = 0.02;
float yawKd = 0.0;

#define PID_MAX   400
#define PID_I_MAX 100

// Include PID tuning utility AFTER PID variables are declared
#include "PID_Tuning.h"

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================

#define MOTOR_MIN     1000
#define MOTOR_MAX     2000
#define MOTOR_ARM     1000
#define MOTOR_IDLE    1100

#define THROTTLE_MIN  0
#define THROTTLE_MAX  1000

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

RF24 radio(RX_CE_PIN, RX_CSN_PIN);
Servo motor1, motor2, motor3, motor4;
ControlData controlData;

unsigned long lastReceiveTime = 0;
bool signalLost = false;
bool radioInitialized = false;
bool armed = false;
bool armSwitchPrevious = false;

float gyroRoll, gyroPitch, gyroYaw;
float accRoll, accPitch;
float angleRoll, anglePitch;

float rollError, rollPrevError, rollIntegral, rollDerivative, rollOutput;
float pitchError, pitchPrevError, pitchIntegral, pitchDerivative, pitchOutput;
float yawError, yawPrevError, yawIntegral, yawDerivative, yawOutput;

int motor1Speed, motor2Speed, motor3Speed, motor4Speed;

unsigned long loopTimer;
unsigned long lastLoopTime;
float dt;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== Drone Flight Controller with PID Tuning ==="));
  Serial.println(F("Commands: RP, RI, RD, PP, PI, PD, YP, YI, YD, PRINT, SAVE"));
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  Wire.begin();
  Wire.setClock(400000);
  
  initMPU6050();
  calibrateGyro();
  initMotors();
  initRadio();
  
  memset(&controlData, 0, sizeof(ControlData));
  
  lastLoopTime = micros();
  loopTimer = micros();
  
  digitalWrite(LED_PIN, LOW);
  
  printPIDValues();  // Print initial values
  Serial.println(F("\nReady! Use serial commands to tune PID."));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long currentMicros = micros();
  dt = (currentMicros - lastLoopTime) / 1000000.0;
  lastLoopTime = currentMicros;
  
  // Process serial PID commands
  processPIDCommand();
  
  receiveData();
  checkSignal();
  handleArming();
  readIMU();
  calculateAngles();
  calculatePID();
  calculateMotorSpeeds();
  applyMotorSpeeds();
  
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug >= 200) {
    lastDebug = millis();
    printDebugInfo();
  }
  
  while (micros() - loopTimer < 4000);
  loopTimer = micros();
}

// ============================================================================
// RADIO FUNCTIONS
// ============================================================================

void initRadio() {
  if (!radio.begin()) {
    Serial.println(F("ERROR: Radio not responding!"));
    radioInitialized = false;
    return;
  }
  
  radio.setChannel(RF_CHANNEL);
  radio.setDataRate(RF_DATA_RATE);
  radio.setPALevel(RF_PA_LEVEL);
  radio.setAutoAck(false);
  radio.setPayloadSize(sizeof(ControlData));
  radio.openReadingPipe(1, RADIO_ADDRESS);
  radio.startListening();
  
  radioInitialized = true;
  Serial.println(F("Radio OK"));
}

void receiveData() {
  if (!radioInitialized) return;
  
  if (radio.available()) {
    radio.read(&controlData, sizeof(ControlData));
    
    if (validateControlData(controlData)) {
      lastReceiveTime = millis();
      signalLost = false;
    }
  }
}

void checkSignal() {
  if (millis() - lastReceiveTime > SIGNAL_TIMEOUT_MS) {
    if (!signalLost) {
      signalLost = true;
      Serial.println(F("FAILSAFE!"));
    }
    
    if (armed) {
      if (controlData.throttle > 0) {
        controlData.throttle = max(0, (int)controlData.throttle - 10);
      }
      controlData.roll = 0;
      controlData.pitch = 0;
      controlData.yaw = 0;
    }
  }
}

// ============================================================================
// MPU6050 FUNCTIONS
// ============================================================================

void initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  
  Serial.println(F("MPU6050 OK"));
}

void calibrateGyro() {
  Serial.println(F("Calibrating gyro... KEEP STILL!"));
  
  float sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 2000;
  
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6);
    
    sumX += (Wire.read() << 8 | Wire.read());
    sumY += (Wire.read() << 8 | Wire.read());
    sumZ += (Wire.read() << 8 | Wire.read());
    
    if (i % 200 == 0) digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delayMicroseconds(500);
  }
  
  gyroXOffset = sumX / samples;
  gyroYOffset = sumY / samples;
  gyroZOffset = sumZ / samples;
  
  Serial.println(F("Gyro calibrated!"));
}

void readIMU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);
  
  int16_t accX = Wire.read() << 8 | Wire.read();
  int16_t accY = Wire.read() << 8 | Wire.read();
  int16_t accZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();  // Skip temp
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  
  gyroRoll = (gyroX - gyroXOffset) / GYRO_SENSITIVITY;
  gyroPitch = (gyroY - gyroYOffset) / GYRO_SENSITIVITY;
  gyroYaw = (gyroZ - gyroZOffset) / GYRO_SENSITIVITY;
  
  accRoll = atan2(accY, accZ) * 180.0 / PI;
  accPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
}

void calculateAngles() {
  const float alpha = 0.98;
  
  static bool initialized = false;
  if (!initialized) {
    angleRoll = accRoll;
    anglePitch = accPitch;
    initialized = true;
    return;
  }
  
  angleRoll += gyroRoll * dt;
  anglePitch += gyroPitch * dt;
  
  angleRoll = alpha * angleRoll + (1.0 - alpha) * accRoll;
  anglePitch = alpha * anglePitch + (1.0 - alpha) * accPitch;
}

// ============================================================================
// MOTOR FUNCTIONS
// ============================================================================

void initMotors() {
  motor1.attach(MOTOR1_PIN, MOTOR_MIN, MOTOR_MAX);
  motor2.attach(MOTOR2_PIN, MOTOR_MIN, MOTOR_MAX);
  motor3.attach(MOTOR3_PIN, MOTOR_MIN, MOTOR_MAX);
  motor4.attach(MOTOR4_PIN, MOTOR_MIN, MOTOR_MAX);
  
  motor1.writeMicroseconds(MOTOR_MIN);
  motor2.writeMicroseconds(MOTOR_MIN);
  motor3.writeMicroseconds(MOTOR_MIN);
  motor4.writeMicroseconds(MOTOR_MIN);
  
  delay(2000);
  Serial.println(F("ESCs OK"));
}

void handleArming() {
  bool armSwitch = controlData.aux1 > 127;
  
  if (armSwitch != armSwitchPrevious) {
    armSwitchPrevious = armSwitch;
    
    if (armSwitch) {
      if (controlData.throttle < 50) {
        armed = true;
        rollIntegral = pitchIntegral = yawIntegral = 0;
        Serial.println(F(">>> ARMED <<<"));
      }
    } else {
      armed = false;
      Serial.println(F(">>> DISARMED <<<"));
    }
  }
}

void calculatePID() {
  bool angleMode = controlData.aux2 < 127;
  
  if (angleMode) {
    float desiredAngleRoll = controlData.roll * 0.05;
    float desiredAnglePitch = controlData.pitch * 0.05;
    
    rollError = desiredAngleRoll - angleRoll;
    pitchError = desiredAnglePitch - anglePitch;
    yawError = (controlData.yaw * 0.5) - gyroYaw;
  } else {
    rollError = (controlData.roll * 0.5) - gyroRoll;
    pitchError = (controlData.pitch * 0.5) - gyroPitch;
    yawError = (controlData.yaw * 0.5) - gyroYaw;
  }
  
  // Roll PID
  rollIntegral += rollError * dt;
  rollIntegral = constrain(rollIntegral, -PID_I_MAX, PID_I_MAX);
  rollDerivative = (rollError - rollPrevError) / dt;
  rollOutput = rollKp * rollError + rollKi * rollIntegral + rollKd * rollDerivative;
  rollOutput = constrain(rollOutput, -PID_MAX, PID_MAX);
  rollPrevError = rollError;
  
  // Pitch PID
  pitchIntegral += pitchError * dt;
  pitchIntegral = constrain(pitchIntegral, -PID_I_MAX, PID_I_MAX);
  pitchDerivative = (pitchError - pitchPrevError) / dt;
  pitchOutput = pitchKp * pitchError + pitchKi * pitchIntegral + pitchKd * pitchDerivative;
  pitchOutput = constrain(pitchOutput, -PID_MAX, PID_MAX);
  pitchPrevError = pitchError;
  
  // Yaw PID
  yawIntegral += yawError * dt;
  yawIntegral = constrain(yawIntegral, -PID_I_MAX, PID_I_MAX);
  yawDerivative = (yawError - yawPrevError) / dt;
  yawOutput = yawKp * yawError + yawKi * yawIntegral + yawKd * yawDerivative;
  yawOutput = constrain(yawOutput, -PID_MAX, PID_MAX);
  yawPrevError = yawError;
  
  if (!armed) {
    rollIntegral = pitchIntegral = yawIntegral = 0;
  }
}

void calculateMotorSpeeds() {
  if (!armed) {
    motor1Speed = motor2Speed = motor3Speed = motor4Speed = MOTOR_MIN;
    return;
  }
  
  int throttle = map(controlData.throttle, 0, 1000, MOTOR_IDLE, MOTOR_MAX);
  
  motor1Speed = throttle + pitchOutput + rollOutput - yawOutput;
  motor2Speed = throttle + pitchOutput - rollOutput + yawOutput;
  motor3Speed = throttle - pitchOutput + rollOutput + yawOutput;
  motor4Speed = throttle - pitchOutput - rollOutput - yawOutput;
  
  motor1Speed = constrain(motor1Speed, MOTOR_IDLE, MOTOR_MAX);
  motor2Speed = constrain(motor2Speed, MOTOR_IDLE, MOTOR_MAX);
  motor3Speed = constrain(motor3Speed, MOTOR_IDLE, MOTOR_MAX);
  motor4Speed = constrain(motor4Speed, MOTOR_IDLE, MOTOR_MAX);
  
  if (controlData.throttle < 50) {
    motor1Speed = motor2Speed = motor3Speed = motor4Speed = MOTOR_MIN;
  }
}

void applyMotorSpeeds() {
  motor1.writeMicroseconds(motor1Speed);
  motor2.writeMicroseconds(motor2Speed);
  motor3.writeMicroseconds(motor3Speed);
  motor4.writeMicroseconds(motor4Speed);
}

void printDebugInfo() {
  Serial.print(armed ? F("ARM ") : F("DIS "));
  Serial.print(signalLost ? F("LOST ") : F("OK "));
  Serial.print(F("R:")); Serial.print(angleRoll, 1);
  Serial.print(F(" P:")); Serial.print(anglePitch, 1);
  Serial.print(F(" M:")); Serial.print(motor1Speed);
  Serial.print(F(",")); Serial.print(motor2Speed);
  Serial.print(F(",")); Serial.print(motor3Speed);
  Serial.print(F(",")); Serial.println(motor4Speed);
}
