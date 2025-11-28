/*
 * DIY Drone Flight Controller
 * 
 * Hardware: Arduino Nano, NRF24L01, MPU6050, MS5611
 * 
 * PIN ASSIGNMENTS (Resolved conflicts):
 * =====================================
 * ESC Motors (PWM):
 *   D3  = Front Left (FL)
 *   D5  = Front Right (FR)
 *   D6  = Rear Right (RR)
 *   D9  = Rear Left (RL)
 * 
 * NRF24L01:
 *   D4  = CE
 *   D10 = CSN
 *   D11 = MOSI (SPI)
 *   D12 = MISO (SPI)
 *   D13 = SCK (SPI)
 * 
 * I2C (MPU6050 + MS5611):
 *   A4  = SDA
 *   A5  = SCL
 * 
 * Controls (using analog pins as digital to avoid conflicts):
 *   A1  = Calibration Button (INPUT_PULLUP, active LOW)
 *   A2  = Smooth Motor Start Button (INPUT_PULLUP, active LOW)
 *   A3  = Arm/Disarm Switch (INPUT_PULLUP, HIGH=Disarmed, LOW=Armed)
 *   D2  = Altitude Hold Switch (INPUT_PULLUP, LOW=Active)
 * 
 * Outputs:
 *   D7  = Status LED
 *   D8  = Buzzer
 * 
 * Analog:
 *   A0  = Battery Voltage Monitor
 */

#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <EEPROM.h>
#include "Gyro.h"
#include <Smoothed.h>
#include <Wire.h>
#include "MS5611.h"

// ==================== PIN DEFINITIONS ====================
// ESC Motor Pins (PWM capable)
const int PIN_ESC_FL = 3;   // Front Left
const int PIN_ESC_FR = 5;   // Front Right
const int PIN_ESC_RR = 6;   // Rear Right
const int PIN_ESC_RL = 9;   // Rear Left

// NRF24L01 Pins
const int PIN_NRF_CE  = 4;
const int PIN_NRF_CSN = 10;

// Control Pins
const int PIN_CALIBRATE_BTN    = A1;  // Calibration button
const int PIN_MOTOR_START_BTN  = A2;  // Smooth motor start button
const int PIN_ARM_SWITCH       = A3;  // Arm/Disarm switch
const int PIN_ALTITUDE_SWITCH  = 2;   // Altitude hold switch

// Output Pins
const int PIN_LED    = 7;
const int PIN_BUZZER = 8;

// Analog Pins
const int PIN_BATTERY = A0;

// ==================== NRF24L01 CONFIG ====================
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const uint64_t PIPE_ADDRESS = 0xF0F0F0F0E1LL;
const uint8_t NRF_CHANNEL = 108;  // Stable channel (avoid WiFi interference)

// ==================== MS5611 ====================
MS5611 ms5611(0x77);

// ==================== DATA STRUCTURES ====================
struct ControlPackage {
  int16_t thrust = 0;
  float x = 0;        // Roll
  float y = 0;        // Pitch
  float z = 0;        // Yaw
  uint16_t id = 0;
  bool but1 = 1;      // Calibration button (RC side)
  bool but2 = 1;      // Motor start button (RC side)
  bool switch1 = 1;   // Arm switch (RC side)
  bool switch2 = 1;   // Altitude hold (RC side)
};

struct AckPackage {
  uint16_t lastId = 0;
  bool armed = false;
  bool altitudeHold = false;
  uint8_t batteryPercent = 100;
  int16_t currentAltitude = 0;
};

ControlPackage rxPackage;
AckPackage ackPackage;

// ==================== SYSTEM OBJECTS ====================
Gyro gyro;
Servo escFL, escFR, escRL, escRR;
Smoothed<float> pressureSmooth;

// ==================== PID PARAMETERS ====================
// Attitude PID
const float KP = 2.0;
const float KI = 0.0001;
const float KD = 0.5;
const float KP_Z = 2.0;  // Yaw

// Altitude PID
float pidAltP = 14.0;
float pidAltI = 2.0;
float pidAltD = 7.5;
const int PID_ALT_MAX = 400;

// ==================== STATE VARIABLES ====================
bool systemArmed = false;
bool altitudeHoldActive = false;
bool nrfConnected = false;
bool calibrationDone = false;

// Control inputs
int thrust = 1000;
Vec3 target = {0, 0, 0};
Vec3 cal = {0, 0, 0};
Vec3 prevError = {0, 0, 0};
Vec3 PID[3] = {{0,0,0}, {0,0,0}, {0,0,0}};  // P, I, D terms

// Motor outputs
int motorFL = 1000;
int motorFR = 1000;
int motorRL = 1000;
int motorRR = 1000;

// Altitude hold variables
float groundPressure = 0;
float altitudeSetpoint = 0;
float pidAltIAccum = 0;
float pidAltOutput = 0;
float lastAltError = 0;
int manualThrottle = 0;

// Pressure averaging
Smoothed<float> altitudeSmooth;
float currentPressure = 0;
float currentAltitude = 0;
int pressureReadCounter = 0;

// Kalman filter
struct QuadProps {
  float height;
  float kalmanVelZ;
  float baroHeight;
} quadProps;

struct Matrix2x2 {
  float m11, m12, m21, m22;
} kalmanProb;

// Timing
const float LOOP_HZ = 140.0;
float loopPeriod = 1.0 / LOOP_HZ;
unsigned long prevLoopTime = 0;

// Safety limits
const int PWM_MIN = 1000;
const int PWM_MAX = 2000;
const int PWM_ARMED_MIN = 1050;
const int THRUST_MAX = 1700;
const float MAX_TILT_ANGLE = 30.0;  // Safety limit

// Communication timeout
float noDataTime = 0;
const float COMM_TIMEOUT = 3.0;  // seconds

// Button/Switch states
bool lastCalibrateBtnState = HIGH;
bool lastMotorStartBtnState = HIGH;
unsigned long calibrateHoldStart = 0;
unsigned long motorStartHoldStart = 0;
bool smoothStartActive = false;
int smoothStartPWM = PWM_MIN;

// Sensitivity
const float SENS_X = -0.45;
const float SENS_Y = 0.45;
const float SENS_Z = -0.01;
const float SENS_THRUST = 1.1;

// Dead zones
const int DEADZONE_XY = 5;
const int DEADZONE_Z = 10;

// Battery monitoring
const float R1 = 1500.0;
const float R2 = 1000.0;

// LED control
unsigned long lastLedBlink = 0;
bool ledState = false;

// ==================== FUNCTION PROTOTYPES ====================
void initializeHardware();
void initializeNRF();
void initializeSensors();
bool receiveRadioWithAck();
void sendAckPayload();
void readLocalControls();
void updateSystemState();
void runCalibration();
void smoothMotorStart();
void calculatePID();
void calculateMotorOutputs();
void runMotors();
void stopMotors();
void calculatePressure();
void calculateAltitudePID();
void initKalman();
void updateKalman();
float readBatteryVoltage();
void updateLED();
void beepConfirm(int pattern);
void resetYaw();
void printDebug();

// ==================== SETUP ====================
void setup() {
  Serial.begin(57600);
  Serial.println(F("=== Drone Flight Controller ==="));
  
  initializeHardware();
  
  // Startup beep sequence
  beepConfirm(1);
  delay(100);
  beepConfirm(2);
  delay(100);
  beepConfirm(3);
  
  initializeNRF();
  initializeSensors();
  
  // Final ready beep
  delay(500);
  beepConfirm(3);
  
  prevLoopTime = micros();
  Serial.println(F("System Ready"));
}

// ==================== MAIN LOOP ====================
void loop() {
  // Receive radio data with ACK
  bool dataReceived = receiveRadioWithAck();
  
  // Read local buttons and switches
  readLocalControls();
  
  // Update system state (arming, altitude hold, etc.)
  updateSystemState();
  
  // Update LED based on state
  updateLED();
  
  if (systemArmed && !smoothStartActive) {
    // Normal flight operations
    gyro.setTarget(target);
    gyro.setCalibration(cal);
    
    calculatePressure();
    gyro.calculateError();
    
    // Safety check - max tilt angle
    if (abs(gyro.error.x) > MAX_TILT_ANGLE || abs(gyro.error.y) > MAX_TILT_ANGLE) {
      Serial.println(F("SAFETY: Max tilt exceeded!"));
      stopMotors();
      systemArmed = false;
      beepConfirm(4);  // Warning beep
    } else {
      calculatePID();
      calculateMotorOutputs();
      runMotors();
    }
  } else if (smoothStartActive) {
    // Smooth motor start mode
    smoothMotorStart();
  } else {
    // Disarmed - keep motors stopped
    stopMotors();
    resetYaw();
  }
  
  // Debug output
  printDebug();
  
  // Maintain loop timing
  while (micros() - prevLoopTime < (unsigned long)(loopPeriod * 1000000));
  prevLoopTime = micros();
}

// ==================== INITIALIZATION ====================
void initializeHardware() {
  // Configure pins
  pinMode(PIN_CALIBRATE_BTN, INPUT_PULLUP);
  pinMode(PIN_MOTOR_START_BTN, INPUT_PULLUP);
  pinMode(PIN_ARM_SWITCH, INPUT_PULLUP);
  pinMode(PIN_ALTITUDE_SWITCH, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BATTERY, INPUT);
  
  // Initialize ESCs
  escFL.attach(PIN_ESC_FL, PWM_MIN, PWM_MAX);
  escFR.attach(PIN_ESC_FR, PWM_MIN, PWM_MAX);
  escRL.attach(PIN_ESC_RL, PWM_MIN, PWM_MAX);
  escRR.attach(PIN_ESC_RR, PWM_MIN, PWM_MAX);
  stopMotors();
  
  Serial.println(F("Hardware initialized"));
}

void initializeNRF() {
  radio.begin();
  radio.setChannel(NRF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);  // Enable ACK
  radio.enableAckPayload();
  radio.setRetries(5, 15);  // 5x250us delay, 15 retries
  radio.openReadingPipe(1, PIPE_ADDRESS);
  radio.startListening();
  
  // Prepare initial ACK payload
  sendAckPayload();
  
  Serial.println(F("NRF24L01 initialized with ACK"));
  Serial.print(F("Channel: "));
  Serial.println(NRF_CHANNEL);
}

void initializeSensors() {
  // Initialize gyro/accelerometer
  gyro.SetupWire(loopPeriod);
  Serial.println(F("MPU6050 initialized"));
  
  // Initialize barometer
  if (ms5611.begin()) {
    Serial.println(F("MS5611 initialized"));
  } else {
    Serial.println(F("MS5611 init failed!"));
  }
  ms5611.setOversampling(OSR_LOW);
  
  // Initialize smoothing filters
  pressureSmooth.begin(SMOOTHED_AVERAGE, 10);
  altitudeSmooth.begin(SMOOTHED_AVERAGE, 10);
  
  // Read EEPROM calibration
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
  
  // Check for valid calibration data
  if (isnan(cal.x) || isnan(cal.y)) {
    cal.x = 0;
    cal.y = 0;
  }
  
  // Initialize Kalman filter
  initKalman();
  
  // Record ground pressure
  delay(500);
  for (int i = 0; i < 50; i++) {
    ms5611.read();
    pressureSmooth.add(ms5611.getPressure());
    delay(20);
  }
  groundPressure = pressureSmooth.get();
  
  Serial.print(F("Ground pressure: "));
  Serial.println(groundPressure);
}

// ==================== COMMUNICATION ====================
bool receiveRadioWithAck() {
  if (radio.available()) {
    radio.read(&rxPackage, sizeof(rxPackage));
    
    // Update connection status
    if (!nrfConnected) {
      nrfConnected = true;
      Serial.println(F("NRF Connected!"));
      beepConfirm(2);
    }
    
    noDataTime = 0;
    
    // Process received data
    if (rxPackage.thrust != 0 || rxPackage.id > 0) {
      // Apply dead zones
      float x = rxPackage.x;
      float y = rxPackage.y;
      float z = rxPackage.z;
      
      if (abs(x) < DEADZONE_XY) x = 0;
      if (abs(y) < DEADZONE_XY) y = 0;
      if (abs(z) < DEADZONE_Z) z = 0;
      
      target.x = x * SENS_X;
      target.y = y * SENS_Y;
      
      if (systemArmed) {
        target.z += z * SENS_Z;
      }
      
      thrust = constrain((int)(rxPackage.thrust * SENS_THRUST), PWM_MIN, THRUST_MAX);
      
      // Prepare ACK payload with current status
      sendAckPayload();
      
      return true;
    }
  } else {
    noDataTime += loopPeriod;
    
    // Check for communication timeout
    if (noDataTime > COMM_TIMEOUT && systemArmed) {
      Serial.println(F("COMM TIMEOUT - Emergency stop!"));
      stopMotors();
      systemArmed = false;
      beepConfirm(4);
    }
    
    if (nrfConnected && noDataTime > 1.0) {
      nrfConnected = false;
      Serial.println(F("NRF Disconnected"));
    }
  }
  
  return false;
}

void sendAckPayload() {
  ackPackage.lastId = rxPackage.id;
  ackPackage.armed = systemArmed;
  ackPackage.altitudeHold = altitudeHoldActive;
  ackPackage.batteryPercent = (uint8_t)constrain(readBatteryVoltage() / 12.6 * 100, 0, 100);
  ackPackage.currentAltitude = (int16_t)currentAltitude;
  
  radio.writeAckPayload(1, &ackPackage, sizeof(ackPackage));
}

// ==================== LOCAL CONTROLS ====================
void readLocalControls() {
  // Read arm switch (A3) - HIGH = Disarmed, LOW = Armed
  bool armSwitchState = digitalRead(PIN_ARM_SWITCH);
  
  // Read altitude hold switch (D2) - LOW = Active
  bool altSwitchState = digitalRead(PIN_ALTITUDE_SWITCH);
  altitudeHoldActive = (altSwitchState == LOW) && systemArmed;
  
  // Read calibration button (A1)
  bool calBtnState = digitalRead(PIN_CALIBRATE_BTN);
  
  // Read motor start button (A2)
  bool motorStartBtnState = digitalRead(PIN_MOTOR_START_BTN);
  
  // Handle arm switch
  if (armSwitchState == HIGH) {
    // Disarmed
    if (systemArmed) {
      systemArmed = false;
      smoothStartActive = false;
      stopMotors();
      Serial.println(F("DISARMED"));
    }
  }
  
  // Handle calibration button (only when disarmed)
  if (calBtnState == LOW && lastCalibrateBtnState == HIGH) {
    calibrateHoldStart = millis();
  }
  
  if (calBtnState == LOW && !systemArmed) {
    if (millis() - calibrateHoldStart > 2000) {
      runCalibration();
      calibrateHoldStart = millis() + 10000;  // Prevent re-trigger
    }
  }
  lastCalibrateBtnState = calBtnState;
  
  // Handle motor start button (only when armed)
  if (motorStartBtnState == LOW && lastMotorStartBtnState == HIGH) {
    motorStartHoldStart = millis();
  }
  
  if (motorStartBtnState == LOW && armSwitchState == LOW && !systemArmed) {
    if (millis() - motorStartHoldStart > 500) {
      // Start smooth motor ramp-up
      smoothStartActive = true;
      smoothStartPWM = PWM_MIN;
      systemArmed = true;
      Serial.println(F("ARMED - Smooth start"));
      beepConfirm(2);
      motorStartHoldStart = millis() + 10000;
    }
  }
  lastMotorStartBtnState = motorStartBtnState;
}

void updateSystemState() {
  // Handle yaw wrap-around
  if (gyro.error.z > 180 || gyro.error.z < -180) {
    resetYaw();
  }
  
  // Update altitude hold setpoint when entering hold mode
  static bool lastAltHoldState = false;
  if (altitudeHoldActive && !lastAltHoldState) {
    altitudeSetpoint = currentPressure;
    pidAltIAccum = 0;
    Serial.println(F("Altitude Hold Engaged"));
  }
  lastAltHoldState = altitudeHoldActive;
}

// ==================== CALIBRATION ====================
void runCalibration() {
  Serial.println(F("Starting calibration..."));
  stopMotors();
  
  // Beep to indicate start
  tone(PIN_BUZZER, 1200, 100);
  digitalWrite(PIN_LED, HIGH);
  delay(300);
  tone(PIN_BUZZER, 1200, 200);
  
  // Calibrate MPU6050
  cal = gyro.calibrate(1000);
  
  // Save to EEPROM
  EEPROM.put(10, cal.x);
  EEPROM.put(15, cal.y);
  
  // Calibrate MS5611 (record new ground pressure)
  pressureSmooth.clear();
  for (int i = 0; i < 50; i++) {
    ms5611.read();
    pressureSmooth.add(ms5611.getPressure());
    delay(20);
  }
  groundPressure = pressureSmooth.get();
  
  // Apply calibration
  gyro.setCalibration(cal);
  
  // Confirmation beep
  digitalWrite(PIN_LED, LOW);
  delay(200);
  tone(PIN_BUZZER, 2200, 200);
  digitalWrite(PIN_LED, HIGH);
  delay(200);
  digitalWrite(PIN_LED, LOW);
  
  calibrationDone = true;
  Serial.println(F("Calibration complete"));
  Serial.print(F("Cal X: ")); Serial.print(cal.x);
  Serial.print(F(" Y: ")); Serial.println(cal.y);
}

// ==================== SMOOTH MOTOR START ====================
void smoothMotorStart() {
  // Gradually increase motor PWM
  smoothStartPWM += 2;
  
  if (smoothStartPWM >= PWM_ARMED_MIN + 50) {
    // Smooth start complete
    smoothStartActive = false;
    Serial.println(F("Smooth start complete - Ready to fly"));
    beepConfirm(3);
    return;
  }
  
  // Write to all motors equally
  escFL.writeMicroseconds(smoothStartPWM);
  escFR.writeMicroseconds(smoothStartPWM);
  escRL.writeMicroseconds(smoothStartPWM);
  escRR.writeMicroseconds(smoothStartPWM);
}

// ==================== PID CALCULATIONS ====================
void calculatePID() {
  if (!systemArmed) {
    PID[0] = {0, 0, 0};
    PID[1] = {0, 0, 0};
    PID[2] = {0, 0, 0};
    prevError = {0, 0, 0};
    return;
  }
  
  // P term
  PID[0].x = gyro.error.x * KP;
  PID[0].y = gyro.error.y * KP;
  PID[0].z = gyro.error.z * KP_Z;
  
  // I term
  PID[1].x += gyro.error.x * loopPeriod * KI;
  PID[1].y += gyro.error.y * loopPeriod * KI;
  PID[1].z += gyro.error.z * loopPeriod * KI;
  
  // D term
  PID[2].x = KD * (gyro.error.x - prevError.x) / loopPeriod;
  PID[2].y = KD * (gyro.error.y - prevError.y) / loopPeriod;
  PID[2].z = KD * (gyro.error.z - prevError.z) / loopPeriod;
  
  prevError = gyro.error;
}

void calculateMotorOutputs() {
  int baseThrust = thrust;
  
  // Apply altitude hold PID output if active
  if (altitudeHoldActive && thrust > 1400 && thrust < 1450) {
    calculateAltitudePID();
    baseThrust = 1450 + (int)pidAltOutput + manualThrottle;
  }
  
  // Calculate individual motor outputs
  // Motor mixing for X configuration:
  // FL: -roll, +pitch, -yaw (CCW)
  // FR: +roll, +pitch, +yaw (CW)
  // RL: -roll, -pitch, +yaw (CW)
  // RR: +roll, -pitch, -yaw (CCW)
  
  float pidX = PID[0].x + PID[1].x + PID[2].x;
  float pidY = PID[0].y + PID[1].y + PID[2].y;
  float pidZ = PID[0].z + PID[2].z;  // No I term for yaw typically
  
  motorFL = baseThrust - pidX + pidY - pidZ;
  motorFR = baseThrust + pidX + pidY + pidZ;
  motorRL = baseThrust - pidX - pidY + pidZ;
  motorRR = baseThrust + pidX - pidY - pidZ;
}

// ==================== ALTITUDE HOLD ====================
void calculatePressure() {
  pressureReadCounter++;
  if (pressureReadCounter >= 10) {
    ms5611.read();
    pressureSmooth.add(ms5611.getPressure());
    pressureReadCounter = 0;
  }
  
  currentPressure = pressureSmooth.get();
  quadProps.baroHeight = currentPressure;
  
  updateKalman();
  
  // Convert pressure to approximate altitude (simplified)
  currentAltitude = (groundPressure - currentPressure) * 8.3;  // ~8.3m per hPa at sea level
}

void calculateAltitudePID() {
  float error = currentPressure - altitudeSetpoint;
  
  // Adaptive P gain
  float pGainMod = 0;
  if (abs(error) > 10) {
    pGainMod = min((abs(error) - 10) / 20.0, 3.0);
  }
  
  // I term accumulation
  pidAltIAccum += (pidAltI / 100.0) * error;
  pidAltIAccum = constrain(pidAltIAccum, -PID_ALT_MAX, PID_ALT_MAX);
  
  // D term
  float dTerm = pidAltD * (error - lastAltError);
  lastAltError = error;
  
  // Calculate output
  pidAltOutput = 100 * (pidAltP + pGainMod) * error + pidAltIAccum + dTerm;
  pidAltOutput = constrain(pidAltOutput, -PID_ALT_MAX, PID_ALT_MAX);
  
  // Manual throttle adjustment
  manualThrottle = 0;
  if (thrust > 1450) {
    manualThrottle = (thrust - 1450) / 3;
    altitudeSetpoint = currentPressure;
    pidAltIAccum = 0;
  } else if (thrust < 1400) {
    manualThrottle = (thrust - 1400) / 5;
    altitudeSetpoint = currentPressure;
    pidAltIAccum = 0;
  }
}

// ==================== KALMAN FILTER ====================
void initKalman() {
  kalmanProb.m11 = 1;
  kalmanProb.m12 = 0;
  kalmanProb.m21 = 0;
  kalmanProb.m22 = 1;
  quadProps.height = 0;
  quadProps.kalmanVelZ = 0;
}

void updateKalman() {
  const float dt = 0.007;  // ~140 Hz
  const float varAcc = 1.0;
  
  // Process noise
  float Q11 = varAcc * 0.25 * pow(dt, 4);
  float Q12 = varAcc * 0.5 * pow(dt, 3);
  float Q21 = Q12;
  float Q22 = varAcc * pow(dt, 2);
  
  // Measurement noise
  const float R11 = 0.008;
  
  // Predict
  float ps1 = quadProps.height + dt * quadProps.kalmanVelZ;
  float ps2 = quadProps.kalmanVelZ;
  
  float opt = dt * kalmanProb.m22;
  float pp12 = kalmanProb.m12 + opt + Q12;
  float pp21 = kalmanProb.m21 + opt;
  float pp11 = kalmanProb.m11 + dt * (kalmanProb.m12 + pp21) + Q11;
  pp21 += Q21;
  float pp22 = kalmanProb.m22 + Q22;
  
  // Update
  float inn = quadProps.baroHeight - ps1;
  float ic = pp11 + R11;
  
  float kg1 = pp11 / ic;
  float kg2 = pp21 / ic;
  
  quadProps.height = ps1 + kg1 * inn;
  quadProps.kalmanVelZ = ps2 + kg2 * inn;
  
  opt = 1 - kg1;
  kalmanProb.m11 = pp11 * opt;
  kalmanProb.m12 = pp12 * opt;
  kalmanProb.m21 = pp21 - pp11 * kg2;
  kalmanProb.m22 = pp22 - pp12 * kg2;
}

// ==================== MOTOR CONTROL ====================
void runMotors() {
  int minPWM = systemArmed ? PWM_ARMED_MIN : PWM_MIN;
  
  // Constrain motor outputs
  motorFL = constrain(motorFL, minPWM, PWM_MAX);
  motorFR = constrain(motorFR, minPWM, PWM_MAX);
  motorRL = constrain(motorRL, minPWM, PWM_MAX);
  motorRR = constrain(motorRR, minPWM, PWM_MAX);
  
  if (systemArmed) {
    escFL.writeMicroseconds(motorFL);
    escFR.writeMicroseconds(motorFR);
    escRL.writeMicroseconds(motorRL);
    escRR.writeMicroseconds(motorRR);
  } else {
    stopMotors();
  }
}

void stopMotors() {
  escFL.writeMicroseconds(PWM_MIN);
  escFR.writeMicroseconds(PWM_MIN);
  escRL.writeMicroseconds(PWM_MIN);
  escRR.writeMicroseconds(PWM_MIN);
  
  motorFL = PWM_MIN;
  motorFR = PWM_MIN;
  motorRL = PWM_MIN;
  motorRR = PWM_MIN;
}

// ==================== UTILITIES ====================
float readBatteryVoltage() {
  int raw = analogRead(PIN_BATTERY);
  float vout = (raw * 5.0) / 1023.0;
  return vout / (R2 / (R1 + R2));
}

void updateLED() {
  bool armSwitch = digitalRead(PIN_ARM_SWITCH);
  
  if (armSwitch == HIGH) {
    // Disarmed - LED ON continuously
    digitalWrite(PIN_LED, HIGH);
  } else if (nrfConnected) {
    // Armed and connected - blink on data received
    if (millis() - lastLedBlink > 100) {
      ledState = !ledState;
      digitalWrite(PIN_LED, ledState);
      lastLedBlink = millis();
    }
  } else {
    // Armed but no connection - fast blink
    if (millis() - lastLedBlink > 50) {
      ledState = !ledState;
      digitalWrite(PIN_LED, ledState);
      lastLedBlink = millis();
    }
  }
}

void beepConfirm(int pattern) {
  switch (pattern) {
    case 1:  // Short low
      tone(PIN_BUZZER, 1000, 300);
      digitalWrite(PIN_LED, HIGH);
      delay(300);
      digitalWrite(PIN_LED, LOW);
      break;
    case 2:  // Medium mid
      tone(PIN_BUZZER, 1600, 500);
      digitalWrite(PIN_LED, HIGH);
      delay(500);
      digitalWrite(PIN_LED, LOW);
      break;
    case 3:  // Short high
      tone(PIN_BUZZER, 2000, 200);
      digitalWrite(PIN_LED, HIGH);
      delay(200);
      digitalWrite(PIN_LED, LOW);
      break;
    case 4:  // Warning - repeated
      for (int i = 0; i < 3; i++) {
        tone(PIN_BUZZER, 1000, 150);
        digitalWrite(PIN_LED, HIGH);
        delay(150);
        digitalWrite(PIN_LED, LOW);
        delay(100);
      }
      break;
  }
}

void resetYaw() {
  gyro.zeroYaw(true);
  target.z = 0;
}

void printDebug() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 200) return;
  lastPrint = millis();
  
  Serial.print(F("A:"));
  Serial.print(systemArmed ? 1 : 0);
  Serial.print(F(" H:"));
  Serial.print(altitudeHoldActive ? 1 : 0);
  Serial.print(F(" T:"));
  Serial.print(thrust);
  Serial.print(F(" M:"));
  Serial.print(motorFL);
  Serial.print(F(","));
  Serial.print(motorFR);
  Serial.print(F(","));
  Serial.print(motorRL);
  Serial.print(F(","));
  Serial.print(motorRR);
  Serial.print(F(" E:"));
  Serial.print((int)gyro.error.x);
  Serial.print(F(","));
  Serial.print((int)gyro.error.y);
  Serial.print(F(","));
  Serial.print((int)gyro.error.z);
  Serial.print(F(" Alt:"));
  Serial.println(currentAltitude);
}
