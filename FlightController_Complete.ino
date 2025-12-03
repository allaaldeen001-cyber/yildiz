/*
 * ============================================================================
 * PROFESSIONAL QUADCOPTER FLIGHT CONTROLLER
 * ============================================================================
 * Target: Arduino Nano (ATmega328P)
 * Motors: RS2205 2300KV
 * 
 * WIRING TABLE:
 * ============================================================================
 * SENSOR/DEVICE        | PIN    | NOTES
 * ---------------------|--------|----------------------------------------
 * MPU6050 SDA          | A4     | I2C Data
 * MPU6050 SCL          | A5     | I2C Clock
 * MPU6050 INT          | D2     | Interrupt (optional)
 * MS5611 SDA           | A4     | I2C Data (shared)
 * MS5611 SCL           | A5     | I2C Clock (shared)
 * NRF24L01 CE          | D8     | Radio Chip Enable
 * NRF24L01 CSN         | D10    | Radio Chip Select
 * NRF24L01 MOSI        | D11    | SPI
 * NRF24L01 MISO        | D12    | SPI
 * NRF24L01 SCK         | D13    | SPI
 * ESC Front Left       | D3     | PWM Motor Output
 * ESC Front Right      | D5     | PWM Motor Output
 * ESC Rear Left        | D9     | PWM Motor Output
 * ESC Rear Right       | D6     | PWM Motor Output
 * Button 1 (Calibrate) | D4     | Active LOW (Internal Pullup)
 * Button 2 (Motor Test)| A0     | Active LOW (Internal Pullup)
 * Button 3 (Takeoff)   | A1     | Active LOW (Internal Pullup)
 * Button 4 (Landing)   | A2     | Active LOW (Internal Pullup)
 * Potentiometer 1 (P)  | A6     | PID P-Gain tuning
 * Potentiometer 2 (D)  | A7     | PID D-Gain tuning
 * Buzzer               | D7     | Active Buzzer
 * LED Status           | Built-in LED (D13) or external
 * 
 * LIBRARIES REQUIRED:
 * - Wire (built-in)
 * - SPI (built-in)
 * - Servo (built-in)
 * - EEPROM (built-in)
 * - RF24 (https://github.com/nRF24/RF24)
 * - MS5611 (https://github.com/jarzebski/Arduino-MS5611)
 * 
 * ============================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <EEPROM.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "MS5611.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define ESC_FL_PIN          3
#define ESC_FR_PIN          5
#define ESC_RL_PIN          9
#define ESC_RR_PIN          6

#define BUTTON_CALIBRATE    4
#define BUTTON_MOTOR_TEST   A0
#define BUTTON_TAKEOFF      A1
#define BUTTON_LANDING      A2

#define POT_P_GAIN          A6
#define POT_D_GAIN          A7

#define BUZZER_PIN          7
#define LED_PIN             LED_BUILTIN

#define NRF_CE_PIN          8
#define NRF_CSN_PIN         10

#define MPU6050_ADDR        0x68
#define MPU6050_INT_PIN     2

// ============================================================================
// CONFIGURATION PARAMETERS
// ============================================================================
#define LOOP_FREQUENCY      250       // Hz (4ms loop time)
#define LOOP_TIME_US        4000      // microseconds

// Motor PWM limits
#define MOTOR_MIN           1000
#define MOTOR_MAX           2000
#define MOTOR_ARMED_IDLE    1050
#define MOTOR_HOVER_BASE    1450      // Approximate hover throttle
#define MOTOR_TAKEOFF       1600      // Takeoff throttle

// PID limits
#define PID_ROLL_PITCH_MAX  400
#define PID_YAW_MAX         200
#define PID_ALT_MAX         300

// Safety limits
#define MAX_TILT_ANGLE      45.0      // degrees
#define MAX_LANDING_TILT    15.0      // degrees during landing
#define MAX_DESCENT_RATE    0.5       // m/s
#define TOUCHDOWN_THRESHOLD 0.05      // m above ground
#define TOUCHDOWN_VELOCITY  0.1       // m/s
#define FAILSAFE_TIMEOUT    1000      // ms

// Sensor fusion
#define GYRO_COMP_WEIGHT    0.96
#define ACCEL_COMP_WEIGHT   0.04

// EEPROM addresses
#define EEPROM_ADDR_CAL_ROLL    10
#define EEPROM_ADDR_CAL_PITCH   14
#define EEPROM_ADDR_GROUND_PRES 18
#define EEPROM_ADDR_MAGIC       22    // Magic number to check if calibrated

#define EEPROM_MAGIC_NUMBER     0xABCD

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Radio packet from transmitter
struct RadioPacket {
  uint16_t throttle;    // 1000-2000
  int16_t  roll;        // -500 to +500
  int16_t  pitch;       // -500 to +500
  int16_t  yaw;         // -500 to +500
  uint8_t  armSwitch;   // 0=disarmed, 1=armed
  uint8_t  modeSwitch;  // 0=stabilize, 1=altitude hold
  uint32_t timestamp;   // For timeout detection
};

// Radio telemetry back to transmitter
struct TelemetryPacket {
  float    roll;        // degrees
  float    pitch;       // degrees
  float    yaw;         // degrees
  float    altitude;    // meters
  float    battery;     // volts
  uint8_t  flightMode;  // 0=disarmed, 1=stabilize, 2=alt_hold, 3=landing
  uint16_t loopTime;    // microseconds
};

// IMU data
struct IMU_Data {
  float roll;           // degrees
  float pitch;          // degrees
  float yaw;            // degrees
  float gyroX;          // deg/s
  float gyroY;          // deg/s
  float gyroZ;          // deg/s
  float accelX;         // g
  float accelY;         // g
  float accelZ;         // g
};

// PID controller
struct PID_Controller {
  float kp;
  float ki;
  float kd;
  float error;
  float lastError;
  float integral;
  float derivative;
  float output;
  float outputMax;
};

// Flight state machine
enum FlightState {
  STATE_DISARMED,
  STATE_ARMED_IDLE,
  STATE_STABILIZE,
  STATE_ALTITUDE_HOLD,
  STATE_TAKEOFF,
  STATE_LANDING,
  STATE_EMERGENCY
};

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const uint64_t radioAddress = 0xF0F0F0F0E1LL;

MS5611 barometer(0x77);

Servo escFL, escFR, escRL, escRR;

RadioPacket rxData = {1000, 0, 0, 0, 0, 0, 0};
TelemetryPacket txData = {0, 0, 0, 0, 0, 0, 0};

IMU_Data imu = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// PID controllers
PID_Controller pidRoll   = {2.0, 0.001, 15.0, 0, 0, 0, 0, 0, PID_ROLL_PITCH_MAX};
PID_Controller pidPitch  = {2.0, 0.001, 15.0, 0, 0, 0, 0, 0, PID_ROLL_PITCH_MAX};
PID_Controller pidYaw    = {3.0, 0.001, 0.5, 0, 0, 0, 0, 0, PID_YAW_MAX};
PID_Controller pidAlt    = {2.0, 0.5, 1.5, 0, 0, 0, 0, 0, PID_ALT_MAX};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
FlightState currentState = STATE_DISARMED;
FlightState lastState = STATE_DISARMED;

// IMU calibration offsets
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accelOffsetX = 0, accelOffsetY = 0;
float rollOffset = 0, pitchOffset = 0;

// Barometer
float groundPressure = 1013.25;     // Default sea level
float currentPressure = 1013.25;
float currentAltitude = 0.0;
float targetAltitude = 0.0;
float takeoffAltitude = 0.0;
float lastAltitude = 0.0;
float verticalVelocity = 0.0;
bool barometerValid = false;

// Motor outputs
int motorFL = MOTOR_MIN;
int motorFR = MOTOR_MIN;
int motorRL = MOTOR_MIN;
int motorRR = MOTOR_MIN;

// Timing
unsigned long lastLoopTime = 0;
unsigned long currentTime = 0;
unsigned long lastRadioRx = 0;
unsigned long lastBaroRead = 0;
float deltaTime = 0.004;  // 4ms nominal

// Calibration and testing
bool calibrationComplete = false;
bool motorTestActive = false;
uint8_t motorTestStep = 0;
unsigned long motorTestTimer = 0;

// Landing state machine
enum LandingPhase {
  LANDING_DESCEND,
  LANDING_DETECT_TOUCHDOWN,
  LANDING_SETTLE,
  LANDING_COMPLETE
};
LandingPhase landingPhase = LANDING_DESCEND;
float landingStartAltitude = 0.0;
unsigned long landingStartTime = 0;
bool touchdownDetected = false;

// Takeoff state machine
bool takeoffActive = false;
float takeoffTargetAltitude = 0.0;
unsigned long takeoffStartTime = 0;

// Button debouncing
unsigned long lastButtonPress[4] = {0, 0, 0, 0};
#define BUTTON_DEBOUNCE_MS 200

// Safety
bool failsafeActive = false;
bool emergencyStop = false;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void initSensors();
void initRadio();
void initMotors();
void calibrateIMU();
void calibrateBarometer();
void readIMU();
void readBarometer();
void updateSensorFusion();
void computePID();
void mixerCalculate();
void updateMotors();
void checkButtons();
void checkFailsafe();
void runStateMachine();
void motorTest();
void executeStabilize();
void executeAltitudeHold();
void executeTakeoff();
void executeLanding();
void emergencyShutdown();
void beep(int frequency, int duration);
void serialDebug();
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
float constrainFloat(float value, float min_val, float max_val);

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_CALIBRATE, INPUT_PULLUP);
  pinMode(BUTTON_MOTOR_TEST, INPUT_PULLUP);
  pinMode(BUTTON_TAKEOFF, INPUT_PULLUP);
  pinMode(BUTTON_LANDING, INPUT_PULLUP);
  
  Serial.println(F("============================================"));
  Serial.println(F("  QUADCOPTER FLIGHT CONTROLLER v2.0"));
  Serial.println(F("============================================"));
  
  // Startup beep sequence - pleasant melody
  beep(523, 150);  // C5
  delay(80);
  beep(659, 150);  // E5
  delay(80);
  beep(784, 200);  // G5
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz fast mode
  
  // Initialize sensors
  initSensors();
  
  // Initialize radio
  initRadio();
  
  // Initialize motors
  initMotors();
  
  // Load calibration from EEPROM
  uint16_t magic;
  EEPROM.get(EEPROM_ADDR_MAGIC, magic);
  if (magic == EEPROM_MAGIC_NUMBER) {
    EEPROM.get(EEPROM_ADDR_CAL_ROLL, rollOffset);
    EEPROM.get(EEPROM_ADDR_CAL_PITCH, pitchOffset);
    EEPROM.get(EEPROM_ADDR_GROUND_PRES, groundPressure);
    calibrationComplete = true;
    Serial.println(F("Calibration loaded from EEPROM"));
  } else {
    Serial.println(F("No calibration found. Press Button 1 to calibrate."));
  }
  
  currentState = STATE_DISARMED;
  
  Serial.println(F("System Ready"));
  Serial.println(F("============================================"));
  Serial.println(F("Button 1 (D4): Calibrate Sensors"));
  Serial.println(F("Button 2 (A0): Motor Direction Test"));
  Serial.println(F("Button 3 (A1): Auto Takeoff"));
  Serial.println(F("Button 4 (A2): Auto Landing"));
  Serial.println(F("Pot 1 (A6): P-Gain Tuning"));
  Serial.println(F("Pot 2 (A7): D-Gain Tuning"));
  Serial.println(F("============================================"));
  
  beep(880, 250);  // A5 - ready tone
  
  lastLoopTime = micros();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  currentTime = micros();
  
  // Wait for loop time
  if (currentTime - lastLoopTime < LOOP_TIME_US) {
    return;
  }
  
  deltaTime = (currentTime - lastLoopTime) * 0.000001; // Convert to seconds
  lastLoopTime = currentTime;
  
  // Read sensors
  readIMU();
  
  // Read barometer at lower rate (50Hz)
  if (currentTime - lastBaroRead > 20000) {
    readBarometer();
    lastBaroRead = currentTime;
  }
  
  // Update sensor fusion
  updateSensorFusion();
  
  // Check buttons
  checkButtons();
  
  // Check radio and failsafe
  if (radio.available()) {
    radio.read(&rxData, sizeof(RadioPacket));
    lastRadioRx = millis();
    failsafeActive = false;
    
    // Prepare and write ACK payload with telemetry
    txData.roll = imu.roll;
    txData.pitch = imu.pitch;
    txData.yaw = imu.yaw;
    txData.altitude = currentAltitude;
    txData.battery = 11.1; // TODO: Read actual voltage
    txData.flightMode = currentState;
    txData.loopTime = deltaTime * 1000000;
    
    // Write ACK payload for next transmission
    radio.writeAckPayload(1, &txData, sizeof(TelemetryPacket));
  }
  checkFailsafe();
  
  // Run state machine
  runStateMachine();
  
  // Compute PID (if armed)
  if (currentState != STATE_DISARMED) {
    computePID();
    mixerCalculate();
  } else {
    motorFL = motorFR = motorRL = motorRR = MOTOR_MIN;
  }
  
  // Update motors
  updateMotors();
  
  // Serial debug (every 100ms)
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 100) {
    serialDebug();
    lastDebug = millis();
  }
}

// ============================================================================
// SENSOR INITIALIZATION
// ============================================================================
void initSensors() {
  Serial.println(F("Initializing MPU6050..."));
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Wake up
  if (Wire.endTransmission() != 0) {
    Serial.println(F("ERROR: MPU6050 not found!"));
    beep(500, 1000);
    while(1);
  }
  
  // Configure gyro: ±500 deg/s (FS_SEL=1)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); // GYRO_CONFIG
  Wire.write(0x08); // FS_SEL=1
  Wire.endTransmission();
  
  // Configure accel: ±4g (AFS_SEL=1)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); // ACCEL_CONFIG
  Wire.write(0x08); // AFS_SEL=1
  Wire.endTransmission();
  
  // Configure DLPF: 98Hz bandwidth
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1A); // CONFIG
  Wire.write(0x02); // DLPF_CFG=2
  Wire.endTransmission();
  
  Serial.println(F("MPU6050 initialized"));
  
  // Initialize MS5611
  Serial.println(F("Initializing MS5611..."));
  if (!barometer.begin()) {
    Serial.println(F("WARNING: MS5611 not found!"));
    Serial.println(F("Altitude hold will be disabled."));
    barometerValid = false;
    beep(800, 500);
  } else {
    barometer.setOversampling(OSR_STANDARD);
    barometerValid = true;
    Serial.println(F("MS5611 initialized"));
  }
  
  delay(100);
}

// ============================================================================
// RADIO INITIALIZATION
// ============================================================================
void initRadio() {
  Serial.println(F("Initializing NRF24L01..."));
  
  if (!radio.begin()) {
    Serial.println(F("ERROR: NRF24L01 not found!"));
    beep(500, 1000);
    while(1);
  }
  
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(103);
  radio.setRetries(3, 5);
  radio.openReadingPipe(1, radioAddress);
  radio.startListening();
  
  Serial.println(F("NRF24L01 initialized (Channel 103)"));
  Serial.println(F("ACK payloads enabled for telemetry"));
}

// ============================================================================
// MOTOR INITIALIZATION
// ============================================================================
void initMotors() {
  Serial.println(F("Initializing ESCs..."));
  
  escFL.attach(ESC_FL_PIN, MOTOR_MIN, MOTOR_MAX);
  escFR.attach(ESC_FR_PIN, MOTOR_MIN, MOTOR_MAX);
  escRL.attach(ESC_RL_PIN, MOTOR_MIN, MOTOR_MAX);
  escRR.attach(ESC_RR_PIN, MOTOR_MIN, MOTOR_MAX);
  
  // Send minimum signal
  escFL.writeMicroseconds(MOTOR_MIN);
  escFR.writeMicroseconds(MOTOR_MIN);
  escRL.writeMicroseconds(MOTOR_MIN);
  escRR.writeMicroseconds(MOTOR_MIN);
  
  delay(2000); // ESC initialization time
  
  Serial.println(F("ESCs armed and ready"));
}

// ============================================================================
// IMU CALIBRATION
// ============================================================================
void calibrateIMU() {
  Serial.println(F("Calibrating IMU..."));
  Serial.println(F("Keep drone level and still!"));
  
  beep(440, 200);  // A4 - calm tone
  delay(1000);
  
  float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
  float sumAccelX = 0, sumAccelY = 0;
  int samples = 1000;
  
  for (int i = 0; i < samples; i++) {
    readIMU();
    
    sumGyroX += imu.gyroX;
    sumGyroY += imu.gyroY;
    sumGyroZ += imu.gyroZ;
    sumAccelX += imu.accelX;
    sumAccelY += imu.accelY;
    
    if (i % 100 == 0) {
      Serial.print(F("."));
    }
    
    delay(3);
  }
  
  gyroOffsetX = sumGyroX / samples;
  gyroOffsetY = sumGyroY / samples;
  gyroOffsetZ = sumGyroZ / samples;
  accelOffsetX = sumAccelX / samples;
  accelOffsetY = sumAccelY / samples;
  
  // Calculate level offsets
  float accelAngleX = atan2(accelOffsetY, sqrt(accelOffsetX * accelOffsetX + 1.0)) * 180.0 / PI;
  float accelAngleY = atan2(-accelOffsetX, sqrt(accelOffsetY * accelOffsetY + 1.0)) * 180.0 / PI;
  
  rollOffset = accelAngleX;
  pitchOffset = accelAngleY;
  
  Serial.println();
  Serial.println(F("IMU calibration complete"));
  Serial.print(F("Roll offset: ")); Serial.println(rollOffset);
  Serial.print(F("Pitch offset: ")); Serial.println(pitchOffset);
  
  beep(659, 200);  // E5
  delay(100);
  beep(784, 200);  // G5
}

// ============================================================================
// BAROMETER CALIBRATION
// ============================================================================
void calibrateBarometer() {
  if (!barometerValid) {
    Serial.println(F("Barometer not available"));
    return;
  }
  
  Serial.println(F("Calibrating barometer (ground reference)..."));
  
  float sumPressure = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    barometer.read();
    sumPressure += barometer.getPressure();
    delay(10);
  }
  
  groundPressure = sumPressure / samples;
  
  Serial.print(F("Ground pressure: "));
  Serial.print(groundPressure);
  Serial.println(F(" hPa"));
  
  // Save to EEPROM
  EEPROM.put(EEPROM_ADDR_CAL_ROLL, rollOffset);
  EEPROM.put(EEPROM_ADDR_CAL_PITCH, pitchOffset);
  EEPROM.put(EEPROM_ADDR_GROUND_PRES, groundPressure);
  EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_NUMBER);
  
  calibrationComplete = true;
  
  Serial.println(F("Calibration saved to EEPROM"));
  beep(880, 300);  // A5 - success tone
}

// ============================================================================
// READ IMU
// ============================================================================
void readIMU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Start at ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  int16_t temp = Wire.read() << 8 | Wire.read();
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();
  
  // Convert to physical units
  // Gyro: ±500 deg/s → 65.5 LSB/(deg/s)
  imu.gyroX = (gx / 65.5) - gyroOffsetX;
  imu.gyroY = (gy / 65.5) - gyroOffsetY;
  imu.gyroZ = (gz / 65.5) - gyroOffsetZ;
  
  // Accel: ±4g → 8192 LSB/g
  imu.accelX = ax / 8192.0;
  imu.accelY = ay / 8192.0;
  imu.accelZ = az / 8192.0;
}

// ============================================================================
// READ BAROMETER
// ============================================================================
void readBarometer() {
  if (!barometerValid) return;
  
  barometer.read();
  currentPressure = barometer.getPressure();
  
  // Check for valid data
  if (currentPressure < 800 || currentPressure > 1200) {
    Serial.println(F("WARNING: Invalid barometer reading"));
    barometerValid = false;
    return;
  }
  
  // Calculate altitude (barometric formula)
  float altitudeRaw = 44330.0 * (1.0 - pow(currentPressure / groundPressure, 0.1903));
  
  // Simple low-pass filter
  currentAltitude = 0.8 * currentAltitude + 0.2 * altitudeRaw;
  
  // Calculate vertical velocity
  verticalVelocity = (currentAltitude - lastAltitude) / (deltaTime * 5.0); // 50Hz update
  lastAltitude = currentAltitude;
}

// ============================================================================
// SENSOR FUSION (Complementary Filter)
// ============================================================================
void updateSensorFusion() {
  // Integrate gyro
  imu.roll += imu.gyroX * deltaTime;
  imu.pitch += imu.gyroY * deltaTime;
  imu.yaw += imu.gyroZ * deltaTime;
  
  // Compensate for yaw rotation on roll/pitch
  imu.roll += imu.pitch * sin(imu.gyroZ * deltaTime * DEG_TO_RAD);
  imu.pitch -= imu.roll * sin(imu.gyroZ * deltaTime * DEG_TO_RAD);
  
  // Calculate accelerometer angles
  float accelTotalVector = sqrt(imu.accelX * imu.accelX + imu.accelY * imu.accelY + imu.accelZ * imu.accelZ);
  
  // Only fuse accelerometer if total acceleration is close to 1g (not during rapid movement)
  if (accelTotalVector > 0.8 && accelTotalVector < 1.2) {
    float accelAngleX = atan2(imu.accelY, sqrt(imu.accelX * imu.accelX + imu.accelZ * imu.accelZ)) * 180.0 / PI;
    float accelAngleY = atan2(-imu.accelX, sqrt(imu.accelY * imu.accelY + imu.accelZ * imu.accelZ)) * 180.0 / PI;
    
    // Complementary filter: 96% gyro, 4% accel
    imu.roll = GYRO_COMP_WEIGHT * imu.roll + ACCEL_COMP_WEIGHT * accelAngleX;
    imu.pitch = GYRO_COMP_WEIGHT * imu.pitch + ACCEL_COMP_WEIGHT * accelAngleY;
  }
  
  // Apply calibration offsets
  imu.roll -= rollOffset;
  imu.pitch -= pitchOffset;
  
  // Keep yaw in -180 to 180 range
  if (imu.yaw > 180) imu.yaw -= 360;
  if (imu.yaw < -180) imu.yaw += 360;
}

// ============================================================================
// PID COMPUTATION
// ============================================================================
float computeSinglePID(PID_Controller &pid, float setpoint, float measurement, float dt) {
  pid.error = setpoint - measurement;
  
  // Proportional
  float pTerm = pid.kp * pid.error;
  
  // Integral (with anti-windup)
  pid.integral += pid.error * dt;
  pid.integral = constrainFloat(pid.integral, -pid.outputMax / pid.ki, pid.outputMax / pid.ki);
  float iTerm = pid.ki * pid.integral;
  
  // Derivative (on measurement to avoid derivative kick)
  pid.derivative = (measurement - pid.lastError) / dt;
  float dTerm = pid.kd * pid.derivative;
  
  pid.lastError = measurement;
  
  // Sum and limit
  pid.output = pTerm + iTerm - dTerm; // Note: -dTerm because derivative is on measurement
  pid.output = constrainFloat(pid.output, -pid.outputMax, pid.outputMax);
  
  return pid.output;
}

void computePID() {
  // Read potentiometers for real-time tuning
  float pGain = mapFloat(analogRead(POT_P_GAIN), 0, 1023, 0.5, 5.0);
  float dGain = mapFloat(analogRead(POT_D_GAIN), 0, 1023, 5.0, 30.0);
  
  pidRoll.kp = pidPitch.kp = pGain;
  pidRoll.kd = pidPitch.kd = dGain;
  
  // Roll and Pitch PID
  float rollSetpoint = mapFloat(rxData.roll, -500, 500, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
  float pitchSetpoint = mapFloat(rxData.pitch, -500, 500, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
  
  // Limit tilt during landing
  if (currentState == STATE_LANDING) {
    rollSetpoint = constrainFloat(rollSetpoint, -MAX_LANDING_TILT, MAX_LANDING_TILT);
    pitchSetpoint = constrainFloat(pitchSetpoint, -MAX_LANDING_TILT, MAX_LANDING_TILT);
  }
  
  computeSinglePID(pidRoll, rollSetpoint, imu.roll, deltaTime);
  computeSinglePID(pidPitch, pitchSetpoint, imu.pitch, deltaTime);
  
  // Yaw PID (rate mode)
  float yawRate = mapFloat(rxData.yaw, -500, 500, -180, 180);
  computeSinglePID(pidYaw, yawRate, imu.gyroZ, deltaTime);
}

// ============================================================================
// MOTOR MIXER
// ============================================================================
void mixerCalculate() {
  int throttle = rxData.throttle;
  
  // Override throttle for specific states
  if (currentState == STATE_ARMED_IDLE) {
    throttle = MOTOR_ARMED_IDLE;
  } else if (currentState == STATE_TAKEOFF) {
    throttle = MOTOR_TAKEOFF;
  } else if (currentState == STATE_ALTITUDE_HOLD || currentState == STATE_LANDING) {
    // Use PID output
    float altError = targetAltitude - currentAltitude;
    computeSinglePID(pidAlt, targetAltitude, currentAltitude, deltaTime * 5.0);
    throttle = MOTOR_HOVER_BASE + (int)pidAlt.output;
    throttle = constrain(throttle, MOTOR_ARMED_IDLE, MOTOR_MAX);
  }
  
  // Quadcopter X configuration mixer
  // FL: -roll, +pitch, +yaw
  // FR: +roll, +pitch, -yaw
  // RL: -roll, -pitch, -yaw
  // RR: +roll, -pitch, +yaw
  
  motorFL = throttle - pidRoll.output + pidPitch.output + pidYaw.output;
  motorFR = throttle + pidRoll.output + pidPitch.output - pidYaw.output;
  motorRL = throttle - pidRoll.output - pidPitch.output - pidYaw.output;
  motorRR = throttle + pidRoll.output - pidPitch.output + pidYaw.output;
  
  // Constrain to valid range
  motorFL = constrain(motorFL, MOTOR_ARMED_IDLE, MOTOR_MAX);
  motorFR = constrain(motorFR, MOTOR_ARMED_IDLE, MOTOR_MAX);
  motorRL = constrain(motorRL, MOTOR_ARMED_IDLE, MOTOR_MAX);
  motorRR = constrain(motorRR, MOTOR_ARMED_IDLE, MOTOR_MAX);
}

// ============================================================================
// UPDATE MOTORS
// ============================================================================
void updateMotors() {
  if (currentState == STATE_DISARMED || emergencyStop) {
    escFL.writeMicroseconds(MOTOR_MIN);
    escFR.writeMicroseconds(MOTOR_MIN);
    escRL.writeMicroseconds(MOTOR_MIN);
    escRR.writeMicroseconds(MOTOR_MIN);
  } else {
    escFL.writeMicroseconds(motorFL);
    escFR.writeMicroseconds(motorFR);
    escRL.writeMicroseconds(motorRL);
    escRR.writeMicroseconds(motorRR);
  }
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================
void checkButtons() {
  unsigned long now = millis();
  
  // Button 1: Calibration
  if (digitalRead(BUTTON_CALIBRATE) == LOW && now - lastButtonPress[0] > BUTTON_DEBOUNCE_MS) {
    lastButtonPress[0] = now;
    
    if (currentState == STATE_DISARMED) {
      Serial.println(F("Starting calibration..."));
      beep(523, 150);  // C5
      calibrateIMU();
      calibrateBarometer();
      Serial.println(F("Calibration complete!"));
      beep(880, 300);  // A5 - success
    } else {
      Serial.println(F("Cannot calibrate while armed!"));
      beep(330, 200);  // E4 - warning
    }
  }
  
  // Button 2: Motor test
  if (digitalRead(BUTTON_MOTOR_TEST) == LOW && now - lastButtonPress[1] > BUTTON_DEBOUNCE_MS) {
    lastButtonPress[1] = now;
    
    if (currentState == STATE_DISARMED && !motorTestActive) {
      Serial.println(F("Starting motor direction test..."));
      motorTestActive = true;
      motorTestStep = 0;
      motorTestTimer = now;
      beep(659, 150);  // E5
    }
  }
  
  // Button 3: Takeoff
  if (digitalRead(BUTTON_TAKEOFF) == LOW && now - lastButtonPress[2] > BUTTON_DEBOUNCE_MS) {
    lastButtonPress[2] = now;
    
    if (currentState == STATE_STABILIZE || currentState == STATE_ALTITUDE_HOLD) {
      if (barometerValid && calibrationComplete) {
        Serial.println(F("Initiating auto takeoff..."));
        currentState = STATE_TAKEOFF;
        takeoffStartTime = now;
        takeoffTargetAltitude = currentAltitude + 1.0; // 1 meter takeoff
        targetAltitude = takeoffTargetAltitude;
        beep(784, 200);  // G5 - takeoff
      } else {
        Serial.println(F("Cannot takeoff: calibrate first or barometer unavailable"));
        beep(330, 300);  // E4 - warning
      }
    }
  }
  
  // Button 4: Landing
  if (digitalRead(BUTTON_LANDING) == LOW && now - lastButtonPress[3] > BUTTON_DEBOUNCE_MS) {
    lastButtonPress[3] = now;
    
    if (currentState == STATE_STABILIZE || currentState == STATE_ALTITUDE_HOLD || currentState == STATE_TAKEOFF) {
      Serial.println(F("Initiating auto landing..."));
      currentState = STATE_LANDING;
      landingStartTime = now;
      landingStartAltitude = currentAltitude;
      landingPhase = LANDING_DESCEND;
      touchdownDetected = false;
      beep(587, 200);  // D5 - landing
    }
  }
}

// ============================================================================
// MOTOR TEST SEQUENCE
// ============================================================================
void motorTest() {
  unsigned long now = millis();
  
  if (now - motorTestTimer > 2000) {
    motorTestStep++;
    motorTestTimer = now;
    
    if (motorTestStep > 4) {
      motorTestActive = false;
      motorTestStep = 0;
      Serial.println(F("Motor test complete"));
      beep(880, 250);  // A5 - complete
      return;
    }
  }
  
  // Spin each motor in sequence
  int testSpeed = 1200; // Low speed for safety
  
  escFL.writeMicroseconds(MOTOR_MIN);
  escFR.writeMicroseconds(MOTOR_MIN);
  escRL.writeMicroseconds(MOTOR_MIN);
  escRR.writeMicroseconds(MOTOR_MIN);
  
  switch(motorTestStep) {
    case 0:
      Serial.println(F("All motors OFF"));
      break;
    case 1:
      Serial.println(F("Front Left spinning"));
      escFL.writeMicroseconds(testSpeed);
      beep(523, 150);  // C5
      break;
    case 2:
      Serial.println(F("Front Right spinning"));
      escFR.writeMicroseconds(testSpeed);
      beep(587, 150);  // D5
      break;
    case 3:
      Serial.println(F("Rear Left spinning"));
      escRL.writeMicroseconds(testSpeed);
      beep(659, 150);  // E5
      break;
    case 4:
      Serial.println(F("Rear Right spinning"));
      escRR.writeMicroseconds(testSpeed);
      beep(698, 150);  // F5
      break;
  }
}

// ============================================================================
// STATE MACHINE
// ============================================================================
void runStateMachine() {
  // Motor test has priority when disarmed
  if (motorTestActive) {
    motorTest();
    return;
  }
  
  // Check arm switch
  if (rxData.armSwitch == 0 && currentState != STATE_DISARMED) {
    // Disarm command
    currentState = STATE_DISARMED;
    Serial.println(F("DISARMED"));
    beep(440, 250);  // A4 - disarmed
    
    // Reset PID integrators
    pidRoll.integral = 0;
    pidPitch.integral = 0;
    pidYaw.integral = 0;
    pidAlt.integral = 0;
    
    return;
  }
  
  if (rxData.armSwitch == 1 && currentState == STATE_DISARMED) {
    // Arm command
    if (calibrationComplete) {
      currentState = STATE_ARMED_IDLE;
      Serial.println(F("ARMED - IDLE"));
      beep(659, 150);  // E5
      delay(100);
      beep(784, 150);  // G5 - armed
      
      // Reset yaw reference
      imu.yaw = 0;
    } else {
      Serial.println(F("Cannot arm: calibration required!"));
      beep(330, 400);  // E4 - warning
      return;
    }
  }
  
  // State transitions
  switch(currentState) {
    case STATE_DISARMED:
      // Nothing to do
      break;
      
    case STATE_ARMED_IDLE:
      // Transition to flight mode based on throttle
      if (rxData.throttle > 1100) {
        if (rxData.modeSwitch == 1 && barometerValid) {
          currentState = STATE_ALTITUDE_HOLD;
          targetAltitude = currentAltitude;
          Serial.println(F("ALTITUDE HOLD MODE"));
          beep(880, 150);  // A5 - alt hold
        } else {
          currentState = STATE_STABILIZE;
          Serial.println(F("STABILIZE MODE"));
          beep(698, 150);  // F5 - stabilize
        }
      }
      break;
      
    case STATE_STABILIZE:
      executeStabilize();
      break;
      
    case STATE_ALTITUDE_HOLD:
      executeAltitudeHold();
      break;
      
    case STATE_TAKEOFF:
      executeTakeoff();
      break;
      
    case STATE_LANDING:
      executeLanding();
      break;
      
    case STATE_EMERGENCY:
      emergencyShutdown();
      break;
  }
  
  // Safety checks
  if (abs(imu.roll) > MAX_TILT_ANGLE || abs(imu.pitch) > MAX_TILT_ANGLE) {
    if (currentState != STATE_LANDING && currentState != STATE_DISARMED) {
      Serial.println(F("EMERGENCY: Excessive tilt!"));
      currentState = STATE_EMERGENCY;
      emergencyStop = true;
    }
  }
}

// ============================================================================
// EXECUTE STABILIZE MODE
// ============================================================================
void executeStabilize() {
  // Check if throttle dropped to idle
  if (rxData.throttle < 1100) {
    currentState = STATE_ARMED_IDLE;
    Serial.println(F("Returning to ARMED IDLE"));
  }
  
  // Check mode switch
  if (rxData.modeSwitch == 1 && barometerValid) {
    currentState = STATE_ALTITUDE_HOLD;
    targetAltitude = currentAltitude;
    Serial.println(F("Switched to ALTITUDE HOLD"));
    beep(880, 120);  // A5
  }
}

// ============================================================================
// EXECUTE ALTITUDE HOLD MODE
// ============================================================================
void executeAltitudeHold() {
  // Adjust target altitude based on throttle stick
  if (rxData.throttle > 1600) {
    targetAltitude += 0.001; // Climb
  } else if (rxData.throttle < 1400) {
    targetAltitude -= 0.001; // Descend
  }
  
  // Clamp target altitude
  targetAltitude = constrainFloat(targetAltitude, 0.0, 50.0);
  
  // Check mode switch
  if (rxData.modeSwitch == 0) {
    currentState = STATE_STABILIZE;
    Serial.println(F("Switched to STABILIZE"));
    beep(698, 120);  // F5
  }
  
  // Check if throttle dropped to idle
  if (rxData.throttle < 1100) {
    currentState = STATE_ARMED_IDLE;
    Serial.println(F("Returning to ARMED IDLE"));
  }
}

// ============================================================================
// EXECUTE TAKEOFF
// ============================================================================
void executeTakeoff() {
  // Smooth ramp-up to target altitude
  if (currentAltitude >= takeoffTargetAltitude - 0.1) {
    // Reached target
    Serial.println(F("Takeoff complete, entering ALTITUDE HOLD"));
    currentState = STATE_ALTITUDE_HOLD;
    beep(880, 250);  // A5 - success
  }
  
  // Timeout after 10 seconds
  if (millis() - takeoffStartTime > 10000) {
    Serial.println(F("Takeoff timeout, entering ALTITUDE HOLD"));
    currentState = STATE_ALTITUDE_HOLD;
  }
}

// ============================================================================
// EXECUTE LANDING
// ============================================================================
void executeLanding() {
  switch(landingPhase) {
    case LANDING_DESCEND:
      // Controlled descent
      targetAltitude = landingStartAltitude - (millis() - landingStartTime) * 0.0002; // ~0.2 m/s descent
      
      // Limit descent rate
      if (targetAltitude < currentAltitude - MAX_DESCENT_RATE * 0.1) {
        targetAltitude = currentAltitude - MAX_DESCENT_RATE * 0.1;
      }
      
      // Check for touchdown
      if (currentAltitude < TOUCHDOWN_THRESHOLD && abs(verticalVelocity) < TOUCHDOWN_VELOCITY) {
        landingPhase = LANDING_DETECT_TOUCHDOWN;
        Serial.println(F("Touchdown detected"));
        beep(523, 150);  // C5 - soft touchdown
      }
      break;
      
    case LANDING_DETECT_TOUCHDOWN:
      // Reduce throttle to settle
      targetAltitude = -0.1; // Below ground
      
      // Wait for stable contact (1 second)
      if (millis() - landingStartTime > 1000) {
        landingPhase = LANDING_SETTLE;
        touchdownDetected = true;
      }
      break;
      
    case LANDING_SETTLE:
      // Gradually reduce to idle
      static unsigned long settleStartTime = millis();
      unsigned long settleElapsed = millis() - settleStartTime;
      
      if (settleElapsed < 2000) {
        // Ramp down over 2 seconds
        int rampThrottle = map(settleElapsed, 0, 2000, MOTOR_HOVER_BASE, MOTOR_ARMED_IDLE);
        motorFL = motorFR = motorRL = motorRR = rampThrottle;
      } else {
        landingPhase = LANDING_COMPLETE;
      }
      break;
      
    case LANDING_COMPLETE:
      Serial.println(F("Landing complete, transitioning to ARMED IDLE"));
      currentState = STATE_ARMED_IDLE;
      beep(659, 300);  // E5 - landing complete
      landingPhase = LANDING_DESCEND; // Reset for next time
      break;
  }
  
  // Emergency abort if tilt exceeds safe limits
  if (abs(imu.roll) > MAX_LANDING_TILT * 1.5 || abs(imu.pitch) > MAX_LANDING_TILT * 1.5) {
    Serial.println(F("Landing aborted: excessive tilt"));
    currentState = STATE_EMERGENCY;
    emergencyStop = true;
  }
}

// ============================================================================
// EMERGENCY SHUTDOWN
// ============================================================================
void emergencyShutdown() {
  emergencyStop = true;
  
  Serial.println(F("!!! EMERGENCY SHUTDOWN !!!"));
  
  escFL.writeMicroseconds(MOTOR_MIN);
  escFR.writeMicroseconds(MOTOR_MIN);
  escRL.writeMicroseconds(MOTOR_MIN);
  escRR.writeMicroseconds(MOTOR_MIN);
  
  // Continuous alarm (gentle warning)
  static unsigned long lastBeep = 0;
  if (millis() - lastBeep > 800) {
    beep(392, 150);  // G4 - warning
    lastBeep = millis();
  }
  
  // Manual recovery: cycle arm switch
  if (rxData.armSwitch == 0) {
    emergencyStop = false;
    currentState = STATE_DISARMED;
    Serial.println(F("Emergency cleared"));
  }
}

// ============================================================================
// FAILSAFE CHECK
// ============================================================================
void checkFailsafe() {
  if (millis() - lastRadioRx > FAILSAFE_TIMEOUT) {
    if (!failsafeActive) {
      failsafeActive = true;
      Serial.println(F("FAILSAFE: Radio link lost!"));
      
      // Initiate emergency landing if flying
      if (currentState == STATE_STABILIZE || currentState == STATE_ALTITUDE_HOLD || currentState == STATE_TAKEOFF) {
        if (barometerValid) {
          Serial.println(F("Initiating emergency auto-landing"));
          currentState = STATE_LANDING;
          landingStartTime = millis();
          landingStartAltitude = currentAltitude;
          landingPhase = LANDING_DESCEND;
        } else {
          Serial.println(F("No barometer - emergency stop"));
          currentState = STATE_EMERGENCY;
          emergencyStop = true;
        }
      } else {
        currentState = STATE_DISARMED;
      }
      
      beep(330, 400);  // E4 - failsafe warning
    }
  }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================
void beep(int frequency, int duration) {
  tone(BUZZER_PIN, frequency, duration);
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrainFloat(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

// ============================================================================
// SERIAL DEBUG
// ============================================================================
void serialDebug() {
  Serial.print(F("State:"));
  Serial.print(currentState);
  Serial.print(F(" | R:"));
  Serial.print(imu.roll, 1);
  Serial.print(F(" P:"));
  Serial.print(imu.pitch, 1);
  Serial.print(F(" Y:"));
  Serial.print(imu.yaw, 1);
  Serial.print(F(" | Alt:"));
  Serial.print(currentAltitude, 2);
  Serial.print(F(" Tgt:"));
  Serial.print(targetAltitude, 2);
  Serial.print(F(" | M:"));
  Serial.print(motorFL);
  Serial.print(F(","));
  Serial.print(motorFR);
  Serial.print(F(","));
  Serial.print(motorRL);
  Serial.print(F(","));
  Serial.print(motorRR);
  Serial.print(F(" | PID_P:"));
  Serial.print(pidRoll.kp, 2);
  Serial.print(F(" D:"));
  Serial.print(pidRoll.kd, 1);
  Serial.print(F(" | FS:"));
  Serial.println(failsafeActive);
}
