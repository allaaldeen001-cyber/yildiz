/**
 * ============================================================================
 * DIY DRONE FLIGHT CONTROLLER
 * ============================================================================
 * 
 * Hardware: Arduino Nano, NRF24L01, MPU6050, MS5611
 * 
 * PIN MAPPING (Revised to avoid conflicts):
 * -----------------------------------------
 * ESC Motors (PWM required):
 *   D3  = Front Left (FL)
 *   D5  = Front Right (FR)
 *   D6  = Rear Right (RR)
 *   D9  = Rear Left (RL)
 * 
 * NRF24L01 Radio:
 *   D4  = CE
 *   D10 = CSN
 *   D11 = MOSI (hardware SPI)
 *   D12 = MISO (hardware SPI)
 *   D13 = SCK  (hardware SPI)
 * 
 * I2C Sensors (MPU6050 + MS5611):
 *   A4  = SDA
 *   A5  = SCL
 * 
 * Buttons & Switches:
 *   D2  = ARM/DISARM Switch (1=Disarmed, 0=Armed)
 *   A0  = Calibration Button (press to calibrate MPU6050 + MS5611)
 *   A1  = Smooth Motor-Start Button
 *   A2  = Altitude-Hold Switch (0=Enabled)
 * 
 * Outputs:
 *   D7  = Status LED
 *   D8  = Buzzer
 * 
 * Battery Monitoring:
 *   A3  = Voltage divider input
 * 
 * ============================================================================
 */

#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "Gyro.h"
#include <Smoothed.h>
#include "MS5611.h"

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// NRF24L01 Radio
#define NRF_CE_PIN    4
#define NRF_CSN_PIN   10
#define NRF_CHANNEL   108  // Stable channel (avoid WiFi interference)

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const uint64_t pipe = 0xF0F0F0F0E1LL;

// MS5611 Barometer
MS5611 baro(0x77);

// ESC Motor Pins (PWM capable)
#define ESC_FL_PIN    3   // Front Left
#define ESC_FR_PIN    5   // Front Right
#define ESC_RR_PIN    6   // Rear Right
#define ESC_RL_PIN    9   // Rear Left

// Button & Switch Pins
#define ARM_SWITCH_PIN        2   // D2: Arm/Disarm (1=Disarmed, 0=Armed)
#define CALIBRATION_BTN_PIN   A0  // Calibration button
#define MOTOR_START_BTN_PIN   A1  // Smooth motor start button
#define ALTITUDE_HOLD_PIN     A2  // Altitude hold switch

// Output Pins
#define LED_PIN       7
#define BUZZER_PIN    8

// Battery Monitoring
#define BATTERY_PIN   A3
#define R1            1500.0  // Voltage divider resistor 1 (ohms)
#define R2            1000.0  // Voltage divider resistor 2 (ohms)

// ============================================================================
// CONFIGURATION PARAMETERS
// ============================================================================

// PID Gains for Pitch/Roll
const float kp = 2.0;         // Proportional gain
const float ki = 0.0001;      // Integral gain
const float kd = 0.5;         // Derivative gain
const float kpZ = 2.0;        // Yaw proportional gain

// Altitude Hold PID
float pid_p_gain_altitude = 14.0;
float pid_i_gain_altitude = 2.0;
float pid_d_gain_altitude = 7.5;
int pid_max_altitude = 400;

// Control Sensitivity
float sensiX = -0.45;
float sensiY = 0.45;
float sensiZ = -0.01;
float sensiThrust = 1.1;

// Joystick Deadzone (Low Pass)
int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;

// Loop Frequency
float hz = 140;               // Target loop frequency (Hz)

// Motor Limits
#define PWM_MAX         2000  // Maximum ESC signal
#define PWM_MIN         1000  // Minimum ESC signal
#define PWM_ARMED_MIN   1050  // Minimum when armed (motors idle)
#define THRUST_MAX      1700  // Maximum thrust limit for safety

// Safety Limits
#define MAX_TILT_ANGLE  30    // Maximum tilt angle (degrees) - Safety limit
#define KILL_ON_ANGLE   true  // Kill motors if angle exceeded

// Smooth Motor Start
#define MOTOR_RAMP_TIME 2000  // Time to ramp up motors (ms)
#define MOTOR_RAMP_MIN  1050
#define MOTOR_RAMP_MAX  1200

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct Package {
  int   thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int   id = 0;
  bool  but1 = 1;
  bool  but2 = 1;
  bool  switch1 = 1;
  bool  switch2 = 1;
};

struct QuadProperties {
  float height;
  float kalmanvel_z;
  float baro_height;
};

struct Matrix2x2 {
  float m11, m12, m21, m22;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

Package package;
Gyro gyro;
Smoothed<float> pressureSmooth;

Servo ESCfl, ESCfr, ESCrl, ESCrr;

// State Variables
bool armed = false;
bool radioLinked = false;
bool calibrating = false;
bool smoothStartActive = false;
bool altitudeHoldActive = false;

// Button States (for edge detection)
bool lastCalibrationBtn = HIGH;
bool lastMotorStartBtn = HIGH;
bool lastArmSwitch = HIGH;
bool lastAltitudeSwitch = HIGH;

// Timing
double timepi = 0;
unsigned long prevTime = 0;
unsigned long lastRadioReceive = 0;
unsigned long smoothStartTime = 0;
unsigned long ledBlinkTime = 0;

// Motor Values
int thrust = PWM_MIN;
int FrontLeft = PWM_MIN;
int FrontRight = PWM_MIN;
int RearLeft = PWM_MIN;
int RearRight = PWM_MIN;
int MIN = PWM_MIN;
int MAX = PWM_MAX;

// PID Variables
Vec3 PID[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
Vec3 target = {0, 0, 0};
Vec3 cal = {0, 0, 0};
Vec3 prevError = {0, 0, 0};

// Altitude Hold Variables
QuadProperties quadprops;
Matrix2x2 current_prob;
float ground_pressure = 0;
float altitude_setpoint = 0;
float pid_i_mem_altitude = 0;
float pid_output_altitude = 0;
float pid_last_altitude_d_error = 0;
float actual_pressure = 0;
int manual_throttle = 0;
bool altitudeHoldInitialized = false;

// Pressure averaging
int32_t pressure_buffer[35];
uint8_t pressure_buffer_index = 0;
float pressure_parachute_previous = 0;
int32_t parachute_throttle = 0;

// Counter for pressure reading
byte pressureReadCounter = 0;

// Kill switch and safety
int killSwitch = 0;
float noDataCount = 0;
float calibrationCounter = 0;
float armingCounter = 0;

// Battery
float batteryVoltage = 0;

// Debugging
bool debugging = false;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void setupRadio();
void setupMotors();
void setupSensors();
void scanI2C();
void readInputs();
bool receiveRadio();
void checkStatus();
void calculatePID();
void calculateMotorSpeeds();
void runMotors();
void stopMotors();
void smoothMotorStart();
void resetYaw();
void calculatePressure();
void altitudeHoldPID();
void initKalmanFilter();
void kalmanPosVel();
void readBattery();
void readEEPROM();
void saveCalibration();
void blinkLED();
void beep(int frequency, int duration);
void indicateLink();
void indicateCalibration();
void indicateArmed();
void printDebug();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(57600);
  
  // Initialize timing
  prevTime = micros();
  timepi = 1.0 / hz;
  
  // Configure pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ARM_SWITCH_PIN, INPUT_PULLUP);
  pinMode(CALIBRATION_BTN_PIN, INPUT_PULLUP);
  pinMode(MOTOR_START_BTN_PIN, INPUT_PULLUP);
  pinMode(ALTITUDE_HOLD_PIN, INPUT_PULLUP);
  
  // Startup indication
  beep(1000, 300);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  beep(1600, 700);
  delay(100);
  beep(2000, 200);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize motors
  setupMotors();
  
  // Initialize radio
  setupRadio();
  
  // Initialize sensors
  setupSensors();
  
  // Load calibration from EEPROM
  readEEPROM();
  
  // Initialize pressure smoothing
  pressureSmooth.begin(SMOOTHED_AVERAGE, 10);
  
  // Initialize Kalman filter
  initKalmanFilter();
  
  // Ready indication
  beep(2000, 500);
  Serial.println(F("=== Flight Controller Ready ==="));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Read all inputs (buttons, switches, radio)
  readInputs();
  
  // Check system status and handle safety
  checkStatus();
  
  // Update gyro target and calibration
  gyro.setTarget(target);
  gyro.setCalibration(cal);
  
  // Read barometer and calculate altitude
  calculatePressure();
  
  // Calculate gyro error
  gyro.calculateError();
  
  // Calculate PID outputs
  calculatePID();
  
  // Handle altitude hold if active
  if (altitudeHoldActive && armed) {
    altitudeHoldPID();
  }
  
  // Calculate motor speeds
  calculateMotorSpeeds();
  
  // Handle smooth motor start
  if (smoothStartActive) {
    smoothMotorStart();
  } else {
    runMotors();
  }
  
  // Update LED status
  updateLED();
  
  // Debug output
  if (debugging) {
    printDebug();
  }
  
  // Wait for next loop iteration
  waitForNextLoop();
}

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

void setupRadio() {
  radio.begin();
  radio.setChannel(NRF_CHANNEL);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  Serial.println(F("Radio initialized"));
}

void setupMotors() {
  ESCfl.attach(ESC_FL_PIN, PWM_MIN, PWM_MAX);
  ESCfr.attach(ESC_FR_PIN, PWM_MIN, PWM_MAX);
  ESCrl.attach(ESC_RL_PIN, PWM_MIN, PWM_MAX);
  ESCrr.attach(ESC_RR_PIN, PWM_MIN, PWM_MAX);
  stopMotors();
  delay(500);
  Serial.println(F("Motors attached"));
}

void scanI2C() {
  Serial.println(F("Scanning I2C bus..."));
  byte count = 0;
  
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print(F("  Found device at 0x"));
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      
      // Identify known devices
      if (addr == 0x68 || addr == 0x69) {
        Serial.print(F(" (MPU6050)"));
      } else if (addr == 0x77 || addr == 0x76) {
        Serial.print(F(" (MS5611/BMP)"));
      }
      Serial.println();
      count++;
    }
  }
  
  if (count == 0) {
    Serial.println(F("  No I2C devices found!"));
    Serial.println(F("  Check wiring: SDA->A4, SCL->A5"));
  } else {
    Serial.print(F("  Found "));
    Serial.print(count);
    Serial.println(F(" device(s)"));
  }
}

void setupSensors() {
  // Initialize I2C
  Wire.begin();
  delay(100);
  
  // Scan for I2C devices first
  scanI2C();
  
  // Check if MPU6050 is present
  Wire.beginTransmission(0x68);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.println(F("ERROR: MPU6050 not found at 0x68!"));
    Serial.println(F("Check wiring and try again."));
    
    // Blink LED and beep to indicate error
    while (1) {
      beep(500, 200);
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(800);
    }
  }
  
  Serial.println(F("MPU6050 found, initializing..."));
  
  // Initialize gyro/accelerometer
  gyro.SetupWire(timepi);
  Serial.println(F("MPU6050 ready"));
  delay(500);
  
  // Check if MS5611 is present
  Wire.beginTransmission(0x77);
  error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.println(F("WARNING: MS5611 not found at 0x77"));
    Serial.println(F("Altitude hold will be disabled"));
  } else {
    // Initialize barometer
    baro.begin();
    baro.setOversampling(OSR_LOW);
    
    // Read initial ground pressure
    delay(100);
    baro.read();
    ground_pressure = baro.getPressure();
    Serial.println(F("MS5611 ready"));
  }
  
  Serial.println(F("Sensors initialized"));
}

// ============================================================================
// INPUT HANDLING
// ============================================================================

void readInputs() {
  // Read buttons and switches
  bool armSwitch = digitalRead(ARM_SWITCH_PIN);
  bool calibrationBtn = digitalRead(CALIBRATION_BTN_PIN);
  bool motorStartBtn = digitalRead(MOTOR_START_BTN_PIN);
  bool altitudeSwitch = digitalRead(ALTITUDE_HOLD_PIN);
  
  // Arm/Disarm switch (1 = Disarmed, 0 = Armed)
  if (armSwitch != lastArmSwitch) {
    if (armSwitch == LOW) {
      // Switching to armed
      if (!armed) {
        armed = true;
        indicateArmed();
        Serial.println(F("ARMED"));
      }
    } else {
      // Switching to disarmed
      armed = false;
      stopMotors();
      smoothStartActive = false;
      Serial.println(F("DISARMED"));
    }
    lastArmSwitch = armSwitch;
  }
  
  // Calibration button (only when disarmed)
  if (calibrationBtn == LOW && lastCalibrationBtn == HIGH && !armed) {
    calibrationCounter += timepi;
  } else if (calibrationBtn == HIGH) {
    calibrationCounter = 0;
  }
  lastCalibrationBtn = calibrationBtn;
  
  // Smooth motor start button (only when armed)
  if (motorStartBtn == LOW && lastMotorStartBtn == HIGH && armed && !smoothStartActive) {
    smoothStartActive = true;
    smoothStartTime = millis();
    Serial.println(F("Smooth motor start initiated"));
  }
  lastMotorStartBtn = motorStartBtn;
  
  // Altitude hold switch
  if (altitudeSwitch != lastAltitudeSwitch) {
    altitudeHoldActive = (altitudeSwitch == LOW);
    if (altitudeHoldActive) {
      altitude_setpoint = actual_pressure;
      altitudeHoldInitialized = true;
      pid_i_mem_altitude = 0;
      Serial.println(F("Altitude Hold ON"));
    } else {
      altitudeHoldInitialized = false;
      Serial.println(F("Altitude Hold OFF"));
    }
    lastAltitudeSwitch = altitudeSwitch;
  }
  
  // Receive radio data
  receiveRadio();
}

bool receiveRadio() {
  if (radio.available()) {
    radio.read(&package, sizeof(package));
    
    // Mark radio as linked
    if (!radioLinked) {
      radioLinked = true;
      indicateLink();
    }
    
    lastRadioReceive = millis();
    noDataCount = 0;
    
    if (package.thrust != 0) {
      // Apply deadzone filtering
      if (package.z < lowPassZ && package.z > -lowPassZ)
        package.z = 0;
      if (package.x < lowPassX && package.x > -lowPassX)
        package.x = 0;
      if (package.y < lowPassY && package.y > -lowPassY)
        package.y = 0;
      
      // Apply sensitivity and calculate targets
      target.x = package.x * sensiX;
      target.y = package.y * sensiY;
      
      if (armed) {
        target.z += package.z * sensiZ;
      }
      
      thrust = package.thrust * sensiThrust;
      thrust = constrain(thrust, PWM_MIN, THRUST_MAX);
      
      return true;
    }
  } else {
    noDataCount += timepi;
  }
  return false;
}

// ============================================================================
// STATUS & SAFETY
// ============================================================================

void checkStatus() {
  // Check for radio timeout (3 seconds)
  if (noDataCount > 3) {
    killSwitch = 2;  // Radio lost
  }
  
  // Check for excessive tilt angle
  if (KILL_ON_ANGLE) {
    if (abs(gyro.error.x) > MAX_TILT_ANGLE || abs(gyro.error.y) > MAX_TILT_ANGLE) {
      killSwitch = 1;  // Angle exceeded
    }
  }
  
  // Handle kill switch conditions
  if (killSwitch > 0) {
    handleKillSwitch();
  }
  
  // Handle calibration (hold button for 2 seconds while disarmed)
  if (calibrationCounter > 2 && !armed) {
    performCalibration();
    calibrationCounter = 0;
  }
  
  // Reset yaw if out of bounds
  if (gyro.error.z > 180 || gyro.error.z < -180) {
    resetYaw();
  }
}

void handleKillSwitch() {
  stopMotors();
  armed = false;
  
  while (killSwitch > 0) {
    // Alarm pattern
    beep(1000, 300);
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(2000);
    
    // Check if radio recovered (for radio-loss kill)
    if (killSwitch == 2 && radio.available()) {
      delay(500);
      if (radio.available()) {
        beep(1500, 1000);
        killSwitch = 0;
        Serial.println(F("Radio recovered"));
      }
    }
  }
}

void performCalibration() {
  Serial.println(F("Calibrating..."));
  calibrating = true;
  stopMotors();
  
  // Indicate calibration start
  beep(1200, 100);
  digitalWrite(LED_PIN, HIGH);
  delay(300);
  beep(1200, 200);
  
  // Calibrate MPU6050
  cal = gyro.calibrate(1000);
  
  // Calibrate MS5611 (set ground pressure)
  baro.read();
  ground_pressure = baro.getPressure();
  
  // Save calibration to EEPROM
  saveCalibration();
  
  // Apply calibration
  gyro.setCalibration(cal);
  
  // Indicate calibration complete
  delay(500);
  beep(2200, 200);
  digitalWrite(LED_PIN, LOW);
  
  calibrating = false;
  Serial.println(F("Calibration complete"));
}

// ============================================================================
// PID CALCULATIONS
// ============================================================================

void calculatePID() {
  if (!armed) {
    // Reset PID when not armed
    PID[0] = {0, 0, 0};
    PID[1] = {0, 0, 0};
    PID[2] = {0, 0, 0};
    prevError = {0, 0, 0};
    resetYaw();
    return;
  }
  
  // Proportional term
  PID[0].x = gyro.error.x * kp;
  PID[0].y = gyro.error.y * kp;
  PID[0].z = gyro.error.z * kpZ;
  
  // Integral term
  PID[1].x += gyro.error.x * timepi * ki;
  PID[1].y += gyro.error.y * timepi * ki;
  PID[1].z += gyro.error.z * timepi * ki;
  
  // Derivative term
  PID[2].x = kd * (gyro.error.x - prevError.x) / timepi;
  PID[2].y = kd * (gyro.error.y - prevError.y) / timepi;
  PID[2].z = kd * (gyro.error.z - prevError.z) / timepi;
  
  prevError = gyro.error;
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void calculateMotorSpeeds() {
  int baseThrust;
  
  // Use altitude hold thrust if active, otherwise manual thrust
  if (altitudeHoldActive && armed) {
    baseThrust = 1450 + pid_output_altitude + manual_throttle;
  } else {
    baseThrust = thrust;
  }
  
  // Calculate individual motor speeds with PID corrections
  // Motor configuration: X-frame
  // FL (CW), FR (CCW), RL (CCW), RR (CW)
  float pidX = PID[0].x + PID[1].x + PID[2].x;
  float pidY = PID[0].y + PID[1].y + PID[2].y;
  float pidZ = PID[0].z + PID[2].z;  // No integral for yaw
  
  RearLeft  = baseThrust - pidX - pidY + pidZ;
  RearRight = baseThrust + pidX - pidY - pidZ;
  FrontLeft = baseThrust - pidX + pidY - pidZ;
  FrontRight = baseThrust + pidX + pidY + pidZ;
}

void runMotors() {
  if (armed) {
    MIN = PWM_ARMED_MIN;
  } else {
    MIN = PWM_MIN;
  }
  
  // Constrain motor values
  FrontLeft = constrain(FrontLeft, MIN, MAX);
  FrontRight = constrain(FrontRight, MIN, MAX);
  RearLeft = constrain(RearLeft, MIN, MAX);
  RearRight = constrain(RearRight, MIN, MAX);
  
  if (armed) {
    ESCfl.write(FrontLeft);
    ESCfr.write(FrontRight);
    ESCrl.write(RearLeft);
    ESCrr.write(RearRight);
  } else {
    stopMotors();
  }
}

void stopMotors() {
  ESCfl.write(0);
  ESCfr.write(0);
  ESCrl.write(0);
  ESCrr.write(0);
  
  FrontLeft = PWM_MIN;
  FrontRight = PWM_MIN;
  RearLeft = PWM_MIN;
  RearRight = PWM_MIN;
  MIN = PWM_MIN;
}

void smoothMotorStart() {
  unsigned long elapsed = millis() - smoothStartTime;
  
  if (elapsed < MOTOR_RAMP_TIME) {
    // Calculate ramped speed
    int rampSpeed = map(elapsed, 0, MOTOR_RAMP_TIME, MOTOR_RAMP_MIN, MOTOR_RAMP_MAX);
    
    ESCfl.write(rampSpeed);
    ESCfr.write(rampSpeed);
    ESCrl.write(rampSpeed);
    ESCrr.write(rampSpeed);
  } else {
    // Ramp complete, return to normal operation
    smoothStartActive = false;
    beep(1500, 200);
    Serial.println(F("Smooth start complete"));
  }
}

// ============================================================================
// ALTITUDE HOLD
// ============================================================================

void calculatePressure() {
  // Read pressure every 10 iterations
  if (pressureReadCounter == 0) {
    baro.read();
    pressureSmooth.add(baro.getPressure());
    pressureReadCounter = 10;
  }
  pressureReadCounter--;
  
  actual_pressure = pressureSmooth.get();
  quadprops.baro_height = actual_pressure;
  
  // Apply Kalman filter
  kalmanPosVel();
}

void altitudeHoldPID() {
  if (!altitudeHoldInitialized) {
    altitude_setpoint = actual_pressure;
    altitudeHoldInitialized = true;
    pid_i_mem_altitude = 0;
    return;
  }
  
  // Check for manual altitude adjustment via throttle
  manual_throttle = 0;
  
  if (thrust > 1450) {
    // Climbing - user pushing throttle up
    altitude_setpoint = actual_pressure;
    manual_throttle = (thrust - 1450) / 3;
  } else if (thrust < 1400) {
    // Descending - user pulling throttle down
    altitude_setpoint = actual_pressure;
    manual_throttle = (thrust - 1400) / 5;
  }
  
  // Calculate altitude error
  float pid_error = actual_pressure - altitude_setpoint;
  
  // Dynamic P-gain based on error magnitude
  float error_gain = 0;
  if (abs(pid_error) > 10) {
    error_gain = (abs(pid_error) - 10) / 20.0;
    error_gain = constrain(error_gain, 0, 3);
  }
  
  // Integral term with anti-windup
  pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error;
  pid_i_mem_altitude = constrain(pid_i_mem_altitude, -pid_max_altitude, pid_max_altitude);
  
  // Derivative term (using Kalman velocity)
  float derivative = pid_d_gain_altitude * quadprops.kalmanvel_z;
  
  // Calculate total PID output
  pid_output_altitude = 100 * (pid_p_gain_altitude + error_gain) * pid_error 
                       + pid_i_mem_altitude 
                       + derivative;
  
  pid_output_altitude = constrain(pid_output_altitude, -pid_max_altitude, pid_max_altitude);
}

// ============================================================================
// KALMAN FILTER
// ============================================================================

void initKalmanFilter() {
  current_prob.m11 = 1;
  current_prob.m12 = 0;
  current_prob.m21 = 0;
  current_prob.m22 = 1;
  quadprops.height = 0;
  quadprops.kalmanvel_z = 0;
}

#define KALMAN_TIMESLICE 0.007  // 140 Hz
#define KALMAN_VAR_ACC   1

void kalmanPosVel() {
  // Process noise covariance
  const float Q11 = KALMAN_VAR_ACC * 0.25 * pow(KALMAN_TIMESLICE, 4);
  const float Q12 = KALMAN_VAR_ACC * 0.5 * pow(KALMAN_TIMESLICE, 3);
  const float Q21 = Q12;
  const float Q22 = KALMAN_VAR_ACC * pow(KALMAN_TIMESLICE, 2);
  
  // Measurement noise
  const float R11 = 0.008;
  
  // Prediction step
  float ps1 = quadprops.height + KALMAN_TIMESLICE * quadprops.kalmanvel_z;
  float ps2 = quadprops.kalmanvel_z;
  
  // Predicted covariance
  float opt = KALMAN_TIMESLICE * current_prob.m22;
  float pp12 = current_prob.m12 + opt + Q12;
  float pp21 = current_prob.m21 + opt + Q21;
  float pp11 = current_prob.m11 + KALMAN_TIMESLICE * (current_prob.m12 + pp21) + Q11;
  float pp22 = current_prob.m22 + Q22;
  
  // Innovation
  float inn = quadprops.baro_height - ps1;
  float ic = pp11 + R11;
  
  // Kalman gain
  float kg1 = pp11 / ic;
  float kg2 = pp21 / ic;
  
  // Update state
  quadprops.height = ps1 + kg1 * inn;
  quadprops.kalmanvel_z = ps2 + kg2 * inn;
  
  // Update covariance
  opt = 1 - kg1;
  current_prob.m11 = pp11 * opt;
  current_prob.m12 = pp12 * opt;
  current_prob.m21 = pp21 - pp11 * kg2;
  current_prob.m22 = pp22 - pp12 * kg2;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void resetYaw() {
  gyro.zeroYaw(true);
  target.z = 0;
}

void readBattery() {
  int raw = analogRead(BATTERY_PIN);
  float vout = (raw * 5.0) / 1023.0;
  batteryVoltage = vout / (R2 / (R1 + R2));
}

void readEEPROM() {
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
  
  // Check for invalid values (first boot)
  if (isnan(cal.x) || isnan(cal.y)) {
    cal.x = 0;
    cal.y = 0;
  }
  
  Serial.print(F("Loaded calibration: X="));
  Serial.print(cal.x);
  Serial.print(F(" Y="));
  Serial.println(cal.y);
}

void saveCalibration() {
  EEPROM.put(10, cal.x);
  EEPROM.put(15, cal.y);
  EEPROM.put(20, ground_pressure);
}

void waitForNextLoop() {
  while (micros() - prevTime < timepi * 1000000);
  prevTime = micros();
}

// ============================================================================
// INDICATORS
// ============================================================================

void beep(int frequency, int duration) {
  tone(BUZZER_PIN, frequency, duration);
}

void indicateLink() {
  beep(1800, 200);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  beep(2200, 200);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  Serial.println(F("Radio linked"));
}

void indicateCalibration() {
  for (int i = 0; i < 3; i++) {
    beep(1500, 100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void indicateArmed() {
  beep(1500, 500);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
}

void updateLED() {
  // LED behavior:
  // - Solid ON when disarmed (warning)
  // - Blink when receiving data
  // - Off otherwise
  
  if (!armed) {
    digitalWrite(LED_PIN, HIGH);  // Solid on when disarmed
  } else if (millis() - lastRadioReceive < 100) {
    // Blink when receiving data
    if ((millis() / 50) % 2 == 0) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

void printDebug() {
  Serial.print(F("FL:"));
  Serial.print(FrontLeft);
  Serial.print(F("\tFR:"));
  Serial.print(FrontRight);
  Serial.print(F("\tRL:"));
  Serial.print(RearLeft);
  Serial.print(F("\tRR:"));
  Serial.print(RearRight);
  Serial.print(F("\tPitch:"));
  Serial.print(gyro.error.x);
  Serial.print(F("\tRoll:"));
  Serial.print(gyro.error.y);
  Serial.print(F("\tYaw:"));
  Serial.print(gyro.error.z);
  Serial.print(F("\tAlt:"));
  Serial.print(actual_pressure);
  Serial.print(F("\tArmed:"));
  Serial.println(armed ? F("YES") : F("NO"));
}
