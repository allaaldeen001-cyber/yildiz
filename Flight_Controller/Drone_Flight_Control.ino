/*
 * DRONE FLIGHT CONTROLLER - Main Code
 * Hardware: Arduino Nano + NRF24L01 + MPU6050 + MS5611
 * 
 * Pin Configuration:
 * - NRF24L01: CE=D4, CSN=D10
 * - Motors: FL=D3, FR=D5, RR=D6, RL=D9
 * - Buzzer: D8
 * - Status LED: D7
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

MS5611 MS5611(0x77);
RF24 radio(4, 10);  // CE, CSN
const uint64_t pipe = 0xF0F0F0F0E1LL;

// Communication Package Structure
struct Package
{
  int   thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int   id = 0;
  bool  but1 = 1;      // Button 1: Calibration
  bool  but2 = 1;      // Button 2: Smooth Motor Start
  bool  switch1 = 1;   // Switch 1: Arm/Disarm
  bool  switch2 = 1;   // Switch 2: Altitude Hold
};

Package package;
Gyro gyro;

Servo ESCfl;
Servo ESCfr;
Servo ESCrl;
Servo ESCrr;

// PID Parameters
const float kp = 2.0;
const float ki = 0.0001;
const float kd = 0.5;
const float kpZ = 2.0;

// Altitude PID Parameters
float pid_p_gain_altitude = 14.0;
float pid_i_gain_altitude = 2.0;
float pid_d_gain_altitude = 7.5;
int pid_max_altitude = 400;

// Altitude Control Variables
float pid_error_gain_altitude, pid_throttle_gain_altitude;
float ground_pressure, altitude_hold_pressure;
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_total_average;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;
float actual_pressure, pid_error_temp;
uint8_t manual_altitude_change;
int16_t manual_throttle;
byte hold = 0;
float actual_pressure_smoothed;
byte pressure_counter = 0;

// Control Sensitivity
float sensiX = -0.45;
float sensiY = 0.45;
float sensiZ = -0.01;
float sensiThrust = 1.1;

// Low Pass Filters
int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;

// System Parameters
float hz = 140.0;
int pMAX = 2000;
int pMIN = 1000;
int MINarmed = 1050;
int maxThrust = 1700;
int maxAngle = 30;  // SAFETY: 30° maximum tilt angle
bool killAngle = true;

// Motor Pins
const int flPIN = 3;
const int frPIN = 5;
const int rrPIN = 6;
const int rlPIN = 9;

// Output Pins
const int BUZZER = 8;
const int LED = 7;

Smoothed <float> smooth;

// System State Variables
int MAX = pMAX;
int MIN = pMIN;
int thrust = pMIN;
int thrust_altitude_adjusted = thrust;
int killSwitch = 0;

float calCount = 0;
float NoDataCount = 0;
float armingCounter = 0;
float smoothMotorStartCounter = 0;

bool dBugging = false;
bool armed = false;
bool smoothMotorStartActive = false;
bool calibrationInProgress = false;
bool nrfConnected = false;
unsigned long lastNrfReceiveTime = 0;
unsigned long lastLedBlinkTime = 0;
bool ledState = false;

// Time Variables
double timepi = 0;
long prevTime = 0;
const float sec_to_micro = 1000000.0;
const float micro_to_sec = 1.0 / 1000000.0;

// Motor Values
int FrontRight = thrust;
int FrontLeft = thrust;
int RearRight = thrust;
int RearLeft = thrust;

// PID Arrays
Vec3 PID[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
Vec3 target = {0, 0, 0};
Vec3 cal = {0, 0, 0};
Vec3 prevError = {0, 0, 0};

// Kalman Filter for Altitude
struct quad_properties {
  float height;
  float kalmanvel_z;
  float baro_height;
};
struct quad_properties quadprops;

struct matrix2x2 {
  float m11, m21, m12, m22;
};
struct matrix2x2 current_prob;

//===================== FUNCTION PROTOTYPES =====================
void Print();
void readEEPROM();
bool receiveRadio();
void checkStatus();
void calculatePID();
void calculateVelocities();
void wait();
void runMotors();
void stopMotors();
void resetYaw();
void calculate_pressure();
void KalmanPosVel();
void initKalmanPosVel();
void ledBlink(int times, int duration);
void ledUpdate();
void buzzerBeep(int frequency, int duration);
void initializeNRF();
bool checkNRFConnection();
void performFullCalibration();
void smoothMotorStart();

//===================== SETUP =====================
void setup() {
  Serial.begin(57600);
  debugging(false);
  prevTime = micros();
  timepi = (1.0 / hz);
  
  // Initialize Outputs
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  
  // Startup Sound Sequence
  Serial.println("=== DRONE FLIGHT CONTROLLER INITIALIZING ===");
  buzzerBeep(1000, 300);
  ledBlink(1, 300);
  delay(100);
  buzzerBeep(1600, 700);
  ledBlink(1, 700);
  delay(100);
  buzzerBeep(2000, 200);
  ledBlink(1, 200);
  
  // Attach ESCs
  ESCfl.attach(flPIN, 1000, 2000);
  ESCfr.attach(frPIN, 1000, 2000);
  ESCrl.attach(rlPIN, 1000, 2000);
  ESCrr.attach(rrPIN, 1000, 2000);
  stopMotors();
  delay(500);
  Serial.println("Motors attached");
  
  // Initialize NRF24L01
  initializeNRF();
  
  // Read Calibration from EEPROM
  readEEPROM();
  
  // Initialize MPU6050
  gyro.SetupWire(timepi);
  delay(500);
  Serial.println("MPU6050 initialized");
  
  // Initialize MS5611 Barometer
  if (MS5611.begin()) {
    MS5611.setOversampling(OSR_STANDARD);
    smooth.begin(SMOOTHED_AVERAGE, 10);
    Serial.println("MS5611 initialized");
    initKalmanPosVel();
  } else {
    Serial.println("MS5611 initialization FAILED!");
  }
  
  // Initialization Complete
  buzzerBeep(2500, 300);
  ledBlink(3, 100);
  Serial.println("=== INITIALIZATION COMPLETE ===");
  Serial.println("Waiting for RC connection...");
}

//===================== MAIN LOOP =====================
void loop() {
  receiveRadio();
  checkStatus();
  
  if (!calibrationInProgress && !smoothMotorStartActive) {
    gyro.setTarget(target);
    gyro.setCalibration(cal);
    calculate_pressure();
    gyro.calculateError();
    calculatePID();
    calculateVelocities();
    runMotors();
  }
  
  ledUpdate();
  Print();
  wait();
}

//===================== NRF INITIALIZATION =====================
void initializeNRF() {
  Serial.println("Initializing NRF24L01...");
  
  if (!radio.begin()) {
    Serial.println("NRF24L01 initialization FAILED!");
    // Error indication
    for(int i = 0; i < 5; i++) {
      buzzerBeep(500, 100);
      ledBlink(1, 100);
      delay(200);
    }
  } else {
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MAX);  // Use MAX for better range
    radio.setChannel(108);  // Use a clean channel
    radio.openReadingPipe(1, pipe);
    radio.startListening();
    radio.flush_rx();
    Serial.println("NRF24L01 OK - Listening on channel 108");
  }
}

//===================== NRF CONNECTION CHECK =====================
bool checkNRFConnection() {
  if (millis() - lastNrfReceiveTime < 1000) {
    if (!nrfConnected) {
      nrfConnected = true;
      buzzerBeep(2000, 200);
      Serial.println("*** RC CONNECTED ***");
    }
    return true;
  } else {
    if (nrfConnected) {
      nrfConnected = false;
      Serial.println("*** RC DISCONNECTED ***");
    }
    return false;
  }
}

//===================== RECEIVE RADIO DATA =====================
bool receiveRadio() {
  if (radio.available()) {
    radio.read(&package, sizeof(package));
    lastNrfReceiveTime = millis();
    
    if (package.thrust != 0) {
      // Apply Low-Pass Filters
      if (abs(package.z) < lowPassZ) package.z = 0;
      if (abs(package.x) < lowPassX) package.x = 0;
      if (abs(package.y) < lowPassY) package.y = 0;
      
      // Apply Sensitivity
      target.x = package.x * sensiX;
      target.y = package.y * sensiY;
      
      if (armed) {
        target.z += package.z * sensiZ;
      }
      
      thrust = package.thrust * sensiThrust;
      
      // Limit Thrust
      if (thrust < MIN) thrust = MIN;
      if (thrust > maxThrust) thrust = maxThrust;
      
      NoDataCount = 0;
      return true;
    }
  }
  
  NoDataCount += timepi;
  return false;
}

//===================== STATUS CHECKS =====================
void checkStatus() {
  // Check NRF Connection
  checkNRFConnection();
  
  // Switch 1: Arm/Disarm
  if (package.switch1 == 0) {
    if (armed) {
      armed = false;
      stopMotors();
      Serial.println("*** DISARMED ***");
      buzzerBeep(1000, 500);
    }
  }
  
  // Reset Yaw if needed
  if (abs(gyro.error.z) > 180) {
    resetYaw();
  }
  
  // Kill Switch - No Data
  if (NoDataCount > 3.0) {
    if (killSwitch == 0) {
      killSwitch = 2;
      Serial.println("KILL SWITCH: No RC data!");
    }
  }
  
  // Kill Switch - Angle Exceeded
  if (killAngle) {
    if (abs(gyro.error.x) > maxAngle || abs(gyro.error.y) > maxAngle) {
      if (killSwitch == 0) {
        killSwitch = 1;
        Serial.print("KILL SWITCH: Angle exceeded! X=");
        Serial.print(gyro.error.x);
        Serial.print(" Y=");
        Serial.println(gyro.error.y);
      }
    }
  }
  
  // Handle Kill Switch
  if (killSwitch > 0) {
    stopMotors();
    armed = false;
    
    while (killSwitch > 0) {
      buzzerBeep(1000, 300);
      ledBlink(1, 300);
      delay(2000);
      
      // Recovery for communication loss
      if (killSwitch == 2 && radio.available()) {
        delay(500);
        if (radio.available()) {
          buzzerBeep(1500, 1000);
          ledBlink(3, 100);
          killSwitch = 0;
          armed = false;
          Serial.println("Kill switch cleared - communication restored");
        }
      }
    }
  }
  
  // Button 2: Smooth Motor Start
  if (package.but2 == 0 && !smoothMotorStartActive) {
    smoothMotorStartCounter += timepi;
    
    if (smoothMotorStartCounter > 1.0 && armed && !calibrationInProgress) {
      smoothMotorStart();
      smoothMotorStartCounter = 0;
    }
  } else {
    smoothMotorStartCounter = 0;
  }
  
  // Button 1: Calibration (only when disarmed)
  if (package.but1 == 0 && !armed) {
    calCount += timepi;
    
    if (calCount > 2.0 && !calibrationInProgress) {
      performFullCalibration();
      calCount = 0;
    }
  } else {
    calCount = 0;
  }
  
  // Long press Button 2: Arming
  if (package.but2 == 0 && !armed) {
    armingCounter += timepi;
    resetYaw();
    
    if (armingCounter > 3.0) {
      armed = true;
      buzzerBeep(1500, 500);
      ledBlink(3, 100);
      Serial.println("*** ARMED ***");
      armingCounter = 0;
    }
  } else {
    if (armed) {
      armingCounter = 0;
    }
  }
}

//===================== FULL CALIBRATION =====================
void performFullCalibration() {
  calibrationInProgress = true;
  stopMotors();
  
  Serial.println("=== STARTING FULL CALIBRATION ===");
  buzzerBeep(1200, 100);
  ledBlink(1, 100);
  delay(300);
  buzzerBeep(1200, 200);
  ledBlink(1, 200);
  
  // Calibrate MPU6050
  Serial.println("Calibrating MPU6050... Keep drone stable!");
  cal = gyro.calibrate(1000);
  
  // Save to EEPROM
  EEPROM.put(10, static_cast<float>(cal.x));
  EEPROM.put(15, static_cast<float>(cal.y));
  
  gyro.setCalibration(cal);
  
  Serial.print("MPU6050 Cal: X=");
  Serial.print(cal.x);
  Serial.print(" Y=");
  Serial.println(cal.y);
  
  delay(500);
  
  // Calibrate MS5611 (Ground Pressure)
  Serial.println("Calibrating MS5611 barometer...");
  ground_pressure = 0;
  for (int i = 0; i < 100; i++) {
    MS5611.read();
    ground_pressure += MS5611.getPressure();
    delay(10);
  }
  ground_pressure /= 100.0;
  
  Serial.print("Ground Pressure: ");
  Serial.println(ground_pressure);
  
  // Calibration Complete
  buzzerBeep(2200, 200);
  ledBlink(1, 200);
  delay(200);
  buzzerBeep(2500, 300);
  ledBlink(3, 100);
  
  Serial.println("=== CALIBRATION COMPLETE ===");
  calibrationInProgress = false;
}

//===================== SMOOTH MOTOR START =====================
void smoothMotorStart() {
  smoothMotorStartActive = true;
  Serial.println("=== SMOOTH MOTOR START ===");
  
  buzzerBeep(1500, 200);
  
  // Ramp motors from min to armed idle
  for (int i = pMIN; i <= MINarmed; i += 5) {
    ESCfl.write(i);
    ESCfr.write(i);
    ESCrl.write(i);
    ESCrr.write(i);
    delay(50);
  }
  
  delay(2000);  // Hold at idle
  
  // Ramp back down
  for (int i = MINarmed; i >= pMIN; i -= 5) {
    ESCfl.write(i);
    ESCfr.write(i);
    ESCrl.write(i);
    ESCrr.write(i);
    delay(50);
  }
  
  stopMotors();
  buzzerBeep(2000, 300);
  Serial.println("Motor test complete");
  
  smoothMotorStartActive = false;
}

//===================== PID CALCULATION =====================
void calculatePID() {
  if (!armed) {
    resetYaw();
    PID[0] = {0, 0, 0};
    PID[1] = {0, 0, 0};
    PID[2] = {0, 0, 0};
    prevError = {0, 0, 0};
    return;
  }
  
  // Proportional
  PID[0].x = gyro.error.x * kp;
  PID[0].y = gyro.error.y * kp;
  PID[0].z = gyro.error.z * kpZ;
  
  // Integral
  PID[1].x += gyro.error.x * timepi * ki;
  PID[1].y += gyro.error.y * timepi * ki;
  PID[1].z += gyro.error.z * timepi * ki;
  
  // Derivative
  PID[2].x = kd * (gyro.error.x - prevError.x) / timepi;
  PID[2].y = kd * (gyro.error.y - prevError.y) / timepi;
  PID[2].z = kd * (gyro.error.z - prevError.z) / timepi;
  
  prevError = gyro.error;
}

//===================== CALCULATE MOTOR VELOCITIES =====================
void calculateVelocities() {
  thrust_altitude_adjusted = thrust;
  
  // Apply altitude hold adjustment if enabled
  if (package.switch2 == 0 && thrust > 1400 && thrust < 1450) {
    thrust_altitude_adjusted = 1450 + pid_output_altitude + manual_throttle;
  }
  
  // Calculate individual motor speeds
  RearLeft  = thrust_altitude_adjusted - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
  RearRight = thrust_altitude_adjusted + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
  FrontLeft = thrust_altitude_adjusted - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
  FrontRight= thrust_altitude_adjusted + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
}

//===================== RUN MOTORS =====================
void runMotors() {
  if (armed) {
    MIN = MINarmed;
  } else {
    MIN = pMIN;
  }
  
  // Constrain motor values
  RearLeft = constrain(RearLeft, MIN, MAX);
  RearRight = constrain(RearRight, MIN, MAX);
  FrontLeft = constrain(FrontLeft, MIN, MAX);
  FrontRight = constrain(FrontRight, MIN, MAX);
  
  // Write to ESCs only if armed
  if (armed) {
    ESCfl.write(FrontLeft);
    ESCfr.write(FrontRight);
    ESCrl.write(RearLeft);
    ESCrr.write(RearRight);
  } else {
    stopMotors();
  }
}

//===================== STOP MOTORS =====================
void stopMotors() {
  ESCfl.write(0);
  ESCfr.write(0);
  ESCrl.write(0);
  ESCrr.write(0);
  
  MIN = pMIN;
  FrontRight = pMIN;
  FrontLeft = pMIN;
  RearLeft = pMIN;
  RearRight = pMIN;
}

//===================== RESET YAW =====================
void resetYaw() {
  gyro.zeroYaw(true);
  target.z = 0;
}

//===================== LED UPDATE =====================
void ledUpdate() {
  if (!armed) {
    // LED stays ON when disarmed
    digitalWrite(LED, HIGH);
  } else {
    // Blink when receiving data
    if (millis() - lastLedBlinkTime < 100) {
      digitalWrite(LED, HIGH);
    } else {
      digitalWrite(LED, LOW);
    }
    
    if (millis() - lastNrfReceiveTime < 50) {
      lastLedBlinkTime = millis();
    }
  }
}

//===================== LED BLINK =====================
void ledBlink(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED, HIGH);
    delay(duration);
    digitalWrite(LED, LOW);
    if (i < times - 1) delay(duration);
  }
}

//===================== BUZZER BEEP =====================
void buzzerBeep(int frequency, int duration) {
  tone(BUZZER, frequency, duration);
  delay(duration);
}

//===================== READ EEPROM =====================
void readEEPROM() {
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
  Serial.print("Loaded calibration from EEPROM: X=");
  Serial.print(cal.x);
  Serial.print(" Y=");
  Serial.println(cal.y);
}

//===================== DEBUGGING =====================
void debugging(bool dBug) {
  if (dBug) {
    dBugging = true;
    Serial.begin(57600);
  }
}

//===================== WAIT FOR LOOP TIMING =====================
void wait() {
  while (micros() - prevTime < timepi * sec_to_micro);
  prevTime = micros();
}

//===================== PRINT DEBUG INFO =====================
void Print() {
  if (!dBugging) return;
  
  Serial.print("Pressure: ");
  Serial.print(actual_pressure);
  Serial.print("\tSmooth: ");
  Serial.print(actual_pressure_smoothed);
  Serial.print("\tPID_Alt: ");
  Serial.print(pid_output_altitude);
  Serial.print("\tErr X: ");
  Serial.print(gyro.error.x);
  Serial.print("\tErr Y: ");
  Serial.print(gyro.error.y);
  Serial.print("\tErr Z: ");
  Serial.println(gyro.error.z);
}
