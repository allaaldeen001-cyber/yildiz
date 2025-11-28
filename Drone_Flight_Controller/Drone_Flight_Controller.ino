/*
 * Drone Flight Controller - Main Code
 * Hardware: Arduino Nano, NRF24L01, MPU6050, MS5611, 4x ESC
 * 
 * Pin Mapping:
 * - NRF24L01: CE=4, CSN=10
 * - Motors: FL=3, FR=5, RR=6, RL=9
 * - Buzzer: D8
 * - LED: D7
 * - MPU6050 & MS5611: I2C (A4/A5)
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

// Communication status
bool radioLinked = false;
unsigned long lastPacketTime = 0;
unsigned long lastLedBlinkTime = 0;
const unsigned long COMM_TIMEOUT = 1000; // 1 second timeout

struct Package
{
  int   thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int   id = 0;
  bool  but1 = 1;      // Calibration button
  bool  but2 = 1;      // Smooth start button
  bool  switch1 = 1;   // Arm/Disarm (1=disarmed, 0=armed)
  bool  switch2 = 1;   // Altitude hold (1=off, 0=on)
};

Package package;
Gyro gyro;

Servo ESCfl;  // Front Left
Servo ESCfr;  // Front Right
Servo ESCrl;  // Rear Left
Servo ESCrr;  // Rear Right

// PID Parameters - Tuned for stability
const float kp = 2.0;
const float ki = 0.0001;
const float kd = 0.5;
const float kpZ = 2.0;

// Altitude PID Parameters
float pid_p_gain_altitude = 14.0;
float pid_i_gain_altitude = 2.0;
float pid_d_gain_altitude = 7.5;
int pid_max_altitude = 400;

float pid_error_gain_altitude, pid_throttle_gain_altitude;
float ground_pressure, altitude_hold_pressure;
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
float actual_pressure, pid_error_temp;
uint8_t manual_altitude_change;
int16_t manual_throttle;
byte hold;
float actual_pressure_filtered;
bool barometer_calibrated = false;

// Control sensitivity
float sensiX = -0.45;
float sensiY = 0.45;
float sensiZ = -0.01;
float sensiThrust = 1.1;

// Low-pass filter thresholds
int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;

// Flight parameters
float hz = 140;
int pMAX = 2000;
int pMIN = 1000;
int MINarmed = 1050;
int maxThrust = 1700;

// Safety: Maximum tilt angle (30 degrees)
const int maxAngle = 30;
bool killAngle = true;

// Motor pins
const int flPIN = 3;  // Front Left
const int frPIN = 5;  // Front Right
const int rrPIN = 6;  // Rear Right
const int rlPIN = 9;  // Rear Left

Smoothed <float> smooth;

// I/O pins
const int BUZZER = 8;
const int LED = 7;

// Motor control variables
int MAX = pMAX;
int MIN = pMIN;
int thrust = pMIN;
int thrust_adjusted = thrust;
int killSwitch = 0;

// State variables
float calCount = 0;
float NoDataCount = 0;
float armingCounter = 0;
bool lastBut2State = 1;  // For smooth start edge detection

bool dBugging = false;
bool armed = false;
bool motorsStarting = false;
int motorStartStep = 0;
unsigned long motorStartTime = 0;

// Timing
double timepi = 0;
long prevTime = 0;
byte counter = 0;

const float sec_to_micro = 1000000;
const float micro_to_sec = 1.0 / 1000000.0;
const float micro_to_ms = 0.001;
const int sec_to_ms = 1000;

// Motor outputs
int FrontRight = thrust;
int FrontLeft = thrust;
int RearRight = thrust;
int RearLeft = thrust;

// PID terms
Vec3 PID[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

Vec3 target = {0, 0, 0};
Vec3 cal = {0, 0, 0};
Vec3 prevError = {0, 0, 0};

// Kalman filter
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

// Function Prototypes
void Print();
void readEEPROM();
void writeEEPROM();
bool receiveRadio();
void checkStatus();
void calculatePID();
void calculateVelocities();
void wait();
void runMotors();
void stopMotors();
void resetYaw();
void calculate_pressure();
void calibrateBarometer();
int led(int t);
void ledBlink();
void KalmanPosVel();
void initKalmanPosVel();
void playStartupMelody();
void playCalibrationMelody();
void playLinkMelody();
void smoothMotorStart();

void setup() {
  Serial.begin(57600);
  debugging(false);
  
  prevTime = micros();
  timepi = (1.0 / hz);
  
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  
  // Startup melody
  playStartupMelody();
  
  // Attach ESCs
  ESCfl.attach(flPIN, 1000, 2000);
  ESCfr.attach(frPIN, 1000, 2000);
  ESCrl.attach(rlPIN, 1000, 2000);
  ESCrr.attach(rrPIN, 1000, 2000);
  stopMotors();
  delay(500);
  Serial.println("Motors \t attached");
  
  // Initialize NRF24L01 with ACK enabled
  radio.begin();
  radio.setAutoAck(true);  // Enable AUTO ACK
  radio.enableAckPayload();
  radio.setRetries(5, 15);  // 5 * 250us delay, 15 retries
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(108);  // Stable channel
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  Serial.println("Radio \t OK - ACK Enabled");
  
  // Read calibration from EEPROM
  readEEPROM();
  
  // Initialize MPU6050
  gyro.SetupWire(timepi);
  delay(500);
  Serial.println("MPU6050 \t OK");
  
  // Initialize MS5611
  if (MS5611.begin()) {
    MS5611.setOversampling(OSR_STANDARD);
    smooth.begin(SMOOTHED_AVERAGE, 10);
    Serial.println("MS5611 \t OK");
  } else {
    Serial.println("MS5611 \t FAILED");
    tone(BUZZER, 500, 1000);
  }
  
  initKalmanPosVel();
  
  tone(BUZZER, 2000, 200);
  led(200);
  
  Serial.println("=== Drone Flight Controller Ready ===");
  Serial.println("Waiting for RC link...");
}

void loop() {
  bool packetReceived = receiveRadio();
  
  // LED blinks when receiving data
  if (packetReceived) {
    ledBlink();
  }
  
  checkStatus();
  gyro.setTarget(target);
  gyro.setCalibration(cal);
  calculate_pressure();
  gyro.calculateError();
  calculatePID();
  calculateVelocities();
  runMotors();
  Print();
  wait();
}

void calculatePID() {
  if (armed == false) {
    resetYaw();
  }
  
  if (armed == true) {
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
  } else {
    PID[0] = {0, 0, 0};
    PID[1] = {0, 0, 0};
    PID[2] = {0, 0, 0};
    prevError = {0, 0, 0};
  }
}

void calculateVelocities() {
  // Altitude hold mode
  if (package.switch2 == 0 && thrust > 1400 && thrust < 1450 && barometer_calibrated) {
    thrust_adjusted = 1450 + pid_output_altitude + manual_throttle;
  } else {
    thrust_adjusted = thrust;
  }
  
  // Calculate motor speeds with PID corrections
  RearLeft = thrust_adjusted - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
  RearRight = thrust_adjusted + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
  FrontLeft = thrust_adjusted - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
  FrontRight = thrust_adjusted + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
}

void runMotors() {
  if (armed == true)
    MIN = MINarmed;
  else
    MIN = pMIN;
  
  // Constrain motor values
  RearLeft = constrain(RearLeft, MIN, MAX);
  RearRight = constrain(RearRight, MIN, MAX);
  FrontLeft = constrain(FrontLeft, MIN, MAX);
  FrontRight = constrain(FrontRight, MIN, MAX);
  
  // Send to ESCs only if armed
  if (armed == true) {
    ESCfl.writeMicroseconds(FrontLeft);
    ESCfr.writeMicroseconds(FrontRight);
    ESCrl.writeMicroseconds(RearLeft);
    ESCrr.writeMicroseconds(RearRight);
  } else {
    stopMotors();
  }
}

void stopMotors() {
  ESCfl.writeMicroseconds(1000);
  ESCfr.writeMicroseconds(1000);
  ESCrl.writeMicroseconds(1000);
  ESCrr.writeMicroseconds(1000);
  
  MIN = pMIN;
  FrontRight = pMIN;
  FrontLeft = pMIN;
  RearLeft = pMIN;
  RearRight = pMIN;
}

bool receiveRadio() {
  if (radio.available()) {
    radio.read(&package, sizeof(package));
    lastPacketTime = millis();
    
    // First packet after connection
    if (!radioLinked) {
      radioLinked = true;
      playLinkMelody();
      Serial.println("RC LINKED!");
    }
    
    if (package.thrust != 0) {
      // Apply low-pass filters
      if (abs(package.z) < lowPassZ) package.z = 0;
      if (abs(package.x) < lowPassX) package.x = 0;
      if (abs(package.y) < lowPassY) package.y = 0;
      
      // Apply sensitivity
      target.x = package.x * sensiX;
      target.y = package.y * sensiY;
      
      if (armed == true)
        target.z += package.z * sensiZ;
      
      thrust = constrain(package.thrust * sensiThrust, MIN, maxThrust);
      
      NoDataCount = 0;
      return true;
    } else {
      NoDataCount += timepi;
      return false;
    }
  } else {
    NoDataCount += timepi;
    
    // Check for communication timeout
    if (radioLinked && (millis() - lastPacketTime > COMM_TIMEOUT)) {
      radioLinked = false;
      Serial.println("RC LINK LOST!");
    }
    
    return false;
  }
}

void checkStatus() {
  // LED stays ON when disarmed (warning)
  if (package.switch1 == 1) {
    digitalWrite(LED, HIGH);
    if (armed) {
      stopMotors();
      armed = false;
      Serial.println("DISARMED");
    }
  }
  
  // Reset yaw if angle too large
  if (abs(gyro.error.z) > 180)
    resetYaw();
  
  // Kill switch on communication loss
  if (NoDataCount > 3) {
    killSwitch = 2;
  }
  
  // Kill switch on excessive angle
  if (killAngle && (abs(gyro.error.x) > maxAngle || abs(gyro.error.y) > maxAngle)) {
    killSwitch = 1;
    Serial.println("KILL: Angle exceeded!");
  }
  
  // Handle kill switch
  if (killSwitch > 0) {
    stopMotors();
    armed = false;
    
    while (killSwitch > 0) {
      tone(BUZZER, 1000, 300);
      led(300);
      delay(2000);
      
      if (killSwitch == 2 && radio.available()) {
        delay(500);
        if (radio.available()) {
          tone(BUZZER, 1500, 1000);
          led(1000);
          killSwitch = 0;
          radioLinked = true;
          Serial.println("Communication restored");
        }
      }
      
      // Manual reset for angle kill
      if (killSwitch == 1) {
        Serial.println("Reset required - check drone orientation");
        delay(3000);
        if (abs(gyro.error.x) < maxAngle && abs(gyro.error.y) < maxAngle) {
          killSwitch = 0;
          tone(BUZZER, 2000, 500);
        }
      }
    }
  }
  
  // Calibration button (but1)
  if (package.but1 == 0 && package.switch1 == 1) {  // Only when disarmed
    calCount += timepi;
    
    if (calCount > 2) {
      stopMotors();
      playCalibrationMelody();
      Serial.println("=== CALIBRATION START ===");
      
      // Calibrate MPU6050
      Serial.println("Calibrating MPU6050...");
      cal = gyro.calibrate(1000);
      writeEEPROM();
      gyro.setCalibration(cal);
      
      // Calibrate MS5611
      Serial.println("Calibrating MS5611...");
      calibrateBarometer();
      
      tone(BUZZER, 2200, 500);
      led(500);
      Serial.println("=== CALIBRATION COMPLETE ===");
      
      calCount = 0;
    }
  } else {
    calCount = 0;
  }
  
  // Smooth motor start button (but2) - edge detection
  if (package.but2 == 0 && lastBut2State == 1 && package.switch1 == 0 && armed) {
    smoothMotorStart();
  }
  lastBut2State = package.but2;
  
  // Arming (switch1 transition from 1 to 0)
  if (package.switch1 == 0 && !armed) {
    armingCounter += timepi;
    
    if (armingCounter > 2) {
      armed = true;
      tone(BUZZER, 1800, 500);
      Serial.println("ARMED");
      armingCounter = 0;
    }
  } else {
    armingCounter = 0;
  }
}

void smoothMotorStart() {
  Serial.println("=== SMOOTH MOTOR START ===");
  tone(BUZZER, 1500, 200);
  
  // Ramp motors from 1050 to 1200 over 3 seconds
  for (int i = 1050; i <= 1200; i += 5) {
    ESCfl.writeMicroseconds(i);
    ESCfr.writeMicroseconds(i);
    ESCrl.writeMicroseconds(i);
    ESCrr.writeMicroseconds(i);
    delay(30);
  }
  
  // Hold for 2 seconds
  delay(2000);
  
  // Ramp down
  for (int i = 1200; i >= 1050; i -= 5) {
    ESCfl.writeMicroseconds(i);
    ESCfr.writeMicroseconds(i);
    ESCrl.writeMicroseconds(i);
    ESCrr.writeMicroseconds(i);
    delay(30);
  }
  
  tone(BUZZER, 2000, 200);
  Serial.println("Motor test complete");
}

void wait() {
  while (micros() - prevTime < timepi * sec_to_micro);
  prevTime = micros();
}

int led(int t) {
  digitalWrite(LED, HIGH);
  delay(t);
  digitalWrite(LED, LOW);
  return t;
}

void ledBlink() {
  unsigned long currentTime = millis();
  if (currentTime - lastLedBlinkTime > 100 && package.switch1 == 0) {  // Only blink when armed
    digitalWrite(LED, !digitalRead(LED));
    lastLedBlinkTime = currentTime;
  }
}

void readEEPROM() {
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
  Serial.print("Loaded calibration: X=");
  Serial.print(cal.x);
  Serial.print(" Y=");
  Serial.println(cal.y);
}

void writeEEPROM() {
  EEPROM.put(10, cal.x);
  EEPROM.put(15, cal.y);
  Serial.println("Calibration saved to EEPROM");
}

void debugging(bool dBug) {
  if (dBug == true) {
    dBugging = true;
    Serial.begin(57600);
    hz = 140;
  }
}

void resetYaw() {
  gyro.zeroYaw(true);
  target.z = 0;
}

void playStartupMelody() {
  tone(BUZZER, 1000, 300);
  led(300);
  delay(100);
  tone(BUZZER, 1600, 700);
  led(700);
  delay(100);
  tone(BUZZER, 2000, 200);
  led(200);
}

void playCalibrationMelody() {
  tone(BUZZER, 1200, 100);
  led(100);
  delay(300);
  tone(BUZZER, 1200, 200);
  led(200);
  delay(300);
}

void playLinkMelody() {
  tone(BUZZER, 2500, 200);
  digitalWrite(LED, HIGH);
  delay(200);
  tone(BUZZER, 3000, 200);
  delay(200);
  digitalWrite(LED, LOW);
}

void Print() {
  if (dBugging) {
    Serial.print("Pressure: ");
    Serial.print(actual_pressure);
    Serial.print("\t Filtered: ");
    Serial.print(actual_pressure_filtered);
    Serial.print("\t PID Alt: ");
    Serial.print(pid_output_altitude);
    Serial.print("\t Angles: X=");
    Serial.print(gyro.error.x);
    Serial.print(" Y=");
    Serial.print(gyro.error.y);
    Serial.print(" Z=");
    Serial.print(gyro.error.z);
    Serial.print("\t Motors: FL=");
    Serial.print(FrontLeft);
    Serial.print(" FR=");
    Serial.print(FrontRight);
    Serial.print(" RL=");
    Serial.print(RearLeft);
    Serial.print(" RR=");
    Serial.println(RearRight);
  }
}
