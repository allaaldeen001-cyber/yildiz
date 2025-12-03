/*
 * QUADCOPTER FLIGHT CONTROLLER - COMPLETE INTEGRATED VERSION
 * Target: Arduino Nano
 * Motors: RS2205 2300KV
 * IMU: MPU6050 (Pin D2)
 * Barometer: MS5611
 * Radio: NRF24L01 (Channel 103)
 */

#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <EEPROM.h>
#include <Wire.h>
#include "MS5611.h"

// ============================================================================
// GYRO CLASS DEFINITION (Gyro.h inline)
// ============================================================================
struct Vec3 {
  float x, y, z;
};

class Gyro {
private:
  Vec3 GyroScaled;
  Vec3 Gyro_angle;
  Vec3 RawAcc;
  Vec3 RawGyro;
  float tmp = 0;
  Vec3 Acc_angle;
  Vec3 target;
  Vec3 cal;
  Vec3 GyroCal;
  float Acc_totalVec;
  
  double Time = 0;
  double prevTime = 0;
  bool countTime = false;
  
  const int    ScaleAcc = 4096;
  const int    ScaleGyro = 65.5;
  const float  rad_to_deg = 180 / 3.141592654;
  const float  deg_to_rad = 3.141592654 / 180;
  double       micro_to_sec = 0.000001;
  float limZ = ScaleGyro / 100;
  
  bool GyroSet = true;

  void setupwire() {
    Wire.begin();
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();
    
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    
    delay(100);
  }

public:
  Vec3 error;
  
  Gyro() {}
  
  void SetupWire(double TIME) {
    countTime = false;
    Time = TIME;
    setupwire();
    calibrateGyro();
  }
  
  void SetupWire() {
    countTime = true;
    setupwire();
    calibrateGyro();
  }
  
  void calculateError() {
    if (countTime) {
      Time = micros() - prevTime;
      Time *= micro_to_sec;
      prevTime = micros();
    }
    
    readingMPU();
    calculateAngle();
  }
  
  void setTarget(Vec3 Target) {
    target = Target;
  }
  
  void setCalibration(Vec3 Cal) {
    cal = Cal;
  }
  
  void readingMPU() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);
    
    RawAcc.x = Wire.read() << 8 | Wire.read();
    RawAcc.y = Wire.read() << 8 | Wire.read();
    RawAcc.z = Wire.read() << 8 | Wire.read();
    tmp = Wire.read() << 8 | Wire.read();
    RawGyro.x = Wire.read() << 8 | Wire.read();
    RawGyro.y = Wire.read() << 8 | Wire.read();
    RawGyro.z = Wire.read() << 8 | Wire.read();
  }
  
  void calculateAngle() {
    GyroScaled.x = ((RawGyro.x - GyroCal.x) / ScaleGyro);
    GyroScaled.y = ((RawGyro.y - GyroCal.y) / ScaleGyro);
    
    if ((RawGyro.z - GyroCal.z) < limZ && (RawGyro.z - GyroCal.z) > (-limZ))
      GyroScaled.z = 0;
    else
      GyroScaled.z = ((RawGyro.z - GyroCal.z) / ScaleGyro);
    
    Gyro_angle.x += GyroScaled.x * Time;
    Gyro_angle.y += GyroScaled.y * Time;
    Gyro_angle.z += GyroScaled.z * Time;
    
    Gyro_angle.x += Gyro_angle.y * sin(GyroScaled.z * Time * deg_to_rad);
    Gyro_angle.y -= Gyro_angle.x * sin(GyroScaled.z * Time * deg_to_rad);
    
    Acc_totalVec = sqrt(pow(RawAcc.x, 2) + pow(RawAcc.y, 2) + pow(RawAcc.z, 2)) / ScaleAcc;
    
    Acc_angle.x = atan(RawAcc.y / sqrt(pow(RawAcc.x, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;
    Acc_angle.y = -atan(RawAcc.x / sqrt(pow(RawAcc.y, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;
    
    if (GyroSet) {
      Gyro_angle.x = Acc_angle.x;
      Gyro_angle.y = Acc_angle.y;
      Gyro_angle.z = 0;
      GyroSet = false;
    }
    
    if (RawAcc.z > -100 && Acc_totalVec > 0.1) {
      Gyro_angle.x = 0.99 * Gyro_angle.x + Acc_angle.x * 0.01;
      Gyro_angle.y = 0.99 * Gyro_angle.y + Acc_angle.y * 0.01;
    }
    
    error.x = Gyro_angle.x - target.x - cal.x;
    error.y = Gyro_angle.y - target.y - cal.y;
    error.z = Gyro_angle.z - target.z - cal.z;
  }
  
  void calibrateGyro() {
    double x = 0, y = 0, z = 0;
    int n = 1500;
    
    for (int i = 0; i < n; i++) {
      readingMPU();
      x += RawGyro.x;
      y += RawGyro.y;
      z += RawGyro.z;
    }
    
    delay(100);
    
    GyroCal.x = x / n;
    GyroCal.y = y / n;
    GyroCal.z = z / n;
  }
  
  Vec3 calibrate(int n) {
    float tempX = 0, tempY = 0;
    Vec3 temp;
    
    setTarget({0, 0, 0});
    setCalibration({0, 0, 0});
    calibrateGyro();
    
    for (int i = 0; i < n; i++) {
      calculateError();
      tempX += error.x;
      tempY += error.y;
    }
    
    temp.x = tempX / n;
    temp.y = tempY / n;
    
    return temp;
  }
  
  void zeroYaw(bool lt) {
    if (lt)
      Gyro_angle.z = 0;
  }
};

// ============================================================================
// GLOBALS AND OBJECTS
// ============================================================================
MS5611 MS5611(0x77);
RF24 radio(4, 10);
const uint64_t pipe = 0xF0F0F0F0E1LL;

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

Package package;
Gyro gyro;

Servo ESCfl, ESCfr, ESCrl, ESCrr;

bool but1, but2, switch1, switch2;

// PID Parameters
const float kp = 2.0;
const float ki = 0.0001;
const float kd = 0.5;
const float kpZ = 2.0;

// Altitude PID
float pid_p_gain_altitude = 14.0;
float pid_i_gain_altitude = 2.0;
float pid_d_gain_altitude = 7.5;
int pid_max_altitude = 400;

float pid_error_gain_altitude, pid_throttle_gain_altitude;
float ground_pressure, altutude_hold_pressure;
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;
float actual_pressure, pid_error_temp;
uint8_t manual_altitude_change;
int16_t manual_throttle;
byte hold;
float actual_pressure_2;
byte counter = 0;

// Kalman Filter
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

// Control Sensitivity
float sensiX = -0.45;
float sensiY = 0.45;
float sensiZ = -0.01;
float sensiThrust = 1.1;

// Low Pass Filters
int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;

// Loop Frequency
float hz = 140;

// Motor Limits
int pMAX = 2000;
int pMIN = 1000;
int MINarmed = 1050;
int maxThrust = 1700;
int maxAngle = 180;
bool killAngle = true;

// Pin Definitions
const int flPIN = 3;
const int frPIN = 5;
const int rrPIN = 6;
const int rlPIN = 9;
const int BUZZER = 8;
const int LED = 7;

// Motor Outputs
int MAX = pMAX;
int MIN = pMIN;
int thrust = pMIN;
int thrust_2 = thrust;
int killSwitch = 0;
int FrontRight = thrust;
int FrontLeft = thrust;
int RearRight = thrust;
int RearLeft = thrust;

// State Variables
float calCount = 0;
float NoDataCount = 0;
float armingCounter = 0;
bool armed = false;

// Timing
double timepi = 0;
long prevTime = 0;
const float sec_to_micro = 1000000;
const float micro_to_sec = 1.0 / 1000000.0;
const float micro_to_ms = 0.001;
const int sec_to_ms = 1000;

// PID Arrays
Vec3 PID[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
Vec3 target = {0, 0, 0};
Vec3 cal = {0, 0, 0};
Vec3 prevError = {0, 0, 0};

// ============================================================================
// KALMAN FILTER FUNCTIONS
// ============================================================================
void initKalmanPosVel(void) {
  current_prob.m11 = 1;
  current_prob.m21 = 0;
  current_prob.m12 = 0;
  current_prob.m22 = 1;
}

#define timeslice 0.007
#define var_acc 1

void KalmanPosVel() {
  const float Q11 = var_acc * 0.25 * (timeslice * timeslice * timeslice * timeslice);
  const float Q12 = var_acc * 0.5 * (timeslice * timeslice * timeslice);
  const float Q21 = var_acc * 0.5 * (timeslice * timeslice * timeslice);
  const float Q22 = var_acc * (timeslice * timeslice);
  const float R11 = 0.008;
  
  float ps1, ps2, opt;
  float pp11, pp12, pp21, pp22;
  float inn, ic, kg1, kg2;
  
  ps1 = quadprops.height + timeslice * quadprops.kalmanvel_z;
  ps2 = quadprops.kalmanvel_z;
  
  opt = timeslice * current_prob.m22;
  pp12 = current_prob.m12 + opt + Q12;
  pp21 = current_prob.m21 + opt;
  pp11 = current_prob.m11 + timeslice * (current_prob.m12 + pp21) + Q11;
  pp21 += Q21;
  pp22 = current_prob.m22 + Q22;
  
  inn = quadprops.baro_height - ps1;
  ic = pp11 + R11;
  
  kg1 = pp11 / ic;
  kg2 = pp21 / ic;
  
  quadprops.height = ps1 + kg1 * inn;
  quadprops.kalmanvel_z = ps2 + kg2 * inn;
  
  opt = 1 - kg1;
  current_prob.m11 = pp11 * opt;
  current_prob.m12 = pp12 * opt;
  current_prob.m21 = pp21 - pp11 * kg2;
  current_prob.m22 = pp22 - pp12 * kg2;
}

// ============================================================================
// BAROMETER FUNCTIONS
// ============================================================================
void calculate_pressure() {
  if (counter == 0) {
    MS5611.read();
    actual_pressure = MS5611.getPressure();
    counter = 10;
  }
  counter--;
  
  quadprops.baro_height = actual_pressure;
  KalmanPosVel();
  actual_pressure_2 = quadprops.kalmanvel_z;
  
  if (package.switch2 == 0 && thrust > 1400 && thrust < 1450) {
    if (manual_altitude_change == 1) {
      pressure_parachute_previous = actual_pressure * 10;
    }
    
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];
    pressure_parachute_previous = actual_pressure * 10;
    parachute_rotating_mem_location++;
    
    if (parachute_rotating_mem_location == 30)
      parachute_rotating_mem_location = 0;
    
    if (switch2 == 0 && thrust < 1450 && thrust > 1400) {
      if (hold == 0) {
        pid_altitude_setpoint = actual_pressure;
        hold = 1;
      }
      
      manual_altitude_change = 0;
      manual_throttle = 0;
      
      if (thrust > 1450) {
        manual_altitude_change = 1;
        pid_altitude_setpoint = actual_pressure;
        manual_throttle = (thrust - 1450) / 3;
      }
      
      if (thrust < 1400) {
        manual_altitude_change = 1;
        pid_altitude_setpoint = actual_pressure;
        manual_throttle = (thrust - 1400) / 5;
      }
      
      pid_altitude_input = actual_pressure;
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;
      
      pid_error_gain_altitude = 0;
      if (pid_error_temp > 10 || pid_error_temp < -10) {
        pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;
        if (pid_error_gain_altitude > 3)
          pid_error_gain_altitude = 3;
      }
      
      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
      if (pid_i_mem_altitude > pid_max_altitude)
        pid_i_mem_altitude = pid_max_altitude;
      else if (pid_i_mem_altitude < pid_max_altitude * -1)
        pid_i_mem_altitude = pid_max_altitude * -1;
      
      pid_output_altitude = (100 * (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle);
      
      if (pid_output_altitude > pid_max_altitude)
        pid_output_altitude = pid_max_altitude;
      else if (pid_output_altitude < pid_max_altitude * -1)
        pid_output_altitude = pid_max_altitude * -1;
    }
  } else {
    hold = 0;
  }
}

// ============================================================================
// CORE FUNCTIONS
// ============================================================================
void calculatePID() {
  if (armed == false)
    resetYaw();
  
  if (armed == true) {
    PID[0].x = gyro.error.x * kp;
    PID[0].y = gyro.error.y * kp;
    PID[0].z = gyro.error.z * kpZ;
    
    PID[1].x += gyro.error.x * timepi * ki;
    PID[1].y += gyro.error.y * timepi * ki;
    PID[1].z += gyro.error.z * timepi * ki;
    
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
  thrust_2 = (1450 + pid_output_altitude + manual_throttle);
  
  if (package.switch2 == 0 && thrust < 1450 && thrust > 1400) {
    RearLeft = thrust_2 - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
    RearRight = thrust_2 + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
    FrontLeft = thrust_2 - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
    FrontRight = thrust_2 + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
  } else {
    RearLeft = thrust - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
    RearRight = thrust + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
    FrontLeft = thrust - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
    FrontRight = thrust + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
  }
}

void runMotors() {
  if (armed == true)
    MIN = MINarmed;
  else
    MIN = pMIN;
  
  RearLeft = constrain(RearLeft, MIN, MAX);
  RearRight = constrain(RearRight, MIN, MAX);
  FrontLeft = constrain(FrontLeft, MIN, MAX);
  FrontRight = constrain(FrontRight, MIN, MAX);
  
  if (armed == true) {
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
  
  MIN = pMIN;
  FrontRight = pMIN;
  FrontLeft = pMIN;
  RearLeft = pMIN;
  RearRight = pMIN;
}

bool receiveRadio() {
  if (radio.available()) {
    radio.read(&package, sizeof(package));
    but1 = package.but1;
    but2 = package.but2;
    switch1 = package.switch1;
    switch2 = package.switch2;
    
    if (package.thrust != 0) {
      if (package.z < lowPassZ && package.z > -lowPassZ)
        package.z = 0;
      if (package.x < lowPassX && package.x > -lowPassX)
        package.x = 0;
      if (package.y < lowPassY && package.y > -lowPassY)
        package.y = 0;
      
      target.x = package.x * sensiX;
      target.y = package.y * sensiY;
      
      if (armed == true)
        target.z += package.z * sensiZ;
      
      thrust = package.thrust * sensiThrust;
      thrust = constrain(thrust, MIN, maxThrust);
      
      NoDataCount = 0;
      return true;
    } else if (package.thrust == 0) {
      NoDataCount += timepi;
      return false;
    }
  } else {
    NoDataCount += timepi;
    return false;
  }
}

void checkStatus() {
  if (switch1 == 0) {
    stopMotors();
    armed = false;
  }
  
  if (gyro.error.z > 180 || gyro.error.z < -180)
    resetYaw();
  
  if (NoDataCount > 3)
    killSwitch = 2;
  
  if (gyro.error.x > maxAngle || gyro.error.x < (-maxAngle)) {
    if (killAngle == true)
      killSwitch = 1;
  }
  
  if (gyro.error.y > maxAngle || gyro.error.y < (-maxAngle)) {
    if (killAngle == true)
      killSwitch = 1;
  }
  
  if (killSwitch > 0) {
    stopMotors();
    
    while (killSwitch > 0) {
      tone(BUZZER, 1000, 300);
      digitalWrite(LED, HIGH);
      delay(300);
      digitalWrite(LED, LOW);
      delay(2000);
      
      if (killSwitch == 2 && radio.available()) {
        delay(500);
        if (radio.available()) {
          tone(BUZZER, 1500, 1000);
          digitalWrite(LED, HIGH);
          delay(1000);
          digitalWrite(LED, LOW);
          killSwitch = 0;
          armed = false;
        }
      }
    }
  }
  
  if (but2 == 0) {
    armingCounter += timepi;
    resetYaw();
    
    if (armingCounter > 2) {
      tone(BUZZER, 1500, 500);
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW);
      
      if (armed == false)
        armed = true;
      
      armingCounter = 0;
    }
  } else {
    armingCounter = 0;
  }
  
  if (but1 == 0) {
    calCount += timepi;
    
    if (calCount > 2) {
      stopMotors();
      tone(BUZZER, 1200, 100);
      digitalWrite(LED, HIGH);
      delay(100);
      digitalWrite(LED, LOW);
      delay(300);
      tone(BUZZER, 1200, 200);
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      
      cal = gyro.calibrate(1000);
      
      EEPROM.put(10, static_cast<float>(cal.x));
      EEPROM.put(15, static_cast<float>(cal.y));
      
      delay(500);
      
      gyro.setCalibration(cal);
      tone(BUZZER, 2200, 200);
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      
      delay(1000);
      calCount = 0;
    }
  } else {
    calCount = 0;
  }
}

void wait() {
  while (micros() - prevTime < timepi * sec_to_micro);
  prevTime = micros();
}

void readEEPROM() {
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
}

void resetYaw() {
  gyro.zeroYaw(true);
  target.z = 0;
}

void Print() {
  Serial.print("actual_pressure= ");
  Serial.print(actual_pressure);
  Serial.print("\t");
  Serial.print("actual_pressure_2= ");
  Serial.print(actual_pressure_2);
  Serial.println("\t");
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(57600);
  prevTime = micros();
  timepi = (1.0 / hz);
  
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  
  tone(BUZZER, 1000, 300);
  digitalWrite(LED, HIGH);
  delay(300);
  digitalWrite(LED, LOW);
  delay(100);
  tone(BUZZER, 1600, 700);
  digitalWrite(LED, HIGH);
  delay(700);
  digitalWrite(LED, LOW);
  delay(100);
  tone(BUZZER, 2000, 200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  
  ESCfl.attach(flPIN, 1000, 2000);
  ESCfr.attach(frPIN, 1000, 2000);
  ESCrl.attach(rlPIN, 1000, 2000);
  ESCrr.attach(rrPIN, 1000, 2000);
  stopMotors();
  delay(500);
  Serial.println("Motors attached");
  
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(103);
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  Serial.println("Radio OK - Channel 103");
  
  readEEPROM();
  gyro.SetupWire(timepi);
  delay(500);
  tone(BUZZER, 2000, 200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  
  MS5611.begin();
  MS5611.setOversampling(OSR_LOW);
  initKalmanPosVel();
  
  Serial.println("Flight Controller Ready");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  receiveRadio();
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
