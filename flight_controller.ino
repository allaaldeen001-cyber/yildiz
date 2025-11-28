#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>
#include <Smoothed.h>
#include <Wire.h>
#include <MS5611.h>
#include <math.h>

/*
 * INTEGRATED FLIGHT CONTROLLER (Arduino Nano / ATmega328)
 * Improvements:
 *  - Automatic gyro re-trim when disarmed (uses live data => better stability)
 *  - RC command filtering + angle limiting (30 deg max)
 *  - Linear throttle path from transmitter (expects 1000-2000µs)
 *  - Ultrasonic ground proximity warning
 *  - Integral windup protection & configurable PID clamps
 */

// ================================================================
// IMU Structures
// ================================================================

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

  static constexpr int ScaleAcc = 8192;
  static constexpr int ScaleGyro = 65.5;
  static constexpr float rad_to_deg = 180.0f / 3.141592654f;
  static constexpr float deg_to_rad = 3.141592654f / 180.0f;
  static constexpr double micro_to_sec = 0.000001;
  float limZ;
  bool GyroSet = true;

 public:
  Vec3 error;

  Gyro() {
    limZ = ScaleGyro / 100.0f;
  }

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

  void SetupWire(double TIME) {
    countTime = false;
    Time = TIME;
    setupwire();
    calibrateGyro();
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

    Acc_angle.x = atan(RawAcc.y / sqrt(pow(RawAcc.x, 2) + pow(RawAcc.z * 0.85f, 2))) * rad_to_deg;
    Acc_angle.y = -atan(RawAcc.x / sqrt(pow(RawAcc.y, 2) + pow(RawAcc.z * 0.85f, 2))) * rad_to_deg;

    if (GyroSet) {
      Gyro_angle.x = Acc_angle.x;
      Gyro_angle.y = Acc_angle.y;
      Gyro_angle.z = 0;
      GyroSet = false;
    }

    if (RawAcc.z > -100 && Acc_totalVec > 0.1f) {
      Gyro_angle.x = 0.99f * Gyro_angle.x + Acc_angle.x * 0.01f;
      Gyro_angle.y = 0.99f * Gyro_angle.y + Acc_angle.y * 0.01f;
    }

    error.x = Gyro_angle.x - target.x - cal.x;
    error.y = Gyro_angle.y - target.y - cal.y;
    error.z = Gyro_angle.z - target.z - cal.z;
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

  void calibrateGyro() {
    double x = 0, y = 0, z = 0;
    const int n = 1500;
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
    float tempX = 0;
    float tempY = 0;
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
    temp.z = 0;
    return temp;
  }

  void zeroYaw(bool lt) {
    if (lt) Gyro_angle.z = 0;
  }

  void setTarget(Vec3 Target) { target = Target; }
  void setCalibration(Vec3 Cal) { cal = Cal; }
};

// ================================================================
// GLOBAL OBJECTS / VARIABLES
// ================================================================

MS5611 MS5611(0x77);
RF24 radio(4, 10);
const uint64_t pipe = 0xF0F0F0F0E1LL;

bool but1, but2, switch1, switch2;
byte counter = 0;

struct Package {
  int thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int id = 0;
  bool but1 = 1;
  bool but2 = 1;
  bool switch1 = 1;
  bool switch2 = 1;
};

Package package;
Gyro gyro;

Servo ESCfl;
Servo ESCfr;
Servo ESCrl;
Servo ESCrr;

const float kp = 2.0f;
const float ki = 0.0001f;
const float kd = 0.5f;
const float kpZ = 2.0f;
const float pidIntegralLimit = 65.0f;
const float pidYawIntegralLimit = 40.0f;

float pid_p_gain_altitude = 14.0f;
float pid_i_gain_altitude = 2.0f;
float pid_d_gain_altitude = 7.5f;
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

float sensiX = -0.45f;
float sensiY = 0.45f;
float sensiZ = -0.01f;
float sensiThrust = 1.0f;

int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;
float hz = 140.0f;

int pMAX = 2000;
int pMIN = 1000;
int MINarmed = 1050;
int maxThrust = 1850;

const int maxAngle = 30;
bool killAngle = true;

const int flPIN = 3;
const int frPIN = 5;
const int rrPIN = 6;
const int rlPIN = 9;

Smoothed<float> smooth;

float vout = 0.0f;
float vin = 0.0f;
int real_voltage = 0;
float R1 = 1500.0f;
float R2 = 1000.0f;

const int BUZZER = 8;
const int LED = 7;

int MAX = pMAX;
int MIN = pMIN;
int thrust = pMIN;
int thrust_2 = thrust;
int killSwitch = 0;

float calCount = 0;
float NoDataCount = 0;
float armingCounter = 0;

bool dBugging = false;
bool armed = false;

double timepi = 0;
long prevTime = 0;
const float sec_to_micro = 1000000.0f;
const float micro_to_sec = 1.0f / 1000000.0f;

int FrontRight = thrust;
int FrontLeft = thrust;
int RearRight = thrust;
int RearLeft = thrust;

Vec3 PID[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
Vec3 target = {0, 0, 0};
Vec3 targetCommand = {0, 0, 0};
Vec3 cal = {0, 0, 0};
Vec3 rawCal = {0, 0, 0};
Vec3 prevError = {0, 0, 0};

struct quad_properties {
  float height;
  float kalmanvel_z;
  float baro_height;
} quadprops;

struct matrix2x2 {
  float m11;
  float m21;
  float m12;
  float m22;
} current_prob;

const int trigPin = A1;
const int echoPin = A2;
long sonicDuration;
int sonicDistance;
unsigned long sonicTimer = 0;

// Auto-trim
float autoTrimAccumulatorX = 0;
float autoTrimAccumulatorY = 0;
int autoTrimSamples = 0;
unsigned long lastAutoTrimStore = 0;
const unsigned long autoTrimStoreInterval = 15000;

// Function prototypes
void Print();
void readEEPROM();
bool receiveRadio();
void checkStatus();
void calculatePID();
void calculateVelocities();
void waitLoopEnd();
void runMotors();
void stopMotors();
void resetYaw();
void calculate_pressure();
void calculate_battery();
void led(int t);
void KalmanPosVel();
void initKalmanPosVel();
void debugging(bool dBug);
void checkGroundProximity();
void applyCommandFiltering();
void autoTrimFromGyro();

void setup() {
  Serial.begin(57600);
  Wire.begin();

  debugging(false);
  prevTime = micros();
  timepi = (1.0f / hz);

  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  tone(BUZZER, 1000, 300);
  led(300);
  delay(100);
  tone(BUZZER, 1600, 700);
  led(700);
  delay(100);
  tone(BUZZER, 2000, 200);
  led(200);

  ESCfl.attach(flPIN, 1000, 2000);
  ESCfr.attach(frPIN, 1000, 2000);
  ESCrl.attach(rlPIN, 1000, 2000);
  ESCrr.attach(rrPIN, 1000, 2000);
  stopMotors();
  delay(500);
  Serial.println(F("Motors attached"));

  radio.begin();
  radio.setChannel(108);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  Serial.println(F("Radio OK (Channel 108)"));

  readEEPROM();
  gyro.SetupWire(timepi);
  delay(500);
  tone(BUZZER, 2000, 200);
  led(200);

  MS5611.begin();
  MS5611.setOversampling(OSR_LOW);
  smooth.begin(SMOOTHED_AVERAGE, 10);

  initKalmanPosVel();
}

void loop() {
  receiveRadio();
  applyCommandFiltering();
  checkStatus();
  checkGroundProximity();

  gyro.setTarget(target);
  gyro.setCalibration(cal);

  calculate_pressure();
  gyro.calculateError();
  autoTrimFromGyro();

  calculatePID();
  calculateVelocities();
  runMotors();
  Print();
  waitLoopEnd();
}

void checkGroundProximity() {
  if (millis() - sonicTimer > 60) {
    sonicTimer = millis();

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    sonicDuration = pulseIn(echoPin, HIGH, 3000);
    sonicDistance = sonicDuration * 0.034f / 2.0f;

    if (sonicDistance > 0 && sonicDistance < 40 && armed) {
      digitalWrite(LED, HIGH);
    } else {
      digitalWrite(LED, LOW);
    }
  }
}

void calculatePID() {
  if (!armed) {
    resetYaw();
  }

  if (armed) {
    PID[0].x = gyro.error.x * kp;
    PID[0].y = gyro.error.y * kp;
    PID[0].z = gyro.error.z * kpZ;

    PID[1].x += gyro.error.x * timepi * ki;
    PID[1].y += gyro.error.y * timepi * ki;
    PID[1].z += gyro.error.z * timepi * ki;

    PID[1].x = constrain(PID[1].x, -pidIntegralLimit, pidIntegralLimit);
    PID[1].y = constrain(PID[1].y, -pidIntegralLimit, pidIntegralLimit);
    PID[1].z = constrain(PID[1].z, -pidYawIntegralLimit, pidYawIntegralLimit);

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

  int tempThrust;
  if (switch2 == 0 && thrust < 1450 && thrust > 1400) {
    tempThrust = thrust_2;
  } else {
    tempThrust = thrust;
  }

  RearLeft = tempThrust - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
  RearRight = tempThrust + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
  FrontLeft = tempThrust - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
  FrontRight = tempThrust + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
}

void runMotors() {
  if (armed)
    MIN = MINarmed;
  else
    MIN = pMIN;

  RearLeft = constrain(RearLeft, MIN, MAX);
  RearRight = constrain(RearRight, MIN, MAX);
  FrontLeft = constrain(FrontLeft, MIN, MAX);
  FrontRight = constrain(FrontRight, MIN, MAX);

  if (armed) {
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
    but1 = package.but1;
    but2 = package.but2;
    switch1 = package.switch1;
    switch2 = package.switch2;

    if (package.thrust != 0) {
      if (abs(package.z) < lowPassZ) package.z = 0;
      if (abs(package.x) < lowPassX) package.x = 0;
      if (abs(package.y) < lowPassY) package.y = 0;

      targetCommand.x = package.x * sensiX;
      targetCommand.y = package.y * sensiY;
      targetCommand.x = constrain(targetCommand.x, -maxAngle, maxAngle);
      targetCommand.y = constrain(targetCommand.y, -maxAngle, maxAngle);

      if (armed) {
        targetCommand.z += package.z * sensiZ;
        targetCommand.z = constrain(targetCommand.z, -120.0f, 120.0f);
      } else {
        targetCommand.z = 0;
      }

      thrust = package.thrust * sensiThrust;
      thrust = constrain(thrust, MIN, maxThrust);

      NoDataCount = 0;
      return true;
    }
  }

  NoDataCount += timepi;
  return false;
}

void applyCommandFiltering() {
  const float alphaXY = 0.18f;
  const float alphaZ = 0.12f;

  target.x += (targetCommand.x - target.x) * alphaXY;
  target.y += (targetCommand.y - target.y) * alphaXY;
  target.z += (targetCommand.z - target.z) * alphaZ;

  target.x = constrain(target.x, -maxAngle, maxAngle);
  target.y = constrain(target.y, -maxAngle, maxAngle);
}

void autoTrimFromGyro() {
  if (armed) {
    autoTrimAccumulatorX = 0;
    autoTrimAccumulatorY = 0;
    autoTrimSamples = 0;
    return;
  }

  if (abs(gyro.error.x) > 0.6f || abs(gyro.error.y) > 0.6f) {
    autoTrimAccumulatorX = 0;
    autoTrimAccumulatorY = 0;
    autoTrimSamples = 0;
    return;
  }

  autoTrimAccumulatorX += gyro.error.x;
  autoTrimAccumulatorY += gyro.error.y;
  autoTrimSamples++;

  if (autoTrimSamples >= 400) {
    cal.x += autoTrimAccumulatorX / autoTrimSamples;
    cal.y += autoTrimAccumulatorY / autoTrimSamples;
    autoTrimAccumulatorX = 0;
    autoTrimAccumulatorY = 0;
    autoTrimSamples = 0;
    gyro.setCalibration(cal);

    if (millis() - lastAutoTrimStore > autoTrimStoreInterval) {
      EEPROM.put(10, static_cast<float>(cal.x));
      EEPROM.put(15, static_cast<float>(cal.y));
      lastAutoTrimStore = millis();
      Serial.println(F("Auto-trim saved"));
    }
  }
}

void checkStatus() {
  if (switch1 == 0) {
    stopMotors();
    armed = false;
  }

  if (abs(gyro.error.z) > 180) resetYaw();

  if (NoDataCount > 3) killSwitch = 2;

  if (killAngle && (abs(gyro.error.x) > maxAngle || abs(gyro.error.y) > maxAngle)) {
    killSwitch = 1;
  }

  if (killSwitch > 0) {
    stopMotors();
    armed = false;

    while (killSwitch > 0) {
      tone(BUZZER, 1000, 300);
      led(300);
      delay(1000);
      if (killSwitch == 2 && radio.available()) {
        delay(500);
        if (radio.available()) {
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
      tone(BUZZER, 1500, 100);
      digitalWrite(LED, HIGH);
      if (!armed)
        armed = true;
      else {
        armed = false;
        stopMotors();
      }
      armingCounter = 0;
    }
  } else {
    digitalWrite(LED, LOW);
    armingCounter = 0;
  }

  if (but1 == 0) {
    calCount += timepi;
    if (calCount > 2) {
      stopMotors();
      armed = false;
      tone(BUZZER, 1200, 100);
      led(100);
      delay(300);
      cal = gyro.calibrate(1000);
      EEPROM.put(10, static_cast<float>(cal.x));
      EEPROM.put(15, static_cast<float>(cal.y));
      delay(500);
      gyro.setCalibration(cal);
      tone(BUZZER, 2200, 200);
      led(200);
      calCount = 0;
    }
  } else {
    calCount = 0;
  }
}

void calculate_battery() {
  real_voltage = analogRead(A0);
  vout = (real_voltage * 5.0f) / 1023.0f;
  vin = vout / (R2 / (R1 + R2));
}

void waitLoopEnd() {
  while (micros() - prevTime < timepi * sec_to_micro) {
    ;  // wait
  }
  prevTime = micros();
}

void led(int t) {
  digitalWrite(LED, HIGH);
  delay(t);
  digitalWrite(LED, LOW);
}

void readEEPROM() {
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
  if (!isfinite(cal.x)) cal.x = 0;
  if (!isfinite(cal.y)) cal.y = 0;
}

void debugging(bool dBug) {
  if (dBug) {
    dBugging = true;
    hz = 140;
  }
}

void resetYaw() {
  gyro.zeroYaw(true);
  targetCommand.z = 0;
  target.z = 0;
}

void Print() {
  if (dBugging) {
    Serial.print(F("Thr:"));
    Serial.print(thrust);
    Serial.print(F(" FL:"));
    Serial.print(FrontLeft);
    Serial.print(F(" FR:"));
    Serial.print(FrontRight);
    Serial.print(F(" RL:"));
    Serial.print(RearLeft);
    Serial.print(F(" RR:"));
    Serial.println(RearRight);
  }
}

void initKalmanPosVel() {
  current_prob.m11 = 1;
  current_prob.m21 = 0;
  current_prob.m12 = 0;
  current_prob.m22 = 1;
}

void KalmanPosVel() {
  const float timeslice = 0.007f;
  const float var_acc = 1;
  const float Q11 = var_acc * 0.25f * pow(timeslice, 4);
  const float Q12 = var_acc * 0.5f * pow(timeslice, 3);
  const float Q21 = var_acc * 0.5f * pow(timeslice, 3);
  const float Q22 = var_acc * pow(timeslice, 2);
  const float R11 = 0.008f;

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

void calculate_pressure() {
  if (counter == 0) {
    MS5611.read();
    smooth.add(MS5611.getPressure());
    counter = 10;
  }
  counter--;

  actual_pressure = smooth.get();
  quadprops.baro_height = actual_pressure;

  KalmanPosVel();
  actual_pressure_2 = quadprops.kalmanvel_z;

  if (switch2 == 0 && thrust > 1400 && thrust < 1450) {
    if (manual_altitude_change == 1) {
      pressure_parachute_previous = actual_pressure * 10;
    }

    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];
    pressure_parachute_previous = actual_pressure * 10;

    parachute_rotating_mem_location++;
    if (parachute_rotating_mem_location == 30) parachute_rotating_mem_location = 0;

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
    if (abs(pid_error_temp) > 10) {
      pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0f;
      if (pid_error_gain_altitude > 3) pid_error_gain_altitude = 3;
    }

    pid_i_mem_altitude += (pid_i_gain_altitude / 100.0f) * pid_error_temp;
    pid_i_mem_altitude = constrain(pid_i_mem_altitude, -pid_max_altitude, pid_max_altitude);

    pid_output_altitude = (100 * (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle);
    pid_output_altitude = constrain(pid_output_altitude, -pid_max_altitude, pid_max_altitude);
  } else {
    hold = 0;
  }
}
