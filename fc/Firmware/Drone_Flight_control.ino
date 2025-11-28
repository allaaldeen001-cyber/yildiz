#include <Servo.h>
#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Smoothed.h>
#include "MS5611.h"
#include "Gyro.h"

MS5611 MS5611(0x77);

constexpr uint8_t PIN_RF_CE = 4;
constexpr uint8_t PIN_RF_CSN = 10;
constexpr uint8_t PIN_ESC_FL = 3;
constexpr uint8_t PIN_ESC_FR = 5;
constexpr uint8_t PIN_ESC_RR = 6;
constexpr uint8_t PIN_ESC_RL = 9;
constexpr uint8_t PIN_LED = 7;
constexpr uint8_t PIN_BUZZER = 8;

RF24 radio(PIN_RF_CE, PIN_RF_CSN);
const uint64_t pipe = 0xF0F0F0F0E1LL;
const uint8_t RF_CHANNEL = 108;

bool but1 = true;
bool but2 = true;
bool switch1 = true;
bool switch2 = true;

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

float pid_p_gain_altitude = 14.0f;
float pid_i_gain_altitude = 2.0f;
float pid_d_gain_altitude = 7.5f;
int pid_max_altitude = 400;

float pid_error_gain_altitude = 0;
float pid_throttle_gain_altitude = 0;
float ground_pressure = 0;
float altutude_hold_pressure = 0;
float pid_i_mem_altitude = 0;
float pid_altitude_setpoint = 0;
float pid_altitude_input = 0;
float pid_output_altitude = 0;
float pid_last_altitude_d_error = 0;
uint8_t parachute_rotating_mem_location = 0;
int32_t parachute_buffer[35] = {0};
int32_t parachute_throttle = 0;
float pressure_parachute_previous = 0;
int32_t pressure_rotating_mem[50] = {0};
int32_t pressure_total_avarage = 0;
uint8_t pressure_rotating_mem_location = 0;
float pressure_rotating_mem_actual = 0;
float actual_pressure = 0;
float pid_error_temp = 0;
uint8_t manual_altitude_change = 0;
int16_t manual_throttle = 0;
byte hold = 0;
float actual_pressure_2 = 0;

float sensiX = -0.45f;
float sensiY = 0.45f;
float sensiZ = -0.01f;
float sensiThrust = 1.1f;

int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;

float hz = 140;

int pMAX = 2000;
int pMIN = 1000;
int MINarmed = 1050;
int maxThrust = 1700;

constexpr int MAX_ANGLE = 30;
bool killAngle = true;

Smoothed<float> smooth;

float vout = 0.0f;
float vin = 0.0f;
int real_voltage = 0;
float R1 = 1500.0f;
float R2 = 1000.0f;

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
bool smoothStartComplete = false;
bool smoothStartPending = false;
bool linkActive = false;

unsigned long lastRxMicros = 0;
unsigned long ledPulseUntil = 0;

long prevTime = 0;
double timepi = 0;

const float sec_to_micro = 1000000;
const float micro_to_sec = 1.0f / 1000000.0f;
const float micro_to_ms = 0.001f;
const int sec_to_ms = 1000;

int FrontRight = thrust;
int FrontLeft = thrust;
int RearRight = thrust;
int RearLeft = thrust;

Vec3 PID[3] = {
  {0, 0, 0},
  {0, 0, 0},
  {0, 0, 0}
};

Vec3 target = {0, 0, 0};
Vec3 cal = {0, 0, 0};
Vec3 rawCal = {0, 0, 0};
Vec3 prevError = {0, 0, 0};

struct quad_properties {
  float height;
  float kalmanvel_z;
  float baro_height;
};
quad_properties quadprops{0, 0, 0};

struct matrix2x2 {
  float m11;
  float m21;
  float m12;
  float m22;
};
matrix2x2 current_prob{0, 0, 0, 0};

void Print();
void readEEPROM();
bool receiveRadio();
void checkStatus();
void calculatePID();
void calculateVelocities();
void waitLoop();
void runMotors();
void stopMotors();
void resetYaw();
void calculate_pressure();
void calculate_battery();
int ledFlash(int duration);
void KalmanPosVel();
void initKalmanPosVel();
void handleCalibrationButton();
void handleSmoothStartButton();
void updateStatusLed();
void updateArmingState();
void announceLink(bool state);
void calibrateBarometer();
void debugging(bool dBug);

void setup() {
  Serial.begin(57600);
  debugging(false);
  prevTime = micros();
  timepi = (1 / hz);

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);

  tone(PIN_BUZZER, 1000, 300);
  ledFlash(300);
  delay(100);
  tone(PIN_BUZZER, 1600, 400);
  ledFlash(400);
  delay(100);
  tone(PIN_BUZZER, 2000, 200);
  ledFlash(200);

  ESCfl.attach(PIN_ESC_FL, 1000, 2000);
  ESCfr.attach(PIN_ESC_FR, 1000, 2000);
  ESCrl.attach(PIN_ESC_RL, 1000, 2000);
  ESCrr.attach(PIN_ESC_RR, 1000, 2000);
  stopMotors();
  delay(500);
  Serial.println("Motors\tattached");

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(RF_CHANNEL);
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  Serial.println("Radio\tOK");

  readEEPROM();
  gyro.SetupWire(timepi);
  delay(200);

  MS5611.begin();
  MS5611.setOversampling(OSR_ULTRA);
  smooth.begin(SMOOTHED_AVERAGE, 10);
  calibrateBarometer();
  initKalmanPosVel();

  tone(PIN_BUZZER, 2000, 200);
  ledFlash(200);
}

void loop() {
  bool packet = receiveRadio();
  if (!packet) {
    if (micros() - lastRxMicros > 500000) {
      announceLink(false);
    }
  }

  updateArmingState();
  handleCalibrationButton();
  handleSmoothStartButton();
  checkStatus();
  gyro.setTarget(target);
  gyro.setCalibration(cal);
  calculate_pressure();
  gyro.calculateError();
  calculatePID();
  calculateVelocities();
  runMotors();
  updateStatusLed();
  Print();
  waitLoop();
}

void updateArmingState() {
  bool desired = (switch1 == 0) && (killSwitch == 0) && linkActive;
  if (!desired && armed) {
    armed = false;
    smoothStartComplete = false;
    stopMotors();
    tone(PIN_BUZZER, 900, 200);
  } else if (desired && !armed) {
    armed = true;
    smoothStartComplete = false;
    tone(PIN_BUZZER, 1500, 200);
    ledFlash(200);
  }
}

void handleCalibrationButton() {
  if (switch1 == 0) {
    calCount = 0;
    return;
  }

  if (but1 == 0) {
    calCount += timepi;
    if (calCount > 2.0f) {
      stopMotors();
      tone(PIN_BUZZER, 1200, 150);
      ledFlash(150);
      delay(200);
      tone(PIN_BUZZER, 1200, 200);
      ledFlash(200);

      cal = gyro.calibrate(1000);
      calibrateBarometer();

      EEPROM.put(10, static_cast<float>(cal.x));
      EEPROM.put(15, static_cast<float>(cal.y));

      gyro.setCalibration(cal);
      tone(PIN_BUZZER, 2200, 200);
      ledFlash(200);
      calCount = 0;
    }
  } else {
    calCount = 0;
  }
}

void handleSmoothStartButton() {
  if (!armed) {
    smoothStartPending = false;
    smoothStartComplete = false;
    return;
  }

  if (but2 == 0 && !smoothStartComplete) {
    smoothStartPending = true;
  }

  if (smoothStartPending) {
    const int startValue = pMIN;
    const int targetValue = MINarmed;
    for (int pulse = startValue; pulse <= targetValue; pulse += 5) {
      ESCfl.writeMicroseconds(pulse);
      ESCfr.writeMicroseconds(pulse);
      ESCrl.writeMicroseconds(pulse);
      ESCrr.writeMicroseconds(pulse);
      delay(15);
    }
    smoothStartPending = false;
    smoothStartComplete = true;
    tone(PIN_BUZZER, 1800, 150);
  }
}

void announceLink(bool state) {
  if (state == linkActive) {
    return;
  }
  linkActive = state;
  if (linkActive) {
    tone(PIN_BUZZER, 1900, 200);
    ledFlash(200);
  } else {
    tone(PIN_BUZZER, 600, 300);
  }
}

void updateStatusLed() {
  if (switch1 == 1) {
    digitalWrite(PIN_LED, HIGH);
    return;
  }

  if (millis() < ledPulseUntil) {
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
  }
}

bool receiveRadio() {
  if (radio.available()) {
    radio.read(&package, sizeof(package));
    but1 = package.but1;
    but2 = package.but2;
    switch1 = package.switch1;
    switch2 = package.switch2;

    if (abs(package.z) < lowPassZ) {
      package.z = 0;
    }
    if (abs(package.x) < lowPassX) {
      package.x = 0;
    }
    if (abs(package.y) < lowPassY) {
      package.y = 0;
    }

    target.x = constrain(package.x * sensiX, -MAX_ANGLE, MAX_ANGLE);
    target.y = constrain(package.y * sensiY, -MAX_ANGLE, MAX_ANGLE);

    if (armed) {
      target.z += package.z * sensiZ;
      target.z = constrain(target.z, -MAX_ANGLE, MAX_ANGLE);
    } else {
      target.z = 0;
    }

    thrust = package.thrust * sensiThrust;
    thrust = constrain(thrust, MIN, maxThrust);

    NoDataCount = 0;
    lastRxMicros = micros();
    ledPulseUntil = millis() + 120;
    announceLink(true);
    return true;
  }

  NoDataCount += timepi;
  if (NoDataCount > 3) {
    killSwitch = 2;
    announceLink(false);
  }
  return false;
}

void checkStatus() {
  if (!armed) {
    stopMotors();
  }

  if (gyro.error.z > 180 || gyro.error.z < -180) {
    resetYaw();
  }

  if (gyro.error.x > MAX_ANGLE || gyro.error.x < (-MAX_ANGLE)) {
    if (killAngle) {
      killSwitch = 1;
    }
  }

  if (gyro.error.y > MAX_ANGLE || gyro.error.y < (-MAX_ANGLE)) {
    if (killAngle) {
      killSwitch = 1;
    }
  }

  if (killSwitch > 0) {
    stopMotors();
    armed = false;
    smoothStartComplete = false;

    while (killSwitch > 0) {
      tone(PIN_BUZZER, 1000, 300);
      ledFlash(300);
      delay(2000);

      if (killSwitch == 2 && radio.available()) {
        delay(500);
        if (radio.available()) {
          tone(PIN_BUZZER, 1500, 1000);
          ledFlash(1000);
          killSwitch = 0;
          armed = false;
        }
      }
    }
  }
}

void calculatePID() {
  if (!armed) {
    PID[0] = {0, 0, 0};
    PID[1] = {0, 0, 0};
    PID[2] = {0, 0, 0};
    prevError = {0, 0, 0};
    resetYaw();
    return;
  }

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
}

void calculateVelocities() {
  thrust_2 = (1450 + pid_output_altitude + manual_throttle);
  const int baseThrust = (switch2 == 0 && thrust < 1500 && thrust > 1350) ? thrust_2 : thrust;

  RearLeft = baseThrust - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
  RearRight = baseThrust + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
  FrontLeft = baseThrust - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
  FrontRight = baseThrust + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
}

void runMotors() {
  MIN = armed ? MINarmed : pMIN;

  RearLeft = constrain(RearLeft, MIN, MAX);
  RearRight = constrain(RearRight, MIN, MAX);
  FrontLeft = constrain(FrontLeft, MIN, MAX);
  FrontRight = constrain(FrontRight, MIN, MAX);

  if (armed && smoothStartComplete) {
    ESCfl.writeMicroseconds(FrontLeft);
    ESCfr.writeMicroseconds(FrontRight);
    ESCrl.writeMicroseconds(RearLeft);
    ESCrr.writeMicroseconds(RearRight);
  } else if (armed && !smoothStartComplete) {
    ESCfl.writeMicroseconds(MINarmed);
    ESCfr.writeMicroseconds(MINarmed);
    ESCrl.writeMicroseconds(MINarmed);
    ESCrr.writeMicroseconds(MINarmed);
  } else {
    stopMotors();
  }
}

void stopMotors() {
  ESCfl.writeMicroseconds(pMIN);
  ESCfr.writeMicroseconds(pMIN);
  ESCrl.writeMicroseconds(pMIN);
  ESCrr.writeMicroseconds(pMIN);

  MIN = pMIN;
  FrontRight = pMIN;
  FrontLeft = pMIN;
  RearLeft = pMIN;
  RearRight = pMIN;
}

void resetYaw() {
  gyro.zeroYaw(true);
  target.z = 0;
}

void readEEPROM() {
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
}

void calibrateBarometer() {
  const int samples = 120;
  float accumulator = 0;
  for (int i = 0; i < samples; i++) {
    MS5611.read();
    accumulator += MS5611.getPressure();
    delay(5);
  }
  ground_pressure = accumulator / samples;
  for (int i = 0; i < 10; i++) {
    smooth.add(ground_pressure);
  }
}

void calculate_battery() {
  real_voltage = analogRead(A0);
  vout = (real_voltage * 5.0f) / 1023.0f;
  vin = vout / (R2 / (R1 + R2));
}

void waitLoop() {
  while (micros() - prevTime < timepi * sec_to_micro) {
  }
  prevTime = micros();
}

int ledFlash(int duration) {
  digitalWrite(PIN_LED, HIGH);
  delay(duration);
  digitalWrite(PIN_LED, LOW);
  return duration;
}

void debugging(bool dBug) {
  if (dBug) {
    dBugging = true;
    Serial.begin(57600);
    hz = 140;
  }
}

void Print() {
  Serial.print("actual_pressure= ");
  Serial.print(actual_pressure);
  Serial.print("\t");
  Serial.print("actual_pressure_2= ");
  Serial.print(actual_pressure_2);
  Serial.print("\t");
  Serial.print("link= ");
  Serial.print(linkActive);
  Serial.print("\t");
  Serial.print("armed= ");
  Serial.println(armed);
}
