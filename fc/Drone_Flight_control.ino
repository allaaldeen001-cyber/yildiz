#include <Servo.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <EEPROM.h>
#include "Gyro.h"
#include <Smoothed.h>
#include <Wire.h>
#include "MS5611.h"

// --------------------------------- Radio --------------------------------------
constexpr uint8_t RADIO_CE_PIN = 4;
constexpr uint8_t RADIO_CSN_PIN = 10;
constexpr uint64_t RADIO_PIPE = 0xF0F0F0F0E1LL;

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

struct Package {
  int   thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int   id = 0;
  bool  but1 = 1;     // D4 on transmitter -> Calibration
  bool  but2 = 1;     // D5 on transmitter -> Smooth-start
  bool  switch1 = 1;  // D3 on transmitter -> Arm (0) / Disarm (1)
  bool  switch2 = 1;  // D2 on transmitter -> Altitude hold (0 = ON)
};

struct AckPayload {
  uint16_t batteryMv = 0;
  float    baroAltitude = 0;
  bool     armed = false;
  bool     altHold = false;
  bool     link = false;
};

Package package;
AckPayload ackPayload;
bool but1 = true;
bool but2 = true;
bool switch1 = true;
bool switch2 = true;

// --------------------------------- Sensors ------------------------------------
MS5611 MS5611(0x77);
Gyro gyro;
Smoothed<float> smooth;

// --------------------------------- ESCs ---------------------------------------
Servo ESCfl;
Servo ESCfr;
Servo ESCrl;
Servo ESCrr;

constexpr uint8_t flPIN = 3;  // Front Left
constexpr uint8_t frPIN = 5;  // Front Right
constexpr uint8_t rrPIN = 6;  // Rear Right
constexpr uint8_t rlPIN = 9;  // Rear Left
constexpr uint8_t BUZZER_PIN = 8;
constexpr uint8_t LED_PIN = 7;

// --------------------------------- PID ----------------------------------------
const float kp = 2.0f;
const float ki = 0.0001f;
const float kd = 0.5f;
const float kpZ = 2.0f;

float pid_p_gain_altitude = 14.0f;
float pid_i_gain_altitude = 2.0f;
float pid_d_gain_altitude = 7.5f;
int   pid_max_altitude = 400;

float pid_error_gain_altitude, pid_throttle_gain_altitude;
float ground_pressure, altitude_hold_pressure;
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input;
float pid_output_altitude, pid_last_altitude_d_error;
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

// --------------------------------- Control ------------------------------------
float sensiX = -0.45f;
float sensiY = 0.45f;
float sensiZ = -0.01f;
float sensiThrust = 1.1f;

int lowPassX = 5;
int lowPassY = 5;
int lowPassZ = 10;

float hz = 140.0f;
int pMAX = 2000;
int pMIN = 1000;
int MINarmed = 1050;
int maxThrust = 1700;
int maxAngle = 30;
bool killAngle = true;

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
bool altitudeHoldEnabled = false;
bool linkEstablished = false;

double timepi = 0;
long prevTime = 0;

const float sec_to_micro = 1000000.0f;
const float micro_to_sec = 1.0f / 1000000.0f;
const float micro_to_ms = 0.001f;
const int   sec_to_ms = 1000;

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
struct quad_properties quadprops;

struct matrix2x2 {
  float m11;
  float m21;
  float m12;
  float m22;
};
struct matrix2x2 current_prob;

// Battery sensing
float vout = 0.0f;
float vin = 0.0f;
int real_voltage = 0;
float R1 = 1500.0f;
float R2 = 1000.0f;

// Indicator timing
unsigned long lastPacketMicros = 0;
unsigned long ledPulseDeadline = 0;
const unsigned long packetTimeoutMicros = 400000; // 0.4s failsafe
const unsigned long ledPulseDurationMs = 80;

// Function prototypes
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
int ledPulse(int duration);
void KalmanPosVel();
void initKalmanPosVel();
void handleCalibrationRequest();
void smoothMotorStart();
void signalLinkAcquired();
void updateIndicators();
void updateAckPayload();
void performBarometerCalibration();

void setup() {
  Serial.begin(57600);
  debugging(false);
  prevTime = micros();
  timepi = 1.0f / hz;

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  tone(BUZZER_PIN, 1000, 300);
  ledPulse(300);
  delay(100);
  tone(BUZZER_PIN, 1600, 700);
  ledPulse(700);
  delay(100);
  tone(BUZZER_PIN, 2000, 200);
  ledPulse(200);

  ESCfl.attach(flPIN, 1000, 2000);
  ESCfr.attach(frPIN, 1000, 2000);
  ESCrl.attach(rlPIN, 1000, 2000);
  ESCrr.attach(rrPIN, 1000, 2000);
  stopMotors();
  delay(500);
  Serial.println("Motors attached");

  if (!radio.begin()) {
    Serial.println("Radio init failed");
    while (true) {
      tone(BUZZER_PIN, 400, 300);
      delay(1000);
    }
  }

  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(3, 15);
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1, RADIO_PIPE);
  radio.startListening();
  Serial.println("Radio ready");

  readEEPROM();
  gyro.SetupWire(timepi);
  delay(500);

  tone(BUZZER_PIN, 2000, 200);
  ledPulse(200);

  MS5611.begin();
  MS5611.setOversampling(OSR_LOW);
  smooth.begin(SMOOTHED_AVERAGE, 10);
  initKalmanPosVel();
  performBarometerCalibration();
}

void loop() {
  bool packet = receiveRadio();
  calculate_battery();
  updateAckPayload();
  radio.writeAckPayload(1, &ackPayload, sizeof(ackPayload));
  checkStatus();
  gyro.setTarget(target);
  gyro.setCalibration(cal);
  calculate_pressure();
  gyro.calculateError();
  calculatePID();
  calculateVelocities();
  runMotors();
  updateIndicators();
  Print();
  waitLoop();
}

bool receiveRadio() {
  if (!radio.available()) {
    NoDataCount += timepi;
    if (micros() - lastPacketMicros > packetTimeoutMicros) {
      killSwitch = 2;
    }
    return false;
  }

  while (radio.available()) {
    radio.read(&package, sizeof(package));
  }

  if (package.thrust == 0) {
    NoDataCount += timepi;
    return false;
  }

  if (!linkEstablished) {
    signalLinkAcquired();
  }
  linkEstablished = true;
  lastPacketMicros = micros();
  NoDataCount = 0;

  if (abs(package.z) < lowPassZ) package.z = 0;
  if (abs(package.x) < lowPassX) package.x = 0;
  if (abs(package.y) < lowPassY) package.y = 0;

  target.x = constrain(package.x * sensiX, -maxAngle, maxAngle);
  target.y = constrain(package.y * sensiY, -maxAngle, maxAngle);

  if (armed) {
    target.z = constrain(target.z + package.z * sensiZ, -maxAngle, maxAngle);
  } else {
    target.z = 0;
  }

  thrust = package.thrust * sensiThrust;
  thrust = constrain(thrust, MIN, maxThrust);

  but1 = package.but1;
  but2 = package.but2;
  switch1 = package.switch1;
  switch2 = package.switch2;
  altitudeHoldEnabled = (switch2 == 0);

  ledPulseDeadline = millis() + ledPulseDurationMs;

  if (but1 == 0 && switch1 == 1) {
    handleCalibrationRequest();
  }
  if (but2 == 0 && switch1 == 0 && armed) {
    smoothMotorStart();
  }
  return true;
}

void signalLinkAcquired() {
  tone(BUZZER_PIN, 2400, 150);
  ledPulse(150);
}

void handleCalibrationRequest() {
  tone(BUZZER_PIN, 1200, 150);
  ledPulse(150);
  stopMotors();
  gyro.setTarget({0, 0, 0});
  gyro.setCalibration({0, 0, 0});
  cal = gyro.calibrate(1200);

  EEPROM.put(10, static_cast<float>(cal.x));
  EEPROM.put(15, static_cast<float>(cal.y));
  performBarometerCalibration();

  tone(BUZZER_PIN, 2000, 200);
  ledPulse(200);
}

void performBarometerCalibration() {
  const int samples = 60;
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    MS5611.read();
    sum += MS5611.getPressure();
    delay(10);
  }
  ground_pressure = sum / samples;
  quadprops.height = 0;
  quadprops.kalmanvel_z = 0;
}

void smoothMotorStart() {
  const int rampTimeMs = 1200;
  const int steps = 60;
  for (int i = 0; i < steps; i++) {
    int pwm = map(i, 0, steps - 1, MIN, MIN + 200);
    ESCfl.write(pwm);
    ESCfr.write(pwm);
    ESCrl.write(pwm);
    ESCrr.write(pwm);
    delay(rampTimeMs / steps);
  }
  thrust = MIN;
}

void updateAckPayload() {
  ackPayload.batteryMv = static_cast<uint16_t>(vin * 1000);
  ackPayload.baroAltitude = actual_pressure_2;
  ackPayload.armed = armed;
  ackPayload.altHold = altitudeHoldEnabled;
  ackPayload.link = linkEstablished;
}

void updateIndicators() {
  bool disarmed = (switch1 == 1);
  if (disarmed) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  if (millis() < ledPulseDeadline) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
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

  const bool altitudeThrottleWindow = altitudeHoldEnabled && thrust > 1400 && thrust < 1450;
  int baseThrust = altitudeThrottleWindow ? thrust_2 : thrust;

  RearLeft = baseThrust - PID[0].x - PID[1].x - PID[2].x - PID[0].y - PID[1].y - PID[2].y + PID[0].z + PID[2].z;
  RearRight = baseThrust + PID[0].x + PID[1].x + PID[2].x - PID[0].y - PID[1].y - PID[2].y - PID[0].z - PID[2].z;
  FrontLeft = baseThrust - PID[0].x - PID[1].x - PID[2].x + PID[0].y + PID[1].y + PID[2].y - PID[0].z - PID[2].z;
  FrontRight = baseThrust + PID[0].x + PID[1].x + PID[2].x + PID[0].y + PID[1].y + PID[2].y + PID[0].z + PID[2].z;
}

void runMotors() {
  if (armed) {
    MIN = MINarmed;
  } else {
    MIN = pMIN;
  }

  RearLeft = constrain(RearLeft, MIN, MAX);
  RearRight = constrain(RearRight, MIN, MAX);
  FrontLeft = constrain(FrontLeft, MIN, MAX);
  FrontRight = constrain(FrontRight, MIN, MAX);

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
  MIN = pMIN;
  FrontRight = pMIN;
  FrontLeft = pMIN;
  RearLeft = pMIN;
  RearRight = pMIN;
}

void checkStatus() {
  if (switch1 == 1) {
    armed = false;
    stopMotors();
  } else if (killSwitch == 0) {
    armed = true;
  }

  if (gyro.error.z > 180 || gyro.error.z < -180) {
    resetYaw();
  }

  if (NoDataCount > 3) {
    killSwitch = 2;
  }

  if (abs(gyro.error.x) > maxAngle || abs(gyro.error.y) > maxAngle) {
    if (killAngle) {
      killSwitch = 1;
    }
  }

  if (killSwitch > 0) {
    stopMotors();
    armed = false;
    while (killSwitch > 0) {
      tone(BUZZER_PIN, 1000, 300);
      ledPulse(300);
      delay(2000);

      if (killSwitch == 2 && radio.available()) {
        delay(500);
        if (radio.available()) {
          tone(BUZZER_PIN, 1500, 1000);
          ledPulse(1000);
          killSwitch = 0;
        }
      }
    }
  }
}

void calculate_battery() {
  real_voltage = analogRead(A0);
  vout = (real_voltage * 5.0f) / 1023.0f;
  vin = vout / (R2 / (R1 + R2));
}

void waitLoop() {
  while (micros() - prevTime < timepi * sec_to_micro);
  prevTime = micros();
}

int ledPulse(int duration) {
  digitalWrite(LED_PIN, HIGH);
  delay(duration);
  digitalWrite(LED_PIN, LOW);
  return duration;
}

void readEEPROM() {
  EEPROM.get(10, cal.x);
  EEPROM.get(15, cal.y);
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

void Print() {
  Serial.print("actual_pressure= ");
  Serial.print(actual_pressure);
  Serial.print("\t");
  Serial.print("actual_pressure_2= ");
  Serial.print(actual_pressure_2);
  Serial.print("\t");
  Serial.println();
}
