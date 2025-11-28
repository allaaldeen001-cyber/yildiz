/*
 * Arduino Nano Flight Controller Firmware
 * Hardware: Arduino Nano + NRF24L01 PA+LNA + MPU6050 + MS5611
 * Motor layout (Quad-X):
 *   FL (D3), FR (D5), RR (D6), RL (D9)
 * NRF: CE=D4, CSN=D10
 * IMU INT: D2, Buzzer=D8, Status LED=D7
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RF24.h>
#include <Servo.h>

#include "../common/CommProtocol.h"

// ----- Pin Map -----
constexpr uint8_t PIN_NRF_CE = 4;
constexpr uint8_t PIN_NRF_CSN = 10;
constexpr uint8_t PIN_LED = 7;
constexpr uint8_t PIN_BUZZER = 8;
constexpr uint8_t PIN_IMU_INT = 2;
constexpr uint8_t PIN_MOTOR_FL = 3;
constexpr uint8_t PIN_MOTOR_FR = 5;
constexpr uint8_t PIN_MOTOR_RR = 6;
constexpr uint8_t PIN_MOTOR_RL = 9;
constexpr uint8_t PIN_BATTERY_SENSE = A6; // optional VBAT divider input

constexpr float ADC_REFERENCE_V = 5.0f;
constexpr float BATTERY_DIVIDER_RATIO = 11.0f; // adjust per resistor divider

// ----- Loop Timing -----
constexpr uint32_t RATE_LOOP_PERIOD_US = 4000;   // 250 Hz inner rate loop
constexpr uint32_t ANGLE_LOOP_PERIOD_US = 10000; // 100 Hz outer angle loop
constexpr uint32_t ALT_LOOP_PERIOD_US = 40000;   // 25 Hz altitude loop

// ----- Motor Limits -----
constexpr int PWM_MIN = 1000;
constexpr int PWM_MAX = 2000;
constexpr int PWM_MAX_LIMITED = PWM_MIN + int((PWM_MAX - PWM_MIN) * 0.65f);
constexpr int PWM_ARM_IDLE = 1100;

constexpr float MAX_TILT_DEG = 30.0f;
constexpr float MAX_RATE_DPS = 250.0f;
constexpr float MAX_YAW_RATE_DPS = 180.0f;

constexpr uint32_t FAILSAFE_TIMEOUT_US = 250000; // 250 ms without packets
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float ONE_G = 9.80665f;

enum AxisIndex { ROLL = 0, PITCH = 1, YAW = 2 };

struct PidAxis {
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float integrator = 0;
  float prevError = 0;
  float outLimit = 0;
  float integratorLimit = 0;

  void configure(float p, float i, float d, float limit, float iLimit) {
    kp = p;
    ki = i;
    kd = d;
    outLimit = limit;
    integratorLimit = iLimit;
  }

  void reset() {
    integrator = 0;
    prevError = 0;
  }

  float update(float error, float dt) {
    if (dt <= 0.0f) {
      return constrain(kp * error, -outLimit, outLimit);
    }
    integrator += error * ki * dt;
    integrator = constrain(integrator, -integratorLimit, integratorLimit);
    float derivative = (error - prevError) / dt;
    prevError = error;
    float output = kp * error + integrator + kd * derivative;
    return constrain(output, -outLimit, outLimit);
  }
};

class BuzzerManager {
 public:
  void begin(uint8_t pin) {
    pin_ = pin;
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);
  }

  void playPattern(const uint16_t *pattern, uint8_t length) {
    pattern_ = pattern;
    length_ = length;
    index_ = 0;
    patternActive_ = length > 0;
    lastToggleMs_ = millis();
    stateHigh_ = true;
    if (patternActive_) {
      digitalWrite(pin_, HIGH);
    } else {
      digitalWrite(pin_, LOW);
    }
  }

  void success() { playPattern(kSuccessPattern, sizeof(kSuccessPattern) / sizeof(uint16_t)); }
  void fail() { playPattern(kFailPattern, sizeof(kFailPattern) / sizeof(uint16_t)); }
  void info() { playPattern(kInfoPattern, sizeof(kInfoPattern) / sizeof(uint16_t)); }

  void update() {
    if (!patternActive_) {
      return;
    }
    uint32_t now = millis();
    if (now - lastToggleMs_ >= pattern_[index_]) {
      lastToggleMs_ = now;
      stateHigh_ = !stateHigh_;
      digitalWrite(pin_, stateHigh_ ? HIGH : LOW);
      ++index_;
      if (index_ >= length_) {
        patternActive_ = false;
        digitalWrite(pin_, LOW);
      }
    }
  }

 private:
  const uint16_t kSuccessPattern[4] = {150, 120, 150, 0};
  const uint16_t kFailPattern[2] = {7000, 0};
  const uint16_t kInfoPattern[6] = {120, 80, 120, 80, 120, 0};

  uint8_t pin_ = 0;
  const uint16_t *pattern_ = nullptr;
  uint8_t length_ = 0;
  uint8_t index_ = 0;
  uint32_t lastToggleMs_ = 0;
  bool patternActive_ = false;
  bool stateHigh_ = false;
};

class Ms5611Driver {
 public:
  bool begin() {
    Wire.beginTransmission(address_);
    Wire.write(0x1E);
    if (Wire.endTransmission() != 0) {
      return false;
    }
    delay(5);
    for (uint8_t i = 0; i < 6; ++i) {
      uint16_t value = readProm(0xA2 + i * 2);
      if (!value) {
        return false;
      }
      C_[i + 1] = value;
    }
    state_ = State::StartPressure;
    sampleReady_ = false;
    return true;
  }

  void service() {
    uint32_t now = micros();
    switch (state_) {
      case State::StartPressure:
        startConversion(0x48); // D1 OSR4096
        readyMicros_ = now + 10000;
        state_ = State::WaitPressure;
        break;
      case State::WaitPressure:
        if (timeReady(now)) {
          rawPressure_ = readAdc();
          state_ = State::StartTemperature;
        }
        break;
      case State::StartTemperature:
        startConversion(0x58); // D2 OSR4096
        readyMicros_ = now + 10000;
        state_ = State::WaitTemperature;
        break;
      case State::WaitTemperature:
        if (timeReady(now)) {
          rawTemperature_ = readAdc();
          compute();
          state_ = State::StartPressure;
        }
        break;
    }
  }

  bool hasSample() const { return sampleReady_; }

  float altitudeMeters() const { return altitude_; }
  float temperatureC() const { return temperature_; }

  void clearSampleFlag() { sampleReady_ = false; }

 private:
  enum class State { StartPressure, WaitPressure, StartTemperature, WaitTemperature };

  bool timeReady(uint32_t now) const {
    return static_cast<int32_t>(now - readyMicros_) >= 0;
  }

  void startConversion(uint8_t command) {
    Wire.beginTransmission(address_);
    Wire.write(command);
    Wire.endTransmission();
  }

  uint32_t readAdc() {
    Wire.beginTransmission(address_);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(address_, static_cast<uint8_t>(3));
    uint32_t value = 0;
    for (uint8_t i = 0; i < 3 && Wire.available(); ++i) {
      value = (value << 8) | Wire.read();
    }
    return value;
  }

  uint16_t readProm(uint8_t reg) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
      return 0;
    }
    Wire.requestFrom(address_, static_cast<uint8_t>(2));
    if (Wire.available() < 2) {
      return 0;
    }
    uint16_t value = (Wire.read() << 8) | Wire.read();
    return value;
  }

  void compute() {
    int32_t dT = static_cast<int32_t>(rawTemperature_) - (static_cast<int32_t>(C_[5]) << 8);
    int64_t off = (static_cast<int64_t>(C_[2]) << 16) + ((int64_t)dT * C_[4]) / 128;
    int64_t sens = (static_cast<int64_t>(C_[1]) << 15) + ((int64_t)dT * C_[3]) / 256;
    int32_t temp = 2000 + ((int64_t)dT * C_[6]) / 8388608;
    int32_t pressure = (((int64_t)rawPressure_ * sens) / 2097152 - off) / 32768;

    temperature_ = temp / 100.0f;
    float pressurePa = pressure;
    altitude_ = 44330.0f * (1.0f - pow(pressurePa / 101325.0f, 0.1903f));
    sampleReady_ = true;
  }

  const uint8_t address_ = 0x77;
  uint16_t C_[7] = {};
  uint32_t readyMicros_ = 0;
  uint32_t rawPressure_ = 0;
  uint32_t rawTemperature_ = 0;
  float altitude_ = 0.0f;
  float temperature_ = 0.0f;
  bool sampleReady_ = false;
  State state_ = State::StartPressure;
};

struct AltitudeEstimator {
  float altitude_m = 0.0f;
  float velocity_mps = 0.0f;
  bool initialized = false;

  void update(float baroAltitude, float verticalAccel, float dt) {
    if (!initialized) {
      altitude_m = baroAltitude;
      velocity_mps = 0.0f;
      initialized = true;
      return;
    }
    velocity_mps += verticalAccel * dt;
    altitude_m += velocity_mps * dt;
    const float alpha = 0.02f;
    altitude_m = alpha * baroAltitude + (1.0f - alpha) * altitude_m;
  }
};

struct MotorTestState {
  bool active = false;
  uint8_t stage = 0;
  uint32_t stageStartMs = 0;
};

// ----- Globals -----
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
Servo motorFL, motorFR, motorRR, motorRL;
BuzzerManager buzzer;
Ms5611Driver ms5611;
AltitudeEstimator altitudeEstimator;

RcToFcPacket rcCommand = {};
FcToRcPacket telemetry = {};

uint8_t lastButtonState = 0;
uint32_t lastRxMicros = 0;
bool linkActive = false;
bool failsafeActive = true;
bool armed = false;
bool altHoldEnabled = false;
bool imuCalibrated = false;
bool imuCalibrating = false;

uint16_t telemetrySeq = 0;
uint16_t motorBaseCmd = PWM_MIN;

PidAxis ratePid[3];
PidAxis anglePid[2];
PidAxis altHeightPid;
PidAxis altVelocityPid;

float rateSetpoint[3] = {0};
float angleCommand[2] = {0};
float yawRateCommand = 0;

float gyroOffset[3] = {0};
float accelOffset[3] = {0};

// Calibration accumulators
int64_t gyroAccum[3] = {0};
int64_t accelAccum[3] = {0};
uint16_t calibSamples = 0;
const uint16_t CALIB_REQUIRED_SAMPLES = 2000;

// Quaternion state for Mahony filter
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float mahonyIntX = 0.0f, mahonyIntY = 0.0f, mahonyIntZ = 0.0f;
constexpr float MAHONY_KP = 3.0f;
constexpr float MAHONY_KI = 0.03f;

// Attitude outputs
float eulerRoll = 0.0f;
float eulerPitch = 0.0f;
float eulerYaw = 0.0f;

float verticalAccelLpf = 0.0f;
float hoverThrottle = PWM_ARM_IDLE;
float altitudeTarget = 0.0f;
int throttleCommandUs = PWM_MIN;
uint32_t lastAltEstimateUs = 0;

MotorTestState motorTest;

uint32_t lastRateLoop = 0;
uint32_t lastAngleLoop = 0;
uint32_t lastAltLoop = 0;
uint32_t lastLedBlink = 0;
bool ledState = false;

// ----- Helper Functions -----
float invSqrt(float x) {
  return 1.0f / sqrtf(x);
}

void writeMotor(int fl, int fr, int rr, int rl) {
  motorFL.writeMicroseconds(constrain(fl, PWM_MIN, PWM_MAX));
  motorFR.writeMicroseconds(constrain(fr, PWM_MIN, PWM_MAX));
  motorRR.writeMicroseconds(constrain(rr, PWM_MIN, PWM_MAX));
  motorRL.writeMicroseconds(constrain(rl, PWM_MIN, PWM_MAX));
}

void writeAllMotors(int value) { writeMotor(value, value, value, value); }

int applyThrottleCurve(uint16_t inputUs) {
  inputUs = constrain(inputUs, PWM_MIN, PWM_MAX);
  float normalized = float(inputUs - PWM_MIN) / float(PWM_MAX - PWM_MIN);
  float curved = normalized * normalized; // softer near bottom
  int limited = PWM_MIN + int(curved * (PWM_MAX_LIMITED - PWM_MIN));
  return limited;
}

float readBatteryMillivolts() {
  int raw = analogRead(PIN_BATTERY_SENSE);
  float voltage = (raw * (ADC_REFERENCE_V / 1023.0f)) * BATTERY_DIVIDER_RATIO;
  return voltage * 1000.0f;
}

void resetPids() {
  for (auto &pid : ratePid) {
    pid.reset();
  }
  for (auto &pid : anglePid) {
    pid.reset();
  }
  altHeightPid.reset();
  altVelocityPid.reset();
}

void disarm() {
  if (!armed) {
    return;
  }
  armed = false;
  throttleCommandUs = PWM_MIN;
  writeAllMotors(PWM_MIN);
  resetPids();
}

void attemptArm() {
  if (armed || !imuCalibrated || failsafeActive) {
    return;
  }
  if (rcCommand.throttle > PWM_MIN + 50) {
    return; // require low throttle
  }
  armed = true;
  hoverThrottle = PWM_ARM_IDLE;
  altitudeTarget = altitudeEstimator.altitude_m;
}

void startCalibration() {
  disarm();
  imuCalibrating = true;
  imuCalibrated = false;
  calibSamples = 0;
  gyroAccum[0] = gyroAccum[1] = gyroAccum[2] = 0;
  accelAccum[0] = accelAccum[1] = accelAccum[2] = 0;
}

void finishCalibration(bool success) {
  imuCalibrating = false;
  imuCalibrated = success;
  if (success) {
    buzzer.success();
  } else {
    buzzer.fail();
  }
}

void handleCalibrationSample(const int16_t ax, const int16_t ay, const int16_t az,
                             const int16_t gx, const int16_t gy, const int16_t gz) {
  if (!imuCalibrating) {
    return;
  }
  gyroAccum[0] += gx;
  gyroAccum[1] += gy;
  gyroAccum[2] += gz;
  accelAccum[0] += ax;
  accelAccum[1] += ay;
  accelAccum[2] += az;
  ++calibSamples;
  if (calibSamples >= CALIB_REQUIRED_SAMPLES) {
    for (int i = 0; i < 3; ++i) {
      gyroOffset[i] = float(gyroAccum[i]) / float(calibSamples);
    }
    accelOffset[0] = float(accelAccum[0]) / float(calibSamples);
    accelOffset[1] = float(accelAccum[1]) / float(calibSamples);
    accelOffset[2] = float(accelAccum[2]) / float(calibSamples) - 16384.0f;
    bool still = fabs(gyroOffset[0]) < 30 && fabs(gyroOffset[1]) < 30 && fabs(gyroOffset[2]) < 30;
    finishCalibration(still);
  }
}

void mahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) {
    return;
  }
  float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;

  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
  float ex = (ay * vz - az * vy);
  float ey = (az * vx - ax * vz);
  float ez = (ax * vy - ay * vx);

  if (MAHONY_KI > 0.0f) {
    mahonyIntX += MAHONY_KI * ex * dt;
    mahonyIntY += MAHONY_KI * ey * dt;
    mahonyIntZ += MAHONY_KI * ez * dt;
    gx += mahonyIntX;
    gy += mahonyIntY;
    gz += mahonyIntZ;
  }

  gx += MAHONY_KP * ex;
  gy += MAHONY_KP * ey;
  gz += MAHONY_KP * ez;

  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;

  float qa = q0;
  float qb = q1;
  float qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void quaternionToEuler() {
  eulerRoll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
  float s = 2.0f * (q0 * q2 - q3 * q1);
  s = constrain(s, -1.0f, 1.0f);
  eulerPitch = asinf(s) * RAD_TO_DEG;
  eulerYaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}

float estimateVerticalAcceleration(float ax, float ay, float az) {
  float qw = q0, qx = q1, qy = q2, qz = q3;
  float r31 = 2.0f * (qx * qz - qw * qy);
  float r32 = 2.0f * (qy * qz + qw * qx);
  float r33 = qw * qw - qx * qx - qy * qy + qz * qz;
  float accelWorldZ = r31 * ax + r32 * ay + r33 * az;
  return accelWorldZ - ONE_G;
}

void updateTelemetry(float batteryMv) {
  telemetry.seq = telemetrySeq++;
  telemetry.rollAngleDeg = int16_t(eulerRoll * 100.0f);
  telemetry.pitchAngleDeg = int16_t(eulerPitch * 100.0f);
  telemetry.yawRateDps = int16_t(yawRateCommand * 10.0f);
  telemetry.altitudeCm = int16_t(altitudeEstimator.altitude_m * 100.0f);
  telemetry.batteryMv = int16_t(batteryMv);
  telemetry.statusFlags = 0;
  if (linkActive) telemetry.statusFlags |= FC_STATUS_LINKED;
  if (armed) telemetry.statusFlags |= FC_STATUS_ARMED;
  if (imuCalibrated) telemetry.statusFlags |= FC_STATUS_CALIBRATED;
  if (altHoldEnabled) telemetry.statusFlags |= FC_STATUS_ALT_HOLD;
  if (failsafeActive) telemetry.statusFlags |= FC_STATUS_FAILSAFE;
  telemetry.linkQuality = linkActive ? 100 : 0;
  finalizePacket(telemetry);
}

void serviceLed() {
  uint32_t interval = linkActive ? 200 : 800;
  uint32_t now = millis();
  if (now - lastLedBlink >= interval) {
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState ? HIGH : LOW);
    lastLedBlink = now;
  }
}

void serviceMotorTest() {
  if (!motorTest.active) {
    return;
  }
  uint32_t now = millis();
  switch (motorTest.stage) {
    case 0: // ESC max pulse
      writeAllMotors(PWM_MAX_LIMITED);
      if (now - motorTest.stageStartMs > 2000) {
        motorTest.stage = 1;
        motorTest.stageStartMs = now;
      }
      break;
    case 1: // ESC min pulse
      writeAllMotors(PWM_MIN);
      if (now - motorTest.stageStartMs > 2000) {
        motorTest.stage = 2;
        motorTest.stageStartMs = now;
      }
      break;
    case 2: // FL ramp
    case 3: // FR ramp
    case 4: // RR ramp
    case 5: { // RL ramp
      uint16_t motorValues[4] = {PWM_MIN, PWM_MIN, PWM_MIN, PWM_MIN};
      uint8_t idx = motorTest.stage - 2;
      uint32_t elapsed = now - motorTest.stageStartMs;
      uint16_t target = PWM_MIN + 300;
      uint16_t value =
          (elapsed >= 1500) ? target : PWM_MIN + (elapsed * (target - PWM_MIN)) / 1500;
      motorValues[idx] = value;
      writeMotor(motorValues[0], motorValues[1], motorValues[2], motorValues[3]);
      if (elapsed > 2000) {
        ++motorTest.stage;
        motorTest.stageStartMs = now;
      }
      break;
    }
    default:
      writeAllMotors(PWM_MIN);
      motorTest.active = false;
      break;
  }
}

void startMotorTest() {
  disarm();
  motorTest.active = true;
  motorTest.stage = 0;
  motorTest.stageStartMs = millis();
  buzzer.info();
}

// ----- Hardware Init -----
bool initMpu() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // gyro config
  Wire.write(0x10); // +-1000 dps
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // accel config
  Wire.write(0x10); // +-8g
  return Wire.endTransmission() == 0;
}

void initRadio() {
  radio.begin();
  radio.setChannel(kNrfChannel);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);
  radio.enableAckPayload();
  radio.openReadingPipe(1, kNrfAddress);
  radio.startListening();
}

void initMotors() {
  motorFL.attach(PIN_MOTOR_FL, PWM_MIN, PWM_MAX);
  motorFR.attach(PIN_MOTOR_FR, PWM_MIN, PWM_MAX);
  motorRR.attach(PIN_MOTOR_RR, PWM_MIN, PWM_MAX);
  motorRL.attach(PIN_MOTOR_RL, PWM_MIN, PWM_MAX);
  writeAllMotors(PWM_MIN);
}

// ----- IMU Read -----
bool readMpuRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  Wire.requestFrom(0x68, 14);
  if (Wire.available() < 14) {
    return false;
  }
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read();
  Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

void processRadio() {
  while (radio.available()) {
    RcToFcPacket incoming;
    radio.read(&incoming, sizeof(incoming));
    if (!validatePacket(incoming)) {
      continue;
    }
    rcCommand = incoming;
    lastRxMicros = micros();
    linkActive = true;
    failsafeActive = false;

    if ((rcCommand.buttons & BUTTON_CALIBRATE) && !(lastButtonState & BUTTON_CALIBRATE)) {
      startCalibration();
    }
    if ((rcCommand.buttons & BUTTON_MOTOR_TEST) && !(lastButtonState & BUTTON_MOTOR_TEST)) {
      startMotorTest();
    }
    lastButtonState = rcCommand.buttons;

    updateTelemetry(readBatteryMillivolts());
    radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
  }

  uint32_t now = micros();
  if (static_cast<int32_t>(now - lastRxMicros) > FAILSAFE_TIMEOUT_US) {
    linkActive = false;
    failsafeActive = true;
    disarm();
  }
}

void updateArmingState() {
  bool armSwitch = (rcCommand.switches & SWITCH_ARM);
  if (!armSwitch || failsafeActive) {
    disarm();
    return;
  }
  if (!armed) {
    attemptArm();
  }
}

void updateAltHoldState() {
  bool requested = (rcCommand.switches & SWITCH_ALT_HOLD) && armed;
  if (requested && !altHoldEnabled) {
    altHoldEnabled = altitudeEstimator.initialized;
    altitudeTarget = altitudeEstimator.altitude_m;
    hoverThrottle = constrain(rcCommand.throttle, PWM_MIN, PWM_MAX_LIMITED);
    altHeightPid.reset();
    altVelocityPid.reset();
  } else if (!requested && altHoldEnabled) {
    altHoldEnabled = false;
  }
}

int computeAltHoldThrottle(float dtAlt) {
  if (!altHoldEnabled || !altitudeEstimator.initialized) {
    return applyThrottleCurve(rcCommand.throttle);
  }
  float manualDelta = (rcCommand.throttle - 1500) / 500.0f; // -1..+1
  const float manualDeadband = 0.05f;
  if (fabs(manualDelta) > manualDeadband) {
    altitudeTarget += manualDelta * dtAlt * 1.0f; // 1 m/s adjust of target
  }

  float altError = altitudeTarget - altitudeEstimator.altitude_m;
  float climbRateCmd = altHeightPid.update(altError, dtAlt);
  climbRateCmd = constrain(climbRateCmd, -1.5f, 1.5f);
  float velError = (climbRateCmd - altitudeEstimator.velocity_mps);
  float throttleAdjust = altVelocityPid.update(velError, dtAlt);
  float commanded = hoverThrottle + throttleAdjust;
  commanded = constrain(commanded, PWM_MIN, PWM_MAX_LIMITED);
  hoverThrottle = 0.99f * hoverThrottle + 0.01f * commanded;
  return int(commanded);
}

void mixAndWriteMotors(float throttle, float rollOut, float pitchOut, float yawOut) {
  int fl = int(throttle + pitchOut + rollOut - yawOut);
  int fr = int(throttle + pitchOut - rollOut + yawOut);
  int rr = int(throttle - pitchOut - rollOut - yawOut);
  int rl = int(throttle - pitchOut + rollOut + yawOut);
  fl = constrain(fl, PWM_MIN, PWM_MAX_LIMITED);
  fr = constrain(fr, PWM_MIN, PWM_MAX_LIMITED);
  rr = constrain(rr, PWM_MIN, PWM_MAX_LIMITED);
  rl = constrain(rl, PWM_MIN, PWM_MAX_LIMITED);
  writeMotor(fl, fr, rr, rl);
}

void setupPids() {
  ratePid[ROLL].configure(0.12f, 0.04f, 0.0008f, 300.0f, 200.0f);
  ratePid[PITCH].configure(0.12f, 0.04f, 0.0008f, 300.0f, 200.0f);
  ratePid[YAW].configure(0.18f, 0.02f, 0.000f, 200.0f, 100.0f);

  anglePid[ROLL].configure(4.5f, 0.0f, 0.12f, MAX_RATE_DPS, MAX_RATE_DPS);
  anglePid[PITCH].configure(4.5f, 0.0f, 0.12f, MAX_RATE_DPS, MAX_RATE_DPS);

  altHeightPid.configure(2.0f, 0.5f, 0.0f, 2.0f, 1.0f);
  altVelocityPid.configure(80.0f, 30.0f, 0.0f, 250.0f, 150.0f);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_IMU_INT, INPUT);
  buzzer.begin(PIN_BUZZER);
  Wire.begin();
  Wire.setClock(400000);
  initMpu();
  ms5611.begin();
  initMotors();
  initRadio();
  setupPids();
  rcCommand.throttle = PWM_MIN;
  rcCommand.roll = 0;
  rcCommand.pitch = 0;
  rcCommand.yaw = 0;
  uint32_t start = micros();
  lastRxMicros = start;
  lastRateLoop = start;
  lastAngleLoop = start;
  lastAltLoop = start;
  failsafeActive = true;
  altHoldEnabled = false;
}

void loop() {
  uint32_t now = micros();
  ms5611.service();
  processRadio();
  serviceLed();
  buzzer.update();

  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  if (!readMpuRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)) {
    return;
  }
  handleCalibrationSample(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);

  float ax = (axRaw - accelOffset[0]) / 4096.0f * ONE_G; // +-8g scale
  float ay = (ayRaw - accelOffset[1]) / 4096.0f * ONE_G;
  float az = (azRaw - accelOffset[2]) / 4096.0f * ONE_G;
  float gx = (gxRaw - gyroOffset[0]) / 32.8f * DEG_TO_RAD; // +-1000 dps -> 32.8 LSB/deg/s
  float gy = (gyRaw - gyroOffset[1]) / 32.8f * DEG_TO_RAD;
  float gz = (gzRaw - gyroOffset[2]) / 32.8f * DEG_TO_RAD;

  updateArmingState();
  updateAltHoldState();

  if (imuCalibrating) {
    writeAllMotors(PWM_MIN);
    serviceMotorTest();
    return;
  }

  if (now - lastRateLoop >= RATE_LOOP_PERIOD_US) {
    float dtRate = (now - lastRateLoop) / 1e6f;
    lastRateLoop = now;
    mahonyUpdate(gx, gy, gz, ax, ay, az, dtRate);
    quaternionToEuler();

    if (ms5611.hasSample()) {
      float dtAltEst = (lastAltEstimateUs == 0) ? dtRate : (now - lastAltEstimateUs) / 1e6f;
      if (dtAltEst < 0.001f) {
        dtAltEst = 0.001f;
      }
      lastAltEstimateUs = now;
      float verticalAccel = estimateVerticalAcceleration(ax, ay, az);
      verticalAccelLpf = 0.7f * verticalAccelLpf + 0.3f * verticalAccel;
      altitudeEstimator.update(ms5611.altitudeMeters(), verticalAccelLpf, dtAltEst);
      ms5611.clearSampleFlag();
    }

    if (now - lastAngleLoop >= ANGLE_LOOP_PERIOD_US) {
      float dtAngle = (now - lastAngleLoop) / 1e6f;
      lastAngleLoop = now;
      float rollCmd = constrain(rcCommand.roll * 0.1f, -MAX_TILT_DEG, MAX_TILT_DEG);
      float pitchCmd = constrain(rcCommand.pitch * 0.1f, -MAX_TILT_DEG, MAX_TILT_DEG);
      angleCommand[ROLL] = rollCmd;
      angleCommand[PITCH] = pitchCmd;
      float rollError = rollCmd - eulerRoll;
      float pitchError = pitchCmd - eulerPitch;
      rateSetpoint[ROLL] = anglePid[ROLL].update(rollError, dtAngle);
      rateSetpoint[PITCH] = anglePid[PITCH].update(pitchError, dtAngle);
      rateSetpoint[ROLL] = constrain(rateSetpoint[ROLL], -MAX_RATE_DPS, MAX_RATE_DPS);
      rateSetpoint[PITCH] = constrain(rateSetpoint[PITCH], -MAX_RATE_DPS, MAX_RATE_DPS);
      yawRateCommand = constrain(rcCommand.yaw * 0.1f, -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS);
      rateSetpoint[YAW] = yawRateCommand;
    }

    if (now - lastAltLoop >= ALT_LOOP_PERIOD_US) {
      float dtAlt = (now - lastAltLoop) / 1e6f;
      lastAltLoop = now;
      if (altHoldEnabled && altitudeEstimator.initialized) {
        throttleCommandUs = computeAltHoldThrottle(dtAlt);
      } else {
        throttleCommandUs = applyThrottleCurve(rcCommand.throttle);
        hoverThrottle = 0.98f * hoverThrottle + 0.02f * throttleCommandUs;
      }
    } else if (!altHoldEnabled) {
      throttleCommandUs = applyThrottleCurve(rcCommand.throttle);
      hoverThrottle = 0.98f * hoverThrottle + 0.02f * throttleCommandUs;
    }

    if (!armed || failsafeActive || motorTest.active) {
      writeAllMotors(PWM_MIN);
    } else {
      float gyroDps[3] = {gx * RAD_TO_DEG, gy * RAD_TO_DEG, gz * RAD_TO_DEG};
      float rateOut[3];
      for (int axis = 0; axis < 3; ++axis) {
        float error = rateSetpoint[axis] - gyroDps[axis];
        rateOut[axis] = ratePid[axis].update(error, dtRate);
      }
      mixAndWriteMotors(throttleCommandUs, rateOut[ROLL], rateOut[PITCH], rateOut[YAW]);
    }
  }

  serviceMotorTest();
}
