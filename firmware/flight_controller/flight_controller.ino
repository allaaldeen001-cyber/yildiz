/**
 * Arduino Nano Quadcopter Flight Controller
 *
 * Features:
 *  - NRF24L01 link with ack telemetry and staged user guidance
 *  - Complementary filter IMU fusion on MPU6050
 *  - PID attitude stabilization for roll/pitch and yaw rate control
 *  - Button-triggered IMU + ESC calibration with EEPROM storage
 *  - Smooth arming workflow with idle motor spool-up and kill switch enforcement
 *  - Connection buzzer + status LEDs for operator feedback
 *  - Relative altitude estimate for Nokia 5110 display telemetry
 */

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <EEPROM.h>
#include <math.h>

#include "../common/config.h"

// ---------------- Pin Mapping ----------------
constexpr uint8_t NRF_CE_PIN = 7;
constexpr uint8_t NRF_CSN_PIN = 8;
constexpr uint8_t BUZZER_PIN = 10;
constexpr uint8_t STATUS_LED_PIN = 4;
constexpr uint8_t ARMED_LED_PIN = 2;
constexpr uint8_t VBAT_PIN = A0;
constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr uint8_t ESC_PINS[4] = {3, 5, 6, 9};  // Front-left, front-right, rear-left, rear-right

// --------------- Control Constants ---------------
constexpr float LOOP_FREQUENCY_HZ = 250.0f;
constexpr float LOOP_PERIOD_US = 1e6f / LOOP_FREQUENCY_HZ;
constexpr uint32_t FAILSAFE_TIMEOUT_US = 300000;   // 0.3 seconds
constexpr uint16_t BASE_MIN_US = 1000;
constexpr uint16_t BASE_MAX_US = 2000;
constexpr uint16_t IDLE_SPIN_US = 1120;
constexpr float MAX_TILT_DEG = 35.0f;
constexpr float MAX_YAW_RATE_DPS = 140.0f;
constexpr float STICK_DEADBAND = 0.03f;
constexpr float STICK_EXPO = 0.25f;
constexpr float ACC_LSB_PER_G = 16384.0f;
constexpr float GYRO_LSB_PER_DPS = 131.0f;
constexpr float GRAVITY = 9.80665f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr uint16_t STATUS_LED_PULSE_MS = 120;
constexpr uint16_t CALIBRATION_MAGIC = 0xDC42;

// PID tuning (can be tweaked in flight testing).
struct PID {
  float kp;
  float ki;
  float kd;
  float integrator;
  float prevError;
  float outputLimit;
  float integratorLimit;

  PID(float p, float i, float d, float outLimit, float iLimit)
      : kp(p), ki(i), kd(d), integrator(0.0f), prevError(0.0f),
        outputLimit(outLimit), integratorLimit(iLimit) {}

  float update(float error, float dt) {
    integrator += error * dt;
    if (integrator > integratorLimit) integrator = integratorLimit;
    if (integrator < -integratorLimit) integrator = -integratorLimit;
    float derivative = (error - prevError) / dt;
    float output = (kp * error) + (ki * integrator) + (kd * derivative);
    if (output > outputLimit) output = outputLimit;
    if (output < -outputLimit) output = -outputLimit;
    prevError = error;
    return output;
  }

  void reset() {
    integrator = 0.0f;
    prevError = 0.0f;
  }
};

PID rollPid(4.2f, 0.03f, 0.15f, 350.0f, 120.0f);
PID pitchPid(4.2f, 0.03f, 0.15f, 350.0f, 120.0f);
PID yawPid(2.0f, 0.01f, 0.0f, 200.0f, 80.0f);

// ----------------- Radio & Telemetry -----------------
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
ControlPacket latestControls = {
    PACKET_VERSION,
    1000, 1500, 1500, 1500,
    0, 0, 0, 0,
    RADIO_CHANNEL,
    0};
TelemetryPacket telemetry = {PACKET_VERSION};

// ----------------- ESCs -----------------
Servo esc[4];
float throttleCommandUs = BASE_MIN_US;
float idleThrottleUs = BASE_MIN_US;
float idleTargetUs = BASE_MIN_US;
bool idleSpinComplete = false;
bool spoolInProgress = false;

// ----------------- IMU / Sensor Fusion -----------------
float rollAngleDeg = 0.0f;
float pitchAngleDeg = 0.0f;
float yawAngleDeg = 0.0f;
float gyroRates[3] = {0.0f, 0.0f, 0.0f};
float altitudeMeters = 0.0f;
float verticalVelocity = 0.0f;
float gyroBias[3] = {0.0f, 0.0f, 0.0f};
float accelBias[3] = {0.0f, 0.0f, 0.0f};

struct CalibrationStore {
  uint16_t magic;
  float gyroBias[3];
  float accelBias[3];
  uint16_t escMinUs;
  uint16_t escMaxUs;
  uint8_t reserved[8];
  uint8_t checksum;
};

CalibrationStore calStore{};
bool imuCalibrated = false;
bool escCalibrated = false;
bool calibrating = false;

// ----------------- State Tracking -----------------
GuideStep guideStep = GuideStep::WAIT_LINK;
bool armed = false;
bool linkActive = false;
bool connectionAnnounced = false;
bool pendingCalibrationRequest = false;
uint32_t lastLoopMicros = 0;
uint32_t lastPacketMicros = 0;
uint32_t lastLinkWindowMillis = 0;
uint16_t windowPacketTotal = 0;
uint16_t windowPacketGood = 0;
uint8_t linkQualityPercent = 0;
bool button1Prev = false;
bool button2Prev = false;
uint32_t statusLedPulseUntil = 0;
float batteryVoltage = 0.0f;

struct BuzzerPattern {
  bool active;
  bool state;
  uint8_t pulsesRemaining;
  uint32_t nextToggleMs;
  uint16_t onDurationMs;
  uint16_t offDurationMs;
} buzzer = {false, false, 0, 0, 120, 120};

// ----------------- Helper Utilities -----------------
float applyDeadband(float value, float deadband) {
  if (fabs(value) < deadband) {
    return 0.0f;
  }
  if (value > 0.0f) {
    return (value - deadband) / (1.0f - deadband);
  }
  return (value + deadband) / (1.0f - deadband);
}

float applyExpo(float value, float expo) {
  return value * (1 - expo) + value * value * value * expo;
}

float clampFloat(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

uint16_t clampPulse(float value) {
  if (value < BASE_MIN_US) return BASE_MIN_US;
  if (value > BASE_MAX_US) return BASE_MAX_US;
  return static_cast<uint16_t>(value);
}

void pulseStatusLed(uint16_t durationMs) {
  statusLedPulseUntil = millis() + durationMs;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void startBuzzer(uint8_t pulses, uint16_t onMs, uint16_t offMs) {
  buzzer.active = true;
  buzzer.state = true;
  buzzer.pulsesRemaining = pulses;
  buzzer.onDurationMs = onMs;
  buzzer.offDurationMs = offMs;
  buzzer.nextToggleMs = millis() + onMs;
  digitalWrite(BUZZER_PIN, HIGH);
}

void stopBuzzer() {
  buzzer.active = false;
  buzzer.state = false;
  buzzer.pulsesRemaining = 0;
  digitalWrite(BUZZER_PIN, LOW);
}

void serviceBuzzer(uint32_t nowMs) {
  if (!buzzer.active) return;
  if (nowMs < buzzer.nextToggleMs) return;
  buzzer.state = !buzzer.state;
  digitalWrite(BUZZER_PIN, buzzer.state ? HIGH : LOW);
  if (buzzer.state) {
    buzzer.nextToggleMs = nowMs + buzzer.onDurationMs;
  } else {
    if (buzzer.pulsesRemaining > 0) {
      --buzzer.pulsesRemaining;
    }
    if (buzzer.pulsesRemaining == 0) {
      stopBuzzer();
      return;
    }
    buzzer.nextToggleMs = nowMs + buzzer.offDurationMs;
  }
}

void serviceStatusLed(uint32_t nowMs) {
  if (statusLedPulseUntil > nowMs) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    return;
  }
  if (!linkActive) {
    digitalWrite(STATUS_LED_PIN, LOW);
    return;
  }
  // Blink at 2 Hz when link is healthy.
  bool blinkState = ((nowMs / 250) % 2) == 0;
  digitalWrite(STATUS_LED_PIN, blinkState ? HIGH : LOW);
}

// ----------------- EEPROM Helpers -----------------
uint8_t checksumCalibration(const CalibrationStore &store) {
  const uint8_t *raw = reinterpret_cast<const uint8_t *>(&store);
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(CalibrationStore) - 1; ++i) {
    sum ^= raw[i];
  }
  return sum;
}

void saveCalibration() {
  calStore.magic = CALIBRATION_MAGIC;
  memcpy(calStore.gyroBias, gyroBias, sizeof(gyroBias));
  memcpy(calStore.accelBias, accelBias, sizeof(accelBias));
  calStore.escMinUs = BASE_MIN_US;
  calStore.escMaxUs = BASE_MAX_US;
  calStore.checksum = checksumCalibration(calStore);
  EEPROM.put(0, calStore);
}

void loadCalibration() {
  EEPROM.get(0, calStore);
  if (calStore.magic != CALIBRATION_MAGIC) {
    imuCalibrated = false;
    escCalibrated = false;
    return;
  }
  if (checksumCalibration(calStore) != calStore.checksum) {
    imuCalibrated = false;
    escCalibrated = false;
    return;
  }
  memcpy(gyroBias, calStore.gyroBias, sizeof(gyroBias));
  memcpy(accelBias, calStore.accelBias, sizeof(accelBias));
  imuCalibrated = true;
  escCalibrated = true;  // ESC calibration is a stored flag in this build.
}

// ----------------- MPU6050 -----------------
bool writeMpu(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool initMpu() {
  if (!writeMpu(0x6B, 0x00)) {  // Wake up
    return false;
  }
  writeMpu(0x1B, 0x00);  // ±250 deg/s
  writeMpu(0x1C, 0x00);  // ±2 g
  return true;
}

bool readRawImu(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)14);
  if (Wire.available() < 14) {
    return false;
  }
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read();  // Temperature high
  Wire.read();  // Temperature low
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

void calibrateImu() {
  constexpr uint16_t samples = 3000;
  long gyroSum[3] = {0, 0, 0};
  long accelSum[3] = {0, 0, 0};
  calibrating = true;
  guideStep = GuideStep::CALIBRATING;
  for (uint16_t i = 0; i < samples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    if (readRawImu(ax, ay, az, gx, gy, gz)) {
      gyroSum[0] += gx;
      gyroSum[1] += gy;
      gyroSum[2] += gz;
      accelSum[0] += ax;
      accelSum[1] += ay;
      accelSum[2] += az;
    }
    if ((i % 100) == 0) {
      pollRadio();  // keep the link alive
    }
    delayMicroseconds(800);
  }
  gyroBias[0] = gyroSum[0] / (float)samples;
  gyroBias[1] = gyroSum[1] / (float)samples;
  gyroBias[2] = gyroSum[2] / (float)samples;
  accelBias[0] = accelSum[0] / (float)samples;
  accelBias[1] = accelSum[1] / (float)samples;
  // Remove 1 g from Z axis bias while level.
  accelBias[2] = (accelSum[2] / (float)samples) - ACC_LSB_PER_G;
  imuCalibrated = true;
  calibrating = false;
}

void calibrateEscs() {
  calibrating = true;
  guideStep = GuideStep::CALIBRATING;
  // Send high throttle to arm ESC calibration mode.
  for (uint8_t i = 0; i < 4; ++i) {
    esc[i].writeMicroseconds(BASE_MAX_US);
  }
  for (uint16_t t = 0; t < 2000; ++t) {
    pollRadio();
    delay(1);
  }
  for (uint8_t i = 0; i < 4; ++i) {
    esc[i].writeMicroseconds(BASE_MIN_US);
  }
  for (uint16_t t = 0; t < 2000; ++t) {
    pollRadio();
    delay(1);
  }
  escCalibrated = true;
  calibrating = false;
}

void performFullCalibration() {
  calibrateImu();
  calibrateEscs();
  saveCalibration();
  idleSpinComplete = false;
  spoolInProgress = false;
  idleTargetUs = BASE_MIN_US;
  pulseStatusLed(400);
  startBuzzer(3, 80, 80);
}

// ----------------- Radio Handling -----------------
void updateLinkQuality(bool goodFrame) {
  ++windowPacketTotal;
  if (goodFrame) {
    ++windowPacketGood;
  }
  uint32_t now = millis();
  if (now - lastLinkWindowMillis >= 500) {
    if (windowPacketTotal == 0) {
      linkQualityPercent = 0;
    } else {
      linkQualityPercent = (windowPacketGood * 100) / windowPacketTotal;
    }
    windowPacketTotal = 0;
    windowPacketGood = 0;
    lastLinkWindowMillis = now;
  }
}

void initRadio() {
  if (!radio.begin()) {
    startBuzzer(5, 80, 80);
    return;
  }
  radio.setChannel(RADIO_CHANNEL);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(0, RADIO_PIPE);
  radio.startListening();
  queueTelemetryAck();
}

void applyControlPacket(const ControlPacket &packet) {
  latestControls = packet;
  lastPacketMicros = micros();
  if (!linkActive) {
    linkActive = true;
    guideStep = GuideStep::REQUEST_KILL;
    connectionAnnounced = false;
  }
  bool button1 = packet.button1 != 0;
  bool button2 = packet.button2 != 0;
  if (button1 && !button1Prev && !calibrating && packet.killSwitch == 0) {
    pendingCalibrationRequest = true;
    pulseStatusLed(STATUS_LED_PULSE_MS);
  }
  if (button2 && !button2Prev && armed && (guideStep == GuideStep::REQUEST_IDLE_SPIN)) {
    idleSpinComplete = false;
    spoolInProgress = true;
    idleTargetUs = IDLE_SPIN_US;
    pulseStatusLed(STATUS_LED_PULSE_MS);
  }
  if ((button1 && !button1Prev) || (button2 && !button2Prev)) {
    pulseStatusLed(STATUS_LED_PULSE_MS);
  }
  button1Prev = button1;
  button2Prev = button2;
}

void pollRadio() {
  while (radio.available()) {
    ControlPacket incoming{};
    radio.read(&incoming, sizeof(incoming));
    bool valid = incoming.version == PACKET_VERSION && isChecksumValid(incoming);
    updateLinkQuality(valid);
    if (valid) {
      applyControlPacket(incoming);
      queueTelemetryAck();
    }
  }
}

void failsafeCheck(uint32_t nowMicros) {
  if (!linkActive) {
    return;
  }
  if ((nowMicros - lastPacketMicros) > FAILSAFE_TIMEOUT_US) {
    linkActive = false;
    armed = false;
    idleSpinComplete = false;
    idleTargetUs = BASE_MIN_US;
    spoolInProgress = false;
    guideStep = GuideStep::WAIT_LINK;
    rollPid.reset();
    pitchPid.reset();
    yawPid.reset();
    startBuzzer(4, 60, 60);
  }
}

// ----------------- Input Mapping -----------------
float normalizeStick(uint16_t pulse) {
  float value = (static_cast<float>(pulse) - 1500.0f) / 500.0f;
  value = clampFloat(value, -1.0f, 1.0f);
  value = applyDeadband(value, STICK_DEADBAND);
  value = applyExpo(value, STICK_EXPO);
  return value;
}

float mapThrottle(uint16_t pulse) {
  float value = normalizeStick(pulse);  // -1..1
  // Shift to 0..1 to keep hover around center while kill switch enforces idle.
  float uni = (value + 1.0f) * 0.5f;
  return BASE_MIN_US + (uni * 1000.0f);
}

float mapAngleChannel(uint16_t pulse) {
  return normalizeStick(pulse) * MAX_TILT_DEG;
}

float mapYawChannel(uint16_t pulse) {
  return normalizeStick(pulse) * MAX_YAW_RATE_DPS;
}

// ----------------- IMU Update -----------------
void updateImu(float dt) {
  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  if (!readRawImu(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz)) {
    return;
  }
  float gx = (rawGx - gyroBias[0]) / GYRO_LSB_PER_DPS;
  float gy = (rawGy - gyroBias[1]) / GYRO_LSB_PER_DPS;
  float gz = (rawGz - gyroBias[2]) / GYRO_LSB_PER_DPS;
  float ax = (rawAx - accelBias[0]) / ACC_LSB_PER_G;
  float ay = (rawAy - accelBias[1]) / ACC_LSB_PER_G;
  float az = (rawAz - accelBias[2]) / ACC_LSB_PER_G;
  gyroRates[0] = gx;
  gyroRates[1] = gy;
  gyroRates[2] = gz;

  rollAngleDeg += gx * dt;
  pitchAngleDeg += gy * dt;
  yawAngleDeg += gz * dt;

  float accRoll = atan2f(ay, az) * RAD_TO_DEG;
  float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  constexpr float alpha = 0.98f;
  rollAngleDeg = alpha * rollAngleDeg + (1.0f - alpha) * accRoll;
  pitchAngleDeg = alpha * pitchAngleDeg + (1.0f - alpha) * accPitch;

  // Altitude estimator (simple complementary filter)
  float sinRoll = sinf(rollAngleDeg * DEG_TO_RAD);
  float cosRoll = cosf(rollAngleDeg * DEG_TO_RAD);
  float sinPitch = sinf(pitchAngleDeg * DEG_TO_RAD);
  float cosPitch = cosf(pitchAngleDeg * DEG_TO_RAD);
  float accZWorld = (cosPitch * cosRoll * az) +
                    (cosPitch * sinRoll * ay) -
                    (sinPitch * ax);
  float linearAccZ = (accZWorld - 1.0f) * GRAVITY;
  verticalVelocity += linearAccZ * dt;
  verticalVelocity *= 0.98f;  // simple damping
  altitudeMeters += verticalVelocity * dt;
  if (altitudeMeters < 0.0f) altitudeMeters = 0.0f;
}

// ----------------- Battery Monitoring -----------------
float readBatteryVoltage() {
  // Assumes 100k (R1) from VBAT to A0 and 10k (R2) from A0 to GND.
  constexpr float VREF = 5.0f;
  constexpr float DIVIDER_RATIO = (100000.0f + 10000.0f) / 10000.0f;
  int raw = analogRead(VBAT_PIN);
  float voltage = (raw / 1023.0f) * VREF * DIVIDER_RATIO;
  return voltage;
}

// ----------------- State Machine -----------------
bool calibrationsComplete() {
  return imuCalibrated && escCalibrated;
}

void handleGuidedWorkflow() {
  if (!linkActive) {
    guideStep = GuideStep::WAIT_LINK;
    armed = false;
    idleSpinComplete = false;
    idleTargetUs = BASE_MIN_US;
    spoolInProgress = false;
    return;
  }
  if (calibrating) {
    guideStep = GuideStep::CALIBRATING;
    return;
  }
  bool killSafe = latestControls.killSwitch == 0;
  bool requestArm = !killSafe;

  if (guideStep == GuideStep::WAIT_LINK) {
    guideStep = GuideStep::REQUEST_KILL;
  }
  if (!killSafe && !armed && guideStep <= GuideStep::REQUEST_CAL) {
    guideStep = GuideStep::REQUEST_KILL;
    return;
  }
  if (guideStep == GuideStep::REQUEST_KILL && killSafe) {
    guideStep = GuideStep::REQUEST_CAL;
  }
  if (guideStep == GuideStep::REQUEST_CAL && calibrationsComplete()) {
    guideStep = GuideStep::REQUEST_ARM;
  }

  if (!calibrationsComplete()) {
    guideStep = GuideStep::REQUEST_CAL;
    return;
  }

  if (!armed) {
    if (!requestArm) {
      guideStep = GuideStep::REQUEST_ARM;
      return;
    }
    armed = true;
    idleSpinComplete = false;
    spoolInProgress = false;
    idleTargetUs = BASE_MIN_US;
    guideStep = GuideStep::REQUEST_IDLE_SPIN;
    pulseStatusLed(STATUS_LED_PULSE_MS);
    startBuzzer(2, 40, 60);
  }

  if (!requestArm) {
    armed = false;
    idleSpinComplete = false;
    spoolInProgress = false;
    idleTargetUs = BASE_MIN_US;
    guideStep = GuideStep::REQUEST_ARM;
    return;
  }

  if (!idleSpinComplete) {
    guideStep = GuideStep::REQUEST_IDLE_SPIN;
    return;
  }

  if (throttleCommandUs > (IDLE_SPIN_US + 15)) {
    guideStep = GuideStep::FLYING;
  } else {
    guideStep = GuideStep::READY;
  }
}

// ----------------- Motor Control -----------------
void writeMotors(float m1, float m2, float m3, float m4) {
  esc[0].writeMicroseconds(clampPulse(m1));
  esc[1].writeMicroseconds(clampPulse(m2));
  esc[2].writeMicroseconds(clampPulse(m3));
  esc[3].writeMicroseconds(clampPulse(m4));
}

void updateMotors(float dt) {
  float targetThrottle = mapThrottle(latestControls.throttle);
  throttleCommandUs = targetThrottle;
  if (!armed) {
    writeMotors(BASE_MIN_US, BASE_MIN_US, BASE_MIN_US, BASE_MIN_US);
    idleThrottleUs = BASE_MIN_US;
    spoolInProgress = false;
    return;
  }

  idleThrottleUs += (idleTargetUs - idleThrottleUs) * 0.02f;
  if (spoolInProgress && fabs(idleThrottleUs - idleTargetUs) < 1.0f) {
    spoolInProgress = false;
    idleSpinComplete = true;
    startBuzzer(2, 70, 70);
  }

  float throttleBase = max(targetThrottle, idleThrottleUs);
  throttleBase = clampFloat(throttleBase, BASE_MIN_US, BASE_MAX_US);

  float rollSet = mapAngleChannel(latestControls.roll);
  float pitchSet = mapAngleChannel(latestControls.pitch);
  float yawRateSet = mapYawChannel(latestControls.yaw);

  float rollError = rollSet - rollAngleDeg;
  float pitchError = pitchSet - pitchAngleDeg;
  float yawError = yawRateSet - gyroRates[2];

  float rollCorr = rollPid.update(rollError, dt);
  float pitchCorr = pitchPid.update(pitchError, dt);
  float yawCorr = yawPid.update(yawError, dt);

  float motor[4];
  motor[0] = throttleBase + pitchCorr + rollCorr - yawCorr;  // Front-left
  motor[1] = throttleBase + pitchCorr - rollCorr + yawCorr;  // Front-right
  motor[2] = throttleBase - pitchCorr + rollCorr + yawCorr;  // Rear-left
  motor[3] = throttleBase - pitchCorr - rollCorr - yawCorr;  // Rear-right

  for (uint8_t i = 0; i < 4; ++i) {
    if (motor[i] < idleThrottleUs) {
      motor[i] = idleThrottleUs;
    }
    if (motor[i] > BASE_MAX_US) {
      motor[i] = BASE_MAX_US;
    }
  }

  writeMotors(motor[0], motor[1], motor[2], motor[3]);
}

// ----------------- Telemetry -----------------
void queueTelemetryAck() {
  telemetry.version = PACKET_VERSION;
  telemetry.altitudeMeters = altitudeMeters;
  telemetry.batteryVoltage = batteryVoltage;
  telemetry.throttleEcho = clampPulse(throttleCommandUs);
  telemetry.rollEcho = latestControls.roll;
  telemetry.pitchEcho = latestControls.pitch;
  telemetry.yawEcho = latestControls.yaw;
  telemetry.armed = armed ? 1 : 0;
  telemetry.imuCalibrated = imuCalibrated ? 1 : 0;
  telemetry.escCalibrated = escCalibrated ? 1 : 0;
  telemetry.linkQuality = linkQualityPercent;
  telemetry.guideStep = static_cast<uint8_t>(guideStep);
  telemetry.checksum = computeChecksum(telemetry);
  radio.writeAckPayload(0, &telemetry, sizeof(telemetry));
}

// ----------------- Setup & Loop -----------------
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ARMED_LED_PIN, OUTPUT);
  pinMode(VBAT_PIN, INPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(ARMED_LED_PIN, LOW);

  Wire.begin();
  Wire.setClock(400000);
  initMpu();
  loadCalibration();

  for (uint8_t i = 0; i < 4; ++i) {
    esc[i].attach(ESC_PINS[i], BASE_MIN_US, BASE_MAX_US);
    esc[i].writeMicroseconds(BASE_MIN_US);
  }

  initRadio();
  startBuzzer(2, 100, 100);
  lastLoopMicros = micros();
  lastLinkWindowMillis = millis();
}

void loop() {
  uint32_t nowMicros = micros();
  uint32_t loopDelta = nowMicros - lastLoopMicros;
  if (loopDelta < LOOP_PERIOD_US) {
    pollRadio();
    serviceBuzzer(millis());
    serviceStatusLed(millis());
    return;
  }
  lastLoopMicros = nowMicros;

  float dt = loopDelta / 1e6f;
  pollRadio();
  failsafeCheck(nowMicros);

  if (pendingCalibrationRequest && !calibrating && latestControls.killSwitch == 0) {
    pendingCalibrationRequest = false;
    performFullCalibration();
  }

  if (linkActive && !connectionAnnounced) {
    startBuzzer(2, 70, 90);
    connectionAnnounced = true;
  }

  updateImu(dt);
  batteryVoltage = readBatteryVoltage();
  handleGuidedWorkflow();
  updateMotors(dt);
  queueTelemetryAck();

  digitalWrite(ARMED_LED_PIN, armed ? HIGH : LOW);
  serviceBuzzer(millis());
  serviceStatusLed(millis());
}
