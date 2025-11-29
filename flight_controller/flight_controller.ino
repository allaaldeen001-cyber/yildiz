/*
 * Flight Controller Firmware for the Professional Nano UAV Platform
 *
 * Responsibilities:
 *  - Fuse MPU6050 (gyro/accel) + MS5611 (barometer) data.
 *  - Enforce a deterministic state machine with arming, calibration, and failsafes.
 *  - Drive four ESCs via Servo pulses (1–2 ms) on pins D3/D5/D6/D9.
 *  - Exchange RF24 control / telemetry frames with CRC + ACK payloads.
 */

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <MPU6050.h>
#include <MS5611.h>
#include <math.h>

// ----------------------------- Pin Map ------------------------------------
constexpr uint8_t PIN_RADIO_CE = 4;
constexpr uint8_t PIN_RADIO_CSN = 10;
constexpr uint8_t PIN_IMU_INT = 2;
constexpr uint8_t PIN_LED = 7;
constexpr uint8_t PIN_BUZZER = 8;

constexpr uint8_t PIN_ESC_FL = 3;
constexpr uint8_t PIN_ESC_FR = 5;
constexpr uint8_t PIN_ESC_RR = 6;
constexpr uint8_t PIN_ESC_RL = 9;

constexpr uint64_t PIPE_ADDRESS = 0xE7E7E7E7E7ULL;

// ------------------------ Protocol Definitions ----------------------------
constexpr uint8_t FLAG_CALIBRATE = 0x01;
constexpr uint8_t FLAG_MOTOR_ENABLE = 0x02;
constexpr uint8_t FLAG_ESC_CAL = 0x04;

constexpr uint8_t SWITCH_ALT_HOLD = 0x01;
constexpr uint8_t SWITCH_KILL_SAFE = 0x02;

struct __attribute__((packed)) ControlFrame {
  uint32_t sequence;
  uint16_t throttleUs;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
  uint8_t flags;
  uint8_t switchMask;
  uint16_t crc;
};

struct __attribute__((packed)) TelemetryFrame {
  uint8_t fcState;
  uint8_t linkQuality;
  uint8_t calibrationCode;
  uint16_t batteryMv;
  int16_t baroAltCm;
  uint8_t faultCode;
};

enum FcState : uint8_t { FC_INIT, FC_READY, FC_CAL, FC_ARMED, FC_ESC_CAL, FC_FAULT };
enum CalCode : uint8_t { CAL_IDLE, CAL_RUNNING, CAL_SUCCESS, CAL_FAILED };
enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_LINK_LOSS = 1,
  FAULT_KILL_SWITCH = 2,
  FAULT_SENSOR = 3,
  FAULT_NOT_CALIBRATED = 4,
  FAULT_ESC_DENIED = 5
};

// ----------------------------- Constants ----------------------------------
constexpr uint16_t kThrottleMinUs = 1000;
constexpr uint16_t kThrottleMaxUs = 2000;
constexpr uint16_t kThrottleCapUs = 1650; // 65 % power cap
constexpr uint8_t kMaxTiltDeg = 30;
constexpr float kAngleKp = 6.0f;
constexpr float kAngleMixUs = 220.0f;
constexpr float kYawMixUs = 120.0f;
constexpr unsigned long kFailsafeMs = 500UL;
constexpr unsigned long kLedFastMs = 100UL;
constexpr unsigned long kLedSlowMs = 500UL;
constexpr unsigned long kBaroPeriodMs = 50UL;
constexpr float kAltKp = 0.8f;
constexpr float kAltKi = 0.04f;
constexpr float kAltIntegralLimit = 150.0f;

#define ENABLE_BATTERY_MONITOR 0

// ------------------------------ Globals -----------------------------------
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);
MPU6050 imu;
MS5611 barometer;
Servo escFL, escFR, escRR, escRL;

volatile bool imuDataReady = false;

ControlFrame currentCommand = {};
TelemetryFrame telemetry = {};

FcState fcState = FC_INIT;
CalCode calibrationCode = CAL_FAILED;
FaultCode lastFault = FAULT_NONE;

uint32_t lastPacketMs = 0;
uint32_t lastImuMicros = 0;
uint32_t lastLedMs = 0;
uint32_t lastBaroMs = 0;
bool ledState = false;
bool altitudeHoldRequest = false;
bool altitudeHoldActive = false;

float attitudeRoll = 0.0f;
float attitudePitch = 0.0f;
float attitudeYaw = 0.0f;
float gyroBias[3] = {0.0f, 0.0f, 0.0f};

float altitudeCm = 0.0f;
float targetAltitudeCm = 0.0f;
float altitudeIntegral = 0.0f;
float baroZeroCm = 0.0f;

uint16_t motorUs[4] = {kThrottleMinUs, kThrottleMinUs, kThrottleMinUs, kThrottleMinUs};

// ---------------------------- Forward Decl --------------------------------
void handleControlFrame(const ControlFrame &frame);
void armMotors();
void disarm(FaultCode reason);
void runEscCalibration();
uint16_t applyAltitudeHold(uint16_t throttleUs);
void updateMotorMix();
void writeMotors();
void publishTelemetry();
void playCalibrationSuccessTone();
void playCalibrationFailureTone();
void playEscCalibrationTone(uint8_t motorIdx);

// ----------------------------- Utilities ----------------------------------
uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool validateFrame(const ControlFrame &frame) {
  uint16_t computed = crc16(reinterpret_cast<const uint8_t *>(&frame), sizeof(ControlFrame) - sizeof(uint16_t));
  return computed == frame.crc;
}

void imuIsr() {
  imuDataReady = true;
}

void beep(uint16_t frequency, uint16_t durationMs) {
  tone(PIN_BUZZER, frequency, durationMs);
  delay(durationMs);
  noTone(PIN_BUZZER);
}

void playCalibrationSuccessTone() {
  beep(2200, 120);
  delay(80);
  beep(2600, 120);
}

void playCalibrationFailureTone() {
  beep(900, 7000);
}

void playEscCalibrationTone(uint8_t motorIdx) {
  tone(PIN_BUZZER, 1400 + motorIdx * 150, 200);
}

void playFailsafeTone() {
  tone(PIN_BUZZER, 600, 400);
  delay(20);
  noTone(PIN_BUZZER);
}

void setAllMotors(uint16_t pulseUs) {
  motorUs[0] = motorUs[1] = motorUs[2] = motorUs[3] = pulseUs;
  writeMotors();
}

void writeMotors() {
  escFL.writeMicroseconds(motorUs[0]);
  escFR.writeMicroseconds(motorUs[1]);
  escRR.writeMicroseconds(motorUs[2]);
  escRL.writeMicroseconds(motorUs[3]);
}

uint16_t readBatteryMv() {
#if ENABLE_BATTERY_MONITOR
  const uint8_t PIN_BATT = A6; // tie via divider (e.g., 1:11)
  const float dividerRatio = 11.0f;
  int raw = analogRead(PIN_BATT);
  float voltage = (raw * (5.0f / 1023.0f)) * dividerRatio;
  return static_cast<uint16_t>(voltage * 1000.0f);
#else
  return 0;
#endif
}

// --------------------------- Sensor Handling ------------------------------
bool initSensors() {
  Wire.begin();
  imu.initialize();
  if (!imu.testConnection()) {
    return false;
  }
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  imu.setDLPFMode(MPU6050_DLPF_BW_42);
  pinMode(PIN_IMU_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), imuIsr, RISING);
  imu.setIntDataReadyEnabled(true);

  if (!barometer.begin()) {
    return false;
  }
  barometer.read();
  delay(10);
  barometer.read();
  baroZeroCm = barometer.getAltitude() * 100.0f;
  altitudeCm = 0.0f;

  return true;
}

bool runGyroCalibration() {
  const uint16_t samples = 1200;
  long sumX = 0;
  long sumY = 0;
  long sumZ = 0;
  for (uint16_t i = 0; i < samples; ++i) {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(2);
  }
  gyroBias[0] = static_cast<float>(sumX) / samples;
  gyroBias[1] = static_cast<float>(sumY) / samples;
  gyroBias[2] = static_cast<float>(sumZ) / samples;
  return true;
}

bool runBarometerCalibration() {
  const uint8_t samples = 60;
  float accum = 0.0f;
  for (uint8_t i = 0; i < samples; ++i) {
    barometer.read();
    accum += barometer.getAltitude();
    delay(20);
  }
  baroZeroCm = (accum / samples) * 100.0f;
  altitudeCm = 0.0f;
  targetAltitudeCm = 0.0f;
  altitudeIntegral = 0.0f;
  return true;
}

void updateAttitude() {
  if (!imuDataReady && (micros() - lastImuMicros) < 5000) {
    return;
  }
  imuDataReady = false;

  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  uint32_t now = micros();
  float dt = (lastImuMicros == 0) ? 0.0f : (now - lastImuMicros) / 1000000.0f;
  lastImuMicros = now;
  if (dt <= 0.0f) {
    return;
  }

  const float gyroScale = 65.5f; // LSB/deg/s @ ±500 dps
  float gyroX = (gx - gyroBias[0]) / gyroScale;
  float gyroY = (gy - gyroBias[1]) / gyroScale;
  float gyroZ = (gz - gyroBias[2]) / gyroScale;

  float accelRoll = atan2f(static_cast<float>(ay), static_cast<float>(az)) * RAD_TO_DEG;
  float accelPitch = atan2f(-static_cast<float>(ax), sqrtf(static_cast<float>(ay) * ay + static_cast<float>(az) * az)) * RAD_TO_DEG;

  attitudeRoll = 0.98f * (attitudeRoll + gyroX * dt) + 0.02f * accelRoll;
  attitudePitch = 0.98f * (attitudePitch + gyroY * dt) + 0.02f * accelPitch;
  attitudeYaw += gyroZ * dt;
  if (attitudeYaw > 180.0f) attitudeYaw -= 360.0f;
  if (attitudeYaw < -180.0f) attitudeYaw += 360.0f;
}

void updateBarometer() {
  if (millis() - lastBaroMs < kBaroPeriodMs) {
    return;
  }
  lastBaroMs = millis();
  barometer.read();
  float rawCm = barometer.getAltitude() * 100.0f - baroZeroCm;
  altitudeCm = altitudeCm * 0.85f + rawCm * 0.15f;
}

// ------------------------------ RF Link -----------------------------------
void initRadio() {
  if (!radio.begin()) {
    disarm(FAULT_SENSOR);
    while (true) {
      digitalWrite(PIN_LED, (millis() / 200) % 2);
    }
  }
  radio.setChannel(103);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.openReadingPipe(1, PIPE_ADDRESS);
  radio.startListening();
  radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
}

void serviceRadio() {
  while (radio.available()) {
    ControlFrame frame;
    radio.read(&frame, sizeof(frame));
    if (!validateFrame(frame)) {
      continue;
    }
    lastPacketMs = millis();
    handleControlFrame(frame);
  }
}

// ------------------------------ Control -----------------------------------
void armMotors() {
  if (fcState == FC_ARMED) {
    return;
  }
  fcState = FC_ARMED;
  altitudeHoldActive = false;
  altitudeIntegral = 0.0f;
  targetAltitudeCm = altitudeCm;
}

void disarm(FaultCode reason) {
  setAllMotors(kThrottleMinUs);
  if (reason == FAULT_NONE) {
    fcState = FC_READY;
  } else if (reason == FAULT_SENSOR) {
    fcState = FC_FAULT;
  } else if (reason == FAULT_LINK_LOSS) {
    fcState = FC_FAULT;
  } else {
    fcState = FC_READY;
  }
  lastFault = reason;
}

void runEscCalibration() {
  if (fcState == FC_ARMED) {
    return;
  }
  fcState = FC_ESC_CAL;
  Serial.println(F("[FC] ESC calibration routine"));
  for (uint8_t idx = 0; idx < 4; ++idx) {
    playEscCalibrationTone(idx);
    for (uint16_t pulse = kThrottleMinUs; pulse <= kThrottleCapUs; pulse += 5) {
      motorUs[idx] = pulse;
      writeMotors();
      delay(4);
    }
    for (int pulse = kThrottleCapUs; pulse >= kThrottleMinUs; pulse -= 5) {
      motorUs[idx] = pulse;
      writeMotors();
      delay(4);
    }
  }
  setAllMotors(kThrottleMinUs);
  fcState = FC_READY;
}

void handleControlFrame(const ControlFrame &frame) {
  altitudeHoldRequest = (frame.switchMask & SWITCH_ALT_HOLD);
  bool killSafe = (frame.switchMask & SWITCH_KILL_SAFE);

  if (!killSafe) {
    disarm(FAULT_KILL_SWITCH);
    return;
  }

  if (fcState == FC_FAULT && lastFault != FAULT_SENSOR) {
    fcState = FC_READY;
  }

  if (frame.flags & FLAG_CALIBRATE) {
    if (fcState == FC_ARMED) {
      lastFault = FAULT_NOT_CALIBRATED;
    } else {
      fcState = FC_CAL;
      calibrationCode = CAL_RUNNING;
      bool gyroOk = runGyroCalibration();
      bool baroOk = runBarometerCalibration();
      if (gyroOk && baroOk) {
        calibrationCode = CAL_SUCCESS;
        playCalibrationSuccessTone();
      } else {
        calibrationCode = CAL_FAILED;
        playCalibrationFailureTone();
      }
      fcState = FC_READY;
    }
  }

  if ((frame.flags & FLAG_ESC_CAL) && fcState != FC_ARMED) {
    if (!altitudeHoldRequest) {
      runEscCalibration();
    } else {
      lastFault = FAULT_ESC_DENIED;
    }
  }

  if ((frame.flags & FLAG_MOTOR_ENABLE) && fcState == FC_READY) {
    if (calibrationCode == CAL_SUCCESS) {
      armMotors();
    } else {
      lastFault = FAULT_NOT_CALIBRATED;
    }
  } else if (!(frame.flags & FLAG_MOTOR_ENABLE) && fcState == FC_ARMED) {
    disarm(FAULT_NONE);
  }

  currentCommand = frame;
}

uint16_t applyAltitudeHold(uint16_t throttleUs) {
  if (!altitudeHoldRequest) {
    altitudeHoldActive = false;
    altitudeIntegral = 0.0f;
    return throttleUs;
  }
  if (!altitudeHoldActive) {
    altitudeHoldActive = true;
    targetAltitudeCm = altitudeCm;
    altitudeIntegral = 0.0f;
  }
  float error = targetAltitudeCm - altitudeCm;
  altitudeIntegral = constrain(altitudeIntegral + error * kAltKi, -kAltIntegralLimit, kAltIntegralLimit);
  float adjust = kAltKp * error + altitudeIntegral;
  float commanded = throttleUs + adjust;
  return static_cast<uint16_t>(constrain(commanded, kThrottleMinUs, kThrottleCapUs));
}

void updateMotorMix() {
  if (fcState != FC_ARMED) {
    setAllMotors(kThrottleMinUs);
    return;
  }

  uint16_t throttle = constrain(currentCommand.throttleUs, kThrottleMinUs, kThrottleMaxUs);
  if (throttle > kThrottleCapUs) {
    throttle = kThrottleCapUs;
  }
  throttle = applyAltitudeHold(throttle);

  float rollCmd = constrain(static_cast<float>(currentCommand.roll) / 500.0f, -1.0f, 1.0f);
  float pitchCmd = constrain(static_cast<float>(currentCommand.pitch) / 500.0f, -1.0f, 1.0f);
  float yawCmd = constrain(static_cast<float>(currentCommand.yaw) / 500.0f, -1.0f, 1.0f);

  float targetRoll = rollCmd * kMaxTiltDeg;
  float targetPitch = pitchCmd * kMaxTiltDeg;

  float rollError = targetRoll - attitudeRoll;
  float pitchError = targetPitch - attitudePitch;

  float rollTerm = constrain(rollError * kAngleKp, -kAngleMixUs, kAngleMixUs);
  float pitchTerm = constrain(pitchError * kAngleKp, -kAngleMixUs, kAngleMixUs);
  float yawTerm = yawCmd * kYawMixUs;

  motorUs[0] = constrain(throttle + pitchTerm - rollTerm + yawTerm, kThrottleMinUs, kThrottleMaxUs); // FL
  motorUs[1] = constrain(throttle + pitchTerm + rollTerm - yawTerm, kThrottleMinUs, kThrottleMaxUs); // FR
  motorUs[2] = constrain(throttle - pitchTerm + rollTerm + yawTerm, kThrottleMinUs, kThrottleMaxUs); // RR
  motorUs[3] = constrain(throttle - pitchTerm - rollTerm - yawTerm, kThrottleMinUs, kThrottleMaxUs); // RL

  writeMotors();
}

// ------------------------------ Telemetry ---------------------------------
uint8_t computeLinkQuality() {
  if (lastPacketMs == 0) {
    return 0;
  }
  unsigned long age = millis() - lastPacketMs;
  if (age < 200) {
    return 100;
  }
  if (age > 1000) {
    return 0;
  }
  long ageLong = static_cast<long>(age);
  return static_cast<uint8_t>(map(ageLong, 200L, 1000L, 90L, 0L));
}

void publishTelemetry() {
  telemetry.fcState = fcState;
  telemetry.calibrationCode = calibrationCode;
  telemetry.faultCode = lastFault;
  telemetry.baroAltCm = static_cast<int16_t>(constrain(altitudeCm, -32768.0f, 32767.0f));
  telemetry.batteryMv = readBatteryMv();
  telemetry.linkQuality = computeLinkQuality();
  radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
}

// ------------------------------ Status LED --------------------------------
void serviceStatusLed() {
  unsigned long interval = (millis() - lastPacketMs) < kFailsafeMs ? kLedFastMs : kLedSlowMs;
  if (fcState == FC_FAULT) {
    interval = 150UL;
  }
  if (millis() - lastLedMs >= interval) {
    lastLedMs = millis();
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState);
  }
}

// ------------------------------ Failsafe ----------------------------------
void enforceFailsafe() {
  if (fcState == FC_ARMED && (millis() - lastPacketMs) > kFailsafeMs) {
    disarm(FAULT_LINK_LOSS);
    playFailsafeTone();
  }
}

// ------------------------------ Arduino -----------------------------------
void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  escFL.attach(PIN_ESC_FL, 1000, 2000);
  escFR.attach(PIN_ESC_FR, 1000, 2000);
  escRR.attach(PIN_ESC_RR, 1000, 2000);
  escRL.attach(PIN_ESC_RL, 1000, 2000);
  setAllMotors(kThrottleMinUs);

  bool sensorsOk = initSensors();
  if (!sensorsOk) {
    disarm(FAULT_SENSOR);
    while (true) {
      digitalWrite(PIN_LED, (millis() / 200) % 2);
      tone(PIN_BUZZER, 600, 250);
      delay(300);
    }
  }

  initRadio();
  fcState = FC_READY;
  calibrationCode = CAL_FAILED; // force explicit calibration from RC
  publishTelemetry();
  Serial.println(F("[FC] Flight controller ready."));
}

void loop() {
  serviceRadio();
  updateAttitude();
  updateBarometer();
  enforceFailsafe();
  updateMotorMix();
  serviceStatusLed();
  publishTelemetry();
}
