#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

#include "filters.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "pid.h"
#include "rf_protocol.h"

using namespace drone;

namespace {
constexpr uint8_t kLedPin = 7;
constexpr uint8_t kBuzzerPin = 8;
constexpr uint8_t kEscPins[4] = {3, 5, 6, 9};
constexpr uint8_t kImuIntPin = 2;
constexpr uint16_t kPwmMin = 1000;
constexpr uint16_t kPwmMax = 2000;
constexpr uint16_t kPwmMaxSafe = 1650;  // 65% throttle ceiling
constexpr float kMaxAngleDeg = 30.0f;
constexpr float kMaxYawRateDps = 120.0f;
constexpr uint32_t kLinkTimeoutUs = 250000;
enum MotorIndex { kMotorFL = 0, kMotorFR, kMotorRR, kMotorRL };

RF24 radio(4, 10);
Servo motors[4];
Mpu6050 imu;
Ms5611 baro;
ComplementaryFilter attitude_filter(0.985f);
PidController roll_pid;
PidController pitch_pid;
PidController yaw_pid;
PidController alt_pid;

RcCommand latest_command = {kPwmMin, 0, 0, 0, 0, 0, 0, 0, 0};
FcStatus status_payload = {};

ImuRaw imu_raw = {};
float ax_g = 0.0f, ay_g = 0.0f, az_g = 0.0f;
float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps = 0.0f;

float gyro_offsets[3] = {0.0f, 0.0f, 0.0f};
float accel_offsets[3] = {0.0f, 0.0f, 0.0f};

uint32_t last_loop_start = 0;
uint32_t last_command_micros = 0;
uint32_t last_led_toggle = 0;
uint32_t last_altitude_sample = 0;

float altitude_cm = 0.0f;
float altitude_hold_target_cm = 0.0f;

bool link_alive = false;
bool imu_calibrated = false;
bool esc_calibrated = false;
bool calibration_running = false;
bool armed = false;
bool led_state = false;

uint8_t prev_buttons = 0;
uint8_t prev_switches = 0;

bool esc_outputs_enabled = false;

const uint16_t kAccel1gCounts = 8192;

void setupRadio() {
  radio.begin();
  radio.setChannel(kRadioChannel);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  radio.setRetries(5, 15);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, kRadioPipe);
  radio.startListening();
}

void attachMotors() {
  for (uint8_t i = 0; i < 4; ++i) {
    motors[i].attach(kEscPins[i], kPwmMin, kPwmMax);
    motors[i].writeMicroseconds(kPwmMin);
  }
}

void playTone(uint16_t freq, uint16_t duration_ms) {
  tone(kBuzzerPin, freq, duration_ms);
  delay(duration_ms + 10);
  noTone(kBuzzerPin);
}

void playCalibrationOkTone() {
  playTone(1600, 180);
  delay(120);
  playTone(1900, 220);
}

void playCalibrationFailTone() {
  playTone(400, 7000);
}

void playEscTone() {
  playTone(1200, 400);
  delay(80);
  playTone(900, 400);
}

void disarmMotors() {
  armed = false;
  for (auto& motor : motors) {
    motor.writeMicroseconds(kPwmMin);
  }
}

bool acquireImuOffsets(uint16_t sample_count) {
  int64_t sum_gyro[3] = {0, 0, 0};
  int64_t sum_accel[3] = {0, 0, 0};
  for (uint16_t i = 0; i < sample_count; ++i) {
    if (!imu.readRaw(imu_raw)) {
      return false;
    }
    sum_accel[0] += imu_raw.ax;
    sum_accel[1] += imu_raw.ay;
    sum_accel[2] += imu_raw.az;
    sum_gyro[0] += imu_raw.gx;
    sum_gyro[1] += imu_raw.gy;
    sum_gyro[2] += imu_raw.gz;
    delay(2);
  }

  gyro_offsets[0] = static_cast<float>(sum_gyro[0]) / sample_count;
  gyro_offsets[1] = static_cast<float>(sum_gyro[1]) / sample_count;
  gyro_offsets[2] = static_cast<float>(sum_gyro[2]) / sample_count;

  accel_offsets[0] = static_cast<float>(sum_accel[0]) / sample_count;
  accel_offsets[1] = static_cast<float>(sum_accel[1]) / sample_count;
  accel_offsets[2] = static_cast<float>(sum_accel[2]) / sample_count - kAccel1gCounts;

  imu.setGyroOffsets(gyro_offsets[0], gyro_offsets[1], gyro_offsets[2]);
  imu.setAccelOffsets(accel_offsets[0], accel_offsets[1], accel_offsets[2]);
  return true;
}

bool runImuCalibration(uint16_t sample_count = 2000) {
  calibration_running = true;
  bool ok = acquireImuOffsets(sample_count);
  calibration_running = false;
  if (ok) {
    imu_calibrated = true;
    playCalibrationOkTone();
  } else {
    imu_calibrated = false;
    playCalibrationFailTone();
  }
  return ok;
}

void runEscCalibration() {
  calibration_running = true;
  disarmMotors();
  playEscTone();
  for (uint8_t idx = 0; idx < 4; ++idx) {
    for (uint16_t pulse = kPwmMin; pulse <= kPwmMaxSafe; pulse += 5) {
      motors[idx].writeMicroseconds(pulse);
      delay(5);
    }
    for (int pulse = kPwmMaxSafe; pulse >= kPwmMin; pulse -= 5) {
      motors[idx].writeMicroseconds(pulse);
      delay(5);
    }
  }
  for (auto& motor : motors) {
    motor.writeMicroseconds(kPwmMin);
  }
  esc_calibrated = true;
  calibration_running = false;
  playEscTone();
}

void updateRadioLink() {
  while (radio.available()) {
    radio.read(&latest_command, sizeof(latest_command));
    last_command_micros = micros();
    link_alive = true;
  }
  if ((micros() - last_command_micros) > kLinkTimeoutUs) {
    if (link_alive) {
      playTone(300, 200);
    }
    link_alive = false;
    disarmMotors();
    status_payload.last_error = kErrorRadio;
  }
}

void updateSensors(float dt_s) {
  status_payload.last_error = kErrorNone;
  if (!imu.readRaw(imu_raw)) {
    status_payload.last_error = kErrorImu;
    return;
  }
  imu.toEngineeringUnits(imu_raw, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
  attitude_filter.update(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt_s);

  if (millis() - last_altitude_sample > 50) {
    float alt_cm = 0.0f;
    if (baro.readAltitude(alt_cm)) {
      altitude_cm = alt_cm;
      status_payload.last_error = kErrorNone;
    } else {
      status_payload.last_error = kErrorBarometer;
    }
    last_altitude_sample = millis();
  }
}

float mapAxisToAngle(int16_t value) {
  return constrain(value, -500, 500) * (kMaxAngleDeg / 500.0f);
}

float mapAxisToYawRate(int16_t value) {
  return constrain(value, -500, 500) * (kMaxYawRateDps / 500.0f);
}

void processActions() {
  uint8_t buttons = latest_command.buttons;
  uint8_t switches = latest_command.switches;
  bool button1_edge = (buttons & kButtonCalibrate) && !(prev_buttons & kButtonCalibrate);
  bool button2_edge = (buttons & kButtonArm) && !(prev_buttons & kButtonArm);

  bool kill_switch_active = !(switches & kSwitchKill);
  bool alt_hold_enabled = (switches & kSwitchAltitudeHold);

  if (kill_switch_active) {
    if (armed) {
      playTone(500, 200);
    }
    disarmMotors();
  } else if (!armed && button2_edge && (latest_command.flags & kFlagRequestArm)) {
    if (imu_calibrated && esc_calibrated) {
      armed = true;
      playTone(1800, 150);
    } else {
      playTone(600, 300);
    }
  }

  if (button1_edge || (latest_command.flags & kFlagRequestImuCalibration)) {
    disarmMotors();
    runImuCalibration();
    attitude_filter.reset(0.0f, 0.0f);
  }

  if (!alt_hold_enabled && !armed && (latest_command.flags & kFlagRequestEscCalibration)) {
    runEscCalibration();
  }

  if (alt_hold_enabled && !(prev_switches & kSwitchAltitudeHold)) {
    altitude_hold_target_cm = altitude_cm;
  }

  prev_buttons = buttons;
  prev_switches = switches;

  status_payload.flags = 0;
  if (link_alive) status_payload.flags |= kStatusLinkAlive;
  if (imu_calibrated) status_payload.flags |= kStatusImuCalibrated;
  if (esc_calibrated) status_payload.flags |= kStatusEscCalibrated;
  if (armed) status_payload.flags |= kStatusArmed;
  if (alt_hold_enabled) status_payload.flags |= kStatusAltitudeHold;
  if (kill_switch_active) status_payload.flags |= kStatusKillActive;
  if (calibration_running) status_payload.flags |= kStatusCalibrationRunning;
  if (status_payload.last_error != kErrorNone) status_payload.flags |= kStatusSensorFault;
}

void mixAndDrive(float dt_s) {
  float throttle = constrain(static_cast<float>(latest_command.throttle_us), kPwmMin, kPwmMaxSafe);

  bool alt_hold = (latest_command.switches & kSwitchAltitudeHold);
  if (alt_hold) {
    float error_cm = altitude_hold_target_cm - altitude_cm;
    float correction = alt_pid.update(error_cm, constrain(dt_s, 0.01f, 0.05f));
    throttle = constrain(throttle + correction, kPwmMin, kPwmMaxSafe);
  }

  float target_roll = mapAxisToAngle(latest_command.roll_cmd);
  float target_pitch = mapAxisToAngle(latest_command.pitch_cmd);
  float target_yaw_rate = mapAxisToYawRate(latest_command.yaw_rate_cmd);

  float roll_correction = roll_pid.update(target_roll - attitude_filter.roll(), dt_s);
  float pitch_correction = pitch_pid.update(target_pitch - attitude_filter.pitch(), dt_s);
  float yaw_correction = yaw_pid.update(target_yaw_rate - attitude_filter.yaw_rate(), dt_s);

  uint16_t motor_commands[4];
  motor_commands[kMotorFL] = throttle + pitch_correction - roll_correction - yaw_correction;
  motor_commands[kMotorFR] = throttle + pitch_correction + roll_correction + yaw_correction;
  motor_commands[kMotorRR] = throttle - pitch_correction + roll_correction - yaw_correction;
  motor_commands[kMotorRL] = throttle - pitch_correction - roll_correction + yaw_correction;

  for (uint8_t i = 0; i < 4; ++i) {
    if (!armed) {
      motors[i].writeMicroseconds(kPwmMin);
    } else {
      motors[i].writeMicroseconds(constrain(motor_commands[i], kPwmMin, kPwmMaxSafe));
    }
  }
}

void updateLed() {
  uint32_t now = millis();
  uint16_t blink_period = armed ? 100 : (link_alive ? 400 : 900);
  if (now - last_led_toggle >= blink_period) {
    led_state = !led_state;
    digitalWrite(kLedPin, led_state);
    last_led_toggle = now;
  }
}

void pushStatus(uint32_t loop_duration_us) {
  status_payload.roll_deg_x10 = static_cast<int16_t>(attitude_filter.roll() * 10.0f);
  status_payload.pitch_deg_x10 = static_cast<int16_t>(attitude_filter.pitch() * 10.0f);
  status_payload.yaw_rate_dps = static_cast<int16_t>(attitude_filter.yaw_rate());
  status_payload.altitude_cm = static_cast<int16_t>(altitude_cm);
  status_payload.loop_time_us = loop_duration_us;
  status_payload.battery_mv = 0;

  radio.writeAckPayload(1, &status_payload, sizeof(status_payload));
}

}  // namespace

void setup() {
  pinMode(kLedPin, OUTPUT);
  pinMode(kBuzzerPin, OUTPUT);
  pinMode(kImuIntPin, INPUT);

  Serial.begin(115200);
  Wire.begin();
  last_command_micros = micros();

  setupRadio();
  attachMotors();

  imu.begin();
  baro.begin();

  roll_pid.configure(3.5f, 0.01f, 0.8f, -200.0f, 200.0f, 150.0f);
  pitch_pid.configure(3.5f, 0.01f, 0.8f, -200.0f, 200.0f, 150.0f);
  yaw_pid.configure(2.0f, 0.0f, 0.5f, -120.0f, 120.0f, 0.0f);
  alt_pid.configure(0.8f, 0.1f, 0.1f, -200.0f, 200.0f, 100.0f);

  runImuCalibration(1500);
  esc_calibrated = false;
  playTone(2200, 150);
}

void loop() {
  uint32_t loop_start = micros();
  float dt_s = (loop_start - last_loop_start) / 1e6f;
  last_loop_start = loop_start;

  updateRadioLink();
  processActions();
  updateSensors(dt_s);

  mixAndDrive(dt_s);
  updateLed();

  pushStatus(micros() - loop_start);
}
