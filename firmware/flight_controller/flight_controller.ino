#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <MPU6050.h>
#include <math.h>

#include "../shared/DroneLink.h"

using namespace DroneLink;

// -----------------------------------------------------------------------------
// Pin assignments (Arduino Nano)
// -----------------------------------------------------------------------------
constexpr uint8_t PIN_NRF_CE     = 4;   // CE -> D4
constexpr uint8_t PIN_NRF_CSN    = 10;  // CSN -> D10
constexpr uint8_t PIN_STATUS_LED = 7;   // Status LED
constexpr uint8_t PIN_BUZZER     = 8;   // Buzzer output
constexpr uint8_t PIN_MPU_INT    = 2;   // MPU6050 interrupt
constexpr uint8_t PIN_ESC_FL     = 3;   // Front-left motor
constexpr uint8_t PIN_ESC_FR     = 5;   // Front-right motor
constexpr uint8_t PIN_ESC_RR     = 6;   // Rear-right motor
constexpr uint8_t PIN_ESC_RL     = 9;   // Rear-left motor

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------
constexpr uint16_t MIN_THROTTLE_US   = 1000;
constexpr uint16_t MAX_THROTTLE_US   = 2000;
constexpr uint16_t SAFE_THROTTLE_US  = MIN_THROTTLE_US;
constexpr uint16_t THROTTLE_LIMIT_US = MIN_THROTTLE_US + (uint16_t)((MAX_THROTTLE_US - MIN_THROTTLE_US) * 0.65f);
constexpr float    MAX_ANGLE_DEG     = 30.0f;   // Max tilt target
constexpr float    MAX_YAW_RATE_DPS  = 150.0f;  // deg/s
constexpr uint32_t LINK_LOSS_MS      = 350;     // Failsafe timeout
constexpr uint32_t COMMAND_STALE_MS  = 2000;
constexpr uint32_t LED_FAST_MS       = 80;
constexpr uint32_t LED_MED_MS        = 250;
constexpr uint32_t LED_FAIL_MS       = 800;
constexpr uint16_t BUZZ_CAL_OK_FREQ  = 1700;
constexpr uint16_t BUZZ_CAL_FAIL_FREQ= 400;
constexpr uint16_t BUZZ_ESC_FREQ     = 2200;
constexpr float    COMPLEMENT_ALPHA  = 0.98f;

// PID gains (tuned conservatively for stability)
constexpr float KP_ROLL  = 3.5f;
constexpr float KD_ROLL  = 0.08f;
constexpr float KP_PITCH = 3.5f;
constexpr float KD_PITCH = 0.08f;
constexpr float KP_YAW   = 2.0f;
constexpr float KD_YAW   = 0.02f;

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------
RF24   radio(PIN_NRF_CE, PIN_NRF_CSN);
MPU6050 imu;
Servo  escFL, escFR, escRR, escRL;

ControlPacket  lastControls{};
TelemetryPacket telemetry{};

float gyroOffsets[3]  = {0.0f};
float accelOffsets[3] = {0.0f};
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawRateDps = 0.0f;
float prevRollError = 0.0f;
float prevPitchError = 0.0f;
float prevYawError = 0.0f;

bool imuReady = false;
bool escReady = false;
bool motorsArmed = false;
bool failsafe = true;
bool altHoldRequested = false;
bool altHoldActive = false;
float altHoldThrottle = MIN_THROTTLE_US;
FlightState flightState = FlightState::DISARMED;

uint8_t lastControlFlags = 0;
uint32_t lastLoopMicros = 0;
uint32_t lastCommandMillis = 0;
uint32_t lastLedToggle = 0;
bool ledState = false;

BuzzerCue pendingCue = BuzzerCue::NONE;
BuzzerCue telemCue = BuzzerCue::NONE;

// -----------------------------------------------------------------------------
// Utility helpers
// -----------------------------------------------------------------------------
template <typename T>
T clamp(T value, T minVal, T maxVal) {
  return (value < minVal) ? minVal : (value > maxVal ? maxVal : value);
}

float mapInput(int16_t value) {
  // Normalize -500..500 to -1..1
  return clamp(value / 500.0f, -1.0f, 1.0f);
}

uint16_t limitThrottle(uint16_t raw) {
  return clamp<uint16_t>(raw, MIN_THROTTLE_US, THROTTLE_LIMIT_US);
}

void buzzTone(uint16_t frequency, uint16_t durationMs) {
  tone(PIN_BUZZER, frequency, durationMs);
  delay(durationMs + 10);
  noTone(PIN_BUZZER);
}

void playCue(BuzzerCue cue) {
  switch (cue) {
    case BuzzerCue::CALIBRATION_OK:
      buzzTone(BUZZ_CAL_OK_FREQ, 200);
      delay(60);
      buzzTone(BUZZ_CAL_OK_FREQ + 300, 220);
      break;
    case BuzzerCue::CALIBRATION_FAIL:
      buzzTone(BUZZ_CAL_FAIL_FREQ, 7000);
      break;
    case BuzzerCue::ESC_SEQUENCE_START:
      buzzTone(BUZZ_ESC_FREQ, 150);
      delay(70);
      buzzTone(BUZZ_ESC_FREQ - 300, 150);
      break;
    case BuzzerCue::ESC_SEQUENCE_DONE:
      buzzTone(BUZZ_ESC_FREQ + 200, 350);
      break;
    case BuzzerCue::ARMING_CHANGE:
      buzzTone(1400, 120);
      break;
    default:
      break;
  }
}

void scheduleCue(BuzzerCue cue) {
  pendingCue = cue;
  telemCue = cue;
}

void writeAllMotors(uint16_t pulse) {
  escFL.writeMicroseconds(pulse);
  escFR.writeMicroseconds(pulse);
  escRR.writeMicroseconds(pulse);
  escRL.writeMicroseconds(pulse);
}

void setFailsafe(bool enabled) {
  if (enabled && !failsafe) {
    scheduleCue(BuzzerCue::ARMING_CHANGE);
  }
  failsafe = enabled;
  if (failsafe) {
    motorsArmed = false;
    flightState = FlightState::DISARMED;
    writeAllMotors(SAFE_THROTTLE_US);
  }
}

void updateStatusLed() {
  uint32_t now = millis();
  uint32_t interval = failsafe ? LED_FAIL_MS : (motorsArmed ? LED_FAST_MS : LED_MED_MS);
  if (now - lastLedToggle >= interval) {
    ledState = !ledState;
    digitalWrite(PIN_STATUS_LED, ledState);
    lastLedToggle = now;
  }
}

// -----------------------------------------------------------------------------
// IMU management
// -----------------------------------------------------------------------------
bool initImu() {
  Wire.begin();
  Wire.setClock(400000);
  imu.initialize();
  if (!imu.testConnection()) {
    Serial.println(F("MPU6050 connection failed"));
    scheduleCue(BuzzerCue::CALIBRATION_FAIL);
    return false;
  }
  pinMode(PIN_MPU_INT, INPUT);
  return true;
}

void performImuCalibration() {
  if (motorsArmed) {
    Serial.println(F("Cannot calibrate while armed"));
    return;
  }

  flightState = FlightState::IMU_CALIBRATING;
  const int samples = 2000;
  long gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
  Serial.println(F("Starting IMU calibration..."));

  for (int i = 0; i < samples; ++i) {
    int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
    imu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);
    gx += rawGx;
    gy += rawGy;
    gz += rawGz;
    ax += rawAx;
    ay += rawAy;
    az += (rawAz - 16384);  // expect 1g on Z
    delay(3);
  }

  gyroOffsets[0] = gx / (float)samples;
  gyroOffsets[1] = gy / (float)samples;
  gyroOffsets[2] = gz / (float)samples;
  accelOffsets[0] = ax / (float)samples;
  accelOffsets[1] = ay / (float)samples;
  accelOffsets[2] = az / (float)samples;

  bool success = (fabs(gyroOffsets[0]) < 50 && fabs(gyroOffsets[1]) < 50 && fabs(gyroOffsets[2]) < 50);
  imuReady = success;
  flightState = FlightState::DISARMED;
  scheduleCue(success ? BuzzerCue::CALIBRATION_OK : BuzzerCue::CALIBRATION_FAIL);
  Serial.println(success ? F("IMU calibration OK") : F("IMU calibration failed"));
}

void updateImu(float dt) {
  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  imu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

  float gx = (rawGx - gyroOffsets[0]) / 131.0f;
  float gy = (rawGy - gyroOffsets[1]) / 131.0f;
  float gz = (rawGz - gyroOffsets[2]) / 131.0f;

  float ax = (rawAx - accelOffsets[0]) / 16384.0f;
  float ay = (rawAy - accelOffsets[1]) / 16384.0f;
  float az = (rawAz - accelOffsets[2]) / 16384.0f;

  float accelRoll = atan2f(ay, az) * RAD_TO_DEG;
  float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  rollDeg = COMPLEMENT_ALPHA * (rollDeg + gx * dt) + (1.0f - COMPLEMENT_ALPHA) * accelRoll;
  pitchDeg = COMPLEMENT_ALPHA * (pitchDeg + gy * dt) + (1.0f - COMPLEMENT_ALPHA) * accelPitch;
  yawRateDps = gz;
}

// -----------------------------------------------------------------------------
// ESC calibration routine
// -----------------------------------------------------------------------------
void runEscCalibration() {
  if (motorsArmed) {
    Serial.println(F("Cannot run ESC calibration while armed"));
    return;
  }
  flightState = FlightState::ESC_CALIBRATING;
  escReady = false;
  scheduleCue(BuzzerCue::ESC_SEQUENCE_START);

  const uint16_t rampTop = THROTTLE_LIMIT_US;
  const uint16_t step = 25;

  auto ramp = [&](Servo &esc) {
    for (uint16_t pulse = MIN_THROTTLE_US; pulse <= rampTop; pulse += step) {
      esc.writeMicroseconds(pulse);
      delay(12);
    }
    for (int pulse = rampTop; pulse >= MIN_THROTTLE_US; pulse -= step) {
      esc.writeMicroseconds(pulse);
      delay(12);
    }
  };

  ramp(escFL);
  ramp(escFR);
  ramp(escRR);
  ramp(escRL);

  writeAllMotors(SAFE_THROTTLE_US);
  escReady = true;
  flightState = FlightState::DISARMED;
  scheduleCue(BuzzerCue::ESC_SEQUENCE_DONE);
  Serial.println(F("ESC calibration done"));
}

// -----------------------------------------------------------------------------
// Motor mix
// -----------------------------------------------------------------------------
void writeMotorMix(float throttleUs, float rollTerm, float pitchTerm, float yawTerm) {
  uint16_t fl = clamp<uint16_t>(throttleUs + pitchTerm + rollTerm - yawTerm, MIN_THROTTLE_US, THROTTLE_LIMIT_US);
  uint16_t fr = clamp<uint16_t>(throttleUs + pitchTerm - rollTerm + yawTerm, MIN_THROTTLE_US, THROTTLE_LIMIT_US);
  uint16_t rr = clamp<uint16_t>(throttleUs - pitchTerm - rollTerm - yawTerm, MIN_THROTTLE_US, THROTTLE_LIMIT_US);
  uint16_t rl = clamp<uint16_t>(throttleUs - pitchTerm + rollTerm + yawTerm, MIN_THROTTLE_US, THROTTLE_LIMIT_US);

  escFL.writeMicroseconds(fl);
  escFR.writeMicroseconds(fr);
  escRR.writeMicroseconds(rr);
  escRL.writeMicroseconds(rl);
}

// -----------------------------------------------------------------------------
// Radio
// -----------------------------------------------------------------------------
bool initRadio() {
  if (!radio.begin()) {
    Serial.println(F("Radio init failed"));
    return false;
  }
  radio.setChannel(kRadioChannel);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(2, 5);
  radio.openReadingPipe(0, kRadioAddress);
  radio.startListening();
  radio.writeAckPayload(0, &telemetry, sizeof(telemetry));
  return true;
}

void pollRadio() {
  bool received = false;
  while (radio.available()) {
    ControlPacket incoming;
    radio.read(&incoming, sizeof(incoming));
    lastControls = incoming;
    received = true;
  }

  if (received) {
    lastCommandMillis = millis();
    setFailsafe(false);
  }
}

// -----------------------------------------------------------------------------
// Control logic
// -----------------------------------------------------------------------------
void handleControlRequests() {
  const uint8_t flags = lastControls.flags;
  bool calibPressed = (flags & CTRL_FLAG_CALIBRATE) && !(lastControlFlags & CTRL_FLAG_CALIBRATE);
  bool armPressed = (flags & CTRL_FLAG_ARM_BUTTON) && !(lastControlFlags & CTRL_FLAG_ARM_BUTTON);
  bool escCalibPressed = (flags & CTRL_FLAG_ESC_CALIB) && !(lastControlFlags & CTRL_FLAG_ESC_CALIB);
  bool masterArm = flags & CTRL_FLAG_ARM_SWITCH;
  bool altHold = flags & CTRL_FLAG_ALT_HOLD;

  if (!masterArm) {
    if (motorsArmed) {
      scheduleCue(BuzzerCue::ARMING_CHANGE);
    }
    motorsArmed = false;
    flightState = FlightState::DISARMED;
  }

  if (calibPressed) {
    performImuCalibration();
  }

  if (escCalibPressed && !motorsArmed) {
    runEscCalibration();
  }

  if (armPressed && masterArm && imuReady && escReady && !failsafe) {
    motorsArmed = !motorsArmed;
    flightState = motorsArmed ? FlightState::ARMED_STABLE : FlightState::DISARMED;
    scheduleCue(BuzzerCue::ARMING_CHANGE);
  }

  altHoldRequested = altHold;
  lastControlFlags = flags;
}

void applyControl(float dt) {
  if (!motorsArmed) {
    writeAllMotors(SAFE_THROTTLE_US);
    altHoldActive = false;
    return;
  }

  uint16_t throttle = limitThrottle(lastControls.throttle);
  if (altHoldRequested && escReady) {
    if (!altHoldActive) {
      altHoldThrottle = throttle;
      altHoldActive = true;
    }
    float diff = throttle - altHoldThrottle;
    altHoldThrottle += diff * 0.05f;  // gently move towards requested throttle
    throttle = (uint16_t)altHoldThrottle;
  } else {
    altHoldActive = false;
    altHoldThrottle = throttle;
  }

  float desiredRoll = mapInput(lastControls.roll) * MAX_ANGLE_DEG;
  float desiredPitch = mapInput(lastControls.pitch) * MAX_ANGLE_DEG;
  float desiredYawRate = mapInput(lastControls.yaw) * MAX_YAW_RATE_DPS;

  float rollError = desiredRoll - rollDeg;
  float pitchError = desiredPitch - pitchDeg;
  float yawError = desiredYawRate - yawRateDps;

  float rollRate = (rollError - prevRollError) / dt;
  float pitchRate = (pitchError - prevPitchError) / dt;
  float yawRate = (yawError - prevYawError) / dt;

  prevRollError = rollError;
  prevPitchError = pitchError;
  prevYawError = yawError;

  float rollTerm = rollError * KP_ROLL + rollRate * KD_ROLL;
  float pitchTerm = pitchError * KP_PITCH + pitchRate * KD_PITCH;
  float yawTerm = yawError * KP_YAW + yawRate * KD_YAW;

  writeMotorMix(throttle, rollTerm, pitchTerm, yawTerm);
}

// -----------------------------------------------------------------------------
// Telemetry
// -----------------------------------------------------------------------------
void publishTelemetry() {
  telemetry.frame++;
  telemetry.roll = (int16_t)(rollDeg * 100.0f);
  telemetry.pitch = (int16_t)(pitchDeg * 100.0f);
  telemetry.yawRate = (int16_t)(yawRateDps * 100.0f);
  telemetry.systemFlags = 0;
  if (!failsafe) telemetry.systemFlags |= TLM_FLAG_LINK_OK;
  if (imuReady) telemetry.systemFlags |= TLM_FLAG_IMU_READY;
  if (escReady) telemetry.systemFlags |= TLM_FLAG_ESC_READY;
  if (motorsArmed) telemetry.systemFlags |= TLM_FLAG_MOTORS_ARMED;
  if (failsafe) telemetry.systemFlags |= TLM_FLAG_FAILSAFE;
  if (altHoldRequested && escReady) telemetry.systemFlags |= TLM_FLAG_ALT_HOLD;
  telemetry.flightState = static_cast<uint8_t>(flightState);
  telemetry.lastCommandMs = clamp<uint16_t>(millis() - lastCommandMillis, 0, 65535);
  telemetry.buzzerCue = static_cast<uint8_t>(telemCue);
  telemetry.reserved = 0;

  radio.writeAckPayload(0, &telemetry, sizeof(telemetry));
  telemCue = BuzzerCue::NONE;
}

// -----------------------------------------------------------------------------
// ESC + IO init
// -----------------------------------------------------------------------------
void initEscs() {
  escFL.attach(PIN_ESC_FL);
  escFR.attach(PIN_ESC_FR);
  escRR.attach(PIN_ESC_RR);
  escRL.attach(PIN_ESC_RL);
  writeAllMotors(SAFE_THROTTLE_US);
}

void indicateStartup() {
  digitalWrite(PIN_STATUS_LED, HIGH);
  buzzTone(1200, 120);
  delay(50);
  buzzTone(1500, 120);
  digitalWrite(PIN_STATUS_LED, LOW);
}

// -----------------------------------------------------------------------------
// Arduino lifecycle
// -----------------------------------------------------------------------------
void setup() {
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
  noTone(PIN_BUZZER);

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("Flight Controller booting..."));

  initImu();
  initEscs();
  initRadio();
  indicateStartup();

  lastControls.throttle = MIN_THROTTLE_US;
  lastControls.roll = 0;
  lastControls.pitch = 0;
  lastControls.yaw = 0;
  lastControls.flags = 0;
  lastControls.sequence = 0;
  lastCommandMillis = millis();

  lastLoopMicros = micros();
}

void loop() {
  uint32_t nowMicros = micros();
  float dt = (nowMicros - lastLoopMicros) / 1e6f;
  if (dt <= 0.0f || dt > 0.1f) {
    dt = 0.002f;  // fallback
  }
  lastLoopMicros = nowMicros;

  pollRadio();
  updateImu(dt);

  if (millis() - lastCommandMillis > LINK_LOSS_MS) {
    setFailsafe(true);
  }

  handleControlRequests();
  applyControl(dt);
  updateStatusLed();

  if (pendingCue != BuzzerCue::NONE) {
    BuzzerCue cue = pendingCue;
    pendingCue = BuzzerCue::NONE;
    playCue(cue);
  }

  publishTelemetry();
}
