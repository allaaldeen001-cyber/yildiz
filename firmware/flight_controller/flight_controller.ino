/*
 * Arduino Nano Quadcopter Flight Controller
 * -------------------------------------------------------------
 * - IMU: MPU6050 (I2C)
 * - Barometer: MS5611 (I2C)
 * - Radio link: nRF24L01+ (RF24 library)
 * - Outputs: 4x ESCs (Servo library), buzzer, status LED
 *
 * Features
 * --------
 * - Radio handshake + guided setup over Serial Monitor
 * - Button 1 command: calibrates IMU, MS5611, and ESC endpoints
 * - Button 2 command: smooth idle spool-up (motors spinning but not flying)
 * - Toggle switch: Kill (down) / Arm (up) gate with throttle-low check
 * - Complementary filter attitude estimation + PID stabilization loop
 * - Ack-payload telemetry back to the transmitter (attitude, altitude, status)
 *
 * Default Serial speed: 115200 bps
 */

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Servo.h>
#include <RF24.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MS5611.h>
#include <math.h>

// ------------------------- Hardware mapping -------------------------
constexpr uint8_t ESC_PINS[4] = {3, 5, 6, 9};   // FL, FR, RR, RL (X layout)
constexpr uint8_t BUZZER_PIN = 4;
constexpr uint8_t STATUS_LED_PIN = 13;
constexpr uint8_t NRF_CE_PIN = 7;
constexpr uint8_t NRF_CSN_PIN = 8;
constexpr uint8_t SPI_SS_PIN = 10;              // keep as output to stay SPI master

// ------------------------- Radio configuration ----------------------
constexpr uint8_t RADIO_PIPE = 1;
constexpr uint8_t RADIO_CHANNEL = 108;          // 2.508 GHz (avoid WiFi)
const byte RADIO_ADDRESS[6] = "CTRL1";          // 5-byte pipe label

constexpr uint8_t BTN_CALIBRATE = 0x01;
constexpr uint8_t BTN_SPOOL = 0x02;

// ------------------------- Control structures -----------------------
struct ControlPacket {
  uint16_t throttle;   // 1000 .. 2000 us
  uint16_t roll;       // 1000 .. 2000 us
  uint16_t pitch;      // 1000 .. 2000 us
  uint16_t yaw;        // 1000 .. 2000 us
  uint8_t armSwitch;   // 0 = kill, 1 = arm
  uint8_t buttons;     // bit0 = calibrate, bit1 = smooth spool
  uint8_t frameId;     // increments every packet for link-quality estimation
} __attribute__((packed));

struct TelemetryPacket {
  uint16_t throttle;
  int16_t rollDeciDeg;
  int16_t pitchDeciDeg;
  int16_t yawDeciDeg;
  int16_t altitudeCm;
  uint8_t linkQuality;
  uint8_t setupStep;
  uint8_t flags;       // bit0 = armed, bit1 = motors at idle
  uint8_t channel;
} __attribute__((packed));

struct CalibrationBlob {
  float gyroBias[3];
  float accelBias[3];
  float pressureRef; // baseline Pa for altitude calculation
  int16_t escMin;
  int16_t escMax;
  uint32_t crc;
} __attribute__((packed));

enum SetupStep : uint8_t {
  STEP_WAIT_LINK = 0,
  STEP_WAIT_KILL,
  STEP_WAIT_CALIBRATION,
  STEP_WAIT_ARM,
  STEP_WAIT_SPOOL,
  STEP_ACTIVE
};

struct PIDAxis {
  float kp;
  float ki;
  float kd;
  float integral;
  float prevError;
};

// ------------------------- Globals ----------------------------------
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Adafruit_MPU6050 imu;
MS5611 baro;
Servo esc[4];

CalibrationBlob cal = {{0, 0, 0}, {0, 0, 0}, 101325.0f, 1000, 2000, 0};
ControlPacket lastRx = {1000, 1500, 1500, 1500, 0, 0, 0};
TelemetryPacket telemetry = {};
SetupStep setupStep = STEP_WAIT_LINK;

PIDAxis pidRoll = {3.2f, 0.015f, 15.0f, 0.0f, 0.0f};
PIDAxis pidPitch = {3.2f, 0.015f, 15.0f, 0.0f, 0.0f};
PIDAxis pidYaw = {2.4f, 0.02f, 0.0f, 0.0f, 0.0f};

uint32_t lastRxMillis = 0;
uint32_t lastSerialMillis = 0;
uint32_t lastBaroMillis = 0;
uint32_t goodPackets = 0;
uint32_t lostPackets = 0;
uint8_t lastButtons = 0;
uint8_t nextFrameId = 0;
bool frameIdInitialized = false;
bool isArmed = false;
bool spoolRequested = false;
bool motorsIdling = false;
bool calibrationComplete = false;
bool calibrationInProgress = false;
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawDeg = 0.0f;
float altitudeMeters = 0.0f;
float gyroRates[3] = {0, 0, 0};
uint32_t lastImuMicros = 0;
float motorFloorUs = 1000.0f;

// ------------------------- Helpers ----------------------------------
uint32_t crc32For(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc ^ 0xFFFFFFFF;
}

void loadCalibration() {
  EEPROM.get(0, cal);
  uint32_t stored = cal.crc;
  cal.crc = 0;
  uint32_t computed = crc32For(reinterpret_cast<uint8_t *>(&cal), sizeof(cal));
  if (stored != computed || cal.escMin < 900 || cal.escMin > 1200 || cal.escMax < 1800 || cal.escMax > 2100) {
    cal = {{0, 0, 0}, {0, 0, 0}, 101325.0f, 1000, 2000, 0};
    Serial.println(F("[CAL] No valid calibration found. Using defaults."));
  } else {
    Serial.println(F("[CAL] Calibration restored from EEPROM."));
  }
}

void saveCalibration() {
  cal.crc = 0;
  uint32_t checksum = crc32For(reinterpret_cast<uint8_t *>(&cal), sizeof(cal));
  cal.crc = checksum;
  EEPROM.put(0, cal);
  Serial.println(F("[CAL] Saved to EEPROM."));
}

template <typename T>
T clampValue(T value, T minVal, T maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

float mapPwmToRange(uint16_t pwm, float minOut, float maxOut) {
  pwm = clampValue<uint16_t>(pwm, 1000, 2000);
  float ratio = (pwm - 1000) / 1000.0f;
  return minOut + ratio * (maxOut - minOut);
}

void beepPattern(uint8_t repeats, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < repeats; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(onMs);
    digitalWrite(BUZZER_PIN, LOW);
    delay(offMs);
  }
}

void writeAllMotors(int pulseUs) {
  pulseUs = clampValue(pulseUs, cal.escMin, cal.escMax);
  for (uint8_t i = 0; i < 4; i++) {
    esc[i].writeMicroseconds(pulseUs);
  }
}

bool throttleIsLow() {
  return lastRx.throttle <= (uint16_t)(cal.escMin + 20);
}

void disarmMotors(const __FlashStringHelper *reason) {
  if (isArmed) {
    Serial.print(F("[STATE] Disarming: "));
    Serial.println(reason);
  }
  isArmed = false;
  spoolRequested = false;
  motorsIdling = false;
  motorFloorUs = cal.escMin;
  writeAllMotors(cal.escMin);
}

void setSetupStep(SetupStep next, const __FlashStringHelper *prompt) {
  if (setupStep == next) {
    return;
  }
  setupStep = next;
  Serial.println();
  Serial.print(F("[GUIDE] "));
  Serial.println(prompt);
}

void requestSetupPrompts() {
  Serial.println(F("\n=== Guided bring-up ==="));
  Serial.println(F("1. Wait for RC + FC NRF link (buzzer chirp when ready)."));
  Serial.println(F("2. Toggle switch DOWN (kill) so we can confirm safety."));
  Serial.println(F("3. Press Button 1 on RC to run IMU + barometer + ESC calibration."));
  Serial.println(F("4. Flip toggle switch UP to arm (throttle must stay at bottom)."));
  Serial.println(F("5. Press Button 2 for a smooth idle motor spin."));
  Serial.println(F("6. When told, increase throttle to lift off."));
  Serial.println();
}

void startCalibrationSequence();
void updateTelemetry();
void updateAttitude(float dt);
void updateFlightControl(float dt);

// ------------------------- Setup ------------------------------------
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(SPI_SS_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(SPI_SS_PIN, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.println(F("\nArduino Nano Quadcopter Flight Controller"));
  Serial.println(F("--------------------------------------------------"));

  Wire.begin();
  Wire.setClock(400000);

  if (!imu.begin()) {
    Serial.println(F("[ERR] MPU6050 not detected. Check wiring."));
    while (true) {
      beepPattern(1, 100, 100);
    }
  }
  imu.setAccelerometerRange(Adafruit_MPU6050::ACCEL_RANGE_4_G);
  imu.setGyroRange(Adafruit_MPU6050::GYRO_RANGE_500_DEG);
  imu.setFilterBandwidth(Adafruit_MPU6050::FILTER_BAND_21_HZ);

  if (!baro.begin()) {
    Serial.println(F("[ERR] MS5611 not detected. Altitude data disabled."));
  } else {
    baro.setOversampling(OSR_ULTRA_HIGH);
  }

  loadCalibration();

  for (uint8_t i = 0; i < 4; i++) {
    esc[i].attach(ESC_PINS[i], 1000, 2000);
    esc[i].writeMicroseconds(cal.escMin);
  }

  if (!radio.begin()) {
    Serial.println(F("[ERR] nRF24L01+ init failed."));
    while (true) {
      beepPattern(2, 80, 80);
      delay(500);
    }
  }
  radio.setChannel(RADIO_CHANNEL);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(2, 5);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.openReadingPipe(RADIO_PIPE, RADIO_ADDRESS);
  radio.startListening();
  radio.writeAckPayload(RADIO_PIPE, &telemetry, sizeof(telemetry));

  lastImuMicros = micros();
  requestSetupPrompts();
  Serial.println(F("Waiting for RC link..."));
  beepPattern(1, 120, 120);
}

// ------------------------- Loop -------------------------------------
void loop() {
  uint32_t loopStartMicros = micros();
  float dt = (loopStartMicros - lastImuMicros) * 1e-6f;
  lastImuMicros = loopStartMicros;
  if (dt < 0.0001f || dt > 0.02f) {
    dt = 0.002f;
  }

  // Process inbound radio packets and guided workflow
  if (radio.available()) {
    ControlPacket rx;
    radio.read(&rx, sizeof(rx));
    lastRx = rx;
    lastRxMillis = millis();

    if (!frameIdInitialized) {
      frameIdInitialized = true;
      nextFrameId = rx.frameId + 1;
    } else {
      uint8_t gap = rx.frameId - nextFrameId;
      lostPackets += gap;
      nextFrameId = rx.frameId + 1;
    }
    goodPackets++;

    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));

    if (setupStep == STEP_WAIT_LINK) {
      setSetupStep(STEP_WAIT_KILL, F("RC link OK. Move toggle DOWN (kill) and keep throttle low."));
      beepPattern(2, 60, 80);
    }

    if (setupStep == STEP_WAIT_KILL) {
      if (rx.armSwitch == 0) {
        setSetupStep(STEP_WAIT_CALIBRATION, F("Kill confirmed. Press Button 1 for full calibration."));
      }
    }

    if (setupStep == STEP_WAIT_CALIBRATION && calibrationComplete) {
      setSetupStep(STEP_WAIT_ARM, F("Calibration stored. Flip toggle UP to arm (throttle still low)."));
    }

    if (rx.armSwitch == 0 && isArmed) {
      disarmMotors(F("Kill switch set to SAFE."));
      setSetupStep(STEP_WAIT_ARM, F("Kill active. Flip ARM switch UP once you are ready."));
    }

    if (setupStep >= STEP_WAIT_ARM && rx.armSwitch == 1 && throttleIsLow() && calibrationComplete && !isArmed) {
      isArmed = true;
      Serial.println(F("[STATE] Armed. Awaiting smooth start command."));
      setSetupStep(STEP_WAIT_SPOOL, F("Armed. Press Button 2 for smooth idle spin."));
      beepPattern(3, 50, 50);
    }

    uint8_t risingEdges = (rx.buttons) & (~lastButtons);

    if ((risingEdges & BTN_CALIBRATE) && !calibrationInProgress) {
      startCalibrationSequence();
      if (calibrationComplete) {
        setSetupStep(STEP_WAIT_ARM, F("Calibration done. Flip toggle UP to arm."));
      }
    }

    if ((risingEdges & BTN_SPOOL) && isArmed) {
      spoolRequested = true;
      Serial.println(F("[STATE] Smooth spool requested."));
      if (setupStep == STEP_WAIT_SPOOL) {
        setSetupStep(STEP_ACTIVE, F("Motors spinning at idle. Drone ready for throttle inputs."));
      }
    }

    lastButtons = rx.buttons;
  }

  if (millis() - lastRxMillis > 250) {
    if (isArmed) {
      disarmMotors(F("Radio timeout"));
      setSetupStep(STEP_WAIT_LINK, F("Radio lost. Restore link, kill switch down."));
      calibrationComplete = true; // keep calibration but restart workflow
    }
  }

  updateAttitude(dt);
  updateFlightControl(dt);
  updateTelemetry();

  if (millis() - lastSerialMillis > 250) {
    lastSerialMillis = millis();
    Serial.print(F("[TEL] Throttle:"));
    Serial.print(lastRx.throttle);
    Serial.print(F(" Roll:"));
    Serial.print(rollDeg, 1);
    Serial.print(F(" Pitch:"));
    Serial.print(pitchDeg, 1);
    Serial.print(F(" Yaw:"));
    Serial.print(yawDeg, 1);
    Serial.print(F(" Alt(m):"));
    Serial.print(altitudeMeters, 2);
    Serial.print(F(" Link:%"));
    Serial.print(telemetry.linkQuality);
    Serial.print(F(" Step:"));
    Serial.println(static_cast<uint8_t>(setupStep));
  }
}

// ------------------------- Flight control ---------------------------
float runPid(PIDAxis &pid, float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;
  pid.integral += error * pid.ki * dt;
  pid.integral = clampValue(pid.integral, -200.0f, 200.0f);
  float derivative = (error - pid.prevError) / dt;
  float output = pid.kp * error + pid.integral + pid.kd * derivative;
  pid.prevError = error;
  return output;
}

void updateAttitude(float dt) {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp);

  float ax = accel.acceleration.x - cal.accelBias[0];
  float ay = accel.acceleration.y - cal.accelBias[1];
  float az = accel.acceleration.z - cal.accelBias[2];
  float gx = (gyro.gyro.x - cal.gyroBias[0]) * RAD_TO_DEG; // deg/s
  float gy = (gyro.gyro.y - cal.gyroBias[1]) * RAD_TO_DEG;
  float gz = (gyro.gyro.z - cal.gyroBias[2]) * RAD_TO_DEG;

  gyroRates[0] = gx;
  gyroRates[1] = gy;
  gyroRates[2] = gz;

  float accelRoll = atan2f(ay, az) * RAD_TO_DEG;
  float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  rollDeg = 0.98f * (rollDeg + gx * dt) + 0.02f * accelRoll;
  pitchDeg = 0.98f * (pitchDeg + gy * dt) + 0.02f * accelPitch;
  yawDeg += gz * dt;

  if (millis() - lastBaroMillis > 25) {
    lastBaroMillis = millis();
    if (baro.read()) {
      float pressure = baro.getPressure();
      if (pressure > 1000) {
        altitudeMeters = 44330.0f * (1.0f - powf(pressure / cal.pressureRef, 0.1903f));
      }
    }
  }
}

void updateFlightControl(float dt) {
  if (!isArmed) {
    writeAllMotors(cal.escMin);
    return;
  }

  if (!spoolRequested) {
    motorFloorUs = cal.escMin;
    motorsIdling = false;
  } else {
    float targetIdle = cal.escMin + 120;
    motorFloorUs += 1.5f;
    if (motorFloorUs > targetIdle) {
      motorFloorUs = targetIdle;
      motorsIdling = true;
    }
  }

  float throttleUs = mapPwmToRange(lastRx.throttle, cal.escMin, cal.escMax);
  throttleUs = max(throttleUs, motorFloorUs);

  float rollSet = mapPwmToRange(lastRx.roll, -25.0f, 25.0f);
  float pitchSet = mapPwmToRange(lastRx.pitch, -25.0f, 25.0f);
  float yawRateSet = mapPwmToRange(lastRx.yaw, -180.0f, 180.0f);

  float rollTerm = runPid(pidRoll, rollSet, rollDeg, dt);
  float pitchTerm = runPid(pidPitch, pitchSet, pitchDeg, dt);
  float yawTerm = runPid(pidYaw, yawRateSet, gyroRates[2], dt);

  float motorOutputs[4];
  motorOutputs[0] = throttleUs + pitchTerm + rollTerm - yawTerm; // Front Left
  motorOutputs[1] = throttleUs + pitchTerm - rollTerm + yawTerm; // Front Right
  motorOutputs[2] = throttleUs - pitchTerm - rollTerm - yawTerm; // Rear Right
  motorOutputs[3] = throttleUs - pitchTerm + rollTerm + yawTerm; // Rear Left

  for (uint8_t i = 0; i < 4; i++) {
    int pulse = static_cast<int>(clampValue(motorOutputs[i], (float)cal.escMin, (float)cal.escMax));
    esc[i].writeMicroseconds(pulse);
  }
}

void updateTelemetry() {
  uint32_t totalPackets = goodPackets + lostPackets;
  uint8_t quality = totalPackets == 0 ? 0 : (uint8_t)clampValue((int)((goodPackets * 100UL) / totalPackets), 0, 100);
  telemetry.throttle = lastRx.throttle;
  telemetry.rollDeciDeg = (int16_t)(rollDeg * 10);
  telemetry.pitchDeciDeg = (int16_t)(pitchDeg * 10);
  telemetry.yawDeciDeg = (int16_t)(yawDeg * 10);
  telemetry.altitudeCm = (int16_t)(altitudeMeters * 100);
  telemetry.linkQuality = quality;
  telemetry.setupStep = static_cast<uint8_t>(setupStep);
  telemetry.flags = (isArmed ? 0x01 : 0x00) | (motorsIdling ? 0x02 : 0x00);
  telemetry.channel = RADIO_CHANNEL;

  radio.writeAckPayload(RADIO_PIPE, &telemetry, sizeof(telemetry));
}

// ------------------------- Calibration -------------------------------
void sampleIMU(float *gyroBiasOut, float *accelBiasOut) {
  const uint16_t samples = 2000;
  float gyroSum[3] = {0, 0, 0};
  float accelSum[3] = {0, 0, 0};
  Serial.print(F("[CAL] Sampling IMU ("));
  Serial.print(samples);
  Serial.println(F(" samples)..."));
  for (uint16_t i = 0; i < samples; i++) {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    imu.getEvent(&accel, &gyro, &temp);
    gyroSum[0] += gyro.gyro.x;
    gyroSum[1] += gyro.gyro.y;
    gyroSum[2] += gyro.gyro.z;
    accelSum[0] += accel.acceleration.x;
    accelSum[1] += accel.acceleration.y;
    accelSum[2] += accel.acceleration.z - 9.80665f;
    delay(2);
  }
  for (uint8_t axis = 0; axis < 3; axis++) {
    gyroBiasOut[axis] = gyroSum[axis] / samples;
    accelBiasOut[axis] = accelSum[axis] / samples;
  }
  Serial.println(F("[CAL] IMU bias stored."));
}

void calibrateBarometer(float *pressureRefOut) {
  if (!baro.begin()) {
    Serial.println(F("[CAL] Skipping barometer (not detected)."));
    return;
  }
  const uint16_t samples = 200;
  float pressureSum = 0;
  Serial.println(F("[CAL] Sampling MS5611..."));
  for (uint16_t i = 0; i < samples; i++) {
    if (baro.read()) {
      pressureSum += baro.getPressure();
    }
    delay(10);
  }
  *pressureRefOut = pressureSum / samples;
  Serial.print(F("[CAL] Sea-level reference (Pa): "));
  Serial.println(*pressureRefOut);
}

void calibrateEscs(int16_t *escMinOut, int16_t *escMaxOut) {
  Serial.println(F("[CAL] ESC calibration starting. REMOVE PROPS!"));
  Serial.println(F("[CAL] Sending max throttle..."));
  writeAllMotors(2000);
  delay(3000);
  Serial.println(F("[CAL] Sending min throttle..."));
  writeAllMotors(1000);
  delay(3000);
  *escMinOut = 1000;
  *escMaxOut = 2000;
  Serial.println(F("[CAL] ESC endpoints stored (1000/2000)."));
}

void startCalibrationSequence() {
  if (isArmed) {
    Serial.println(F("[CAL] Cannot calibrate while armed."));
    return;
  }
  calibrationInProgress = true;
  calibrationComplete = false;
  Serial.println(F("\n[CAL] Starting full calibration. Keep drone still."));
  beepPattern(3, 120, 80);

  sampleIMU(cal.gyroBias, cal.accelBias);
  calibrateBarometer(&cal.pressureRef);
  calibrateEscs(&cal.escMin, &cal.escMax);

  saveCalibration();
  calibrationComplete = true;
  calibrationInProgress = false;
  Serial.println(F("[CAL] All sensors + ESC calibrated."));
  beepPattern(2, 200, 200);
}
