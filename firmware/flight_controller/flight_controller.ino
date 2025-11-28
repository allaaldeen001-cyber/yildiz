/*
 * Build & Calibrate Arduino Quadcopter Drone - Flight Controller
 * --------------------------------------------------------------
 * Hardware: Arduino Nano, MPU6050, MS5611, NRF24L01+, 4x ESC, buzzer, status LED.
 * Radio protocol: point-to-point with ACK payloads. Matching transmitter sketch
 *  is located at firmware/radio_controller/radio_controller.ino
 *
 * The sketch provides:
 *  - IMU + barometer calibration (Button 1 on RC)
 *  - ESC calibration and smooth spool-up (Button 2 on RC)
 *  - Serial monitor guided bring-up procedure
 *  - Complementary filter for attitude estimate
 *  - PID stabilization loop stub (replace gains with tuned values)
 *
 * Libraries required (install via Arduino Library Manager):
 *  - RF24 by TMRh20
 *  - Servo (built-in)
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <EEPROM.h>
#include <math.h>

// ---------------------------- Pin Definition -----------------------------
constexpr uint8_t PIN_ESC_1 = 3;
constexpr uint8_t PIN_ESC_2 = 5;
constexpr uint8_t PIN_ESC_3 = 6;
constexpr uint8_t PIN_ESC_4 = 9;
constexpr uint8_t PIN_BUZZER = 10;
constexpr uint8_t PIN_STATUS_LED = 12;
constexpr uint8_t PIN_GUIDE_LED = 4;

constexpr uint8_t PIN_NRF_CE = 7;
constexpr uint8_t PIN_NRF_CSN = 8;

constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint8_t MS5611_ADDR = 0x77;

constexpr uint16_t RADIO_CHANNEL = 90;
const byte RADIO_PIPE[6] = "FCN01";

constexpr float RAD_TO_DEG = 57.2957795f;
constexpr float DEG_TO_RAD = 0.017453292f;

constexpr uint16_t ESC_MIN_US = 1000;
constexpr uint16_t ESC_MAX_US = 2000;

constexpr float COMPLEMENTARY_ALPHA = 0.98f;

enum GuideStep : uint8_t {
  GUIDE_WAIT_LINK = 0,
  GUIDE_REQUIRE_KILL,
  GUIDE_REQUIRE_CAL,
  GUIDE_REQUIRE_ARM,
  GUIDE_REQUIRE_SPOOL,
  GUIDE_READY,
  GUIDE_FLYING
};

struct RcPacket {
  uint16_t throttleUs;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  uint8_t armSwitch;   // 0 = kill, 1 = arm
  uint8_t buttonCal;   // button 1
  uint8_t buttonSpin;  // button 2
  uint8_t channel;
  uint16_t frameCounter;
};

struct TelemetryPacket {
  uint16_t throttleUs;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  float altitudeCm;
  uint8_t guideStep;
  uint8_t linkQuality;
};

struct CalibrationBlob {
  float gyroBias[3];
  float accelBias[3];
  float pressureOffset;
  uint16_t escMinUs;
  uint16_t escMaxUs;
  uint32_t crc;
};

RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
Servo esc1, esc2, esc3, esc4;

GuideStep guideStep = GUIDE_WAIT_LINK;
bool guideMessageDirty = true;

RcPacket lastPacket {};
bool linkAlive = false;
unsigned long lastPacketMs = 0;

float attitudeRoll = 0.0f;
float attitudePitch = 0.0f;
float yawRate = 0.0f;
float altitudeCm = 0.0f;

float gyroBias[3] = {0};
float accelBias[3] = {0};
float pressureOffset = 0.0f;
uint16_t escMinCalUs = ESC_MIN_US;
uint16_t escMaxCalUs = ESC_MAX_US;
bool calibrationLoaded = false;

float pidIntegral[3] = {0};

bool isArmed = false;
bool spinConfirmed = false;
bool calibrationPending = false;

unsigned long lastSensorUs = 0;
uint16_t ms5611Prom[7] = {0};

// Forward declarations
void configureImu();
void configureMs5611();
void readImu(int16_t* accel, int16_t* gyro);
float readAltitudeCm();
void loadCalibration();
void saveCalibration();
void runFullCalibration();
void runEscCalibration();
void playTone(uint16_t freq, uint16_t ms);
void setGuideStep(GuideStep step);
void handleRadio();
void updateAttitude(float dt);
void runController(float dt);
void writeEscs(uint16_t throttleUs, float rollTerm, float pitchTerm, float yawTerm);
uint32_t simpleCrc32(const uint8_t* data, size_t len);
void printGuide();
void smoothSpinTest();

void setup() {
  Serial.begin(115200);
  while (!Serial) {;}

  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_GUIDE_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
  digitalWrite(PIN_GUIDE_LED, LOW);

  Wire.begin();
  configureImu();
  configureMs5611();
  loadCalibration();

  esc1.attach(PIN_ESC_1);
  esc2.attach(PIN_ESC_2);
  esc3.attach(PIN_ESC_3);
  esc4.attach(PIN_ESC_4);
  esc1.writeMicroseconds(ESC_MIN_US);
  esc2.writeMicroseconds(ESC_MIN_US);
  esc3.writeMicroseconds(ESC_MIN_US);
  esc4.writeMicroseconds(ESC_MIN_US);

  if (!radio.begin()) {
    Serial.println(F("[NRF] Radio initialization failed!"));
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(RADIO_CHANNEL);
  radio.enableAckPayload();
  radio.openReadingPipe(1, RADIO_PIPE);
  radio.startListening();

  lastSensorUs = micros();
  Serial.println(F("=== Build & Calibrate Quadcopter FC ==="));
  Serial.println(F("Waiting for RC link..."));
}

void loop() {
  const unsigned long nowUs = micros();
  float dt = (nowUs - lastSensorUs) / 1000000.0f;
  if (dt < 0.0005f) {
    dt = 0.0005f;
  }
  lastSensorUs = nowUs;

  handleRadio();

  if (linkAlive && isArmed) {
    updateAttitude(dt);
    runController(dt);
  } else {
    writeEscs(ESC_MIN_US, 0, 0, 0);
    pidIntegral[0] = pidIntegral[1] = pidIntegral[2] = 0;
  }

  // Link watchdog
  if (linkAlive && millis() - lastPacketMs > 300) {
    linkAlive = false;
    isArmed = false;
    spinConfirmed = false;
    setGuideStep(GUIDE_WAIT_LINK);
    Serial.println(F("[Failsafe] RC link lost. Disarming."));
    playTone(200, 150);
  }

  printGuide();
}

void handleRadio() {
  bool packetReceived = false;
  while (radio.available()) {
    radio.read(&lastPacket, sizeof(lastPacket));
    packetReceived = true;
  }

  if (!packetReceived) {
    return;
  }

  lastPacketMs = millis();
  if (!linkAlive) {
    linkAlive = true;
    playTone(2500, 120);
    delay(40);
    playTone(2500, 120);
    setGuideStep(GUIDE_REQUIRE_KILL);
    Serial.println(F("[NRF] Link established."));
  }

  TelemetryPacket telemetry {};
  telemetry.throttleUs = lastPacket.throttleUs;
  telemetry.roll = attitudeRoll;
  telemetry.pitch = attitudePitch;
  telemetry.yaw = yawRate;
  telemetry.altitudeCm = altitudeCm;
  telemetry.guideStep = static_cast<uint8_t>(guideStep);
  telemetry.linkQuality = constrain((uint8_t)min(255UL, millis() - lastPacketMs), 0, 255);
  radio.writeAckPayload(1, &telemetry, sizeof(telemetry));

  bool killActive = lastPacket.armSwitch == 0;

  // Blink status LED whenever RC sends button press feedback
  if (lastPacket.buttonCal || lastPacket.buttonSpin) {
    digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
  }

  switch (guideStep) {
    case GUIDE_WAIT_LINK:
      break;
    case GUIDE_REQUIRE_KILL:
      if (killActive) {
        setGuideStep(GUIDE_REQUIRE_CAL);
      }
      break;
    case GUIDE_REQUIRE_CAL:
      if (!killActive) {
        Serial.println(F("[Guide] Kill switch must remain OFF during calibration."));
        setGuideStep(GUIDE_REQUIRE_KILL);
        break;
      }
      if (lastPacket.buttonCal && !calibrationPending) {
        calibrationPending = true;
        runFullCalibration();
        calibrationPending = false;
        setGuideStep(GUIDE_REQUIRE_ARM);
      }
      break;
    case GUIDE_REQUIRE_ARM:
      if (killActive) {
        isArmed = false;
      } else {
        setGuideStep(GUIDE_REQUIRE_SPOOL);
      }
      break;
    case GUIDE_REQUIRE_SPOOL:
      if (killActive) {
        setGuideStep(GUIDE_REQUIRE_ARM);
        break;
      }
      if (!spinConfirmed && lastPacket.buttonSpin) {
        smoothSpinTest();
        spinConfirmed = true;
        setGuideStep(GUIDE_READY);
      }
      break;
    case GUIDE_READY:
      if (killActive) {
        spinConfirmed = false;
        isArmed = false;
        setGuideStep(GUIDE_REQUIRE_ARM);
      } else if (spinConfirmed) {
        isArmed = true;
        if (lastPacket.throttleUs > escMinCalUs + 50) {
          setGuideStep(GUIDE_FLYING);
        }
      }
      break;
    case GUIDE_FLYING:
      if (killActive) {
        isArmed = false;
        spinConfirmed = false;
        setGuideStep(GUIDE_REQUIRE_ARM);
      }
      break;
  }
}

void setGuideStep(GuideStep step) {
  if (guideStep == step) {
    return;
  }
  guideStep = step;
  guideMessageDirty = true;
  switch (guideStep) {
    case GUIDE_WAIT_LINK:
      digitalWrite(PIN_GUIDE_LED, LOW);
      break;
    case GUIDE_REQUIRE_KILL:
    case GUIDE_REQUIRE_CAL:
      digitalWrite(PIN_GUIDE_LED, (millis() / 200) % 2);
      break;
    case GUIDE_REQUIRE_ARM:
    case GUIDE_REQUIRE_SPOOL:
      digitalWrite(PIN_GUIDE_LED, HIGH);
      break;
    case GUIDE_READY:
      digitalWrite(PIN_GUIDE_LED, HIGH);
      playTone(1200, 120);
      break;
    case GUIDE_FLYING:
      digitalWrite(PIN_GUIDE_LED, HIGH);
      break;
  }
}

void printGuide() {
  if (!guideMessageDirty) {
    return;
  }
  guideMessageDirty = false;

  switch (guideStep) {
    case GUIDE_WAIT_LINK:
      Serial.println(F("[Guide] Waiting for RC link..."));
      Serial.println(F("        Ensure TX is powered and bound."));
      break;
    case GUIDE_REQUIRE_KILL:
      Serial.println(F("[Guide] Toggle switch to KILL (safe) position."));
      Serial.println(F("        FC confirms before continuing."));
      break;
    case GUIDE_REQUIRE_CAL:
      Serial.println(F("[Guide] Press Button 1 on RC to start IMU+MS5611+ESC calibration."));
      Serial.println(F("        Keep props removed. FC will store calibration in EEPROM."));
      break;
    case GUIDE_REQUIRE_ARM:
      Serial.println(F("[Guide] Move toggle to ARM position when safe to continue."));
      Serial.println(F("        FC will still keep motors OFF until smooth spin test."));
      break;
    case GUIDE_REQUIRE_SPOOL:
      Serial.println(F("[Guide] Press Button 2 for smooth motor spool-up test."));
      Serial.println(F("        Motors will gently spin up and down without lift."));
      break;
    case GUIDE_READY:
      Serial.println(F("[Guide] Drone ready. Increase throttle slowly to take off."));
      Serial.println(F("        Serial monitor streaming live telemetry..."));
      break;
    case GUIDE_FLYING:
      Serial.println(F("[Guide] In flight. Use sticks: Throttle/Yaw (left), Pitch/Roll (right)."));
      break;
  }
}

void configureImu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00); // Wake up
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00); // ±250 dps
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x08); // ±4g
  Wire.endTransmission();

  delay(50);
}

void configureMs5611() {
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(0x1E); // reset
  Wire.endTransmission();
  delay(10);
  for (uint8_t i = 0; i < 6; ++i) {
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(0xA2 + i * 2);
    Wire.endTransmission();
    Wire.requestFrom(MS5611_ADDR, (uint8_t)2);
    ms5611Prom[i] = (Wire.read() << 8) | Wire.read();
  }
}

uint32_t readMs5611Adc(uint8_t command) {
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(command);
  Wire.endTransmission();
  delayMicroseconds(9500); // OSR4096
  Wire.beginTransmission(MS5611_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(MS5611_ADDR, (uint8_t)3);
  uint32_t value = 0;
  value = ((uint32_t)Wire.read() << 16) | ((uint32_t)Wire.read() << 8) | Wire.read();
  return value;
}

float readAltitudeCm() {
  const uint32_t D1 = readMs5611Adc(0x48); // pressure
  const uint32_t D2 = readMs5611Adc(0x58); // temperature
  const int32_t dT = D2 - ((int32_t)ms5611Prom[4] << 8);
  int64_t OFF = ((int64_t)ms5611Prom[1] << 16) + ((int64_t)ms5611Prom[3] * dT) / 128;
  int64_t SENS = ((int64_t)ms5611Prom[0] << 15) + ((int64_t)ms5611Prom[2] * dT) / 256;
  const int32_t TEMP = 2000 + (int64_t)dT * ms5611Prom[5] / 8388608;
  (void)TEMP;
  int32_t P = (int32_t)((((int64_t)D1 * SENS) / 2097152 - OFF) / 32768);
  const float pressure = P; // Pa
  float altitude = (1.0f - pow(pressure / 101325.0f, 0.1903f)) * 44330.0f * 100.0f;
  altitudeCm = altitude - pressureOffset;
  return altitudeCm;
}

void readImu(int16_t* accel, int16_t* gyro) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  accel[0] = (Wire.read() << 8) | Wire.read();
  accel[1] = (Wire.read() << 8) | Wire.read();
  accel[2] = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp
  gyro[0] = (Wire.read() << 8) | Wire.read();
  gyro[1] = (Wire.read() << 8) | Wire.read();
  gyro[2] = (Wire.read() << 8) | Wire.read();
}

void updateAttitude(float dt) {
  int16_t accelRaw[3], gyroRaw[3];
  readImu(accelRaw, gyroRaw);

  const float gyroScale = 131.0f;
  const float accelScale = 8192.0f; // ±4g

  float gx = (gyroRaw[0] - gyroBias[0]) / gyroScale;
  float gy = (gyroRaw[1] - gyroBias[1]) / gyroScale;
  float gz = (gyroRaw[2] - gyroBias[2]) / gyroScale;

  float ax = (accelRaw[0] - accelBias[0]) / accelScale;
  float ay = (accelRaw[1] - accelBias[1]) / accelScale;
  float az = (accelRaw[2] - accelBias[2]) / accelScale;

  float accRoll = atan2(ay, az) * RAD_TO_DEG;
  float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  attitudeRoll = COMPLEMENTARY_ALPHA * (attitudeRoll + gx * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accRoll;
  attitudePitch = COMPLEMENTARY_ALPHA * (attitudePitch + gy * dt) + (1.0f - COMPLEMENTARY_ALPHA) * accPitch;
  yawRate = gz;

  altitudeCm = readAltitudeCm();
}

void runController(float dt) {
  constexpr float KP_ROLL = 4.0f;
  constexpr float KI_ROLL = 0.03f;
  constexpr float KD_ROLL = 10.0f;
  constexpr float KP_PITCH = 4.0f;
  constexpr float KI_PITCH = 0.03f;
  constexpr float KD_PITCH = 10.0f;
  constexpr float KP_YAW = 2.0f;
  constexpr float KI_YAW = 0.02f;
  constexpr float KD_YAW = 0.0f;

  const float rollSet = lastPacket.roll * 0.04f;   // ~±40 deg
  const float pitchSet = lastPacket.pitch * 0.04f;
  const float yawSet = lastPacket.yaw * 0.3f;      // deg/s

  float rollError = rollSet - attitudeRoll;
  float pitchError = pitchSet - attitudePitch;
  float yawError = yawSet - yawRate;

  pidIntegral[0] = constrain(pidIntegral[0] + rollError * dt * KI_ROLL, -50.0f, 50.0f);
  pidIntegral[1] = constrain(pidIntegral[1] + pitchError * dt * KI_PITCH, -50.0f, 50.0f);
  pidIntegral[2] = constrain(pidIntegral[2] + yawError * dt * KI_YAW, -50.0f, 50.0f);

  float rollTerm = KP_ROLL * rollError + pidIntegral[0] + KD_ROLL * (rollError - 0) / dt;
  float pitchTerm = KP_PITCH * pitchError + pidIntegral[1] + KD_PITCH * (pitchError - 0) / dt;
  float yawTerm = KP_YAW * yawError + pidIntegral[2] + KD_YAW * (yawError - 0) / dt;

  uint16_t throttle = constrain(lastPacket.throttleUs, escMinCalUs, escMaxCalUs);
  writeEscs(throttle, rollTerm, pitchTerm, yawTerm);
}

void writeEscs(uint16_t throttleUs, float rollTerm, float pitchTerm, float yawTerm) {
  int16_t m1 = throttleUs + rollTerm - pitchTerm + yawTerm;
  int16_t m2 = throttleUs - rollTerm - pitchTerm - yawTerm;
  int16_t m3 = throttleUs - rollTerm + pitchTerm + yawTerm;
  int16_t m4 = throttleUs + rollTerm + pitchTerm - yawTerm;

  m1 = constrain(m1, escMinCalUs, escMaxCalUs);
  m2 = constrain(m2, escMinCalUs, escMaxCalUs);
  m3 = constrain(m3, escMinCalUs, escMaxCalUs);
  m4 = constrain(m4, escMinCalUs, escMaxCalUs);

  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);
}

void runFullCalibration() {
  Serial.println(F("[Cal] Starting IMU + MS5611 + ESC calibration. Props OFF!"));
  playTone(1000, 200);
  delay(200);
  playTone(1000, 200);

  const size_t samples = 2000;
  long gyroSum[3] = {0};
  long accelSum[3] = {0};
  for (size_t i = 0; i < samples; ++i) {
    int16_t accel[3], gyro[3];
    readImu(accel, gyro);
    gyroSum[0] += gyro[0];
    gyroSum[1] += gyro[1];
    gyroSum[2] += gyro[2];
    accelSum[0] += accel[0];
    accelSum[1] += accel[1];
    accelSum[2] += accel[2];
    delay(3);
  }

  for (int i = 0; i < 3; ++i) {
    gyroBias[i] = gyroSum[i] / (float)samples;
    accelBias[i] = accelSum[i] / (float)samples;
  }
  accelBias[2] -= 8192.0f; // remove 1g

  pressureOffset = readAltitudeCm();
  runEscCalibration();

  saveCalibration();
  Serial.println(F("[Cal] Completed and saved to EEPROM."));
  playTone(2000, 200);
}

void runEscCalibration() {
  Serial.println(F("[ESC] Calibrating: sending MAX for 2 s, then MIN for 2 s."));
  Serial.println(F("      Make sure battery is connected and props removed."));
  writeEscs(ESC_MAX_US, 0, 0, 0);
  delay(2000);
  writeEscs(ESC_MIN_US, 0, 0, 0);
  delay(2000);
  escMinCalUs = ESC_MIN_US;
  escMaxCalUs = ESC_MAX_US;
}

void smoothSpinTest() {
  Serial.println(F("[Motors] Smooth spin-up test running..."));
  const uint16_t target = escMinCalUs + 200;
  for (uint16_t pulse = escMinCalUs; pulse < target; pulse += 2) {
    writeEscs(pulse, 0, 0, 0);
    delay(15);
  }
  delay(1200);
  for (uint16_t pulse = target; pulse > escMinCalUs; pulse -= 2) {
    writeEscs(pulse, 0, 0, 0);
    delay(15);
  }
  writeEscs(ESC_MIN_US, 0, 0, 0);
  Serial.println(F("[Motors] Spin test finished."));
}

void loadCalibration() {
  CalibrationBlob blob;
  EEPROM.get(0, blob);
  const uint32_t expected = simpleCrc32(reinterpret_cast<uint8_t*>(&blob), sizeof(blob) - sizeof(uint32_t));
  if (blob.crc == expected) {
    memcpy(gyroBias, blob.gyroBias, sizeof(gyroBias));
    memcpy(accelBias, blob.accelBias, sizeof(accelBias));
    pressureOffset = blob.pressureOffset;
    escMinCalUs = blob.escMinUs;
    escMaxCalUs = blob.escMaxUs;
    calibrationLoaded = true;
    Serial.println(F("[Cal] Calibration loaded from EEPROM."));
  } else {
    Serial.println(F("[Cal] No valid calibration found. Run Button 1 procedure."));
  }
}

void saveCalibration() {
  CalibrationBlob blob {};
  memcpy(blob.gyroBias, gyroBias, sizeof(gyroBias));
  memcpy(blob.accelBias, accelBias, sizeof(accelBias));
  blob.pressureOffset = pressureOffset;
  blob.escMinUs = escMinCalUs;
  blob.escMaxUs = escMaxCalUs;
  blob.crc = simpleCrc32(reinterpret_cast<uint8_t*>(&blob), sizeof(blob) - sizeof(uint32_t));
  EEPROM.put(0, blob);
}

uint32_t simpleCrc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320;
      } else {
        crc >>= 1;
      }
    }
  }
  return ~crc;
}

void playTone(uint16_t freq, uint16_t ms) {
  tone(PIN_BUZZER, freq, ms);
  delay(ms);
  noTone(PIN_BUZZER);
}
