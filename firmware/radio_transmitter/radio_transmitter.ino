/*
 * Arduino Nano RC Transmitter for NRF24L01+
 * -------------------------------------------------------------
 * Hardware:
 * - Arduino Nano
 * - nRF24L01+ (CE = D7, CSN = D8)
 * - 2x joysticks (Throttle/Yaw on A0/A1, Pitch/Roll on A2/A3)
 * - Toggle switch (Arm / Kill) on D4 (active LOW = kill)
 * - Button 1 (calibration command) on D2 (active LOW)
 * - Button 2 (smooth spool command) on D3 (active LOW)
 * - Status LED on D5 (blinks on successful TX or button events)
 *
 * Features:
 * - Stick calibration stored in EEPROM (hold Button 1 at power-up to recalibrate)
 * - Throttle guard fixes "center = 1000" issue by learning joystick min/max
 * - Sends guided-command bits (calibration + spool) to the flight controller
 * - Receives telemetry via ACK payload and prints it to Serial (115200)
 */

#include <SPI.h>
#include <EEPROM.h>
#include <RF24.h>
#include <math.h>

// ------------------------- Pin mapping ------------------------------
constexpr uint8_t PIN_BUTTON_CAL = 2;
constexpr uint8_t PIN_BUTTON_SPOOL = 3;
constexpr uint8_t PIN_ARM_SWITCH = 4;   // toggle: LOW = kill, HIGH = arm
constexpr uint8_t PIN_STATUS_LED = 5;
constexpr uint8_t PIN_NRF_CE = 7;
constexpr uint8_t PIN_NRF_CSN = 8;

constexpr uint8_t AXIS_THROTTLE = A0;
constexpr uint8_t AXIS_YAW = A1;
constexpr uint8_t AXIS_PITCH = A2;
constexpr uint8_t AXIS_ROLL = A3;

constexpr uint8_t BTN_CALIBRATE = 0x01;
constexpr uint8_t BTN_SPOOL = 0x02;

const byte RADIO_ADDRESS[6] = "CTRL1";
constexpr uint8_t RADIO_CHANNEL = 108;
constexpr uint32_t LOOP_INTERVAL_US = 5000; // 200 Hz update rate

struct ControlPacket {
  uint16_t throttle;
  uint16_t roll;
  uint16_t pitch;
  uint16_t yaw;
  uint8_t armSwitch;
  uint8_t buttons;
  uint8_t frameId;
} __attribute__((packed));

struct TelemetryPacket {
  uint16_t throttle;
  int16_t rollDeciDeg;
  int16_t pitchDeciDeg;
  int16_t yawDeciDeg;
  int16_t altitudeCm;
  uint8_t linkQuality;
  uint8_t setupStep;
  uint8_t flags;
  uint8_t channel;
} __attribute__((packed));

struct StickCal {
  uint16_t min;
  uint16_t center;
  uint16_t max;
};

struct TxCalibration {
  StickCal throttle;
  StickCal yaw;
  StickCal pitch;
  StickCal roll;
  uint32_t crc;
} __attribute__((packed));

RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
TxCalibration stickCal;
TelemetryPacket telemetry;
ControlPacket txPacket = {1000, 1500, 1500, 1500, 0, 0, 0};
uint8_t frameCounter = 0;
uint32_t lastSendMicros = 0;
uint32_t lastSerialMillis = 0;
uint32_t txSuccess = 0;
uint32_t txFails = 0;

// ------------------------- Utilities --------------------------------
float clampf(float v, float minV, float maxV) {
  if (v < minV) return minV;
  if (v > maxV) return maxV;
  return v;
}

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

void saveTxCalibration() {
  stickCal.crc = 0;
  uint32_t crc = crc32For(reinterpret_cast<uint8_t *>(&stickCal), sizeof(stickCal));
  stickCal.crc = crc;
  EEPROM.put(0, stickCal);
}

void loadTxCalibration() {
  EEPROM.get(0, stickCal);
  uint32_t stored = stickCal.crc;
  stickCal.crc = 0;
  uint32_t computed = crc32For(reinterpret_cast<uint8_t *>(&stickCal), sizeof(stickCal));
  if (stored != computed || stickCal.throttle.min == 0xFFFF) {
    stickCal.throttle = {200, 250, 820};
    stickCal.yaw = {200, 512, 820};
    stickCal.pitch = {200, 512, 820};
    stickCal.roll = {200, 512, 820};
    Serial.println(F("[TX] No stick calibration found. Using defaults."));
  } else {
    Serial.println(F("[TX] Loaded stick calibration."));
  }
}

uint16_t readAxis(uint8_t pin) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 8; i++) {
    sum += analogRead(pin);
  }
  return sum / 8;
}

StickCal captureStickExtents(uint8_t pin, bool expectCenter) {
  StickCal cal = {UINT16_MAX, 512, 0};
  uint32_t start = millis();
  while (millis() - start < 5000) {
    uint16_t sample = readAxis(pin);
    if (sample < cal.min) cal.min = sample;
    if (sample > cal.max) cal.max = sample;
    if (expectCenter) {
      cal.center = (cal.min + cal.max) / 2;
    } else {
      cal.center = cal.min; // throttle uses min as resting point
    }
    delay(5);
  }
  return cal;
}

void calibrateSticks() {
  Serial.println(F("\n[TX] Stick calibration mode"));
  Serial.println(F("- Move all joysticks to their extremes repeatedly."));
  Serial.println(F("- Keep throttle stick fully DOWN when finished."));
  Serial.println(F("- Release pitch/roll/yaw to center before timeout."));
  digitalWrite(PIN_STATUS_LED, HIGH);

  stickCal.throttle = captureStickExtents(AXIS_THROTTLE, false);
  stickCal.yaw = captureStickExtents(AXIS_YAW, true);
  stickCal.pitch = captureStickExtents(AXIS_PITCH, true);
  stickCal.roll = captureStickExtents(AXIS_ROLL, true);

  saveTxCalibration();
  digitalWrite(PIN_STATUS_LED, LOW);
  Serial.println(F("[TX] Calibration saved."));
  delay(500);
}

float normalizeAxis(const StickCal &cal, uint16_t raw) {
  if (raw >= cal.center) {
    int16_t span = (int16_t)cal.max - (int16_t)cal.center;
    if (span < 1) span = 1;
    return (float)(raw - cal.center) / span;
  } else {
    int16_t span = (int16_t)cal.center - (int16_t)cal.min;
    if (span < 1) span = 1;
    return (float)(raw - cal.center) / span;
  }
}

uint16_t pwmFromCenteredAxis(const StickCal &cal, uint16_t raw) {
  float norm = normalizeAxis(cal, raw);
  norm = clampf(norm, -1.0f, 1.0f);
  return (uint16_t)(1500 + norm * 500);
}

uint16_t pwmFromThrottle(const StickCal &cal, uint16_t raw) {
  int16_t span = (int16_t)cal.max - (int16_t)cal.min;
  if (span < 1) span = 1;
  float norm = (float)(raw - cal.min) / span;
  norm = clampf(norm, 0.0f, 1.0f);
  norm = pow(norm, 1.4f); // finer resolution near idle
  if (norm < 0.02f) norm = 0.0f; // safety idle
  return (uint16_t)(1000 + norm * 1000);
}

bool readButton(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

void blinkStatus(uint8_t count, uint16_t onMs = 60, uint16_t offMs = 60) {
  for (uint8_t i = 0; i < count; i++) {
    digitalWrite(PIN_STATUS_LED, HIGH);
    delay(onMs);
    digitalWrite(PIN_STATUS_LED, LOW);
    delay(offMs);
  }
}

void setupRadio() {
  if (!radio.begin()) {
    Serial.println(F("[TX] NRF init failed."));
    while (true) {
      blinkStatus(2, 80, 120);
      delay(500);
    }
  }
  radio.setChannel(RADIO_CHANNEL);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(3, 5);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.openWritingPipe(RADIO_ADDRESS);
  radio.stopListening();
}

void printTelemetry() {
  Serial.print(F("[RC] Throttle:"));
  Serial.print(txPacket.throttle);
  Serial.print(F(" | Roll:"));
  Serial.print(telemetry.rollDeciDeg / 10.0f, 1);
  Serial.print(F(" | Pitch:"));
  Serial.print(telemetry.pitchDeciDeg / 10.0f, 1);
  Serial.print(F(" | Yaw:"));
  Serial.print(telemetry.yawDeciDeg / 10.0f, 1);
  Serial.print(F(" | Alt(m):"));
  Serial.print(telemetry.altitudeCm / 100.0f, 2);
  Serial.print(F(" | Link:%"));
  Serial.print(telemetry.linkQuality);
  Serial.print(F(" | Step:"));
  Serial.print(telemetry.setupStep);
  Serial.print(F(" | Flags:"));
  Serial.print(telemetry.flags, BIN);
  Serial.print(F(" | Ch:"));
  Serial.println(telemetry.channel);
}

// ------------------------- Arduino setup ----------------------------
void setup() {
  pinMode(PIN_BUTTON_CAL, INPUT_PULLUP);
  pinMode(PIN_BUTTON_SPOOL, INPUT_PULLUP);
  pinMode(PIN_ARM_SWITCH, INPUT_PULLUP);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.println(F("\nArduino Nano RC Transmitter"));
  Serial.println(F("--------------------------------------"));

  loadTxCalibration();

  if (readButton(PIN_BUTTON_CAL)) {
    calibrateSticks();
  }

  setupRadio();
  blinkStatus(3, 80, 80);
  Serial.println(F("[TX] Radio ready. Follow Serial instructions from flight controller."));
}

// ------------------------- Main loop --------------------------------
void loop() {
  uint32_t nowMicros = micros();
  if (nowMicros - lastSendMicros < LOOP_INTERVAL_US) {
    return;
  }
  lastSendMicros = nowMicros;

  bool armSwitchState = digitalRead(PIN_ARM_SWITCH) == HIGH;
  bool buttonCal = readButton(PIN_BUTTON_CAL);
  bool buttonSpool = readButton(PIN_BUTTON_SPOOL);

  uint16_t rawThrottle = readAxis(AXIS_THROTTLE);
  uint16_t rawYaw = readAxis(AXIS_YAW);
  uint16_t rawPitch = readAxis(AXIS_PITCH);
  uint16_t rawRoll = readAxis(AXIS_ROLL);

  txPacket.throttle = pwmFromThrottle(stickCal.throttle, rawThrottle);
  txPacket.yaw = pwmFromCenteredAxis(stickCal.yaw, rawYaw);
  txPacket.pitch = pwmFromCenteredAxis(stickCal.pitch, rawPitch);
  txPacket.roll = pwmFromCenteredAxis(stickCal.roll, rawRoll);
  txPacket.armSwitch = armSwitchState ? 1 : 0;
  txPacket.buttons = 0;
  if (buttonCal) txPacket.buttons |= BTN_CALIBRATE;
  if (buttonSpool) txPacket.buttons |= BTN_SPOOL;
  txPacket.frameId = frameCounter++;

  bool ok = radio.write(&txPacket, sizeof(txPacket));
  if (ok) {
    txSuccess++;
    digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
    if (radio.isAckPayloadAvailable()) {
      radio.read(&telemetry, sizeof(telemetry));
    }
  } else {
    txFails++;
  }

  if (millis() - lastSerialMillis > 300) {
    lastSerialMillis = millis();
    printTelemetry();
  }
}
