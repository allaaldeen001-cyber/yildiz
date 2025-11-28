/*
 * Build & Calibrate Arduino Quadcopter Drone - Radio Controller
 * -------------------------------------------------------------
 * Hardware: Arduino Nano, NRF24L01+, 2x joysticks, toggle (kill/arm),
 *           Button1 (calibration command), Button2 (smooth spin command),
 *           status LED for communication feedback.
 *
 * Features:
 *  - NRF24 link with ACK payload reception from the flight controller.
 *  - Stick calibration mode (hold both buttons during power-on).
 *  - Auto-centering with deadband to prevent throttle mid-stick runaway.
 *  - Smooth throttle filter for safer motor spin-up.
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>
#include <math.h>

// ---------------- Pin mapping -----------------
constexpr uint8_t PIN_JOY_LY = A0; // throttle
constexpr uint8_t PIN_JOY_LX = A1; // yaw
constexpr uint8_t PIN_JOY_RY = A2; // pitch
constexpr uint8_t PIN_JOY_RX = A3; // roll

constexpr uint8_t PIN_TOGGLE = 2;      // kill/arm switch (LOW = kill)
constexpr uint8_t PIN_BUTTON_CAL = 4;  // Button 1
constexpr uint8_t PIN_BUTTON_SPIN = 5; // Button 2
constexpr uint8_t PIN_STATUS_LED = 6;

constexpr uint8_t PIN_NRF_CE = 9;
constexpr uint8_t PIN_NRF_CSN = 10;

constexpr uint16_t ESC_MIN_US = 1000;
constexpr uint16_t ESC_MAX_US = 2000;

constexpr uint16_t RADIO_CHANNEL = 90;
const byte RADIO_PIPE[6] = "FCN01";

struct RcPacket {
  uint16_t throttleUs;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  uint8_t armSwitch;
  uint8_t buttonCal;
  uint8_t buttonSpin;
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

struct StickCal {
  uint16_t min;
  uint16_t center;
  uint16_t max;
};

struct TxCalibrationBlob {
  uint16_t magic;
  StickCal sticks[4]; // throttle, yaw, pitch, roll
  uint32_t crc;
};

RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);

StickCal calThrottle {180, 200, 880};
StickCal calYaw {120, 512, 900};
StickCal calPitch {120, 512, 900};
StickCal calRoll {120, 512, 900};

uint16_t frameCounter = 0;
unsigned long lastTelemetryMs = 0;
TelemetryPacket lastTelemetry {};

bool buttonCalLatched = false;
bool buttonSpinLatched = false;

// Forward declarations
void loadTxCalibration();
void saveTxCalibration();
void runTxCalibrationMode();
StickCal* stickByIndex(uint8_t idx);
int16_t mapAxis(int raw, const StickCal& cal);
uint16_t mapThrottle(int raw, const StickCal& cal);
bool readButtonMomentary(uint8_t pin, bool& latch);
void printTelemetry();

void setup() {
  Serial.begin(115200);
  while (!Serial) {;}

  pinMode(PIN_TOGGLE, INPUT_PULLUP);
  pinMode(PIN_BUTTON_CAL, INPUT_PULLUP);
  pinMode(PIN_BUTTON_SPIN, INPUT_PULLUP);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);

  loadTxCalibration();

  if (digitalRead(PIN_BUTTON_CAL) == LOW && digitalRead(PIN_BUTTON_SPIN) == LOW) {
    runTxCalibrationMode();
  }

  if (!radio.begin()) {
    Serial.println(F("[NRF] Radio init failed."));
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(RADIO_CHANNEL);
  radio.enableAckPayload();
  radio.openWritingPipe(RADIO_PIPE);
  radio.stopListening();

  Serial.println(F("=== RC Transmitter Ready ==="));
}

void loop() {
  const bool killSwitch = digitalRead(PIN_TOGGLE) == LOW;
  const uint16_t rawThrottle = analogRead(PIN_JOY_LY);
  const uint16_t rawYaw = analogRead(PIN_JOY_LX);
  const uint16_t rawPitch = analogRead(PIN_JOY_RY);
  const uint16_t rawRoll = analogRead(PIN_JOY_RX);

  const bool buttonCalPressed = readButtonMomentary(PIN_BUTTON_CAL, buttonCalLatched);
  const bool buttonSpinPressed = readButtonMomentary(PIN_BUTTON_SPIN, buttonSpinLatched);

  static float throttleFiltered = ESC_MIN_US;
  uint16_t throttleTarget = killSwitch ? ESC_MIN_US : mapThrottle(rawThrottle, calThrottle);
  throttleFiltered += (throttleTarget - throttleFiltered) * 0.05f;

  RcPacket packet {};
  packet.throttleUs = static_cast<uint16_t>(throttleFiltered);
  packet.roll = mapAxis(rawRoll, calRoll);
  packet.pitch = mapAxis(rawPitch, calPitch);
  packet.yaw = mapAxis(rawYaw, calYaw);
  packet.armSwitch = killSwitch ? 0 : 1;
  packet.buttonCal = buttonCalPressed ? 1 : 0;
  packet.buttonSpin = buttonSpinPressed ? 1 : 0;
  packet.channel = 1;
  packet.frameCounter = frameCounter++;

  bool sent = radio.write(&packet, sizeof(packet));
  if (sent) {
    digitalWrite(PIN_STATUS_LED, HIGH);
    if (radio.isAckPayloadAvailable()) {
      radio.read(&lastTelemetry, sizeof(lastTelemetry));
      lastTelemetryMs = millis();
    }
  } else {
    digitalWrite(PIN_STATUS_LED, (millis() / 200) % 2);
  }

  printTelemetry();
  delay(5);
}

void printTelemetry() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 200) {
    return;
  }
  lastPrint = millis();

  Serial.print(F("Throttle: "));
  Serial.print(lastTelemetry.throttleUs);
  Serial.print(F(" | Roll: "));
  Serial.print(lastTelemetry.roll);
  Serial.print(F(" | Pitch: "));
  Serial.print(lastTelemetry.pitch);
  Serial.print(F(" | Yaw(dps): "));
  Serial.print(lastTelemetry.yaw);
  Serial.print(F(" | Alt(cm): "));
  Serial.print(lastTelemetry.altitudeCm, 1);
  Serial.print(F(" | Step: "));
  Serial.print(lastTelemetry.guideStep);
  Serial.print(F(" | Link: "));
  Serial.println(lastTelemetry.linkQuality);
}

bool readButtonMomentary(uint8_t pin, bool& latch) {
  bool pressed = digitalRead(pin) == LOW;
  bool pulse = false;
  if (pressed && !latch) {
    pulse = true;
    latch = true;
  } else if (!pressed && latch) {
    latch = false;
  }
  return pulse;
}

int16_t mapAxis(int raw, const StickCal& cal) {
  const int16_t spanPos = max(1, (int16_t)cal.max - (int16_t)cal.center);
  const int16_t spanNeg = max(1, (int16_t)cal.center - (int16_t)cal.min);
  float normalized = 0.0f;
  if (raw >= cal.center) {
    normalized = (raw - cal.center) / (float)spanPos;
  } else {
    normalized = -(cal.center - raw) / (float)spanNeg;
  }
  normalized = constrain(normalized, -1.0f, 1.0f);
  const float deadband = 0.03f;
  if (fabs(normalized) < deadband) {
    normalized = 0.0f;
  }
  // gentle expo
  normalized = normalized * normalized * normalized + (1 - fabs(normalized)) * normalized * 0.3f;
  return (int16_t)(normalized * 500.0f);
}

uint16_t mapThrottle(int raw, const StickCal& cal) {
  const int16_t span = max(1, (int16_t)cal.max - (int16_t)cal.min);
  float normalized = (raw - cal.min) / (float)span;
  normalized = constrain(normalized, 0.0f, 1.0f);
  // hold idle zone until 5% travel to avoid 1000us mid-stick accidents
  if (normalized < 0.05f) {
    normalized = 0.0f;
  } else {
    normalized = (normalized - 0.05f) / 0.95f;
  }
  // expo curve
  normalized = normalized * normalized;
  return ESC_MIN_US + normalized * (ESC_MAX_US - ESC_MIN_US);
}

void loadTxCalibration() {
  TxCalibrationBlob blob;
  EEPROM.get(0, blob);
  const uint16_t MAGIC = 0xABCD;
  if (blob.magic != MAGIC) {
    Serial.println(F("[RC-Cal] No stored calibration, using defaults."));
    return;
  }
  uint32_t crc = 0xFFFFFFFF;
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&blob);
  for (size_t i = 0; i < sizeof(blob) - sizeof(uint32_t); ++i) {
    crc ^= bytes[i];
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : crc >> 1;
    }
  }
  crc = ~crc;
  if (crc != blob.crc) {
    Serial.println(F("[RC-Cal] CRC mismatch, using defaults."));
    return;
  }
  calThrottle = blob.sticks[0];
  calYaw = blob.sticks[1];
  calPitch = blob.sticks[2];
  calRoll = blob.sticks[3];
  Serial.println(F("[RC-Cal] Calibration loaded."));
}

void saveTxCalibration() {
  TxCalibrationBlob blob {};
  blob.magic = 0xABCD;
  blob.sticks[0] = calThrottle;
  blob.sticks[1] = calYaw;
  blob.sticks[2] = calPitch;
  blob.sticks[3] = calRoll;

  uint32_t crc = 0xFFFFFFFF;
  uint8_t* bytes = reinterpret_cast<uint8_t*>(&blob);
  for (size_t i = 0; i < sizeof(blob) - sizeof(uint32_t); ++i) {
    crc ^= bytes[i];
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : crc >> 1;
    }
  }
  blob.crc = ~crc;
  EEPROM.put(0, blob);
  Serial.println(F("[RC-Cal] Saved."));
}

StickCal* stickByIndex(uint8_t idx) {
  switch (idx) {
    case 0: return &calThrottle;
    case 1: return &calYaw;
    case 2: return &calPitch;
    case 3: return &calRoll;
    default: return &calThrottle;
  }
}

void runTxCalibrationMode() {
  Serial.println(F("=== RC Stick Calibration Mode ==="));
  Serial.println(F("Move each stick (including throttle) to full travel for 5 seconds..."));
  for (uint8_t i = 0; i < 4; ++i) {
    StickCal* cal = stickByIndex(i);
    cal->min = 1023;
    cal->max = 0;
    cal->center = 512;
  }

  const uint32_t durationMs = 5000;
  uint32_t start = millis();
  while (millis() - start < durationMs) {
    StickCal* throttle = &calThrottle;
    StickCal* yaw = &calYaw;
    StickCal* pitch = &calPitch;
    StickCal* roll = &calRoll;
    uint16_t raw[4] = {
      (uint16_t)analogRead(PIN_JOY_LY),
      (uint16_t)analogRead(PIN_JOY_LX),
      (uint16_t)analogRead(PIN_JOY_RY),
      (uint16_t)analogRead(PIN_JOY_RX)
    };
    StickCal* cals[4] = {throttle, yaw, pitch, roll};
    for (uint8_t i = 0; i < 4; ++i) {
      cals[i]->min = min(cals[i]->min, raw[i]);
      cals[i]->max = max(cals[i]->max, raw[i]);
    }
    delay(5);
  }

  calYaw.center = (calYaw.max + calYaw.min) / 2;
  calPitch.center = (calPitch.max + calPitch.min) / 2;
  calRoll.center = (calRoll.max + calRoll.min) / 2;
  calThrottle.center = calThrottle.min + (calThrottle.max - calThrottle.min) / 2;

  saveTxCalibration();
}
