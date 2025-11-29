#include <SPI.h>
#include <RF24.h>

#include "../shared/DroneLink.h"

using namespace DroneLink;

// -----------------------------------------------------------------------------
// Pin map (Arduino Nano)
// -----------------------------------------------------------------------------
constexpr uint8_t PIN_NRF_CE   = 9;   // CE -> D9
constexpr uint8_t PIN_NRF_CSN  = 10;  // CSN -> D10
constexpr uint8_t PIN_JOY_TH   = A0;  // Left vertical (Throttle)
constexpr uint8_t PIN_JOY_YAW  = A1;  // Left horizontal (Yaw)
constexpr uint8_t PIN_JOY_PIT  = A2;  // Right vertical (Pitch)
constexpr uint8_t PIN_JOY_ROLL = A3;  // Right horizontal (Roll)
constexpr uint8_t PIN_BTN_CAL  = 4;   // Button 1: calibration
constexpr uint8_t PIN_BTN_ARM  = 5;   // Button 2: motors/arming
constexpr uint8_t PIN_SW_ALT   = 2;   // Switch 1: altitude hold enable
constexpr uint8_t PIN_SW_ARM   = 3;   // Switch 2: arming/kill switch

constexpr uint16_t MIN_THROTTLE_US  = 1000;
constexpr uint16_t MAX_THROTTLE_US  = 2000;
constexpr uint16_t SEND_PERIOD_MS   = 20;    // 50 Hz uplink
constexpr uint16_t STATUS_PERIOD_MS = 300;   // Serial print cadence
constexpr uint16_t LONG_PRESS_MS    = 1500;  // ESC calibration trigger
constexpr uint16_t ANALOG_DEADZONE  = 8;     // Raw counts
constexpr uint32_t LINK_TIMEOUT_MS  = 350;   // Link considered stale

// -----------------------------------------------------------------------------
// Input helpers
// -----------------------------------------------------------------------------
struct DebouncedButton {
  uint8_t pin;
  bool stableState = false;
  bool lastReading = false;
  uint32_t lastFlip = 0;
  bool edgeFlag = false;
  bool edgeValue = false;

  void begin() {
    pinMode(pin, INPUT_PULLUP);
    lastReading = digitalRead(pin) == LOW;
    stableState = lastReading;
    lastFlip = millis();
  }

  void poll() {
    bool reading = digitalRead(pin) == LOW;
    if (reading != lastReading) {
      lastFlip = millis();
      lastReading = reading;
    }
    if ((millis() - lastFlip) > 15 && reading != stableState) {
      stableState = reading;
      edgeFlag = true;
      edgeValue = stableState;
    } else {
      edgeFlag = false;
    }
  }

  bool isPressed() const { return stableState; }
  bool pressedEdge() const { return edgeFlag && edgeValue; }
  bool releasedEdge() const { return edgeFlag && !edgeValue; }
};

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
DebouncedButton btnCal{PIN_BTN_CAL};
DebouncedButton btnArm{PIN_BTN_ARM};

ControlPacket control{};
TelemetryPacket telemetry{};

uint32_t txSequence = 0;
uint32_t lastSend = 0;
uint32_t armPressedAt = 0;
uint32_t lastAckMillis = 0;
uint32_t lastStatusPrint = 0;

bool armButtonPulse = false;
bool escCalibrationPulse = false;
bool linkHealthy = false;

BuzzerCue lastCueReported = BuzzerCue::NONE;

// -----------------------------------------------------------------------------
// Utility functions
// -----------------------------------------------------------------------------
uint16_t readThrottle() {
  int raw = analogRead(PIN_JOY_TH);
  raw = constrain(raw, 0, 1023);
  return map(raw, 0, 1023, MIN_THROTTLE_US, MAX_THROTTLE_US);
}

int16_t readCenteredAxis(uint8_t pin) {
  int raw = analogRead(pin);
  raw = constrain(raw, 0, 1023);
  int delta = raw - 512;
  if (abs(delta) < ANALOG_DEADZONE) {
    delta = 0;
  }
  float scaled = delta * (500.0f / 512.0f);
  scaled = constrain(scaled, -500.0f, 500.0f);
  return static_cast<int16_t>(scaled);
}

bool readSwitchHigh(uint8_t pin) {
  return digitalRead(pin) == HIGH;  // HIGH when open (default), LOW when grounded
}

void initRadio() {
  if (!radio.begin()) {
    Serial.println(F("NRF24 init failed"));
  }
  radio.setChannel(kRadioChannel);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(2, 5);
  radio.openWritingPipe(kRadioAddress);
  radio.stopListening();
}

void updateButtons() {
  btnCal.poll();
  btnArm.poll();

  if (btnArm.pressedEdge()) {
    armPressedAt = millis();
  }
  if (btnArm.releasedEdge()) {
    uint32_t held = millis() - armPressedAt;
    bool altHoldDisabled = !readSwitchHigh(PIN_SW_ALT);
    if (held >= LONG_PRESS_MS && altHoldDisabled) {
      escCalibrationPulse = true;
      Serial.println(F("ESC calibration request sent"));
    } else {
      armButtonPulse = true;
    }
  }
}

void buildControlPacket() {
  control.throttle = readThrottle();
  control.yaw = readCenteredAxis(PIN_JOY_YAW);
  control.pitch = readCenteredAxis(PIN_JOY_PIT);
  control.roll = readCenteredAxis(PIN_JOY_ROLL);
  control.flags = 0;
  if (btnCal.isPressed()) control.flags |= CTRL_FLAG_CALIBRATE;
  if (armButtonPulse)     control.flags |= CTRL_FLAG_ARM_BUTTON;
  if (readSwitchHigh(PIN_SW_ALT)) control.flags |= CTRL_FLAG_ALT_HOLD;
  if (readSwitchHigh(PIN_SW_ARM)) control.flags |= CTRL_FLAG_ARM_SWITCH;
  if (escCalibrationPulse) control.flags |= CTRL_FLAG_ESC_CALIB;
  control.reserved = 0;
  control.sequence = static_cast<uint16_t>(++txSequence);

  armButtonPulse = false;
  escCalibrationPulse = false;
}

void sendPacket() {
  buildControlPacket();
  bool ok = radio.write(&control, sizeof(control));
  if (!ok) {
    linkHealthy = false;
    return;
  }

  if (radio.isAckPayloadAvailable()) {
    radio.read(&telemetry, sizeof(telemetry));
    lastAckMillis = millis();
  }
  linkHealthy = (millis() - lastAckMillis) < LINK_TIMEOUT_MS;
}

const __FlashStringHelper* cueToText(BuzzerCue cue) {
  switch (cue) {
    case BuzzerCue::CALIBRATION_OK:   return F("Calibration OK");
    case BuzzerCue::CALIBRATION_FAIL: return F("Calibration FAIL");
    case BuzzerCue::ESC_SEQUENCE_START:return F("ESC sequence start");
    case BuzzerCue::ESC_SEQUENCE_DONE:return F("ESC sequence done");
    case BuzzerCue::ARMING_CHANGE:    return F("Arm state changed");
    default:                          return F("");
  }
}

void processTelemetry() {
  BuzzerCue cue = static_cast<BuzzerCue>(telemetry.buzzerCue);
  if (cue == BuzzerCue::NONE) {
    lastCueReported = BuzzerCue::NONE;
    return;
  }
  if (cue != lastCueReported) {
    Serial.print(F("[FC] "));
    Serial.println(cueToText(cue));
    lastCueReported = cue;
  }
}

void printStatus() {
  if (millis() - lastStatusPrint < STATUS_PERIOD_MS) {
    return;
  }
  lastStatusPrint = millis();

  bool altHold = readSwitchHigh(PIN_SW_ALT);
  bool armSwitch = readSwitchHigh(PIN_SW_ARM);

  Serial.print(F("Link:")); Serial.print(linkHealthy ? F("OK") : F("--"));
  Serial.print(F(" | Seq:")); Serial.print(control.sequence);
  Serial.print(F(" | ArmSW:")); Serial.print(armSwitch);
  Serial.print(F(" | AltHold:")); Serial.print(altHold);
  Serial.print(F(" | Btn1:")); Serial.print(btnCal.isPressed());
  Serial.print(F(" | Btn2:")); Serial.print(btnArm.isPressed());

  if (linkHealthy) {
    Serial.print(F(" | IMU:")); Serial.print((telemetry.systemFlags & TLM_FLAG_IMU_READY) ? F("ready") : F("--"));
    Serial.print(F(" | ESC:")); Serial.print((telemetry.systemFlags & TLM_FLAG_ESC_READY) ? F("ready") : F("--"));
    Serial.print(F(" | Armed:")); Serial.print((telemetry.systemFlags & TLM_FLAG_MOTORS_ARMED) ? F("1") : F("0"));
    Serial.print(F(" | FState:")); Serial.print(telemetry.flightState);
  }
  Serial.println();
}

// -----------------------------------------------------------------------------
// Arduino lifecycle
// -----------------------------------------------------------------------------
void setup() {
  pinMode(PIN_SW_ALT, INPUT_PULLUP);
  pinMode(PIN_SW_ARM, INPUT_PULLUP);

  btnCal.begin();
  btnArm.begin();

  analogReference(DEFAULT);
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("Remote Controller booting"));

  initRadio();
  lastSend = millis();
}

void loop() {
  updateButtons();

  if (millis() - lastSend >= SEND_PERIOD_MS) {
    sendPacket();
    lastSend = millis();
    processTelemetry();
  }

  printStatus();
}
