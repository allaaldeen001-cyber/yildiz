#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>

#include "rf_protocol.h"

using namespace drone;

namespace {
constexpr uint8_t kRadioCePin = 9;
constexpr uint8_t kRadioCsnPin = 10;

constexpr uint8_t kJoyThrottlePin = A0;
constexpr uint8_t kJoyYawPin = A1;
constexpr uint8_t kJoyPitchPin = A2;
constexpr uint8_t kJoyRollPin = A3;

constexpr uint8_t kButtonCalPin = 4;
constexpr uint8_t kButtonArmPin = 5;
constexpr uint8_t kSwitchAltPin = 2;
constexpr uint8_t kSwitchKillPin = 3;

constexpr uint16_t kFrameIntervalUs = 20000;  // 50 Hz update rate
constexpr uint16_t kScreenIntervalMs = 250;

RF24 radio(kRadioCePin, kRadioCsnPin);
RcCommand tx_command = {1000, 0, 0, 0, 0, 0, 0, 0, 0};
FcStatus last_status = {};

uint32_t last_frame_micros = 0;
uint32_t last_screen_ms = 0;
uint32_t frame_counter = 0;

uint8_t prev_button_mask = 0;
bool link_ok = false;

int16_t readAxisToCommand(uint8_t pin) {
  int16_t raw = analogRead(pin) - 512;
  if (abs(raw) < 10) {
    raw = 0;
  }
  return constrain(raw, -512, 512) * 500 / 512;
}

uint16_t readThrottle() {
  int16_t raw = analogRead(kJoyThrottlePin);
  return constrain(map(raw, 0, 1023, 1000, 2000), 1000, 2000);
}

uint8_t buildButtonMask() {
  uint8_t mask = 0;
  if (digitalRead(kButtonCalPin) == LOW) {
    mask |= kButtonCalibrate;
  }
  if (digitalRead(kButtonArmPin) == LOW) {
    mask |= kButtonArm;
  }
  return mask;
}

uint8_t buildSwitchMask() {
  uint8_t mask = 0;
  if (digitalRead(kSwitchAltPin) == HIGH) {
    mask |= kSwitchAltitudeHold;
  }
  if (digitalRead(kSwitchKillPin) == HIGH) {
    mask |= kSwitchKill;
  }
  return mask;
}

void updateCommandFromInputs() {
  tx_command.throttle_us = readThrottle();
  tx_command.roll_cmd = readAxisToCommand(kJoyRollPin);
  tx_command.pitch_cmd = readAxisToCommand(kJoyPitchPin);
  tx_command.yaw_rate_cmd = readAxisToCommand(kJoyYawPin);
  tx_command.buttons = buildButtonMask();
  tx_command.switches = buildSwitchMask();
  tx_command.flags = 0;
  tx_command.aux = 0;

  bool button1_edge = (tx_command.buttons & kButtonCalibrate) && !(prev_button_mask & kButtonCalibrate);
  bool button2_edge = (tx_command.buttons & kButtonArm) && !(prev_button_mask & kButtonArm);

  bool kill_switch_enabled = (tx_command.switches & kSwitchKill);
  bool altitude_hold_on = (tx_command.switches & kSwitchAltitudeHold);

  if (button1_edge) {
    tx_command.flags |= kFlagRequestImuCalibration;
  }

  if (button2_edge) {
    if (!altitude_hold_on && !kill_switch_enabled) {
      tx_command.flags |= kFlagRequestEscCalibration;
    } else if (kill_switch_enabled) {
      tx_command.flags |= kFlagRequestArm;
    }
  }

  prev_button_mask = tx_command.buttons;
  tx_command.sequence = ++frame_counter;
}

void sendFrame() {
  radio.stopListening();
  bool ok = radio.write(&tx_command, sizeof(tx_command));
  radio.startListening();

  if (ok && radio.isAckPayloadAvailable()) {
    radio.read(&last_status, sizeof(last_status));
    link_ok = true;
  } else {
    link_ok = false;
  }

  tx_command.flags = 0;
}

const char* errorToString(uint8_t err) {
  switch (err) {
    case kErrorNone:
      return "NONE";
    case kErrorImu:
      return "IMU";
    case kErrorBarometer:
      return "BARO";
    case kErrorRadio:
      return "LINK";
    case kErrorWatchdog:
      return "WDOG";
    default:
      return "UNK";
  }
}

void printScreen() {
  Serial.print(link_ok ? F("[LINKED] ") : F("[NO LINK] "));
  Serial.print(F("THR:"));
  Serial.print(tx_command.throttle_us);
  Serial.print(F(" BTN1:"));
  Serial.print((tx_command.buttons & kButtonCalibrate) ? F("1") : F("0"));
  Serial.print(F(" BTN2:"));
  Serial.print((tx_command.buttons & kButtonArm) ? F("1") : F("0"));
  Serial.print(F(" SW1(ALT):"));
  Serial.print((tx_command.switches & kSwitchAltitudeHold) ? F("1") : F("0"));
  Serial.print(F(" SW2(ARM):"));
  Serial.print((tx_command.switches & kSwitchKill) ? F("1") : F("0"));

  if (link_ok) {
    Serial.print(F(" | FC:"));
    Serial.print((last_status.flags & kStatusArmed) ? F("ARMED") : F("SAFE"));
    Serial.print(F(" CAL:"));
    Serial.print((last_status.flags & kStatusImuCalibrated) ? F("IMU") : F("--"));
    Serial.print(F("/"));
    Serial.print((last_status.flags & kStatusEscCalibrated) ? F("ESC") : F("--"));
    Serial.print(F(" ALT:"));
    Serial.print(last_status.altitude_cm / 100.0f, 2);
    Serial.print(F("m"));
    Serial.print(F(" R:"));
    Serial.print(last_status.roll_deg_x10 / 10.0f, 1);
    Serial.print(F(" P:"));
    Serial.print(last_status.pitch_deg_x10 / 10.0f, 1);
    Serial.print(F(" ERR:"));
    Serial.print(errorToString(last_status.last_error));
  }
  Serial.println();
}

void setupRadio() {
  radio.begin();
  radio.setChannel(kRadioChannel);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  radio.setRetries(5, 15);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe(kRadioPipe);
  radio.openReadingPipe(1, kRadioPipe);
  radio.stopListening();
}

void initIo() {
  pinMode(kButtonCalPin, INPUT_PULLUP);
  pinMode(kButtonArmPin, INPUT_PULLUP);
  pinMode(kSwitchAltPin, INPUT_PULLUP);
  pinMode(kSwitchKillPin, INPUT_PULLUP);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  initIo();
  setupRadio();
  last_frame_micros = micros();
  last_screen_ms = millis();
}

void loop() {
  uint32_t now_us = micros();
  if (now_us - last_frame_micros >= kFrameIntervalUs) {
    last_frame_micros = now_us;
    updateCommandFromInputs();
    sendFrame();
  }

  if (millis() - last_screen_ms >= kScreenIntervalMs) {
    last_screen_ms = millis();
    printScreen();
  }
}
