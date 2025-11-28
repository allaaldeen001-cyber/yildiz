/*
 * Arduino Nano Remote Controller Firmware
 * Hardware: Arduino Nano + NRF24L01 PA+LNA
 * Joysticks: Throttle/Yaw=A0/A1, Pitch/Roll=A2/A3
 * Buttons: Button_1(D4), Button_2(D5)
 * Switches: SW_1(D2 AltHold), SW_2(D3 Arm/Kill)
 * NRF: CE=D9, CSN=D10
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

#include "../common/CommProtocol.h"

// Pin map
constexpr uint8_t PIN_SW_ALTHOLD = 2;
constexpr uint8_t PIN_SW_ARM = 3;
constexpr uint8_t PIN_BTN_CAL = 4;
constexpr uint8_t PIN_BTN_ESC = 5;
constexpr uint8_t PIN_CE = 9;
constexpr uint8_t PIN_CSN = 10;

constexpr uint8_t PIN_STICK_THROTTLE = A0;
constexpr uint8_t PIN_STICK_YAW = A1;
constexpr uint8_t PIN_STICK_PITCH = A2;
constexpr uint8_t PIN_STICK_ROLL = A3;

// Stick calibration (raw ADC range)
constexpr int STICK_MIN = 80;
constexpr int STICK_MAX = 980;
constexpr uint8_t ANALOG_SAMPLES = 4;

constexpr float MAX_ANGLE_COMMAND_DEG = 30.0f;
constexpr float MAX_YAW_RATE_DPS = 180.0f;

constexpr uint32_t TX_PERIOD_US = 10000;     // 100 Hz command rate
constexpr uint32_t STATUS_PERIOD_MS = 100;   // 10 Hz serial status update
constexpr uint32_t LINK_TIMEOUT_US = 300000; // 300 ms watchdog

RF24 radio(PIN_CE, PIN_CSN);
RcToFcPacket txPacket = {};
FcToRcPacket rxTelemetry = {};

uint32_t lastTxMicros = 0;
uint32_t lastStatusMs = 0;
uint32_t lastLinkMicros = 0;
uint16_t txSeq = 0;

bool linkAlive = false;
bool telemetryValid = false;
uint32_t telemetryTimestampMs = 0;

// Cached UI states for serial printing
bool swAltHoldState = false;
bool swArmState = false;
bool btnCalState = false;
bool btnEscState = false;

int readAnalogAveraged(uint8_t pin) {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < ANALOG_SAMPLES; ++i) {
    acc += analogRead(pin);
  }
  return acc / ANALOG_SAMPLES;
}

int mapAnalogToPwm(int raw) {
  raw = constrain(raw, STICK_MIN, STICK_MAX);
  return map(raw, STICK_MIN, STICK_MAX, 1000, 2000);
}

int16_t mapStickToAngle(int pwmValue, float maxDeg) {
  float normalized = float(pwmValue - 1500) / 500.0f;
  normalized = constrain(normalized, -1.0f, 1.0f);
  return int16_t(normalized * maxDeg * 10.0f);
}

int16_t mapStickToYawRate(int pwmValue, float maxDps) {
  float normalized = float(pwmValue - 1500) / 500.0f;
  normalized = constrain(normalized, -1.0f, 1.0f);
  return int16_t(normalized * maxDps * 10.0f);
}

void readSwitchesAndButtons() {
  swAltHoldState = digitalRead(PIN_SW_ALTHOLD) == HIGH;
  swArmState = digitalRead(PIN_SW_ARM) == HIGH;
  btnCalState = digitalRead(PIN_BTN_CAL) == LOW; // active-low
  btnEscState = digitalRead(PIN_BTN_ESC) == LOW; // active-low
}

void buildPacket() {
  readSwitchesAndButtons();
  txPacket.seq = ++txSeq;
  txPacket.throttle = mapAnalogToPwm(readAnalogAveraged(PIN_STICK_THROTTLE));
  txPacket.roll = mapStickToAngle(mapAnalogToPwm(readAnalogAveraged(PIN_STICK_ROLL)), MAX_ANGLE_COMMAND_DEG);
  txPacket.pitch = mapStickToAngle(mapAnalogToPwm(readAnalogAveraged(PIN_STICK_PITCH)), MAX_ANGLE_COMMAND_DEG);
  txPacket.yaw = mapStickToYawRate(mapAnalogToPwm(readAnalogAveraged(PIN_STICK_YAW)), MAX_YAW_RATE_DPS);
  txPacket.switches = 0;
  if (swAltHoldState) txPacket.switches |= SWITCH_ALT_HOLD;
  if (swArmState) txPacket.switches |= SWITCH_ARM;
  if (!swArmState) {
    txPacket.throttle = 1000; // hard-kill throttle when the ARM switch is LOW
  }
  txPacket.buttons = 0;
  if (btnCalState) txPacket.buttons |= BUTTON_CALIBRATE;
  if (btnEscState) txPacket.buttons |= BUTTON_MOTOR_TEST;
  txPacket.aux = 0;
  finalizePacket(txPacket);
}

void processAckPayload() {
  while (radio.isAckPayloadAvailable()) {
    radio.read(&rxTelemetry, sizeof(rxTelemetry));
    if (validatePacket(rxTelemetry)) {
      telemetryValid = true;
      telemetryTimestampMs = millis();
      linkAlive = true;
      lastLinkMicros = micros();
    }
  }
}

void sendCommand() {
  buildPacket();
  bool success = radio.write(&txPacket, sizeof(txPacket));
  if (success) {
    lastLinkMicros = micros();
    linkAlive = true;
  }
  processAckPayload();
  if (!success && (micros() - lastLinkMicros > LINK_TIMEOUT_US)) {
    linkAlive = false;
  }
}

void updateLinkStatus() {
  if (static_cast<int32_t>(micros() - lastLinkMicros) > static_cast<int32_t>(LINK_TIMEOUT_US)) {
    linkAlive = false;
  }
}

void printStatus() {
  Serial.print(F("Link:"));
  Serial.print(linkAlive ? F("LINKED") : F("NO LINK"));
  Serial.print(F(" | SW2 Arm:"));
  Serial.print(swArmState ? F("ARM") : F("KILL"));
  Serial.print(F(" | SW1 Alt:"));
  Serial.print(swAltHoldState ? F("ON") : F("OFF"));
  Serial.print(F(" | Btn1 Cal:"));
  Serial.print(btnCalState ? F("PRESSED") : F("IDLE"));
  Serial.print(F(" | Btn2 ESC:"));
  Serial.print(btnEscState ? F("PRESSED") : F("IDLE"));
  if (telemetryValid) {
    Serial.print(F(" | FC Arm:"));
    Serial.print((rxTelemetry.statusFlags & FC_STATUS_ARMED) ? F("ARMED") : F("SAFE"));
    Serial.print(F(" | Cal:"));
    Serial.print((rxTelemetry.statusFlags & FC_STATUS_CALIBRATED) ? F("OK") : F("REQ"));
    Serial.print(F(" | Alt:"));
    Serial.print(rxTelemetry.altitudeCm / 100.0f, 2);
    Serial.print(F("m"));
    Serial.print(F(" | Roll:"));
    Serial.print(rxTelemetry.rollAngleDeg / 100.0f, 2);
    Serial.print(F("deg"));
    Serial.print(F(" | Age:"));
    Serial.print(millis() - telemetryTimestampMs);
    Serial.print(F("ms"));
  }
  Serial.println();
}

void setupRadio() {
  radio.begin();
  radio.setChannel(kNrfChannel);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);
  radio.enableAckPayload();
  radio.openWritingPipe(kNrfAddress);
  radio.stopListening();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SW_ALTHOLD, INPUT_PULLUP);
  pinMode(PIN_SW_ARM, INPUT_PULLUP);
  pinMode(PIN_BTN_CAL, INPUT_PULLUP);
  pinMode(PIN_BTN_ESC, INPUT_PULLUP);
  setupRadio();
  lastTxMicros = micros();
  lastStatusMs = millis();
  lastLinkMicros = micros();
}

void loop() {
  uint32_t nowMicros = micros();
  if (nowMicros - lastTxMicros >= TX_PERIOD_US) {
    lastTxMicros = nowMicros;
    sendCommand();
  }

  updateLinkStatus();

  uint32_t nowMs = millis();
  if (nowMs - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = nowMs;
    printStatus();
  }
}
