#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>
#include <math.h>

constexpr uint8_t PIN_RADIO_CE = 9;
constexpr uint8_t PIN_RADIO_CSN = 10;
constexpr uint8_t PIN_SWITCH_ALT = 2;
constexpr uint8_t PIN_SWITCH_ARM = 3;
constexpr uint8_t PIN_BUTTON_CAL = 4;
constexpr uint8_t PIN_BUTTON_SMOOTH = 5;

constexpr uint8_t PIN_THROTTLE = A0;  // Left vertical
constexpr uint8_t PIN_YAW      = A1;  // Left horizontal
constexpr uint8_t PIN_PITCH    = A2;  // Right vertical
constexpr uint8_t PIN_ROLL     = A3;  // Right horizontal

constexpr uint64_t RADIO_PIPE = 0xF0F0F0F0E1LL;
constexpr uint8_t RADIO_CHANNEL = 90;

RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);

Smoothed<float> throttleFilter;
Smoothed<float> yawFilter;
Smoothed<float> pitchFilter;
Smoothed<float> rollFilter;

struct Package {
  int   thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int   id = 0;
  bool  but1 = true;
  bool  but2 = true;
  bool  switch1 = true;
  bool  switch2 = true;
};

Package package;
int packetId = 0;

float applyDeadband(float value, float deadband) {
  if (fabsf(value) < deadband) {
    return 0.0f;
  }
  return value;
}

void setup() {
  pinMode(PIN_SWITCH_ALT, INPUT_PULLUP);
  pinMode(PIN_SWITCH_ARM, INPUT_PULLUP);
  pinMode(PIN_BUTTON_CAL, INPUT_PULLUP);
  pinMode(PIN_BUTTON_SMOOTH, INPUT_PULLUP);

  Serial.begin(57600);

  radio.begin();
  radio.setChannel(RADIO_CHANNEL);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(RADIO_PIPE);
  radio.stopListening();

  throttleFilter.begin(SMOOTHED_EXPONENTIAL, 4);
  yawFilter.begin(SMOOTHED_EXPONENTIAL, 4);
  pitchFilter.begin(SMOOTHED_EXPONENTIAL, 4);
  rollFilter.begin(SMOOTHED_EXPONENTIAL, 4);
}

void loop() {
  readJoysticks();
  package.id = packetId++;
  package.but1 = digitalRead(PIN_BUTTON_CAL);
  package.but2 = digitalRead(PIN_BUTTON_SMOOTH);
  package.switch1 = digitalRead(PIN_SWITCH_ARM);
  package.switch2 = digitalRead(PIN_SWITCH_ALT);

  radio.write(&package, sizeof(package));
  printPackage();
  delay(5);
}

void readJoysticks() {
  const float throttleRaw = 1023 - analogRead(PIN_THROTTLE);
  const float pitchRaw = 1023 - analogRead(PIN_PITCH);
  const float yawRaw = analogRead(PIN_YAW);
  const float rollRaw = analogRead(PIN_ROLL);

  throttleFilter.add(throttleRaw);
  pitchFilter.add(pitchRaw);
  yawFilter.add(yawRaw);
  rollFilter.add(rollRaw);

  const float throttleValue = throttleFilter.get();
  const float yawValue = yawFilter.get() - 512.0f;
  const float pitchValue = pitchFilter.get() - 512.0f;
  const float rollValue = rollFilter.get() - 512.0f;

  const float throttleNorm = constrain(throttleValue, 0.0f, 1023.0f) / 1023.0f;
  package.thrust = 1000 + static_cast<int>(throttleNorm * 1000.0f);
  package.z = applyDeadband(yawValue, 12.0f);
  package.x = applyDeadband(rollValue, 12.0f);
  package.y = applyDeadband(pitchValue, 12.0f);
}

void printPackage() {
  Serial.print(package.thrust);
  Serial.print('\t');
  Serial.print(package.z);
  Serial.print('\t');
  Serial.print(package.x);
  Serial.print('\t');
  Serial.println(package.y);
}
