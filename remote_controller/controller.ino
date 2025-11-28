#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

const uint8_t NRF_CHANNEL = 108;
const uint64_t pipe = 0xF0F0F0F0E1LL;

const uint8_t SWITCH_ALT_PIN = 2;
const uint8_t SWITCH_ARM_PIN = 3;
const uint8_t BUTTON_CAL_PIN = 4;
const uint8_t BUTTON_START_PIN = 5;

const uint8_t THROTTLE_PIN = A0;
const uint8_t YAW_PIN = A1;
const uint8_t PITCH_PIN = A2;
const uint8_t ROLL_PIN = A3;

const int ANALOG_MAX = 1023;
const int ANALOG_CENTER = 512;
const int THRUST_MIN = 1000;
const int THRUST_MAX = 2000;

const float AXIS_SCALE = 0.6f;

RF24 radio(9, 10);

Smoothed<float> throttleFilter;
Smoothed<float> yawFilter;
Smoothed<float> pitchFilter;
Smoothed<float> rollFilter;

int packetId = 0;
bool but1 = true;
bool but2 = true;
bool switch1 = true;
bool switch2 = true;

struct Package {
  int thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int id = 0;
  bool but1 = 1;
  bool but2 = 1;
  bool switch1 = 1;
  bool switch2 = 1;
};

Package package;

void setup() {
  pinMode(SWITCH_ALT_PIN, INPUT_PULLUP);
  pinMode(SWITCH_ARM_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CAL_PIN, INPUT_PULLUP);
  pinMode(BUTTON_START_PIN, INPUT_PULLUP);

  Serial.begin(57600);

  radio.begin();
  radio.setAutoAck(false);
  radio.setChannel(NRF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipe);
  radio.stopListening();

  throttleFilter.begin(SMOOTHED_EXPONENTIAL, 5);
  yawFilter.begin(SMOOTHED_EXPONENTIAL, 5);
  pitchFilter.begin(SMOOTHED_EXPONENTIAL, 5);
  rollFilter.begin(SMOOTHED_EXPONENTIAL, 5);
}

void loop() {
  readControls();
  package.id = packetId++;
  package.but1 = but1;
  package.but2 = but2;
  package.switch1 = switch1;
  package.switch2 = switch2;
  radio.write(&package, sizeof(package));
  printPackage();
  delay(5);
}

void readControls() {
  but1 = digitalRead(BUTTON_CAL_PIN);
  but2 = digitalRead(BUTTON_START_PIN);
  switch1 = digitalRead(SWITCH_ARM_PIN);
  switch2 = digitalRead(SWITCH_ALT_PIN);

  throttleFilter.add(analogRead(THROTTLE_PIN));
  yawFilter.add(analogRead(YAW_PIN));
  pitchFilter.add(analogRead(PITCH_PIN));
  rollFilter.add(analogRead(ROLL_PIN));

  const float rawThrottle = throttleFilter.get();
  const float rawYaw = yawFilter.get();
  const float rawPitch = pitchFilter.get();
  const float rawRoll = rollFilter.get();

  const float throttleInverted = ANALOG_MAX - rawThrottle;
  const float throttleRatio = throttleInverted / ANALOG_MAX;
  package.thrust = constrain(static_cast<int>(THRUST_MIN + throttleRatio * (THRUST_MAX - THRUST_MIN)), THRUST_MIN, THRUST_MAX);

  const float yawValue = (rawYaw - ANALOG_CENTER) * AXIS_SCALE;
  const float pitchValue = ((ANALOG_MAX - rawPitch) - ANALOG_CENTER) * AXIS_SCALE;
  const float rollValue = (rawRoll - ANALOG_CENTER) * AXIS_SCALE;

  package.x = rollValue;
  package.y = pitchValue;
  package.z = yawValue;
}

void printPackage() {
  Serial.print("Thrust:");
  Serial.print(package.thrust);
  Serial.print("\tX:");
  Serial.print(package.x);
  Serial.print("\tY:");
  Serial.print(package.y);
  Serial.print("\tZ:");
  Serial.println(package.z);
}
