#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

const uint8_t RADIO_CE_PIN = 9;
const uint8_t RADIO_CSN_PIN = 10;
const uint8_t RADIO_CHANNEL = 108;
const uint64_t RADIO_PIPE = 0xF0F0F0F0E1LL;

const uint8_t THROTTLE_PIN = A0;  // left vertical
const uint8_t YAW_PIN = A1;       // left horizontal
const uint8_t PITCH_PIN = A2;     // right vertical
const uint8_t ROLL_PIN = A3;      // right horizontal

const uint8_t BUTTON_CAL_PIN = 4;
const uint8_t BUTTON_SMOOTH_PIN = 5;
const uint8_t SWITCH_ARM_PIN = 3;
const uint8_t SWITCH_ALT_PIN = 2;

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

struct ControlPacket {
  int16_t thrust = 1000;
  int16_t roll = 0;
  int16_t pitch = 0;
  int16_t yaw = 0;
  uint16_t id = 0;
  bool buttonCal = false;
  bool buttonSmooth = false;
  bool switchArm = true;
  bool switchAltHold = false;
};

ControlPacket packet;

Smoothed<int16_t> throttleFilter;
Smoothed<int16_t> yawFilter;
Smoothed<int16_t> pitchFilter;
Smoothed<int16_t> rollFilter;

void setupFilters() {
  throttleFilter.begin(SMOOTHED_EXPONENTIAL, 4);
  yawFilter.begin(SMOOTHED_EXPONENTIAL, 4);
  pitchFilter.begin(SMOOTHED_EXPONENTIAL, 4);
  rollFilter.begin(SMOOTHED_EXPONENTIAL, 4);
}

void setup() {
  pinMode(BUTTON_CAL_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SMOOTH_PIN, INPUT_PULLUP);
  pinMode(SWITCH_ARM_PIN, INPUT_PULLUP);
  pinMode(SWITCH_ALT_PIN, INPUT_PULLUP);

  Serial.begin(57600);

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(RADIO_CHANNEL);
  radio.openWritingPipe(RADIO_PIPE);
  radio.stopListening();

  setupFilters();
}

void loop() {
  readInputs();
  radio.write(&packet, sizeof(packet));
  printPacket();
  delay(5);
}

void readInputs() {
  int throttleRaw = 1023 - analogRead(THROTTLE_PIN);
  int yawRaw = analogRead(YAW_PIN) - 512;
  int pitchRaw = (1023 - analogRead(PITCH_PIN)) - 512;
  int rollRaw = analogRead(ROLL_PIN) - 512;

  throttleFilter.add(throttleRaw);
  yawFilter.add(yawRaw);
  pitchFilter.add(pitchRaw);
  rollFilter.add(rollRaw);

  int16_t throttleValue = map(throttleFilter.get(), 0, 1023, 1000, 2000);
  throttleValue = constrain(throttleValue, 1000, 2000);

  int16_t yawValue = map(yawFilter.get(), -512, 512, -500, 500);
  int16_t pitchValue = map(pitchFilter.get(), -512, 512, -500, 500);
  int16_t rollValue = map(rollFilter.get(), -512, 512, -500, 500);

  packet.thrust = throttleValue;
  packet.yaw = yawValue;
  packet.pitch = pitchValue;
  packet.roll = rollValue;
  packet.id++;

  packet.buttonCal = digitalRead(BUTTON_CAL_PIN) == LOW;
  packet.buttonSmooth = digitalRead(BUTTON_SMOOTH_PIN) == LOW;
  packet.switchArm = digitalRead(SWITCH_ARM_PIN);
  packet.switchAltHold = digitalRead(SWITCH_ALT_PIN) == LOW;
}

void printPacket() {
  Serial.print("T:");
  Serial.print(packet.thrust);
  Serial.print("\tR:");
  Serial.print(packet.roll);
  Serial.print("\tP:");
  Serial.print(packet.pitch);
  Serial.print("\tY:");
  Serial.println(packet.yaw);
}
