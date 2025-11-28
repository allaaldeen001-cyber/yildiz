#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

RF24 radio(9, 10);
const uint64_t pipe = 0xF0F0F0F0E1LL;

const uint8_t ALT_HOLD_SWITCH_PIN = 2; // Switch 2
const uint8_t ARM_SWITCH_PIN = 3;      // Switch 1
const uint8_t CAL_BUTTON_PIN = 4;      // Button 1
const uint8_t SMOOTH_BUTTON_PIN = 5;   // Button 2

const uint8_t THROTTLE_PIN = A0; // Left vertical
const uint8_t YAW_PIN = A1;      // Left horizontal
const uint8_t PITCH_PIN = A2;    // Right vertical
const uint8_t ROLL_PIN = A3;     // Right horizontal

Smoothed<float> smoothThrottle;
Smoothed<float> smoothYaw;
Smoothed<float> smoothPitch;
Smoothed<float> smoothRoll;

struct Package {
  int   thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int   id = 0;
  bool  but1 = 1;
  bool  but2 = 1;
  bool  switch1 = 1;
  bool  switch2 = 1;
};

struct AckPayload {
  uint16_t batteryMv = 0;
  float    baroAltitude = 0;
  bool     armed = false;
  bool     altHold = false;
  bool     link = false;
};

Package package;
AckPayload ackPayload;

bool linkOk = false;
unsigned long lastSend = 0;
const unsigned long sendIntervalMs = 5;

void setup() {
  pinMode(ALT_HOLD_SWITCH_PIN, INPUT_PULLUP);
  pinMode(ARM_SWITCH_PIN, INPUT_PULLUP);
  pinMode(CAL_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SMOOTH_BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(57600);

  if (!radio.begin()) {
    Serial.println("NRF init failed");
    while (true) {}
  }

  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(3, 15);
  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipe);
  radio.stopListening();

  smoothThrottle.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothYaw.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothPitch.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothRoll.begin(SMOOTHED_EXPONENTIAL, 5);
}

void loop() {
  if (millis() - lastSend < sendIntervalMs) {
    return;
  }
  lastSend = millis();

  readInputs();
  bool success = radio.write(&package, sizeof(package));
  linkOk = success;

  if (radio.isAckPayloadAvailable()) {
    radio.read(&ackPayload, sizeof(ackPayload));
  }

  printPackage();
}

void readInputs() {
  package.id++;

  int rawThrottle = 1023 - analogRead(THROTTLE_PIN);
  int rawYaw = analogRead(YAW_PIN);
  int rawPitch = 1023 - analogRead(PITCH_PIN);
  int rawRoll = analogRead(ROLL_PIN);

  smoothThrottle.add(rawThrottle);
  smoothYaw.add(rawYaw);
  smoothPitch.add(rawPitch);
  smoothRoll.add(rawRoll);

  long throttleValue = static_cast<long>(smoothThrottle.get());
  long yawValue = static_cast<long>(smoothYaw.get() - 512.0f);
  long pitchValue = static_cast<long>(smoothPitch.get() - 512.0f);
  long rollValue = static_cast<long>(smoothRoll.get() - 512.0f);

  package.thrust = constrain(map(throttleValue, 0, 1023, 1000, 2000), 1000, 2000);
  package.z = constrain(map(yawValue, -512, 512, -250, 250), -250, 250);
  package.y = constrain(map(pitchValue, -512, 512, -70, 70), -70, 70);
  package.x = constrain(map(rollValue, -512, 512, -70, 70), -70, 70);

  package.but1 = digitalRead(CAL_BUTTON_PIN);
  package.but2 = digitalRead(SMOOTH_BUTTON_PIN);
  package.switch1 = digitalRead(ARM_SWITCH_PIN);
  package.switch2 = digitalRead(ALT_HOLD_SWITCH_PIN);
}

void printPackage() {
  Serial.print("T:");
  Serial.print(package.thrust);
  Serial.print("\tR:");
  Serial.print(package.x);
  Serial.print("\tP:");
  Serial.print(package.y);
  Serial.print("\tY:");
  Serial.print(package.z);
  Serial.print("\tArm:");
  Serial.print(package.switch1 == 0 ? "ON" : "OFF");
  Serial.print("\tAltHold:");
  Serial.print(package.switch2 == 0 ? "ON" : "OFF");
  Serial.print("\tCal:");
  Serial.print(package.but1 == 0 ? "HOLD" : "-");
  Serial.print("\tSoft:");
  Serial.print(package.but2 == 0 ? "HOLD" : "-");
  Serial.print("\tLink:");
  Serial.print(linkOk ? "OK" : "LOST");

  Serial.print("\tACK batt:");
  Serial.print(ackPayload.batteryMv);
  Serial.print("mV");
  Serial.print("\tAlt:");
  Serial.print(ackPayload.baroAltitude);
  Serial.print("\tArmed:");
  Serial.print(ackPayload.armed ? "Y" : "N");
  Serial.print("\tAltHold:");
  Serial.print(ackPayload.altHold ? "Y" : "N");
  Serial.println();
}
