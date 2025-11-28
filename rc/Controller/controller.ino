#include <SPI.h>
#include <RF24.h>
#include <Smoothed.h>
#include <math.h>

RF24 radio(9, 10);
const uint64_t pipe = 0xF0F0F0F0E1LL;
const uint8_t RF_CHANNEL = 108;

const uint8_t PIN_SWITCH_ARM = 3;   // D3
const uint8_t PIN_SWITCH_ALT = 2;   // D2
const uint8_t PIN_BUTTON_CAL = 4;   // D4
const uint8_t PIN_BUTTON_SMOOTH = 5; // D5

const uint8_t PIN_THROTTLE = A0;
const uint8_t PIN_YAW = A1;
const uint8_t PIN_PITCH = A2;
const uint8_t PIN_ROLL = A3;

Smoothed<float> throttleFilter;
Smoothed<float> yawFilter;
Smoothed<float> pitchFilter;
Smoothed<float> rollFilter;

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
int packetId = 0;

struct AxisConfig {
  float scale;
  int16_t offset;
  uint8_t deadband;
  bool invert;
};

const AxisConfig yawConfig{0.12f, 0, 8, false};
const AxisConfig pitchConfig{0.12f, 0, 8, true};
const AxisConfig rollConfig{0.12f, 0, 8, false};
const float THROTTLE_SCALE = (2000.0f - 1000.0f) / 1023.0f;

float sampleAxis(uint8_t pin, Smoothed<float>& filter) {
  filter.add(analogRead(pin));
  return filter.get();
}

float shapeAxis(float raw, const AxisConfig& cfg) {
  float centered = raw - 512.0f + cfg.offset;
  if (cfg.invert) {
    centered = -centered;
  }
  if (fabs(centered) < cfg.deadband) {
    centered = 0;
  }
  return centered * cfg.scale;
}

void readInputs() {
  float throttleRaw = sampleAxis(PIN_THROTTLE, throttleFilter);
  float yawRaw = sampleAxis(PIN_YAW, yawFilter);
  float pitchRaw = sampleAxis(PIN_PITCH, pitchFilter);
  float rollRaw = sampleAxis(PIN_ROLL, rollFilter);

  int throttleValue = static_cast<int>((1023.0f - throttleRaw) * THROTTLE_SCALE) + 1000;
  throttleValue = constrain(throttleValue, 1000, 2000);

  package.thrust = throttleValue;
  package.z = shapeAxis(yawRaw, yawConfig);
  package.x = shapeAxis(rollRaw, rollConfig);
  package.y = shapeAxis(pitchRaw, pitchConfig);

  package.but1 = digitalRead(PIN_BUTTON_CAL);
  package.but2 = digitalRead(PIN_BUTTON_SMOOTH);
  package.switch1 = digitalRead(PIN_SWITCH_ARM);
  package.switch2 = digitalRead(PIN_SWITCH_ALT);
  package.id = packetId++;
}

void printPackage() {
  Serial.print("thr: ");
  Serial.print(package.thrust);
  Serial.print("\t x: ");
  Serial.print(package.x);
  Serial.print("\t y: ");
  Serial.print(package.y);
  Serial.print("\t z: ");
  Serial.print(package.z);
  Serial.print("\t arm: ");
  Serial.print(package.switch1);
  Serial.print(" alt: ");
  Serial.println(package.switch2);
}

void setup() {
  pinMode(PIN_SWITCH_ARM, INPUT_PULLUP);
  pinMode(PIN_SWITCH_ALT, INPUT_PULLUP);
  pinMode(PIN_BUTTON_CAL, INPUT_PULLUP);
  pinMode(PIN_BUTTON_SMOOTH, INPUT_PULLUP);

  Serial.begin(57600);

  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(RF_CHANNEL);
  radio.openWritingPipe(pipe);
  radio.stopListening();

  throttleFilter.begin(SMOOTHED_EXPONENTIAL, 5);
  yawFilter.begin(SMOOTHED_EXPONENTIAL, 5);
  pitchFilter.begin(SMOOTHED_EXPONENTIAL, 5);
  rollFilter.begin(SMOOTHED_EXPONENTIAL, 5);
}

void loop() {
  readInputs();
  radio.write(&package, sizeof(package));
  printPackage();
  delay(5);
}
