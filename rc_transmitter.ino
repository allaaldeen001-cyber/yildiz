#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Smoothed.h>

/*
 * RC TRANSMITTER (Arduino Nano)
 * Improvements:
 *  - Automatic stick centering calibration on startup
 *  - Throttle direction fix with runtime range capture
 *  - Exponential smoothing on all axes (Smoothed lib)
 *  - Rich serial telemetry with link statistics
 *  - Linear throttle mapping to 1000-2000µs window
 */

// NRF24 CE / CSN pins
RF24 radio(9, 10);
const uint64_t pipe = 0xF0F0F0F0E1LL;

// Analog pins
constexpr uint8_t XL_pin = A1;  // Roll (Left stick X)
constexpr uint8_t YL_pin = A0;  // Throttle (Left stick Y)
constexpr uint8_t XR_pin = A3;  // Yaw (Right stick X)
constexpr uint8_t YR_pin = A2;  // Pitch (Right stick Y)

// Digital inputs
constexpr uint8_t SWITCH2_PIN = 2;
constexpr uint8_t SWITCH1_PIN = 3;
constexpr uint8_t BUTTON1_PIN = 4;
constexpr uint8_t BUTTON2_PIN = 5;

struct AxisCalibration {
  float scale = 0.10f;
  float offset = 0.0f;
  int16_t center = 512;
  uint8_t deadband = 6;
};

struct ThrottleCalibration {
  int16_t minRaw = 0;
  int16_t maxRaw = 1023;
  bool reversed = true;  // Fix: stick up -> higher throttle
};

AxisCalibration rollCal;
AxisCalibration pitchCal;
AxisCalibration yawCal;
AxisCalibration auxCal;  // extra channel (not used but kept for completeness)
ThrottleCalibration throttleCal;

// Smoothed filters
Smoothed<float> smoothRoll;
Smoothed<float> smoothPitch;
Smoothed<float> smoothYaw;
Smoothed<float> smoothAux;
Smoothed<float> smoothThrottle;

struct Package {
  int thrust = 0;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  int id = 0;
  bool but1 = true;
  bool but2 = true;
  bool switch1 = true;
  bool switch2 = true;
};

Package package;

// Telemetry
uint32_t packetId = 0;
uint32_t packetsSent = 0;
uint32_t packetsAcked = 0;
unsigned long lastTelemetryPrint = 0;

// Forward declarations
void readInputs();
void calibrateSticks();
int applyAxisCalibration(float raw, const AxisCalibration &cal);
int mapThrottle(int raw);
void printTelemetry(bool linkOk);

void setup() {
  pinMode(SWITCH2_PIN, INPUT_PULLUP);
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  Serial.begin(57600);
  while (!Serial) {
    ;  // Wait for serial
  }
  Serial.println(F("== RC Tx Boot =="));

  radio.begin();
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.setRetries(5, 15);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipe);
  radio.stopListening();

  smoothRoll.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothPitch.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothYaw.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothAux.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothThrottle.begin(SMOOTHED_EXPONENTIAL, 8);

  calibrateSticks();
  Serial.println(F("RC Ready - Channel 108 - PA LOW"));
}

void loop() {
  readInputs();

  package.x = applyAxisCalibration(smoothYaw.get(), yawCal);
  package.y = applyAxisCalibration(smoothPitch.get(), pitchCal);
  package.z = applyAxisCalibration(smoothRoll.get(), rollCal);
  package.thrust = mapThrottle(static_cast<int>(smoothThrottle.get()));
  package.thrust = constrain(package.thrust, 1000, 2000);

  package.id = packetId++;
  package.but1 = digitalRead(BUTTON1_PIN);
  package.but2 = digitalRead(BUTTON2_PIN);
  package.switch1 = digitalRead(SWITCH1_PIN);
  package.switch2 = digitalRead(SWITCH2_PIN);

  packagesSent++;
  bool ok = radio.write(&package, sizeof(package));
  if (ok) {
    packetsAcked++;
  }

  if (millis() - lastTelemetryPrint > 100) {
    printTelemetry(ok);
    lastTelemetryPrint = millis();
  }

  delay(5);
}

void readInputs() {
  smoothYaw.add(analogRead(XR_pin));
  smoothRoll.add(analogRead(XL_pin));
  smoothPitch.add(1023 - analogRead(YR_pin));  // invert pitch
  smoothThrottle.add(analogRead(YL_pin));
}

void calibrateSticks() {
  Serial.println(F("Center all sticks, throttle LOW. Hold still..."));
  delay(1200);

  const uint16_t samples = 400;
  long rollSum = 0, pitchSum = 0, yawSum = 0, auxSum = 0;

  for (uint16_t i = 0; i < samples; ++i) {
    rollSum += analogRead(XL_pin);
    pitchSum += 1023 - analogRead(YR_pin);
    yawSum += analogRead(XR_pin);
    auxSum += analogRead(A6);  // free analog pin for expansion
    delay(2);
  }

  rollCal.center = rollSum / samples;
  pitchCal.center = pitchSum / samples;
  yawCal.center = yawSum / samples;
  auxCal.center = auxSum / samples;

  Serial.print(F("Centers -> Roll: "));
  Serial.print(rollCal.center);
  Serial.print(F(" Pitch: "));
  Serial.print(pitchCal.center);
  Serial.print(F(" Yaw: "));
  Serial.println(yawCal.center);

  Serial.println(F("Sweep throttle (low -> high -> low)..."));
  throttleCal.minRaw = 1023;
  throttleCal.maxRaw = 0;
  unsigned long start = millis();
  while (millis() - start < 3000) {
    int raw = analogRead(YL_pin);
    if (raw < throttleCal.minRaw) throttleCal.minRaw = raw;
    if (raw > throttleCal.maxRaw) throttleCal.maxRaw = raw;
    delay(5);
  }

  if (throttleCal.maxRaw - throttleCal.minRaw < 150) {
    throttleCal.minRaw = 0;
    throttleCal.maxRaw = 1023;
    Serial.println(F("Throttle range defaulted (movement not detected)"));
  }

  Serial.print(F("Throttle range -> "));
  Serial.print(throttleCal.minRaw);
  Serial.print(F(" - "));
  Serial.println(throttleCal.maxRaw);
  Serial.println(F("Throttle direction: up = faster"));
}

int applyAxisCalibration(float raw, const AxisCalibration &cal) {
  int centered = static_cast<int>(raw) - cal.center;
  if (abs(centered) < cal.deadband) {
    centered = 0;
  }
  return centered * cal.scale + cal.offset;
}

int mapThrottle(int raw) {
  int clamped = constrain(raw, throttleCal.minRaw, throttleCal.maxRaw);
  if (throttleCal.reversed) {
    clamped = throttleCal.maxRaw - (clamped - throttleCal.minRaw);
  }
  long mapped = map(clamped,
                    throttleCal.minRaw,
                    throttleCal.maxRaw,
                    1000,
                    2000);
  return static_cast<int>(mapped);
}

void printTelemetry(bool linkOk) {
  Serial.print(F("T:"));
  Serial.print(package.thrust);
  Serial.print(F(" | Roll:"));
  Serial.print(package.z, 2);
  Serial.print(F(" | Pitch:"));
  Serial.print(package.y, 2);
  Serial.print(F(" | Yaw:"));
  Serial.print(package.x, 2);
  Serial.print(F(" | Link: "));
  Serial.print(linkOk ? F("OK") : F("LOST"));

  if (packetsSent > 0) {
    float success = (static_cast<float>(packetsAcked) / packetsSent) * 100.0f;
    Serial.print(F(" | Ack%: "));
    Serial.print(success, 1);
  }
  Serial.print(F(" | ID: "));
  Serial.println(package.id);
}
