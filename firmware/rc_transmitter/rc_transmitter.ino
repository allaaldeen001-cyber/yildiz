/**
 * Arduino Nano Quadcopter RC Transmitter
 *
 * Hardware:
 *  - Arduino Nano
 *  - NRF24L01 (CE=D8, CSN=D9, MOSI=D11, MISO=D12, SCK=D13)
 *  - Nokia 5110 (software SPI on A4/A5 + D5-D7)
 *  - Two 2-axis joysticks on A0-A3
 *  - Toggle switch on D4 (kill / arm)
 *  - Button 1 on D2 (IMU/ESC calibration request)
 *  - Button 2 on D3 (idle spin request)
 */

#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <math.h>

#include "../common/config.h"

// --------------- Pin Mapping ----------------
constexpr uint8_t BUTTON1_PIN = 2;
constexpr uint8_t BUTTON2_PIN = 3;
constexpr uint8_t TOGGLE_PIN = 4;
constexpr uint8_t LCD_DC_PIN = 5;
constexpr uint8_t LCD_CS_PIN = 6;
constexpr uint8_t LCD_RST_PIN = 7;
constexpr uint8_t NRF_CE_PIN = 8;
constexpr uint8_t NRF_CSN_PIN = 9;
constexpr uint8_t LCD_SCLK_PIN = A4;
constexpr uint8_t LCD_DIN_PIN = A5;
constexpr uint8_t STICK_PINS[RC_CHANNEL_COUNT] = {A0, A1, A2, A3};  // Throttle, Yaw, Pitch, Roll

constexpr uint32_t TX_INTERVAL_US = 10000;  // 100 Hz uplink
constexpr uint16_t STICK_CAL_MAGIC = 0x5AC1;

Adafruit_PCD8544 display(LCD_SCLK_PIN, LCD_DIN_PIN, LCD_DC_PIN, LCD_CS_PIN, LCD_RST_PIN);
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// --------------- Stick Calibration Storage ---------------
struct StickCalAxis {
  uint16_t min;
  uint16_t center;
  uint16_t max;
};

struct StickCalStore {
  uint16_t magic;
  StickCalAxis axes[RC_CHANNEL_COUNT];
  uint8_t checksum;
};

StickCalStore stickCal{};
bool stickCalValid = false;
float stickFiltered[RC_CHANNEL_COUNT] = {512.0f, 512.0f, 512.0f, 512.0f};

uint8_t checksumStick(const StickCalStore &store) {
  const uint8_t *raw = reinterpret_cast<const uint8_t *>(&store);
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(StickCalStore) - 1; ++i) {
    sum ^= raw[i];
  }
  return sum;
}

void saveStickCal() {
  stickCal.magic = STICK_CAL_MAGIC;
  stickCal.checksum = checksumStick(stickCal);
  EEPROM.put(0, stickCal);
  stickCalValid = true;
}

void loadStickCal() {
  EEPROM.get(0, stickCal);
  if (stickCal.magic != STICK_CAL_MAGIC) {
    stickCalValid = false;
    return;
  }
  if (checksumStick(stickCal) != stickCal.checksum) {
    stickCalValid = false;
    return;
  }
  stickCalValid = true;
}

// --------------- Utility Functions ---------------
float clampFloat(float value, float minVal, float maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

float applyDeadband(float value, float deadband) {
  if (fabs(value) < deadband) {
    return 0.0f;
  }
  if (value > 0.0f) {
    return (value - deadband) / (1.0f - deadband);
  }
  return (value + deadband) / (1.0f - deadband);
}

float applyExpo(float value, float expo) {
  return value * (1.0f - expo) + value * value * value * expo;
}

int readAxisRaw(uint8_t axis) {
  int raw = analogRead(STICK_PINS[axis]);
  stickFiltered[axis] = (stickFiltered[axis] * 0.7f) + (raw * 0.3f);
  return static_cast<int>(stickFiltered[axis] + 0.5f);
}

float normalizeAxis(uint8_t axisIndex, int raw) {
  const StickCalAxis &cal = stickCal.axes[axisIndex];
  int span = (raw >= static_cast<int>(cal.center))
                 ? static_cast<int>(cal.max) - static_cast<int>(cal.center)
                 : static_cast<int>(cal.center) - static_cast<int>(cal.min);
  if (span < 1) span = 1;
  float normalized = (static_cast<float>(raw) - cal.center) / span;
  normalized = clampFloat(normalized, -1.0f, 1.0f);
  normalized = applyDeadband(normalized, 0.02f);
  normalized = applyExpo(normalized, 0.2f);
  return normalized;
}

uint16_t mapBipolarAxis(uint8_t axisIndex, int raw) {
  if (!stickCalValid) {
    return 1500;
  }
  float normalized = normalizeAxis(axisIndex, raw);
  float pwm = 1500.0f + (normalized * 500.0f);
  return static_cast<uint16_t>(constrain(pwm, 1000.0f, 2000.0f));
}

uint16_t mapThrottleAxis(int raw, bool killEngaged) {
  if (!stickCalValid) {
    return 1000;
  }
  float normalized = normalizeAxis(0, raw);
  float pwm = 1500.0f + (normalized * 500.0f);
  uint16_t throttle = static_cast<uint16_t>(constrain(pwm, 1000.0f, 2000.0f));
  if (killEngaged) {
    throttle = 1000;
  }
  return throttle;
}

// --------------- Guide Messages ---------------
GuideStep latestGuideStep = GuideStep::WAIT_LINK;
const char *guideShortLabel(GuideStep step) {
  switch (step) {
    case GuideStep::WAIT_LINK: return "Link";
    case GuideStep::REQUEST_KILL: return "Kill";
    case GuideStep::REQUEST_CAL: return "Cal";
    case GuideStep::CALIBRATING: return "Cal..";
    case GuideStep::REQUEST_ARM: return "Arm";
    case GuideStep::REQUEST_IDLE_SPIN: return "Idle";
    case GuideStep::READY: return "Ready";
    case GuideStep::FLYING: return "Fly";
    default: return "--";
  }
}

const char *guideInstruction(GuideStep step) {
  switch (step) {
    case GuideStep::WAIT_LINK: return "Power FC+RC";
    case GuideStep::REQUEST_KILL: return "Toggle->KILL";
    case GuideStep::REQUEST_CAL: return "BTN1 -> CAL";
    case GuideStep::CALIBRATING: return "Calibrating...";
    case GuideStep::REQUEST_ARM: return "Toggle->ARM";
    case GuideStep::REQUEST_IDLE_SPIN: return "BTN2 -> Idle";
    case GuideStep::READY: return "Ease throttle";
    case GuideStep::FLYING: return "Fly safe & chk";
    default: return "";
  }
}

// --------------- Radio / Telemetry ---------------
TelemetryPacket telemetry{};
bool telemetryValid = false;
bool linkAlive = false;
uint32_t lastAckMillis = 0;
uint32_t lastTxMicros = 0;
ControlPacket lastPacketSent = {
    PACKET_VERSION,
    1000, 1500, 1500, 1500,
    0, 0, 0, 0,
    RADIO_CHANNEL,
    0};

void updateLinkStatus(bool frameOk) {
  if (frameOk) {
    linkAlive = true;
    lastAckMillis = millis();
  } else if (millis() - lastAckMillis > 350) {
    linkAlive = false;
    telemetryValid = false;
  }
}

void readTelemetryAck() {
  while (radio.isAckPayloadAvailable()) {
    radio.read(&telemetry, sizeof(telemetry));
    if (telemetry.version == PACKET_VERSION && isChecksumValid(telemetry)) {
      telemetryValid = true;
      latestGuideStep = static_cast<GuideStep>(telemetry.guideStep);
      updateLinkStatus(true);
    }
  }
}

ControlPacket buildControlPacket() {
  ControlPacket packet{};
  packet.version = PACKET_VERSION;

  bool killEngaged = digitalRead(TOGGLE_PIN) == LOW;
  int rawThrottle = readAxisRaw(0);
  int rawYaw = readAxisRaw(1);
  int rawPitch = readAxisRaw(2);
  int rawRoll = readAxisRaw(3);

  packet.throttle = mapThrottleAxis(rawThrottle, killEngaged);
  packet.yaw = mapBipolarAxis(1, rawYaw);
  packet.pitch = mapBipolarAxis(2, rawPitch);
  packet.roll = mapBipolarAxis(3, rawRoll);
  packet.button1 = digitalRead(BUTTON1_PIN) == LOW ? 1 : 0;
  packet.button2 = digitalRead(BUTTON2_PIN) == LOW ? 1 : 0;
  packet.killSwitch = killEngaged ? 0 : 1;
  packet.reserved = stickCalValid ? 1 : 0;
  packet.channel = RADIO_CHANNEL;
  packet.checksum = computeChecksum(packet);
  return packet;
}

void transmitFrame() {
  ControlPacket packet = buildControlPacket();
  bool success = radio.write(&packet, sizeof(packet));
  lastPacketSent = packet;
  if (success) {
    readTelemetryAck();
  }
  updateLinkStatus(success);
}

// --------------- Display -----------------
uint32_t lastDisplayUpdate = 0;

void drawHeader() {
  display.setCursor(0, 0);
  display.print("NRF:");
  display.print(linkAlive ? "OK " : "NO ");
  display.print("CH");
  display.print(RADIO_CHANNEL);
  display.setCursor(0, 8);
  display.print("Step:");
  display.print(guideShortLabel(latestGuideStep));
}

void drawInstruction() {
  display.setCursor(0, 16);
  display.print(guideInstruction(latestGuideStep));
}

void drawTelemetryData(const ControlPacket &packet) {
  display.setCursor(0, 24);
  display.print("Thr:");
  display.print(packet.throttle);
  display.print(" Y:");
  display.print(packet.yaw);

  display.setCursor(0, 32);
  display.print("P:");
  display.print(packet.pitch);
  display.print(" R:");
  display.print(packet.roll);

  display.setCursor(0, 40);
  display.print("Alt:");
  if (telemetryValid) {
    char altBuf[8];
    dtostrf(telemetry.altitudeMeters, 4, 1, altBuf);
    display.print(altBuf);
  } else {
    display.print("---");
  }
  display.print(" Sw:");
  display.print(packet.killSwitch == 0 ? "KILL" : "ARM");
}

void updateDisplay(const ControlPacket &packet) {
  uint32_t now = millis();
  if (now - lastDisplayUpdate < 80) {
    return;
  }
  lastDisplayUpdate = now;
  display.clearDisplay();
  drawHeader();
  drawInstruction();
  drawTelemetryData(packet);
  display.display();
}

// --------------- Stick Calibration Wizard ---------------
void promptLine(const char *line1, const char *line2) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(line1);
  display.setCursor(0, 10);
  display.print(line2);
  display.display();
}

void collectStickCenters() {
  promptLine("Hold sticks", "centered...");
  for (uint8_t axis = 0; axis < RC_CHANNEL_COUNT; ++axis) {
    long sum = 0;
    const uint16_t samples = 400;
    for (uint16_t i = 0; i < samples; ++i) {
      sum += analogRead(STICK_PINS[axis]);
      delay(4);
    }
    stickCal.axes[axis].center = sum / samples;
    stickCal.axes[axis].min = stickCal.axes[axis].center;
    stickCal.axes[axis].max = stickCal.axes[axis].center;
  }
}

void collectStickExtremes() {
  promptLine("Move sticks", "to all corners");
  uint32_t start = millis();
  while (millis() - start < 6000) {
    for (uint8_t axis = 0; axis < RC_CHANNEL_COUNT; ++axis) {
      int value = analogRead(STICK_PINS[axis]);
      if (value < stickCal.axes[axis].min) stickCal.axes[axis].min = value;
      if (value > stickCal.axes[axis].max) stickCal.axes[axis].max = value;
    }
    delay(5);
  }
}

void finalizeStickCal() {
  for (uint8_t axis = 0; axis < RC_CHANNEL_COUNT; ++axis) {
    if (stickCal.axes[axis].max <= stickCal.axes[axis].min + 10) {
      stickCal.axes[axis].min = 100;
      stickCal.axes[axis].max = 900;
      stickCal.axes[axis].center = 512;
    }
  }
  saveStickCal();
  promptLine("Stick cal saved", "Release buttons");
  delay(1200);
}

void runStickCalibrationWizard() {
  collectStickCenters();
  collectStickExtremes();
  finalizeStickCal();
}

// --------------- Setup / Loop ---------------
void initDisplay() {
  display.begin();
  display.setContrast(60);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.display();
}

void initRadio() {
  radio.begin();
  radio.setChannel(RADIO_CHANNEL);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(3, 5);
  radio.openWritingPipe(RADIO_PIPE);
  radio.stopListening();
}

void configureInputs() {
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(TOGGLE_PIN, INPUT_PULLUP);
}

void setup() {
  configureInputs();
  initDisplay();
  loadStickCal();

  bool calibrationRequested = (digitalRead(BUTTON1_PIN) == LOW) && (digitalRead(BUTTON2_PIN) == LOW);
  if (!stickCalValid || calibrationRequested) {
    runStickCalibrationWizard();
  }

  initRadio();
  lastTxMicros = micros();
  telemetryValid = false;
  latestGuideStep = GuideStep::WAIT_LINK;
  promptLine("RC Ready", "Connecting...");
  delay(500);
}

void loop() {
  uint32_t nowMicros = micros();
  if (nowMicros - lastTxMicros >= TX_INTERVAL_US) {
    lastTxMicros = nowMicros;
    transmitFrame();
  }
  updateDisplay(lastPacketSent);
}
