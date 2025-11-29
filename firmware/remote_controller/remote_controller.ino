/*
 * Professional quadcopter remote controller firmware for Arduino Nano.
 *
 * Hardware summary:
 *  - NRF24L01+ PA/LNA (CE -> D9, CSN -> D10) configured on RF channel 103.
 *  - Two analog joysticks (Throttle/Yaw on A0/A1, Pitch/Roll on A2/A3).
 *  - Push buttons: BTN1(D4) for IMU calibration, BTN2(D5) for arming/ESC calibration.
 *  - Toggle switches: SW1(D2) altitude-hold latch, SW2(D3) arming/kill switch.
 *  - Serial monitor works as a miniature GCS for status and telemetry feedback.
 */

#include <SPI.h>
#include <RF24.h>

// -------------------- Pin definitions --------------------
constexpr uint8_t PIN_RADIO_CE = 9;
constexpr uint8_t PIN_RADIO_CSN = 10;

constexpr uint8_t PIN_BTN_CAL = 4;
constexpr uint8_t PIN_BTN_ARM = 5;
constexpr uint8_t PIN_SWITCH_ALT = 2;
constexpr uint8_t PIN_SWITCH_KILL = 3;

constexpr uint8_t PIN_LED_LINK = LED_BUILTIN;

constexpr uint8_t PIN_STICK_THROTTLE = A0;
constexpr uint8_t PIN_STICK_YAW = A1;
constexpr uint8_t PIN_STICK_PITCH = A2;
constexpr uint8_t PIN_STICK_ROLL = A3;

// -------------------- Radio configuration --------------------
const byte PIPE_FC[6] = "FC103";  // writing pipe (flight controller)
const byte PIPE_RC[6] = "RC103";  // reading pipe for ACK payloads
constexpr uint8_t RADIO_CHANNEL = 103;

// -------------------- Protocol structures --------------------
enum ControlFlags : uint8_t {
  FLAG_ALT_HOLD   = 1 << 0,
  FLAG_KILL       = 1 << 1,
  FLAG_CALIBRATE  = 1 << 2,
  FLAG_ARM_BTN    = 1 << 3,
  FLAG_ESC_CAL    = 1 << 4
};

struct __attribute__((packed)) ControlFrame {
  uint16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
  uint8_t flags;
  uint8_t reserved;
  uint16_t frameId;
};

struct __attribute__((packed)) TelemetryFrame {
  uint16_t packetMisses;
  uint16_t loopTimeUs;
  uint8_t state;
  uint8_t faultFlags;
  uint8_t imuCalibrated;
  uint8_t escCalibrated;
  int8_t batteryMvHundreds;
};

// -------------------- Globals --------------------
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);
ControlFrame frame = {};
TelemetryFrame telemetry = {};

uint16_t frameCounter = 0;
uint32_t lastAckMs = 0;
uint32_t lastPrintMs = 0;
uint8_t linkDropCounter = 0;

// -------------------- Helpers --------------------
int16_t readCenteredStick(uint8_t pin) {
  int16_t raw = analogRead(pin) - 512;
  if (abs(raw) < 8) {
    raw = 0;
  }
  raw = constrain(raw, -512, 512);
  return raw;
}

uint16_t readThrottle(uint8_t pin) {
  int16_t raw = analogRead(pin);
  raw = constrain(raw, 0, 1023);
  return static_cast<uint16_t>(raw);
}

void configureInputs() {
  pinMode(PIN_BTN_CAL, INPUT_PULLUP);
  pinMode(PIN_BTN_ARM, INPUT_PULLUP);
  pinMode(PIN_SWITCH_ALT, INPUT_PULLUP);
  pinMode(PIN_SWITCH_KILL, INPUT_PULLUP);
  pinMode(PIN_LED_LINK, OUTPUT);
}

void setupRadio() {
  radio.begin();
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(5, 15);
  radio.openWritingPipe(PIPE_FC);
  radio.openReadingPipe(1, PIPE_RC);
  radio.stopListening();
}

void buildControlFrame() {
  frame.throttle = readThrottle(PIN_STICK_THROTTLE);
  frame.yaw = readCenteredStick(PIN_STICK_YAW);
  frame.pitch = readCenteredStick(PIN_STICK_PITCH);
  frame.roll = readCenteredStick(PIN_STICK_ROLL);
  frame.flags = 0;

  bool altHoldEnabled = (digitalRead(PIN_SWITCH_ALT) == LOW);
  bool armSwitchEnabled = (digitalRead(PIN_SWITCH_KILL) == LOW);  // LOW = armed, HIGH = kill
  bool buttonCal = (digitalRead(PIN_BTN_CAL) == LOW);
  bool buttonArm = (digitalRead(PIN_BTN_ARM) == LOW);

  if (altHoldEnabled) {
    frame.flags |= FLAG_ALT_HOLD;
  }

  if (!armSwitchEnabled) {
    frame.flags |= FLAG_KILL;
  }

  if (buttonCal) {
    frame.flags |= FLAG_CALIBRATE;
  }

  if (buttonArm) {
    if (!altHoldEnabled) {
      frame.flags |= FLAG_ESC_CAL;  // SW1 at 0 + button2 = ESC calibration request
    } else {
      frame.flags |= FLAG_ARM_BTN;
    }
  }

  frame.frameId = frameCounter++;
}

void transmitFrame() {
  bool success = radio.write(&frame, sizeof(frame));
  if (!success) {
    linkDropCounter++;
  }

  while (radio.isAckPayloadAvailable()) {
    radio.read(&telemetry, sizeof(telemetry));
    lastAckMs = millis();
  }

  if (millis() - lastAckMs > 250) {
    digitalWrite(PIN_LED_LINK, LOW);
  } else {
    digitalWrite(PIN_LED_LINK, HIGH);
  }
}

void printStatus() {
  if (millis() - lastPrintMs < 250) {
    return;
  }
  lastPrintMs = millis();
  bool linkHealthy = (millis() - lastAckMs) < 250;

  Serial.print("LINK:");
  Serial.print(linkHealthy ? "OK" : "---");
  Serial.print(" | Frame:");
  Serial.print(frame.frameId);
  Serial.print(" | Throttle:");
  Serial.print(frame.throttle);
  Serial.print(" | Yaw/Pitch/Roll:");
  Serial.print(frame.yaw);
  Serial.print('/');
  Serial.print(frame.pitch);
  Serial.print('/');
  Serial.print(frame.roll);
  Serial.print(" | AltHold:");
  Serial.print((frame.flags & FLAG_ALT_HOLD) ? "ON" : "OFF");
  Serial.print(" | ArmSw:");
  Serial.print((frame.flags & FLAG_KILL) ? "SAFE" : "ARMED");
  Serial.print(" | Btn1:");
  Serial.print((frame.flags & FLAG_CALIBRATE) ? 1 : 0);
  Serial.print(" | Btn2:");
  Serial.print((frame.flags & FLAG_ARM_BTN) ? 1 : 0);

  Serial.print(" || FC state:");
  Serial.print(telemetry.state);
  Serial.print(" faults:");
  Serial.print(telemetry.faultFlags, HEX);
  Serial.print(" imuCal:");
  Serial.print(telemetry.imuCalibrated ? "Y" : "N");
  Serial.print(" escCal:");
  Serial.print(telemetry.escCalibrated ? "Y" : "N");
  Serial.print(" loop us:");
  Serial.print(telemetry.loopTimeUs);
  Serial.print(" pkMiss:");
  Serial.println(telemetry.packetMisses);
}

void setup() {
  Serial.begin(115200);
  configureInputs();
  setupRadio();
  digitalWrite(PIN_LED_LINK, LOW);
  Serial.println("RC Ready. Power FC to establish link...");
}

void loop() {
  buildControlFrame();
  transmitFrame();
  printStatus();
  delay(15);  // ~60 Hz update rate
}
