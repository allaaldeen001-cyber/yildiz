/*
 * Remote Controller Firmware for the Professional Nano UAV Platform
 *
 * Responsibilities:
 *  - Sample dual joysticks, buttons, and safety switches.
 *  - Package commands into a CRC-protected control frame.
 *  - Transmit via nRF24L01+ (channel 103) with ACK payloads enabled.
 *  - Display link, button, and calibration state via the serial monitor.
 */

#include <SPI.h>
#include <RF24.h>

// ----------------------------- Pin Map ------------------------------------
constexpr uint8_t PIN_RADIO_CE = 9;
constexpr uint8_t PIN_RADIO_CSN = 10;

constexpr uint8_t PIN_BTN_CAL = 4;   // Button 1
constexpr uint8_t PIN_BTN_MOTOR = 5; // Button 2
constexpr uint8_t PIN_SW_ALT = 2;    // Switch 1 (altitude hold gate)
constexpr uint8_t PIN_SW_KILL = 3;   // Switch 2 (kill / arm gate)

constexpr uint8_t PIN_STICK_THROTTLE = A0; // Left stick vertical
constexpr uint8_t PIN_STICK_YAW = A1;       // Left stick horizontal
constexpr uint8_t PIN_STICK_PITCH = A2;     // Right stick vertical
constexpr uint8_t PIN_STICK_ROLL = A3;      // Right stick horizontal

// ------------------------ Protocol Definitions ----------------------------
constexpr uint8_t FLAG_CALIBRATE = 0x01;
constexpr uint8_t FLAG_MOTOR_ENABLE = 0x02;
constexpr uint8_t FLAG_ESC_CAL = 0x04;

constexpr uint8_t SWITCH_ALT_HOLD = 0x01;
constexpr uint8_t SWITCH_KILL_SAFE = 0x02;

constexpr uint64_t PIPE_ADDRESS = 0xE7E7E7E7E7ULL;

struct __attribute__((packed)) ControlFrame {
  uint32_t sequence;
  uint16_t throttleUs;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
  uint8_t flags;
  uint8_t switchMask;
  uint16_t crc;
};

struct __attribute__((packed)) TelemetryFrame {
  uint8_t fcState;
  uint8_t linkQuality;
  uint8_t calibrationCode;
  uint16_t batteryMv;
  int16_t baroAltCm;
  uint8_t faultCode;
};

enum FcState : uint8_t { FC_INIT, FC_READY, FC_CAL, FC_ARMED, FC_ESC_CAL, FC_FAULT };
enum CalCode : uint8_t { CAL_IDLE, CAL_RUNNING, CAL_SUCCESS, CAL_FAILED };

constexpr unsigned long kHudPeriodMs = 250UL;
constexpr unsigned long kLongPressMs = 1200UL;

constexpr uint16_t kThrottleMinUs = 1000;
constexpr uint16_t kThrottleMaxUs = 2000;

constexpr uint8_t kStickSamples = 4;
constexpr unsigned long kLinkDropMs = 500UL;

RF24 radio(PIN_RADIO_CE, PIN_RADIO_CSN);

ControlFrame lastFrameSent = {};
TelemetryFrame lastTelemetry = {};

uint32_t sequenceCounter = 0;
uint32_t lastAckMs = 0;
uint8_t linkQualityLocal = 0;

bool requestCalibration = false;
bool motorsLatched = false;
bool requestEscCalibration = false;

bool btnCalPrev = false;
bool btnMotorPrev = false;
uint32_t btnMotorPressMs = 0;

uint32_t lastHudMs = 0;

// --------------------------- Helper Functions -----------------------------
uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

int readAveragedAnalog(uint8_t pin) {
  long total = 0;
  for (uint8_t i = 0; i < kStickSamples; ++i) {
    total += analogRead(pin);
  }
  return total / kStickSamples;
}

uint16_t readThrottleUs() {
  int reading = readAveragedAnalog(PIN_STICK_THROTTLE);
  return constrain(map(reading, 0, 1023, kThrottleMinUs, kThrottleMaxUs), kThrottleMinUs, kThrottleMaxUs);
}

int16_t readAxisCentered(uint8_t pin) {
  int reading = readAveragedAnalog(pin);
  long mapped = map(reading, 0, 1023, -500, 500);
  if (abs(mapped) < 6) {
    mapped = 0; // deadband for noise
  }
  return constrain(mapped, -500, 500);
}

bool readButton(uint8_t pin) {
  return digitalRead(pin) == LOW; // active-low due to pull-ups
}

bool readSwitchHigh(uint8_t pin) {
  // Switch ties pin to GND when in position "0" as per wiring note.
  return digitalRead(pin) == HIGH; // HIGH = not pulled to GND => logical 1
}

const char *stateToText(uint8_t state) {
  switch (state) {
    case FC_INIT: return "INIT";
    case FC_READY: return "READY";
    case FC_CAL: return "CAL";
    case FC_ARMED: return "ARMED";
    case FC_ESC_CAL: return "ESC";
    case FC_FAULT: return "FAULT";
    default: return "UNK";
  }
}

const char *calToText(uint8_t code) {
  switch (code) {
    case CAL_IDLE: return "IDLE";
    case CAL_RUNNING: return "RUN";
    case CAL_SUCCESS: return "OK";
    case CAL_FAILED: return "FAIL";
    default: return "UNK";
  }
}

// ---------------------------- Radio Handling ------------------------------
void setupRadio() {
  if (!radio.begin()) {
    Serial.println(F("[RC] RF init failed"));
    while (true) {
      delay(100);
    }
  }
  radio.setChannel(103);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.openWritingPipe(PIPE_ADDRESS);
  radio.stopListening();
}

bool transmitFrame(const ControlFrame &frame) {
  bool ok = radio.write(&frame, sizeof(frame));
  if (ok) {
    linkQualityLocal = (linkQualityLocal > 94) ? 100 : (linkQualityLocal + 5);
  } else if (linkQualityLocal > 2) {
    linkQualityLocal -= 2;
  }

  while (radio.isAckPayloadAvailable()) {
    uint8_t payloadSize = radio.getDynamicPayloadSize();
    if (payloadSize > sizeof(TelemetryFrame)) {
      payloadSize = sizeof(TelemetryFrame);
    }
    radio.read(&lastTelemetry, payloadSize);
    lastAckMs = millis();
  }

  return ok;
}

// ----------------------------- Input Logic --------------------------------
void serviceButtons() {
  bool btnCalNow = readButton(PIN_BTN_CAL);
  bool btnMotorNow = readButton(PIN_BTN_MOTOR);

  if (btnCalNow && !btnCalPrev) {
    requestCalibration = true;
  }
  btnCalPrev = btnCalNow;

  if (btnMotorNow && !btnMotorPrev) {
    btnMotorPressMs = millis();
  } else if (!btnMotorNow && btnMotorPrev) {
    uint32_t held = millis() - btnMotorPressMs;
    if (held > kLongPressMs && !readSwitchHigh(PIN_SW_ALT)) {
      requestEscCalibration = true; // Switch 1 at "0" per SOP
      motorsLatched = false;
    } else {
      motorsLatched = !motorsLatched;
    }
  }
  btnMotorPrev = btnMotorNow;
}

ControlFrame buildFrame() {
  ControlFrame frame{};
  frame.sequence = ++sequenceCounter;
  frame.throttleUs = readThrottleUs();
  frame.yaw = readAxisCentered(PIN_STICK_YAW);
  frame.pitch = readAxisCentered(PIN_STICK_PITCH);
  frame.roll = readAxisCentered(PIN_STICK_ROLL);

  bool altHoldRequest = readSwitchHigh(PIN_SW_ALT);
  bool killSafe = readSwitchHigh(PIN_SW_KILL);

  frame.flags = 0;
  if (requestCalibration) {
    frame.flags |= FLAG_CALIBRATE;
    requestCalibration = false;
  }
  if (requestEscCalibration) {
    frame.flags |= FLAG_ESC_CAL;
    requestEscCalibration = false;
  }
  if (motorsLatched && killSafe) {
    frame.flags |= FLAG_MOTOR_ENABLE;
  } else if (!killSafe) {
    motorsLatched = false;
  }

  frame.switchMask = 0;
  if (altHoldRequest) {
    frame.switchMask |= SWITCH_ALT_HOLD;
  }
  if (killSafe) {
    frame.switchMask |= SWITCH_KILL_SAFE;
  }

  frame.crc = crc16(reinterpret_cast<const uint8_t *>(&frame), sizeof(ControlFrame) - sizeof(uint16_t));
  lastFrameSent = frame;
  return frame;
}

// ---------------------------- Telemetry HUD -------------------------------
bool linkIsAlive() {
  return (millis() - lastAckMs) < kLinkDropMs;
}

void printHud() {
  if (millis() - lastHudMs < kHudPeriodMs) {
    return;
  }
  lastHudMs = millis();

  Serial.print(F("Link:"));
  Serial.print(linkIsAlive() ? F("OK ") : F("---"));
  Serial.print(F("RSSI:"));
  Serial.print(linkQualityLocal);
  Serial.print(F("%  Th:"));
  Serial.print(lastFrameSent.throttleUs);
  Serial.print(F("  Y/P/R:"));
  Serial.print(lastFrameSent.yaw);
  Serial.print('/');
  Serial.print(lastFrameSent.pitch);
  Serial.print('/');
  Serial.println(lastFrameSent.roll);

  Serial.print(F("BTN cal:"));
  Serial.print(readButton(PIN_BTN_CAL));
  Serial.print(F(" arm:"));
  Serial.print(motorsLatched ? 1 : 0);
  Serial.print(F("  SW alt:"));
  Serial.print(readSwitchHigh(PIN_SW_ALT) ? 1 : 0);
  Serial.print(F(" kill:"));
  Serial.println(readSwitchHigh(PIN_SW_KILL) ? 1 : 0);

  Serial.print(F("FC ST:"));
  Serial.print(stateToText(lastTelemetry.fcState));
  Serial.print(F("  Cal:"));
  Serial.print(calToText(lastTelemetry.calibrationCode));
  Serial.print(F("  Alt:"));
  Serial.print(lastTelemetry.baroAltCm);
  Serial.print(F("cm  Fault:"));
  Serial.println(lastTelemetry.faultCode);

  Serial.println();
}

// ------------------------------- Arduino ----------------------------------
void setup() {
  pinMode(PIN_BTN_CAL, INPUT_PULLUP);
  pinMode(PIN_BTN_MOTOR, INPUT_PULLUP);
  pinMode(PIN_SW_ALT, INPUT_PULLUP);
  pinMode(PIN_SW_KILL, INPUT_PULLUP);

  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    // wait for monitor (optional)
  }

  setupRadio();
  Serial.println(F("[RC] Remote controller ready."));
}

void loop() {
  serviceButtons();
  ControlFrame frame = buildFrame();
  transmitFrame(frame);
  printHud();
}
