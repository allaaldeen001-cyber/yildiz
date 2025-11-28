#include <RF24.h>
#include <SPI.h>
#include <Smoothed.h>

constexpr uint8_t RADIO_CE_PIN = 9;
constexpr uint8_t RADIO_CSN_PIN = 10;
constexpr uint8_t RADIO_CHANNEL = 108;
const uint64_t RADIO_PIPE = 0xF0F0F0F0E1LL;

constexpr uint8_t SWITCH_ALT_PIN = 2;  // Altitude hold
constexpr uint8_t SWITCH_ARM_PIN = 3;  // Arm / disarm
constexpr uint8_t BUTTON_CAL_PIN = 4;  // Calibration
constexpr uint8_t BUTTON_START_PIN = 5;  // Smooth-start

constexpr uint8_t THROTTLE_PIN = A0;
constexpr uint8_t YAW_PIN = A1;
constexpr uint8_t PITCH_PIN = A2;
constexpr uint8_t ROLL_PIN = A3;

constexpr int pulseMIN = 1000;
constexpr int pulseMAX = 2000;
constexpr float maxTiltCmd = 30.0f;
constexpr float maxYawCmd = 120.0f;

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);

struct Package {
    int thrust = 0;
    float x = 0;
    float y = 0;
    float z = 0;
    uint16_t id = 0;
    bool but1 = true;
    bool but2 = true;
    bool switch1 = true;
    bool switch2 = true;
};

struct Telemetry {
    uint16_t batteryMv = 0;
    bool armed = false;
    bool linkActive = false;
    bool altHoldEnabled = false;
    uint16_t lastPacketId = 0;
};

Package package;
Telemetry telemetry;

Smoothed<float> throttleFilter;
Smoothed<float> yawFilter;
Smoothed<float> pitchFilter;
Smoothed<float> rollFilter;

unsigned long lastPrintMs = 0;

void setup() {
    pinMode(SWITCH_ALT_PIN, INPUT_PULLUP);
    pinMode(SWITCH_ARM_PIN, INPUT_PULLUP);
    pinMode(BUTTON_CAL_PIN, INPUT_PULLUP);
    pinMode(BUTTON_START_PIN, INPUT_PULLUP);

    Serial.begin(57600);

    radio.begin();
    radio.setChannel(RADIO_CHANNEL);
    radio.setAutoAck(true);
    radio.enableAckPayload();
    radio.setRetries(5, 15);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.openWritingPipe(RADIO_PIPE);
    radio.stopListening();

    throttleFilter.begin(SMOOTHED_EXPONENTIAL, 8);
    yawFilter.begin(SMOOTHED_EXPONENTIAL, 8);
    pitchFilter.begin(SMOOTHED_EXPONENTIAL, 8);
    rollFilter.begin(SMOOTHED_EXPONENTIAL, 8);
}

void loop() {
    readInputs();
    sendPackage();
    maybePrint();
    delay(10);
}

void readInputs() {
    throttleFilter.add(static_cast<float>(1023 - analogRead(THROTTLE_PIN)));
    yawFilter.add(static_cast<float>(analogRead(YAW_PIN) - 512));
    pitchFilter.add(static_cast<float>((1023 - analogRead(PITCH_PIN)) - 512));
    rollFilter.add(static_cast<float>(analogRead(ROLL_PIN) - 512));

    package.thrust = constrain(map(static_cast<int>(throttleFilter.get()), 0, 1023, pulseMIN, pulseMAX), pulseMIN,
                               pulseMAX);

    const float yawNorm = yawFilter.get() / 512.0f;
    const float pitchNorm = pitchFilter.get() / 512.0f;
    const float rollNorm = rollFilter.get() / 512.0f;

    package.z = constrain(yawNorm * maxYawCmd, -maxYawCmd, maxYawCmd);
    package.y = constrain(pitchNorm * maxTiltCmd, -maxTiltCmd, maxTiltCmd);
    package.x = constrain(rollNorm * maxTiltCmd, -maxTiltCmd, maxTiltCmd);

    package.but1 = digitalRead(BUTTON_CAL_PIN);
    package.but2 = digitalRead(BUTTON_START_PIN);
    package.switch1 = digitalRead(SWITCH_ARM_PIN);
    package.switch2 = digitalRead(SWITCH_ALT_PIN);

    package.id++;
}

void sendPackage() {
    bool delivered = radio.write(&package, sizeof(package));

    if (delivered && radio.isAckPayloadAvailable()) {
        radio.read(&telemetry, sizeof(telemetry));
    }

    if (!delivered) {
        Serial.println(F("Link lost - check RF channel or power"));
    }
}

void maybePrint() {
    if (millis() - lastPrintMs < 250) {
        return;
    }
    lastPrintMs = millis();

    Serial.print(F("Throttle:"));
    Serial.print(package.thrust);
    Serial.print(F("\tRoll:"));
    Serial.print(package.x);
    Serial.print(F("\tPitch:"));
    Serial.print(package.y);
    Serial.print(F("\tYaw:"));
    Serial.print(package.z);
    Serial.print(F("\tArm:"));
    Serial.print(!package.switch1);
    Serial.print(F("\tAlt:"));
    Serial.print(!package.switch2);
    Serial.print(F("\tBattery:"));
    Serial.println(telemetry.batteryMv);
}
