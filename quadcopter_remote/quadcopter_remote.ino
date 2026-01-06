/**
 * ============================================================================
 * QUADCOPTER REMOTE CONTROLLER FIRMWARE v2.0
 * ============================================================================
 * 
 * Target: Arduino Nano (ATmega328P @ 16MHz)
 * 
 * Features:
 *   - NRF24L01 with ACK mode for reliable communication
 *   - Serial debug output for monitoring
 *   - Joystick calibration
 *   - Visual and audio feedback
 * 
 * Hardware Configuration:
 *   NRF24L01: CE → D9, CSN → D10
 *   Toggle Switch (Arm): D2
 *   Toggle Switch (Alt Hold): D3
 *   Push Button (Calibrate): D4
 *   Push Button (Motor Test): D5
 *   Joystick Throttle: A0
 *   Joystick Yaw: A1
 *   Joystick Pitch: A2
 *   Joystick Roll: A3
 *   Buzzer: D6
 *   LED: D7
 * 
 * Required Libraries:
 *   - RF24 (TMRh20): https://github.com/nRF24/RF24
 * 
 * Author: Flight Control Systems
 * Version: 2.0.0
 * 
 * ============================================================================
 */

#include <SPI.h>
#include <RF24.h>

// ============================================================================
// CONFIGURATION - MUST MATCH FLIGHT CONTROLLER!
// ============================================================================

// RF Channel - MUST MATCH FLIGHT CONTROLLER
#define RF_CHANNEL          108

// Enable serial debug output
#define DEBUG_SERIAL        true
#define SERIAL_BAUD         115200

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

namespace Pins {
    // NRF24L01
    constexpr uint8_t RF_CE  = 9;
    constexpr uint8_t RF_CSN = 10;
    
    // Digital Inputs (active LOW with pull-up)
    constexpr uint8_t SW_ARM      = 2;   // Toggle switch - Arm/Disarm
    constexpr uint8_t SW_ALTHOLD  = 3;   // Toggle switch - Altitude Hold
    constexpr uint8_t BTN_CALIB   = 4;   // Push button - Calibration
    constexpr uint8_t BTN_MOTOR   = 5;   // Push button - Motor test
    
    // Analog Inputs
    constexpr uint8_t JOY_THROTTLE = A0;
    constexpr uint8_t JOY_YAW      = A1;
    constexpr uint8_t JOY_PITCH    = A2;
    constexpr uint8_t JOY_ROLL     = A3;
    
    // User Interface
    constexpr uint8_t BUZZER = 6;
    constexpr uint8_t LED    = 7;
}

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

namespace Timing {
    constexpr uint32_t TX_PERIOD_MS     = 20;     // 50 Hz transmission
    constexpr uint32_t INPUT_PERIOD_MS  = 10;     // 100 Hz input sampling
    constexpr uint32_t DEBUG_PERIOD_MS  = 250;    // 4 Hz debug output
    constexpr uint32_t LED_PERIOD_MS    = 500;    // LED blink rate
    constexpr uint32_t DEBOUNCE_MS      = 50;     // Button debounce
}

// ============================================================================
// RF CONFIGURATION - MUST MATCH FLIGHT CONTROLLER!
// ============================================================================

namespace RFConfig {
    constexpr uint8_t CHANNEL = RF_CHANNEL;
    constexpr uint8_t PAYLOAD_SIZE = 16;
    const uint8_t PIPE_ADDRESS[6] = "QUAD1";  // 5-byte address + null
    
    // ACK configuration
    constexpr uint8_t RETRY_DELAY = 5;   // 5 = 1500us delay
    constexpr uint8_t RETRY_COUNT = 3;   // 3 retries
    
    // Switch bit definitions
    constexpr uint8_t SW_ARM_BIT       = 0;
    constexpr uint8_t SW_CALIBRATE_BIT = 1;
    constexpr uint8_t SW_MOTORTEST_BIT = 2;
    constexpr uint8_t SW_ALTHOLD_BIT   = 3;
}

// Control packet structure (16 bytes) - MUST MATCH RECEIVER
struct __attribute__((packed)) ControlPacket {
    uint16_t throttle;    // 0-1000
    int16_t  yaw;         // -500 to +500
    int16_t  pitch;       // -500 to +500
    int16_t  roll;        // -500 to +500
    uint8_t  switches;    // Bitfield
    uint8_t  checksum;    // XOR checksum
    uint32_t sequence;    // Packet counter
    uint8_t  channel;     // RF channel (for verification)
    uint8_t  reserved;
    
    void calculateChecksum() {
        const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
        uint8_t calc = 0;
        for (uint8_t i = 0; i < 9; i++) {
            calc ^= data[i];
        }
        checksum = calc;
    }
};

// ============================================================================
// JOYSTICK CONFIGURATION
// ============================================================================

namespace JoystickConfig {
    // Deadband (ADC units, ~5% of range)
    constexpr int16_t DEADBAND = 25;
    
    // Expo factor (0.0 = linear, 0.5 = medium expo)
    constexpr float EXPO = 0.3f;
}

// Joystick calibration data
struct JoystickCalibration {
    int16_t throttleMin = 0;
    int16_t throttleMax = 1023;
    int16_t yawCenter = 512;
    int16_t pitchCenter = 512;
    int16_t rollCenter = 512;
};

JoystickCalibration joyCal;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

RF24 radio(Pins::RF_CE, Pins::RF_CSN);

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Control packet
ControlPacket txPacket;
uint32_t packetSequence = 0;

// Raw joystick values
int16_t rawThrottle = 0;
int16_t rawYaw = 0;
int16_t rawPitch = 0;
int16_t rawRoll = 0;

// Processed joystick values
int16_t throttle = 0;
int16_t yaw = 0;
int16_t pitch = 0;
int16_t roll = 0;

// Switch states
bool armSwitch = false;
bool altHoldSwitch = false;
bool calibButton = false;
bool motorTestButton = false;

// Previous switch states for edge detection
bool prevArmSwitch = false;
bool prevCalibButton = false;
bool prevMotorTestButton = false;

// Communication status
bool rfInitialized = false;
bool rfConnected = false;
uint32_t packetsSent = 0;
uint32_t packetsAcked = 0;
uint32_t packetsFailed = 0;
uint32_t lastAckTime = 0;

// Timing
uint32_t lastTxTime = 0;
uint32_t lastInputTime = 0;
uint32_t lastDebugTime = 0;
uint32_t lastLedTime = 0;
bool ledState = false;

// ============================================================================
// BUZZER FUNCTIONS
// ============================================================================

void beep(uint16_t duration, uint16_t freq = 2000) {
    tone(Pins::BUZZER, freq, duration);
}

void beepBlocking(uint16_t duration, uint16_t freq = 2000) {
    tone(Pins::BUZZER, freq);
    delay(duration);
    noTone(Pins::BUZZER);
}

void buzzerStartup() {
    beepBlocking(100, 1500);
    delay(50);
    beepBlocking(100, 2000);
    delay(50);
    beepBlocking(100, 2500);
    delay(50);
    beepBlocking(200, 3000);
}

void buzzerButtonPress() {
    beep(30, 2500);
}

void buzzerArmOn() {
    beepBlocking(100, 2000);
    delay(50);
    beepBlocking(200, 2500);
}

void buzzerArmOff() {
    beepBlocking(200, 1500);
}

void buzzerConnected() {
    // 3 quick beeps for connection
    for (int i = 0; i < 3; i++) {
        beepBlocking(50, 2500);
        delay(50);
    }
}

void buzzerDisconnected() {
    beepBlocking(300, 800);
}

void buzzerError() {
    for (int i = 0; i < 3; i++) {
        beepBlocking(150, 800);
        delay(100);
    }
}

void buzzerTxSuccess() {
    // Very short beep for successful TX (optional, can be noisy)
    // beep(10, 3000);
}

// ============================================================================
// SERIAL DEBUG OUTPUT
// ============================================================================

void printDebugHeader() {
    Serial.println(F("\n========================================"));
    Serial.println(F("  QUADCOPTER REMOTE CONTROLLER v2.0"));
    Serial.println(F("========================================"));
    Serial.print(F("RF Channel: "));
    Serial.println(RF_CHANNEL);
    Serial.println(F(""));
}

void printStatus() {
    if (!DEBUG_SERIAL) return;
    
    Serial.println(F("\n--- RC STATUS ---"));
    
    // RF Status
    Serial.print(F("RF: "));
    if (rfConnected) {
        Serial.print(F("CONNECTED"));
    } else {
        Serial.print(F("DISCONNECTED"));
    }
    Serial.print(F(" | Sent: "));
    Serial.print(packetsSent);
    Serial.print(F(" | ACK: "));
    Serial.print(packetsAcked);
    Serial.print(F(" | Fail: "));
    Serial.print(packetsFailed);
    
    // Calculate success rate
    if (packetsSent > 0) {
        float successRate = (float)packetsAcked / packetsSent * 100.0f;
        Serial.print(F(" | Rate: "));
        Serial.print(successRate, 1);
        Serial.print(F("%"));
    }
    Serial.println();
    
    // Raw Joystick Values
    Serial.print(F("RAW: T="));
    Serial.print(rawThrottle);
    Serial.print(F(" Y="));
    Serial.print(rawYaw);
    Serial.print(F(" P="));
    Serial.print(rawPitch);
    Serial.print(F(" R="));
    Serial.println(rawRoll);
    
    // Processed Values
    Serial.print(F("OUT: T="));
    Serial.print(throttle);
    Serial.print(F(" Y="));
    Serial.print(yaw);
    Serial.print(F(" P="));
    Serial.print(pitch);
    Serial.print(F(" R="));
    Serial.println(roll);
    
    // Switches
    Serial.print(F("SW: ARM="));
    Serial.print(armSwitch ? "ON" : "OFF");
    Serial.print(F(" ALT="));
    Serial.print(altHoldSwitch ? "ON" : "OFF");
    Serial.print(F(" CAL="));
    Serial.print(calibButton ? "ON" : "OFF");
    Serial.print(F(" MTR="));
    Serial.println(motorTestButton ? "ON" : "OFF");
    
    Serial.println(F("---"));
}

// ============================================================================
// JOYSTICK PROCESSING
// ============================================================================

int16_t applyDeadbandAndScale(int16_t raw, int16_t center, int16_t deadband, int16_t outMin, int16_t outMax) {
    int16_t deviation = raw - center;
    
    // Apply deadband
    if (abs(deviation) < deadband) {
        return 0;  // Center output
    }
    
    // Remove deadband from range
    if (deviation > 0) {
        deviation -= deadband;
    } else {
        deviation += deadband;
    }
    
    // Calculate max deviation after deadband
    int16_t maxDev = 512 - deadband;
    
    // Scale to output range
    int16_t halfRange = (outMax - outMin) / 2;
    return (int32_t)deviation * halfRange / maxDev;
}

int16_t applyExpo(int16_t value, float expo) {
    // Expo curve: output = input * (1-expo) + input^3 * expo
    float normalized = value / 500.0f;  // -1 to 1
    float curved = normalized * (1.0f - expo) + 
                  (normalized * normalized * normalized) * expo;
    return (int16_t)(curved * 500.0f);
}

void readJoysticks() {
    // Read raw ADC values (average 4 samples for noise reduction)
    int32_t sum[4] = {0, 0, 0, 0};
    for (uint8_t i = 0; i < 4; i++) {
        sum[0] += analogRead(Pins::JOY_THROTTLE);
        sum[1] += analogRead(Pins::JOY_YAW);
        sum[2] += analogRead(Pins::JOY_PITCH);
        sum[3] += analogRead(Pins::JOY_ROLL);
    }
    rawThrottle = sum[0] / 4;
    rawYaw = sum[1] / 4;
    rawPitch = sum[2] / 4;
    rawRoll = sum[3] / 4;
    
    // Process throttle (unidirectional: 0 to 1000)
    throttle = map(rawThrottle, joyCal.throttleMin, joyCal.throttleMax, 0, 1000);
    throttle = constrain(throttle, 0, 1000);
    
    // Process centered axes (bidirectional: -500 to +500)
    yaw = applyDeadbandAndScale(rawYaw, joyCal.yawCenter, 
                                 JoystickConfig::DEADBAND, -500, 500);
    pitch = applyDeadbandAndScale(rawPitch, joyCal.pitchCenter,
                                   JoystickConfig::DEADBAND, -500, 500);
    roll = applyDeadbandAndScale(rawRoll, joyCal.rollCenter,
                                  JoystickConfig::DEADBAND, -500, 500);
    
    // Apply expo
    yaw = applyExpo(yaw, JoystickConfig::EXPO);
    pitch = applyExpo(pitch, JoystickConfig::EXPO);
    roll = applyExpo(roll, JoystickConfig::EXPO);
}

void readSwitches() {
    // Read switches (active LOW with internal pull-up)
    prevArmSwitch = armSwitch;
    prevCalibButton = calibButton;
    prevMotorTestButton = motorTestButton;
    
    armSwitch = !digitalRead(Pins::SW_ARM);
    altHoldSwitch = !digitalRead(Pins::SW_ALTHOLD);
    calibButton = !digitalRead(Pins::BTN_CALIB);
    motorTestButton = !digitalRead(Pins::BTN_MOTOR);
    
    // Edge detection for buttons
    if (calibButton && !prevCalibButton) {
        buzzerButtonPress();
        Serial.println(F("Calibrate button pressed"));
    }
    if (motorTestButton && !prevMotorTestButton) {
        buzzerButtonPress();
        Serial.println(F("Motor test button pressed"));
    }
    
    // Arm switch change
    if (armSwitch != prevArmSwitch) {
        if (armSwitch) {
            buzzerArmOn();
            Serial.println(F("ARM switch: ON"));
        } else {
            buzzerArmOff();
            Serial.println(F("ARM switch: OFF"));
        }
    }
}

void calibrateJoystickCenters() {
    Serial.println(F("Calibrating joystick centers..."));
    Serial.println(F("Keep all sticks centered!"));
    
    beepBlocking(200, 1500);
    delay(500);
    
    // Average 50 samples
    int32_t sum[3] = {0, 0, 0};
    for (int i = 0; i < 50; i++) {
        sum[0] += analogRead(Pins::JOY_YAW);
        sum[1] += analogRead(Pins::JOY_PITCH);
        sum[2] += analogRead(Pins::JOY_ROLL);
        delay(20);
    }
    
    joyCal.yawCenter = sum[0] / 50;
    joyCal.pitchCenter = sum[1] / 50;
    joyCal.rollCenter = sum[2] / 50;
    
    Serial.print(F("Calibration done: Yaw="));
    Serial.print(joyCal.yawCenter);
    Serial.print(F(" Pitch="));
    Serial.print(joyCal.pitchCenter);
    Serial.print(F(" Roll="));
    Serial.println(joyCal.rollCenter);
    
    beepBlocking(100, 2000);
    delay(50);
    beepBlocking(100, 2500);
    delay(50);
    beepBlocking(200, 3000);
}

// ============================================================================
// RF COMMUNICATION
// ============================================================================

bool setupRadio() {
    Serial.println(F("Initializing NRF24L01..."));
    Serial.print(F("Channel: "));
    Serial.println(RFConfig::CHANNEL);
    
    if (!radio.begin()) {
        Serial.println(F("ERROR: NRF24L01 not found!"));
        return false;
    }
    
    // Configure radio for ACK mode (must match receiver)
    radio.setChannel(RFConfig::CHANNEL);
    radio.setDataRate(RF24_2MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(RFConfig::PAYLOAD_SIZE);
    
    // Enable ACK mode
    radio.setAutoAck(true);
    radio.setRetries(RFConfig::RETRY_DELAY, RFConfig::RETRY_COUNT);
    
    // CRC for data integrity
    radio.setCRCLength(RF24_CRC_16);
    
    // Open writing pipe
    radio.openWritingPipe(RFConfig::PIPE_ADDRESS);
    
    // Stop listening (we're transmitting)
    radio.stopListening();
    
    Serial.println(F("NRF24L01 configured in ACK mode (Transmitter)."));
    
    rfInitialized = true;
    return true;
}

bool sendPacket() {
    if (!rfInitialized) return false;
    
    // Build packet
    txPacket.throttle = throttle;
    txPacket.yaw = yaw;
    txPacket.pitch = pitch;
    txPacket.roll = roll;
    
    // Build switch byte
    txPacket.switches = 0;
    if (armSwitch)        txPacket.switches |= (1 << RFConfig::SW_ARM_BIT);
    if (calibButton)      txPacket.switches |= (1 << RFConfig::SW_CALIBRATE_BIT);
    if (motorTestButton)  txPacket.switches |= (1 << RFConfig::SW_MOTORTEST_BIT);
    if (altHoldSwitch)    txPacket.switches |= (1 << RFConfig::SW_ALTHOLD_BIT);
    
    txPacket.sequence = packetSequence++;
    txPacket.channel = RFConfig::CHANNEL;
    txPacket.reserved = 0;
    
    // Calculate checksum
    txPacket.calculateChecksum();
    
    // Send packet
    packetsSent++;
    bool success = radio.write(&txPacket, sizeof(txPacket));
    
    if (success) {
        packetsAcked++;
        lastAckTime = millis();
        
        if (!rfConnected) {
            rfConnected = true;
            Serial.println(F("\n*** CONNECTED TO DRONE! ***"));
            buzzerConnected();
        }
        
        return true;
    } else {
        packetsFailed++;
        
        if (rfConnected && (millis() - lastAckTime > 500)) {
            rfConnected = false;
            Serial.println(F("\n*** CONNECTION LOST! ***"));
            buzzerDisconnected();
        }
        
        return false;
    }
}

// ============================================================================
// LED UPDATE
// ============================================================================

void updateLED() {
    uint32_t now = millis();
    
    if (rfConnected) {
        if (armSwitch) {
            // Solid when armed and connected
            digitalWrite(Pins::LED, HIGH);
        } else {
            // Slow blink when connected but disarmed
            if (now - lastLedTime >= 500) {
                lastLedTime = now;
                ledState = !ledState;
                digitalWrite(Pins::LED, ledState);
            }
        }
    } else {
        // Fast blink when not connected
        if (now - lastLedTime >= 100) {
            lastLedTime = now;
            ledState = !ledState;
            digitalWrite(Pins::LED, ledState);
        }
    }
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
    // Initialize serial
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000);
    
    printDebugHeader();
    
    // Initialize pins
    pinMode(Pins::LED, OUTPUT);
    pinMode(Pins::BUZZER, OUTPUT);
    pinMode(Pins::SW_ARM, INPUT_PULLUP);
    pinMode(Pins::SW_ALTHOLD, INPUT_PULLUP);
    pinMode(Pins::BTN_CALIB, INPUT_PULLUP);
    pinMode(Pins::BTN_MOTOR, INPUT_PULLUP);
    
    digitalWrite(Pins::LED, HIGH);
    
    // Startup sound
    buzzerStartup();
    
    // Initialize radio
    if (!setupRadio()) {
        Serial.println(F("RADIO INIT FAILED!"));
        buzzerError();
        while (1) {
            // Blink LED rapidly to indicate error
            digitalWrite(Pins::LED, !digitalRead(Pins::LED));
            delay(100);
        }
    }
    
    // Calibrate joystick centers (assumes sticks are centered at startup)
    calibrateJoystickCenters();
    
    Serial.println(F("\n*** REMOTE READY ***"));
    Serial.println(F("Transmitting on channel "));
    Serial.println(RF_CHANNEL);
    Serial.println(F("Searching for drone..."));
    
    // Initialize timing
    lastTxTime = millis();
    lastInputTime = millis();
    lastDebugTime = millis();
    lastLedTime = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    uint32_t now = millis();
    
    // Read inputs (100 Hz)
    if (now - lastInputTime >= Timing::INPUT_PERIOD_MS) {
        lastInputTime = now;
        readJoysticks();
        readSwitches();
    }
    
    // Transmit packet (50 Hz)
    if (now - lastTxTime >= Timing::TX_PERIOD_MS) {
        lastTxTime = now;
        sendPacket();
    }
    
    // Update LED
    updateLED();
    
    // Debug output (4 Hz)
    if (DEBUG_SERIAL && (now - lastDebugTime >= Timing::DEBUG_PERIOD_MS)) {
        lastDebugTime = now;
        printStatus();
    }
}

/**
 * ============================================================================
 * WIRING INSTRUCTIONS
 * ============================================================================
 * 
 * NRF24L01 Module:
 *   VCC  → 3.3V (IMPORTANT: NOT 5V!)
 *   GND  → GND
 *   CE   → D9
 *   CSN  → D10
 *   SCK  → D13
 *   MOSI → D11
 *   MISO → D12
 *   IRQ  → Not connected
 * 
 *   TIP: Add 10-100µF capacitor between VCC and GND of NRF24L01
 *        for stable operation!
 * 
 * Joysticks (2-axis potentiometer type):
 *   VCC → 5V
 *   GND → GND
 *   Throttle (vertical axis) → A0
 *   Yaw (horizontal axis)    → A1
 *   Pitch (vertical axis)    → A2
 *   Roll (horizontal axis)   → A3
 * 
 * Toggle Switches (connect between pin and GND):
 *   ARM Switch     → D2
 *   Alt Hold Switch→ D3
 * 
 * Push Buttons (connect between pin and GND):
 *   Calibrate      → D4
 *   Motor Test     → D5
 * 
 * Buzzer:
 *   + → D6 (through 100Ω resistor if needed)
 *   - → GND
 * 
 * LED:
 *   + → D7 (through 220Ω resistor)
 *   - → GND
 * 
 * ============================================================================
 * USAGE
 * ============================================================================
 * 
 * 1. Power on the REMOTE first
 * 2. Wait for startup beeps
 * 3. Ensure ARM switch is OFF
 * 4. Power on the DRONE
 * 5. Wait for connection (buzzer will beep 3 times on RC, 5 times on drone)
 * 6. Check serial monitor on both devices for status
 * 7. Move throttle to minimum
 * 8. Flip ARM switch to ON
 * 9. Drone should beep and LED goes solid
 * 10. Slowly increase throttle to fly
 * 
 * TO DISARM:
 * - Flip ARM switch to OFF
 * - OR: If connection is lost, drone auto-disarms
 * 
 * ============================================================================
 */
