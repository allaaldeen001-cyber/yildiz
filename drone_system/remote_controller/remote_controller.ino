/**
 * @file remote_controller.ino
 * @brief Professional Quadcopter Remote Controller
 * @author UAV Embedded Systems Engineer
 * @version 2.0
 * @date 2024
 * 
 * Arduino Nano based remote controller with:
 * - Dual joysticks (Throttle, Yaw, Pitch, Roll)
 * - Button inputs (Calibration, Motor On)
 * - Toggle switches (Altitude Hold, Arm/Disarm)
 * - NRF24L01 PA+LNA communication with ACK
 * - Serial monitor for status display
 * 
 * Hardware Configuration:
 * - NRF24L01: CE=D9, CSN=D10, SPI(D11,D12,D13)
 * - Left Joystick:  V=A0(Throttle), H=A1(Yaw)
 * - Right Joystick: V=A2(Pitch), H=A3(Roll)
 * - Button 1: D4 (Calibration)
 * - Button 2: D5 (Motor On/Arm)
 * - Switch 1: D2 (Altitude Hold)
 * - Switch 2: D3 (Arm/Disarm Kill Switch)
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "config.h"
#include "../shared/protocol.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// ============================================================================
// STATE VARIABLES
// ============================================================================

// Connection state
volatile bool nrfInitialized = false;
volatile bool droneConnected = false;
volatile uint8_t signalQuality = 0;
unsigned long lastAckTime = 0;
unsigned long connectionLostTime = 0;
uint32_t packetsSent = 0;
uint32_t packetsAcked = 0;

// Input states
int16_t rawThrottle = 0, rawYaw = 0, rawPitch = 0, rawRoll = 0;
int16_t filteredThrottle = 0, filteredYaw = 0, filteredPitch = 0, filteredRoll = 0;
bool button1State = false, button2State = false;
bool switch1State = false, switch2State = false;
bool prevButton1 = false, prevButton2 = false;

// Timing
unsigned long lastTxTime = 0;
unsigned long lastSerialTime = 0;
uint8_t txSequence = 0;

// Display mode
DisplayMode_t displayMode = DISPLAY_TELEMETRY;

// ============================================================================
// COMMUNICATION DATA
// ============================================================================

RCPacket_t txPacket;
FCTelemetry_t rxTelemetry;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void initializeHardware();
void initializeNRF24();
void readInputs();
int16_t readJoystick(uint8_t pin, int16_t minOut, int16_t maxOut, bool invertAxis);
int16_t applyDeadband(int16_t value, int16_t deadband);
int16_t filterInput(int16_t newValue, int16_t oldValue, float alpha);
void buildPacket();
bool transmitPacket();
void processAckPayload();
void updateSerialMonitor();
void printStatusBar();
void printControlInputs();
void printTelemetry();
void printConnectionStatus();
const char* getStatusString(uint8_t status);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial first for debug output
    Serial.begin(SERIAL_BAUD_RATE);
    
    // Print startup banner
    Serial.println(F("\n"));
    Serial.println(F("╔════════════════════════════════════════════════╗"));
    Serial.println(F("║     DRONE REMOTE CONTROLLER v2.0               ║"));
    Serial.println(F("║     Professional UAV Embedded System           ║"));
    Serial.println(F("╚════════════════════════════════════════════════╝"));
    Serial.println();
    
    // Initialize hardware
    initializeHardware();
    
    // Initialize NRF24L01
    initializeNRF24();
    
    // Initialize packet
    memset(&txPacket, 0, sizeof(txPacket));
    txPacket.header = 0xAA;
    
    // Initialize timing
    lastTxTime = millis();
    lastSerialTime = millis();
    
    Serial.println(F("\n[READY] Remote Controller Initialized"));
    Serial.println(F("[INFO]  Searching for drone..."));
    Serial.println(F("═══════════════════════════════════════════════════\n"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    unsigned long currentTime = millis();
    
    // ========== READ INPUTS ==========
    readInputs();
    
    // ========== TRANSMIT AT FIXED RATE ==========
    if (currentTime - lastTxTime >= TX_PERIOD_MS) {
        lastTxTime = currentTime;
        
        // Build and send packet
        buildPacket();
        bool success = transmitPacket();
        
        // Update connection status
        if (success) {
            lastAckTime = currentTime;
            if (!droneConnected) {
                droneConnected = true;
                Serial.println(F("\n[OK] DRONE CONNECTED!"));
            }
            processAckPayload();
        }
        
        // Check for connection timeout
        if (currentTime - lastAckTime > CONNECTION_TIMEOUT) {
            if (droneConnected) {
                droneConnected = false;
                connectionLostTime = currentTime;
                Serial.println(F("\n[WARN] CONNECTION LOST!"));
            }
        }
        
        // Calculate signal quality
        if (packetsSent > 0) {
            signalQuality = (uint8_t)((packetsAcked * 100UL) / packetsSent);
        }
    }
    
    // ========== UPDATE SERIAL MONITOR ==========
    if (currentTime - lastSerialTime >= SERIAL_UPDATE_MS) {
        lastSerialTime = currentTime;
        updateSerialMonitor();
    }
}

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================

void initializeHardware() {
    Serial.println(F("[INIT] Configuring hardware..."));
    
    // Configure button pins with internal pullup
    pinMode(BUTTON_1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_2_PIN, INPUT_PULLUP);
    
    // Configure switch pins with internal pullup
    pinMode(SWITCH_1_PIN, INPUT_PULLUP);
    pinMode(SWITCH_2_PIN, INPUT_PULLUP);
    
    // Configure analog reference
    analogReference(DEFAULT);
    
    // Initial ADC reads to stabilize
    for (int i = 0; i < 10; i++) {
        analogRead(JOYSTICK_LEFT_V);
        analogRead(JOYSTICK_LEFT_H);
        analogRead(JOYSTICK_RIGHT_V);
        analogRead(JOYSTICK_RIGHT_H);
    }
    
    Serial.println(F("[OK]   Hardware configured"));
}

// ============================================================================
// NRF24L01 INITIALIZATION
// ============================================================================

void initializeNRF24() {
    Serial.println(F("[INIT] Initializing NRF24L01 PA+LNA..."));
    
    if (!radio.begin()) {
        Serial.println(F("[FAIL] NRF24L01 not detected!"));
        Serial.println(F("[ERR]  Check wiring: CE=D9, CSN=D10"));
        nrfInitialized = false;
        // Continue anyway - will retry
        return;
    }
    
    // Configure radio for maximum reliability
    radio.setChannel(NRF_CHANNEL);
    radio.setPALevel(NRF_PA_LEVEL);
    radio.setDataRate(NRF_DATA_RATE);
    radio.setCRCLength(NRF_CRC_LENGTH);
    radio.setRetries(NRF_RETRY_DELAY, NRF_RETRY_COUNT);
    radio.setPayloadSize(NRF_PAYLOAD_SIZE);
    
    // Enable ACK payloads for telemetry reception
    radio.enableAckPayload();
    radio.enableDynamicPayloads();
    
    // Open pipes
    radio.openWritingPipe(RC_TO_FC_ADDR);
    radio.openReadingPipe(1, FC_TO_RC_ADDR);
    
    // Stop listening (we're transmitting)
    radio.stopListening();
    
    nrfInitialized = true;
    
    Serial.print(F("[OK]   NRF24L01 ready on channel "));
    Serial.println(NRF_CHANNEL);
    Serial.print(F("[INFO] TX Power: "));
    Serial.println(F("MAX (PA+LNA)"));
    Serial.print(F("[INFO] Data Rate: "));
    Serial.println(F("250kbps"));
}

// ============================================================================
// INPUT READING
// ============================================================================

void readInputs() {
    // ========== JOYSTICKS ==========
    // Read with oversampling for noise reduction
    int32_t sumLV = 0, sumLH = 0, sumRV = 0, sumRH = 0;
    
    for (int i = 0; i < ADC_OVERSAMPLE; i++) {
        sumLV += analogRead(JOYSTICK_LEFT_V);
        sumLH += analogRead(JOYSTICK_LEFT_H);
        sumRV += analogRead(JOYSTICK_RIGHT_V);
        sumRH += analogRead(JOYSTICK_RIGHT_H);
    }
    
    rawThrottle = sumLV / ADC_OVERSAMPLE;
    rawYaw = sumLH / ADC_OVERSAMPLE;
    rawPitch = sumRV / ADC_OVERSAMPLE;
    rawRoll = sumRH / ADC_OVERSAMPLE;
    
    // Apply filtering
    filteredThrottle = filterInput(rawThrottle, filteredThrottle, INPUT_FILTER_ALPHA);
    filteredYaw = filterInput(rawYaw, filteredYaw, INPUT_FILTER_ALPHA);
    filteredPitch = filterInput(rawPitch, filteredPitch, INPUT_FILTER_ALPHA);
    filteredRoll = filterInput(rawRoll, filteredRoll, INPUT_FILTER_ALPHA);
    
    // ========== BUTTONS ==========
    // Active LOW with pullup
    button1State = !digitalRead(BUTTON_1_PIN);
    button2State = !digitalRead(BUTTON_2_PIN);
    
    // ========== SWITCHES ==========
    // Active LOW with pullup (ON = LOW = 0)
    // Switch pulled to GND = ON (armed/enabled)
    switch1State = !digitalRead(SWITCH_1_PIN);  // Altitude hold
    switch2State = !digitalRead(SWITCH_2_PIN);  // Arm switch
}

int16_t readJoystick(uint8_t pin, int16_t minOut, int16_t maxOut, bool invertAxis) {
    int16_t raw = analogRead(pin);
    
    if (invertAxis) {
        raw = JOYSTICK_ADC_MAX - raw;
    }
    
    return map(raw, JOYSTICK_ADC_MIN, JOYSTICK_ADC_MAX, minOut, maxOut);
}

int16_t applyDeadband(int16_t value, int16_t deadband) {
    if (abs(value) < deadband) {
        return 0;
    }
    return value;
}

int16_t filterInput(int16_t newValue, int16_t oldValue, float alpha) {
    return (int16_t)(alpha * newValue + (1.0 - alpha) * oldValue);
}

// ============================================================================
// PACKET BUILDING
// ============================================================================

void buildPacket() {
    txPacket.header = 0xAA;
    txPacket.command = CMD_CONTROL;
    
    // Map joystick values to control ranges
    // Throttle: 0-1023 -> 1000-2000 (no center deadband)
    txPacket.throttle = map(filteredThrottle, 0, 1023, THROTTLE_MIN, THROTTLE_MAX);
    txPacket.throttle = constrain(txPacket.throttle, THROTTLE_MIN, THROTTLE_MAX);
    
    // Yaw: 0-1023 -> -500 to +500 (with deadband)
    int16_t yawCentered = filteredYaw - 512;
    yawCentered = applyDeadband(yawCentered, JOYSTICK_DEADBAND);
    txPacket.yaw = map(yawCentered, -512, 511, YAW_MIN, YAW_MAX);
    
    // Pitch: 0-1023 -> -500 to +500 (with deadband)
    // Inverted: Push UP = positive pitch (forward tilt)
    int16_t pitchCentered = (1023 - filteredPitch) - 512;
    pitchCentered = applyDeadband(pitchCentered, JOYSTICK_DEADBAND);
    txPacket.pitch = map(pitchCentered, -512, 511, PITCH_MIN, PITCH_MAX);
    
    // Roll: 0-1023 -> -500 to +500 (with deadband)
    int16_t rollCentered = filteredRoll - 512;
    rollCentered = applyDeadband(rollCentered, JOYSTICK_DEADBAND);
    txPacket.roll = map(rollCentered, -512, 511, ROLL_MIN, ROLL_MAX);
    
    // Switches and buttons
    txPacket.aux1 = switch1State ? 1 : 0;   // SW1: Altitude hold
    txPacket.aux2 = switch2State ? 1 : 0;   // SW2: Arm switch
    txPacket.btn1 = button1State ? 1 : 0;   // BTN1: Calibration
    txPacket.btn2 = button2State ? 1 : 0;   // BTN2: Motor on
    
    // Sequence number
    txPacket.sequence = txSequence++;
    
    // Calculate checksum
    txPacket.checksum = calculateChecksum((uint8_t*)&txPacket, sizeof(txPacket));
}

// ============================================================================
// RADIO TRANSMISSION
// ============================================================================

bool transmitPacket() {
    if (!nrfInitialized) {
        // Try to reinitialize
        initializeNRF24();
        return false;
    }
    
    packetsSent++;
    
    // Send packet and wait for ACK
    bool success = radio.write(&txPacket, sizeof(txPacket));
    
    if (success) {
        packetsAcked++;
    }
    
    return success;
}

void processAckPayload() {
    // Check if ACK payload is available
    if (radio.isAckPayloadAvailable()) {
        radio.read(&rxTelemetry, sizeof(rxTelemetry));
        
        // Validate telemetry
        if (validateFCPacket(&rxTelemetry)) {
            // Valid telemetry received
        }
    }
}

// ============================================================================
// SERIAL MONITOR DISPLAY
// ============================================================================

void updateSerialMonitor() {
    // Clear screen and move cursor to home (ANSI escape codes)
    Serial.print(F("\033[H"));  // Cursor to home
    
    // Print header
    printStatusBar();
    
    // Print sections based on display mode
    printConnectionStatus();
    Serial.println();
    printControlInputs();
    Serial.println();
    printTelemetry();
    Serial.println();
    printInstructions();
}

void printStatusBar() {
    Serial.println(F("┌─────────────────────────────────────────────────┐"));
    Serial.print(F("│ DRONE RC v2.0 │ "));
    
    if (droneConnected) {
        Serial.print(F("▓▓ LINKED ▓▓"));
    } else {
        Serial.print(F("░░ NO LINK ░░"));
    }
    
    Serial.print(F(" │ Signal: "));
    if (signalQuality >= 80) {
        Serial.print(F("████"));
    } else if (signalQuality >= 60) {
        Serial.print(F("███░"));
    } else if (signalQuality >= 40) {
        Serial.print(F("██░░"));
    } else if (signalQuality >= 20) {
        Serial.print(F("█░░░"));
    } else {
        Serial.print(F("░░░░"));
    }
    Serial.print(signalQuality);
    Serial.println(F("% │"));
    Serial.println(F("└─────────────────────────────────────────────────┘"));
}

void printConnectionStatus() {
    Serial.println(F("┌─ CONNECTION STATUS ────────────────────────────┐"));
    
    Serial.print(F("│ NRF24L01: "));
    Serial.print(nrfInitialized ? F("OK ") : F("ERR"));
    Serial.print(F(" │ Channel: "));
    Serial.print(NRF_CHANNEL);
    Serial.print(F(" │ Packets: "));
    Serial.print(packetsAcked);
    Serial.print(F("/"));
    Serial.print(packetsSent);
    Serial.println(F("     │"));
    
    Serial.print(F("│ Drone Status: "));
    if (droneConnected) {
        Serial.print(getStatusString(rxTelemetry.status));
    } else {
        Serial.print(F("DISCONNECTED"));
    }
    Serial.println(F("                          │"));
    
    Serial.println(F("└─────────────────────────────────────────────────┘"));
}

void printControlInputs() {
    Serial.println(F("┌─ CONTROL INPUTS ──────────────────────────────┐"));
    
    // Joystick values
    Serial.print(F("│ THR: "));
    printBar(txPacket.throttle, THROTTLE_MIN, THROTTLE_MAX, 10);
    Serial.print(F(" "));
    printPadded(txPacket.throttle, 4);
    
    Serial.print(F(" │ YAW: "));
    printBarCentered(txPacket.yaw, YAW_MIN, YAW_MAX, 10);
    Serial.print(F(" "));
    printPaddedSigned(txPacket.yaw, 4);
    Serial.println(F(" │"));
    
    Serial.print(F("│ PIT: "));
    printBarCentered(txPacket.pitch, PITCH_MIN, PITCH_MAX, 10);
    Serial.print(F(" "));
    printPaddedSigned(txPacket.pitch, 4);
    
    Serial.print(F(" │ ROL: "));
    printBarCentered(txPacket.roll, ROLL_MIN, ROLL_MAX, 10);
    Serial.print(F(" "));
    printPaddedSigned(txPacket.roll, 4);
    Serial.println(F(" │"));
    
    // Buttons and switches
    Serial.print(F("│ BTN1[CAL]: "));
    Serial.print(button1State ? F("■") : F("□"));
    Serial.print(F(" │ BTN2[ARM]: "));
    Serial.print(button2State ? F("■") : F("□"));
    Serial.print(F(" │ SW1[ALT]: "));
    Serial.print(switch1State ? F("ON ") : F("OFF"));
    Serial.print(F(" │ SW2[KILL]: "));
    Serial.print(switch2State ? F("ON ") : F("OFF"));
    Serial.println(F(" │"));
    
    Serial.println(F("└─────────────────────────────────────────────────┘"));
}

void printTelemetry() {
    Serial.println(F("┌─ DRONE TELEMETRY ─────────────────────────────┐"));
    
    if (droneConnected) {
        // Angles
        Serial.print(F("│ Roll: "));
        printPaddedSigned(rxTelemetry.roll_angle / 10, 4);
        Serial.print(F("° │ Pitch: "));
        printPaddedSigned(rxTelemetry.pitch_angle / 10, 4);
        Serial.print(F("° │ Yaw: "));
        printPaddedSigned(rxTelemetry.yaw_angle / 10, 4);
        Serial.println(F("°        │"));
        
        // Motors
        Serial.print(F("│ Motors: FL="));
        printPadded(rxTelemetry.motor_fl, 4);
        Serial.print(F(" FR="));
        printPadded(rxTelemetry.motor_fr, 4);
        Serial.print(F(" RR="));
        printPadded(rxTelemetry.motor_rr, 4);
        Serial.print(F(" RL="));
        printPadded(rxTelemetry.motor_rl, 4);
        Serial.println(F(" │"));
        
    } else {
        Serial.println(F("│ Waiting for drone connection...               │"));
        Serial.println(F("│                                               │"));
    }
    
    Serial.println(F("└─────────────────────────────────────────────────┘"));
}

void printInstructions() {
    Serial.println(F("┌─ FLIGHT PROCEDURE ────────────────────────────┐"));
    Serial.println(F("│ 1. Ensure SW2 (Kill) is OFF                   │"));
    Serial.println(F("│ 2. Press BTN1 to calibrate IMU                │"));
    Serial.println(F("│ 3. SW1=OFF + BTN2 for ESC calibration         │"));
    Serial.println(F("│ 4. SW1=ON, SW2=ON, THR=LOW + BTN2 to ARM      │"));
    Serial.println(F("│ 5. Increase throttle to fly                   │"));
    Serial.println(F("│ 6. SW2=OFF to emergency DISARM                │"));
    Serial.println(F("└─────────────────────────────────────────────────┘"));
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void printBar(int16_t value, int16_t minVal, int16_t maxVal, int width) {
    int filled = map(value, minVal, maxVal, 0, width);
    filled = constrain(filled, 0, width);
    
    Serial.print(F("["));
    for (int i = 0; i < width; i++) {
        Serial.print(i < filled ? F("█") : F("░"));
    }
    Serial.print(F("]"));
}

void printBarCentered(int16_t value, int16_t minVal, int16_t maxVal, int width) {
    int center = width / 2;
    int pos = map(value, minVal, maxVal, 0, width);
    pos = constrain(pos, 0, width);
    
    Serial.print(F("["));
    for (int i = 0; i < width; i++) {
        if (i == center) {
            Serial.print(F("│"));
        } else if ((pos <= center && i >= pos && i < center) ||
                   (pos > center && i > center && i <= pos)) {
            Serial.print(F("█"));
        } else {
            Serial.print(F("░"));
        }
    }
    Serial.print(F("]"));
}

void printPadded(int16_t value, int width) {
    int digits = 1;
    int temp = abs(value);
    while (temp >= 10) {
        temp /= 10;
        digits++;
    }
    
    for (int i = digits; i < width; i++) {
        Serial.print(F(" "));
    }
    Serial.print(value);
}

void printPaddedSigned(int16_t value, int width) {
    if (value >= 0) {
        Serial.print(F("+"));
        printPadded(value, width - 1);
    } else {
        printPadded(value, width);
    }
}

const char* getStatusString(uint8_t status) {
    switch (status) {
        case STATUS_BOOT:           return "BOOT";
        case STATUS_IDLE:           return "IDLE";
        case STATUS_CALIBRATING:    return "CALIBRATING";
        case STATUS_CALIBRATED:     return "CALIBRATED";
        case STATUS_CALIBRATION_FAIL: return "CAL_FAIL";
        case STATUS_ESC_CALIBRATING: return "ESC_CAL";
        case STATUS_ESC_CALIBRATED: return "ESC_OK";
        case STATUS_ARMED:          return "ARMED";
        case STATUS_DISARMED:       return "DISARMED";
        case STATUS_FLYING:         return "FLYING";
        case STATUS_LOW_BATTERY:    return "LOW_BATT";
        case STATUS_ERROR:          return "ERROR";
        default:                    return "UNKNOWN";
    }
}
