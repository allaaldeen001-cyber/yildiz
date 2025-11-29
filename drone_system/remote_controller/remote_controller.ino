/*
 * ============================================================================
 * PROFESSIONAL DRONE REMOTE CONTROLLER
 * Arduino Nano Based RC Transmitter
 * ============================================================================
 * 
 * Hardware Configuration:
 * - Arduino Nano (ATmega328P)
 * - NRF24L01 PA+LNA (CE: D9, CSN: D10)
 * - Left Joystick:  V(Throttle): A0, H(Yaw): A1
 * - Right Joystick: V(Pitch): A2, H(Roll): A3
 * - Button 1 (Calibration): D4
 * - Button 2 (Motors On): D5
 * - Switch 1 (Alt Hold): D2
 * - Switch 2 (Arm/Kill): D3
 * 
 * Control Mapping:
 * ┌─────────────────────────────────────────────────────────────┐
 * │  LEFT STICK              RIGHT STICK                        │
 * │  ┌───────┐               ┌───────┐                          │
 * │  │   ↑   │ Throttle+     │   ↑   │ Pitch Forward            │
 * │  │ ← ○ → │ Yaw L/R       │ ← ○ → │ Roll Left/Right          │
 * │  │   ↓   │ Throttle-     │   ↓   │ Pitch Backward           │
 * │  └───────┘               └───────┘                          │
 * │                                                             │
 * │  SW1: Altitude Hold    SW2: Arm/Disarm (Kill Switch)        │
 * │  BTN1: Calibration     BTN2: Enable Motors                  │
 * └─────────────────────────────────────────────────────────────┘
 * 
 * Author: UAV Systems Engineer
 * Version: 1.0.0
 * ============================================================================
 */

#include <SPI.h>
#include <RF24.h>
#include "config.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// NRF24L01
#define NRF_CE_PIN            9
#define NRF_CSN_PIN           10

// Joysticks (Analog)
#define JOY_LEFT_V_PIN        A0    // Throttle
#define JOY_LEFT_H_PIN        A1    // Yaw
#define JOY_RIGHT_V_PIN       A2    // Pitch
#define JOY_RIGHT_H_PIN       A3    // Roll

// Buttons (Digital, active LOW)
#define BTN_CALIBRATE_PIN     4     // Button 1
#define BTN_MOTORS_PIN        5     // Button 2

// Switches (Digital, active LOW)
#define SW_ALT_HOLD_PIN       2     // Switch 1 - Altitude Hold
#define SW_ARM_PIN            3     // Switch 2 - Arm/Disarm

// ============================================================================
// CONFIGURATION
// ============================================================================

// Joystick calibration (default values, can be calibrated)
struct JoystickCalibration {
    int16_t minVal;
    int16_t maxVal;
    int16_t center;
    bool reversed;
};

JoystickCalibration joyCal[4] = {
    {0, 1023, 512, false},  // Throttle (A0) - not reversed
    {0, 1023, 512, false},  // Yaw (A1)
    {0, 1023, 512, false},  // Pitch (A2)
    {0, 1023, 512, false}   // Roll (A3)
};

// Deadzone for joysticks
#define DEADZONE              30

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// NRF24L01
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// Communication packets
CommandPacket_t txCommand;
TelemetryPacket_t rxTelemetry;

// Joystick readings
int16_t throttleRaw, yawRaw, pitchRaw, rollRaw;
int16_t throttle, yaw, pitch, roll;

// Button/Switch state
bool btnCalibrate = false, btnMotors = false;
bool swAltHold = false, swArm = false;
bool prevBtnCalibrate = false, prevBtnMotors = false;

// Communication status
bool isConnected = false;
uint32_t lastTxTime = 0;
uint32_t lastRxTime = 0;
uint8_t txSequence = 0;
uint8_t failedTxCount = 0;
uint8_t successTxCount = 0;

// Display update
uint32_t lastDisplayTime = 0;
#define DISPLAY_INTERVAL      250  // 4Hz display update

// Status from FC
uint8_t fcStatus = 0;
int16_t fcPitch = 0, fcRoll = 0, fcYaw = 0;
uint16_t fcAltitude = 0;
uint8_t fcBattery = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void initNRF24();
void readJoysticks();
void readButtons();
int16_t applyDeadzone(int16_t value, int16_t center, int16_t deadzone);
int16_t mapJoystick(int16_t raw, JoystickCalibration* cal, bool centerReturn);
void buildPacket();
void sendPacket();
void processAck();
void printStatus();
void printHeader();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial for monitoring
    Serial.begin(115200);
    
    // Print header
    printHeader();
    
    // Initialize pins
    pinMode(BTN_CALIBRATE_PIN, INPUT_PULLUP);
    pinMode(BTN_MOTORS_PIN, INPUT_PULLUP);
    pinMode(SW_ALT_HOLD_PIN, INPUT_PULLUP);
    pinMode(SW_ARM_PIN, INPUT_PULLUP);
    
    // Initialize NRF24L01
    Serial.println(F("[INIT] Initializing NRF24L01..."));
    initNRF24();
    
    // Initialize packet
    txCommand.header = 0xAA;
    
    // Initial readings
    readJoysticks();
    readButtons();
    
    Serial.println(F("[INIT] Remote Controller Ready!"));
    Serial.println(F("========================================"));
    Serial.println(F(""));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Read inputs
    readJoysticks();
    readButtons();
    
    // Build and send packet (50Hz)
    if (millis() - lastTxTime >= COMM_UPDATE_RATE_MS) {
        lastTxTime = millis();
        buildPacket();
        sendPacket();
    }
    
    // Update display (4Hz)
    if (millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
        lastDisplayTime = millis();
        printStatus();
    }
    
    // Check connection timeout
    if (millis() - lastRxTime > COMM_TIMEOUT_MS) {
        if (isConnected) {
            isConnected = false;
            Serial.println(F("\n[WARN] Connection to drone LOST!"));
        }
    }
}

// ============================================================================
// NRF24L01 INITIALIZATION
// ============================================================================

void initNRF24() {
    if (!radio.begin()) {
        Serial.println(F("[ERROR] NRF24L01 not responding!"));
        Serial.println(F("[ERROR] Check wiring: CE=D9, CSN=D10"));
        while (1) {
            delay(1000);
        }
    }
    
    // Configure radio
    radio.setChannel(NRF_CHANNEL);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(sizeof(CommandPacket_t));
    
    // Enable ACK payloads
    radio.enableAckPayload();
    radio.enableDynamicPayloads();
    radio.setAutoAck(true);
    radio.setRetries(3, 5);  // 3 retries, 1.5ms delay
    
    // Set addresses (opposite of FC)
    radio.openWritingPipe(RC_TO_FC_ADDR);
    radio.openReadingPipe(1, FC_TO_RC_ADDR);
    
    // Start in TX mode
    radio.stopListening();
    
    Serial.println(F("[NRF24] Initialized"));
    Serial.print(F("[NRF24] Channel: "));
    Serial.println(NRF_CHANNEL);
    Serial.print(F("[NRF24] Data Rate: 250Kbps"));
    Serial.println(F(""));
}

// ============================================================================
// INPUT READING
// ============================================================================

void readJoysticks() {
    // Read raw ADC values
    throttleRaw = analogRead(JOY_LEFT_V_PIN);
    yawRaw = analogRead(JOY_LEFT_H_PIN);
    pitchRaw = analogRead(JOY_RIGHT_V_PIN);
    rollRaw = analogRead(JOY_RIGHT_H_PIN);
    
    // Map throttle (0-1000, no center return)
    throttle = mapJoystick(throttleRaw, &joyCal[0], false);
    if (throttle < 0) throttle = 0;  // Throttle is always 0-1000
    
    // Map yaw, pitch, roll (-500 to +500, with center return)
    yaw = mapJoystick(yawRaw, &joyCal[1], true);
    pitch = mapJoystick(pitchRaw, &joyCal[2], true);
    roll = mapJoystick(rollRaw, &joyCal[3], true);
    
    // Apply deadzone
    yaw = applyDeadzone(yaw, 0, DEADZONE);
    pitch = applyDeadzone(pitch, 0, DEADZONE);
    roll = applyDeadzone(roll, 0, DEADZONE);
}

void readButtons() {
    // Read buttons (active LOW)
    bool newBtnCalibrate = !digitalRead(BTN_CALIBRATE_PIN);
    bool newBtnMotors = !digitalRead(BTN_MOTORS_PIN);
    
    // Debounce (detect rising edge)
    static uint32_t lastBtnTime = 0;
    if (millis() - lastBtnTime > 50) {
        if (newBtnCalibrate && !prevBtnCalibrate) {
            btnCalibrate = true;
            lastBtnTime = millis();
        } else {
            btnCalibrate = false;
        }
        
        if (newBtnMotors && !prevBtnMotors) {
            btnMotors = true;
            lastBtnTime = millis();
        } else {
            btnMotors = false;
        }
    }
    
    prevBtnCalibrate = newBtnCalibrate;
    prevBtnMotors = newBtnMotors;
    
    // Read switches (active LOW, directly map state)
    swAltHold = !digitalRead(SW_ALT_HOLD_PIN);
    swArm = !digitalRead(SW_ARM_PIN);
}

int16_t applyDeadzone(int16_t value, int16_t center, int16_t deadzone) {
    int16_t diff = value - center;
    if (abs(diff) < deadzone) {
        return center;
    }
    return value;
}

int16_t mapJoystick(int16_t raw, JoystickCalibration* cal, bool centerReturn) {
    int16_t value;
    
    if (cal->reversed) {
        raw = cal->maxVal - (raw - cal->minVal);
    }
    
    if (centerReturn) {
        // Map to -500 to +500
        if (raw < cal->center) {
            value = map(raw, cal->minVal, cal->center, -500, 0);
        } else {
            value = map(raw, cal->center, cal->maxVal, 0, 500);
        }
        value = constrain(value, -500, 500);
    } else {
        // Map to 0-1000
        value = map(raw, cal->minVal, cal->maxVal, 0, 1000);
        value = constrain(value, 0, 1000);
    }
    
    return value;
}

// ============================================================================
// PACKET HANDLING
// ============================================================================

void buildPacket() {
    txCommand.header = 0xAA;
    txCommand.throttle = (uint16_t)throttle;
    txCommand.yaw = yaw;
    txCommand.pitch = pitch;
    txCommand.roll = roll;
    
    // Build button flags
    txCommand.buttons = 0;
    if (btnCalibrate) txCommand.buttons |= BTN_CALIBRATE;
    if (btnMotors) txCommand.buttons |= BTN_MOTORS_ON;
    
    // Build switch flags
    txCommand.switches = 0;
    if (swAltHold) txCommand.switches |= SW_ALT_HOLD;
    if (swArm) txCommand.switches |= SW_ARM;
    
    // No special command
    txCommand.command = CMD_NONE;
    
    // Sequence number
    txCommand.sequence = txSequence++;
    
    // Calculate checksum
    txCommand.checksum = calculateChecksum((uint8_t*)&txCommand, sizeof(CommandPacket_t));
}

void sendPacket() {
    bool success = radio.write(&txCommand, sizeof(CommandPacket_t));
    
    if (success) {
        successTxCount++;
        failedTxCount = 0;
        
        // Check for ACK payload
        if (radio.isAckPayloadAvailable()) {
            radio.read(&rxTelemetry, sizeof(TelemetryPacket_t));
            processAck();
        }
    } else {
        failedTxCount++;
        if (failedTxCount > 10) {
            if (isConnected) {
                isConnected = false;
            }
        }
    }
}

void processAck() {
    // Validate telemetry packet
    if (validateTelemetryPacket(&rxTelemetry)) {
        lastRxTime = millis();
        
        if (!isConnected) {
            isConnected = true;
            Serial.println(F("\n[INFO] Connected to drone!"));
        }
        
        // Update telemetry data
        fcStatus = rxTelemetry.status;
        fcPitch = rxTelemetry.pitch;
        fcRoll = rxTelemetry.roll;
        fcYaw = rxTelemetry.yaw;
        fcAltitude = rxTelemetry.altitude;
        fcBattery = rxTelemetry.battery;
    }
}

// ============================================================================
// SERIAL DISPLAY
// ============================================================================

void printHeader() {
    Serial.println(F(""));
    Serial.println(F("╔══════════════════════════════════════════════════════════════╗"));
    Serial.println(F("║     PROFESSIONAL DRONE REMOTE CONTROLLER v1.0                ║"));
    Serial.println(F("║     Arduino Nano - NRF24L01 PA+LNA                           ║"));
    Serial.println(F("╠══════════════════════════════════════════════════════════════╣"));
    Serial.println(F("║  Controls:                                                   ║"));
    Serial.println(F("║  Left Stick:  Throttle (Up/Down), Yaw (Left/Right)           ║"));
    Serial.println(F("║  Right Stick: Pitch (Up/Down), Roll (Left/Right)             ║"));
    Serial.println(F("║                                                              ║"));
    Serial.println(F("║  Button 1 (D4): Calibration                                  ║"));
    Serial.println(F("║  Button 2 (D5): Enable Motors / ESC Cal                      ║"));
    Serial.println(F("║  Switch 1 (D2): Altitude Hold                                ║"));
    Serial.println(F("║  Switch 2 (D3): Arm/Disarm (Kill Switch)                     ║"));
    Serial.println(F("╚══════════════════════════════════════════════════════════════╝"));
    Serial.println(F(""));
}

void printStatus() {
    // Clear line and print status
    Serial.println(F(""));
    Serial.println(F("────────────────────────────────────────────────────────────────"));
    
    // Connection status
    Serial.print(F("│ LINK: "));
    if (isConnected) {
        Serial.print(F("✓ CONNECTED"));
    } else {
        Serial.print(F("✗ DISCONNECTED"));
    }
    Serial.print(F("  │  TX: "));
    Serial.print(successTxCount % 1000);
    Serial.println(F(" pkts"));
    
    // Joystick values
    Serial.println(F("├──────────────────────────────────────────────────────────────┤"));
    Serial.print(F("│ STICKS:  THR: "));
    printPadded(throttle, 4);
    Serial.print(F("  YAW: "));
    printPaddedSigned(yaw, 4);
    Serial.print(F("  PIT: "));
    printPaddedSigned(pitch, 4);
    Serial.print(F("  ROL: "));
    printPaddedSigned(roll, 4);
    Serial.println(F(" │"));
    
    // Buttons and switches
    Serial.println(F("├──────────────────────────────────────────────────────────────┤"));
    Serial.print(F("│ CONTROLS: "));
    Serial.print(F("BTN1["));
    Serial.print(!digitalRead(BTN_CALIBRATE_PIN) ? F("X") : F(" "));
    Serial.print(F("] BTN2["));
    Serial.print(!digitalRead(BTN_MOTORS_PIN) ? F("X") : F(" "));
    Serial.print(F("] SW1["));
    Serial.print(swAltHold ? F("ON ") : F("OFF"));
    Serial.print(F("] SW2["));
    Serial.print(swArm ? F("ARM") : F("DIS"));
    Serial.println(F("]    │"));
    
    // Drone status (if connected)
    if (isConnected) {
        Serial.println(F("├──────────────────────────────────────────────────────────────┤"));
        Serial.print(F("│ DRONE STATUS: "));
        
        // Status flags
        if (fcStatus & STATUS_ARMED) Serial.print(F("[ARMED] "));
        else Serial.print(F("[SAFE]  "));
        
        if (fcStatus & STATUS_CALIBRATED) Serial.print(F("[CAL] "));
        else Serial.print(F("[---] "));
        
        if (fcStatus & STATUS_ALT_HOLD) Serial.print(F("[ALT] "));
        else Serial.print(F("[---] "));
        
        if (fcStatus & STATUS_MOTORS_ON) Serial.print(F("[MOT] "));
        else Serial.print(F("[---] "));
        
        Serial.println(F("          │"));
        
        Serial.println(F("├──────────────────────────────────────────────────────────────┤"));
        Serial.print(F("│ ANGLES:  PIT: "));
        printPaddedFloat(fcPitch / 10.0, 6);
        Serial.print(F("°  ROL: "));
        printPaddedFloat(fcRoll / 10.0, 6);
        Serial.print(F("°  YAW: "));
        printPaddedFloat(fcYaw / 10.0, 6);
        Serial.println(F("° │"));
        
        Serial.print(F("│ ALT: "));
        printPaddedFloat(fcAltitude / 100.0, 6);
        Serial.print(F(" m   BATT: "));
        printPadded(fcBattery, 3);
        Serial.println(F("%                          │"));
    }
    
    Serial.println(F("────────────────────────────────────────────────────────────────"));
    
    // Instructions based on state
    Serial.println(F(""));
    if (!isConnected) {
        Serial.println(F(">>> Waiting for drone connection..."));
        Serial.println(F(">>> Make sure drone is powered ON"));
    } else if (!(fcStatus & STATUS_CALIBRATED)) {
        Serial.println(F(">>> Press BTN1 to calibrate the drone (keep it still)"));
    } else if (!(fcStatus & STATUS_ESC_CALIBRATED)) {
        Serial.println(F(">>> Set SW1 to OFF, then press BTN2 for ESC calibration"));
    } else if (!(fcStatus & STATUS_MOTORS_ON)) {
        Serial.println(F(">>> Press BTN2 to enable motors"));
    } else if (!(fcStatus & STATUS_ARMED)) {
        Serial.println(F(">>> Set SW2 to ARM position to arm the drone"));
    } else {
        Serial.println(F(">>> READY TO FLY! Use sticks to control."));
        Serial.println(F(">>> SW2 to OFF = EMERGENCY STOP"));
    }
}

void printPadded(int16_t value, uint8_t width) {
    char buffer[8];
    sprintf(buffer, "%*d", width, value);
    Serial.print(buffer);
}

void printPaddedSigned(int16_t value, uint8_t width) {
    char buffer[8];
    if (value >= 0) {
        sprintf(buffer, "+%*d", width - 1, value);
    } else {
        sprintf(buffer, "%*d", width, value);
    }
    Serial.print(buffer);
}

void printPaddedFloat(float value, uint8_t width) {
    char buffer[12];
    dtostrf(value, width, 1, buffer);
    Serial.print(buffer);
}
