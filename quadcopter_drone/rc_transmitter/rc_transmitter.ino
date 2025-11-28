/*
 * ============================================================================
 * ARDUINO NANO RC TRANSMITTER
 * ============================================================================
 * 
 * Hardware:
 * - Arduino Nano (ATmega328P)
 * - NRF24L01 (2.4GHz wireless transceiver)
 * - 2x Analog Joysticks
 * - 1x Toggle Switch (Arm/Disarm)
 * - 2x Push Buttons
 * - 1x Status LED
 * 
 * Features:
 * - Joystick calibration with EEPROM storage
 * - Safe throttle mapping (stick down = motors off)
 * - Telemetry display
 * - Connection status indication
 * 
 * Control Layout:
 * - Left Joystick:  Throttle (Y), Yaw (X)
 * - Right Joystick: Pitch (Y), Roll (X)
 * - Toggle Switch:  Arm/Disarm
 * - Button 1:       Calibration
 * - Button 2:       Motor Test
 * 
 * Author: Embedded Systems Professional
 * ============================================================================
 */

#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>
#include "config.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// Pipe addresses (must match flight controller)
const uint64_t pipeAddressTX = NRF_PIPE_ADDRESS;
const uint64_t pipeAddressRX = NRF_PIPE_ADDRESS + 1;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Data structures
RCData rcData;
TelemetryData telemetry;
JoystickCalData joyCal;

// Button states (for debouncing)
bool lastBtn1State = HIGH;
bool lastBtn2State = HIGH;
bool lastArmState = HIGH;
unsigned long lastBtn1Press = 0;
unsigned long lastBtn2Press = 0;
#define DEBOUNCE_MS 50

// Timing
unsigned long lastTxTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long lastLEDToggle = 0;
bool ledState = false;

// Connection status
bool connected = false;
uint32_t txSuccessCount = 0;
uint32_t txFailCount = 0;

// Calibration mode
bool calibrationMode = false;
unsigned long calibrationStartTime = 0;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void initializeNRF();
void readJoysticks();
void readButtons();
void sendRCData();
bool receiveTelemetry();
void updateStatusLED();
void runJoystickCalibration();
void loadCalibration();
void saveCalibration();
uint16_t mapJoystickValue(uint16_t raw, uint16_t minVal, uint16_t centerVal, 
                          uint16_t maxVal, bool isThrottle);
uint16_t applyDeadzone(uint16_t value, uint16_t center, uint16_t deadzone);
void printStatus();
void printTelemetry();
uint8_t calculateChecksum(uint8_t* data, uint8_t length);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 2000);
    
    printWelcomeBanner();
    
    // Configure pins
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(BTN1_PIN, INPUT_PULLUP);
    pinMode(BTN2_PIN, INPUT_PULLUP);
    pinMode(ARM_SWITCH_PIN, INPUT_PULLUP);
    
    // Flash LED to indicate startup
    for (int i = 0; i < 3; i++) {
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(100);
        digitalWrite(STATUS_LED_PIN, LOW);
        delay(100);
    }
    
    // Load joystick calibration from EEPROM
    loadCalibration();
    
    // Initialize NRF24L01
    initializeNRF();
    
    // Initialize RC data to safe values
    rcData.throttle = OUTPUT_MIN;
    rcData.yaw = OUTPUT_CENTER;
    rcData.pitch = OUTPUT_CENTER;
    rcData.roll = OUTPUT_CENTER;
    rcData.armSwitch = 0;
    rcData.button1 = 0;
    rcData.button2 = 0;
    
    Serial.println(F(""));
    Serial.println(F("╔════════════════════════════════════════╗"));
    Serial.println(F("║       RC TRANSMITTER READY             ║"));
    Serial.println(F("╠════════════════════════════════════════╣"));
    Serial.println(F("║  • Set ARM switch to DISARM            ║"));
    Serial.println(F("║  • Move throttle to MINIMUM            ║"));
    Serial.println(F("║  • Power on flight controller          ║"));
    Serial.println(F("╚════════════════════════════════════════╝"));
    Serial.println(F(""));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    unsigned long now = millis();
    
    // Handle calibration mode
    if (calibrationMode) {
        runJoystickCalibration();
        return;
    }
    
    // Read inputs
    readJoysticks();
    readButtons();
    
    // Transmit data at fixed rate
    if (now - lastTxTime >= TX_RATE_MS) {
        sendRCData();
        lastTxTime = now;
        
        // Try to receive telemetry after each transmission
        if (receiveTelemetry()) {
            connected = true;
            lastTelemetryTime = now;
        }
    }
    
    // Check connection status
    if (now - lastTelemetryTime > 500) {
        connected = false;
    }
    
    // Update status LED
    updateStatusLED();
    
    // Print status periodically
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 500) {
        printStatus();
        if (connected) {
            printTelemetry();
        }
        lastPrint = now;
    }
}

// ============================================================================
// NRF24L01 INITIALIZATION
// ============================================================================

void initializeNRF() {
    Serial.print(F("Initializing NRF24L01... "));
    
    if (!radio.begin()) {
        Serial.println(F("FAILED!"));
        Serial.println(F("Check wiring: CE=D9, CSN=D10, MOSI=D11, MISO=D12, SCK=D13"));
        
        // Keep blinking LED to indicate error
        while (true) {
            digitalWrite(STATUS_LED_PIN, HIGH);
            delay(100);
            digitalWrite(STATUS_LED_PIN, LOW);
            delay(100);
        }
    }
    
    // Configure radio
    radio.setChannel(NRF_CHANNEL);
    radio.setPALevel(NRF_PA_LEVEL);
    radio.setDataRate(NRF_DATA_RATE);
    radio.setRetries(3, 10);  // 3 retries, 10*250us delay
    radio.setPayloadSize(sizeof(RCData));
    radio.setCRCLength(RF24_CRC_16);
    radio.setAutoAck(true);
    
    // Open pipes
    radio.openWritingPipe(pipeAddressTX);
    radio.openReadingPipe(1, pipeAddressRX);
    
    // Start in TX mode
    radio.stopListening();
    
    Serial.println(F("OK!"));
    Serial.print(F("Channel: "));
    Serial.println(NRF_CHANNEL);
}

// ============================================================================
// JOYSTICK READING
// ============================================================================

void readJoysticks() {
    // Read raw ADC values
    uint16_t rawThrottle = analogRead(LEFT_JOY_Y_PIN);
    uint16_t rawYaw = analogRead(LEFT_JOY_X_PIN);
    uint16_t rawPitch = analogRead(RIGHT_JOY_Y_PIN);
    uint16_t rawRoll = analogRead(RIGHT_JOY_X_PIN);
    
    // Map throttle (special handling - no centering)
    // When stick is down (rest position), output should be 1000 (motors off)
    rcData.throttle = mapJoystickValue(rawThrottle, 
                                        joyCal.throttleMin, joyCal.throttleCenter,
                                        joyCal.throttleMax, true);
    
    // Map yaw, pitch, roll (with center and deadzone)
    rcData.yaw = mapJoystickValue(rawYaw,
                                   joyCal.yawMin, joyCal.yawCenter,
                                   joyCal.yawMax, false);
    rcData.yaw = applyDeadzone(rcData.yaw, OUTPUT_CENTER, JOYSTICK_DEADZONE);
    
    rcData.pitch = mapJoystickValue(rawPitch,
                                     joyCal.pitchMin, joyCal.pitchCenter,
                                     joyCal.pitchMax, false);
    rcData.pitch = applyDeadzone(rcData.pitch, OUTPUT_CENTER, JOYSTICK_DEADZONE);
    
    rcData.roll = mapJoystickValue(rawRoll,
                                    joyCal.rollMin, joyCal.rollCenter,
                                    joyCal.rollMax, false);
    rcData.roll = applyDeadzone(rcData.roll, OUTPUT_CENTER, JOYSTICK_DEADZONE);
}

uint16_t mapJoystickValue(uint16_t raw, uint16_t minVal, uint16_t centerVal,
                          uint16_t maxVal, bool isThrottle) {
    uint16_t output;
    
    if (isThrottle) {
        // For throttle: map full range from min to max
        // Assuming stick down = min, stick up = max
        // This ensures stick at rest (down) = OUTPUT_MIN
        
        #if THROTTLE_HAS_SPRING
            // If throttle has centering spring, center = idle
            if (raw < centerVal) {
                output = map(raw, minVal, centerVal, OUTPUT_MIN, OUTPUT_CENTER);
            } else {
                output = map(raw, centerVal, maxVal, OUTPUT_CENTER, OUTPUT_MAX);
            }
        #else
            // No spring - full range mapping
            // Check if the joystick is inverted (min > max in calibration)
            if (minVal < maxVal) {
                output = map(raw, minVal, maxVal, OUTPUT_MIN, OUTPUT_MAX);
            } else {
                // Inverted joystick
                output = map(raw, maxVal, minVal, OUTPUT_MAX, OUTPUT_MIN);
            }
        #endif
    } else {
        // For yaw/pitch/roll: map with center point
        if (raw < centerVal) {
            // Below center
            output = map(raw, minVal, centerVal, OUTPUT_MIN, OUTPUT_CENTER);
        } else {
            // Above center
            output = map(raw, centerVal, maxVal, OUTPUT_CENTER, OUTPUT_MAX);
        }
    }
    
    // Constrain to valid range
    return constrain(output, OUTPUT_MIN, OUTPUT_MAX);
}

uint16_t applyDeadzone(uint16_t value, uint16_t center, uint16_t deadzone) {
    if (abs((int)value - (int)center) < deadzone) {
        return center;
    }
    return value;
}

// ============================================================================
// BUTTON READING
// ============================================================================

void readButtons() {
    unsigned long now = millis();
    
    // Read button 1 (calibration) with debounce
    bool btn1State = digitalRead(BTN1_PIN);
    if (btn1State == LOW && lastBtn1State == HIGH && (now - lastBtn1Press) > DEBOUNCE_MS) {
        rcData.button1 = 1;
        lastBtn1Press = now;
        
        // Check for long press to enter calibration mode
        unsigned long pressStart = now;
        while (digitalRead(BTN1_PIN) == LOW && (millis() - pressStart) < 3000) {
            // Wait
        }
        if (millis() - pressStart >= 3000) {
            // Long press - enter calibration mode
            calibrationMode = true;
            calibrationStartTime = millis();
            Serial.println(F("\n>>> ENTERING CALIBRATION MODE <<<\n"));
        }
    } else {
        rcData.button1 = 0;
    }
    lastBtn1State = btn1State;
    
    // Read button 2 (motor test) with debounce
    bool btn2State = digitalRead(BTN2_PIN);
    if (btn2State == LOW && lastBtn2State == HIGH && (now - lastBtn2Press) > DEBOUNCE_MS) {
        rcData.button2 = 1;
        lastBtn2Press = now;
        digitalWrite(STATUS_LED_PIN, HIGH);  // Flash LED
    } else {
        rcData.button2 = 0;
    }
    lastBtn2State = btn2State;
    
    // Read arm switch
    // LOW = armed (switch flipped), HIGH = disarmed (default position)
    rcData.armSwitch = (digitalRead(ARM_SWITCH_PIN) == LOW) ? 1 : 0;
}

// ============================================================================
// DATA TRANSMISSION
// ============================================================================

void sendRCData() {
    // Calculate checksum
    rcData.checksum = calculateChecksum((uint8_t*)&rcData, sizeof(RCData) - 1);
    
    // Ensure in TX mode
    radio.stopListening();
    
    // Send data
    bool success = radio.write(&rcData, sizeof(RCData));
    
    if (success) {
        txSuccessCount++;
        
        // Blink LED on successful transmission
        if (!connected) {
            digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        }
    } else {
        txFailCount++;
    }
}

bool receiveTelemetry() {
    // Switch to RX mode
    radio.startListening();
    
    // Wait briefly for response
    unsigned long startTime = millis();
    while (!radio.available()) {
        if (millis() - startTime > 5) {
            radio.stopListening();
            return false;
        }
    }
    
    // Read telemetry
    radio.read(&telemetry, sizeof(TelemetryData));
    
    // Back to TX mode
    radio.stopListening();
    
    return true;
}

uint8_t calculateChecksum(uint8_t* data, uint8_t length) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// ============================================================================
// STATUS LED
// ============================================================================

void updateStatusLED() {
    unsigned long now = millis();
    
    if (connected) {
        // Slow blink when connected
        if (now - lastLEDToggle >= LED_BLINK_RATE) {
            ledState = !ledState;
            digitalWrite(STATUS_LED_PIN, ledState);
            lastLEDToggle = now;
        }
    } else {
        // Fast blink when not connected
        if (now - lastLEDToggle >= 100) {
            ledState = !ledState;
            digitalWrite(STATUS_LED_PIN, ledState);
            lastLEDToggle = now;
        }
    }
}

// ============================================================================
// JOYSTICK CALIBRATION
// ============================================================================

void runJoystickCalibration() {
    static int calibrationStep = 0;
    static unsigned long stepStartTime = 0;
    
    switch (calibrationStep) {
        case 0:
            // Introduction
            Serial.println(F(""));
            Serial.println(F("╔════════════════════════════════════════════════════════╗"));
            Serial.println(F("║           JOYSTICK CALIBRATION                         ║"));
            Serial.println(F("╠════════════════════════════════════════════════════════╣"));
            Serial.println(F("║  This calibration ensures safe throttle operation:     ║"));
            Serial.println(F("║                                                        ║"));
            Serial.println(F("║  • Stick DOWN = Throttle MIN (motors off)              ║"));
            Serial.println(F("║  • Stick UP   = Throttle MAX (full power)              ║"));
            Serial.println(F("║                                                        ║"));
            Serial.println(F("║  This prevents accidental motor startup!               ║"));
            Serial.println(F("╚════════════════════════════════════════════════════════╝"));
            Serial.println(F(""));
            delay(2000);
            calibrationStep = 1;
            stepStartTime = millis();
            break;
            
        case 1:
            // Step 1: Release all sticks to center
            Serial.println(F("Step 1: Release all joysticks (let them center)"));
            Serial.println(F("        Waiting 3 seconds..."));
            delay(3000);
            
            // Read center values
            joyCal.throttleCenter = analogRead(LEFT_JOY_Y_PIN);
            joyCal.yawCenter = analogRead(LEFT_JOY_X_PIN);
            joyCal.pitchCenter = analogRead(RIGHT_JOY_Y_PIN);
            joyCal.rollCenter = analogRead(RIGHT_JOY_X_PIN);
            
            Serial.println(F("Centers recorded:"));
            Serial.print(F("  Throttle: ")); Serial.println(joyCal.throttleCenter);
            Serial.print(F("  Yaw:      ")); Serial.println(joyCal.yawCenter);
            Serial.print(F("  Pitch:    ")); Serial.println(joyCal.pitchCenter);
            Serial.print(F("  Roll:     ")); Serial.println(joyCal.rollCenter);
            
            // Initialize min/max
            joyCal.throttleMin = joyCal.throttleMax = joyCal.throttleCenter;
            joyCal.yawMin = joyCal.yawMax = joyCal.yawCenter;
            joyCal.pitchMin = joyCal.pitchMax = joyCal.pitchCenter;
            joyCal.rollMin = joyCal.rollMax = joyCal.rollCenter;
            
            calibrationStep = 2;
            stepStartTime = millis();
            break;
            
        case 2:
            // Step 2: Move sticks to extremes
            Serial.println(F(""));
            Serial.println(F("Step 2: Move ALL sticks to EXTREME positions"));
            Serial.println(F("        Push each stick: UP, DOWN, LEFT, RIGHT"));
            Serial.println(F("        You have 10 seconds..."));
            Serial.println(F(""));
            
            stepStartTime = millis();
            calibrationStep = 3;
            break;
            
        case 3:
            // Collecting extremes
            if (millis() - stepStartTime < 10000) {
                // Read and update extremes
                uint16_t t = analogRead(LEFT_JOY_Y_PIN);
                uint16_t y = analogRead(LEFT_JOY_X_PIN);
                uint16_t p = analogRead(RIGHT_JOY_Y_PIN);
                uint16_t r = analogRead(RIGHT_JOY_X_PIN);
                
                if (t < joyCal.throttleMin) joyCal.throttleMin = t;
                if (t > joyCal.throttleMax) joyCal.throttleMax = t;
                if (y < joyCal.yawMin) joyCal.yawMin = y;
                if (y > joyCal.yawMax) joyCal.yawMax = y;
                if (p < joyCal.pitchMin) joyCal.pitchMin = p;
                if (p > joyCal.pitchMax) joyCal.pitchMax = p;
                if (r < joyCal.rollMin) joyCal.rollMin = r;
                if (r > joyCal.rollMax) joyCal.rollMax = r;
                
                // Progress indicator
                static unsigned long lastDot = 0;
                if (millis() - lastDot > 500) {
                    Serial.print(".");
                    lastDot = millis();
                }
            } else {
                Serial.println(F(" Done!"));
                calibrationStep = 4;
            }
            break;
            
        case 4:
            // Show results and save
            Serial.println(F(""));
            Serial.println(F("Calibration Results:"));
            Serial.println(F("────────────────────────────────────────"));
            Serial.print(F("Throttle: Min=")); Serial.print(joyCal.throttleMin);
            Serial.print(F(" Center=")); Serial.print(joyCal.throttleCenter);
            Serial.print(F(" Max=")); Serial.println(joyCal.throttleMax);
            
            Serial.print(F("Yaw:      Min=")); Serial.print(joyCal.yawMin);
            Serial.print(F(" Center=")); Serial.print(joyCal.yawCenter);
            Serial.print(F(" Max=")); Serial.println(joyCal.yawMax);
            
            Serial.print(F("Pitch:    Min=")); Serial.print(joyCal.pitchMin);
            Serial.print(F(" Center=")); Serial.print(joyCal.pitchCenter);
            Serial.print(F(" Max=")); Serial.println(joyCal.pitchMax);
            
            Serial.print(F("Roll:     Min=")); Serial.print(joyCal.rollMin);
            Serial.print(F(" Center=")); Serial.print(joyCal.rollCenter);
            Serial.print(F(" Max=")); Serial.println(joyCal.rollMax);
            
            // Validate
            bool valid = true;
            if (joyCal.throttleMax - joyCal.throttleMin < 300) {
                Serial.println(F("WARNING: Throttle range too small!"));
                valid = false;
            }
            
            if (valid) {
                // Save to EEPROM
                saveCalibration();
                Serial.println(F(""));
                Serial.println(F("✓ Calibration saved to EEPROM!"));
            } else {
                Serial.println(F(""));
                Serial.println(F("✗ Calibration failed - please retry"));
            }
            
            calibrationStep = 5;
            break;
            
        case 5:
            // Exit calibration mode
            Serial.println(F(""));
            Serial.println(F("Exiting calibration mode..."));
            calibrationMode = false;
            calibrationStep = 0;
            break;
    }
}

void loadCalibration() {
    Serial.print(F("Loading joystick calibration... "));
    
    // Check signature
    if (EEPROM.read(EEPROM_JOY_SIGNATURE) == 0xCA) {
        EEPROM.get(EEPROM_JOY_DATA, joyCal);
        Serial.println(F("OK"));
        
        Serial.println(F("Calibration values:"));
        Serial.print(F("  Throttle: ")); Serial.print(joyCal.throttleMin);
        Serial.print(F("-")); Serial.print(joyCal.throttleCenter);
        Serial.print(F("-")); Serial.println(joyCal.throttleMax);
    } else {
        // Use defaults
        Serial.println(F("Not found - using defaults"));
        
        joyCal.throttleMin = 0;
        joyCal.throttleMax = 1023;
        joyCal.throttleCenter = 512;
        joyCal.yawMin = 0;
        joyCal.yawMax = 1023;
        joyCal.yawCenter = 512;
        joyCal.pitchMin = 0;
        joyCal.pitchMax = 1023;
        joyCal.pitchCenter = 512;
        joyCal.rollMin = 0;
        joyCal.rollMax = 1023;
        joyCal.rollCenter = 512;
        
        Serial.println(F(""));
        Serial.println(F("╔════════════════════════════════════════╗"));
        Serial.println(F("║  Hold Button 1 for 3 seconds to        ║"));
        Serial.println(F("║  start joystick calibration            ║"));
        Serial.println(F("╚════════════════════════════════════════╝"));
    }
}

void saveCalibration() {
    joyCal.signature = 0xCA;
    EEPROM.write(EEPROM_JOY_SIGNATURE, 0xCA);
    EEPROM.put(EEPROM_JOY_DATA, joyCal);
}

// ============================================================================
// STATUS DISPLAY
// ============================================================================

void printStatus() {
    Serial.println(F(""));
    Serial.println(F("┌────────────────────────────────────────────────────────┐"));
    Serial.print(F("│ RC STATUS: "));
    Serial.print(connected ? F("CONNECTED ") : F("SEARCHING "));
    Serial.print(F("| ARM: "));
    Serial.print(rcData.armSwitch ? F("ARMED  ") : F("SAFE   "));
    Serial.print(F("| TX: "));
    Serial.print(txSuccessCount);
    Serial.print(F("/"));
    Serial.print(txSuccessCount + txFailCount);
    Serial.println(F("  │"));
    Serial.println(F("├────────────────────────────────────────────────────────┤"));
    
    // Control values
    Serial.print(F("│ THR: "));
    printPaddedNumber(rcData.throttle, 4);
    Serial.print(F(" │ YAW: "));
    printPaddedNumber(rcData.yaw, 4);
    Serial.print(F(" │ PIT: "));
    printPaddedNumber(rcData.pitch, 4);
    Serial.print(F(" │ ROL: "));
    printPaddedNumber(rcData.roll, 4);
    Serial.println(F(" │"));
    
    // Visual throttle bar
    Serial.print(F("│ Throttle: ["));
    int bars = (rcData.throttle - OUTPUT_MIN) / 50;
    for (int i = 0; i < 20; i++) {
        Serial.print(i < bars ? '#' : '-');
    }
    Serial.print(F("] "));
    Serial.print((rcData.throttle - OUTPUT_MIN) / 10);
    Serial.println(F("%              │"));
    
    Serial.println(F("└────────────────────────────────────────────────────────┘"));
}

void printTelemetry() {
    Serial.println(F("┌─────────────────── TELEMETRY ───────────────────────────┐"));
    Serial.print(F("│ Roll: "));
    Serial.print(telemetry.roll / 10.0f, 1);
    Serial.print(F("° │ Pitch: "));
    Serial.print(telemetry.pitch / 10.0f, 1);
    Serial.print(F("° │ Alt: "));
    Serial.print(telemetry.altitude / 100.0f, 2);
    Serial.println(F("m         │"));
    
    Serial.print(F("│ Battery: "));
    Serial.print(telemetry.batteryVoltage / 100.0f, 1);
    Serial.print(F("V │ Mode: "));
    Serial.print(telemetry.flightMode);
    Serial.print(F(" │ RSSI: "));
    Serial.print(telemetry.rssi);
    Serial.println(F("dBm         │"));
    
    Serial.print(F("│ Status: "));
    Serial.print(telemetry.armed ? F("ARMED   ") : F("DISARMED"));
    Serial.print(F(" │ Flags: 0x"));
    if (telemetry.status < 0x10) Serial.print(F("0"));
    Serial.print(telemetry.status, HEX);
    Serial.println(F("                           │"));
    
    Serial.println(F("└────────────────────────────────────────────────────────┘"));
}

void printPaddedNumber(int num, int width) {
    int digits = 1;
    int temp = num;
    while (temp >= 10) {
        temp /= 10;
        digits++;
    }
    for (int i = digits; i < width; i++) {
        Serial.print(' ');
    }
    Serial.print(num);
}

void printWelcomeBanner() {
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("╔══════════════════════════════════════════════════════════════════╗"));
    Serial.println(F("║                                                                  ║"));
    Serial.println(F("║    ██████╗  ██████╗    ████████╗██╗  ██╗                         ║"));
    Serial.println(F("║    ██╔══██╗██╔════╝    ╚══██╔══╝╚██╗██╔╝                         ║"));
    Serial.println(F("║    ██████╔╝██║            ██║    ╚███╔╝                          ║"));
    Serial.println(F("║    ██╔══██╗██║            ██║    ██╔██╗                          ║"));
    Serial.println(F("║    ██║  ██║╚██████╗       ██║   ██╔╝ ██╗                         ║"));
    Serial.println(F("║    ╚═╝  ╚═╝ ╚═════╝       ╚═╝   ╚═╝  ╚═╝                         ║"));
    Serial.println(F("║                                                                  ║"));
    Serial.println(F("║           RC TRANSMITTER FOR QUADCOPTER                          ║"));
    Serial.println(F("║                    v1.0                                          ║"));
    Serial.println(F("║                                                                  ║"));
    Serial.println(F("╚══════════════════════════════════════════════════════════════════╝"));
    Serial.println(F(""));
}
