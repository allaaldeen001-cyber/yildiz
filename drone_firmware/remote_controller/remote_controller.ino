/**
 * ============================================================================
 * PROFESSIONAL DRONE REMOTE CONTROLLER FIRMWARE
 * ============================================================================
 * 
 * Complete remote control transmitter for Arduino Nano
 * 
 * Hardware:
 * - Arduino Nano (ATmega328P)
 * - NRF24L01 PA+LNA (CE→D9, CSN→D10)
 * - Left Joystick: Throttle (A0), Yaw (A1)
 * - Right Joystick: Pitch (A2), Roll (A3)
 * - Button 1 (D4): IMU Calibration
 * - Button 2 (D5): Motor ON / ESC Calibration
 * - Switch 1 (D2): Altitude Hold
 * - Switch 2 (D3): ARM / DISARM (Kill Switch)
 * 
 * Features:
 * - 100 Hz command transmission
 * - Bidirectional telemetry via ACK payloads
 * - Button debouncing
 * - Serial status output
 * - Joystick filtering and calibration
 * 
 * Author: UAV Firmware Engineer
 * Version: 1.0.0
 * 
 * ============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

#include "config.h"
#include "../shared/protocol.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// TX packet
RCCommandPacket txPacket;

// RX telemetry
FCTelemetryPacket rxTelemetry;
bool telemetryReceived = false;

// ============================================================================
// STATE VARIABLES
// ============================================================================

// Timing
uint32_t lastTxTime = 0;
uint32_t lastSerialTime = 0;
uint8_t packetId = 0;

// Communication state
bool linked = false;
uint32_t lastAckTime = 0;
uint32_t txCount = 0;
uint32_t ackCount = 0;

// Button states
bool button1State = false;
bool button2State = false;
bool button1LastState = false;
bool button2LastState = false;
uint32_t button1LastDebounce = 0;
uint32_t button2LastDebounce = 0;
bool button1Pressed = false;  // Single-shot flag
bool button2Pressed = false;  // Single-shot flag

// Switch states
bool swAltHold = false;
bool swArm = false;

// Joystick values
uint16_t throttleRaw = 0;
uint16_t yawRaw = 0;
uint16_t pitchRaw = 0;
uint16_t rollRaw = 0;

// Channel outputs
uint16_t throttleChannel = CHANNEL_MIN;
uint16_t yawChannel = CHANNEL_CENTER;
uint16_t pitchChannel = CHANNEL_CENTER;
uint16_t rollChannel = CHANNEL_CENTER;

// Joystick filter states
float throttleFiltered = 0;
float yawFiltered = 0;
float pitchFiltered = 0;
float rollFiltered = 0;
const float FILTER_ALPHA = 0.7f;  // Low-pass filter coefficient

// Calibration request tracking
bool calibrationRequested = false;
bool escCalibrationRequested = false;
bool motorTestRequested = false;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void initializeNRF();
void readInputs();
void processJoysticks();
void processButtons();
void processSwitches();
void buildPacket();
void transmitPacket();
void receiveAck();
void updateSerial();
void updateLED();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial
    #if ENABLE_SERIAL
    Serial.begin(SERIAL_BAUD);
    Serial.println(F(""));
    Serial.println(F("===================================="));
    Serial.println(F("   DRONE REMOTE CONTROLLER v1.0"));
    Serial.println(F("===================================="));
    Serial.println(F(""));
    #endif
    
    // Initialize LED
    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);
    
    // Initialize buttons with pull-up
    pinMode(PIN_BUTTON_1, INPUT_PULLUP);
    pinMode(PIN_BUTTON_2, INPUT_PULLUP);
    
    // Initialize switches with pull-up
    pinMode(PIN_SW_ALTHOLD, INPUT_PULLUP);
    pinMode(PIN_SW_ARM, INPUT_PULLUP);
    
    // Initialize analog pins (optional, but explicit)
    pinMode(PIN_THROTTLE, INPUT);
    pinMode(PIN_YAW, INPUT);
    pinMode(PIN_PITCH, INPUT);
    pinMode(PIN_ROLL, INPUT);
    
    // Initialize NRF24L01
    initializeNRF();
    
    // Initialize TX packet
    memset(&txPacket, 0, sizeof(txPacket));
    txPacket.protocol_version = PROTOCOL_VERSION;
    txPacket.throttle = CHANNEL_MIN;
    txPacket.yaw = CHANNEL_CENTER;
    txPacket.pitch = CHANNEL_CENTER;
    txPacket.roll = CHANNEL_CENTER;
    
    // Take initial joystick readings
    for (int i = 0; i < 10; i++) {
        throttleFiltered = analogRead(PIN_THROTTLE);
        yawFiltered = analogRead(PIN_YAW);
        pitchFiltered = analogRead(PIN_PITCH);
        rollFiltered = analogRead(PIN_ROLL);
        delay(10);
    }
    
    #if ENABLE_SERIAL
    Serial.println(F("RC: Initialization complete"));
    Serial.println(F(""));
    #endif
    
    // LED indication
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_LED_STATUS, HIGH);
        delay(100);
        digitalWrite(PIN_LED_STATUS, LOW);
        delay(100);
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    uint32_t now = millis();
    
    // Read all inputs
    readInputs();
    
    // Transmit at specified rate
    if (now - lastTxTime >= TX_INTERVAL_MS) {
        lastTxTime = now;
        
        // Build and transmit packet
        buildPacket();
        transmitPacket();
        
        // Try to receive ACK payload
        receiveAck();
    }
    
    // Update serial output
    #if ENABLE_SERIAL
    if (now - lastSerialTime >= SERIAL_UPDATE_MS) {
        lastSerialTime = now;
        updateSerial();
    }
    #endif
    
    // Update LED status
    updateLED();
}

// ============================================================================
// NRF24L01 INITIALIZATION
// ============================================================================

void initializeNRF() {
    if (!radio.begin()) {
        #if ENABLE_SERIAL
        Serial.println(F("RC: NRF24 initialization FAILED!"));
        #endif
        
        // Error indication
        while (1) {
            digitalWrite(PIN_LED_STATUS, HIGH);
            delay(100);
            digitalWrite(PIN_LED_STATUS, LOW);
            delay(100);
        }
    }
    
    // Configure radio
    radio.setChannel(NRF_CHANNEL);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setRetries(NRF_RETRY_DELAY, NRF_RETRY_COUNT);
    radio.setPayloadSize(NRF_PAYLOAD_SIZE);
    
    // Enable auto-acknowledgment and ACK payloads
    radio.setAutoAck(true);
    radio.enableAckPayload();
    radio.enableDynamicPayloads();
    
    // Open writing pipe
    radio.openWritingPipe(NRF_PIPE_ADDRESS);
    radio.openReadingPipe(1, NRF_PIPE_ADDRESS);
    
    // Set to transmit mode
    radio.stopListening();
    
    #if ENABLE_SERIAL
    Serial.println(F("RC: NRF24 initialized successfully"));
    Serial.print(F("    Channel: ")); Serial.println(NRF_CHANNEL);
    #endif
}

// ============================================================================
// INPUT READING
// ============================================================================

void readInputs() {
    // Read joysticks
    processJoysticks();
    
    // Read buttons
    processButtons();
    
    // Read switches
    processSwitches();
}

void processJoysticks() {
    // Read raw values
    throttleRaw = analogRead(PIN_THROTTLE);
    yawRaw = analogRead(PIN_YAW);
    pitchRaw = analogRead(PIN_PITCH);
    rollRaw = analogRead(PIN_ROLL);
    
    // Apply low-pass filter
    throttleFiltered = throttleFiltered * FILTER_ALPHA + throttleRaw * (1.0f - FILTER_ALPHA);
    yawFiltered = yawFiltered * FILTER_ALPHA + yawRaw * (1.0f - FILTER_ALPHA);
    pitchFiltered = pitchFiltered * FILTER_ALPHA + pitchRaw * (1.0f - FILTER_ALPHA);
    rollFiltered = rollFiltered * FILTER_ALPHA + rollRaw * (1.0f - FILTER_ALPHA);
    
    // Convert to channel values (1000-2000)
    
    // Throttle: 0-1023 → 1000-2000 (no center)
    throttleChannel = map((uint16_t)throttleFiltered, 
                          THROTTLE_MIN_CAL, THROTTLE_MAX_CAL, 
                          THROTTLE_OUTPUT_MIN, THROTTLE_OUTPUT_MAX);
    
    // Yaw: 0-1023 → 1000-2000 (with center and deadband)
    int16_t yawOffset = (int16_t)yawFiltered - YAW_CENTER;
    if (abs(yawOffset) < STICK_DEADBAND) {
        yawChannel = CHANNEL_CENTER;
    } else {
        if (yawOffset > 0) {
            yawChannel = map(yawFiltered, YAW_CENTER + STICK_DEADBAND, YAW_MAX_CAL, 
                            CHANNEL_CENTER, CHANNEL_MAX);
        } else {
            yawChannel = map(yawFiltered, YAW_MIN_CAL, YAW_CENTER - STICK_DEADBAND, 
                            CHANNEL_MIN, CHANNEL_CENTER);
        }
    }
    
    // Pitch: 0-1023 → 1000-2000 (with center and deadband)
    int16_t pitchOffset = (int16_t)pitchFiltered - PITCH_CENTER;
    if (abs(pitchOffset) < STICK_DEADBAND) {
        pitchChannel = CHANNEL_CENTER;
    } else {
        if (pitchOffset > 0) {
            pitchChannel = map(pitchFiltered, PITCH_CENTER + STICK_DEADBAND, PITCH_MAX_CAL, 
                              CHANNEL_CENTER, CHANNEL_MAX);
        } else {
            pitchChannel = map(pitchFiltered, PITCH_MIN_CAL, PITCH_CENTER - STICK_DEADBAND, 
                              CHANNEL_MIN, CHANNEL_CENTER);
        }
    }
    
    // Roll: 0-1023 → 1000-2000 (with center and deadband)
    int16_t rollOffset = (int16_t)rollFiltered - ROLL_CENTER;
    if (abs(rollOffset) < STICK_DEADBAND) {
        rollChannel = CHANNEL_CENTER;
    } else {
        if (rollOffset > 0) {
            rollChannel = map(rollFiltered, ROLL_CENTER + STICK_DEADBAND, ROLL_MAX_CAL, 
                             CHANNEL_CENTER, CHANNEL_MAX);
        } else {
            rollChannel = map(rollFiltered, ROLL_MIN_CAL, ROLL_CENTER - STICK_DEADBAND, 
                             CHANNEL_MIN, CHANNEL_CENTER);
        }
    }
    
    // Constrain all channels
    throttleChannel = constrain(throttleChannel, CHANNEL_MIN, CHANNEL_MAX);
    yawChannel = constrain(yawChannel, CHANNEL_MIN, CHANNEL_MAX);
    pitchChannel = constrain(pitchChannel, CHANNEL_MIN, CHANNEL_MAX);
    rollChannel = constrain(rollChannel, CHANNEL_MIN, CHANNEL_MAX);
    
    #ifdef DEBUG_STICKS
    static uint32_t lastDebug = 0;
    if (millis() - lastDebug >= 200) {
        lastDebug = millis();
        Serial.print(F("Raw: T=")); Serial.print(throttleRaw);
        Serial.print(F(" Y=")); Serial.print(yawRaw);
        Serial.print(F(" P=")); Serial.print(pitchRaw);
        Serial.print(F(" R=")); Serial.println(rollRaw);
    }
    #endif
}

void processButtons() {
    uint32_t now = millis();
    
    // Read button states (active low)
    bool reading1 = !digitalRead(PIN_BUTTON_1);
    bool reading2 = !digitalRead(PIN_BUTTON_2);
    
    // Debounce button 1
    if (reading1 != button1LastState) {
        button1LastDebounce = now;
    }
    if ((now - button1LastDebounce) > DEBOUNCE_MS) {
        if (reading1 != button1State) {
            button1State = reading1;
            if (button1State) {
                // Button pressed - set flag
                button1Pressed = true;
                calibrationRequested = true;
                #if ENABLE_SERIAL
                Serial.println(F("RC: Button 1 pressed - Calibration requested"));
                #endif
            }
        }
    }
    button1LastState = reading1;
    
    // Debounce button 2
    if (reading2 != button2LastState) {
        button2LastDebounce = now;
    }
    if ((now - button2LastDebounce) > DEBOUNCE_MS) {
        if (reading2 != button2State) {
            button2State = reading2;
            if (button2State) {
                // Button pressed - set flag
                button2Pressed = true;
                escCalibrationRequested = true;
                motorTestRequested = true;
                #if ENABLE_SERIAL
                Serial.println(F("RC: Button 2 pressed - ESC Cal/Motor Test requested"));
                #endif
            }
        }
    }
    button2LastState = reading2;
}

void processSwitches() {
    // Read switch states (active low - switch connected to GND)
    swAltHold = !digitalRead(PIN_SW_ALTHOLD);
    swArm = !digitalRead(PIN_SW_ARM);
}

// ============================================================================
// PACKET BUILDING
// ============================================================================

void buildPacket() {
    // Increment packet ID
    txPacket.packet_id = packetId++;
    
    // Set channel values
    txPacket.throttle = throttleChannel;
    txPacket.yaw = yawChannel;
    txPacket.pitch = pitchChannel;
    txPacket.roll = rollChannel;
    
    // Build command flags
    txPacket.command_flags = 0;
    
    if (swArm) {
        txPacket.command_flags |= CMD_FLAG_ARMED;
    }
    
    if (swAltHold) {
        txPacket.command_flags |= CMD_FLAG_ALT_HOLD;
    }
    
    if (calibrationRequested) {
        txPacket.command_flags |= CMD_FLAG_CALIBRATE_IMU;
        calibrationRequested = false;  // Clear after sending once
    }
    
    if (escCalibrationRequested) {
        txPacket.command_flags |= CMD_FLAG_CALIBRATE_ESC;
        escCalibrationRequested = false;
    }
    
    if (motorTestRequested) {
        txPacket.command_flags |= CMD_FLAG_MOTOR_TEST;
        motorTestRequested = false;
    }
    
    // Emergency stop if not armed
    if (!swArm) {
        txPacket.command_flags |= CMD_FLAG_EMERGENCY;
    }
    
    // Calculate checksum
    txPacket.checksum = calculateChecksum((uint8_t*)&txPacket, sizeof(RCCommandPacket) - 1);
    
    #ifdef DEBUG_CHANNELS
    static uint32_t lastDebug = 0;
    if (millis() - lastDebug >= 200) {
        lastDebug = millis();
        Serial.print(F("Ch: T=")); Serial.print(txPacket.throttle);
        Serial.print(F(" Y=")); Serial.print(txPacket.yaw);
        Serial.print(F(" P=")); Serial.print(txPacket.pitch);
        Serial.print(F(" R=")); Serial.print(txPacket.roll);
        Serial.print(F(" Flags=")); Serial.println(txPacket.command_flags, BIN);
    }
    #endif
}

// ============================================================================
// TRANSMISSION
// ============================================================================

void transmitPacket() {
    txCount++;
    
    // Transmit with auto-ack
    bool success = radio.write(&txPacket, sizeof(RCCommandPacket));
    
    if (success) {
        ackCount++;
        lastAckTime = millis();
        linked = true;
    }
    
    // Check for link timeout
    if (millis() - lastAckTime > LINK_TIMEOUT_MS) {
        linked = false;
    }
    
    #ifdef DEBUG_NRF
    if (txCount % 100 == 0) {
        Serial.print(F("TX: ")); Serial.print(txCount);
        Serial.print(F(" ACK: ")); Serial.print(ackCount);
        Serial.print(F(" Rate: ")); Serial.print((ackCount * 100) / txCount);
        Serial.println(F("%"));
    }
    #endif
}

void receiveAck() {
    // Check if ACK payload is available
    if (radio.isAckPayloadAvailable()) {
        radio.read(&rxTelemetry, sizeof(FCTelemetryPacket));
        
        // Validate checksum
        if (validateChecksum((uint8_t*)&rxTelemetry, sizeof(FCTelemetryPacket))) {
            telemetryReceived = true;
        }
    }
}

// ============================================================================
// SERIAL OUTPUT
// ============================================================================

void updateSerial() {
    #if ENABLE_SERIAL
    
    // Print header every 20 updates
    static uint8_t headerCounter = 0;
    if (headerCounter == 0) {
        Serial.println(F(""));
        Serial.println(F("------------------------------------------------------------"));
        Serial.println(F(" LINK | ARM | ALT | THROT |  YAW  | PITCH | ROLL  | CALIB"));
        Serial.println(F("------------------------------------------------------------"));
    }
    headerCounter++;
    if (headerCounter >= 20) headerCounter = 0;
    
    // Link status
    if (linked) {
        Serial.print(F("  OK  "));
    } else {
        Serial.print(F(" LOST "));
    }
    Serial.print(F("| "));
    
    // Arm status
    if (swArm) {
        Serial.print(F("YES "));
    } else {
        Serial.print(F(" NO "));
    }
    Serial.print(F("| "));
    
    // Alt hold status
    if (swAltHold) {
        Serial.print(F("ON  "));
    } else {
        Serial.print(F("OFF "));
    }
    Serial.print(F("|  "));
    
    // Channel values
    Serial.print(throttleChannel);
    Serial.print(F("  |  "));
    
    Serial.print(yawChannel);
    Serial.print(F("  |  "));
    
    Serial.print(pitchChannel);
    Serial.print(F("  |  "));
    
    Serial.print(rollChannel);
    Serial.print(F("  | "));
    
    // Calibration status from telemetry
    if (telemetryReceived) {
        switch (rxTelemetry.calib_status) {
            case CALIB_IDLE:
                Serial.print(F("IDLE"));
                break;
            case CALIB_IN_PROGRESS:
                Serial.print(F("BUSY"));
                break;
            case CALIB_SUCCESS:
                Serial.print(F(" OK "));
                break;
            case CALIB_FAIL_MOTION:
                Serial.print(F("MOVE"));
                break;
            case CALIB_FAIL_RANGE:
                Serial.print(F("RANG"));
                break;
            case CALIB_FAIL_TIMEOUT:
                Serial.print(F("TIME"));
                break;
            case CALIB_ESC_COMPLETE:
                Serial.print(F("ESC "));
                break;
            default:
                Serial.print(F("----"));
                break;
        }
    } else {
        Serial.print(F("----"));
    }
    
    Serial.println();
    
    // Print telemetry details if available
    if (telemetryReceived && linked) {
        static uint8_t telemetryPrintCounter = 0;
        telemetryPrintCounter++;
        
        if (telemetryPrintCounter >= 5) {  // Print every 5th update
            telemetryPrintCounter = 0;
            
            Serial.println(F(""));
            Serial.print(F("  FC Telemetry: Roll="));
            Serial.print(rxTelemetry.roll_angle / 100.0f, 1);
            Serial.print(F("° Pitch="));
            Serial.print(rxTelemetry.pitch_angle / 100.0f, 1);
            Serial.print(F("° Alt="));
            Serial.print(rxTelemetry.altitude);
            Serial.print(F("cm Loop="));
            Serial.print(rxTelemetry.loop_time_us);
            Serial.print(F("us Link="));
            Serial.print(rxTelemetry.link_quality);
            Serial.println(F("%"));
            
            // Motor outputs
            Serial.print(F("  Motors: FL="));
            Serial.print(map(rxTelemetry.motor_fl, 0, 255, 0, 100));
            Serial.print(F("% FR="));
            Serial.print(map(rxTelemetry.motor_fr, 0, 255, 0, 100));
            Serial.print(F("% RR="));
            Serial.print(map(rxTelemetry.motor_rr, 0, 255, 0, 100));
            Serial.print(F("% RL="));
            Serial.print(map(rxTelemetry.motor_rl, 0, 255, 0, 100));
            Serial.println(F("%"));
            
            // Status flags
            Serial.print(F("  Status: "));
            if (rxTelemetry.status_flags & STATUS_FLAG_ARMED) Serial.print(F("ARMED "));
            if (rxTelemetry.status_flags & STATUS_FLAG_ALT_HOLD) Serial.print(F("ALT_HOLD "));
            if (rxTelemetry.status_flags & STATUS_FLAG_CALIBRATED) Serial.print(F("CALIBRATED "));
            if (rxTelemetry.status_flags & STATUS_FLAG_FAILSAFE) Serial.print(F("FAILSAFE "));
            if (rxTelemetry.status_flags & STATUS_FLAG_ERROR) Serial.print(F("ERROR "));
            Serial.println();
        }
    }
    
    #endif
}

// ============================================================================
// LED STATUS
// ============================================================================

void updateLED() {
    static uint32_t lastToggle = 0;
    static bool ledState = false;
    uint32_t now = millis();
    
    if (linked) {
        // Solid ON when linked
        digitalWrite(PIN_LED_STATUS, HIGH);
    } else {
        // Blink when not linked
        if (now - lastToggle >= 200) {
            lastToggle = now;
            ledState = !ledState;
            digitalWrite(PIN_LED_STATUS, ledState);
        }
    }
}
