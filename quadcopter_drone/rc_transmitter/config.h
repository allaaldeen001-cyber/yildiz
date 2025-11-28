/*
 * ============================================================================
 * RC TRANSMITTER - CONFIGURATION
 * ============================================================================
 * Pin definitions and settings for the RC transmitter
 * Hardware: Arduino Nano (ATmega328P)
 * ============================================================================
 */

#ifndef RC_CONFIG_H
#define RC_CONFIG_H

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

// --- Joystick Analog Pins ---
#define LEFT_JOY_Y_PIN      A0  // Throttle
#define LEFT_JOY_X_PIN      A1  // Yaw
#define RIGHT_JOY_Y_PIN     A2  // Pitch
#define RIGHT_JOY_X_PIN     A3  // Roll

// --- Button and Switch Pins ---
#define BTN1_PIN            3   // Calibration button (with internal pullup)
#define BTN2_PIN            4   // Motor test button (with internal pullup)
#define ARM_SWITCH_PIN      5   // Arm/Disarm toggle switch (with internal pullup)

// --- Status LED ---
#define STATUS_LED_PIN      2   // Status indicator LED

// --- NRF24L01 Pins ---
#define NRF_CE_PIN          9   // Chip Enable
#define NRF_CSN_PIN         10  // Chip Select Not
// MOSI = D11 (hardware SPI)
// MISO = D12 (hardware SPI)
// SCK  = D13 (hardware SPI)

// ============================================================================
// NRF24L01 CONFIGURATION
// ============================================================================
#define NRF_CHANNEL         76              // Must match flight controller!
#define NRF_PA_LEVEL        RF24_PA_HIGH    // Power level
#define NRF_DATA_RATE       RF24_250KBPS    // Data rate (lower = better range)
#define NRF_PIPE_ADDRESS    0xE8E8F0F0E1LL  // Must match flight controller!

// ============================================================================
// JOYSTICK CONFIGURATION
// ============================================================================

// Raw ADC range (0-1023 for Arduino 10-bit ADC)
#define ADC_MIN             0
#define ADC_MAX             1023
#define ADC_CENTER          512

// Output range (standard RC PWM values)
#define OUTPUT_MIN          1000
#define OUTPUT_MAX          2000
#define OUTPUT_CENTER       1500

// Deadzone around center (for pitch/roll/yaw)
#define JOYSTICK_DEADZONE   20

// Throttle specific - spring vs no-spring
// If your throttle stick has a centering spring, set this to true
#define THROTTLE_HAS_SPRING false

// ============================================================================
// TIMING
// ============================================================================
#define TX_RATE_MS          10      // Transmission rate (100Hz)
#define TELEMETRY_TIMEOUT   100     // Telemetry receive timeout (ms)
#define SERIAL_BAUD         115200  // Serial debug baud rate
#define LED_BLINK_RATE      500     // LED blink rate when connected

// ============================================================================
// EEPROM ADDRESSES (for joystick calibration)
// ============================================================================
#define EEPROM_JOY_SIGNATURE    100     // Signature byte
#define EEPROM_JOY_DATA         101     // Calibration data start

// ============================================================================
// DATA STRUCTURES (must match flight controller)
// ============================================================================

// RC Data packet to transmitter
struct RCData {
    uint16_t throttle;      // 1000-2000
    uint16_t yaw;           // 1000-2000
    uint16_t pitch;         // 1000-2000
    uint16_t roll;          // 1000-2000
    uint8_t  armSwitch;     // 0 = disarmed, 1 = armed
    uint8_t  button1;       // Calibration button
    uint8_t  button2;       // Motor test button
    uint8_t  checksum;      // Data integrity check
};

// Telemetry data from flight controller
struct TelemetryData {
    int16_t  roll;          // Current roll angle * 10
    int16_t  pitch;         // Current pitch angle * 10
    int16_t  yaw;           // Current yaw angle * 10
    uint16_t altitude;      // Altitude in cm
    uint16_t batteryVoltage;// Battery voltage * 100
    uint8_t  armed;         // Armed status
    uint8_t  flightMode;    // Current flight mode
    int8_t   rssi;          // Signal strength
    uint8_t  status;        // System status flags
};

// Joystick calibration data
struct JoystickCalData {
    uint16_t throttleMin, throttleMax, throttleCenter;
    uint16_t yawMin, yawMax, yawCenter;
    uint16_t pitchMin, pitchMax, pitchCenter;
    uint16_t rollMin, rollMax, rollCenter;
    uint8_t  signature;
};

#endif // RC_CONFIG_H
