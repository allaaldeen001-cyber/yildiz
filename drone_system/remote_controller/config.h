/**
 * @file config.h
 * @brief Remote Controller Hardware Configuration
 * @author UAV Embedded Systems Engineer
 * @version 2.0
 * 
 * Pin definitions and hardware configuration for Arduino Nano
 * based quadcopter remote controller.
 */

#ifndef RC_CONFIG_H
#define RC_CONFIG_H

// ============================================================================
// ARDUINO NANO PIN DEFINITIONS
// ============================================================================

// NRF24L01 PA+LNA Module (SPI + Control)
#define NRF_CE_PIN          9       // D9 - Chip Enable
#define NRF_CSN_PIN         10      // D10 - Chip Select (SPI SS)
// MOSI: D11 (hardware SPI)
// MISO: D12 (hardware SPI)
// SCK:  D13 (hardware SPI)

// Joysticks (Analog inputs)
#define JOYSTICK_LEFT_V     A0      // Left vertical (Throttle)
#define JOYSTICK_LEFT_H     A1      // Left horizontal (Yaw)
#define JOYSTICK_RIGHT_V    A2      // Right vertical (Pitch)
#define JOYSTICK_RIGHT_H    A3      // Right horizontal (Roll)

// Push Buttons (Digital inputs with internal pullup)
#define BUTTON_1_PIN        4       // D4 - Calibration button
#define BUTTON_2_PIN        5       // D5 - Motor on / Arm button

// Toggle Switches (Digital inputs with internal pullup)
#define SWITCH_1_PIN        2       // D2 - Altitude hold
#define SWITCH_2_PIN        3       // D3 - Arm/Disarm (Kill switch)

// ============================================================================
// JOYSTICK CONFIGURATION
// ============================================================================

// Raw ADC ranges (10-bit: 0-1023)
#define JOYSTICK_ADC_MIN    0
#define JOYSTICK_ADC_MAX    1023
#define JOYSTICK_ADC_CENTER 512

// Deadband to filter joystick noise
#define JOYSTICK_DEADBAND   20

// Throttle output range (PWM microseconds)
#define THROTTLE_MIN        1000
#define THROTTLE_MAX        2000

// Control axis output ranges
#define YAW_MIN             -500
#define YAW_MAX             500
#define PITCH_MIN           -500
#define PITCH_MAX           500
#define ROLL_MIN            -500
#define ROLL_MAX            500

// Joystick calibration (adjust these per your joysticks)
#define THROTTLE_STICK_MIN  0
#define THROTTLE_STICK_MAX  1023
#define YAW_STICK_MIN       0
#define YAW_STICK_MAX       1023
#define PITCH_STICK_MIN     0
#define PITCH_STICK_MAX     1023
#define ROLL_STICK_MIN      0
#define ROLL_STICK_MAX      1023

// ============================================================================
// INPUT FILTERING
// ============================================================================

// Exponential filter coefficient (0.0 - 1.0)
// Lower = smoother but more lag
#define INPUT_FILTER_ALPHA  0.8

// Number of ADC samples to average
#define ADC_OVERSAMPLE      4

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

// Control loop frequency
#define TX_FREQUENCY_HZ     50      // 50Hz transmission rate
#define TX_PERIOD_MS        20      // 20ms between transmissions

// Serial update rate
#define SERIAL_UPDATE_MS    100     // Update serial every 100ms

// Connection timeout
#define CONNECTION_TIMEOUT  500     // 500ms without ACK = disconnected

// ============================================================================
// SERIAL MONITOR CONFIGURATION
// ============================================================================

#define SERIAL_BAUD_RATE    115200

// Display modes
typedef enum {
    DISPLAY_MINIMAL = 0,    // Just connection status
    DISPLAY_CONTROLS,       // Control inputs
    DISPLAY_TELEMETRY,      // Full telemetry
    DISPLAY_DEBUG           // Debug info
} DisplayMode_t;

// ============================================================================
// STATUS STRINGS
// ============================================================================

const char* const STATUS_STRINGS[] = {
    "BOOT",
    "IDLE",
    "CALIBRATING",
    "CALIBRATED",
    "CAL_FAIL",
    "ESC_CAL",
    "ESC_OK",
    "RESERVED7",
    "RESERVED8",
    "RESERVED9",
    "RESERVEDA",
    "RESERVEDB",
    "RESERVEDC",
    "RESERVEDD",
    "RESERVEDE",
    "RESERVEDF",
    "ARMED",         // 0x10
    "DISARMED",      // 0x11
    "RESERVED12",
    "RESERVED13",
    "RESERVED14",
    "RESERVED15",
    "RESERVED16",
    "RESERVED17",
    "RESERVED18",
    "RESERVED19",
    "RESERVED1A",
    "RESERVED1B",
    "RESERVED1C",
    "RESERVED1D",
    "RESERVED1E",
    "RESERVED1F",
    "FLYING"         // 0x20
};

#endif // RC_CONFIG_H
