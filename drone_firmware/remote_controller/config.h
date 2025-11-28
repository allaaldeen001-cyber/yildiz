/**
 * ============================================================================
 * REMOTE CONTROLLER CONFIGURATION
 * ============================================================================
 * 
 * Central configuration file for all RC parameters
 * 
 * ============================================================================
 */

#ifndef RC_CONFIG_H
#define RC_CONFIG_H

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

// NRF24L01 PA+LNA Module
#define PIN_NRF_CE          9     // D9
#define PIN_NRF_CSN         10    // D10 (Hardware SPI)
// MOSI = D11, MISO = D12, SCK = D13 (Hardware SPI)

// Joysticks (Analog inputs)
#define PIN_THROTTLE        A0    // Left stick vertical
#define PIN_YAW             A1    // Left stick horizontal
#define PIN_PITCH           A2    // Right stick vertical
#define PIN_ROLL            A3    // Right stick horizontal

// Buttons (Digital inputs with pull-up)
#define PIN_BUTTON_1        4     // D4 - Calibration button
#define PIN_BUTTON_2        5     // D5 - Motor ON / ESC calibration

// Toggle Switches (Digital inputs)
#define PIN_SW_ALTHOLD      2     // D2 - Altitude Hold ON/OFF
#define PIN_SW_ARM          3     // D3 - ARM / DISARM (Kill Switch)

// Status LED (optional - using built-in LED)
#define PIN_LED_STATUS      LED_BUILTIN  // D13

// ============================================================================
// JOYSTICK CALIBRATION
// ============================================================================

// Raw ADC range (10-bit)
#define ADC_MIN             0
#define ADC_MAX             1023

// Center positions (calibrate for your joysticks)
#define THROTTLE_CENTER     512
#define YAW_CENTER          512
#define PITCH_CENTER        512
#define ROLL_CENTER         512

// Deadband around center (raw ADC units)
#define STICK_DEADBAND      20

// Joystick calibration values (adjust for your hardware)
#define THROTTLE_MIN_CAL    0
#define THROTTLE_MAX_CAL    1023
#define YAW_MIN_CAL         0
#define YAW_MAX_CAL         1023
#define PITCH_MIN_CAL       0
#define PITCH_MAX_CAL       1023
#define ROLL_MIN_CAL        0
#define ROLL_MAX_CAL        1023

// ============================================================================
// OUTPUT RANGES
// ============================================================================

// Standard RC channel range
#define CHANNEL_MIN         1000
#define CHANNEL_MAX         2000
#define CHANNEL_CENTER      1500

// Throttle specific
#define THROTTLE_OUTPUT_MIN 1000
#define THROTTLE_OUTPUT_MAX 2000

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

// Transmission rate
#define TX_INTERVAL_MS      10    // 100 Hz update rate

// Button debounce
#define DEBOUNCE_MS         50    // 50ms debounce time

// Link timeout for status display
#define LINK_TIMEOUT_MS     500

// Serial update rate
#define SERIAL_UPDATE_MS    200   // 5 Hz serial output

// ============================================================================
// NRF24L01 CONFIGURATION
// ============================================================================

#define NRF_CHANNEL         103   // Must match FC
#define NRF_DATA_RATE       RF24_250KBPS
#define NRF_PA_LEVEL        RF24_PA_MAX
#define NRF_RETRY_DELAY     5     // 1500µs
#define NRF_RETRY_COUNT     15

// ============================================================================
// DEBUG OPTIONS
// ============================================================================

#define ENABLE_SERIAL       1     // Enable serial output
#define SERIAL_BAUD         115200

// Uncomment to enable specific debug output
// #define DEBUG_STICKS        // Print raw stick values
// #define DEBUG_CHANNELS      // Print channel values
// #define DEBUG_NRF           // Print NRF status

#endif // RC_CONFIG_H
