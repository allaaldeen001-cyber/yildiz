/*
 * TRANSMITTER CONFIGURATION FILE
 * ==============================
 * Edit these values to customize your controller
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================
// RADIO CONFIGURATION
// ============================================
#define RADIO_CHANNEL 108           // 0-125 (2.4-2.525 GHz) - MUST MATCH RECEIVER!
#define RADIO_PA_LEVEL RF24_PA_MAX  // RF24_PA_MIN, LOW, HIGH, MAX
#define RADIO_DATARATE RF24_250KBPS // RF24_250KBPS, 1MBPS, 2MBPS
#define RADIO_ADDRESS "DRON1"       // Must match receiver (5 chars)

// Retry settings
#define RADIO_RETRY_DELAY 3         // x 250us (3 = 750us)
#define RADIO_RETRY_COUNT 5         // Number of retries

// ============================================
// JOYSTICK CONFIGURATION
// ============================================
#define THROTTLE_PIN A0             // Left stick vertical
#define YAW_PIN A1                  // Left stick horizontal
#define PITCH_PIN A2                // Right stick vertical
#define ROLL_PIN A3                 // Right stick horizontal

// Joystick calibration (if needed)
#define STICK_MIN 0                 // Minimum analog reading
#define STICK_MAX 1023              // Maximum analog reading
#define STICK_CENTER 512            // Center reading

// Deadband (prevents jitter at center)
#define STICK_DEADBAND 10           // ±10 counts at center

// ============================================
// SWITCH CONFIGURATION
// ============================================
#define ARM_SWITCH_PIN 7            // Arm/disarm switch
#define MODE_SWITCH_PIN 8           // Flight mode switch
#define AUX1_SWITCH_PIN -1          // Additional switch (set to -1 to disable)
#define AUX2_SWITCH_PIN -1          // Additional switch (set to -1 to disable)

// Switch polarity (true = inverted pullup, false = normal)
#define SWITCHES_INVERTED true

// ============================================
// CONTROL MAPPING
// ============================================
// Output range (standard PWM microseconds)
#define CONTROL_MIN 1000
#define CONTROL_CENTER 1500
#define CONTROL_MAX 2000

// ============================================
// TIMING CONFIGURATION
// ============================================
#define TX_UPDATE_RATE 50           // Hz (control packet rate)
#define TX_INTERVAL 20              // ms (1000/50 = 20ms)

// ============================================
// DISPLAY & FEEDBACK
// ============================================
#define STATUS_LED_PIN 13           // Status LED
#define BUZZER_PIN -1               // Buzzer pin (set to -1 to disable)

#define TELEMETRY_PRINT_INTERVAL 500  // ms between telemetry prints
#define DEBUG_PRINT_INTERVAL 1000     // ms between debug prints

// ============================================
// SAFETY & WARNINGS
// ============================================
#define CONNECTION_TIMEOUT 1000     // ms - warn if no response
#define LOW_BATTERY_WARNING 11.1    // V - warn if battery below this
#define PACKET_LOSS_WARNING 5.0     // % - warn if loss above this

// ============================================
// DEBUG OPTIONS
// ============================================
#define ENABLE_SERIAL_DEBUG true    // Enable/disable serial debug
#define SERIAL_BAUD_RATE 115200     // Serial baud rate

#endif // CONFIG_H
