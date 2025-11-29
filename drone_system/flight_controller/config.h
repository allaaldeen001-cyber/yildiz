/**
 * @file config.h
 * @brief Flight Controller Hardware Configuration
 * @author UAV Embedded Systems Engineer
 * @version 2.0
 * 
 * Pin definitions and hardware configuration for Arduino Nano
 * based quadcopter flight controller.
 */

#ifndef FC_CONFIG_H
#define FC_CONFIG_H

// ============================================================================
// ARDUINO NANO PIN DEFINITIONS
// ============================================================================

// NRF24L01 PA+LNA Module (SPI + Control)
#define NRF_CE_PIN          4       // D4 - Chip Enable
#define NRF_CSN_PIN         10      // D10 - Chip Select (SPI SS)
// MOSI: D11 (hardware SPI)
// MISO: D12 (hardware SPI)
// SCK:  D13 (hardware SPI)

// MPU6050 IMU (I2C + Interrupt)
#define MPU_INT_PIN         2       // D2 - Interrupt pin (INT0)
// SDA: A4 (hardware I2C)
// SCL: A5 (hardware I2C)

// Status Indicators
#define BUZZER_PIN          8       // D8 - Piezo buzzer
#define LED_STATUS_PIN      7       // D7 - Status LED

// ESC Motor Outputs (PWM capable pins)
#define MOTOR_FL_PIN        3       // D3 - Front Left (PWM)
#define MOTOR_FR_PIN        5       // D5 - Front Right (PWM)
#define MOTOR_RR_PIN        6       // D6 - Rear Right (PWM)
#define MOTOR_RL_PIN        9       // D9 - Rear Left (PWM)

// ============================================================================
// MPU6050 CONFIGURATION
// ============================================================================

#define MPU6050_ADDRESS     0x68    // I2C address (AD0 = GND)
#define MPU_GYRO_CONFIG     0x08    // ±500°/s (for stability)
#define MPU_ACCEL_CONFIG    0x00    // ±2g (high precision)
#define MPU_DLPF_CONFIG     0x03    // 44Hz bandwidth (smooth)
#define MPU_SAMPLE_RATE     4       // 200Hz sample rate

// Calibration samples
#define GYRO_CALIBRATION_SAMPLES    2000
#define ACCEL_CALIBRATION_SAMPLES   500

// ============================================================================
// PID CONTROLLER CONFIGURATION
// ============================================================================

// Roll PID gains (tuned for stability)
#define PID_ROLL_KP         1.3
#define PID_ROLL_KI         0.04
#define PID_ROLL_KD         15.0

// Pitch PID gains
#define PID_PITCH_KP        1.3
#define PID_PITCH_KI        0.04
#define PID_PITCH_KD        15.0

// Yaw PID gains (rate-based)
#define PID_YAW_KP          4.0
#define PID_YAW_KI          0.02
#define PID_YAW_KD          0.0

// PID output limits
#define PID_MAX_OUTPUT      400
#define PID_I_MAX           100     // Anti-windup

// ============================================================================
// MOTOR/ESC CONFIGURATION
// ============================================================================

// ESC timing (standard PWM protocol)
#define ESC_MIN_PULSE       1000    // Minimum pulse width (µs)
#define ESC_MAX_PULSE       2000    // Maximum pulse width (µs)
#define ESC_ARM_PULSE       1000    // Arming pulse width
#define ESC_CALIBRATE_HIGH  2000    // ESC calibration high
#define ESC_CALIBRATE_LOW   1000    // ESC calibration low

// Motor mixing configuration (X-quad configuration)
// Viewed from above:
//     FL(CW) ----- FR(CCW)
//         \       /
//          \     /
//           \   /
//          /     \
//         /       \
//     RL(CCW) ---- RR(CW)

// Motor direction: 1 = CW, -1 = CCW
#define MOTOR_FL_DIR        1       // Clockwise
#define MOTOR_FR_DIR        -1      // Counter-clockwise
#define MOTOR_RR_DIR        1       // Clockwise
#define MOTOR_RL_DIR        -1      // Counter-clockwise

// ============================================================================
// SAFETY CONFIGURATION
// ============================================================================

#define MAX_TILT_ANGLE      30.0    // Maximum tilt angle (degrees)
#define MAX_THROTTLE_PCT    65      // Maximum throttle percentage
#define FAILSAFE_TIMEOUT    500     // Failsafe timeout (ms)
#define ARM_DELAY           2000    // Delay before arming (ms)
#define DISARM_TIMEOUT      3000    // Auto-disarm if no throttle (ms)

// ============================================================================
// LOOP TIMING
// ============================================================================

#define LOOP_FREQUENCY      250     // Main loop frequency (Hz)
#define LOOP_PERIOD_US      4000    // Loop period in microseconds
#define TELEMETRY_DIVIDER   5       // Send telemetry every N loops

// ============================================================================
// BUZZER PATTERNS
// ============================================================================

// Calibration success: Two short beeps
#define BEEP_CALIBRATION_OK_COUNT   2
#define BEEP_CALIBRATION_OK_FREQ    2000
#define BEEP_CALIBRATION_OK_DUR     100

// Calibration fail: One long beep (7 seconds)
#define BEEP_CALIBRATION_FAIL_FREQ  1000
#define BEEP_CALIBRATION_FAIL_DUR   7000

// ESC calibration beep
#define BEEP_ESC_FREQ               1500
#define BEEP_ESC_DUR                200

// Arm beep
#define BEEP_ARM_FREQ               2500
#define BEEP_ARM_DUR                150

// Low battery warning
#define BEEP_LOWBATT_FREQ           800
#define BEEP_LOWBATT_DUR            500

// ============================================================================
// LED PATTERNS
// ============================================================================

typedef enum {
    LED_OFF = 0,
    LED_ON,
    LED_BLINK_SLOW,     // 0.5Hz - Waiting for connection
    LED_BLINK_FAST,     // 2Hz - Calibrating
    LED_BLINK_DOUBLE,   // Double blink - Armed
    LED_SOLID           // Solid on - Flying
} LEDPattern_t;

#endif // FC_CONFIG_H
