/*
 * Configuration File for Professional Drone System
 * 
 * This file contains all configurable parameters for both
 * Flight Controller and Remote Controller boards.
 * 
 * Modify values here instead of changing code directly.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// FLIGHT CONTROLLER CONFIGURATION
// ============================================================================

// --- Pin Definitions ---
#define FC_NRF_CE_PIN       4
#define FC_NRF_CSN_PIN      10
#define FC_MPU_INT_PIN      2
#define FC_BUZZER_PIN       8
#define FC_LED_PIN          7
#define FC_MOTOR_FL_PIN     3
#define FC_MOTOR_FR_PIN     5
#define FC_MOTOR_RR_PIN     6
#define FC_MOTOR_RL_PIN     9

// --- NRF24L01 Settings ---
#define NRF_CHANNEL         103       // 0-125, must match RC
#define NRF_DATA_RATE       RF24_250KBPS  // RF24_250KBPS, RF24_1MBPS, RF24_2MBPS
#define NRF_PA_LEVEL        RF24_PA_MAX   // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
#define NRF_AUTO_ACK        true
#define NRF_RETRIES_DELAY   5         // 0-15 (delay = 250µs × (1 + value))
#define NRF_RETRIES_COUNT   15        // 0-15

// --- Loop Timing ---
#define LOOP_FREQUENCY      250       // Hz (4000µs per loop)
#define LOOP_TIME_US        4000      // Microseconds

// --- Motor Limits ---
#define MOTOR_MIN           1000      // Minimum PWM (µs)
#define MOTOR_MAX           2000      // Maximum PWM (µs)
#define MOTOR_ARM_LEVEL     1050      // Minimum armed throttle
#define THROTTLE_SAFE_MAX   1650      // 65% throttle cap for safety

// --- Angle Limits ---
#define MAX_ANGLE_DEGREES   30.0      // Maximum tilt angle (degrees)

// --- PID Gains - Roll ---
#define KP_ROLL             1.5       // Proportional gain
#define KI_ROLL             0.05      // Integral gain
#define KD_ROLL             15.0      // Derivative gain

// --- PID Gains - Pitch ---
#define KP_PITCH            1.5
#define KI_PITCH            0.05
#define KD_PITCH            15.0

// --- PID Gains - Yaw ---
#define KP_YAW              3.0
#define KI_YAW              0.02
#define KD_YAW              0.0       // Usually zero for yaw

// --- PID Gains - Altitude Hold ---
#define KP_ALT              2.0
#define KI_ALT              0.1
#define KD_ALT              1.5

// --- PID Anti-Windup Limits ---
#define MAX_INTEGRAL_ROLL   400.0     // Maximum integral accumulation
#define MAX_INTEGRAL_PITCH  400.0
#define MAX_INTEGRAL_YAW    400.0
#define MAX_INTEGRAL_ALT    400.0

// --- Sensor Settings ---
#define MPU6050_I2C_ADDR    0x68      // 0x68 or 0x69
#define MPU6050_GYRO_SCALE  65.5      // LSB per °/s for ±500°/s range
#define MPU6050_ACCEL_SCALE 4096.0    // LSB per g for ±8g range

#define MS5611_I2C_ADDR     0x77      // Usually 0x77

// --- Complementary Filter ---
#define GYRO_WEIGHT         0.98      // Gyro contribution (0.96-0.99)
#define ACCEL_WEIGHT        0.02      // Accel contribution (1 - GYRO_WEIGHT)

// --- Calibration ---
#define CALIB_SAMPLES       2000      // Number of samples for gyro calibration
#define CALIB_MAX_OFFSET    10.0      // Maximum acceptable gyro offset (°/s)

// --- Failsafe ---
#define SIGNAL_TIMEOUT_MS   1000      // Disarm after signal loss (ms)

// --- Telemetry ---
#define TELEMETRY_RATE_HZ   20        // Telemetry transmission rate

// --- Battery Monitoring (if implemented) ---
#define BATTERY_ADC_PIN     A6        // Analog pin for voltage divider
#define BATTERY_R1          10000     // Voltage divider R1 (Ω)
#define BATTERY_R2          2200      // Voltage divider R2 (Ω)
#define BATTERY_MIN_VOLTAGE 10.5      // Warning voltage (3S = 10.5V)

// ============================================================================
// REMOTE CONTROLLER CONFIGURATION
// ============================================================================

// --- Pin Definitions ---
#define RC_NRF_CE_PIN       9
#define RC_NRF_CSN_PIN      10
#define RC_THROTTLE_PIN     A0
#define RC_YAW_PIN          A1
#define RC_PITCH_PIN        A2
#define RC_ROLL_PIN         A3
#define RC_SW_ALTHOLD_PIN   2
#define RC_SW_ARM_PIN       3
#define RC_BTN_CALIB_PIN    4
#define RC_BTN_ESC_PIN      5

// --- Joystick Calibration ---
#define STICK_MIN           0         // Minimum ADC value
#define STICK_MAX           1023      // Maximum ADC value
#define STICK_CENTER        512       // Center position
#define STICK_DEADZONE      30        // ±deadzone around center

// --- Output Ranges ---
#define THROTTLE_OUT_MIN    1000      // PWM output minimum
#define THROTTLE_OUT_MAX    2000      // PWM output maximum
#define CONTROL_RANGE       500       // ±500 for pitch/roll/yaw

// --- Update Rates ---
#define RC_UPDATE_RATE_HZ   50        // TX rate (20ms interval)
#define RC_DISPLAY_RATE_HZ  5         // Serial display update (200ms)

// --- Serial Communication ---
#define RC_SERIAL_BAUD      115200    // Serial monitor baud rate

// --- Button Debounce ---
#define BUTTON_DEBOUNCE_MS  500       // Minimum time between button presses

// --- Connection Timeout ---
#define RC_TIMEOUT_MS       1000      // Connection lost after (ms)

// ============================================================================
// SHARED CONFIGURATION
// ============================================================================

// --- NRF24L01 Addresses ---
#define NRF_ADDR_DRONE      "DRONE"   // FC receiving address
#define NRF_ADDR_REMOTE     "REMOT"   // RC receiving address

// --- Data Packet Sizes ---
#define RC_DATA_SIZE        sizeof(RCData)
#define TELEMETRY_SIZE      sizeof(TelemetryData)

// ============================================================================
// ADVANCED SETTINGS (Don't change unless you know what you're doing!)
// ============================================================================

// --- MPU6050 Digital Low-Pass Filter ---
// 0 = 260Hz, 1 = 184Hz, 2 = 98Hz, 3 = 42Hz, 4 = 20Hz, 5 = 10Hz, 6 = 5Hz
#define MPU6050_DLPF_CFG    2         // 98Hz recommended

// --- MPU6050 Gyro Range ---
// 0 = ±250°/s, 1 = ±500°/s, 2 = ±1000°/s, 3 = ±2000°/s
#define MPU6050_GYRO_RANGE  1         // ±500°/s recommended

// --- MPU6050 Accel Range ---
// 0 = ±2g, 1 = ±4g, 2 = ±8g, 3 = ±16g
#define MPU6050_ACCEL_RANGE 2         // ±8g recommended

// --- I2C Clock Speed ---
#define I2C_CLOCK_SPEED     400000    // 400kHz Fast Mode

// ============================================================================
// FEATURE FLAGS
// ============================================================================

// Enable/disable features (comment out to disable)
#define ENABLE_ALTITUDE_HOLD          // Barometer-based altitude hold
#define ENABLE_TELEMETRY              // Bidirectional telemetry
#define ENABLE_SERIAL_DEBUG           // Serial debugging (FC)
#define ENABLE_RC_DISPLAY             // Serial display (RC)
// #define ENABLE_BATTERY_MONITOR     // Battery voltage monitoring (not implemented yet)
// #define ENABLE_BLACKBOX            // Flight data logging (not implemented yet)
// #define ENABLE_GPS                 // GPS features (not implemented yet)

// ============================================================================
// SAFETY FEATURES (DO NOT DISABLE!)
// ============================================================================

#define SAFETY_ANGLE_LIMIT            // Prevent angles > MAX_ANGLE_DEGREES
#define SAFETY_THROTTLE_CAP           // Cap throttle at THROTTLE_SAFE_MAX
#define SAFETY_ARM_CHECK              // Require low throttle to arm
#define SAFETY_SIGNAL_LOSS_FAILSAFE   // Auto-disarm on signal loss
#define SAFETY_CALIBRATION_REQUIRED   // Must calibrate before arming

// ============================================================================
// DEBUG OPTIONS
// ============================================================================

// Uncomment to enable specific debug outputs (reduces performance!)
// #define DEBUG_PID_VALUES           // Print PID calculations
// #define DEBUG_SENSOR_RAW           // Print raw sensor data
// #define DEBUG_MOTOR_OUTPUTS        // Print motor speeds
// #define DEBUG_NRF_COMMUNICATION    // Print NRF TX/RX status
// #define DEBUG_LOOP_TIMING          // Print loop execution time

// ============================================================================
// VERSION INFO
// ============================================================================

#define FIRMWARE_VERSION    "1.0.0"
#define BUILD_DATE          __DATE__
#define BUILD_TIME          __TIME__

#endif // CONFIG_H
