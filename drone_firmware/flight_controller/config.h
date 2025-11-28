/**
 * ============================================================================
 * FLIGHT CONTROLLER CONFIGURATION
 * ============================================================================
 * 
 * Central configuration file for all FC parameters
 * Modify these values to tune your drone
 * 
 * ============================================================================
 */

#ifndef FC_CONFIG_H
#define FC_CONFIG_H

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

// NRF24L01 PA+LNA Module
#define PIN_NRF_CE          4     // D4
#define PIN_NRF_CSN         10    // D10 (Hardware SPI)
// MOSI = D11, MISO = D12, SCK = D13 (Hardware SPI)

// MPU6050 IMU
#define PIN_MPU_INT         2     // D2 - Interrupt pin

// MS5611 Barometer - I2C (A4=SDA, A5=SCL)
// No additional pins needed

// ESC Motor Outputs (PWM capable pins)
#define PIN_MOTOR_FL        3     // D3 - Front Left (Timer2)
#define PIN_MOTOR_FR        5     // D5 - Front Right (Timer0)
#define PIN_MOTOR_RR        6     // D6 - Rear Right (Timer0)
#define PIN_MOTOR_RL        9     // D9 - Rear Left (Timer1)

// Status Indicators
#define PIN_BUZZER          8     // D8
#define PIN_LED_STATUS      7     // D7

// ============================================================================
// I2C ADDRESSES
// ============================================================================

#define MPU6050_ADDRESS     0x68  // AD0 pin low
#define MS5611_ADDRESS      0x77  // CSB pin high

// ============================================================================
// TIMING CONFIGURATION (in microseconds)
// ============================================================================

// Main control loop timing
#define LOOP_RATE_HZ        400               // Target: 400 Hz main loop
#define LOOP_PERIOD_US      (1000000UL / LOOP_RATE_HZ)  // 2500 µs

// Sub-loop dividers
#define RATE_PID_DIVIDER    1     // Rate PID runs every loop (400 Hz)
#define ANGLE_PID_DIVIDER   4     // Angle PID runs at 100 Hz
#define ALTITUDE_DIVIDER    8     // Altitude runs at 50 Hz
#define BARO_DIVIDER        20    // Barometer reads at 20 Hz
#define TELEMETRY_DIVIDER   40    // Telemetry at 10 Hz

// Safety timeouts
#define FAILSAFE_TIMEOUT_MS     500   // Link loss timeout
#define ARM_TIMEOUT_MS          5000  // Auto-disarm if no throttle
#define CALIBRATION_TIMEOUT_MS  10000 // Max calibration time

// ============================================================================
// FLIGHT LIMITS
// ============================================================================

// Throttle limits
#define THROTTLE_MIN        1000
#define THROTTLE_IDLE       1050  // Minimum armed throttle
#define THROTTLE_MAX        2000
#define THROTTLE_LIMIT_PCT  65    // Max throttle percentage (safety limit)
#define THROTTLE_LIMIT      (THROTTLE_MIN + ((THROTTLE_MAX - THROTTLE_MIN) * THROTTLE_LIMIT_PCT / 100))

// Angle limits (degrees)
#define MAX_ROLL_ANGLE      30.0f
#define MAX_PITCH_ANGLE     30.0f
#define MAX_YAW_RATE        180.0f  // degrees/second

// Rate limits (degrees/second)
#define MAX_ROLL_RATE       300.0f
#define MAX_PITCH_RATE      300.0f

// ============================================================================
// PID CONFIGURATION - RATE CONTROLLER (Inner Loop)
// ============================================================================

// Roll Rate PID
#define PID_ROLL_RATE_KP    0.7f
#define PID_ROLL_RATE_KI    0.3f
#define PID_ROLL_RATE_KD    0.03f

// Pitch Rate PID
#define PID_PITCH_RATE_KP   0.7f
#define PID_PITCH_RATE_KI   0.3f
#define PID_PITCH_RATE_KD   0.03f

// Yaw Rate PID
#define PID_YAW_RATE_KP     2.0f
#define PID_YAW_RATE_KI     0.5f
#define PID_YAW_RATE_KD     0.0f

// Rate PID limits
#define RATE_PID_IMAX       200.0f  // Integral windup limit
#define RATE_PID_OUTMAX     400.0f  // Output limit

// ============================================================================
// PID CONFIGURATION - ANGLE CONTROLLER (Outer Loop)
// ============================================================================

// Roll Angle PID
#define PID_ROLL_ANGLE_KP   4.5f
#define PID_ROLL_ANGLE_KI   0.02f
#define PID_ROLL_ANGLE_KD   0.0f

// Pitch Angle PID
#define PID_PITCH_ANGLE_KP  4.5f
#define PID_PITCH_ANGLE_KI  0.02f
#define PID_PITCH_ANGLE_KD  0.0f

// Angle PID limits
#define ANGLE_PID_IMAX      50.0f
#define ANGLE_PID_OUTMAX    MAX_ROLL_RATE

// ============================================================================
// PID CONFIGURATION - ALTITUDE CONTROLLER
// ============================================================================

// Altitude PID (outer)
#define PID_ALT_KP          0.5f
#define PID_ALT_KI          0.02f
#define PID_ALT_KD          0.1f
#define PID_ALT_IMAX        100.0f

// Vertical velocity PID (inner)
#define PID_VVEL_KP         0.15f
#define PID_VVEL_KI         0.02f
#define PID_VVEL_KD         0.01f
#define PID_VVEL_IMAX       50.0f

// Altitude limits
#define MAX_CLIMB_RATE      200.0f  // cm/s
#define MAX_DESCENT_RATE    150.0f  // cm/s
#define ALT_THROTTLE_DEADBAND 50    // Stick deadband for alt hold

// ============================================================================
// SENSOR CALIBRATION DEFAULTS
// ============================================================================

// MPU6050 offsets (update after calibration)
#define GYRO_OFFSET_X       0
#define GYRO_OFFSET_Y       0
#define GYRO_OFFSET_Z       0
#define ACCEL_OFFSET_X      0
#define ACCEL_OFFSET_Y      0
#define ACCEL_OFFSET_Z      0

// Calibration samples
#define CALIB_SAMPLES       500     // Number of samples for averaging
#define CALIB_MOTION_THRESH 50      // Max gyro variance during calibration

// ============================================================================
// FILTER CONFIGURATION
// ============================================================================

// Mahony AHRS filter gains
#define MAHONY_KP           10.0f   // Proportional gain
#define MAHONY_KI           0.005f  // Integral gain

// Complementary filter for altitude
#define COMP_FILTER_ALPHA   0.98f   // Barometer weight (vs accelerometer)

// Low-pass filter coefficients
#define GYRO_LPF_ALPHA      0.8f    // Gyro smoothing (0-1, higher = more smoothing)
#define ACCEL_LPF_ALPHA     0.9f    // Accelerometer smoothing
#define BARO_LPF_ALPHA      0.7f    // Barometer smoothing

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================

// Motor output range (PWM values)
#define MOTOR_MIN           1000    // Minimum PWM (motors off)
#define MOTOR_IDLE          1050    // Idle spin when armed
#define MOTOR_MAX           2000    // Maximum PWM

// Motor mixing (X-configuration)
// Front-Left:  +Roll, +Pitch, -Yaw (CCW)
// Front-Right: -Roll, +Pitch, +Yaw (CW)
// Rear-Right:  -Roll, -Pitch, -Yaw (CCW)
// Rear-Left:   +Roll, -Pitch, +Yaw (CW)

// ESC protocol
#define ESC_PWM_FREQ        490     // Standard ESC PWM frequency
#define ESC_UPDATE_US       2000    // ESC update period

// ============================================================================
// NRF24L01 CONFIGURATION
// ============================================================================

#define NRF_CHANNEL         103     // RF channel (0-125)
#define NRF_DATA_RATE       RF24_250KBPS  // Low data rate for range
#define NRF_PA_LEVEL        RF24_PA_MAX   // Maximum power
#define NRF_RETRY_DELAY     5       // 1500µs retry delay
#define NRF_RETRY_COUNT     15      // Max retries

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================

// Uncomment to enable debug features
// #define DEBUG_SERIAL        // Enable serial debug output
// #define DEBUG_TIMING        // Print loop timing info
// #define DEBUG_SENSORS       // Print raw sensor data
// #define DEBUG_PID           // Print PID values
// #define DEBUG_MOTORS        // Print motor outputs

#ifdef DEBUG_SERIAL
    #define SERIAL_BAUD     115200
#endif

// ============================================================================
// SAFETY FEATURES
// ============================================================================

#define ENABLE_FAILSAFE     1       // Enable link-loss failsafe
#define ENABLE_TILT_SAFETY  1       // Kill motors on excessive tilt
#define MAX_SAFE_TILT       60.0f   // Maximum safe tilt angle (degrees)
#define ENABLE_LOWBATT      1       // Enable low battery detection
#define LOW_BATT_THRESHOLD  10500   // Low battery voltage (mV) - 3.5V per cell

#endif // FC_CONFIG_H
