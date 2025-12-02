/*
 * DRONE CONFIGURATION FILE
 * ========================
 * Edit these values to customize your drone
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================
// RADIO CONFIGURATION
// ============================================
#define RADIO_CHANNEL 108           // 0-125 (2.4-2.525 GHz)
#define RADIO_PA_LEVEL RF24_PA_MAX  // RF24_PA_MIN, LOW, HIGH, MAX
#define RADIO_DATARATE RF24_250KBPS // RF24_250KBPS, 1MBPS, 2MBPS
#define RADIO_ADDRESS "DRON1"       // Must match transmitter (5 chars)

// ============================================
// PID TUNING - ROLL AXIS
// ============================================
#define PID_ROLL_P 1.3              // Proportional gain
#define PID_ROLL_I 0.04             // Integral gain
#define PID_ROLL_D 18.0             // Derivative gain

// ============================================
// PID TUNING - PITCH AXIS
// ============================================
#define PID_PITCH_P 1.3             // Proportional gain
#define PID_PITCH_I 0.04            // Integral gain
#define PID_PITCH_D 18.0            // Derivative gain

// ============================================
// PID TUNING - YAW AXIS
// ============================================
#define PID_YAW_P 2.0               // Proportional gain
#define PID_YAW_I 0.02              // Integral gain
#define PID_YAW_D 0.0               // Derivative gain (usually 0)

// ============================================
// PID LIMITS
// ============================================
#define PID_MAX_OUTPUT 400          // Maximum PID output
#define PID_I_MAX_ACCUMULATION 200  // Anti-windup limit

// ============================================
// FLIGHT PARAMETERS
// ============================================
#define MAX_ANGLE_STABILIZE 30.0    // Max tilt angle in stabilize mode (degrees)
#define MAX_RATE_ACRO 180.0         // Max rotation rate in acro mode (deg/s)
#define MAX_YAW_RATE 180.0          // Max yaw rate (deg/s)

// ============================================
// MOTOR CONFIGURATION
// ============================================
#define MOTOR_MIN_ARMED 1100        // Minimum motor speed when armed
#define MOTOR_MIN_DISARMED 1000     // Minimum motor speed when disarmed
#define MOTOR_MAX 2000              // Maximum motor speed

// Motor pins (PWM capable pins only!)
#define MOTOR1_PIN 3                // Front-Left
#define MOTOR2_PIN 5                // Front-Right
#define MOTOR3_PIN 6                // Back-Right
#define MOTOR4_PIN 11               // Back-Left

// ============================================
// SENSOR CONFIGURATION
// ============================================
#define MPU6050_ADDRESS 0x68        // I2C address (0x68 or 0x69)
#define GYRO_SENSITIVITY 65.5       // For ±500°/s range
#define ACCEL_SENSITIVITY 4096.0    // For ±8g range

// Complementary filter (gyro vs accel weight)
#define GYRO_WEIGHT 0.98            // 98% gyro, 2% accel
#define ACCEL_WEIGHT 0.02

// ============================================
// TIMING CONFIGURATION
// ============================================
#define LOOP_FREQUENCY 250          // Hz (main loop rate)
#define LOOP_PERIOD 4000            // microseconds (1/250Hz = 4ms)

// ============================================
// SAFETY CONFIGURATION
// ============================================
#define FAILSAFE_TIMEOUT 1000       // ms without signal before failsafe
#define MIN_THROTTLE_TO_ARM 1050    // Throttle must be below this to arm
#define BATTERY_MIN_VOLTAGE 10.5    // Minimum battery voltage (adjust for your battery)
#define BATTERY_WARNING_VOLTAGE 11.1 // Warning voltage

// ============================================
// CALIBRATION
// ============================================
#define GYRO_CALIBRATION_SAMPLES 2000 // Number of samples for gyro calibration

// ============================================
// DEBUG OPTIONS
// ============================================
#define DEBUG_PRINT_INTERVAL 250    // ms between debug prints
#define ENABLE_SERIAL_DEBUG true    // Enable/disable serial debug
#define SERIAL_BAUD_RATE 115200     // Serial baud rate

#endif // CONFIG_H
