/*
 * ============================================================================
 * QUADCOPTER FLIGHT CONTROLLER - CONFIGURATION
 * ============================================================================
 * Pin definitions, constants, and tunable parameters
 * Hardware: Arduino Nano (ATmega328P)
 * ============================================================================
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

// --- ESC/Motor Pins (PWM capable) ---
#define MOTOR1_PIN      3   // Front-Right (CCW)
#define MOTOR2_PIN      9   // Rear-Right  (CW)
#define MOTOR3_PIN      10  // Rear-Left   (CCW)
#define MOTOR4_PIN      6   // Front-Left  (CW)

// --- NRF24L01 Pins ---
#define NRF_CE_PIN      4   // Chip Enable
#define NRF_CSN_PIN     7   // Chip Select Not
// MOSI = D11 (hardware SPI)
// MISO = D12 (hardware SPI)
// SCK  = D13 (hardware SPI)

// --- I2C Sensors (Hardware I2C) ---
// SDA = A4 (MPU6050, MS5611)
// SCL = A5 (MPU6050, MS5611)

// --- Status Indicators ---
#define STATUS_LED_PIN  2   // General status LED
#define ARM_LED_PIN     5   // Armed indicator LED
#define BUZZER_PIN      8   // Active buzzer

// ============================================================================
// SENSOR ADDRESSES
// ============================================================================
#define MPU6050_ADDRESS     0x68
#define MS5611_ADDRESS      0x77

// ============================================================================
// NRF24L01 CONFIGURATION
// ============================================================================
#define NRF_CHANNEL         76              // RF channel (0-125)
#define NRF_PA_LEVEL        RF24_PA_HIGH    // Power level
#define NRF_DATA_RATE       RF24_250KBPS    // Data rate (lower = better range)
#define NRF_PIPE_ADDRESS    0xE8E8F0F0E1LL  // Pipe address

// ============================================================================
// ESC CONFIGURATION
// ============================================================================
#define ESC_MIN_PULSE       1000    // Minimum pulse width (µs) - motors off
#define ESC_MAX_PULSE       2000    // Maximum pulse width (µs) - full throttle
#define ESC_ARM_PULSE       1000    // Arming pulse width
#define ESC_IDLE_PULSE      1050    // Idle speed when armed (slight spin)

// Motor mixing - determines how each motor responds to commands
// X-configuration quadcopter motor layout:
//   M4(CW)   M1(CCW)
//       \   /
//        \ /
//        / \
//       /   \
//   M3(CCW)  M2(CW)

// ============================================================================
// FLIGHT PARAMETERS
// ============================================================================

// --- Control Input Ranges ---
#define THROTTLE_MIN        1000    // Minimum throttle value from RC
#define THROTTLE_MAX        2000    // Maximum throttle value from RC
#define THROTTLE_DEADZONE   1050    // Below this = motors off
#define THROTTLE_ARM_MAX    1100    // Max throttle to allow arming

// --- Stick Input Center and Range ---
#define STICK_CENTER        1500    // Center position for pitch/roll/yaw
#define STICK_MIN           1000    // Minimum stick value
#define STICK_MAX           2000    // Maximum stick value
#define STICK_DEADZONE      20      // Deadzone around center

// --- Maximum Angles (degrees) ---
#define MAX_PITCH_ANGLE     30.0f   // Maximum pitch angle
#define MAX_ROLL_ANGLE      30.0f   // Maximum roll angle
#define MAX_YAW_RATE        180.0f  // Maximum yaw rate (deg/s)

// ============================================================================
// PID CONTROLLER DEFAULTS
// ============================================================================

// --- Roll PID ---
#define PID_ROLL_P          1.2f
#define PID_ROLL_I          0.02f
#define PID_ROLL_D          18.0f

// --- Pitch PID ---
#define PID_PITCH_P         1.2f
#define PID_PITCH_I         0.02f
#define PID_PITCH_D         18.0f

// --- Yaw PID ---
#define PID_YAW_P           2.0f
#define PID_YAW_I           0.02f
#define PID_YAW_D           0.0f

// --- Altitude PID (for altitude hold) ---
#define PID_ALT_P           1.5f
#define PID_ALT_I           0.01f
#define PID_ALT_D           0.5f

// --- PID Output Limits ---
#define PID_MAX_OUTPUT      400     // Maximum PID output value
#define PID_I_MAX           100     // Maximum integral term

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================
#define LOOP_FREQUENCY      250     // Main loop frequency (Hz)
#define LOOP_TIME_US        4000    // Loop time in microseconds (1000000/250)

#define NRF_TIMEOUT_MS      500     // RC signal timeout (ms)
#define TELEMETRY_RATE_MS   100     // Telemetry update rate (ms)

// ============================================================================
// CALIBRATION SETTINGS
// ============================================================================
#define CALIBRATION_SAMPLES     2000    // Number of samples for gyro calibration
#define ACCEL_CALIBRATION_TIME  5000    // Accelerometer calibration time (ms)
#define ESC_CALIBRATION_TIME    5000    // ESC calibration time (ms)

// ============================================================================
// EEPROM ADDRESSES
// ============================================================================
#define EEPROM_SIGNATURE        0x42    // Magic byte to verify EEPROM data
#define EEPROM_ADDR_SIGNATURE   0       // Signature address
#define EEPROM_ADDR_GYRO_X      1       // Gyro X offset (2 bytes)
#define EEPROM_ADDR_GYRO_Y      3       // Gyro Y offset (2 bytes)
#define EEPROM_ADDR_GYRO_Z      5       // Gyro Z offset (2 bytes)
#define EEPROM_ADDR_ACCEL_X     7       // Accel X offset (2 bytes)
#define EEPROM_ADDR_ACCEL_Y     9       // Accel Y offset (2 bytes)
#define EEPROM_ADDR_ACCEL_Z     11      // Accel Z offset (2 bytes)
#define EEPROM_ADDR_PID_ROLL    13      // Roll PID (12 bytes)
#define EEPROM_ADDR_PID_PITCH   25      // Pitch PID (12 bytes)
#define EEPROM_ADDR_PID_YAW     37      // Yaw PID (12 bytes)
#define EEPROM_ADDR_ESC_CAL     49      // ESC calibration flag

// ============================================================================
// SAFETY FEATURES
// ============================================================================
#define FAILSAFE_ENABLED        true
#define FAILSAFE_THROTTLE       1000    // Throttle value on signal loss
#define LOW_BATTERY_VOLTAGE     10.5f   // Low battery warning (V)
#define CRITICAL_BATTERY_VOLTAGE 10.0f  // Auto-land voltage (V)

// ============================================================================
// DEBUG OPTIONS
// ============================================================================
#define DEBUG_SERIAL            true    // Enable serial debugging
#define DEBUG_BAUD_RATE         115200  // Serial baud rate
#define PRINT_GYRO_DATA         false   // Print raw gyro data
#define PRINT_ACCEL_DATA        false   // Print raw accel data
#define PRINT_MOTOR_VALUES      false   // Print motor PWM values
#define PRINT_PID_VALUES        false   // Print PID calculations
#define PRINT_RC_VALUES         true    // Print RC input values

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// RC Data packet from transmitter
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

// Telemetry data packet to transmitter
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

// Drone state structure
struct DroneState {
    // Attitude
    float roll;
    float pitch;
    float yaw;
    
    // Rates
    float rollRate;
    float pitchRate;
    float yawRate;
    
    // Altitude
    float altitude;
    float verticalSpeed;
    
    // Status flags
    bool armed;
    bool calibrated;
    bool rcConnected;
    bool lowBattery;
    bool failsafe;
    
    // Flight mode
    uint8_t flightMode;
};

// Motor values
struct MotorOutputs {
    uint16_t motor1;    // Front-Right
    uint16_t motor2;    // Rear-Right
    uint16_t motor3;    // Rear-Left
    uint16_t motor4;    // Front-Left
};

// Calibration data
struct CalibrationData {
    int16_t gyroOffsetX;
    int16_t gyroOffsetY;
    int16_t gyroOffsetZ;
    int16_t accelOffsetX;
    int16_t accelOffsetY;
    int16_t accelOffsetZ;
    bool isCalibrated;
};

// ============================================================================
// FLIGHT MODES
// ============================================================================
#define FLIGHT_MODE_STABILIZE   0   // Angle mode with auto-level
#define FLIGHT_MODE_ACRO        1   // Rate mode for acrobatics
#define FLIGHT_MODE_ALT_HOLD    2   // Altitude hold mode

// ============================================================================
// STATUS FLAGS (bitmask)
// ============================================================================
#define STATUS_GYRO_OK          0x01
#define STATUS_ACCEL_OK         0x02
#define STATUS_BARO_OK          0x04
#define STATUS_NRF_OK           0x08
#define STATUS_ESC_OK           0x10
#define STATUS_CALIBRATED       0x20
#define STATUS_ARMED            0x40
#define STATUS_FLYING           0x80

#endif // CONFIG_H
