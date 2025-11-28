/**
 * ============================================================================
 * DRONE COMMUNICATION PROTOCOL DEFINITIONS
 * ============================================================================
 * 
 * Shared header file for RC ↔ FC NRF24L01 communication
 * 
 * Protocol Version: 1.0
 * Author: UAV Firmware Engineer
 * 
 * This file defines all data structures for bidirectional communication
 * between the Remote Controller and Flight Controller.
 * 
 * ============================================================================
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// ============================================================================
// PROTOCOL CONSTANTS
// ============================================================================

#define PROTOCOL_VERSION        0x01
#define NRF_CHANNEL            103
#define NRF_PAYLOAD_SIZE       32

// Communication timing
#define TX_INTERVAL_MS         10      // 100 Hz update rate
#define LINK_TIMEOUT_MS        500     // Failsafe trigger timeout
#define HEARTBEAT_INTERVAL_MS  100     // Heartbeat for link quality

// ============================================================================
// COMMAND FLAGS (Bit-packed in command_flags field)
// ============================================================================

#define CMD_FLAG_ARMED          (1 << 0)  // Bit 0: Armed state
#define CMD_FLAG_ALT_HOLD       (1 << 1)  // Bit 1: Altitude hold enabled
#define CMD_FLAG_CALIBRATE_IMU  (1 << 2)  // Bit 2: Request IMU calibration
#define CMD_FLAG_CALIBRATE_ESC  (1 << 3)  // Bit 3: Request ESC calibration
#define CMD_FLAG_MOTOR_TEST     (1 << 4)  // Bit 4: Motor test mode
#define CMD_FLAG_HEADLESS       (1 << 5)  // Bit 5: Headless mode (reserved)
#define CMD_FLAG_RTH            (1 << 6)  // Bit 6: Return to home (reserved)
#define CMD_FLAG_EMERGENCY      (1 << 7)  // Bit 7: Emergency stop

// ============================================================================
// STATUS FLAGS (Bit-packed in status_flags field)
// ============================================================================

#define STATUS_FLAG_ARMED       (1 << 0)  // Bit 0: FC is armed
#define STATUS_FLAG_ALT_HOLD    (1 << 1)  // Bit 1: Alt hold active
#define STATUS_FLAG_CALIBRATED  (1 << 2)  // Bit 2: Sensors calibrated
#define STATUS_FLAG_LOW_BATT    (1 << 3)  // Bit 3: Low battery warning
#define STATUS_FLAG_FAILSAFE    (1 << 4)  // Bit 4: Failsafe active
#define STATUS_FLAG_GPS_FIX     (1 << 5)  // Bit 5: GPS fix (reserved)
#define STATUS_FLAG_ERROR       (1 << 6)  // Bit 6: Error condition
#define STATUS_FLAG_MOTOR_TEST  (1 << 7)  // Bit 7: Motor test active

// ============================================================================
// CALIBRATION STATUS CODES
// ============================================================================

#define CALIB_IDLE              0x00
#define CALIB_IN_PROGRESS       0x01
#define CALIB_SUCCESS           0x02
#define CALIB_FAIL_MOTION       0x03  // Failed: motion detected
#define CALIB_FAIL_RANGE        0x04  // Failed: values out of range
#define CALIB_FAIL_TIMEOUT      0x05  // Failed: timeout
#define CALIB_ESC_COMPLETE      0x06  // ESC calibration complete

// ============================================================================
// RC → FC COMMAND PACKET (32 bytes max)
// ============================================================================

typedef struct __attribute__((packed)) {
    // Header (2 bytes)
    uint8_t  protocol_version;    // Protocol version for compatibility check
    uint8_t  packet_id;           // Rolling packet ID for loss detection
    
    // Control channels (8 bytes) - Range: 1000-2000
    uint16_t throttle;            // Left stick vertical (A0)
    uint16_t yaw;                 // Left stick horizontal (A1)
    uint16_t pitch;               // Right stick vertical (A2)
    uint16_t roll;                // Right stick horizontal (A3)
    
    // Command flags (1 byte)
    uint8_t  command_flags;       // Bit-packed command flags
    
    // Auxiliary channels (2 bytes) - For future expansion
    uint8_t  aux1;                // Auxiliary channel 1 (0-255)
    uint8_t  aux2;                // Auxiliary channel 2 (0-255)
    
    // Checksum (1 byte)
    uint8_t  checksum;            // XOR checksum of all previous bytes
    
} RCCommandPacket;  // Total: 14 bytes

// ============================================================================
// FC → RC TELEMETRY PACKET (32 bytes max)
// ============================================================================

typedef struct __attribute__((packed)) {
    // Header (2 bytes)
    uint8_t  protocol_version;    // Protocol version
    uint8_t  packet_id;           // Rolling packet ID
    
    // Status (1 byte)
    uint8_t  status_flags;        // Bit-packed status flags
    
    // Attitude (6 bytes) - Scaled: angle * 100
    int16_t  roll_angle;          // Roll angle in centidegrees
    int16_t  pitch_angle;         // Pitch angle in centidegrees
    int16_t  yaw_angle;           // Yaw angle in centidegrees
    
    // Altitude (4 bytes)
    int16_t  altitude;            // Altitude in cm (relative to arm point)
    int16_t  vertical_speed;      // Vertical speed in cm/s
    
    // Battery (2 bytes)
    uint16_t battery_voltage;     // Battery voltage in mV
    
    // Calibration status (1 byte)
    uint8_t  calib_status;        // Current calibration status
    
    // Loop timing (2 bytes)
    uint16_t loop_time_us;        // Main loop time in microseconds
    
    // Link quality (1 byte)
    uint8_t  link_quality;        // 0-100% link quality
    
    // Motor outputs (4 bytes) - For debugging
    uint8_t  motor_fl;            // Front-left motor (0-255)
    uint8_t  motor_fr;            // Front-right motor (0-255)
    uint8_t  motor_rr;            // Rear-right motor (0-255)
    uint8_t  motor_rl;            // Rear-left motor (0-255)
    
    // Checksum (1 byte)
    uint8_t  checksum;            // XOR checksum
    
} FCTelemetryPacket;  // Total: 24 bytes

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Calculate XOR checksum for a data buffer
 * @param data Pointer to data buffer
 * @param len  Length of data (excluding checksum byte)
 * @return XOR checksum
 */
static inline uint8_t calculateChecksum(const uint8_t* data, uint8_t len) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * Validate packet checksum
 * @param data     Pointer to packet data
 * @param len      Total packet length including checksum
 * @return true if checksum is valid
 */
static inline bool validateChecksum(const uint8_t* data, uint8_t len) {
    if (len < 2) return false;
    uint8_t calculated = calculateChecksum(data, len - 1);
    return (calculated == data[len - 1]);
}

/**
 * Constrain a value between min and max
 */
static inline int16_t constrainValue(int16_t value, int16_t min, int16_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * Map value from one range to another
 */
static inline int32_t mapValue(int32_t x, int32_t in_min, int32_t in_max, 
                                int32_t out_min, int32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ============================================================================
// NRF24L01 PIPE ADDRESSES
// ============================================================================

// 5-byte addresses for NRF24L01 pipes
// Pipe 0: RC → FC (commands)
// Pipe 1: FC → RC (telemetry via ACK payload)
static const uint8_t NRF_PIPE_ADDRESS[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };

#endif // PROTOCOL_H
