/**
 * @file protocol.h
 * @brief Shared Communication Protocol for Drone System
 * @author UAV Embedded Systems Engineer
 * @version 2.0
 * @date 2024
 * 
 * This header defines the communication protocol between
 * Flight Controller (FC) and Remote Controller (RC) boards.
 * Uses NRF24L01 PA+LNA with ACK payload for bidirectional comms.
 */

#ifndef DRONE_PROTOCOL_H
#define DRONE_PROTOCOL_H

#include <stdint.h>

// ============================================================================
// NRF24L01 CONFIGURATION
// ============================================================================

#define NRF_CHANNEL         103         // RF Channel (2.4GHz + 103MHz)
#define NRF_PA_LEVEL        RF24_PA_MAX // Maximum power for PA+LNA module
#define NRF_DATA_RATE       RF24_250KBPS// Best range with PA+LNA
#define NRF_CRC_LENGTH      RF24_CRC_16 // 16-bit CRC for reliability
#define NRF_RETRY_DELAY     5           // 1500us retry delay
#define NRF_RETRY_COUNT     15          // Max 15 retries
#define NRF_PAYLOAD_SIZE    32          // Fixed payload size

// Communication addresses (5-byte)
const uint8_t RC_TO_FC_ADDR[6] = "DRN01";  // RC transmits to FC
const uint8_t FC_TO_RC_ADDR[6] = "DRN02";  // FC transmits (ACK) to RC

// ============================================================================
// COMMAND DEFINITIONS
// ============================================================================

// Commands from RC to FC
typedef enum {
    CMD_NONE            = 0x00,
    CMD_HEARTBEAT       = 0x01,
    CMD_CALIBRATE_IMU   = 0x10,
    CMD_CALIBRATE_ESC   = 0x11,
    CMD_ARM             = 0x20,
    CMD_DISARM          = 0x21,
    CMD_CONTROL         = 0x30,
    CMD_SET_PID         = 0x40,
    CMD_REQUEST_STATUS  = 0x50,
} DroneCommand_t;

// FC Status codes (sent via ACK)
typedef enum {
    STATUS_BOOT             = 0x00,
    STATUS_IDLE             = 0x01,
    STATUS_CALIBRATING      = 0x02,
    STATUS_CALIBRATED       = 0x03,
    STATUS_CALIBRATION_FAIL = 0x04,
    STATUS_ESC_CALIBRATING  = 0x05,
    STATUS_ESC_CALIBRATED   = 0x06,
    STATUS_ARMED            = 0x10,
    STATUS_DISARMED         = 0x11,
    STATUS_FLYING           = 0x20,
    STATUS_LOW_BATTERY      = 0x30,
    STATUS_ERROR            = 0xFF,
} DroneStatus_t;

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * @brief Control packet from RC to FC (32 bytes)
 */
typedef struct __attribute__((packed)) {
    uint8_t  header;        // Always 0xAA for validation
    uint8_t  command;       // DroneCommand_t
    uint16_t throttle;      // 1000-2000 (mapped from joystick)
    int16_t  yaw;           // -500 to +500 (rotation rate)
    int16_t  pitch;         // -500 to +500 (tilt forward/back)
    int16_t  roll;          // -500 to +500 (tilt left/right)
    uint8_t  aux1;          // Switch 1 state (altitude hold)
    uint8_t  aux2;          // Switch 2 state (arm/disarm)
    uint8_t  btn1;          // Button 1 state (calibration)
    uint8_t  btn2;          // Button 2 state (motor on)
    uint8_t  sequence;      // Packet sequence number
    uint8_t  reserved[18];  // Reserved for future use
    uint8_t  checksum;      // XOR checksum
} RCPacket_t;

/**
 * @brief Telemetry packet from FC to RC (via ACK payload, 32 bytes)
 */
typedef struct __attribute__((packed)) {
    uint8_t  header;        // Always 0xBB for validation
    uint8_t  status;        // DroneStatus_t
    int16_t  roll_angle;    // Current roll angle x10 (degrees)
    int16_t  pitch_angle;   // Current pitch angle x10 (degrees)
    int16_t  yaw_angle;     // Current yaw angle x10 (degrees)
    int16_t  altitude;      // Altitude in cm (if sensor available)
    uint16_t battery_mv;    // Battery voltage in mV
    uint16_t motor_fl;      // Front-Left motor PWM
    uint16_t motor_fr;      // Front-Right motor PWM
    uint16_t motor_rr;      // Rear-Right motor PWM
    uint16_t motor_rl;      // Rear-Left motor PWM
    uint8_t  signal_quality;// Link quality 0-100%
    uint8_t  error_code;    // Error code if any
    uint8_t  sequence;      // Packet sequence number
    uint8_t  reserved[6];   // Reserved for future use
    uint8_t  checksum;      // XOR checksum
} FCTelemetry_t;

// ============================================================================
// SAFETY LIMITS
// ============================================================================

#define THROTTLE_MIN        1000
#define THROTTLE_MAX        2000
#define THROTTLE_IDLE       1050
#define THROTTLE_CAP        1650    // 65% max throttle for safety

#define MAX_ANGLE           30      // Maximum tilt angle in degrees
#define MAX_YAW_RATE        180     // Max yaw rate deg/s

#define FAILSAFE_TIMEOUT_MS 500     // Time before failsafe activates
#define HEARTBEAT_INTERVAL  50      // Heartbeat every 50ms (20Hz control loop)

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Calculate XOR checksum for packet
 */
static inline uint8_t calculateChecksum(const uint8_t* data, uint8_t len) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len - 1; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief Validate RC packet
 */
static inline bool validateRCPacket(const RCPacket_t* pkt) {
    if (pkt->header != 0xAA) return false;
    uint8_t calc = calculateChecksum((const uint8_t*)pkt, sizeof(RCPacket_t));
    return (calc == pkt->checksum);
}

/**
 * @brief Validate FC telemetry packet
 */
static inline bool validateFCPacket(const FCTelemetry_t* pkt) {
    if (pkt->header != 0xBB) return false;
    uint8_t calc = calculateChecksum((const uint8_t*)pkt, sizeof(FCTelemetry_t));
    return (calc == pkt->checksum);
}

#endif // DRONE_PROTOCOL_H
