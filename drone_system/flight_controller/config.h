/*
 * ============================================================================
 * DRONE SYSTEM CONFIGURATION
 * Professional UAV Embedded System
 * ============================================================================
 * 
 * Common configuration file shared between Flight Controller and Remote Controller
 * 
 * Author: UAV Systems Engineer
 * Version: 1.0.0
 * ============================================================================
 */

#ifndef DRONE_CONFIG_H
#define DRONE_CONFIG_H

// ============================================================================
// NRF24L01 COMMUNICATION CONFIGURATION
// ============================================================================

#define NRF_CHANNEL           103       // RF channel (0-125)
#define NRF_DATA_RATE         RF24_250KBPS  // Data rate for long range
#define NRF_PA_LEVEL          RF24_PA_MAX   // Maximum power for PA+LNA module
#define NRF_PAYLOAD_SIZE      32        // Maximum payload size

// Communication addresses (5 bytes)
const uint8_t RC_TO_FC_ADDR[6] = "RC2FC";  // RC transmits, FC receives
const uint8_t FC_TO_RC_ADDR[6] = "FC2RC";  // FC transmits (ACK), RC receives

// Communication timing
#define COMM_TIMEOUT_MS       500       // Connection lost timeout
#define COMM_UPDATE_RATE_MS   20        // 50Hz update rate
#define ACK_PAYLOAD_SIZE      16        // ACK payload size

// ============================================================================
// CONTROL RANGES
// ============================================================================

// Joystick ADC range
#define JOYSTICK_MIN          0
#define JOYSTICK_MAX          1023
#define JOYSTICK_CENTER       512
#define JOYSTICK_DEADZONE     30

// PWM output range (ESC)
#define ESC_MIN_PULSE         1000      // Minimum throttle
#define ESC_MAX_PULSE         2000      // Maximum throttle
#define ESC_ARM_PULSE         1050      // Arm threshold
#define ESC_IDLE_PULSE        1100      // Idle when armed

// Control scaling
#define THROTTLE_MAX_PERCENT  65        // Safety: Max 65% throttle
#define MAX_ANGLE_DEG         30        // Safety: Max tilt angle
#define MAX_YAW_RATE          180       // Max yaw rate (deg/s)

// ============================================================================
// COMMAND PACKET STRUCTURE (RC -> FC)
// ============================================================================

typedef struct __attribute__((packed)) {
    uint8_t  header;          // Packet header (0xAA)
    uint16_t throttle;        // 0-1000 mapped value
    int16_t  yaw;             // -500 to +500
    int16_t  pitch;           // -500 to +500
    int16_t  roll;            // -500 to +500
    uint8_t  buttons;         // Bit flags for buttons
    uint8_t  switches;        // Bit flags for switches
    uint8_t  command;         // Special commands
    uint8_t  sequence;        // Packet sequence number
    uint8_t  checksum;        // XOR checksum
} CommandPacket_t;

// Button bit flags
#define BTN_CALIBRATE         0x01      // Button 1 - Calibration
#define BTN_MOTORS_ON         0x02      // Button 2 - Motors on
#define BTN_RESERVED1         0x04
#define BTN_RESERVED2         0x08

// Switch bit flags
#define SW_ALT_HOLD           0x01      // Switch 1 - Altitude hold
#define SW_ARM                0x02      // Switch 2 - Arm/Disarm (kill switch)
#define SW_RESERVED1          0x04
#define SW_RESERVED2          0x08

// Commands
#define CMD_NONE              0x00
#define CMD_CALIBRATE_GYRO    0x01
#define CMD_CALIBRATE_ESC     0x02
#define CMD_EMERGENCY_STOP    0x03
#define CMD_RESET             0x04

// ============================================================================
// TELEMETRY PACKET STRUCTURE (FC -> RC via ACK)
// ============================================================================

typedef struct __attribute__((packed)) {
    uint8_t  header;          // Packet header (0x55)
    uint8_t  status;          // System status flags
    int16_t  pitch;           // Current pitch angle x10
    int16_t  roll;            // Current roll angle x10
    int16_t  yaw;             // Current yaw angle x10
    uint16_t altitude;        // Altitude in cm
    uint8_t  battery;         // Battery percentage (0-100)
    uint8_t  rssi;            // Signal strength (placeholder)
    uint8_t  sequence;        // Response sequence
    uint8_t  checksum;        // XOR checksum
} TelemetryPacket_t;

// Status flags
#define STATUS_CONNECTED      0x01
#define STATUS_ARMED          0x02
#define STATUS_CALIBRATED     0x04
#define STATUS_ALT_HOLD       0x08
#define STATUS_LOW_BATTERY    0x10
#define STATUS_ERROR          0x20
#define STATUS_ESC_CALIBRATED 0x40
#define STATUS_MOTORS_ON      0x80

// ============================================================================
// PACKET HELPERS
// ============================================================================

// Calculate XOR checksum
inline uint8_t calculateChecksum(const uint8_t* data, uint8_t len) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len - 1; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Validate command packet
inline bool validateCommandPacket(const CommandPacket_t* pkt) {
    if (pkt->header != 0xAA) return false;
    return (calculateChecksum((const uint8_t*)pkt, sizeof(CommandPacket_t)) == pkt->checksum);
}

// Validate telemetry packet
inline bool validateTelemetryPacket(const TelemetryPacket_t* pkt) {
    if (pkt->header != 0x55) return false;
    return (calculateChecksum((const uint8_t*)pkt, sizeof(TelemetryPacket_t)) == pkt->checksum);
}

#endif // DRONE_CONFIG_H
