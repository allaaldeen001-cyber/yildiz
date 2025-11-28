/*
 * Drone Communication Protocol
 * Shared header for FC and RC
 * Optimized for NRF24L01 with ACK mode
 */

#ifndef DRONE_PROTOCOL_H
#define DRONE_PROTOCOL_H

// NRF24L01 Configuration
#define NRF_CHANNEL 103
#define NRF_PAYLOAD_SIZE 32

// RC to FC Command Structure (32 bytes max)
struct RC_Command {
  uint16_t throttle;    // 1000-2000 (neutral ~1500)
  uint16_t yaw;         // 1000-2000 (neutral ~1500)
  uint16_t pitch;       // 1000-2000 (neutral ~1500)
  uint16_t roll;        // 1000-2000 (neutral ~1500)
  uint8_t button1;      // Calibration button (0/1)
  uint8_t button2;      // Motor ON/ESC cal (0/1)
  uint8_t sw1;          // Altitude Hold (0/1)
  uint8_t sw2;          // ARM/DISARM (0/1)
  uint8_t checksum;     // Simple checksum
} __attribute__((packed));

// FC to RC Telemetry Structure (32 bytes max)
struct FC_Telemetry {
  float roll;           // Current roll angle (degrees)
  float pitch;          // Current pitch angle (degrees)
  float yaw_rate;       // Yaw rate (deg/s)
  float altitude;       // Altitude (meters)
  float vertical_speed; // Vertical speed (m/s)
  uint8_t armed;        // Armed status (0/1)
  uint8_t altitude_hold; // Altitude hold active (0/1)
  uint8_t link_status;  // Link status (0/1)
  uint8_t calibration_status; // Calibration status (0/1)
  uint8_t checksum;     // Simple checksum
} __attribute__((packed));

// Calculate checksum for RC command
inline uint8_t calculateRC_Checksum(RC_Command* cmd) {
  uint8_t sum = 0;
  uint8_t* data = (uint8_t*)cmd;
  for (int i = 0; i < sizeof(RC_Command) - 1; i++) {
    sum ^= data[i];
  }
  return sum;
}

// Calculate checksum for FC telemetry
inline uint8_t calculateFC_Checksum(FC_Telemetry* tel) {
  uint8_t sum = 0;
  uint8_t* data = (uint8_t*)tel;
  for (int i = 0; i < sizeof(FC_Telemetry) - 1; i++) {
    sum ^= data[i];
  }
  return sum;
}

// Validate checksum
inline bool validateChecksum(uint8_t* data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len - 1; i++) {
    sum ^= data[i];
  }
  return (sum == data[len - 1]);
}

#endif // DRONE_PROTOCOL_H
