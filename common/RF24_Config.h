/*
 * RF24_Config.h - Common NRF24L01 Configuration
 * TMRh20 RF24 Library - Optimized for Drone Control
 * 
 * IMPORTANT: This file must be the same on both transmitter and receiver!
 */

#ifndef RF24_CONFIG_H
#define RF24_CONFIG_H

// ============================================================================
// NRF24L01 PIN CONFIGURATION
// ============================================================================

// Transmitter pins (Arduino Nano/Uno)
#define TX_CE_PIN   9
#define TX_CSN_PIN  10

// Receiver pins (Arduino Nano/Uno) 
#define RX_CE_PIN   9
#define RX_CSN_PIN  10

// ============================================================================
// RF24 RADIO SETTINGS - OPTIMIZED FOR DRONE CONTROL
// ============================================================================

// Radio address - must match on TX and RX (5 bytes)
const byte RADIO_ADDRESS[6] = "DRN01";

// Channel (0-125) - Choose a clear channel, avoid WiFi (channels 1-25 overlap)
// Recommended: 76-100 for less interference
#define RF_CHANNEL  100

// Data rate options:
// RF24_250KBPS - Longest range, highest latency
// RF24_1MBPS   - Balanced
// RF24_2MBPS   - Shortest range, lowest latency (BEST FOR DRONES)
#define RF_DATA_RATE  RF24_2MBPS

// Power level options:
// RF24_PA_MIN   (-18dBm)
// RF24_PA_LOW   (-12dBm)
// RF24_PA_HIGH  (-6dBm)
// RF24_PA_MAX   (0dBm) - RECOMMENDED for drones
#define RF_PA_LEVEL  RF24_PA_MAX

// ============================================================================
// COMMUNICATION PROTOCOL
// ============================================================================

// Transmission frequency (Hz) - How often transmitter sends data
#define TX_FREQUENCY_HZ  100  // 100Hz = 10ms between packets

// Signal timeout (ms) - When to trigger failsafe
#define SIGNAL_TIMEOUT_MS  200  // 200ms without signal = failsafe

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Control data sent from transmitter to receiver (12 bytes)
struct ControlData {
  uint16_t throttle;    // 0-1000 (mapped from joystick)
  int16_t roll;         // -500 to +500 (mapped from joystick)
  int16_t pitch;        // -500 to +500 (mapped from joystick)
  int16_t yaw;          // -500 to +500 (mapped from joystick)
  uint8_t aux1;         // 0-255 (auxiliary channel 1 - arm/disarm)
  uint8_t aux2;         // 0-255 (auxiliary channel 2 - flight mode)
  uint8_t checksum;     // Simple checksum for data validation
};

// Telemetry data sent back from receiver (optional, if using ACK payload)
struct TelemetryData {
  uint16_t batteryVoltage;  // Battery voltage * 100 (e.g., 1120 = 11.20V)
  int16_t altitude;         // Altitude in cm
  uint8_t signalStrength;   // 0-100%
  uint8_t flightMode;       // Current flight mode
  uint8_t armed;            // Armed status
  uint8_t checksum;         // Checksum
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Calculate simple XOR checksum
inline uint8_t calculateChecksum(const uint8_t* data, uint8_t length) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < length - 1; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// Validate control data checksum
inline bool validateControlData(const ControlData& data) {
  return data.checksum == calculateChecksum((uint8_t*)&data, sizeof(ControlData));
}

// Set control data checksum
inline void setControlChecksum(ControlData& data) {
  data.checksum = calculateChecksum((uint8_t*)&data, sizeof(ControlData));
}

#endif // RF24_CONFIG_H
