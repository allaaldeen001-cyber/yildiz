#ifndef DRONE_LINK_H
#define DRONE_LINK_H

#include <Arduino.h>

namespace DroneLink {

constexpr uint8_t kRadioChannel = 103;           // Shared RF channel
constexpr byte kRadioAddress[6] = "DLINK";     // 5-byte pipe address

// Control flag bit positions
enum ControlFlags : uint8_t {
  CTRL_FLAG_CALIBRATE    = 0x01,  // Button 1
  CTRL_FLAG_ARM_BUTTON   = 0x02,  // Button 2 short press
  CTRL_FLAG_ALT_HOLD     = 0x04,  // Switch 1: 1=hold altitude
  CTRL_FLAG_ARM_SWITCH   = 0x08,  // Switch 2: 1=arming allowed
  CTRL_FLAG_ESC_CALIB    = 0x10,  // Button 2 long press when ALT HOLD off
  CTRL_FLAG_RESERVED     = 0x20   // Spare for future use
};

// Telemetry/system flag bit positions
enum TelemetryFlags : uint8_t {
  TLM_FLAG_LINK_OK       = 0x01,
  TLM_FLAG_IMU_READY     = 0x02,
  TLM_FLAG_ESC_READY     = 0x04,
  TLM_FLAG_MOTORS_ARMED  = 0x08,
  TLM_FLAG_FAILSAFE      = 0x10,
  TLM_FLAG_ALT_HOLD      = 0x20
};

enum class FlightState : uint8_t {
  DISARMED = 0,
  IMU_CALIBRATING,
  ESC_CALIBRATING,
  ARMED_STABLE
};

enum class BuzzerCue : uint8_t {
  NONE = 0,
  CALIBRATION_OK,
  CALIBRATION_FAIL,
  ESC_SEQUENCE_START,
  ESC_SEQUENCE_DONE,
  ARMING_CHANGE
};

struct __attribute__((packed)) ControlPacket {
  uint16_t throttle;   // microseconds (1000-2000)
  int16_t roll;        // -500 .. 500
  int16_t pitch;       // -500 .. 500
  int16_t yaw;         // -500 .. 500
  uint8_t flags;       // ControlFlags bitfield
  uint8_t reserved;    // Align to even bytes for easier extensions
  uint16_t sequence;   // Monotonic counter from RC
};

struct __attribute__((packed)) TelemetryPacket {
  uint16_t frame;          // FC frame counter
  int16_t roll;            // deg * 100
  int16_t pitch;           // deg * 100
  int16_t yawRate;         // deg/s * 100
  uint8_t systemFlags;     // TelemetryFlags bitfield
  uint8_t flightState;     // FlightState enum
  uint16_t lastCommandMs;  // Age of last valid RC command
  uint8_t buzzerCue;       // BuzzerCue enum echoed back
  uint8_t reserved;        // Keep packet even-sized for NRF24
};

static_assert(sizeof(ControlPacket) <= 32, "ControlPacket exceeds NRF24 payload limit");
static_assert(sizeof(TelemetryPacket) <= 32, "TelemetryPacket exceeds NRF24 payload limit");

}  // namespace DroneLink

#endif
