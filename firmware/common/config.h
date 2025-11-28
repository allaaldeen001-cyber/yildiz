#pragma once

#include <Arduino.h>

// Shared radio configuration.
constexpr uint64_t RADIO_PIPE = 0xE8E8F0F0E1LL;
constexpr uint8_t RADIO_CHANNEL = 90;
constexpr uint8_t PACKET_VERSION = 1;
constexpr uint8_t RC_CHANNEL_COUNT = 4;

// High-level guide steps that the RC display walks through.
enum class GuideStep : uint8_t {
  WAIT_LINK = 0,
  REQUEST_KILL = 1,
  REQUEST_CAL = 2,
  CALIBRATING = 3,
  REQUEST_ARM = 4,
  REQUEST_IDLE_SPIN = 5,
  READY = 6,
  FLYING = 7
};

#pragma pack(push, 1)
struct ControlPacket {
  uint8_t version;
  uint16_t throttle;
  uint16_t roll;
  uint16_t pitch;
  uint16_t yaw;
  uint8_t button1;       // IMU/ESC calibration request.
  uint8_t button2;       // Smooth idle spin request.
  uint8_t killSwitch;    // 0 = kill, 1 = arm request.
  uint8_t reserved;      // Reserved for future expansion / joystick health.
  uint8_t channel;       // Radio channel for display confirmation.
  uint8_t checksum;
};

struct TelemetryPacket {
  uint8_t version;
  float altitudeMeters;
  float batteryVoltage;
  uint16_t throttleEcho;
  uint16_t rollEcho;
  uint16_t pitchEcho;
  uint16_t yawEcho;
  uint8_t armed;
  uint8_t imuCalibrated;
  uint8_t escCalibrated;
  uint8_t linkQuality;   // 0-100 (% of successful frames).
  uint8_t guideStep;     // GuideStep enum value.
  uint8_t checksum;
};
#pragma pack(pop)

static_assert(sizeof(ControlPacket) <= 32, "ControlPacket exceeds NRF24 payload");
static_assert(sizeof(TelemetryPacket) <= 32, "TelemetryPacket exceeds NRF24 payload");

template <typename T>
uint8_t computeChecksum(const T &packet) {
  const uint8_t *raw = reinterpret_cast<const uint8_t *>(&packet);
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(T) - 1; ++i) {
    sum ^= raw[i];
  }
  return sum;
}

inline bool isChecksumValid(const ControlPacket &packet) {
  return computeChecksum(packet) == packet.checksum;
}

inline bool isChecksumValid(const TelemetryPacket &packet) {
  return computeChecksum(packet) == packet.checksum;
}
