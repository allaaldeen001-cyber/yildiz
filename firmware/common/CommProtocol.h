#pragma once

#include <Arduino.h>

// Radio channel and addressing shared by FC and RC.
constexpr uint8_t kNrfChannel = 103;
constexpr uint64_t kNrfAddress = 0xE8E8F0F0E1LL;

// Switch bit assignments sent from RC to FC.
constexpr uint8_t SWITCH_ALT_HOLD = 0x01;   // SW_1 on RC
constexpr uint8_t SWITCH_ARM = 0x02;        // SW_2 on RC (armed when HIGH)

// Button bit assignments.
constexpr uint8_t BUTTON_CALIBRATE = 0x01;  // Button_1
constexpr uint8_t BUTTON_MOTOR_TEST = 0x02; // Button_2

// Status flags reported by FC to RC.
constexpr uint8_t FC_STATUS_LINKED = 0x01;
constexpr uint8_t FC_STATUS_ARMED = 0x02;
constexpr uint8_t FC_STATUS_CALIBRATED = 0x04;
constexpr uint8_t FC_STATUS_ALT_HOLD = 0x08;
constexpr uint8_t FC_STATUS_FAILSAFE = 0x10;

#pragma pack(push, 1)
struct RcToFcPacket {
  uint16_t seq;
  uint16_t throttle;  // 1000-2000 us equivalent command
  int16_t roll;        // -500..+500 deg * 10 (scaled to tenths)
  int16_t pitch;       // -500..+500 deg * 10
  int16_t yaw;         // -500..+500 deg/s * 10
  uint8_t switches;    // bitfield, see SWITCH_* masks
  uint8_t buttons;     // momentary bits, see BUTTON_* masks
  uint8_t aux;         // reserved for future use
  uint16_t checksum;
};

struct FcToRcPacket {
  uint16_t seq;
  int16_t rollAngleDeg;   // deg * 100
  int16_t pitchAngleDeg;  // deg * 100
  int16_t yawRateDps;     // deg/s * 10
  int16_t altitudeCm;     // centimeters
  int16_t batteryMv;      // millivolts
  uint8_t statusFlags;    // FC_STATUS_* bits
  uint8_t linkQuality;    // percentage 0-100
  uint16_t checksum;
};
#pragma pack(pop)

inline uint16_t simpleChecksum(const uint8_t *data, size_t length) {
  uint16_t sum = 0;
  for (size_t i = 0; i < length; ++i) {
    sum += data[i];
  }
  return sum;
}

inline void finalizePacket(RcToFcPacket &pkt) {
  pkt.checksum = simpleChecksum(reinterpret_cast<const uint8_t *>(&pkt),
                                sizeof(RcToFcPacket) - sizeof(pkt.checksum));
}

inline bool validatePacket(const RcToFcPacket &pkt) {
  return pkt.checksum ==
         simpleChecksum(reinterpret_cast<const uint8_t *>(&pkt),
                        sizeof(RcToFcPacket) - sizeof(pkt.checksum));
}

inline void finalizePacket(FcToRcPacket &pkt) {
  pkt.checksum = simpleChecksum(reinterpret_cast<const uint8_t *>(&pkt),
                                sizeof(FcToRcPacket) - sizeof(pkt.checksum));
}

inline bool validatePacket(const FcToRcPacket &pkt) {
  return pkt.checksum ==
         simpleChecksum(reinterpret_cast<const uint8_t *>(&pkt),
                        sizeof(FcToRcPacket) - sizeof(pkt.checksum));
}
