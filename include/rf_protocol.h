#pragma once

#include <stdint.h>

namespace drone {

// RF transport parameters shared by both boards
constexpr uint8_t kRadioChannel = 103;
constexpr uint64_t kRadioPipe = 0xE8E8F0F0A1LL;

// Button mapping bits (active low on hardware, processed as active high in software)
enum RcButton : uint8_t {
  kButtonCalibrate = 0x01,
  kButtonArm = 0x02
};

// Switch mapping bits
enum RcSwitch : uint8_t {
  kSwitchAltitudeHold = 0x01,
  kSwitchKill = 0x02
};

// Momentary command flags piggybacked on every RC frame
enum RcCommandFlags : uint8_t {
  kFlagRequestImuCalibration = 0x01,
  kFlagRequestEscCalibration = 0x02,
  kFlagRequestArm = 0x04
};

// Flight controller status bit-field
enum FcStatusFlags : uint8_t {
  kStatusLinkAlive = 0x01,
  kStatusImuCalibrated = 0x02,
  kStatusEscCalibrated = 0x04,
  kStatusArmed = 0x08,
  kStatusAltitudeHold = 0x10,
  kStatusKillActive = 0x20,
  kStatusSensorFault = 0x40,
  kStatusCalibrationRunning = 0x80
};

enum FcErrorCode : uint8_t {
  kErrorNone = 0,
  kErrorImu = 1,
  kErrorBarometer = 2,
  kErrorRadio = 3,
  kErrorWatchdog = 4
};

#pragma pack(push, 1)
struct RcCommand {
  uint16_t throttle_us;    // 1000-2000us command window
  int16_t roll_cmd;        // desired roll angle scaled to +/-500 -> +/-30deg
  int16_t pitch_cmd;       // desired pitch angle scaled to +/-500 -> +/-30deg
  int16_t yaw_rate_cmd;    // desired yaw rate scaled to +/-500 -> +/-120deg/s
  uint8_t buttons;         // RcButton bits (1 = pressed)
  uint8_t switches;        // RcSwitch bits (1 = ON)
  uint8_t flags;           // RcCommandFlags bits for momentary actions
  uint8_t aux;             // spare for future use (must be TX filled with 0)
  uint32_t sequence;       // monotonically increasing frame counter
};

struct FcStatus {
  uint8_t flags;           // FcStatusFlags bits
  int16_t roll_deg_x10;    // roll angle * 10 (deg)
  int16_t pitch_deg_x10;   // pitch angle *10 (deg)
  int16_t yaw_rate_dps;    // deg/s
  int16_t altitude_cm;     // relative altitude
  uint16_t battery_mv;     // optional voltage sense (0 if unused)
  uint32_t loop_time_us;   // main loop duration
  uint8_t last_error;      // FcErrorCode
  uint8_t reserved;        // padding / future use
};
#pragma pack(pop)

static_assert(sizeof(RcCommand) == 16, "RcCommand must remain 16 bytes for RF efficiency");
static_assert(sizeof(FcStatus) == 18, "FcStatus must remain 18 bytes for RF efficiency");

}  // namespace drone
