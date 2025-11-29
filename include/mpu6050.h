#pragma once

#include <Arduino.h>

namespace drone {

struct ImuRaw {
  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;
};

class Mpu6050 {
 public:
  bool begin(uint8_t address = 0x68);
  bool readRaw(ImuRaw& raw);

  void setGyroOffsets(float x, float y, float z);
  void setAccelOffsets(float x, float y, float z);

  void toEngineeringUnits(const ImuRaw& raw,
                          float& ax_g,
                          float& ay_g,
                          float& az_g,
                          float& gx_dps,
                          float& gy_dps,
                          float& gz_dps) const;

 private:
  bool writeByte(uint8_t reg, uint8_t value);
  bool readBytes(uint8_t reg, uint8_t length, uint8_t* dest);

  uint8_t address_ = 0x68;
  float gyro_offsets_[3] = {0.0f, 0.0f, 0.0f};
  float accel_offsets_[3] = {0.0f, 0.0f, 0.0f};
};

}  // namespace drone
