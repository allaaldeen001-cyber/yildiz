#include "mpu6050.h"

#include <Wire.h>

namespace drone {

namespace {
constexpr float kGyroSensitivity = 65.5f;   // LSB/deg/s for +/-500dps
constexpr float kAccelSensitivity = 8192.0f;  // LSB/g for +/-4g
}

bool Mpu6050::begin(uint8_t address) {
  address_ = address;
  delay(100);
  if (!writeByte(0x6B, 0x00)) {  // Wake up device
    return false;
  }
  if (!writeByte(0x1B, 0x08)) {  // +/-500 dps
    return false;
  }
  if (!writeByte(0x1C, 0x08)) {  // +/-4 g
    return false;
  }
  if (!writeByte(0x1A, 0x03)) {  // DLPF ~44 Hz
    return false;
  }
  return true;
}

bool Mpu6050::readRaw(ImuRaw& raw) {
  uint8_t buffer[14];
  if (!readBytes(0x3B, sizeof(buffer), buffer)) {
    return false;
  }

  raw.ax = (buffer[0] << 8) | buffer[1];
  raw.ay = (buffer[2] << 8) | buffer[3];
  raw.az = (buffer[4] << 8) | buffer[5];
  raw.gx = (buffer[8] << 8) | buffer[9];
  raw.gy = (buffer[10] << 8) | buffer[11];
  raw.gz = (buffer[12] << 8) | buffer[13];
  return true;
}

void Mpu6050::setGyroOffsets(float x, float y, float z) {
  gyro_offsets_[0] = x;
  gyro_offsets_[1] = y;
  gyro_offsets_[2] = z;
}

void Mpu6050::setAccelOffsets(float x, float y, float z) {
  accel_offsets_[0] = x;
  accel_offsets_[1] = y;
  accel_offsets_[2] = z;
}

void Mpu6050::toEngineeringUnits(const ImuRaw& raw,
                                 float& ax_g,
                                 float& ay_g,
                                 float& az_g,
                                 float& gx_dps,
                                 float& gy_dps,
                                 float& gz_dps) const {
  ax_g = (static_cast<float>(raw.ax) - accel_offsets_[0]) / kAccelSensitivity;
  ay_g = (static_cast<float>(raw.ay) - accel_offsets_[1]) / kAccelSensitivity;
  az_g = (static_cast<float>(raw.az) - accel_offsets_[2]) / kAccelSensitivity;

  gx_dps = (static_cast<float>(raw.gx) - gyro_offsets_[0]) / kGyroSensitivity;
  gy_dps = (static_cast<float>(raw.gy) - gyro_offsets_[1]) / kGyroSensitivity;
  gz_dps = (static_cast<float>(raw.gz) - gyro_offsets_[2]) / kGyroSensitivity;
}

bool Mpu6050::writeByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address_);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool Mpu6050::readBytes(uint8_t reg, uint8_t length, uint8_t* dest) {
  Wire.beginTransmission(address_);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  Wire.requestFrom(address_, length, static_cast<uint8_t>(true));
  uint8_t idx = 0;
  while (Wire.available() && idx < length) {
    dest[idx++] = Wire.read();
  }
  return idx == length;
}

}  // namespace drone
