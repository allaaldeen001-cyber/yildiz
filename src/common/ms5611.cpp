#include "ms5611.h"

#include <Wire.h>
#include <math.h>

namespace drone {

namespace {
constexpr uint8_t kCmdReset = 0x1E;
constexpr uint8_t kCmdAdcRead = 0x00;
constexpr uint8_t kCmdConvD1 = 0x48;  // OSR=4096 pressure
constexpr uint8_t kCmdConvD2 = 0x58;  // OSR=4096 temperature
}

bool Ms5611::begin(uint8_t address) {
  address_ = address;
  if (!reset()) {
    return false;
  }
  delay(10);
  return readProm();
}

bool Ms5611::readPressureTemperature(float& pressure_pa, float& temperature_c) {
  uint32_t D1 = 0;
  uint32_t D2 = 0;
  if (!readAdc(kCmdConvD1, D1)) {
    return false;
  }
  if (!readAdc(kCmdConvD2, D2)) {
    return false;
  }

  int32_t dT = static_cast<int32_t>(D2) - (static_cast<int32_t>(prom_[5]) << 8);
  int64_t OFF = (static_cast<int64_t>(prom_[2]) << 16) + ((static_cast<int64_t>(prom_[4]) * dT) >> 7);
  int64_t SENS = (static_cast<int64_t>(prom_[1]) << 15) + ((static_cast<int64_t>(prom_[3]) * dT) >> 8);
  int32_t TEMP = 2000 + ((static_cast<int64_t>(dT) * prom_[6]) >> 23);

  int64_t T2 = 0;
  int64_t OFF2 = 0;
  int64_t SENS2 = 0;

  if (TEMP < 2000) {
    int64_t temp_minus = TEMP - 2000;
    T2 = (static_cast<int64_t>(dT) * dT) >> 31;
    OFF2 = (5 * temp_minus * temp_minus) >> 1;
    SENS2 = (5 * temp_minus * temp_minus) >> 2;
    if (TEMP < -1500) {
      int64_t temp2 = TEMP + 1500;
      OFF2 += 7 * temp2 * temp2;
      SENS2 += (11 * temp2 * temp2) >> 1;
    }
  }

  TEMP -= static_cast<int32_t>(T2);
  OFF -= OFF2;
  SENS -= SENS2;

  int32_t P = static_cast<int32_t>((((static_cast<int64_t>(D1) * SENS) >> 21) - OFF) >> 15);

  temperature_c = TEMP / 100.0f;
  pressure_pa = static_cast<float>(P);
  return true;
}

bool Ms5611::readAltitude(float& altitude_cm) {
  float pressure = 0.0f;
  float temperature = 0.0f;
  if (!readPressureTemperature(pressure, temperature)) {
    return false;
  }
  float ratio = pressure / sea_level_pressure_pa_;
  float altitude_m = (1.0f - powf(ratio, 0.190284f)) * 44330.0f;
  altitude_cm = altitude_m * 100.0f;
  return true;
}

bool Ms5611::reset() {
  Wire.beginTransmission(address_);
  Wire.write(kCmdReset);
  return Wire.endTransmission() == 0;
}

bool Ms5611::readProm() {
  for (uint8_t i = 0; i < 8; ++i) {
    Wire.beginTransmission(address_);
    Wire.write(0xA0 | (i << 1));
    if (Wire.endTransmission(false) != 0) {
      return false;
    }
    Wire.requestFrom(address_, static_cast<uint8_t>(2));
    if (Wire.available() < 2) {
      return false;
    }
    uint16_t value = (Wire.read() << 8) | Wire.read();
    prom_[i] = value;
  }
  return true;
}

bool Ms5611::readAdc(uint8_t command, uint32_t& value) {
  Wire.beginTransmission(address_);
  Wire.write(command);
  if (Wire.endTransmission() != 0) {
    return false;
  }
  delayMicroseconds(10000);  // Max conversion time for OSR=4096

  Wire.beginTransmission(address_);
  Wire.write(kCmdAdcRead);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  Wire.requestFrom(address_, static_cast<uint8_t>(3));
  if (Wire.available() < 3) {
    return false;
  }
  value = (static_cast<uint32_t>(Wire.read()) << 16);
  value |= (static_cast<uint32_t>(Wire.read()) << 8);
  value |= Wire.read();
  return true;
}

}  // namespace drone
