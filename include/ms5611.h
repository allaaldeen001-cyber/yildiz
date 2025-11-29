#pragma once

#include <Arduino.h>

namespace drone {

class Ms5611 {
 public:
  bool begin(uint8_t address = 0x77);
  bool readPressureTemperature(float& pressure_pa, float& temperature_c);
  bool readAltitude(float& altitude_cm);

  void setSeaLevelPressure(float pressure_pa) { sea_level_pressure_pa_ = pressure_pa; }
  float seaLevelPressure() const { return sea_level_pressure_pa_; }

 private:
  bool reset();
  bool readProm();
  bool readAdc(uint8_t command, uint32_t& value);

  uint8_t address_ = 0x77;
  uint16_t prom_[8] = {0};
  float sea_level_pressure_pa_ = 101325.0f;
};

}  // namespace drone
