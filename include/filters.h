#pragma once

#include <Arduino.h>

namespace drone {

class ComplementaryFilter {
 public:
  explicit ComplementaryFilter(float alpha = 0.98f) : alpha_(alpha) {}

  void setAlpha(float alpha) { alpha_ = constrain(alpha, 0.0f, 1.0f); }

  void reset(float roll_deg = 0.0f, float pitch_deg = 0.0f) {
    roll_deg_ = roll_deg;
    pitch_deg_ = pitch_deg;
    initialized_ = true;
  }

  void update(float gx_dps, float gy_dps, float gz_dps, float ax, float ay, float az, float dt_s) {
    if (dt_s <= 0.0f) {
      return;
    }

    float roll_acc = atan2f(ay, az) * RAD_TO_DEG;
    float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    float roll_gyro = roll_deg_ + gx_dps * dt_s;
    float pitch_gyro = pitch_deg_ + gy_dps * dt_s;

    if (!initialized_) {
      roll_deg_ = roll_acc;
      pitch_deg_ = pitch_acc;
      initialized_ = true;
      return;
    }

    roll_deg_ = alpha_ * roll_gyro + (1.0f - alpha_) * roll_acc;
    pitch_deg_ = alpha_ * pitch_gyro + (1.0f - alpha_) * pitch_acc;
    yaw_rate_dps_ = gz_dps;
  }

  float roll() const { return roll_deg_; }
  float pitch() const { return pitch_deg_; }
  float yaw_rate() const { return yaw_rate_dps_; }

 private:
  float alpha_ = 0.98f;
  float roll_deg_ = 0.0f;
  float pitch_deg_ = 0.0f;
  float yaw_rate_dps_ = 0.0f;
  bool initialized_ = false;
};

}  // namespace drone
