#pragma once

#include <Arduino.h>

namespace drone {

class PidController {
 public:
  void configure(float kp, float ki, float kd, float out_min, float out_max, float i_limit) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    out_min_ = out_min;
    out_max_ = out_max;
    i_limit_ = fabs(i_limit);
  }

  void reset() {
    integrator_ = 0.0f;
    prev_error_ = 0.0f;
    first_update_ = true;
  }

  float update(float error, float dt_s) {
    if (dt_s <= 0.0f) {
      return last_output_;
    }

    float proportional = kp_ * error;

    integrator_ += ki_ * error * dt_s;
    integrator_ = constrain(integrator_, -i_limit_, i_limit_);

    float derivative = 0.0f;
    if (first_update_) {
      derivative = 0.0f;
      first_update_ = false;
    } else {
      derivative = kd_ * (error - prev_error_) / dt_s;
    }
    prev_error_ = error;

    last_output_ = constrain(proportional + integrator_ + derivative, out_min_, out_max_);
    return last_output_;
  }

  float last_output() const { return last_output_; }

 private:
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;
  float out_min_ = -500.0f;
  float out_max_ = 500.0f;
  float i_limit_ = 200.0f;

  float integrator_ = 0.0f;
  float prev_error_ = 0.0f;
  float last_output_ = 0.0f;
  bool first_update_ = true;
};

}  // namespace drone
