#include <math.h>

void calculate_pressure() {
  static uint8_t sampleDivider = 0;
  if (sampleDivider == 0) {
    MS5611.read();
    smooth.add(MS5611.getPressure());
    sampleDivider = 10;
  }
  sampleDivider--;

  actual_pressure = smooth.get();
  quadprops.baro_height = actual_pressure;
  KalmanPosVel();
  initKalmanPosVel();
  actual_pressure_2 = quadprops.kalmanvel_z;

  const bool altitudeHoldRequested = (switch2 == 0);
  const bool throttleGate = (thrust > 1350 && thrust < 1650);

  if (!(altitudeHoldRequested && throttleGate && armed)) {
    hold = 0;
    manual_altitude_change = 0;
    manual_throttle = 0;
    pid_output_altitude = 0;
    return;
  }

  if (manual_altitude_change == 1) {
    pressure_parachute_previous = actual_pressure * 10;
  }

  parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];
  parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;
  parachute_throttle += parachute_buffer[parachute_rotating_mem_location];
  pressure_parachute_previous = actual_pressure * 10;
  parachute_rotating_mem_location++;
  if (parachute_rotating_mem_location == 30) {
    parachute_rotating_mem_location = 0;
  }

  if (hold == 0) {
    pid_altitude_setpoint = actual_pressure;
    hold = 1;
  }

  manual_altitude_change = 0;
  manual_throttle = 0;

  if (thrust >= 1650) {
    manual_altitude_change = 1;
    pid_altitude_setpoint = actual_pressure;
    manual_throttle = (thrust - 1650) / 3;
  } else if (thrust <= 1350) {
    manual_altitude_change = 1;
    pid_altitude_setpoint = actual_pressure;
    manual_throttle = (thrust - 1350) / 5;
  }

  pid_altitude_input = actual_pressure;
  pid_error_temp = pid_altitude_input - pid_altitude_setpoint;

  pid_error_gain_altitude = 0;
  if (fabs(pid_error_temp) > 10) {
    pid_error_gain_altitude = (fabs(pid_error_temp) - 10) / 20.0f;
    if (pid_error_gain_altitude > 3) {
      pid_error_gain_altitude = 3;
    }
  }

  pid_i_mem_altitude += (pid_i_gain_altitude / 100.0f) * pid_error_temp;
  pid_i_mem_altitude = constrain(pid_i_mem_altitude, -pid_max_altitude, pid_max_altitude);

  pid_output_altitude = (100 * (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle);
  pid_output_altitude = constrain(pid_output_altitude, -pid_max_altitude, pid_max_altitude);
}
