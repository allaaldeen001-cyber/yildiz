void calculate_pressure() {
  static byte counter = 0;

  if (counter == 0) {
    MS5611.read();
    smooth.add(MS5611.getPressure());
    counter = 10;
  }
  counter--;

  actual_pressure = smooth.get();
  quadprops.baro_height = actual_pressure;
  KalmanPosVel();
  initKalmanPosVel();
  actual_pressure_2 = quadprops.kalmanvel_z;

  if (!(altitudeHoldEnabled && thrust > 1400 && thrust < 1450)) {
    hold = 0;
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

  if (thrust > 1450) {
    manual_altitude_change = 1;
    pid_altitude_setpoint = actual_pressure;
    manual_throttle = (thrust - 1450) / 3;
  } else if (thrust < 1400) {
    manual_altitude_change = 1;
    pid_altitude_setpoint = actual_pressure;
    manual_throttle = (thrust - 1400) / 5;
  }

  pid_altitude_input = actual_pressure;
  pid_error_temp = pid_altitude_input - pid_altitude_setpoint;

  pid_error_gain_altitude = 0;
  if (abs(pid_error_temp) > 10) {
    pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0f;
    pid_error_gain_altitude = min(pid_error_gain_altitude, 3.0f);
  }

  pid_i_mem_altitude += (pid_i_gain_altitude / 100.0f) * pid_error_temp;
  pid_i_mem_altitude = constrain(pid_i_mem_altitude, -pid_max_altitude, pid_max_altitude);

  pid_output_altitude =
    (100.0f * (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp) +
    pid_i_mem_altitude +
    pid_d_gain_altitude * parachute_throttle;

  pid_output_altitude = constrain(pid_output_altitude, -pid_max_altitude, pid_max_altitude);
}
