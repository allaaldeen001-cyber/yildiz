void calculate_pressure() {
    if (counter == 0) {
        MS5611.read();
        smooth.add(MS5611.getPressure());
        counter = 10;
    }
    counter--;

    actual_pressure = smooth.get();
    quadprops.baro_height = actual_pressure;
    KalmanPosVel();
    actual_pressure_2 = quadprops.kalmanvel_z;

    if (!altitudeHoldEnabled) {
        hold = 0;
        manual_altitude_change = 0;
        manual_throttle = 0;
        pid_i_mem_altitude *= 0.95f;
        pid_output_altitude = 0;
        return;
    }

    if (manual_altitude_change == 1) {
        pressure_parachute_previous = actual_pressure * 10.0f;
    }

    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];
    parachute_buffer[parachute_rotating_mem_location] =
        actual_pressure * 10.0f - pressure_parachute_previous;
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];
    pressure_parachute_previous = actual_pressure * 10.0f;
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

    if (thrust > 1500) {
        manual_altitude_change = 1;
        pid_altitude_setpoint = actual_pressure;
        manual_throttle = (thrust - 1500) / 3;
    }

    if (thrust < 1350) {
        manual_altitude_change = 1;
        pid_altitude_setpoint = actual_pressure;
        manual_throttle = (thrust - 1350) / 5;
    }

    pid_altitude_input = actual_pressure;
    pid_error_temp = pid_altitude_input - pid_altitude_setpoint;

    pid_error_gain_altitude = 0;
    if (pid_error_temp > 10 || pid_error_temp < -10) {
        pid_error_gain_altitude = (fabs(pid_error_temp) - 10) / 20.0f;
        if (pid_error_gain_altitude > 3) {
            pid_error_gain_altitude = 3;
        }
    }

    pid_i_mem_altitude += (pid_i_gain_altitude / 100.0f) * pid_error_temp;
    if (pid_i_mem_altitude > pid_max_altitude) {
        pid_i_mem_altitude = pid_max_altitude;
    } else if (pid_i_mem_altitude < -pid_max_altitude) {
        pid_i_mem_altitude = -pid_max_altitude;
    }

    pid_output_altitude = (100.0f * (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp +
                           pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle);

    if (pid_output_altitude > pid_max_altitude) {
        pid_output_altitude = pid_max_altitude;
    } else if (pid_output_altitude < -pid_max_altitude) {
        pid_output_altitude = -pid_max_altitude;
    }
}
