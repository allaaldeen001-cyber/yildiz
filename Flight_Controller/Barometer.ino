/*
 * BAROMETER - Altitude Hold Functions
 * MS5611 Barometric Pressure Sensor Integration
 */

//===================== CALCULATE PRESSURE =====================
void calculate_pressure() {
  // Read barometer every 10 cycles to reduce processing load
  if (pressure_counter == 0) {
    MS5611.read();
    smooth.add(MS5611.getPressure());
    pressure_counter = 10;
  }
  pressure_counter--;
  
  actual_pressure = smooth.get();
  quadprops.baro_height = actual_pressure;
  
  // Apply Kalman filter for smooth altitude estimation
  KalmanPosVel();
  actual_pressure_smoothed = quadprops.kalmanvel_z;
  
  // Altitude Hold Mode (Switch 2 = 0)
  if (package.switch2 == 0 && thrust > 1400 && thrust < 1450 && armed) {
    
    // Initialize pressure tracking on first entry to altitude hold
    if (manual_altitude_change == 1) {
      pressure_parachute_previous = actual_pressure * 10.0;
    }
    
    // Calculate pressure change rate using circular buffer
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10.0 - pressure_parachute_previous;
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];
    pressure_parachute_previous = actual_pressure * 10.0;
    
    parachute_rotating_mem_location++;
    if (parachute_rotating_mem_location >= 30) {
      parachute_rotating_mem_location = 0;
    }
    
    // Set altitude setpoint on first entry
    if (hold == 0) {
      pid_altitude_setpoint = actual_pressure;
      hold = 1;
      Serial.print("Altitude Hold ENGAGED at pressure: ");
      Serial.println(pid_altitude_setpoint);
    }
    
    // Check for manual altitude adjustment via throttle stick
    manual_altitude_change = 0;
    manual_throttle = 0;
    
    // Throttle up: increase altitude
    if (thrust > 1450) {
      manual_altitude_change = 1;
      pid_altitude_setpoint = actual_pressure;
      manual_throttle = (thrust - 1450) / 3;
    }
    
    // Throttle down: decrease altitude
    if (thrust < 1400) {
      manual_altitude_change = 1;
      pid_altitude_setpoint = actual_pressure;
      manual_throttle = (thrust - 1400) / 5;
    }
    
    // Calculate altitude PID
    pid_altitude_input = actual_pressure;
    pid_error_temp = pid_altitude_input - pid_altitude_setpoint;
    
    // Adaptive P-gain based on error magnitude
    pid_error_gain_altitude = 0;
    if (abs(pid_error_temp) > 10) {
      pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;
      if (pid_error_gain_altitude > 3.0) {
        pid_error_gain_altitude = 3.0;
      }
    }
    
    // Integral term with anti-windup
    pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
    pid_i_mem_altitude = constrain(pid_i_mem_altitude, -pid_max_altitude, pid_max_altitude);
    
    // Complete PID output calculation
    pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp 
                        + pid_i_mem_altitude 
                        + pid_d_gain_altitude * parachute_throttle;
    
    pid_output_altitude = constrain(pid_output_altitude, -pid_max_altitude, pid_max_altitude);
    
  } else {
    // Altitude hold not active - reset state
    hold = 0;
    pid_output_altitude = 0;
    pid_i_mem_altitude = 0;
    manual_altitude_change = 0;
  }
}
