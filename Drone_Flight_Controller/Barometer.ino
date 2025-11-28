/*
 * Barometer - MS5611 Altitude Control
 * Handles pressure reading, filtering, and altitude hold PID
 */

void calibrateBarometer() {
  Serial.println("Calibrating barometer ground pressure...");
  digitalWrite(LED, HIGH);
  
  float sum = 0;
  int samples = 50;
  
  for (int i = 0; i < samples; i++) {
    MS5611.read();
    sum += MS5611.getPressure();
    delay(20);
    
    // Progress indicator
    if (i % 10 == 0) {
      tone(BUZZER, 1500, 50);
    }
  }
  
  ground_pressure = sum / samples;
  altitude_hold_pressure = ground_pressure;
  barometer_calibrated = true;
  
  digitalWrite(LED, LOW);
  Serial.print("Ground pressure: ");
  Serial.print(ground_pressure);
  Serial.println(" hPa");
}

void calculate_pressure() {
  // Read barometer every 10 cycles for efficiency
  if (counter == 0) {
    MS5611.read();
    smooth.add(MS5611.getPressure());
    counter = 10;
  }
  counter--;
  
  actual_pressure = smooth.get();
  
  // Apply Kalman filter
  quadprops.baro_height = actual_pressure;
  KalmanPosVel();
  actual_pressure_filtered = quadprops.kalmanvel_z;
  
  // Altitude hold logic
  if (package.switch2 == 0 && thrust > 1400 && thrust < 1450 && barometer_calibrated) {
    
    // Initialize altitude hold setpoint
    if (hold == 0) {
      pid_altitude_setpoint = actual_pressure;
      altitude_hold_pressure = actual_pressure;
      hold = 1;
      Serial.println("Altitude Hold ENABLED");
      tone(BUZZER, 2000, 100);
    }
    
    // Handle manual altitude changes
    manual_altitude_change = 0;
    manual_throttle = 0;
    
    if (thrust > 1450) {
      // Climb
      manual_altitude_change = 1;
      pid_altitude_setpoint = actual_pressure;
      manual_throttle = (thrust - 1450) / 3;
    } else if (thrust < 1400) {
      // Descend
      manual_altitude_change = 1;
      pid_altitude_setpoint = actual_pressure;
      manual_throttle = (thrust - 1400) / 5;
    }
    
    // Update pressure change buffer
    if (manual_altitude_change == 1) {
      pressure_parachute_previous = actual_pressure * 10;
    }
    
    // Rotating buffer for derivative calculation
    static uint8_t parachute_rotating_mem_location = 0;
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];
    pressure_parachute_previous = actual_pressure * 10;
    parachute_rotating_mem_location++;
    if (parachute_rotating_mem_location >= 30) parachute_rotating_mem_location = 0;
    
    // Calculate PID for altitude
    pid_altitude_input = actual_pressure;
    pid_error_temp = pid_altitude_input - pid_altitude_setpoint;
    
    // Adaptive P-gain based on error magnitude
    pid_error_gain_altitude = 0;
    if (abs(pid_error_temp) > 10) {
      pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;
      if (pid_error_gain_altitude > 3) pid_error_gain_altitude = 3;
    }
    
    // I-term accumulation with anti-windup
    pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
    pid_i_mem_altitude = constrain(pid_i_mem_altitude, -pid_max_altitude, pid_max_altitude);
    
    // Complete PID calculation
    pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp 
                         + pid_i_mem_altitude 
                         + pid_d_gain_altitude * (parachute_throttle / 100.0);
    
    pid_output_altitude = constrain(pid_output_altitude, -pid_max_altitude, pid_max_altitude);
    
  } else {
    // Altitude hold disabled
    if (hold == 1) {
      Serial.println("Altitude Hold DISABLED");
    }
    hold = 0;
    pid_output_altitude = 0;
    pid_i_mem_altitude = 0;
    parachute_throttle = 0;
    
    // Clear buffer
    for (int i = 0; i < 35; i++) {
      parachute_buffer[i] = 0;
    }
  }
}
