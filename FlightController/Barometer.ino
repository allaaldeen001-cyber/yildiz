/*
 * Barometer.ino
 * =============
 * MS5611 barometer reading and altitude hold PID control
 * This file is part of the FlightController project
 */

// Read barometer and calculate pressure (called from main loop)
void calculatePressure() {
    // Read barometer every 10 loops (~14Hz at 140Hz main loop)
    if (baroCounter == 0) {
        ms5611.read();
        pressureSmooth.add(ms5611.getPressure());
        baroCounter = 10;
    }
    baroCounter--;
    
    // Get smoothed pressure
    actualPressure = pressureSmooth.get();
    
    // Update Kalman filter
    quadProps.baroHeight = actualPressure;
    KalmanPosVel();
}

// Calculate altitude hold PID (called when altitude hold is active)
void calculateAltitudeHold() {
    // Only activate altitude hold when thrust is in the "hover zone" (1400-1450)
    if (currentThrust > 1400 && currentThrust < 1450) {
        
        // Initialize setpoint on first entry
        if (!holdingAltitude) {
            altitudeSetpoint = actualPressure;
            holdingAltitude = true;
            pid_i_mem_altitude = 0;
            
            // Reset parachute detection
            pressureParachutePrev = actualPressure * 10;
            for (int i = 0; i < 35; i++) {
                parachuteBuffer[i] = 0;
            }
            parachuteThrottle = 0;
            parachuteMemLocation = 0;
            
            Serial.println(F("Altitude hold engaged"));
        }
        
        // Parachute detection (rate of descent monitoring)
        updateParachuteDetection();
        
        // Manual altitude adjustment
        manualAltitudeChange = 0;
        manualThrottle = 0;
        
        // Allow manual override above/below the hold zone
        if (currentThrust > 1450) {
            manualAltitudeChange = 1;
            altitudeSetpoint = actualPressure;
            manualThrottle = (currentThrust - 1450) / 3;
        }
        
        if (currentThrust < 1400) {
            manualAltitudeChange = 1;
            altitudeSetpoint = actualPressure;
            manualThrottle = (currentThrust - 1400) / 5;
        }
        
        // Calculate altitude PID
        float altitudeError = actualPressure - altitudeSetpoint;
        
        // Adaptive P-gain based on error magnitude
        float errorGain = 0;
        if (abs(altitudeError) > 10) {
            errorGain = (abs(altitudeError) - 10) / 20.0;
            errorGain = constrain(errorGain, 0, 3);
        }
        
        // Integral term
        pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * altitudeError;
        pid_i_mem_altitude = constrain(pid_i_mem_altitude, -pid_max_altitude, pid_max_altitude);
        
        // Calculate PID output
        // Note: Using parachuteThrottle as derivative term (measures rate of change)
        pid_output_altitude = (100 * (pid_p_gain_altitude + errorGain) * altitudeError + 
                               pid_i_mem_altitude + 
                               pid_d_gain_altitude * parachuteThrottle);
        
        // Constrain output
        pid_output_altitude = constrain(pid_output_altitude, -pid_max_altitude, pid_max_altitude);
        
    } else {
        // Outside hold zone - reset
        holdingAltitude = false;
        pid_output_altitude = 0;
        manualThrottle = 0;
    }
}

// Update parachute detection (monitors rate of pressure change)
void updateParachuteDetection() {
    // Reset on manual altitude change
    if (manualAltitudeChange == 1) {
        pressureParachutePrev = actualPressure * 10;
    }
    
    // Circular buffer for rate of change
    parachuteThrottle -= parachuteBuffer[parachuteMemLocation];
    parachuteBuffer[parachuteMemLocation] = actualPressure * 10 - pressureParachutePrev;
    parachuteThrottle += parachuteBuffer[parachuteMemLocation];
    
    pressureParachutePrev = actualPressure * 10;
    
    parachuteMemLocation++;
    if (parachuteMemLocation >= 30) {
        parachuteMemLocation = 0;
    }
}

// Get altitude in meters from pressure (relative to ground)
float getAltitudeMeters() {
    // Using barometric formula: h = 44330 * (1 - (P/P0)^0.1903)
    // Simplified for small altitude differences
    return 44330.0 * (1.0 - pow(actualPressure / groundPressure, 0.1903));
}

// Get vertical velocity from Kalman filter
float getVerticalVelocity() {
    return quadProps.kalmanVelZ;
}
