/**
 * ============================================================================
 * ALTITUDE CONTROLLER
 * ============================================================================
 * 
 * Altitude estimation and control using:
 * - MS5611 barometer
 * - Vertical acceleration from IMU
 * - Complementary filter for sensor fusion
 * - Cascade PID for altitude hold
 * 
 * ============================================================================
 */

#ifndef ALTITUDE_H
#define ALTITUDE_H

#include <Arduino.h>
#include "config.h"
#include "pid.h"

// ============================================================================
// ALTITUDE ESTIMATOR CLASS
// ============================================================================

class AltitudeEstimator {
public:
    // Estimated states
    float altitude;        // Altitude in cm (relative to arm point)
    float velocity;        // Vertical velocity in cm/s
    float acceleration;    // Vertical acceleration in cm/s²
    
    // Raw sensor data
    float baroAltitude;    // Raw barometer altitude
    float accelZ;          // Vertical acceleration from IMU
    
    /**
     * Initialize the altitude estimator
     */
    void begin() {
        altitude = 0.0f;
        velocity = 0.0f;
        acceleration = 0.0f;
        baroAltitude = 0.0f;
        accelZ = 0.0f;
        
        lastUpdateTime = micros();
        referenceAltitude = 0.0f;
    }
    
    /**
     * Update the estimator with new sensor data
     * 
     * @param baroAlt  Barometer altitude in cm
     * @param vertAccel  Vertical acceleration in g (positive = up)
     */
    void update(float baroAlt, float vertAccel) {
        uint32_t now = micros();
        float dt = (now - lastUpdateTime) / 1000000.0f;
        lastUpdateTime = now;
        
        if (dt <= 0.0f || dt > 0.5f) {
            return;
        }
        
        baroAltitude = baroAlt;
        accelZ = vertAccel * 980.665f;  // Convert g to cm/s²
        
        // Complementary filter
        // High-pass filter accelerometer, low-pass filter barometer
        
        // Predict state using accelerometer
        float predictedVelocity = velocity + accelZ * dt;
        float predictedAltitude = altitude + velocity * dt + 0.5f * accelZ * dt * dt;
        
        // Correct with barometer
        float baroError = baroAlt - predictedAltitude;
        
        // Apply complementary filter corrections
        altitude = predictedAltitude + COMP_FILTER_ALPHA * baroError;
        velocity = predictedVelocity + (COMP_FILTER_ALPHA * 0.5f) * baroError / dt;
        
        // Limit velocity estimate
        velocity = constrain(velocity, -500.0f, 500.0f);
        
        acceleration = accelZ;
    }
    
    /**
     * Set current altitude as reference (zero point)
     */
    void setReference() {
        referenceAltitude = baroAltitude;
        altitude = 0.0f;
        velocity = 0.0f;
    }
    
    /**
     * Reset the estimator
     */
    void reset() {
        altitude = 0.0f;
        velocity = 0.0f;
        acceleration = 0.0f;
    }
    
private:
    uint32_t lastUpdateTime;
    float referenceAltitude;
};

// ============================================================================
// ALTITUDE HOLD CONTROLLER CLASS
// ============================================================================

class AltitudeHoldController {
public:
    // State
    bool enabled;
    float targetAltitude;
    float throttleOutput;
    
    // PIDs
    PIDController altitudePID;   // Outer loop: altitude → velocity
    PIDController velocityPID;   // Inner loop: velocity → throttle
    
    /**
     * Initialize the altitude hold controller
     */
    void begin() {
        enabled = false;
        targetAltitude = 0.0f;
        throttleOutput = 0.0f;
        baseThrottle = 1500;
        
        // Configure altitude PID
        altitudePID.setGains(PID_ALT_KP, PID_ALT_KI, PID_ALT_KD);
        altitudePID.setLimits(PID_ALT_IMAX, MAX_CLIMB_RATE);
        
        // Configure velocity PID
        velocityPID.setGains(PID_VVEL_KP, PID_VVEL_KI, PID_VVEL_KD);
        velocityPID.setLimits(PID_VVEL_IMAX, 200.0f);
        
        lastUpdateTime = micros();
    }
    
    /**
     * Enable altitude hold at current altitude
     * 
     * @param currentAltitude  Current altitude in cm
     * @param currentThrottle  Current throttle value for baseline
     */
    void enable(float currentAltitude, uint16_t currentThrottle) {
        if (!enabled) {
            targetAltitude = currentAltitude;
            baseThrottle = currentThrottle;
            
            // Reset PIDs for smooth transition
            altitudePID.reset();
            velocityPID.reset();
            
            throttleOutput = 0.0f;
            enabled = true;
        }
    }
    
    /**
     * Disable altitude hold
     */
    void disable() {
        enabled = false;
        throttleOutput = 0.0f;
    }
    
    /**
     * Update the altitude hold controller
     * 
     * @param currentAltitude  Current altitude in cm
     * @param currentVelocity  Current vertical velocity in cm/s
     * @param throttleStick    Throttle stick input (1000-2000)
     * @return Throttle adjustment (-500 to +500)
     */
    float update(float currentAltitude, float currentVelocity, uint16_t throttleStick) {
        if (!enabled) {
            return 0.0f;
        }
        
        uint32_t now = micros();
        float dt = (now - lastUpdateTime) / 1000000.0f;
        lastUpdateTime = now;
        
        if (dt <= 0.0f || dt > 0.5f) {
            return throttleOutput;
        }
        
        // Check if pilot is commanding altitude change via throttle
        int16_t stickDeviation = (int16_t)throttleStick - 1500;
        
        if (abs(stickDeviation) > ALT_THROTTLE_DEADBAND) {
            // Pilot is commanding climb/descend
            // Map stick to climb rate
            float commandedRate;
            if (stickDeviation > 0) {
                commandedRate = mapFloat(stickDeviation, ALT_THROTTLE_DEADBAND, 
                                        500, 0, MAX_CLIMB_RATE);
            } else {
                commandedRate = mapFloat(stickDeviation, -500, 
                                        -ALT_THROTTLE_DEADBAND, -MAX_DESCENT_RATE, 0);
            }
            
            // Update target altitude based on commanded rate
            targetAltitude += commandedRate * dt;
            
            // Direct velocity control during manual input
            float velError = commandedRate - currentVelocity;
            throttleOutput = velocityPID.computeFromError(velError, dt);
        } else {
            // Hold current target altitude
            // Outer loop: altitude → velocity setpoint
            float velocitySetpoint = altitudePID.compute(targetAltitude, 
                                                          currentAltitude, dt);
            
            // Inner loop: velocity → throttle
            float velError = velocitySetpoint - currentVelocity;
            throttleOutput = velocityPID.computeFromError(velError, dt);
        }
        
        // Limit output
        throttleOutput = constrain(throttleOutput, -200.0f, 200.0f);
        
        return throttleOutput;
    }
    
    /**
     * Get the combined throttle output for motor mixing
     * 
     * @param stickThrottle  Raw throttle stick value
     * @return Combined throttle value
     */
    uint16_t getThrottleOutput(uint16_t stickThrottle) {
        if (!enabled) {
            return stickThrottle;
        }
        
        // Use base throttle as reference, add altitude correction
        float combined = baseThrottle + throttleOutput;
        return constrain((uint16_t)combined, THROTTLE_MIN, THROTTLE_LIMIT);
    }
    
    /**
     * Set target altitude directly
     */
    void setTargetAltitude(float altitude) {
        targetAltitude = altitude;
    }
    
    /**
     * Get current target altitude
     */
    float getTargetAltitude() const {
        return targetAltitude;
    }
    
private:
    uint32_t lastUpdateTime;
    uint16_t baseThrottle;
    
    /**
     * Map float value from one range to another
     */
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

#endif // ALTITUDE_H
