/**
 * ============================================================================
 * PID CONTROLLER
 * ============================================================================
 * 
 * Professional PID controller implementation with:
 * - Anti-windup protection
 * - Derivative filtering
 * - Output limiting
 * - Setpoint weighting
 * 
 * ============================================================================
 */

#ifndef PID_H
#define PID_H

#include <Arduino.h>

// ============================================================================
// PID CONTROLLER CLASS
// ============================================================================

class PIDController {
public:
    // PID gains
    float Kp;
    float Ki;
    float Kd;
    
    // Limits
    float iMax;       // Integral windup limit
    float outMax;     // Output limit
    
    // Setpoint weighting (0-1)
    float setpointWeight;
    
    /**
     * Constructor with default values
     */
    PIDController() {
        Kp = 1.0f;
        Ki = 0.0f;
        Kd = 0.0f;
        iMax = 100.0f;
        outMax = 400.0f;
        setpointWeight = 1.0f;
        
        reset();
    }
    
    /**
     * Constructor with gains
     */
    PIDController(float kp, float ki, float kd, float integralMax, float outputMax) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        iMax = integralMax;
        outMax = outputMax;
        setpointWeight = 1.0f;
        
        reset();
    }
    
    /**
     * Set PID gains
     */
    void setGains(float kp, float ki, float kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }
    
    /**
     * Set limits
     */
    void setLimits(float integralMax, float outputMax) {
        iMax = integralMax;
        outMax = outputMax;
    }
    
    /**
     * Reset the controller state
     */
    void reset() {
        integral = 0.0f;
        prevError = 0.0f;
        prevMeasurement = 0.0f;
        prevDerivative = 0.0f;
        output = 0.0f;
    }
    
    /**
     * Compute PID output
     * 
     * @param setpoint  Desired value
     * @param measurement  Current value
     * @param dt  Time delta in seconds
     * @return PID output
     */
    float compute(float setpoint, float measurement, float dt) {
        if (dt <= 0.0f || dt > 1.0f) {
            return output;  // Return previous output for invalid dt
        }
        
        // Calculate error
        float error = setpoint - measurement;
        
        // Proportional term
        float pTerm = Kp * error;
        
        // Integral term with anti-windup
        integral += Ki * error * dt;
        integral = constrain(integral, -iMax, iMax);
        float iTerm = integral;
        
        // Derivative term (on measurement to avoid derivative kick)
        // Using low-pass filter on derivative
        float derivative = -(measurement - prevMeasurement) / dt;
        derivative = 0.7f * prevDerivative + 0.3f * derivative;  // Low-pass filter
        float dTerm = Kd * derivative;
        prevMeasurement = measurement;
        prevDerivative = derivative;
        
        // Calculate total output
        output = pTerm + iTerm + dTerm;
        
        // Output limiting with back-calculation anti-windup
        if (output > outMax) {
            integral -= (output - outMax) * 0.5f;
            output = outMax;
        } else if (output < -outMax) {
            integral -= (output + outMax) * 0.5f;
            output = -outMax;
        }
        
        prevError = error;
        
        return output;
    }
    
    /**
     * Compute PID output using error directly
     * (for rate control where setpoint is the rate)
     * 
     * @param error  Error value (setpoint - measurement)
     * @param dt  Time delta in seconds
     * @return PID output
     */
    float computeFromError(float error, float dt) {
        if (dt <= 0.0f || dt > 1.0f) {
            return output;
        }
        
        // Proportional term
        float pTerm = Kp * error;
        
        // Integral term with anti-windup
        integral += Ki * error * dt;
        integral = constrain(integral, -iMax, iMax);
        float iTerm = integral;
        
        // Derivative term with filtering
        float derivative = (error - prevError) / dt;
        derivative = 0.7f * prevDerivative + 0.3f * derivative;
        float dTerm = Kd * derivative;
        prevError = error;
        prevDerivative = derivative;
        
        // Calculate total output
        output = pTerm + iTerm + dTerm;
        
        // Output limiting
        output = constrain(output, -outMax, outMax);
        
        return output;
    }
    
    /**
     * Get current integral value
     */
    float getIntegral() const {
        return integral;
    }
    
    /**
     * Get last output
     */
    float getOutput() const {
        return output;
    }
    
    /**
     * Reduce integral by a factor (for smooth transitions)
     */
    void decayIntegral(float factor) {
        integral *= factor;
    }
    
private:
    float integral;
    float prevError;
    float prevMeasurement;
    float prevDerivative;
    float output;
};

// ============================================================================
// CASCADE PID CONTROLLER
// ============================================================================

/**
 * Cascade controller combining outer (angle) and inner (rate) loops
 */
class CascadePID {
public:
    PIDController outerLoop;  // Angle controller
    PIDController innerLoop;  // Rate controller
    
    /**
     * Constructor
     */
    CascadePID() {
        // Will be configured via setGains()
    }
    
    /**
     * Configure the cascade controller
     */
    void configure(float outerKp, float outerKi, float outerKd,
                   float innerKp, float innerKi, float innerKd,
                   float angleMax, float rateMax) {
        outerLoop.setGains(outerKp, outerKi, outerKd);
        outerLoop.setLimits(ANGLE_PID_IMAX, rateMax);
        
        innerLoop.setGains(innerKp, innerKi, innerKd);
        innerLoop.setLimits(RATE_PID_IMAX, RATE_PID_OUTMAX);
    }
    
    /**
     * Compute cascade PID output
     * 
     * @param angleSetpoint  Desired angle
     * @param angleMeasurement  Current angle
     * @param rateMeasurement  Current rate
     * @param dtOuter  Time delta for outer loop
     * @param dtInner  Time delta for inner loop
     * @return Control output
     */
    float compute(float angleSetpoint, float angleMeasurement,
                  float rateMeasurement, float dtOuter, float dtInner) {
        // Outer loop: angle → rate setpoint
        float rateSetpoint = outerLoop.compute(angleSetpoint, angleMeasurement, dtOuter);
        
        // Inner loop: rate → control output
        float rateError = rateSetpoint - rateMeasurement;
        return innerLoop.computeFromError(rateError, dtInner);
    }
    
    /**
     * Compute inner loop only (for rate mode)
     */
    float computeRateOnly(float rateSetpoint, float rateMeasurement, float dt) {
        float rateError = rateSetpoint - rateMeasurement;
        return innerLoop.computeFromError(rateError, dt);
    }
    
    /**
     * Reset both loops
     */
    void reset() {
        outerLoop.reset();
        innerLoop.reset();
    }
};

#endif // PID_H
