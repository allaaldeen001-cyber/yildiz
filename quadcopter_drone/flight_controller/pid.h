/*
 * ============================================================================
 * PID CONTROLLER
 * ============================================================================
 * Proportional-Integral-Derivative controller for flight stabilization
 * Includes anti-windup, derivative filtering, and output limiting
 * ============================================================================
 */

#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "config.h"

class PIDController {
public:
    // Constructor
    PIDController();
    
    // Initialize with gains
    void init(float kp, float ki, float kd);
    
    // Set gains individually
    void setP(float kp) { Kp = kp; }
    void setI(float ki) { Ki = ki; }
    void setD(float kd) { Kd = kd; }
    void setGains(float kp, float ki, float kd);
    
    // Set output limits
    void setOutputLimits(float min, float max);
    void setIntegralLimits(float min, float max);
    
    // Set derivative filter coefficient (0-1, lower = more filtering)
    void setDerivativeFilter(float alpha) { derivativeAlpha = alpha; }
    
    // Calculate PID output
    // setpoint: desired value
    // measurement: current measured value
    // dt: time delta in seconds
    float calculate(float setpoint, float measurement, float dt);
    
    // Calculate using rate (for rate-mode flight)
    // setpoint: desired rate
    // rate: current measured rate (gyro reading)
    float calculateRate(float setpoint, float rate, float dt);
    
    // Reset integral and derivative terms
    void reset();
    
    // Getters
    float getKp() { return Kp; }
    float getKi() { return Ki; }
    float getKd() { return Kd; }
    float getError() { return lastError; }
    float getIntegral() { return integral; }
    float getDerivative() { return derivative; }
    float getOutput() { return lastOutput; }
    
    // Get individual terms (for debugging/tuning)
    float getPTerm() { return pTerm; }
    float getITerm() { return iTerm; }
    float getDTerm() { return dTerm; }

private:
    // PID gains
    float Kp, Ki, Kd;
    
    // State variables
    float lastError;
    float integral;
    float derivative;
    float lastMeasurement;
    float lastOutput;
    
    // Individual terms (for debugging)
    float pTerm, iTerm, dTerm;
    
    // Output limits
    float outputMin, outputMax;
    float integralMin, integralMax;
    
    // Derivative filter
    float derivativeAlpha;
    float filteredDerivative;
    
    // First run flag
    bool firstRun;
};

// ============================================================================
// Multi-axis PID controller for flight control
// ============================================================================

class FlightPID {
public:
    FlightPID();
    
    // Initialize all axes
    void init();
    
    // Load default gains
    void loadDefaults();
    
    // Calculate all axes
    void calculate(float rollSetpoint, float pitchSetpoint, float yawSetpoint,
                   float rollAngle, float pitchAngle, float yawRate,
                   float rollRate, float pitchRate,
                   float dt);
    
    // Get outputs
    float getRollOutput()  { return rollOutput; }
    float getPitchOutput() { return pitchOutput; }
    float getYawOutput()   { return yawOutput; }
    
    // Set mode (angle vs rate)
    void setAngleMode(bool enabled) { angleMode = enabled; }
    
    // Access individual controllers
    PIDController& getRollPID()  { return rollPID; }
    PIDController& getPitchPID() { return pitchPID; }
    PIDController& getYawPID()   { return yawPID; }
    PIDController& getRollRatePID()  { return rollRatePID; }
    PIDController& getPitchRatePID() { return pitchRatePID; }
    
    // Reset all controllers
    void reset();

private:
    // Angle (outer loop) controllers
    PIDController rollPID;
    PIDController pitchPID;
    
    // Rate (inner loop) controllers
    PIDController rollRatePID;
    PIDController pitchRatePID;
    PIDController yawPID;  // Yaw is rate-only
    
    // Outputs
    float rollOutput;
    float pitchOutput;
    float yawOutput;
    
    // Mode
    bool angleMode;
};

#endif // PID_H
