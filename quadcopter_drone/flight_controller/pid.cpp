/*
 * ============================================================================
 * PID CONTROLLER - IMPLEMENTATION
 * ============================================================================
 */

#include "pid.h"

// ============================================================================
// PIDController Implementation
// ============================================================================

PIDController::PIDController() {
    Kp = 0.0f;
    Ki = 0.0f;
    Kd = 0.0f;
    
    lastError = 0.0f;
    integral = 0.0f;
    derivative = 0.0f;
    lastMeasurement = 0.0f;
    lastOutput = 0.0f;
    
    pTerm = iTerm = dTerm = 0.0f;
    
    outputMin = -PID_MAX_OUTPUT;
    outputMax = PID_MAX_OUTPUT;
    integralMin = -PID_I_MAX;
    integralMax = PID_I_MAX;
    
    derivativeAlpha = 0.2f;  // Default derivative filter
    filteredDerivative = 0.0f;
    
    firstRun = true;
}

void PIDController::init(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    reset();
}

void PIDController::setGains(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void PIDController::setOutputLimits(float min, float max) {
    outputMin = min;
    outputMax = max;
}

void PIDController::setIntegralLimits(float min, float max) {
    integralMin = min;
    integralMax = max;
}

float PIDController::calculate(float setpoint, float measurement, float dt) {
    // Prevent division by zero
    if (dt <= 0.0f) {
        return lastOutput;
    }
    
    // Calculate error
    float error = setpoint - measurement;
    
    // Proportional term
    pTerm = Kp * error;
    
    // Integral term with anti-windup
    integral += error * dt;
    integral = constrain(integral, integralMin / Ki, integralMax / Ki);
    iTerm = Ki * integral;
    
    // Derivative term (on measurement to avoid derivative kick)
    if (firstRun) {
        derivative = 0.0f;
        firstRun = false;
    } else {
        // Derivative on measurement (not error) to avoid kicks on setpoint change
        float rawDerivative = -(measurement - lastMeasurement) / dt;
        
        // Low-pass filter on derivative
        filteredDerivative = derivativeAlpha * rawDerivative + 
                            (1.0f - derivativeAlpha) * filteredDerivative;
        derivative = filteredDerivative;
    }
    dTerm = Kd * derivative;
    
    // Calculate total output
    float output = pTerm + iTerm + dTerm;
    
    // Apply output limits
    output = constrain(output, outputMin, outputMax);
    
    // Anti-windup: if output is saturated, prevent integral from growing
    if (output == outputMax || output == outputMin) {
        // Don't accumulate integral if saturated
        integral -= error * dt;
    }
    
    // Store state for next iteration
    lastError = error;
    lastMeasurement = measurement;
    lastOutput = output;
    
    return output;
}

float PIDController::calculateRate(float setpoint, float rate, float dt) {
    // Prevent division by zero
    if (dt <= 0.0f) {
        return lastOutput;
    }
    
    // Calculate error (setpoint - current rate)
    float error = setpoint - rate;
    
    // Proportional term
    pTerm = Kp * error;
    
    // Integral term with anti-windup
    integral += error * dt;
    integral = constrain(integral, integralMin / (Ki + 0.0001f), integralMax / (Ki + 0.0001f));
    iTerm = Ki * integral;
    
    // Derivative term (on error for rate mode)
    if (firstRun) {
        derivative = 0.0f;
        firstRun = false;
    } else {
        float rawDerivative = (error - lastError) / dt;
        
        // Low-pass filter on derivative
        filteredDerivative = derivativeAlpha * rawDerivative + 
                            (1.0f - derivativeAlpha) * filteredDerivative;
        derivative = filteredDerivative;
    }
    dTerm = Kd * derivative;
    
    // Calculate total output
    float output = pTerm + iTerm + dTerm;
    
    // Apply output limits
    output = constrain(output, outputMin, outputMax);
    
    // Anti-windup
    if (output == outputMax || output == outputMin) {
        integral -= error * dt;
    }
    
    // Store state
    lastError = error;
    lastOutput = output;
    
    return output;
}

void PIDController::reset() {
    lastError = 0.0f;
    integral = 0.0f;
    derivative = 0.0f;
    lastMeasurement = 0.0f;
    lastOutput = 0.0f;
    filteredDerivative = 0.0f;
    pTerm = iTerm = dTerm = 0.0f;
    firstRun = true;
}

// ============================================================================
// FlightPID Implementation
// ============================================================================

FlightPID::FlightPID() {
    rollOutput = 0.0f;
    pitchOutput = 0.0f;
    yawOutput = 0.0f;
    angleMode = true;
}

void FlightPID::init() {
    loadDefaults();
    
    // Set output limits
    rollPID.setOutputLimits(-MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
    pitchPID.setOutputLimits(-MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
    
    rollRatePID.setOutputLimits(-PID_MAX_OUTPUT, PID_MAX_OUTPUT);
    pitchRatePID.setOutputLimits(-PID_MAX_OUTPUT, PID_MAX_OUTPUT);
    yawPID.setOutputLimits(-PID_MAX_OUTPUT, PID_MAX_OUTPUT);
    
    // Set integral limits
    rollPID.setIntegralLimits(-PID_I_MAX, PID_I_MAX);
    pitchPID.setIntegralLimits(-PID_I_MAX, PID_I_MAX);
    rollRatePID.setIntegralLimits(-PID_I_MAX, PID_I_MAX);
    pitchRatePID.setIntegralLimits(-PID_I_MAX, PID_I_MAX);
    yawPID.setIntegralLimits(-PID_I_MAX, PID_I_MAX);
}

void FlightPID::loadDefaults() {
    // Outer loop (angle) - typically lower gains
    rollPID.init(3.0f, 0.01f, 0.0f);
    pitchPID.init(3.0f, 0.01f, 0.0f);
    
    // Inner loop (rate) - higher gains for responsiveness
    rollRatePID.init(PID_ROLL_P, PID_ROLL_I, PID_ROLL_D);
    pitchRatePID.init(PID_PITCH_P, PID_PITCH_I, PID_PITCH_D);
    
    // Yaw (rate only)
    yawPID.init(PID_YAW_P, PID_YAW_I, PID_YAW_D);
}

void FlightPID::calculate(float rollSetpoint, float pitchSetpoint, float yawSetpoint,
                          float rollAngle, float pitchAngle, float yawRate,
                          float rollRate, float pitchRate,
                          float dt) {
    
    if (angleMode) {
        // Cascaded PID: Outer loop outputs desired rate, inner loop outputs motor command
        
        // Outer loop: angle error -> desired rate
        float rollRateSetpoint = rollPID.calculate(rollSetpoint, rollAngle, dt);
        float pitchRateSetpoint = pitchPID.calculate(pitchSetpoint, pitchAngle, dt);
        
        // Inner loop: rate error -> motor command
        rollOutput = rollRatePID.calculateRate(rollRateSetpoint, rollRate, dt);
        pitchOutput = pitchRatePID.calculateRate(pitchRateSetpoint, pitchRate, dt);
    } else {
        // Rate mode: direct rate control (for acrobatics)
        rollOutput = rollRatePID.calculateRate(rollSetpoint, rollRate, dt);
        pitchOutput = pitchRatePID.calculateRate(pitchSetpoint, pitchRate, dt);
    }
    
    // Yaw is always rate-based
    yawOutput = yawPID.calculateRate(yawSetpoint, yawRate, dt);
}

void FlightPID::reset() {
    rollPID.reset();
    pitchPID.reset();
    rollRatePID.reset();
    pitchRatePID.reset();
    yawPID.reset();
    
    rollOutput = 0.0f;
    pitchOutput = 0.0f;
    yawOutput = 0.0f;
}
