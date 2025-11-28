/**
 * ============================================================================
 * AHRS - ATTITUDE AND HEADING REFERENCE SYSTEM
 * ============================================================================
 * 
 * Mahony AHRS filter implementation for attitude estimation
 * Fuses accelerometer and gyroscope data using quaternions
 * 
 * Based on Sebastian Madgwick's open-source filter algorithm
 * Optimized for Arduino Nano (minimal floating-point operations)
 * 
 * ============================================================================
 */

#ifndef AHRS_H
#define AHRS_H

#include <Arduino.h>
#include <math.h>
#include "config.h"

// ============================================================================
// AHRS CLASS - MAHONY FILTER IMPLEMENTATION
// ============================================================================

class AHRS {
public:
    // Euler angles (output)
    float roll;       // Roll angle in degrees
    float pitch;      // Pitch angle in degrees
    float yaw;        // Yaw angle in degrees
    
    // Angular rates (from gyro)
    float rollRate;   // Roll rate in °/s
    float pitchRate;  // Pitch rate in °/s
    float yawRate;    // Yaw rate in °/s
    
    /**
     * Initialize the AHRS filter
     */
    void begin() {
        // Initialize quaternion to identity (level orientation)
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        
        // Initialize integral error
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
        
        // Initialize outputs
        roll = 0.0f;
        pitch = 0.0f;
        yaw = 0.0f;
        rollRate = 0.0f;
        pitchRate = 0.0f;
        yawRate = 0.0f;
        
        // Timing
        lastUpdateTime = micros();
    }
    
    /**
     * Update the AHRS with new sensor data
     * 
     * @param gx Gyroscope X (roll rate) in °/s
     * @param gy Gyroscope Y (pitch rate) in °/s
     * @param gz Gyroscope Z (yaw rate) in °/s
     * @param ax Accelerometer X in g
     * @param ay Accelerometer Y in g
     * @param az Accelerometer Z in g
     */
    void update(float gx, float gy, float gz, float ax, float ay, float az) {
        // Calculate time delta
        uint32_t now = micros();
        float dt = (now - lastUpdateTime) / 1000000.0f;
        lastUpdateTime = now;
        
        // Clamp dt to prevent instability
        if (dt <= 0.0f || dt > 0.1f) {
            dt = 0.0025f;  // Default to expected loop time
        }
        
        // Store rates for external use
        rollRate = gx;
        pitchRate = gy;
        yawRate = gz;
        
        // Convert gyro rates to radians/s
        float gxRad = gx * DEG_TO_RAD;
        float gyRad = gy * DEG_TO_RAD;
        float gzRad = gz * DEG_TO_RAD;
        
        // Mahony AHRS update
        updateMahony(gxRad, gyRad, gzRad, ax, ay, az, dt);
        
        // Convert quaternion to Euler angles
        computeEulerAngles();
    }
    
    /**
     * Reset the filter to level orientation
     */
    void reset() {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
        
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
        
        roll = 0.0f;
        pitch = 0.0f;
        yaw = 0.0f;
    }
    
    /**
     * Initialize attitude from accelerometer
     * Use when stationary at startup
     */
    void initFromAccel(float ax, float ay, float az) {
        // Calculate initial roll and pitch from accelerometer
        float initialRoll = atan2(ay, az);
        float initialPitch = atan2(-ax, sqrt(ay * ay + az * az));
        
        // Convert to quaternion (yaw = 0)
        float cr = cos(initialRoll / 2.0f);
        float sr = sin(initialRoll / 2.0f);
        float cp = cos(initialPitch / 2.0f);
        float sp = sin(initialPitch / 2.0f);
        
        q0 = cr * cp;
        q1 = sr * cp;
        q2 = cr * sp;
        q3 = -sr * sp;
        
        // Normalize
        float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
        
        computeEulerAngles();
    }
    
    /**
     * Get rotation matrix element for coordinate transformation
     */
    float getRotationMatrix(int row, int col) {
        // Rotation matrix from quaternion
        float R[3][3];
        
        R[0][0] = 1.0f - 2.0f * (q2*q2 + q3*q3);
        R[0][1] = 2.0f * (q1*q2 - q0*q3);
        R[0][2] = 2.0f * (q1*q3 + q0*q2);
        
        R[1][0] = 2.0f * (q1*q2 + q0*q3);
        R[1][1] = 1.0f - 2.0f * (q1*q1 + q3*q3);
        R[1][2] = 2.0f * (q2*q3 - q0*q1);
        
        R[2][0] = 2.0f * (q1*q3 - q0*q2);
        R[2][1] = 2.0f * (q2*q3 + q0*q1);
        R[2][2] = 1.0f - 2.0f * (q1*q1 + q2*q2);
        
        return R[row][col];
    }
    
    /**
     * Get vertical acceleration (world frame Z, corrected for gravity)
     * Returns acceleration in g (positive = upward)
     */
    float getVerticalAccel(float ax, float ay, float az) {
        // Transform accelerometer to world frame and subtract gravity
        float worldZ = ax * getRotationMatrix(2, 0) + 
                       ay * getRotationMatrix(2, 1) + 
                       az * getRotationMatrix(2, 2);
        
        return worldZ - 1.0f;  // Subtract gravity (1g)
    }
    
private:
    // Quaternion elements
    float q0, q1, q2, q3;
    
    // Integral error for Mahony
    float integralFBx, integralFBy, integralFBz;
    
    // Timing
    uint32_t lastUpdateTime;
    
    // Conversion constant
    static constexpr float DEG_TO_RAD = 0.017453292519943295f;
    static constexpr float RAD_TO_DEG = 57.29577951308232f;
    
    /**
     * Mahony AHRS update algorithm
     */
    void updateMahony(float gx, float gy, float gz, 
                      float ax, float ay, float az, float dt) {
        float recipNorm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float qa, qb, qc;
        
        // Compute feedback only if accelerometer measurement valid
        float accelMag = ax * ax + ay * ay + az * az;
        if (accelMag > 0.01f && accelMag < 4.0f) {  // Between 0.1g and 2g
            // Normalize accelerometer measurement
            recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;
            
            // Estimated direction of gravity (from quaternion)
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5f + q3 * q3;
            
            // Error is cross product between estimated and measured direction of gravity
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);
            
            // Compute and apply integral feedback if enabled
            if (MAHONY_KI > 0.0f) {
                integralFBx += MAHONY_KI * halfex * dt;
                integralFBy += MAHONY_KI * halfey * dt;
                integralFBz += MAHONY_KI * halfez * dt;
                
                // Limit integral windup
                float integralLimit = 0.5f;
                integralFBx = constrain(integralFBx, -integralLimit, integralLimit);
                integralFBy = constrain(integralFBy, -integralLimit, integralLimit);
                integralFBz = constrain(integralFBz, -integralLimit, integralLimit);
                
                gx += integralFBx;
                gy += integralFBy;
                gz += integralFBz;
            }
            
            // Apply proportional feedback
            gx += MAHONY_KP * halfex;
            gy += MAHONY_KP * halfey;
            gz += MAHONY_KP * halfez;
        }
        
        // Integrate rate of change of quaternion
        gx *= 0.5f * dt;
        gy *= 0.5f * dt;
        gz *= 0.5f * dt;
        
        qa = q0;
        qb = q1;
        qc = q2;
        
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);
        
        // Normalize quaternion
        recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }
    
    /**
     * Convert quaternion to Euler angles
     */
    void computeEulerAngles() {
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
        float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
        roll = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
        
        // Pitch (y-axis rotation)
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (fabs(sinp) >= 1.0f) {
            pitch = copysign(90.0f, sinp);  // Use 90 degrees if out of range
        } else {
            pitch = asin(sinp) * RAD_TO_DEG;
        }
        
        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
        float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
        yaw = atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
    }
};

#endif // AHRS_H
