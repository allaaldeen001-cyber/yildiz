/*
 * ============================================================================
 * MPU6050 IMU SENSOR HANDLER
 * ============================================================================
 * 6-axis IMU: 3-axis gyroscope + 3-axis accelerometer
 * Uses complementary filter for attitude estimation
 * ============================================================================
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// MPU6050 Register Addresses
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_PWR_MGMT_2      0x6C
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_WHO_AM_I        0x75

// Gyroscope sensitivity (LSB/deg/s)
#define GYRO_SENSITIVITY_250DPS     131.0f
#define GYRO_SENSITIVITY_500DPS     65.5f
#define GYRO_SENSITIVITY_1000DPS    32.8f
#define GYRO_SENSITIVITY_2000DPS    16.4f

// Accelerometer sensitivity (LSB/g)
#define ACCEL_SENSITIVITY_2G        16384.0f
#define ACCEL_SENSITIVITY_4G        8192.0f
#define ACCEL_SENSITIVITY_8G        4096.0f
#define ACCEL_SENSITIVITY_16G       2048.0f

class MPU6050 {
public:
    // Constructor
    MPU6050();
    
    // Initialization
    bool begin();
    bool testConnection();
    
    // Data reading
    void readRawData();
    void readGyro();
    void readAccel();
    
    // Processed data
    void calculateAngles(float dt);
    void updateAttitude(float dt);
    
    // Calibration
    void calibrateGyro(int samples = CALIBRATION_SAMPLES);
    void calibrateAccel();
    void setGyroOffsets(int16_t x, int16_t y, int16_t z);
    void setAccelOffsets(int16_t x, int16_t y, int16_t z);
    
    // Getters - Raw values
    int16_t getRawAccelX() { return rawAccelX; }
    int16_t getRawAccelY() { return rawAccelY; }
    int16_t getRawAccelZ() { return rawAccelZ; }
    int16_t getRawGyroX()  { return rawGyroX; }
    int16_t getRawGyroY()  { return rawGyroY; }
    int16_t getRawGyroZ()  { return rawGyroZ; }
    
    // Getters - Scaled values (deg/s for gyro, g for accel)
    float getGyroX() { return gyroX; }
    float getGyroY() { return gyroY; }
    float getGyroZ() { return gyroZ; }
    float getAccelX() { return accelX; }
    float getAccelY() { return accelY; }
    float getAccelZ() { return accelZ; }
    
    // Getters - Angles (degrees)
    float getRoll()  { return roll; }
    float getPitch() { return pitch; }
    float getYaw()   { return yaw; }
    
    // Getters - Angular rates (deg/s)
    float getRollRate()  { return gyroX; }
    float getPitchRate() { return gyroY; }
    float getYawRate()   { return gyroZ; }
    
    // Calibration offsets
    int16_t getGyroOffsetX() { return gyroOffsetX; }
    int16_t getGyroOffsetY() { return gyroOffsetY; }
    int16_t getGyroOffsetZ() { return gyroOffsetZ; }
    int16_t getAccelOffsetX() { return accelOffsetX; }
    int16_t getAccelOffsetY() { return accelOffsetY; }
    int16_t getAccelOffsetZ() { return accelOffsetZ; }
    
    // Status
    bool isCalibrated() { return calibrated; }
    int16_t getTemperature();

private:
    // I2C helper functions
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length);
    
    // Raw sensor values
    int16_t rawAccelX, rawAccelY, rawAccelZ;
    int16_t rawGyroX, rawGyroY, rawGyroZ;
    int16_t rawTemp;
    
    // Calibration offsets
    int16_t gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    int16_t accelOffsetX, accelOffsetY, accelOffsetZ;
    
    // Processed values (after offset correction and scaling)
    float gyroX, gyroY, gyroZ;      // deg/s
    float accelX, accelY, accelZ;   // g
    
    // Calculated angles
    float roll, pitch, yaw;
    
    // Complementary filter coefficient
    float alpha;
    
    // Sensitivity scales
    float gyroScale;
    float accelScale;
    
    // Status
    bool calibrated;
};

#endif // MPU6050_H
