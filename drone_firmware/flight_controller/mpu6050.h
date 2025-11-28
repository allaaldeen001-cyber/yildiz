/**
 * ============================================================================
 * MPU6050 IMU DRIVER
 * ============================================================================
 * 
 * Optimized driver for MPU6050 6-DOF IMU
 * Provides raw and filtered gyroscope/accelerometer data
 * 
 * ============================================================================
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// ============================================================================
// MPU6050 REGISTER DEFINITIONS
// ============================================================================

#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_INT_PIN_CFG     0x37
#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_REG_INT_STATUS      0x3A
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_USER_CTRL       0x6A
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_PWR_MGMT_2      0x6C
#define MPU6050_REG_WHO_AM_I        0x75

// Configuration values
#define MPU6050_CLOCK_PLL_XGYRO     0x01
#define MPU6050_DLPF_BW_42          0x03  // 42Hz DLPF
#define MPU6050_GYRO_FS_500         0x08  // ±500°/s
#define MPU6050_ACCEL_FS_8          0x10  // ±8g

// Scale factors
#define GYRO_SCALE_500DPS           65.5f    // LSB/(°/s) for ±500°/s
#define ACCEL_SCALE_8G              4096.0f  // LSB/g for ±8g

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct MPU6050_RawData {
    int16_t ax, ay, az;   // Raw accelerometer
    int16_t gx, gy, gz;   // Raw gyroscope
    int16_t temp;         // Raw temperature
};

struct MPU6050_Data {
    // Calibrated and filtered values
    float gyroX, gyroY, gyroZ;     // Gyroscope in °/s
    float accelX, accelY, accelZ;  // Accelerometer in g
    float temperature;              // Temperature in °C
    
    // Raw values for calibration
    int16_t rawGyroX, rawGyroY, rawGyroZ;
    int16_t rawAccelX, rawAccelY, rawAccelZ;
};

struct MPU6050_Calibration {
    int16_t gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    int16_t accelOffsetX, accelOffsetY, accelOffsetZ;
    bool calibrated;
};

// ============================================================================
// MPU6050 DRIVER CLASS
// ============================================================================

class MPU6050Driver {
public:
    MPU6050_Data data;
    MPU6050_Calibration calibration;
    
    /**
     * Initialize the MPU6050
     * @return true if successful
     */
    bool begin() {
        Wire.begin();
        Wire.setClock(400000);  // 400kHz I2C
        
        // Check WHO_AM_I register
        if (readRegister(MPU6050_REG_WHO_AM_I) != 0x68) {
            return false;
        }
        
        // Wake up device (clear sleep bit)
        writeRegister(MPU6050_REG_PWR_MGMT_1, MPU6050_CLOCK_PLL_XGYRO);
        delay(100);
        
        // Configure sample rate divider (1kHz / (1 + 1) = 500Hz)
        writeRegister(MPU6050_REG_SMPLRT_DIV, 0x01);
        
        // Configure DLPF (42Hz bandwidth)
        writeRegister(MPU6050_REG_CONFIG, MPU6050_DLPF_BW_42);
        
        // Configure gyroscope (±500°/s)
        writeRegister(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_500);
        
        // Configure accelerometer (±8g)
        writeRegister(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_8);
        
        // Enable data ready interrupt
        writeRegister(MPU6050_REG_INT_PIN_CFG, 0x10);  // INT pin config
        writeRegister(MPU6050_REG_INT_ENABLE, 0x01);   // Data ready interrupt
        
        // Initialize calibration with defaults
        calibration.gyroOffsetX = GYRO_OFFSET_X;
        calibration.gyroOffsetY = GYRO_OFFSET_Y;
        calibration.gyroOffsetZ = GYRO_OFFSET_Z;
        calibration.accelOffsetX = ACCEL_OFFSET_X;
        calibration.accelOffsetY = ACCEL_OFFSET_Y;
        calibration.accelOffsetZ = ACCEL_OFFSET_Z;
        calibration.calibrated = false;
        
        // Initialize filter states
        filteredGyroX = filteredGyroY = filteredGyroZ = 0;
        filteredAccelX = filteredAccelY = filteredAccelZ = 0;
        
        delay(50);
        return true;
    }
    
    /**
     * Read all sensor data with filtering
     */
    void update() {
        MPU6050_RawData raw;
        
        // Burst read all sensor registers (14 bytes)
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(MPU6050_REG_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)14);
        
        // Read accelerometer (big-endian)
        raw.ax = (Wire.read() << 8) | Wire.read();
        raw.ay = (Wire.read() << 8) | Wire.read();
        raw.az = (Wire.read() << 8) | Wire.read();
        
        // Read temperature
        raw.temp = (Wire.read() << 8) | Wire.read();
        
        // Read gyroscope
        raw.gx = (Wire.read() << 8) | Wire.read();
        raw.gy = (Wire.read() << 8) | Wire.read();
        raw.gz = (Wire.read() << 8) | Wire.read();
        
        // Store raw values
        data.rawGyroX = raw.gx;
        data.rawGyroY = raw.gy;
        data.rawGyroZ = raw.gz;
        data.rawAccelX = raw.ax;
        data.rawAccelY = raw.ay;
        data.rawAccelZ = raw.az;
        
        // Apply calibration offsets and convert to physical units
        float gx = (float)(raw.gx - calibration.gyroOffsetX) / GYRO_SCALE_500DPS;
        float gy = (float)(raw.gy - calibration.gyroOffsetY) / GYRO_SCALE_500DPS;
        float gz = (float)(raw.gz - calibration.gyroOffsetZ) / GYRO_SCALE_500DPS;
        
        float ax = (float)(raw.ax - calibration.accelOffsetX) / ACCEL_SCALE_8G;
        float ay = (float)(raw.ay - calibration.accelOffsetY) / ACCEL_SCALE_8G;
        float az = (float)(raw.az - calibration.accelOffsetZ) / ACCEL_SCALE_8G;
        
        // Apply low-pass filter to gyro
        filteredGyroX = filteredGyroX * GYRO_LPF_ALPHA + gx * (1.0f - GYRO_LPF_ALPHA);
        filteredGyroY = filteredGyroY * GYRO_LPF_ALPHA + gy * (1.0f - GYRO_LPF_ALPHA);
        filteredGyroZ = filteredGyroZ * GYRO_LPF_ALPHA + gz * (1.0f - GYRO_LPF_ALPHA);
        
        // Apply low-pass filter to accelerometer
        filteredAccelX = filteredAccelX * ACCEL_LPF_ALPHA + ax * (1.0f - ACCEL_LPF_ALPHA);
        filteredAccelY = filteredAccelY * ACCEL_LPF_ALPHA + ay * (1.0f - ACCEL_LPF_ALPHA);
        filteredAccelZ = filteredAccelZ * ACCEL_LPF_ALPHA + az * (1.0f - ACCEL_LPF_ALPHA);
        
        // Store filtered values
        data.gyroX = filteredGyroX;
        data.gyroY = filteredGyroY;
        data.gyroZ = filteredGyroZ;
        data.accelX = filteredAccelX;
        data.accelY = filteredAccelY;
        data.accelZ = filteredAccelZ;
        
        // Convert temperature
        data.temperature = (float)raw.temp / 340.0f + 36.53f;
    }
    
    /**
     * Calibrate gyroscope and accelerometer
     * Device must be stationary on a level surface!
     * @return CALIB_SUCCESS or error code
     */
    uint8_t calibrate() {
        int32_t gxSum = 0, gySum = 0, gzSum = 0;
        int32_t axSum = 0, aySum = 0, azSum = 0;
        int16_t gxMin = 32767, gxMax = -32768;
        int16_t gyMin = 32767, gyMax = -32768;
        int16_t gzMin = 32767, gzMax = -32768;
        
        MPU6050_RawData raw;
        
        for (int i = 0; i < CALIB_SAMPLES; i++) {
            // Read raw data
            Wire.beginTransmission(MPU6050_ADDRESS);
            Wire.write(MPU6050_REG_ACCEL_XOUT_H);
            Wire.endTransmission(false);
            Wire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)14);
            
            raw.ax = (Wire.read() << 8) | Wire.read();
            raw.ay = (Wire.read() << 8) | Wire.read();
            raw.az = (Wire.read() << 8) | Wire.read();
            Wire.read(); Wire.read();  // Skip temperature
            raw.gx = (Wire.read() << 8) | Wire.read();
            raw.gy = (Wire.read() << 8) | Wire.read();
            raw.gz = (Wire.read() << 8) | Wire.read();
            
            // Accumulate sums
            gxSum += raw.gx;
            gySum += raw.gy;
            gzSum += raw.gz;
            axSum += raw.ax;
            aySum += raw.ay;
            azSum += raw.az;
            
            // Track min/max for motion detection
            if (raw.gx < gxMin) gxMin = raw.gx;
            if (raw.gx > gxMax) gxMax = raw.gx;
            if (raw.gy < gyMin) gyMin = raw.gy;
            if (raw.gy > gyMax) gyMax = raw.gy;
            if (raw.gz < gzMin) gzMin = raw.gz;
            if (raw.gz > gzMax) gzMax = raw.gz;
            
            delay(2);  // ~500 samples/second
        }
        
        // Check for motion during calibration
        int16_t gxRange = gxMax - gxMin;
        int16_t gyRange = gyMax - gyMin;
        int16_t gzRange = gzMax - gzMin;
        
        if (gxRange > CALIB_MOTION_THRESH || 
            gyRange > CALIB_MOTION_THRESH || 
            gzRange > CALIB_MOTION_THRESH) {
            return CALIB_FAIL_MOTION;
        }
        
        // Calculate averages
        calibration.gyroOffsetX = gxSum / CALIB_SAMPLES;
        calibration.gyroOffsetY = gySum / CALIB_SAMPLES;
        calibration.gyroOffsetZ = gzSum / CALIB_SAMPLES;
        
        // For accelerometer, assume Z axis is pointing up (1g)
        calibration.accelOffsetX = axSum / CALIB_SAMPLES;
        calibration.accelOffsetY = aySum / CALIB_SAMPLES;
        calibration.accelOffsetZ = (azSum / CALIB_SAMPLES) - (int16_t)ACCEL_SCALE_8G;
        
        calibration.calibrated = true;
        
        return CALIB_SUCCESS;
    }
    
    /**
     * Check if data is ready (interrupt flag)
     */
    bool dataReady() {
        return (readRegister(MPU6050_REG_INT_STATUS) & 0x01);
    }
    
private:
    // Filter states
    float filteredGyroX, filteredGyroY, filteredGyroZ;
    float filteredAccelX, filteredAccelY, filteredAccelZ;
    
    void writeRegister(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }
    
    uint8_t readRegister(uint8_t reg) {
        Wire.beginTransmission(MPU6050_ADDRESS);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)1);
        return Wire.read();
    }
};

#endif // MPU6050_H
