/*
 * ============================================================================
 * MPU6050 IMU SENSOR HANDLER - IMPLEMENTATION
 * ============================================================================
 */

#include "mpu6050.h"

MPU6050::MPU6050() {
    // Initialize raw values
    rawAccelX = rawAccelY = rawAccelZ = 0;
    rawGyroX = rawGyroY = rawGyroZ = 0;
    
    // Initialize offsets
    gyroOffsetX = gyroOffsetY = gyroOffsetZ = 0;
    accelOffsetX = accelOffsetY = accelOffsetZ = 0;
    
    // Initialize processed values
    gyroX = gyroY = gyroZ = 0.0f;
    accelX = accelY = accelZ = 0.0f;
    
    // Initialize angles
    roll = pitch = yaw = 0.0f;
    
    // Complementary filter coefficient (0.98 = trust gyro more)
    alpha = 0.98f;
    
    // Default sensitivity scales
    gyroScale = GYRO_SENSITIVITY_500DPS;
    accelScale = ACCEL_SENSITIVITY_8G;
    
    calibrated = false;
}

bool MPU6050::begin() {
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C clock
    
    // Check if MPU6050 is connected
    if (!testConnection()) {
        return false;
    }
    
    // Wake up MPU6050 (clear sleep bit)
    writeRegister(MPU6050_REG_PWR_MGMT_1, 0x00);
    delay(100);
    
    // Set clock source to PLL with X-axis gyroscope reference
    writeRegister(MPU6050_REG_PWR_MGMT_1, 0x01);
    delay(10);
    
    // Set sample rate divider (1kHz / (1 + 3) = 250Hz)
    writeRegister(MPU6050_REG_SMPLRT_DIV, 0x03);
    
    // Set DLPF (Digital Low Pass Filter) to ~44Hz bandwidth
    // This reduces noise but adds ~4.9ms delay
    writeRegister(MPU6050_REG_CONFIG, 0x03);
    
    // Set gyroscope range to ±500 deg/s
    writeRegister(MPU6050_REG_GYRO_CONFIG, 0x08);
    gyroScale = GYRO_SENSITIVITY_500DPS;
    
    // Set accelerometer range to ±8g
    writeRegister(MPU6050_REG_ACCEL_CONFIG, 0x10);
    accelScale = ACCEL_SENSITIVITY_8G;
    
    // Disable interrupts
    writeRegister(MPU6050_REG_INT_ENABLE, 0x00);
    
    delay(100);
    return true;
}

bool MPU6050::testConnection() {
    uint8_t whoAmI = readRegister(MPU6050_REG_WHO_AM_I);
    return (whoAmI == 0x68 || whoAmI == 0x72);  // 0x68 for MPU6050, 0x72 for some variants
}

void MPU6050::readRawData() {
    uint8_t buffer[14];
    
    // Read all 14 bytes at once (Accel, Temp, Gyro)
    readRegisters(MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
    
    // Parse accelerometer data (big-endian)
    rawAccelX = (int16_t)((buffer[0] << 8) | buffer[1]);
    rawAccelY = (int16_t)((buffer[2] << 8) | buffer[3]);
    rawAccelZ = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // Parse temperature data
    rawTemp = (int16_t)((buffer[6] << 8) | buffer[7]);
    
    // Parse gyroscope data (big-endian)
    rawGyroX = (int16_t)((buffer[8] << 8) | buffer[9]);
    rawGyroY = (int16_t)((buffer[10] << 8) | buffer[11]);
    rawGyroZ = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    // Apply calibration offsets and convert to physical units
    gyroX = (float)(rawGyroX - gyroOffsetX) / gyroScale;
    gyroY = (float)(rawGyroY - gyroOffsetY) / gyroScale;
    gyroZ = (float)(rawGyroZ - gyroOffsetZ) / gyroScale;
    
    accelX = (float)(rawAccelX - accelOffsetX) / accelScale;
    accelY = (float)(rawAccelY - accelOffsetY) / accelScale;
    accelZ = (float)(rawAccelZ - accelOffsetZ) / accelScale;
}

void MPU6050::readGyro() {
    uint8_t buffer[6];
    readRegisters(MPU6050_REG_GYRO_XOUT_H, buffer, 6);
    
    rawGyroX = (int16_t)((buffer[0] << 8) | buffer[1]);
    rawGyroY = (int16_t)((buffer[2] << 8) | buffer[3]);
    rawGyroZ = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    gyroX = (float)(rawGyroX - gyroOffsetX) / gyroScale;
    gyroY = (float)(rawGyroY - gyroOffsetY) / gyroScale;
    gyroZ = (float)(rawGyroZ - gyroOffsetZ) / gyroScale;
}

void MPU6050::readAccel() {
    uint8_t buffer[6];
    readRegisters(MPU6050_REG_ACCEL_XOUT_H, buffer, 6);
    
    rawAccelX = (int16_t)((buffer[0] << 8) | buffer[1]);
    rawAccelY = (int16_t)((buffer[2] << 8) | buffer[3]);
    rawAccelZ = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    accelX = (float)(rawAccelX - accelOffsetX) / accelScale;
    accelY = (float)(rawAccelY - accelOffsetY) / accelScale;
    accelZ = (float)(rawAccelZ - accelOffsetZ) / accelScale;
}

void MPU6050::calculateAngles(float dt) {
    // Calculate angles from accelerometer
    float accelRoll = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0f / PI;
    float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0f / PI;
    
    // Integrate gyroscope data
    roll += gyroX * dt;
    pitch += gyroY * dt;
    yaw += gyroZ * dt;
    
    // Complementary filter: combine gyro and accel data
    // High-pass filter on gyro, low-pass filter on accel
    roll = alpha * roll + (1.0f - alpha) * accelRoll;
    pitch = alpha * pitch + (1.0f - alpha) * accelPitch;
    
    // Yaw drift correction would need magnetometer
    // For now, just let it drift or use external yaw reference
}

void MPU6050::updateAttitude(float dt) {
    readRawData();
    calculateAngles(dt);
}

void MPU6050::calibrateGyro(int samples) {
    long sumX = 0, sumY = 0, sumZ = 0;
    uint8_t buffer[6];
    
    Serial.println(F("Calibrating gyroscope..."));
    Serial.println(F("Keep the drone perfectly still!"));
    
    // Discard first readings
    for (int i = 0; i < 100; i++) {
        readRegisters(MPU6050_REG_GYRO_XOUT_H, buffer, 6);
        delay(3);
    }
    
    // Collect samples
    for (int i = 0; i < samples; i++) {
        readRegisters(MPU6050_REG_GYRO_XOUT_H, buffer, 6);
        
        sumX += (int16_t)((buffer[0] << 8) | buffer[1]);
        sumY += (int16_t)((buffer[2] << 8) | buffer[3]);
        sumZ += (int16_t)((buffer[4] << 8) | buffer[5]);
        
        // Progress indicator
        if (i % 500 == 0) {
            Serial.print(F("Progress: "));
            Serial.print((i * 100) / samples);
            Serial.println(F("%"));
        }
        
        delay(3);
    }
    
    // Calculate average offsets
    gyroOffsetX = sumX / samples;
    gyroOffsetY = sumY / samples;
    gyroOffsetZ = sumZ / samples;
    
    Serial.println(F("Gyro calibration complete!"));
    Serial.print(F("Offsets - X: ")); Serial.print(gyroOffsetX);
    Serial.print(F(" Y: ")); Serial.print(gyroOffsetY);
    Serial.print(F(" Z: ")); Serial.println(gyroOffsetZ);
    
    calibrated = true;
}

void MPU6050::calibrateAccel() {
    long sumX = 0, sumY = 0, sumZ = 0;
    int samples = 1000;
    uint8_t buffer[6];
    
    Serial.println(F("Calibrating accelerometer..."));
    Serial.println(F("Place drone on a flat, level surface!"));
    
    delay(2000);  // Give user time to position drone
    
    // Collect samples
    for (int i = 0; i < samples; i++) {
        readRegisters(MPU6050_REG_ACCEL_XOUT_H, buffer, 6);
        
        sumX += (int16_t)((buffer[0] << 8) | buffer[1]);
        sumY += (int16_t)((buffer[2] << 8) | buffer[3]);
        sumZ += (int16_t)((buffer[4] << 8) | buffer[5]);
        
        delay(3);
    }
    
    // Calculate offsets (Z should read 1g when level)
    accelOffsetX = sumX / samples;
    accelOffsetY = sumY / samples;
    accelOffsetZ = (sumZ / samples) - (int16_t)accelScale;  // Subtract 1g
    
    Serial.println(F("Accel calibration complete!"));
    Serial.print(F("Offsets - X: ")); Serial.print(accelOffsetX);
    Serial.print(F(" Y: ")); Serial.print(accelOffsetY);
    Serial.print(F(" Z: ")); Serial.println(accelOffsetZ);
}

void MPU6050::setGyroOffsets(int16_t x, int16_t y, int16_t z) {
    gyroOffsetX = x;
    gyroOffsetY = y;
    gyroOffsetZ = z;
    calibrated = true;
}

void MPU6050::setAccelOffsets(int16_t x, int16_t y, int16_t z) {
    accelOffsetX = x;
    accelOffsetY = y;
    accelOffsetZ = z;
}

int16_t MPU6050::getTemperature() {
    // Temperature in degrees C = (rawTemp / 340.0) + 36.53
    return (int16_t)((rawTemp / 340.0f) + 36.53f);
}

void MPU6050::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t MPU6050::readRegister(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDRESS, (uint8_t)1);
    return Wire.read();
}

void MPU6050::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDRESS, length);
    
    for (uint8_t i = 0; i < length && Wire.available(); i++) {
        buffer[i] = Wire.read();
    }
}
