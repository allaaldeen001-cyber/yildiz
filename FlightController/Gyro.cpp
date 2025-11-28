/*
 * Gyro.cpp
 * ========
 * MPU6050 Gyroscope/Accelerometer driver implementation
 * Uses complementary filter for attitude estimation
 */

#include "Arduino.h"
#include <Wire.h>
#include "Gyro.h"

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// Register addresses
#define REG_PWR_MGMT_1  0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

// Internal I2C setup
void Gyro::setupwire() {
    Wire.begin();
    
    // Wake up MPU6050 (clear sleep bit)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission(true);
    
    // Configure gyroscope
    // FS_SEL = 1: ±500°/s, 65.5 LSB/(°/s)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_GYRO_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission();
    
    // Configure accelerometer
    // AFS_SEL = 2: ±8g, 4096 LSB/g (actually using 8192 in code)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_ACCEL_CONFIG);
    Wire.write(0x10);
    Wire.endTransmission();
    
    delay(100);
}

// Setup with fixed time period
void Gyro::SetupWire(double TIME) {
    countTime = false;
    Time = TIME;
    setupwire();
    calibrateGyro();
}

// Setup with automatic timing
void Gyro::SetupWire() {
    countTime = true;
    setupwire();
    calibrateGyro();
}

// Main calculation function
void Gyro::calculateError() {
    // Auto timing if enabled
    if (countTime) {
        Time = micros() - prevTime;
        Time *= micro_to_sec;
        prevTime = micros();
    }
    
    readingMPU();
    calculateAngle();
}

// Set target angles
void Gyro::setTarget(Vec3 Target) {
    target = Target;
}

// Set calibration offsets
void Gyro::setCalibration(Vec3 Cal) {
    cal = Cal;
}

// Read raw values from MPU6050
void Gyro::readingMPU() {
    // Request 14 bytes starting from ACCEL_XOUT_H
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);
    
    // Read accelerometer (6 bytes)
    RawAcc.x = Wire.read() << 8 | Wire.read();
    RawAcc.y = Wire.read() << 8 | Wire.read();
    RawAcc.z = Wire.read() << 8 | Wire.read();
    
    // Read temperature (2 bytes, unused)
    tmp = Wire.read() << 8 | Wire.read();
    
    // Read gyroscope (6 bytes)
    RawGyro.x = Wire.read() << 8 | Wire.read();
    RawGyro.y = Wire.read() << 8 | Wire.read();
    RawGyro.z = Wire.read() << 8 | Wire.read();
}

// Calculate attitude from sensor data
void Gyro::calculateAngle() {
    // Scale gyroscope values
    GyroScaled.x = (RawGyro.x - GyroCal.x) / ScaleGyro;
    GyroScaled.y = (RawGyro.y - GyroCal.y) / ScaleGyro;
    
    // Z-axis with low-pass filter (reduce noise)
    if (abs(RawGyro.z - GyroCal.z) < limZ) {
        GyroScaled.z = 0;
    } else {
        GyroScaled.z = (RawGyro.z - GyroCal.z) / ScaleGyro;
    }
    
    // Integrate angular velocity over time
    Gyro_angle.x += GyroScaled.x * Time;
    Gyro_angle.y += GyroScaled.y * Time;
    Gyro_angle.z += GyroScaled.z * Time;
    
    // Transfer between axes due to yaw rotation
    // (prevents gimbal lock issues)
    Gyro_angle.x += Gyro_angle.y * sin(GyroScaled.z * Time * deg_to_rad);
    Gyro_angle.y -= Gyro_angle.x * sin(GyroScaled.z * Time * deg_to_rad);
    
    // Calculate acceleration angle (tilt from gravity)
    Acc_totalVec = sqrt(pow(RawAcc.x, 2) + pow(RawAcc.y, 2) + pow(RawAcc.z, 2)) / ScaleAcc;
    
    // Acceleration-based angles (pitch and roll)
    // Using 0.85 factor on Z to compensate for sensor mounting
    Acc_angle.x =  atan(RawAcc.y / sqrt(pow(RawAcc.x, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;
    Acc_angle.y = -atan(RawAcc.x / sqrt(pow(RawAcc.y, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;
    
    // First iteration: use accelerometer angle directly
    if (GyroSet) {
        Gyro_angle.x = Acc_angle.x;
        Gyro_angle.y = Acc_angle.y;
        Gyro_angle.z = 0;
        GyroSet = false;
    }
    
    // Complementary filter: 99% gyro + 1% accelerometer
    // Only apply if acceleration is valid (not in free-fall or extreme motion)
    if (RawAcc.z > -100 && Acc_totalVec > 0.1) {
        Gyro_angle.x = 0.99 * Gyro_angle.x + 0.01 * Acc_angle.x;
        Gyro_angle.y = 0.99 * Gyro_angle.y + 0.01 * Acc_angle.y;
    }
    
    // Calculate error: actual angle - target - calibration
    error.x = Gyro_angle.x - target.x - cal.x;
    error.y = Gyro_angle.y - target.y - cal.y;
    error.z = Gyro_angle.z - target.z - cal.z;
}

// Calibrate gyroscope (find zero offset)
void Gyro::calibrateGyro() {
    double x = 0;
    double y = 0;
    double z = 0;
    
    const int samples = 1500;
    
    Serial.println(F("Calibrating gyroscope..."));
    
    for (int i = 0; i < samples; i++) {
        readingMPU();
        x += RawGyro.x;
        y += RawGyro.y;
        z += RawGyro.z;
        
        // Visual feedback
        if (i % 300 == 0) Serial.print(".");
    }
    Serial.println(F(" Done"));
    
    delay(100);
    
    GyroCal.x = x / samples;
    GyroCal.y = y / samples;
    GyroCal.z = z / samples;
}

// Full calibration including level position
Vec3 Gyro::calibrate(int n) {
    float tempX = 0;
    float tempY = 0;
    Vec3 temp;
    
    // Reset targets and calibration
    setTarget({0, 0, 0});
    setCalibration({0, 0, 0});
    
    // Recalibrate gyro zero
    calibrateGyro();
    
    Serial.println(F("Calibrating level position..."));
    
    for (int i = 0; i < n; i++) {
        calculateError();
        tempX += error.x;
        tempY += error.y;
        
        if (i % 200 == 0) Serial.print(".");
    }
    Serial.println(F(" Done"));
    
    temp.x = tempX / n;
    temp.y = tempY / n;
    temp.z = 0;
    
    return temp;
}

// Reset yaw to zero
void Gyro::zeroYaw(bool reset) {
    if (reset) {
        Gyro_angle.z = 0;
    }
}
