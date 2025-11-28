/*
 * Gyro.cpp - MPU6050 Gyroscope/Accelerometer Library Implementation
 * 
 * Uses I2C to communicate with MPU6050 sensor.
 * Implements complementary filter for stable angle estimation.
 */

#include "Arduino.h"
#include <Wire.h>
#include "Gyro.h"

// MPU6050 I2C Address
#define MPU6050_ADDR 0x68

// Register addresses
#define REG_PWR_MGMT_1  0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

// Constructor
Gyro::Gyro() {
  GyroScaled = {0, 0, 0};
  Gyro_angle = {0, 0, 0};
  RawAcc = {0, 0, 0};
  RawGyro = {0, 0, 0};
  tmp = 0;
  Acc_angle = {0, 0, 0};
  target = {0, 0, 0};
  cal = {0, 0, 0};
  GyroCal = {0, 0, 0};
  Acc_totalVec = 0;
  Time = 0;
  prevTime = 0;
  countTime = false;
  limZ = ScaleGyro / 100.0;
  GyroSet = true;
  error = {0, 0, 0};
}

void Gyro::setupwire() {
  // Initialize I2C
  Wire.begin();
  
  // Wake up MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);  // Clear sleep bit
  Wire.endTransmission(true);

  // Configure Gyroscope - FS_SEL=1 (500 deg/sec, 65.5 LSB/deg/sec)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_GYRO_CONFIG);
  Wire.write(0x08);  // FS_SEL = 1
  Wire.endTransmission();

  // Configure Accelerometer - AFS_SEL=2 (8g, 4096 LSB/g)
  // Note: Using AFS_SEL=1 (4g, 8192 LSB/g) as per original code
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_CONFIG);
  Wire.write(0x10);  // AFS_SEL = 2 (actually gives 4096 LSB/g)
  Wire.endTransmission();

  delay(100);
}

void Gyro::SetupWire(double TIME) {
  countTime = false;
  Time = TIME;
  setupwire();
  calibrateGyro();
}

void Gyro::SetupWire() {
  countTime = true;
  setupwire();
  calibrateGyro();
}

void Gyro::calculateError() {
  // Auto-calculate time step if needed
  if (countTime) {
    Time = (micros() - prevTime) * micro_to_sec;
    prevTime = micros();
  }

  readingMPU();
  calculateAngle();
}

void Gyro::setTarget(Vec3 Target) {
  target = Target;
}

void Gyro::setCalibration(Vec3 Cal) {
  cal = Cal;
}

void Gyro::readingMPU() {
  // Request sensor data starting from accelerometer registers
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  // Read accelerometer data (big-endian, 16-bit)
  RawAcc.x = (Wire.read() << 8) | Wire.read();
  RawAcc.y = (Wire.read() << 8) | Wire.read();
  RawAcc.z = (Wire.read() << 8) | Wire.read();

  // Read temperature (unused)
  tmp = (Wire.read() << 8) | Wire.read();

  // Read gyroscope data
  RawGyro.x = (Wire.read() << 8) | Wire.read();
  RawGyro.y = (Wire.read() << 8) | Wire.read();
  RawGyro.z = (Wire.read() << 8) | Wire.read();
}

void Gyro::calculateAngle() {
  // Scale gyro values to deg/sec
  GyroScaled.x = (RawGyro.x - GyroCal.x) / ScaleGyro;
  GyroScaled.y = (RawGyro.y - GyroCal.y) / ScaleGyro;

  // Apply low-pass filter to Z axis
  float rawZ = RawGyro.z - GyroCal.z;
  if (rawZ < limZ && rawZ > -limZ) {
    GyroScaled.z = 0;
  } else {
    GyroScaled.z = rawZ / ScaleGyro;
  }

  // Integrate angular velocity over time
  Gyro_angle.x += GyroScaled.x * Time;
  Gyro_angle.y += GyroScaled.y * Time;
  Gyro_angle.z += GyroScaled.z * Time;

  // Compensate for yaw rotation affecting pitch/roll
  Gyro_angle.x += Gyro_angle.y * sin(GyroScaled.z * Time * deg_to_rad);
  Gyro_angle.y -= Gyro_angle.x * sin(GyroScaled.z * Time * deg_to_rad);

  // Calculate acceleration magnitude
  Acc_totalVec = sqrt(pow(RawAcc.x, 2) + pow(RawAcc.y, 2) + pow(RawAcc.z, 2)) / ScaleAcc;

  // Calculate angles from accelerometer
  // Note: 0.85 factor on Z for improved accuracy
  Acc_angle.x = atan(RawAcc.y / sqrt(pow(RawAcc.x, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;
  Acc_angle.y = -atan(RawAcc.x / sqrt(pow(RawAcc.y, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;

  // Initialize or update angle using complementary filter
  if (GyroSet) {
    // First iteration - use accelerometer angle directly
    Gyro_angle.x = Acc_angle.x;
    Gyro_angle.y = Acc_angle.y;
    Gyro_angle.z = 0;
    GyroSet = false;
  } else {
    // Apply complementary filter (99% gyro, 1% accelerometer)
    // Only if accelerometer reading is valid
    if (RawAcc.z > -100 && Acc_totalVec > 0.1) {
      Gyro_angle.x = 0.99 * Gyro_angle.x + 0.01 * Acc_angle.x;
      Gyro_angle.y = 0.99 * Gyro_angle.y + 0.01 * Acc_angle.y;
    }
  }

  // Calculate error (difference from target, accounting for calibration)
  error.x = Gyro_angle.x - target.x - cal.x;
  error.y = Gyro_angle.y - target.y - cal.y;
  error.z = Gyro_angle.z - target.z - cal.z;
}

void Gyro::calibrateGyro() {
  double sumX = 0, sumY = 0, sumZ = 0;
  const int samples = 1500;

  for (int i = 0; i < samples; i++) {
    readingMPU();
    sumX += RawGyro.x;
    sumY += RawGyro.y;
    sumZ += RawGyro.z;
  }

  delay(100);

  // Calculate average offset
  GyroCal.x = sumX / samples;
  GyroCal.y = sumY / samples;
  GyroCal.z = sumZ / samples;
}

Vec3 Gyro::calibrate(int samples) {
  float tempX = 0, tempY = 0;
  Vec3 result;

  // Reset target and calibration
  setTarget({0, 0, 0});
  setCalibration({0, 0, 0});
  calibrateGyro();

  // Sample errors
  for (int i = 0; i < samples; i++) {
    calculateError();
    tempX += error.x;
    tempY += error.y;
  }

  // Return average error as calibration offset
  result.x = tempX / samples;
  result.y = tempY / samples;
  result.z = 0;

  return result;
}

void Gyro::zeroYaw(bool reset) {
  if (reset) {
    Gyro_angle.z = 0;
  }
}
