/**
 * ============================================================================
 * GYRO.CPP - MPU6050 Gyroscope/Accelerometer Implementation
 * ============================================================================
 */

#include "Arduino.h"
#include <Wire.h>
#include "Gyro.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

Gyro::Gyro() {
  // Initialize all vectors to zero
  GyroScaled = {0, 0, 0};
  Gyro_angle = {0, 0, 0};
  RawAcc = {0, 0, 0};
  RawGyro = {0, 0, 0};
  Acc_angle = {0, 0, 0};
  target = {0, 0, 0};
  cal = {0, 0, 0};
  GyroCal = {0, 0, 0};
  error = {0, 0, 0};
  
  tmp = 0;
  Acc_totalVec = 0;
  Time = 0;
  prevTime = 0;
  countTime = false;
  GyroSet = true;
  
  // Low pass filter threshold for Z axis
  limZ = ScaleGyro / 100.0;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void Gyro::setupwire() {
  // Initialize I2C
  Wire.begin();
  
  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Set to zero to wake up
  Wire.endTransmission(true);
  
  // Configure Gyroscope
  // FS_SEL = 1 (±500 deg/s, 65.5 LSB/deg/s)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x08);  // FS_SEL = 1
  Wire.endTransmission();
  
  // Configure Accelerometer
  // AFS_SEL = 2 (±8g, 4096 LSB/g) - Note: original code used 0x10 which is ±8g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x10);  // AFS_SEL = 2 (±8g)
  Wire.endTransmission();
  
  // Configure Digital Low Pass Filter
  // DLPF_CFG = 3 (44Hz accelerometer, 42Hz gyroscope)
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x03);  // DLPF_CFG = 3
  Wire.endTransmission();
  
  delay(100);
}

void Gyro::SetupWire(double TIME) {
  // Use fixed time step
  countTime = false;
  Time = TIME;
  
  setupwire();
  calibrateGyro();
  
  Serial.println(F("Gyro initialized with fixed timestep"));
}

void Gyro::SetupWire() {
  // Use automatic time measurement
  countTime = true;
  
  setupwire();
  calibrateGyro();
  
  Serial.println(F("Gyro initialized with auto timestep"));
}

// ============================================================================
// MAIN INTERFACE
// ============================================================================

void Gyro::calculateError() {
  // Update time if using automatic measurement
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

// ============================================================================
// SENSOR READING
// ============================================================================

void Gyro::readingMPU() {
  // Request 14 bytes starting from ACCEL_XOUT_H (0x3B)
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  
  // Read accelerometer data (6 bytes)
  RawAcc.x = (Wire.read() << 8) | Wire.read();  // ACCEL_XOUT
  RawAcc.y = (Wire.read() << 8) | Wire.read();  // ACCEL_YOUT
  RawAcc.z = (Wire.read() << 8) | Wire.read();  // ACCEL_ZOUT
  
  // Read temperature (2 bytes) - unused but must be read
  tmp = (Wire.read() << 8) | Wire.read();
  
  // Read gyroscope data (6 bytes)
  RawGyro.x = (Wire.read() << 8) | Wire.read();  // GYRO_XOUT
  RawGyro.y = (Wire.read() << 8) | Wire.read();  // GYRO_YOUT
  RawGyro.z = (Wire.read() << 8) | Wire.read();  // GYRO_ZOUT
}

// ============================================================================
// ANGLE CALCULATION
// ============================================================================

void Gyro::calculateAngle() {
  // Scale gyroscope values to deg/s
  GyroScaled.x = (RawGyro.x - GyroCal.x) / ScaleGyro;
  GyroScaled.y = (RawGyro.y - GyroCal.y) / ScaleGyro;
  
  // Apply low-pass filter to Z axis to reduce drift
  float rawZ = RawGyro.z - GyroCal.z;
  if (rawZ < limZ && rawZ > -limZ) {
    GyroScaled.z = 0;
  } else {
    GyroScaled.z = rawZ / ScaleGyro;
  }
  
  // Integrate angular velocity to get angle
  Gyro_angle.x += GyroScaled.x * Time;
  Gyro_angle.y += GyroScaled.y * Time;
  Gyro_angle.z += GyroScaled.z * Time;
  
  // Compensate for yaw-induced roll/pitch coupling
  // When the drone rotates around Z, X and Y axes mix
  Gyro_angle.x += Gyro_angle.y * sin(GyroScaled.z * Time * deg_to_rad);
  Gyro_angle.y -= Gyro_angle.x * sin(GyroScaled.z * Time * deg_to_rad);
  
  // Calculate total acceleration vector magnitude (in g)
  Acc_totalVec = sqrt(pow(RawAcc.x, 2) + pow(RawAcc.y, 2) + pow(RawAcc.z, 2)) / ScaleAcc;
  
  // Calculate angles from accelerometer using trigonometry
  // Note: 0.85 factor on Z is for calibration/mounting compensation
  Acc_angle.x = atan(RawAcc.y / sqrt(pow(RawAcc.x, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;
  Acc_angle.y = -atan(RawAcc.x / sqrt(pow(RawAcc.y, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;
  
  // First iteration: initialize with accelerometer angle
  if (GyroSet) {
    Gyro_angle.x = Acc_angle.x;
    Gyro_angle.y = Acc_angle.y;
    Gyro_angle.z = 0;
    GyroSet = false;
    return;
  }
  
  // Complementary filter: blend gyro (fast) with accelerometer (stable)
  // Only apply when acceleration reading is valid
  // (drone not accelerating too much and Z accel is positive = right side up)
  if (RawAcc.z > -100 && Acc_totalVec > 0.1 && Acc_totalVec < 2.0) {
    Gyro_angle.x = 0.99 * Gyro_angle.x + 0.01 * Acc_angle.x;
    Gyro_angle.y = 0.99 * Gyro_angle.y + 0.01 * Acc_angle.y;
  }
  
  // Calculate error: current angle - target angle - calibration offset
  error.x = Gyro_angle.x - target.x - cal.x;
  error.y = Gyro_angle.y - target.y - cal.y;
  error.z = Gyro_angle.z - target.z - cal.z;
}

// ============================================================================
// CALIBRATION
// ============================================================================

void Gyro::calibrateGyro() {
  double x = 0, y = 0, z = 0;
  const int n = 1500;
  
  Serial.print(F("Calibrating gyro bias"));
  
  for (int i = 0; i < n; i++) {
    readingMPU();
    x += RawGyro.x;
    y += RawGyro.y;
    z += RawGyro.z;
    
    if (i % 300 == 0) {
      Serial.print(F("."));
    }
  }
  
  GyroCal.x = x / n;
  GyroCal.y = y / n;
  GyroCal.z = z / n;
  
  Serial.println(F(" Done"));
  Serial.print(F("Gyro bias: X="));
  Serial.print(GyroCal.x);
  Serial.print(F(" Y="));
  Serial.print(GyroCal.y);
  Serial.print(F(" Z="));
  Serial.println(GyroCal.z);
  
  delay(100);
}

Vec3 Gyro::calibrate(int n) {
  float tempX = 0, tempY = 0;
  Vec3 temp = {0, 0, 0};
  
  // Reset target and calibration for measurement
  setTarget({0, 0, 0});
  setCalibration({0, 0, 0});
  
  // Recalibrate gyro bias
  calibrateGyro();
  
  // Measure average error (level offset)
  Serial.print(F("Measuring level offset"));
  for (int i = 0; i < n; i++) {
    calculateError();
    tempX += error.x;
    tempY += error.y;
    
    if (i % 200 == 0) {
      Serial.print(F("."));
    }
  }
  Serial.println(F(" Done"));
  
  temp.x = tempX / n;
  temp.y = tempY / n;
  temp.z = 0;  // Yaw is always zeroed, not calibrated
  
  Serial.print(F("Level offset: X="));
  Serial.print(temp.x);
  Serial.print(F(" Y="));
  Serial.println(temp.y);
  
  return temp;
}

void Gyro::zeroYaw(bool lt) {
  if (lt) {
    Gyro_angle.z = 0;
  }
}
