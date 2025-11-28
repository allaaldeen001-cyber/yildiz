#include "Arduino.h"
#include <Wire.h>
#include "Gyro.h"

void Gyro::setupwire()
{
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C speed
  
  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up
  Wire.endTransmission(true);
  
  // Configure gyroscope range: ±500°/s (FS_SEL=1)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x08);  // FS_SEL=1 -> 65.5 LSB/(°/s)
  Wire.endTransmission();
  
  // Configure accelerometer range: ±8g (AFS_SEL=2)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x10);  // AFS_SEL=2 -> 4096 LSB/g
  Wire.endTransmission();
  
  // Configure low-pass filter (DLPF)
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // CONFIG register
  Wire.write(0x03);  // DLPF_CFG=3 -> ~44Hz bandwidth
  Wire.endTransmission();
  
  delay(100);
}

void Gyro::SetupWire(double TIME)
{
  countTime = false;
  Time = TIME;
  setupwire();
  calibrateGyro();
}

void Gyro::SetupWire()
{
  countTime = true;
  setupwire();
  calibrateGyro();
}

void Gyro::calculateError()
{
  if (countTime) {
    Time = micros() - prevTime;
    Time *= micro_to_sec;
    prevTime = micros();
  }
  
  readingMPU();
  calculateAngle();
}

void Gyro::setTarget(Vec3 Target)
{
  target = Target;
}

void Gyro::setCalibration(Vec3 Cal)
{
  cal = Cal;
}

void Gyro::readingMPU()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // Start at ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  
  // Read accelerometer (6 bytes)
  RawAcc.x = (Wire.read() << 8) | Wire.read();
  RawAcc.y = (Wire.read() << 8) | Wire.read();
  RawAcc.z = (Wire.read() << 8) | Wire.read();
  
  // Skip temperature (2 bytes)
  tmp = (Wire.read() << 8) | Wire.read();
  
  // Read gyroscope (6 bytes)
  RawGyro.x = (Wire.read() << 8) | Wire.read();
  RawGyro.y = (Wire.read() << 8) | Wire.read();
  RawGyro.z = (Wire.read() << 8) | Wire.read();
}

void Gyro::calculateAngle()
{
  // Scale gyro values to deg/s
  GyroScaled.x = (RawGyro.x - GyroCal.x) / ScaleGyro;
  GyroScaled.y = (RawGyro.y - GyroCal.y) / ScaleGyro;
  
  // Low-pass filter for Z axis
  float rawZ = RawGyro.z - GyroCal.z;
  if (abs(rawZ) < limZ)
    GyroScaled.z = 0;
  else
    GyroScaled.z = rawZ / ScaleGyro;
  
  // Integrate angular velocity to get angle
  Gyro_angle.x += GyroScaled.x * Time;
  Gyro_angle.y += GyroScaled.y * Time;
  Gyro_angle.z += GyroScaled.z * Time;
  
  // Compensate for rotation around Z axis
  Gyro_angle.x += Gyro_angle.y * sin(GyroScaled.z * Time * deg_to_rad);
  Gyro_angle.y -= Gyro_angle.x * sin(GyroScaled.z * Time * deg_to_rad);
  
  // Calculate acceleration angles
  Acc_totalVec = sqrt(sq(RawAcc.x) + sq(RawAcc.y) + sq(RawAcc.z)) / ScaleAcc;
  
  Acc_angle.x = atan2(RawAcc.y, sqrt(sq(RawAcc.x) + sq(RawAcc.z * 0.85))) * rad_to_deg;
  Acc_angle.y = -atan2(RawAcc.x, sqrt(sq(RawAcc.y) + sq(RawAcc.z * 0.85))) * rad_to_deg;
  
  // Initialize on first run
  if (GyroSet) {
    Gyro_angle.x = Acc_angle.x;
    Gyro_angle.y = Acc_angle.y;
    Gyro_angle.z = 0;
    GyroSet = false;
  }
  
  // Complementary filter: 99% gyro, 1% accelerometer
  if (RawAcc.z > -100 && Acc_totalVec > 0.1) {
    Gyro_angle.x = 0.99 * Gyro_angle.x + 0.01 * Acc_angle.x;
    Gyro_angle.y = 0.99 * Gyro_angle.y + 0.01 * Acc_angle.y;
  }
  
  // Calculate error
  error.x = Gyro_angle.x - target.x - cal.x;
  error.y = Gyro_angle.y - target.y - cal.y;
  error.z = Gyro_angle.z - target.z - cal.z;
}

void Gyro::calibrateGyro()
{
  double x = 0, y = 0, z = 0;
  int n = 1500;
  
  Serial.println("Calibrating gyro... Keep drone still!");
  
  for (int i = 0; i < n; i++) {
    readingMPU();
    x += RawGyro.x;
    y += RawGyro.y;
    z += RawGyro.z;
    delay(2);
  }
  
  GyroCal.x = x / n;
  GyroCal.y = y / n;
  GyroCal.z = z / n;
  
  Serial.println("Gyro calibration complete");
}

Vec3 Gyro::calibrate(int n)
{
  float tempX = 0, tempY = 0;
  Vec3 temp;
  
  setTarget({0, 0, 0});
  setCalibration({0, 0, 0});
  calibrateGyro();
  
  Serial.println("Calibrating level... Keep drone level!");
  
  for (int i = 0; i < n; i++) {
    calculateError();
    tempX += error.x;
    tempY += error.y;
    delay(5);
  }
  
  temp.x = tempX / n;
  temp.y = tempY / n;
  temp.z = 0;
  
  Serial.print("Calibration offsets: X=");
  Serial.print(temp.x);
  Serial.print(" Y=");
  Serial.println(temp.y);
  
  return temp;
}

void Gyro::zeroYaw(bool lt)
{
  if (lt)
    Gyro_angle.z = 0;
}
