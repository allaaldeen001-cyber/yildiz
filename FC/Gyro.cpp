#include "Arduino.h"
#include <Wire.h>
#include "Gyro.h"

void Gyro::setupwire() {
  // Initialize I2C
  Wire.begin();
  Wire.beginTransmission(0x68);  // MPU6050 address
  Wire.write(0x6B);              // PWR_MGMT_1 register
  Wire.write(0);                  // Wake up MPU6050
  Wire.endTransmission(true);
  
  // Configure gyro: FS_SEL = 1 (500 deg/sec, 65.5 LSB/deg/sec)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);              // GYRO_CONFIG register
  Wire.write(0x08);               // FS_SEL = 1
  Wire.endTransmission();
  
  // Configure accelerometer: AFS_SEL = 2 (4096 LSB/g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);              // ACCEL_CONFIG register
  Wire.write(0x10);              // AFS_SEL = 2
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
  if (countTime) {
    Time = micros() - prevTime;
    Time *= micro_to_sec;
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
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  
  // Read accelerometer
  RawAcc.x = Wire.read() << 8 | Wire.read();  // 0x3B & 0x3C
  RawAcc.y = Wire.read() << 8 | Wire.read();  // 0x3D & 0x3E
  RawAcc.z = Wire.read() << 8 | Wire.read();  // 0x3F & 0x40
  
  tmp = Wire.read() << 8 | Wire.read();       // 0x41 & 0x42 (temperature)
  
  // Read gyroscope
  RawGyro.x = Wire.read() << 8 | Wire.read();  // 0x43 & 0x44
  RawGyro.y = Wire.read() << 8 | Wire.read();  // 0x45 & 0x46
  RawGyro.z = Wire.read() << 8 | Wire.read();  // 0x47 & 0x48
}

void Gyro::calculateAngle() {
  // Scale gyro values
  GyroScaled.x = ((RawGyro.x - GyroCal.x) / ScaleGyro);
  GyroScaled.y = ((RawGyro.y - GyroCal.y) / ScaleGyro);
  
  // Low pass filter for Z axis
  if ((RawGyro.z - GyroCal.z) < limZ && (RawGyro.z - GyroCal.z) > (-limZ)) {
    GyroScaled.z = 0;
  } else {
    GyroScaled.z = ((RawGyro.z - GyroCal.z) / ScaleGyro);
  }
  
  // Integrate angular speed over time
  Gyro_angle.x += GyroScaled.x * Time;
  Gyro_angle.y += GyroScaled.y * Time;
  Gyro_angle.z += GyroScaled.z * Time;
  
  // Compensate for rotation around Z axis
  Gyro_angle.x += Gyro_angle.y * sin(GyroScaled.z * Time * deg_to_rad);
  Gyro_angle.y -= Gyro_angle.x * sin(GyroScaled.z * Time * deg_to_rad);
  
  // Calculate acceleration angle
  Acc_totalVec = sqrt(pow(RawAcc.x, 2) + pow(RawAcc.y, 2) + pow(RawAcc.z, 2)) / ScaleAcc;
  
  Acc_angle.x = atan(RawAcc.y / sqrt(pow(RawAcc.x, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;
  Acc_angle.y = -atan(RawAcc.x / sqrt(pow(RawAcc.y, 2) + pow(RawAcc.z * 0.85, 2))) * rad_to_deg;
  
  // Initialize on first iteration
  if (GyroSet) {
    Gyro_angle.x = Acc_angle.x;
    Gyro_angle.y = Acc_angle.y;
    Gyro_angle.z = 0;
    GyroSet = false;
  }
  
  // Complementary filter: 99% gyro, 1% accelerometer
  if (RawAcc.z > -100 && Acc_totalVec > 0.1) {
    Gyro_angle.x = 0.99 * Gyro_angle.x + Acc_angle.x * 0.01;
    Gyro_angle.y = 0.99 * Gyro_angle.y + Acc_angle.y * 0.01;
  }
  
  // Calculate error
  error.x = Gyro_angle.x - target.x - cal.x;
  error.y = Gyro_angle.y - target.y - cal.y;
  error.z = Gyro_angle.z - target.z - cal.z;
}

void Gyro::calibrateGyro() {
  double x = 0;
  double y = 0;
  double z = 0;
  int n = 1500;
  
  for (int i = 0; i < n; i++) {
    readingMPU();
    x += RawGyro.x;
    y += RawGyro.y;
    z += RawGyro.z;
  }
  
  delay(100);
  
  GyroCal.x = x / n;
  GyroCal.y = y / n;
  GyroCal.z = z / n;
}

Vec3 Gyro::calibrate(int n) {
  float tempX = 0;
  float tempY = 0;
  Vec3 temp;
  
  setTarget({0, 0, 0});
  setCalibration({0, 0, 0});
  calibrateGyro();
  
  for (int i = 0; i < n; i++) {
    calculateError();
    tempX += error.x;
    tempY += error.y;
  }
  
  temp.x = tempX / n;
  temp.y = tempY / n;
  
  return temp;
}

void Gyro::zeroYaw(bool lt) {
  if (lt) {
    Gyro_angle.z = 0;
  }
}
