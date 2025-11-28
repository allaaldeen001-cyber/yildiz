#include <Wire.h>
#include <math.h>
#include "Gyro.h"

void Gyro::setupwire() {
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  delay(100);
}

void Gyro::SetupWire(double loopTime) {
  countTime = false;
  Time = loopTime;
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

  RawAcc.x = Wire.read() << 8 | Wire.read();
  RawAcc.y = Wire.read() << 8 | Wire.read();
  RawAcc.z = Wire.read() << 8 | Wire.read();

  tmp = Wire.read() << 8 | Wire.read();

  RawGyro.x = Wire.read() << 8 | Wire.read();
  RawGyro.y = Wire.read() << 8 | Wire.read();
  RawGyro.z = Wire.read() << 8 | Wire.read();
}

void Gyro::calculateAngle() {
  GyroScaled.x = (RawGyro.x - GyroCal.x) / ScaleGyro;
  GyroScaled.y = (RawGyro.y - GyroCal.y) / ScaleGyro;

  const float zDelta = RawGyro.z - GyroCal.z;
  if (zDelta < limZ && zDelta > -limZ) {
    GyroScaled.z = 0;
  } else {
    GyroScaled.z = zDelta / ScaleGyro;
  }

  Gyro_angle.x += GyroScaled.x * Time;
  Gyro_angle.y += GyroScaled.y * Time;
  Gyro_angle.z += GyroScaled.z * Time;

  Gyro_angle.x += Gyro_angle.y * sin(GyroScaled.z * Time * deg_to_rad);
  Gyro_angle.y -= Gyro_angle.x * sin(GyroScaled.z * Time * deg_to_rad);

  Acc_totalVec = sqrt(pow(RawAcc.x, 2) + pow(RawAcc.y, 2) + pow(RawAcc.z, 2)) / ScaleAcc;
  Acc_angle.x = atan(RawAcc.y / sqrt(pow(RawAcc.x, 2) + pow(RawAcc.z * 0.85f, 2))) * rad_to_deg;
  Acc_angle.y = -atan(RawAcc.x / sqrt(pow(RawAcc.y, 2) + pow(RawAcc.z * 0.85f, 2))) * rad_to_deg;

  if (GyroSet) {
    Gyro_angle = Acc_angle;
    Gyro_angle.z = 0;
    GyroSet = false;
  }

  if (RawAcc.z > -100 && Acc_totalVec > 0.1f) {
    Gyro_angle.x = 0.99f * Gyro_angle.x + Acc_angle.x * 0.01f;
    Gyro_angle.y = 0.99f * Gyro_angle.y + Acc_angle.y * 0.01f;
  }

  error.x = Gyro_angle.x - target.x - cal.x;
  error.y = Gyro_angle.y - target.y - cal.y;
  error.z = Gyro_angle.z - target.z - cal.z;
}

void Gyro::calibrateGyro() {
  double x = 0;
  double y = 0;
  double z = 0;
  const int n = 1500;

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
  Vec3 temp = {0, 0, 0};
  setTarget({0, 0, 0});
  setCalibration({0, 0, 0});
  calibrateGyro();

  float sumX = 0;
  float sumY = 0;

  for (int i = 0; i < n; i++) {
    calculateError();
    sumX += error.x;
    sumY += error.y;
  }

  temp.x = sumX / n;
  temp.y = sumY / n;
  temp.z = 0;
  return temp;
}

void Gyro::zeroYaw(bool latch) {
  if (latch) {
    Gyro_angle.z = 0;
  }
}
