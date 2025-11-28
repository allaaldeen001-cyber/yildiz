#include "Gyro.h"

#include <Wire.h>

void Gyro::setupWireInternal() {
    Wire.begin();
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);  // PWR_MGMT_1
    Wire.write(0x00);
    Wire.endTransmission(true);

    Wire.beginTransmission(0x68);
    Wire.write(0x1B);  // GYRO_CONFIG
    Wire.write(0x08);  // FS_SEL = 1 (65.5 LSB/deg/sec)
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1C);  // ACCEL_CONFIG
    Wire.write(0x10);  // AFS_SEL = 0x10 => 4096 LSB/g
    Wire.endTransmission();

    delay(100);
}

void Gyro::SetupWire(double loopTimeSeconds) {
    measureTime = false;
    timePerLoop = loopTimeSeconds;
    setupWireInternal();
    calibrateGyro();
}

void Gyro::SetupWire() {
    measureTime = true;
    setupWireInternal();
    calibrateGyro();
}

void Gyro::calculateError() {
    if (measureTime) {
        timePerLoop = micros() - prevTime;
        timePerLoop *= microToSec;
        prevTime = micros();
    }

    readingMPU();
    calculateAngle();
}

void Gyro::setTarget(Vec3 Target) { target = Target; }

void Gyro::setCalibration(Vec3 Cal) { cal = Cal; }

void Gyro::readingMPU() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);

    rawAcc.x = Wire.read() << 8 | Wire.read();
    rawAcc.y = Wire.read() << 8 | Wire.read();
    rawAcc.z = Wire.read() << 8 | Wire.read();

    tmp = Wire.read() << 8 | Wire.read();

    rawGyro.x = Wire.read() << 8 | Wire.read();
    rawGyro.y = Wire.read() << 8 | Wire.read();
    rawGyro.z = Wire.read() << 8 | Wire.read();
}

void Gyro::calculateAngle() {
    gyroScaled.x = (rawGyro.x - gyroCal.x) / scaleGyro;
    gyroScaled.y = (rawGyro.y - gyroCal.y) / scaleGyro;

    const float rawZ = rawGyro.z - gyroCal.z;
    if (rawZ < limZ && rawZ > -limZ) {
        gyroScaled.z = 0;
    } else {
        gyroScaled.z = rawZ / scaleGyro;
    }

    gyroAngle.x += gyroScaled.x * timePerLoop;
    gyroAngle.y += gyroScaled.y * timePerLoop;
    gyroAngle.z += gyroScaled.z * timePerLoop;

    gyroAngle.x += gyroAngle.y * sinf(gyroScaled.z * timePerLoop * degToRad);
    gyroAngle.y -= gyroAngle.x * sinf(gyroScaled.z * timePerLoop * degToRad);

    accTotalVec = sqrt(pow(rawAcc.x, 2) + pow(rawAcc.y, 2) + pow(rawAcc.z, 2)) / scaleAcc;

    accAngle.x = atanf(rawAcc.y / sqrt(pow(rawAcc.x, 2) + pow(rawAcc.z * 0.85f, 2))) * radToDeg;
    accAngle.y = -atanf(rawAcc.x / sqrt(pow(rawAcc.y, 2) + pow(rawAcc.z * 0.85f, 2))) * radToDeg;

    if (gyroSet) {
        gyroAngle = accAngle;
        gyroAngle.z = 0;
        gyroSet = false;
    }

    if (rawAcc.z > -100 && accTotalVec > 0.1f) {
        gyroAngle.x = 0.99f * gyroAngle.x + 0.01f * accAngle.x;
        gyroAngle.y = 0.99f * gyroAngle.y + 0.01f * accAngle.y;
    }

    error.x = gyroAngle.x - target.x - cal.x;
    error.y = gyroAngle.y - target.y - cal.y;
    error.z = gyroAngle.z - target.z - cal.z;
}

void Gyro::calibrateGyro() {
    double x = 0;
    double y = 0;
    double z = 0;

    constexpr int samples = 1500;
    for (int i = 0; i < samples; i++) {
        readingMPU();
        x += rawGyro.x;
        y += rawGyro.y;
        z += rawGyro.z;
    }

    delay(100);

    gyroCal.x = x / samples;
    gyroCal.y = y / samples;
    gyroCal.z = z / samples;
}

Vec3 Gyro::calibrate(int n) {
    float tempX = 0;
    float tempY = 0;
    Vec3 temp{0, 0, 0};

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
    temp.z = 0;

    return temp;
}

void Gyro::zeroYaw(bool lt) {
    if (lt) {
        gyroAngle.z = 0;
    }
}
