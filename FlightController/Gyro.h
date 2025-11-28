/*
 * Gyro.h
 * ======
 * MPU6050 Gyroscope/Accelerometer driver
 * Provides attitude estimation using complementary filter
 */

#ifndef Gyro_h
#define Gyro_h

#include "Arduino.h"

// 3D Vector structure
struct Vec3 {
    float x, y, z;
};

class Gyro {
private:
    // Scaled sensor values
    Vec3 GyroScaled;     // Angular speed in deg/sec
    Vec3 Gyro_angle;     // Integrated gyro angle in deg
    
    // Raw sensor values
    Vec3 RawAcc;         // Raw accelerometer
    Vec3 RawGyro;        // Raw gyroscope
    float tmp = 0;       // Temperature (unused)
    
    // Calculated angles
    Vec3 Acc_angle;      // Acceleration-derived angle in deg
    Vec3 target;         // Target angle in deg
    Vec3 cal;            // Calibration offset in deg
    Vec3 GyroCal;        // Gyroscope calibration offset
    float Acc_totalVec;  // Total acceleration vector magnitude
    
    // Timing
    double Time = 0;
    double prevTime = 0;
    bool countTime = false;
    
    // Scale factors (from MPU6050 datasheet)
    const int ScaleAcc = 8192;              // ±4g = 8192 LSB/g
    const float ScaleGyro = 65.5;           // ±500°/s = 65.5 LSB/(°/s)
    const float rad_to_deg = 180.0 / 3.141592654;
    const float deg_to_rad = 3.141592654 / 180.0;
    const double micro_to_sec = 0.000001;
    float limZ;                             // Z-axis low pass threshold
    
    bool GyroSet = true;                    // Flag for first iteration
    
public:
    // Constructor
    Gyro() {
        limZ = ScaleGyro / 100;
    }
    
    // Current error (difference between actual and target)
    Vec3 error;
    
    // Public methods
    void zeroYaw(bool reset);
    void calculateError();
    void setTarget(Vec3 Target);
    void setCalibration(Vec3 Cal);
    void readingMPU();
    void calibrateGyro();
    Vec3 calibrate(int n);
    void calculateAngle();
    void SetupWire(double TIME);    // Setup with fixed time period
    void SetupWire();               // Setup with auto timing
    
private:
    void setupwire();               // Internal I2C setup
};

#endif
