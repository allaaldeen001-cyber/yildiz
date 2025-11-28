/*
 * Gyro.h - MPU6050 Gyroscope/Accelerometer Library
 * 
 * Provides angle calculation using complementary filter
 * for drone flight control applications.
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
  // Scaled sensor data
  Vec3 GyroScaled;      // Angular velocity in deg/sec
  Vec3 Gyro_angle;      // Integrated gyro angle in degrees

  // Raw sensor readings
  Vec3 RawAcc;          // Raw accelerometer values
  Vec3 RawGyro;         // Raw gyroscope values
  float tmp;            // Temperature (unused)

  // Calculated angles
  Vec3 Acc_angle;       // Accelerometer-based angle in degrees
  Vec3 target;          // Target angle in degrees
  Vec3 cal;             // Calibration offset in degrees
  Vec3 GyroCal;         // Raw gyro calibration values
  float Acc_totalVec;   // Total acceleration vector magnitude

  // Timing
  double Time;          // Time per iteration in seconds
  double prevTime;      // Previous timestamp
  bool countTime;       // Auto-count time flag

  // Scaling constants
  static const int ScaleAcc = 8192;                     // LSB/g (AFS_SEL=1)
  static const int ScaleGyro = 65.5;                    // LSB/(deg/s) (FS_SEL=1)
  static constexpr float rad_to_deg = 180.0 / 3.141592654;
  static constexpr float deg_to_rad = 3.141592654 / 180.0;
  static constexpr double micro_to_sec = 0.000001;
  
  // Low pass filter threshold for Z axis
  float limZ;

  // First iteration flag
  bool GyroSet;

  // Private helper functions
  void setupwire();

public:
  // Constructor
  Gyro();

  // Current error (difference between actual and target angle)
  Vec3 error;

  // Public methods
  void zeroYaw(bool reset);
  void calculateError();
  void setTarget(Vec3 Target);
  void setCalibration(Vec3 Cal);
  void readingMPU();
  void calibrateGyro();
  Vec3 calibrate(int samples);
  void calculateAngle();
  void SetupWire(double TIME);  // With fixed time step
  void SetupWire();             // With auto time measurement
};

#endif
