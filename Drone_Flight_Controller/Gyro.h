#ifndef Gyro_h
#define Gyro_h

#include "Arduino.h"

struct Vec3
{
  float x, y, z;
};

class Gyro
{
private:
  Vec3 GyroScaled;      // Angular speed in deg/sec
  Vec3 Gyro_angle;      // Gyro integral in deg
  Vec3 RawAcc;          // Raw acceleration values
  Vec3 RawGyro;         // Raw gyro values
  float tmp = 0;
  
  Vec3 Acc_angle;       // Acceleration angle in deg
  Vec3 target;          // Target angle in deg
  Vec3 cal;             // Calibration offset in deg
  Vec3 GyroCal;         // Gyro calibration values
  float Acc_totalVec;   // Total acceleration vector magnitude
  
  // Timing
  double Time = 0;
  double prevTime = 0;
  bool countTime = false;
  
  // Scale factors
  const float ScaleAcc = 4096.0;     // For ±8g range
  const float ScaleGyro = 65.5;      // For ±500°/s range
  const float rad_to_deg = 180.0 / 3.141592654;
  const float deg_to_rad = 3.141592654 / 180.0;
  const double micro_to_sec = 0.000001;
  float limZ = ScaleGyro / 100;      // Low pass for Z axis
  
  bool GyroSet = true;

public:
  Gyro() {}
  
  Vec3 error;
  
  // Function prototypes
  void zeroYaw(bool lt);
  void calculateError();
  void setTarget(Vec3 Target);
  void setCalibration(Vec3 Cal);
  void readingMPU();
  void calibrateGyro();
  Vec3 calibrate(int n);
  void calculateAngle();
  void SetupWire(double TIME);
  void SetupWire();
  
private:
  void setupwire();
};

#endif
