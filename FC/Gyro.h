#ifndef Gyro_h
#define Gyro_h

#include "Arduino.h"

struct Vec3 {
  float x, y, z;
};

class Gyro {
private:
  Vec3 GyroScaled;      // Angular speed in deg/sec
  Vec3 Gyro_angle;      // Gyro integral in deg
  Vec3 RawAcc;          // Raw acceleration values
  Vec3 RawGyro;         // Raw angular velocity
  float tmp = 0;        // Temporary variable
  
  Vec3 Acc_angle;       // Acceleration angle in deg
  Vec3 target;          // Target angle in deg
  Vec3 cal;             // Calibration angle in deg
  Vec3 GyroCal;         // Calibration for raw gyro values
  float Acc_totalVec;   // Total vector of raw acceleration
  
  // Time per iteration
  double Time = 0;
  double prevTime = 0;
  bool countTime = false;
  
  // Scale factors
  const int ScaleAcc = 8192;                  // Datasheet
  const int ScaleGyro = 65.5;                // Datasheet
  const float rad_to_deg = 180 / 3.141592654;
  const float deg_to_rad = 3.141592654 / 180;
  double micro_to_sec = 0.000001;
  float limZ = ScaleGyro / 100;              // Low pass filter for Gyro Z
  
  bool GyroSet = true;                       // Flag for first iteration

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
  void setupwire();
};

#endif
