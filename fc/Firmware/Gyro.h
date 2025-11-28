#ifndef Gyro_h
#define Gyro_h

#include <Arduino.h>

struct Vec3 {
  float x;
  float y;
  float z;
};

class Gyro {
public:
  Gyro() = default;

  Vec3 error{0, 0, 0};

  void zeroYaw(bool latchTop);
  void calculateError();
  void setTarget(Vec3 Target);
  void setCalibration(Vec3 Cal);
  void readingMPU();
  void calibrateGyro();
  Vec3 calibrate(int samples);
  void calculateAngle();
  void SetupWire(double iterationTime);
  void SetupWire();

private:
  void setupwire();

  Vec3 GyroScaled{0, 0, 0};
  Vec3 Gyro_angle{0, 0, 0};
  Vec3 RawAcc{0, 0, 0};
  Vec3 RawGyro{0, 0, 0};
  Vec3 Acc_angle{0, 0, 0};
  Vec3 target{0, 0, 0};
  Vec3 cal{0, 0, 0};
  Vec3 GyroCal{0, 0, 0};
  float Acc_totalVec = 0.0f;
  float tmp = 0.0f;

  double Time = 0.0;
  double prevTime = 0.0;
  bool countTime = false;

  static constexpr int ScaleAcc = 8192;      // Datasheet value
  static constexpr int ScaleGyro = 65.5;     // Datasheet value
  static constexpr float rad_to_deg = 180.0f / 3.141592654f;
  static constexpr float deg_to_rad = 3.141592654f / 180.0f;
  static constexpr double micro_to_sec = 0.000001;
  const float limZ = ScaleGyro / 100.0f;

  bool GyroSet = true;
};

#endif
