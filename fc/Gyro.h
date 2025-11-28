#ifndef Gyro_h
#define Gyro_h

#include <Arduino.h>

struct Vec3 {
    float x;
    float y;
    float z;
};

class Gyro {
  private:
    Vec3 gyroScaled;      // angular speed in deg/sec
    Vec3 gyroAngle;       // integrated gyro angle in deg

    Vec3 rawAcc;          // raw acceleration vector
    Vec3 rawGyro;         // raw angular velocity
    float tmp = 0.0f;

    Vec3 accAngle;        // calculated angle from accelerometer
    Vec3 target;          // requested attitude in deg/sec
    Vec3 cal;             // trim offsets in deg/sec
    Vec3 gyroCal;         // gyro calibration offsets
    float accTotalVec;    // total acceleration magnitude

    double timePerLoop = 0.0;
    double prevTime = 0.0;
    bool measureTime = false;

    static constexpr int scaleAcc = 8192;
    static constexpr int scaleGyro = 65.5;
    static constexpr float radToDeg = 180.0f / 3.141592654f;
    static constexpr float degToRad = 3.141592654f / 180.0f;
    static constexpr double microToSec = 0.000001;

    float limZ = scaleGyro / 100.0f;
    bool gyroSet = true;

    void setupWireInternal();

  public:
    Vec3 error{0, 0, 0};

    Gyro() = default;

    void zeroYaw(bool lt);
    void calculateError();
    void setTarget(Vec3 Target);
    void setCalibration(Vec3 Cal);
    void readingMPU();
    void calibrateGyro();
    Vec3 calibrate(int n);
    void calculateAngle();
    void SetupWire(double loopTimeSeconds);
    void SetupWire();
};

#endif  // Gyro_h
