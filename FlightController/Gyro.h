/**
 * ============================================================================
 * GYRO.H - MPU6050 Gyroscope/Accelerometer Interface
 * ============================================================================
 * 
 * This library provides:
 * - MPU6050 initialization and configuration
 * - Gyroscope and accelerometer reading
 * - Complementary filter for angle estimation
 * - Gyro drift calibration
 * 
 * ============================================================================
 */

#ifndef Gyro_h
#define Gyro_h

#include "Arduino.h"

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * 3D Vector structure for angles, rates, and errors
 */
struct Vec3 {
  float x;
  float y;
  float z;
};

// ============================================================================
// GYRO CLASS
// ============================================================================

class Gyro {
  
private:
  // Scaled sensor values
  Vec3 GyroScaled;      // Angular velocity in deg/sec
  Vec3 Gyro_angle;      // Integrated gyro angle in degrees
  
  // Raw sensor readings
  Vec3 RawAcc;          // Raw accelerometer values
  Vec3 RawGyro;         // Raw gyroscope values
  float tmp;            // Temperature (unused)
  
  // Calculated angles
  Vec3 Acc_angle;       // Accelerometer-derived angle in degrees
  
  // Target and calibration
  Vec3 target;          // Target angle in degrees
  Vec3 cal;             // Calibration offset in degrees
  Vec3 GyroCal;         // Gyro bias calibration values
  
  // Total acceleration vector magnitude
  float Acc_totalVec;
  
  // Timing
  double Time;          // Time per iteration in seconds
  double prevTime;      // Previous time in microseconds
  bool countTime;       // Flag to determine timing mode
  
  // MPU6050 scaling constants (from datasheet)
  static const int ScaleAcc = 8192;              // LSB/g at ±4g range
  static const int ScaleGyro = 65.5;             // LSB/(deg/s) at ±500 deg/s
  
  // Conversion constants
  static constexpr float rad_to_deg = 180.0 / 3.141592654;
  static constexpr float deg_to_rad = 3.141592654 / 180.0;
  static constexpr double micro_to_sec = 0.000001;
  
  // Low pass filter threshold for Z axis gyro
  float limZ;
  
  // First iteration flag for angle initialization
  bool GyroSet;
  
  // Private initialization helper
  void setupwire();

public:
  // Constructor
  Gyro();
  
  // Public error vector (accessible by main program)
  Vec3 error;
  
  // =========================================================================
  // INITIALIZATION
  // =========================================================================
  
  /**
   * Initialize with fixed time step
   * @param TIME Time per iteration in seconds (1/frequency)
   */
  void SetupWire(double TIME);
  
  /**
   * Initialize with automatic time measurement
   */
  void SetupWire();
  
  // =========================================================================
  // MAIN INTERFACE
  // =========================================================================
  
  /**
   * Read sensors and calculate error
   * Call this every loop iteration
   */
  void calculateError();
  
  /**
   * Set target angles
   * @param Target Target angles {x, y, z} in degrees
   */
  void setTarget(Vec3 Target);
  
  /**
   * Set calibration offsets
   * @param Cal Calibration offsets {x, y, z} in degrees
   */
  void setCalibration(Vec3 Cal);
  
  /**
   * Perform gyro bias calibration
   * Drone must be stationary during this process
   */
  void calibrateGyro();
  
  /**
   * Perform full calibration and return offsets
   * @param n Number of samples to average
   * @return Calibration offset vector
   */
  Vec3 calibrate(int n);
  
  /**
   * Reset yaw angle to zero
   * @param lt If true, reset yaw
   */
  void zeroYaw(bool lt);
  
  // =========================================================================
  // INTERNAL METHODS
  // =========================================================================
  
  /**
   * Read raw values from MPU6050
   */
  void readingMPU();
  
  /**
   * Calculate angles from raw data
   */
  void calculateAngle();
};

#endif
