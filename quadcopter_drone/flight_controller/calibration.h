/*
 * ============================================================================
 * CALIBRATION ROUTINES
 * ============================================================================
 * Handles calibration for IMU, barometer, ESCs, and joysticks
 * Saves calibration data to EEPROM
 * ============================================================================
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "motors.h"

class Calibration {
public:
    // Constructor
    Calibration();
    
    // Initialize with sensor references
    void init(MPU6050* imu, MS5611* baro, MotorController* motors);
    
    // Full calibration routine (all sensors)
    bool runFullCalibration();
    
    // Individual calibrations
    bool calibrateIMU();
    bool calibrateBarometer();
    bool calibrateESCs();
    
    // EEPROM operations
    bool saveToEEPROM();
    bool loadFromEEPROM();
    bool isEEPROMValid();
    void clearEEPROM();
    
    // Joystick calibration (stored on RC side)
    void printJoystickCalibrationGuide();
    
    // Status
    bool isCalibrated() { return calibrated; }
    bool isIMUCalibrated() { return imuCalibrated; }
    bool isBaroCalibrated() { return baroCalibrated; }
    bool isESCCalibrated() { return escCalibrated; }
    
    // Get calibration data
    CalibrationData getCalibrationData() { return calData; }

private:
    // Sensor references
    MPU6050* pIMU;
    MS5611* pBaro;
    MotorController* pMotors;
    
    // Calibration data
    CalibrationData calData;
    
    // Status flags
    bool calibrated;
    bool imuCalibrated;
    bool baroCalibrated;
    bool escCalibrated;
    
    // Print calibration progress
    void printProgress(int current, int total, const char* message);
    void printCalibrationHeader(const char* title);
    void printCalibrationResult(bool success, const char* message);
};

// ============================================================================
// JOYSTICK CALIBRATION DATA (stored on RC transmitter)
// ============================================================================

struct JoystickCalibration {
    // Left joystick (Throttle/Yaw)
    uint16_t leftYMin;      // Throttle min (stick down)
    uint16_t leftYMax;      // Throttle max (stick up)
    uint16_t leftYCenter;   // Throttle center (for centering)
    uint16_t leftXMin;      // Yaw left
    uint16_t leftXMax;      // Yaw right
    uint16_t leftXCenter;   // Yaw center
    
    // Right joystick (Pitch/Roll)
    uint16_t rightYMin;     // Pitch back
    uint16_t rightYMax;     // Pitch forward
    uint16_t rightYCenter;  // Pitch center
    uint16_t rightXMin;     // Roll left
    uint16_t rightXMax;     // Roll right
    uint16_t rightXCenter;  // Roll center
    
    // Calibration valid flag
    uint8_t signature;
};

// ============================================================================
// Joystick calibration helper class (for RC transmitter)
// ============================================================================

class JoystickCalibrator {
public:
    JoystickCalibrator();
    
    // Calibration routine
    bool runCalibration(uint8_t throttlePin, uint8_t yawPin,
                        uint8_t pitchPin, uint8_t rollPin);
    
    // Apply calibration to raw values
    uint16_t applyThrottleCalibration(uint16_t raw);
    uint16_t applyYawCalibration(uint16_t raw);
    uint16_t applyPitchCalibration(uint16_t raw);
    uint16_t applyRollCalibration(uint16_t raw);
    
    // EEPROM operations
    bool saveToEEPROM();
    bool loadFromEEPROM();
    bool isCalibrated() { return calibrated; }
    
    // Get calibration data
    JoystickCalibration getCalibration() { return cal; }
    
private:
    JoystickCalibration cal;
    bool calibrated;
    
    // Map function that handles inverted ranges
    uint16_t mapJoystick(uint16_t raw, uint16_t inMin, uint16_t inCenter, 
                         uint16_t inMax, uint16_t outMin, uint16_t outMax);
    
    // EEPROM addresses for joystick cal (on RC)
    static const int EEPROM_JOY_ADDR = 100;
};

#endif // CALIBRATION_H
