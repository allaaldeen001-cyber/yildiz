/*
 * ============================================================================
 * CALIBRATION ROUTINES - IMPLEMENTATION
 * ============================================================================
 */

#include "calibration.h"

// ============================================================================
// Calibration Class Implementation
// ============================================================================

Calibration::Calibration() {
    pIMU = nullptr;
    pBaro = nullptr;
    pMotors = nullptr;
    
    calibrated = false;
    imuCalibrated = false;
    baroCalibrated = false;
    escCalibrated = false;
    
    // Initialize calibration data
    calData.gyroOffsetX = 0;
    calData.gyroOffsetY = 0;
    calData.gyroOffsetZ = 0;
    calData.accelOffsetX = 0;
    calData.accelOffsetY = 0;
    calData.accelOffsetZ = 0;
    calData.isCalibrated = false;
}

void Calibration::init(MPU6050* imu, MS5611* baro, MotorController* motors) {
    pIMU = imu;
    pBaro = baro;
    pMotors = motors;
    
    // Try to load existing calibration
    if (loadFromEEPROM()) {
        Serial.println(F("Loaded calibration from EEPROM"));
    }
}

bool Calibration::runFullCalibration() {
    Serial.println(F(""));
    Serial.println(F("╔════════════════════════════════════════════════════════╗"));
    Serial.println(F("║           FULL CALIBRATION PROCEDURE                   ║"));
    Serial.println(F("║                                                        ║"));
    Serial.println(F("║  This will calibrate:                                  ║"));
    Serial.println(F("║  1. IMU (Gyroscope + Accelerometer)                    ║"));
    Serial.println(F("║  2. Barometer baseline                                 ║"));
    Serial.println(F("║  3. ESCs (if requested)                                ║"));
    Serial.println(F("║                                                        ║"));
    Serial.println(F("║  IMPORTANT: Place drone on flat, level surface!        ║"));
    Serial.println(F("║  Keep drone completely still during calibration!       ║"));
    Serial.println(F("╚════════════════════════════════════════════════════════╝"));
    Serial.println(F(""));
    
    delay(3000);
    
    // Calibrate IMU
    if (!calibrateIMU()) {
        Serial.println(F("IMU calibration failed!"));
        return false;
    }
    
    delay(500);
    
    // Calibrate barometer
    if (!calibrateBarometer()) {
        Serial.println(F("Barometer calibration failed!"));
        return false;
    }
    
    // Ask about ESC calibration
    Serial.println(F(""));
    Serial.println(F("ESC calibration is optional (only needed once)."));
    Serial.println(F("Skipping ESC calibration..."));
    
    // Save to EEPROM
    if (!saveToEEPROM()) {
        Serial.println(F("Failed to save calibration to EEPROM!"));
        return false;
    }
    
    calibrated = true;
    
    Serial.println(F(""));
    Serial.println(F("╔════════════════════════════════════════════════════════╗"));
    Serial.println(F("║         CALIBRATION COMPLETE AND SAVED!                ║"));
    Serial.println(F("╚════════════════════════════════════════════════════════╝"));
    Serial.println(F(""));
    
    return true;
}

bool Calibration::calibrateIMU() {
    if (pIMU == nullptr) {
        Serial.println(F("ERROR: IMU not initialized!"));
        return false;
    }
    
    printCalibrationHeader("IMU CALIBRATION");
    
    Serial.println(F("Place drone on flat surface and keep STILL!"));
    Serial.println(F("Calibrating in 3 seconds..."));
    delay(3000);
    
    // Calibrate gyroscope
    Serial.println(F("\n[1/2] Calibrating Gyroscope..."));
    pIMU->calibrateGyro(CALIBRATION_SAMPLES);
    
    // Calibrate accelerometer
    Serial.println(F("\n[2/2] Calibrating Accelerometer..."));
    pIMU->calibrateAccel();
    
    // Store calibration data
    calData.gyroOffsetX = pIMU->getGyroOffsetX();
    calData.gyroOffsetY = pIMU->getGyroOffsetY();
    calData.gyroOffsetZ = pIMU->getGyroOffsetZ();
    calData.accelOffsetX = pIMU->getAccelOffsetX();
    calData.accelOffsetY = pIMU->getAccelOffsetY();
    calData.accelOffsetZ = pIMU->getAccelOffsetZ();
    
    imuCalibrated = true;
    printCalibrationResult(true, "IMU calibration successful!");
    
    return true;
}

bool Calibration::calibrateBarometer() {
    if (pBaro == nullptr) {
        Serial.println(F("ERROR: Barometer not initialized!"));
        return false;
    }
    
    printCalibrationHeader("BAROMETER CALIBRATION");
    
    Serial.println(F("Calibrating pressure baseline..."));
    Serial.println(F("This sets current position as zero altitude."));
    
    pBaro->calibrateBaseline(100);
    
    baroCalibrated = true;
    printCalibrationResult(true, "Barometer calibration successful!");
    
    Serial.print(F("Baseline: "));
    Serial.print(pBaro->getBaseline());
    Serial.println(F(" hPa"));
    
    return true;
}

bool Calibration::calibrateESCs() {
    if (pMotors == nullptr) {
        Serial.println(F("ERROR: Motors not initialized!"));
        return false;
    }
    
    printCalibrationHeader("ESC CALIBRATION");
    
    pMotors->calibrateESCs();
    
    escCalibrated = true;
    printCalibrationResult(true, "ESC calibration successful!");
    
    return true;
}

bool Calibration::saveToEEPROM() {
    Serial.print(F("Saving calibration to EEPROM... "));
    
    // Write signature
    EEPROM.write(EEPROM_ADDR_SIGNATURE, EEPROM_SIGNATURE);
    
    // Write gyro offsets
    EEPROM.put(EEPROM_ADDR_GYRO_X, calData.gyroOffsetX);
    EEPROM.put(EEPROM_ADDR_GYRO_Y, calData.gyroOffsetY);
    EEPROM.put(EEPROM_ADDR_GYRO_Z, calData.gyroOffsetZ);
    
    // Write accel offsets
    EEPROM.put(EEPROM_ADDR_ACCEL_X, calData.accelOffsetX);
    EEPROM.put(EEPROM_ADDR_ACCEL_Y, calData.accelOffsetY);
    EEPROM.put(EEPROM_ADDR_ACCEL_Z, calData.accelOffsetZ);
    
    // Write ESC calibration flag
    EEPROM.write(EEPROM_ADDR_ESC_CAL, escCalibrated ? 1 : 0);
    
    Serial.println(F("OK!"));
    return true;
}

bool Calibration::loadFromEEPROM() {
    // Check signature
    if (!isEEPROMValid()) {
        Serial.println(F("No valid calibration data in EEPROM"));
        return false;
    }
    
    Serial.print(F("Loading calibration from EEPROM... "));
    
    // Read gyro offsets
    EEPROM.get(EEPROM_ADDR_GYRO_X, calData.gyroOffsetX);
    EEPROM.get(EEPROM_ADDR_GYRO_Y, calData.gyroOffsetY);
    EEPROM.get(EEPROM_ADDR_GYRO_Z, calData.gyroOffsetZ);
    
    // Read accel offsets
    EEPROM.get(EEPROM_ADDR_ACCEL_X, calData.accelOffsetX);
    EEPROM.get(EEPROM_ADDR_ACCEL_Y, calData.accelOffsetY);
    EEPROM.get(EEPROM_ADDR_ACCEL_Z, calData.accelOffsetZ);
    
    // Read ESC calibration flag
    escCalibrated = (EEPROM.read(EEPROM_ADDR_ESC_CAL) == 1);
    
    // Apply to IMU if available
    if (pIMU != nullptr) {
        pIMU->setGyroOffsets(calData.gyroOffsetX, calData.gyroOffsetY, calData.gyroOffsetZ);
        pIMU->setAccelOffsets(calData.accelOffsetX, calData.accelOffsetY, calData.accelOffsetZ);
    }
    
    imuCalibrated = true;
    calData.isCalibrated = true;
    calibrated = true;
    
    Serial.println(F("OK!"));
    
    // Print loaded values
    Serial.println(F("Loaded calibration values:"));
    Serial.print(F("  Gyro offsets: X="));
    Serial.print(calData.gyroOffsetX);
    Serial.print(F(" Y="));
    Serial.print(calData.gyroOffsetY);
    Serial.print(F(" Z="));
    Serial.println(calData.gyroOffsetZ);
    Serial.print(F("  Accel offsets: X="));
    Serial.print(calData.accelOffsetX);
    Serial.print(F(" Y="));
    Serial.print(calData.accelOffsetY);
    Serial.print(F(" Z="));
    Serial.println(calData.accelOffsetZ);
    
    return true;
}

bool Calibration::isEEPROMValid() {
    return (EEPROM.read(EEPROM_ADDR_SIGNATURE) == EEPROM_SIGNATURE);
}

void Calibration::clearEEPROM() {
    Serial.print(F("Clearing EEPROM calibration data... "));
    EEPROM.write(EEPROM_ADDR_SIGNATURE, 0xFF);
    Serial.println(F("Done"));
}

void Calibration::printJoystickCalibrationGuide() {
    Serial.println(F(""));
    Serial.println(F("╔════════════════════════════════════════════════════════╗"));
    Serial.println(F("║           JOYSTICK CALIBRATION GUIDE                   ║"));
    Serial.println(F("╠════════════════════════════════════════════════════════╣"));
    Serial.println(F("║                                                        ║"));
    Serial.println(F("║  The joystick calibration solves the dangerous         ║"));
    Serial.println(F("║  center-throttle problem:                              ║"));
    Serial.println(F("║                                                        ║"));
    Serial.println(F("║  PROBLEM: If throttle range is 1000-2000,              ║"));
    Serial.println(F("║           center position = 1500 = 50% throttle!       ║"));
    Serial.println(F("║           This is DANGEROUS - drone would fly!         ║"));
    Serial.println(F("║                                                        ║"));
    Serial.println(F("║  SOLUTION: Calibration maps joystick so that:          ║"));
    Serial.println(F("║           - Stick DOWN (rest position) = 1000          ║"));
    Serial.println(F("║           - Stick UP (full push) = 2000                ║"));
    Serial.println(F("║                                                        ║"));
    Serial.println(F("║  This ensures drone is safe when sticks are released!  ║"));
    Serial.println(F("║                                                        ║"));
    Serial.println(F("╚════════════════════════════════════════════════════════╝"));
    Serial.println(F(""));
}

void Calibration::printProgress(int current, int total, const char* message) {
    Serial.print(message);
    Serial.print(F(" ["));
    int progress = (current * 20) / total;
    for (int i = 0; i < 20; i++) {
        Serial.print(i < progress ? '#' : '-');
    }
    Serial.print(F("] "));
    Serial.print((current * 100) / total);
    Serial.println(F("%"));
}

void Calibration::printCalibrationHeader(const char* title) {
    Serial.println(F(""));
    Serial.println(F("─────────────────────────────────────────"));
    Serial.print(F("  "));
    Serial.println(title);
    Serial.println(F("─────────────────────────────────────────"));
}

void Calibration::printCalibrationResult(bool success, const char* message) {
    Serial.println(F(""));
    if (success) {
        Serial.print(F("✓ "));
    } else {
        Serial.print(F("✗ "));
    }
    Serial.println(message);
}

// ============================================================================
// JoystickCalibrator Implementation
// ============================================================================

JoystickCalibrator::JoystickCalibrator() {
    calibrated = false;
    
    // Default values (typical potentiometer range)
    cal.leftYMin = 0;
    cal.leftYMax = 1023;
    cal.leftYCenter = 512;
    cal.leftXMin = 0;
    cal.leftXMax = 1023;
    cal.leftXCenter = 512;
    
    cal.rightYMin = 0;
    cal.rightYMax = 1023;
    cal.rightYCenter = 512;
    cal.rightXMin = 0;
    cal.rightXMax = 1023;
    cal.rightXCenter = 512;
    
    cal.signature = 0;
}

bool JoystickCalibrator::runCalibration(uint8_t throttlePin, uint8_t yawPin,
                                        uint8_t pitchPin, uint8_t rollPin) {
    Serial.println(F(""));
    Serial.println(F("╔════════════════════════════════════════════════════════╗"));
    Serial.println(F("║           JOYSTICK CALIBRATION                         ║"));
    Serial.println(F("╚════════════════════════════════════════════════════════╝"));
    Serial.println(F(""));
    
    // Initialize with center values
    uint16_t readings[4];
    
    // Step 1: Find centers (release sticks)
    Serial.println(F("Step 1: Release all joysticks to center position"));
    Serial.println(F("        (Let them return to rest position)"));
    Serial.println(F("        Waiting 3 seconds..."));
    delay(3000);
    
    // Read center values
    cal.leftYCenter = analogRead(throttlePin);
    cal.leftXCenter = analogRead(yawPin);
    cal.rightYCenter = analogRead(pitchPin);
    cal.rightXCenter = analogRead(rollPin);
    
    Serial.println(F("Centers recorded!"));
    Serial.print(F("  Throttle center: ")); Serial.println(cal.leftYCenter);
    Serial.print(F("  Yaw center: ")); Serial.println(cal.leftXCenter);
    Serial.print(F("  Pitch center: ")); Serial.println(cal.rightYCenter);
    Serial.print(F("  Roll center: ")); Serial.println(cal.rightXCenter);
    
    // Initialize min/max with center values
    cal.leftYMin = cal.leftYMax = cal.leftYCenter;
    cal.leftXMin = cal.leftXMax = cal.leftXCenter;
    cal.rightYMin = cal.rightYMax = cal.rightYCenter;
    cal.rightXMin = cal.rightXMax = cal.rightXCenter;
    
    // Step 2: Find extremes
    Serial.println(F(""));
    Serial.println(F("Step 2: Move all joysticks to their EXTREME positions"));
    Serial.println(F("        Move each stick: UP, DOWN, LEFT, RIGHT"));
    Serial.println(F("        You have 10 seconds..."));
    Serial.println(F(""));
    
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
        // Read current values
        readings[0] = analogRead(throttlePin);
        readings[1] = analogRead(yawPin);
        readings[2] = analogRead(pitchPin);
        readings[3] = analogRead(rollPin);
        
        // Update min/max for throttle
        if (readings[0] < cal.leftYMin) cal.leftYMin = readings[0];
        if (readings[0] > cal.leftYMax) cal.leftYMax = readings[0];
        
        // Update min/max for yaw
        if (readings[1] < cal.leftXMin) cal.leftXMin = readings[1];
        if (readings[1] > cal.leftXMax) cal.leftXMax = readings[1];
        
        // Update min/max for pitch
        if (readings[2] < cal.rightYMin) cal.rightYMin = readings[2];
        if (readings[2] > cal.rightYMax) cal.rightYMax = readings[2];
        
        // Update min/max for roll
        if (readings[3] < cal.rightXMin) cal.rightXMin = readings[3];
        if (readings[3] > cal.rightXMax) cal.rightXMax = readings[3];
        
        // Progress indicator
        if ((millis() - startTime) % 1000 < 50) {
            Serial.print(F("."));
        }
        
        delay(10);
    }
    
    Serial.println(F(" Done!"));
    Serial.println(F(""));
    Serial.println(F("Calibration Results:"));
    Serial.println(F("────────────────────────────────────────"));
    Serial.print(F("Throttle: Min=")); Serial.print(cal.leftYMin);
    Serial.print(F(" Center=")); Serial.print(cal.leftYCenter);
    Serial.print(F(" Max=")); Serial.println(cal.leftYMax);
    
    Serial.print(F("Yaw:      Min=")); Serial.print(cal.leftXMin);
    Serial.print(F(" Center=")); Serial.print(cal.leftXCenter);
    Serial.print(F(" Max=")); Serial.println(cal.leftXMax);
    
    Serial.print(F("Pitch:    Min=")); Serial.print(cal.rightYMin);
    Serial.print(F(" Center=")); Serial.print(cal.rightYCenter);
    Serial.print(F(" Max=")); Serial.println(cal.rightYMax);
    
    Serial.print(F("Roll:     Min=")); Serial.print(cal.rightXMin);
    Serial.print(F(" Center=")); Serial.print(cal.rightXCenter);
    Serial.print(F(" Max=")); Serial.println(cal.rightXMax);
    
    // Validate calibration
    bool valid = true;
    if (cal.leftYMax - cal.leftYMin < 300) {
        Serial.println(F("WARNING: Throttle range too small!"));
        valid = false;
    }
    
    cal.signature = 0x4A;  // 'J' for joystick
    calibrated = valid;
    
    if (valid) {
        Serial.println(F(""));
        Serial.println(F("✓ Joystick calibration successful!"));
    }
    
    return valid;
}

uint16_t JoystickCalibrator::applyThrottleCalibration(uint16_t raw) {
    // Map from raw ADC to 1000-2000 range
    // IMPORTANT: Throttle maps from bottom (min) to top (max)
    // When stick is at rest (down), output should be 1000 (motors off)
    return mapJoystick(raw, cal.leftYMin, cal.leftYCenter, cal.leftYMax, 
                       THROTTLE_MIN, THROTTLE_MAX);
}

uint16_t JoystickCalibrator::applyYawCalibration(uint16_t raw) {
    // Map from raw ADC to 1000-2000 range (center = 1500)
    return mapJoystick(raw, cal.leftXMin, cal.leftXCenter, cal.leftXMax,
                       STICK_MIN, STICK_MAX);
}

uint16_t JoystickCalibrator::applyPitchCalibration(uint16_t raw) {
    return mapJoystick(raw, cal.rightYMin, cal.rightYCenter, cal.rightYMax,
                       STICK_MIN, STICK_MAX);
}

uint16_t JoystickCalibrator::applyRollCalibration(uint16_t raw) {
    return mapJoystick(raw, cal.rightXMin, cal.rightXCenter, cal.rightXMax,
                       STICK_MIN, STICK_MAX);
}

uint16_t JoystickCalibrator::mapJoystick(uint16_t raw, uint16_t inMin, uint16_t inCenter,
                                         uint16_t inMax, uint16_t outMin, uint16_t outMax) {
    uint16_t outCenter = (outMin + outMax) / 2;
    
    if (raw < inCenter) {
        // Map lower half
        return map(raw, inMin, inCenter, outMin, outCenter);
    } else {
        // Map upper half
        return map(raw, inCenter, inMax, outCenter, outMax);
    }
}

bool JoystickCalibrator::saveToEEPROM() {
    EEPROM.put(EEPROM_JOY_ADDR, cal);
    Serial.println(F("Joystick calibration saved to EEPROM"));
    return true;
}

bool JoystickCalibrator::loadFromEEPROM() {
    JoystickCalibration temp;
    EEPROM.get(EEPROM_JOY_ADDR, temp);
    
    if (temp.signature == 0x4A) {
        cal = temp;
        calibrated = true;
        Serial.println(F("Joystick calibration loaded from EEPROM"));
        return true;
    }
    
    Serial.println(F("No valid joystick calibration in EEPROM"));
    return false;
}
