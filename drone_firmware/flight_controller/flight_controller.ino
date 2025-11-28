/**
 * ============================================================================
 * PROFESSIONAL DRONE FLIGHT CONTROLLER FIRMWARE
 * ============================================================================
 * 
 * Complete flight control system for Arduino Nano quadcopter
 * 
 * Hardware:
 * - Arduino Nano (ATmega328P)
 * - NRF24L01 PA+LNA (CE→D4, CSN→D10)
 * - MPU6050 IMU (I2C, INT→D2)
 * - MS5611 Barometer (I2C)
 * - 4x ESC (D3, D5, D6, D9)
 * - Buzzer (D8)
 * - Status LED (D7)
 * 
 * Features:
 * - Mahony AHRS for attitude estimation (400 Hz)
 * - Cascade PID control (angle → rate)
 * - Altitude hold with barometer fusion
 * - NRF24L01 bidirectional communication
 * - Comprehensive safety features
 * 
 * Author: UAV Firmware Engineer
 * Version: 1.0.0
 * 
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

// Include all modules
#include "config.h"
#include "../shared/protocol.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "ahrs.h"
#include "pid.h"
#include "motors.h"
#include "altitude.h"
#include "nrf_comm.h"
#include "buzzer.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Sensors
MPU6050Driver imu;
MS5611Driver baro;

// Attitude and altitude estimation
AHRS ahrs;
AltitudeEstimator altEstimator;

// Controllers
CascadePID rollController;
CascadePID pitchController;
PIDController yawRateController;
AltitudeHoldController altHoldController;

// Outputs
MotorController motors;

// Communication
NRFComm nrf;

// Status
BuzzerController buzzer;

// ============================================================================
// STATE VARIABLES
// ============================================================================

// System state
enum SystemState {
    STATE_INIT,
    STATE_PREFLIGHT,
    STATE_ARMED,
    STATE_FLYING,
    STATE_FAILSAFE,
    STATE_ERROR
};

SystemState systemState = STATE_INIT;

// Calibration state
uint8_t calibrationStatus = CALIB_IDLE;
bool imuCalibrated = false;
bool escCalibrationRequested = false;
bool motorTestRequested = false;

// Timing
uint32_t loopStartTime;
uint32_t lastLoopTime;
uint16_t loopTimeUs;
uint32_t loopCounter;

// Sub-loop counters
uint8_t anglePIDCounter = 0;
uint8_t altitudeCounter = 0;
uint8_t baroCounter = 0;
uint8_t telemetryCounter = 0;

// LED state
uint32_t lastLedToggle = 0;
bool ledState = false;

// Control inputs (processed from RC)
float rollCommand = 0.0f;    // -30 to +30 degrees
float pitchCommand = 0.0f;   // -30 to +30 degrees
float yawRateCommand = 0.0f; // -180 to +180 deg/s
uint16_t throttleCommand = THROTTLE_MIN;

// PID outputs
float rollOutput = 0.0f;
float pitchOutput = 0.0f;
float yawOutput = 0.0f;

// Arm state tracking
bool wasArmed = false;
bool altHoldWasEnabled = false;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void initializeSensors();
void initializeControllers();
void processCommands();
void runAHRS();
void runAnglePID(float dt);
void runRatePID(float dt);
void runAltitude();
void updateMotors();
void updateTelemetry();
void checkSafety();
void handleCalibration();
void handleESCCalibration();
void handleMotorTest();
void updateStatusLED();
void buzzerCallback(int count);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial for debugging (if enabled)
    #ifdef DEBUG_SERIAL
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("FC: Initializing..."));
    #endif
    
    // Initialize status LED
    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);
    
    // Initialize buzzer
    buzzer.begin();
    buzzer.beep(100);  // Startup beep
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  // 400kHz
    
    // Initialize sensors
    initializeSensors();
    
    // Initialize NRF24L01
    if (!nrf.begin()) {
        #ifdef DEBUG_SERIAL
        Serial.println(F("FC: NRF24 init failed!"));
        #endif
        systemState = STATE_ERROR;
        buzzer.beeps(5, 500, 200);  // Error indication
    } else {
        #ifdef DEBUG_SERIAL
        Serial.println(F("FC: NRF24 initialized"));
        #endif
    }
    
    // Initialize AHRS
    ahrs.begin();
    
    // Initialize altitude estimator
    altEstimator.begin();
    altHoldController.begin();
    
    // Initialize controllers
    initializeControllers();
    
    // Initialize motors
    motors.begin();
    
    // Wait for sensors to stabilize
    delay(500);
    
    // Take initial barometer reading
    baro.forceRead();
    
    // Initialize AHRS from accelerometer
    imu.update();
    ahrs.initFromAccel(imu.data.accelX, imu.data.accelY, imu.data.accelZ);
    
    // Ready indication
    buzzer.beeps(2, 100, 100);
    
    // Enter preflight state
    systemState = STATE_PREFLIGHT;
    
    #ifdef DEBUG_SERIAL
    Serial.println(F("FC: Ready"));
    #endif
    
    // Initialize timing
    lastLoopTime = micros();
    loopCounter = 0;
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    loopStartTime = micros();
    
    // Calculate time delta
    uint32_t now = micros();
    float dt = (now - lastLoopTime) / 1000000.0f;
    lastLoopTime = now;
    
    // Clamp dt for safety
    if (dt <= 0.0f || dt > 0.1f) {
        dt = LOOP_PERIOD_US / 1000000.0f;
    }
    
    // ========================================================================
    // 1. READ SENSORS
    // ========================================================================
    
    // Read IMU (every loop - 400 Hz)
    imu.update();
    
    // Read barometer (every 20th loop - 20 Hz)
    baroCounter++;
    if (baroCounter >= BARO_DIVIDER) {
        baroCounter = 0;
        baro.update();
    }
    
    // ========================================================================
    // 2. COMMUNICATION
    // ========================================================================
    
    // Check for RC commands
    if (nrf.update()) {
        // New packet received
        processCommands();
    }
    
    // Update status LED based on link
    updateStatusLED();
    
    // ========================================================================
    // 3. ATTITUDE ESTIMATION (AHRS)
    // ========================================================================
    
    runAHRS();
    
    // ========================================================================
    // 4. ALTITUDE ESTIMATION
    // ========================================================================
    
    altitudeCounter++;
    if (altitudeCounter >= ALTITUDE_DIVIDER) {
        altitudeCounter = 0;
        runAltitude();
    }
    
    // ========================================================================
    // 5. CONTROL LOOPS
    // ========================================================================
    
    // Angle PID (outer loop - 100 Hz)
    anglePIDCounter++;
    if (anglePIDCounter >= ANGLE_PID_DIVIDER) {
        anglePIDCounter = 0;
        runAnglePID(dt * ANGLE_PID_DIVIDER);
    }
    
    // Rate PID (inner loop - 400 Hz)
    runRatePID(dt);
    
    // ========================================================================
    // 6. MOTOR OUTPUT
    // ========================================================================
    
    updateMotors();
    
    // ========================================================================
    // 7. SAFETY CHECKS
    // ========================================================================
    
    checkSafety();
    
    // ========================================================================
    // 8. TELEMETRY
    // ========================================================================
    
    telemetryCounter++;
    if (telemetryCounter >= TELEMETRY_DIVIDER) {
        telemetryCounter = 0;
        updateTelemetry();
    }
    
    // ========================================================================
    // 9. HOUSEKEEPING
    // ========================================================================
    
    // Update buzzer
    buzzer.update();
    
    // Handle calibration requests
    if (calibrationStatus == CALIB_IN_PROGRESS) {
        handleCalibration();
    }
    
    // Handle ESC calibration
    if (escCalibrationRequested) {
        handleESCCalibration();
    }
    
    // Handle motor test
    if (motorTestRequested) {
        handleMotorTest();
    }
    
    // Calculate loop time
    loopTimeUs = micros() - loopStartTime;
    loopCounter++;
    
    // ========================================================================
    // 10. LOOP TIMING
    // ========================================================================
    
    // Wait for next loop period
    uint32_t elapsed = micros() - loopStartTime;
    if (elapsed < LOOP_PERIOD_US) {
        delayMicroseconds(LOOP_PERIOD_US - elapsed);
    }
    
    #ifdef DEBUG_TIMING
    if (loopCounter % 400 == 0) {
        Serial.print(F("Loop: "));
        Serial.print(loopTimeUs);
        Serial.println(F("us"));
    }
    #endif
}

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

void initializeSensors() {
    // Initialize MPU6050
    if (!imu.begin()) {
        #ifdef DEBUG_SERIAL
        Serial.println(F("FC: MPU6050 init failed!"));
        #endif
        systemState = STATE_ERROR;
        buzzer.beeps(3, 500, 200);
    } else {
        #ifdef DEBUG_SERIAL
        Serial.println(F("FC: MPU6050 initialized"));
        #endif
    }
    
    // Initialize MS5611
    if (!baro.begin()) {
        #ifdef DEBUG_SERIAL
        Serial.println(F("FC: MS5611 init failed!"));
        #endif
        // Barometer failure is not critical - continue without altitude hold
    } else {
        #ifdef DEBUG_SERIAL
        Serial.println(F("FC: MS5611 initialized"));
        #endif
    }
}

void initializeControllers() {
    // Configure roll cascade controller
    rollController.configure(
        PID_ROLL_ANGLE_KP, PID_ROLL_ANGLE_KI, PID_ROLL_ANGLE_KD,
        PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD,
        MAX_ROLL_ANGLE, MAX_ROLL_RATE
    );
    
    // Configure pitch cascade controller
    pitchController.configure(
        PID_PITCH_ANGLE_KP, PID_PITCH_ANGLE_KI, PID_PITCH_ANGLE_KD,
        PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD,
        MAX_PITCH_ANGLE, MAX_PITCH_RATE
    );
    
    // Configure yaw rate controller
    yawRateController.setGains(PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD);
    yawRateController.setLimits(RATE_PID_IMAX, RATE_PID_OUTMAX);
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

void processCommands() {
    // Check for arm/disarm
    bool armRequested = nrf.isArmed();
    
    // Handle arm state changes
    if (armRequested && !wasArmed) {
        // Arming requested
        if (canArm()) {
            motors.arm();
            systemState = STATE_ARMED;
            buzzer.play(PATTERN_ARM);
            
            // Set altitude reference
            altEstimator.setReference();
            baro.setReferenceAltitude();
            
            // Reset controllers
            rollController.reset();
            pitchController.reset();
            yawRateController.reset();
            
            #ifdef DEBUG_SERIAL
            Serial.println(F("FC: Armed"));
            #endif
        }
    } else if (!armRequested && wasArmed) {
        // Disarming requested
        motors.disarm();
        systemState = STATE_PREFLIGHT;
        buzzer.play(PATTERN_DISARM);
        
        // Disable altitude hold
        altHoldController.disable();
        
        #ifdef DEBUG_SERIAL
        Serial.println(F("FC: Disarmed"));
        #endif
    }
    wasArmed = armRequested;
    
    // Process altitude hold request
    bool altHoldRequested = nrf.isAltHoldRequested();
    if (altHoldRequested && !altHoldWasEnabled && motors.armed) {
        altHoldController.enable(altEstimator.altitude, throttleCommand);
        #ifdef DEBUG_SERIAL
        Serial.println(F("FC: Alt hold enabled"));
        #endif
    } else if (!altHoldRequested && altHoldWasEnabled) {
        altHoldController.disable();
        #ifdef DEBUG_SERIAL
        Serial.println(F("FC: Alt hold disabled"));
        #endif
    }
    altHoldWasEnabled = altHoldRequested;
    
    // Check for calibration request
    if (nrf.isCalibrationRequested() && !motors.armed) {
        calibrationStatus = CALIB_IN_PROGRESS;
    }
    
    // Check for ESC calibration request
    if (nrf.isESCCalibrationRequested() && !motors.armed) {
        escCalibrationRequested = true;
    }
    
    // Check for motor test request
    if (nrf.isMotorTestRequested() && !motors.armed) {
        motorTestRequested = true;
    }
    
    // Map RC inputs to control commands
    // Throttle: 1000-2000 direct
    throttleCommand = nrf.getThrottle();
    
    // Roll: 1000-2000 → -30 to +30 degrees
    rollCommand = mapFloat(nrf.getRoll(), 1000, 2000, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
    
    // Pitch: 1000-2000 → -30 to +30 degrees
    pitchCommand = mapFloat(nrf.getPitch(), 1000, 2000, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
    
    // Yaw: 1000-2000 → -180 to +180 deg/s
    yawRateCommand = mapFloat(nrf.getYaw(), 1000, 2000, -MAX_YAW_RATE, MAX_YAW_RATE);
    
    // Apply deadband to yaw
    if (abs(yawRateCommand) < 5.0f) {
        yawRateCommand = 0.0f;
    }
}

bool canArm() {
    // Check conditions for arming
    if (systemState == STATE_ERROR) return false;
    if (!nrf.isLinked()) return false;
    if (!imuCalibrated) return false;
    if (throttleCommand > THROTTLE_IDLE + 50) return false;  // Throttle must be low
    if (abs(ahrs.roll) > 10.0f || abs(ahrs.pitch) > 10.0f) return false;  // Must be level
    
    return true;
}

// ============================================================================
// AHRS UPDATE
// ============================================================================

void runAHRS() {
    // Update Mahony AHRS filter
    ahrs.update(
        imu.data.gyroX, imu.data.gyroY, imu.data.gyroZ,
        imu.data.accelX, imu.data.accelY, imu.data.accelZ
    );
    
    #ifdef DEBUG_SENSORS
    if (loopCounter % 100 == 0) {
        Serial.print(F("R:")); Serial.print(ahrs.roll);
        Serial.print(F(" P:")); Serial.print(ahrs.pitch);
        Serial.print(F(" Y:")); Serial.println(ahrs.yaw);
    }
    #endif
}

// ============================================================================
// CONTROL LOOPS
// ============================================================================

void runAnglePID(float dt) {
    // Outer loop generates rate setpoints from angle error
    // This is handled inside the cascade controller
}

void runRatePID(float dt) {
    if (!motors.armed) {
        rollOutput = 0.0f;
        pitchOutput = 0.0f;
        yawOutput = 0.0f;
        return;
    }
    
    // Cascade PID: angle → rate → output
    rollOutput = rollController.compute(
        rollCommand, ahrs.roll,
        ahrs.rollRate,
        dt * ANGLE_PID_DIVIDER, dt
    );
    
    pitchOutput = pitchController.compute(
        pitchCommand, ahrs.pitch,
        ahrs.pitchRate,
        dt * ANGLE_PID_DIVIDER, dt
    );
    
    // Yaw is rate-only control
    float yawError = yawRateCommand - ahrs.yawRate;
    yawOutput = yawRateController.computeFromError(yawError, dt);
    
    #ifdef DEBUG_PID
    if (loopCounter % 100 == 0) {
        Serial.print(F("Roll:")); Serial.print(rollOutput);
        Serial.print(F(" Pitch:")); Serial.print(pitchOutput);
        Serial.print(F(" Yaw:")); Serial.println(yawOutput);
    }
    #endif
}

// ============================================================================
// ALTITUDE CONTROL
// ============================================================================

void runAltitude() {
    // Update altitude estimator
    float vertAccel = ahrs.getVerticalAccel(
        imu.data.accelX, imu.data.accelY, imu.data.accelZ
    );
    altEstimator.update(baro.altitudeFiltered, vertAccel);
    
    // Run altitude hold if enabled
    if (altHoldController.enabled && motors.armed) {
        altHoldController.update(
            altEstimator.altitude,
            altEstimator.velocity,
            throttleCommand
        );
    }
}

// ============================================================================
// MOTOR OUTPUT
// ============================================================================

void updateMotors() {
    if (systemState == STATE_FAILSAFE || systemState == STATE_ERROR) {
        motors.emergencyStop();
        return;
    }
    
    // Get throttle (with altitude hold adjustment if enabled)
    uint16_t throttle;
    if (altHoldController.enabled) {
        throttle = altHoldController.getThrottleOutput(throttleCommand);
    } else {
        throttle = throttleCommand;
    }
    
    // Mix control outputs
    motors.mix(throttle, rollOutput, pitchOutput, yawOutput);
    
    // Write to ESCs
    motors.writeMotors();
    
    #ifdef DEBUG_MOTORS
    if (loopCounter % 100 == 0) {
        Serial.print(F("M: "));
        Serial.print(motors.motorOutput[0]); Serial.print(F(" "));
        Serial.print(motors.motorOutput[1]); Serial.print(F(" "));
        Serial.print(motors.motorOutput[2]); Serial.print(F(" "));
        Serial.println(motors.motorOutput[3]);
    }
    #endif
}

// ============================================================================
// SAFETY FUNCTIONS
// ============================================================================

void checkSafety() {
    // Check for failsafe conditions
    
    // 1. Link loss
    #if ENABLE_FAILSAFE
    if (!nrf.isLinked() && motors.armed) {
        systemState = STATE_FAILSAFE;
        motors.emergencyStop();
        buzzer.play(PATTERN_FAILSAFE);
        
        #ifdef DEBUG_SERIAL
        Serial.println(F("FC: FAILSAFE - Link lost!"));
        #endif
    }
    #endif
    
    // 2. Emergency stop from RC
    if (nrf.isEmergencyStop()) {
        motors.emergencyStop();
        systemState = STATE_PREFLIGHT;
        
        #ifdef DEBUG_SERIAL
        Serial.println(F("FC: Emergency stop"));
        #endif
    }
    
    // 3. Excessive tilt
    #if ENABLE_TILT_SAFETY
    if (motors.armed) {
        if (abs(ahrs.roll) > MAX_SAFE_TILT || abs(ahrs.pitch) > MAX_SAFE_TILT) {
            motors.emergencyStop();
            systemState = STATE_FAILSAFE;
            buzzer.play(PATTERN_FAILSAFE);
            
            #ifdef DEBUG_SERIAL
            Serial.println(F("FC: FAILSAFE - Excessive tilt!"));
            #endif
        }
    }
    #endif
}

// ============================================================================
// CALIBRATION
// ============================================================================

void handleCalibration() {
    #ifdef DEBUG_SERIAL
    Serial.println(F("FC: Starting IMU calibration..."));
    #endif
    
    // Perform calibration
    uint8_t result = imu.calibrate();
    
    if (result == CALIB_SUCCESS) {
        calibrationStatus = CALIB_SUCCESS;
        imuCalibrated = true;
        buzzer.play(PATTERN_CALIB_OK);
        
        #ifdef DEBUG_SERIAL
        Serial.println(F("FC: Calibration successful"));
        Serial.print(F("Gyro offsets: "));
        Serial.print(imu.calibration.gyroOffsetX); Serial.print(F(", "));
        Serial.print(imu.calibration.gyroOffsetY); Serial.print(F(", "));
        Serial.println(imu.calibration.gyroOffsetZ);
        #endif
    } else {
        calibrationStatus = result;
        buzzer.play(PATTERN_CALIB_FAIL);
        
        #ifdef DEBUG_SERIAL
        Serial.print(F("FC: Calibration failed: "));
        Serial.println(result);
        #endif
    }
}

void handleESCCalibration() {
    #ifdef DEBUG_SERIAL
    Serial.println(F("FC: Starting ESC calibration..."));
    #endif
    
    motors.calibrateESC(buzzerCallback);
    escCalibrationRequested = false;
    
    calibrationStatus = CALIB_ESC_COMPLETE;
    
    #ifdef DEBUG_SERIAL
    Serial.println(F("FC: ESC calibration complete"));
    #endif
}

void handleMotorTest() {
    #ifdef DEBUG_SERIAL
    Serial.println(F("FC: Starting motor test..."));
    #endif
    
    motors.motorTest(buzzerCallback);
    motorTestRequested = false;
    
    #ifdef DEBUG_SERIAL
    Serial.println(F("FC: Motor test complete"));
    #endif
}

void buzzerCallback(int count) {
    buzzer.beeps(count, 150, 100);
}

// ============================================================================
// TELEMETRY
// ============================================================================

void updateTelemetry() {
    // Build status flags
    uint8_t status = 0;
    if (motors.armed) status |= STATUS_FLAG_ARMED;
    if (altHoldController.enabled) status |= STATUS_FLAG_ALT_HOLD;
    if (imuCalibrated) status |= STATUS_FLAG_CALIBRATED;
    if (systemState == STATE_FAILSAFE) status |= STATUS_FLAG_FAILSAFE;
    if (systemState == STATE_ERROR) status |= STATUS_FLAG_ERROR;
    if (motorTestRequested) status |= STATUS_FLAG_MOTOR_TEST;
    
    // Update NRF telemetry
    nrf.setTelemetry(
        status,
        (int16_t)(ahrs.roll * 100),
        (int16_t)(ahrs.pitch * 100),
        (int16_t)(ahrs.yaw * 100),
        (int16_t)altEstimator.altitude,
        (int16_t)altEstimator.velocity,
        0,  // Battery voltage (not implemented)
        calibrationStatus,
        loopTimeUs,
        motors.getMotorPercent(MOTOR_FL),
        motors.getMotorPercent(MOTOR_FR),
        motors.getMotorPercent(MOTOR_RR),
        motors.getMotorPercent(MOTOR_RL)
    );
}

// ============================================================================
// STATUS LED
// ============================================================================

void updateStatusLED() {
    uint32_t now = millis();
    
    if (systemState == STATE_ERROR) {
        // Rapid blinking for error
        if (now - lastLedToggle >= 100) {
            ledState = !ledState;
            digitalWrite(PIN_LED_STATUS, ledState);
            lastLedToggle = now;
        }
    } else if (systemState == STATE_FAILSAFE) {
        // Very rapid blinking for failsafe
        if (now - lastLedToggle >= 50) {
            ledState = !ledState;
            digitalWrite(PIN_LED_STATUS, ledState);
            lastLedToggle = now;
        }
    } else if (!nrf.isLinked()) {
        // Slow blinking when no link
        if (now - lastLedToggle >= 500) {
            ledState = !ledState;
            digitalWrite(PIN_LED_STATUS, ledState);
            lastLedToggle = now;
        }
    } else if (motors.armed) {
        // Solid on when armed and linked
        digitalWrite(PIN_LED_STATUS, HIGH);
    } else {
        // Fast blinking when linked but not armed
        if (now - lastLedToggle >= 200) {
            ledState = !ledState;
            digitalWrite(PIN_LED_STATUS, ledState);
            lastLedToggle = now;
        }
    }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
