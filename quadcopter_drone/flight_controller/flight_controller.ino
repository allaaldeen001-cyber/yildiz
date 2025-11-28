/*
 * ============================================================================
 * ARDUINO NANO QUADCOPTER FLIGHT CONTROLLER
 * ============================================================================
 * 
 * Hardware:
 * - Arduino Nano (ATmega328P)
 * - MPU6050 (6-axis IMU)
 * - MS5611 (Barometric pressure sensor)
 * - NRF24L01 (2.4GHz wireless transceiver)
 * - 4x ESC (Electronic Speed Controllers)
 * - Buzzer (status feedback)
 * - 2x LED (status indicators)
 * 
 * Features:
 * - PID-based flight stabilization
 * - Angle mode (auto-level)
 * - Altitude estimation
 * - Wireless RC control
 * - Failsafe on signal loss
 * - EEPROM calibration storage
 * 
 * Author: Embedded Systems Professional
 * ============================================================================
 */

#include "config.h"
#include "mpu6050.h"
#include "ms5611.h"
#include "pid.h"
#include "motors.h"
#include "nrf_comm.h"
#include "calibration.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

MPU6050 imu;
MS5611 baro;
FlightPID flightPID;
MotorController motors;
NRFComm nrf(NRF_CE_PIN, NRF_CSN_PIN);
Calibration calibration;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Drone state
DroneState droneState;
RCData rcData;
TelemetryData telemetry;
MotorOutputs motorOutputs;

// Timing
unsigned long loopStartTime;
unsigned long lastLoopTime;
unsigned long lastTelemetryTime;
unsigned long lastRCTime;
float dt;

// Status tracking
uint8_t systemStatus = 0;
bool setupComplete = false;
bool waitingForKillSwitch = true;
bool waitingForCalibration = false;
bool waitingForArm = false;
bool motorsTestRequested = false;

// LED blinking
unsigned long lastLEDBlink = 0;
bool ledState = false;

// User guide state machine
enum SetupState {
    SETUP_INIT,
    SETUP_WAIT_NRF,
    SETUP_WAIT_KILL_SWITCH,
    SETUP_WAIT_CALIBRATION,
    SETUP_WAIT_ARM,
    SETUP_WAIT_MOTOR_TEST,
    SETUP_READY,
    SETUP_FLYING
};
SetupState setupState = SETUP_INIT;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void initializeHardware();
void initializeSensors();
void processRCData();
void updateSensors();
void calculatePID();
void mixMotors();
void updateMotors();
void handleSafety();
void sendTelemetry();
void updateStatusLED();
void beepBuzzer(int times, int duration);
void printTelemetry();
void handleUserGuide();
void printWelcomeMessage();
void printReadyMessage();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial communication
    Serial.begin(DEBUG_BAUD_RATE);
    while (!Serial && millis() < 3000);  // Wait for serial (with timeout)
    
    printWelcomeMessage();
    
    // Initialize hardware
    initializeHardware();
    
    // Initialize sensors
    initializeSensors();
    
    // Initialize PID controllers
    flightPID.init();
    Serial.println(F("PID controllers initialized"));
    
    // Initialize motors (but don't arm)
    motors.init();
    
    // Initialize NRF24L01
    if (nrf.beginAsReceiver()) {
        systemStatus |= STATUS_NRF_OK;
        beepBuzzer(2, 100);  // Two short beeps
    } else {
        Serial.println(F("!!! NRF24L01 FAILED - Check wiring !!!"));
        beepBuzzer(5, 200);  // Error beeps
    }
    
    // Initialize calibration
    calibration.init(&imu, &baro, &motors);
    
    // Check if we have valid calibration
    if (calibration.isCalibrated()) {
        systemStatus |= STATUS_CALIBRATED;
        Serial.println(F("Using saved calibration data"));
    }
    
    // Initialize timing
    lastLoopTime = micros();
    lastTelemetryTime = millis();
    lastRCTime = millis();
    
    // Start user guide
    setupState = SETUP_WAIT_NRF;
    
    Serial.println(F(""));
    Serial.println(F("═══════════════════════════════════════════════════════"));
    Serial.println(F("              SETUP COMPLETE - STARTING GUIDE"));
    Serial.println(F("═══════════════════════════════════════════════════════"));
    Serial.println(F(""));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    loopStartTime = micros();
    
    // Calculate delta time
    dt = (loopStartTime - lastLoopTime) / 1000000.0f;
    lastLoopTime = loopStartTime;
    
    // Clamp dt to reasonable values
    if (dt > 0.05f) dt = 0.05f;
    if (dt < 0.001f) dt = 0.001f;
    
    // Process RC data
    processRCData();
    
    // Handle user guide state machine
    handleUserGuide();
    
    // Update sensors
    updateSensors();
    
    // Handle safety (failsafe, kill switch)
    handleSafety();
    
    // Only calculate PID and mix motors if armed
    if (droneState.armed && setupState == SETUP_FLYING) {
        calculatePID();
        mixMotors();
    }
    
    // Update motor outputs
    updateMotors();
    
    // Send telemetry periodically
    if (millis() - lastTelemetryTime >= TELEMETRY_RATE_MS) {
        sendTelemetry();
        if (setupState == SETUP_FLYING) {
            printTelemetry();
        }
        lastTelemetryTime = millis();
    }
    
    // Update status LED
    updateStatusLED();
    
    // Maintain loop timing
    while (micros() - loopStartTime < LOOP_TIME_US) {
        // Wait
    }
}

// ============================================================================
// INITIALIZATION FUNCTIONS
// ============================================================================

void initializeHardware() {
    Serial.println(F("Initializing hardware..."));
    
    // Configure pins
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(ARM_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    
    // Turn off LEDs
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(ARM_LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    
    // Startup beep
    beepBuzzer(1, 200);
    
    Serial.println(F("Hardware pins configured"));
}

void initializeSensors() {
    Serial.println(F("Initializing sensors..."));
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C
    
    // Initialize MPU6050
    Serial.print(F("  MPU6050... "));
    if (imu.begin()) {
        systemStatus |= STATUS_GYRO_OK | STATUS_ACCEL_OK;
        Serial.println(F("OK"));
    } else {
        Serial.println(F("FAILED!"));
    }
    
    // Initialize MS5611
    Serial.print(F("  MS5611... "));
    if (baro.begin()) {
        systemStatus |= STATUS_BARO_OK;
        Serial.println(F("OK"));
    } else {
        Serial.println(F("FAILED!"));
    }
    
    Serial.println(F("Sensors initialized"));
}

// ============================================================================
// RC DATA PROCESSING
// ============================================================================

void processRCData() {
    // Check for new data from RC transmitter
    if (nrf.receiveData(&rcData)) {
        lastRCTime = millis();
        droneState.rcConnected = true;
        
        // Blink LED on successful receive
        if (ledState == LOW) {
            digitalWrite(STATUS_LED_PIN, HIGH);
        }
        
        // Handle calibration button
        if (rcData.button1 && setupState == SETUP_WAIT_CALIBRATION) {
            Serial.println(F("\n>>> CALIBRATION BUTTON PRESSED <<<\n"));
            calibration.runFullCalibration();
            systemStatus |= STATUS_CALIBRATED;
            setupState = SETUP_WAIT_ARM;
            beepBuzzer(3, 100);
        }
        
        // Handle motor test button
        if (rcData.button2 && motorsTestRequested && droneState.armed) {
            Serial.println(F("\n>>> MOTOR TEST BUTTON PRESSED <<<\n"));
            motors.smoothTest(1150, 3000);  // Test at low speed for 3 seconds
            motorsTestRequested = false;
            setupState = SETUP_READY;
            beepBuzzer(2, 100);
        }
        
        // Handle arm switch
        if (rcData.armSwitch && !droneState.armed) {
            // Arming requested
            if (setupState >= SETUP_WAIT_ARM && rcData.throttle < THROTTLE_ARM_MAX) {
                Serial.println(F("\n╔════════════════════════════════════════╗"));
                Serial.println(F("║           DRONE ARMED!                 ║"));
                Serial.println(F("╚════════════════════════════════════════╝\n"));
                droneState.armed = true;
                motors.arm();
                digitalWrite(ARM_LED_PIN, HIGH);
                systemStatus |= STATUS_ARMED;
                beepBuzzer(1, 500);
                
                if (setupState == SETUP_WAIT_ARM) {
                    setupState = SETUP_WAIT_MOTOR_TEST;
                    motorsTestRequested = true;
                }
            } else if (rcData.throttle >= THROTTLE_ARM_MAX) {
                Serial.println(F("Cannot arm - throttle too high!"));
            }
        } else if (!rcData.armSwitch && droneState.armed) {
            // Disarming (kill switch)
            Serial.println(F("\n╔════════════════════════════════════════╗"));
            Serial.println(F("║          DRONE DISARMED!               ║"));
            Serial.println(F("╚════════════════════════════════════════╝\n"));
            droneState.armed = false;
            motors.disarm();
            flightPID.reset();
            digitalWrite(ARM_LED_PIN, LOW);
            systemStatus &= ~STATUS_ARMED;
            beepBuzzer(2, 200);
        }
    }
    
    // Check for signal loss
    if (millis() - lastRCTime > NRF_TIMEOUT_MS) {
        droneState.rcConnected = false;
    }
}

// ============================================================================
// SENSOR UPDATES
// ============================================================================

void updateSensors() {
    // Update IMU
    imu.updateAttitude(dt);
    
    droneState.roll = imu.getRoll();
    droneState.pitch = imu.getPitch();
    droneState.yaw = imu.getYaw();
    droneState.rollRate = imu.getRollRate();
    droneState.pitchRate = imu.getPitchRate();
    droneState.yawRate = imu.getYawRate();
    
    // Update barometer (non-blocking)
    baro.update();
    
    if (baro.isReady()) {
        droneState.altitude = baro.getAltitude();
        droneState.verticalSpeed = baro.getVerticalSpeed();
    }
}

// ============================================================================
// PID CALCULATION
// ============================================================================

void calculatePID() {
    // Convert RC inputs to setpoints
    float rollSetpoint = 0.0f;
    float pitchSetpoint = 0.0f;
    float yawSetpoint = 0.0f;
    
    // Apply deadzone and convert to angles/rates
    int16_t rollInput = rcData.roll - STICK_CENTER;
    int16_t pitchInput = rcData.pitch - STICK_CENTER;
    int16_t yawInput = rcData.yaw - STICK_CENTER;
    
    // Apply deadzone
    if (abs(rollInput) < STICK_DEADZONE) rollInput = 0;
    if (abs(pitchInput) < STICK_DEADZONE) pitchInput = 0;
    if (abs(yawInput) < STICK_DEADZONE) yawInput = 0;
    
    // Convert to angles (for angle mode)
    rollSetpoint = (float)rollInput / 500.0f * MAX_ROLL_ANGLE;
    pitchSetpoint = (float)pitchInput / 500.0f * MAX_PITCH_ANGLE;
    yawSetpoint = (float)yawInput / 500.0f * MAX_YAW_RATE;
    
    // Calculate PID outputs
    flightPID.calculate(
        rollSetpoint, pitchSetpoint, yawSetpoint,
        droneState.roll, droneState.pitch, droneState.yawRate,
        droneState.rollRate, droneState.pitchRate,
        dt
    );
}

// ============================================================================
// MOTOR MIXING
// ============================================================================

void mixMotors() {
    // Get PID outputs
    float rollOutput = flightPID.getRollOutput();
    float pitchOutput = flightPID.getPitchOutput();
    float yawOutput = flightPID.getYawOutput();
    
    // Mix motors
    motors.mix(rcData.throttle, rollOutput, pitchOutput, yawOutput);
}

// ============================================================================
// MOTOR UPDATE
// ============================================================================

void updateMotors() {
    motors.update();
}

// ============================================================================
// SAFETY HANDLING
// ============================================================================

void handleSafety() {
    // Failsafe - signal loss
    if (!droneState.rcConnected && droneState.armed) {
        Serial.println(F("!!! FAILSAFE - RC SIGNAL LOST !!!"));
        droneState.failsafe = true;
        motors.emergencyStop();
        droneState.armed = false;
        flightPID.reset();
        digitalWrite(ARM_LED_PIN, LOW);
        beepBuzzer(10, 50);  // Rapid beeping
    }
    
    // Kill switch handling is done in processRCData()
}

// ============================================================================
// TELEMETRY
// ============================================================================

void sendTelemetry() {
    // Prepare telemetry data
    telemetry.roll = (int16_t)(droneState.roll * 10);
    telemetry.pitch = (int16_t)(droneState.pitch * 10);
    telemetry.yaw = (int16_t)(droneState.yaw * 10);
    telemetry.altitude = (uint16_t)(droneState.altitude * 100);
    telemetry.batteryVoltage = 1120;  // Placeholder (11.2V)
    telemetry.armed = droneState.armed ? 1 : 0;
    telemetry.flightMode = FLIGHT_MODE_STABILIZE;
    telemetry.rssi = nrf.getRSSI();
    telemetry.status = systemStatus;
    
    // Send telemetry
    nrf.sendTelemetry(&telemetry);
}

void printTelemetry() {
    Serial.println(F(""));
    Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
    Serial.println(F("║                    QUADCOPTER TELEMETRY                        ║"));
    Serial.println(F("╠════════════════════════════════════════════════════════════════╣"));
    
    // Control inputs
    Serial.print(F("║ THR: "));
    Serial.print(rcData.throttle);
    Serial.print(F("  YAW: "));
    Serial.print(rcData.yaw);
    Serial.print(F("  PIT: "));
    Serial.print(rcData.pitch);
    Serial.print(F("  ROL: "));
    Serial.print(rcData.roll);
    Serial.println(F("        ║"));
    
    // Angles
    Serial.print(F("║ Roll: "));
    Serial.print(droneState.roll, 1);
    Serial.print(F("°  Pitch: "));
    Serial.print(droneState.pitch, 1);
    Serial.print(F("°  Yaw: "));
    Serial.print(droneState.yaw, 1);
    Serial.println(F("°                   ║"));
    
    // Altitude
    Serial.print(F("║ Alt: "));
    Serial.print(droneState.altitude, 2);
    Serial.print(F("m  VSpeed: "));
    Serial.print(droneState.verticalSpeed, 2);
    Serial.println(F(" m/s                          ║"));
    
    // Motor values
    Serial.print(F("║ M1: "));
    Serial.print(motors.getMotor1());
    Serial.print(F("  M2: "));
    Serial.print(motors.getMotor2());
    Serial.print(F("  M3: "));
    Serial.print(motors.getMotor3());
    Serial.print(F("  M4: "));
    Serial.print(motors.getMotor4());
    Serial.println(F("       ║"));
    
    // Status
    Serial.print(F("║ STATUS: "));
    Serial.print(droneState.armed ? F("ARMED") : F("DISARMED"));
    Serial.print(F(" | "));
    Serial.print(droneState.rcConnected ? F("RC OK") : F("NO RC"));
    Serial.print(F(" | CH: "));
    Serial.print(nrf.getChannel());
    Serial.println(F("                      ║"));
    
    Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
}

// ============================================================================
// STATUS LED
// ============================================================================

void updateStatusLED() {
    unsigned long now = millis();
    
    // Different blink patterns based on state
    uint16_t blinkInterval;
    
    if (!droneState.rcConnected) {
        blinkInterval = 100;  // Fast blink - no connection
    } else if (!droneState.armed) {
        blinkInterval = 500;  // Slow blink - connected but disarmed
    } else {
        blinkInterval = 1000; // Very slow blink - armed
    }
    
    if (now - lastLEDBlink >= blinkInterval) {
        ledState = !ledState;
        digitalWrite(STATUS_LED_PIN, ledState);
        lastLEDBlink = now;
    }
}

// ============================================================================
// BUZZER
// ============================================================================

void beepBuzzer(int times, int duration) {
    for (int i = 0; i < times; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(duration);
        digitalWrite(BUZZER_PIN, LOW);
        if (i < times - 1) {
            delay(duration);
        }
    }
}

// ============================================================================
// USER GUIDE STATE MACHINE
// ============================================================================

void handleUserGuide() {
    static unsigned long lastGuideMessage = 0;
    unsigned long now = millis();
    
    switch (setupState) {
        case SETUP_WAIT_NRF:
            if (systemStatus & STATUS_NRF_OK) {
                if (droneState.rcConnected) {
                    Serial.println(F(""));
                    Serial.println(F("╔════════════════════════════════════════╗"));
                    Serial.println(F("║   ✓ NRF24L01 CONNECTED SUCCESSFULLY!  ║"));
                    Serial.println(F("╚════════════════════════════════════════╝"));
                    beepBuzzer(2, 100);
                    setupState = SETUP_WAIT_KILL_SWITCH;
                    lastGuideMessage = now;
                }
            } else if (now - lastGuideMessage > 3000) {
                Serial.println(F("Waiting for NRF24L01 connection..."));
                Serial.println(F("Make sure RC transmitter is powered on."));
                lastGuideMessage = now;
            }
            break;
            
        case SETUP_WAIT_KILL_SWITCH:
            if (now - lastGuideMessage > 2000) {
                Serial.println(F(""));
                Serial.println(F("╔════════════════════════════════════════╗"));
                Serial.println(F("║       STEP 1: SET KILL SWITCH          ║"));
                Serial.println(F("╠════════════════════════════════════════╣"));
                Serial.println(F("║  Set the ARM/DISARM toggle switch to   ║"));
                Serial.println(F("║  DISARM position (kill switch mode)    ║"));
                Serial.println(F("╚════════════════════════════════════════╝"));
                lastGuideMessage = now;
            }
            
            // Check if kill switch is off (disarmed)
            if (!rcData.armSwitch && droneState.rcConnected) {
                Serial.println(F(""));
                Serial.println(F("✓ Kill switch confirmed - DISARMED"));
                setupState = SETUP_WAIT_CALIBRATION;
                beepBuzzer(1, 100);
                lastGuideMessage = now;
            }
            break;
            
        case SETUP_WAIT_CALIBRATION:
            if (now - lastGuideMessage > 2000) {
                if (systemStatus & STATUS_CALIBRATED) {
                    Serial.println(F(""));
                    Serial.println(F("╔════════════════════════════════════════╗"));
                    Serial.println(F("║     CALIBRATION DATA FOUND IN EEPROM   ║"));
                    Serial.println(F("╠════════════════════════════════════════╣"));
                    Serial.println(F("║  Press Button 1 to RE-CALIBRATE, or    ║"));
                    Serial.println(F("║  Proceed to ARM the drone              ║"));
                    Serial.println(F("╚════════════════════════════════════════╝"));
                    setupState = SETUP_WAIT_ARM;
                } else {
                    Serial.println(F(""));
                    Serial.println(F("╔════════════════════════════════════════╗"));
                    Serial.println(F("║       STEP 2: CALIBRATION              ║"));
                    Serial.println(F("╠════════════════════════════════════════╣"));
                    Serial.println(F("║  1. Place drone on FLAT surface        ║"));
                    Serial.println(F("║  2. Keep drone PERFECTLY STILL         ║"));
                    Serial.println(F("║  3. Press Button 1 to start            ║"));
                    Serial.println(F("╚════════════════════════════════════════╝"));
                }
                lastGuideMessage = now;
            }
            break;
            
        case SETUP_WAIT_ARM:
            if (now - lastGuideMessage > 2000) {
                Serial.println(F(""));
                Serial.println(F("╔════════════════════════════════════════╗"));
                Serial.println(F("║       STEP 3: ARM THE DRONE            ║"));
                Serial.println(F("╠════════════════════════════════════════╣"));
                Serial.println(F("║  1. Move THROTTLE to MINIMUM (down)    ║"));
                Serial.println(F("║  2. Toggle ARM switch to ARM position  ║"));
                Serial.println(F("║                                        ║"));
                Serial.println(F("║  ⚠ PROPS WILL SPIN - STAY CLEAR! ⚠    ║"));
                Serial.println(F("╚════════════════════════════════════════╝"));
                lastGuideMessage = now;
            }
            break;
            
        case SETUP_WAIT_MOTOR_TEST:
            if (now - lastGuideMessage > 2000) {
                Serial.println(F(""));
                Serial.println(F("╔════════════════════════════════════════╗"));
                Serial.println(F("║       STEP 4: MOTOR TEST               ║"));
                Serial.println(F("╠════════════════════════════════════════╣"));
                Serial.println(F("║  Press Button 2 to test motors         ║"));
                Serial.println(F("║  Motors will spin up slowly for test   ║"));
                Serial.println(F("║                                        ║"));
                Serial.println(F("║  Verify all 4 motors spin correctly    ║"));
                Serial.println(F("╚════════════════════════════════════════╝"));
                lastGuideMessage = now;
            }
            break;
            
        case SETUP_READY:
            printReadyMessage();
            setupState = SETUP_FLYING;
            break;
            
        case SETUP_FLYING:
            // Normal flight operations
            break;
    }
}

// ============================================================================
// MESSAGES
// ============================================================================

void printWelcomeMessage() {
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("╔══════════════════════════════════════════════════════════════════╗"));
    Serial.println(F("║                                                                  ║"));
    Serial.println(F("║     █████╗ ██████╗ ██████╗ ██╗   ██╗██╗███╗   ██╗ ██████╗       ║"));
    Serial.println(F("║    ██╔══██╗██╔══██╗██╔══██╗██║   ██║██║████╗  ██║██╔═══██╗      ║"));
    Serial.println(F("║    ███████║██████╔╝██║  ██║██║   ██║██║██╔██╗ ██║██║   ██║      ║"));
    Serial.println(F("║    ██╔══██║██╔══██╗██║  ██║██║   ██║██║██║╚██╗██║██║   ██║      ║"));
    Serial.println(F("║    ██║  ██║██║  ██║██████╔╝╚██████╔╝██║██║ ╚████║╚██████╔╝      ║"));
    Serial.println(F("║    ╚═╝  ╚═╝╚═╝  ╚═╝╚═════╝  ╚═════╝ ╚═╝╚═╝  ╚═══╝ ╚═════╝       ║"));
    Serial.println(F("║                                                                  ║"));
    Serial.println(F("║           QUADCOPTER FLIGHT CONTROLLER v1.0                      ║"));
    Serial.println(F("║                                                                  ║"));
    Serial.println(F("╚══════════════════════════════════════════════════════════════════╝"));
    Serial.println(F(""));
}

void printReadyMessage() {
    Serial.println(F(""));
    Serial.println(F("╔══════════════════════════════════════════════════════════════════╗"));
    Serial.println(F("║                                                                  ║"));
    Serial.println(F("║    ██████╗ ███████╗ █████╗ ██████╗ ██╗   ██╗                     ║"));
    Serial.println(F("║    ██╔══██╗██╔════╝██╔══██╗██╔══██╗╚██╗ ██╔╝                     ║"));
    Serial.println(F("║    ██████╔╝█████╗  ███████║██║  ██║ ╚████╔╝                      ║"));
    Serial.println(F("║    ██╔══██╗██╔══╝  ██╔══██║██║  ██║  ╚██╔╝                       ║"));
    Serial.println(F("║    ██║  ██║███████╗██║  ██║██████╔╝   ██║                        ║"));
    Serial.println(F("║    ╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝╚═════╝    ╚═╝                        ║"));
    Serial.println(F("║                                                                  ║"));
    Serial.println(F("║              🚁 DRONE IS READY TO FLY! 🚁                        ║"));
    Serial.println(F("║                                                                  ║"));
    Serial.println(F("║   Controls:                                                      ║"));
    Serial.println(F("║   ─────────                                                      ║"));
    Serial.println(F("║   LEFT STICK:  Throttle (Up/Down), Yaw (Left/Right)              ║"));
    Serial.println(F("║   RIGHT STICK: Pitch (Up/Down), Roll (Left/Right)                ║"));
    Serial.println(F("║                                                                  ║"));
    Serial.println(F("║   Safety:                                                        ║"));
    Serial.println(F("║   ────────                                                       ║"));
    Serial.println(F("║   • Toggle ARM switch to DISARM to stop motors                   ║"));
    Serial.println(F("║   • Keep throttle LOW when not flying                            ║"));
    Serial.println(F("║                                                                  ║"));
    Serial.println(F("║   ⚠ FLY SAFE! STAY AWAY FROM PEOPLE AND OBSTACLES! ⚠            ║"));
    Serial.println(F("║                                                                  ║"));
    Serial.println(F("╚══════════════════════════════════════════════════════════════════╝"));
    Serial.println(F(""));
}
