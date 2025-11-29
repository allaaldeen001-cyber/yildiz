/**
 * @file flight_controller.ino
 * @brief Professional Quadcopter Flight Controller
 * @author UAV Embedded Systems Engineer
 * @version 2.0
 * @date 2024
 * 
 * Arduino Nano based flight controller with:
 * - MPU6050 IMU with complementary filter
 * - PID stabilization (Roll, Pitch, Yaw)
 * - NRF24L01 PA+LNA communication with ACK
 * - 4x ESC control with safety features
 * - Failsafe and auto-disarm
 * 
 * Hardware Configuration:
 * - NRF24L01: CE=D4, CSN=D10, SPI(D11,D12,D13)
 * - MPU6050:  SDA=A4, SCL=A5, INT=D2
 * - Motors:   FL=D3, FR=D5, RR=D6, RL=D9
 * - Buzzer:   D8
 * - LED:      D7
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>
#include "config.h"
#include "../shared/protocol.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Servo motorFL, motorFR, motorRR, motorRL;

// ============================================================================
// STATE VARIABLES
// ============================================================================

// System state
volatile DroneStatus_t systemStatus = STATUS_BOOT;
volatile bool nrfConnected = false;
volatile bool imuReady = false;
volatile bool motorsArmed = false;
volatile bool escCalibrated = false;
volatile bool imuCalibrated = false;

// Timing
unsigned long loopTimer = 0;
unsigned long lastPacketTime = 0;
unsigned long lastTelemetryTime = 0;
unsigned long ledTimer = 0;
unsigned long armStartTime = 0;
uint8_t loopCounter = 0;

// LED state
LEDPattern_t currentLEDPattern = LED_BLINK_SLOW;
bool ledState = false;

// ============================================================================
// IMU DATA
// ============================================================================

// Raw sensor data
int16_t gyroX, gyroY, gyroZ;
int16_t accelX, accelY, accelZ;
int16_t temperature;

// Calibration offsets
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;

// Filtered angles (degrees)
float angleRoll = 0, anglePitch = 0, angleYaw = 0;

// Gyro rates (deg/s)
float gyroRollRate = 0, gyroPitchRate = 0, gyroYawRate = 0;

// Complementary filter coefficient
const float COMP_FILTER_ALPHA = 0.98;

// ============================================================================
// PID CONTROLLER DATA
// ============================================================================

// PID errors
float rollError = 0, rollErrorPrev = 0, rollErrorI = 0;
float pitchError = 0, pitchErrorPrev = 0, pitchErrorI = 0;
float yawError = 0, yawErrorPrev = 0, yawErrorI = 0;

// PID outputs
float rollPID = 0, pitchPID = 0, yawPID = 0;

// Setpoints from RC
float rollSetpoint = 0, pitchSetpoint = 0, yawSetpoint = 0;
uint16_t throttleSetpoint = THROTTLE_MIN;

// ============================================================================
// MOTOR DATA
// ============================================================================

uint16_t motorPulseFL = ESC_MIN_PULSE;
uint16_t motorPulseFR = ESC_MIN_PULSE;
uint16_t motorPulseRR = ESC_MIN_PULSE;
uint16_t motorPulseRL = ESC_MIN_PULSE;

// ============================================================================
// COMMUNICATION DATA
// ============================================================================

RCPacket_t rcPacket;
FCTelemetry_t telemetry;
uint8_t packetSequence = 0;

// Switch states
bool sw1_altHold = false;
bool sw2_armSwitch = false;
bool btn1_calibrate = false;
bool btn2_motorOn = false;
bool prevBtn1 = false;
bool prevBtn2 = false;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void initializeHardware();
void initializeNRF24();
void initializeMPU6050();
void initializeMotors();
void calibrateIMU();
void calibrateESC();
void readIMU();
void calculateAngles(float dt);
void calculatePID(float dt);
void mixMotors();
void updateMotors();
void processRadio();
void updateTelemetry();
void handleFailsafe();
void updateLED();
void playBuzzerPattern(uint8_t pattern);
void beep(uint16_t frequency, uint16_t duration);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial for debugging (optional)
    Serial.begin(115200);
    Serial.println(F("=== Drone Flight Controller v2.0 ==="));
    
    // Initialize all hardware
    initializeHardware();
    
    // Wait for systems to stabilize
    delay(500);
    
    // Initialize NRF24L01
    initializeNRF24();
    
    // Initialize MPU6050
    initializeMPU6050();
    
    // Initialize motors (but don't arm)
    initializeMotors();
    
    // Set initial status
    systemStatus = STATUS_IDLE;
    currentLEDPattern = LED_BLINK_SLOW;
    
    // Startup beep
    beep(2000, 100);
    delay(100);
    beep(2500, 100);
    
    Serial.println(F("System Ready - Waiting for RC..."));
    
    // Initialize loop timer
    loopTimer = micros();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // ========== TIMING CONTROL ==========
    // Wait for precise loop timing (250Hz = 4000µs)
    while (micros() - loopTimer < LOOP_PERIOD_US);
    float dt = (micros() - loopTimer) / 1000000.0;
    loopTimer = micros();
    
    // ========== RADIO COMMUNICATION ==========
    processRadio();
    
    // ========== FAILSAFE CHECK ==========
    handleFailsafe();
    
    // ========== IMU PROCESSING ==========
    if (imuReady) {
        readIMU();
        calculateAngles(dt);
    }
    
    // ========== CONTROL PROCESSING ==========
    if (motorsArmed && nrfConnected) {
        // Only process PID if armed and connected
        calculatePID(dt);
        mixMotors();
    } else {
        // Reset PID integrators when not flying
        rollErrorI = 0;
        pitchErrorI = 0;
        yawErrorI = 0;
        
        // Set motors to minimum
        motorPulseFL = ESC_MIN_PULSE;
        motorPulseFR = ESC_MIN_PULSE;
        motorPulseRR = ESC_MIN_PULSE;
        motorPulseRL = ESC_MIN_PULSE;
    }
    
    // ========== MOTOR OUTPUT ==========
    updateMotors();
    
    // ========== TELEMETRY UPDATE ==========
    loopCounter++;
    if (loopCounter >= TELEMETRY_DIVIDER) {
        loopCounter = 0;
        updateTelemetry();
    }
    
    // ========== LED UPDATE ==========
    updateLED();
}

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================

void initializeHardware() {
    // Configure pins
    pinMode(LED_STATUS_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(MPU_INT_PIN, INPUT);
    
    // Initial states
    digitalWrite(LED_STATUS_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C
    
    Serial.println(F("Hardware initialized"));
}

// ============================================================================
// NRF24L01 INITIALIZATION
// ============================================================================

void initializeNRF24() {
    Serial.println(F("Initializing NRF24L01..."));
    
    if (!radio.begin()) {
        Serial.println(F("NRF24L01 FAILED!"));
        systemStatus = STATUS_ERROR;
        // Continuous error beep
        while (1) {
            beep(500, 200);
            delay(300);
        }
    }
    
    // Configure radio
    radio.setChannel(NRF_CHANNEL);
    radio.setPALevel(NRF_PA_LEVEL);
    radio.setDataRate(NRF_DATA_RATE);
    radio.setCRCLength(NRF_CRC_LENGTH);
    radio.setRetries(NRF_RETRY_DELAY, NRF_RETRY_COUNT);
    radio.setPayloadSize(NRF_PAYLOAD_SIZE);
    
    // Enable ACK payloads for bidirectional communication
    radio.enableAckPayload();
    radio.enableDynamicPayloads();
    
    // Open pipes
    radio.openWritingPipe(FC_TO_RC_ADDR);
    radio.openReadingPipe(1, RC_TO_FC_ADDR);
    
    // Start listening
    radio.startListening();
    
    // Prepare initial ACK payload
    memset(&telemetry, 0, sizeof(telemetry));
    telemetry.header = 0xBB;
    telemetry.status = STATUS_IDLE;
    radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
    
    Serial.print(F("NRF24L01 OK - Channel: "));
    Serial.println(NRF_CHANNEL);
}

// ============================================================================
// MPU6050 INITIALIZATION
// ============================================================================

void initializeMPU6050() {
    Serial.println(F("Initializing MPU6050..."));
    
    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up
    Wire.endTransmission();
    delay(100);
    
    // Configure gyroscope (±500°/s)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1B); // GYRO_CONFIG
    Wire.write(MPU_GYRO_CONFIG);
    Wire.endTransmission();
    
    // Configure accelerometer (±2g)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1C); // ACCEL_CONFIG
    Wire.write(MPU_ACCEL_CONFIG);
    Wire.endTransmission();
    
    // Configure DLPF (Digital Low Pass Filter)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1A); // CONFIG
    Wire.write(MPU_DLPF_CONFIG);
    Wire.endTransmission();
    
    // Configure sample rate (200Hz)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x19); // SMPLRT_DIV
    Wire.write(MPU_SAMPLE_RATE);
    Wire.endTransmission();
    
    // Check WHO_AM_I
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x75);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS, 1);
    uint8_t whoami = Wire.read();
    
    if (whoami == 0x68) {
        imuReady = true;
        Serial.println(F("MPU6050 OK"));
    } else {
        Serial.print(F("MPU6050 FAILED! WHO_AM_I: 0x"));
        Serial.println(whoami, HEX);
        systemStatus = STATUS_ERROR;
    }
}

// ============================================================================
// MOTOR INITIALIZATION
// ============================================================================

void initializeMotors() {
    Serial.println(F("Initializing Motors..."));
    
    // Attach ESCs to servo library
    motorFL.attach(MOTOR_FL_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorFR.attach(MOTOR_FR_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorRR.attach(MOTOR_RR_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorRL.attach(MOTOR_RL_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    
    // Send minimum throttle to arm ESCs
    motorFL.writeMicroseconds(ESC_MIN_PULSE);
    motorFR.writeMicroseconds(ESC_MIN_PULSE);
    motorRR.writeMicroseconds(ESC_MIN_PULSE);
    motorRL.writeMicroseconds(ESC_MIN_PULSE);
    
    Serial.println(F("Motors initialized (not armed)"));
}

// ============================================================================
// IMU CALIBRATION
// ============================================================================

void calibrateIMU() {
    Serial.println(F("Starting IMU Calibration..."));
    Serial.println(F("Keep drone LEVEL and STILL!"));
    
    systemStatus = STATUS_CALIBRATING;
    currentLEDPattern = LED_BLINK_FAST;
    
    // Reset offsets
    gyroOffsetX = 0;
    gyroOffsetY = 0;
    gyroOffsetZ = 0;
    accelOffsetX = 0;
    accelOffsetY = 0;
    accelOffsetZ = 0;
    
    // Accumulate samples
    long gx = 0, gy = 0, gz = 0;
    long ax = 0, ay = 0, az = 0;
    
    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
        readIMU();
        gx += gyroX;
        gy += gyroY;
        gz += gyroZ;
        ax += accelX;
        ay += accelY;
        az += accelZ;
        
        // Progress indicator
        if (i % 500 == 0) {
            Serial.print(F("."));
            digitalWrite(LED_STATUS_PIN, !digitalRead(LED_STATUS_PIN));
        }
        delayMicroseconds(1000);
    }
    Serial.println();
    
    // Calculate averages
    gyroOffsetX = gx / (float)GYRO_CALIBRATION_SAMPLES;
    gyroOffsetY = gy / (float)GYRO_CALIBRATION_SAMPLES;
    gyroOffsetZ = gz / (float)GYRO_CALIBRATION_SAMPLES;
    
    accelOffsetX = ax / (float)GYRO_CALIBRATION_SAMPLES;
    accelOffsetY = ay / (float)GYRO_CALIBRATION_SAMPLES;
    // Z-axis should read ~16384 (1g) when level
    accelOffsetZ = (az / (float)GYRO_CALIBRATION_SAMPLES) - 16384.0;
    
    // Validate calibration (gyro drift should be minimal)
    float maxDrift = max(abs(gyroOffsetX), max(abs(gyroOffsetY), abs(gyroOffsetZ)));
    
    if (maxDrift < 500) {
        imuCalibrated = true;
        systemStatus = STATUS_CALIBRATED;
        currentLEDPattern = LED_BLINK_SLOW;
        
        Serial.println(F("IMU Calibration SUCCESS!"));
        Serial.print(F("Gyro offsets: "));
        Serial.print(gyroOffsetX); Serial.print(F(", "));
        Serial.print(gyroOffsetY); Serial.print(F(", "));
        Serial.println(gyroOffsetZ);
        
        // Two beeps for success
        beep(BEEP_CALIBRATION_OK_FREQ, BEEP_CALIBRATION_OK_DUR);
        delay(150);
        beep(BEEP_CALIBRATION_OK_FREQ, BEEP_CALIBRATION_OK_DUR);
    } else {
        imuCalibrated = false;
        systemStatus = STATUS_CALIBRATION_FAIL;
        
        Serial.println(F("IMU Calibration FAILED!"));
        Serial.print(F("Max drift: "));
        Serial.println(maxDrift);
        
        // One long beep for failure (7 seconds)
        beep(BEEP_CALIBRATION_FAIL_FREQ, BEEP_CALIBRATION_FAIL_DUR);
    }
    
    // Reset angles
    angleRoll = 0;
    anglePitch = 0;
    angleYaw = 0;
}

// ============================================================================
// ESC CALIBRATION
// ============================================================================

void calibrateESC() {
    Serial.println(F("Starting ESC Calibration..."));
    Serial.println(F("REMOVE PROPELLERS!"));
    
    systemStatus = STATUS_ESC_CALIBRATING;
    currentLEDPattern = LED_BLINK_FAST;
    
    // ESC calibration sequence
    // Step 1: Send maximum throttle
    Serial.println(F("Sending MAX throttle..."));
    motorFL.writeMicroseconds(ESC_CALIBRATE_HIGH);
    motorFR.writeMicroseconds(ESC_CALIBRATE_HIGH);
    motorRR.writeMicroseconds(ESC_CALIBRATE_HIGH);
    motorRL.writeMicroseconds(ESC_CALIBRATE_HIGH);
    
    beep(2000, 100);
    delay(3000);
    
    // Step 2: Send minimum throttle
    Serial.println(F("Sending MIN throttle..."));
    motorFL.writeMicroseconds(ESC_CALIBRATE_LOW);
    motorFR.writeMicroseconds(ESC_CALIBRATE_LOW);
    motorRR.writeMicroseconds(ESC_CALIBRATE_LOW);
    motorRL.writeMicroseconds(ESC_CALIBRATE_LOW);
    
    beep(1500, 100);
    delay(2000);
    
    // Step 3: Test each motor
    Serial.println(F("Testing motors..."));
    
    uint16_t testPulse = 1150; // Low speed test
    
    // Test FL
    Serial.println(F("Motor FL..."));
    beep(BEEP_ESC_FREQ, BEEP_ESC_DUR);
    for (int i = 0; i < 50; i++) {
        motorFL.writeMicroseconds(ESC_MIN_PULSE + (testPulse - ESC_MIN_PULSE) * i / 50);
        delay(20);
    }
    delay(500);
    motorFL.writeMicroseconds(ESC_MIN_PULSE);
    delay(500);
    
    // Test FR
    Serial.println(F("Motor FR..."));
    beep(BEEP_ESC_FREQ, BEEP_ESC_DUR);
    for (int i = 0; i < 50; i++) {
        motorFR.writeMicroseconds(ESC_MIN_PULSE + (testPulse - ESC_MIN_PULSE) * i / 50);
        delay(20);
    }
    delay(500);
    motorFR.writeMicroseconds(ESC_MIN_PULSE);
    delay(500);
    
    // Test RR
    Serial.println(F("Motor RR..."));
    beep(BEEP_ESC_FREQ, BEEP_ESC_DUR);
    for (int i = 0; i < 50; i++) {
        motorRR.writeMicroseconds(ESC_MIN_PULSE + (testPulse - ESC_MIN_PULSE) * i / 50);
        delay(20);
    }
    delay(500);
    motorRR.writeMicroseconds(ESC_MIN_PULSE);
    delay(500);
    
    // Test RL
    Serial.println(F("Motor RL..."));
    beep(BEEP_ESC_FREQ, BEEP_ESC_DUR);
    for (int i = 0; i < 50; i++) {
        motorRL.writeMicroseconds(ESC_MIN_PULSE + (testPulse - ESC_MIN_PULSE) * i / 50);
        delay(20);
    }
    delay(500);
    motorRL.writeMicroseconds(ESC_MIN_PULSE);
    
    escCalibrated = true;
    systemStatus = STATUS_ESC_CALIBRATED;
    currentLEDPattern = LED_BLINK_SLOW;
    
    Serial.println(F("ESC Calibration Complete!"));
    
    // Confirmation beep pattern (different from IMU)
    beep(1000, 100);
    delay(100);
    beep(1500, 100);
    delay(100);
    beep(2000, 100);
}

// ============================================================================
// READ IMU DATA
// ============================================================================

void readIMU() {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x3B); // Starting register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, 14, true);
    
    // Read accelerometer
    accelX = Wire.read() << 8 | Wire.read();
    accelY = Wire.read() << 8 | Wire.read();
    accelZ = Wire.read() << 8 | Wire.read();
    
    // Read temperature
    temperature = Wire.read() << 8 | Wire.read();
    
    // Read gyroscope
    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();
}

// ============================================================================
// CALCULATE ANGLES (COMPLEMENTARY FILTER)
// ============================================================================

void calculateAngles(float dt) {
    // Apply calibration offsets
    float gx = (gyroX - gyroOffsetX) / 65.5;  // Convert to deg/s (±500°/s range)
    float gy = (gyroY - gyroOffsetY) / 65.5;
    float gz = (gyroZ - gyroOffsetZ) / 65.5;
    
    float ax = accelX - accelOffsetX;
    float ay = accelY - accelOffsetY;
    float az = accelZ - accelOffsetZ;
    
    // Store gyro rates for PID
    gyroRollRate = gx;
    gyroPitchRate = gy;
    gyroYawRate = gz;
    
    // Calculate angles from accelerometer
    float accelRoll = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    
    // Complementary filter
    angleRoll = COMP_FILTER_ALPHA * (angleRoll + gx * dt) + (1.0 - COMP_FILTER_ALPHA) * accelRoll;
    anglePitch = COMP_FILTER_ALPHA * (anglePitch + gy * dt) + (1.0 - COMP_FILTER_ALPHA) * accelPitch;
    
    // Yaw from gyro integration only (no magnetometer)
    angleYaw += gz * dt;
    
    // Normalize yaw to ±180
    if (angleYaw > 180) angleYaw -= 360;
    if (angleYaw < -180) angleYaw += 360;
}

// ============================================================================
// PID CONTROLLER
// ============================================================================

void calculatePID(float dt) {
    // ========== ROLL PID ==========
    rollError = rollSetpoint - angleRoll;
    rollErrorI += rollError * dt;
    rollErrorI = constrain(rollErrorI, -PID_I_MAX, PID_I_MAX); // Anti-windup
    float rollErrorD = (rollError - rollErrorPrev) / dt;
    rollErrorPrev = rollError;
    
    rollPID = PID_ROLL_KP * rollError + 
              PID_ROLL_KI * rollErrorI + 
              PID_ROLL_KD * (-gyroRollRate); // D term on measurement
    
    // ========== PITCH PID ==========
    pitchError = pitchSetpoint - anglePitch;
    pitchErrorI += pitchError * dt;
    pitchErrorI = constrain(pitchErrorI, -PID_I_MAX, PID_I_MAX);
    float pitchErrorD = (pitchError - pitchErrorPrev) / dt;
    pitchErrorPrev = pitchError;
    
    pitchPID = PID_PITCH_KP * pitchError + 
               PID_PITCH_KI * pitchErrorI + 
               PID_PITCH_KD * (-gyroPitchRate);
    
    // ========== YAW PID (Rate-based) ==========
    yawError = yawSetpoint - gyroYawRate;
    yawErrorI += yawError * dt;
    yawErrorI = constrain(yawErrorI, -PID_I_MAX, PID_I_MAX);
    yawErrorPrev = yawError;
    
    yawPID = PID_YAW_KP * yawError + 
             PID_YAW_KI * yawErrorI;
    
    // Constrain PID outputs
    rollPID = constrain(rollPID, -PID_MAX_OUTPUT, PID_MAX_OUTPUT);
    pitchPID = constrain(pitchPID, -PID_MAX_OUTPUT, PID_MAX_OUTPUT);
    yawPID = constrain(yawPID, -PID_MAX_OUTPUT, PID_MAX_OUTPUT);
}

// ============================================================================
// MOTOR MIXING (X-QUAD CONFIGURATION)
// ============================================================================

void mixMotors() {
    // Apply throttle cap for safety (65% max)
    uint16_t throttle = map(throttleSetpoint, THROTTLE_MIN, THROTTLE_MAX, 
                            THROTTLE_MIN, THROTTLE_CAP);
    
    // X-quad motor mixing
    // FL (CW):  +Pitch, +Roll, +Yaw
    // FR (CCW): +Pitch, -Roll, -Yaw
    // RR (CW):  -Pitch, -Roll, +Yaw
    // RL (CCW): -Pitch, +Roll, -Yaw
    
    int16_t fl = throttle + pitchPID + rollPID + yawPID;
    int16_t fr = throttle + pitchPID - rollPID - yawPID;
    int16_t rr = throttle - pitchPID - rollPID + yawPID;
    int16_t rl = throttle - pitchPID + rollPID - yawPID;
    
    // Constrain to valid ESC range
    motorPulseFL = constrain(fl, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorPulseFR = constrain(fr, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorPulseRR = constrain(rr, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorPulseRL = constrain(rl, ESC_MIN_PULSE, ESC_MAX_PULSE);
    
    // Safety: If throttle is at minimum, ensure motors are off
    if (throttleSetpoint < THROTTLE_IDLE) {
        motorPulseFL = ESC_MIN_PULSE;
        motorPulseFR = ESC_MIN_PULSE;
        motorPulseRR = ESC_MIN_PULSE;
        motorPulseRL = ESC_MIN_PULSE;
    }
}

// ============================================================================
// UPDATE MOTORS
// ============================================================================

void updateMotors() {
    if (motorsArmed) {
        motorFL.writeMicroseconds(motorPulseFL);
        motorFR.writeMicroseconds(motorPulseFR);
        motorRR.writeMicroseconds(motorPulseRR);
        motorRL.writeMicroseconds(motorPulseRL);
    } else {
        // Motors disarmed - send minimum pulse
        motorFL.writeMicroseconds(ESC_MIN_PULSE);
        motorFR.writeMicroseconds(ESC_MIN_PULSE);
        motorRR.writeMicroseconds(ESC_MIN_PULSE);
        motorRL.writeMicroseconds(ESC_MIN_PULSE);
    }
}

// ============================================================================
// RADIO COMMUNICATION
// ============================================================================

void processRadio() {
    // Check for incoming packets
    if (radio.available()) {
        // Read packet
        radio.read(&rcPacket, sizeof(rcPacket));
        
        // Validate packet
        if (validateRCPacket(&rcPacket)) {
            lastPacketTime = millis();
            
            // Update connection status
            if (!nrfConnected) {
                nrfConnected = true;
                currentLEDPattern = LED_BLINK_SLOW;
                Serial.println(F("RC Connected!"));
            }
            
            // Process command
            processCommand();
            
            // Prepare ACK payload for next transmission
            updateTelemetry();
            radio.writeAckPayload(1, &telemetry, sizeof(telemetry));
        }
    }
}

void processCommand() {
    // Extract control inputs
    throttleSetpoint = rcPacket.throttle;
    yawSetpoint = rcPacket.yaw / 10.0;     // Convert to deg/s
    pitchSetpoint = rcPacket.pitch / 10.0;  // Convert to degrees
    rollSetpoint = rcPacket.roll / 10.0;    // Convert to degrees
    
    // Apply angle limits
    pitchSetpoint = constrain(pitchSetpoint, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    rollSetpoint = constrain(rollSetpoint, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
    yawSetpoint = constrain(yawSetpoint, -MAX_YAW_RATE, MAX_YAW_RATE);
    
    // Read switch states
    sw1_altHold = rcPacket.aux1;
    sw2_armSwitch = rcPacket.aux2;
    
    // Detect button edges
    bool btn1Current = rcPacket.btn1;
    bool btn2Current = rcPacket.btn2;
    
    // Button 1: Calibration (rising edge)
    if (btn1Current && !prevBtn1) {
        if (!motorsArmed) {
            calibrateIMU();
        }
    }
    prevBtn1 = btn1Current;
    
    // Button 2: ESC Calibration / Motor On (rising edge)
    if (btn2Current && !prevBtn2) {
        if (!sw1_altHold && !motorsArmed) {
            // SW1 is OFF - ESC calibration mode
            calibrateESC();
        }
    }
    prevBtn2 = btn2Current;
    
    // Switch 2: Arm/Disarm (Kill Switch)
    if (!sw2_armSwitch) {
        // Kill switch activated - IMMEDIATE DISARM
        if (motorsArmed) {
            motorsArmed = false;
            systemStatus = STATUS_DISARMED;
            currentLEDPattern = LED_BLINK_SLOW;
            Serial.println(F("DISARMED (Kill Switch)"));
            beep(1000, 200);
        }
    } else {
        // Arm switch is in ARM position
        // Check if we can arm (button 2 pressed, calibrated, throttle low)
        if (btn2Current && !motorsArmed && imuCalibrated && 
            throttleSetpoint < THROTTLE_IDLE && sw1_altHold) {
            motorsArmed = true;
            systemStatus = STATUS_ARMED;
            currentLEDPattern = LED_BLINK_DOUBLE;
            Serial.println(F("ARMED!"));
            beep(BEEP_ARM_FREQ, BEEP_ARM_DUR);
            delay(100);
            beep(BEEP_ARM_FREQ, BEEP_ARM_DUR);
        }
    }
    
    // Update flying status
    if (motorsArmed && throttleSetpoint > THROTTLE_IDLE) {
        systemStatus = STATUS_FLYING;
        currentLEDPattern = LED_SOLID;
    } else if (motorsArmed) {
        systemStatus = STATUS_ARMED;
        currentLEDPattern = LED_BLINK_DOUBLE;
    }
}

// ============================================================================
// FAILSAFE
// ============================================================================

void handleFailsafe() {
    unsigned long timeSincePacket = millis() - lastPacketTime;
    
    // Check for signal loss
    if (timeSincePacket > FAILSAFE_TIMEOUT_MS) {
        if (nrfConnected) {
            nrfConnected = false;
            Serial.println(F("RC Signal LOST!"));
            currentLEDPattern = LED_BLINK_FAST;
        }
        
        // Failsafe action: gradual throttle reduction
        if (motorsArmed) {
            throttleSetpoint = max(THROTTLE_MIN, throttleSetpoint - 2);
            
            // Auto-disarm if throttle reaches minimum
            if (throttleSetpoint <= THROTTLE_MIN) {
                motorsArmed = false;
                systemStatus = STATUS_DISARMED;
                Serial.println(F("DISARMED (Failsafe)"));
                beep(800, 500);
            }
        }
    }
}

// ============================================================================
// TELEMETRY UPDATE
// ============================================================================

void updateTelemetry() {
    telemetry.header = 0xBB;
    telemetry.status = systemStatus;
    telemetry.roll_angle = (int16_t)(angleRoll * 10);
    telemetry.pitch_angle = (int16_t)(anglePitch * 10);
    telemetry.yaw_angle = (int16_t)(angleYaw * 10);
    telemetry.altitude = 0; // No altitude sensor
    telemetry.battery_mv = 0; // No battery sensor
    telemetry.motor_fl = motorPulseFL;
    telemetry.motor_fr = motorPulseFR;
    telemetry.motor_rr = motorPulseRR;
    telemetry.motor_rl = motorPulseRL;
    telemetry.signal_quality = nrfConnected ? 100 : 0;
    telemetry.error_code = 0;
    telemetry.sequence = packetSequence++;
    telemetry.checksum = calculateChecksum((uint8_t*)&telemetry, sizeof(telemetry));
}

// ============================================================================
// LED CONTROL
// ============================================================================

void updateLED() {
    unsigned long now = millis();
    
    switch (currentLEDPattern) {
        case LED_OFF:
            digitalWrite(LED_STATUS_PIN, LOW);
            break;
            
        case LED_ON:
        case LED_SOLID:
            digitalWrite(LED_STATUS_PIN, HIGH);
            break;
            
        case LED_BLINK_SLOW:
            if (now - ledTimer > 1000) {
                ledTimer = now;
                ledState = !ledState;
                digitalWrite(LED_STATUS_PIN, ledState);
            }
            break;
            
        case LED_BLINK_FAST:
            if (now - ledTimer > 250) {
                ledTimer = now;
                ledState = !ledState;
                digitalWrite(LED_STATUS_PIN, ledState);
            }
            break;
            
        case LED_BLINK_DOUBLE:
            // Double blink pattern
            {
                unsigned long phase = (now / 100) % 10;
                if (phase == 0 || phase == 2) {
                    digitalWrite(LED_STATUS_PIN, HIGH);
                } else {
                    digitalWrite(LED_STATUS_PIN, LOW);
                }
            }
            break;
    }
}

// ============================================================================
// BUZZER CONTROL
// ============================================================================

void beep(uint16_t frequency, uint16_t duration) {
    tone(BUZZER_PIN, frequency, duration);
    delay(duration);
    noTone(BUZZER_PIN);
}

void playBuzzerPattern(uint8_t pattern) {
    switch (pattern) {
        case 0: // Startup
            beep(2000, 100);
            delay(50);
            beep(2500, 100);
            break;
            
        case 1: // Calibration OK
            beep(2000, 100);
            delay(100);
            beep(2000, 100);
            break;
            
        case 2: // Calibration Fail
            beep(1000, 7000);
            break;
            
        case 3: // Armed
            beep(2500, 150);
            delay(100);
            beep(2500, 150);
            break;
            
        case 4: // Disarmed
            beep(1000, 200);
            break;
            
        case 5: // ESC test
            beep(1500, 200);
            break;
    }
}
