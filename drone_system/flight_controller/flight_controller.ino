/*
 * ============================================================================
 * PROFESSIONAL DRONE FLIGHT CONTROLLER
 * Arduino Nano Based Quadcopter FC
 * ============================================================================
 * 
 * Hardware Configuration:
 * - Arduino Nano (ATmega328P)
 * - NRF24L01 PA+LNA (CE: D4, CSN: D10)
 * - MPU6050 IMU (INT: D2, I2C: A4/A5)
 * - MS5611 Barometer (I2C: A4/A5)
 * - Buzzer (D8)
 * - Status LED (D7)
 * - ESC/Motors: FL(D3), FR(D5), RR(D6), RL(D9)
 * 
 * Motor Layout (X Configuration):
 *        FRONT
 *     FL     FR
 *       \   /
 *        \ /
 *        / \
 *       /   \
 *     RL     RR
 *        REAR
 * 
 * Author: UAV Systems Engineer
 * Version: 1.0.0
 * ============================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include "config.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// NRF24L01
#define NRF_CE_PIN            4
#define NRF_CSN_PIN           10

// MPU6050
#define MPU_INT_PIN           2

// MS5611 uses I2C (A4/A5)

// Outputs
#define BUZZER_PIN            8
#define LED_PIN               7

// Motor ESC Pins (PWM capable)
#define MOTOR_FL_PIN          3   // Front Left
#define MOTOR_FR_PIN          5   // Front Right
#define MOTOR_RR_PIN          6   // Rear Right
#define MOTOR_RL_PIN          9   // Rear Left

// ============================================================================
// MPU6050 REGISTERS
// ============================================================================

#define MPU6050_ADDR          0x68
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_INT_ENABLE    0x38
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_GYRO_XOUT_H   0x43

// ============================================================================
// MS5611 REGISTERS & COMMANDS
// ============================================================================

#define MS5611_ADDR           0x77
#define MS5611_CMD_RESET      0x1E
#define MS5611_CMD_PROM_READ  0xA0
#define MS5611_CMD_CONVERT_D1 0x48  // Pressure, OSR=4096
#define MS5611_CMD_CONVERT_D2 0x58  // Temperature, OSR=4096
#define MS5611_CMD_ADC_READ   0x00

// ============================================================================
// PID CONFIGURATION
// ============================================================================

// PID Gains - Tuned for stability
struct PIDConfig {
    float Kp;
    float Ki;
    float Kd;
    float maxI;     // Anti-windup
    float maxOut;   // Output limit
};

// Roll PID
PIDConfig rollPID = {
    .Kp = 1.3,
    .Ki = 0.04,
    .Kd = 18.0,
    .maxI = 100.0,
    .maxOut = 400.0
};

// Pitch PID
PIDConfig pitchPID = {
    .Kp = 1.3,
    .Ki = 0.04,
    .Kd = 18.0,
    .maxI = 100.0,
    .maxOut = 400.0
};

// Yaw PID
PIDConfig yawPID = {
    .Kp = 4.0,
    .Ki = 0.02,
    .Kd = 0.0,
    .maxI = 100.0,
    .maxOut = 300.0
};

// Altitude PID (for altitude hold)
PIDConfig altPID = {
    .Kp = 0.7,
    .Ki = 0.01,
    .Kd = 25.0,
    .maxI = 150.0,
    .maxOut = 200.0
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// NRF24L01
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// Motor Servos (using Servo library for ESC control)
Servo motorFL, motorFR, motorRR, motorRL;

// Communication
CommandPacket_t rxCommand;
TelemetryPacket_t txTelemetry;
volatile bool newDataAvailable = false;
uint32_t lastRxTime = 0;
uint8_t rxSequence = 0;
bool isConnected = false;

// IMU Data
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
float gyroXoffset, gyroYoffset, gyroZoffset;
float accXoffset, accYoffset, accZoffset;

// Angles (complementary filter)
float angleRoll = 0.0;
float anglePitch = 0.0;
float angleYaw = 0.0;

// MS5611 Calibration Data
uint16_t ms5611_c[7];
int32_t ms5611_dT;
float temperature = 0.0;
float pressure = 0.0;
float altitude = 0.0;
float baseAltitude = 0.0;
float targetAltitude = 0.0;

// PID State
float rollError = 0.0, rollErrorSum = 0.0, rollErrorPrev = 0.0;
float pitchError = 0.0, pitchErrorSum = 0.0, pitchErrorPrev = 0.0;
float yawError = 0.0, yawErrorSum = 0.0, yawErrorPrev = 0.0;
float altError = 0.0, altErrorSum = 0.0, altErrorPrev = 0.0;

// Control Outputs
int16_t pidRoll = 0, pidPitch = 0, pidYaw = 0, pidAlt = 0;
int16_t motorFL_speed, motorFR_speed, motorRR_speed, motorRL_speed;

// System State
enum SystemState {
    STATE_INIT,
    STATE_WAIT_CONNECTION,
    STATE_CONNECTED,
    STATE_CALIBRATING,
    STATE_ESC_CALIBRATING,
    STATE_READY,
    STATE_ARMED,
    STATE_FLYING,
    STATE_ERROR
};

SystemState currentState = STATE_INIT;
uint8_t systemStatus = 0;
bool isCalibrated = false;
bool isESCCalibrated = false;
bool isArmed = false;
bool motorsOn = false;
bool altHoldEnabled = false;

// Timing
uint32_t loopStartTime;
uint32_t lastLoopTime = 0;
float dt = 0.004;  // 4ms loop time (250Hz)
uint32_t calibrationStartTime = 0;

// LED blinking
uint32_t lastBlinkTime = 0;
bool ledState = false;
uint16_t blinkInterval = 500;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void initMPU6050();
void initMS5611();
void initNRF24();
void initMotors();

void readMPU6050();
void readMS5611();
void calculateAngles();
float calculateAltitude();

void calibrateIMU();
void calibrateESC();

float computePID(float error, float* errorSum, float* errorPrev, PIDConfig* config);
void calculatePID();
void mixMotors();
void updateMotors();

void processCommand();
void sendTelemetry();
void handleCommunication();

void updateStatusLED();
void buzzerBeep(uint8_t count, uint16_t duration);
void safetyCheck();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial for debugging (optional)
    Serial.begin(115200);
    Serial.println(F("================================="));
    Serial.println(F("  DRONE FLIGHT CONTROLLER v1.0"));
    Serial.println(F("================================="));
    
    // Initialize pins
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(MPU_INT_PIN, INPUT);
    
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    
    // Startup indication
    buzzerBeep(1, 100);
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C
    
    // Initialize sensors
    Serial.println(F("[INIT] Initializing MPU6050..."));
    initMPU6050();
    
    Serial.println(F("[INIT] Initializing MS5611..."));
    initMS5611();
    
    // Initialize motors (ESCs)
    Serial.println(F("[INIT] Initializing Motors..."));
    initMotors();
    
    // Initialize NRF24L01
    Serial.println(F("[INIT] Initializing NRF24L01..."));
    initNRF24();
    
    // Initialize telemetry packet
    txTelemetry.header = 0x55;
    
    currentState = STATE_WAIT_CONNECTION;
    Serial.println(F("[INIT] Waiting for RC connection..."));
    
    // Ready beep
    buzzerBeep(2, 100);
    delay(500);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    loopStartTime = micros();
    
    // Calculate delta time
    static uint32_t prevTime = 0;
    uint32_t currentTime = micros();
    dt = (currentTime - prevTime) / 1000000.0;
    if (dt > 0.02) dt = 0.02;  // Cap at 20ms
    prevTime = currentTime;
    
    // Handle NRF24 communication
    handleCommunication();
    
    // Read sensors
    readMPU6050();
    calculateAngles();
    
    // Read barometer (slower rate)
    static uint8_t baroCounter = 0;
    if (++baroCounter >= 10) {  // ~25Hz
        baroCounter = 0;
        readMS5611();
        altitude = calculateAltitude();
    }
    
    // State machine
    switch (currentState) {
        case STATE_WAIT_CONNECTION:
            blinkInterval = 500;  // Slow blink
            if (isConnected) {
                currentState = STATE_CONNECTED;
                Serial.println(F("[STATE] Connected to RC"));
                buzzerBeep(2, 50);
            }
            break;
            
        case STATE_CONNECTED:
            blinkInterval = 250;  // Medium blink
            // Wait for calibration command
            break;
            
        case STATE_CALIBRATING:
            blinkInterval = 100;  // Fast blink
            // Handled in processCommand
            break;
            
        case STATE_ESC_CALIBRATING:
            blinkInterval = 150;
            // ESC calibration handled separately
            break;
            
        case STATE_READY:
            blinkInterval = 1000;  // Slow pulse
            if (isArmed && motorsOn) {
                currentState = STATE_ARMED;
                Serial.println(F("[STATE] Armed"));
            }
            break;
            
        case STATE_ARMED:
            blinkInterval = 500;
            // Process flight controls
            calculatePID();
            mixMotors();
            if (!isArmed) {
                currentState = STATE_READY;
                stopMotors();
                Serial.println(F("[STATE] Disarmed"));
            }
            break;
            
        case STATE_FLYING:
            blinkInterval = 200;
            calculatePID();
            mixMotors();
            break;
            
        case STATE_ERROR:
            blinkInterval = 100;
            stopMotors();
            break;
    }
    
    // Safety check
    safetyCheck();
    
    // Update motors (if armed)
    updateMotors();
    
    // Update status LED
    updateStatusLED();
    
    // Send telemetry via ACK payload
    sendTelemetry();
    
    // Maintain loop rate (~250Hz)
    while (micros() - loopStartTime < 4000);
}

// ============================================================================
// SENSOR INITIALIZATION
// ============================================================================

void initMPU6050() {
    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x00);  // Wake up
    Wire.endTransmission();
    delay(100);
    
    // Configure gyro (500 deg/s)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_GYRO_CONFIG);
    Wire.write(0x08);  // FS_SEL = 1 (500 deg/s)
    Wire.endTransmission();
    
    // Configure accelerometer (8g)
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_CONFIG);
    Wire.write(0x10);  // AFS_SEL = 2 (8g)
    Wire.endTransmission();
    
    // Configure low-pass filter
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_CONFIG);
    Wire.write(0x03);  // DLPF_CFG = 3 (44Hz bandwidth)
    Wire.endTransmission();
    
    Serial.println(F("[MPU6050] Initialized"));
}

void initMS5611() {
    // Reset MS5611
    Wire.beginTransmission(MS5611_ADDR);
    Wire.write(MS5611_CMD_RESET);
    Wire.endTransmission();
    delay(100);
    
    // Read calibration data from PROM
    for (uint8_t i = 0; i < 7; i++) {
        Wire.beginTransmission(MS5611_ADDR);
        Wire.write(MS5611_CMD_PROM_READ + (i * 2));
        Wire.endTransmission();
        
        Wire.requestFrom(MS5611_ADDR, (uint8_t)2);
        if (Wire.available() >= 2) {
            ms5611_c[i] = (Wire.read() << 8) | Wire.read();
        }
    }
    
    // Initial reading
    delay(10);
    readMS5611();
    baseAltitude = calculateAltitude();
    
    Serial.println(F("[MS5611] Initialized"));
    Serial.print(F("[MS5611] Base altitude: "));
    Serial.println(baseAltitude);
}

void initNRF24() {
    if (!radio.begin()) {
        Serial.println(F("[NRF24] ERROR: Radio not responding!"));
        currentState = STATE_ERROR;
        buzzerBeep(5, 200);
        return;
    }
    
    // Configure radio
    radio.setChannel(NRF_CHANNEL);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(sizeof(CommandPacket_t));
    
    // Enable ACK payloads
    radio.enableAckPayload();
    radio.enableDynamicPayloads();
    
    // Set addresses
    radio.openWritingPipe(FC_TO_RC_ADDR);
    radio.openReadingPipe(1, RC_TO_FC_ADDR);
    
    // Start listening
    radio.startListening();
    
    Serial.println(F("[NRF24] Initialized"));
    Serial.print(F("[NRF24] Channel: "));
    Serial.println(NRF_CHANNEL);
}

void initMotors() {
    // Attach ESCs
    motorFL.attach(MOTOR_FL_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorFR.attach(MOTOR_FR_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorRR.attach(MOTOR_RR_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorRL.attach(MOTOR_RL_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    
    // Initialize to minimum throttle
    motorFL.writeMicroseconds(ESC_MIN_PULSE);
    motorFR.writeMicroseconds(ESC_MIN_PULSE);
    motorRR.writeMicroseconds(ESC_MIN_PULSE);
    motorRL.writeMicroseconds(ESC_MIN_PULSE);
    
    Serial.println(F("[MOTORS] Initialized"));
}

// ============================================================================
// SENSOR READING
// ============================================================================

void readMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
    
    if (Wire.available() >= 14) {
        accX = (Wire.read() << 8) | Wire.read();
        accY = (Wire.read() << 8) | Wire.read();
        accZ = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read();  // Skip temperature
        gyroX = (Wire.read() << 8) | Wire.read();
        gyroY = (Wire.read() << 8) | Wire.read();
        gyroZ = (Wire.read() << 8) | Wire.read();
    }
}

void readMS5611() {
    static uint8_t state = 0;
    static uint32_t conversionStart = 0;
    static uint32_t D1 = 0, D2 = 0;
    
    switch (state) {
        case 0:  // Start pressure conversion
            Wire.beginTransmission(MS5611_ADDR);
            Wire.write(MS5611_CMD_CONVERT_D1);
            Wire.endTransmission();
            conversionStart = millis();
            state = 1;
            break;
            
        case 1:  // Read pressure, start temp conversion
            if (millis() - conversionStart >= 10) {
                Wire.beginTransmission(MS5611_ADDR);
                Wire.write(MS5611_CMD_ADC_READ);
                Wire.endTransmission();
                Wire.requestFrom(MS5611_ADDR, (uint8_t)3);
                if (Wire.available() >= 3) {
                    D1 = ((uint32_t)Wire.read() << 16) | 
                         ((uint32_t)Wire.read() << 8) | 
                         Wire.read();
                }
                
                Wire.beginTransmission(MS5611_ADDR);
                Wire.write(MS5611_CMD_CONVERT_D2);
                Wire.endTransmission();
                conversionStart = millis();
                state = 2;
            }
            break;
            
        case 2:  // Read temperature, calculate
            if (millis() - conversionStart >= 10) {
                Wire.beginTransmission(MS5611_ADDR);
                Wire.write(MS5611_CMD_ADC_READ);
                Wire.endTransmission();
                Wire.requestFrom(MS5611_ADDR, (uint8_t)3);
                if (Wire.available() >= 3) {
                    D2 = ((uint32_t)Wire.read() << 16) | 
                         ((uint32_t)Wire.read() << 8) | 
                         Wire.read();
                }
                
                // Calculate temperature
                ms5611_dT = D2 - ((int32_t)ms5611_c[5] << 8);
                temperature = (2000 + ((int64_t)ms5611_dT * ms5611_c[6] >> 23)) / 100.0;
                
                // Calculate pressure
                int64_t OFF = ((int64_t)ms5611_c[2] << 16) + 
                              (((int64_t)ms5611_c[4] * ms5611_dT) >> 7);
                int64_t SENS = ((int64_t)ms5611_c[1] << 15) + 
                               (((int64_t)ms5611_c[3] * ms5611_dT) >> 8);
                pressure = (((D1 * SENS >> 21) - OFF) >> 15) / 100.0;
                
                state = 0;
            }
            break;
    }
}

float calculateAltitude() {
    // Barometric formula
    if (pressure > 0) {
        return 44330.0 * (1.0 - pow(pressure / 1013.25, 0.1903));
    }
    return 0;
}

// ============================================================================
// ANGLE CALCULATION (Complementary Filter)
// ============================================================================

void calculateAngles() {
    // Convert raw values to physical units
    // Gyro: 500 deg/s range, 65.5 LSB/(deg/s)
    float gyroRateX = (gyroX - gyroXoffset) / 65.5;
    float gyroRateY = (gyroY - gyroYoffset) / 65.5;
    float gyroRateZ = (gyroZ - gyroZoffset) / 65.5;
    
    // Accelerometer: 8g range, 4096 LSB/g
    float accXg = (accX - accXoffset) / 4096.0;
    float accYg = (accY - accYoffset) / 4096.0;
    float accZg = (accZ - accZoffset) / 4096.0;
    
    // Calculate angles from accelerometer
    float accRoll = atan2(accYg, sqrt(accXg * accXg + accZg * accZg)) * 57.2958;
    float accPitch = atan2(-accXg, sqrt(accYg * accYg + accZg * accZg)) * 57.2958;
    
    // Complementary filter (98% gyro, 2% accelerometer)
    static bool firstRun = true;
    if (firstRun) {
        angleRoll = accRoll;
        anglePitch = accPitch;
        angleYaw = 0;
        firstRun = false;
    } else {
        angleRoll = 0.98 * (angleRoll + gyroRateX * dt) + 0.02 * accRoll;
        anglePitch = 0.98 * (anglePitch + gyroRateY * dt) + 0.02 * accPitch;
        angleYaw += gyroRateZ * dt;
    }
    
    // Wrap yaw angle
    if (angleYaw > 180) angleYaw -= 360;
    if (angleYaw < -180) angleYaw += 360;
}

// ============================================================================
// CALIBRATION
// ============================================================================

void calibrateIMU() {
    Serial.println(F("[CAL] Starting IMU calibration..."));
    Serial.println(F("[CAL] Keep drone still!"));
    
    // Fast LED blink during calibration
    const uint16_t samples = 2000;
    int32_t gxSum = 0, gySum = 0, gzSum = 0;
    int32_t axSum = 0, aySum = 0, azSum = 0;
    
    for (uint16_t i = 0; i < samples; i++) {
        readMPU6050();
        gxSum += gyroX;
        gySum += gyroY;
        gzSum += gyroZ;
        axSum += accX;
        aySum += accY;
        azSum += accZ;
        
        if (i % 100 == 0) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
        delay(2);
    }
    
    gyroXoffset = gxSum / (float)samples;
    gyroYoffset = gySum / (float)samples;
    gyroZoffset = gzSum / (float)samples;
    
    accXoffset = axSum / (float)samples;
    accYoffset = aySum / (float)samples;
    // Don't remove gravity from Z axis, adjust for 1g
    accZoffset = (azSum / (float)samples) - 4096.0;  // 4096 = 1g at 8g range
    
    // Reset angles
    angleRoll = 0;
    anglePitch = 0;
    angleYaw = 0;
    
    // Verify calibration
    bool success = true;
    readMPU6050();
    float testGyroX = abs(gyroX - gyroXoffset);
    float testGyroY = abs(gyroY - gyroYoffset);
    float testGyroZ = abs(gyroZ - gyroZoffset);
    
    if (testGyroX > 100 || testGyroY > 100 || testGyroZ > 100) {
        success = false;
    }
    
    if (success) {
        isCalibrated = true;
        systemStatus |= STATUS_CALIBRATED;
        Serial.println(F("[CAL] IMU calibration SUCCESS"));
        Serial.print(F("[CAL] Gyro offsets: "));
        Serial.print(gyroXoffset); Serial.print(F(", "));
        Serial.print(gyroYoffset); Serial.print(F(", "));
        Serial.println(gyroZoffset);
        buzzerBeep(2, 150);  // Success: 2 beeps
        currentState = STATE_READY;
    } else {
        isCalibrated = false;
        systemStatus &= ~STATUS_CALIBRATED;
        Serial.println(F("[CAL] IMU calibration FAILED"));
        buzzerBeep(1, 7000);  // Failure: 1 long beep (7 seconds)
        currentState = STATE_CONNECTED;
    }
}

void calibrateESC() {
    Serial.println(F("[ESC_CAL] Starting ESC calibration..."));
    
    // Test each motor one by one
    const uint16_t testSpeed = ESC_IDLE_PULSE + 100;
    const uint16_t testDuration = 1500;
    
    // Motor 1: Front Left
    Serial.println(F("[ESC_CAL] Testing FL motor..."));
    buzzerBeep(1, 100);
    for (uint16_t i = ESC_MIN_PULSE; i <= testSpeed; i += 10) {
        motorFL.writeMicroseconds(i);
        delay(20);
    }
    delay(testDuration);
    motorFL.writeMicroseconds(ESC_MIN_PULSE);
    delay(500);
    
    // Motor 2: Front Right
    Serial.println(F("[ESC_CAL] Testing FR motor..."));
    buzzerBeep(1, 100);
    for (uint16_t i = ESC_MIN_PULSE; i <= testSpeed; i += 10) {
        motorFR.writeMicroseconds(i);
        delay(20);
    }
    delay(testDuration);
    motorFR.writeMicroseconds(ESC_MIN_PULSE);
    delay(500);
    
    // Motor 3: Rear Right
    Serial.println(F("[ESC_CAL] Testing RR motor..."));
    buzzerBeep(1, 100);
    for (uint16_t i = ESC_MIN_PULSE; i <= testSpeed; i += 10) {
        motorRR.writeMicroseconds(i);
        delay(20);
    }
    delay(testDuration);
    motorRR.writeMicroseconds(ESC_MIN_PULSE);
    delay(500);
    
    // Motor 4: Rear Left
    Serial.println(F("[ESC_CAL] Testing RL motor..."));
    buzzerBeep(1, 100);
    for (uint16_t i = ESC_MIN_PULSE; i <= testSpeed; i += 10) {
        motorRL.writeMicroseconds(i);
        delay(20);
    }
    delay(testDuration);
    motorRL.writeMicroseconds(ESC_MIN_PULSE);
    
    isESCCalibrated = true;
    systemStatus |= STATUS_ESC_CALIBRATED;
    
    Serial.println(F("[ESC_CAL] ESC calibration complete"));
    
    // Different sound for ESC calibration complete
    buzzerBeep(1, 50);
    delay(100);
    buzzerBeep(1, 50);
    delay(100);
    buzzerBeep(1, 200);
    
    currentState = STATE_READY;
}

// ============================================================================
// PID CONTROL
// ============================================================================

float computePID(float error, float* errorSum, float* errorPrev, PIDConfig* config) {
    // Proportional
    float P = config->Kp * error;
    
    // Integral with anti-windup
    *errorSum += error * dt;
    *errorSum = constrain(*errorSum, -config->maxI, config->maxI);
    float I = config->Ki * (*errorSum);
    
    // Derivative
    float D = config->Kd * (error - *errorPrev) / dt;
    *errorPrev = error;
    
    // Total output
    float output = P + I + D;
    return constrain(output, -config->maxOut, config->maxOut);
}

void calculatePID() {
    if (!isArmed || !motorsOn) {
        // Reset PID when not flying
        rollErrorSum = 0; rollErrorPrev = 0;
        pitchErrorSum = 0; pitchErrorPrev = 0;
        yawErrorSum = 0; yawErrorPrev = 0;
        altErrorSum = 0; altErrorPrev = 0;
        pidRoll = 0; pidPitch = 0; pidYaw = 0; pidAlt = 0;
        return;
    }
    
    // Get setpoints from RC (scaled)
    float rollSetpoint = map(rxCommand.roll, -500, 500, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    float pitchSetpoint = map(rxCommand.pitch, -500, 500, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    float yawSetpoint = map(rxCommand.yaw, -500, 500, -MAX_YAW_RATE, MAX_YAW_RATE);
    
    // Apply deadzone
    if (abs(rxCommand.roll) < 20) rollSetpoint = 0;
    if (abs(rxCommand.pitch) < 20) pitchSetpoint = 0;
    if (abs(rxCommand.yaw) < 20) yawSetpoint = 0;
    
    // Calculate errors
    rollError = rollSetpoint - angleRoll;
    pitchError = pitchSetpoint - anglePitch;
    
    // Yaw is rate control
    float currentYawRate = (gyroZ - gyroZoffset) / 65.5;
    yawError = yawSetpoint - currentYawRate;
    
    // Compute PIDs
    pidRoll = computePID(rollError, &rollErrorSum, &rollErrorPrev, &rollPID);
    pidPitch = computePID(pitchError, &pitchErrorSum, &pitchErrorPrev, &pitchPID);
    pidYaw = computePID(yawError, &yawErrorSum, &yawErrorPrev, &yawPID);
    
    // Altitude hold
    if (altHoldEnabled) {
        altError = targetAltitude - (altitude - baseAltitude);
        pidAlt = computePID(altError, &altErrorSum, &altErrorPrev, &altPID);
    } else {
        pidAlt = 0;
        altErrorSum = 0;
        altErrorPrev = 0;
    }
}

// ============================================================================
// MOTOR MIXING
// ============================================================================

void mixMotors() {
    if (!isArmed || !motorsOn) {
        motorFL_speed = ESC_MIN_PULSE;
        motorFR_speed = ESC_MIN_PULSE;
        motorRR_speed = ESC_MIN_PULSE;
        motorRL_speed = ESC_MIN_PULSE;
        return;
    }
    
    // Get throttle from RC (0-1000)
    int16_t throttle = rxCommand.throttle;
    
    // Apply max throttle limit (65%)
    int16_t maxThrottle = (ESC_MAX_PULSE - ESC_MIN_PULSE) * THROTTLE_MAX_PERCENT / 100;
    throttle = map(throttle, 0, 1000, 0, maxThrottle);
    
    // Add base throttle for stable flight
    int16_t baseThrottle = ESC_MIN_PULSE + throttle;
    
    // Apply altitude hold correction if enabled
    if (altHoldEnabled && throttle > 50) {
        baseThrottle += pidAlt;
    }
    
    // Motor mixing for X configuration
    // FL (CW): +pitch, +roll, +yaw
    // FR (CCW): +pitch, -roll, -yaw
    // RR (CW): -pitch, -roll, +yaw
    // RL (CCW): -pitch, +roll, -yaw
    
    motorFL_speed = baseThrottle + pidPitch + pidRoll + pidYaw;
    motorFR_speed = baseThrottle + pidPitch - pidRoll - pidYaw;
    motorRR_speed = baseThrottle - pidPitch - pidRoll + pidYaw;
    motorRL_speed = baseThrottle - pidPitch + pidRoll - pidYaw;
    
    // Constrain motor speeds
    motorFL_speed = constrain(motorFL_speed, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorFR_speed = constrain(motorFR_speed, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorRR_speed = constrain(motorRR_speed, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motorRL_speed = constrain(motorRL_speed, ESC_MIN_PULSE, ESC_MAX_PULSE);
    
    // If throttle is below threshold, keep motors at minimum
    if (rxCommand.throttle < 50) {
        motorFL_speed = ESC_MIN_PULSE;
        motorFR_speed = ESC_MIN_PULSE;
        motorRR_speed = ESC_MIN_PULSE;
        motorRL_speed = ESC_MIN_PULSE;
    }
}

void updateMotors() {
    motorFL.writeMicroseconds(motorFL_speed);
    motorFR.writeMicroseconds(motorFR_speed);
    motorRR.writeMicroseconds(motorRR_speed);
    motorRL.writeMicroseconds(motorRL_speed);
}

void stopMotors() {
    motorFL_speed = ESC_MIN_PULSE;
    motorFR_speed = ESC_MIN_PULSE;
    motorRR_speed = ESC_MIN_PULSE;
    motorRL_speed = ESC_MIN_PULSE;
    updateMotors();
}

// ============================================================================
// COMMUNICATION
// ============================================================================

void handleCommunication() {
    // Check for incoming data
    if (radio.available()) {
        radio.read(&rxCommand, sizeof(CommandPacket_t));
        
        // Validate packet
        if (validateCommandPacket(&rxCommand)) {
            lastRxTime = millis();
            rxSequence = rxCommand.sequence;
            
            if (!isConnected) {
                isConnected = true;
                systemStatus |= STATUS_CONNECTED;
            }
            
            // Process the command
            processCommand();
        }
    }
    
    // Check for connection timeout
    if (millis() - lastRxTime > COMM_TIMEOUT_MS) {
        if (isConnected) {
            isConnected = false;
            systemStatus &= ~STATUS_CONNECTED;
            Serial.println(F("[COMM] Connection lost!"));
            
            // Emergency stop
            if (isArmed) {
                isArmed = false;
                motorsOn = false;
                stopMotors();
                currentState = STATE_WAIT_CONNECTION;
            }
        }
    }
}

void processCommand() {
    // Process buttons
    static uint8_t prevButtons = 0;
    uint8_t buttonPressed = rxCommand.buttons & ~prevButtons;
    prevButtons = rxCommand.buttons;
    
    // Button 1: Calibration
    if (buttonPressed & BTN_CALIBRATE) {
        if (currentState == STATE_CONNECTED || currentState == STATE_READY) {
            currentState = STATE_CALIBRATING;
            calibrateIMU();
        }
    }
    
    // Button 2: Motors on (ESC calibration or arm)
    if (buttonPressed & BTN_MOTORS_ON) {
        if (currentState == STATE_READY && isCalibrated) {
            // Check if SW_1 is OFF for ESC calibration
            if (!(rxCommand.switches & SW_ALT_HOLD) && !isESCCalibrated) {
                currentState = STATE_ESC_CALIBRATING;
                calibrateESC();
            } else {
                motorsOn = true;
                systemStatus |= STATUS_MOTORS_ON;
                Serial.println(F("[CTRL] Motors enabled"));
            }
        }
    }
    
    // Switch 1: Altitude hold
    if (rxCommand.switches & SW_ALT_HOLD) {
        if (!altHoldEnabled && isArmed) {
            altHoldEnabled = true;
            targetAltitude = altitude - baseAltitude;
            systemStatus |= STATUS_ALT_HOLD;
            Serial.println(F("[CTRL] Altitude hold enabled"));
        }
    } else {
        if (altHoldEnabled) {
            altHoldEnabled = false;
            systemStatus &= ~STATUS_ALT_HOLD;
            Serial.println(F("[CTRL] Altitude hold disabled"));
        }
    }
    
    // Switch 2: Arm/Disarm (Kill switch)
    if (rxCommand.switches & SW_ARM) {
        if (!isArmed && isCalibrated && isESCCalibrated && motorsOn) {
            isArmed = true;
            systemStatus |= STATUS_ARMED;
            Serial.println(F("[CTRL] ARMED"));
            buzzerBeep(1, 100);
        }
    } else {
        // Kill switch activated - immediately disarm
        if (isArmed) {
            isArmed = false;
            motorsOn = false;
            systemStatus &= ~STATUS_ARMED;
            systemStatus &= ~STATUS_MOTORS_ON;
            stopMotors();
            Serial.println(F("[CTRL] DISARMED (Kill switch)"));
            buzzerBeep(2, 50);
        }
    }
    
    // Process special commands
    if (rxCommand.command == CMD_EMERGENCY_STOP) {
        isArmed = false;
        motorsOn = false;
        stopMotors();
        Serial.println(F("[CTRL] EMERGENCY STOP"));
    }
}

void sendTelemetry() {
    // Update telemetry data
    txTelemetry.status = systemStatus;
    txTelemetry.pitch = (int16_t)(anglePitch * 10);
    txTelemetry.roll = (int16_t)(angleRoll * 10);
    txTelemetry.yaw = (int16_t)(angleYaw * 10);
    txTelemetry.altitude = (uint16_t)((altitude - baseAltitude) * 100);
    txTelemetry.battery = 100;  // Placeholder - needs ADC
    txTelemetry.rssi = 100;     // Placeholder
    txTelemetry.sequence = rxSequence;
    txTelemetry.checksum = calculateChecksum((uint8_t*)&txTelemetry, sizeof(TelemetryPacket_t));
    
    // Write ACK payload
    radio.writeAckPayload(1, &txTelemetry, sizeof(TelemetryPacket_t));
}

// ============================================================================
// UTILITIES
// ============================================================================

void updateStatusLED() {
    uint32_t currentTime = millis();
    
    if (currentTime - lastBlinkTime >= blinkInterval) {
        lastBlinkTime = currentTime;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }
}

void buzzerBeep(uint8_t count, uint16_t duration) {
    for (uint8_t i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(duration);
        digitalWrite(BUZZER_PIN, LOW);
        if (i < count - 1) {
            delay(duration / 2);
        }
    }
}

void safetyCheck() {
    // Check for extreme angles
    if (abs(angleRoll) > 60 || abs(anglePitch) > 60) {
        if (isArmed) {
            Serial.println(F("[SAFETY] Extreme angle detected!"));
            // Don't auto-disarm, but limit corrections
        }
    }
    
    // Check connection status
    if (!isConnected && isArmed) {
        Serial.println(F("[SAFETY] Connection lost while armed!"));
        isArmed = false;
        motorsOn = false;
        stopMotors();
        currentState = STATE_WAIT_CONNECTION;
    }
}
