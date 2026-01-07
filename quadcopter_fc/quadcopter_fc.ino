/**
 * ============================================================================
 *                    QUADCOPTER FLIGHT CONTROLLER v6.0
 *                      PROFESSIONAL STABILITY EDITION
 * ============================================================================
 * 
 * Target: F330 / RS2205 2300KV / 3S 1500mAh
 * 
 * FEATURES:
 *   ✓ Complementary filter (tunable, no DMP dependency)
 *   ✓ Cascaded PID: Outer angle loop + Inner rate loop
 *   ✓ Gyro low-pass filtering
 *   ✓ D-term filtering (prevents oscillation)
 *   ✓ Motor output smoothing
 *   ✓ Setpoint smoothing (stick filtering)
 *   ✓ Anti-windup on integral
 *   ✓ Working rate pot
 *   ✓ Level calibration saved to EEPROM
 *   ✓ Comprehensive debugging
 * 
 * ============================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <EEPROM.h>

// ============================================================================
//                              CONFIGURATION
// ============================================================================
#define RF_CHANNEL          108
#define SERIAL_BAUD         115200

// ============================================================================
//                            PIN DEFINITIONS
// ============================================================================
#define PIN_MOTOR_FL        3       // Front-Left  CCW
#define PIN_MOTOR_FR        5       // Front-Right CW
#define PIN_MOTOR_RL        6       // Rear-Left   CW
#define PIN_MOTOR_RR        9       // Rear-Right  CCW
#define PIN_RF_CE           4
#define PIN_RF_CSN          10
#define PIN_LED             7
#define PIN_BUZZER          8

// MPU6050
#define MPU_ADDR            0x68
#define MPU_PWR_MGMT_1      0x6B
#define MPU_SMPLRT_DIV      0x19
#define MPU_CONFIG          0x1A
#define MPU_GYRO_CONFIG     0x1B
#define MPU_ACCEL_CONFIG    0x1C
#define MPU_ACCEL_XOUT_H    0x3B

// ============================================================================
//                          FLIGHT PARAMETERS
// ============================================================================
#define MAX_ANGLE           45.0f       // Max tilt angle (degrees)
#define MAX_RATE            300.0f      // Max rotation rate (deg/s)
#define MAX_YAW_RATE        200.0f      // Max yaw rate (deg/s)

#define ESC_MIN             1000
#define ESC_MAX             2000
#define ESC_IDLE            1080        // Idle speed
#define ESC_ARM_THR         50          // Max throttle to arm

#define RF_TIMEOUT_MS       500
#define LOOP_TIME_US        4000        // 250Hz main loop

// ============================================================================
//                         SENSOR CONFIGURATION
// ============================================================================
// Gyro scale: 500 deg/s = 65.5 LSB/deg/s
#define GYRO_SCALE          65.5f

// Accel scale: ±8g = 4096 LSB/g
#define ACCEL_SCALE         4096.0f

// Complementary filter coefficient (0.90-0.99)
// Higher = trust gyro more, Lower = trust accel more
#define COMP_FILTER_ALPHA   0.996f

// Gyro low-pass filter coefficient (0.0-1.0)
// Lower = more filtering, Higher = faster response
#define GYRO_LPF_ALPHA      0.7f

// ============================================================================
//                         CALIBRATION (EEPROM)
// ============================================================================
#define EEPROM_MAGIC        0xFC
#define EEPROM_ADDR         0

struct CalibrationData {
    uint8_t magic;
    int16_t gyroOffsetX;
    int16_t gyroOffsetY;
    int16_t gyroOffsetZ;
    int16_t accelOffsetX;
    int16_t accelOffsetY;
    int16_t accelOffsetZ;
    float levelRollOffset;
    float levelPitchOffset;
};

CalibrationData cal = {
    EEPROM_MAGIC,
    0, 0, 0,        // Gyro offsets
    0, 0, 0,        // Accel offsets
    0.0f, 0.0f      // Level offsets
};

// ============================================================================
//                              PID TUNING
// ============================================================================
// === OUTER LOOP (Angle) ===
// Converts angle error to desired rate
#define ANGLE_KP_ROLL       5.0f
#define ANGLE_KP_PITCH      5.0f

// === INNER LOOP (Rate) ===
// Converts rate error to motor output
#define RATE_KP_ROLL        0.8f
#define RATE_KI_ROLL        0.003f
#define RATE_KD_ROLL        0.015f

#define RATE_KP_PITCH       0.8f
#define RATE_KI_PITCH       0.003f
#define RATE_KD_PITCH       0.015f

#define RATE_KP_YAW         2.0f
#define RATE_KI_YAW         0.01f
#define RATE_KD_YAW         0.0f

// PID limits
#define RATE_I_MAX          100.0f      // Integral windup limit
#define PID_OUTPUT_MAX      400.0f      // Max PID output

// D-term low-pass filter
#define DTERM_LPF_ALPHA     0.5f

// ============================================================================
//                         MOTOR SMOOTHING
// ============================================================================
// Motor output filter (0.0-1.0, lower = smoother)
#define MOTOR_LPF_ALPHA     0.6f

// Setpoint filter (stick smoothing)
#define SETPOINT_LPF_ALPHA  0.4f

// ============================================================================
//                          RF PACKET STRUCTURE
// ============================================================================
struct __attribute__((packed)) RxPacket {
    uint16_t throttle;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint8_t switches;
    uint8_t checksum;
    uint32_t seq;
    uint8_t ratePot;
    uint8_t expoPot;
    
    bool valid() const {
        uint8_t c = 0;
        const uint8_t* d = (const uint8_t*)this;
        for (int i = 0; i < 9; i++) c ^= d[i];
        return c == checksum;
    }
};

#define SW_ARM      0
#define SW_CALIB    1
#define SW_MOTOR    2
#define SW_ALTHOLD  3
#define SW_BEEPER   4
#define SW_HEADLESS 5

// ============================================================================
//                           GLOBAL OBJECTS
// ============================================================================
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
Servo escFL, escFR, escRL, escRR;
const uint8_t radioAddr[6] = "QUAD1";

// ============================================================================
//                         SENSOR VARIABLES
// ============================================================================
// Raw sensor data
int16_t accelRawX, accelRawY, accelRawZ;
int16_t gyroRawX, gyroRawY, gyroRawZ;
int16_t tempRaw;

// Filtered gyro rates (deg/s)
float gyroRateX = 0, gyroRateY = 0, gyroRateZ = 0;
float gyroRateX_prev = 0, gyroRateY_prev = 0, gyroRateZ_prev = 0;

// Attitude (degrees)
float roll = 0, pitch = 0, yaw = 0;

// ============================================================================
//                         RADIO VARIABLES
// ============================================================================
RxPacket rx;
uint32_t lastRxTime = 0;
uint32_t rxCount = 0;
bool rfConnected = false;

// ============================================================================
//                         FLIGHT STATE
// ============================================================================
enum FlightState { DISARMED, ARMED, FAILSAFE };
FlightState flightState = DISARMED;

// Commands (filtered)
float throttleCmd = 0;
float rollCmd = 0, pitchCmd = 0, yawCmd = 0;

// Rate setpoints (from angle PID or direct rate mode)
float rollRateSetpoint = 0, pitchRateSetpoint = 0, yawRateSetpoint = 0;

// PID state
float rollRateI = 0, pitchRateI = 0, yawRateI = 0;
float rollRateD_prev = 0, pitchRateD_prev = 0, yawRateD_prev = 0;
float rollPID = 0, pitchPID = 0, yawPID = 0;

// Motor outputs
float motorFL = ESC_MIN, motorFR = ESC_MIN;
float motorRL = ESC_MIN, motorRR = ESC_MIN;

// Rate multiplier from pot
float rateMultiplier = 1.0f;

// State tracking
bool prevArmSwitch = false;
bool prevCalibBtn = false;
bool prevMotorBtn = false;
bool prevBeeperBtn = false;
bool prevHeadlessBtn = false;
bool headlessMode = false;
float headlessRef = 0;

// Motor test
bool motorTestActive = false;
uint32_t motorTestStart = 0;
uint8_t motorTestPhase = 0;

// Level calibration
bool levelCalActive = false;
uint32_t calibBtnHoldTime = 0;

// Timing
uint32_t loopTime = 0;
uint32_t debugTime = 0;
uint32_t ledTime = 0;
bool ledState = false;

// ============================================================================
//                            BUZZER
// ============================================================================
void beep(uint16_t ms, uint16_t freq = 2000) {
    tone(PIN_BUZZER, freq, ms);
}

void beepWait(uint16_t ms, uint16_t freq = 2000) {
    tone(PIN_BUZZER, freq);
    delay(ms);
    noTone(PIN_BUZZER);
}

// ============================================================================
//                         EEPROM FUNCTIONS
// ============================================================================
void loadCalibration() {
    EEPROM.get(EEPROM_ADDR, cal);
    if (cal.magic != EEPROM_MAGIC) {
        // First run - set defaults
        cal.magic = EEPROM_MAGIC;
        cal.gyroOffsetX = cal.gyroOffsetY = cal.gyroOffsetZ = 0;
        cal.accelOffsetX = cal.accelOffsetY = cal.accelOffsetZ = 0;
        cal.levelRollOffset = cal.levelPitchOffset = 0;
        saveCalibration();
    }
    Serial.println(F("Calibration loaded:"));
    Serial.print(F("  Gyro: ")); 
    Serial.print(cal.gyroOffsetX); Serial.print(F(",")); 
    Serial.print(cal.gyroOffsetY); Serial.print(F(","));
    Serial.println(cal.gyroOffsetZ);
    Serial.print(F("  Level: R=")); Serial.print(cal.levelRollOffset, 2);
    Serial.print(F(" P=")); Serial.println(cal.levelPitchOffset, 2);
}

void saveCalibration() {
    EEPROM.put(EEPROM_ADDR, cal);
    Serial.println(F("Calibration saved!"));
}

// ============================================================================
//                         MPU6050 FUNCTIONS
// ============================================================================
void mpuWrite(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

void mpuInit() {
    // Wake up MPU
    mpuWrite(MPU_PWR_MGMT_1, 0x00);
    delay(100);
    
    // Sample rate = 1kHz / (1 + SMPLRT_DIV)
    mpuWrite(MPU_SMPLRT_DIV, 0x03);     // 250Hz sample rate
    
    // DLPF = 3 (44Hz bandwidth)
    mpuWrite(MPU_CONFIG, 0x03);
    
    // Gyro config: ±500 deg/s
    mpuWrite(MPU_GYRO_CONFIG, 0x08);
    
    // Accel config: ±8g
    mpuWrite(MPU_ACCEL_CONFIG, 0x10);
    
    delay(50);
}

bool mpuRead() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
    if (Wire.available() < 14) return false;
    
    accelRawX = (Wire.read() << 8) | Wire.read();
    accelRawY = (Wire.read() << 8) | Wire.read();
    accelRawZ = (Wire.read() << 8) | Wire.read();
    tempRaw = (Wire.read() << 8) | Wire.read();
    gyroRawX = (Wire.read() << 8) | Wire.read();
    gyroRawY = (Wire.read() << 8) | Wire.read();
    gyroRawZ = (Wire.read() << 8) | Wire.read();
    
    return true;
}

void mpuCalibrate() {
    Serial.println(F("Calibrating gyro... KEEP STILL!"));
    beepWait(500, 1500);
    delay(500);
    
    int32_t sumGX = 0, sumGY = 0, sumGZ = 0;
    int32_t sumAX = 0, sumAY = 0, sumAZ = 0;
    int samples = 500;
    
    for (int i = 0; i < samples; i++) {
        if (mpuRead()) {
            sumGX += gyroRawX;
            sumGY += gyroRawY;
            sumGZ += gyroRawZ;
            sumAX += accelRawX;
            sumAY += accelRawY;
            sumAZ += accelRawZ;
        }
        delay(4);
    }
    
    cal.gyroOffsetX = sumGX / samples;
    cal.gyroOffsetY = sumGY / samples;
    cal.gyroOffsetZ = sumGZ / samples;
    cal.accelOffsetX = sumAX / samples;
    cal.accelOffsetY = sumAY / samples;
    cal.accelOffsetZ = (sumAZ / samples) - (int16_t)ACCEL_SCALE; // Remove 1g
    
    saveCalibration();
    
    Serial.print(F("Gyro offsets: "));
    Serial.print(cal.gyroOffsetX); Serial.print(F(", "));
    Serial.print(cal.gyroOffsetY); Serial.print(F(", "));
    Serial.println(cal.gyroOffsetZ);
    
    beepWait(200, 2500);
    delay(100);
    beepWait(200, 3000);
}

// ============================================================================
//                    COMPLEMENTARY FILTER
// ============================================================================
void updateAttitude(float dt) {
    // Apply calibration offsets
    float gx = (gyroRawX - cal.gyroOffsetX) / GYRO_SCALE;
    float gy = (gyroRawY - cal.gyroOffsetY) / GYRO_SCALE;
    float gz = (gyroRawZ - cal.gyroOffsetZ) / GYRO_SCALE;
    
    float ax = (accelRawX - cal.accelOffsetX) / ACCEL_SCALE;
    float ay = (accelRawY - cal.accelOffsetY) / ACCEL_SCALE;
    float az = (accelRawZ - cal.accelOffsetZ) / ACCEL_SCALE;
    
    // Low-pass filter on gyro rates
    gyroRateX = gyroRateX_prev * (1.0f - GYRO_LPF_ALPHA) + gx * GYRO_LPF_ALPHA;
    gyroRateY = gyroRateY_prev * (1.0f - GYRO_LPF_ALPHA) + gy * GYRO_LPF_ALPHA;
    gyroRateZ = gyroRateZ_prev * (1.0f - GYRO_LPF_ALPHA) + gz * GYRO_LPF_ALPHA;
    gyroRateX_prev = gyroRateX;
    gyroRateY_prev = gyroRateY;
    gyroRateZ_prev = gyroRateZ;
    
    // Accelerometer angles (only valid when not accelerating)
    float accelRoll = atan2(ay, az) * 57.2958f;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958f;
    
    // Complementary filter
    // Gyro integration (fast but drifts)
    roll += gyroRateX * dt;
    pitch += gyroRateY * dt;
    yaw += gyroRateZ * dt;
    
    // Fuse with accelerometer (slow but stable)
    roll = COMP_FILTER_ALPHA * roll + (1.0f - COMP_FILTER_ALPHA) * accelRoll;
    pitch = COMP_FILTER_ALPHA * pitch + (1.0f - COMP_FILTER_ALPHA) * accelPitch;
    
    // Apply level calibration
    roll -= cal.levelRollOffset;
    pitch -= cal.levelPitchOffset;
    
    // Normalize yaw to ±180
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
}

// ============================================================================
//                         LEVEL CALIBRATION
// ============================================================================
void doLevelCalibration() {
    Serial.println(F("\n*** LEVEL CALIBRATION ***"));
    Serial.println(F("Place drone on FLAT surface!"));
    beepWait(1000, 1500);
    delay(1000);
    
    // Temporarily remove level offsets
    float tempRollOff = cal.levelRollOffset;
    float tempPitchOff = cal.levelPitchOffset;
    cal.levelRollOffset = 0;
    cal.levelPitchOffset = 0;
    
    // Average attitude over 2 seconds
    float sumRoll = 0, sumPitch = 0;
    int count = 0;
    uint32_t start = millis();
    
    while (millis() - start < 2000) {
        if (mpuRead()) {
            updateAttitude(0.004f);
            sumRoll += roll;
            sumPitch += pitch;
            count++;
        }
        delay(4);
    }
    
    if (count > 100) {
        cal.levelRollOffset = sumRoll / count;
        cal.levelPitchOffset = sumPitch / count;
        saveCalibration();
        
        Serial.print(F("Level offsets: R="));
        Serial.print(cal.levelRollOffset, 2);
        Serial.print(F(" P="));
        Serial.println(cal.levelPitchOffset, 2);
        
        beepWait(200, 2000);
        delay(100);
        beepWait(200, 2500);
        delay(100);
        beepWait(400, 3000);
    } else {
        // Restore old values
        cal.levelRollOffset = tempRollOff;
        cal.levelPitchOffset = tempPitchOff;
        Serial.println(F("FAILED!"));
        beepWait(500, 500);
    }
    
    levelCalActive = false;
}

// ============================================================================
//                        ESC CALIBRATION
// ============================================================================
void escCalibration() {
    Serial.println(F("\n*** ESC CALIBRATION ***"));
    
    escFL.writeMicroseconds(ESC_MAX);
    escFR.writeMicroseconds(ESC_MAX);
    escRL.writeMicroseconds(ESC_MAX);
    escRR.writeMicroseconds(ESC_MAX);
    
    beepWait(1000, 3000);
    Serial.println(F("ESCs at MAX - release button after ESC beeps"));
    
    uint32_t start = millis();
    while (millis() - start < 15000) {
        if (radio.available()) {
            RxPacket p;
            radio.read(&p, sizeof(p));
            if (p.valid() && !(p.switches & (1 << SW_CALIB))) break;
        }
        delay(50);
    }
    
    escFL.writeMicroseconds(ESC_MIN);
    escFR.writeMicroseconds(ESC_MIN);
    escRL.writeMicroseconds(ESC_MIN);
    escRR.writeMicroseconds(ESC_MIN);
    
    beepWait(500, 1500);
    delay(2000);
    
    Serial.println(F("ESC Calibration DONE!"));
    beepWait(200, 2000);
    beepWait(200, 2500);
    beepWait(400, 3000);
}

// ============================================================================
//                           RADIO UPDATE
// ============================================================================
void updateRadio() {
    while (radio.available()) {
        RxPacket p;
        radio.read(&p, sizeof(p));
        
        if (p.valid()) {
            rx = p;
            lastRxTime = millis();
            rxCount++;
            
            if (!rfConnected) {
                rfConnected = true;
                Serial.println(F("\n*** RF CONNECTED ***"));
                for (int i = 0; i < 5; i++) { beepWait(60, 2500); delay(60); }
            }
        }
    }
    
    // Timeout
    if (rfConnected && millis() - lastRxTime > RF_TIMEOUT_MS) {
        rfConnected = false;
        Serial.println(F("\n*** RF LOST ***"));
        if (flightState == ARMED) {
            flightState = FAILSAFE;
            beep(500, 800);
        }
    }
}

// ============================================================================
//                         MOTOR TEST
// ============================================================================
void runMotorTest() {
    uint32_t t = millis() - motorTestStart;
    uint16_t speed = ESC_IDLE;
    
    motorFL = motorFR = motorRL = motorRR = ESC_MIN;
    
    if (t < 600) { motorFL = speed; motorTestPhase = 1; }
    else if (t < 1200) { motorFR = speed; motorTestPhase = 2; }
    else if (t < 1800) { motorRL = speed; motorTestPhase = 3; }
    else if (t < 2400) { motorRR = speed; motorTestPhase = 4; }
    else if (t < 3200) { 
        motorFL = motorFR = motorRL = motorRR = speed; 
        motorTestPhase = 5; 
    }
    else {
        motorTestActive = false;
        motorTestPhase = 0;
        motorFL = motorFR = motorRL = motorRR = ESC_MIN;
        beep(200, 2500);
        Serial.println(F("Motor test DONE"));
    }
    
    escFL.writeMicroseconds((uint16_t)motorFL);
    escFR.writeMicroseconds((uint16_t)motorFR);
    escRL.writeMicroseconds((uint16_t)motorRL);
    escRR.writeMicroseconds((uint16_t)motorRR);
}

// ============================================================================
//                        PROCESS COMMANDS
// ============================================================================
void processCommands() {
    if (!rfConnected) {
        throttleCmd = 0;
        rollCmd = pitchCmd = yawCmd = 0;
        return;
    }
    
    // Rate multiplier from pot (0.5 to 1.5)
    rateMultiplier = 0.5f + (rx.ratePot / 255.0f);
    
    // Raw commands
    float rawThrottle = rx.throttle;
    float rawRoll = (rx.roll / 500.0f) * MAX_ANGLE * rateMultiplier;
    float rawPitch = (rx.pitch / 500.0f) * MAX_ANGLE * rateMultiplier;
    float rawYaw = (rx.yaw / 500.0f) * MAX_YAW_RATE * rateMultiplier;
    
    // Smooth setpoints (low-pass filter on stick inputs)
    throttleCmd = throttleCmd * (1.0f - SETPOINT_LPF_ALPHA) + rawThrottle * SETPOINT_LPF_ALPHA;
    rollCmd = rollCmd * (1.0f - SETPOINT_LPF_ALPHA) + rawRoll * SETPOINT_LPF_ALPHA;
    pitchCmd = pitchCmd * (1.0f - SETPOINT_LPF_ALPHA) + rawPitch * SETPOINT_LPF_ALPHA;
    yawCmd = yawCmd * (1.0f - SETPOINT_LPF_ALPHA) + rawYaw * SETPOINT_LPF_ALPHA;
    
    // Parse switches
    bool armSw = rx.switches & (1 << SW_ARM);
    bool calibBtn = rx.switches & (1 << SW_CALIB);
    bool motorBtn = rx.switches & (1 << SW_MOTOR);
    bool beeperBtn = rx.switches & (1 << SW_BEEPER);
    bool headlessBtn = rx.switches & (1 << SW_HEADLESS);
    
    // === BEEPER ===
    if (beeperBtn && !prevBeeperBtn) {
        beep(500, 2500);
    } else if (beeperBtn) {
        static uint32_t lastBeep = 0;
        if (millis() - lastBeep > 600) { beep(500, 2500); lastBeep = millis(); }
    }
    prevBeeperBtn = beeperBtn;
    
    // === HEADLESS ===
    if (headlessBtn && !prevHeadlessBtn) {
        headlessMode = !headlessMode;
        if (headlessMode) headlessRef = yaw;
        Serial.print(F("Headless: ")); Serial.println(headlessMode ? F("ON") : F("OFF"));
        beep(100, headlessMode ? 2500 : 1500);
    }
    prevHeadlessBtn = headlessBtn;
    
    // Apply headless transformation
    if (headlessMode) {
        float diff = (yaw - headlessRef) * 0.01745329f;
        float c = cos(diff), s = sin(diff);
        float r = rollCmd, p = pitchCmd;
        rollCmd = r * c + p * s;
        pitchCmd = -r * s + p * c;
    }
    
    // === CALIBRATION BUTTON ===
    // Long press (3s) = level calibration
    // Short press = motor test
    if (calibBtn && !prevCalibBtn) {
        calibBtnHoldTime = millis();
    }
    if (calibBtn && millis() - calibBtnHoldTime > 3000 && !levelCalActive && flightState == DISARMED) {
        levelCalActive = true;
        beep(1000, 1500);
        Serial.println(F("Release for LEVEL CAL"));
    }
    if (!calibBtn && prevCalibBtn) {
        if (levelCalActive) {
            doLevelCalibration();
        } else if (flightState == DISARMED && !motorTestActive) {
            motorTestActive = true;
            motorTestStart = millis();
            Serial.println(F("MOTOR TEST"));
            beep(100, 2000);
        }
    }
    prevCalibBtn = calibBtn;
    
    // === MOTOR TEST BUTTON ===
    if (motorBtn && !prevMotorBtn && flightState == DISARMED && !motorTestActive) {
        motorTestActive = true;
        motorTestStart = millis();
        Serial.println(F("MOTOR TEST"));
        beep(100, 2000);
    }
    prevMotorBtn = motorBtn;
    
    // === ARM/DISARM ===
    if (armSw && !prevArmSwitch && flightState == DISARMED) {
        if (throttleCmd <= ESC_ARM_THR && rfConnected) {
            flightState = ARMED;
            
            // Reset PID
            rollRateI = pitchRateI = yawRateI = 0;
            rollRateD_prev = pitchRateD_prev = yawRateD_prev = 0;
            rollPID = pitchPID = yawPID = 0;
            
            headlessRef = yaw;
            motorFL = motorFR = motorRL = motorRR = ESC_MIN;
            
            Serial.println(F("*** ARMED ***"));
            beepWait(100, 2000);
            delay(100);
            beepWait(200, 2500);
        } else {
            beep(300, 500);
        }
    }
    if (!armSw && flightState == ARMED) {
        flightState = DISARMED;
        Serial.println(F("*** DISARMED ***"));
        beepWait(300, 1500);
    }
    
    // Failsafe recovery
    if (flightState == FAILSAFE && (!armSw || (rfConnected && throttleCmd < 100))) {
        flightState = DISARMED;
        beep(200, 1500);
    }
    
    prevArmSwitch = armSw;
}

// ============================================================================
//                           CASCADED PID
// ============================================================================
void updatePID(float dt) {
    if (flightState != ARMED) {
        rollPID = pitchPID = yawPID = 0;
        rollRateI = pitchRateI = yawRateI = 0;
        return;
    }
    
    // === OUTER LOOP: Angle to Rate ===
    // Simple P-controller converts angle error to desired rate
    float rollAngleError = rollCmd - roll;
    float pitchAngleError = pitchCmd - pitch;
    
    rollRateSetpoint = ANGLE_KP_ROLL * rollAngleError;
    pitchRateSetpoint = ANGLE_KP_PITCH * pitchAngleError;
    yawRateSetpoint = yawCmd;  // Yaw is direct rate control
    
    // Limit rate setpoints
    rollRateSetpoint = constrain(rollRateSetpoint, -MAX_RATE, MAX_RATE);
    pitchRateSetpoint = constrain(pitchRateSetpoint, -MAX_RATE, MAX_RATE);
    yawRateSetpoint = constrain(yawRateSetpoint, -MAX_YAW_RATE, MAX_YAW_RATE);
    
    // === INNER LOOP: Rate PID ===
    // ROLL
    float rollRateError = rollRateSetpoint - gyroRateX;
    rollRateI += rollRateError * dt;
    rollRateI = constrain(rollRateI, -RATE_I_MAX, RATE_I_MAX);
    float rollRateD = (rollRateError - rollRateD_prev) / dt;
    rollRateD = rollRateD_prev * (1.0f - DTERM_LPF_ALPHA) + rollRateD * DTERM_LPF_ALPHA;
    rollRateD_prev = rollRateError;
    rollPID = RATE_KP_ROLL * rollRateError + RATE_KI_ROLL * rollRateI + RATE_KD_ROLL * rollRateD;
    rollPID = constrain(rollPID, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
    
    // PITCH
    float pitchRateError = pitchRateSetpoint - gyroRateY;
    pitchRateI += pitchRateError * dt;
    pitchRateI = constrain(pitchRateI, -RATE_I_MAX, RATE_I_MAX);
    float pitchRateD = (pitchRateError - pitchRateD_prev) / dt;
    pitchRateD = pitchRateD_prev * (1.0f - DTERM_LPF_ALPHA) + pitchRateD * DTERM_LPF_ALPHA;
    pitchRateD_prev = pitchRateError;
    pitchPID = RATE_KP_PITCH * pitchRateError + RATE_KI_PITCH * pitchRateI + RATE_KD_PITCH * pitchRateD;
    pitchPID = constrain(pitchPID, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
    
    // YAW
    float yawRateError = yawRateSetpoint - gyroRateZ;
    yawRateI += yawRateError * dt;
    yawRateI = constrain(yawRateI, -RATE_I_MAX, RATE_I_MAX);
    yawPID = RATE_KP_YAW * yawRateError + RATE_KI_YAW * yawRateI;
    yawPID = constrain(yawPID, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
}

// ============================================================================
//                           MOTOR MIXING
// ============================================================================
void updateMotors() {
    if (motorTestActive) {
        runMotorTest();
        return;
    }
    
    if (flightState != ARMED) {
        motorFL = motorFR = motorRL = motorRR = ESC_MIN;
        escFL.writeMicroseconds(ESC_MIN);
        escFR.writeMicroseconds(ESC_MIN);
        escRL.writeMicroseconds(ESC_MIN);
        escRR.writeMicroseconds(ESC_MIN);
        return;
    }
    
    // Throttle mapping
    float thr = ESC_IDLE + (throttleCmd / 1000.0f) * (ESC_MAX - ESC_IDLE);
    
    // Quad-X motor mixing
    float targetFL = thr - rollPID + pitchPID - yawPID;
    float targetFR = thr + rollPID + pitchPID + yawPID;
    float targetRL = thr - rollPID - pitchPID + yawPID;
    float targetRR = thr + rollPID - pitchPID - yawPID;
    
    // Constrain
    targetFL = constrain(targetFL, ESC_IDLE, ESC_MAX);
    targetFR = constrain(targetFR, ESC_IDLE, ESC_MAX);
    targetRL = constrain(targetRL, ESC_IDLE, ESC_MAX);
    targetRR = constrain(targetRR, ESC_IDLE, ESC_MAX);
    
    // Low-pass filter on motor outputs (prevents sudden changes)
    motorFL = motorFL * (1.0f - MOTOR_LPF_ALPHA) + targetFL * MOTOR_LPF_ALPHA;
    motorFR = motorFR * (1.0f - MOTOR_LPF_ALPHA) + targetFR * MOTOR_LPF_ALPHA;
    motorRL = motorRL * (1.0f - MOTOR_LPF_ALPHA) + targetRL * MOTOR_LPF_ALPHA;
    motorRR = motorRR * (1.0f - MOTOR_LPF_ALPHA) + targetRR * MOTOR_LPF_ALPHA;
    
    // Write to ESCs
    escFL.writeMicroseconds((uint16_t)motorFL);
    escFR.writeMicroseconds((uint16_t)motorFR);
    escRL.writeMicroseconds((uint16_t)motorRL);
    escRR.writeMicroseconds((uint16_t)motorRR);
}

// ============================================================================
//                            LED UPDATE
// ============================================================================
void updateLED() {
    uint16_t interval;
    
    if (flightState == ARMED) {
        interval = headlessMode ? 150 : 0;
    } else if (flightState == FAILSAFE) {
        interval = 80;
    } else if (motorTestActive) {
        interval = 100;
    } else {
        interval = 500;
    }
    
    if (interval == 0) {
        digitalWrite(PIN_LED, HIGH);
    } else if (millis() - ledTime >= interval) {
        ledTime = millis();
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState);
    }
}

// ============================================================================
//                           DEBUG OUTPUT
// ============================================================================
void printDebug() {
    // State
    if (flightState == ARMED) Serial.print(F("ARM "));
    else if (flightState == FAILSAFE) Serial.print(F("FAIL"));
    else Serial.print(F("DIS "));
    
    // RF
    Serial.print(F(" RF:")); Serial.print(rfConnected ? rxCount : 0);
    
    // Rate pot
    Serial.print(F(" Rate:")); Serial.print((int)(rateMultiplier * 100)); Serial.print(F("%"));
    
    // Switches
    Serial.print(F(" SW:"));
    Serial.print((rx.switches & (1<<SW_ARM)) ? F("A") : F("-"));
    Serial.print((rx.switches & (1<<SW_CALIB)) ? F("C") : F("-"));
    Serial.print((rx.switches & (1<<SW_MOTOR)) ? F("M") : F("-"));
    Serial.print((rx.switches & (1<<SW_BEEPER)) ? F("B") : F("-"));
    Serial.print(headlessMode ? F("L") : F("-"));
    
    // Throttle
    Serial.print(F(" T:")); Serial.print((int)throttleCmd);
    
    // Attitude
    Serial.print(F(" R:")); Serial.print(roll, 1);
    Serial.print(F(" P:")); Serial.print(pitch, 1);
    
    // Gyro rates
    Serial.print(F(" GR:")); Serial.print((int)gyroRateX);
    Serial.print(F(",")); Serial.print((int)gyroRateY);
    
    // PID output
    Serial.print(F(" PID:")); 
    Serial.print((int)rollPID); Serial.print(F(","));
    Serial.print((int)pitchPID);
    
    // Motors
    Serial.print(F(" M:"));
    Serial.print((int)motorFL); Serial.print(F(","));
    Serial.print((int)motorFR); Serial.print(F(","));
    Serial.print((int)motorRL); Serial.print(F(","));
    Serial.print((int)motorRR);
    
    if (motorTestActive) {
        Serial.print(F(" [TEST:")); Serial.print(motorTestPhase); Serial.print(F("]"));
    }
    
    Serial.println();
}

// ============================================================================
//                              SETUP
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);
    
    Serial.println(F("\n================================="));
    Serial.println(F("  QuadFC v6.0 PROFESSIONAL"));
    Serial.println(F("  Complementary Filter Edition"));
    Serial.println(F("  F330 / RS2205 2300KV / 3S"));
    Serial.println(F("=================================\n"));
    
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    beepWait(100, 1500);
    delay(50);
    beepWait(100, 2000);
    delay(50);
    beepWait(200, 2500);
    
    // ===== I2C + MPU6050 =====
    Serial.println(F("Init I2C..."));
    Wire.begin();
    Wire.setClock(400000);
    
    Serial.println(F("Init MPU6050..."));
    mpuInit();
    
    // Test connection
    Wire.beginTransmission(MPU_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println(F("  MPU6050 FAILED!"));
        while (1) { beepWait(200, 500); delay(300); }
    }
    Serial.println(F("  OK"));
    
    // ===== Load calibration =====
    loadCalibration();
    
    // ===== ESCs =====
    Serial.println(F("Init ESCs..."));
    escFL.attach(PIN_MOTOR_FL, ESC_MIN, ESC_MAX);
    escFR.attach(PIN_MOTOR_FR, ESC_MIN, ESC_MAX);
    escRL.attach(PIN_MOTOR_RL, ESC_MIN, ESC_MAX);
    escRR.attach(PIN_MOTOR_RR, ESC_MIN, ESC_MAX);
    
    escFL.writeMicroseconds(ESC_MIN);
    escFR.writeMicroseconds(ESC_MIN);
    escRL.writeMicroseconds(ESC_MIN);
    escRR.writeMicroseconds(ESC_MIN);
    Serial.println(F("  FL=D3 FR=D5 RL=D6 RR=D9"));
    
    // ===== NRF24L01 =====
    Serial.println(F("Init NRF24L01..."));
    if (!radio.begin()) {
        Serial.println(F("  FAILED!"));
        while (1) { beepWait(200, 700); delay(300); }
    }
    radio.flush_rx();
    radio.flush_tx();
    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(16);
    radio.setAutoAck(false);
    radio.disableDynamicPayloads();
    radio.setCRCLength(RF24_CRC_16);
    radio.openReadingPipe(1, radioAddr);
    radio.startListening();
    Serial.print(F("  OK - CH:")); Serial.println(RF_CHANNEL);
    
    // ===== ESC Calibration check =====
    Serial.println(F("Hold D4 for ESC calibration (2s)..."));
    bool escCalMode = false;
    uint32_t start = millis();
    while (millis() - start < 2000) {
        if (radio.available()) {
            RxPacket p;
            radio.read(&p, sizeof(p));
            if (p.valid() && (p.switches & (1 << SW_CALIB))) {
                escCalMode = true;
                break;
            }
        }
        delay(50);
    }
    if (escCalMode) escCalibration();
    
    // ===== Initial gyro calibration if needed =====
    if (cal.gyroOffsetX == 0 && cal.gyroOffsetY == 0 && cal.gyroOffsetZ == 0) {
        Serial.println(F("First run - calibrating gyro..."));
        mpuCalibrate();
    }
    
    // ===== Ready =====
    Serial.println(F("\n================================="));
    Serial.println(F("           READY"));
    Serial.println(F("================================="));
    Serial.println(F("CASCADED PID:"));
    Serial.print(F("  Angle P: ")); Serial.println(ANGLE_KP_ROLL);
    Serial.print(F("  Rate P:")); Serial.print(RATE_KP_ROLL);
    Serial.print(F(" I:")); Serial.print(RATE_KI_ROLL, 4);
    Serial.print(F(" D:")); Serial.println(RATE_KD_ROLL, 4);
    Serial.println(F("CONTROLS:"));
    Serial.println(F("  D2=Arm D4=Short:MotorTest,Long:LevelCal"));
    Serial.println(F("  D5=MotorTest D6=Beeper D7=Headless"));
    Serial.println(F("  A6=Rate(50-150%) A7=Expo"));
    Serial.println(F("=================================\n"));
    
    beepWait(100, 2000);
    delay(100);
    beepWait(100, 2500);
    delay(100);
    beepWait(200, 3000);
    
    loopTime = micros();
    debugTime = ledTime = millis();
}

// ============================================================================
//                             MAIN LOOP
// ============================================================================
void loop() {
    uint32_t now = micros();
    
    // Fixed 250Hz loop
    if (now - loopTime >= LOOP_TIME_US) {
        float dt = (now - loopTime) / 1000000.0f;
        loopTime = now;
        
        // Read sensors
        mpuRead();
        
        // Update attitude
        updateAttitude(dt);
        
        // Radio
        updateRadio();
        
        // Commands
        processCommands();
        
        // PID
        updatePID(dt);
        
        // Motors
        updateMotors();
    }
    
    // LED
    updateLED();
    
    // Debug at 5Hz
    if (millis() - debugTime >= 200) {
        debugTime = millis();
        printDebug();
    }
}
