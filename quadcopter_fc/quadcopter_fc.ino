/**
 * ============================================================================
 *                    QUADCOPTER FLIGHT CONTROLLER v6.1
 *                      PROFESSIONAL STABILITY EDITION
 * ============================================================================
 * 
 * Target: F330 / RS2205 2300KV / 3S 1500mAh
 * 
 * FIXES in v6.1:
 *   ✓ Fixed MPU6050 initialization
 *   ✓ Added raw sensor debugging
 *   ✓ Better I2C error handling
 *   ✓ Verified complementary filter
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
#define PIN_MOTOR_FL        3
#define PIN_MOTOR_FR        5
#define PIN_MOTOR_RL        6
#define PIN_MOTOR_RR        9
#define PIN_RF_CE           4
#define PIN_RF_CSN          10
#define PIN_LED             7
#define PIN_BUZZER          8

// ============================================================================
//                          FLIGHT PARAMETERS
// ============================================================================
#define MAX_ANGLE           45.0f
#define MAX_RATE            300.0f
#define MAX_YAW_RATE        200.0f

#define ESC_MIN             1000
#define ESC_MAX             2000
#define ESC_IDLE            1080
#define ESC_ARM_THR         50

#define RF_TIMEOUT_MS       500
#define LOOP_TIME_US        4000

// ============================================================================
//                         SENSOR CONFIGURATION
// ============================================================================
#define GYRO_SCALE          65.5f       // 500 deg/s
#define ACCEL_SCALE         4096.0f     // 8g

#define COMP_FILTER_ALPHA   0.996f
#define GYRO_LPF_ALPHA      0.7f

// ============================================================================
//                         CALIBRATION (EEPROM)
// ============================================================================
#define EEPROM_MAGIC        0xFC
#define EEPROM_ADDR         0

struct CalibrationData {
    uint8_t magic;
    int16_t gyroOffX, gyroOffY, gyroOffZ;
    int16_t accelOffX, accelOffY, accelOffZ;
    float levelRoll, levelPitch;
};

CalibrationData cal;

// ============================================================================
//                              PID TUNING
// ============================================================================
#define ANGLE_KP            5.0f

#define RATE_KP             0.8f
#define RATE_KI             0.003f
#define RATE_KD             0.015f

#define RATE_KP_YAW         2.0f
#define RATE_KI_YAW         0.01f

#define RATE_I_MAX          100.0f
#define PID_OUTPUT_MAX      400.0f
#define DTERM_LPF           0.5f
#define MOTOR_LPF           0.6f
#define SETPOINT_LPF        0.4f

// ============================================================================
//                          RF PACKET STRUCTURE
// ============================================================================
struct __attribute__((packed)) RxPacket {
    uint16_t throttle;
    int16_t yaw, pitch, roll;
    uint8_t switches;
    uint8_t checksum;
    uint32_t seq;
    uint8_t ratePot, expoPot;
    
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
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
int16_t temperature;

float gyroRateX = 0, gyroRateY = 0, gyroRateZ = 0;
float gyroRateX_f = 0, gyroRateY_f = 0, gyroRateZ_f = 0;

float roll = 0, pitch = 0, yaw = 0;
bool mpuOK = false;
uint32_t mpuReads = 0;
uint32_t mpuFails = 0;

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

float throttleCmd = 0;
float rollCmd = 0, pitchCmd = 0, yawCmd = 0;
float rollRateSP = 0, pitchRateSP = 0, yawRateSP = 0;

float rollI = 0, pitchI = 0, yawI = 0;
float rollD_prev = 0, pitchD_prev = 0;
float rollPID = 0, pitchPID = 0, yawPID = 0;

float motorFL = ESC_MIN, motorFR = ESC_MIN;
float motorRL = ESC_MIN, motorRR = ESC_MIN;

float rateMultiplier = 1.0f;

bool prevArm = false, prevCalib = false, prevMotor = false;
bool prevBeeper = false, prevHeadless = false;
bool headlessMode = false;
float headlessRef = 0;

bool motorTest = false;
uint32_t motorTestStart = 0;
uint8_t motorTestPhase = 0;

bool levelCal = false;
uint32_t calibHoldTime = 0;

uint32_t loopTime = 0;
uint32_t debugTime = 0;
uint32_t ledTime = 0;
bool ledState = false;

// ============================================================================
//                            BUZZER
// ============================================================================
void beep(uint16_t ms, uint16_t f = 2000) { tone(PIN_BUZZER, f, ms); }
void beepWait(uint16_t ms, uint16_t f = 2000) { tone(PIN_BUZZER, f); delay(ms); noTone(PIN_BUZZER); }

// ============================================================================
//                         EEPROM FUNCTIONS
// ============================================================================
void loadCal() {
    EEPROM.get(EEPROM_ADDR, cal);
    if (cal.magic != EEPROM_MAGIC) {
        cal.magic = EEPROM_MAGIC;
        cal.gyroOffX = cal.gyroOffY = cal.gyroOffZ = 0;
        cal.accelOffX = cal.accelOffY = cal.accelOffZ = 0;
        cal.levelRoll = cal.levelPitch = 0;
        EEPROM.put(EEPROM_ADDR, cal);
    }
    Serial.print(F("Cal: G=")); 
    Serial.print(cal.gyroOffX); Serial.print(F(",")); 
    Serial.print(cal.gyroOffY); Serial.print(F(","));
    Serial.print(cal.gyroOffZ);
    Serial.print(F(" L=R")); Serial.print(cal.levelRoll, 1);
    Serial.print(F(" P")); Serial.println(cal.levelPitch, 1);
}

void saveCal() {
    EEPROM.put(EEPROM_ADDR, cal);
    Serial.println(F("Cal saved!"));
}

// ============================================================================
//                         MPU6050 FUNCTIONS
// ============================================================================
void mpuWriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(0x68);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t mpuReadReg(uint8_t reg) {
    Wire.beginTransmission(0x68);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x68, (uint8_t)1);
    return Wire.read();
}

bool mpuInit() {
    Serial.println(F("Init MPU6050..."));
    
    // Check WHO_AM_I
    uint8_t whoami = mpuReadReg(0x75);
    Serial.print(F("  WHO_AM_I: 0x")); Serial.println(whoami, HEX);
    if (whoami != 0x68 && whoami != 0x98) {
        Serial.println(F("  ERROR: Wrong WHO_AM_I!"));
        return false;
    }
    
    // Reset device
    mpuWriteReg(0x6B, 0x80);  // PWR_MGMT_1 = RESET
    delay(100);
    
    // Wake up, use PLL with X gyro
    mpuWriteReg(0x6B, 0x01);  // PWR_MGMT_1 = CLKSEL=1
    delay(50);
    
    // Sample rate divider: 1kHz / (1+3) = 250Hz
    mpuWriteReg(0x19, 0x03);
    
    // DLPF config: 44Hz bandwidth
    mpuWriteReg(0x1A, 0x03);
    
    // Gyro config: ±500°/s (FS_SEL=1)
    mpuWriteReg(0x1B, 0x08);
    
    // Accel config: ±8g (AFS_SEL=2)
    mpuWriteReg(0x1C, 0x10);
    
    // Disable I2C master mode
    mpuWriteReg(0x6A, 0x00);
    
    // Disable FIFO
    mpuWriteReg(0x23, 0x00);
    
    delay(50);
    
    // Test read
    if (mpuRead()) {
        Serial.println(F("  MPU6050 OK!"));
        Serial.print(F("  Raw: A=")); 
        Serial.print(accelX); Serial.print(F(",")); 
        Serial.print(accelY); Serial.print(F(","));
        Serial.print(accelZ);
        Serial.print(F(" G=")); 
        Serial.print(gyroX); Serial.print(F(",")); 
        Serial.print(gyroY); Serial.print(F(","));
        Serial.println(gyroZ);
        return true;
    } else {
        Serial.println(F("  ERROR: Cannot read data!"));
        return false;
    }
}

bool mpuRead() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) {
        mpuFails++;
        return false;
    }
    
    uint8_t count = Wire.requestFrom((uint8_t)0x68, (uint8_t)14, (uint8_t)true);
    if (count != 14) {
        mpuFails++;
        return false;
    }
    
    accelX = (Wire.read() << 8) | Wire.read();
    accelY = (Wire.read() << 8) | Wire.read();
    accelZ = (Wire.read() << 8) | Wire.read();
    temperature = (Wire.read() << 8) | Wire.read();
    gyroX = (Wire.read() << 8) | Wire.read();
    gyroY = (Wire.read() << 8) | Wire.read();
    gyroZ = (Wire.read() << 8) | Wire.read();
    
    mpuReads++;
    return true;
}

void mpuCalibrate() {
    Serial.println(F("\n*** GYRO CALIBRATION ***"));
    Serial.println(F("Keep drone PERFECTLY STILL!"));
    beepWait(500, 1500);
    delay(1000);
    
    int32_t sumGX = 0, sumGY = 0, sumGZ = 0;
    int32_t sumAX = 0, sumAY = 0, sumAZ = 0;
    int n = 0;
    
    for (int i = 0; i < 500; i++) {
        if (mpuRead()) {
            sumGX += gyroX;
            sumGY += gyroY;
            sumGZ += gyroZ;
            sumAX += accelX;
            sumAY += accelY;
            sumAZ += accelZ;
            n++;
        }
        delay(4);
    }
    
    if (n > 400) {
        cal.gyroOffX = sumGX / n;
        cal.gyroOffY = sumGY / n;
        cal.gyroOffZ = sumGZ / n;
        cal.accelOffX = sumAX / n;
        cal.accelOffY = sumAY / n;
        cal.accelOffZ = (sumAZ / n) - 4096;  // Remove 1g
        
        saveCal();
        
        Serial.print(F("Gyro offsets: "));
        Serial.print(cal.gyroOffX); Serial.print(F(", "));
        Serial.print(cal.gyroOffY); Serial.print(F(", "));
        Serial.println(cal.gyroOffZ);
        
        beepWait(200, 2500); delay(100);
        beepWait(200, 3000);
    } else {
        Serial.println(F("FAILED - not enough samples!"));
        beepWait(500, 500);
    }
}

// ============================================================================
//                    COMPLEMENTARY FILTER
// ============================================================================
void updateAttitude(float dt) {
    // Apply calibration
    float gx = (float)(gyroX - cal.gyroOffX) / GYRO_SCALE;
    float gy = (float)(gyroY - cal.gyroOffY) / GYRO_SCALE;
    float gz = (float)(gyroZ - cal.gyroOffZ) / GYRO_SCALE;
    
    float ax = (float)(accelX - cal.accelOffX) / ACCEL_SCALE;
    float ay = (float)(accelY - cal.accelOffY) / ACCEL_SCALE;
    float az = (float)(accelZ - cal.accelOffZ) / ACCEL_SCALE;
    
    // Low-pass filter gyro
    gyroRateX_f = gyroRateX_f * (1.0f - GYRO_LPF_ALPHA) + gx * GYRO_LPF_ALPHA;
    gyroRateY_f = gyroRateY_f * (1.0f - GYRO_LPF_ALPHA) + gy * GYRO_LPF_ALPHA;
    gyroRateZ_f = gyroRateZ_f * (1.0f - GYRO_LPF_ALPHA) + gz * GYRO_LPF_ALPHA;
    
    // Use filtered values
    gyroRateX = gyroRateX_f;
    gyroRateY = gyroRateY_f;
    gyroRateZ = gyroRateZ_f;
    
    // Accelerometer angles
    float accelRoll = atan2(ay, az) * 57.2958f;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958f;
    
    // Complementary filter
    roll = COMP_FILTER_ALPHA * (roll + gyroRateX * dt) + (1.0f - COMP_FILTER_ALPHA) * accelRoll;
    pitch = COMP_FILTER_ALPHA * (pitch + gyroRateY * dt) + (1.0f - COMP_FILTER_ALPHA) * accelPitch;
    yaw += gyroRateZ * dt;
    
    // Apply level calibration
    roll -= cal.levelRoll;
    pitch -= cal.levelPitch;
    
    // Normalize yaw
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
}

// ============================================================================
//                         LEVEL CALIBRATION
// ============================================================================
void doLevelCal() {
    Serial.println(F("\n*** LEVEL CALIBRATION ***"));
    Serial.println(F("Place on FLAT surface!"));
    beepWait(1000, 1500);
    delay(1000);
    
    // Temporarily clear level offsets
    float oldR = cal.levelRoll;
    float oldP = cal.levelPitch;
    cal.levelRoll = 0;
    cal.levelPitch = 0;
    
    float sumR = 0, sumP = 0;
    int n = 0;
    uint32_t start = millis();
    
    while (millis() - start < 2000) {
        if (mpuRead()) {
            updateAttitude(0.004f);
            sumR += roll;
            sumP += pitch;
            n++;
        }
        delay(4);
    }
    
    if (n > 200) {
        cal.levelRoll = sumR / n;
        cal.levelPitch = sumP / n;
        saveCal();
        
        Serial.print(F("Level: R=")); Serial.print(cal.levelRoll, 2);
        Serial.print(F(" P=")); Serial.println(cal.levelPitch, 2);
        
        beepWait(200, 2000); delay(100);
        beepWait(200, 2500); delay(100);
        beepWait(400, 3000);
    } else {
        cal.levelRoll = oldR;
        cal.levelPitch = oldP;
        Serial.println(F("FAILED!"));
        beepWait(500, 500);
    }
    
    levelCal = false;
}

// ============================================================================
//                        ESC CALIBRATION
// ============================================================================
void escCal() {
    Serial.println(F("\n*** ESC CALIBRATION ***"));
    
    escFL.writeMicroseconds(ESC_MAX);
    escFR.writeMicroseconds(ESC_MAX);
    escRL.writeMicroseconds(ESC_MAX);
    escRR.writeMicroseconds(ESC_MAX);
    
    beepWait(1000, 3000);
    Serial.println(F("ESCs at MAX - release button after beeps"));
    
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
    
    Serial.println(F("ESC Cal DONE!"));
    beepWait(200, 2000); beepWait(200, 2500); beepWait(400, 3000);
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
    else if (t < 3200) { motorFL = motorFR = motorRL = motorRR = speed; motorTestPhase = 5; }
    else {
        motorTest = false;
        motorTestPhase = 0;
        motorFL = motorFR = motorRL = motorRR = ESC_MIN;
        beep(200, 2500);
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
        throttleCmd = rollCmd = pitchCmd = yawCmd = 0;
        return;
    }
    
    // Rate pot (50% to 150%)
    rateMultiplier = 0.5f + (rx.ratePot / 255.0f);
    
    // Raw commands
    float rawThr = rx.throttle;
    float rawRoll = (rx.roll / 500.0f) * MAX_ANGLE * rateMultiplier;
    float rawPitch = (rx.pitch / 500.0f) * MAX_ANGLE * rateMultiplier;
    float rawYaw = (rx.yaw / 500.0f) * MAX_YAW_RATE * rateMultiplier;
    
    // Smooth setpoints
    throttleCmd = throttleCmd * (1.0f - SETPOINT_LPF) + rawThr * SETPOINT_LPF;
    rollCmd = rollCmd * (1.0f - SETPOINT_LPF) + rawRoll * SETPOINT_LPF;
    pitchCmd = pitchCmd * (1.0f - SETPOINT_LPF) + rawPitch * SETPOINT_LPF;
    yawCmd = yawCmd * (1.0f - SETPOINT_LPF) + rawYaw * SETPOINT_LPF;
    
    // Switches
    bool armSw = rx.switches & (1 << SW_ARM);
    bool calibBtn = rx.switches & (1 << SW_CALIB);
    bool motorBtn = rx.switches & (1 << SW_MOTOR);
    bool beeperBtn = rx.switches & (1 << SW_BEEPER);
    bool headlessBtn = rx.switches & (1 << SW_HEADLESS);
    
    // Beeper
    if (beeperBtn && !prevBeeper) beep(500, 2500);
    else if (beeperBtn) {
        static uint32_t lb = 0;
        if (millis() - lb > 600) { beep(500, 2500); lb = millis(); }
    }
    prevBeeper = beeperBtn;
    
    // Headless
    if (headlessBtn && !prevHeadless) {
        headlessMode = !headlessMode;
        if (headlessMode) headlessRef = yaw;
        beep(100, headlessMode ? 2500 : 1500);
    }
    prevHeadless = headlessBtn;
    
    if (headlessMode) {
        float d = (yaw - headlessRef) * 0.01745329f;
        float c = cos(d), s = sin(d);
        float r = rollCmd, p = pitchCmd;
        rollCmd = r * c + p * s;
        pitchCmd = -r * s + p * c;
    }
    
    // Calibration button (long=level, short=motor test)
    if (calibBtn && !prevCalib) calibHoldTime = millis();
    if (calibBtn && millis() - calibHoldTime > 3000 && !levelCal && flightState == DISARMED) {
        levelCal = true;
        beep(1000, 1500);
    }
    if (!calibBtn && prevCalib) {
        if (levelCal) doLevelCal();
        else if (flightState == DISARMED && !motorTest) {
            motorTest = true;
            motorTestStart = millis();
            beep(100, 2000);
        }
    }
    prevCalib = calibBtn;
    
    // Motor test button
    if (motorBtn && !prevMotor && flightState == DISARMED && !motorTest) {
        motorTest = true;
        motorTestStart = millis();
        beep(100, 2000);
    }
    prevMotor = motorBtn;
    
    // Arm/Disarm
    if (armSw && !prevArm && flightState == DISARMED) {
        if (throttleCmd <= ESC_ARM_THR && rfConnected && mpuOK) {
            flightState = ARMED;
            rollI = pitchI = yawI = 0;
            rollD_prev = pitchD_prev = 0;
            rollPID = pitchPID = yawPID = 0;
            headlessRef = yaw;
            motorFL = motorFR = motorRL = motorRR = ESC_MIN;
            Serial.println(F("*** ARMED ***"));
            beepWait(100, 2000); delay(100); beepWait(200, 2500);
        } else {
            beep(300, 500);
        }
    }
    if (!armSw && flightState == ARMED) {
        flightState = DISARMED;
        Serial.println(F("*** DISARMED ***"));
        beepWait(300, 1500);
    }
    
    if (flightState == FAILSAFE && (!armSw || (rfConnected && throttleCmd < 100))) {
        flightState = DISARMED;
        beep(200, 1500);
    }
    
    prevArm = armSw;
}

// ============================================================================
//                           CASCADED PID
// ============================================================================
void updatePID(float dt) {
    if (flightState != ARMED) {
        rollPID = pitchPID = yawPID = 0;
        rollI = pitchI = yawI = 0;
        return;
    }
    
    // OUTER: Angle → Rate
    float rollErr = rollCmd - roll;
    float pitchErr = pitchCmd - pitch;
    
    rollRateSP = constrain(ANGLE_KP * rollErr, -MAX_RATE, MAX_RATE);
    pitchRateSP = constrain(ANGLE_KP * pitchErr, -MAX_RATE, MAX_RATE);
    yawRateSP = constrain(yawCmd, -MAX_YAW_RATE, MAX_YAW_RATE);
    
    // INNER: Rate PID
    // Roll
    float rErr = rollRateSP - gyroRateX;
    rollI = constrain(rollI + rErr * dt, -RATE_I_MAX, RATE_I_MAX);
    float rD = (rErr - rollD_prev) / dt;
    rD = rollD_prev * (1.0f - DTERM_LPF) + rD * DTERM_LPF;
    rollD_prev = rErr;
    rollPID = constrain(RATE_KP * rErr + RATE_KI * rollI + RATE_KD * rD, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
    
    // Pitch
    float pErr = pitchRateSP - gyroRateY;
    pitchI = constrain(pitchI + pErr * dt, -RATE_I_MAX, RATE_I_MAX);
    float pD = (pErr - pitchD_prev) / dt;
    pD = pitchD_prev * (1.0f - DTERM_LPF) + pD * DTERM_LPF;
    pitchD_prev = pErr;
    pitchPID = constrain(RATE_KP * pErr + RATE_KI * pitchI + RATE_KD * pD, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
    
    // Yaw
    float yErr = yawRateSP - gyroRateZ;
    yawI = constrain(yawI + yErr * dt, -RATE_I_MAX, RATE_I_MAX);
    yawPID = constrain(RATE_KP_YAW * yErr + RATE_KI_YAW * yawI, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
}

// ============================================================================
//                           MOTOR MIXING
// ============================================================================
void updateMotors() {
    if (motorTest) { runMotorTest(); return; }
    
    if (flightState != ARMED) {
        motorFL = motorFR = motorRL = motorRR = ESC_MIN;
        escFL.writeMicroseconds(ESC_MIN);
        escFR.writeMicroseconds(ESC_MIN);
        escRL.writeMicroseconds(ESC_MIN);
        escRR.writeMicroseconds(ESC_MIN);
        return;
    }
    
    float thr = ESC_IDLE + (throttleCmd / 1000.0f) * (ESC_MAX - ESC_IDLE);
    
    float tFL = thr - rollPID + pitchPID - yawPID;
    float tFR = thr + rollPID + pitchPID + yawPID;
    float tRL = thr - rollPID - pitchPID + yawPID;
    float tRR = thr + rollPID - pitchPID - yawPID;
    
    tFL = constrain(tFL, ESC_IDLE, ESC_MAX);
    tFR = constrain(tFR, ESC_IDLE, ESC_MAX);
    tRL = constrain(tRL, ESC_IDLE, ESC_MAX);
    tRR = constrain(tRR, ESC_IDLE, ESC_MAX);
    
    motorFL = motorFL * (1.0f - MOTOR_LPF) + tFL * MOTOR_LPF;
    motorFR = motorFR * (1.0f - MOTOR_LPF) + tFR * MOTOR_LPF;
    motorRL = motorRL * (1.0f - MOTOR_LPF) + tRL * MOTOR_LPF;
    motorRR = motorRR * (1.0f - MOTOR_LPF) + tRR * MOTOR_LPF;
    
    escFL.writeMicroseconds((uint16_t)motorFL);
    escFR.writeMicroseconds((uint16_t)motorFR);
    escRL.writeMicroseconds((uint16_t)motorRL);
    escRR.writeMicroseconds((uint16_t)motorRR);
}

// ============================================================================
//                            LED
// ============================================================================
void updateLED() {
    uint16_t interval = (flightState == ARMED) ? (headlessMode ? 150 : 0) :
                        (flightState == FAILSAFE) ? 80 :
                        (motorTest) ? 100 : 500;
    
    if (interval == 0) digitalWrite(PIN_LED, HIGH);
    else if (millis() - ledTime >= interval) {
        ledTime = millis();
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState);
    }
}

// ============================================================================
//                           DEBUG
// ============================================================================
void printDebug() {
    // State
    Serial.print(flightState == ARMED ? F("ARM ") : (flightState == FAILSAFE ? F("FAIL") : F("DIS ")));
    
    // RF
    Serial.print(F(" RF:")); Serial.print(rfConnected ? rxCount : 0);
    
    // Rate
    Serial.print(F(" R%:")); Serial.print((int)(rateMultiplier * 100));
    
    // Switches
    Serial.print(F(" SW:"));
    Serial.print((rx.switches & (1<<SW_ARM)) ? F("A") : F("-"));
    Serial.print((rx.switches & (1<<SW_CALIB)) ? F("C") : F("-"));
    Serial.print((rx.switches & (1<<SW_MOTOR)) ? F("M") : F("-"));
    Serial.print((rx.switches & (1<<SW_BEEPER)) ? F("B") : F("-"));
    Serial.print(headlessMode ? F("L") : F("-"));
    
    // Throttle
    Serial.print(F(" T:")); Serial.print((int)throttleCmd);
    
    // RAW sensor (to verify MPU is working)
    Serial.print(F(" RAW:")); 
    Serial.print(gyroX); Serial.print(F(",")); 
    Serial.print(gyroY);
    
    // Attitude
    Serial.print(F(" Ang:R")); Serial.print(roll, 1);
    Serial.print(F(" P")); Serial.print(pitch, 1);
    
    // Gyro rates
    Serial.print(F(" Rate:")); Serial.print((int)gyroRateX);
    Serial.print(F(",")); Serial.print((int)gyroRateY);
    
    // PID
    Serial.print(F(" PID:")); 
    Serial.print((int)rollPID); Serial.print(F(","));
    Serial.print((int)pitchPID);
    
    // Motors
    Serial.print(F(" M:"));
    Serial.print((int)motorFL); Serial.print(F(","));
    Serial.print((int)motorFR); Serial.print(F(","));
    Serial.print((int)motorRL); Serial.print(F(","));
    Serial.print((int)motorRR);
    
    // MPU stats
    Serial.print(F(" MPU:"));
    Serial.print(mpuReads); Serial.print(F("/")); Serial.print(mpuFails);
    
    if (motorTest) Serial.print(F(" [TEST]"));
    
    Serial.println();
}

// ============================================================================
//                              SETUP
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);
    
    Serial.println(F("\n=================================="));
    Serial.println(F("   QuadFC v6.1 PROFESSIONAL"));
    Serial.println(F("   Complementary Filter + Debug"));
    Serial.println(F("==================================\n"));
    
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    beepWait(100, 1500); delay(50);
    beepWait(100, 2000); delay(50);
    beepWait(200, 2500);
    
    // I2C
    Serial.println(F("Init I2C..."));
    Wire.begin();
    Wire.setClock(400000);
    
    // MPU6050
    mpuOK = mpuInit();
    if (!mpuOK) {
        Serial.println(F("MPU6050 FAILED - CHECK WIRING!"));
        Serial.println(F("  SDA -> A4"));
        Serial.println(F("  SCL -> A5"));
        Serial.println(F("  VCC -> 5V or 3.3V"));
        Serial.println(F("  GND -> GND"));
        while (1) { beepWait(200, 500); delay(300); }
    }
    
    // Load calibration
    loadCal();
    
    // ESCs
    Serial.println(F("Init ESCs..."));
    escFL.attach(PIN_MOTOR_FL, ESC_MIN, ESC_MAX);
    escFR.attach(PIN_MOTOR_FR, ESC_MIN, ESC_MAX);
    escRL.attach(PIN_MOTOR_RL, ESC_MIN, ESC_MAX);
    escRR.attach(PIN_MOTOR_RR, ESC_MIN, ESC_MAX);
    escFL.writeMicroseconds(ESC_MIN);
    escFR.writeMicroseconds(ESC_MIN);
    escRL.writeMicroseconds(ESC_MIN);
    escRR.writeMicroseconds(ESC_MIN);
    
    // NRF24L01
    Serial.println(F("Init NRF24L01..."));
    if (!radio.begin()) {
        Serial.println(F("NRF24 FAILED!"));
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
    
    // ESC calibration check
    Serial.println(F("Hold D4 for ESC cal (2s)..."));
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
    if (escCalMode) escCal();
    
    // Auto gyro calibration if needed
    if (cal.gyroOffX == 0 && cal.gyroOffY == 0 && cal.gyroOffZ == 0) {
        Serial.println(F("First run - calibrating gyro..."));
        mpuCalibrate();
    }
    
    Serial.println(F("\n=================================="));
    Serial.println(F("          READY"));
    Serial.println(F("=================================="));
    Serial.println(F("D4 short=MotorTest, long=LevelCal"));
    Serial.println(F("D5=MotorTest D6=Beeper D7=Headless"));
    Serial.println(F("==================================\n"));
    
    beepWait(100, 2000); delay(100);
    beepWait(100, 2500); delay(100);
    beepWait(200, 3000);
    
    loopTime = micros();
    debugTime = ledTime = millis();
}

// ============================================================================
//                             MAIN LOOP
// ============================================================================
void loop() {
    uint32_t now = micros();
    
    if (now - loopTime >= LOOP_TIME_US) {
        float dt = (now - loopTime) / 1000000.0f;
        loopTime = now;
        
        // Read MPU
        if (mpuRead()) {
            updateAttitude(dt);
        }
        
        updateRadio();
        processCommands();
        updatePID(dt);
        updateMotors();
    }
    
    updateLED();
    
    if (millis() - debugTime >= 200) {
        debugTime = millis();
        printDebug();
    }
}
