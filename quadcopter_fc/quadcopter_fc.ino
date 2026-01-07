/**
 * ============================================================================
 *                    QUADCOPTER FLIGHT CONTROLLER v5.0
 *                         ROCK SOLID EDITION
 * ============================================================================
 * 
 * Target: F330 / RS2205 2300KV / 3S 1500mAh
 * 
 * FEATURES:
 *   - Comprehensive motor testing
 *   - Level calibration mode
 *   - All motors same speed when level
 *   - Extensive debugging
 *   - Verified button detection
 * 
 * BUTTONS:
 *   D4 (CALIB)  = Short press: Motor test / Long press (3s): Level calibration
 *   D5 (MOTOR)  = Motor test
 *   D6 (BEEPER) = Find drone beeper
 *   D7 (HEADLESS) = Toggle headless mode
 * 
 * ============================================================================
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
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
#define MAX_ANGLE           45
#define MAX_YAW_RATE        200

#define ESC_MIN             1000
#define ESC_MAX             2000
#define ESC_IDLE            1100
#define ESC_ARM_THR         50

#define RF_TIMEOUT          500

// ============================================================================
//                         MPU6050 CALIBRATION
// ============================================================================
#define ACCEL_X_OFF         -2366
#define ACCEL_Y_OFF         755
#define ACCEL_Z_OFF         -2006
#define GYRO_X_OFF          10
#define GYRO_Y_OFF          21
#define GYRO_Z_OFF          -19

// ============================================================================
//                         PID - CONSERVATIVE START
// ============================================================================
// These are intentionally LOW - increase after successful hover
float Kp_roll = 1.8f;
float Ki_roll = 0.01f;
float Kd_roll = 0.8f;

float Kp_pitch = 1.8f;
float Ki_pitch = 0.01f;
float Kd_pitch = 0.8f;

float Kp_yaw = 2.5f;
float Ki_yaw = 0.02f;
float Kd_yaw = 0.0f;

#define PID_MAX             300
#define INTEGRAL_MAX        80

// ============================================================================
//                         LEVEL TRIM (saved to EEPROM)
// ============================================================================
#define EEPROM_MAGIC        0xAB
#define EEPROM_ADDR         0

struct LevelTrim {
    uint8_t magic;
    float rollOffset;
    float pitchOffset;
};

LevelTrim levelTrim = {EEPROM_MAGIC, 0.0f, 0.0f};

// ============================================================================
//                          RF PACKET
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
//                           GLOBALS
// ============================================================================
MPU6050 mpu;
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
Servo esc[4];
const uint8_t escPin[4] = {PIN_MOTOR_FL, PIN_MOTOR_FR, PIN_MOTOR_RL, PIN_MOTOR_RR};
const uint8_t addr[6] = "QUAD1";

// DMP
bool dmpOK = false;
uint8_t fifo[64];
Quaternion quat;
VectorFloat grav;
float ypr[3];

// Attitude
float roll = 0, pitch = 0, yaw = 0;
float rollRate = 0, pitchRate = 0, yawRate = 0;
float prevRoll = 0, prevPitch = 0, prevYaw = 0;
uint32_t attTime = 0;

// Radio
RxPacket rx;
uint32_t lastRx = 0;
uint32_t rxCount = 0;
bool rfOK = false;

// Flight
enum State { DISARMED, ARMED, FAILSAFE } state = DISARMED;

int16_t thrCmd = 0;
float rollCmd = 0, pitchCmd = 0, yawCmd = 0;

// PID
float rollI = 0, pitchI = 0, yawI = 0;
float rollD_prev = 0, pitchD_prev = 0;
float pidRoll = 0, pidPitch = 0, pidYaw = 0;

// Motors
uint16_t motor[4] = {ESC_MIN, ESC_MIN, ESC_MIN, ESC_MIN};

// State
bool prevArm = false;
bool prevCalib = false;
bool prevMotor = false;
bool prevBeeper = false;
bool prevHeadless = false;
bool headless = false;
float headlessRef = 0;

// Calibration
bool levelCalMode = false;
uint32_t calibBtnTime = 0;

// Motor test
bool motorTest = false;
uint32_t motorTestStart = 0;
uint8_t motorTestPhase = 0;

// Timing
uint32_t pidTime = 0;
uint32_t debugTime = 0;
uint32_t ledTime = 0;

// ============================================================================
//                            BUZZER
// ============================================================================
void beep(uint16_t ms, uint16_t f = 2000) { tone(PIN_BUZZER, f, ms); }
void beepWait(uint16_t ms, uint16_t f = 2000) { tone(PIN_BUZZER, f); delay(ms); noTone(PIN_BUZZER); }

// ============================================================================
//                          EEPROM FUNCTIONS
// ============================================================================
void loadTrim() {
    EEPROM.get(EEPROM_ADDR, levelTrim);
    if (levelTrim.magic != EEPROM_MAGIC) {
        levelTrim.magic = EEPROM_MAGIC;
        levelTrim.rollOffset = 0;
        levelTrim.pitchOffset = 0;
    }
    Serial.print(F("Trim: R=")); Serial.print(levelTrim.rollOffset, 2);
    Serial.print(F(" P=")); Serial.println(levelTrim.pitchOffset, 2);
}

void saveTrim() {
    EEPROM.put(EEPROM_ADDR, levelTrim);
    Serial.println(F("Trim SAVED"));
}

// ============================================================================
//                        LEVEL CALIBRATION
// ============================================================================
void doLevelCalibration() {
    Serial.println(F("\n*** LEVEL CALIBRATION ***"));
    Serial.println(F("Place drone on FLAT surface!"));
    beepWait(500, 1500);
    delay(1000);
    
    // Read average attitude over 2 seconds
    float sumRoll = 0, sumPitch = 0;
    int count = 0;
    uint32_t start = millis();
    
    while (millis() - start < 2000) {
        if (mpu.dmpGetCurrentFIFOPacket(fifo)) {
            mpu.dmpGetQuaternion(&quat, fifo);
            mpu.dmpGetGravity(&grav, &quat);
            mpu.dmpGetYawPitchRoll(ypr, &quat, &grav);
            sumRoll += ypr[2] * 57.2958f;
            sumPitch += ypr[1] * 57.2958f;
            count++;
        }
        delay(10);
    }
    
    if (count > 50) {
        levelTrim.rollOffset = sumRoll / count;
        levelTrim.pitchOffset = sumPitch / count;
        saveTrim();
        
        Serial.print(F("New offsets: R=")); Serial.print(levelTrim.rollOffset, 2);
        Serial.print(F(" P=")); Serial.println(levelTrim.pitchOffset, 2);
        
        beepWait(200, 2000); delay(100);
        beepWait(200, 2500); delay(100);
        beepWait(400, 3000);
    } else {
        Serial.println(F("FAILED - no IMU data"));
        beepWait(500, 500);
    }
    
    levelCalMode = false;
}

// ============================================================================
//                        ESC CALIBRATION
// ============================================================================
void escCalibration() {
    Serial.println(F("\n*** ESC CALIBRATION ***"));
    
    for (int i = 0; i < 4; i++) esc[i].writeMicroseconds(ESC_MAX);
    beepWait(1000, 3000);
    Serial.println(F("ESCs at MAX - wait for beeps, then release button"));
    
    uint32_t start = millis();
    while (millis() - start < 15000) {
        if (radio.available()) {
            RxPacket p;
            radio.read(&p, sizeof(p));
            if (p.valid() && !(p.switches & (1 << SW_CALIB))) break;
        }
        delay(50);
    }
    
    for (int i = 0; i < 4; i++) esc[i].writeMicroseconds(ESC_MIN);
    beepWait(500, 1500);
    delay(2000);
    
    Serial.println(F("ESC Calibration DONE"));
    beepWait(200, 2000); beepWait(200, 2500); beepWait(400, 3000);
}

// ============================================================================
//                          DMP UPDATE
// ============================================================================
void updateIMU() {
    if (!dmpOK) return;
    
    if (mpu.dmpGetCurrentFIFOPacket(fifo)) {
        mpu.dmpGetQuaternion(&quat, fifo);
        mpu.dmpGetGravity(&grav, &quat);
        mpu.dmpGetYawPitchRoll(ypr, &quat, &grav);
        
        // Apply level trim offset
        float newRoll = ypr[2] * 57.2958f - levelTrim.rollOffset;
        float newPitch = ypr[1] * 57.2958f - levelTrim.pitchOffset;
        float newYaw = ypr[0] * 57.2958f;
        
        // Calculate rates
        uint32_t now = micros();
        float dt = (now - attTime) / 1000000.0f;
        
        if (attTime > 0 && dt > 0.001f && dt < 0.05f) {
            rollRate = (newRoll - prevRoll) / dt;
            pitchRate = (newPitch - prevPitch) / dt;
            
            float dYaw = newYaw - prevYaw;
            if (dYaw > 180) dYaw -= 360;
            if (dYaw < -180) dYaw += 360;
            yawRate = dYaw / dt;
        }
        
        roll = prevRoll = newRoll;
        pitch = prevPitch = newPitch;
        yaw = prevYaw = newYaw;
        attTime = now;
    }
}

// ============================================================================
//                          RADIO UPDATE
// ============================================================================
void updateRadio() {
    while (radio.available()) {
        RxPacket p;
        radio.read(&p, sizeof(p));
        
        if (p.valid()) {
            rx = p;
            lastRx = millis();
            rxCount++;
            
            if (!rfOK) {
                rfOK = true;
                Serial.println(F("\n*** RF CONNECTED ***"));
                for (int i = 0; i < 5; i++) { beepWait(60, 2500); delay(60); }
            }
        }
    }
    
    if (rfOK && millis() - lastRx > RF_TIMEOUT) {
        rfOK = false;
        Serial.println(F("\n*** RF LOST ***"));
        if (state == ARMED) {
            state = FAILSAFE;
            beep(500, 800);
        }
    }
}

// ============================================================================
//                         MOTOR TEST
// ============================================================================
void runMotorTest() {
    uint32_t t = millis() - motorTestStart;
    
    // Stop all first
    for (int i = 0; i < 4; i++) motor[i] = ESC_MIN;
    
    // Test sequence: FL, FR, RL, RR, ALL
    if (t < 600) {
        motor[0] = ESC_IDLE;
        motorTestPhase = 1;
    } else if (t < 1200) {
        motor[1] = ESC_IDLE;
        motorTestPhase = 2;
    } else if (t < 1800) {
        motor[2] = ESC_IDLE;
        motorTestPhase = 3;
    } else if (t < 2400) {
        motor[3] = ESC_IDLE;
        motorTestPhase = 4;
    } else if (t < 3200) {
        // ALL motors at same speed
        for (int i = 0; i < 4; i++) motor[i] = ESC_IDLE;
        motorTestPhase = 5;
    } else {
        motorTest = false;
        motorTestPhase = 0;
        for (int i = 0; i < 4; i++) motor[i] = ESC_MIN;
        beep(200, 2500);
        Serial.println(F("Motor test DONE"));
    }
    
    // Write to ESCs
    for (int i = 0; i < 4; i++) esc[i].writeMicroseconds(motor[i]);
}

// ============================================================================
//                        PROCESS COMMANDS
// ============================================================================
void processCommands() {
    if (!rfOK) {
        thrCmd = 0;
        rollCmd = pitchCmd = yawCmd = 0;
        return;
    }
    
    // Get commands
    thrCmd = rx.throttle;
    
    // Rate multiplier from pot (0.5 to 1.5)
    float rate = 0.5f + rx.ratePot / 255.0f;
    
    rollCmd = (rx.roll / 500.0f) * MAX_ANGLE * rate;
    pitchCmd = (rx.pitch / 500.0f) * MAX_ANGLE * rate;
    yawCmd = (rx.yaw / 500.0f) * MAX_YAW_RATE * rate;
    
    // Button states
    bool armSw = rx.switches & (1 << SW_ARM);
    bool calibBtn = rx.switches & (1 << SW_CALIB);
    bool motorBtn = rx.switches & (1 << SW_MOTOR);
    bool beeperBtn = rx.switches & (1 << SW_BEEPER);
    bool headlessBtn = rx.switches & (1 << SW_HEADLESS);
    
    // ===== BEEPER =====
    if (beeperBtn && !prevBeeper) {
        beep(500, 2500);
        Serial.println(F("BEEP!"));
    } else if (beeperBtn) {
        static uint32_t lastB = 0;
        if (millis() - lastB > 600) { beep(500, 2500); lastB = millis(); }
    }
    prevBeeper = beeperBtn;
    
    // ===== HEADLESS =====
    if (headlessBtn && !prevHeadless) {
        headless = !headless;
        if (headless) headlessRef = yaw;
        Serial.print(F("Headless: ")); Serial.println(headless ? F("ON") : F("OFF"));
        beep(100, headless ? 2500 : 1500);
    }
    prevHeadless = headlessBtn;
    
    // Apply headless transformation
    if (headless) {
        float diff = (yaw - headlessRef) * 0.01745329f;
        float c = cos(diff), s = sin(diff);
        float r = rollCmd, p = pitchCmd;
        rollCmd = r * c + p * s;
        pitchCmd = -r * s + p * c;
    }
    
    // ===== CALIBRATION BUTTON =====
    // Long press (3s) = level calibration
    // Short press = motor test
    if (calibBtn && !prevCalib) {
        calibBtnTime = millis();
    }
    if (calibBtn && millis() - calibBtnTime > 3000 && !levelCalMode && state == DISARMED) {
        levelCalMode = true;
        beep(1000, 1500);
        Serial.println(F("Release for LEVEL CAL"));
    }
    if (!calibBtn && prevCalib) {
        if (levelCalMode) {
            doLevelCalibration();
        } else if (state == DISARMED && !motorTest) {
            // Short press - motor test
            motorTest = true;
            motorTestStart = millis();
            motorTestPhase = 0;
            Serial.println(F("MOTOR TEST"));
            beep(100, 2000);
        }
    }
    prevCalib = calibBtn;
    
    // ===== MOTOR TEST BUTTON (D5) =====
    if (motorBtn && !prevMotor && state == DISARMED && !motorTest) {
        motorTest = true;
        motorTestStart = millis();
        motorTestPhase = 0;
        Serial.println(F("MOTOR TEST"));
        beep(100, 2000);
    }
    prevMotor = motorBtn;
    
    // ===== ARM/DISARM =====
    if (armSw && !prevArm && state == DISARMED) {
        if (thrCmd <= ESC_ARM_THR && rfOK && dmpOK) {
            state = ARMED;
            
            // Reset PID
            rollI = pitchI = yawI = 0;
            rollD_prev = pitchD_prev = 0;
            pidRoll = pidPitch = pidYaw = 0;
            
            headlessRef = yaw;
            for (int i = 0; i < 4; i++) motor[i] = ESC_MIN;
            
            Serial.println(F("*** ARMED ***"));
            beepWait(100, 2000); delay(100); beepWait(200, 2500);
        } else {
            Serial.println(F("Cannot ARM - check throttle/RF/IMU"));
            beep(300, 500);
        }
    }
    if (!armSw && state == ARMED) {
        state = DISARMED;
        Serial.println(F("*** DISARMED ***"));
        beepWait(300, 1500);
    }
    
    // Failsafe recovery
    if (state == FAILSAFE && (!armSw || (rfOK && thrCmd < 100))) {
        state = DISARMED;
        beep(200, 1500);
    }
    
    prevArm = armSw;
}

// ============================================================================
//                              PID
// ============================================================================
void updatePID(float dt) {
    if (state != ARMED) {
        pidRoll = pidPitch = pidYaw = 0;
        rollI = pitchI = yawI = 0;
        return;
    }
    
    // ===== ROLL PID =====
    float errR = rollCmd - roll;
    rollI += errR * dt;
    rollI = constrain(rollI, -INTEGRAL_MAX / Ki_roll, INTEGRAL_MAX / Ki_roll);
    float dR = -rollRate;  // D on measurement
    dR = rollD_prev * 0.7f + dR * 0.3f;  // LPF
    rollD_prev = dR;
    pidRoll = Kp_roll * errR + Ki_roll * rollI + Kd_roll * dR;
    pidRoll = constrain(pidRoll, -PID_MAX, PID_MAX);
    
    // ===== PITCH PID =====
    float errP = pitchCmd - pitch;
    pitchI += errP * dt;
    pitchI = constrain(pitchI, -INTEGRAL_MAX / Ki_pitch, INTEGRAL_MAX / Ki_pitch);
    float dP = -pitchRate;
    dP = pitchD_prev * 0.7f + dP * 0.3f;
    pitchD_prev = dP;
    pidPitch = Kp_pitch * errP + Ki_pitch * pitchI + Kd_pitch * dP;
    pidPitch = constrain(pidPitch, -PID_MAX, PID_MAX);
    
    // ===== YAW PID (rate mode) =====
    float errY = yawCmd - yawRate;
    yawI += errY * dt;
    yawI = constrain(yawI, -INTEGRAL_MAX / Ki_yaw, INTEGRAL_MAX / Ki_yaw);
    pidYaw = Kp_yaw * errY + Ki_yaw * yawI;
    pidYaw = constrain(pidYaw, -PID_MAX, PID_MAX);
}

// ============================================================================
//                          MOTOR MIXING
// ============================================================================
void updateMotors() {
    if (motorTest) {
        runMotorTest();
        return;
    }
    
    if (state != ARMED) {
        for (int i = 0; i < 4; i++) {
            motor[i] = ESC_MIN;
            esc[i].writeMicroseconds(ESC_MIN);
        }
        return;
    }
    
    // Throttle mapping
    float thr = ESC_IDLE + (thrCmd / 1000.0f) * (ESC_MAX - ESC_IDLE);
    
    // Quad-X mixing
    // FL (0): -roll +pitch -yaw (CCW)
    // FR (1): +roll +pitch +yaw (CW)
    // RL (2): -roll -pitch +yaw (CW)
    // RR (3): +roll -pitch -yaw (CCW)
    float m0 = thr - pidRoll + pidPitch - pidYaw;
    float m1 = thr + pidRoll + pidPitch + pidYaw;
    float m2 = thr - pidRoll - pidPitch + pidYaw;
    float m3 = thr + pidRoll - pidPitch - pidYaw;
    
    // Constrain
    motor[0] = constrain((int)m0, ESC_IDLE, ESC_MAX);
    motor[1] = constrain((int)m1, ESC_IDLE, ESC_MAX);
    motor[2] = constrain((int)m2, ESC_IDLE, ESC_MAX);
    motor[3] = constrain((int)m3, ESC_IDLE, ESC_MAX);
    
    // Write
    for (int i = 0; i < 4; i++) esc[i].writeMicroseconds(motor[i]);
}

// ============================================================================
//                            LED
// ============================================================================
void updateLED() {
    static bool s = false;
    uint16_t interval;
    
    if (state == ARMED) interval = headless ? 150 : 0;
    else if (state == FAILSAFE) interval = 80;
    else if (motorTest) interval = 100;
    else interval = 500;
    
    if (interval == 0) {
        digitalWrite(PIN_LED, HIGH);
    } else if (millis() - ledTime >= interval) {
        ledTime = millis();
        s = !s;
        digitalWrite(PIN_LED, s);
    }
}

// ============================================================================
//                           DEBUG
// ============================================================================
void printDebug() {
    // State
    Serial.print(state == ARMED ? F("ARM ") : (state == FAILSAFE ? F("FAIL") : F("DIS ")));
    
    // RF
    Serial.print(F(" RF:")); Serial.print(rfOK ? rxCount : 0);
    
    // Switches (show actual bits received)
    Serial.print(F(" SW:0x")); Serial.print(rx.switches, HEX);
    Serial.print(F(" ["));
    Serial.print((rx.switches & (1<<SW_ARM)) ? F("A") : F("-"));
    Serial.print((rx.switches & (1<<SW_CALIB)) ? F("C") : F("-"));
    Serial.print((rx.switches & (1<<SW_MOTOR)) ? F("M") : F("-"));
    Serial.print((rx.switches & (1<<SW_ALTHOLD)) ? F("H") : F("-"));
    Serial.print((rx.switches & (1<<SW_BEEPER)) ? F("B") : F("-"));
    Serial.print((rx.switches & (1<<SW_HEADLESS)) ? F("L") : F("-"));
    Serial.print(F("]"));
    
    // Throttle
    Serial.print(F(" T:")); Serial.print(thrCmd);
    
    // Angles
    Serial.print(F(" R:")); Serial.print(roll, 1);
    Serial.print(F(" P:")); Serial.print(pitch, 1);
    
    // PID
    Serial.print(F(" PID:")); 
    Serial.print((int)pidRoll); Serial.print(F(","));
    Serial.print((int)pidPitch); Serial.print(F(","));
    Serial.print((int)pidYaw);
    
    // Motors
    Serial.print(F(" M:"));
    for (int i = 0; i < 4; i++) {
        Serial.print(motor[i]);
        if (i < 3) Serial.print(F(","));
    }
    
    // Motor test phase
    if (motorTest) {
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
    
    Serial.println(F("\n=============================="));
    Serial.println(F("   QuadFC v5.0 ROCK SOLID"));
    Serial.println(F("   F330 / RS2205 2300KV / 3S"));
    Serial.println(F("==============================\n"));
    
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    beepWait(100, 1500); delay(50);
    beepWait(100, 2000); delay(50);
    beepWait(200, 2500);
    
    // ===== ESCs =====
    Serial.println(F("Init ESCs..."));
    for (int i = 0; i < 4; i++) {
        esc[i].attach(escPin[i], ESC_MIN, ESC_MAX);
        esc[i].writeMicroseconds(ESC_MIN);
    }
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
    radio.openReadingPipe(1, addr);
    radio.startListening();
    Serial.print(F("  OK - CH:")); Serial.println(RF_CHANNEL);
    
    // ===== ESC CALIBRATION CHECK =====
    Serial.println(F("Waiting 2s for ESC cal button..."));
    bool escCal = false;
    uint32_t start = millis();
    while (millis() - start < 2000) {
        if (radio.available()) {
            RxPacket p;
            radio.read(&p, sizeof(p));
            if (p.valid() && (p.switches & (1 << SW_CALIB))) {
                escCal = true;
                break;
            }
        }
        delay(50);
    }
    if (escCal) escCalibration();
    
    // ===== MPU6050 =====
    Serial.println(F("Init MPU6050..."));
    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println(F("  MPU6050 FAILED!"));
        while (1) { beepWait(200, 500); delay(300); }
    }
    
    uint8_t dmpStat = mpu.dmpInitialize();
    mpu.setXAccelOffset(ACCEL_X_OFF);
    mpu.setYAccelOffset(ACCEL_Y_OFF);
    mpu.setZAccelOffset(ACCEL_Z_OFF);
    mpu.setXGyroOffset(GYRO_X_OFF);
    mpu.setYGyroOffset(GYRO_Y_OFF);
    mpu.setZGyroOffset(GYRO_Z_OFF);
    
    if (dmpStat == 0) {
        Serial.print(F("  Calibrating..."));
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        dmpOK = true;
        Serial.println(F("OK"));
    } else {
        Serial.print(F("  DMP FAILED: ")); Serial.println(dmpStat);
        while (1) { beepWait(200, 600); delay(300); }
    }
    
    // ===== LOAD LEVEL TRIM =====
    Serial.println(F("Loading level trim..."));
    loadTrim();
    
    // ===== READY =====
    Serial.println(F("\n=============================="));
    Serial.println(F("         READY"));
    Serial.println(F("=============================="));
    Serial.println(F("CONTROLS:"));
    Serial.println(F("  D2 = ARM switch"));
    Serial.println(F("  D4 = Short:MotorTest Long:LevelCal"));
    Serial.println(F("  D5 = Motor Test"));
    Serial.println(F("  D6 = Beeper"));
    Serial.println(F("  D7 = Headless"));
    Serial.println(F("  A6 = Rate pot"));
    Serial.println(F(""));
    Serial.println(F("PID (low for safety):"));
    Serial.print(F("  P=")); Serial.print(Kp_roll);
    Serial.print(F(" I=")); Serial.print(Ki_roll);
    Serial.print(F(" D=")); Serial.println(Kd_roll);
    Serial.println(F("==============================\n"));
    
    beepWait(100, 2000); delay(100);
    beepWait(100, 2500); delay(100);
    beepWait(200, 3000);
    
    pidTime = micros();
    debugTime = ledTime = millis();
}

// ============================================================================
//                             MAIN LOOP
// ============================================================================
void loop() {
    uint32_t now = micros();
    
    // Radio - every loop
    updateRadio();
    
    // IMU - every loop
    updateIMU();
    
    // PID + Motors at 250Hz
    if (now - pidTime >= 4000) {
        float dt = (now - pidTime) / 1000000.0f;
        pidTime = now;
        
        processCommands();
        updatePID(dt);
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
