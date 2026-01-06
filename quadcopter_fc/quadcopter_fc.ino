/**
 * QUADCOPTER FC - Using I2Cdev + MPU6050 DMP
 * Optimized for Arduino Nano flash size
 * 
 * Libraries Required:
 *   - I2Cdev (Jeff Rowberg)
 *   - MPU6050 (Jeff Rowberg) 
 *   - RF24 (TMRh20)
 *   - MS5611 (Rob Tillaart)
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SPI.h>
#include <RF24.h>
#include <MS5611.h>
#include <Servo.h>

// ==================== CONFIG ====================
#define RF_CHANNEL      108
#define DEBUG           1       // 0 to save flash
#define BAUD            115200

// ==================== PINS ====================
#define M_FL  3
#define M_FR  5
#define M_RL  6
#define M_RR  9
#define RF_CE 4
#define RF_CS 10
#define LED   7
#define BZR   8

// ==================== PARAMS ====================
#define MAX_ANGLE     45
#define MAX_YAW       180
#define ESC_MIN       1000
#define ESC_MAX       2000
#define ESC_IDLE      1150
#define ARM_THR_MAX   50
#define RF_TIMEOUT    500
#define MOTOR_RATE    100
#define ALT_PID_MAX   150
#define ALT_RATE      50

// ==================== PACKET ====================
struct __attribute__((packed)) Pkt {
    uint16_t thr;
    int16_t yaw, pitch, roll;
    uint8_t sw, chk;
    uint32_t seq;
    uint8_t ch, res;
    bool ok() const {
        const uint8_t* d = (const uint8_t*)this;
        uint8_t c = 0;
        for (uint8_t i = 0; i < 9; i++) c ^= d[i];
        return c == chk;
    }
};

// ==================== OBJECTS ====================
MPU6050 mpu;
MS5611 baro(0x77);
RF24 radio(RF_CE, RF_CS);
Servo sFL, sFR, sRL, sRR;

// ==================== DMP ====================
bool dmpOK = false;
uint8_t devStat;
uint16_t pktSize;
uint8_t fifoBuf[64];
Quaternion q;
VectorFloat grav;
float ypr[3];

// ==================== FLIGHT ====================
float roll = 0, pitch = 0, yaw = 0, yawRate = 0;
float prevYaw = 0;

// Baro
float baseP = 0, alt = 0, altF = 0, altP = 0;
float vv = 0, vvF = 0;
uint32_t altT = 0;
bool baroOK = false;

// Radio
Pkt rx;
uint32_t lastPkt = 0, pktCnt = 0;
bool rfOK = false;
const uint8_t addr[6] = "QUAD1";

// State
enum { ST_DIS, ST_ARM, ST_FAIL };
uint8_t state = ST_DIS;

// Commands
int16_t tCmd = 0;
float rCmd = 0, pCmd = 0, yCmd = 0;
bool altHoldSw = false, altHoldOn = false;
float tgtAlt = 0;

// PID
float rOut = 0, pOut = 0, yOut = 0, aOut = 0, aPrev = 0;

// Motors
uint16_t mFL = ESC_MIN, mFR = ESC_MIN, mRL = ESC_MIN, mRR = ESC_MIN;
uint16_t pFL = ESC_MIN, pFR = ESC_MIN, pRL = ESC_MIN, pRR = ESC_MIN;

bool prevArm = false;
uint32_t tImu = 0, tPid = 0, tRf = 0, tBar = 0, tDbg = 0, tLed = 0;

// ==================== PID ====================
class PID {
public:
    float kp, ki, kd, intg, pMeas, oMin, oMax, iMax;
    bool init;
    PID(float p, float i, float d) : kp(p), ki(i), kd(d), intg(0), pMeas(0),
        oMin(-500), oMax(500), iMax(200), init(false) {}
    void rst() { intg = 0; pMeas = 0; init = false; }
    float calc(float sp, float m, float dt) {
        float e = sp - m;
        if (!init) { pMeas = m; init = true; }
        float P = kp * e;
        intg = constrain(intg + ki * e * dt, -iMax, iMax);
        float D = kd * (-(m - pMeas) / dt);
        pMeas = m;
        return constrain(P + intg + D, oMin, oMax);
    }
};

PID pidR(6.0, 0.03, 2.5);
PID pidP(6.0, 0.03, 2.5);
PID pidY(4.0, 0.02, 0.0);
PID pidA(15.0, 0.1, 8.0);

// ==================== BEEP ====================
void beep(uint16_t ms, uint16_t f = 2000) { tone(BZR, f, ms); }

// ==================== DMP UPDATE ====================
bool updateDMP() {
    if (!dmpOK) return false;
    
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuf)) {
        mpu.dmpGetQuaternion(&q, fifoBuf);
        mpu.dmpGetGravity(&grav, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &grav);
        
        float newYaw = ypr[0] * 57.2958;
        pitch = ypr[1] * 57.2958;
        roll = ypr[2] * 57.2958;
        
        // Yaw rate
        static uint32_t yawT = 0;
        uint32_t now = micros();
        if (yawT > 0) {
            float dt = (now - yawT) / 1000000.0;
            if (dt > 0 && dt < 0.1) {
                float dY = newYaw - prevYaw;
                if (dY > 180) dY -= 360;
                if (dY < -180) dY += 360;
                yawRate = dY / dt;
            }
        }
        prevYaw = newYaw;
        yaw = newYaw;
        yawT = now;
        return true;
    }
    return false;
}

// ==================== BARO ====================
void updateBaro() {
    if (!baroOK) return;
    baro.read();
    float p = baro.getPressure();
    if (p < 300 || p > 1200) return;
    
    float raw = 44330.0 * (1.0 - pow(p / baseP, 0.1903));
    float chg = constrain(raw - alt, -0.2, 0.2);
    alt += chg;
    altF = 0.85 * altF + 0.15 * alt;
    
    uint32_t now = micros();
    float dt = (now - altT) / 1000000.0;
    if (altT > 0 && dt > 0.01 && dt < 0.1) {
        float rv = constrain((altF - altP) / dt, -3.0, 3.0);
        vvF = 0.8 * vvF + 0.2 * rv;
        vv = vvF;
    }
    altP = altF;
    altT = now;
}

// ==================== RADIO ====================
void updateRadio() {
    if (radio.available()) {
        Pkt p;
        radio.read(&p, sizeof(p));
        if (p.ok()) {
            rx = p;
            lastPkt = millis();
            pktCnt++;
            if (!rfOK) {
                rfOK = true;
                for (int i = 0; i < 5; i++) { beep(80, 2500); delay(160); }
            }
        }
    }
    if (rfOK && millis() - lastPkt > RF_TIMEOUT) {
        rfOK = false;
        if (state == ST_ARM) {
            state = ST_FAIL;
            beep(500, 800);
        }
    }
}

// ==================== COMMANDS ====================
void processCmd() {
    if (!rfOK) { tCmd = 0; rCmd = pCmd = yCmd = 0; return; }
    
    tCmd = rx.thr;
    rCmd = map(rx.roll, -500, 500, -MAX_ANGLE, MAX_ANGLE);
    pCmd = map(rx.pitch, -500, 500, -MAX_ANGLE, MAX_ANGLE);
    yCmd = map(rx.yaw, -500, 500, -MAX_YAW, MAX_YAW);
    
    bool arm = rx.sw & 1;
    altHoldSw = rx.sw & 8;
    
    if (arm && !prevArm && state == ST_DIS) {
        if (tCmd <= ARM_THR_MAX && rfOK && dmpOK) {
            state = ST_ARM;
            pidR.rst(); pidP.rst(); pidY.rst(); pidA.rst();
            aPrev = 0; tgtAlt = altF;
            beep(100, 2000); delay(100); beep(200, 2500);
        }
    } else if (!arm && state == ST_ARM) {
        state = ST_DIS;
        beep(300, 1500);
    }
    
    if (state == ST_FAIL && (!arm || (rfOK && tCmd < 100))) {
        state = ST_DIS;
        beep(200, 1500);
    }
    
    prevArm = arm;
}

// ==================== PID ====================
void updatePID(float dt) {
    float tR = pidR.calc(rCmd, roll, dt);
    float tP = pidP.calc(pCmd, pitch, dt);
    float tY = pidY.calc(yCmd, yawRate, dt);
    
    if (state != ST_ARM) {
        rOut = tR; pOut = tP; yOut = tY;
        aOut = aPrev = 0;
        return;
    }
    
    rOut = tR; pOut = tP; yOut = tY;
    
    bool gnd = (tCmd < 200) || (abs(altF) < 0.3 && abs(vv) < 0.5);
    altHoldOn = altHoldSw && !gnd && baroOK;
    
    if (altHoldOn) {
        if (abs(tCmd - 500) > 50) tgtAlt += ((tCmd - 500) / 500.0) * 0.01;
        float raw = pidA.calc(tgtAlt, altF, dt);
        raw = constrain(raw, -ALT_PID_MAX, ALT_PID_MAX);
        float chg = constrain(raw - aPrev, -ALT_RATE, ALT_RATE);
        aOut = aPrev + chg;
        aPrev = aOut;
    } else {
        aOut = aPrev = 0;
        pidA.rst();
        if (altHoldSw) tgtAlt = altF;
    }
}

// ==================== MOTORS ====================
uint16_t rLim(uint16_t t, uint16_t p) {
    int16_t c = constrain((int16_t)t - (int16_t)p, -MOTOR_RATE, MOTOR_RATE);
    return p + c;
}

void updateMotors() {
    if (state != ST_ARM) {
        mFL = mFR = mRL = mRR = ESC_MIN;
        pFL = pFR = pRL = pRR = ESC_MIN;
    } else {
        int16_t base = map(tCmd, 0, 1000, ESC_IDLE, ESC_MAX) - ESC_MIN;
        if (altHoldOn) base += (int16_t)aOut;
        
        int16_t fl = base - (int16_t)rOut + (int16_t)pOut - (int16_t)yOut;
        int16_t fr = base + (int16_t)rOut + (int16_t)pOut + (int16_t)yOut;
        int16_t rl = base - (int16_t)rOut - (int16_t)pOut + (int16_t)yOut;
        int16_t rr = base + (int16_t)rOut - (int16_t)pOut - (int16_t)yOut;
        
        uint16_t tFL = constrain(fl + ESC_MIN, ESC_IDLE, ESC_MAX);
        uint16_t tFR = constrain(fr + ESC_MIN, ESC_IDLE, ESC_MAX);
        uint16_t tRL = constrain(rl + ESC_MIN, ESC_IDLE, ESC_MAX);
        uint16_t tRR = constrain(rr + ESC_MIN, ESC_IDLE, ESC_MAX);
        
        mFL = rLim(tFL, pFL); pFL = mFL;
        mFR = rLim(tFR, pFR); pFR = mFR;
        mRL = rLim(tRL, pRL); pRL = mRL;
        mRR = rLim(tRR, pRR); pRR = mRR;
    }
    
    sFL.writeMicroseconds(mFL);
    sFR.writeMicroseconds(mFR);
    sRL.writeMicroseconds(mRL);
    sRR.writeMicroseconds(mRR);
}

// ==================== LED ====================
void updateLED() {
    static bool on = false;
    uint16_t iv = (state == ST_ARM) ? 0 : (state == ST_FAIL) ? 100 : 500;
    if (iv == 0) { digitalWrite(LED, HIGH); return; }
    if (millis() - tLed >= iv) {
        tLed = millis();
        on = !on;
        digitalWrite(LED, on);
    }
}

// ==================== SETUP ====================
void setup() {
#if DEBUG
    Serial.begin(BAUD);
    Serial.println(F("QuadFC DMP"));
    Serial.print(F("CH:")); Serial.println(RF_CHANNEL);
#endif
    
    pinMode(LED, OUTPUT);
    pinMode(BZR, OUTPUT);
    digitalWrite(LED, HIGH);
    
    beep(100, 1500); delay(150);
    beep(100, 2000); delay(150);
    beep(200, 2500);
    
    // MPU6050 + DMP
#if DEBUG
    Serial.print(F("MPU..."));
#endif
    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();
    
    if (!mpu.testConnection()) {
#if DEBUG
        Serial.println(F("FAIL-conn"));
#endif
        while(1) { beep(200, 500); delay(400); }
    }
    
    devStat = mpu.dmpInitialize();
    
    // Offsets from IMU_Zero calibration
    mpu.setXAccelOffset(-2366);
    mpu.setYAccelOffset(755);
    mpu.setZAccelOffset(-2006);
    mpu.setXGyroOffset(10);
    mpu.setYGyroOffset(21);
    mpu.setZGyroOffset(-19);
    
    if (devStat == 0) {
#if DEBUG
        Serial.print(F("Cal..."));
#endif
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        pktSize = mpu.dmpGetFIFOPacketSize();
        dmpOK = true;
#if DEBUG
        Serial.println(F("OK"));
#endif
    } else {
#if DEBUG
        Serial.print(F("FAIL-")); Serial.println(devStat);
#endif
        while(1) { beep(200, 600); delay(400); }
    }
    
    // Barometer
#if DEBUG
    Serial.print(F("Baro..."));
#endif
    if (baro.begin()) {
        float sum = 0;
        for (int i = 0; i < 10; i++) { baro.read(); sum += baro.getPressure(); delay(50); }
        baseP = sum / 10.0;
        baroOK = true;
#if DEBUG
        Serial.println(F("OK"));
#endif
    } else {
#if DEBUG
        Serial.println(F("FAIL"));
#endif
    }
    
    // Radio
#if DEBUG
    Serial.print(F("RF..."));
#endif
    if (!radio.begin()) {
#if DEBUG
        Serial.println(F("FAIL"));
#endif
        while(1) { beep(200, 700); delay(400); }
    }
    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF24_2MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(16);
    radio.setAutoAck(true);
    radio.setRetries(5, 3);
    radio.setCRCLength(RF24_CRC_16);
    radio.openReadingPipe(1, addr);
    radio.startListening();
#if DEBUG
    Serial.println(F("OK"));
#endif
    
    // ESCs
    sFL.attach(M_FL, ESC_MIN, ESC_MAX);
    sFR.attach(M_FR, ESC_MIN, ESC_MAX);
    sRL.attach(M_RL, ESC_MIN, ESC_MAX);
    sRR.attach(M_RR, ESC_MIN, ESC_MAX);
    sFL.writeMicroseconds(ESC_MIN);
    sFR.writeMicroseconds(ESC_MIN);
    sRL.writeMicroseconds(ESC_MIN);
    sRR.writeMicroseconds(ESC_MIN);
    
    pidA.oMin = -ALT_PID_MAX;
    pidA.oMax = ALT_PID_MAX;
    pidA.iMax = 50;
    
#if DEBUG
    Serial.println(F("Ready!"));
#endif
    beep(100, 2000); delay(100);
    beep(100, 2500); delay(100);
    beep(200, 3000);
    
    tImu = tPid = tRf = tBar = micros();
    tDbg = tLed = millis();
}

// ==================== LOOP ====================
void loop() {
    uint32_t now = micros();
    
    // DMP ~100Hz (as fast as available)
    updateDMP();
    
    // Radio 50Hz
    if (now - tRf >= 20000) {
        tRf = now;
        updateRadio();
        processCmd();
    }
    
    // Baro 40Hz
    if (now - tBar >= 25000) {
        tBar = now;
        updateBaro();
    }
    
    // PID + Motors 250Hz
    if (now - tPid >= 4000) {
        float dt = (now - tPid) / 1000000.0;
        tPid = now;
        updatePID(dt);
        updateMotors();
    }
    
    updateLED();
    
#if DEBUG
    if (millis() - tDbg >= 200) {
        tDbg = millis();
        Serial.print(state == ST_ARM ? F("ARM") : state == ST_FAIL ? F("FAIL") : F("DIS"));
        Serial.print(F(" RF:")); Serial.print(rfOK ? pktCnt : 0);
        Serial.print(F(" R:")); Serial.print(roll, 1);
        Serial.print(F(" P:")); Serial.print(pitch, 1);
        Serial.print(F(" PID:")); Serial.print(rOut, 0);
        Serial.print(F(",")); Serial.print(pOut, 0);
        Serial.print(F(" M:")); Serial.print(mFL);
        Serial.print(F(",")); Serial.print(mFR);
        Serial.print(F(",")); Serial.print(mRL);
        Serial.print(F(",")); Serial.println(mRR);
    }
#endif
}
