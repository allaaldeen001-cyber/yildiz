/**
 * ============================================================================
 * QUADCOPTER FLIGHT CONTROLLER - COMPACT VERSION
 * ============================================================================
 * 
 * Target: Arduino Nano (ATmega328P @ 16MHz)
 * Optimized for minimal flash usage
 * 
 * Libraries Required:
 *   - Wire (built-in)
 *   - SPI (built-in)  
 *   - RF24 (TMRh20)
 *   - MS5611 (Rob Tillaart)
 *   - Servo (built-in)
 * 
 * ============================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <MS5611.h>
#include <Servo.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define RF_CHANNEL      108       // Must match remote!
#define DEBUG_ENABLED   1         // Set to 0 to save ~2KB flash
#define SERIAL_BAUD     115200

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

#define PIN_MOTOR_FL    3
#define PIN_MOTOR_FR    5
#define PIN_MOTOR_RL    6
#define PIN_MOTOR_RR    9
#define PIN_RF_CE       4
#define PIN_RF_CSN      10
#define PIN_MPU_INT     2
#define PIN_LED         7
#define PIN_BUZZER      8

// ============================================================================
// FLIGHT PARAMETERS
// ============================================================================

#define MAX_ANGLE       45.0f
#define MAX_YAW_RATE    180.0f
#define ESC_MIN         1000
#define ESC_MAX         2000
#define ESC_IDLE        1150
#define THROTTLE_ARM_MAX 50

// ============================================================================
// TIMING (microseconds)
// ============================================================================

#define IMU_PERIOD      2000      // 500 Hz
#define PID_PERIOD      4000      // 250 Hz
#define RF_PERIOD       20000     // 50 Hz
#define BARO_PERIOD     25000     // 40 Hz

// ============================================================================
// ALTITUDE HOLD LIMITS
// ============================================================================

#define ALT_LPF_ALPHA       0.85f
#define VVEL_LPF_ALPHA      0.80f
#define ALT_MAX_CHANGE      0.20f
#define VVEL_MAX            3.0f
#define ALT_GROUND_THRESH   0.30f
#define THR_GROUND_THRESH   200
#define ALT_PID_MAX         150.0f
#define ALT_PID_RATE        50.0f
#define MOTOR_RATE_LIMIT    100    // INCREASED from 50 for faster response

// ============================================================================
// RF PACKET (16 bytes)
// ============================================================================

struct __attribute__((packed)) Packet {
    uint16_t throttle;
    int16_t yaw, pitch, roll;
    uint8_t switches;
    uint8_t checksum;
    uint32_t seq;
    uint8_t ch;
    uint8_t res;
    
    bool valid() const {
        const uint8_t* d = (const uint8_t*)this;
        uint8_t c = 0;
        for (uint8_t i = 0; i < 9; i++) c ^= d[i];
        return c == checksum;
    }
};

// Switch bits
#define SW_ARM      0
#define SW_CALIB    1
#define SW_MOTOR    2
#define SW_ALTHOLD  3

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

RF24 radio(PIN_RF_CE, PIN_RF_CSN);
MS5611 baro(0x77);
Servo escFL, escFR, escRL, escRR;

// ============================================================================
// MPU6050 REGISTERS
// ============================================================================

#define MPU_ADDR        0x68
#define REG_PWR_MGMT    0x6B
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_ACCEL_OUT   0x3B
#define REG_SMPLRT      0x19

// ============================================================================
// IMU VARIABLES
// ============================================================================

int16_t accelRaw[3], gyroRaw[3];
int16_t accelOff[3], gyroOff[3];
float accelG[3], gyroDeg[3];
float roll = 0, pitch = 0, yaw = 0;
bool imuReady = false, imuCalibrated = false;

// Complementary filter
// Lower = faster response to tilt, but more noise
// Higher = smoother but slower response
#define COMP_ALPHA  0.96f   // Reduced from 0.98 for faster tilt response

// ============================================================================
// BAROMETER VARIABLES  
// ============================================================================

float basePressure = 0;
float altitude = 0, altFiltered = 0, altPrev = 0;
float vVel = 0, vVelFiltered = 0;
uint32_t altPrevTime = 0;
bool baroOK = false;

// ============================================================================
// RADIO VARIABLES
// ============================================================================

Packet rxPkt;
uint32_t lastPktTime = 0, pktCount = 0, pktLost = 0, lastSeq = 0;
bool rfOK = false, rfPaired = false;
const uint8_t rfAddr[6] = "QUAD1";

// ============================================================================
// FLIGHT STATE
// ============================================================================

enum State { ST_INIT, ST_DISARMED, ST_ARMED, ST_FAILSAFE };
State state = ST_INIT;

int16_t thrCmd = 0;
float rollCmd = 0, pitchCmd = 0, yawCmd = 0;
bool altHoldSw = false, altHoldActive = false;
float targetAlt = 0;

// PID outputs
float rollOut = 0, pitchOut = 0, yawOut = 0, altOut = 0, prevAltOut = 0;

// Motor values
uint16_t mFL = ESC_MIN, mFR = ESC_MIN, mRL = ESC_MIN, mRR = ESC_MIN;
uint16_t prevFL = ESC_MIN, prevFR = ESC_MIN, prevRL = ESC_MIN, prevRR = ESC_MIN;

// Switches
bool prevArm = false;

// Timing
uint32_t lastImu = 0, lastPid = 0, lastRf = 0, lastBaro = 0;
uint32_t lastDebug = 0, lastLed = 0;
bool ledOn = false;

// ============================================================================
// SIMPLE PID CLASS
// ============================================================================

class PID {
public:
    float kp, ki, kd;
    float integral, prevMeas;
    float outMin, outMax, intMax;
    bool init;
    
    PID(float p, float i, float d) : kp(p), ki(i), kd(d), 
        integral(0), prevMeas(0), outMin(-500), outMax(500), intMax(200), init(false) {}
    
    void reset() { integral = 0; prevMeas = 0; init = false; }
    
    float calc(float sp, float meas, float dt) {
        float err = sp - meas;
        if (!init) { prevMeas = meas; init = true; }
        
        float p = kp * err;
        integral = constrain(integral + ki * err * dt, -intMax, intMax);
        float d = kd * (-(meas - prevMeas) / dt);
        prevMeas = meas;
        
        return constrain(p + integral + d, outMin, outMax);
    }
};

// ============================================================================
// PID TUNING - ADJUST THESE FOR YOUR QUAD!
// ============================================================================
// Increase Kp if response is too weak
// Increase Kd if oscillating
// Increase Ki if drifting

//              Kp     Ki     Kd
PID pidRoll  (6.0f,  0.03f,  2.5f);   // Roll  - INCREASED for stronger response
PID pidPitch (6.0f,  0.03f,  2.5f);   // Pitch - INCREASED for stronger response
PID pidYaw   (4.0f,  0.02f,  0.0f);   // Yaw rate
PID pidAlt   (15.0f, 0.1f,   8.0f);   // Altitude

// ============================================================================
// MPU6050 FUNCTIONS
// ============================================================================

void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

bool mpuInit() {
    Wire.begin();
    Wire.setClock(400000);
    
    // Check connection
    Wire.beginTransmission(MPU_ADDR);
    if (Wire.endTransmission() != 0) return false;
    
    mpuWrite(REG_PWR_MGMT, 0x00);   // Wake up
    delay(10);
    mpuWrite(REG_PWR_MGMT, 0x01);   // PLL with X gyro
    mpuWrite(REG_SMPLRT, 0x00);     // 1kHz sample rate
    mpuWrite(REG_CONFIG, 0x03);     // 44Hz DLPF
    mpuWrite(REG_GYRO_CFG, 0x00);   // ±250°/s
    mpuWrite(REG_ACCEL_CFG, 0x00);  // ±2g
    
    return true;
}

bool mpuRead() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_OUT);
    if (Wire.endTransmission(false) != 0) return false;
    
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
    if (Wire.available() < 14) return false;
    
    accelRaw[0] = (Wire.read() << 8) | Wire.read();
    accelRaw[1] = (Wire.read() << 8) | Wire.read();
    accelRaw[2] = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  // Skip temp
    gyroRaw[0] = (Wire.read() << 8) | Wire.read();
    gyroRaw[1] = (Wire.read() << 8) | Wire.read();
    gyroRaw[2] = (Wire.read() << 8) | Wire.read();
    
    return true;
}

void mpuCalibrate() {
    int32_t sa[3] = {0}, sg[3] = {0};
    
    for (int i = 0; i < 500; i++) {
        mpuRead();
        for (int j = 0; j < 3; j++) {
            sa[j] += accelRaw[j];
            sg[j] += gyroRaw[j];
        }
        delay(2);
    }
    
    for (int j = 0; j < 3; j++) {
        accelOff[j] = sa[j] / 500;
        gyroOff[j] = sg[j] / 500;
    }
    accelOff[2] -= 16384;  // Z should read 1g
    
    imuCalibrated = true;
}

void mpuProcess() {
    // Apply calibration and convert
    accelG[0] = (accelRaw[0] - accelOff[0]) / 16384.0f;
    accelG[1] = (accelRaw[1] - accelOff[1]) / 16384.0f;
    accelG[2] = (accelRaw[2] - accelOff[2]) / 16384.0f;
    gyroDeg[0] = (gyroRaw[0] - gyroOff[0]) / 131.0f;
    gyroDeg[1] = (gyroRaw[1] - gyroOff[1]) / 131.0f;
    gyroDeg[2] = (gyroRaw[2] - gyroOff[2]) / 131.0f;
}

void updateAttitude(float dt) {
    // Angles from accelerometer
    float accRoll = atan2(accelG[1], accelG[2]) * 57.2958f;
    float accPitch = atan2(-accelG[0], sqrt(accelG[1]*accelG[1] + accelG[2]*accelG[2])) * 57.2958f;
    
    // Complementary filter
    roll = COMP_ALPHA * (roll + gyroDeg[0] * dt) + (1 - COMP_ALPHA) * accRoll;
    pitch = COMP_ALPHA * (pitch + gyroDeg[1] * dt) + (1 - COMP_ALPHA) * accPitch;
    yaw += gyroDeg[2] * dt;
    
    // Normalize yaw
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
}

// ============================================================================
// BAROMETER FUNCTIONS
// ============================================================================

bool baroInit() {
    if (!baro.begin()) return false;
    
    // Get baseline
    float sum = 0;
    for (int i = 0; i < 10; i++) {
        baro.read();
        sum += baro.getPressure();
        delay(50);
    }
    basePressure = sum / 10.0f;
    
    return true;
}

void baroUpdate() {
    baro.read();
    float p = baro.getPressure();
    
    if (p < 300 || p > 1200) return;  // Invalid
    
    // Raw altitude
    float rawAlt = 44330.0f * (1.0f - pow(p / basePressure, 0.1903f));
    
    // Rate limit
    float change = constrain(rawAlt - altitude, -ALT_MAX_CHANGE, ALT_MAX_CHANGE);
    altitude += change;
    
    // LPF
    altFiltered = ALT_LPF_ALPHA * altFiltered + (1 - ALT_LPF_ALPHA) * altitude;
    
    // Velocity from filtered altitude
    uint32_t now = micros();
    float dt = (now - altPrevTime) / 1000000.0f;
    
    if (altPrevTime > 0 && dt > 0.01f && dt < 0.1f) {
        float rawVel = constrain((altFiltered - altPrev) / dt, -VVEL_MAX, VVEL_MAX);
        vVelFiltered = VVEL_LPF_ALPHA * vVelFiltered + (1 - VVEL_LPF_ALPHA) * rawVel;
        vVel = vVelFiltered;
    }
    
    altPrev = altFiltered;
    altPrevTime = now;
}

// ============================================================================
// RADIO FUNCTIONS
// ============================================================================

bool radioInit() {
    if (!radio.begin()) return false;
    
    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF24_2MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(16);
    radio.setAutoAck(true);
    radio.setRetries(5, 3);
    radio.setCRCLength(RF24_CRC_16);
    radio.openReadingPipe(1, rfAddr);
    radio.startListening();
    
    return true;
}

void radioUpdate() {
    if (radio.available()) {
        Packet pkt;
        radio.read(&pkt, sizeof(pkt));
        
        if (pkt.valid()) {
            if (pktCount > 0 && pkt.seq > lastSeq + 1) {
                pktLost += pkt.seq - lastSeq - 1;
            }
            lastSeq = pkt.seq;
            rxPkt = pkt;
            lastPktTime = millis();
            pktCount++;
            
            if (!rfPaired) {
                rfPaired = true;
                // 5 beeps for pairing
                for (int i = 0; i < 5; i++) {
                    tone(PIN_BUZZER, 2500, 80);
                    delay(160);
                }
            }
            rfOK = true;
        }
    }
    
    // Check timeout
    if (rfOK && millis() - lastPktTime > 500) {
        rfOK = false;
        if (state == ST_ARMED) {
            state = ST_FAILSAFE;
            tone(PIN_BUZZER, 800, 500);
        }
    }
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

void processCommands() {
    if (!rfOK) {
        thrCmd = 0;
        rollCmd = pitchCmd = yawCmd = 0;
        return;
    }
    
    thrCmd = rxPkt.throttle;
    rollCmd = map(rxPkt.roll, -500, 500, -MAX_ANGLE, MAX_ANGLE);
    pitchCmd = map(rxPkt.pitch, -500, 500, -MAX_ANGLE, MAX_ANGLE);
    yawCmd = map(rxPkt.yaw, -500, 500, -MAX_YAW_RATE, MAX_YAW_RATE);
    
    bool armSw = rxPkt.switches & (1 << SW_ARM);
    altHoldSw = rxPkt.switches & (1 << SW_ALTHOLD);
    
    // Arm/disarm
    if (armSw && !prevArm && state == ST_DISARMED) {
        if (thrCmd <= THROTTLE_ARM_MAX && rfOK && imuCalibrated) {
            state = ST_ARMED;
            pidRoll.reset(); pidPitch.reset(); pidYaw.reset(); pidAlt.reset();
            prevAltOut = 0;
            targetAlt = altFiltered;
            tone(PIN_BUZZER, 2000, 100); delay(100);
            tone(PIN_BUZZER, 2500, 200);
        }
    } else if (!armSw && state == ST_ARMED) {
        state = ST_DISARMED;
        tone(PIN_BUZZER, 1500, 300);
    }
    
    // Failsafe recovery
    if (!armSw && state == ST_FAILSAFE) {
        state = ST_DISARMED;
    }
    
    prevArm = armSw;
}

// ============================================================================
// PID UPDATE
// ============================================================================

void updatePID(float dt) {
    // ALWAYS calculate PID for roll/pitch (so we can see response in debug)
    // This helps testing auto-level by hand without arming
    
    float tempRollOut = pidRoll.calc(rollCmd, roll, dt);
    float tempPitchOut = pidPitch.calc(pitchCmd, pitch, dt);
    float tempYawOut = pidYaw.calc(yawCmd, gyroDeg[2], dt);
    
    if (state != ST_ARMED) {
        // Store PID outputs for debug display, but don't apply to motors
        rollOut = tempRollOut;
        pitchOut = tempPitchOut;
        yawOut = tempYawOut;
        altOut = prevAltOut = 0;
        return;  // Don't reset PID - keeps calculating for debug view
    }
    
    // When armed, use the calculated values
    rollOut = tempRollOut;
    pitchOut = tempPitchOut;
    yawOut = tempYawOut;
    
    // Ground detection
    bool onGround = (thrCmd < THR_GROUND_THRESH) || 
                    (abs(altFiltered) < ALT_GROUND_THRESH && abs(vVel) < 0.5f);
    altHoldActive = altHoldSw && !onGround && baroOK;
    
    if (altHoldActive) {
        float thrDev = thrCmd - 500;
        if (abs(thrDev) > 50) {
            targetAlt += (thrDev / 500.0f) * 0.01f;
        }
        
        float raw = pidAlt.calc(targetAlt, altFiltered, dt);
        raw = constrain(raw, -ALT_PID_MAX, ALT_PID_MAX);
        
        // Rate limit
        float change = constrain(raw - prevAltOut, -ALT_PID_RATE, ALT_PID_RATE);
        altOut = prevAltOut + change;
        prevAltOut = altOut;
    } else {
        altOut = prevAltOut = 0;
        pidAlt.reset();
        if (altHoldSw) targetAlt = altFiltered;
    }
}

// ============================================================================
// MOTOR MIXING
// ============================================================================

uint16_t rateLimit(uint16_t target, uint16_t prev) {
    int16_t change = constrain((int16_t)target - (int16_t)prev, -MOTOR_RATE_LIMIT, MOTOR_RATE_LIMIT);
    return prev + change;
}

void updateMotors() {
    if (state != ST_ARMED) {
        mFL = mFR = mRL = mRR = ESC_MIN;
        prevFL = prevFR = prevRL = prevRR = ESC_MIN;
    } else {
        int16_t base = map(thrCmd, 0, 1000, ESC_IDLE, ESC_MAX) - ESC_MIN;
        if (altHoldActive) base += (int16_t)altOut;
        
        int16_t fl = base - (int16_t)rollOut + (int16_t)pitchOut - (int16_t)yawOut;
        int16_t fr = base + (int16_t)rollOut + (int16_t)pitchOut + (int16_t)yawOut;
        int16_t rl = base - (int16_t)rollOut - (int16_t)pitchOut + (int16_t)yawOut;
        int16_t rr = base + (int16_t)rollOut - (int16_t)pitchOut - (int16_t)yawOut;
        
        uint16_t tFL = constrain(fl + ESC_MIN, ESC_IDLE, ESC_MAX);
        uint16_t tFR = constrain(fr + ESC_MIN, ESC_IDLE, ESC_MAX);
        uint16_t tRL = constrain(rl + ESC_MIN, ESC_IDLE, ESC_MAX);
        uint16_t tRR = constrain(rr + ESC_MIN, ESC_IDLE, ESC_MAX);
        
        mFL = rateLimit(tFL, prevFL); prevFL = mFL;
        mFR = rateLimit(tFR, prevFR); prevFR = mFR;
        mRL = rateLimit(tRL, prevRL); prevRL = mRL;
        mRR = rateLimit(tRR, prevRR); prevRR = mRR;
    }
    
    escFL.writeMicroseconds(mFL);
    escFR.writeMicroseconds(mFR);
    escRL.writeMicroseconds(mRL);
    escRR.writeMicroseconds(mRR);
}

// ============================================================================
// LED UPDATE
// ============================================================================

void updateLED() {
    uint32_t now = millis();
    uint16_t interval = (state == ST_ARMED) ? 0 : 
                        (state == ST_FAILSAFE) ? 100 : 500;
    
    if (interval == 0) {
        digitalWrite(PIN_LED, HIGH);
    } else if (now - lastLed >= interval) {
        lastLed = now;
        ledOn = !ledOn;
        digitalWrite(PIN_LED, ledOn);
    }
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

#if DEBUG_ENABLED
void printDebug() {
    Serial.println(F("----------------------------------------"));
    
    // State
    Serial.print(F("State: "));
    Serial.print(state == ST_ARMED ? F("ARMED") : state == ST_FAILSAFE ? F("FAILSAFE") : F("DISARMED"));
    Serial.print(F("  RF: "));
    Serial.println(rfOK ? F("Connected") : F("Disconnected"));
    
    // Angles - TILT THE DRONE TO SEE THESE CHANGE
    Serial.print(F("Angles -> Roll: "));
    Serial.print(roll, 1);
    Serial.print(F("°  Pitch: "));
    Serial.print(pitch, 1);
    Serial.print(F("°  Yaw: "));
    Serial.print(yaw, 1);
    Serial.println(F("°"));
    
    // PID OUTPUT - THESE SHOULD CHANGE WHEN YOU TILT!
    Serial.print(F("PID Out-> Roll: "));
    Serial.print(rollOut, 0);
    Serial.print(F("  Pitch: "));
    Serial.print(pitchOut, 0);
    Serial.print(F("  Yaw: "));
    Serial.println(yawOut, 0);
    
    // Command from RC
    Serial.print(F("RC Cmd -> Thr: "));
    Serial.print(thrCmd);
    Serial.print(F("  Roll: "));
    Serial.print(rollCmd, 0);
    Serial.print(F("  Pitch: "));
    Serial.print(pitchCmd, 0);
    Serial.println();
    
    // Motors
    Serial.print(F("Motors -> FL:"));
    Serial.print(mFL);
    Serial.print(F(" FR:"));
    Serial.print(mFR);
    Serial.print(F(" RL:"));
    Serial.print(mRL);
    Serial.print(F(" RR:"));
    Serial.println(mRR);
}
#endif

// ============================================================================
// SETUP
// ============================================================================

void setup() {
#if DEBUG_ENABLED
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("\nQuad FC v2.1"));
    Serial.print(F("CH:")); Serial.println(RF_CHANNEL);
#endif
    
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    // Startup beeps
    tone(PIN_BUZZER, 1500, 100); delay(150);
    tone(PIN_BUZZER, 2000, 100); delay(150);
    tone(PIN_BUZZER, 2500, 200); delay(250);
    
    // Init MPU6050
#if DEBUG_ENABLED
    Serial.print(F("MPU..."));
#endif
    if (!mpuInit()) {
#if DEBUG_ENABLED
        Serial.println(F("FAIL"));
#endif
        while(1) { tone(PIN_BUZZER, 500, 200); delay(400); }
    }
    imuReady = true;
#if DEBUG_ENABLED
    Serial.println(F("OK"));
    Serial.print(F("Calibrating..."));
#endif
    mpuCalibrate();
#if DEBUG_ENABLED
    Serial.println(F("OK"));
#endif
    
    // Init barometer
#if DEBUG_ENABLED
    Serial.print(F("Baro..."));
#endif
    baroOK = baroInit();
#if DEBUG_ENABLED
    Serial.println(baroOK ? F("OK") : F("FAIL"));
#endif
    
    // Init radio
#if DEBUG_ENABLED
    Serial.print(F("RF..."));
#endif
    if (!radioInit()) {
#if DEBUG_ENABLED
        Serial.println(F("FAIL"));
#endif
        while(1) { tone(PIN_BUZZER, 600, 200); delay(400); }
    }
#if DEBUG_ENABLED
    Serial.println(F("OK"));
#endif
    
    // Init ESCs
    escFL.attach(PIN_MOTOR_FL, ESC_MIN, ESC_MAX);
    escFR.attach(PIN_MOTOR_FR, ESC_MIN, ESC_MAX);
    escRL.attach(PIN_MOTOR_RL, ESC_MIN, ESC_MAX);
    escRR.attach(PIN_MOTOR_RR, ESC_MIN, ESC_MAX);
    escFL.writeMicroseconds(ESC_MIN);
    escFR.writeMicroseconds(ESC_MIN);
    escRL.writeMicroseconds(ESC_MIN);
    escRR.writeMicroseconds(ESC_MIN);
    
    // Init PID limits
    pidAlt.outMin = -ALT_PID_MAX;
    pidAlt.outMax = ALT_PID_MAX;
    pidAlt.intMax = 50;
    
    state = ST_DISARMED;
    
#if DEBUG_ENABLED
    Serial.println(F("Ready!"));
#endif
    tone(PIN_BUZZER, 2000, 100); delay(100);
    tone(PIN_BUZZER, 2500, 100); delay(100);
    tone(PIN_BUZZER, 3000, 200);
    
    lastImu = lastPid = lastRf = lastBaro = micros();
    lastDebug = lastLed = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    uint32_t now = micros();
    
    // IMU (500 Hz)
    if (now - lastImu >= IMU_PERIOD) {
        float dt = (now - lastImu) / 1000000.0f;
        lastImu = now;
        
        if (mpuRead()) {
            mpuProcess();
            updateAttitude(dt);
        }
    }
    
    // Radio (50 Hz)
    if (now - lastRf >= RF_PERIOD) {
        lastRf = now;
        radioUpdate();
        processCommands();
    }
    
    // Barometer (40 Hz)
    if (now - lastBaro >= BARO_PERIOD) {
        lastBaro = now;
        if (baroOK) baroUpdate();
    }
    
    // PID + Motors (250 Hz)
    if (now - lastPid >= PID_PERIOD) {
        float dt = (now - lastPid) / 1000000.0f;
        lastPid = now;
        
        updatePID(dt);
        updateMotors();
    }
    
    // LED
    updateLED();
    
    // Debug (5 Hz)
#if DEBUG_ENABLED
    if (millis() - lastDebug >= 200) {
        lastDebug = millis();
        printDebug();
    }
#endif
}
