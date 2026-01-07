/**
 * ============================================================================
 *                    QUADCOPTER FLIGHT CONTROLLER v4.0
 *                        OPTIMIZED FOR STABILITY
 * ============================================================================
 * 
 * Target Hardware:
 *   Frame:   F330 (330mm)
 *   Motors:  RS2205 2300KV (fast response!)
 *   Battery: 3S 11.1V 1500mAh
 *   ESC:     BLHeli (recommended)
 * 
 * Optimizations:
 *   - Removed barometer (more flash space)
 *   - MPU6050 DMP for hardware sensor fusion (best stability)
 *   - PID tuned for 2300KV motors
 *   - D-term filtering to reduce noise
 *   - Motor output smoothing
 *   - Anti-windup on integral
 *   - 250Hz PID loop
 * 
 * ============================================================================
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// ============================================================================
//                              CONFIGURATION
// ============================================================================
#define RF_CHANNEL          108
#define ENABLE_DEBUG        1
#define SERIAL_BAUD         115200

// ============================================================================
//                            PIN DEFINITIONS
// ============================================================================
#define PIN_MOTOR_FL        3       // Front-Left  (CCW)
#define PIN_MOTOR_FR        5       // Front-Right (CW)
#define PIN_MOTOR_RL        6       // Rear-Left   (CW)
#define PIN_MOTOR_RR        9       // Rear-Right  (CCW)
#define PIN_RF_CE           4
#define PIN_RF_CSN          10
#define PIN_LED             7
#define PIN_BUZZER          8

// ============================================================================
//                          FLIGHT PARAMETERS
// ============================================================================
// Max angles (degrees) - reduced for stability
#define MAX_ROLL_ANGLE      40
#define MAX_PITCH_ANGLE     40
#define MAX_YAW_RATE        200     // degrees/sec

// ESC Configuration
#define ESC_MIN_US          1000
#define ESC_MAX_US          2000
#define ESC_IDLE_US         1080    // Low idle for 2300KV
#define ESC_ARM_THROTTLE    50

// Timing
#define RF_TIMEOUT_MS       500
#define PID_LOOP_US         4000    // 250Hz PID loop

// Motor smoothing (0.0-1.0, lower = smoother)
#define MOTOR_SMOOTH_ALPHA  0.4f

// ============================================================================
//                         MPU6050 CALIBRATION
//          Run IMU_Zero sketch to get YOUR sensor's values!
// ============================================================================
#define ACCEL_OFFSET_X      -2366
#define ACCEL_OFFSET_Y      755
#define ACCEL_OFFSET_Z      -2006
#define GYRO_OFFSET_X       10
#define GYRO_OFFSET_Y       21
#define GYRO_OFFSET_Z       -19

// ============================================================================
//                           PID TUNING
//              Tuned for RS2205 2300KV on F330 frame
// ============================================================================
// ROLL PID - Start conservative, increase if sluggish
#define PID_ROLL_KP         2.5f    // Proportional (main response)
#define PID_ROLL_KI         0.02f   // Integral (drift correction)
#define PID_ROLL_KD         1.2f    // Derivative (dampening)

// PITCH PID - Usually same as roll for symmetric frame
#define PID_PITCH_KP        2.5f
#define PID_PITCH_KI        0.02f
#define PID_PITCH_KD        1.2f

// YAW PID - Rate-based control
#define PID_YAW_KP          3.0f
#define PID_YAW_KI          0.02f
#define PID_YAW_KD          0.0f    // Usually 0 for yaw

// PID Limits
#define PID_OUTPUT_MAX      400     // Max PID output
#define PID_INTEGRAL_MAX    100     // Anti-windup limit

// D-term low-pass filter coefficient (0.0-1.0, lower = more filtering)
#define DTERM_LPF_ALPHA     0.3f

// ============================================================================
//                          RF PACKET STRUCTURE
// ============================================================================
struct __attribute__((packed)) ControlPacket {
    uint16_t throttle;      // 0-1000
    int16_t  yaw;           // -500 to +500
    int16_t  pitch;         // -500 to +500
    int16_t  roll;          // -500 to +500
    uint8_t  switches;      // Bit flags
    uint8_t  checksum;
    uint32_t sequence;
    uint8_t  ratePot;       // Rate adjustment 0-255
    uint8_t  expoPot;       // Expo (processed on RC)
    
    bool isValid() const {
        const uint8_t* data = (const uint8_t*)this;
        uint8_t calc = 0;
        for (uint8_t i = 0; i < 9; i++) calc ^= data[i];
        return calc == checksum;
    }
};

// Switch bit definitions
#define SW_ARM              0
#define SW_CALIBRATE        1
#define SW_MOTORTEST        2
#define SW_ALTHOLD          3
#define SW_BEEPER           4
#define SW_HEADLESS         5

// ============================================================================
//                           GLOBAL OBJECTS
// ============================================================================
MPU6050 mpu;
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
Servo escFL, escFR, escRL, escRR;
const uint8_t radioAddress[6] = "QUAD1";

// ============================================================================
//                             DMP VARIABLES
// ============================================================================
bool dmpReady = false;
uint8_t dmpStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// Attitude (degrees)
float roll = 0, pitch = 0, yaw = 0;

// Gyro rates (degrees/sec) - from DMP quaternion derivative
float rollRate = 0, pitchRate = 0, yawRate = 0;
float prevRoll = 0, prevPitch = 0, prevYaw = 0;
uint32_t attitudeTime = 0;

// Headless mode
float headlessRef = 0;
bool headlessMode = false;

// ============================================================================
//                            RADIO VARIABLES
// ============================================================================
ControlPacket rxPacket;
uint32_t lastPacketTime = 0;
uint32_t packetCount = 0;
bool radioConnected = false;
float rateMultiplier = 1.0f;

// ============================================================================
//                           FLIGHT STATE
// ============================================================================
enum FlightState { STATE_DISARMED, STATE_ARMED, STATE_FAILSAFE };
FlightState flightState = STATE_DISARMED;

// Commands
int16_t throttleCmd = 0;
float rollCmd = 0, pitchCmd = 0, yawCmd = 0;

// PID outputs
float rollPID = 0, pitchPID = 0, yawPID = 0;

// Motor values
float motorFL = ESC_MIN_US, motorFR = ESC_MIN_US;
float motorRL = ESC_MIN_US, motorRR = ESC_MIN_US;

// State tracking
bool prevArmSwitch = false;
bool prevHeadlessBtn = false;
bool prevBeeperBtn = false;

// Timing
uint32_t timePID = 0;
uint32_t timeDebug = 0;
uint32_t timeLED = 0;

// ============================================================================
//                         OPTIMIZED PID CONTROLLER
// ============================================================================
class PID {
public:
    float Kp, Ki, Kd;
    float integral;
    float prevError;
    float prevDterm;
    bool firstRun;
    
    PID(float p, float i, float d) : 
        Kp(p), Ki(i), Kd(d), integral(0), prevError(0), prevDterm(0), firstRun(true) {}
    
    void reset() {
        integral = 0;
        prevError = 0;
        prevDterm = 0;
        firstRun = true;
    }
    
    float compute(float setpoint, float measurement, float rate, float dt) {
        float error = setpoint - measurement;
        
        // First run - initialize
        if (firstRun) {
            prevError = error;
            firstRun = false;
            return 0;
        }
        
        // P-term
        float pTerm = Kp * error;
        
        // I-term with anti-windup
        integral += error * dt;
        integral = constrain(integral, -PID_INTEGRAL_MAX / Ki, PID_INTEGRAL_MAX / Ki);
        float iTerm = Ki * integral;
        
        // D-term on measurement rate (not error) with low-pass filter
        // Using gyro rate directly is smoother than differentiating angle
        float dTerm = -Kd * rate;
        
        // Low-pass filter on D-term to reduce noise
        dTerm = prevDterm * (1.0f - DTERM_LPF_ALPHA) + dTerm * DTERM_LPF_ALPHA;
        prevDterm = dTerm;
        
        prevError = error;
        
        // Sum and constrain
        float output = pTerm + iTerm + dTerm;
        return constrain(output, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
    }
};

PID pidRoll(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
PID pidPitch(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
PID pidYaw(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD);

// ============================================================================
//                            BUZZER FUNCTIONS
// ============================================================================
void beep(uint16_t ms, uint16_t freq = 2000) {
    tone(PIN_BUZZER, freq, ms);
}

void beepBlocking(uint16_t ms, uint16_t freq = 2000) {
    tone(PIN_BUZZER, freq);
    delay(ms);
    noTone(PIN_BUZZER);
}

void soundStartup() {
    beepBlocking(100, 1500);
    delay(50);
    beepBlocking(100, 2000);
    delay(50);
    beepBlocking(200, 2500);
}

void soundArmed() {
    beepBlocking(100, 2000);
    delay(100);
    beepBlocking(200, 2500);
}

void soundDisarmed() {
    beepBlocking(300, 1500);
}

void soundPaired() {
    for (int i = 0; i < 5; i++) {
        beepBlocking(60, 2500);
        delay(60);
    }
}

void soundReady() {
    beepBlocking(100, 2000);
    delay(100);
    beepBlocking(100, 2500);
    delay(100);
    beepBlocking(200, 3000);
}

// ============================================================================
//                        ESC CALIBRATION MODE
// ============================================================================
void escCalibration() {
    Serial.println(F("\n*** ESC CALIBRATION ***"));
    
    escFL.writeMicroseconds(ESC_MAX_US);
    escFR.writeMicroseconds(ESC_MAX_US);
    escRL.writeMicroseconds(ESC_MAX_US);
    escRR.writeMicroseconds(ESC_MAX_US);
    
    beepBlocking(1000, 3000);
    Serial.println(F("MAX - Release button after ESC beeps"));
    
    // Wait for button release
    uint32_t start = millis();
    while (millis() - start < 15000) {
        if (radio.available()) {
            ControlPacket p;
            radio.read(&p, sizeof(p));
            if (p.isValid() && !(p.switches & (1 << SW_CALIBRATE))) break;
        }
        delay(50);
    }
    
    escFL.writeMicroseconds(ESC_MIN_US);
    escFR.writeMicroseconds(ESC_MIN_US);
    escRL.writeMicroseconds(ESC_MIN_US);
    escRR.writeMicroseconds(ESC_MIN_US);
    
    beepBlocking(500, 1500);
    delay(2000);
    
    Serial.println(F("ESC Calibration DONE!"));
    soundReady();
}

// ============================================================================
//                           DMP UPDATE
// ============================================================================
void updateDMP() {
    if (!dmpReady) return;
    
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // Convert to degrees
        float newYaw = ypr[0] * 57.2958f;
        float newPitch = ypr[1] * 57.2958f;
        float newRoll = ypr[2] * 57.2958f;
        
        // Calculate rates from angle change
        uint32_t now = micros();
        float dt = (now - attitudeTime) / 1000000.0f;
        
        if (attitudeTime > 0 && dt > 0.001f && dt < 0.05f) {
            // Rate calculation with wrap-around handling for yaw
            rollRate = (newRoll - prevRoll) / dt;
            pitchRate = (newPitch - prevPitch) / dt;
            
            float deltaYaw = newYaw - prevYaw;
            if (deltaYaw > 180) deltaYaw -= 360;
            if (deltaYaw < -180) deltaYaw += 360;
            yawRate = deltaYaw / dt;
        }
        
        prevRoll = roll = newRoll;
        prevPitch = pitch = newPitch;
        prevYaw = yaw = newYaw;
        attitudeTime = now;
    }
}

// ============================================================================
//                           RADIO UPDATE
// ============================================================================
void updateRadio() {
    while (radio.available()) {
        ControlPacket p;
        radio.read(&p, sizeof(p));
        
        if (p.isValid()) {
            rxPacket = p;
            lastPacketTime = millis();
            packetCount++;
            
            if (!radioConnected) {
                radioConnected = true;
                Serial.println(F("\n*** RF CONNECTED ***"));
                soundPaired();
            }
        }
    }
    
    // Timeout check
    if (radioConnected && (millis() - lastPacketTime > RF_TIMEOUT_MS)) {
        radioConnected = false;
        Serial.println(F("\n*** RF LOST ***"));
        
        if (flightState == STATE_ARMED) {
            flightState = STATE_FAILSAFE;
            beep(500, 800);
        }
    }
}

// ============================================================================
//                        MOTOR TEST SEQUENCE
// ============================================================================
bool motorTestActive = false;
uint32_t motorTestStart = 0;
bool prevMotorBtn = false;

void runMotorTest() {
    uint32_t t = millis() - motorTestStart;
    uint16_t speed = ESC_IDLE_US;
    
    escFL.writeMicroseconds(t < 400 ? speed : ESC_MIN_US);
    escFR.writeMicroseconds((t >= 400 && t < 800) ? speed : ESC_MIN_US);
    escRL.writeMicroseconds((t >= 800 && t < 1200) ? speed : ESC_MIN_US);
    escRR.writeMicroseconds((t >= 1200 && t < 1600) ? speed : ESC_MIN_US);
    
    if (t >= 1600 && t < 2000) {
        escFL.writeMicroseconds(speed);
        escFR.writeMicroseconds(speed);
        escRL.writeMicroseconds(speed);
        escRR.writeMicroseconds(speed);
    }
    
    if (t >= 2000) {
        motorTestActive = false;
        escFL.writeMicroseconds(ESC_MIN_US);
        escFR.writeMicroseconds(ESC_MIN_US);
        escRL.writeMicroseconds(ESC_MIN_US);
        escRR.writeMicroseconds(ESC_MIN_US);
        beep(200, 2500);
    }
}

// ============================================================================
//                        PROCESS COMMANDS
// ============================================================================
void processCommands() {
    if (!radioConnected) {
        throttleCmd = 0;
        rollCmd = pitchCmd = yawCmd = 0;
        return;
    }
    
    // Rate multiplier from pot (0.5 to 1.5)
    rateMultiplier = 0.5f + (rxPacket.ratePot / 255.0f);
    
    // Throttle
    throttleCmd = rxPacket.throttle;
    
    // Stick to angle/rate conversion with rate multiplier
    float rawRoll = (rxPacket.roll / 500.0f) * MAX_ROLL_ANGLE * rateMultiplier;
    float rawPitch = (rxPacket.pitch / 500.0f) * MAX_PITCH_ANGLE * rateMultiplier;
    float rawYaw = (rxPacket.yaw / 500.0f) * MAX_YAW_RATE * rateMultiplier;
    
    // Headless mode toggle
    bool headlessBtn = rxPacket.switches & (1 << SW_HEADLESS);
    if (headlessBtn && !prevHeadlessBtn) {
        headlessMode = !headlessMode;
        if (headlessMode) {
            headlessRef = yaw;
            Serial.println(F("HEADLESS ON"));
        } else {
            Serial.println(F("HEADLESS OFF"));
        }
        beep(100, headlessMode ? 2500 : 1500);
    }
    prevHeadlessBtn = headlessBtn;
    
    // Headless transformation
    if (headlessMode) {
        float yawDiff = (yaw - headlessRef) * 0.01745329f;
        float c = cos(yawDiff);
        float s = sin(yawDiff);
        rollCmd = rawRoll * c + rawPitch * s;
        pitchCmd = -rawRoll * s + rawPitch * c;
    } else {
        rollCmd = rawRoll;
        pitchCmd = rawPitch;
    }
    yawCmd = rawYaw;
    
    // Parse switches
    bool armSwitch = rxPacket.switches & (1 << SW_ARM);
    bool motorBtn = rxPacket.switches & (1 << SW_MOTORTEST);
    bool calibBtn = rxPacket.switches & (1 << SW_CALIBRATE);
    bool beeperBtn = rxPacket.switches & (1 << SW_BEEPER);
    
    // Beeper
    if (beeperBtn && !prevBeeperBtn) beep(500, 2500);
    else if (beeperBtn) {
        static uint32_t lastBeep = 0;
        if (millis() - lastBeep > 600) { beep(500, 2500); lastBeep = millis(); }
    }
    prevBeeperBtn = beeperBtn;
    
    // Motor test
    if ((motorBtn || calibBtn) && !prevMotorBtn && flightState == STATE_DISARMED && !motorTestActive) {
        motorTestActive = true;
        motorTestStart = millis();
        beep(100, 2000);
    }
    prevMotorBtn = motorBtn || calibBtn;
    
    if (motorTestActive) {
        runMotorTest();
        return;
    }
    
    // ARM/DISARM
    if (armSwitch && !prevArmSwitch && flightState == STATE_DISARMED) {
        if (throttleCmd <= ESC_ARM_THROTTLE && radioConnected && dmpReady) {
            flightState = STATE_ARMED;
            pidRoll.reset();
            pidPitch.reset();
            pidYaw.reset();
            headlessRef = yaw;
            motorFL = motorFR = motorRL = motorRR = ESC_MIN_US;
            Serial.println(F("*** ARMED ***"));
            soundArmed();
        } else {
            beep(200, 500);
        }
    } else if (!armSwitch && flightState == STATE_ARMED) {
        flightState = STATE_DISARMED;
        Serial.println(F("*** DISARMED ***"));
        soundDisarmed();
    }
    
    // Failsafe recovery
    if (flightState == STATE_FAILSAFE && (!armSwitch || (radioConnected && throttleCmd < 100))) {
        flightState = STATE_DISARMED;
        beep(200, 1500);
    }
    
    prevArmSwitch = armSwitch;
}

// ============================================================================
//                              PID UPDATE
// ============================================================================
void updatePID(float dt) {
    // Calculate PID outputs using gyro rates for D-term
    rollPID = pidRoll.compute(rollCmd, roll, rollRate, dt);
    pitchPID = pidPitch.compute(pitchCmd, pitch, pitchRate, dt);
    yawPID = pidYaw.compute(yawCmd, yawRate, yawRate, dt);  // Yaw is rate-based
}

// ============================================================================
//                           MOTOR MIXING
// ============================================================================
void updateMotors() {
    if (flightState != STATE_ARMED) {
        // Motors off
        motorFL = motorFR = motorRL = motorRR = ESC_MIN_US;
        escFL.writeMicroseconds(ESC_MIN_US);
        escFR.writeMicroseconds(ESC_MIN_US);
        escRL.writeMicroseconds(ESC_MIN_US);
        escRR.writeMicroseconds(ESC_MIN_US);
        return;
    }
    
    // Throttle to PWM (with idle)
    float baseThrottle = ESC_IDLE_US + (throttleCmd / 1000.0f) * (ESC_MAX_US - ESC_IDLE_US) - ESC_MIN_US;
    
    // Quad-X mixing:
    //   FL (CCW): - roll + pitch - yaw
    //   FR (CW):  + roll + pitch + yaw
    //   RL (CW):  - roll - pitch + yaw
    //   RR (CCW): + roll - pitch - yaw
    float targetFL = baseThrottle - rollPID + pitchPID - yawPID;
    float targetFR = baseThrottle + rollPID + pitchPID + yawPID;
    float targetRL = baseThrottle - rollPID - pitchPID + yawPID;
    float targetRR = baseThrottle + rollPID - pitchPID - yawPID;
    
    // Add ESC minimum and constrain
    targetFL = constrain(targetFL + ESC_MIN_US, ESC_IDLE_US, ESC_MAX_US);
    targetFR = constrain(targetFR + ESC_MIN_US, ESC_IDLE_US, ESC_MAX_US);
    targetRL = constrain(targetRL + ESC_MIN_US, ESC_IDLE_US, ESC_MAX_US);
    targetRR = constrain(targetRR + ESC_MIN_US, ESC_IDLE_US, ESC_MAX_US);
    
    // Smooth motor output (prevents sudden jumps)
    motorFL = motorFL * (1.0f - MOTOR_SMOOTH_ALPHA) + targetFL * MOTOR_SMOOTH_ALPHA;
    motorFR = motorFR * (1.0f - MOTOR_SMOOTH_ALPHA) + targetFR * MOTOR_SMOOTH_ALPHA;
    motorRL = motorRL * (1.0f - MOTOR_SMOOTH_ALPHA) + targetRL * MOTOR_SMOOTH_ALPHA;
    motorRR = motorRR * (1.0f - MOTOR_SMOOTH_ALPHA) + targetRR * MOTOR_SMOOTH_ALPHA;
    
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
    static bool state = false;
    uint16_t interval;
    
    switch (flightState) {
        case STATE_ARMED:    interval = headlessMode ? 150 : 0; break;
        case STATE_FAILSAFE: interval = 80; break;
        default:             interval = 500; break;
    }
    
    if (interval == 0) {
        digitalWrite(PIN_LED, HIGH);
    } else if (millis() - timeLED >= interval) {
        timeLED = millis();
        state = !state;
        digitalWrite(PIN_LED, state);
    }
}

// ============================================================================
//                           DEBUG OUTPUT
// ============================================================================
#if ENABLE_DEBUG
void printDebug() {
    // State
    switch (flightState) {
        case STATE_ARMED:    Serial.print(F("ARM ")); break;
        case STATE_FAILSAFE: Serial.print(F("FAIL")); break;
        default:             Serial.print(F("DIS ")); break;
    }
    
    // RF
    Serial.print(F("RF:")); Serial.print(radioConnected ? packetCount : 0);
    
    // Switches
    Serial.print(F(" SW:"));
    Serial.print((rxPacket.switches & 1) ? F("A") : F("-"));
    Serial.print(headlessMode ? F("L") : F("-"));
    
    // Throttle and rate
    Serial.print(F(" T:")); Serial.print(throttleCmd);
    Serial.print(F(" R%:")); Serial.print((int)(rateMultiplier * 100));
    
    // Attitude
    Serial.print(F(" R:")); Serial.print(roll, 1);
    Serial.print(F(" P:")); Serial.print(pitch, 1);
    Serial.print(F(" Y:")); Serial.print(yaw, 1);
    
    // PID output
    Serial.print(F(" PID:"));
    Serial.print((int)rollPID); Serial.print(F(","));
    Serial.print((int)pitchPID); Serial.print(F(","));
    Serial.print((int)yawPID);
    
    // Motors
    Serial.print(F(" M:"));
    Serial.print((int)motorFL); Serial.print(F(","));
    Serial.print((int)motorFR); Serial.print(F(","));
    Serial.print((int)motorRL); Serial.print(F(","));
    Serial.println((int)motorRR);
}
#endif

// ============================================================================
//                              SETUP
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("\n============================="));
    Serial.println(F("  QuadFC v4.0 - STABILITY"));
    Serial.println(F("  F330 / RS2205 2300KV / 3S"));
    Serial.println(F("============================="));
    
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    soundStartup();
    
    // ========== ESCs ==========
    Serial.println(F("Init ESCs..."));
    escFL.attach(PIN_MOTOR_FL, ESC_MIN_US, ESC_MAX_US);
    escFR.attach(PIN_MOTOR_FR, ESC_MIN_US, ESC_MAX_US);
    escRL.attach(PIN_MOTOR_RL, ESC_MIN_US, ESC_MAX_US);
    escRR.attach(PIN_MOTOR_RR, ESC_MIN_US, ESC_MAX_US);
    
    escFL.writeMicroseconds(ESC_MIN_US);
    escFR.writeMicroseconds(ESC_MIN_US);
    escRL.writeMicroseconds(ESC_MIN_US);
    escRR.writeMicroseconds(ESC_MIN_US);
    Serial.println(F("  OK"));
    
    // ========== NRF24L01 ==========
    Serial.println(F("Init NRF24L01..."));
    if (!radio.begin()) {
        Serial.println(F("  FAILED!"));
        while (1) { beepBlocking(200, 700); delay(300); }
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
    radio.openReadingPipe(1, radioAddress);
    radio.startListening();
    Serial.print(F("  OK (CH:")); Serial.print(RF_CHANNEL); Serial.println(F(")"));
    
    // ========== ESC Calibration Check ==========
    Serial.println(F("Hold D4 for ESC calibration..."));
    delay(500);
    
    bool escCalMode = false;
    uint32_t checkStart = millis();
    while (millis() - checkStart < 2000) {
        if (radio.available()) {
            ControlPacket p;
            radio.read(&p, sizeof(p));
            if (p.isValid() && (p.switches & (1 << SW_CALIBRATE))) {
                escCalMode = true;
                break;
            }
        }
        delay(50);
    }
    
    if (escCalMode) escCalibration();
    
    // ========== MPU6050 + DMP ==========
    Serial.println(F("Init MPU6050 DMP..."));
    Wire.begin();
    Wire.setClock(400000);
    
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println(F("  MPU6050 FAILED!"));
        while (1) { beepBlocking(200, 500); delay(300); }
    }
    
    dmpStatus = mpu.dmpInitialize();
    
    // Apply calibration offsets
    mpu.setXAccelOffset(ACCEL_OFFSET_X);
    mpu.setYAccelOffset(ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(ACCEL_OFFSET_Z);
    mpu.setXGyroOffset(GYRO_OFFSET_X);
    mpu.setYGyroOffset(GYRO_OFFSET_Y);
    mpu.setZGyroOffset(GYRO_OFFSET_Z);
    
    if (dmpStatus == 0) {
        Serial.print(F("  Calibrating..."));
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        Serial.println(F("OK"));
    } else {
        Serial.print(F("  DMP FAILED ("));
        Serial.print(dmpStatus);
        Serial.println(F(")"));
        while (1) { beepBlocking(200, 600); delay(300); }
    }
    
    // ========== READY ==========
    Serial.println(F("\n============================="));
    Serial.println(F("         READY"));
    Serial.println(F("============================="));
    Serial.println(F("PID (Roll/Pitch):"));
    Serial.print(F("  P=")); Serial.print(PID_ROLL_KP);
    Serial.print(F(" I=")); Serial.print(PID_ROLL_KI);
    Serial.print(F(" D=")); Serial.println(PID_ROLL_KD);
    Serial.println(F("Controls:"));
    Serial.println(F("  D2=Arm D6=Beeper D7=Headless"));
    Serial.println(F("  A6=Rate A7=Expo"));
    Serial.println(F("=============================\n"));
    
    soundReady();
    
    timePID = micros();
    timeDebug = timeLED = millis();
}

// ============================================================================
//                             MAIN LOOP
// ============================================================================
void loop() {
    uint32_t now = micros();
    
    // Radio check - every loop
    updateRadio();
    
    // DMP update - as fast as available
    updateDMP();
    
    // PID + Motors at 250Hz
    if (now - timePID >= PID_LOOP_US) {
        float dt = (now - timePID) / 1000000.0f;
        timePID = now;
        
        processCommands();
        
        if (!motorTestActive) {
            updatePID(dt);
            updateMotors();
        }
    }
    
    // LED
    updateLED();
    
    // Debug at 5Hz
#if ENABLE_DEBUG
    if (millis() - timeDebug >= 200) {
        timeDebug = millis();
        printDebug();
    }
#endif
}
