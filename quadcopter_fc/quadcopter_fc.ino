/**
 * ============================================================================
 *                    QUADCOPTER FLIGHT CONTROLLER v3.3
 * ============================================================================
 * 
 * NEW FEATURES:
 *   - BEEPER button (D6 on RC) - makes drone beep to find it
 *   - HEADLESS mode (D7 on RC) - controls relative to pilot, not drone
 *   - RATE pot (A6 on RC) - adjusts sensitivity 50%-150%
 *   - ESC Calibration mode
 * 
 * ============================================================================
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <SPI.h>
#include <RF24.h>
#include <MS5611.h>
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
#define MAX_ROLL_ANGLE      45
#define MAX_PITCH_ANGLE     45
#define MAX_YAW_RATE        180

#define ESC_MIN_US          1000
#define ESC_MAX_US          2000
#define ESC_IDLE_US         1100
#define ESC_ARM_THROTTLE    50

#define RF_TIMEOUT_MS       500
#define MOTOR_RATE_LIMIT    30
#define ALT_PID_MAX         100
#define ALT_RATE_LIMIT      20
#define ALT_GROUND_THRESH   0.30
#define THR_GROUND_THRESH   200

// ============================================================================
//                         MPU6050 CALIBRATION
// ============================================================================
#define ACCEL_OFFSET_X      -2366
#define ACCEL_OFFSET_Y      755
#define ACCEL_OFFSET_Z      -2006
#define GYRO_OFFSET_X       10
#define GYRO_OFFSET_Y       21
#define GYRO_OFFSET_Z       -19

// ============================================================================
//                           PID TUNING
// ============================================================================
#define PID_ROLL_KP         3.0
#define PID_ROLL_KI         0.01
#define PID_ROLL_KD         1.0

#define PID_PITCH_KP        3.0
#define PID_PITCH_KI        0.01
#define PID_PITCH_KD        1.0

#define PID_YAW_KP          2.0
#define PID_YAW_KI          0.01
#define PID_YAW_KD          0.0

#define PID_ALT_KP          10.0
#define PID_ALT_KI          0.05
#define PID_ALT_KD          5.0

// ============================================================================
//                          RF PACKET STRUCTURE
// ============================================================================
struct __attribute__((packed)) ControlPacket {
    uint16_t throttle;
    int16_t  yaw;
    int16_t  pitch;
    int16_t  roll;
    uint8_t  switches;
    uint8_t  checksum;
    uint32_t sequence;
    uint8_t  ratePot;       // Rate adjustment from RC
    uint8_t  expoPot;       // Expo (used on RC side)
    
    bool isValid() const {
        const uint8_t* data = (const uint8_t*)this;
        uint8_t calc = 0;
        for (uint8_t i = 0; i < 9; i++) calc ^= data[i];
        return calc == checksum;
    }
};

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
MS5611 baro(0x77);
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
Servo escFL, escFR, escRL, escRR;

// ============================================================================
//                             DMP VARIABLES
// ============================================================================
bool dmpReady = false;
uint8_t dmpStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
Quaternion quaternion;
VectorFloat gravity;
float ypr[3];

float roll = 0, pitch = 0, yaw = 0;
float yawRate = 0, prevYaw = 0;
uint32_t yawTime = 0;

// Headless mode reference
float headlessYawRef = 0;
bool headlessMode = false;

// ============================================================================
//                          BAROMETER VARIABLES
// ============================================================================
bool baroReady = false;
float basePressure = 0;
float altitude = 0, altitudeFiltered = 0, altitudePrev = 0;
float vertVel = 0, vertVelFiltered = 0;
uint32_t baroTime = 0;

// ============================================================================
//                            RADIO VARIABLES
// ============================================================================
ControlPacket rxPacket;
uint32_t lastValidPacketTime = 0;
uint32_t packetCount = 0;
bool radioConnected = false;
const uint8_t radioAddress[6] = "QUAD1";

// Rate multiplier from pot (0.5 to 1.5)
float rateMultiplier = 1.0f;

// ============================================================================
//                           FLIGHT STATE
// ============================================================================
enum FlightState { 
    STATE_DISARMED, 
    STATE_ARMED, 
    STATE_FAILSAFE,
    STATE_ESC_CALIB
};
FlightState flightState = STATE_DISARMED;

int16_t throttleCmd = 0;
float rollCmd = 0, pitchCmd = 0, yawCmd = 0;
bool altHoldSwitch = false, altHoldActive = false;
float targetAltitude = 0;

float rollOutput = 0, pitchOutput = 0, yawOutput = 0;
float altOutput = 0, altOutputPrev = 0;

uint16_t motorFL = ESC_MIN_US, motorFR = ESC_MIN_US;
uint16_t motorRL = ESC_MIN_US, motorRR = ESC_MIN_US;
uint16_t motorFL_prev = ESC_MIN_US, motorFR_prev = ESC_MIN_US;
uint16_t motorRL_prev = ESC_MIN_US, motorRR_prev = ESC_MIN_US;

float motorFL_smooth = ESC_MIN_US, motorFR_smooth = ESC_MIN_US;
float motorRL_smooth = ESC_MIN_US, motorRR_smooth = ESC_MIN_US;

bool prevArmSwitch = false;
bool prevHeadlessSwitch = false;

uint32_t timePID = 0, timeBaro = 0;
uint32_t timeDebug = 0, timeLED = 0;

// ============================================================================
//                             PID CONTROLLER
// ============================================================================
class PIDController {
public:
    float Kp, Ki, Kd;
    float integral, prevMeasurement, prevDerivative;
    float outputMin, outputMax, integralMax;
    bool initialized;
    
    PIDController(float p, float i, float d) : 
        Kp(p), Ki(i), Kd(d), integral(0), prevMeasurement(0), prevDerivative(0),
        outputMin(-500), outputMax(500), integralMax(200), initialized(false) {}
    
    void reset() {
        integral = 0;
        prevMeasurement = 0;
        prevDerivative = 0;
        initialized = false;
    }
    
    float calculate(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;
        
        if (!initialized) {
            prevMeasurement = measurement;
            initialized = true;
            return 0;
        }
        
        float pTerm = Kp * error;
        integral += Ki * error * dt;
        integral = constrain(integral, -integralMax, integralMax);
        
        float derivative = -(measurement - prevMeasurement) / dt;
        derivative = prevDerivative * 0.7f + derivative * 0.3f;
        float dTerm = Kd * derivative;
        
        prevMeasurement = measurement;
        prevDerivative = derivative;
        
        return constrain(pTerm + integral + dTerm, outputMin, outputMax);
    }
};

PIDController pidRoll(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
PIDController pidPitch(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
PIDController pidYaw(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD);
PIDController pidAlt(PID_ALT_KP, PID_ALT_KI, PID_ALT_KD);

// ============================================================================
//                            BUZZER FUNCTIONS
// ============================================================================
void beep(uint16_t duration, uint16_t freq = 2000) {
    tone(PIN_BUZZER, freq, duration);
}

void beepBlocking(uint16_t duration, uint16_t freq = 2000) {
    tone(PIN_BUZZER, freq);
    delay(duration);
    noTone(PIN_BUZZER);
}

void soundStartup() {
    beepBlocking(100, 1500); delay(50);
    beepBlocking(100, 2000); delay(50);
    beepBlocking(200, 2500);
}

void soundArmed() {
    beepBlocking(100, 2000); delay(100);
    beepBlocking(200, 2500);
}

void soundDisarmed() {
    beepBlocking(300, 1500);
}

void soundPaired() {
    for (int i = 0; i < 5; i++) {
        beepBlocking(80, 2500);
        delay(80);
    }
}

void soundFailsafe() {
    beep(500, 800);
}

void soundReady() {
    beepBlocking(100, 2000); delay(100);
    beepBlocking(100, 2500); delay(100);
    beepBlocking(200, 3000);
}

// ============================================================================
//                        ESC CALIBRATION MODE
// ============================================================================
void escCalibrationMode() {
    Serial.println(F("\n*** ESC CALIBRATION ***"));
    Serial.println(F("Setting MAX throttle..."));
    
    escFL.writeMicroseconds(ESC_MAX_US);
    escFR.writeMicroseconds(ESC_MAX_US);
    escRL.writeMicroseconds(ESC_MAX_US);
    escRR.writeMicroseconds(ESC_MAX_US);
    
    beepBlocking(1000, 3000);
    
    Serial.println(F("Release button when ESCs beep..."));
    
    uint32_t start = millis();
    while (millis() - start < 10000) {
        if (radio.available()) {
            ControlPacket pkt;
            radio.read(&pkt, sizeof(pkt));
            if (pkt.isValid() && !(pkt.switches & (1 << SW_CALIBRATE))) {
                break;
            }
        }
        delay(50);
    }
    
    Serial.println(F("Setting MIN throttle..."));
    
    escFL.writeMicroseconds(ESC_MIN_US);
    escFR.writeMicroseconds(ESC_MIN_US);
    escRL.writeMicroseconds(ESC_MIN_US);
    escRR.writeMicroseconds(ESC_MIN_US);
    
    beepBlocking(500, 1500);
    delay(2000);
    
    Serial.println(F("ESC Calibration DONE!"));
    beepBlocking(200, 2000);
    beepBlocking(200, 2500);
    beepBlocking(400, 3000);
}

// ============================================================================
//                           DMP UPDATE
// ============================================================================
bool updateDMP() {
    if (!dmpReady) return false;
    
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &quaternion);
        mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
        
        float newYaw = ypr[0] * 57.2958f;
        pitch = ypr[1] * 57.2958f;
        roll = ypr[2] * 57.2958f;
        
        uint32_t now = micros();
        if (yawTime > 0) {
            float dt = (now - yawTime) / 1000000.0f;
            if (dt > 0 && dt < 0.1f) {
                float deltaYaw = newYaw - prevYaw;
                if (deltaYaw > 180) deltaYaw -= 360;
                if (deltaYaw < -180) deltaYaw += 360;
                yawRate = deltaYaw / dt;
            }
        }
        prevYaw = newYaw;
        yaw = newYaw;
        yawTime = now;
        
        return true;
    }
    return false;
}

// ============================================================================
//                         BAROMETER UPDATE
// ============================================================================
void updateBarometer() {
    if (!baroReady) return;
    
    baro.read();
    float pressure = baro.getPressure();
    if (pressure < 300 || pressure > 1200) return;
    
    float rawAlt = 44330.0f * (1.0f - pow(pressure / basePressure, 0.1903f));
    float change = constrain(rawAlt - altitude, -0.15f, 0.15f);
    altitude += change;
    
    altitudeFiltered = 0.92f * altitudeFiltered + 0.08f * altitude;
    
    uint32_t now = micros();
    float dt = (now - baroTime) / 1000000.0f;
    
    if (baroTime > 0 && dt > 0.01f && dt < 0.1f) {
        float rawVel = constrain((altitudeFiltered - altitudePrev) / dt, -2.0f, 2.0f);
        vertVelFiltered = 0.9f * vertVelFiltered + 0.1f * rawVel;
        vertVel = vertVelFiltered;
    }
    
    altitudePrev = altitudeFiltered;
    baroTime = now;
}

// ============================================================================
//                           RADIO UPDATE
// ============================================================================
void updateRadio() {
    while (radio.available()) {
        ControlPacket packet;
        radio.read(&packet, sizeof(packet));
        
        if (packet.isValid()) {
            rxPacket = packet;
            lastValidPacketTime = millis();
            packetCount++;
            
            if (!radioConnected) {
                radioConnected = true;
                Serial.println(F("\n*** RF CONNECTED! ***"));
                soundPaired();
            }
        }
    }
    
    if (radioConnected && (millis() - lastValidPacketTime > RF_TIMEOUT_MS)) {
        radioConnected = false;
        Serial.println(F("\n*** RF LOST! ***"));
        
        if (flightState == STATE_ARMED) {
            flightState = STATE_FAILSAFE;
            soundFailsafe();
        }
    }
}

// ============================================================================
//                        COMMAND PROCESSING
// ============================================================================
bool motorTestActive = false;
bool prevMotorTestBtn = false;
uint32_t motorTestStart = 0;

void runMotorTest() {
    uint32_t elapsed = millis() - motorTestStart;
    uint16_t testSpeed = ESC_IDLE_US;
    
    if (elapsed < 500) {
        escFL.writeMicroseconds(testSpeed); escFR.writeMicroseconds(ESC_MIN_US);
        escRL.writeMicroseconds(ESC_MIN_US); escRR.writeMicroseconds(ESC_MIN_US);
    } else if (elapsed < 1000) {
        escFL.writeMicroseconds(ESC_MIN_US); escFR.writeMicroseconds(testSpeed);
        escRL.writeMicroseconds(ESC_MIN_US); escRR.writeMicroseconds(ESC_MIN_US);
    } else if (elapsed < 1500) {
        escFL.writeMicroseconds(ESC_MIN_US); escFR.writeMicroseconds(ESC_MIN_US);
        escRL.writeMicroseconds(testSpeed); escRR.writeMicroseconds(ESC_MIN_US);
    } else if (elapsed < 2000) {
        escFL.writeMicroseconds(ESC_MIN_US); escFR.writeMicroseconds(ESC_MIN_US);
        escRL.writeMicroseconds(ESC_MIN_US); escRR.writeMicroseconds(testSpeed);
    } else if (elapsed < 2500) {
        escFL.writeMicroseconds(testSpeed); escFR.writeMicroseconds(testSpeed);
        escRL.writeMicroseconds(testSpeed); escRR.writeMicroseconds(testSpeed);
    } else {
        motorTestActive = false;
        escFL.writeMicroseconds(ESC_MIN_US); escFR.writeMicroseconds(ESC_MIN_US);
        escRL.writeMicroseconds(ESC_MIN_US); escRR.writeMicroseconds(ESC_MIN_US);
        beep(200, 2500);
    }
}

void processCommands() {
    if (!radioConnected) {
        throttleCmd = 0;
        rollCmd = pitchCmd = yawCmd = 0;
        return;
    }
    
    // Update rate multiplier from pot (0-255 -> 0.5-1.5)
    rateMultiplier = 0.5f + (rxPacket.ratePot / 255.0f);
    
    throttleCmd = rxPacket.throttle;
    
    // Get raw commands
    float rawRoll = map(rxPacket.roll, -500, 500, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
    float rawPitch = map(rxPacket.pitch, -500, 500, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
    float rawYaw = map(rxPacket.yaw, -500, 500, -MAX_YAW_RATE, MAX_YAW_RATE);
    
    // Apply rate multiplier
    rawRoll *= rateMultiplier;
    rawPitch *= rateMultiplier;
    rawYaw *= rateMultiplier;
    
    // HEADLESS MODE: Transform commands based on initial heading
    bool headlessSwitch = rxPacket.switches & (1 << SW_HEADLESS);
    
    // Detect headless mode toggle
    if (headlessSwitch && !prevHeadlessSwitch) {
        headlessMode = !headlessMode;
        if (headlessMode) {
            headlessYawRef = yaw;  // Save current heading as reference
            Serial.println(F("HEADLESS ON"));
            beep(100, 2500);
        } else {
            Serial.println(F("HEADLESS OFF"));
            beep(100, 1500);
        }
    }
    prevHeadlessSwitch = headlessSwitch;
    
    if (headlessMode) {
        // Transform roll/pitch based on yaw difference from reference
        float yawDiff = (yaw - headlessYawRef) * 0.01745329f;  // to radians
        float cosYaw = cos(yawDiff);
        float sinYaw = sin(yawDiff);
        
        rollCmd = rawRoll * cosYaw + rawPitch * sinYaw;
        pitchCmd = -rawRoll * sinYaw + rawPitch * cosYaw;
    } else {
        rollCmd = rawRoll;
        pitchCmd = rawPitch;
    }
    yawCmd = rawYaw;
    
    // Parse switches
    bool armSwitch = rxPacket.switches & (1 << SW_ARM);
    bool motorTestBtn = rxPacket.switches & (1 << SW_MOTORTEST);
    bool calibBtn = rxPacket.switches & (1 << SW_CALIBRATE);
    bool beeperBtn = rxPacket.switches & (1 << SW_BEEPER);
    altHoldSwitch = rxPacket.switches & (1 << SW_ALTHOLD);
    
    // BEEPER: Make noise when button pressed (to find drone)
    static bool prevBeeperBtn = false;
    if (beeperBtn && !prevBeeperBtn) {
        beep(500, 2500);
    } else if (beeperBtn) {
        // Continuous beep while held
        static uint32_t lastBeep = 0;
        if (millis() - lastBeep > 600) {
            beep(500, 2500);
            lastBeep = millis();
        }
    }
    prevBeeperBtn = beeperBtn;
    
    // Motor test
    if ((calibBtn || motorTestBtn) && !prevMotorTestBtn && flightState == STATE_DISARMED && !motorTestActive) {
        motorTestActive = true;
        motorTestStart = millis();
        beep(100, 2000);
    }
    prevMotorTestBtn = calibBtn || motorTestBtn;
    
    if (motorTestActive) {
        runMotorTest();
        return;
    }
    
    // Arm logic
    if (armSwitch && !prevArmSwitch && flightState == STATE_DISARMED) {
        if (throttleCmd <= ESC_ARM_THROTTLE && radioConnected && dmpReady) {
            flightState = STATE_ARMED;
            pidRoll.reset(); pidPitch.reset(); pidYaw.reset(); pidAlt.reset();
            altOutputPrev = 0;
            targetAltitude = altitudeFiltered;
            motorFL_smooth = motorFR_smooth = motorRL_smooth = motorRR_smooth = ESC_MIN_US;
            headlessYawRef = yaw;  // Set headless reference on arm
            Serial.println(F("*** ARMED ***"));
            soundArmed();
        } else {
            beep(200, 500);
        }
    } 
    else if (!armSwitch && flightState == STATE_ARMED) {
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
//                            PID UPDATE
// ============================================================================
void updatePID(float dt) {
    rollOutput = pidRoll.calculate(rollCmd, roll, dt);
    pitchOutput = pidPitch.calculate(pitchCmd, pitch, dt);
    yawOutput = pidYaw.calculate(yawCmd, yawRate, dt);
    
    if (flightState != STATE_ARMED) {
        altOutput = altOutputPrev = 0;
        return;
    }
    
    bool onGround = (throttleCmd < THR_GROUND_THRESH) || 
                    (abs(altitudeFiltered) < ALT_GROUND_THRESH && abs(vertVel) < 0.5f);
    altHoldActive = altHoldSwitch && !onGround && baroReady;
    
    if (altHoldActive) {
        float thrDev = throttleCmd - 500;
        if (abs(thrDev) > 50) {
            targetAltitude += (thrDev / 500.0f) * 0.005f;
        }
        
        float rawAlt = pidAlt.calculate(targetAltitude, altitudeFiltered, dt);
        rawAlt = constrain(rawAlt, -ALT_PID_MAX, ALT_PID_MAX);
        
        float altChange = constrain(rawAlt - altOutputPrev, -ALT_RATE_LIMIT, ALT_RATE_LIMIT);
        altOutput = altOutputPrev + altChange;
        altOutputPrev = altOutput;
    } else {
        altOutput = altOutputPrev = 0;
        pidAlt.reset();
        if (altHoldSwitch) targetAltitude = altitudeFiltered;
    }
}

// ============================================================================
//                           MOTOR MIXING
// ============================================================================
void updateMotors() {
    if (flightState != STATE_ARMED) {
        motorFL = motorFR = motorRL = motorRR = ESC_MIN_US;
        motorFL_smooth = motorFR_smooth = motorRL_smooth = motorRR_smooth = ESC_MIN_US;
        motorFL_prev = motorFR_prev = motorRL_prev = motorRR_prev = ESC_MIN_US;
        
        escFL.writeMicroseconds(ESC_MIN_US);
        escFR.writeMicroseconds(ESC_MIN_US);
        escRL.writeMicroseconds(ESC_MIN_US);
        escRR.writeMicroseconds(ESC_MIN_US);
        return;
    }
    
    int16_t baseThrottle = map(throttleCmd, 0, 1000, ESC_IDLE_US, ESC_MAX_US) - ESC_MIN_US;
    if (altHoldActive) baseThrottle += (int16_t)altOutput;
    
    int16_t fl = baseThrottle - (int16_t)rollOutput + (int16_t)pitchOutput - (int16_t)yawOutput;
    int16_t fr = baseThrottle + (int16_t)rollOutput + (int16_t)pitchOutput + (int16_t)yawOutput;
    int16_t rl = baseThrottle - (int16_t)rollOutput - (int16_t)pitchOutput + (int16_t)yawOutput;
    int16_t rr = baseThrottle + (int16_t)rollOutput - (int16_t)pitchOutput - (int16_t)yawOutput;
    
    uint16_t targetFL = constrain(fl + ESC_MIN_US, ESC_IDLE_US, ESC_MAX_US);
    uint16_t targetFR = constrain(fr + ESC_MIN_US, ESC_IDLE_US, ESC_MAX_US);
    uint16_t targetRL = constrain(rl + ESC_MIN_US, ESC_IDLE_US, ESC_MAX_US);
    uint16_t targetRR = constrain(rr + ESC_MIN_US, ESC_IDLE_US, ESC_MAX_US);
    
    float alpha = 0.3f;
    motorFL_smooth = motorFL_smooth * (1.0f - alpha) + targetFL * alpha;
    motorFR_smooth = motorFR_smooth * (1.0f - alpha) + targetFR * alpha;
    motorRL_smooth = motorRL_smooth * (1.0f - alpha) + targetRL * alpha;
    motorRR_smooth = motorRR_smooth * (1.0f - alpha) + targetRR * alpha;
    
    int16_t changeFL = constrain((int16_t)motorFL_smooth - (int16_t)motorFL_prev, -MOTOR_RATE_LIMIT, MOTOR_RATE_LIMIT);
    int16_t changeFR = constrain((int16_t)motorFR_smooth - (int16_t)motorFR_prev, -MOTOR_RATE_LIMIT, MOTOR_RATE_LIMIT);
    int16_t changeRL = constrain((int16_t)motorRL_smooth - (int16_t)motorRL_prev, -MOTOR_RATE_LIMIT, MOTOR_RATE_LIMIT);
    int16_t changeRR = constrain((int16_t)motorRR_smooth - (int16_t)motorRR_prev, -MOTOR_RATE_LIMIT, MOTOR_RATE_LIMIT);
    
    motorFL = motorFL_prev + changeFL; motorFL_prev = motorFL;
    motorFR = motorFR_prev + changeFR; motorFR_prev = motorFR;
    motorRL = motorRL_prev + changeRL; motorRL_prev = motorRL;
    motorRR = motorRR_prev + changeRR; motorRR_prev = motorRR;
    
    escFL.writeMicroseconds(motorFL);
    escFR.writeMicroseconds(motorFR);
    escRL.writeMicroseconds(motorRL);
    escRR.writeMicroseconds(motorRR);
}

// ============================================================================
//                            LED UPDATE
// ============================================================================
void updateLED() {
    static bool ledState = false;
    uint16_t interval = (flightState == STATE_ARMED) ? 0 : 
                        (flightState == STATE_FAILSAFE) ? 100 : 500;
    
    // Fast blink in headless mode
    if (headlessMode && flightState == STATE_ARMED) {
        interval = 200;
    }
    
    if (interval == 0) {
        digitalWrite(PIN_LED, HIGH);
    } else if (millis() - timeLED >= interval) {
        timeLED = millis();
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState);
    }
}

// ============================================================================
//                           DEBUG OUTPUT
// ============================================================================
#if ENABLE_DEBUG
void printDebug() {
    switch (flightState) {
        case STATE_ARMED:    Serial.print(F("ARM ")); break;
        case STATE_FAILSAFE: Serial.print(F("FAIL")); break;
        default:             Serial.print(F("DIS ")); break;
    }
    
    Serial.print(F(" RF:")); Serial.print(radioConnected ? packetCount : 0);
    
    // Switches
    Serial.print(F(" SW:"));
    Serial.print((rxPacket.switches & (1<<SW_ARM)) ? F("A") : F("-"));
    Serial.print((rxPacket.switches & (1<<SW_ALTHOLD)) ? F("H") : F("-"));
    Serial.print((rxPacket.switches & (1<<SW_BEEPER)) ? F("B") : F("-"));
    Serial.print(headlessMode ? F("L") : F("-"));
    
    Serial.print(F(" T:")); Serial.print(throttleCmd);
    Serial.print(F(" R:")); Serial.print(roll, 1);
    Serial.print(F(" P:")); Serial.print(pitch, 1);
    
    Serial.print(F(" Rate:")); Serial.print((int)(rateMultiplier * 100)); Serial.print(F("%"));
    
    Serial.print(F(" M:"));
    Serial.print(motorFL); Serial.print(F(","));
    Serial.print(motorFR); Serial.print(F(","));
    Serial.print(motorRL); Serial.print(F(","));
    Serial.println(motorRR);
}
#endif

// ============================================================================
//                              SETUP
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("\n=== QuadFC v3.3 ==="));
    Serial.print(F("RF Channel: ")); Serial.println(RF_CHANNEL);
    
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    soundStartup();
    
    // ESCs
    Serial.println(F("Init ESCs..."));
    escFL.attach(PIN_MOTOR_FL, ESC_MIN_US, ESC_MAX_US);
    escFR.attach(PIN_MOTOR_FR, ESC_MIN_US, ESC_MAX_US);
    escRL.attach(PIN_MOTOR_RL, ESC_MIN_US, ESC_MAX_US);
    escRR.attach(PIN_MOTOR_RR, ESC_MIN_US, ESC_MAX_US);
    escFL.writeMicroseconds(ESC_MIN_US);
    escFR.writeMicroseconds(ESC_MIN_US);
    escRL.writeMicroseconds(ESC_MIN_US);
    escRR.writeMicroseconds(ESC_MIN_US);
    
    // NRF24L01
    Serial.println(F("Init NRF24L01..."));
    if (!radio.begin()) {
        Serial.println(F("NRF24 FAIL!"));
        while (1) { beepBlocking(200, 700); delay(300); }
    }
    
    radio.flush_rx(); radio.flush_tx();
    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(16);
    radio.setAutoAck(false);
    radio.disableDynamicPayloads();
    radio.setCRCLength(RF24_CRC_16);
    radio.openReadingPipe(1, radioAddress);
    radio.startListening();
    Serial.println(F("NRF24 OK"));
    
    // Check ESC calibration
    Serial.println(F("Hold D4 for ESC calibration..."));
    delay(1000);
    
    bool calibMode = false;
    uint32_t checkStart = millis();
    while (millis() - checkStart < 2000) {
        if (radio.available()) {
            ControlPacket pkt;
            radio.read(&pkt, sizeof(pkt));
            if (pkt.isValid() && (pkt.switches & (1 << SW_CALIBRATE))) {
                calibMode = true;
                break;
            }
        }
        delay(50);
    }
    
    if (calibMode) {
        escCalibrationMode();
    }
    
    // MPU6050
    Serial.print(F("Init MPU6050..."));
    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println(F("FAIL"));
        while (1) { beepBlocking(200, 500); delay(300); }
    }
    
    dmpStatus = mpu.dmpInitialize();
    mpu.setXAccelOffset(ACCEL_OFFSET_X);
    mpu.setYAccelOffset(ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(ACCEL_OFFSET_Z);
    mpu.setXGyroOffset(GYRO_OFFSET_X);
    mpu.setYGyroOffset(GYRO_OFFSET_Y);
    mpu.setZGyroOffset(GYRO_OFFSET_Z);
    
    if (dmpStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        Serial.println(F("OK"));
    } else {
        Serial.println(F("FAIL"));
        while (1) { beepBlocking(200, 600); delay(300); }
    }
    
    // MS5611
    Serial.print(F("Init MS5611..."));
    if (baro.begin()) {
        float sum = 0;
        for (int i = 0; i < 10; i++) {
            baro.read(); sum += baro.getPressure(); delay(50);
        }
        basePressure = sum / 10.0f;
        baroReady = true;
        Serial.println(F("OK"));
    } else {
        Serial.println(F("FAIL"));
    }
    
    pidAlt.outputMin = -ALT_PID_MAX;
    pidAlt.outputMax = ALT_PID_MAX;
    pidAlt.integralMax = 30;
    
    Serial.println(F("\n*** READY ***"));
    Serial.println(F("Controls:"));
    Serial.println(F("  D6 = Beeper (find drone)"));
    Serial.println(F("  D7 = Headless mode"));
    Serial.println(F("  A6 = Rate (50-150%)"));
    Serial.println(F("  A7 = Expo curve"));
    Serial.println();
    
    soundReady();
    
    timePID = timeBaro = micros();
    timeDebug = timeLED = millis();
}

// ============================================================================
//                             MAIN LOOP
// ============================================================================
void loop() {
    uint32_t now = micros();
    
    updateRadio();
    updateDMP();
    
    static uint32_t timeCmd = 0;
    if (now - timeCmd >= 20000) {
        timeCmd = now;
        processCommands();
    }
    
    if (now - timeBaro >= 25000) {
        timeBaro = now;
        updateBarometer();
    }
    
    if (now - timePID >= 4000) {
        float dt = (now - timePID) / 1000000.0f;
        timePID = now;
        updatePID(dt);
        updateMotors();
    }
    
    updateLED();
    
#if ENABLE_DEBUG
    if (millis() - timeDebug >= 200) {
        timeDebug = millis();
        printDebug();
    }
#endif
}
