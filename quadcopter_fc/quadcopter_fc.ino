/**
 * ============================================================================
 *                    QUADCOPTER FLIGHT CONTROLLER v3.0
 * ============================================================================
 * 
 * Features:
 *   ✓ MPU6050 with DMP (hardware sensor fusion)
 *   ✓ MS5611 barometer (altitude hold)
 *   ✓ NRF24L01 with ACK (reliable communication)
 *   ✓ Auto-level (angle mode)
 *   ✓ Auto-disarm on signal loss
 *   ✓ Filtered altitude with rate limiting
 *   ✓ Motor output smoothing
 * 
 * Hardware:
 *   MCU: Arduino Nano (ATmega328P)
 *   IMU: MPU6050 (I2C 0x68)
 *   Baro: MS5611 (I2C 0x77)
 *   Radio: NRF24L01+ (SPI)
 *   Motors: 4x ESC (PWM 1000-2000µs)
 * 
 * Libraries Required:
 *   - I2Cdev (Jeff Rowberg): github.com/jrowberg/i2cdevlib
 *   - MPU6050 (Jeff Rowberg): github.com/jrowberg/i2cdevlib
 *   - RF24 (TMRh20): github.com/nRF24/RF24
 *   - MS5611 (Rob Tillaart): github.com/RobTillaart/MS5611
 * 
 * Pin Configuration:
 *   D2  - MPU6050 INT (optional)
 *   D3  - Motor Front-Left (CCW)
 *   D4  - NRF24 CE
 *   D5  - Motor Front-Right (CW)
 *   D6  - Motor Rear-Left (CW)
 *   D7  - Status LED
 *   D8  - Buzzer
 *   D9  - Motor Rear-Right (CCW)
 *   D10 - NRF24 CSN
 *   D11 - NRF24 MOSI
 *   D12 - NRF24 MISO
 *   D13 - NRF24 SCK
 *   A4  - I2C SDA (MPU6050, MS5611)
 *   A5  - I2C SCL (MPU6050, MS5611)
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

// RF Channel (0-125) - MUST MATCH REMOTE CONTROLLER!
#define RF_CHANNEL          108

// Serial debug (set to 0 to save ~2KB flash if needed)
#define ENABLE_DEBUG        1
#define SERIAL_BAUD         115200

// ============================================================================
//                            PIN DEFINITIONS
// ============================================================================

#define PIN_MOTOR_FL        3       // Front-Left motor (CCW)
#define PIN_MOTOR_FR        5       // Front-Right motor (CW)
#define PIN_MOTOR_RL        6       // Rear-Left motor (CW)
#define PIN_MOTOR_RR        9       // Rear-Right motor (CCW)
#define PIN_RF_CE           4       // NRF24 Chip Enable
#define PIN_RF_CSN          10      // NRF24 Chip Select
#define PIN_LED             7       // Status LED
#define PIN_BUZZER          8       // Buzzer

// ============================================================================
//                          FLIGHT PARAMETERS
// ============================================================================

// Angle limits
#define MAX_ROLL_ANGLE      45      // Max roll angle (degrees)
#define MAX_PITCH_ANGLE     45      // Max pitch angle (degrees)
#define MAX_YAW_RATE        180     // Max yaw rate (deg/sec)

// ESC configuration
#define ESC_MIN_US          1000    // Minimum pulse width
#define ESC_MAX_US          2000    // Maximum pulse width
#define ESC_IDLE_US         1150    // Idle speed when armed
#define ESC_ARM_THROTTLE    50      // Max throttle to allow arming

// Safety
#define RF_TIMEOUT_MS       500     // Disarm after this many ms without signal
#define MOTOR_RATE_LIMIT    100     // Max motor change per cycle (µs)

// Altitude hold
#define ALT_PID_MAX         150     // Max altitude PID output (µs)
#define ALT_RATE_LIMIT      50      // Max alt PID change per cycle
#define ALT_GROUND_THRESH   0.30    // Below this = on ground (m)
#define THR_GROUND_THRESH   200     // Below this throttle = on ground

// ============================================================================
//                         MPU6050 CALIBRATION
// ============================================================================
// Run IMU_Zero example from MPU6050 library to get these values for YOUR sensor

#define ACCEL_OFFSET_X      -2366
#define ACCEL_OFFSET_Y      755
#define ACCEL_OFFSET_Z      -2006
#define GYRO_OFFSET_X       10
#define GYRO_OFFSET_Y       21
#define GYRO_OFFSET_Z       -19

// ============================================================================
//                           PID TUNING
// ============================================================================
// Adjust these values to tune flight characteristics
//
// PROBLEM          -> FIX
// ----------------    ---------------------
// Oscillation      -> Reduce Kp or increase Kd
// Sluggish         -> Increase Kp
// Drifting         -> Increase Ki
// Overshoots       -> Increase Kd
// Fast vibration   -> Reduce Kd

//                          Kp      Ki      Kd
#define PID_ROLL_KP         6.0
#define PID_ROLL_KI         0.03
#define PID_ROLL_KD         2.5

#define PID_PITCH_KP        6.0
#define PID_PITCH_KI        0.03
#define PID_PITCH_KD        2.5

#define PID_YAW_KP          4.0
#define PID_YAW_KI          0.02
#define PID_YAW_KD          0.0

#define PID_ALT_KP          15.0
#define PID_ALT_KI          0.1
#define PID_ALT_KD          8.0

// ============================================================================
//                          RF PACKET STRUCTURE
// ============================================================================

struct __attribute__((packed)) ControlPacket {
    uint16_t throttle;      // 0-1000
    int16_t  yaw;           // -500 to +500
    int16_t  pitch;         // -500 to +500
    int16_t  roll;          // -500 to +500
    uint8_t  switches;      // Bit 0:Arm, 1:Calib, 2:MotorTest, 3:AltHold
    uint8_t  checksum;      // XOR of bytes 0-8
    uint32_t sequence;      // Packet counter
    uint8_t  channel;       // RF channel verification
    uint8_t  reserved;
    
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

// Attitude
float roll = 0, pitch = 0, yaw = 0;
float yawRate = 0, prevYaw = 0;
uint32_t yawTime = 0;

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
uint32_t lastPacketTime = 0;
uint32_t packetCount = 0;
bool radioConnected = false;
const uint8_t radioAddress[6] = "QUAD1";

// ============================================================================
//                           FLIGHT STATE
// ============================================================================

enum FlightState { 
    STATE_DISARMED, 
    STATE_ARMED, 
    STATE_FAILSAFE 
};
FlightState flightState = STATE_DISARMED;

// Control commands
int16_t throttleCmd = 0;
float rollCmd = 0, pitchCmd = 0, yawCmd = 0;
bool altHoldSwitch = false, altHoldActive = false;
float targetAltitude = 0;

// PID outputs
float rollOutput = 0, pitchOutput = 0, yawOutput = 0;
float altOutput = 0, altOutputPrev = 0;

// Motor outputs
uint16_t motorFL = ESC_MIN_US, motorFR = ESC_MIN_US;
uint16_t motorRL = ESC_MIN_US, motorRR = ESC_MIN_US;
uint16_t motorFL_prev = ESC_MIN_US, motorFR_prev = ESC_MIN_US;
uint16_t motorRL_prev = ESC_MIN_US, motorRR_prev = ESC_MIN_US;

// Previous arm switch state
bool prevArmSwitch = false;

// Timing
uint32_t timeIMU = 0, timePID = 0, timeRF = 0, timeBaro = 0;
uint32_t timeDebug = 0, timeLED = 0;

// ============================================================================
//                             PID CONTROLLER
// ============================================================================

class PIDController {
public:
    float Kp, Ki, Kd;
    float integral, prevMeasurement;
    float outputMin, outputMax, integralMax;
    bool initialized;
    
    PIDController(float p, float i, float d) : 
        Kp(p), Ki(i), Kd(d), integral(0), prevMeasurement(0),
        outputMin(-500), outputMax(500), integralMax(200), initialized(false) {}
    
    void reset() {
        integral = 0;
        prevMeasurement = 0;
        initialized = false;
    }
    
    float calculate(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;
        
        if (!initialized) {
            prevMeasurement = measurement;
            initialized = true;
        }
        
        // Proportional
        float pTerm = Kp * error;
        
        // Integral with anti-windup
        integral = constrain(integral + Ki * error * dt, -integralMax, integralMax);
        
        // Derivative on measurement (not error) to avoid derivative kick
        float derivative = -(measurement - prevMeasurement) / dt;
        float dTerm = Kd * derivative;
        prevMeasurement = measurement;
        
        return constrain(pTerm + integral + dTerm, outputMin, outputMax);
    }
};

// PID controllers
PIDController pidRoll(PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD);
PIDController pidPitch(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);
PIDController pidYaw(PID_YAW_KP, PID_YAW_KI, PID_YAW_KD);
PIDController pidAlt(PID_ALT_KP, PID_ALT_KI, PID_ALT_KD);

// ============================================================================
//                            BUZZER FUNCTIONS
// ============================================================================

void beep(uint16_t duration, uint16_t frequency = 2000) {
    tone(PIN_BUZZER, frequency, duration);
}

void beepBlocking(uint16_t duration, uint16_t frequency = 2000) {
    tone(PIN_BUZZER, frequency);
    delay(duration);
    noTone(PIN_BUZZER);
}

void soundStartup() {
    beepBlocking(100, 1500); delay(50);
    beepBlocking(100, 2000); delay(50);
    beepBlocking(100, 2500); delay(50);
    beepBlocking(200, 3000);
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
//                           DMP UPDATE
// ============================================================================

bool updateDMP() {
    if (!dmpReady) return false;
    
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &quaternion);
        mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
        
        // Convert to degrees
        float newYaw = ypr[0] * 57.2958f;
        pitch = ypr[1] * 57.2958f;
        roll = ypr[2] * 57.2958f;
        
        // Calculate yaw rate
        uint32_t now = micros();
        if (yawTime > 0) {
            float dt = (now - yawTime) / 1000000.0f;
            if (dt > 0 && dt < 0.1f) {
                float deltaYaw = newYaw - prevYaw;
                // Handle wrap-around
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
    
    // Validate reading
    if (pressure < 300 || pressure > 1200) return;
    
    // Calculate altitude
    float rawAltitude = 44330.0f * (1.0f - pow(pressure / basePressure, 0.1903f));
    
    // Rate limit changes (max 0.2m per sample)
    float change = constrain(rawAltitude - altitude, -0.2f, 0.2f);
    altitude += change;
    
    // Low-pass filter
    altitudeFiltered = 0.85f * altitudeFiltered + 0.15f * altitude;
    
    // Calculate vertical velocity
    uint32_t now = micros();
    float dt = (now - baroTime) / 1000000.0f;
    
    if (baroTime > 0 && dt > 0.01f && dt < 0.1f) {
        float rawVel = constrain((altitudeFiltered - altitudePrev) / dt, -3.0f, 3.0f);
        vertVelFiltered = 0.8f * vertVelFiltered + 0.2f * rawVel;
        vertVel = vertVelFiltered;
    }
    
    altitudePrev = altitudeFiltered;
    baroTime = now;
}

// ============================================================================
//                           RADIO UPDATE
// ============================================================================

void updateRadio() {
    if (radio.available()) {
        ControlPacket packet;
        radio.read(&packet, sizeof(packet));
        
        if (packet.isValid()) {
            rxPacket = packet;
            lastPacketTime = millis();
            packetCount++;
            
            // First connection
            if (!radioConnected) {
                radioConnected = true;
                soundPaired();
            }
        }
    }
    
    // Check for timeout
    if (radioConnected && (millis() - lastPacketTime > RF_TIMEOUT_MS)) {
        radioConnected = false;
        
        if (flightState == STATE_ARMED) {
            flightState = STATE_FAILSAFE;
            soundFailsafe();
        }
    }
}

// ============================================================================
//                        COMMAND PROCESSING
// ============================================================================

void processCommands() {
    if (!radioConnected) {
        throttleCmd = 0;
        rollCmd = pitchCmd = yawCmd = 0;
        return;
    }
    
    // Extract commands
    throttleCmd = rxPacket.throttle;
    rollCmd = map(rxPacket.roll, -500, 500, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE);
    pitchCmd = map(rxPacket.pitch, -500, 500, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);
    yawCmd = map(rxPacket.yaw, -500, 500, -MAX_YAW_RATE, MAX_YAW_RATE);
    
    // Extract switches
    bool armSwitch = rxPacket.switches & (1 << SW_ARM);
    altHoldSwitch = rxPacket.switches & (1 << SW_ALTHOLD);
    
    // Arm logic
    if (armSwitch && !prevArmSwitch && flightState == STATE_DISARMED) {
        // Arming conditions
        if (throttleCmd <= ESC_ARM_THROTTLE && radioConnected && dmpReady) {
            flightState = STATE_ARMED;
            pidRoll.reset();
            pidPitch.reset();
            pidYaw.reset();
            pidAlt.reset();
            altOutputPrev = 0;
            targetAltitude = altitudeFiltered;
            soundArmed();
        }
    } 
    // Disarm
    else if (!armSwitch && flightState == STATE_ARMED) {
        flightState = STATE_DISARMED;
        soundDisarmed();
    }
    
    // Failsafe recovery
    if (flightState == STATE_FAILSAFE) {
        if (!armSwitch || (radioConnected && throttleCmd < 100)) {
            flightState = STATE_DISARMED;
            beep(200, 1500);
        }
    }
    
    prevArmSwitch = armSwitch;
}

// ============================================================================
//                            PID UPDATE
// ============================================================================

void updatePID(float dt) {
    // Always calculate PID (for debugging even when disarmed)
    float tempRoll = pidRoll.calculate(rollCmd, roll, dt);
    float tempPitch = pidPitch.calculate(pitchCmd, pitch, dt);
    float tempYaw = pidYaw.calculate(yawCmd, yawRate, dt);
    
    // Store for display
    rollOutput = tempRoll;
    pitchOutput = tempPitch;
    yawOutput = tempYaw;
    
    if (flightState != STATE_ARMED) {
        altOutput = altOutputPrev = 0;
        return;
    }
    
    // Ground detection
    bool onGround = (throttleCmd < THR_GROUND_THRESH) || 
                    (abs(altitudeFiltered) < ALT_GROUND_THRESH && abs(vertVel) < 0.5f);
    
    altHoldActive = altHoldSwitch && !onGround && baroReady;
    
    if (altHoldActive) {
        // Adjust target altitude with throttle
        float thrDeviation = throttleCmd - 500;
        if (abs(thrDeviation) > 50) {
            targetAltitude += (thrDeviation / 500.0f) * 0.01f;
        }
        
        // Calculate altitude PID
        float rawAltOut = pidAlt.calculate(targetAltitude, altitudeFiltered, dt);
        rawAltOut = constrain(rawAltOut, -ALT_PID_MAX, ALT_PID_MAX);
        
        // Rate limit
        float altChange = constrain(rawAltOut - altOutputPrev, -ALT_RATE_LIMIT, ALT_RATE_LIMIT);
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

uint16_t rateLimitMotor(uint16_t target, uint16_t previous) {
    int16_t change = constrain((int16_t)target - (int16_t)previous, 
                               -MOTOR_RATE_LIMIT, MOTOR_RATE_LIMIT);
    return previous + change;
}

void updateMotors() {
    if (flightState != STATE_ARMED) {
        motorFL = motorFR = motorRL = motorRR = ESC_MIN_US;
        motorFL_prev = motorFR_prev = motorRL_prev = motorRR_prev = ESC_MIN_US;
    } else {
        // Base throttle
        int16_t baseThrottle = map(throttleCmd, 0, 1000, ESC_IDLE_US, ESC_MAX_US) - ESC_MIN_US;
        
        // Add altitude hold correction
        if (altHoldActive) {
            baseThrottle += (int16_t)altOutput;
        }
        
        // Motor mixing (Quad-X configuration)
        // FL (CCW) = Throttle - Roll + Pitch - Yaw
        // FR (CW)  = Throttle + Roll + Pitch + Yaw
        // RL (CW)  = Throttle - Roll - Pitch + Yaw
        // RR (CCW) = Throttle + Roll - Pitch - Yaw
        
        int16_t fl = baseThrottle - (int16_t)rollOutput + (int16_t)pitchOutput - (int16_t)yawOutput;
        int16_t fr = baseThrottle + (int16_t)rollOutput + (int16_t)pitchOutput + (int16_t)yawOutput;
        int16_t rl = baseThrottle - (int16_t)rollOutput - (int16_t)pitchOutput + (int16_t)yawOutput;
        int16_t rr = baseThrottle + (int16_t)rollOutput - (int16_t)pitchOutput - (int16_t)yawOutput;
        
        // Constrain
        uint16_t targetFL = constrain(fl + ESC_MIN_US, ESC_IDLE_US, ESC_MAX_US);
        uint16_t targetFR = constrain(fr + ESC_MIN_US, ESC_IDLE_US, ESC_MAX_US);
        uint16_t targetRL = constrain(rl + ESC_MIN_US, ESC_IDLE_US, ESC_MAX_US);
        uint16_t targetRR = constrain(rr + ESC_MIN_US, ESC_IDLE_US, ESC_MAX_US);
        
        // Rate limit
        motorFL = rateLimitMotor(targetFL, motorFL_prev); motorFL_prev = motorFL;
        motorFR = rateLimitMotor(targetFR, motorFR_prev); motorFR_prev = motorFR;
        motorRL = rateLimitMotor(targetRL, motorRL_prev); motorRL_prev = motorRL;
        motorRR = rateLimitMotor(targetRR, motorRR_prev); motorRR_prev = motorRR;
    }
    
    // Write to ESCs
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
    uint16_t interval;
    
    switch (flightState) {
        case STATE_ARMED:    interval = 0; break;      // Solid ON
        case STATE_FAILSAFE: interval = 100; break;    // Fast blink
        default:             interval = 500; break;    // Slow blink
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
    // State
    switch (flightState) {
        case STATE_ARMED:    Serial.print(F("ARM ")); break;
        case STATE_FAILSAFE: Serial.print(F("FAIL")); break;
        default:             Serial.print(F("DIS ")); break;
    }
    
    // RF
    Serial.print(F(" RF:"));
    Serial.print(radioConnected ? packetCount : 0);
    
    // Angles
    Serial.print(F(" R:"));
    Serial.print(roll, 1);
    Serial.print(F(" P:"));
    Serial.print(pitch, 1);
    
    // PID
    Serial.print(F(" PID:"));
    Serial.print((int)rollOutput);
    Serial.print(F(","));
    Serial.print((int)pitchOutput);
    
    // Altitude
    if (baroReady) {
        Serial.print(F(" Alt:"));
        Serial.print(altitudeFiltered, 2);
        Serial.print(altHoldActive ? F("*") : F(""));
    }
    
    // Motors
    Serial.print(F(" M:"));
    Serial.print(motorFL);
    Serial.print(F(","));
    Serial.print(motorFR);
    Serial.print(F(","));
    Serial.print(motorRL);
    Serial.print(F(","));
    Serial.println(motorRR);
}
#endif

// ============================================================================
//                              SETUP
// ============================================================================

void setup() {
#if ENABLE_DEBUG
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("\n=== QuadFC v3.0 ==="));
    Serial.print(F("RF Channel: "));
    Serial.println(RF_CHANNEL);
#endif
    
    // Initialize pins
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    soundStartup();
    
    // ---- Initialize MPU6050 + DMP ----
#if ENABLE_DEBUG
    Serial.print(F("MPU6050..."));
#endif
    
    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();
    
    if (!mpu.testConnection()) {
#if ENABLE_DEBUG
        Serial.println(F("FAIL (connection)"));
#endif
        while (1) { beepBlocking(200, 500); delay(300); }
    }
    
    dmpStatus = mpu.dmpInitialize();
    
    // Set calibration offsets
    mpu.setXAccelOffset(ACCEL_OFFSET_X);
    mpu.setYAccelOffset(ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(ACCEL_OFFSET_Z);
    mpu.setXGyroOffset(GYRO_OFFSET_X);
    mpu.setYGyroOffset(GYRO_OFFSET_Y);
    mpu.setZGyroOffset(GYRO_OFFSET_Z);
    
    if (dmpStatus == 0) {
#if ENABLE_DEBUG
        Serial.print(F("calibrating..."));
#endif
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
#if ENABLE_DEBUG
        Serial.println(F("OK"));
#endif
    } else {
#if ENABLE_DEBUG
        Serial.print(F("FAIL (DMP error "));
        Serial.print(dmpStatus);
        Serial.println(F(")"));
#endif
        while (1) { beepBlocking(200, 600); delay(300); }
    }
    
    // ---- Initialize MS5611 Barometer ----
#if ENABLE_DEBUG
    Serial.print(F("MS5611..."));
#endif
    
    if (baro.begin()) {
        // Get baseline pressure
        float sum = 0;
        for (int i = 0; i < 10; i++) {
            baro.read();
            sum += baro.getPressure();
            delay(50);
        }
        basePressure = sum / 10.0f;
        baroReady = true;
#if ENABLE_DEBUG
        Serial.print(F("OK ("));
        Serial.print(basePressure, 1);
        Serial.println(F(" mbar)"));
#endif
    } else {
#if ENABLE_DEBUG
        Serial.println(F("FAIL (continuing without altitude hold)"));
#endif
    }
    
    // ---- Initialize NRF24L01 ----
#if ENABLE_DEBUG
    Serial.print(F("NRF24L01..."));
#endif
    
    if (!radio.begin()) {
#if ENABLE_DEBUG
        Serial.println(F("FAIL"));
#endif
        while (1) { beepBlocking(200, 700); delay(300); }
    }
    
    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF24_2MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(sizeof(ControlPacket));
    radio.setAutoAck(true);
    radio.setRetries(5, 3);
    radio.setCRCLength(RF24_CRC_16);
    radio.openReadingPipe(1, radioAddress);
    radio.startListening();
    
#if ENABLE_DEBUG
    Serial.println(F("OK"));
#endif
    
    // ---- Initialize ESCs ----
#if ENABLE_DEBUG
    Serial.print(F("ESCs..."));
#endif
    
    escFL.attach(PIN_MOTOR_FL, ESC_MIN_US, ESC_MAX_US);
    escFR.attach(PIN_MOTOR_FR, ESC_MIN_US, ESC_MAX_US);
    escRL.attach(PIN_MOTOR_RL, ESC_MIN_US, ESC_MAX_US);
    escRR.attach(PIN_MOTOR_RR, ESC_MIN_US, ESC_MAX_US);
    
    escFL.writeMicroseconds(ESC_MIN_US);
    escFR.writeMicroseconds(ESC_MIN_US);
    escRL.writeMicroseconds(ESC_MIN_US);
    escRR.writeMicroseconds(ESC_MIN_US);
    
#if ENABLE_DEBUG
    Serial.println(F("OK"));
#endif
    
    // ---- Configure PID limits ----
    pidAlt.outputMin = -ALT_PID_MAX;
    pidAlt.outputMax = ALT_PID_MAX;
    pidAlt.integralMax = 50;
    
    // ---- Ready! ----
#if ENABLE_DEBUG
    Serial.println(F("\n*** READY ***"));
    Serial.println(F("Waiting for RC connection...\n"));
#endif
    
    soundReady();
    
    // Initialize timing
    timeIMU = timePID = timeRF = timeBaro = micros();
    timeDebug = timeLED = millis();
}

// ============================================================================
//                             MAIN LOOP
// ============================================================================

void loop() {
    uint32_t now = micros();
    
    // Update DMP (as fast as available, ~100Hz)
    updateDMP();
    
    // Update Radio (50Hz)
    if (now - timeRF >= 20000) {
        timeRF = now;
        updateRadio();
        processCommands();
    }
    
    // Update Barometer (40Hz)
    if (now - timeBaro >= 25000) {
        timeBaro = now;
        updateBarometer();
    }
    
    // Update PID + Motors (250Hz)
    if (now - timePID >= 4000) {
        float dt = (now - timePID) / 1000000.0f;
        timePID = now;
        updatePID(dt);
        updateMotors();
    }
    
    // Update LED
    updateLED();
    
    // Debug output (5Hz)
#if ENABLE_DEBUG
    if (millis() - timeDebug >= 200) {
        timeDebug = millis();
        printDebug();
    }
#endif
}
