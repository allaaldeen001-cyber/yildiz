/**
 * ============================================================================
 *    QUADCOPTER FC - V5 ROLL FIX + ANTI-OSCILLATION
 * ============================================================================
 * 
 * V5 CRITICAL FIXES:
 *   ✓ FIXED: Roll mixing signs INVERTED (was causing drift + instability)
 *   ✓ FIXED: Very low PID gains to stop oscillation/vibration
 *   ✓ FIXED: Balanced motor trims (all same value)
 *   ✓ FIXED: Heavy filtering for stability
 * 
 * MOTOR LAYOUT (X-configuration, viewed from above):
 *        FRONT
 *   FL(CCW)  FR(CW)
 *       X
 *   RL(CW)   RR(CCW)
 * 
 * ROLL MIXING FIX EXPLANATION:
 *   When tilted RIGHT: roll angle positive, rollPID becomes NEGATIVE
 *   To correct: LEFT motors must INCREASE, RIGHT motors must DECREASE
 *   So: FL,RL need -rollPID (negative becomes positive = increase)
 *       FR,RR need +rollPID (stays negative = decrease)
 * 
 * ============================================================================
 */
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// ============================================================================
//                           CONFIGURATION
// ============================================================================
#define RF_CHANNEL          108
#define SERIAL_BAUD         115200

// ============================================================================
//                          PIN DEFINITIONS
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
//                          ESC PARAMETERS
// ============================================================================
#define ESC_MIN             1000
#define ESC_MAX             2000
#define ESC_IDLE            1150    // Lower idle for better control
#define ESC_ARM_THR         50
#define ESC_MAX_THROTTLE    1700

// ============================================================================
//                    MOTOR TRIM - ALL BALANCED
// ============================================================================
// Start with all trims equal - adjust only after drone is stable
#define TRIM_FL             0
#define TRIM_FR             0
#define TRIM_RL             0
#define TRIM_RR             0

// Pitch/Roll offset trim (for CG compensation)
#define PITCH_TRIM          0
#define ROLL_TRIM           0

// ============================================================================
//                    SAFETY LIMITS
// ============================================================================
#define MAX_TILT_ANGLE      60.0f
#define EMERGENCY_ANGLE     45.0f
#define MAX_GYRO_RATE       500.0f

// ============================================================================
//             PID GAINS - VERY LOW TO STOP OSCILLATION
// ============================================================================
// Start VERY low - increase slowly after confirming stability

// OUTER LOOP - Angle PID
#define ANGLE_ROLL_KP       0.8f    // Very low - increase if sluggish
#define ANGLE_ROLL_KI       0.0f    
#define ANGLE_ROLL_KD       0.0f    

#define ANGLE_PITCH_KP      0.8f
#define ANGLE_PITCH_KI      0.0f
#define ANGLE_PITCH_KD      0.0f

// INNER LOOP - Rate PID (most critical for oscillation)
#define RATE_ROLL_KP        0.15f   // Very low - main oscillation cause
#define RATE_ROLL_KI        0.0f    
#define RATE_ROLL_KD        0.005f  // Small D for damping

#define RATE_PITCH_KP       0.15f
#define RATE_PITCH_KI       0.0f
#define RATE_PITCH_KD       0.005f

// YAW
#define PID_YAW_KP          0.8f
#define PID_YAW_KI          0.0f
#define PID_YAW_KD          0.0f

// PID limits
#define ANGLE_I_MAX         30.0f
#define RATE_I_MAX          50.0f
#define RATE_OUTPUT_MAX     150.0f  // Reduced max output
#define ANGLE_RATE_MAX      100.0f

// ============================================================================
//                    FILTERING - HEAVY FOR STABILITY
// ============================================================================
#define SETPOINT_LPF_ALPHA  0.15f   // Smooth setpoint changes
#define D_TERM_LPF_ALPHA    0.1f    // Heavy D filtering
#define OUTPUT_RATE_LIMIT   15.0f   // Limit rate of change

#define GYRO_LPF_ALPHA      0.3f    // Heavy gyro filtering
#define ACCEL_LPF_ALPHA     0.2f    // Heavy accel filtering
#define MOTOR_LPF_ALPHA     0.25f   // Heavy motor smoothing
#define RC_LPF_ALPHA        0.5f
#define POT_LPF_ALPHA       0.1f
#define RC_DEADBAND         25
#define GYRO_DEADBAND       0.5f

// ============================================================================
//                    COMPLEMENTARY FILTER
// ============================================================================
#define COMP_FILTER_ALPHA   0.96f

// ============================================================================
//                    AXIS CONFIGURATION
// ============================================================================
#define PITCH_INVERT    -1.0f
#define ROLL_INVERT      1.0f
#define RC_ROLL_INVERT      -1.0f
#define RC_PITCH_INVERT     -1.0f
#define RC_YAW_INVERT       -1.0f

// ============================================================================
//                          RF PACKET
// ============================================================================
struct __attribute__((packed)) ControlPacket {
    uint16_t throttle;
    int16_t  yaw, pitch, roll;
    uint8_t  switches, checksum;
    uint32_t sequence;
    uint8_t  channel, auxData;
    
    bool isValid() const {
        const uint8_t* data = (const uint8_t*)this;
        uint8_t calc = 0;
        for (uint8_t i = 0; i < 9; i++) calc ^= data[i];
        return calc == checksum;
    }
};

#define SW_ARM          0
#define SW_CALIBRATE    1
#define SW_MOTORTEST    2
#define SW_ALTHOLD      3
#define SW_FLIGHTMODE   4
#define SW_RATES        5

#define FMODE_ANGLE     0
#define FMODE_HORIZON   1
#define FMODE_ACRO      2

// ============================================================================
//                         GLOBAL OBJECTS
// ============================================================================
MPU6050 mpu;
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
Servo escFL, escFR, escRL, escRR;

// ============================================================================
//                         CALIBRATION DATA
// ============================================================================
struct CalibrationData {
    int16_t axOffset, ayOffset, azOffset;
    int16_t gxOffset, gyOffset, gzOffset;
    float rollOffset, pitchOffset;
    bool valid;
} calibration = {0, 0, 0, 0, 0, 0, 0, 0, false};

// ============================================================================
//                         IMU VARIABLES
// ============================================================================
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float roll = 0, pitch = 0, yaw = 0;
float rollRate = 0, pitchRate = 0, yawRate = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
bool biasLocked = false;

// ============================================================================
//                  CASCADED PID STATE
// ============================================================================
struct CascadedPIDState {
    float angleIntegral;
    float prevAngle;
    float rateIntegral;
    float prevRate;
    float prevDterm;
    float filteredSetpoint;
    float prevOutput;
    
    void reset() {
        angleIntegral = 0;
        prevAngle = 0;
        rateIntegral = 0;
        prevRate = 0;
        prevDterm = 0;
        filteredSetpoint = 0;
        prevOutput = 0;
    }
} pidRollState, pidPitchState;

struct SimplePIDState {
    float integral;
    float prevError;
    float prevDerivative;
    
    void reset() {
        integral = 0;
        prevError = 0;
        prevDerivative = 0;
    }
} pidYawState;

float rollPID = 0, pitchPID = 0, yawPID = 0;
float gainMultiplier = 1.0f;
float maxAngle = 25.0f;

// ============================================================================
//                         RADIO VARIABLES
// ============================================================================
ControlPacket rxPacket;
const uint8_t radioAddress[6] = "QUAD1";
uint32_t lastValidPacket = 0;
uint32_t packetCount = 0;
bool radioConnected = false;

// ============================================================================
//                         FLIGHT STATE
// ============================================================================
enum State { DISARMED, ARMED, FAILSAFE, EMERGENCY };
State flightState = DISARMED;

float throttleCmd = 0;
float rollCmd = 0, pitchCmd = 0, yawCmd = 0;
uint8_t flightMode = FMODE_ANGLE;

uint16_t motorFL = ESC_MIN, motorFR = ESC_MIN;
uint16_t motorRL = ESC_MIN, motorRR = ESC_MIN;
float motorFL_f = ESC_MIN, motorFR_f = ESC_MIN;
float motorRL_f = ESC_MIN, motorRR_f = ESC_MIN;

bool prevArm = false;

// Safety tracking
uint32_t emergencyDisarmTime = 0;
uint8_t emergencyCount = 0;

// Timing
uint32_t lastIMUTime = 0;
uint32_t lastRFTime = 0;
uint32_t lastDebugTime = 0;
uint32_t lastSafetyCheck = 0;

// ============================================================================
//                         UTILITY FUNCTIONS
// ============================================================================
void beep(uint16_t freq, uint16_t ms) {
    tone(PIN_BUZZER, freq, ms);
}

void beepBlocking(uint16_t freq, uint16_t ms) {
    tone(PIN_BUZZER, freq);
    delay(ms);
    noTone(PIN_BUZZER);
}

void beepPattern(uint8_t count, uint16_t freq, uint16_t onTime, uint16_t offTime) {
    for (uint8_t i = 0; i < count; i++) {
        beepBlocking(freq, onTime);
        if (i < count - 1) delay(offTime);
    }
}

float applyDeadband(float value, float band) {
    if (abs(value) < band) return 0;
    return value;
}

float lowPassFilter(float current, float newValue, float alpha) {
    return current + alpha * (newValue - current);
}

float constrainFloat(float value, float minVal, float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

float rateLimitChange(float current, float target, float maxChange) {
    float diff = target - current;
    if (diff > maxChange) return current + maxChange;
    if (diff < -maxChange) return current - maxChange;
    return target;
}

// ============================================================================
//                    EMERGENCY DISARM
// ============================================================================
void emergencyDisarm(const char* reason) {
    flightState = EMERGENCY;
    
    escFL.writeMicroseconds(ESC_MIN);
    escFR.writeMicroseconds(ESC_MIN);
    escRL.writeMicroseconds(ESC_MIN);
    escRR.writeMicroseconds(ESC_MIN);
    
    Serial.print(F("\n*** EMERGENCY DISARM: "));
    Serial.print(reason);
    Serial.println(F(" ***\n"));
    
    beepPattern(5, 1000, 100, 100);
    
    emergencyDisarmTime = millis();
    emergencyCount++;
}

// ============================================================================
//                    SAFETY CHECKS
// ============================================================================
void checkSafety() {
    if (flightState != ARMED) return;
    
    float totalTilt = sqrt(roll*roll + pitch*pitch);
    
    if (totalTilt > MAX_TILT_ANGLE) {
        emergencyDisarm("TILT >60 DEG");
        return;
    }
    
    if (totalTilt > EMERGENCY_ANGLE) {
        if (millis() - lastSafetyCheck > 200) {
            beep(2000, 50);
            lastSafetyCheck = millis();
        }
    }
    
    float maxRate = max(abs(rollRate), max(abs(pitchRate), abs(yawRate)));
    if (maxRate > MAX_GYRO_RATE) {
        emergencyDisarm("EXTREME ROTATION");
        return;
    }
}

// ============================================================================
//                    MPU6050 CALIBRATION
// ============================================================================
void calibrateMPU6050() {
    Serial.println(F("\n========================================"));
    Serial.println(F("   MPU6050 CALIBRATION"));
    Serial.println(F("========================================"));
    Serial.println(F("Place drone FLAT and LEVEL!"));
    Serial.println(F("Starting in 3 seconds..."));
    
    beepPattern(3, 1500, 200, 300);
    delay(1000);
    
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    delay(100);
    
    int16_t ax_off = 0, ay_off = 0, az_off = 0;
    int16_t gx_off = 0, gy_off = 0, gz_off = 0;
    
    Serial.println(F("Calibrating..."));
    
    for (int iter = 0; iter < 6; iter++) {
        Serial.print(F("Pass ")); Serial.print(iter + 1); Serial.print(F("/6"));
        
        int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
        int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
        
        for (int i = 0; i < 500; i++) {
            int16_t ax, ay, az, gx, gy, gz;
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            
            ax_sum += ax;
            ay_sum += ay;
            az_sum += az;
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
            
            delayMicroseconds(2000);
        }
        
        int16_t ax_avg = ax_sum / 500;
        int16_t ay_avg = ay_sum / 500;
        int16_t az_avg = az_sum / 500;
        int16_t gx_avg = gx_sum / 500;
        int16_t gy_avg = gy_sum / 500;
        int16_t gz_avg = gz_sum / 500;
        
        ax_off -= ax_avg / 8;
        ay_off -= ay_avg / 8;
        az_off += (16384 - az_avg) / 8;
        gx_off -= gx_avg / 4;
        gy_off -= gy_avg / 4;
        gz_off -= gz_avg / 4;
        
        mpu.setXAccelOffset(ax_off);
        mpu.setYAccelOffset(ay_off);
        mpu.setZAccelOffset(az_off);
        mpu.setXGyroOffset(gx_off);
        mpu.setYGyroOffset(gy_off);
        mpu.setZGyroOffset(gz_off);
        
        Serial.print(F(" - Err: "));
        Serial.println(abs(gx_avg) + abs(gy_avg) + abs(gz_avg));
        
        delay(50);
    }
    
    calibration.axOffset = ax_off;
    calibration.ayOffset = ay_off;
    calibration.azOffset = az_off;
    calibration.gxOffset = gx_off;
    calibration.gyOffset = gy_off;
    calibration.gzOffset = gz_off;
    
    Serial.println(F("Measuring level trim..."));
    delay(200);
    
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    
    for (int i = 0; i < 1000; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        ax_sum += ax; ay_sum += ay; az_sum += az;
        delayMicroseconds(1000);
    }
    
    float ax_avg = ax_sum / 1000.0f;
    float ay_avg = ay_sum / 1000.0f;
    float az_avg = az_sum / 1000.0f;
    
    calibration.rollOffset = atan2(ay_avg, az_avg) * 57.2958f;
    calibration.pitchOffset = atan2(-ax_avg, sqrt(ay_avg*ay_avg + az_avg*az_avg)) * 57.2958f;
    
    Serial.print(F("Level trim: R=")); Serial.print(calibration.rollOffset, 2);
    Serial.print(F("° P=")); Serial.print(calibration.pitchOffset, 2);
    Serial.println(F("°"));
    
    calibration.valid = true;
    Serial.println(F("*** CALIBRATION COMPLETE ***\n"));
    beepPattern(2, 2500, 100, 100);
}

// ============================================================================
//                    READ IMU
// ============================================================================
void readIMU() {
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
    
    float gx_dps = gx_raw / 65.5f;
    float gy_dps = gy_raw / 65.5f;
    float gz_dps = gz_raw / 65.5f;
    
    float ax_g = ax_raw / 16384.0f;
    float ay_g = ay_raw / 16384.0f;
    float az_g = az_raw / 16384.0f;
    
    gyroX = lowPassFilter(gyroX, gx_dps, GYRO_LPF_ALPHA);
    gyroY = lowPassFilter(gyroY, gy_dps, GYRO_LPF_ALPHA);
    gyroZ = lowPassFilter(gyroZ, gz_dps, GYRO_LPF_ALPHA);
    
    accelX = lowPassFilter(accelX, ax_g, ACCEL_LPF_ALPHA);
    accelY = lowPassFilter(accelY, ay_g, ACCEL_LPF_ALPHA);
    accelZ = lowPassFilter(accelZ, az_g, ACCEL_LPF_ALPHA);
    
    if (flightState == DISARMED && !biasLocked) {
        gyroBiasX = lowPassFilter(gyroBiasX, gyroX, 0.005f);
        gyroBiasY = lowPassFilter(gyroBiasY, gyroY, 0.005f);
        gyroBiasZ = lowPassFilter(gyroBiasZ, gyroZ, 0.005f);
    }
    
    float gyroX_comp = gyroX - gyroBiasX;
    float gyroY_comp = gyroY - gyroBiasY;
    float gyroZ_comp = gyroZ - gyroBiasZ;
    
    rollRate = applyDeadband(gyroX_comp, GYRO_DEADBAND);
    pitchRate = applyDeadband(gyroY_comp, GYRO_DEADBAND);
    yawRate = applyDeadband(gyroZ_comp, GYRO_DEADBAND);
}

// ============================================================================
//                    UPDATE ANGLES
// ============================================================================
void updateAngles(float dt) {
    float accelRoll = atan2(accelY, accelZ) * 57.2958f;
    float accelPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 57.2958f;
    
    accelRoll -= calibration.rollOffset;
    accelPitch -= calibration.pitchOffset;
    
    accelRoll *= ROLL_INVERT;
    accelPitch *= PITCH_INVERT;
    
    float pitchRateAdj = pitchRate * PITCH_INVERT;
    float rollRateAdj = rollRate * ROLL_INVERT;
    
    roll = COMP_FILTER_ALPHA * (roll + rollRateAdj * dt) + 
           (1.0f - COMP_FILTER_ALPHA) * accelRoll;
    pitch = COMP_FILTER_ALPHA * (pitch + pitchRateAdj * dt) + 
            (1.0f - COMP_FILTER_ALPHA) * accelPitch;
    
    yaw += yawRate * dt;
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
}

// ============================================================================
//                         UPDATE RADIO
// ============================================================================
void updateRadio() {
    while (radio.available()) {
        ControlPacket packet;
        radio.read(&packet, sizeof(packet));
        
        if (packet.isValid() && packet.channel == RF_CHANNEL) {
            rxPacket = packet;
            lastValidPacket = millis();
            packetCount++;
            
            uint8_t gainVal = (packet.auxData >> 4) & 0x0F;
            uint8_t angleVal = packet.auxData & 0x0F;
            
            float targetGain = map(gainVal, 0, 15, 50, 150) / 100.0f;
            float targetAngle = map(angleVal, 0, 15, 15, 35);
            
            gainMultiplier = lowPassFilter(gainMultiplier, targetGain, POT_LPF_ALPHA);
            maxAngle = lowPassFilter(maxAngle, targetAngle, POT_LPF_ALPHA);
            
            bool fmodeBit = packet.switches & (1 << SW_FLIGHTMODE);
            bool rateBit = packet.switches & (1 << SW_RATES);
            
            if (!fmodeBit) flightMode = FMODE_ANGLE;
            else if (fmodeBit && !rateBit) flightMode = FMODE_HORIZON;
            else flightMode = FMODE_ACRO;
            
            if (!radioConnected) {
                radioConnected = true;
                Serial.println(F("*** RF CONNECTED ***"));
                beepPattern(3, 2500, 80, 80);
            }
        }
    }
    
    if (radioConnected && (millis() - lastValidPacket > 500)) {
        radioConnected = false;
        if (flightState == ARMED) {
            emergencyDisarm("RF LOST");
        }
    }
}

// ============================================================================
//                         PROCESS COMMANDS
// ============================================================================
void processCommands() {
    if (!radioConnected) {
        throttleCmd = 0;
        rollCmd = pitchCmd = yawCmd = 0;
        return;
    }
    
    float rawThrottle = rxPacket.throttle;
    float rawRoll = applyDeadband(rxPacket.roll * RC_ROLL_INVERT, RC_DEADBAND);
    float rawPitch = applyDeadband(rxPacket.pitch * RC_PITCH_INVERT, RC_DEADBAND);
    float rawYaw = applyDeadband(rxPacket.yaw * RC_YAW_INVERT, RC_DEADBAND);
    
    throttleCmd = lowPassFilter(throttleCmd, rawThrottle, RC_LPF_ALPHA);
    rollCmd = lowPassFilter(rollCmd, rawRoll, RC_LPF_ALPHA);
    pitchCmd = lowPassFilter(pitchCmd, rawPitch, RC_LPF_ALPHA);
    yawCmd = lowPassFilter(yawCmd, rawYaw, RC_LPF_ALPHA);
    
    bool armSwitch = rxPacket.switches & (1 << SW_ARM);
    
    if (armSwitch && !prevArm && flightState == DISARMED) {
        bool throttleLow = (throttleCmd < ESC_ARM_THR);
        bool isLevel = (abs(roll) < 8.0f && abs(pitch) < 8.0f);
        bool calibOK = calibration.valid;
        
        if (throttleLow && isLevel && calibOK && radioConnected) {
            flightState = ARMED;
            biasLocked = true;
            pidRollState.reset();
            pidPitchState.reset();
            pidYawState.reset();
            Serial.println(F("\n*** ARMED ***"));
            beepBlocking(2000, 100);
            delay(100);
            beepBlocking(2500, 200);
        } else {
            Serial.println(F("ARM FAILED:"));
            if (!throttleLow) Serial.println(F("  - Throttle not low"));
            if (!isLevel) {
                Serial.print(F("  - Not level (R:")); Serial.print(roll, 1);
                Serial.print(F(" P:")); Serial.print(pitch, 1); Serial.println(F(")"));
            }
            if (!calibOK) Serial.println(F("  - Calibration invalid"));
            beepPattern(3, 500, 100, 100);
        }
    } else if (!armSwitch && (flightState == ARMED || flightState == FAILSAFE)) {
        flightState = DISARMED;
        biasLocked = false;
        Serial.println(F("\n*** DISARMED ***"));
        beepBlocking(1500, 300);
    } else if (flightState == EMERGENCY && !armSwitch) {
        if (millis() - emergencyDisarmTime > 5000) {
            flightState = DISARMED;
            Serial.println(F("Emergency cleared. Ready to arm.\n"));
        }
    }
    
    prevArm = armSwitch;
}

// ============================================================================
//                         CASCADED PID
// ============================================================================
float calculateCascadedPID(float targetAngle, float currentAngle, float currentRate, 
                           float dt, CascadedPIDState& state,
                           float angleKp, float angleKi, 
                           float rateKp, float rateKi, float rateKd) {
    
    // Setpoint filtering
    state.filteredSetpoint = lowPassFilter(state.filteredSetpoint, targetAngle, SETPOINT_LPF_ALPHA);
    float smoothedTarget = state.filteredSetpoint;
    
    // OUTER LOOP (Angle)
    float angleError = smoothedTarget - currentAngle;
    float targetRate = angleKp * gainMultiplier * angleError;
    
    if (abs(angleError) > 0.5f && angleKi > 0) {
        state.angleIntegral += angleError * dt;
        state.angleIntegral = constrainFloat(state.angleIntegral, -ANGLE_I_MAX, ANGLE_I_MAX);
        targetRate += angleKi * gainMultiplier * state.angleIntegral;
    }
    
    targetRate = constrainFloat(targetRate, -ANGLE_RATE_MAX, ANGLE_RATE_MAX);
    
    // INNER LOOP (Rate)
    float rateError = targetRate - currentRate;
    float rateP = rateKp * gainMultiplier * rateError;
    
    float rateI = 0;
    if (rateKi > 0) {
        state.rateIntegral += rateError * dt;
        state.rateIntegral = constrainFloat(state.rateIntegral, -RATE_I_MAX, RATE_I_MAX);
        rateI = rateKi * gainMultiplier * state.rateIntegral;
    }
    
    // D term on measurement
    float rateDelta = (currentRate - state.prevRate) / dt;
    state.prevRate = currentRate;
    
    float rateD_raw = -rateKd * gainMultiplier * rateDelta;
    state.prevDterm = lowPassFilter(state.prevDterm, rateD_raw, D_TERM_LPF_ALPHA);
    float rateD = state.prevDterm;
    
    float output = rateP + rateI + rateD;
    
    // Rate limiting
    output = rateLimitChange(state.prevOutput, output, OUTPUT_RATE_LIMIT);
    state.prevOutput = output;
    
    return constrainFloat(output, -RATE_OUTPUT_MAX, RATE_OUTPUT_MAX);
}

float calculateSimplePID(float setpoint, float measurement, float dt,
                         float Kp, float Ki, float Kd, SimplePIDState& state) {
    float error = setpoint - measurement;
    
    float P = Kp * gainMultiplier * error;
    
    if (abs(error) > 0.5f) {
        state.integral += error * dt;
    }
    state.integral = constrainFloat(state.integral, -100.0f, 100.0f);
    float I = Ki * gainMultiplier * state.integral;
    
    float rawD = (error - state.prevError) / dt;
    float D_filtered = lowPassFilter(state.prevDerivative, rawD, D_TERM_LPF_ALPHA);
    state.prevDerivative = D_filtered;
    float D = Kd * gainMultiplier * D_filtered;
    
    state.prevError = error;
    
    return constrainFloat(P + I + D, -RATE_OUTPUT_MAX, RATE_OUTPUT_MAX);
}

// ============================================================================
//                         UPDATE PID
// ============================================================================
void updatePID(float dt) {
    if (flightState != ARMED) {
        rollPID = pitchPID = yawPID = 0;
        return;
    }
    
    float adjRollRate = rollRate * ROLL_INVERT;
    float adjPitchRate = pitchRate * PITCH_INVERT;
    
    if (flightMode == FMODE_ANGLE) {
        float targetRoll = (rollCmd / 500.0f) * maxAngle;
        float targetPitch = (pitchCmd / 500.0f) * maxAngle;
        
        rollPID = calculateCascadedPID(targetRoll, roll, adjRollRate, dt, pidRollState,
                                       ANGLE_ROLL_KP, ANGLE_ROLL_KI,
                                       RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD);
        
        pitchPID = calculateCascadedPID(targetPitch, pitch, adjPitchRate, dt, pidPitchState,
                                        ANGLE_PITCH_KP, ANGLE_PITCH_KI,
                                        RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD);
        
    } else if (flightMode == FMODE_HORIZON) {
        float targetRoll = (rollCmd / 500.0f) * maxAngle;
        float targetPitch = (pitchCmd / 500.0f) * maxAngle;
        
        float angleRollPID = calculateCascadedPID(targetRoll, roll, adjRollRate, dt, pidRollState,
                                                  ANGLE_ROLL_KP, ANGLE_ROLL_KI,
                                                  RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD);
        float anglePitchPID = calculateCascadedPID(targetPitch, pitch, adjPitchRate, dt, pidPitchState,
                                                   ANGLE_PITCH_KP, ANGLE_PITCH_KI,
                                                   RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD);
        
        rollPID = 0.7f * angleRollPID + 0.3f * (rollCmd * 0.3f);
        pitchPID = 0.7f * anglePitchPID + 0.3f * (pitchCmd * 0.3f);
        
    } else {
        rollPID = rollCmd * 0.3f;
        pitchPID = pitchCmd * 0.3f;
    }
    
    float targetYawRate = (yawCmd / 500.0f) * 150.0f;
    yawPID = calculateSimplePID(targetYawRate, yawRate, dt, 
                                PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, pidYawState);
}

// ============================================================================
//                  UPDATE MOTORS - ROLL SIGNS FIXED!
// ============================================================================
void updateMotors() {
    if (flightState != ARMED) {
        motorFL = motorFR = motorRL = motorRR = ESC_MIN;
        motorFL_f = motorFR_f = motorRL_f = motorRR_f = ESC_MIN;
    } else {
        int16_t baseThr = map(throttleCmd, 0, 1000, ESC_IDLE, ESC_MAX_THROTTLE) - ESC_MIN;
        
        // Minimum throttle for control authority
        if (baseThr < 100) baseThr = 100;
        
        int16_t r = (int16_t)rollPID;
        int16_t p = (int16_t)pitchPID;
        int16_t y = (int16_t)yawPID;
        
        // ================================================================
        // CORRECTED MOTOR MIXING - ROLL SIGNS FIXED
        // ================================================================
        // Roll: When tilted RIGHT, rollPID is NEGATIVE
        //       LEFT motors (FL,RL) need to INCREASE: use -rollPID
        //       RIGHT motors (FR,RR) need to DECREASE: use +rollPID
        //
        // Pitch: When nose DOWN, pitchPID is POSITIVE  
        //        FRONT motors need to INCREASE: use +pitchPID
        //        REAR motors need to DECREASE: use -pitchPID
        //
        // Yaw: For CW rotation, yawPID is POSITIVE
        //      CCW motors (FL,RR) INCREASE: use +yawPID
        //      CW motors (FR,RL) DECREASE: use -yawPID
        
        int16_t fl = baseThr - r + p + y;  // Left, Front, CCW
        int16_t fr = baseThr + r + p - y;  // Right, Front, CW
        int16_t rl = baseThr - r - p - y;  // Left, Rear, CW
        int16_t rr = baseThr + r - p + y;  // Right, Rear, CCW
        
        // Apply trims
        fl += TRIM_FL + ROLL_TRIM + PITCH_TRIM;
        fr += TRIM_FR - ROLL_TRIM + PITCH_TRIM;
        rl += TRIM_RL + ROLL_TRIM - PITCH_TRIM;
        rr += TRIM_RR - ROLL_TRIM - PITCH_TRIM;
        
        // Constrain
        uint16_t tFL = constrain(fl + ESC_MIN, ESC_MIN, ESC_MAX);
        uint16_t tFR = constrain(fr + ESC_MIN, ESC_MIN, ESC_MAX);
        uint16_t tRL = constrain(rl + ESC_MIN, ESC_MIN, ESC_MAX);
        uint16_t tRR = constrain(rr + ESC_MIN, ESC_MIN, ESC_MAX);
        
        // Smooth motor outputs
        motorFL_f = lowPassFilter(motorFL_f, tFL, MOTOR_LPF_ALPHA);
        motorFR_f = lowPassFilter(motorFR_f, tFR, MOTOR_LPF_ALPHA);
        motorRL_f = lowPassFilter(motorRL_f, tRL, MOTOR_LPF_ALPHA);
        motorRR_f = lowPassFilter(motorRR_f, tRR, MOTOR_LPF_ALPHA);
        
        motorFL = (uint16_t)motorFL_f;
        motorFR = (uint16_t)motorFR_f;
        motorRL = (uint16_t)motorRL_f;
        motorRR = (uint16_t)motorRR_f;
    }
    
    escFL.writeMicroseconds(motorFL);
    escFR.writeMicroseconds(motorFR);
    escRL.writeMicroseconds(motorRL);
    escRR.writeMicroseconds(motorRR);
}

// ============================================================================
//                         LED UPDATE
// ============================================================================
void updateLED() {
    static uint32_t lastToggle = 0;
    static bool ledState = false;
    
    uint16_t interval;
    if (flightState == ARMED) interval = 0;
    else if (flightState == EMERGENCY) interval = 50;
    else if (!calibration.valid) interval = 200;
    else interval = 500;
    
    if (interval == 0) {
        digitalWrite(PIN_LED, HIGH);
    } else if (millis() - lastToggle >= interval) {
        lastToggle = millis();
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState);
    }
}

// ============================================================================
//                         DEBUG OUTPUT
// ============================================================================
void printDebug() {
    if (flightState == ARMED) Serial.print(F("ARM "));
    else if (flightState == EMERGENCY) Serial.print(F("EMG "));
    else Serial.print(F("DIS "));
    
    // Show angles and which way correction should go
    Serial.print(F("| R:")); Serial.print(roll, 1);
    if (roll > 3) Serial.print(F("L"));  // Tilted right, correct left
    else if (roll < -3) Serial.print(F("R"));
    
    Serial.print(F(" P:")); Serial.print(pitch, 1);
    if (pitch < -3) Serial.print(F("U"));  // Nose down, correct up
    else if (pitch > 3) Serial.print(F("D"));
    
    // PID outputs
    Serial.print(F(" | PID:")); Serial.print(rollPID, 0);
    Serial.print(F(",")); Serial.print(pitchPID, 0);
    
    // Motors
    Serial.print(F(" | M:")); Serial.print(motorFL);
    Serial.print(F(",")); Serial.print(motorFR);
    Serial.print(F(",")); Serial.print(motorRL);
    Serial.print(F(",")); Serial.print(motorRR);
    
    // Balance
    int16_t LR = (motorFL + motorRL) - (motorFR + motorRR);
    int16_t FB = (motorFL + motorFR) - (motorRL + motorRR);
    Serial.print(F(" LR:")); Serial.print(LR);
    Serial.print(F(" FB:")); Serial.print(FB);
    
    Serial.println();
}

// ============================================================================
//                              SETUP
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 1000);
    
    Serial.println(F("\n=========================================="));
    Serial.println(F("  QUADCOPTER FC - V5"));
    Serial.println(F("  Roll Fix + Anti-Oscillation"));
    Serial.println(F("==========================================\n"));
    
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    beepPattern(3, 2000, 100, 100);
    
    Wire.begin();
    Wire.setClock(400000);
    
    Serial.print(F("Init MPU6050..."));
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println(F("FAILED!"));
        while (1) {
            beepBlocking(500, 200);
            delay(300);
        }
    }
    Serial.println(F("OK"));
    
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);
    
    calibrateMPU6050();
    
    Serial.print(F("Init NRF24L01..."));
    if (!radio.begin()) {
        Serial.println(F("FAILED!"));
        while (1) {
            beepBlocking(600, 200);
            delay(300);
        }
    }
    
    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(16);
    radio.setAutoAck(false);
    radio.disableDynamicPayloads();
    radio.setCRCLength(RF24_CRC_16);
    radio.openReadingPipe(1, radioAddress);
    radio.startListening();
    Serial.println(F("OK"));
    
    Serial.print(F("Init ESCs..."));
    escFL.attach(PIN_MOTOR_FL, ESC_MIN, ESC_MAX);
    escFR.attach(PIN_MOTOR_FR, ESC_MIN, ESC_MAX);
    escRL.attach(PIN_MOTOR_RL, ESC_MIN, ESC_MAX);
    escRR.attach(PIN_MOTOR_RR, ESC_MIN, ESC_MAX);
    
    escFL.writeMicroseconds(ESC_MIN);
    escFR.writeMicroseconds(ESC_MIN);
    escRL.writeMicroseconds(ESC_MIN);
    escRR.writeMicroseconds(ESC_MIN);
    Serial.println(F("OK"));
    
    Serial.println(F("\n*** SYSTEM READY - V5 ***"));
    Serial.println(F("\nV5 FIXES:"));
    Serial.println(F("  1. ROLL mixing signs FIXED"));
    Serial.println(F("  2. Very low PID gains"));
    Serial.println(F("  3. Heavy filtering"));
    Serial.println(F("  4. Balanced trims"));
    Serial.println(F("\nGains (increase if too sluggish):"));
    Serial.println(F("  ANGLE_KP=0.8  RATE_KP=0.15"));
    Serial.println(F("\nWaiting for radio...\n"));
    
    beepPattern(2, 2500, 150, 150);
    
    lastIMUTime = micros();
    lastRFTime = lastDebugTime = millis();
}

// ============================================================================
//                         MAIN LOOP - 500Hz
// ============================================================================
void loop() {
    uint32_t nowMicros = micros();
    uint32_t nowMillis = millis();
    
    if (nowMicros - lastIMUTime >= 2000) {
        float dt = (nowMicros - lastIMUTime) / 1000000.0f;
        lastIMUTime = nowMicros;
        
        if (dt > 0.01f) dt = 0.002f;
        
        readIMU();
        updateAngles(dt);
        updatePID(dt);
        updateMotors();
        checkSafety();
    }
    
    if (nowMillis - lastRFTime >= 20) {
        lastRFTime = nowMillis;
        updateRadio();
        processCommands();
    }
    
    updateLED();
    
    if (nowMillis - lastDebugTime >= 200) {
        lastDebugTime = nowMillis;
        printDebug();
    }
}
