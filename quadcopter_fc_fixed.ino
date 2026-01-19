/**
 * ============================================================================
 *    QUADCOPTER FC - STABILIZED VERSION WITH FIXES
 * ============================================================================
 * 
 * FIXES APPLIED:
 *   ✓ MOTOR MIXING CORRECTED - Roll sign was inverted
 *   ✓ RR MOTOR TRIM INCREASED - Was too low causing drift
 *   ✓ IMPROVED PID GAINS - Better damping, less oscillation
 *   ✓ RC DIRECTION VERIFIED - Pitch forward = nose down
 *   ✓ BETTER FILTERING - Reduced noise-induced oscillation
 *   ✓ MOTOR SPEED COMPENSATION - Individual motor scaling
 *   ✓ DIAGNOSTIC OUTPUT - Better debugging
 *   ✓ CASCADED PID with derivative-on-measurement
 * 
 * MOTOR LAYOUT (X-configuration, viewed from above):
 *        FRONT
 *   FL(CCW)  FR(CW)
 *       X
 *   RL(CW)   RR(CCW)
 * 
 * COORDINATE SYSTEM:
 *   - Positive Roll = Right side down
 *   - Positive Pitch = Nose down  
 *   - Positive Yaw = Clockwise rotation
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
#define DEBUG_ENABLED       1       // Set to 0 to disable debug output

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
// For RS2205/2300KV motors - these are POWERFUL racing motors!
// Start with lower throttle range for safety

#define ESC_MIN             1000
#define ESC_MAX             2000
#define ESC_IDLE            1100    // Low idle for 2300KV motors
#define ESC_ARM_THR         100     // Increased threshold for arming check
#define ESC_MAX_THROTTLE    1700    // Limited max for safety with powerful motors

// Debug throttle reception
#define DEBUG_THROTTLE      1       // Show received throttle values

// ============================================================================
//                    MOTOR TRIM - ADJUSTED FOR RR DRIFT
// ============================================================================
// These values compensate for motor/ESC/prop differences
// INCREASE trim for motors that spin SLOWER
// RR was spinning too slow - significantly increased

#define TRIM_FL             +30     // Front-left motor trim
#define TRIM_FR             +50     // Front-right motor trim  
#define TRIM_RL             +80     // Rear-left motor trim
#define TRIM_RR             +120    // Rear-right - INCREASED (was +60, motor was slow)

// Pitch trim for CG offset (if drone tilts forward/back at hover)
#define PITCH_TRIM_FRONT    +10     // Add to front motors
#define PITCH_TRIM_REAR     -10     // Add to rear motors

// Motor speed scaling factors (1.0 = normal, >1.0 = boost weak motor)
// If RR is still slow after trim, increase this
#define MOTOR_SCALE_FL      1.00f
#define MOTOR_SCALE_FR      1.00f
#define MOTOR_SCALE_RL      1.00f
#define MOTOR_SCALE_RR      1.05f   // 5% boost to RR motor

// ============================================================================
//                    SAFETY LIMITS
// ============================================================================
#define MAX_TILT_ANGLE      60.0f
#define EMERGENCY_ANGLE     50.0f   // Warning at 50 degrees
#define MAX_GYRO_RATE       400.0f  // Max rotation rate deg/s

// ============================================================================
//             CASCADED PID GAINS - FOR RS2205/2300KV MOTORS
// ============================================================================
// 
// RS2205/2300KV motors are VERY responsive - need LOWER gains!
// 
// TUNING NOTES:
//   - If oscillating: REDUCE RATE_KP, INCREASE RATE_KD
//   - If sluggish: INCREASE ANGLE_KP (slowly)
//   - If drifting: Add small ANGLE_KI
//   - If vibrating: REDUCE all gains, increase filtering
//

// OUTER LOOP - Angle PID (angle error → target rate)
#define ANGLE_ROLL_KP       2.0f    // Low for powerful motors
#define ANGLE_ROLL_KI       0.01f   // Small I for drift correction
#define ANGLE_ROLL_KD       0.0f    // Inner loop handles D

#define ANGLE_PITCH_KP      2.0f
#define ANGLE_PITCH_KI      0.01f
#define ANGLE_PITCH_KD      0.0f

// INNER LOOP - Rate PID (rate error → motor output)
// LOWER gains for 2300KV motors - they respond fast!
#define RATE_ROLL_KP        0.25f   // Low for powerful motors
#define RATE_ROLL_KI        0.0f    // Keep at 0 to prevent oscillation
#define RATE_ROLL_KD        0.020f  // Damping

#define RATE_PITCH_KP       0.25f
#define RATE_PITCH_KI       0.0f
#define RATE_PITCH_KD       0.020f

// YAW (single loop)
#define PID_YAW_KP          0.8f    // Low for powerful motors
#define PID_YAW_KI          0.005f  // Small I for yaw hold
#define PID_YAW_KD          0.0f

// PID limits
#define ANGLE_I_MAX         30.0f   // Max angle integral (deg*sec)
#define RATE_I_MAX          50.0f   // Max rate integral
#define RATE_OUTPUT_MAX     250.0f  // Max motor adjustment from rate PID
#define ANGLE_RATE_MAX      120.0f  // Max target rate from angle loop (deg/sec)

// ============================================================================
//                    ANTI-OSCILLATION FILTERING
// ============================================================================
// Setpoint filter - smooths stick input
#define SETPOINT_LPF_ALPHA  0.2f    // Reduced from 0.3 - smoother

// D-term lowpass - removes HF noise
#define D_TERM_LPF_ALPHA    0.15f   // Reduced from 0.2 - more filtering

// Output rate limiter
#define OUTPUT_RATE_LIMIT   20.0f   // Reduced from 30 - gentler corrections

// ============================================================================
//                    GENERAL FILTERING
// ============================================================================
#define GYRO_LPF_ALPHA      0.4f    // More filtering (was 0.5)
#define ACCEL_LPF_ALPHA     0.2f    // More filtering (was 0.3)
#define MOTOR_LPF_ALPHA     0.3f    // More smoothing (was 0.4)
#define RC_LPF_ALPHA        0.5f    // Slightly more filtering
#define POT_LPF_ALPHA       0.1f

#define RC_DEADBAND         25
#define GYRO_DEADBAND       0.5f    // Increased from 0.3

// ============================================================================
//                    COMPLEMENTARY FILTER
// ============================================================================
#define COMP_FILTER_ALPHA   0.98f

// ============================================================================
//                    AXIS CONFIGURATION
// ============================================================================
// These inversions depend on your IMU mounting orientation
// Adjust if roll/pitch directions are wrong

#define PITCH_INVERT    -1.0f       // Invert pitch angle
#define ROLL_INVERT      1.0f       // Normal roll angle

// RC stick inversions - adjust to match your transmitter
// When stick goes UP/RIGHT, the value should be POSITIVE
#define RC_ROLL_INVERT      -1.0f   // Invert if roll stick backwards
#define RC_PITCH_INVERT     1.0f    // Normal pitch direction
#define RC_YAW_INVERT       -1.0f   // Invert if yaw backwards

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

// IMU health tracking
uint32_t lastIMURead = 0;
uint32_t imuReadCount = 0;
bool imuHealthy = true;

// ============================================================================
//                  CASCADED PID STATE
// ============================================================================
struct CascadedPIDState {
    // Outer loop (angle) state
    float angleIntegral;
    float prevAngle;
    
    // Inner loop (rate) state
    float rateIntegral;
    float prevRate;
    float prevDterm;
    
    // Setpoint filtering
    float filteredSetpoint;
    
    // Output rate limiting
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

// Simple PID state for yaw
struct SimplePIDState {
    float integral;
    float prevError;
    float prevMeasurement;  // For derivative on measurement
    float prevDerivative;
    
    void reset() {
        integral = 0;
        prevError = 0;
        prevMeasurement = 0;
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

// Timing - 500Hz main loop
uint32_t lastIMUTime = 0;
uint32_t lastRFTime = 0;
uint32_t lastDebugTime = 0;
uint32_t lastSafetyCheck = 0;
uint32_t loopCount = 0;

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

// Rate limiter - prevents sudden changes
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
    
    // Immediately stop all motors
    escFL.writeMicroseconds(ESC_MIN);
    escFR.writeMicroseconds(ESC_MIN);
    escRL.writeMicroseconds(ESC_MIN);
    escRR.writeMicroseconds(ESC_MIN);
    
    motorFL = motorFR = motorRL = motorRR = ESC_MIN;
    motorFL_f = motorFR_f = motorRL_f = motorRR_f = ESC_MIN;
    
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
    
    float totalTilt = sqrt(roll * roll + pitch * pitch);
    
    // Emergency disarm at extreme angle
    if (totalTilt > MAX_TILT_ANGLE) {
        emergencyDisarm("TILT >60 DEG");
        return;
    }
    
    // Warning beep at high angle
    if (totalTilt > EMERGENCY_ANGLE) {
        if (millis() - lastSafetyCheck > 200) {
            beep(2000, 50);
            lastSafetyCheck = millis();
        }
    }
    
    // Check rotation rate
    float maxRate = max(abs(rollRate), max(abs(pitchRate), abs(yawRate)));
    if (maxRate > MAX_GYRO_RATE) {
        emergencyDisarm("EXTREME ROTATION");
        return;
    }
    
    // Check IMU health
    if (!imuHealthy) {
        emergencyDisarm("IMU FAILURE");
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
    
    // Reset all offsets
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
    
    // Iterative calibration
    for (int iter = 0; iter < 6; iter++) {  // Increased iterations
        Serial.print(F("Pass ")); Serial.print(iter + 1); Serial.print(F("/6"));
        
        int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
        int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
        
        for (int i = 0; i < 1000; i++) {
            int16_t ax, ay, az, gx, gy, gz;
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            
            ax_sum += ax;
            ay_sum += ay;
            az_sum += az;
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
            
            delayMicroseconds(1000);
        }
        
        int16_t ax_avg = ax_sum / 1000;
        int16_t ay_avg = ay_sum / 1000;
        int16_t az_avg = az_sum / 1000;
        int16_t gx_avg = gx_sum / 1000;
        int16_t gy_avg = gy_sum / 1000;
        int16_t gz_avg = gz_sum / 1000;
        
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
        
        Serial.print(F(" - Gyro err: "));
        Serial.println(abs(gx_avg) + abs(gy_avg) + abs(gz_avg));
        
        delay(50);
    }
    
    calibration.axOffset = ax_off;
    calibration.ayOffset = ay_off;
    calibration.azOffset = az_off;
    calibration.gxOffset = gx_off;
    calibration.gyOffset = gy_off;
    calibration.gzOffset = gz_off;
    
    // Measure level trim
    Serial.println(F("Measuring level trim..."));
    delay(100);
    
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    
    for (int i = 0; i < 500; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        ax_sum += ax; 
        ay_sum += ay; 
        az_sum += az;
        delayMicroseconds(1000);
    }
    
    float ax_avg = ax_sum / 500.0f;
    float ay_avg = ay_sum / 500.0f;
    float az_avg = az_sum / 500.0f;
    
    calibration.rollOffset = atan2(ay_avg, az_avg) * 57.2958f;
    calibration.pitchOffset = atan2(-ax_avg, sqrt(ay_avg * ay_avg + az_avg * az_avg)) * 57.2958f;
    
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
    // Track timing for IMU health
    uint32_t startRead = micros();
    
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
    
    uint32_t readTime = micros() - startRead;
    
    // Check for I2C timeout (should be < 1ms normally)
    if (readTime > 5000) {
        imuHealthy = false;
        Serial.println(F("WARNING: IMU read slow!"));
    } else {
        imuHealthy = true;
    }
    
    imuReadCount++;
    lastIMURead = millis();
    
    // Convert to degrees per second (±500 dps range)
    float gx_dps = gx_raw / 65.5f;
    float gy_dps = gy_raw / 65.5f;
    float gz_dps = gz_raw / 65.5f;
    
    // Convert to g (±2g range)
    float ax_g = ax_raw / 16384.0f;
    float ay_g = ay_raw / 16384.0f;
    float az_g = az_raw / 16384.0f;
    
    // Apply low-pass filter
    gyroX = lowPassFilter(gyroX, gx_dps, GYRO_LPF_ALPHA);
    gyroY = lowPassFilter(gyroY, gy_dps, GYRO_LPF_ALPHA);
    gyroZ = lowPassFilter(gyroZ, gz_dps, GYRO_LPF_ALPHA);
    
    accelX = lowPassFilter(accelX, ax_g, ACCEL_LPF_ALPHA);
    accelY = lowPassFilter(accelY, ay_g, ACCEL_LPF_ALPHA);
    accelZ = lowPassFilter(accelZ, az_g, ACCEL_LPF_ALPHA);
    
    // Update gyro bias when disarmed (adaptive calibration)
    if (flightState == DISARMED && !biasLocked) {
        gyroBiasX = lowPassFilter(gyroBiasX, gyroX, 0.005f);  // Slower adaptation
        gyroBiasY = lowPassFilter(gyroBiasY, gyroY, 0.005f);
        gyroBiasZ = lowPassFilter(gyroBiasZ, gyroZ, 0.005f);
    }
    
    // Compensate for bias
    float gyroX_comp = gyroX - gyroBiasX;
    float gyroY_comp = gyroY - gyroBiasY;
    float gyroZ_comp = gyroZ - gyroBiasZ;
    
    // Apply deadband to reduce noise
    rollRate = applyDeadband(gyroX_comp, GYRO_DEADBAND);
    pitchRate = applyDeadband(gyroY_comp, GYRO_DEADBAND);
    yawRate = applyDeadband(gyroZ_comp, GYRO_DEADBAND);
}

// ============================================================================
//                    UPDATE ANGLES
// ============================================================================
void updateAngles(float dt) {
    // Calculate angles from accelerometer
    float accelRoll = atan2(accelY, accelZ) * 57.2958f;
    float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958f;
    
    // Apply calibration offset
    accelRoll -= calibration.rollOffset;
    accelPitch -= calibration.pitchOffset;
    
    // Apply axis inversion
    accelRoll *= ROLL_INVERT;
    accelPitch *= PITCH_INVERT;
    
    // Apply axis inversion to rates
    float pitchRateAdj = pitchRate * PITCH_INVERT;
    float rollRateAdj = rollRate * ROLL_INVERT;
    
    // Complementary filter - trust gyro short-term, accel long-term
    roll = COMP_FILTER_ALPHA * (roll + rollRateAdj * dt) + 
           (1.0f - COMP_FILTER_ALPHA) * accelRoll;
    pitch = COMP_FILTER_ALPHA * (pitch + pitchRateAdj * dt) + 
            (1.0f - COMP_FILTER_ALPHA) * accelPitch;
    
    // Integrate yaw (no absolute reference)
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
            
            // Decode potentiometer data
            uint8_t gainVal = (packet.auxData >> 4) & 0x0F;
            uint8_t angleVal = packet.auxData & 0x0F;
            
            float targetGain = map(gainVal, 0, 15, 50, 150) / 100.0f;
            float targetAngle = map(angleVal, 0, 15, 15, 35);
            
            gainMultiplier = lowPassFilter(gainMultiplier, targetGain, POT_LPF_ALPHA);
            maxAngle = lowPassFilter(maxAngle, targetAngle, POT_LPF_ALPHA);
            
            // Decode flight mode
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
    
    // Failsafe on radio loss
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
    
    // Get raw commands from packet
    // rxPacket.throttle should be 0-1000 from remote
    float rawThrottle = rxPacket.throttle;
    float rawRoll = applyDeadband(rxPacket.roll * RC_ROLL_INVERT, RC_DEADBAND);
    float rawPitch = applyDeadband(rxPacket.pitch * RC_PITCH_INVERT, RC_DEADBAND);
    float rawYaw = applyDeadband(rxPacket.yaw * RC_YAW_INVERT, RC_DEADBAND);
    
    // Debug: Print raw throttle from packet occasionally
    static uint32_t lastThrDebug = 0;
    if (millis() - lastThrDebug > 1000) {
        lastThrDebug = millis();
        Serial.print(F("RX Packet THR: ")); Serial.print(rxPacket.throttle);
        Serial.print(F(" -> filtered: ")); Serial.println((int)throttleCmd);
    }
    
    // Apply low-pass filter for smooth control
    // Use faster filter for throttle (0.3) for quicker response
    throttleCmd = lowPassFilter(throttleCmd, rawThrottle, 0.4f);
    rollCmd = lowPassFilter(rollCmd, rawRoll, RC_LPF_ALPHA);
    pitchCmd = lowPassFilter(pitchCmd, rawPitch, RC_LPF_ALPHA);
    yawCmd = lowPassFilter(yawCmd, rawYaw, RC_LPF_ALPHA);
    
    // Arm/disarm logic
    bool armSwitch = rxPacket.switches & (1 << SW_ARM);
    
    if (armSwitch && !prevArm && flightState == DISARMED) {
        bool throttleLow = (throttleCmd < ESC_ARM_THR);
        bool isLevel = (abs(roll) < 8.0f && abs(pitch) < 8.0f);  // Relaxed from 5 deg
        bool calibOK = calibration.valid;
        
        if (throttleLow && isLevel && calibOK && radioConnected) {
            flightState = ARMED;
            biasLocked = true;
            
            // Reset all PID states
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
//           CASCADED PID - IMPROVED IMPLEMENTATION
// ============================================================================
float calculateCascadedPID(float targetAngle, float currentAngle, float currentRate, 
                           float dt, CascadedPIDState& state,
                           float angleKp, float angleKi, 
                           float rateKp, float rateKi, float rateKd) {
    
    // ==================== SETPOINT FILTERING ====================
    state.filteredSetpoint = lowPassFilter(state.filteredSetpoint, targetAngle, SETPOINT_LPF_ALPHA);
    float smoothedTarget = state.filteredSetpoint;
    
    // ==================== OUTER LOOP (ANGLE) ====================
    float angleError = smoothedTarget - currentAngle;
    
    // P term
    float targetRate = angleKp * gainMultiplier * angleError;
    
    // I term with anti-windup
    if (abs(angleError) > 0.5f && angleKi > 0) {
        state.angleIntegral += angleError * dt;
        state.angleIntegral = constrainFloat(state.angleIntegral, -ANGLE_I_MAX, ANGLE_I_MAX);
        targetRate += angleKi * gainMultiplier * state.angleIntegral;
    } else if (abs(angleError) < 1.0f) {
        // Decay integral when near target
        state.angleIntegral *= 0.99f;
    }
    
    // Limit target rate
    targetRate = constrainFloat(targetRate, -ANGLE_RATE_MAX, ANGLE_RATE_MAX);
    
    // ==================== INNER LOOP (RATE) ====================
    float rateError = targetRate - currentRate;
    
    // P term
    float rateP = rateKp * gainMultiplier * rateError;
    
    // I term (usually keep at 0)
    float rateI = 0;
    if (rateKi > 0) {
        state.rateIntegral += rateError * dt;
        state.rateIntegral = constrainFloat(state.rateIntegral, -RATE_I_MAX, RATE_I_MAX);
        rateI = rateKi * gainMultiplier * state.rateIntegral;
    }
    
    // D term: DERIVATIVE ON MEASUREMENT (not error!)
    // This prevents derivative kick when setpoint changes
    float rateDelta = (currentRate - state.prevRate) / dt;
    state.prevRate = currentRate;
    
    // Filter D term heavily to remove noise
    float rateD_raw = -rateKd * gainMultiplier * rateDelta;
    state.prevDterm = lowPassFilter(state.prevDterm, rateD_raw, D_TERM_LPF_ALPHA);
    float rateD = state.prevDterm;
    
    // Combine rate PID
    float output = rateP + rateI + rateD;
    
    // ==================== OUTPUT RATE LIMITING ====================
    output = rateLimitChange(state.prevOutput, output, OUTPUT_RATE_LIMIT);
    state.prevOutput = output;
    
    return constrainFloat(output, -RATE_OUTPUT_MAX, RATE_OUTPUT_MAX);
}

// Improved yaw PID with derivative on measurement
float calculateYawPID(float targetRate, float currentRate, float dt, SimplePIDState& state) {
    float error = targetRate - currentRate;
    
    // P term
    float P = PID_YAW_KP * gainMultiplier * error;
    
    // I term with deadband
    if (abs(error) > 1.0f) {
        state.integral += error * dt;
    } else {
        state.integral *= 0.98f;  // Decay when near target
    }
    state.integral = constrainFloat(state.integral, -50.0f, 50.0f);
    float I = PID_YAW_KI * gainMultiplier * state.integral;
    
    // D term on measurement
    float rateDelta = (currentRate - state.prevMeasurement) / dt;
    state.prevMeasurement = currentRate;
    
    float D_raw = -PID_YAW_KD * gainMultiplier * rateDelta;
    state.prevDerivative = lowPassFilter(state.prevDerivative, D_raw, D_TERM_LPF_ALPHA);
    float D = state.prevDerivative;
    
    state.prevError = error;
    
    float output = P + I + D;
    return constrainFloat(output, -RATE_OUTPUT_MAX, RATE_OUTPUT_MAX);
}

// ============================================================================
//                         UPDATE PID
// ============================================================================
void updatePID(float dt) {
    if (flightState != ARMED) {
        rollPID = pitchPID = yawPID = 0;
        return;
    }
    
    // Apply axis inversion to rates for PID
    float adjRollRate = rollRate * ROLL_INVERT;
    float adjPitchRate = pitchRate * PITCH_INVERT;
    
    if (flightMode == FMODE_ANGLE) {
        // ANGLE MODE: Cascaded PID for stability
        float targetRoll = (rollCmd / 500.0f) * maxAngle;
        float targetPitch = (pitchCmd / 500.0f) * maxAngle;
        
        rollPID = calculateCascadedPID(targetRoll, roll, adjRollRate, dt, pidRollState,
                                       ANGLE_ROLL_KP, ANGLE_ROLL_KI,
                                       RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD);
        
        pitchPID = calculateCascadedPID(targetPitch, pitch, adjPitchRate, dt, pidPitchState,
                                        ANGLE_PITCH_KP, ANGLE_PITCH_KI,
                                        RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD);
        
    } else if (flightMode == FMODE_HORIZON) {
        // HORIZON MODE: Blend angle and rate
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
        // ACRO MODE: Direct rate control
        rollPID = rollCmd * 0.4f;
        pitchPID = pitchCmd * 0.4f;
    }
    
    // Yaw control
    float targetYawRate = (yawCmd / 500.0f) * 150.0f;  // Max 150 deg/s yaw rate
    yawPID = calculateYawPID(targetYawRate, yawRate, dt, pidYawState);
}

// ============================================================================
//                         UPDATE MOTORS
// ============================================================================
void updateMotors() {
    if (flightState != ARMED) {
        motorFL = motorFR = motorRL = motorRR = ESC_MIN;
        motorFL_f = motorFR_f = motorRL_f = motorRR_f = ESC_MIN;
    } else {
        // Map throttle to motor range
        int16_t baseThr = map(throttleCmd, 0, 1000, ESC_IDLE, ESC_MAX_THROTTLE) - ESC_MIN;
        
        // ================================================================
        // CORRECTED MOTOR MIXING FOR X-QUAD
        // ================================================================
        // Positive roll = right side down = MORE power to RIGHT motors (FR, RR)
        // Positive pitch = nose down = MORE power to FRONT motors (FL, FR)
        // Positive yaw = clockwise = MORE power to CCW motors (FL, RR)
        //
        // Previous code had roll signs INVERTED which caused drift!
        
        int16_t fl = baseThr - (int16_t)rollPID + (int16_t)pitchPID + (int16_t)yawPID;
        int16_t fr = baseThr + (int16_t)rollPID + (int16_t)pitchPID - (int16_t)yawPID;
        int16_t rl = baseThr - (int16_t)rollPID - (int16_t)pitchPID - (int16_t)yawPID;
        int16_t rr = baseThr + (int16_t)rollPID - (int16_t)pitchPID + (int16_t)yawPID;
        
        // Apply motor-specific trims
        fl += TRIM_FL + PITCH_TRIM_FRONT;
        fr += TRIM_FR + PITCH_TRIM_FRONT;
        rl += TRIM_RL + PITCH_TRIM_REAR;
        rr += TRIM_RR + PITCH_TRIM_REAR;
        
        // Apply motor scaling for weak motors
        float fl_scaled = fl * MOTOR_SCALE_FL;
        float fr_scaled = fr * MOTOR_SCALE_FR;
        float rl_scaled = rl * MOTOR_SCALE_RL;
        float rr_scaled = rr * MOTOR_SCALE_RR;
        
        // Add ESC_MIN and constrain
        uint16_t tFL = constrain((int16_t)fl_scaled + ESC_MIN, ESC_MIN, ESC_MAX);
        uint16_t tFR = constrain((int16_t)fr_scaled + ESC_MIN, ESC_MIN, ESC_MAX);
        uint16_t tRL = constrain((int16_t)rl_scaled + ESC_MIN, ESC_MIN, ESC_MAX);
        uint16_t tRR = constrain((int16_t)rr_scaled + ESC_MIN, ESC_MIN, ESC_MAX);
        
        // Smooth motor outputs to prevent jitter
        motorFL_f = lowPassFilter(motorFL_f, tFL, MOTOR_LPF_ALPHA);
        motorFR_f = lowPassFilter(motorFR_f, tFR, MOTOR_LPF_ALPHA);
        motorRL_f = lowPassFilter(motorRL_f, tRL, MOTOR_LPF_ALPHA);
        motorRR_f = lowPassFilter(motorRR_f, tRR, MOTOR_LPF_ALPHA);
        
        motorFL = (uint16_t)motorFL_f;
        motorFR = (uint16_t)motorFR_f;
        motorRL = (uint16_t)motorRL_f;
        motorRR = (uint16_t)motorRR_f;
    }
    
    // Write to ESCs
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
    if (flightState == ARMED) interval = 0;  // Solid on
    else if (flightState == EMERGENCY) interval = 50;  // Fast blink
    else if (!calibration.valid) interval = 200;  // Medium blink
    else interval = 500;  // Slow blink
    
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
#if DEBUG_ENABLED
    // State
    if (flightState == ARMED) Serial.print(F("ARM "));
    else if (flightState == EMERGENCY) Serial.print(F("EMG "));
    else Serial.print(F("DIS "));
    
#if DEBUG_THROTTLE
    // Show received throttle command
    Serial.print(F("| THR:")); Serial.print((int)throttleCmd);
    
    // Throttle bar visualization
    Serial.print(F(" ["));
    int bars = (int)throttleCmd / 100;
    for (int i = 0; i < 10; i++) {
        Serial.print(i < bars ? '#' : '-');
    }
    Serial.print(F("] "));
#endif
    
    // Angles
    Serial.print(F("| R:")); Serial.print(roll, 1);
    Serial.print(F(" P:")); Serial.print(pitch, 1);
    
    // Motor values
    Serial.print(F(" | M:")); Serial.print(motorFL);
    Serial.print(F(",")); Serial.print(motorFR);
    Serial.print(F(",")); Serial.print(motorRL);
    Serial.print(F(",")); Serial.print(motorRR);
    
    // Radio status
    if (!radioConnected) {
        Serial.print(F(" [NO RADIO]"));
    } else {
        Serial.print(F(" [RF OK]"));
    }
    
    // Loop rate
    Serial.print(F(" Hz:")); Serial.print(loopCount * 5);
    loopCount = 0;
    
    Serial.println();
#endif
}

// ============================================================================
//                              SETUP
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 1000);
    
    Serial.println(F("\n=========================================="));
    Serial.println(F("  QUADCOPTER FC - STABILIZED VERSION"));
    Serial.println(F("  Fixed motor mixing + RR trim"));
    Serial.println(F("==========================================\n"));
    
    // Initialize pins
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    beepPattern(3, 2000, 100, 100);
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  // 400kHz for faster reads
    
    // Initialize MPU6050
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
    
    // Configure MPU6050
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);   // ±500 deg/s
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // ±2g
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);              // 42Hz low-pass
    
    // Calibrate
    calibrateMPU6050();
    
    // Initialize radio
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
    
    // Initialize ESCs
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
    
    // Print configuration
    Serial.println(F("\n*** SYSTEM READY ***"));
    Serial.println(F("\nMotor trims:"));
    Serial.print(F("  FL: ")); Serial.println(TRIM_FL);
    Serial.print(F("  FR: ")); Serial.println(TRIM_FR);
    Serial.print(F("  RL: ")); Serial.println(TRIM_RL);
    Serial.print(F("  RR: ")); Serial.println(TRIM_RR);
    Serial.print(F("  RR scale: ")); Serial.println(MOTOR_SCALE_RR);
    
    Serial.println(F("\nPID gains (with multiplier 1.0):"));
    Serial.print(F("  Angle P: ")); Serial.println(ANGLE_ROLL_KP);
    Serial.print(F("  Rate P: ")); Serial.println(RATE_ROLL_KP);
    Serial.print(F("  Rate D: ")); Serial.println(RATE_ROLL_KD);
    
    Serial.println(F("\nFIXES APPLIED:"));
    Serial.println(F("  1. Motor mixing roll sign CORRECTED"));
    Serial.println(F("  2. RR trim increased (was +60, now +120)"));
    Serial.println(F("  3. RR motor 5% boost scaling"));
    Serial.println(F("  4. Reduced PID aggression"));
    Serial.println(F("  5. Improved filtering"));
    
    Serial.println(F("\nIf still drifting to RR:"));
    Serial.println(F("  - Increase TRIM_RR more"));
    Serial.println(F("  - Increase MOTOR_SCALE_RR"));
    Serial.println(F("  - Check RR motor/ESC/prop"));
    
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
    
    // Main control loop at 500Hz (every 2000us)
    if (nowMicros - lastIMUTime >= 2000) {
        float dt = (nowMicros - lastIMUTime) / 1000000.0f;
        lastIMUTime = nowMicros;
        loopCount++;
        
        readIMU();
        updateAngles(dt);
        updatePID(dt);
        updateMotors();
        checkSafety();
    }
    
    // Radio at 50Hz
    if (nowMillis - lastRFTime >= 20) {
        lastRFTime = nowMillis;
        updateRadio();
        processCommands();
    }
    
    // LED update
    updateLED();
    
    // Debug at 5Hz
    if (nowMillis - lastDebugTime >= 200) {
        lastDebugTime = nowMillis;
        printDebug();
    }
}
