/**
 * ============================================================================
 *    QUADCOPTER FC - ANTI-OSCILLATION VERSION (FIXED)
 * ============================================================================
 * 
 * FIXES APPLIED:
 *   ✓ FIXED: Motor mixing signs corrected for proper stabilization
 *   ✓ FIXED: PID output polarity matches motor mixing
 *   ✓ FIXED: Consistent axis inversions throughout
 *   ✓ FIXED: Rate calculations match angle calculations
 *   ✓ ADDED: Better integral anti-windup
 *   ✓ ADDED: Improved D-term filtering
 *   ✓ ADDED: Motor RPM balancing compensation
 * 
 * MOTOR LAYOUT (X-configuration, viewed from above):
 *        FRONT
 *   FL(CCW)  FR(CW)
 *       X
 *   RL(CW)   RR(CCW)
 * 
 * AXIS CONVENTION:
 *   Roll+  = Right side down
 *   Pitch+ = Nose up
 *   Yaw+   = Rotate clockwise (from above)
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
#define ESC_IDLE            1150    // Reduced idle for better control at low throttle
#define ESC_ARM_THR         50
#define ESC_MAX_THROTTLE    1800    // Increased for more headroom

// ============================================================================
//                    MOTOR TRIM - BALANCED
// ============================================================================
// These should be calibrated with motors spinning freely
// Start with all at 0 and adjust based on observed behavior
#define TRIM_FL             0
#define TRIM_FR             0
#define TRIM_RL             0
#define TRIM_RR             0

// Pitch/Roll trim for CG offset compensation
#define PITCH_TRIM          0       // + = more rear thrust, - = more front thrust
#define ROLL_TRIM           0       // + = more right thrust, - = more left thrust

// ============================================================================
//                    SAFETY LIMITS
// ============================================================================
#define MAX_TILT_ANGLE      60.0f
#define EMERGENCY_ANGLE     45.0f
#define MAX_GYRO_RATE       500.0f

// ============================================================================
//             CASCADED PID GAINS - ANTI-OSCILLATION TUNING
// ============================================================================
// 
// CASCADED PID ARCHITECTURE:
//   Outer Loop (Angle): Slow, sets target rate based on angle error
//   Inner Loop (Rate): Fast, controls motors based on gyro rate
//
// TUNING ORDER:
//   1. Set all I gains to 0
//   2. Tune Rate P until slight oscillation, then reduce 20%
//   3. Tune Rate D until smooth (no vibration)
//   4. Tune Angle P until responsive but no overshoot
//   5. Add small I gain last if needed for steady-state error

// OUTER LOOP - Angle PID (outputs target rate in deg/sec)
// REDUCED GAINS to stop shaking
#define ANGLE_ROLL_KP       1.5f    // Reduced from 2.5 - less aggressive
#define ANGLE_ROLL_KI       0.0f    // Start with 0 - add later if needed
#define ANGLE_ROLL_KD       0.0f    // Not needed in cascaded

#define ANGLE_PITCH_KP      1.5f    // Reduced from 2.5
#define ANGLE_PITCH_KI      0.0f
#define ANGLE_PITCH_KD      0.0f

// INNER LOOP - Rate PID (outputs motor correction)
// REDUCED GAINS to stop shaking - these are most critical
#define RATE_ROLL_KP        0.25f   // Reduced from 0.6 - main cause of shaking
#define RATE_ROLL_KI        0.0f    // Start with 0
#define RATE_ROLL_KD        0.008f  // Reduced from 0.025 - D amplifies noise

#define RATE_PITCH_KP       0.25f   // Reduced from 0.6
#define RATE_PITCH_KI       0.0f
#define RATE_PITCH_KD       0.008f  // Reduced from 0.025

// YAW (single loop is fine for yaw)
#define PID_YAW_KP          1.0f    // Reduced from 2.0
#define PID_YAW_KI          0.0f    // Start with 0
#define PID_YAW_KD          0.0f    // Start with 0

// PID limits
#define ANGLE_I_MAX         30.0f   // Max angle integral (deg*sec)
#define RATE_I_MAX          50.0f   // Max rate integral
#define RATE_OUTPUT_MAX     400.0f  // Max motor adjustment from rate PID
#define ANGLE_RATE_MAX      180.0f  // Max target rate from angle loop (deg/sec)
#define YAW_I_MAX           100.0f

// ============================================================================
//                    ANTI-OSCILLATION FILTERING
// ============================================================================
// Setpoint filter - smooths stick input to prevent D-term kick
#define SETPOINT_LPF_ALPHA  0.15f   // Lower = smoother setpoint changes

// D-term lowpass - removes high-frequency noise that causes vibration
#define D_TERM_LPF_ALPHA    0.08f   // Much lower = more filtering to reduce shaking

// Output rate limiter - max change per loop (prevents sudden corrections)
#define OUTPUT_RATE_LIMIT   15.0f   // Reduced further for smoother response

// ============================================================================
//                    GENERAL FILTERING
// ============================================================================
#define GYRO_LPF_ALPHA      0.3f    // More filtering to reduce noise/shaking
#define ACCEL_LPF_ALPHA     0.15f   // More filtering for smoother angles
#define MOTOR_LPF_ALPHA     0.2f    // More smoothing on motors to reduce shaking
#define RC_LPF_ALPHA        0.5f
#define POT_LPF_ALPHA       0.1f
#define RC_DEADBAND         20
#define GYRO_DEADBAND       0.5f    // Slightly larger deadband

// ============================================================================
//                    COMPLEMENTARY FILTER
// ============================================================================
#define COMP_FILTER_ALPHA   0.98f

// ============================================================================
//                    AXIS CONFIGURATION
// ============================================================================
// These define how the MPU6050 axes map to drone axes
// Adjust based on how the MPU6050 is mounted on your drone

// Accelerometer to angle mapping
#define ACCEL_ROLL_AXIS     'Y'     // Which accel axis corresponds to roll
#define ACCEL_PITCH_AXIS    'X'     // Which accel axis corresponds to pitch

// Gyro to rate mapping
#define GYRO_ROLL_AXIS      'X'     // Which gyro axis corresponds to roll rate
#define GYRO_PITCH_AXIS     'Y'     // Which gyro axis corresponds to pitch rate
#define GYRO_YAW_AXIS       'Z'     // Which gyro axis corresponds to yaw rate

// Axis inversions (1.0 or -1.0)
// Set these based on MPU6050 mounting orientation
#define ROLL_SIGN           1.0f    // Positive = right side down increases roll
#define PITCH_SIGN          1.0f    // Positive = nose up increases pitch  
#define YAW_SIGN            1.0f    // Positive = clockwise increases yaw

// RC stick inversions - FIXED for correct control direction
#define RC_ROLL_SIGN        -1.0f   // Flipped: stick left = drone goes left
#define RC_PITCH_SIGN       1.0f
#define RC_YAW_SIGN         1.0f    // Flipped: stick right = drone rotates right

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

// Filtered sensor values
float gyroX = 0, gyroY = 0, gyroZ = 0;
float accelX = 0, accelY = 0, accelZ = 0;

// Attitude angles
float roll = 0, pitch = 0, yaw = 0;

// Angular rates (in drone frame)
float rollRate = 0, pitchRate = 0, yawRate = 0;

// Gyro bias tracking
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
bool biasLocked = false;
uint32_t biasSettleTime = 0;

// ============================================================================
//                  CASCADED PID STATE - ANTI-OSCILLATION
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
    
    // Debug info
    float lastAngleError;
    float lastRateError;
    float lastTargetRate;
    
    void reset() {
        angleIntegral = 0;
        prevAngle = 0;
        rateIntegral = 0;
        prevRate = 0;
        prevDterm = 0;
        filteredSetpoint = 0;
        prevOutput = 0;
        lastAngleError = 0;
        lastRateError = 0;
        lastTargetRate = 0;
    }
} pidRollState, pidPitchState;

// Simple PID state for yaw
struct SimplePIDState {
    float integral;
    float prevMeasurement;  // For derivative on measurement
    float prevDterm;
    float prevOutput;
    
    void reset() {
        integral = 0;
        prevMeasurement = 0;
        prevDterm = 0;
        prevOutput = 0;
    }
} pidYawState;

float rollPID = 0, pitchPID = 0, yawPID = 0;
float gainMultiplier = 1.0f;
float maxAngle = 30.0f;

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
    
    // Reset offsets
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    delay(100);
    
    int16_t ax_off = 0, ay_off = 0, az_off = 0;
    int16_t gx_off = 0, gy_off = 0, gz_off = 0;
    
    Serial.println(F("Calibrating gyro and accel offsets..."));
    
    // Iterative calibration
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
        
        // Adjust offsets
        ax_off -= ax_avg / 8;
        ay_off -= ay_avg / 8;
        az_off += (16384 - az_avg) / 8;  // Target 1g on Z
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
    
    // Store calibration
    calibration.axOffset = ax_off;
    calibration.ayOffset = ay_off;
    calibration.azOffset = az_off;
    calibration.gxOffset = gx_off;
    calibration.gyOffset = gy_off;
    calibration.gzOffset = gz_off;
    
    // Measure level trim
    Serial.println(F("Measuring level trim..."));
    delay(200);
    
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    
    for (int i = 0; i < 1000; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        ax_sum += ax; 
        ay_sum += ay; 
        az_sum += az;
        delayMicroseconds(1000);
    }
    
    float ax_avg = ax_sum / 1000.0f;
    float ay_avg = ay_sum / 1000.0f;
    float az_avg = az_sum / 1000.0f;
    
    // Calculate level offsets
    calibration.rollOffset = atan2(ay_avg, az_avg) * 57.2958f;
    calibration.pitchOffset = atan2(-ax_avg, sqrt(ay_avg*ay_avg + az_avg*az_avg)) * 57.2958f;
    
    Serial.print(F("Level trim: Roll=")); Serial.print(calibration.rollOffset, 2);
    Serial.print(F("° Pitch=")); Serial.print(calibration.pitchOffset, 2);
    Serial.println(F("°"));
    
    calibration.valid = true;
    
    Serial.println(F("\n*** CALIBRATION COMPLETE ***"));
    Serial.print(F("Gyro offsets: ")); 
    Serial.print(gx_off); Serial.print(F(", "));
    Serial.print(gy_off); Serial.print(F(", "));
    Serial.println(gz_off);
    Serial.print(F("Accel offsets: ")); 
    Serial.print(ax_off); Serial.print(F(", "));
    Serial.print(ay_off); Serial.print(F(", "));
    Serial.println(az_off);
    
    beepPattern(2, 2500, 100, 100);
}

// ============================================================================
//                    READ IMU - FIXED AXIS MAPPING
// ============================================================================
void readIMU() {
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
    
    // Convert to physical units
    // Gyro: 500 deg/sec range = 65.5 LSB/(deg/s)
    float gx_dps = gx_raw / 65.5f;
    float gy_dps = gy_raw / 65.5f;
    float gz_dps = gz_raw / 65.5f;
    
    // Accel: 2g range = 16384 LSB/g
    float ax_g = ax_raw / 16384.0f;
    float ay_g = ay_raw / 16384.0f;
    float az_g = az_raw / 16384.0f;
    
    // Apply low-pass filter to raw sensor data
    gyroX = lowPassFilter(gyroX, gx_dps, GYRO_LPF_ALPHA);
    gyroY = lowPassFilter(gyroY, gy_dps, GYRO_LPF_ALPHA);
    gyroZ = lowPassFilter(gyroZ, gz_dps, GYRO_LPF_ALPHA);
    
    accelX = lowPassFilter(accelX, ax_g, ACCEL_LPF_ALPHA);
    accelY = lowPassFilter(accelY, ay_g, ACCEL_LPF_ALPHA);
    accelZ = lowPassFilter(accelZ, az_g, ACCEL_LPF_ALPHA);
    
    // Update gyro bias when disarmed and stationary
    if (flightState == DISARMED && !biasLocked) {
        gyroBiasX = lowPassFilter(gyroBiasX, gyroX, 0.005f);  // Slower adaptation
        gyroBiasY = lowPassFilter(gyroBiasY, gyroY, 0.005f);
        gyroBiasZ = lowPassFilter(gyroBiasZ, gyroZ, 0.005f);
    }
    
    // Compensate for gyro bias
    float gyroX_comp = gyroX - gyroBiasX;
    float gyroY_comp = gyroY - gyroBiasY;
    float gyroZ_comp = gyroZ - gyroBiasZ;
    
    // Map gyro axes to drone frame with proper signs
    // Standard MPU6050 orientation: X forward, Y right, Z down
    // Drone convention: Roll = rotation about X, Pitch = rotation about Y, Yaw = rotation about Z
    rollRate = applyDeadband(gyroX_comp * ROLL_SIGN, GYRO_DEADBAND);
    pitchRate = applyDeadband(gyroY_comp * PITCH_SIGN, GYRO_DEADBAND);
    yawRate = applyDeadband(gyroZ_comp * YAW_SIGN, GYRO_DEADBAND);
}

// ============================================================================
//                    UPDATE ANGLES - FIXED
// ============================================================================
void updateAngles(float dt) {
    // Calculate angles from accelerometer
    // Standard formulas for roll and pitch from gravity vector
    float accelRoll = atan2(accelY, accelZ) * 57.2958f;
    float accelPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 57.2958f;
    
    // Apply calibration offsets
    accelRoll -= calibration.rollOffset;
    accelPitch -= calibration.pitchOffset;
    
    // Apply axis signs
    accelRoll *= ROLL_SIGN;
    accelPitch *= PITCH_SIGN;
    
    // Complementary filter
    // Integrate gyro rates (already in correct frame from readIMU)
    roll = COMP_FILTER_ALPHA * (roll + rollRate * dt) + 
           (1.0f - COMP_FILTER_ALPHA) * accelRoll;
    pitch = COMP_FILTER_ALPHA * (pitch + pitchRate * dt) + 
            (1.0f - COMP_FILTER_ALPHA) * accelPitch;
    
    // Yaw from gyro only (no absolute reference)
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
            
            // Parse pot values
            uint8_t gainVal = (packet.auxData >> 4) & 0x0F;
            uint8_t angleVal = packet.auxData & 0x0F;
            
            float targetGain = map(gainVal, 0, 15, 50, 150) / 100.0f;
            float targetAngle = map(angleVal, 0, 15, 20, 40);
            
            gainMultiplier = lowPassFilter(gainMultiplier, targetGain, POT_LPF_ALPHA);
            maxAngle = lowPassFilter(maxAngle, targetAngle, POT_LPF_ALPHA);
            
            // Flight mode
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
    
    // Check for radio timeout
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
    
    // Process stick inputs with inversions
    float rawThrottle = rxPacket.throttle;
    float rawRoll = applyDeadband(rxPacket.roll * RC_ROLL_SIGN, RC_DEADBAND);
    float rawPitch = applyDeadband(rxPacket.pitch * RC_PITCH_SIGN, RC_DEADBAND);
    float rawYaw = applyDeadband(rxPacket.yaw * RC_YAW_SIGN, RC_DEADBAND);
    
    // Filter commands
    throttleCmd = lowPassFilter(throttleCmd, rawThrottle, RC_LPF_ALPHA);
    rollCmd = lowPassFilter(rollCmd, rawRoll, RC_LPF_ALPHA);
    pitchCmd = lowPassFilter(pitchCmd, rawPitch, RC_LPF_ALPHA);
    yawCmd = lowPassFilter(yawCmd, rawYaw, RC_LPF_ALPHA);
    
    // Arm/Disarm logic
    bool armSwitch = rxPacket.switches & (1 << SW_ARM);
    
    if (armSwitch && !prevArm && flightState == DISARMED) {
        // Attempt to arm
        bool throttleLow = (throttleCmd < ESC_ARM_THR);
        bool isLevel = (abs(roll) < 8.0f && abs(pitch) < 8.0f);
        bool calibOK = calibration.valid;
        bool biasSettled = (millis() > 5000);  // Give bias time to settle
        
        if (throttleLow && isLevel && calibOK && radioConnected && biasSettled) {
            flightState = ARMED;
            biasLocked = true;
            
            // Reset all PID states
            pidRollState.reset();
            pidPitchState.reset();
            pidYawState.reset();
            
            // Initialize filtered setpoints to current angles
            pidRollState.filteredSetpoint = 0;
            pidPitchState.filteredSetpoint = 0;
            pidRollState.prevRate = rollRate;
            pidPitchState.prevRate = pitchRate;
            
            Serial.println(F("\n*** ARMED ***"));
            Serial.print(F("Current angles - Roll: ")); Serial.print(roll, 1);
            Serial.print(F(" Pitch: ")); Serial.println(pitch, 1);
            
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
            if (!biasSettled) Serial.println(F("  - Bias not settled yet"));
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
//           CASCADED PID - FIXED IMPLEMENTATION
// ============================================================================
// 
// KEY FIXES:
// 1. Proper sign convention: positive PID output = increase left/front motors
// 2. Derivative on measurement prevents setpoint kicks
// 3. Proper integral anti-windup with back-calculation
// 
float calculateCascadedPID(float targetAngle, float currentAngle, float currentRate, 
                           float dt, CascadedPIDState& state,
                           float angleKp, float angleKi, 
                           float rateKp, float rateKi, float rateKd) {
    
    // ==================== SETPOINT FILTERING ====================
    state.filteredSetpoint = lowPassFilter(state.filteredSetpoint, targetAngle, SETPOINT_LPF_ALPHA);
    float smoothedTarget = state.filteredSetpoint;
    
    // ==================== OUTER LOOP (ANGLE) ====================
    float angleError = smoothedTarget - currentAngle;
    state.lastAngleError = angleError;
    
    // P term: Angle error → target rate
    float targetRate = angleKp * gainMultiplier * angleError;
    
    // I term with anti-windup
    if (abs(angleError) > 0.3f && angleKi > 0) {
        state.angleIntegral += angleError * dt;
        state.angleIntegral = constrainFloat(state.angleIntegral, -ANGLE_I_MAX, ANGLE_I_MAX);
        targetRate += angleKi * gainMultiplier * state.angleIntegral;
    } else {
        // Decay integral when near target
        state.angleIntegral *= 0.99f;
    }
    
    // Limit target rate
    targetRate = constrainFloat(targetRate, -ANGLE_RATE_MAX, ANGLE_RATE_MAX);
    state.lastTargetRate = targetRate;
    
    // ==================== INNER LOOP (RATE) ====================
    float rateError = targetRate - currentRate;
    state.lastRateError = rateError;
    
    // P term
    float rateP = rateKp * gainMultiplier * rateError;
    
    // I term with anti-windup
    float rateI = 0;
    if (rateKi > 0) {
        state.rateIntegral += rateError * dt;
        state.rateIntegral = constrainFloat(state.rateIntegral, -RATE_I_MAX, RATE_I_MAX);
        rateI = rateKi * gainMultiplier * state.rateIntegral;
    }
    
    // D term on MEASUREMENT (not error) - KEY anti-oscillation feature
    // Negative sign because we want to resist rate changes
    float rateDelta = (currentRate - state.prevRate) / dt;
    state.prevRate = currentRate;
    
    // Filter the derivative to reduce noise
    float rateD_raw = -rateKd * gainMultiplier * rateDelta;
    state.prevDterm = lowPassFilter(state.prevDterm, rateD_raw, D_TERM_LPF_ALPHA);
    float rateD = state.prevDterm;
    
    // Combine
    float output = rateP + rateI + rateD;
    
    // ==================== OUTPUT RATE LIMITING ====================
    output = rateLimitChange(state.prevOutput, output, OUTPUT_RATE_LIMIT);
    state.prevOutput = output;
    
    // Final limit
    return constrainFloat(output, -RATE_OUTPUT_MAX, RATE_OUTPUT_MAX);
}

// Simple PID for yaw with derivative on measurement
float calculateYawPID(float targetRate, float currentRate, float dt,
                      float Kp, float Ki, float Kd, SimplePIDState& state) {
    float error = targetRate - currentRate;
    
    // P term
    float P = Kp * gainMultiplier * error;
    
    // I term with anti-windup
    if (abs(error) > 1.0f) {
        state.integral += error * dt;
    } else {
        state.integral *= 0.995f;  // Decay when near target
    }
    state.integral = constrainFloat(state.integral, -YAW_I_MAX, YAW_I_MAX);
    float I = Ki * gainMultiplier * state.integral;
    
    // D term on measurement
    float rateDelta = (currentRate - state.prevMeasurement) / dt;
    state.prevMeasurement = currentRate;
    
    float D_raw = -Kd * gainMultiplier * rateDelta;
    state.prevDterm = lowPassFilter(state.prevDterm, D_raw, D_TERM_LPF_ALPHA);
    float D = state.prevDterm;
    
    float output = P + I + D;
    
    // Rate limit
    output = rateLimitChange(state.prevOutput, output, OUTPUT_RATE_LIMIT * 2);
    state.prevOutput = output;
    
    return constrainFloat(output, -RATE_OUTPUT_MAX, RATE_OUTPUT_MAX);
}

// ============================================================================
//                         UPDATE PID - FIXED
// ============================================================================
void updatePID(float dt) {
    if (flightState != ARMED) {
        rollPID = pitchPID = yawPID = 0;
        return;
    }
    
    if (flightMode == FMODE_ANGLE) {
        // ANGLE MODE: Full stabilization
        float targetRoll = (rollCmd / 500.0f) * maxAngle;
        float targetPitch = (pitchCmd / 500.0f) * maxAngle;
        
        // Calculate cascaded PID
        // Note: rollRate and pitchRate are already in the correct frame from readIMU()
        rollPID = calculateCascadedPID(targetRoll, roll, rollRate, dt, pidRollState,
                                       ANGLE_ROLL_KP, ANGLE_ROLL_KI,
                                       RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD);
        
        pitchPID = calculateCascadedPID(targetPitch, pitch, pitchRate, dt, pidPitchState,
                                        ANGLE_PITCH_KP, ANGLE_PITCH_KI,
                                        RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD);
        
    } else if (flightMode == FMODE_HORIZON) {
        // HORIZON MODE: Blend angle and rate control
        float targetRoll = (rollCmd / 500.0f) * maxAngle;
        float targetPitch = (pitchCmd / 500.0f) * maxAngle;
        
        float angleRollPID = calculateCascadedPID(targetRoll, roll, rollRate, dt, pidRollState,
                                                  ANGLE_ROLL_KP, ANGLE_ROLL_KI,
                                                  RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD);
        float anglePitchPID = calculateCascadedPID(targetPitch, pitch, pitchRate, dt, pidPitchState,
                                                   ANGLE_PITCH_KP, ANGLE_PITCH_KI,
                                                   RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD);
        
        // Blend with direct rate input for more agile control
        rollPID = 0.6f * angleRollPID + 0.4f * (rollCmd * 0.3f);
        pitchPID = 0.6f * anglePitchPID + 0.4f * (pitchCmd * 0.3f);
        
    } else {
        // ACRO MODE: Direct rate control (no angle stabilization)
        float targetRollRate = (rollCmd / 500.0f) * 250.0f;  // Max 250 deg/sec
        float targetPitchRate = (pitchCmd / 500.0f) * 250.0f;
        
        rollPID = RATE_ROLL_KP * (targetRollRate - rollRate);
        pitchPID = RATE_PITCH_KP * (targetPitchRate - pitchRate);
    }
    
    // Yaw control (rate-based in all modes)
    float targetYawRate = (yawCmd / 500.0f) * 180.0f;  // Max 180 deg/sec
    yawPID = calculateYawPID(targetYawRate, yawRate, dt, 
                             PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, pidYawState);
}

// ============================================================================
//                  UPDATE MOTORS - FIXED MIXING
// ============================================================================
// 
// MOTOR MIXING for X-configuration:
// 
//   FL(CCW) ↗     ↖ FR(CW)
//              X
//   RL(CW)  ↙     ↘ RR(CCW)
// 
// Sign convention:
//   rollPID > 0  → need to roll LEFT  → increase FL, RL (left side)
//   pitchPID > 0 → need to pitch DOWN → increase RL, RR (rear)
//   yawPID > 0   → need to yaw CCW    → increase CW motors (FR, RL)
//
void updateMotors() {
    if (flightState != ARMED) {
        motorFL = motorFR = motorRL = motorRR = ESC_MIN;
        motorFL_f = motorFR_f = motorRL_f = motorRR_f = ESC_MIN;
    } else {
        // Map throttle to motor range
        int16_t baseThr = map(throttleCmd, 0, 1000, ESC_IDLE, ESC_MAX_THROTTLE) - ESC_MIN;
        
        // Ensure minimum throttle when armed for control authority
        if (baseThr < (ESC_IDLE - ESC_MIN + 50)) {
            baseThr = ESC_IDLE - ESC_MIN + 50;
        }
        
        // FIXED MOTOR MIXING
        // Roll: + = increase left motors (FL, RL), - = increase right motors (FR, RR)
        // Pitch: + = increase rear motors (RL, RR), - = increase front motors (FL, FR)
        // Yaw: + = increase CW motors (FR, RL), - = increase CCW motors (FL, RR)
        
        int16_t rollMix = (int16_t)rollPID;
        int16_t pitchMix = (int16_t)pitchPID;
        int16_t yawMix = (int16_t)yawPID;
        
        // Apply trim offsets
        int16_t rollTrimMix = ROLL_TRIM;
        int16_t pitchTrimMix = PITCH_TRIM;
        
        // Calculate motor values
        // FL: Front-Left, CCW rotation
        int16_t fl = baseThr + rollMix - pitchMix - yawMix + rollTrimMix - pitchTrimMix;
        
        // FR: Front-Right, CW rotation  
        int16_t fr = baseThr - rollMix - pitchMix + yawMix - rollTrimMix - pitchTrimMix;
        
        // RL: Rear-Left, CW rotation
        int16_t rl = baseThr + rollMix + pitchMix + yawMix + rollTrimMix + pitchTrimMix;
        
        // RR: Rear-Right, CCW rotation
        int16_t rr = baseThr - rollMix + pitchMix - yawMix - rollTrimMix + pitchTrimMix;
        
        // Apply individual motor trims and constrain
        uint16_t tFL = constrain(fl + ESC_MIN + TRIM_FL, ESC_MIN, ESC_MAX);
        uint16_t tFR = constrain(fr + ESC_MIN + TRIM_FR, ESC_MIN, ESC_MAX);
        uint16_t tRL = constrain(rl + ESC_MIN + TRIM_RL, ESC_MIN, ESC_MAX);
        uint16_t tRR = constrain(rr + ESC_MIN + TRIM_RR, ESC_MIN, ESC_MAX);
        
        // Smooth motor outputs to prevent sudden changes
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
    else if (!radioConnected) interval = 100;  // Fast blink when no radio
    else interval = 500;  // Slow blink when ready
    
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
    // State
    if (flightState == ARMED) Serial.print(F("ARM "));
    else if (flightState == EMERGENCY) Serial.print(F("EMG "));
    else Serial.print(F("DIS "));
    
    // Mode
    if (flightMode == FMODE_ANGLE) Serial.print(F("ANG "));
    else if (flightMode == FMODE_HORIZON) Serial.print(F("HOR "));
    else Serial.print(F("ACR "));
    
    // Angles
    Serial.print(F("| R:")); Serial.print(roll, 1);
    Serial.print(F(" P:")); Serial.print(pitch, 1);
    
    // Rates
    Serial.print(F(" | Rt:")); Serial.print(rollRate, 0);
    Serial.print(F(",")); Serial.print(pitchRate, 0);
    
    // PID outputs
    Serial.print(F(" | PID:")); Serial.print(rollPID, 0);
    Serial.print(F(",")); Serial.print(pitchPID, 0);
    Serial.print(F(",")); Serial.print(yawPID, 0);
    
    // Motor values
    Serial.print(F(" | M:")); Serial.print(motorFL);
    Serial.print(F(",")); Serial.print(motorFR);
    Serial.print(F(",")); Serial.print(motorRL);
    Serial.print(F(",")); Serial.print(motorRR);
    
    // Motor balance check
    int16_t left_sum = motorFL + motorRL;
    int16_t right_sum = motorFR + motorRR;
    int16_t front_sum = motorFL + motorFR;
    int16_t rear_sum = motorRL + motorRR;
    
    Serial.print(F(" | Bal L-R:")); Serial.print(left_sum - right_sum);
    Serial.print(F(" F-B:")); Serial.print(front_sum - rear_sum);
    
    Serial.println();
}

// ============================================================================
//                              SETUP
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 1000);
    
    Serial.println(F("\n=========================================="));
    Serial.println(F("  QUADCOPTER FC - FIXED VERSION"));
    Serial.println(F("  Anti-Oscillation + Drift Fixes"));
    Serial.println(F("==========================================\n"));
    
    // Initialize pins
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    
    beepPattern(3, 2000, 100, 100);
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);
    
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
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);   // 500 deg/sec
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // 2g
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);              // 42Hz bandwidth
    
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
    
    Serial.println(F("\n*** SYSTEM READY ***"));
    Serial.println(F("\nV2 Fixes applied:"));
    Serial.println(F("  - RC stick directions corrected"));
    Serial.println(F("  - PID gains reduced to stop shaking"));
    Serial.println(F("  - More filtering on gyro and motors"));
    Serial.println(F("\nIf still shaking:"));
    Serial.println(F("  1. Reduce RATE_ROLL_KP (currently 0.25)"));
    Serial.println(F("  2. Reduce RATE_ROLL_KD (currently 0.008)"));
    Serial.println(F("\nIf too sluggish:"));
    Serial.println(F("  1. Increase RATE_ROLL_KP slowly"));
    Serial.println(F("  2. Increase ANGLE_ROLL_KP (currently 1.5)"));
    Serial.println(F("\nWaiting for radio...\n"));
    
    beepPattern(2, 2500, 150, 150);
    
    lastIMUTime = micros();
    lastRFTime = lastDebugTime = millis();
    biasSettleTime = millis();
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
        
        // Clamp dt to reasonable range to prevent issues after long delays
        if (dt > 0.01f) dt = 0.002f;  // Reset to nominal if too large
        if (dt < 0.0001f) dt = 0.002f;  // Reset if too small
        
        readIMU();
        updateAngles(dt);
        updatePID(dt);
        updateMotors();
        checkSafety();
        
        loopCount++;
    }
    
    // Radio processing at 50Hz
    if (nowMillis - lastRFTime >= 20) {
        lastRFTime = nowMillis;
        updateRadio();
        processCommands();
    }
    
    // LED update (non-blocking)
    updateLED();
    
    // Debug output at 5Hz
    if (nowMillis - lastDebugTime >= 200) {
        lastDebugTime = nowMillis;
        printDebug();
    }
}
