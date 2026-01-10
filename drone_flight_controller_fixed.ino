/**
 * ============================================================================
 *          ULTIMATE QUADCOPTER FLIGHT CONTROLLER - FIXED VERSION
 * ============================================================================
 * 
 * FIXES APPLIED:
 *   ✓ Fixed PWM timer conflicts (moved motors to Timer1/Timer2 pins)
 *   ✓ Fixed low-pass filter direction (was causing massive lag)
 *   ✓ Improved complementary filter balance for better leveling
 *   ✓ Reduced excessive filtering that caused sluggish response
 *   ✓ Improved PID gains for better self-leveling
 *   ✓ Added drone direction indicator to serial output
 *   ✓ Fixed gyro deadband that was preventing fine corrections
 * 
 * MOTOR LAYOUT (X-configuration, viewed from above):
 *     FRONT
 *   FL(CCW) FR(CW)
 *       X
 *   RL(CW)  RR(CCW)
 *     BACK
 * 
 * HARDWARE:
 *   - Arduino Nano
 *   - NRF24L01 (CE=D4, CSN=D10, 3.3V + capacitor)
 *   - MPU6050 (SDA=A4, SCL=A5, 5V)
 *   - 4x ESCs (D3=FL, D9=FR, D10=RL, D11=RR) - CHANGED for timer compatibility
 *   - Buzzer (D8)
 *   - LED (D7)
 * 
 * PWM TIMER NOTES:
 *   Arduino Nano Timer allocation:
 *   - Timer 0: Pin 5, 6 (used by millis()/delay() - AVOID for Servo!)
 *   - Timer 1: Pin 9, 10 (16-bit, best for precise PWM)
 *   - Timer 2: Pin 3, 11 (8-bit, good for PWM)
 *   Using pins 3, 9, 10, 11 avoids Timer 0 conflicts!
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
//                              CONFIGURATION
// ============================================================================
#define RF_CHANNEL          108
#define SERIAL_BAUD         115200

// ============================================================================
//                            PIN DEFINITIONS
// ============================================================================
// FIXED: Moved motors away from Timer0 pins (5,6) to avoid millis() conflicts
// Original: PIN_MOTOR_FR=5, PIN_MOTOR_RL=6 (Timer 0 - CONFLICTS with millis!)
// Fixed: Use Timer1 (9,10) and Timer2 (3,11) pins only
#define PIN_MOTOR_FL        3     // Timer 2
#define PIN_MOTOR_FR        9     // Timer 1
#define PIN_MOTOR_RL        10    // Timer 1 (was 6 - Timer 0 CONFLICT!)
#define PIN_MOTOR_RR        11    // Timer 2 (was 9)

#define PIN_RF_CE           4
#define PIN_RF_CSN          8     // Changed from 10 since we need 10 for motor
#define PIN_LED             7
#define PIN_BUZZER          2     // Changed from 8

// ============================================================================
//                          ESC PARAMETERS
// ============================================================================
#define ESC_MIN             1000
#define ESC_MAX             2000
#define ESC_IDLE            1050  // Lower idle for better low throttle control
#define ESC_ARM_THR         50

// ============================================================================
//                    PID GAINS - TUNED FOR BETTER LEVELING
// ============================================================================
// FIXED: Increased gains for more responsive leveling
#define PID_ROLL_KP         1.8f    // Was 1.2 - too weak
#define PID_ROLL_KI         0.015f  // Was 0.002 - way too weak for leveling!
#define PID_ROLL_KD         0.8f    // Was 0.5 - increased for better damping

#define PID_PITCH_KP        1.8f
#define PID_PITCH_KI        0.015f
#define PID_PITCH_KD        0.8f

#define PID_YAW_KP          2.5f
#define PID_YAW_KI          0.005f
#define PID_YAW_KD          0.0f

#define PID_I_MAX           50.0f   // Was 25 - allow more integral action
#define PID_OUTPUT_MAX      300.0f  // Was 250

// ============================================================================
//                    CALIBRATION SETTINGS
// ============================================================================
#define CALIBRATION_SAMPLES     2000
#define CALIBRATION_ITERATIONS  6
#define LEVEL_CHECK_THRESHOLD   3.0f

// ============================================================================
//                    FILTERING - FIXED FOR BETTER RESPONSE
// ============================================================================
// CRITICAL FIX: The original filter was BACKWARDS!
// Original: alpha*current + (1-alpha)*new with high alpha = sluggish response
// For LPF: use LOWER alpha to track new values faster
// Formula: output = (1-alpha)*current + alpha*new
// OR equivalently: output = current + alpha*(new - current)

// These values now represent how fast we track NEW values (higher = faster)
#define GYRO_LPF_ALPHA      0.5f    // Was 0.7 (inverted!) - faster gyro response
#define ACCEL_LPF_ALPHA     0.3f    // Was 0.85 (inverted!) - moderate accel filtering
#define ANGLE_LPF_ALPHA     0.0f    // Was 0.9 - DISABLED extra smoothing (causes lag!)
#define PID_D_LPF_ALPHA     0.3f    // Was 0.6 (inverted!)
#define MOTOR_LPF_ALPHA     0.4f    // Was 0.85 (inverted!) - faster motor response

#define RC_LPF_ALPHA        0.5f
#define POT_LPF_ALPHA       0.1f
#define RC_DEADBAND         20
#define GYRO_DEADBAND       0.1f    // Was 0.3 - allow finer corrections

// ============================================================================
//                    COMPLEMENTARY FILTER - FIXED
// ============================================================================
// FIXED: Better balance between gyro and accelerometer
// Original 0.996 = 99.6% gyro, 0.4% accel = very slow drift correction!
// For good leveling, need more accelerometer influence
#define COMP_FILTER_ALPHA   0.98f   // Trust gyro 98%, accel 2% (was 99.6%)

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
} calibration;

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
//                         PID VARIABLES
// ============================================================================
struct PIDState {
    float integral;
    float prevError;
    float prevDerivative;
    
    void reset() {
        integral = 0;
        prevError = 0;
        prevDerivative = 0;
    }
} pidRoll, pidPitch, pidYaw;

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
enum State { DISARMED, ARMED, FAILSAFE };
State flightState = DISARMED;

float throttleCmd = 0;
float rollCmd = 0, pitchCmd = 0, yawCmd = 0;
uint8_t flightMode = FMODE_ANGLE;

uint16_t motorFL = ESC_MIN, motorFR = ESC_MIN;
uint16_t motorRL = ESC_MIN, motorRR = ESC_MIN;
float motorFL_f = ESC_MIN, motorFR_f = ESC_MIN;
float motorRL_f = ESC_MIN, motorRR_f = ESC_MIN;

bool prevArm = false;

// Timing
uint32_t lastIMUTime = 0;
uint32_t lastPIDTime = 0;
uint32_t lastRFTime = 0;
uint32_t lastDebugTime = 0;

// ============================================================================
//                         DIRECTION INDICATOR
// ============================================================================
// Direction thresholds
#define DIR_THRESHOLD_TILT  5.0f   // Degrees to trigger direction
#define DIR_THRESHOLD_YAW   20.0f  // Deg/sec for rotation

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

// FIXED: Proper low-pass filter implementation
// alpha = smoothing factor (0-1), higher = faster response to new values
float lowPassFilter(float current, float newValue, float alpha) {
    return current + alpha * (newValue - current);
    // Equivalent to: (1-alpha)*current + alpha*newValue
}

float constrainFloat(float value, float minVal, float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

// ============================================================================
//                    GET DRONE DIRECTION STRING
// ============================================================================
void getDroneDirection(char* dirBuffer, size_t bufSize) {
    // Start with empty string
    dirBuffer[0] = '\0';
    
    // Check pitch (nose up/down)
    if (pitch < -DIR_THRESHOLD_TILT) {
        strcat(dirBuffer, "NOSE-DOWN ");
    } else if (pitch > DIR_THRESHOLD_TILT) {
        strcat(dirBuffer, "NOSE-UP ");
    }
    
    // Check roll (tilt left/right)
    if (roll < -DIR_THRESHOLD_TILT) {
        strcat(dirBuffer, "LEFT-TILT ");
    } else if (roll > DIR_THRESHOLD_TILT) {
        strcat(dirBuffer, "RIGHT-TILT ");
    }
    
    // Check yaw rate (spinning)
    if (yawRate < -DIR_THRESHOLD_YAW) {
        strcat(dirBuffer, "SPIN-LEFT ");
    } else if (yawRate > DIR_THRESHOLD_YAW) {
        strcat(dirBuffer, "SPIN-RIGHT ");
    }
    
    // If level and stable
    if (strlen(dirBuffer) == 0) {
        strcpy(dirBuffer, "LEVEL ");
    }
}

// Get direction as visual ASCII indicator
void printDirectionVisual() {
    // Create simple ASCII representation
    // Center dot represents the drone, arrows show tilt direction
    
    Serial.print(F("DIR["));
    
    // Pitch indicator (front/back)
    if (pitch < -DIR_THRESHOLD_TILT) {
        Serial.print(F("v"));  // Nose down
    } else if (pitch > DIR_THRESHOLD_TILT) {
        Serial.print(F("^"));  // Nose up
    } else {
        Serial.print(F("-"));  // Level pitch
    }
    
    // Roll indicator (left/right)
    if (roll < -DIR_THRESHOLD_TILT) {
        Serial.print(F("<"));  // Left tilt
    } else if (roll > DIR_THRESHOLD_TILT) {
        Serial.print(F(">"));  // Right tilt
    } else {
        Serial.print(F("|"));  // Level roll
    }
    
    // Yaw indicator
    if (yawRate < -DIR_THRESHOLD_YAW) {
        Serial.print(F("\\"));  // Spinning left (CCW from above)
    } else if (yawRate > DIR_THRESHOLD_YAW) {
        Serial.print(F("/"));  // Spinning right (CW from above)
    } else {
        Serial.print(F("o"));  // No spin
    }
    
    Serial.print(F("] "));
}

// ============================================================================
//                    MPU6050 ADVANCED CALIBRATION
// ============================================================================
void calibrateMPU6050() {
    Serial.println(F("\n========================================"));
    Serial.println(F("   MPU6050 ADVANCED CALIBRATION"));
    Serial.println(F("========================================"));
    Serial.println(F("Place drone on FLAT LEVEL surface!"));
    Serial.println(F("DO NOT MOVE during calibration!"));
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
    
    Serial.println(F("\nCalibrating (this takes ~30 seconds)..."));
    
    for (int iter = 0; iter < CALIBRATION_ITERATIONS; iter++) {
        Serial.print(F("Pass ")); Serial.print(iter + 1);
        Serial.print(F("/")); Serial.print(CALIBRATION_ITERATIONS);
        
        int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
        int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
        
        for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
            int16_t ax, ay, az, gx, gy, gz;
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            
            ax_sum += ax;
            ay_sum += ay;
            az_sum += az;
            gx_sum += gx;
            gy_sum += gy;
            gz_sum += gz;
            
            delayMicroseconds(500);
        }
        
        int16_t ax_avg = ax_sum / CALIBRATION_SAMPLES;
        int16_t ay_avg = ay_sum / CALIBRATION_SAMPLES;
        int16_t az_avg = az_sum / CALIBRATION_SAMPLES;
        int16_t gx_avg = gx_sum / CALIBRATION_SAMPLES;
        int16_t gy_avg = gy_sum / CALIBRATION_SAMPLES;
        int16_t gz_avg = gz_sum / CALIBRATION_SAMPLES;
        
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
        Serial.print(abs(gx_avg) + abs(gy_avg) + abs(gz_avg));
        Serial.print(F(" Accel err: "));
        Serial.println(abs(ax_avg) + abs(ay_avg) + abs(az_avg - 16384));
        
        delay(50);
    }
    
    calibration.axOffset = ax_off;
    calibration.ayOffset = ay_off;
    calibration.azOffset = az_off;
    calibration.gxOffset = gx_off;
    calibration.gyOffset = gy_off;
    calibration.gzOffset = gz_off;
    
    Serial.println(F("\nVerifying calibration..."));
    delay(100);
    
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    
    for (int i = 0; i < 500; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        ax_sum += ax; ay_sum += ay; az_sum += az;
        gx_sum += gx; gy_sum += gy; gz_sum += gz;
        delayMicroseconds(1000);
    }
    
    float ax_final = ax_sum / 500.0f;
    float ay_final = ay_sum / 500.0f;
    float az_final = az_sum / 500.0f;
    float gx_final = gx_sum / 500.0f;
    float gy_final = gy_sum / 500.0f;
    float gz_final = gz_sum / 500.0f;
    
    Serial.println(F("\n--- CALIBRATION RESULTS ---"));
    Serial.print(F("Accel (target 0,0,16384): "));
    Serial.print(ax_final, 0); Serial.print(F(", "));
    Serial.print(ay_final, 0); Serial.print(F(", "));
    Serial.println(az_final, 0);
    
    Serial.print(F("Gyro (target 0,0,0): "));
    Serial.print(gx_final, 1); Serial.print(F(", "));
    Serial.print(gy_final, 1); Serial.print(F(", "));
    Serial.println(gz_final, 1);
    
    float accelRoll = atan2(ay_final, az_final) * 57.2958f;
    float accelPitch = atan2(-ax_final, sqrt(ay_final*ay_final + az_final*az_final)) * 57.2958f;
    
    calibration.rollOffset = accelRoll;
    calibration.pitchOffset = accelPitch;
    
    Serial.print(F("Level offset (Roll, Pitch): "));
    Serial.print(calibration.rollOffset, 2); Serial.print(F("°, "));
    Serial.print(calibration.pitchOffset, 2); Serial.println(F("°"));
    
    bool gyroOK = (abs(gx_final) < 10 && abs(gy_final) < 10 && abs(gz_final) < 10);
    bool accelOK = (abs(ax_final) < 500 && abs(ay_final) < 500 && abs(az_final - 16384) < 500);
    
    if (gyroOK && accelOK) {
        calibration.valid = true;
        Serial.println(F("\n*** CALIBRATION SUCCESS! ***"));
        beepPattern(2, 2500, 100, 100);
    } else {
        calibration.valid = false;
        Serial.println(F("\n*** CALIBRATION POOR - Try again on flat surface ***"));
        beepPattern(4, 800, 100, 100);
    }
    
    Serial.println(F("========================================\n"));
}

// ============================================================================
//                         READ IMU - FIXED
// ============================================================================
void readIMU() {
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
    
    float gx_dps = gx_raw / 65.5f;
    float gy_dps = gy_raw / 65.5f;
    float gz_dps = gz_raw / 65.5f;
    
    float ax_g = ax_raw / 16384.0f;
    float ay_g = ay_raw / 16384.0f;
    float az_g = az_raw / 16384.0f;
    
    // FIXED: Low-pass filter now correctly tracks new values
    gyroX = lowPassFilter(gyroX, gx_dps, GYRO_LPF_ALPHA);
    gyroY = lowPassFilter(gyroY, gy_dps, GYRO_LPF_ALPHA);
    gyroZ = lowPassFilter(gyroZ, gz_dps, GYRO_LPF_ALPHA);
    
    accelX = lowPassFilter(accelX, ax_g, ACCEL_LPF_ALPHA);
    accelY = lowPassFilter(accelY, ay_g, ACCEL_LPF_ALPHA);
    accelZ = lowPassFilter(accelZ, az_g, ACCEL_LPF_ALPHA);
    
    // Track gyro bias when stationary
    if (flightState == DISARMED && !biasLocked) {
        gyroBiasX = lowPassFilter(gyroBiasX, gyroX, 0.01f);  // Slow bias tracking
        gyroBiasY = lowPassFilter(gyroBiasY, gyroY, 0.01f);
        gyroBiasZ = lowPassFilter(gyroBiasZ, gyroZ, 0.01f);
    }
    
    float gyroX_comp = gyroX - gyroBiasX;
    float gyroY_comp = gyroY - gyroBiasY;
    float gyroZ_comp = gyroZ - gyroBiasZ;
    
    // FIXED: Smaller deadband for finer corrections
    rollRate = applyDeadband(gyroX_comp, GYRO_DEADBAND);
    pitchRate = applyDeadband(gyroY_comp, GYRO_DEADBAND);
    yawRate = applyDeadband(gyroZ_comp, GYRO_DEADBAND);
}

// ============================================================================
//                    UPDATE ANGLES - FIXED COMPLEMENTARY FILTER
// ============================================================================
void updateAngles(float dt) {
    // Calculate angles from accelerometer
    float accelRoll = atan2(accelY, accelZ) * 57.2958f;
    float accelPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 57.2958f;
    
    // Apply level trim
    accelRoll -= calibration.rollOffset;
    accelPitch -= calibration.pitchOffset;
    
    // FIXED: Better balanced complementary filter
    // More accelerometer influence (2%) for better drift correction
    roll = COMP_FILTER_ALPHA * (roll + rollRate * dt) + (1.0f - COMP_FILTER_ALPHA) * accelRoll;
    pitch = COMP_FILTER_ALPHA * (pitch + pitchRate * dt) + (1.0f - COMP_FILTER_ALPHA) * accelPitch;
    
    // Yaw from gyro only
    yaw += yawRate * dt;
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
    
    // REMOVED: Extra angle smoothing that was causing lag
    // The complementary filter output is already smooth enough
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
                Serial.println(F("RF CONNECTED!"));
                beepPattern(3, 2500, 80, 80);
            }
        }
    }
    
    if (radioConnected && (millis() - lastValidPacket > 500)) {
        radioConnected = false;
        if (flightState == ARMED) {
            flightState = FAILSAFE;
            beep(800, 500);
        }
        Serial.println(F("RF LOST!"));
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
    float rawRoll = applyDeadband(rxPacket.roll, RC_DEADBAND);
    float rawPitch = applyDeadband(rxPacket.pitch, RC_DEADBAND);
    float rawYaw = applyDeadband(rxPacket.yaw, RC_DEADBAND);
    
    throttleCmd = lowPassFilter(throttleCmd, rawThrottle, RC_LPF_ALPHA);
    rollCmd = lowPassFilter(rollCmd, rawRoll, RC_LPF_ALPHA);
    pitchCmd = lowPassFilter(pitchCmd, rawPitch, RC_LPF_ALPHA);
    yawCmd = lowPassFilter(yawCmd, rawYaw, RC_LPF_ALPHA);
    
    bool armSwitch = rxPacket.switches & (1 << SW_ARM);
    
    if (armSwitch && !prevArm && flightState == DISARMED) {
        bool throttleLow = (throttleCmd < ESC_ARM_THR);
        bool isLevel = (abs(roll) < LEVEL_CHECK_THRESHOLD && abs(pitch) < LEVEL_CHECK_THRESHOLD);
        bool calibOK = calibration.valid;
        
        if (throttleLow && isLevel && calibOK && radioConnected) {
            flightState = ARMED;
            biasLocked = true;
            pidRoll.reset();
            pidPitch.reset();
            pidYaw.reset();
            Serial.println(F("*** ARMED ***"));
            beepBlocking(2000, 100);
            delay(100);
            beepBlocking(2500, 200);
        } else {
            Serial.println(F("ARM FAILED:"));
            if (!throttleLow) Serial.println(F("  - Throttle not low"));
            if (!isLevel) {
                Serial.print(F("  - Not level (R:")); 
                Serial.print(roll, 1); 
                Serial.print(F(" P:")); 
                Serial.print(pitch, 1);
                Serial.println(F(")"));
            }
            if (!calibOK) Serial.println(F("  - Calibration invalid"));
            if (!radioConnected) Serial.println(F("  - Radio not connected"));
            beepPattern(3, 500, 100, 100);
        }
    } else if (!armSwitch && flightState != DISARMED) {
        flightState = DISARMED;
        biasLocked = false;
        Serial.println(F("*** DISARMED ***"));
        beepBlocking(1500, 300);
    }
    
    prevArm = armSwitch;
}

// ============================================================================
//                         PID CALCULATION - FIXED
// ============================================================================
float calculatePID(float setpoint, float measurement, float dt,
                   float Kp, float Ki, float Kd, PIDState& state) {
    float error = setpoint - measurement;
    
    float P = Kp * gainMultiplier * error;
    
    // FIXED: Conditional integral - only accumulate when error is significant
    // and limit integral windup more aggressively
    if (abs(error) > 0.5f) {  // Only integrate significant errors
        state.integral += error * dt;
    }
    state.integral = constrainFloat(state.integral, -PID_I_MAX, PID_I_MAX);
    float I = Ki * gainMultiplier * state.integral;
    
    // FIXED: Better derivative filtering
    float rawD = (error - state.prevError) / dt;
    float D_filtered = lowPassFilter(state.prevDerivative, rawD, PID_D_LPF_ALPHA);
    state.prevDerivative = D_filtered;
    float D = Kd * gainMultiplier * D_filtered;
    
    state.prevError = error;
    
    float output = P + I + D;
    return constrainFloat(output, -PID_OUTPUT_MAX, PID_OUTPUT_MAX);
}

// ============================================================================
//                         UPDATE PID
// ============================================================================
void updatePID(float dt) {
    if (flightState != ARMED) {
        rollPID = pitchPID = yawPID = 0;
        return;
    }
    
    if (flightMode == FMODE_ANGLE) {
        // Self-leveling mode - PID controls angle
        float targetRoll = (rollCmd / 500.0f) * maxAngle;
        float targetPitch = (pitchCmd / 500.0f) * maxAngle;
        
        rollPID = calculatePID(targetRoll, roll, dt, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, pidRoll);
        pitchPID = calculatePID(targetPitch, pitch, dt, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, pidPitch);
        
    } else if (flightMode == FMODE_HORIZON) {
        float targetRoll = (rollCmd / 500.0f) * maxAngle;
        float targetPitch = (pitchCmd / 500.0f) * maxAngle;
        
        float angleRollPID = calculatePID(targetRoll, roll, dt, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, pidRoll);
        float anglePitchPID = calculatePID(targetPitch, pitch, dt, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, pidPitch);
        
        rollPID = 0.7f * angleRollPID + 0.3f * (rollCmd * 0.5f);
        pitchPID = 0.7f * anglePitchPID + 0.3f * (pitchCmd * 0.5f);
        
    } else {
        // Acro mode
        rollPID = rollCmd * 0.5f;
        pitchPID = pitchCmd * 0.5f;
    }
    
    float targetYawRate = (yawCmd / 500.0f) * 180.0f;
    yawPID = calculatePID(targetYawRate, yawRate, dt, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, pidYaw);
}

// ============================================================================
//                         UPDATE MOTORS - FIXED
// ============================================================================
void updateMotors() {
    if (flightState != ARMED && flightState != FAILSAFE) {
        motorFL = motorFR = motorRL = motorRR = ESC_MIN;
        motorFL_f = motorFR_f = motorRL_f = motorRR_f = ESC_MIN;
    } else {
        int16_t baseThr = map(throttleCmd, 0, 1000, ESC_IDLE, ESC_MAX) - ESC_MIN;
        
        if (flightState == FAILSAFE) {
            baseThr = baseThr * 0.5f;
        }
        
        // Motor mixing for X-quad
        int16_t fl = baseThr + (int16_t)rollPID + (int16_t)pitchPID + (int16_t)yawPID;
        int16_t fr = baseThr - (int16_t)rollPID + (int16_t)pitchPID - (int16_t)yawPID;
        int16_t rl = baseThr + (int16_t)rollPID - (int16_t)pitchPID - (int16_t)yawPID;
        int16_t rr = baseThr - (int16_t)rollPID - (int16_t)pitchPID + (int16_t)yawPID;
        
        uint16_t tFL = constrain(fl + ESC_MIN, ESC_MIN, ESC_MAX);
        uint16_t tFR = constrain(fr + ESC_MIN, ESC_MIN, ESC_MAX);
        uint16_t tRL = constrain(rl + ESC_MIN, ESC_MIN, ESC_MAX);
        uint16_t tRR = constrain(rr + ESC_MIN, ESC_MIN, ESC_MAX);
        
        // FIXED: Faster motor response (less smoothing)
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
    else if (flightState == FAILSAFE) interval = 100;
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
//                         DEBUG OUTPUT - WITH DIRECTION
// ============================================================================
void printDebug() {
    // State
    Serial.print(flightState == ARMED ? F("ARM ") : 
                flightState == FAILSAFE ? F("FAIL") : F("DIS "));
    
    // Direction indicator (NEW!)
    printDirectionVisual();
    
    // Gain
    Serial.print(F("G:")); Serial.print(gainMultiplier, 1);
    Serial.print(F("x"));
    
    // Angles
    Serial.print(F(" R:")); Serial.print(roll, 1);
    Serial.print(F(" P:")); Serial.print(pitch, 1);
    Serial.print(F(" Yr:")); Serial.print(yawRate, 1);
    
    // PID outputs
    Serial.print(F(" |PID R:")); Serial.print(rollPID, 0);
    Serial.print(F(" P:")); Serial.print(pitchPID, 0);
    
    // Motors
    Serial.print(F(" |M:")); Serial.print(motorFL);
    Serial.print(F(",")); Serial.print(motorFR);
    Serial.print(F(",")); Serial.print(motorRL);
    Serial.print(F(",")); Serial.print(motorRR);
    
    Serial.println();
}

// Extended debug with text direction
void printDebugExtended() {
    printDebug();
    
    // Print detailed direction
    char dirBuffer[64];
    getDroneDirection(dirBuffer, sizeof(dirBuffer));
    Serial.print(F("  >> DIRECTION: "));
    Serial.println(dirBuffer);
}

// ============================================================================
//                              SETUP
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 1000);
    
    Serial.println(F("\n=========================================="));
    Serial.println(F("  QUADCOPTER FC - FIXED VERSION"));
    Serial.println(F("  PWM/Leveling/Direction Fixes Applied"));
    Serial.println(F("=========================================="));
    
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
    
    // FIXED: Using new pin assignments that avoid Timer 0
    Serial.print(F("Init ESCs (pins 3,9,10,11)..."));
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
    Serial.println(F("Direction indicators:"));
    Serial.println(F("  ^ = nose up,  v = nose down"));
    Serial.println(F("  < = left tilt, > = right tilt"));
    Serial.println(F("  \\ = spin left, / = spin right"));
    Serial.println(F("  -|o = level and stable"));
    Serial.println(F("\nWaiting for radio connection..."));
    Serial.println(F("Keep drone LEVEL before arming!\n"));
    
    beepPattern(2, 2500, 150, 150);
    
    lastIMUTime = lastPIDTime = micros();
    lastRFTime = lastDebugTime = millis();
}

// ============================================================================
//                             MAIN LOOP - FIXED TIMING
// ============================================================================
void loop() {
    uint32_t nowMicros = micros();
    uint32_t nowMillis = millis();
    
    // Read IMU as fast as possible
    readIMU();
    
    // Update angles at ~500Hz (every 2000µs)
    if (nowMicros - lastIMUTime >= 2000) {
        float dt = (nowMicros - lastIMUTime) / 1000000.0f;
        lastIMUTime = nowMicros;
        updateAngles(dt);
    }
    
    // Update radio (non-blocking)
    updateRadio();
    
    // Process commands at ~50Hz
    if (nowMillis - lastRFTime >= 20) {
        lastRFTime = nowMillis;
        processCommands();
    }
    
    // Update PID and motors at ~250Hz (every 4000µs)
    if (nowMicros - lastPIDTime >= 4000) {
        float dt = (nowMicros - lastPIDTime) / 1000000.0f;
        lastPIDTime = nowMicros;
        updatePID(dt);
        updateMotors();
    }
    
    // Update LED
    updateLED();
    
    // Debug at ~5Hz with direction indicator
    if (nowMillis - lastDebugTime >= 200) {
        lastDebugTime = nowMillis;
        printDebug();
    }
}
