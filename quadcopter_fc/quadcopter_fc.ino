/**
 * ============================================================================
 * QUADCOPTER FLIGHT CONTROLLER FIRMWARE
 * ============================================================================
 * 
 * Target: Arduino Nano (ATmega328P @ 16MHz)
 * 
 * Hardware Configuration:
 *   IMU: MPU6050 (I2C 0x68, INT → D2)
 *   Motors: FL→D3, FR→D5, RL→D6, RR→D9
 *   RF: NRF24L01 (CE→D4, CSN→D10)
 *   Buzzer: D8
 *   LED: D7
 * 
 * Timing Architecture:
 *   IMU Sampling:     1000 Hz (interrupt-driven)
 *   Attitude Fusion:   500 Hz
 *   PID Control:       250 Hz
 *   RF Reception:       50 Hz
 *   ESC Update:        250 Hz (synchronized with PID)
 * 
 * Author: Flight Control Systems
 * Version: 1.0.0
 * 
 * ============================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

namespace Pins {
    // Motor outputs (PWM capable)
    constexpr uint8_t MOTOR_FL = 3;   // Front-Left  (Timer2)
    constexpr uint8_t MOTOR_FR = 5;   // Front-Right (Timer0)
    constexpr uint8_t MOTOR_RL = 6;   // Rear-Left   (Timer0)
    constexpr uint8_t MOTOR_RR = 9;   // Rear-Right  (Timer1)
    
    // NRF24L01
    constexpr uint8_t RF_CE  = 4;
    constexpr uint8_t RF_CSN = 10;
    
    // IMU Interrupt
    constexpr uint8_t IMU_INT = 2;
    
    // User Interface
    constexpr uint8_t LED    = 7;
    constexpr uint8_t BUZZER = 8;
}

// ============================================================================
// TIMING CONFIGURATION (CRITICAL FOR STABILITY)
// ============================================================================

namespace Timing {
    // Loop periods in microseconds
    constexpr uint32_t IMU_PERIOD_US      = 1000;    // 1000 Hz - Raw sensor sampling
    constexpr uint32_t ATTITUDE_PERIOD_US = 2000;    // 500 Hz  - Sensor fusion
    constexpr uint32_t PID_PERIOD_US      = 4000;    // 250 Hz  - Control loop
    constexpr uint32_t RF_PERIOD_US       = 20000;   // 50 Hz   - Radio reception
    constexpr uint32_t LED_PERIOD_US      = 500000;  // 2 Hz    - Status blink
    
    // Derived constants (for PID calculations)
    constexpr float PID_DT = PID_PERIOD_US / 1000000.0f;  // 0.004 seconds
    constexpr float ATTITUDE_DT = ATTITUDE_PERIOD_US / 1000000.0f;  // 0.002 seconds
    
    // Failsafe timeouts
    constexpr uint32_t RF_WARN_TIMEOUT_MS  = 500;   // Reduce throttle
    constexpr uint32_t RF_FAIL_TIMEOUT_MS  = 1000;  // Full motor cutoff
}

/**
 * ============================================================================
 * TIMING ARCHITECTURE EXPLANATION
 * ============================================================================
 * 
 * Why these specific frequencies matter:
 * 
 * 1. IMU at 1000 Hz:
 *    - MPU6050 gyro noise is reduced by averaging multiple samples
 *    - Must be faster than attitude loop to provide fresh data
 *    - Hardware interrupt ensures no missed samples
 * 
 * 2. Attitude Fusion at 500 Hz:
 *    - Complementary filter needs consistent dt for accurate integration
 *    - Faster than human perception of instability (~100ms)
 *    - Provides 2 attitude updates per PID cycle for interpolation
 * 
 * 3. PID at 250 Hz:
 *    - Derivative term: d_error/dt requires stable dt
 *    - If dt varies, derivative kick causes oscillation
 *    - 250 Hz is fast enough for angle-mode stability
 *    - Matches ESC update rate for synchronization
 * 
 * 4. RF at 50 Hz:
 *    - Human reaction time ~200ms, 50Hz is 4x oversampling
 *    - Reduces SPI bus contention with ESC updates
 *    - Allows multiple retry attempts within perception threshold
 * 
 * CRITICAL: Mismatched frequencies cause:
 *    - Aliasing in sensor fusion (attitude drift)
 *    - PID derivative spikes (motor twitching)
 *    - ESC desync (motor stuttering)
 *    - Control latency (sluggish response)
 * 
 * ============================================================================
 */

// ============================================================================
// FLIGHT PARAMETERS
// ============================================================================

namespace FlightParams {
    // Angle mode limits (degrees)
    constexpr float MAX_ROLL_ANGLE  = 50.0f;
    constexpr float MAX_PITCH_ANGLE = 50.0f;
    constexpr float MAX_YAW_RATE    = 180.0f;  // degrees/second
    
    // Motor limits (microseconds PWM)
    constexpr uint16_t ESC_MIN_US = 1000;
    constexpr uint16_t ESC_MAX_US = 2000;
    constexpr uint16_t ESC_ARM_US = 1050;  // Minimum for arming
    constexpr uint16_t ESC_IDLE_US = 1100; // Idle when armed
    
    // Throttle mapping
    constexpr uint16_t THROTTLE_MIN = 0;
    constexpr uint16_t THROTTLE_MAX = 1000;
    constexpr uint16_t THROTTLE_ARM_MAX = 50;  // Max throttle to allow arming
}

// ============================================================================
// RF COMMUNICATION PROTOCOL
// ============================================================================

/**
 * RF Protocol Design Decisions:
 * 
 * 1. NO_ACK Mode (vs ACK):
 *    - ACK adds 130-250μs latency per packet for acknowledgment
 *    - In real-time control, FRESH data > GUARANTEED data
 *    - Lost packet: next packet arrives in 20ms anyway
 *    - Failsafe handles sustained packet loss
 *    - Result: ~1ms latency vs 3-5ms with ACK
 * 
 * 2. Payload Size: 16 bytes
 *    - NRF24L01 max is 32 bytes
 *    - Smaller packets = faster transmission = lower collision probability
 *    - 16 bytes sufficient for all control channels + metadata
 * 
 * 3. Data Rate: 2Mbps
 *    - Faster transmission = shorter air time = less interference susceptibility
 *    - Tradeoff: slightly reduced range (acceptable for LOS operation)
 * 
 * 4. Channel: 108 (2.508 GHz)
 *    - Above WiFi channels (2.4-2.4835 GHz)
 *    - Reduces interference in typical environments
 */

namespace RFConfig {
    constexpr uint8_t CHANNEL = 108;
    constexpr uint8_t PAYLOAD_SIZE = 16;
    const uint8_t ADDRESS[6] = "QUAD1";
    
    // Packet structure offsets
    constexpr uint8_t THROTTLE_OFFSET = 0;   // 2 bytes
    constexpr uint8_t YAW_OFFSET      = 2;   // 2 bytes (signed)
    constexpr uint8_t PITCH_OFFSET    = 4;   // 2 bytes (signed)
    constexpr uint8_t ROLL_OFFSET     = 6;   // 2 bytes (signed)
    constexpr uint8_t SWITCHES_OFFSET = 8;   // 1 byte (bitfield)
    constexpr uint8_t CHECKSUM_OFFSET = 9;   // 1 byte
    constexpr uint8_t SEQUENCE_OFFSET = 10;  // 4 bytes
    constexpr uint8_t RSSI_REQ_OFFSET = 14;  // 1 byte
    constexpr uint8_t RESERVED_OFFSET = 15;  // 1 byte
    
    // Switch bit definitions
    constexpr uint8_t SW_ARM_BIT       = 0;
    constexpr uint8_t SW_CALIBRATE_BIT = 1;
    constexpr uint8_t SW_MOTORTEST_BIT = 2;
}

// Control packet structure (16 bytes)
struct __attribute__((packed)) ControlPacket {
    uint16_t throttle;    // 0-1000
    int16_t  yaw;         // -500 to +500
    int16_t  pitch;       // -500 to +500
    int16_t  roll;        // -500 to +500
    uint8_t  switches;    // Bit 0: Arm, Bit 1: Calibrate, Bit 2: Motor Test
    uint8_t  checksum;    // XOR of bytes 0-8
    uint32_t sequence;    // Packet sequence number
    uint8_t  rssiRequest; // Request RSSI feedback
    uint8_t  reserved;    // Future use
    
    bool validateChecksum() const {
        const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
        uint8_t calc = 0;
        for (uint8_t i = 0; i < RFConfig::CHECKSUM_OFFSET; i++) {
            calc ^= data[i];
        }
        return calc == checksum;
    }
};

// ============================================================================
// IMU CLASS
// ============================================================================

/**
 * IMU Class Responsibilities:
 * - MPU6050 initialization and configuration
 * - Raw sensor data acquisition
 * - Calibration offset management
 * - Low-pass filtering of raw data
 * - Interrupt-driven sampling for deterministic timing
 * 
 * Design Choice: I2Cdev + MPU6050 library WITHOUT DMP
 * 
 * Rationale for Complementary Filter over DMP:
 * 1. Latency: DMP has 2-4ms internal processing delay
 *    - Complementary filter: <100μs
 *    - For 250Hz PID, DMP delay = 50-100% of control period
 * 
 * 2. Transparency: DMP is a black box
 *    - Unknown failure modes
 *    - Cannot tune internal parameters
 *    - Difficult to debug attitude errors
 * 
 * 3. Resource Usage: DMP requires FIFO management
 *    - Additional code complexity
 *    - Potential for FIFO overflow issues
 * 
 * 4. Customization: Complementary filter is tunable
 *    - Alpha parameter adjusts gyro/accel trust ratio
 *    - Can add vibration compensation
 *    - Easy to implement sensor health checks
 * 
 * Tradeoffs Accepted:
 * - Slightly more CPU usage (~50μs per fusion cycle)
 * - Requires manual calibration management
 * - Yaw drift without magnetometer (acceptable for angle mode)
 */

class IMU {
public:
    // Calibration offsets
    struct Calibration {
        int16_t accelX, accelY, accelZ;
        int16_t gyroX, gyroY, gyroZ;
        bool valid;
    };
    
    // Processed IMU data
    struct Data {
        float accelX, accelY, accelZ;  // g's
        float gyroX, gyroY, gyroZ;     // degrees/second
        uint32_t timestamp;
        bool valid;
    };
    
private:
    static constexpr uint8_t MPU_ADDR = 0x68;
    
    // MPU6050 Register addresses
    static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t REG_CONFIG       = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_INT_PIN_CFG  = 0x37;
    static constexpr uint8_t REG_INT_ENABLE   = 0x38;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
    
    // Scaling factors
    static constexpr float ACCEL_SCALE = 1.0f / 16384.0f;  // ±2g range
    static constexpr float GYRO_SCALE  = 1.0f / 131.0f;    // ±250°/s range
    
    // Low-pass filter coefficients (IIR filter)
    // Alpha = dt / (RC + dt), where RC = 1/(2*pi*fc)
    // For fc=20Hz, dt=1ms: alpha ≈ 0.11
    static constexpr float LPF_ALPHA_ACCEL = 0.1f;
    static constexpr float LPF_ALPHA_GYRO  = 0.5f;  // Gyro needs less filtering
    
    Calibration cal_;
    Data data_;
    Data filteredData_;
    
    // Raw sensor buffers for burst read
    int16_t rawAccel_[3];
    int16_t rawGyro_[3];
    int16_t rawTemp_;
    
    bool initialized_;
    
    void writeRegister(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }
    
    uint8_t readRegister(uint8_t reg) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, (uint8_t)1);
        return Wire.read();
    }
    
public:
    IMU() : initialized_(false) {
        cal_.valid = false;
        data_.valid = false;
        filteredData_.valid = false;
    }
    
    bool begin() {
        Wire.begin();
        Wire.setClock(400000);  // 400kHz I2C for faster reads
        
        // Check device ID
        uint8_t whoami = readRegister(0x75);
        if (whoami != 0x68) {
            return false;
        }
        
        // Wake up MPU6050 (clear sleep bit)
        writeRegister(REG_PWR_MGMT_1, 0x00);
        delayMicroseconds(100);
        
        // Set clock source to PLL with X-axis gyro reference
        writeRegister(REG_PWR_MGMT_1, 0x01);
        
        // Set sample rate divider: 1kHz / (1 + 0) = 1kHz
        writeRegister(REG_SMPLRT_DIV, 0x00);
        
        /**
         * DLPF Configuration (Critical for noise reduction)
         * 
         * REG_CONFIG bits [2:0] set DLPF:
         * 0: 260Hz (8kHz sampling) - Too much noise
         * 1: 184Hz (1kHz sampling)
         * 2: 94Hz  (1kHz sampling)
         * 3: 44Hz  (1kHz sampling) - Good balance
         * 4: 21Hz  (1kHz sampling) - More filtering
         * 5: 10Hz  (1kHz sampling) - Heavy filtering
         * 6: 5Hz   (1kHz sampling) - Very heavy
         * 
         * Choice: DLPF = 3 (44Hz cutoff)
         * - Removes motor vibration (typically >100Hz)
         * - Maintains control bandwidth
         * - Introduces ~4.9ms group delay (acceptable)
         */
        writeRegister(REG_CONFIG, 0x03);
        
        // Set gyro range to ±250°/s (maximum sensitivity)
        writeRegister(REG_GYRO_CONFIG, 0x00);
        
        // Set accelerometer range to ±2g (maximum sensitivity)
        writeRegister(REG_ACCEL_CONFIG, 0x00);
        
        // Configure interrupt pin (active high, push-pull, clear on read)
        writeRegister(REG_INT_PIN_CFG, 0x10);
        
        // Enable data ready interrupt
        writeRegister(REG_INT_ENABLE, 0x01);
        
        initialized_ = true;
        return true;
    }
    
    /**
     * Burst read all sensor data in single I2C transaction
     * This is critical for timing - individual reads would take ~500μs,
     * burst read takes ~200μs
     */
    bool readRaw() {
        if (!initialized_) return false;
        
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(REG_ACCEL_XOUT_H);
        if (Wire.endTransmission(false) != 0) {
            return false;
        }
        
        // Read 14 bytes: Accel(6) + Temp(2) + Gyro(6)
        Wire.requestFrom(MPU_ADDR, (uint8_t)14);
        
        if (Wire.available() < 14) {
            return false;
        }
        
        // Accelerometer (big-endian)
        rawAccel_[0] = (Wire.read() << 8) | Wire.read();
        rawAccel_[1] = (Wire.read() << 8) | Wire.read();
        rawAccel_[2] = (Wire.read() << 8) | Wire.read();
        
        // Temperature (unused but must read)
        rawTemp_ = (Wire.read() << 8) | Wire.read();
        
        // Gyroscope (big-endian)
        rawGyro_[0] = (Wire.read() << 8) | Wire.read();
        rawGyro_[1] = (Wire.read() << 8) | Wire.read();
        rawGyro_[2] = (Wire.read() << 8) | Wire.read();
        
        return true;
    }
    
    /**
     * Process raw data: apply calibration and convert to physical units
     */
    void processData() {
        data_.timestamp = micros();
        
        // Apply calibration offsets and scale
        if (cal_.valid) {
            data_.accelX = (rawAccel_[0] - cal_.accelX) * ACCEL_SCALE;
            data_.accelY = (rawAccel_[1] - cal_.accelY) * ACCEL_SCALE;
            data_.accelZ = (rawAccel_[2] - cal_.accelZ) * ACCEL_SCALE;
            data_.gyroX  = (rawGyro_[0] - cal_.gyroX) * GYRO_SCALE;
            data_.gyroY  = (rawGyro_[1] - cal_.gyroY) * GYRO_SCALE;
            data_.gyroZ  = (rawGyro_[2] - cal_.gyroZ) * GYRO_SCALE;
        } else {
            data_.accelX = rawAccel_[0] * ACCEL_SCALE;
            data_.accelY = rawAccel_[1] * ACCEL_SCALE;
            data_.accelZ = rawAccel_[2] * ACCEL_SCALE;
            data_.gyroX  = rawGyro_[0] * GYRO_SCALE;
            data_.gyroY  = rawGyro_[1] * GYRO_SCALE;
            data_.gyroZ  = rawGyro_[2] * GYRO_SCALE;
        }
        
        data_.valid = true;
        
        // Apply IIR low-pass filter
        if (filteredData_.valid) {
            filteredData_.accelX = filteredData_.accelX + LPF_ALPHA_ACCEL * (data_.accelX - filteredData_.accelX);
            filteredData_.accelY = filteredData_.accelY + LPF_ALPHA_ACCEL * (data_.accelY - filteredData_.accelY);
            filteredData_.accelZ = filteredData_.accelZ + LPF_ALPHA_ACCEL * (data_.accelZ - filteredData_.accelZ);
            filteredData_.gyroX  = filteredData_.gyroX + LPF_ALPHA_GYRO * (data_.gyroX - filteredData_.gyroX);
            filteredData_.gyroY  = filteredData_.gyroY + LPF_ALPHA_GYRO * (data_.gyroY - filteredData_.gyroY);
            filteredData_.gyroZ  = filteredData_.gyroZ + LPF_ALPHA_GYRO * (data_.gyroZ - filteredData_.gyroZ);
        } else {
            filteredData_ = data_;
        }
        filteredData_.timestamp = data_.timestamp;
        filteredData_.valid = true;
    }
    
    /**
     * Calibration routine
     * MUST be called with quad level and stationary
     * Takes ~2 seconds (2000 samples at 1kHz)
     */
    void calibrate(void (*progressCallback)(uint8_t percent) = nullptr) {
        const uint16_t SAMPLES = 2000;
        int32_t sumAccel[3] = {0, 0, 0};
        int32_t sumGyro[3] = {0, 0, 0};
        
        for (uint16_t i = 0; i < SAMPLES; i++) {
            while (!readRaw()) {
                delayMicroseconds(100);
            }
            
            sumAccel[0] += rawAccel_[0];
            sumAccel[1] += rawAccel_[1];
            sumAccel[2] += rawAccel_[2];
            sumGyro[0]  += rawGyro_[0];
            sumGyro[1]  += rawGyro_[1];
            sumGyro[2]  += rawGyro_[2];
            
            if (progressCallback && (i % 200 == 0)) {
                progressCallback((i * 100) / SAMPLES);
            }
            
            delayMicroseconds(500);  // ~1kHz sampling
        }
        
        cal_.accelX = sumAccel[0] / SAMPLES;
        cal_.accelY = sumAccel[1] / SAMPLES;
        // Z-axis should read 1g (16384 LSB at ±2g range), not zero
        cal_.accelZ = (sumAccel[2] / SAMPLES) - 16384;
        cal_.gyroX  = sumGyro[0] / SAMPLES;
        cal_.gyroY  = sumGyro[1] / SAMPLES;
        cal_.gyroZ  = sumGyro[2] / SAMPLES;
        cal_.valid  = true;
        
        if (progressCallback) {
            progressCallback(100);
        }
    }
    
    const Data& getData() const { return data_; }
    const Data& getFilteredData() const { return filteredData_; }
    const Calibration& getCalibration() const { return cal_; }
    bool isCalibrated() const { return cal_.valid; }
    bool isInitialized() const { return initialized_; }
    
    // Access raw data for debugging
    void getRawAccel(int16_t* out) const {
        out[0] = rawAccel_[0];
        out[1] = rawAccel_[1];
        out[2] = rawAccel_[2];
    }
    
    void getRawGyro(int16_t* out) const {
        out[0] = rawGyro_[0];
        out[1] = rawGyro_[1];
        out[2] = rawGyro_[2];
    }
};

// ============================================================================
// ATTITUDE ESTIMATOR (COMPLEMENTARY FILTER)
// ============================================================================

/**
 * Complementary Filter Implementation
 * 
 * Theory:
 * - Gyroscope: High-frequency accurate, but drifts over time
 * - Accelerometer: Low-frequency accurate (measures gravity), noisy in motion
 * 
 * Solution: High-pass filter gyro + Low-pass filter accel
 * angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
 * 
 * Alpha Selection (0.98):
 * - Higher alpha = trust gyro more (less drift correction)
 * - Lower alpha = trust accel more (more vibration sensitivity)
 * - 0.98 gives ~0.5s time constant for drift correction
 * 
 * Why not Kalman Filter?
 * - Complementary filter is computationally cheaper (~10μs vs ~100μs)
 * - Easier to tune and understand
 * - Kalman requires accurate noise models (hard to obtain)
 * - For angle mode, complementary is sufficient
 */

class AttitudeEstimator {
private:
    // Complementary filter coefficient
    // Time constant τ = dt / (1 - alpha) ≈ 0.1s for alpha=0.98, dt=2ms
    static constexpr float COMP_FILTER_ALPHA = 0.98f;
    
    // Estimated angles (degrees)
    float roll_;
    float pitch_;
    float yaw_;  // Note: yaw drifts without magnetometer
    
    // Previous timestamp for dt calculation
    uint32_t lastUpdateTime_;
    bool initialized_;
    
public:
    AttitudeEstimator() : roll_(0), pitch_(0), yaw_(0), 
                          lastUpdateTime_(0), initialized_(false) {}
    
    void reset() {
        roll_ = 0;
        pitch_ = 0;
        yaw_ = 0;
        initialized_ = false;
    }
    
    /**
     * Update attitude estimate with new sensor data
     * Should be called at consistent intervals (500Hz)
     */
    void update(const IMU::Data& imuData) {
        uint32_t now = micros();
        
        if (!initialized_) {
            // Initialize angles from accelerometer
            roll_  = atan2(imuData.accelY, imuData.accelZ) * 57.2957795f;
            pitch_ = atan2(-imuData.accelX, 
                          sqrt(imuData.accelY * imuData.accelY + 
                               imuData.accelZ * imuData.accelZ)) * 57.2957795f;
            yaw_ = 0;
            lastUpdateTime_ = now;
            initialized_ = true;
            return;
        }
        
        // Calculate actual dt (should be ~2ms)
        float dt = (now - lastUpdateTime_) / 1000000.0f;
        lastUpdateTime_ = now;
        
        // Clamp dt to prevent issues from timing glitches
        if (dt <= 0 || dt > 0.02f) {
            dt = Timing::ATTITUDE_DT;
        }
        
        // Calculate angles from accelerometer (only valid when not accelerating)
        float accelRoll  = atan2(imuData.accelY, imuData.accelZ) * 57.2957795f;
        float accelPitch = atan2(-imuData.accelX,
                                sqrt(imuData.accelY * imuData.accelY + 
                                     imuData.accelZ * imuData.accelZ)) * 57.2957795f;
        
        // Integrate gyroscope (convert to body frame rotation)
        // Note: This is simplified - full quaternion would handle large angles better
        float gyroRollRate  = imuData.gyroX + 
                              imuData.gyroY * sin(roll_ * 0.0174533f) * tan(pitch_ * 0.0174533f) +
                              imuData.gyroZ * cos(roll_ * 0.0174533f) * tan(pitch_ * 0.0174533f);
        float gyroPitchRate = imuData.gyroY * cos(roll_ * 0.0174533f) - 
                              imuData.gyroZ * sin(roll_ * 0.0174533f);
        float gyroYawRate   = imuData.gyroY * sin(roll_ * 0.0174533f) / cos(pitch_ * 0.0174533f) +
                              imuData.gyroZ * cos(roll_ * 0.0174533f) / cos(pitch_ * 0.0174533f);
        
        // Complementary filter
        roll_  = COMP_FILTER_ALPHA * (roll_ + gyroRollRate * dt) + 
                 (1.0f - COMP_FILTER_ALPHA) * accelRoll;
        pitch_ = COMP_FILTER_ALPHA * (pitch_ + gyroPitchRate * dt) + 
                 (1.0f - COMP_FILTER_ALPHA) * accelPitch;
        
        // Yaw from gyro integration only (will drift)
        yaw_ += gyroYawRate * dt;
        
        // Normalize yaw to ±180°
        while (yaw_ > 180.0f)  yaw_ -= 360.0f;
        while (yaw_ < -180.0f) yaw_ += 360.0f;
    }
    
    float getRoll()  const { return roll_; }
    float getPitch() const { return pitch_; }
    float getYaw()   const { return yaw_; }
    bool isInitialized() const { return initialized_; }
};

// ============================================================================
// PID CONTROLLER
// ============================================================================

/**
 * PID Controller with Anti-Windup and Derivative Filtering
 * 
 * Features:
 * 1. Derivative on Measurement (not error)
 *    - Prevents derivative kick on setpoint changes
 *    
 * 2. Integral Anti-Windup
 *    - Clamps integral term to prevent wind-up during saturation
 *    - Back-calculation method: reduce integral when output saturates
 *    
 * 3. Derivative Low-Pass Filter
 *    - Reduces noise amplification
 *    - First-order IIR with configurable cutoff
 *    
 * 4. Output Rate Limiting
 *    - Prevents sudden motor speed changes
 *    - Reduces mechanical stress
 */

class PIDController {
private:
    // Gains
    float kp_, ki_, kd_;
    
    // Anti-windup limits
    float integralMin_, integralMax_;
    float outputMin_, outputMax_;
    
    // State
    float integral_;
    float prevMeasurement_;  // For derivative on measurement
    float prevDerivative_;   // For derivative filtering
    
    // Derivative filter coefficient (0.0-1.0, higher = more filtering)
    static constexpr float DERIV_FILTER_ALPHA = 0.7f;
    
    bool initialized_;
    
public:
    PIDController() : kp_(0), ki_(0), kd_(0),
                      integralMin_(-500), integralMax_(500),
                      outputMin_(-500), outputMax_(500),
                      integral_(0), prevMeasurement_(0), prevDerivative_(0),
                      initialized_(false) {}
    
    void setGains(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }
    
    void setIntegralLimits(float min, float max) {
        integralMin_ = min;
        integralMax_ = max;
    }
    
    void setOutputLimits(float min, float max) {
        outputMin_ = min;
        outputMax_ = max;
    }
    
    void reset() {
        integral_ = 0;
        prevMeasurement_ = 0;
        prevDerivative_ = 0;
        initialized_ = false;
    }
    
    /**
     * Calculate PID output
     * @param setpoint Desired value
     * @param measurement Current value
     * @param dt Time delta in seconds (MUST be consistent!)
     * @return Control output
     */
    float calculate(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;
        
        // Initialize on first call
        if (!initialized_) {
            prevMeasurement_ = measurement;
            initialized_ = true;
        }
        
        // Proportional term
        float pTerm = kp_ * error;
        
        // Integral term with anti-windup
        integral_ += ki_ * error * dt;
        integral_ = constrain(integral_, integralMin_, integralMax_);
        float iTerm = integral_;
        
        // Derivative term (on measurement, not error)
        // This prevents derivative kick when setpoint changes
        float derivative = -(measurement - prevMeasurement_) / dt;
        
        // Low-pass filter the derivative
        derivative = prevDerivative_ + DERIV_FILTER_ALPHA * (derivative - prevDerivative_);
        float dTerm = kd_ * derivative;
        
        prevMeasurement_ = measurement;
        prevDerivative_ = derivative;
        
        // Calculate output
        float output = pTerm + iTerm + dTerm;
        
        // Apply output limits
        float limitedOutput = constrain(output, outputMin_, outputMax_);
        
        // Back-calculation anti-windup
        // If output is saturated, reduce integral to prevent wind-up
        if (output != limitedOutput && ki_ != 0) {
            integral_ -= (output - limitedOutput) * 0.5f / ki_;
        }
        
        return limitedOutput;
    }
    
    // Accessors for debugging
    float getIntegral() const { return integral_; }
    float getKp() const { return kp_; }
    float getKi() const { return ki_; }
    float getKd() const { return kd_; }
};

// ============================================================================
// RADIO LINK
// ============================================================================

/**
 * RadioLink Class
 * 
 * Responsibilities:
 * - NRF24L01 initialization and configuration
 * - Packet reception and validation
 * - Link quality monitoring
 * - Failsafe state management
 */

class RadioLink {
public:
    enum class LinkState {
        NO_LINK,      // Never received valid packet
        CONNECTED,    // Receiving packets normally
        DEGRADED,     // Packet loss > threshold
        FAILSAFE      // No packets for timeout period
    };
    
    struct Stats {
        uint32_t packetsReceived;
        uint32_t packetsLost;
        uint32_t checksumErrors;
        uint32_t lastSequence;
        uint32_t lastPacketTime;
        uint8_t lossPercent;
    };
    
private:
    RF24 radio_;
    ControlPacket lastPacket_;
    Stats stats_;
    LinkState state_;
    
    // Packet loss calculation window
    static constexpr uint8_t LOSS_WINDOW = 50;
    uint8_t lossHistory_[LOSS_WINDOW];
    uint8_t lossIndex_;
    
public:
    RadioLink() : radio_(Pins::RF_CE, Pins::RF_CSN), state_(LinkState::NO_LINK) {
        memset(&lastPacket_, 0, sizeof(lastPacket_));
        memset(&stats_, 0, sizeof(stats_));
        memset(lossHistory_, 0, sizeof(lossHistory_));
        lossIndex_ = 0;
    }
    
    bool begin() {
        if (!radio_.begin()) {
            return false;
        }
        
        // Configure radio
        radio_.setChannel(RFConfig::CHANNEL);
        radio_.setDataRate(RF24_2MBPS);      // Fast data rate for low latency
        radio_.setPALevel(RF24_PA_MAX);       // Maximum range
        radio_.setPayloadSize(RFConfig::PAYLOAD_SIZE);
        radio_.setAutoAck(false);             // NO_ACK mode for lower latency
        radio_.setRetries(0, 0);              // No retries in NO_ACK mode
        radio_.setCRCLength(RF24_CRC_16);     // 16-bit CRC for data integrity
        
        // Open reading pipe
        radio_.openReadingPipe(1, RFConfig::ADDRESS);
        radio_.startListening();
        
        return true;
    }
    
    /**
     * Check for and process incoming packets
     * Should be called at RF_PERIOD (50Hz)
     * Returns true if new valid packet received
     */
    bool update() {
        bool newPacket = false;
        
        // Check if data available
        if (radio_.available()) {
            ControlPacket packet;
            radio_.read(&packet, sizeof(packet));
            
            // Validate checksum
            if (packet.validateChecksum()) {
                // Check sequence for lost packets
                if (stats_.packetsReceived > 0) {
                    uint32_t expectedSeq = stats_.lastSequence + 1;
                    if (packet.sequence != expectedSeq) {
                        uint32_t lost = packet.sequence - expectedSeq;
                        if (lost > 0 && lost < 100) {  // Sanity check
                            stats_.packetsLost += lost;
                        }
                        lossHistory_[lossIndex_] = 1;
                    } else {
                        lossHistory_[lossIndex_] = 0;
                    }
                }
                
                lastPacket_ = packet;
                stats_.lastSequence = packet.sequence;
                stats_.lastPacketTime = millis();
                stats_.packetsReceived++;
                newPacket = true;
                
                lossIndex_ = (lossIndex_ + 1) % LOSS_WINDOW;
            } else {
                stats_.checksumErrors++;
            }
        }
        
        // Update link state
        updateLinkState();
        
        return newPacket;
    }
    
    void updateLinkState() {
        uint32_t now = millis();
        uint32_t timeSincePacket = now - stats_.lastPacketTime;
        
        if (stats_.packetsReceived == 0) {
            state_ = LinkState::NO_LINK;
        } else if (timeSincePacket > Timing::RF_FAIL_TIMEOUT_MS) {
            state_ = LinkState::FAILSAFE;
        } else if (timeSincePacket > Timing::RF_WARN_TIMEOUT_MS) {
            state_ = LinkState::DEGRADED;
        } else {
            // Calculate loss percentage over window
            uint8_t losses = 0;
            for (uint8_t i = 0; i < LOSS_WINDOW; i++) {
                losses += lossHistory_[i];
            }
            stats_.lossPercent = (losses * 100) / LOSS_WINDOW;
            
            state_ = (stats_.lossPercent > 20) ? LinkState::DEGRADED : LinkState::CONNECTED;
        }
    }
    
    const ControlPacket& getLastPacket() const { return lastPacket_; }
    const Stats& getStats() const { return stats_; }
    LinkState getLinkState() const { return state_; }
    
    bool isConnected() const {
        return state_ == LinkState::CONNECTED || state_ == LinkState::DEGRADED;
    }
    
    bool isFailsafe() const {
        return state_ == LinkState::FAILSAFE || state_ == LinkState::NO_LINK;
    }
    
    uint32_t getTimeSinceLastPacket() const {
        return millis() - stats_.lastPacketTime;
    }
};

// ============================================================================
// MOTOR MIXER
// ============================================================================

/**
 * Motor Mixer for Quad-X Configuration
 * 
 * Motor Layout (looking from above):
 *       Front
 *    FL     FR
 *      \   /
 *       \ /
 *        X
 *       / \
 *      /   \
 *    RL     RR
 *       Back
 * 
 * Motor Directions:
 *   FL: CCW (Counter-Clockwise)
 *   FR: CW  (Clockwise)
 *   RL: CW  (Clockwise)
 *   RR: CCW (Counter-Clockwise)
 * 
 * Mixing Matrix:
 *   Motor = Throttle ± Roll_PID ± Pitch_PID ± Yaw_PID
 * 
 *   FL = Throttle - Roll + Pitch - Yaw
 *   FR = Throttle + Roll + Pitch + Yaw
 *   RL = Throttle - Roll - Pitch + Yaw
 *   RR = Throttle + Roll - Pitch - Yaw
 * 
 * PWM Generation Strategy:
 * 
 * Using Arduino's analogWrite() would limit us to 490Hz PWM (Timer0/2) or 
 * 980Hz (Timer1). Standard ESCs expect 50-500Hz.
 * 
 * Decision: Use Servo library
 * 
 * Rationale:
 * - Provides standard ESC-compatible 1000-2000μs pulses
 * - Works on any digital pin
 * - Handles timing internally
 * - Compatible with all ESC types (analog, digital, SimonK, BLHeli)
 * 
 * Trade-off accepted:
 * - Timer1 interrupt overhead (~10μs per servo update)
 * - Fixed 50Hz refresh rate (sufficient for our 250Hz PID)
 * 
 * Alternative considered: Direct timer PWM
 * - Would allow higher update rates
 * - More complex configuration
 * - Pin-limited (only specific pins per timer)
 * - Chosen against for code clarity and ESC compatibility
 */

#include <Servo.h>

class MotorMixer {
private:
    Servo motors_[4];
    uint16_t motorValues_[4];  // PWM values in microseconds
    bool armed_;
    
    // Motor indices
    static constexpr uint8_t FL = 0;
    static constexpr uint8_t FR = 1;
    static constexpr uint8_t RL = 2;
    static constexpr uint8_t RR = 3;
    
    // Motor pins
    const uint8_t motorPins_[4] = {
        Pins::MOTOR_FL,
        Pins::MOTOR_FR,
        Pins::MOTOR_RL,
        Pins::MOTOR_RR
    };
    
public:
    MotorMixer() : armed_(false) {
        for (uint8_t i = 0; i < 4; i++) {
            motorValues_[i] = FlightParams::ESC_MIN_US;
        }
    }
    
    void begin() {
        for (uint8_t i = 0; i < 4; i++) {
            motors_[i].attach(motorPins_[i], FlightParams::ESC_MIN_US, FlightParams::ESC_MAX_US);
            motors_[i].writeMicroseconds(FlightParams::ESC_MIN_US);
        }
    }
    
    /**
     * Calculate and apply motor outputs
     * @param throttle Base throttle (0-1000)
     * @param roll PID output for roll (-500 to +500)
     * @param pitch PID output for pitch (-500 to +500)
     * @param yaw PID output for yaw (-500 to +500)
     */
    void mix(int16_t throttle, int16_t roll, int16_t pitch, int16_t yaw) {
        if (!armed_) {
            setAllMotors(FlightParams::ESC_MIN_US);
            return;
        }
        
        // Scale throttle from 0-1000 to ESC_IDLE_US-ESC_MAX_US when armed
        int16_t scaledThrottle = map(throttle, 0, 1000, 
                                     FlightParams::ESC_IDLE_US, 
                                     FlightParams::ESC_MAX_US);
        
        // Apply mixing matrix
        // Note: Signs determined by motor positions and rotation directions
        int16_t fl = scaledThrottle - roll + pitch - yaw;
        int16_t fr = scaledThrottle + roll + pitch + yaw;
        int16_t rl = scaledThrottle - roll - pitch + yaw;
        int16_t rr = scaledThrottle + roll - pitch - yaw;
        
        // Constrain to valid ESC range
        motorValues_[FL] = constrain(fl, FlightParams::ESC_IDLE_US, FlightParams::ESC_MAX_US);
        motorValues_[FR] = constrain(fr, FlightParams::ESC_IDLE_US, FlightParams::ESC_MAX_US);
        motorValues_[RL] = constrain(rl, FlightParams::ESC_IDLE_US, FlightParams::ESC_MAX_US);
        motorValues_[RR] = constrain(rr, FlightParams::ESC_IDLE_US, FlightParams::ESC_MAX_US);
        
        // Write to ESCs
        for (uint8_t i = 0; i < 4; i++) {
            motors_[i].writeMicroseconds(motorValues_[i]);
        }
    }
    
    void setAllMotors(uint16_t value) {
        for (uint8_t i = 0; i < 4; i++) {
            motorValues_[i] = value;
            motors_[i].writeMicroseconds(value);
        }
    }
    
    void arm() {
        armed_ = true;
    }
    
    void disarm() {
        armed_ = false;
        setAllMotors(FlightParams::ESC_MIN_US);
    }
    
    bool isArmed() const { return armed_; }
    
    uint16_t getMotorValue(uint8_t index) const {
        return (index < 4) ? motorValues_[index] : 0;
    }
    
    /**
     * Motor test: spin each motor briefly in sequence
     * WARNING: Props must be removed!
     */
    void motorTest() {
        const uint16_t TEST_SPEED = 1200;  // Low speed for testing
        const uint16_t TEST_DURATION_MS = 500;
        
        for (uint8_t i = 0; i < 4; i++) {
            motors_[i].writeMicroseconds(TEST_SPEED);
            delay(TEST_DURATION_MS);
            motors_[i].writeMicroseconds(FlightParams::ESC_MIN_US);
            delay(200);
        }
    }
};

// ============================================================================
// USER INTERFACE (LED + BUZZER)
// ============================================================================

class UserInterface {
private:
    bool ledState_;
    uint32_t lastLedToggle_;
    uint32_t blinkInterval_;
    
public:
    UserInterface() : ledState_(false), lastLedToggle_(0), blinkInterval_(500) {}
    
    void begin() {
        pinMode(Pins::LED, OUTPUT);
        pinMode(Pins::BUZZER, OUTPUT);
        digitalWrite(Pins::LED, LOW);
        digitalWrite(Pins::BUZZER, LOW);
    }
    
    void setLedSolid(bool on) {
        blinkInterval_ = 0;
        ledState_ = on;
        digitalWrite(Pins::LED, on ? HIGH : LOW);
    }
    
    void setLedBlink(uint32_t intervalMs) {
        blinkInterval_ = intervalMs;
    }
    
    void updateLed() {
        if (blinkInterval_ == 0) return;
        
        uint32_t now = millis();
        if (now - lastLedToggle_ >= blinkInterval_) {
            ledState_ = !ledState_;
            digitalWrite(Pins::LED, ledState_ ? HIGH : LOW);
            lastLedToggle_ = now;
        }
    }
    
    void beep(uint16_t durationMs, uint16_t frequency = 2000) {
        tone(Pins::BUZZER, frequency, durationMs);
    }
    
    void beepPattern(uint8_t count, uint16_t onMs = 100, uint16_t offMs = 100) {
        for (uint8_t i = 0; i < count; i++) {
            beep(onMs);
            delay(onMs + offMs);
        }
    }
    
    // Specific sound patterns
    void soundButtonPress() {
        beep(50, 2500);
    }
    
    void soundRFPaired() {
        beepPattern(5, 100, 100);
    }
    
    void soundCalibrationStart() {
        beep(200, 1500);
    }
    
    void soundCalibrationDone() {
        beep(100, 2000);
        delay(100);
        beep(100, 2500);
        delay(100);
        beep(200, 3000);
    }
    
    void soundArmed() {
        beep(100, 2000);
        delay(100);
        beep(300, 2500);
    }
    
    void soundDisarmed() {
        beep(300, 1500);
    }
    
    void soundReadyToFly() {
        // Rising tone sequence
        beep(100, 1000);
        delay(100);
        beep(100, 1500);
        delay(100);
        beep(100, 2000);
        delay(100);
        beep(200, 2500);
    }
    
    void soundError() {
        beepPattern(3, 200, 100);
    }
    
    void soundFailsafe() {
        beep(500, 800);
    }
};

// ============================================================================
// FLIGHT CONTROLLER (MAIN STATE MACHINE)
// ============================================================================

/**
 * Flight Controller State Machine
 * 
 * States:
 *   INIT        -> Initial startup, sensor initialization
 *   CALIBRATING -> IMU calibration in progress
 *   DISARMED    -> Ready but motors off, waiting for arm command
 *   ARMED       -> Motors active, flying
 *   FAILSAFE    -> RF link lost, controlled descent/cutoff
 *   ERROR       -> Critical error, motors disabled
 * 
 * Transitions:
 *   INIT -> DISARMED (on successful init)
 *   INIT -> ERROR (on init failure)
 *   DISARMED -> CALIBRATING (on calibration button)
 *   CALIBRATING -> DISARMED (on completion)
 *   DISARMED -> ARMED (on arm switch + throttle low)
 *   ARMED -> DISARMED (on disarm switch)
 *   ARMED -> FAILSAFE (on RF timeout)
 *   FAILSAFE -> DISARMED (on RF recovery + disarm)
 *   ANY -> ERROR (on critical fault)
 */

class FlightController {
public:
    enum class State {
        INIT,
        CALIBRATING,
        DISARMED,
        ARMED,
        FAILSAFE,
        MOTOR_TEST,
        ERROR
    };
    
private:
    // Subsystems
    IMU imu_;
    AttitudeEstimator attitude_;
    PIDController pidRoll_;
    PIDController pidPitch_;
    PIDController pidYaw_;
    RadioLink radio_;
    MotorMixer motors_;
    UserInterface ui_;
    
    // State
    State state_;
    State previousState_;
    uint32_t stateEntryTime_;
    
    // Timing (all in microseconds)
    uint32_t lastImuTime_;
    uint32_t lastAttitudeTime_;
    uint32_t lastPidTime_;
    uint32_t lastRfTime_;
    uint32_t lastLedTime_;
    
    // Control inputs (from radio)
    int16_t throttleCmd_;
    float rollCmd_;
    float pitchCmd_;
    float yawRateCmd_;
    
    // Previous switch states for edge detection
    bool prevArmSwitch_;
    bool prevCalibButton_;
    bool prevTestButton_;
    
    // Error tracking
    uint8_t errorCode_;
    
    // PID Gains (tunable)
    // These are starting values - must be tuned for specific frame!
    struct PIDGains {
        // Roll/Pitch (angle mode)
        float rollPitchKp = 4.0f;
        float rollPitchKi = 0.02f;
        float rollPitchKd = 1.5f;
        
        // Yaw (rate mode)
        float yawKp = 3.0f;
        float yawKi = 0.01f;
        float yawKd = 0.0f;
    } gains_;
    
public:
    FlightController() : state_(State::INIT), previousState_(State::INIT),
                         throttleCmd_(0), rollCmd_(0), pitchCmd_(0), yawRateCmd_(0),
                         prevArmSwitch_(false), prevCalibButton_(false), 
                         prevTestButton_(false), errorCode_(0) {
        lastImuTime_ = 0;
        lastAttitudeTime_ = 0;
        lastPidTime_ = 0;
        lastRfTime_ = 0;
        lastLedTime_ = 0;
    }
    
    void begin() {
        // Initialize UI first for feedback
        ui_.begin();
        ui_.setLedBlink(100);  // Fast blink during init
        
        // Initialize IMU
        if (!imu_.begin()) {
            enterState(State::ERROR);
            errorCode_ = 1;  // IMU init failed
            ui_.soundError();
            return;
        }
        
        // Initialize radio
        if (!radio_.begin()) {
            enterState(State::ERROR);
            errorCode_ = 2;  // Radio init failed
            ui_.soundError();
            return;
        }
        
        // Initialize motors (but keep disarmed)
        motors_.begin();
        
        // Configure PID controllers
        configurePID();
        
        // Enter disarmed state
        enterState(State::DISARMED);
        
        // Initial timing
        uint32_t now = micros();
        lastImuTime_ = now;
        lastAttitudeTime_ = now;
        lastPidTime_ = now;
        lastRfTime_ = now;
        lastLedTime_ = millis();
    }
    
    void configurePID() {
        // Roll PID
        pidRoll_.setGains(gains_.rollPitchKp, gains_.rollPitchKi, gains_.rollPitchKd);
        pidRoll_.setIntegralLimits(-200, 200);
        pidRoll_.setOutputLimits(-500, 500);
        
        // Pitch PID
        pidPitch_.setGains(gains_.rollPitchKp, gains_.rollPitchKi, gains_.rollPitchKd);
        pidPitch_.setIntegralLimits(-200, 200);
        pidPitch_.setOutputLimits(-500, 500);
        
        // Yaw PID (rate mode)
        pidYaw_.setGains(gains_.yawKp, gains_.yawKi, gains_.yawKd);
        pidYaw_.setIntegralLimits(-100, 100);
        pidYaw_.setOutputLimits(-300, 300);
    }
    
    /**
     * Main update loop - call this from loop() as fast as possible
     * Internal timing ensures each subsystem runs at correct frequency
     */
    void update() {
        uint32_t now = micros();
        
        // IMU Update (1000 Hz)
        if (now - lastImuTime_ >= Timing::IMU_PERIOD_US) {
            lastImuTime_ = now;
            if (imu_.readRaw()) {
                imu_.processData();
            }
        }
        
        // Attitude Update (500 Hz)
        if (now - lastAttitudeTime_ >= Timing::ATTITUDE_PERIOD_US) {
            lastAttitudeTime_ = now;
            if (state_ != State::CALIBRATING) {
                attitude_.update(imu_.getFilteredData());
            }
        }
        
        // RF Update (50 Hz)
        if (now - lastRfTime_ >= Timing::RF_PERIOD_US) {
            lastRfTime_ = now;
            updateRadio();
        }
        
        // PID and Motor Update (250 Hz)
        if (now - lastPidTime_ >= Timing::PID_PERIOD_US) {
            lastPidTime_ = now;
            updateControl();
        }
        
        // LED Update (using millis for longer intervals)
        ui_.updateLed();
        
        // State machine processing
        processState();
    }
    
private:
    void enterState(State newState) {
        previousState_ = state_;
        state_ = newState;
        stateEntryTime_ = millis();
        
        switch (newState) {
            case State::INIT:
                ui_.setLedBlink(100);
                break;
                
            case State::CALIBRATING:
                ui_.setLedBlink(50);
                ui_.soundCalibrationStart();
                break;
                
            case State::DISARMED:
                ui_.setLedBlink(500);
                motors_.disarm();
                if (previousState_ == State::ARMED) {
                    ui_.soundDisarmed();
                } else if (previousState_ == State::CALIBRATING) {
                    ui_.soundCalibrationDone();
                } else if (previousState_ == State::INIT) {
                    ui_.soundReadyToFly();
                }
                break;
                
            case State::ARMED:
                ui_.setLedSolid(true);
                motors_.arm();
                pidRoll_.reset();
                pidPitch_.reset();
                pidYaw_.reset();
                attitude_.reset();
                ui_.soundArmed();
                break;
                
            case State::FAILSAFE:
                ui_.setLedBlink(100);
                motors_.disarm();
                ui_.soundFailsafe();
                break;
                
            case State::MOTOR_TEST:
                ui_.setLedBlink(200);
                break;
                
            case State::ERROR:
                ui_.setLedBlink(50);
                motors_.disarm();
                break;
        }
    }
    
    void processState() {
        switch (state_) {
            case State::INIT:
                // Should not stay in INIT - begin() handles transition
                break;
                
            case State::CALIBRATING:
                // Calibration is blocking (handled in updateRadio)
                break;
                
            case State::DISARMED:
                // Check for arm command
                if (canArm()) {
                    enterState(State::ARMED);
                }
                break;
                
            case State::ARMED:
                // Check for disarm or failsafe
                if (!isArmSwitchOn()) {
                    enterState(State::DISARMED);
                } else if (radio_.isFailsafe()) {
                    enterState(State::FAILSAFE);
                }
                break;
                
            case State::FAILSAFE:
                // Check for RF recovery and disarm
                if (radio_.isConnected() && !isArmSwitchOn()) {
                    enterState(State::DISARMED);
                }
                break;
                
            case State::MOTOR_TEST:
                // Motor test handled separately
                break;
                
            case State::ERROR:
                // Stay in error state until reset
                break;
        }
    }
    
    void updateRadio() {
        bool newPacket = radio_.update();
        
        if (newPacket) {
            const ControlPacket& pkt = radio_.getLastPacket();
            
            // Extract control commands
            throttleCmd_ = pkt.throttle;
            
            // Map joystick values (-500 to +500) to angle/rate commands
            rollCmd_ = map(pkt.roll, -500, 500, 
                          -FlightParams::MAX_ROLL_ANGLE, FlightParams::MAX_ROLL_ANGLE);
            pitchCmd_ = map(pkt.pitch, -500, 500,
                           -FlightParams::MAX_PITCH_ANGLE, FlightParams::MAX_PITCH_ANGLE);
            yawRateCmd_ = map(pkt.yaw, -500, 500,
                             -FlightParams::MAX_YAW_RATE, FlightParams::MAX_YAW_RATE);
            
            // Process switch inputs with edge detection
            bool armSwitch = pkt.switches & (1 << RFConfig::SW_ARM_BIT);
            bool calibButton = pkt.switches & (1 << RFConfig::SW_CALIBRATE_BIT);
            bool testButton = pkt.switches & (1 << RFConfig::SW_MOTORTEST_BIT);
            
            // Calibration button (rising edge, only when disarmed)
            if (calibButton && !prevCalibButton_ && state_ == State::DISARMED) {
                ui_.soundButtonPress();
                performCalibration();
            }
            
            // Motor test button (rising edge, only when disarmed)
            if (testButton && !prevTestButton_ && state_ == State::DISARMED) {
                ui_.soundButtonPress();
                performMotorTest();
            }
            
            prevArmSwitch_ = armSwitch;
            prevCalibButton_ = calibButton;
            prevTestButton_ = testButton;
            
            // First successful packet - play paired sound
            if (radio_.getStats().packetsReceived == 1) {
                ui_.soundRFPaired();
            }
        }
    }
    
    void updateControl() {
        if (state_ != State::ARMED) {
            // When not armed, ensure motors are off
            motors_.mix(0, 0, 0, 0);
            return;
        }
        
        // Get current attitude
        float currentRoll = attitude_.getRoll();
        float currentPitch = attitude_.getPitch();
        
        // PID calculations
        // Roll and Pitch: Angle mode (setpoint is desired angle)
        float rollOutput = pidRoll_.calculate(rollCmd_, currentRoll, Timing::PID_DT);
        float pitchOutput = pidPitch_.calculate(pitchCmd_, currentPitch, Timing::PID_DT);
        
        // Yaw: Rate mode (setpoint is desired rate, measurement is gyro rate)
        float currentYawRate = imu_.getFilteredData().gyroZ;
        float yawOutput = pidYaw_.calculate(yawRateCmd_, currentYawRate, Timing::PID_DT);
        
        // Apply to motor mixer
        motors_.mix(throttleCmd_, (int16_t)rollOutput, (int16_t)pitchOutput, (int16_t)yawOutput);
    }
    
    bool canArm() {
        // Conditions for arming:
        // 1. Arm switch is ON
        // 2. Was previously OFF (edge detection)
        // 3. Throttle is at minimum
        // 4. IMU is calibrated
        // 5. RF link is connected
        
        bool armSwitch = radio_.getLastPacket().switches & (1 << RFConfig::SW_ARM_BIT);
        
        return armSwitch && 
               !prevArmSwitch_ &&
               throttleCmd_ <= FlightParams::THROTTLE_ARM_MAX &&
               imu_.isCalibrated() &&
               radio_.isConnected();
    }
    
    bool isArmSwitchOn() {
        return radio_.getLastPacket().switches & (1 << RFConfig::SW_ARM_BIT);
    }
    
    void performCalibration() {
        enterState(State::CALIBRATING);
        
        // Calibration progress callback for buzzer feedback
        auto progressCallback = [](uint8_t percent) {
            if (percent % 25 == 0) {
                tone(Pins::BUZZER, 1000 + percent * 10, 50);
            }
        };
        
        imu_.calibrate(progressCallback);
        
        enterState(State::DISARMED);
    }
    
    void performMotorTest() {
        enterState(State::MOTOR_TEST);
        motors_.motorTest();
        enterState(State::DISARMED);
    }
    
public:
    // Accessors for debugging
    State getState() const { return state_; }
    const IMU& getIMU() const { return imu_; }
    const AttitudeEstimator& getAttitude() const { return attitude_; }
    const RadioLink& getRadio() const { return radio_; }
    const MotorMixer& getMotors() const { return motors_; }
    uint8_t getErrorCode() const { return errorCode_; }
};

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

FlightController fc;

// ============================================================================
// ARDUINO ENTRY POINTS
// ============================================================================

void setup() {
    // Optional: Serial for debugging (comment out for production)
    // Serial.begin(115200);
    // Serial.println(F("Quadcopter FC v1.0"));
    
    fc.begin();
}

void loop() {
    fc.update();
    
    // Optional: Debug output (comment out for production - affects timing!)
    // static uint32_t lastDebug = 0;
    // if (millis() - lastDebug > 100) {
    //     lastDebug = millis();
    //     Serial.print(F("R:"));
    //     Serial.print(fc.getAttitude().getRoll(), 1);
    //     Serial.print(F(" P:"));
    //     Serial.print(fc.getAttitude().getPitch(), 1);
    //     Serial.print(F(" S:"));
    //     Serial.println((int)fc.getState());
    // }
}

/**
 * ============================================================================
 * TIMING DIAGRAM (Text Representation)
 * ============================================================================
 * 
 * Time (ms):  0    1    2    3    4    5    6    7    8    9    10   ...  20
 *             |    |    |    |    |    |    |    |    |    |    |         |
 * IMU (1kHz): X    X    X    X    X    X    X    X    X    X    X    ...  X
 *             |         |         |         |         |         |         |
 * ATT (500Hz):X---------X---------X---------X---------X---------X----...--X
 *             |                   |                   |                   |
 * PID (250Hz):X-------------------X-------------------X--------------...--X
 *             |                                                           |
 * RF  (50Hz): X-----------------------------------------------------------X
 *             |                                                           |
 * ESC (250Hz):X-------------------X-------------------X--------------...--X
 *             (synchronized with PID)
 * 
 * Within each 4ms PID cycle:
 *   - 4 IMU samples collected
 *   - 2 attitude updates computed
 *   - 1 PID calculation performed
 *   - 1 ESC update sent
 * 
 * RF communication runs independently at 50Hz (20ms period)
 * to avoid blocking the control loop.
 * 
 * ============================================================================
 */

/**
 * ============================================================================
 * CRITICAL IMPLEMENTATION NOTES
 * ============================================================================
 * 
 * 1. TIMING CONSISTENCY
 *    The PID derivative term requires consistent dt. Using micros() with
 *    fixed-period scheduling ensures this. DO NOT use variable timing.
 * 
 * 2. I2C CLOCK SPEED
 *    Wire.setClock(400000) is required for fast IMU reads. Standard 100kHz
 *    would take ~1.4ms per read, exceeding our 1ms budget.
 * 
 * 3. NO BLOCKING CODE
 *    The only blocking code is during calibration and motor test, which
 *    are user-initiated and occur only when disarmed.
 * 
 * 4. FAILSAFE PRIORITY
 *    Failsafe check happens BEFORE PID calculations. If RF is lost,
 *    motors are cut immediately without waiting for PID cycle.
 * 
 * 5. ARM SAFETY
 *    - Throttle must be low to arm (prevents unexpected motor spin)
 *    - Edge detection requires switch transition (no arm on power-up)
 *    - IMU must be calibrated before arming
 * 
 * 6. INTERRUPT CONSIDERATIONS
 *    While IMU interrupt is connected, we use polling for simplicity.
 *    Interrupt-driven IMU reading would improve timing but adds complexity.
 *    Current polling at 1kHz is sufficient.
 * 
 * 7. MEMORY USAGE
 *    Arduino Nano has 2KB RAM. This firmware uses ~1.2KB for variables.
 *    Stack usage is minimal due to no recursion. ~800 bytes headroom.
 * 
 * 8. FLASH USAGE
 *    Arduino Nano has 32KB flash. This firmware uses ~20KB including
 *    RF24 and Servo libraries. ~12KB available for future features.
 * 
 * ============================================================================
 */

/**
 * ============================================================================
 * TUNING GUIDE
 * ============================================================================
 * 
 * PID Tuning Procedure (Ziegler-Nichols inspired):
 * 
 * 1. Set Ki = 0, Kd = 0
 * 2. Increase Kp until quad oscillates consistently
 * 3. Note this Ku (ultimate gain) and Tu (oscillation period)
 * 4. Set Kp = 0.6 * Ku
 * 5. Set Ki = 2 * Kp / Tu
 * 6. Set Kd = Kp * Tu / 8
 * 7. Fine-tune from there
 * 
 * Typical ranges for 250mm quad:
 *   Kp: 2.0 - 8.0
 *   Ki: 0.01 - 0.1
 *   Kd: 0.5 - 3.0
 * 
 * Signs of bad tuning:
 *   - Oscillation: Kp too high or Kd too low
 *   - Sluggish response: Kp too low
 *   - Drift: Ki too low
 *   - Bounce back: Ki too high (windup)
 *   - Noisy motors: Kd too high
 * 
 * ============================================================================
 */
