/**
 * ============================================================================
 * QUADCOPTER FLIGHT CONTROLLER FIRMWARE v2.0
 * ============================================================================
 * 
 * Target: Arduino Nano (ATmega328P @ 16MHz)
 * 
 * Features:
 *   - MPU6050 with DMP (Jeff Rowberg's I2Cdev library)
 *   - MS5611 Barometer for altitude hold
 *   - NRF24L01 with ACK mode for reliable communication
 *   - Serial debug output for monitoring
 *   - Auto-disarm on connection loss
 * 
 * Hardware Configuration:
 *   MPU6050: I2C (SDA=A4, SCL=A5), INT→D2
 *   MS5611:  I2C (SDA=A4, SCL=A5), address 0x77
 *   Motors:  FL→D3, FR→D5, RL→D6, RR→D9
 *   NRF24L01: CE→D4, CSN→D10
 *   Buzzer: D8
 *   LED: D7
 * 
 * Required Libraries:
 *   - I2Cdev (Jeff Rowberg): https://github.com/jrowberg/i2cdevlib
 *   - MPU6050 (Jeff Rowberg): https://github.com/jrowberg/i2cdevlib
 *   - MS5611 (Rob Tillaart): https://github.com/RobTillaart/MS5611
 *   - RF24 (TMRh20): https://github.com/nRF24/RF24
 * 
 * Author: Flight Control Systems
 * Version: 2.0.0
 * 
 * ============================================================================
 */

// ============================================================================
// LIBRARY INCLUDES
// ============================================================================

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <MS5611.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

// ============================================================================
// CONFIGURATION - ADJUST THESE FOR YOUR SETUP
// ============================================================================

// RF Channel - MUST MATCH ON BOTH DEVICES (0-125, avoid WiFi: use 100+)
#define RF_CHANNEL          108

// Set to true to enable detailed serial debug output
#define DEBUG_SERIAL        true
#define SERIAL_BAUD         115200

// Altitude hold settings
#define ALTITUDE_HOLD_ENABLED  true
#define TAKEOFF_ALTITUDE_M     1.0f    // Target altitude for auto-takeoff
#define LANDING_RATE_MS        0.3f    // Descent rate in m/s for landing

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
    
    // MPU6050 Interrupt
    constexpr uint8_t MPU_INT = 2;
    
    // User Interface
    constexpr uint8_t LED    = 7;
    constexpr uint8_t BUZZER = 8;
}

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

namespace Timing {
    // Loop periods in microseconds
    constexpr uint32_t DMP_PERIOD_US      = 10000;   // 100 Hz - DMP output rate
    constexpr uint32_t PID_PERIOD_US      = 4000;    // 250 Hz - Control loop
    constexpr uint32_t RF_PERIOD_US       = 20000;   // 50 Hz  - Radio
    constexpr uint32_t BARO_PERIOD_US     = 25000;   // 40 Hz  - Barometer
    constexpr uint32_t DEBUG_PERIOD_MS    = 200;     // 5 Hz   - Serial output
    constexpr uint32_t LED_PERIOD_MS      = 500;     // 2 Hz   - LED blink
    
    // Derived constants
    constexpr float PID_DT = PID_PERIOD_US / 1000000.0f;
    
    // Failsafe timeouts (milliseconds)
    constexpr uint32_t RF_TIMEOUT_MS      = 500;     // Disarm after this
    constexpr uint32_t RF_WARN_MS         = 200;     // Warning threshold
}

// ============================================================================
// FLIGHT PARAMETERS
// ============================================================================

namespace FlightParams {
    // Angle limits (degrees)
    constexpr float MAX_ROLL_ANGLE  = 45.0f;
    constexpr float MAX_PITCH_ANGLE = 45.0f;
    constexpr float MAX_YAW_RATE    = 180.0f;  // deg/s
    
    // Motor limits (microseconds PWM)
    constexpr uint16_t ESC_MIN_US  = 1000;
    constexpr uint16_t ESC_MAX_US  = 2000;
    constexpr uint16_t ESC_ARM_US  = 1000;
    constexpr uint16_t ESC_IDLE_US = 1150;
    
    // Throttle
    constexpr uint16_t THROTTLE_MIN = 0;
    constexpr uint16_t THROTTLE_MAX = 1000;
    constexpr uint16_t THROTTLE_ARM_MAX = 50;
    
    // Altitude hold
    constexpr float ALT_HOLD_DEADBAND = 50;  // Throttle deadband for alt hold
}

// ============================================================================
// RF PROTOCOL - ACK MODE FOR RELIABLE COMMUNICATION
// ============================================================================

/**
 * ACK vs NO_ACK Decision: Using ACK Mode
 * 
 * Reasons for ACK mode in this application:
 * 1. RELIABILITY: Ensures packets are received, critical for debugging
 * 2. FEEDBACK: Know immediately if communication fails
 * 3. LATENCY: With 2Mbps and short packets, ACK adds only ~0.5ms
 * 4. SAFETY: Better to have confirmed commands than fast unconfirmed ones
 * 5. DEBUG: Can track actual packet success rate
 * 
 * Configuration:
 * - Auto-ACK enabled
 * - 3 retries with 1ms delay
 * - Total worst-case latency: ~4ms (acceptable for 50Hz control)
 */

namespace RFConfig {
    constexpr uint8_t CHANNEL = RF_CHANNEL;
    constexpr uint8_t PAYLOAD_SIZE = 16;
    const uint8_t PIPE_ADDRESS[6] = "QUAD1";  // 5-byte address + null
    
    // ACK configuration
    constexpr uint8_t RETRY_DELAY = 5;   // 5 = 1500us delay
    constexpr uint8_t RETRY_COUNT = 3;   // 3 retries
    
    // Switch bit definitions
    constexpr uint8_t SW_ARM_BIT       = 0;
    constexpr uint8_t SW_CALIBRATE_BIT = 1;
    constexpr uint8_t SW_MOTORTEST_BIT = 2;
    constexpr uint8_t SW_ALTHOLD_BIT   = 3;
}

// Control packet structure (16 bytes) - MUST MATCH TRANSMITTER
struct __attribute__((packed)) ControlPacket {
    uint16_t throttle;    // 0-1000
    int16_t  yaw;         // -500 to +500
    int16_t  pitch;       // -500 to +500
    int16_t  roll;        // -500 to +500
    uint8_t  switches;    // Bitfield
    uint8_t  checksum;    // XOR checksum
    uint32_t sequence;    // Packet counter
    uint8_t  channel;     // RF channel (for verification)
    uint8_t  reserved;
    
    bool validateChecksum() const {
        const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
        uint8_t calc = 0;
        for (uint8_t i = 0; i < 9; i++) {
            calc ^= data[i];
        }
        return calc == checksum;
    }
};

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// MPU6050 with DMP
MPU6050 mpu;

// MS5611 Barometer
MS5611 baro(0x77);  // Default I2C address

// NRF24L01 Radio
RF24 radio(Pins::RF_CE, Pins::RF_CSN);

// ESC Servos
Servo motorFL, motorFR, motorRL, motorRR;

// ============================================================================
// DMP VARIABLES
// ============================================================================

// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3];  // yaw, pitch, roll in radians

// Converted angles in degrees
float rollAngle = 0;
float pitchAngle = 0;
float yawAngle = 0;
float yawRate = 0;

// Previous yaw for rate calculation
float prevYaw = 0;
uint32_t prevYawTime = 0;

// Interrupt flag
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

// ============================================================================
// BAROMETER VARIABLES
// ============================================================================

float baselinePressure = 0;
float currentAltitude = 0;
float altitudeFiltered = 0;
float verticalVelocity = 0;
float prevAltitude = 0;
uint32_t prevAltTime = 0;
bool baroReady = false;

// Complementary filter for altitude
constexpr float ALT_FILTER_ALPHA = 0.9f;

// ============================================================================
// RADIO VARIABLES
// ============================================================================

ControlPacket rxPacket;
uint32_t lastPacketTime = 0;
uint32_t packetsReceived = 0;
uint32_t packetsLost = 0;
uint32_t lastSequence = 0;
bool rfConnected = false;
bool rfPaired = false;

// ============================================================================
// FLIGHT CONTROL VARIABLES
// ============================================================================

enum class FlightState {
    INIT,
    DISARMED,
    ARMED,
    FAILSAFE,
    CALIBRATING,
    ERROR
};

FlightState flightState = FlightState::INIT;
bool motorsArmed = false;

// Control inputs
int16_t throttleCmd = 0;
float rollCmd = 0;
float pitchCmd = 0;
float yawRateCmd = 0;
bool altHoldEnabled = false;
float targetAltitude = 0;

// PID outputs
float rollOutput = 0;
float pitchOutput = 0;
float yawOutput = 0;
float altOutput = 0;

// Motor outputs
uint16_t motorFL_us = FlightParams::ESC_MIN_US;
uint16_t motorFR_us = FlightParams::ESC_MIN_US;
uint16_t motorRL_us = FlightParams::ESC_MIN_US;
uint16_t motorRR_us = FlightParams::ESC_MIN_US;

// Switch states
bool prevArmSwitch = false;
bool prevCalibSwitch = false;
bool prevTestSwitch = false;

// ============================================================================
// PID CONTROLLER CLASS
// ============================================================================

class PID {
public:
    float kp, ki, kd;
    float integral;
    float prevError;
    float prevMeasurement;
    float outputMin, outputMax;
    float integralMax;
    bool initialized;
    
    PID(float p = 0, float i = 0, float d = 0) 
        : kp(p), ki(i), kd(d), integral(0), prevError(0), prevMeasurement(0),
          outputMin(-500), outputMax(500), integralMax(200), initialized(false) {}
    
    void reset() {
        integral = 0;
        prevError = 0;
        prevMeasurement = 0;
        initialized = false;
    }
    
    float compute(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;
        
        if (!initialized) {
            prevMeasurement = measurement;
            prevError = error;
            initialized = true;
        }
        
        // Proportional
        float pTerm = kp * error;
        
        // Integral with anti-windup
        integral += ki * error * dt;
        integral = constrain(integral, -integralMax, integralMax);
        
        // Derivative on measurement (not error) to avoid derivative kick
        float derivative = -(measurement - prevMeasurement) / dt;
        float dTerm = kd * derivative;
        
        prevMeasurement = measurement;
        prevError = error;
        
        float output = pTerm + integral + dTerm;
        return constrain(output, outputMin, outputMax);
    }
};

// PID Controllers
PID pidRoll(4.0f, 0.02f, 1.5f);
PID pidPitch(4.0f, 0.02f, 1.5f);
PID pidYaw(3.0f, 0.01f, 0.0f);
PID pidAlt(50.0f, 0.5f, 30.0f);  // Altitude PID

// ============================================================================
// TIMING VARIABLES
// ============================================================================

uint32_t lastDmpTime = 0;
uint32_t lastPidTime = 0;
uint32_t lastRfTime = 0;
uint32_t lastBaroTime = 0;
uint32_t lastDebugTime = 0;
uint32_t lastLedTime = 0;
bool ledState = false;

// ============================================================================
// BUZZER FUNCTIONS
// ============================================================================

void beep(uint16_t duration, uint16_t freq = 2000) {
    tone(Pins::BUZZER, freq, duration);
}

void beepBlocking(uint16_t duration, uint16_t freq = 2000) {
    tone(Pins::BUZZER, freq);
    delay(duration);
    noTone(Pins::BUZZER);
}

void buzzerPaired() {
    // 5 quick beeps for successful pairing
    for (int i = 0; i < 5; i++) {
        beepBlocking(80, 2500);
        delay(80);
    }
}

void buzzerPacketReceived() {
    // Very short beep for packet received (only first few)
    static uint32_t lastBeep = 0;
    if (millis() - lastBeep > 500) {  // Max once per 500ms
        beep(20, 3000);
        lastBeep = millis();
    }
}

void buzzerArmed() {
    beepBlocking(100, 2000);
    delay(50);
    beepBlocking(200, 2500);
}

void buzzerDisarmed() {
    beepBlocking(300, 1500);
}

void buzzerError() {
    for (int i = 0; i < 3; i++) {
        beepBlocking(150, 800);
        delay(100);
    }
}

void buzzerCalibrationDone() {
    beepBlocking(100, 1500);
    delay(50);
    beepBlocking(100, 2000);
    delay(50);
    beepBlocking(200, 2500);
}

void buzzerStartup() {
    beepBlocking(100, 1500);
    delay(50);
    beepBlocking(100, 2000);
    delay(50);
    beepBlocking(100, 2500);
    delay(50);
    beepBlocking(200, 3000);
}

// ============================================================================
// SERIAL DEBUG OUTPUT
// ============================================================================

void printDebugHeader() {
    Serial.println(F("\n========================================"));
    Serial.println(F("  QUADCOPTER FLIGHT CONTROLLER v2.0"));
    Serial.println(F("========================================"));
    Serial.print(F("RF Channel: "));
    Serial.println(RF_CHANNEL);
    Serial.println(F(""));
}

void printStatus() {
    if (!DEBUG_SERIAL) return;
    
    Serial.println(F("\n--- STATUS ---"));
    
    // Flight State
    Serial.print(F("State: "));
    switch (flightState) {
        case FlightState::INIT:       Serial.println(F("INIT")); break;
        case FlightState::DISARMED:   Serial.println(F("DISARMED")); break;
        case FlightState::ARMED:      Serial.println(F("ARMED")); break;
        case FlightState::FAILSAFE:   Serial.println(F("FAILSAFE")); break;
        case FlightState::CALIBRATING:Serial.println(F("CALIBRATING")); break;
        case FlightState::ERROR:      Serial.println(F("ERROR")); break;
    }
    
    // RF Status
    Serial.print(F("RF: "));
    if (rfConnected) {
        Serial.print(F("CONNECTED | Pkts: "));
        Serial.print(packetsReceived);
        Serial.print(F(" | Lost: "));
        Serial.print(packetsLost);
        Serial.print(F(" | Last: "));
        Serial.print(millis() - lastPacketTime);
        Serial.println(F("ms ago"));
    } else {
        Serial.print(F("DISCONNECTED | Last: "));
        Serial.print(millis() - lastPacketTime);
        Serial.println(F("ms ago"));
    }
    
    // IMU Data
    Serial.print(F("IMU: Roll="));
    Serial.print(rollAngle, 1);
    Serial.print(F("° Pitch="));
    Serial.print(pitchAngle, 1);
    Serial.print(F("° Yaw="));
    Serial.print(yawAngle, 1);
    Serial.println(F("°"));
    
    // Barometer Data
    if (baroReady) {
        Serial.print(F("ALT: "));
        Serial.print(altitudeFiltered, 2);
        Serial.print(F("m | Vvel: "));
        Serial.print(verticalVelocity, 2);
        Serial.println(F("m/s"));
    }
    
    // Control Inputs
    Serial.print(F("CMD: Thr="));
    Serial.print(throttleCmd);
    Serial.print(F(" R="));
    Serial.print(rollCmd, 1);
    Serial.print(F(" P="));
    Serial.print(pitchCmd, 1);
    Serial.print(F(" Y="));
    Serial.print(yawRateCmd, 1);
    Serial.print(F(" AltHold="));
    Serial.println(altHoldEnabled ? "ON" : "OFF");
    
    // Motor Outputs
    Serial.print(F("MTR: FL="));
    Serial.print(motorFL_us);
    Serial.print(F(" FR="));
    Serial.print(motorFR_us);
    Serial.print(F(" RL="));
    Serial.print(motorRL_us);
    Serial.print(F(" RR="));
    Serial.println(motorRR_us);
    
    Serial.println(F("---"));
}

// ============================================================================
// SETUP FUNCTIONS
// ============================================================================

bool setupMPU() {
    Serial.println(F("Initializing MPU6050..."));
    
    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);
    
    // Initialize MPU6050
    mpu.initialize();
    
    // Verify connection
    if (!mpu.testConnection()) {
        Serial.println(F("ERROR: MPU6050 connection failed!"));
        return false;
    }
    Serial.println(F("MPU6050 connected."));
    
    // Initialize DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // Set gyro offsets (these should be calibrated for your specific MPU6050)
    // You can get these values using the IMU_Zero example from I2Cdev
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    
    if (devStatus == 0) {
        // Calibration
        Serial.println(F("Calibrating DMP (keep level and still)..."));
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println(F("DMP calibration complete."));
        
        // Enable DMP
        mpu.setDMPEnabled(true);
        
        // Setup interrupt
        pinMode(Pins::MPU_INT, INPUT);
        attachInterrupt(digitalPinToInterrupt(Pins::MPU_INT), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        
        // Get packet size
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        dmpReady = true;
        Serial.print(F("DMP ready! Packet size: "));
        Serial.println(packetSize);
        return true;
    } else {
        Serial.print(F("ERROR: DMP initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        return false;
    }
}

bool setupBarometer() {
    Serial.println(F("Initializing MS5611..."));
    
    if (!baro.begin()) {
        Serial.println(F("ERROR: MS5611 not found!"));
        return false;
    }
    
    Serial.println(F("MS5611 connected."));
    
    // Get baseline pressure (average of 10 readings)
    Serial.println(F("Getting baseline pressure..."));
    float sumPressure = 0;
    for (int i = 0; i < 10; i++) {
        baro.read();
        sumPressure += baro.getPressure();
        delay(50);
    }
    baselinePressure = sumPressure / 10.0f;
    
    Serial.print(F("Baseline pressure: "));
    Serial.print(baselinePressure);
    Serial.println(F(" mbar"));
    
    baroReady = true;
    return true;
}

bool setupRadio() {
    Serial.println(F("Initializing NRF24L01..."));
    Serial.print(F("Channel: "));
    Serial.println(RFConfig::CHANNEL);
    
    if (!radio.begin()) {
        Serial.println(F("ERROR: NRF24L01 not found!"));
        return false;
    }
    
    // Configure radio for ACK mode
    radio.setChannel(RFConfig::CHANNEL);
    radio.setDataRate(RF24_2MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(RFConfig::PAYLOAD_SIZE);
    
    // Enable ACK mode
    radio.setAutoAck(true);
    radio.setRetries(RFConfig::RETRY_DELAY, RFConfig::RETRY_COUNT);
    
    // CRC for data integrity
    radio.setCRCLength(RF24_CRC_16);
    
    // Open reading pipe
    radio.openReadingPipe(1, RFConfig::PIPE_ADDRESS);
    
    // Start listening
    radio.startListening();
    
    Serial.println(F("NRF24L01 configured in ACK mode."));
    Serial.println(F("Waiting for transmitter..."));
    
    return true;
}

void setupMotors() {
    Serial.println(F("Initializing ESCs..."));
    
    motorFL.attach(Pins::MOTOR_FL, FlightParams::ESC_MIN_US, FlightParams::ESC_MAX_US);
    motorFR.attach(Pins::MOTOR_FR, FlightParams::ESC_MIN_US, FlightParams::ESC_MAX_US);
    motorRL.attach(Pins::MOTOR_RL, FlightParams::ESC_MIN_US, FlightParams::ESC_MAX_US);
    motorRR.attach(Pins::MOTOR_RR, FlightParams::ESC_MIN_US, FlightParams::ESC_MAX_US);
    
    // Initialize to minimum
    motorFL.writeMicroseconds(FlightParams::ESC_MIN_US);
    motorFR.writeMicroseconds(FlightParams::ESC_MIN_US);
    motorRL.writeMicroseconds(FlightParams::ESC_MIN_US);
    motorRR.writeMicroseconds(FlightParams::ESC_MIN_US);
    
    Serial.println(F("ESCs initialized."));
}

// ============================================================================
// UPDATE FUNCTIONS
// ============================================================================

void updateDMP() {
    if (!dmpReady) return;
    
    // Check for DMP data
    if (!mpuInterrupt && fifoCount < packetSize) {
        return;
    }
    
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    // Check for overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        return;
    }
    
    // Check for DMP data ready
    if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }
        
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        
        // Get quaternion and calculate angles
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // Convert to degrees (DMP gives radians)
        yawAngle = ypr[0] * 180.0f / M_PI;
        pitchAngle = ypr[1] * 180.0f / M_PI;
        rollAngle = ypr[2] * 180.0f / M_PI;
        
        // Calculate yaw rate
        uint32_t now = micros();
        if (prevYawTime > 0) {
            float dt = (now - prevYawTime) / 1000000.0f;
            if (dt > 0 && dt < 0.1f) {
                float yawDiff = yawAngle - prevYaw;
                // Handle wrap-around
                if (yawDiff > 180) yawDiff -= 360;
                if (yawDiff < -180) yawDiff += 360;
                yawRate = yawDiff / dt;
            }
        }
        prevYaw = yawAngle;
        prevYawTime = now;
    }
}

void updateBarometer() {
    if (!baroReady) return;
    
    // Read barometer
    baro.read();
    float pressure = baro.getPressure();
    
    // Calculate altitude using barometric formula
    // altitude = 44330 * (1 - (P/P0)^0.1903)
    currentAltitude = 44330.0f * (1.0f - pow(pressure / baselinePressure, 0.1903f));
    
    // Apply complementary filter
    altitudeFiltered = ALT_FILTER_ALPHA * altitudeFiltered + (1.0f - ALT_FILTER_ALPHA) * currentAltitude;
    
    // Calculate vertical velocity
    uint32_t now = micros();
    if (prevAltTime > 0) {
        float dt = (now - prevAltTime) / 1000000.0f;
        if (dt > 0 && dt < 0.1f) {
            verticalVelocity = (altitudeFiltered - prevAltitude) / dt;
        }
    }
    prevAltitude = altitudeFiltered;
    prevAltTime = now;
}

void updateRadio() {
    uint32_t now = millis();
    
    // Check for packets
    if (radio.available()) {
        ControlPacket packet;
        radio.read(&packet, sizeof(packet));
        
        // Validate checksum
        if (packet.validateChecksum()) {
            // Check for lost packets
            if (packetsReceived > 0 && packet.sequence > lastSequence + 1) {
                packetsLost += (packet.sequence - lastSequence - 1);
            }
            lastSequence = packet.sequence;
            
            // Update control values
            rxPacket = packet;
            lastPacketTime = now;
            packetsReceived++;
            
            // First packet - pairing success!
            if (!rfPaired) {
                rfPaired = true;
                Serial.println(F("\n*** RF PAIRED! ***"));
                buzzerPaired();
            }
            
            // Connection status
            if (!rfConnected) {
                rfConnected = true;
                Serial.println(F("RF: Connected"));
            }
            
            // Beep on packet (occasionally)
            buzzerPacketReceived();
        }
    }
    
    // Check for connection timeout
    uint32_t timeSincePacket = now - lastPacketTime;
    
    if (rfConnected && timeSincePacket > Timing::RF_TIMEOUT_MS) {
        rfConnected = false;
        Serial.println(F("RF: CONNECTION LOST!"));
        
        // SAFETY: Force disarm on connection loss
        if (motorsArmed) {
            motorsArmed = false;
            flightState = FlightState::FAILSAFE;
            Serial.println(F("*** FAILSAFE - MOTORS DISARMED ***"));
            beep(500, 800);  // Long low beep for failsafe
        }
    }
}

void processCommands() {
    if (!rfConnected) {
        // No commands if not connected - safety
        throttleCmd = 0;
        rollCmd = 0;
        pitchCmd = 0;
        yawRateCmd = 0;
        return;
    }
    
    // Extract commands from packet
    throttleCmd = rxPacket.throttle;
    
    // Map stick values to angles/rates
    rollCmd = map(rxPacket.roll, -500, 500, 
                  -FlightParams::MAX_ROLL_ANGLE, FlightParams::MAX_ROLL_ANGLE);
    pitchCmd = map(rxPacket.pitch, -500, 500,
                   -FlightParams::MAX_PITCH_ANGLE, FlightParams::MAX_PITCH_ANGLE);
    yawRateCmd = map(rxPacket.yaw, -500, 500,
                     -FlightParams::MAX_YAW_RATE, FlightParams::MAX_YAW_RATE);
    
    // Process switches
    bool armSwitch = rxPacket.switches & (1 << RFConfig::SW_ARM_BIT);
    bool calibSwitch = rxPacket.switches & (1 << RFConfig::SW_CALIBRATE_BIT);
    bool testSwitch = rxPacket.switches & (1 << RFConfig::SW_MOTORTEST_BIT);
    bool altHoldSwitch = rxPacket.switches & (1 << RFConfig::SW_ALTHOLD_BIT);
    
    // Arm/Disarm logic
    if (armSwitch && !prevArmSwitch) {
        // Rising edge of arm switch
        if (!motorsArmed && flightState == FlightState::DISARMED) {
            // Check arming conditions
            if (throttleCmd <= FlightParams::THROTTLE_ARM_MAX && rfConnected && dmpReady) {
                motorsArmed = true;
                flightState = FlightState::ARMED;
                pidRoll.reset();
                pidPitch.reset();
                pidYaw.reset();
                pidAlt.reset();
                targetAltitude = altitudeFiltered;
                Serial.println(F("*** ARMED ***"));
                buzzerArmed();
            } else {
                Serial.println(F("Cannot arm: Check throttle, RF, and sensors"));
                buzzerError();
            }
        }
    } else if (!armSwitch && prevArmSwitch) {
        // Falling edge of arm switch - disarm
        if (motorsArmed) {
            motorsArmed = false;
            flightState = FlightState::DISARMED;
            Serial.println(F("*** DISARMED ***"));
            buzzerDisarmed();
        }
    }
    
    // Also disarm if switch is off (safety)
    if (!armSwitch && motorsArmed) {
        motorsArmed = false;
        flightState = FlightState::DISARMED;
        Serial.println(F("*** DISARMED (switch off) ***"));
        buzzerDisarmed();
    }
    
    // Altitude hold toggle
    altHoldEnabled = altHoldSwitch && ALTITUDE_HOLD_ENABLED && baroReady;
    
    // Update target altitude when alt hold is first enabled
    if (altHoldEnabled && !prevArmSwitch) {  // Using prevArmSwitch as proxy for state change
        targetAltitude = altitudeFiltered;
    }
    
    prevArmSwitch = armSwitch;
    prevCalibSwitch = calibSwitch;
    prevTestSwitch = testSwitch;
}

void updatePID() {
    if (!motorsArmed) {
        // Reset PIDs when disarmed
        pidRoll.reset();
        pidPitch.reset();
        pidYaw.reset();
        pidAlt.reset();
        rollOutput = 0;
        pitchOutput = 0;
        yawOutput = 0;
        altOutput = 0;
        return;
    }
    
    // Roll and Pitch: Angle mode (self-leveling)
    rollOutput = pidRoll.compute(rollCmd, rollAngle, Timing::PID_DT);
    pitchOutput = pidPitch.compute(pitchCmd, pitchAngle, Timing::PID_DT);
    
    // Yaw: Rate mode
    yawOutput = pidYaw.compute(yawRateCmd, yawRate, Timing::PID_DT);
    
    // Altitude hold
    if (altHoldEnabled) {
        // Adjust target altitude based on throttle deviation from center
        float throttleDeviation = throttleCmd - 500;  // Center = 500
        if (abs(throttleDeviation) > FlightParams::ALT_HOLD_DEADBAND) {
            // Throttle outside deadband - adjust target altitude
            targetAltitude += (throttleDeviation / 500.0f) * 0.02f;  // Adjust rate
        }
        
        // PID for altitude
        altOutput = pidAlt.compute(targetAltitude, altitudeFiltered, Timing::PID_DT);
    } else {
        altOutput = 0;
    }
}

void updateMotors() {
    if (!motorsArmed) {
        // Motors off when disarmed
        motorFL_us = FlightParams::ESC_MIN_US;
        motorFR_us = FlightParams::ESC_MIN_US;
        motorRL_us = FlightParams::ESC_MIN_US;
        motorRR_us = FlightParams::ESC_MIN_US;
    } else {
        // Calculate base throttle
        int16_t baseThrottle;
        if (altHoldEnabled) {
            // Altitude hold: use PID output + hover throttle estimate
            baseThrottle = 500 + (int16_t)altOutput;  // 500 = approximate hover
        } else {
            // Manual throttle
            baseThrottle = map(throttleCmd, 0, 1000, 
                              FlightParams::ESC_IDLE_US, FlightParams::ESC_MAX_US);
            baseThrottle -= FlightParams::ESC_MIN_US;  // Make relative to min
        }
        
        // Mix outputs
        // Motor layout (Quad-X):
        //   FL (CCW) = Throttle - Roll + Pitch - Yaw
        //   FR (CW)  = Throttle + Roll + Pitch + Yaw
        //   RL (CW)  = Throttle - Roll - Pitch + Yaw
        //   RR (CCW) = Throttle + Roll - Pitch - Yaw
        
        int16_t fl = baseThrottle - (int16_t)rollOutput + (int16_t)pitchOutput - (int16_t)yawOutput;
        int16_t fr = baseThrottle + (int16_t)rollOutput + (int16_t)pitchOutput + (int16_t)yawOutput;
        int16_t rl = baseThrottle - (int16_t)rollOutput - (int16_t)pitchOutput + (int16_t)yawOutput;
        int16_t rr = baseThrottle + (int16_t)rollOutput - (int16_t)pitchOutput - (int16_t)yawOutput;
        
        // Convert to PWM and constrain
        motorFL_us = constrain(fl + FlightParams::ESC_MIN_US, FlightParams::ESC_IDLE_US, FlightParams::ESC_MAX_US);
        motorFR_us = constrain(fr + FlightParams::ESC_MIN_US, FlightParams::ESC_IDLE_US, FlightParams::ESC_MAX_US);
        motorRL_us = constrain(rl + FlightParams::ESC_MIN_US, FlightParams::ESC_IDLE_US, FlightParams::ESC_MAX_US);
        motorRR_us = constrain(rr + FlightParams::ESC_MIN_US, FlightParams::ESC_IDLE_US, FlightParams::ESC_MAX_US);
    }
    
    // Write to ESCs
    motorFL.writeMicroseconds(motorFL_us);
    motorFR.writeMicroseconds(motorFR_us);
    motorRL.writeMicroseconds(motorRL_us);
    motorRR.writeMicroseconds(motorRR_us);
}

void updateLED() {
    uint32_t now = millis();
    
    switch (flightState) {
        case FlightState::ARMED:
            // Solid on when armed
            digitalWrite(Pins::LED, HIGH);
            break;
            
        case FlightState::DISARMED:
            // Slow blink when disarmed
            if (now - lastLedTime >= 500) {
                lastLedTime = now;
                ledState = !ledState;
                digitalWrite(Pins::LED, ledState);
            }
            break;
            
        case FlightState::FAILSAFE:
            // Fast blink in failsafe
            if (now - lastLedTime >= 100) {
                lastLedTime = now;
                ledState = !ledState;
                digitalWrite(Pins::LED, ledState);
            }
            break;
            
        default:
            // Very fast blink during init/error
            if (now - lastLedTime >= 50) {
                lastLedTime = now;
                ledState = !ledState;
                digitalWrite(Pins::LED, ledState);
            }
            break;
    }
}

// ============================================================================
// MAIN SETUP
// ============================================================================

void setup() {
    // Initialize serial first for debugging
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000);  // Wait for serial (with timeout)
    
    printDebugHeader();
    
    // Initialize pins
    pinMode(Pins::LED, OUTPUT);
    pinMode(Pins::BUZZER, OUTPUT);
    digitalWrite(Pins::LED, HIGH);
    
    // Startup sound
    buzzerStartup();
    
    // Initialize subsystems
    bool initOk = true;
    
    if (!setupMPU()) {
        Serial.println(F("MPU6050 FAILED!"));
        initOk = false;
    }
    
    if (!setupBarometer()) {
        Serial.println(F("MS5611 FAILED! (Continuing without altitude hold)"));
        // Not fatal - can fly without altitude hold
    }
    
    if (!setupRadio()) {
        Serial.println(F("NRF24L01 FAILED!"));
        initOk = false;
    }
    
    setupMotors();
    
    // Initialize PIDs
    pidRoll.outputMin = -500;
    pidRoll.outputMax = 500;
    pidPitch.outputMin = -500;
    pidPitch.outputMax = 500;
    pidYaw.outputMin = -300;
    pidYaw.outputMax = 300;
    pidAlt.outputMin = -300;
    pidAlt.outputMax = 300;
    
    // Set initial state
    if (initOk) {
        flightState = FlightState::DISARMED;
        Serial.println(F("\n*** SYSTEM READY ***"));
        Serial.println(F("Waiting for RC connection..."));
        Serial.println(F("(ARM switch must be OFF, throttle at minimum to arm)"));
    } else {
        flightState = FlightState::ERROR;
        Serial.println(F("\n*** INITIALIZATION FAILED ***"));
        buzzerError();
    }
    
    // Initialize timing
    uint32_t now = micros();
    lastDmpTime = now;
    lastPidTime = now;
    lastRfTime = now;
    lastBaroTime = now;
    lastDebugTime = millis();
    lastLedTime = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    uint32_t nowMicros = micros();
    uint32_t nowMillis = millis();
    
    // Always update DMP (interrupt-driven, but check regularly)
    updateDMP();
    
    // RF Update (50 Hz)
    if (nowMicros - lastRfTime >= Timing::RF_PERIOD_US) {
        lastRfTime = nowMicros;
        updateRadio();
        processCommands();
    }
    
    // Barometer Update (40 Hz)
    if (nowMicros - lastBaroTime >= Timing::BARO_PERIOD_US) {
        lastBaroTime = nowMicros;
        updateBarometer();
    }
    
    // PID Update (250 Hz)
    if (nowMicros - lastPidTime >= Timing::PID_PERIOD_US) {
        lastPidTime = nowMicros;
        updatePID();
        updateMotors();
    }
    
    // LED Update
    updateLED();
    
    // Debug Output (5 Hz)
    if (DEBUG_SERIAL && (nowMillis - lastDebugTime >= Timing::DEBUG_PERIOD_MS)) {
        lastDebugTime = nowMillis;
        printStatus();
    }
}
