/**
 * ============================================================================
 * QUADCOPTER REMOTE CONTROLLER FIRMWARE
 * ============================================================================
 * 
 * Target: Arduino Nano (ATmega328P @ 16MHz)
 * 
 * Hardware Configuration:
 *   NRF24L01: CE → D9, CSN → D10
 *   Toggle Switch (Arm): D2
 *   Toggle Switch (Aux): D3
 *   Push Button (Calibrate): D4
 *   Push Button (Motor Test): D5
 *   Joystick Throttle: A0
 *   Joystick Yaw: A1
 *   Joystick Pitch: A2
 *   Joystick Roll: A3
 *   Buzzer (optional): D6
 *   Battery Voltage: A6 (optional)
 * 
 * Transmission Rate: 50 Hz
 * 
 * Author: Flight Control Systems
 * Version: 1.0.0
 * 
 * ============================================================================
 */

#include <SPI.h>
#include <RF24.h>

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

namespace Pins {
    // NRF24L01
    constexpr uint8_t RF_CE  = 9;
    constexpr uint8_t RF_CSN = 10;
    
    // Digital Inputs
    constexpr uint8_t SW_ARM      = 2;   // Toggle switch - Arm/Disarm
    constexpr uint8_t SW_AUX      = 3;   // Toggle switch - Auxiliary
    constexpr uint8_t BTN_CALIB   = 4;   // Push button - Calibration
    constexpr uint8_t BTN_MOTOR   = 5;   // Push button - Motor test
    
    // Analog Inputs
    constexpr uint8_t JOY_THROTTLE = A0;
    constexpr uint8_t JOY_YAW      = A1;
    constexpr uint8_t JOY_PITCH    = A2;
    constexpr uint8_t JOY_ROLL     = A3;
    
    // Optional
    constexpr uint8_t BUZZER      = 6;
    constexpr uint8_t BATT_SENSE  = A6;
}

// ============================================================================
// RF CONFIGURATION (Must match Flight Controller)
// ============================================================================

namespace RFConfig {
    constexpr uint8_t CHANNEL = 108;
    constexpr uint8_t PAYLOAD_SIZE = 16;
    const uint8_t ADDRESS[6] = "QUAD1";
    
    // Switch bit definitions
    constexpr uint8_t SW_ARM_BIT       = 0;
    constexpr uint8_t SW_CALIBRATE_BIT = 1;
    constexpr uint8_t SW_MOTORTEST_BIT = 2;
    constexpr uint8_t SW_AUX_BIT       = 3;
}

// Control packet structure (16 bytes) - Must match FC
struct __attribute__((packed)) ControlPacket {
    uint16_t throttle;    // 0-1000
    int16_t  yaw;         // -500 to +500
    int16_t  pitch;       // -500 to +500
    int16_t  roll;        // -500 to +500
    uint8_t  switches;    // Bit 0: Arm, Bit 1: Calibrate, Bit 2: Motor Test, Bit 3: Aux
    uint8_t  checksum;    // XOR of bytes 0-8
    uint32_t sequence;    // Packet sequence number
    uint8_t  rssiRequest; // Request RSSI feedback
    uint8_t  reserved;    // Future use
    
    void calculateChecksum() {
        const uint8_t* data = reinterpret_cast<const uint8_t*>(this);
        uint8_t calc = 0;
        for (uint8_t i = 0; i < 9; i++) {  // Bytes 0-8
            calc ^= data[i];
        }
        checksum = calc;
    }
};

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

namespace Timing {
    constexpr uint32_t TX_PERIOD_MS = 20;      // 50 Hz transmission
    constexpr uint32_t INPUT_PERIOD_MS = 10;   // 100 Hz input sampling
    constexpr uint32_t LED_PERIOD_MS = 500;    // 2 Hz status blink
    constexpr uint32_t DEBOUNCE_MS = 50;       // Button debounce time
}

// ============================================================================
// JOYSTICK CONFIGURATION
// ============================================================================

namespace JoystickConfig {
    // ADC range (10-bit)
    constexpr int16_t ADC_MIN = 0;
    constexpr int16_t ADC_MAX = 1023;
    constexpr int16_t ADC_CENTER = 512;
    
    // Deadband (in ADC units, ~5%)
    constexpr int16_t DEADBAND = 25;
    
    // Output range
    constexpr int16_t OUTPUT_MIN = -500;
    constexpr int16_t OUTPUT_MAX = 500;
    constexpr int16_t THROTTLE_MIN = 0;
    constexpr int16_t THROTTLE_MAX = 1000;
    
    // Expo factor (0.0 = linear, 1.0 = maximum expo)
    // Expo gives more precision around center
    constexpr float EXPO = 0.3f;
    
    // Calibration storage
    struct Calibration {
        int16_t throttleMin, throttleMax;
        int16_t yawCenter;
        int16_t pitchCenter;
        int16_t rollCenter;
        bool valid;
    };
}

// ============================================================================
// JOYSTICK INPUT HANDLER
// ============================================================================

class JoystickInput {
private:
    JoystickConfig::Calibration cal_;
    
    // Raw ADC values
    int16_t rawThrottle_;
    int16_t rawYaw_;
    int16_t rawPitch_;
    int16_t rawRoll_;
    
    // Processed values
    int16_t throttle_;
    int16_t yaw_;
    int16_t pitch_;
    int16_t roll_;
    
    // Apply exponential curve for precision near center
    int16_t applyExpo(int16_t value, float expo) {
        float normalized = value / 500.0f;  // -1 to 1
        float curved = normalized * (1.0f - expo) + 
                      (normalized * normalized * normalized) * expo;
        return (int16_t)(curved * 500.0f);
    }
    
    // Apply deadband and mapping
    int16_t processAxis(int16_t raw, int16_t center, int16_t outMin, int16_t outMax) {
        int16_t deviation = raw - center;
        
        // Apply deadband
        if (abs(deviation) < JoystickConfig::DEADBAND) {
            return (outMin + outMax) / 2;  // Center output
        }
        
        // Remove deadband from calculation
        if (deviation > 0) {
            deviation -= JoystickConfig::DEADBAND;
        } else {
            deviation += JoystickConfig::DEADBAND;
        }
        
        // Calculate effective range
        int16_t maxDeviation = (JoystickConfig::ADC_MAX - center) - JoystickConfig::DEADBAND;
        
        // Map to output range
        int16_t halfRange = (outMax - outMin) / 2;
        int16_t output = ((int32_t)deviation * halfRange) / maxDeviation;
        
        return constrain(output + (outMin + outMax) / 2, outMin, outMax);
    }
    
public:
    JoystickInput() : throttle_(0), yaw_(0), pitch_(0), roll_(0) {
        cal_.valid = false;
        // Default calibration (assuming centered pots at startup)
        cal_.throttleMin = 0;
        cal_.throttleMax = 1023;
        cal_.yawCenter = 512;
        cal_.pitchCenter = 512;
        cal_.rollCenter = 512;
    }
    
    void begin() {
        // Configure analog inputs (default in Arduino)
        // analogReference(DEFAULT);  // 5V reference
    }
    
    void update() {
        // Read raw ADC values (average 4 samples for noise reduction)
        int32_t sum[4] = {0, 0, 0, 0};
        for (uint8_t i = 0; i < 4; i++) {
            sum[0] += analogRead(Pins::JOY_THROTTLE);
            sum[1] += analogRead(Pins::JOY_YAW);
            sum[2] += analogRead(Pins::JOY_PITCH);
            sum[3] += analogRead(Pins::JOY_ROLL);
        }
        rawThrottle_ = sum[0] / 4;
        rawYaw_      = sum[1] / 4;
        rawPitch_    = sum[2] / 4;
        rawRoll_     = sum[3] / 4;
        
        // Process throttle (unidirectional: 0 to 1000)
        throttle_ = map(rawThrottle_, cal_.throttleMin, cal_.throttleMax,
                       JoystickConfig::THROTTLE_MIN, JoystickConfig::THROTTLE_MAX);
        throttle_ = constrain(throttle_, JoystickConfig::THROTTLE_MIN, 
                             JoystickConfig::THROTTLE_MAX);
        
        // Process centered axes (bidirectional: -500 to +500)
        yaw_   = processAxis(rawYaw_, cal_.yawCenter,
                            JoystickConfig::OUTPUT_MIN, JoystickConfig::OUTPUT_MAX);
        pitch_ = processAxis(rawPitch_, cal_.pitchCenter,
                            JoystickConfig::OUTPUT_MIN, JoystickConfig::OUTPUT_MAX);
        roll_  = processAxis(rawRoll_, cal_.rollCenter,
                            JoystickConfig::OUTPUT_MIN, JoystickConfig::OUTPUT_MAX);
        
        // Apply expo curve
        yaw_   = applyExpo(yaw_, JoystickConfig::EXPO);
        pitch_ = applyExpo(pitch_, JoystickConfig::EXPO);
        roll_  = applyExpo(roll_, JoystickConfig::EXPO);
    }
    
    /**
     * Calibrate joystick centers
     * Call with sticks centered
     */
    void calibrateCenters() {
        // Take average of multiple samples
        int32_t sum[3] = {0, 0, 0};
        for (uint8_t i = 0; i < 32; i++) {
            sum[0] += analogRead(Pins::JOY_YAW);
            sum[1] += analogRead(Pins::JOY_PITCH);
            sum[2] += analogRead(Pins::JOY_ROLL);
            delay(10);
        }
        cal_.yawCenter   = sum[0] / 32;
        cal_.pitchCenter = sum[1] / 32;
        cal_.rollCenter  = sum[2] / 32;
        cal_.valid = true;
    }
    
    /**
     * Calibrate throttle range
     * Call with throttle at minimum, then at maximum
     */
    void calibrateThrottleMin() {
        int32_t sum = 0;
        for (uint8_t i = 0; i < 32; i++) {
            sum += analogRead(Pins::JOY_THROTTLE);
            delay(10);
        }
        cal_.throttleMin = sum / 32;
    }
    
    void calibrateThrottleMax() {
        int32_t sum = 0;
        for (uint8_t i = 0; i < 32; i++) {
            sum += analogRead(Pins::JOY_THROTTLE);
            delay(10);
        }
        cal_.throttleMax = sum / 32;
    }
    
    // Getters
    int16_t getThrottle() const { return throttle_; }
    int16_t getYaw()      const { return yaw_; }
    int16_t getPitch()    const { return pitch_; }
    int16_t getRoll()     const { return roll_; }
    
    // Raw value getters (for debugging)
    int16_t getRawThrottle() const { return rawThrottle_; }
    int16_t getRawYaw()      const { return rawYaw_; }
    int16_t getRawPitch()    const { return rawPitch_; }
    int16_t getRawRoll()     const { return rawRoll_; }
    
    const JoystickConfig::Calibration& getCalibration() const { return cal_; }
};

// ============================================================================
// SWITCH/BUTTON INPUT HANDLER
// ============================================================================

class SwitchInput {
private:
    // Debounce state
    struct DebouncedInput {
        bool state;
        bool lastReading;
        uint32_t lastChangeTime;
    };
    
    DebouncedInput armSwitch_;
    DebouncedInput auxSwitch_;
    DebouncedInput calibButton_;
    DebouncedInput motorButton_;
    
    // Edge detection
    bool prevArm_;
    bool prevCalib_;
    bool prevMotor_;
    
    bool updateDebounced(DebouncedInput& input, uint8_t pin, bool activeLow = true) {
        bool reading = digitalRead(pin);
        if (activeLow) reading = !reading;
        
        if (reading != input.lastReading) {
            input.lastChangeTime = millis();
            input.lastReading = reading;
        }
        
        if ((millis() - input.lastChangeTime) >= Timing::DEBOUNCE_MS) {
            if (reading != input.state) {
                input.state = reading;
                return true;  // State changed
            }
        }
        return false;
    }
    
public:
    SwitchInput() {
        armSwitch_ = {false, false, 0};
        auxSwitch_ = {false, false, 0};
        calibButton_ = {false, false, 0};
        motorButton_ = {false, false, 0};
        prevArm_ = false;
        prevCalib_ = false;
        prevMotor_ = false;
    }
    
    void begin() {
        // Configure inputs with pull-ups (switches connect to GND when active)
        pinMode(Pins::SW_ARM, INPUT_PULLUP);
        pinMode(Pins::SW_AUX, INPUT_PULLUP);
        pinMode(Pins::BTN_CALIB, INPUT_PULLUP);
        pinMode(Pins::BTN_MOTOR, INPUT_PULLUP);
    }
    
    void update() {
        prevArm_ = armSwitch_.state;
        prevCalib_ = calibButton_.state;
        prevMotor_ = motorButton_.state;
        
        updateDebounced(armSwitch_, Pins::SW_ARM);
        updateDebounced(auxSwitch_, Pins::SW_AUX);
        updateDebounced(calibButton_, Pins::BTN_CALIB);
        updateDebounced(motorButton_, Pins::BTN_MOTOR);
    }
    
    // Current states
    bool isArmed()       const { return armSwitch_.state; }
    bool isAuxOn()       const { return auxSwitch_.state; }
    bool isCalibPressed() const { return calibButton_.state; }
    bool isMotorPressed() const { return motorButton_.state; }
    
    // Edge detection (rising edge = button just pressed)
    bool armJustChanged()   const { return armSwitch_.state != prevArm_; }
    bool calibJustPressed() const { return calibButton_.state && !prevCalib_; }
    bool motorJustPressed() const { return motorButton_.state && !prevMotor_; }
    
    // Build switch byte for packet
    uint8_t getSwitchByte() const {
        uint8_t sw = 0;
        if (armSwitch_.state)    sw |= (1 << RFConfig::SW_ARM_BIT);
        if (calibButton_.state)  sw |= (1 << RFConfig::SW_CALIBRATE_BIT);
        if (motorButton_.state)  sw |= (1 << RFConfig::SW_MOTORTEST_BIT);
        if (auxSwitch_.state)    sw |= (1 << RFConfig::SW_AUX_BIT);
        return sw;
    }
};

// ============================================================================
// RADIO TRANSMITTER
// ============================================================================

class RadioTransmitter {
private:
    RF24 radio_;
    uint32_t packetSequence_;
    uint32_t packetsSent_;
    uint32_t lastTxTime_;
    bool initialized_;
    
public:
    RadioTransmitter() : radio_(Pins::RF_CE, Pins::RF_CSN),
                         packetSequence_(0), packetsSent_(0),
                         lastTxTime_(0), initialized_(false) {}
    
    bool begin() {
        if (!radio_.begin()) {
            return false;
        }
        
        // Configure radio (must match receiver)
        radio_.setChannel(RFConfig::CHANNEL);
        radio_.setDataRate(RF24_2MBPS);
        radio_.setPALevel(RF24_PA_MAX);
        radio_.setPayloadSize(RFConfig::PAYLOAD_SIZE);
        radio_.setAutoAck(false);          // NO_ACK mode
        radio_.setRetries(0, 0);           // No retries
        radio_.setCRCLength(RF24_CRC_16);
        
        // Open writing pipe
        radio_.openWritingPipe(RFConfig::ADDRESS);
        radio_.stopListening();
        
        initialized_ = true;
        return true;
    }
    
    bool send(ControlPacket& packet) {
        if (!initialized_) return false;
        
        // Set sequence number
        packet.sequence = packetSequence_++;
        
        // Calculate checksum
        packet.calculateChecksum();
        
        // Transmit (non-blocking in NO_ACK mode)
        bool success = radio_.write(&packet, sizeof(packet));
        
        if (success) {
            packetsSent_++;
            lastTxTime_ = millis();
        }
        
        return success;
    }
    
    uint32_t getPacketsSent() const { return packetsSent_; }
    uint32_t getLastTxTime()  const { return lastTxTime_; }
    bool isInitialized()      const { return initialized_; }
};

// ============================================================================
// USER INTERFACE
// ============================================================================

class UserInterface {
private:
    bool buzzerEnabled_;
    
public:
    UserInterface() : buzzerEnabled_(true) {}
    
    void begin() {
        pinMode(Pins::BUZZER, OUTPUT);
        digitalWrite(Pins::BUZZER, LOW);
    }
    
    void beep(uint16_t durationMs, uint16_t frequency = 2000) {
        if (buzzerEnabled_) {
            tone(Pins::BUZZER, frequency, durationMs);
        }
    }
    
    void beepPattern(uint8_t count, uint16_t onMs = 100, uint16_t offMs = 100) {
        for (uint8_t i = 0; i < count; i++) {
            beep(onMs);
            delay(onMs + offMs);
        }
    }
    
    // Sound patterns
    void soundStartup() {
        beep(100, 2000);
        delay(100);
        beep(100, 2500);
        delay(100);
        beep(200, 3000);
    }
    
    void soundButtonPress() {
        beep(30, 2500);
    }
    
    void soundArmChange(bool armed) {
        if (armed) {
            beep(100, 2000);
            delay(50);
            beep(200, 2500);
        } else {
            beep(200, 1500);
        }
    }
    
    void soundError() {
        beepPattern(3, 200, 100);
    }
    
    void soundTxOk() {
        beep(20, 3000);
    }
    
    void setBuzzerEnabled(bool enabled) {
        buzzerEnabled_ = enabled;
    }
};

// ============================================================================
// REMOTE CONTROLLER (MAIN CLASS)
// ============================================================================

class RemoteController {
private:
    JoystickInput joystick_;
    SwitchInput switches_;
    RadioTransmitter radio_;
    UserInterface ui_;
    
    // Timing
    uint32_t lastInputTime_;
    uint32_t lastTxTime_;
    
    // State
    bool rfOk_;
    bool prevArmed_;
    
    // Current packet
    ControlPacket packet_;
    
public:
    RemoteController() : rfOk_(false), prevArmed_(false) {
        lastInputTime_ = 0;
        lastTxTime_ = 0;
        memset(&packet_, 0, sizeof(packet_));
    }
    
    void begin() {
        // Initialize UI first
        ui_.begin();
        
        // Initialize inputs
        joystick_.begin();
        switches_.begin();
        
        // Initialize radio
        rfOk_ = radio_.begin();
        
        if (rfOk_) {
            ui_.soundStartup();
        } else {
            ui_.soundError();
        }
        
        // Initial calibration of joystick centers
        // Assumes sticks are centered at startup
        joystick_.calibrateCenters();
        
        // Initialize timing
        lastInputTime_ = millis();
        lastTxTime_ = millis();
    }
    
    void update() {
        uint32_t now = millis();
        
        // Update inputs (100 Hz)
        if (now - lastInputTime_ >= Timing::INPUT_PERIOD_MS) {
            lastInputTime_ = now;
            
            joystick_.update();
            switches_.update();
            
            // Handle button presses
            if (switches_.calibJustPressed()) {
                ui_.soundButtonPress();
            }
            if (switches_.motorJustPressed()) {
                ui_.soundButtonPress();
            }
            
            // Handle arm switch changes
            bool armed = switches_.isArmed();
            if (armed != prevArmed_) {
                ui_.soundArmChange(armed);
                prevArmed_ = armed;
            }
        }
        
        // Transmit packet (50 Hz)
        if (now - lastTxTime_ >= Timing::TX_PERIOD_MS) {
            lastTxTime_ = now;
            
            // Build packet
            packet_.throttle = joystick_.getThrottle();
            packet_.yaw      = joystick_.getYaw();
            packet_.pitch    = joystick_.getPitch();
            packet_.roll     = joystick_.getRoll();
            packet_.switches = switches_.getSwitchByte();
            packet_.rssiRequest = 0;
            packet_.reserved = 0;
            
            // Send packet
            if (rfOk_) {
                radio_.send(packet_);
            }
        }
    }
    
    // Debug accessors
    const JoystickInput& getJoystick() const { return joystick_; }
    const SwitchInput& getSwitches() const { return switches_; }
    const RadioTransmitter& getRadio() const { return radio_; }
    bool isRfOk() const { return rfOk_; }
};

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

RemoteController remote;

// ============================================================================
// ARDUINO ENTRY POINTS
// ============================================================================

void setup() {
    // Optional: Serial for debugging
    // Serial.begin(115200);
    // Serial.println(F("Quad Remote v1.0"));
    
    remote.begin();
}

void loop() {
    remote.update();
    
    // Optional: Debug output
    // static uint32_t lastDebug = 0;
    // if (millis() - lastDebug > 200) {
    //     lastDebug = millis();
    //     Serial.print(F("T:"));
    //     Serial.print(remote.getJoystick().getThrottle());
    //     Serial.print(F(" Y:"));
    //     Serial.print(remote.getJoystick().getYaw());
    //     Serial.print(F(" P:"));
    //     Serial.print(remote.getJoystick().getPitch());
    //     Serial.print(F(" R:"));
    //     Serial.print(remote.getJoystick().getRoll());
    //     Serial.print(F(" SW:0x"));
    //     Serial.println(remote.getSwitches().getSwitchByte(), HEX);
    // }
}

/**
 * ============================================================================
 * USAGE NOTES
 * ============================================================================
 * 
 * 1. JOYSTICK WIRING
 *    - Connect VCC to 5V
 *    - Connect GND to GND
 *    - Connect each axis output to the respective analog pin
 *    - Ensure throttle returns to minimum when released (spring return)
 *      OR implement software low-throttle lock
 * 
 * 2. SWITCH WIRING
 *    - Toggle switches: Connect between pin and GND
 *    - Push buttons: Connect between pin and GND
 *    - Internal pull-ups are used (active LOW)
 * 
 * 3. ARM SWITCH SAFETY
 *    - Use a toggle switch with a guard cover for arm/disarm
 *    - Consider adding a physical key switch for additional safety
 * 
 * 4. CALIBRATION
 *    - On startup, ensure all sticks are centered
 *    - Throttle range is assumed 0-1023 by default
 *    - For full throttle calibration, modify code to include
 *      a calibration mode triggered by button combination
 * 
 * 5. BATTERY MONITORING (Optional)
 *    - Add voltage divider to A6 (2:1 for 2S LiPo)
 *    - Monitor and warn when battery is low
 * 
 * ============================================================================
 */
