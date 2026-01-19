/**
 * ============================================================================
 *          QUADCOPTER REMOTE CONTROLLER - THROTTLE FIX
 * ============================================================================
 * 
 * THROTTLE FIX: Added proper min/max detection and raw value display
 * 
 * RC STICK DIRECTION CONVENTION (Mode 2 - most common):
 * 
 * LEFT STICK:                    RIGHT STICK:
 *   Throttle (up/down)           Pitch (up/down)
 *   Yaw (left/right)             Roll (left/right)
 * 
 * EXPECTED OUTPUT WHEN STICKS MOVE:
 *   - Throttle UP    → throttle value INCREASES (0→1000)
 *   - Yaw RIGHT      → yaw value POSITIVE (+500)
 *   - Pitch UP       → pitch value NEGATIVE (-500) [nose down]
 *   - Roll RIGHT     → roll value POSITIVE (+500) [right side down]
 * 
 * HARDWARE:
 *   - Arduino Nano
 *   - NRF24L01 (CE=D9, CSN=D10)
 *   - 2x Joysticks (A0-A3)
 *   - 2x POTs (A6-A7)
 *   - Switches (D2-D7)
 * 
 * FOR RS2205/2300KV MOTORS:
 *   - These are high-power racing motors
 *   - Ensure ESCs support high refresh rate
 *   - May need lower PID gains due to faster response
 * 
 * ============================================================================
 */

#include <SPI.h>
#include <RF24.h>

// ============================================================================
//                              CONFIGURATION
// ============================================================================
#define RF_CHANNEL          108
#define SERIAL_BAUD         115200
#define DEBUG_THROTTLE      1       // Show raw throttle values for debugging

// ============================================================================
//                            PIN DEFINITIONS
// ============================================================================
#define PIN_RF_CE           9
#define PIN_RF_CSN          10

#define PIN_SW_ARM          2
#define PIN_SW_ALTHOLD      3
#define PIN_BTN_CALIB       4
#define PIN_BTN_MOTOR       5
#define PIN_BTN_FMODE       6
#define PIN_BTN_RATES       7

#define PIN_LED             13

#define PIN_JOY_THROTTLE    A0
#define PIN_JOY_YAW         A1
#define PIN_JOY_PITCH       A2
#define PIN_JOY_ROLL        A3
#define PIN_POT_GAIN        A6
#define PIN_POT_ANGLE       A7

// ============================================================================
//                     JOYSTICK DIRECTION CONFIGURATION
// ============================================================================
// Set these to +1 or -1 to correct stick directions
// If stick moves opposite to expected, flip the sign

#define THROTTLE_INVERT     false   // Set true if throttle is backwards
#define YAW_DIRECTION       +1      // +1 = right stick = positive yaw
#define PITCH_DIRECTION     -1      // -1 = up stick = negative (nose down)
#define ROLL_DIRECTION      +1      // +1 = right stick = positive (right down)

// ============================================================================
//                         JOYSTICK CONFIGURATION
// ============================================================================
#define DEADBAND            30
#define EXPO_FACTOR         0.30f   // Exponential curve

#define ADC_SAMPLES         4       // Reduced for faster response
#define JOY_LPF_ALPHA       0.5f    // Faster throttle response
#define POT_LPF_ALPHA       0.85f

// Throttle specific settings
#define THROTTLE_MIN_DEFAULT    0       // Default if calibration fails
#define THROTTLE_MAX_DEFAULT    1023
#define THROTTLE_MARGIN         50      // Margin for min/max detection

// ============================================================================
//                         TIMING CONFIGURATION
// ============================================================================
#define TX_RATE_MS          20      // 50Hz transmission
#define INPUT_RATE_MS       5       // 200Hz input reading
#define DEBUG_RATE_MS       500     // Debug output rate
#define LED_RATE_MS         500

// ============================================================================
//                          RF PACKET STRUCTURE
// ============================================================================
struct __attribute__((packed)) ControlPacket {
    uint16_t throttle;
    int16_t  yaw, pitch, roll;
    uint8_t  switches, checksum;
    uint32_t sequence;
    uint8_t  channel, auxData;
    
    void calcChecksum() {
        uint8_t* data = (uint8_t*)this;
        uint8_t calc = 0;
        for (uint8_t i = 0; i < 9; i++) calc ^= data[i];
        checksum = calc;
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

#define RATE_SLOW       0
#define RATE_NORMAL     1
#define RATE_FAST       2

// ============================================================================
//                           GLOBAL OBJECTS
// ============================================================================
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
const uint8_t radioAddress[6] = "QUAD1";

// ============================================================================
//                          GLOBAL VARIABLES
// ============================================================================
ControlPacket txPacket;
uint32_t packetSequence = 0;

// Calibration values
struct {
    int16_t thrMin = THROTTLE_MIN_DEFAULT;
    int16_t thrMax = THROTTLE_MAX_DEFAULT;
    int16_t yawCenter = 512;
    int16_t pitchCenter = 512;
    int16_t rollCenter = 512;
    bool thrInverted = THROTTLE_INVERT;
    bool valid = false;
} joyCal;

// Raw ADC values
int16_t rawThrottle = 0, rawYaw = 0, rawPitch = 0, rawRoll = 0;
int16_t rawPotGain = 0, rawPotAngle = 0;

// Filtered values
float throttle = 0, yaw = 0, pitch = 0, roll = 0;
float potGain = 512, potAngle = 512;

// Switch states
bool swArm = false, swAltHold = false;
bool btnCalib = false, btnMotor = false;
bool btnFMode = false, btnRates = false;
bool prevArm = false, prevFMode = false, prevRates = false;

// Flight mode and rate
uint8_t flightMode = FMODE_ANGLE;
uint8_t rateMode = RATE_NORMAL;

// Radio stats
bool radioOK = false;
bool connected = false;
uint32_t packetsSent = 0;
uint32_t packetsFailed = 0;

// Timing
uint32_t timeTX = 0, timeInput = 0, timeDebug = 0, timeLED = 0;
bool ledState = false;

// ============================================================================
//                         UTILITY FUNCTIONS
// ============================================================================
float lowPassFilter(float current, float target, float alpha) {
    return alpha * current + (1.0f - alpha) * target;
}

int16_t applyDeadband(int16_t value, int16_t center, int16_t band) {
    int16_t deviation = value - center;
    if (abs(deviation) < band) return 0;
    return (deviation > 0) ? (deviation - band) : (deviation + band);
}

float applyExpo(float value, float expo) {
    float normalized = value / 500.0f;
    float curved = normalized * (1.0f - expo) + 
                   (normalized * normalized * normalized) * expo;
    return curved * 500.0f;
}

// ============================================================================
//                      MULTI-SAMPLE ADC READ
// ============================================================================
int16_t readADC_Averaged(uint8_t pin) {
    int32_t sum = 0;
    for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
        sum += analogRead(pin);
    }
    return sum / ADC_SAMPLES;
}

// ============================================================================
//                         READ JOYSTICKS
// ============================================================================
void readJoysticks() {
    // Read raw values
    rawThrottle = readADC_Averaged(PIN_JOY_THROTTLE);
    rawYaw = readADC_Averaged(PIN_JOY_YAW);
    rawPitch = readADC_Averaged(PIN_JOY_PITCH);
    rawRoll = readADC_Averaged(PIN_JOY_ROLL);
    
    // Process throttle (0-1000 range)
    // Handle both normal and inverted throttle joysticks
    int16_t thrScaled;
    if (joyCal.thrInverted) {
        // Inverted: high ADC = low throttle
        thrScaled = map(rawThrottle, joyCal.thrMax, joyCal.thrMin, 0, 1000);
    } else {
        // Normal: low ADC = low throttle
        thrScaled = map(rawThrottle, joyCal.thrMin, joyCal.thrMax, 0, 1000);
    }
    thrScaled = constrain(thrScaled, 0, 1000);
    
    // Apply deadband to centered sticks (not throttle!)
    int16_t yawDev = applyDeadband(rawYaw, joyCal.yawCenter, DEADBAND);
    int16_t pitchDev = applyDeadband(rawPitch, joyCal.pitchCenter, DEADBAND);
    int16_t rollDev = applyDeadband(rawRoll, joyCal.rollCenter, DEADBAND);
    
    // Scale to -500 to +500
    int16_t maxDev = 512 - DEADBAND;
    int16_t yawScaled = ((int32_t)yawDev * 500 * YAW_DIRECTION) / maxDev;
    int16_t pitchScaled = ((int32_t)pitchDev * 500 * PITCH_DIRECTION) / maxDev;
    int16_t rollScaled = ((int32_t)rollDev * 500 * ROLL_DIRECTION) / maxDev;
    
    // Apply exponential curve
    float yawExpo = applyExpo(yawScaled, EXPO_FACTOR);
    float pitchExpo = applyExpo(pitchScaled, EXPO_FACTOR);
    float rollExpo = applyExpo(rollScaled, EXPO_FACTOR);
    
    // Apply low-pass filter (faster for throttle)
    throttle = lowPassFilter(throttle, thrScaled, 0.3f);  // Fast response for throttle
    yaw = lowPassFilter(yaw, yawExpo, JOY_LPF_ALPHA);
    pitch = lowPassFilter(pitch, pitchExpo, JOY_LPF_ALPHA);
    roll = lowPassFilter(roll, rollExpo, JOY_LPF_ALPHA);
}

// ============================================================================
//                         READ POTENTIOMETERS
// ============================================================================
void readPots() {
    rawPotGain = readADC_Averaged(PIN_POT_GAIN);
    rawPotAngle = readADC_Averaged(PIN_POT_ANGLE);
    
    potGain = lowPassFilter(potGain, rawPotGain, POT_LPF_ALPHA);
    potAngle = lowPassFilter(potAngle, rawPotAngle, POT_LPF_ALPHA);
}

// ============================================================================
//                         READ SWITCHES
// ============================================================================
void readSwitches() {
    bool rawArm = !digitalRead(PIN_SW_ARM);
    bool rawAltHold = !digitalRead(PIN_SW_ALTHOLD);
    bool rawCalib = !digitalRead(PIN_BTN_CALIB);
    bool rawMotor = !digitalRead(PIN_BTN_MOTOR);
    bool rawFMode = !digitalRead(PIN_BTN_FMODE);
    bool rawRates = !digitalRead(PIN_BTN_RATES);
    
    // Debounce arm switch
    static uint32_t armDebounce = 0;
    if (rawArm != swArm && millis() - armDebounce > 50) {
        armDebounce = millis();
        swArm = rawArm;
        prevArm = swArm;
    }
    
    swAltHold = rawAltHold;
    btnCalib = rawCalib;
    btnMotor = rawMotor;
    
    // Flight mode toggle
    if (rawFMode && !prevFMode) {
        flightMode = (flightMode + 1) % 3;
        Serial.print(F("Mode: "));
        Serial.println(flightMode == FMODE_ANGLE ? F("ANGLE") :
                      flightMode == FMODE_HORIZON ? F("HORIZON") : F("ACRO"));
    }
    prevFMode = rawFMode;
    
    // Rate toggle
    if (rawRates && !prevRates) {
        rateMode = (rateMode + 1) % 3;
        Serial.print(F("Rate: "));
        Serial.println(rateMode == RATE_SLOW ? F("SLOW") :
                      rateMode == RATE_NORMAL ? F("NORMAL") : F("FAST"));
    }
    prevRates = rawRates;
}

// ============================================================================
//                      CALIBRATE JOYSTICKS
// ============================================================================
void calibrateJoysticks() {
    Serial.println(F("\n========================================"));
    Serial.println(F("       JOYSTICK CALIBRATION"));
    Serial.println(F("========================================"));
    
    // Step 1: Read current throttle position (should be at minimum)
    Serial.println(F("\nStep 1: Keep THROTTLE at MINIMUM (stick DOWN)"));
    Serial.println(F("        Keep other sticks CENTERED"));
    Serial.println(F("Starting in 3 seconds..."));
    
    for (int i = 3; i > 0; i--) {
        Serial.print(i); Serial.print(F("... "));
        delay(1000);
    }
    Serial.println();
    
    // Sample throttle minimum and other centers
    int32_t sumThrMin = 0, sumYaw = 0, sumPitch = 0, sumRoll = 0;
    Serial.print(F("Reading"));
    for (int i = 0; i < 100; i++) {
        sumThrMin += analogRead(PIN_JOY_THROTTLE);
        sumYaw += analogRead(PIN_JOY_YAW);
        sumPitch += analogRead(PIN_JOY_PITCH);
        sumRoll += analogRead(PIN_JOY_ROLL);
        if (i % 20 == 0) Serial.print(F("."));
        delay(10);
    }
    Serial.println(F(" done"));
    
    int16_t thrMinReading = sumThrMin / 100;
    joyCal.yawCenter = sumYaw / 100;
    joyCal.pitchCenter = sumPitch / 100;
    joyCal.rollCenter = sumRoll / 100;
    
    Serial.print(F("Throttle MIN reading: ")); Serial.println(thrMinReading);
    Serial.print(F("Yaw center: ")); Serial.println(joyCal.yawCenter);
    Serial.print(F("Pitch center: ")); Serial.println(joyCal.pitchCenter);
    Serial.print(F("Roll center: ")); Serial.println(joyCal.rollCenter);
    
    // Step 2: Read throttle maximum
    Serial.println(F("\nStep 2: Move THROTTLE to MAXIMUM (stick UP)"));
    Serial.println(F("Hold it there..."));
    delay(2000);
    
    int32_t sumThrMax = 0;
    Serial.print(F("Reading"));
    for (int i = 0; i < 100; i++) {
        sumThrMax += analogRead(PIN_JOY_THROTTLE);
        if (i % 20 == 0) Serial.print(F("."));
        delay(10);
    }
    Serial.println(F(" done"));
    
    int16_t thrMaxReading = sumThrMax / 100;
    Serial.print(F("Throttle MAX reading: ")); Serial.println(thrMaxReading);
    
    // Determine if throttle is inverted
    // Normal: MIN position gives LOW ADC value
    // Inverted: MIN position gives HIGH ADC value
    if (thrMinReading > thrMaxReading) {
        // Throttle is INVERTED (high ADC at min position)
        joyCal.thrMin = thrMaxReading;  // Lower ADC value
        joyCal.thrMax = thrMinReading;  // Higher ADC value
        joyCal.thrInverted = true;
        Serial.println(F("Throttle detected as INVERTED"));
    } else {
        // Throttle is NORMAL (low ADC at min position)
        joyCal.thrMin = thrMinReading;
        joyCal.thrMax = thrMaxReading;
        joyCal.thrInverted = false;
        Serial.println(F("Throttle detected as NORMAL"));
    }
    
    // Validate range
    int16_t thrRange = joyCal.thrMax - joyCal.thrMin;
    if (thrRange < 200) {
        Serial.println(F("\n*** WARNING: Throttle range too small! ***"));
        Serial.println(F("Check wiring or use default values."));
        joyCal.thrMin = 0;
        joyCal.thrMax = 1023;
        joyCal.thrInverted = THROTTLE_INVERT;
    }
    
    Serial.println(F("\n--- CALIBRATION SUMMARY ---"));
    Serial.print(F("Throttle: ")); Serial.print(joyCal.thrMin);
    Serial.print(F(" - ")); Serial.print(joyCal.thrMax);
    Serial.print(F(" (inverted: ")); Serial.print(joyCal.thrInverted ? "YES" : "NO");
    Serial.println(F(")"));
    Serial.print(F("Range: ")); Serial.println(thrRange);
    
    joyCal.valid = true;
    Serial.println(F("\n*** CALIBRATION COMPLETE ***"));
    Serial.println(F("Move throttle up/down to verify it works.\n"));
}

// ============================================================================
//                         INIT RADIO
// ============================================================================
bool initRadio() {
    if (!radio.begin()) return false;
    
    radio.flush_tx();
    radio.flush_rx();
    
    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF24_1MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(16);
    radio.setAutoAck(false);
    radio.disableDynamicPayloads();
    radio.setCRCLength(RF24_CRC_16);
    radio.setRetries(0, 0);
    radio.openWritingPipe(radioAddress);
    radio.stopListening();
    
    if (!radio.isPVariant()) return false;
    
    return true;
}

// ============================================================================
//                         SEND PACKET
// ============================================================================
bool sendPacket() {
    txPacket.throttle = (uint16_t)constrain(throttle, 0, 1000);
    txPacket.yaw = (int16_t)constrain(yaw, -500, 500);
    txPacket.pitch = (int16_t)constrain(pitch, -500, 500);
    txPacket.roll = (int16_t)constrain(roll, -500, 500);
    
    txPacket.switches = 0;
    if (swArm)     txPacket.switches |= (1 << SW_ARM);
    if (btnCalib)  txPacket.switches |= (1 << SW_CALIBRATE);
    if (btnMotor)  txPacket.switches |= (1 << SW_MOTORTEST);
    if (swAltHold) txPacket.switches |= (1 << SW_ALTHOLD);
    
    if (flightMode == FMODE_HORIZON) txPacket.switches |= (1 << SW_FLIGHTMODE);
    if (flightMode == FMODE_ACRO)    txPacket.switches |= (1 << SW_FLIGHTMODE) | (1 << SW_RATES);
    
    if (rateMode == RATE_FAST && flightMode != FMODE_ACRO) 
        txPacket.switches |= (1 << SW_RATES);
    
    uint8_t gainValue = map(potGain, 0, 1023, 0, 15);
    uint8_t angleValue = map(potAngle, 0, 1023, 0, 15);
    txPacket.auxData = (gainValue << 4) | angleValue;
    
    txPacket.sequence = packetSequence++;
    txPacket.channel = RF_CHANNEL;
    txPacket.calcChecksum();
    
    bool success = radio.write(&txPacket, sizeof(txPacket));
    
    if (success) {
        packetsSent++;
        if (!connected && packetsSent > 20) {
            connected = true;
            Serial.println(F("\n*** TRANSMITTING ***"));
        }
    } else {
        packetsFailed++;
        radio.flush_tx();
    }
    
    return success;
}

// ============================================================================
//                         LED UPDATE
// ============================================================================
void updateLED() {
    uint32_t now = millis();
    
    if (connected) {
        if (swArm) {
            digitalWrite(PIN_LED, HIGH);
        } else {
            if (now - timeLED >= 500) {
                timeLED = now;
                ledState = !ledState;
                digitalWrite(PIN_LED, ledState);
            }
        }
    } else {
        if (now - timeLED >= 100) {
            timeLED = now;
            ledState = !ledState;
            digitalWrite(PIN_LED, ledState);
        }
    }
}

// ============================================================================
//                         DEBUG OUTPUT
// ============================================================================
void printDebug() {
    // Packet stats
    Serial.print(F("TX:")); Serial.print(packetsSent);
    
    if (packetsFailed > 0) {
        Serial.print(F(" FAIL:")); Serial.print(packetsFailed);
    }
    
#if DEBUG_THROTTLE
    // Show RAW throttle value for debugging
    Serial.print(F(" | RAW:")); Serial.print(rawThrottle);
#endif
    
    // Stick values (processed)
    Serial.print(F(" | T:")); Serial.print((int)throttle);
    Serial.print(F(" Y:")); Serial.print((int)yaw);
    Serial.print(F(" P:")); Serial.print((int)pitch);
    Serial.print(F(" R:")); Serial.print((int)roll);
    
    // Throttle bar visualization
    Serial.print(F(" ["));
    int bars = (int)throttle / 100;  // 0-10 bars
    for (int i = 0; i < 10; i++) {
        Serial.print(i < bars ? F("#") : F("-"));
    }
    Serial.print(F("]"));
    
    // Direction indicators for other sticks
    if (abs(yaw) > 50) Serial.print(yaw > 0 ? F(" Y>") : F(" <Y"));
    if (abs(pitch) > 50) Serial.print(pitch < 0 ? F(" Pv") : F(" P^"));
    if (abs(roll) > 50) Serial.print(roll > 0 ? F(" R>") : F(" <R"));
    
    // Arm status
    if (swArm) Serial.print(F(" [ARM]"));
    
    Serial.println();
}

// ============================================================================
//                     DIRECTION TEST MODE
// ============================================================================
void runDirectionTest() {
    Serial.println(F("\n=== STICK DIRECTION TEST ==="));
    Serial.println(F("Move each stick and verify output:"));
    Serial.println(F(""));
    Serial.println(F("Expected behavior:"));
    Serial.println(F("  Throttle UP    → T increases (0→1000)"));
    Serial.println(F("  Yaw RIGHT      → Y positive (+500)"));
    Serial.println(F("  Pitch UP       → P negative (-500) [nose down command]"));
    Serial.println(F("  Roll RIGHT     → R positive (+500) [right bank command]"));
    Serial.println(F(""));
    Serial.println(F("If any direction is wrong, flip the corresponding"));
    Serial.println(F("DIRECTION define at the top of the code."));
    Serial.println(F("================================\n"));
}

// ============================================================================
//                              SETUP
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("\n*** QUADCOPTER REMOTE - DIRECTION VERIFIED ***"));
    
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_SW_ARM, INPUT_PULLUP);
    pinMode(PIN_SW_ALTHOLD, INPUT_PULLUP);
    pinMode(PIN_BTN_CALIB, INPUT_PULLUP);
    pinMode(PIN_BTN_MOTOR, INPUT_PULLUP);
    pinMode(PIN_BTN_FMODE, INPUT_PULLUP);
    pinMode(PIN_BTN_RATES, INPUT_PULLUP);
    
    digitalWrite(PIN_LED, HIGH);
    
    // Initialize radio
    Serial.println(F("Init NRF24L01..."));
    if (!initRadio()) {
        Serial.println(F("*** NRF24 FAIL! ***"));
        while (1) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            delay(200);
        }
    }
    Serial.println(F("NRF24 OK!"));
    
    // Show direction configuration
    Serial.println(F("\nDirection configuration:"));
    Serial.print(F("  THROTTLE_INVERT: ")); Serial.println(THROTTLE_INVERT ? "YES" : "NO");
    Serial.print(F("  YAW_DIRECTION: ")); Serial.println(YAW_DIRECTION);
    Serial.print(F("  PITCH_DIRECTION: ")); Serial.println(PITCH_DIRECTION);
    Serial.print(F("  ROLL_DIRECTION: ")); Serial.println(ROLL_DIRECTION);
    
    // Calibrate
    calibrateJoysticks();
    
    // Run direction test
    runDirectionTest();
    
    Serial.println(F("*** SYSTEM READY ***\n"));
    
    timeTX = timeInput = timeDebug = timeLED = millis();
}

// ============================================================================
//                             MAIN LOOP
// ============================================================================
void loop() {
    uint32_t now = millis();
    
    // Read inputs at 200Hz
    if (now - timeInput >= INPUT_RATE_MS) {
        timeInput = now;
        readJoysticks();
        readPots();
        readSwitches();
    }
    
    // Send packet at 50Hz
    if (now - timeTX >= TX_RATE_MS) {
        timeTX = now;
        sendPacket();
    }
    
    // Update LED
    updateLED();
    
    // Debug at 2Hz
    if (now - timeDebug >= DEBUG_RATE_MS) {
        timeDebug = now;
        printDebug();
    }
}
