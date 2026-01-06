/**
 * ============================================================================
 *                    QUADCOPTER REMOTE CONTROLLER v3.0
 * ============================================================================
 * 
 * Features:
 *   ✓ NRF24L01 with ACK mode (reliable communication)
 *   ✓ 4-axis joystick input with deadband
 *   ✓ Arm/Disarm safety switch
 *   ✓ Altitude hold toggle
 *   ✓ Calibration and motor test buttons
 *   ✓ Audio and visual feedback
 *   ✓ Connection status monitoring
 * 
 * Hardware:
 *   MCU: Arduino Nano (ATmega328P)
 *   Radio: NRF24L01+ (SPI)
 *   Joysticks: 2x dual-axis potentiometer
 *   Switches: 2x toggle, 2x push button
 * 
 * Libraries Required:
 *   - RF24 (TMRh20): github.com/nRF24/RF24
 * 
 * Pin Configuration:
 *   D2  - ARM Toggle Switch (to GND)
 *   D3  - Altitude Hold Toggle Switch (to GND)
 *   D4  - Calibration Push Button (to GND)
 *   D5  - Motor Test Push Button (to GND)
 *   D6  - Buzzer
 *   D7  - Status LED
 *   D9  - NRF24 CE
 *   D10 - NRF24 CSN
 *   D11 - NRF24 MOSI
 *   D12 - NRF24 MISO
 *   D13 - NRF24 SCK
 *   A0  - Throttle Joystick
 *   A1  - Yaw Joystick
 *   A2  - Pitch Joystick
 *   A3  - Roll Joystick
 * 
 * ============================================================================
 */

#include <SPI.h>
#include <RF24.h>

// ============================================================================
//                              CONFIGURATION
// ============================================================================

// RF Channel (0-125) - MUST MATCH FLIGHT CONTROLLER!
#define RF_CHANNEL          108

// Serial debug (always on for troubleshooting)
#define SERIAL_BAUD         115200

// ============================================================================
//                            PIN DEFINITIONS
// ============================================================================

#define PIN_RF_CE           9
#define PIN_RF_CSN          10
#define PIN_SW_ARM          2       // Toggle switch - Arm/Disarm
#define PIN_SW_ALTHOLD      3       // Toggle switch - Altitude Hold
#define PIN_BTN_CALIB       4       // Push button - Calibration
#define PIN_BTN_MOTOR       5       // Push button - Motor Test
#define PIN_BUZZER          6
#define PIN_LED             7
#define PIN_JOY_THROTTLE    A0
#define PIN_JOY_YAW         A1
#define PIN_JOY_PITCH       A2
#define PIN_JOY_ROLL        A3

// ============================================================================
//                         JOYSTICK CONFIGURATION
// ============================================================================

#define DEADBAND            30      // ADC deadband (0-1023 scale)
#define EXPO_FACTOR         0.3f   // Expo curve (0=linear, 1=max curve)

// ============================================================================
//                         TIMING CONFIGURATION
// ============================================================================

#define TX_RATE_MS          20      // 50Hz transmission rate
#define INPUT_RATE_MS       10      // 100Hz input sampling
#define DEBUG_RATE_MS       250     // 4Hz debug output
#define LED_RATE_MS         500     // LED blink rate
#define DEBOUNCE_MS         50      // Button debounce

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
    uint8_t  channel;       // RF channel
    uint8_t  reserved;
    
    void calcChecksum() {
        uint8_t* data = (uint8_t*)this;
        uint8_t calc = 0;
        for (uint8_t i = 0; i < 9; i++) calc ^= data[i];
        checksum = calc;
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

RF24 radio(PIN_RF_CE, PIN_RF_CSN);
const uint8_t radioAddress[6] = "QUAD1";

// ============================================================================
//                          GLOBAL VARIABLES
// ============================================================================

// Packet
ControlPacket txPacket;
uint32_t packetSequence = 0;

// Joystick calibration
struct {
    int16_t throttleMin = 0;
    int16_t throttleMax = 1023;
    int16_t yawCenter = 512;
    int16_t pitchCenter = 512;
    int16_t rollCenter = 512;
} joyCal;

// Raw joystick values
int16_t rawThrottle = 0, rawYaw = 0, rawPitch = 0, rawRoll = 0;

// Processed values
int16_t throttle = 0, yaw = 0, pitch = 0, roll = 0;

// Switches
bool swArm = false, swAltHold = false;
bool btnCalib = false, btnMotor = false;
bool prevArm = false, prevCalib = false, prevMotor = false;

// Communication
bool radioOK = false;
bool connected = false;
uint32_t packetsSent = 0;
uint32_t packetsAcked = 0;
uint32_t packetsFailed = 0;
uint32_t lastAckTime = 0;

// Timing
uint32_t timeTX = 0, timeInput = 0, timeDebug = 0, timeLED = 0;
bool ledState = false;

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
    beepBlocking(100, 2500); delay(50);
    beepBlocking(200, 3000);
}

void soundButtonPress() {
    beep(30, 2500);
}

void soundArmOn() {
    beepBlocking(100, 2000); delay(50);
    beepBlocking(200, 2500);
}

void soundArmOff() {
    beepBlocking(200, 1500);
}

void soundConnected() {
    for (int i = 0; i < 3; i++) {
        beepBlocking(50, 2500);
        delay(50);
    }
}

void soundDisconnected() {
    beepBlocking(300, 800);
}

// ============================================================================
//                         JOYSTICK FUNCTIONS
// ============================================================================

int16_t applyDeadbandAndScale(int16_t raw, int16_t center, int16_t outMin, int16_t outMax) {
    int16_t deviation = raw - center;
    
    // Apply deadband
    if (abs(deviation) < DEADBAND) {
        return 0;
    }
    
    // Remove deadband from range
    if (deviation > 0) {
        deviation -= DEADBAND;
    } else {
        deviation += DEADBAND;
    }
    
    // Scale to output range
    int16_t maxDev = 512 - DEADBAND;
    int16_t halfRange = (outMax - outMin) / 2;
    return ((int32_t)deviation * halfRange) / maxDev;
}

int16_t applyExpo(int16_t value, float expo) {
    float normalized = value / 500.0f;
    float curved = normalized * (1.0f - expo) + 
                   (normalized * normalized * normalized) * expo;
    return (int16_t)(curved * 500.0f);
}

void readJoysticks() {
    // Average 4 samples for noise reduction
    int32_t sum[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        sum[0] += analogRead(PIN_JOY_THROTTLE);
        sum[1] += analogRead(PIN_JOY_YAW);
        sum[2] += analogRead(PIN_JOY_PITCH);
        sum[3] += analogRead(PIN_JOY_ROLL);
    }
    rawThrottle = sum[0] / 4;
    rawYaw = sum[1] / 4;
    rawPitch = sum[2] / 4;
    rawRoll = sum[3] / 4;
    
    // Process throttle (0-1000, no center)
    throttle = map(rawThrottle, joyCal.throttleMin, joyCal.throttleMax, 0, 1000);
    throttle = constrain(throttle, 0, 1000);
    
    // Process centered axes (-500 to +500)
    yaw = applyDeadbandAndScale(rawYaw, joyCal.yawCenter, -500, 500);
    pitch = applyDeadbandAndScale(rawPitch, joyCal.pitchCenter, -500, 500);
    roll = applyDeadbandAndScale(rawRoll, joyCal.rollCenter, -500, 500);
    
    // Apply expo
    yaw = applyExpo(yaw, EXPO_FACTOR);
    pitch = applyExpo(pitch, EXPO_FACTOR);
    roll = applyExpo(roll, EXPO_FACTOR);
}

void readSwitches() {
    // Save previous states
    prevArm = swArm;
    prevCalib = btnCalib;
    prevMotor = btnMotor;
    
    // Read switches (active LOW with pullup)
    swArm = !digitalRead(PIN_SW_ARM);
    swAltHold = !digitalRead(PIN_SW_ALTHOLD);
    btnCalib = !digitalRead(PIN_BTN_CALIB);
    btnMotor = !digitalRead(PIN_BTN_MOTOR);
    
    // Edge detection with sound
    if (btnCalib && !prevCalib) {
        soundButtonPress();
    }
    if (btnMotor && !prevMotor) {
        soundButtonPress();
    }
    if (swArm != prevArm) {
        if (swArm) {
            soundArmOn();
        } else {
            soundArmOff();
        }
    }
}

void calibrateJoystickCenters() {
    Serial.println(F("Calibrating joystick centers..."));
    Serial.println(F("Keep sticks CENTERED!"));
    
    beepBlocking(200, 1500);
    delay(500);
    
    int32_t sum[3] = {0, 0, 0};
    for (int i = 0; i < 50; i++) {
        sum[0] += analogRead(PIN_JOY_YAW);
        sum[1] += analogRead(PIN_JOY_PITCH);
        sum[2] += analogRead(PIN_JOY_ROLL);
        delay(20);
    }
    
    joyCal.yawCenter = sum[0] / 50;
    joyCal.pitchCenter = sum[1] / 50;
    joyCal.rollCenter = sum[2] / 50;
    
    Serial.print(F("Calibration done! Centers: Yaw="));
    Serial.print(joyCal.yawCenter);
    Serial.print(F(" Pitch="));
    Serial.print(joyCal.pitchCenter);
    Serial.print(F(" Roll="));
    Serial.println(joyCal.rollCenter);
    
    beepBlocking(100, 2000); delay(50);
    beepBlocking(100, 2500); delay(50);
    beepBlocking(200, 3000);
}

// ============================================================================
//                          RADIO FUNCTIONS
// ============================================================================

bool initRadio() {
    if (!radio.begin()) {
        return false;
    }
    
    radio.setChannel(RF_CHANNEL);
    radio.setDataRate(RF24_2MBPS);
    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(16);  // Fixed 16 bytes - must match receiver!
    radio.setAutoAck(true);
    radio.setRetries(5, 3);    // 5 = 1500us delay, 3 retries
    radio.setCRCLength(RF24_CRC_16);
    radio.openWritingPipe(radioAddress);
    radio.stopListening();
    
    Serial.print(F("  Channel: ")); Serial.println(RF_CHANNEL);
    Serial.print(F("  Address: ")); 
    for(int i=0; i<5; i++) { Serial.print((char)radioAddress[i]); }
    Serial.println();
    Serial.println(F("  Payload: 16 bytes"));
    
    return true;
}

bool sendPacket() {
    // Build packet
    txPacket.throttle = throttle;
    txPacket.yaw = yaw;
    txPacket.pitch = pitch;
    txPacket.roll = roll;
    
    // Build switches byte
    txPacket.switches = 0;
    if (swArm)     txPacket.switches |= (1 << SW_ARM);
    if (btnCalib)  txPacket.switches |= (1 << SW_CALIBRATE);
    if (btnMotor)  txPacket.switches |= (1 << SW_MOTORTEST);
    if (swAltHold) txPacket.switches |= (1 << SW_ALTHOLD);
    
    txPacket.sequence = packetSequence++;
    txPacket.channel = RF_CHANNEL;
    txPacket.reserved = 0;
    txPacket.calcChecksum();
    
    // Send
    packetsSent++;
    bool success = radio.write(&txPacket, sizeof(txPacket));
    
    if (success) {
        packetsAcked++;
        lastAckTime = millis();
        
        if (!connected) {
            connected = true;
            Serial.println(F("\n*** CONNECTED TO DRONE! ***"));
            soundConnected();
        }
        return true;
    } else {
        packetsFailed++;
        
        if (connected && millis() - lastAckTime > 500) {
            connected = false;
            Serial.println(F("\n*** CONNECTION LOST! ***"));
            soundDisconnected();
        }
        return false;
    }
}

// ============================================================================
//                            LED UPDATE
// ============================================================================

void updateLED() {
    uint32_t now = millis();
    
    if (connected) {
        if (swArm) {
            // Solid when armed
            digitalWrite(PIN_LED, HIGH);
        } else {
            // Slow blink when connected
            if (now - timeLED >= 500) {
                timeLED = now;
                ledState = !ledState;
                digitalWrite(PIN_LED, ledState);
            }
        }
    } else {
        // Fast blink when disconnected
        if (now - timeLED >= 100) {
            timeLED = now;
            ledState = !ledState;
            digitalWrite(PIN_LED, ledState);
        }
    }
}

// ============================================================================
//                           DEBUG OUTPUT
// ============================================================================

void printDebug() {
    // Connection status
    Serial.print(connected ? F("CONN ") : F("---- "));
    
    // Packet stats
    Serial.print(F("TX:"));
    Serial.print(packetsSent);
    Serial.print(F(" OK:"));
    Serial.print(packetsAcked);
    Serial.print(F(" FAIL:"));
    Serial.print(packetsFailed);
    
    // Success rate
    if (packetsSent > 0) {
        float rate = (float)packetsAcked / packetsSent * 100.0f;
        Serial.print(F(" ("));
        Serial.print(rate, 1);
        Serial.print(F("%)"));
    }
    
    // Joystick values
    Serial.print(F(" | T:"));
    Serial.print(throttle);
    Serial.print(F(" Y:"));
    Serial.print(yaw);
    Serial.print(F(" P:"));
    Serial.print(pitch);
    Serial.print(F(" R:"));
    Serial.print(roll);
    
    // Switches
    Serial.print(F(" | ARM:"));
    Serial.print(swArm ? F("ON") : F("--"));
    Serial.print(F(" ALT:"));
    Serial.println(swAltHold ? F("ON") : F("--"));
}

// ============================================================================
//                              SETUP
// ============================================================================

void setup() {
    // ALWAYS init serial first for debugging
    Serial.begin(SERIAL_BAUD);
    delay(100);
    Serial.println();
    Serial.println(F("============================="));
    Serial.println(F("   QUADCOPTER REMOTE v3.0"));
    Serial.println(F("============================="));
    Serial.print(F("RF Channel: "));
    Serial.println(RF_CHANNEL);
    Serial.println();
    
    // Initialize pins
    Serial.println(F("Init pins..."));
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_SW_ARM, INPUT_PULLUP);
    pinMode(PIN_SW_ALTHOLD, INPUT_PULLUP);
    pinMode(PIN_BTN_CALIB, INPUT_PULLUP);
    pinMode(PIN_BTN_MOTOR, INPUT_PULLUP);
    
    digitalWrite(PIN_LED, HIGH);
    Serial.println(F("Pins OK"));
    
    // Startup beep
    Serial.println(F("Buzzer test..."));
    soundStartup();
    Serial.println(F("Buzzer OK"));
    
    // Initialize radio
    Serial.println(F("Init NRF24L01..."));
    Serial.print(F("  CE pin: ")); Serial.println(PIN_RF_CE);
    Serial.print(F("  CSN pin: ")); Serial.println(PIN_RF_CSN);
    
    if (!initRadio()) {
        Serial.println(F("*** NRF24L01 FAILED! ***"));
        Serial.println(F("Check wiring:"));
        Serial.println(F("  VCC  -> 3.3V (NOT 5V!)"));
        Serial.println(F("  GND  -> GND"));
        Serial.println(F("  CE   -> D9"));
        Serial.println(F("  CSN  -> D10"));
        Serial.println(F("  SCK  -> D13"));
        Serial.println(F("  MOSI -> D11"));
        Serial.println(F("  MISO -> D12"));
        Serial.println(F("Add 10-100uF capacitor on VCC!"));
        
        while (1) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            beepBlocking(200, 500);
            delay(300);
        }
    }
    
    Serial.println(F("NRF24L01 OK!"));
    
    // Test reading joysticks
    Serial.println(F("Testing joysticks..."));
    Serial.print(F("  Throttle (A0): ")); Serial.println(analogRead(PIN_JOY_THROTTLE));
    Serial.print(F("  Yaw (A1): ")); Serial.println(analogRead(PIN_JOY_YAW));
    Serial.print(F("  Pitch (A2): ")); Serial.println(analogRead(PIN_JOY_PITCH));
    Serial.print(F("  Roll (A3): ")); Serial.println(analogRead(PIN_JOY_ROLL));
    
    // Test switches
    Serial.println(F("Testing switches..."));
    Serial.print(F("  ARM (D2): ")); Serial.println(digitalRead(PIN_SW_ARM) ? "OFF" : "ON");
    Serial.print(F("  ALT (D3): ")); Serial.println(digitalRead(PIN_SW_ALTHOLD) ? "OFF" : "ON");
    Serial.print(F("  CAL (D4): ")); Serial.println(digitalRead(PIN_BTN_CALIB) ? "OFF" : "ON");
    Serial.print(F("  MTR (D5): ")); Serial.println(digitalRead(PIN_BTN_MOTOR) ? "OFF" : "ON");
    
    // Calibrate joystick centers
    Serial.println();
    calibrateJoystickCenters();
    
    Serial.println();
    Serial.println(F("============================="));
    Serial.println(F("       READY TO FLY!"));
    Serial.println(F("============================="));
    Serial.println(F("Transmitting to drone..."));
    Serial.println();
    
    beepBlocking(100, 2000); delay(100);
    beepBlocking(100, 2500); delay(100);
    beepBlocking(200, 3000);
    
    // Initialize timing
    timeTX = timeInput = millis();
    timeDebug = timeLED = millis();
}

// ============================================================================
//                             MAIN LOOP
// ============================================================================

void loop() {
    uint32_t now = millis();
    
    // Read inputs (100Hz)
    if (now - timeInput >= INPUT_RATE_MS) {
        timeInput = now;
        readJoysticks();
        readSwitches();
    }
    
    // Transmit (50Hz)
    if (now - timeTX >= TX_RATE_MS) {
        timeTX = now;
        sendPacket();
    }
    
    // Update LED
    updateLED();
    
    // Debug output (4Hz)
    if (now - timeDebug >= DEBUG_RATE_MS) {
        timeDebug = now;
        printDebug();
    }
}
