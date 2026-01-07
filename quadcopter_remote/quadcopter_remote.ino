/**
 * ============================================================================
 *                    QUADCOPTER REMOTE CONTROLLER v3.2
 * ============================================================================
 * 
 * ESC CALIBRATION PROCEDURE:
 *   1. Power OFF drone (disconnect battery)
 *   2. Power ON remote
 *   3. HOLD D4 (calibrate) button
 *   4. While holding D4, connect battery to drone
 *   5. Wait for ESC beeps (high tone = max throttle)
 *   6. RELEASE D4 button
 *   7. Wait for ESC beeps (low tone = min throttle)
 *   8. Done! Drone will restart normally
 * 
 * CONTROLS:
 *   D2 (toggle) - ARM/DISARM
 *   D3 (toggle) - Altitude Hold ON/OFF
 *   D4 (button) - Calibration / Motor Test
 *   D5 (button) - Motor Test
 *   A0 - Throttle
 *   A1 - Yaw
 *   A2 - Pitch
 *   A3 - Roll
 * 
 * ============================================================================
 */

#include <SPI.h>
#include <RF24.h>

// ============================================================================
//                              CONFIGURATION
// ============================================================================
#define RF_CHANNEL          108     // MUST match flight controller!
#define SERIAL_BAUD         115200

// ============================================================================
//                            PIN DEFINITIONS
// ============================================================================
#define PIN_RF_CE           9
#define PIN_RF_CSN          10
#define PIN_SW_ARM          2
#define PIN_SW_ALTHOLD      3
#define PIN_BTN_CALIB       4
#define PIN_BTN_MOTOR       5
#define PIN_BUZZER          6
#define PIN_LED             7
#define PIN_JOY_THROTTLE    A0
#define PIN_JOY_YAW         A1
#define PIN_JOY_PITCH       A2
#define PIN_JOY_ROLL        A3

// ============================================================================
//                         JOYSTICK CONFIGURATION
// ============================================================================
#define DEADBAND            30
#define EXPO_FACTOR         0.3f

// ============================================================================
//                         TIMING CONFIGURATION
// ============================================================================
#define TX_RATE_MS          20      // 50Hz transmission
#define INPUT_RATE_MS       10      // 100Hz input sampling
#define DEBUG_RATE_MS       500     // 2Hz debug output

// ============================================================================
//                          RF PACKET STRUCTURE
// ============================================================================
struct __attribute__((packed)) ControlPacket {
    uint16_t throttle;
    int16_t  yaw;
    int16_t  pitch;
    int16_t  roll;
    uint8_t  switches;
    uint8_t  checksum;
    uint32_t sequence;
    uint8_t  channel;
    uint8_t  reserved;
    
    void calcChecksum() {
        uint8_t* data = (uint8_t*)this;
        uint8_t calc = 0;
        for (uint8_t i = 0; i < 9; i++) calc ^= data[i];
        checksum = calc;
    }
};

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
ControlPacket txPacket;
uint32_t packetSequence = 0;

struct {
    int16_t throttleMin = 0;
    int16_t throttleMax = 1023;
    int16_t yawCenter = 512;
    int16_t pitchCenter = 512;
    int16_t rollCenter = 512;
} joyCal;

int16_t rawThrottle = 0, rawYaw = 0, rawPitch = 0, rawRoll = 0;
int16_t throttle = 0, yaw = 0, pitch = 0, roll = 0;

bool swArm = false, swAltHold = false;
bool btnCalib = false, btnMotor = false;
bool prevArm = false, prevCalib = false, prevMotor = false;

bool transmitting = false;
uint32_t packetsSent = 0;

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

void soundReady() {
    beepBlocking(100, 2000); delay(100);
    beepBlocking(100, 2500); delay(100);
    beepBlocking(200, 3000);
}

// ============================================================================
//                         JOYSTICK FUNCTIONS
// ============================================================================
int16_t applyDeadbandAndScale(int16_t raw, int16_t center, int16_t outMin, int16_t outMax) {
    int16_t deviation = raw - center;
    
    if (abs(deviation) < DEADBAND) return 0;
    
    if (deviation > 0) deviation -= DEADBAND;
    else deviation += DEADBAND;
    
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
    // Average 4 readings for stability
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
    
    // Scale throttle
    throttle = map(rawThrottle, joyCal.throttleMin, joyCal.throttleMax, 0, 1000);
    throttle = constrain(throttle, 0, 1000);
    
    // Center-based controls
    yaw = applyDeadbandAndScale(rawYaw, joyCal.yawCenter, -500, 500);
    pitch = applyDeadbandAndScale(rawPitch, joyCal.pitchCenter, -500, 500);
    roll = applyDeadbandAndScale(rawRoll, joyCal.rollCenter, -500, 500);
    
    // Apply expo
    yaw = applyExpo(yaw, EXPO_FACTOR);
    pitch = applyExpo(pitch, EXPO_FACTOR);
    roll = applyExpo(roll, EXPO_FACTOR);
}

void readSwitches() {
    prevArm = swArm;
    prevCalib = btnCalib;
    prevMotor = btnMotor;
    
    // Read raw values (active LOW with pullup)
    bool rawArm = !digitalRead(PIN_SW_ARM);
    bool rawAltHold = !digitalRead(PIN_SW_ALTHOLD);
    bool rawCalib = !digitalRead(PIN_BTN_CALIB);
    bool rawMotor = !digitalRead(PIN_BTN_MOTOR);
    
    // Debounce ARM and ALT HOLD switches
    static bool lastRawArm = false;
    static bool lastRawAltHold = false;
    static uint32_t armStableTime = 0;
    static uint32_t altHoldStableTime = 0;
    uint32_t now = millis();
    
    if (rawArm != lastRawArm) {
        armStableTime = now;
        lastRawArm = rawArm;
    } else if (now - armStableTime > 50) {
        swArm = rawArm;
    }
    
    if (rawAltHold != lastRawAltHold) {
        altHoldStableTime = now;
        lastRawAltHold = rawAltHold;
    } else if (now - altHoldStableTime > 50) {
        swAltHold = rawAltHold;
    }
    
    // Buttons (no debounce needed - single press detection)
    btnCalib = rawCalib;
    btnMotor = rawMotor;
    
    // Sound feedback
    if (btnCalib && !prevCalib) soundButtonPress();
    if (btnMotor && !prevMotor) soundButtonPress();
    if (swArm != prevArm) {
        if (swArm) soundArmOn();
        else soundArmOff();
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
    
    Serial.print(F("Centers: Y="));
    Serial.print(joyCal.yawCenter);
    Serial.print(F(" P="));
    Serial.print(joyCal.pitchCenter);
    Serial.print(F(" R="));
    Serial.println(joyCal.rollCenter);
    
    beepBlocking(100, 2000); delay(50);
    beepBlocking(100, 2500); delay(50);
    beepBlocking(200, 3000);
}

// ============================================================================
//                          RADIO FUNCTIONS
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
    
    delay(100);
    return radio.isPVariant();
}

void sendPacket() {
    // Build packet
    txPacket.throttle = throttle;
    txPacket.yaw = yaw;
    txPacket.pitch = pitch;
    txPacket.roll = roll;
    
    txPacket.switches = 0;
    if (swArm)     txPacket.switches |= (1 << SW_ARM);
    if (btnCalib)  txPacket.switches |= (1 << SW_CALIBRATE);
    if (btnMotor)  txPacket.switches |= (1 << SW_MOTORTEST);
    if (swAltHold) txPacket.switches |= (1 << SW_ALTHOLD);
    
    txPacket.sequence = packetSequence++;
    txPacket.channel = RF_CHANNEL;
    txPacket.reserved = 0;
    txPacket.calcChecksum();
    
    // Send (NO_ACK mode)
    radio.write(&txPacket, sizeof(txPacket));
    packetsSent++;
    
    if (!transmitting && packetsSent > 10) {
        transmitting = true;
        Serial.println(F("\n*** TRANSMITTING ***"));
    }
}

// ============================================================================
//                            LED UPDATE
// ============================================================================
void updateLED() {
    uint32_t now = millis();
    uint16_t interval = swArm ? 0 : (transmitting ? 500 : 100);
    
    if (interval == 0) {
        digitalWrite(PIN_LED, HIGH);
    } else if (now - timeLED >= interval) {
        timeLED = now;
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState);
    }
}

// ============================================================================
//                           DEBUG OUTPUT
// ============================================================================
void printDebug() {
    Serial.print(F("TX:")); Serial.print(packetsSent);
    
    Serial.print(F(" | T:")); Serial.print(throttle);
    Serial.print(F(" Y:")); Serial.print(yaw);
    Serial.print(F(" P:")); Serial.print(pitch);
    Serial.print(F(" R:")); Serial.print(roll);
    
    Serial.print(F(" | SW:"));
    Serial.print(swArm ? F("A") : F("-"));
    Serial.print(btnCalib ? F("C") : F("-"));
    Serial.print(btnMotor ? F("M") : F("-"));
    Serial.print(swAltHold ? F("H") : F("-"));
    
    Serial.println();
}

// ============================================================================
//                              SETUP
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);
    Serial.println();
    Serial.println(F("================================"));
    Serial.println(F("   QUADCOPTER REMOTE v3.2"));
    Serial.println(F("================================"));
    Serial.print(F("RF Channel: ")); Serial.println(RF_CHANNEL);
    Serial.println();
    
    // Setup pins
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_SW_ARM, INPUT_PULLUP);
    pinMode(PIN_SW_ALTHOLD, INPUT_PULLUP);
    pinMode(PIN_BTN_CALIB, INPUT_PULLUP);
    pinMode(PIN_BTN_MOTOR, INPUT_PULLUP);
    
    digitalWrite(PIN_LED, HIGH);
    soundStartup();
    
    // Init Radio
    Serial.println(F("Init NRF24L01..."));
    Serial.println(F("CRITICAL: 10-100uF cap on VCC!"));
    
    if (!initRadio()) {
        Serial.println(F("*** NRF24L01 FAILED! ***"));
        while (1) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            beepBlocking(200, 500);
            delay(300);
        }
    }
    Serial.println(F("NRF24L01 OK"));
    
    // Show joystick values
    Serial.println(F("\nJoystick raw values:"));
    Serial.print(F("  Throttle: ")); Serial.println(analogRead(PIN_JOY_THROTTLE));
    Serial.print(F("  Yaw:      ")); Serial.println(analogRead(PIN_JOY_YAW));
    Serial.print(F("  Pitch:    ")); Serial.println(analogRead(PIN_JOY_PITCH));
    Serial.print(F("  Roll:     ")); Serial.println(analogRead(PIN_JOY_ROLL));
    
    // Show switch states
    Serial.println(F("\nSwitch states:"));
    Serial.print(F("  ARM (D2):      ")); Serial.println(!digitalRead(PIN_SW_ARM) ? F("ON") : F("OFF"));
    Serial.print(F("  ALT HOLD (D3): ")); Serial.println(!digitalRead(PIN_SW_ALTHOLD) ? F("ON") : F("OFF"));
    Serial.print(F("  CALIB (D4):    ")); Serial.println(!digitalRead(PIN_BTN_CALIB) ? F("PRESSED") : F("released"));
    Serial.print(F("  MOTOR (D5):    ")); Serial.println(!digitalRead(PIN_BTN_MOTOR) ? F("PRESSED") : F("released"));
    
    Serial.println();
    calibrateJoystickCenters();
    
    Serial.println();
    Serial.println(F("================================"));
    Serial.println(F("        READY TO FLY!"));
    Serial.println(F("================================"));
    Serial.println();
    Serial.println(F("ESC CALIBRATION:"));
    Serial.println(F("  1. Power OFF drone"));
    Serial.println(F("  2. Hold D4 button"));
    Serial.println(F("  3. Power ON drone (keep D4 held)"));
    Serial.println(F("  4. Wait for high beep"));
    Serial.println(F("  5. Release D4"));
    Serial.println(F("  6. Wait for low beep"));
    Serial.println(F("  7. Done!"));
    Serial.println();
    
    soundReady();
    
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
    
    // Debug output (2Hz)
    if (now - timeDebug >= DEBUG_RATE_MS) {
        timeDebug = now;
        printDebug();
    }
}
