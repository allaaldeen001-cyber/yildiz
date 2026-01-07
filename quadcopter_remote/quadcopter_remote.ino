/**
 * ============================================================================
 *                    QUADCOPTER REMOTE CONTROLLER v3.3
 * ============================================================================
 * 
 * CONTROLS:
 *   D2 (toggle)  - ARM/DISARM
 *   D3 (toggle)  - Altitude Hold ON/OFF
 *   D4 (button)  - Calibration / ESC Cal
 *   D5 (button)  - Motor Test
 *   D6 (button)  - BEEPER (find drone)
 *   D7 (button)  - HEADLESS mode toggle
 *   
 *   A0 - Throttle
 *   A1 - Yaw
 *   A2 - Pitch
 *   A3 - Roll
 *   A6 - RATE pot (sensitivity 50%-150%)
 *   A7 - EXPO pot (stick curve 0%-50%)
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

// ============================================================================
//                            PIN DEFINITIONS
// ============================================================================
#define PIN_RF_CE           9
#define PIN_RF_CSN          10

// Switches and Buttons
#define PIN_SW_ARM          2
#define PIN_SW_ALTHOLD      3
#define PIN_BTN_CALIB       4
#define PIN_BTN_MOTOR       5
#define PIN_BTN_BEEPER      6       // NEW: Find drone beeper
#define PIN_BTN_HEADLESS    7       // NEW: Headless mode

// Joysticks
#define PIN_JOY_THROTTLE    A0
#define PIN_JOY_YAW         A1
#define PIN_JOY_PITCH       A2
#define PIN_JOY_ROLL        A3

// Adjustment Pots
#define PIN_POT_RATE        A6      // NEW: Rate/sensitivity adjustment
#define PIN_POT_EXPO        A7      // NEW: Expo curve adjustment

// ============================================================================
//                         JOYSTICK CONFIGURATION
// ============================================================================
#define DEADBAND            30

// ============================================================================
//                         TIMING CONFIGURATION
// ============================================================================
#define TX_RATE_MS          20
#define INPUT_RATE_MS       10
#define DEBUG_RATE_MS       500

// ============================================================================
//                          RF PACKET STRUCTURE
// ============================================================================
struct __attribute__((packed)) ControlPacket {
    uint16_t throttle;
    int16_t  yaw;
    int16_t  pitch;
    int16_t  roll;
    uint8_t  switches;      // 8 switch bits
    uint8_t  checksum;
    uint32_t sequence;
    uint8_t  ratePot;       // A6 pot value (0-255)
    uint8_t  expoPot;       // A7 pot value (0-255)
    
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
#define SW_BEEPER           4       // NEW
#define SW_HEADLESS         5       // NEW

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

// Pot values (0-255)
uint8_t ratePotValue = 128;     // Default 50% = 1.0x rate
uint8_t expoPotValue = 77;      // Default ~30% expo

// Switch states
bool swArm = false, swAltHold = false, swHeadless = false;
bool btnCalib = false, btnMotor = false, btnBeeper = false;
bool prevArm = false, prevHeadless = false;

bool transmitting = false;
uint32_t packetsSent = 0;

uint32_t timeTX = 0, timeInput = 0, timeDebug = 0;

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
    // expo: 0.0 = linear, 0.5 = very curved
    float normalized = value / 500.0f;
    float curved = normalized * (1.0f - expo) + 
                   (normalized * normalized * normalized) * expo;
    return (int16_t)(curved * 500.0f);
}

void readJoysticks() {
    // Average 4 readings
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
    
    // Read pots (A6, A7 are analog-only pins)
    int rawRate = analogRead(PIN_POT_RATE);
    int rawExpo = analogRead(PIN_POT_EXPO);
    ratePotValue = map(rawRate, 0, 1023, 0, 255);
    expoPotValue = map(rawExpo, 0, 1023, 0, 255);
    
    // Calculate expo from pot (0-255 -> 0.0-0.5)
    float expo = expoPotValue / 510.0f;
    
    // Scale throttle
    throttle = map(rawThrottle, joyCal.throttleMin, joyCal.throttleMax, 0, 1000);
    throttle = constrain(throttle, 0, 1000);
    
    // Center-based controls with deadband
    yaw = applyDeadbandAndScale(rawYaw, joyCal.yawCenter, -500, 500);
    pitch = applyDeadbandAndScale(rawPitch, joyCal.pitchCenter, -500, 500);
    roll = applyDeadbandAndScale(rawRoll, joyCal.rollCenter, -500, 500);
    
    // Apply expo (from A7 pot)
    yaw = applyExpo(yaw, expo);
    pitch = applyExpo(pitch, expo);
    roll = applyExpo(roll, expo);
}

void readSwitches() {
    prevArm = swArm;
    prevHeadless = swHeadless;
    
    // Read all inputs (active LOW with internal pullup)
    bool rawArm = !digitalRead(PIN_SW_ARM);
    bool rawAltHold = !digitalRead(PIN_SW_ALTHOLD);
    bool rawCalib = !digitalRead(PIN_BTN_CALIB);
    bool rawMotor = !digitalRead(PIN_BTN_MOTOR);
    bool rawBeeper = !digitalRead(PIN_BTN_BEEPER);
    bool rawHeadless = !digitalRead(PIN_BTN_HEADLESS);
    
    // Debounce toggle switches
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
    
    // Buttons (direct read)
    btnCalib = rawCalib;
    btnMotor = rawMotor;
    btnBeeper = rawBeeper;
    
    // Headless toggle (press to toggle)
    static bool lastRawHeadless = false;
    if (rawHeadless && !lastRawHeadless) {
        swHeadless = !swHeadless;  // Toggle on press
        Serial.print(F("Headless: "));
        Serial.println(swHeadless ? F("ON") : F("OFF"));
    }
    lastRawHeadless = rawHeadless;
}

void calibrateJoystickCenters() {
    Serial.println(F("Calibrating centers..."));
    Serial.println(F("Keep sticks CENTERED!"));
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
    
    // Pack all switches into one byte
    txPacket.switches = 0;
    if (swArm)      txPacket.switches |= (1 << SW_ARM);
    if (btnCalib)   txPacket.switches |= (1 << SW_CALIBRATE);
    if (btnMotor)   txPacket.switches |= (1 << SW_MOTORTEST);
    if (swAltHold)  txPacket.switches |= (1 << SW_ALTHOLD);
    if (btnBeeper)  txPacket.switches |= (1 << SW_BEEPER);
    if (swHeadless) txPacket.switches |= (1 << SW_HEADLESS);
    
    txPacket.sequence = packetSequence++;
    txPacket.ratePot = ratePotValue;
    txPacket.expoPot = expoPotValue;
    txPacket.calcChecksum();
    
    radio.write(&txPacket, sizeof(txPacket));
    packetsSent++;
    
    if (!transmitting && packetsSent > 10) {
        transmitting = true;
        Serial.println(F("\n*** TRANSMITTING ***"));
    }
}

// ============================================================================
//                           DEBUG OUTPUT
// ============================================================================
void printDebug() {
    Serial.print(F("TX:")); Serial.print(packetsSent);
    
    Serial.print(F(" T:")); Serial.print(throttle);
    Serial.print(F(" Y:")); Serial.print(yaw);
    Serial.print(F(" P:")); Serial.print(pitch);
    Serial.print(F(" R:")); Serial.print(roll);
    
    // Switches: A=Arm, C=Calib, M=Motor, H=AltHold, B=Beeper, L=Headless
    Serial.print(F(" SW:"));
    Serial.print(swArm ? F("A") : F("-"));
    Serial.print(btnCalib ? F("C") : F("-"));
    Serial.print(btnMotor ? F("M") : F("-"));
    Serial.print(swAltHold ? F("H") : F("-"));
    Serial.print(btnBeeper ? F("B") : F("-"));
    Serial.print(swHeadless ? F("L") : F("-"));
    
    // Pots
    Serial.print(F(" RATE:")); Serial.print(ratePotValue);
    Serial.print(F(" EXPO:")); Serial.print(expoPotValue);
    
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
    Serial.println(F("   QUADCOPTER REMOTE v3.3"));
    Serial.println(F("================================"));
    Serial.print(F("RF Channel: ")); Serial.println(RF_CHANNEL);
    Serial.println();
    
    // Setup all input pins with pullups
    pinMode(PIN_SW_ARM, INPUT_PULLUP);
    pinMode(PIN_SW_ALTHOLD, INPUT_PULLUP);
    pinMode(PIN_BTN_CALIB, INPUT_PULLUP);
    pinMode(PIN_BTN_MOTOR, INPUT_PULLUP);
    pinMode(PIN_BTN_BEEPER, INPUT_PULLUP);
    pinMode(PIN_BTN_HEADLESS, INPUT_PULLUP);
    
    // Init Radio
    Serial.println(F("Init NRF24L01..."));
    if (!initRadio()) {
        Serial.println(F("*** NRF24L01 FAILED! ***"));
        while (1) { delay(500); }
    }
    Serial.println(F("NRF24L01 OK"));
    
    // Show all inputs
    Serial.println(F("\n--- INPUT TEST ---"));
    Serial.println(F("Joysticks:"));
    Serial.print(F("  Throttle (A0): ")); Serial.println(analogRead(PIN_JOY_THROTTLE));
    Serial.print(F("  Yaw (A1):      ")); Serial.println(analogRead(PIN_JOY_YAW));
    Serial.print(F("  Pitch (A2):    ")); Serial.println(analogRead(PIN_JOY_PITCH));
    Serial.print(F("  Roll (A3):     ")); Serial.println(analogRead(PIN_JOY_ROLL));
    
    Serial.println(F("Pots:"));
    Serial.print(F("  Rate (A6):     ")); Serial.println(analogRead(PIN_POT_RATE));
    Serial.print(F("  Expo (A7):     ")); Serial.println(analogRead(PIN_POT_EXPO));
    
    Serial.println(F("Switches/Buttons:"));
    Serial.print(F("  ARM (D2):      ")); Serial.println(!digitalRead(PIN_SW_ARM) ? F("ON") : F("OFF"));
    Serial.print(F("  ALTHOLD (D3):  ")); Serial.println(!digitalRead(PIN_SW_ALTHOLD) ? F("ON") : F("OFF"));
    Serial.print(F("  CALIB (D4):    ")); Serial.println(!digitalRead(PIN_BTN_CALIB) ? F("PRESSED") : F("-"));
    Serial.print(F("  MOTOR (D5):    ")); Serial.println(!digitalRead(PIN_BTN_MOTOR) ? F("PRESSED") : F("-"));
    Serial.print(F("  BEEPER (D6):   ")); Serial.println(!digitalRead(PIN_BTN_BEEPER) ? F("PRESSED") : F("-"));
    Serial.print(F("  HEADLESS (D7): ")); Serial.println(!digitalRead(PIN_BTN_HEADLESS) ? F("PRESSED") : F("-"));
    
    Serial.println();
    calibrateJoystickCenters();
    
    Serial.println();
    Serial.println(F("================================"));
    Serial.println(F("         CONTROLS"));
    Serial.println(F("================================"));
    Serial.println(F("D2: ARM toggle"));
    Serial.println(F("D3: Altitude Hold toggle"));
    Serial.println(F("D4: Calibration / ESC Cal"));
    Serial.println(F("D5: Motor Test"));
    Serial.println(F("D6: BEEPER (find drone)"));
    Serial.println(F("D7: HEADLESS mode toggle"));
    Serial.println(F("A6: Rate (sensitivity)"));
    Serial.println(F("A7: Expo (stick curve)"));
    Serial.println(F("================================\n"));
    
    timeTX = timeInput = millis();
    timeDebug = millis();
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
    
    // Debug output (2Hz)
    if (now - timeDebug >= DEBUG_RATE_MS) {
        timeDebug = now;
        printDebug();
    }
}
