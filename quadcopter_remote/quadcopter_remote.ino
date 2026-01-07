/**
 * ============================================================================
 *                    QUADCOPTER REMOTE CONTROLLER v5.0
 *                         ROCK SOLID EDITION
 * ============================================================================
 * 
 * CONTROLS:
 *   D2 (toggle)  = ARM/DISARM
 *   D3 (toggle)  = Altitude Hold (not used in v5)
 *   D4 (button)  = Calibration / Motor Test
 *   D5 (button)  = Motor Test
 *   D6 (button)  = BEEPER (find drone)
 *   D7 (button)  = HEADLESS mode toggle
 *   
 *   A0 = Throttle
 *   A1 = Yaw
 *   A2 = Pitch
 *   A3 = Roll
 *   A6 = RATE pot (sensitivity)
 *   A7 = EXPO pot (stick curve)
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
#define PIN_CE              9
#define PIN_CSN             10

#define PIN_ARM             2
#define PIN_ALTHOLD         3
#define PIN_CALIB           4
#define PIN_MOTOR           5
#define PIN_BEEPER          6
#define PIN_HEADLESS        7

#define PIN_THROTTLE        A0
#define PIN_YAW             A1
#define PIN_PITCH           A2
#define PIN_ROLL            A3
#define PIN_RATE            A6
#define PIN_EXPO            A7

// ============================================================================
//                         CONFIGURATION
// ============================================================================
#define DEADBAND            25
#define TX_RATE_MS          20      // 50Hz
#define INPUT_RATE_MS       10      // 100Hz
#define DEBUG_RATE_MS       250     // 4Hz - faster debug

// ============================================================================
//                          RF PACKET
// ============================================================================
struct __attribute__((packed)) TxPacket {
    uint16_t throttle;      // 0-1000
    int16_t yaw;            // -500 to +500
    int16_t pitch;          // -500 to +500
    int16_t roll;           // -500 to +500
    uint8_t switches;       // Bit flags
    uint8_t checksum;
    uint32_t seq;
    uint8_t ratePot;        // 0-255
    uint8_t expoPot;        // 0-255
    
    void calcChecksum() {
        uint8_t c = 0;
        uint8_t* d = (uint8_t*)this;
        for (int i = 0; i < 9; i++) c ^= d[i];
        checksum = c;
    }
};

#define SW_ARM      0
#define SW_CALIB    1
#define SW_MOTOR    2
#define SW_ALTHOLD  3
#define SW_BEEPER   4
#define SW_HEADLESS 5

// ============================================================================
//                           GLOBALS
// ============================================================================
RF24 radio(PIN_CE, PIN_CSN);
const uint8_t addr[6] = "QUAD1";
TxPacket tx;
uint32_t seq = 0;
uint32_t txCount = 0;

// Joystick calibration
int16_t thrMin = 0, thrMax = 1023;
int16_t yawCenter = 512, pitchCenter = 512, rollCenter = 512;

// Raw values
int16_t rawThr, rawYaw, rawPitch, rawRoll;
int16_t rawRate, rawExpo;

// Processed values
int16_t throttle = 0;
int16_t yawVal = 0, pitchVal = 0, rollVal = 0;
uint8_t ratePot = 128, expoPot = 77;

// Switch states
bool swArm = false;
bool swAltHold = false;
bool btnCalib = false;
bool btnMotor = false;
bool btnBeeper = false;
bool btnHeadless = false;

// Headless toggle state
bool headlessOn = false;
bool prevHeadlessBtn = false;

// Timing
uint32_t timeTx = 0;
uint32_t timeInput = 0;
uint32_t timeDebug = 0;

// ============================================================================
//                         JOYSTICK FUNCTIONS
// ============================================================================
int16_t processStick(int16_t raw, int16_t center, float expo) {
    int16_t dev = raw - center;
    
    // Deadband
    if (abs(dev) < DEADBAND) return 0;
    
    // Remove deadband from calculation
    if (dev > 0) dev -= DEADBAND;
    else dev += DEADBAND;
    
    // Scale to -500 to +500
    int16_t maxDev = 512 - DEADBAND;
    float scaled = (float)dev / maxDev * 500.0f;
    
    // Apply expo (0.0 = linear, 0.5 = very curved)
    float norm = scaled / 500.0f;
    float curved = norm * (1.0f - expo) + (norm * norm * norm) * expo;
    
    return constrain((int16_t)(curved * 500.0f), -500, 500);
}

void readInputs() {
    // Read all analog inputs (average 4 samples)
    int32_t sum[6] = {0};
    for (int i = 0; i < 4; i++) {
        sum[0] += analogRead(PIN_THROTTLE);
        sum[1] += analogRead(PIN_YAW);
        sum[2] += analogRead(PIN_PITCH);
        sum[3] += analogRead(PIN_ROLL);
        sum[4] += analogRead(PIN_RATE);
        sum[5] += analogRead(PIN_EXPO);
    }
    rawThr = sum[0] / 4;
    rawYaw = sum[1] / 4;
    rawPitch = sum[2] / 4;
    rawRoll = sum[3] / 4;
    rawRate = sum[4] / 4;
    rawExpo = sum[5] / 4;
    
    // Process pots
    ratePot = map(rawRate, 0, 1023, 0, 255);
    expoPot = map(rawExpo, 0, 1023, 0, 255);
    
    // Expo from pot (0-255 -> 0.0-0.5)
    float expo = expoPot / 510.0f;
    
    // Process throttle (0-1000)
    throttle = map(rawThr, thrMin, thrMax, 0, 1000);
    throttle = constrain(throttle, 0, 1000);
    
    // Process sticks with expo
    yawVal = processStick(rawYaw, yawCenter, expo);
    pitchVal = processStick(rawPitch, pitchCenter, expo);
    rollVal = processStick(rawRoll, rollCenter, expo);
    
    // Read switches (active LOW with pullup)
    swArm = !digitalRead(PIN_ARM);
    swAltHold = !digitalRead(PIN_ALTHOLD);
    btnCalib = !digitalRead(PIN_CALIB);
    btnMotor = !digitalRead(PIN_MOTOR);
    btnBeeper = !digitalRead(PIN_BEEPER);
    
    // Headless button - toggle mode
    bool headlessBtn = !digitalRead(PIN_HEADLESS);
    if (headlessBtn && !prevHeadlessBtn) {
        headlessOn = !headlessOn;
        Serial.print(F("HEADLESS: ")); Serial.println(headlessOn ? F("ON") : F("OFF"));
    }
    prevHeadlessBtn = headlessBtn;
    btnHeadless = headlessOn;  // Send toggle state, not button state
}

void calibrateSticks() {
    Serial.println(F("\nCalibrating joystick centers..."));
    Serial.println(F("Keep sticks CENTERED!"));
    delay(500);
    
    int32_t sum[3] = {0};
    for (int i = 0; i < 50; i++) {
        sum[0] += analogRead(PIN_YAW);
        sum[1] += analogRead(PIN_PITCH);
        sum[2] += analogRead(PIN_ROLL);
        delay(20);
    }
    
    yawCenter = sum[0] / 50;
    pitchCenter = sum[1] / 50;
    rollCenter = sum[2] / 50;
    
    Serial.print(F("Centers: Y=")); Serial.print(yawCenter);
    Serial.print(F(" P=")); Serial.print(pitchCenter);
    Serial.print(F(" R=")); Serial.println(rollCenter);
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
    radio.openWritingPipe(addr);
    radio.stopListening();
    
    delay(100);
    return true;
}

void sendPacket() {
    // Build packet
    tx.throttle = throttle;
    tx.yaw = yawVal;
    tx.pitch = pitchVal;
    tx.roll = rollVal;
    
    // Pack switches
    tx.switches = 0;
    if (swArm)      tx.switches |= (1 << SW_ARM);
    if (btnCalib)   tx.switches |= (1 << SW_CALIB);
    if (btnMotor)   tx.switches |= (1 << SW_MOTOR);
    if (swAltHold)  tx.switches |= (1 << SW_ALTHOLD);
    if (btnBeeper)  tx.switches |= (1 << SW_BEEPER);
    if (btnHeadless) tx.switches |= (1 << SW_HEADLESS);
    
    tx.seq = seq++;
    tx.ratePot = ratePot;
    tx.expoPot = expoPot;
    tx.calcChecksum();
    
    // Send
    radio.write(&tx, sizeof(tx));
    txCount++;
}

// ============================================================================
//                           DEBUG OUTPUT
// ============================================================================
void printDebug() {
    Serial.print(F("TX:")); Serial.print(txCount);
    
    // Sticks
    Serial.print(F(" T:")); Serial.print(throttle);
    Serial.print(F(" Y:")); Serial.print(yawVal);
    Serial.print(F(" P:")); Serial.print(pitchVal);
    Serial.print(F(" R:")); Serial.print(rollVal);
    
    // Switches - show hex and individual bits
    Serial.print(F(" SW:0x")); Serial.print(tx.switches, HEX);
    Serial.print(F(" ["));
    Serial.print(swArm ? F("A") : F("-"));
    Serial.print(btnCalib ? F("C") : F("-"));
    Serial.print(btnMotor ? F("M") : F("-"));
    Serial.print(swAltHold ? F("H") : F("-"));
    Serial.print(btnBeeper ? F("B") : F("-"));
    Serial.print(btnHeadless ? F("L") : F("-"));
    Serial.print(F("]"));
    
    // Pots
    Serial.print(F(" Rate:")); Serial.print(ratePot);
    Serial.print(F(" Expo:")); Serial.print(expoPot);
    
    Serial.println();
}

// ============================================================================
//                              SETUP
// ============================================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);
    
    Serial.println(F("\n=============================="));
    Serial.println(F("   QuadRC v5.0 ROCK SOLID"));
    Serial.println(F("==============================\n"));
    
    // Setup pins
    pinMode(PIN_ARM, INPUT_PULLUP);
    pinMode(PIN_ALTHOLD, INPUT_PULLUP);
    pinMode(PIN_CALIB, INPUT_PULLUP);
    pinMode(PIN_MOTOR, INPUT_PULLUP);
    pinMode(PIN_BEEPER, INPUT_PULLUP);
    pinMode(PIN_HEADLESS, INPUT_PULLUP);
    
    // Init radio
    Serial.println(F("Init NRF24L01..."));
    Serial.println(F("  IMPORTANT: Add 10-100uF cap on VCC!"));
    
    if (!initRadio()) {
        Serial.println(F("  NRF24L01 FAILED!"));
        while (1) { delay(500); }
    }
    Serial.print(F("  OK - CH:")); Serial.println(RF_CHANNEL);
    
    // Test inputs
    Serial.println(F("\n--- INPUT TEST ---"));
    Serial.println(F("Joysticks (raw):"));
    Serial.print(F("  Throttle (A0): ")); Serial.println(analogRead(PIN_THROTTLE));
    Serial.print(F("  Yaw (A1):      ")); Serial.println(analogRead(PIN_YAW));
    Serial.print(F("  Pitch (A2):    ")); Serial.println(analogRead(PIN_PITCH));
    Serial.print(F("  Roll (A3):     ")); Serial.println(analogRead(PIN_ROLL));
    
    Serial.println(F("Pots (raw):"));
    Serial.print(F("  Rate (A6):     ")); Serial.println(analogRead(PIN_RATE));
    Serial.print(F("  Expo (A7):     ")); Serial.println(analogRead(PIN_EXPO));
    
    Serial.println(F("Switches (pressed=1):"));
    Serial.print(F("  ARM (D2):      ")); Serial.println(!digitalRead(PIN_ARM));
    Serial.print(F("  ALTHOLD (D3):  ")); Serial.println(!digitalRead(PIN_ALTHOLD));
    Serial.print(F("  CALIB (D4):    ")); Serial.println(!digitalRead(PIN_CALIB));
    Serial.print(F("  MOTOR (D5):    ")); Serial.println(!digitalRead(PIN_MOTOR));
    Serial.print(F("  BEEPER (D6):   ")); Serial.println(!digitalRead(PIN_BEEPER));
    Serial.print(F("  HEADLESS (D7): ")); Serial.println(!digitalRead(PIN_HEADLESS));
    
    // Calibrate
    calibrateSticks();
    
    Serial.println(F("\n=============================="));
    Serial.println(F("         READY"));
    Serial.println(F("=============================="));
    Serial.println(F("CONTROLS:"));
    Serial.println(F("  D2 = ARM switch"));
    Serial.println(F("  D4 = Short:MotorTest Long:LevelCal"));
    Serial.println(F("  D5 = Motor Test"));
    Serial.println(F("  D6 = Beeper (hold)"));
    Serial.println(F("  D7 = Headless toggle"));
    Serial.println(F("  A6 = Rate pot"));
    Serial.println(F("  A7 = Expo pot"));
    Serial.println(F("==============================\n"));
    
    timeTx = timeInput = millis();
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
        readInputs();
    }
    
    // Transmit (50Hz)
    if (now - timeTx >= TX_RATE_MS) {
        timeTx = now;
        sendPacket();
    }
    
    // Debug (4Hz - faster for testing)
    if (now - timeDebug >= DEBUG_RATE_MS) {
        timeDebug = now;
        printDebug();
    }
}
