/*
 * Drone Remote Controller
 * =======================
 * Arduino Nano based RC transmitter for quadcopter
 * 
 * Hardware:
 * - Arduino Nano
 * - NRF24L01 Radio Module
 * - 2x Dual-axis Joysticks
 * - 2x Push Buttons
 * - 2x Toggle Switches
 * 
 * Pin Configuration:
 * - D2:  Altitude-Hold Switch
 * - D3:  Arm/Disarm Switch
 * - D4:  Calibration Button
 * - D5:  Motor Start Button
 * - D9:  NRF24L01 CE
 * - D10: NRF24L01 CSN
 * 
 * Analog Pins:
 * - A0:  Throttle (Left Y) - Up = increase, Down = decrease
 * - A1:  Yaw (Left X) - Left = CCW, Right = CW
 * - A2:  Pitch (Right Y) - Up = forward, Down = backward
 * - A3:  Roll (Right X) - Left = left, Right = right
 * 
 * Joystick Layout:
 * +---------------+    +---------------+
 * |    THROTTLE   |    |     PITCH     |
 * |       ↑       |    |       ↑       |
 * |   ← YAW →     |    |   ← ROLL →    |
 * |       ↓       |    |       ↓       |
 * |  LEFT STICK   |    |  RIGHT STICK  |
 * +---------------+    +---------------+
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

// ==================== PIN DEFINITIONS ====================
// NRF24L01 Pins
const int PIN_NRF_CE = 9;
const int PIN_NRF_CSN = 10;

// Button/Switch Pins
const int PIN_ALT_HOLD_SW = 2;     // Altitude Hold Switch
const int PIN_ARM_SW = 3;          // Arm/Disarm Switch
const int PIN_CALIB_BTN = 4;       // Calibration Button
const int PIN_MOTOR_START_BTN = 5; // Motor Start Button

// Joystick Analog Pins
const int PIN_THROTTLE = A0;  // Left Y - Throttle
const int PIN_YAW = A1;       // Left X - Yaw
const int PIN_PITCH = A2;     // Right Y - Pitch
const int PIN_ROLL = A3;      // Right X - Roll

// ==================== RADIO SETUP ====================
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const uint64_t PIPE_ADDRESS = 0xF0F0F0F0E1LL;
const uint8_t NRF_CHANNEL = 108;  // Match FC channel

// ==================== JOYSTICK CALIBRATION ====================
// Scale factors for each axis
float scaleThrottle = 1.5;
float scaleYaw = -0.1;
float scalePitch = -0.1;
float scaleRoll = 0.1;

// Center calibration offsets (adjust for your joysticks)
// These values center the joystick at 512 (mid-point of 0-1023)
int calThrottle = -500;  // Throttle offset
int calYaw = -512;       // Yaw center offset
int calPitch = -507;     // Pitch center offset
int calRoll = -527;      // Roll center offset

// Output offsets
int offsetThrottle = 1300;  // Base throttle value
int offsetYaw = 0;
int offsetPitch = 0;
int offsetRoll = 0;

// ==================== SMOOTHING FILTERS ====================
Smoothed<float> smoothThrottle;
Smoothed<float> smoothYaw;
Smoothed<float> smoothPitch;
Smoothed<float> smoothRoll;

// ==================== DATA STRUCTURE ====================
struct ControlPackage {
    int thrust = 0;
    float x = 0;      // Roll
    float y = 0;      // Pitch
    float z = 0;      // Yaw
    int id = 0;
    bool but1 = 1;    // Calibration button (1 = not pressed)
    bool but2 = 1;    // Motor start button (1 = not pressed)
    bool switch1 = 1; // Arm switch (1 = disarmed)
    bool switch2 = 1; // Altitude hold switch (1 = off)
};

ControlPackage package;

// ==================== STATE VARIABLES ====================
int rawThrottle, rawYaw, rawPitch, rawRoll;
float filteredThrottle, filteredYaw, filteredPitch, filteredRoll;
int packetID = 0;

// ==================== TIMING ====================
unsigned long lastTransmit = 0;
const unsigned long TRANSMIT_INTERVAL = 10;  // ~100 Hz

// ==================== FUNCTION PROTOTYPES ====================
void readJoysticks();
void readButtons();
void buildPackage();
void transmitPackage();
void printDebug();

// ==================== SETUP ====================
void setup() {
    Serial.begin(57600);
    
    // Configure input pins with pull-ups
    pinMode(PIN_ALT_HOLD_SW, INPUT_PULLUP);
    pinMode(PIN_ARM_SW, INPUT_PULLUP);
    pinMode(PIN_CALIB_BTN, INPUT_PULLUP);
    pinMode(PIN_MOTOR_START_BTN, INPUT_PULLUP);
    
    // Initialize radio
    radio.begin();
    radio.setChannel(NRF_CHANNEL);
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.openWritingPipe(PIPE_ADDRESS);
    radio.stopListening();
    
    // Initialize smoothing filters
    smoothThrottle.begin(SMOOTHED_EXPONENTIAL, 5);
    smoothYaw.begin(SMOOTHED_EXPONENTIAL, 5);
    smoothPitch.begin(SMOOTHED_EXPONENTIAL, 5);
    smoothRoll.begin(SMOOTHED_EXPONENTIAL, 5);
    
    Serial.println(F("=== Remote Controller Ready ==="));
    Serial.print(F("Radio channel: "));
    Serial.println(NRF_CHANNEL);
}

// ==================== MAIN LOOP ====================
void loop() {
    // Read all inputs
    readJoysticks();
    readButtons();
    
    // Build and transmit package at fixed rate
    if (millis() - lastTransmit >= TRANSMIT_INTERVAL) {
        buildPackage();
        transmitPackage();
        printDebug();
        lastTransmit = millis();
    }
}

// ==================== INPUT FUNCTIONS ====================

void readJoysticks() {
    // Read raw analog values
    // Note: Some axes are inverted (1023 - value) for correct orientation
    
    // Throttle: Up = high value (inverted because joystick may read opposite)
    rawThrottle = max(analogRead(PIN_THROTTLE), 0);
    
    // Yaw: Right = positive, Left = negative
    rawYaw = analogRead(PIN_YAW);
    
    // Pitch: Forward (up) = positive (inverted)
    rawPitch = 1023 - analogRead(PIN_PITCH);
    
    // Roll: Right = positive
    rawRoll = analogRead(PIN_ROLL);
    
    // Apply smoothing
    smoothThrottle.add(rawThrottle);
    smoothYaw.add(rawYaw);
    smoothPitch.add(rawPitch);
    smoothRoll.add(rawRoll);
    
    // Get filtered values
    filteredThrottle = smoothThrottle.get();
    filteredYaw = smoothYaw.get();
    filteredPitch = smoothPitch.get();
    filteredRoll = smoothRoll.get();
}

void readButtons() {
    // Read buttons/switches (active LOW with pull-ups)
    // Invert logic so: pressed = 0, not pressed = 1
    package.but1 = digitalRead(PIN_CALIB_BTN);        // Calibration
    package.but2 = digitalRead(PIN_MOTOR_START_BTN);  // Motor start
    package.switch1 = digitalRead(PIN_ARM_SW);        // Arm (1=disarmed, 0=armed)
    package.switch2 = digitalRead(PIN_ALT_HOLD_SW);   // Alt hold (1=off, 0=on)
}

void buildPackage() {
    // Calculate scaled values
    // Thrust: Direct value for ESC
    package.thrust = (filteredThrottle + calThrottle) * scaleThrottle + offsetThrottle;
    package.thrust = constrain(package.thrust, 1000, 2000);
    
    // Roll (X): Center at 0, positive = right
    package.x = (filteredRoll + calRoll) * scaleRoll + offsetRoll;
    
    // Pitch (Y): Center at 0, positive = forward
    package.y = (filteredPitch + calPitch) * scalePitch + offsetPitch;
    
    // Yaw (Z): Center at 0, positive = clockwise
    package.z = (filteredYaw + calYaw) * scaleYaw + offsetYaw;
    
    // Packet ID for tracking
    package.id = packetID++;
}

void transmitPackage() {
    radio.write(&package, sizeof(package));
}

// ==================== DEBUG FUNCTIONS ====================

void printDebug() {
    Serial.print(F("T:"));
    Serial.print(package.thrust);
    Serial.print(F("\tZ:"));
    Serial.print(package.z, 1);
    Serial.print(F("\tX:"));
    Serial.print(package.x, 1);
    Serial.print(F("\tY:"));
    Serial.print(package.y, 1);
    Serial.print(F("\tArm:"));
    Serial.print(package.switch1);
    Serial.print(F("\tAlt:"));
    Serial.print(package.switch2);
    Serial.print(F("\tBut1:"));
    Serial.print(package.but1);
    Serial.print(F("\tBut2:"));
    Serial.println(package.but2);
}

// ==================== CALIBRATION HELPER ====================
/*
 * To calibrate your joysticks:
 * 
 * 1. Center all sticks and read the raw values
 * 2. Set cal values to: -rawValue (to center at 0)
 * 
 * For Throttle:
 * - Move stick to minimum and note the value
 * - Set calThrottle so minimum maps to ~1000
 * - Set scaleThrottle so maximum maps to ~2000
 * 
 * Example calibration procedure:
 * void calibrateJoysticks() {
 *     Serial.println("Center all sticks and press a key...");
 *     while (!Serial.available());
 *     Serial.read();
 *     
 *     int centerYaw = analogRead(PIN_YAW);
 *     int centerPitch = analogRead(PIN_PITCH);
 *     int centerRoll = analogRead(PIN_ROLL);
 *     
 *     Serial.print("calYaw = "); Serial.println(-centerYaw);
 *     Serial.print("calPitch = "); Serial.println(-centerPitch);
 *     Serial.print("calRoll = "); Serial.println(-centerRoll);
 * }
 */
