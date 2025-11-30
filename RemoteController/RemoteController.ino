/*
 * Professional Quadcopter Remote Controller
 * 
 * Hardware Configuration:
 * - Arduino Nano
 * - nRF24L01+ PA+LNA (CE:D9, CSN:D10)
 * - Left Joystick: V:A0, H:A1
 * - Right Joystick: V:A2, H:A3
 * - Buttons: B1:D4, B2:D5, B3:D6, B4:D7
 * - Toggle Switches: SW1:D2, SW2:D3
 * 
 * Author: Professional Quadcopter System
 * Version: 2.0
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ============================================
// PIN DEFINITIONS
// ============================================
#define CE_PIN 9
#define CSN_PIN 10

// Joystick pins
#define JOY_LV A0  // Left Vertical (Throttle)
#define JOY_LH A1  // Left Horizontal (Yaw)
#define JOY_RV A2  // Right Vertical (Pitch)
#define JOY_RH A3  // Right Horizontal (Roll)

// Button pins
#define BTN_1 4
#define BTN_2 5
#define BTN_3 6
#define BTN_4 7

// Toggle switch pins
#define SW_1 2  // Arm/Disarm
#define SW_2 3  // Flight mode

// ============================================
// COMMUNICATION STRUCTURE
// ============================================
struct ControlData {
  int16_t throttle;     // 1000-2000
  int16_t roll;         // -500 to +500
  int16_t pitch;        // -500 to +500
  int16_t yaw;          // -500 to +500
  uint8_t switches;     // Bit field for switches
  uint8_t buttons;      // Bit field for buttons
  uint8_t checksum;     // Data integrity
};

struct TelemetryData {
  float roll;
  float pitch;
  float yaw;
  uint8_t battery;
  uint8_t flightMode;
  uint8_t armed;
};

// ============================================
// GLOBAL VARIABLES
// ============================================
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipeOut = 0xE8E8F0F0E1LL;
const uint64_t pipeIn = 0xE8E8F0F0E2LL;

ControlData txData;
TelemetryData telemetry;

// Joystick calibration
int joyLVCenter = 512, joyLHCenter = 512;
int joyRVCenter = 512, joyRHCenter = 512;

// Joystick deadband
const int DEADBAND = 10;

// Timing
unsigned long lastTransmitTime = 0;
const unsigned long TRANSMIT_INTERVAL = 20;  // 50Hz

// Connection status
bool connected = false;
unsigned long lastAckTime = 0;
const unsigned long CONNECTION_TIMEOUT = 1000;

// ============================================
// SETUP
// ============================================
void setup() {
  Serial.begin(115200);
  
  Serial.println(F("================================="));
  Serial.println(F("Quadcopter Remote Controller v2.0"));
  Serial.println(F("================================="));
  
  // Initialize input pins
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
  pinMode(BTN_3, INPUT_PULLUP);
  pinMode(BTN_4, INPUT_PULLUP);
  pinMode(SW_1, INPUT_PULLUP);
  pinMode(SW_2, INPUT_PULLUP);
  
  // Initialize radio
  if (!initRadio()) {
    Serial.println(F("[ERROR] Radio initialization failed!"));
    while (1) {
      delay(1000);
    }
  }
  
  // Calibrate joysticks
  calibrateJoysticks();
  
  Serial.println(F("System Ready!"));
  Serial.println(F(""));
  Serial.println(F("=== CONTROLS ==="));
  Serial.println(F("Left Stick:"));
  Serial.println(F("  Vertical   -> Throttle (altitude)"));
  Serial.println(F("  Horizontal -> Yaw (rotate left/right)"));
  Serial.println(F("Right Stick:"));
  Serial.println(F("  Vertical   -> Pitch (forward/backward)"));
  Serial.println(F("  Horizontal -> Roll (left/right)"));
  Serial.println(F(""));
  Serial.println(F("Switches:"));
  Serial.println(F("  SW1 -> ARM/DISARM"));
  Serial.println(F("  SW2 -> Flight Mode (ANGLE/ACRO)"));
  Serial.println(F(""));
  Serial.println(F("Buttons:"));
  Serial.println(F("  BTN1 -> Calibration (recalibrate gyro)"));
  Serial.println(F("  BTN2 -> Motor Test (hold to spin motors)"));
  Serial.println(F("  BTN3 -> Buzzer Beep (find drone)"));
  Serial.println(F("  BTN4 -> [Reserved]"));
  Serial.println(F(""));
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
  if (millis() - lastTransmitTime >= TRANSMIT_INTERVAL) {
    // Read all inputs
    readJoysticks();
    readSwitches();
    readButtons();
    
    // Calculate checksum
    txData.checksum = (txData.throttle + txData.roll + txData.pitch + txData.yaw) & 0xFF;
    
    // Transmit data
    transmitData();
    
    // Print status
    printStatus();
    
    lastTransmitTime = millis();
  }
}

// ============================================
// RADIO INITIALIZATION
// ============================================
bool initRadio() {
  if (!radio.begin()) {
    Serial.println(F("[ERROR] Radio hardware not responding!"));
    return false;
  }
  
  radio.setChannel(108);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  
  radio.openWritingPipe(pipeOut);
  radio.openReadingPipe(1, pipeIn);
  radio.stopListening();
  
  Serial.println(F("[OK] Radio initialized"));
  return true;
}

// ============================================
// CALIBRATE JOYSTICKS
// ============================================
void calibrateJoysticks() {
  Serial.println(F("Calibrating joysticks..."));
  Serial.println(F("Center all sticks!"));
  delay(2000);
  
  int samples = 100;
  long lvSum = 0, lhSum = 0, rvSum = 0, rhSum = 0;
  
  for (int i = 0; i < samples; i++) {
    lvSum += analogRead(JOY_LV);
    lhSum += analogRead(JOY_LH);
    rvSum += analogRead(JOY_RV);
    rhSum += analogRead(JOY_RH);
    delay(10);
  }
  
  joyLVCenter = lvSum / samples;
  joyLHCenter = lhSum / samples;
  joyRVCenter = rvSum / samples;
  joyRHCenter = rhSum / samples;
  
  Serial.println(F("[OK] Calibration complete!"));
  Serial.print(F("Centers: LV="));
  Serial.print(joyLVCenter);
  Serial.print(F(" LH="));
  Serial.print(joyLHCenter);
  Serial.print(F(" RV="));
  Serial.print(joyRVCenter);
  Serial.print(F(" RH="));
  Serial.println(joyRHCenter);
  Serial.println();
}

// ============================================
// READ JOYSTICKS
// ============================================
void readJoysticks() {
  // Read raw values
  int rawLV = analogRead(JOY_LV);
  int rawLH = analogRead(JOY_LH);
  int rawRV = analogRead(JOY_RV);
  int rawRH = analogRead(JOY_RH);
  
  // Left Vertical -> Throttle (1000-2000)
  int lvOffset = rawLV - joyLVCenter;
  if (abs(lvOffset) < DEADBAND) lvOffset = 0;
  txData.throttle = map(rawLV, 0, 1023, 1000, 2000);
  txData.throttle = constrain(txData.throttle, 1000, 2000);
  
  // Left Horizontal -> Yaw (-500 to +500)
  int lhOffset = rawLH - joyLHCenter;
  if (abs(lhOffset) < DEADBAND) lhOffset = 0;
  txData.yaw = map(lhOffset, -512, 512, -500, 500);
  txData.yaw = constrain(txData.yaw, -500, 500);
  
  // Right Vertical -> Pitch (-500 to +500)
  int rvOffset = rawRV - joyRVCenter;
  if (abs(rvOffset) < DEADBAND) rvOffset = 0;
  txData.pitch = map(rvOffset, -512, 512, -500, 500);
  txData.pitch = constrain(txData.pitch, -500, 500);
  
  // Right Horizontal -> Roll (-500 to +500)
  int rhOffset = rawRH - joyRHCenter;
  if (abs(rhOffset) < DEADBAND) rhOffset = 0;
  txData.roll = map(rhOffset, -512, 512, -500, 500);
  txData.roll = constrain(txData.roll, -500, 500);
}

// ============================================
// READ SWITCHES
// ============================================
void readSwitches() {
  txData.switches = 0;
  
  // SW1: Arm/Disarm (bit 0)
  if (digitalRead(SW_1) == LOW) {
    txData.switches |= 0x01;
  }
  
  // SW2: Flight mode (bit 1)
  if (digitalRead(SW_2) == LOW) {
    txData.switches |= 0x02;
  }
}

// ============================================
// READ BUTTONS
// ============================================
void readButtons() {
  txData.buttons = 0;
  
  // Button 1 (bit 0)
  if (digitalRead(BTN_1) == LOW) {
    txData.buttons |= 0x01;
  }
  
  // Button 2 (bit 1)
  if (digitalRead(BTN_2) == LOW) {
    txData.buttons |= 0x02;
  }
  
  // Button 3 (bit 2)
  if (digitalRead(BTN_3) == LOW) {
    txData.buttons |= 0x04;
  }
  
  // Button 4 (bit 3)
  if (digitalRead(BTN_4) == LOW) {
    txData.buttons |= 0x08;
  }
}

// ============================================
// TRANSMIT DATA
// ============================================
void transmitData() {
  bool success = radio.write(&txData, sizeof(ControlData));
  
  if (success) {
    if (!connected) {
      Serial.println(F("[CONNECTED]"));
    }
    connected = true;
    lastAckTime = millis();
  } else {
    if (millis() - lastAckTime > CONNECTION_TIMEOUT) {
      if (connected) {
        Serial.println(F("[DISCONNECTED]"));
      }
      connected = false;
    }
  }
}

// ============================================
// PRINT STATUS
// ============================================
void printStatus() {
  static unsigned long lastPrintTime = 0;
  
  if (millis() - lastPrintTime > 200) {  // Print every 200ms
    // Joystick values
    Serial.print(F("T:"));
    Serial.print(txData.throttle);
    Serial.print(F(" R:"));
    Serial.print(txData.roll);
    Serial.print(F(" P:"));
    Serial.print(txData.pitch);
    Serial.print(F(" Y:"));
    Serial.print(txData.yaw);
    
    // Switches
    Serial.print(F(" | SW1:"));
    Serial.print((txData.switches & 0x01) ? F("ARM") : F("SAFE"));
    Serial.print(F(" SW2:"));
    Serial.print((txData.switches & 0x02) ? F("ANGLE") : F("ACRO"));
    
    // Buttons (show which are pressed)
    Serial.print(F(" | BTN:"));
    if (txData.buttons & 0x01) Serial.print(F("CAL "));
    if (txData.buttons & 0x02) Serial.print(F("TEST "));
    if (txData.buttons & 0x04) Serial.print(F("BUZZ "));
    if (txData.buttons & 0x08) Serial.print(F("4 "));
    if (txData.buttons == 0) Serial.print(F("- "));
    
    // Connection status
    Serial.print(F("| "));
    if (connected) {
      Serial.print(F("✓CONN"));
    } else {
      Serial.print(F("✗NO_SIG"));
    }
    
    Serial.println();
    lastPrintTime = millis();
  }
}
