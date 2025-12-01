/*
 * ========================================
 * QUADCOPTER REMOTE CONTROLLER
 * ========================================
 * 
 * Hardware: Arduino Nano + nRF24L01+ + Joysticks + Buttons
 * 
 * Features:
 * - Dual joystick control (4 axes)
 * - 4 function buttons
 * - 2 toggle switches
 * - Wireless transmission via nRF24L01+
 * - Real-time status feedback
 * - Low latency (50Hz update rate)
 * 
 * Controls:
 * - Left Stick Y:  Throttle (altitude)
 * - Left Stick X:  Yaw (rotation)
 * - Right Stick Y: Pitch (forward/back)
 * - Right Stick X: Roll (left/right)
 * 
 * Buttons:
 * - Button 1 (D4): Calibrate sensors
 * - Button 2 (D5): Motor test mode
 * - Button 3 (D6): ARM/DISARM
 * - Button 4 (D7): Soft landing
 * 
 * Switches:
 * - Switch 1 (D2): Reserved for future use
 * - Switch 2 (D3): ANGLE/ACRO mode
 * 
 * Author: DIY Quadcopter Project
 * Version: 1.0
 */

// ========================================
// LIBRARY INCLUDES
// ========================================
#include <SPI.h>
#include <RF24.h>

// ========================================
// PIN DEFINITIONS
// ========================================

// Joystick analog inputs
#define THROTTLE_PIN  A0    // Left stick vertical
#define YAW_PIN       A1    // Left stick horizontal
#define PITCH_PIN     A2    // Right stick vertical
#define ROLL_PIN      A3    // Right stick horizontal

// Button inputs (with internal pull-up)
#define BTN_CALIBRATE  4    // Button 1
#define BTN_MOTORTEST  5    // Button 2
#define BTN_ARM        6    // Button 3
#define BTN_LAND       7    // Button 4

// Switch inputs (with internal pull-up)
#define SW_RESERVED    2    // Switch 1
#define SW_MODE        3    // Switch 2 (ANGLE/ACRO)

// Radio pins
#define CE_PIN         9
#define CSN_PIN        10

// Status LED (optional - built-in LED on pin 13)
#define LED_PIN        13

// ========================================
// HARDWARE OBJECTS
// ========================================
RF24 radio(CE_PIN, CSN_PIN);

// ========================================
// RADIO CONFIGURATION
// ========================================
const byte address[6] = "00001"; // Must match flight controller

// Data structure for transmitting commands (must match receiver)
struct RadioData {
  int throttle;     // 0-1023 (from joystick)
  int yaw;          // 0-1023
  int pitch;        // 0-1023
  int roll;         // 0-1023
  bool armed;       // Armed state
  bool calibrate;   // Calibration command
  bool motorTest;   // Motor test mode
  bool softLand;    // Auto-land command
  bool angleMode;   // true=ANGLE, false=ACRO
};

RadioData txData;

// ========================================
// JOYSTICK CALIBRATION
// ========================================
// Center positions (will be calibrated at startup)
int throttleCenter = 512;
int yawCenter = 512;
int pitchCenter = 512;
int rollCenter = 512;

// Deadband to prevent drift (±20)
const int DEADBAND = 20;

// ========================================
// BUTTON STATE TRACKING
// ========================================
// Previous button states for edge detection
bool btnCalibrateLastState = HIGH;
bool btnMotorTestLastState = HIGH;
bool btnArmLastState = HIGH;
bool btnLandLastState = HIGH;

// Debounce timing
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50ms debounce

// ========================================
// FLIGHT STATE
// ========================================
bool armed = false;
bool angleMode = true; // Default to ANGLE mode (safer)

// ========================================
// TIMING VARIABLES
// ========================================
unsigned long lastTransmitTime = 0;
const unsigned long TRANSMIT_INTERVAL = 20; // 20ms = 50Hz

unsigned long lastStatusTime = 0;
const unsigned long STATUS_INTERVAL = 500; // 500ms status update

// ========================================
// SETUP FUNCTION
// ========================================
void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println(F("=== Remote Controller Starting ==="));
  
  // Initialize pins
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize buttons with internal pull-up resistors
  pinMode(BTN_CALIBRATE, INPUT_PULLUP);
  pinMode(BTN_MOTORTEST, INPUT_PULLUP);
  pinMode(BTN_ARM, INPUT_PULLUP);
  pinMode(BTN_LAND, INPUT_PULLUP);
  
  // Initialize switches with internal pull-up resistors
  pinMode(SW_RESERVED, INPUT_PULLUP);
  pinMode(SW_MODE, INPUT_PULLUP);
  
  // Startup LED sequence
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize radio
  Serial.println(F("Initializing nRF24L01+..."));
  if (!radio.begin()) {
    Serial.println(F("Radio hardware not responding!"));
    while (1) {
      // Stuck in error state
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  Serial.println(F("Radio initialized!"));
  
  // Optimal radio configuration for reliability
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);       // Maximum power for range
  radio.setDataRate(RF24_250KBPS);     // Slowest = most reliable
  radio.setChannel(108);               // Same channel as receiver
  radio.setRetries(15, 15);            // Max retries (15x250μs delay, 15 retries)
  radio.setCRCLength(RF24_CRC_16);     // 16-bit CRC for error detection
  radio.setAutoAck(true);              // Enable auto-acknowledgment
  radio.stopListening();               // Transmitter mode
  
  Serial.println(F("Radio configuration optimized!"));
  Serial.println(F("If transmission fails:"));
  Serial.println(F("  1. Check 10uF capacitor on nRF24"));
  Serial.println(F("  2. Reduce distance"));
  Serial.println(F("  3. Check 3.3V power supply"));
  
  // Calibrate joystick center positions
  calibrateJoysticks();
  
  // Initialize transmit data
  txData.throttle = 0;
  txData.yaw = 512;
  txData.pitch = 512;
  txData.roll = 512;
  txData.armed = false;
  txData.calibrate = false;
  txData.motorTest = false;
  txData.softLand = false;
  txData.angleMode = true;
  
  Serial.println(F("=== Remote Controller Ready ==="));
  Serial.println(F("Controls:"));
  Serial.println(F("  Left Stick Y:  Throttle"));
  Serial.println(F("  Left Stick X:  Yaw"));
  Serial.println(F("  Right Stick Y: Pitch"));
  Serial.println(F("  Right Stick X: Roll"));
  Serial.println(F(""));
  Serial.println(F("Buttons:"));
  Serial.println(F("  Button 1: Calibrate Sensors"));
  Serial.println(F("  Button 2: Motor Test"));
  Serial.println(F("  Button 3: ARM/DISARM"));
  Serial.println(F("  Button 4: Soft Landing"));
  Serial.println(F(""));
  Serial.println(F("Switch 2: ANGLE/ACRO Mode"));
  Serial.println(F(""));
  
  digitalWrite(LED_PIN, HIGH); // Ready
}

// ========================================
// MAIN LOOP
// ========================================
void loop() {
  // Read all inputs
  readJoysticks();
  readButtons();
  readSwitches();
  
  // Transmit data at regular intervals
  if (millis() - lastTransmitTime >= TRANSMIT_INTERVAL) {
    transmitData();
    lastTransmitTime = millis();
  }
  
  // Status updates
  if (millis() - lastStatusTime >= STATUS_INTERVAL) {
    printStatus();
    lastStatusTime = millis();
  }
  
  // Update LED (blink when armed)
  if (armed) {
    digitalWrite(LED_PIN, (millis() / 500) % 2); // Slow blink
  } else {
    digitalWrite(LED_PIN, HIGH); // Solid when disarmed
  }
}

// ========================================
// JOYSTICK CALIBRATION
// ========================================
void calibrateJoysticks() {
  Serial.println(F("Calibrating joysticks..."));
  Serial.println(F("Center all sticks!"));
  
  delay(2000); // Give user time to center sticks
  
  // Read center positions
  throttleCenter = analogRead(THROTTLE_PIN);
  yawCenter = analogRead(YAW_PIN);
  pitchCenter = analogRead(PITCH_PIN);
  rollCenter = analogRead(ROLL_PIN);
  
  Serial.println(F("Joystick centers:"));
  Serial.print(F("  Throttle: ")); Serial.println(throttleCenter);
  Serial.print(F("  Yaw: ")); Serial.println(yawCenter);
  Serial.print(F("  Pitch: ")); Serial.println(pitchCenter);
  Serial.print(F("  Roll: ")); Serial.println(rollCenter);
  Serial.println(F("Calibration complete!"));
}

// ========================================
// INPUT READING
// ========================================
void readJoysticks() {
  // Read raw values (0-1023)
  int throttleRaw = analogRead(THROTTLE_PIN);
  int yawRaw = analogRead(YAW_PIN);
  int pitchRaw = analogRead(PITCH_PIN);
  int rollRaw = analogRead(ROLL_PIN);
  
  // Apply deadband to yaw, pitch, roll (throttle doesn't need deadband)
  yawRaw = applyDeadband(yawRaw, yawCenter, DEADBAND);
  pitchRaw = applyDeadband(pitchRaw, pitchCenter, DEADBAND);
  rollRaw = applyDeadband(rollRaw, rollCenter, DEADBAND);
  
  // Store in transmit structure
  txData.throttle = constrain(throttleRaw, 0, 1023);
  txData.yaw = constrain(yawRaw, 0, 1023);
  txData.pitch = constrain(pitchRaw, 0, 1023);
  txData.roll = constrain(rollRaw, 0, 1023);
}

int applyDeadband(int value, int center, int deadband) {
  // If value is within deadband of center, return center
  if (abs(value - center) < deadband) {
    return center;
  }
  return value;
}

void readButtons() {
  // Read current button states (LOW = pressed due to pull-up)
  bool btnCalibrate = !digitalRead(BTN_CALIBRATE);
  bool btnMotorTest = !digitalRead(BTN_MOTORTEST);
  bool btnArm = !digitalRead(BTN_ARM);
  bool btnLand = !digitalRead(BTN_LAND);
  
  // Button 1: Calibrate (one-shot command)
  if (btnCalibrate && !btnCalibrateLastState) {
    // Rising edge detected
    Serial.println(F("CALIBRATE pressed"));
    txData.calibrate = true;
  } else {
    txData.calibrate = false;
  }
  btnCalibrateLastState = btnCalibrate;
  
  // Button 2: Motor Test (one-shot command)
  if (btnMotorTest && !btnMotorTestLastState) {
    Serial.println(F("MOTOR TEST pressed"));
    txData.motorTest = true;
  } else {
    txData.motorTest = false;
  }
  btnMotorTestLastState = btnMotorTest;
  
  // Button 3: ARM/DISARM (toggle)
  if (btnArm && !btnArmLastState) {
    armed = !armed; // Toggle
    txData.armed = armed;
    
    if (armed) {
      Serial.println(F("ARMING..."));
    } else {
      Serial.println(F("DISARMING..."));
    }
  }
  btnArmLastState = btnArm;
  
  // Button 4: Soft Landing (one-shot command)
  if (btnLand && !btnLandLastState) {
    Serial.println(F("SOFT LANDING pressed"));
    txData.softLand = true;
  } else {
    txData.softLand = false;
  }
  btnLandLastState = btnLand;
}

void readSwitches() {
  // Switch 2: ANGLE/ACRO mode
  // HIGH (up) = ANGLE mode, LOW (down) = ACRO mode
  angleMode = digitalRead(SW_MODE); // HIGH = true (ANGLE), LOW = false (ACRO)
  txData.angleMode = angleMode;
  
  // Switch 1: Reserved for future features
  // bool sw1 = digitalRead(SW_RESERVED);
}

// ========================================
// DATA TRANSMISSION
// ========================================
void transmitData() {
  // Send data packet
  bool success = radio.write(&txData, sizeof(RadioData));
  
  // Track transmission success/failure
  static unsigned long failCount = 0;
  static unsigned long successCount = 0;
  static unsigned long lastFailReport = 0;
  
  if (success) {
    successCount++;
    // Reset fail count on success
    if (failCount > 0) {
      failCount = 0; // Connection restored
    }
  } else {
    failCount++;
    
    // Report transmission failures (but not too frequently)
    if (millis() - lastFailReport > 5000) { // Every 5 seconds
      Serial.print(F("⚠️  Transmission issues: "));
      Serial.print(failCount);
      Serial.println(F(" recent fails"));
      Serial.println(F("Tips: 1) Check capacitor, 2) Reduce distance, 3) Remove obstacles"));
      lastFailReport = millis();
    }
  }
  
  // Report statistics periodically
  static unsigned long lastStatsReport = 0;
  if (millis() - lastStatsReport > 30000) { // Every 30 seconds
    unsigned long total = successCount + failCount;
    if (total > 0) {
      float successRate = (successCount * 100.0) / total;
      Serial.print(F("📡 Link quality: "));
      Serial.print(successRate, 1);
      Serial.print(F("% ("));
      Serial.print(successCount);
      Serial.print(F(" success, "));
      Serial.print(failCount);
      Serial.println(F(" fails)"));
      
      if (successRate < 90.0) {
        Serial.println(F("⚠️  Poor link quality! Check:"));
        Serial.println(F("   - 10uF capacitor on nRF24"));
        Serial.println(F("   - 3.3V power stable"));
        Serial.println(F("   - Distance < 50m"));
        Serial.println(F("   - No metal obstacles"));
      }
    }
    lastStatsReport = millis();
  }
  
  // Reset one-shot commands after transmission
  if (txData.calibrate) {
    txData.calibrate = false;
  }
  if (txData.motorTest) {
    txData.motorTest = false;
  }
  if (txData.softLand) {
    txData.softLand = false;
  }
}

// ========================================
// STATUS DISPLAY
// ========================================
void printStatus() {
  Serial.println(F("=== STATUS ==="));
  
  // Joystick values
  Serial.print(F("Throttle: "));
  Serial.print(txData.throttle);
  Serial.print(F(" | Yaw: "));
  Serial.print(txData.yaw);
  Serial.print(F(" | Pitch: "));
  Serial.print(txData.pitch);
  Serial.print(F(" | Roll: "));
  Serial.println(txData.roll);
  
  // Flight state
  Serial.print(F("Armed: "));
  Serial.print(armed ? "YES" : "NO");
  Serial.print(F(" | Mode: "));
  Serial.println(angleMode ? "ANGLE" : "ACRO");
  
  Serial.println(F(""));
}

// ========================================
// HELPER FUNCTIONS
// ========================================

// Optional: Add battery voltage monitoring
float readBatteryVoltage() {
  // If you add a voltage divider to A6 or A7
  // int reading = analogRead(A6);
  // float voltage = reading * (5.0 / 1023.0) * VOLTAGE_DIVIDER_RATIO;
  // return voltage;
  return 0.0; // Placeholder
}

// ========================================
// END OF REMOTE CONTROLLER CODE
// ========================================
