/*
 * ============================================================================
 * PROFESSIONAL QUADCOPTER REMOTE CONTROLLER
 * ============================================================================
 * 
 * Hardware: Arduino Nano
 * Author: Professional UAV Embedded Systems
 * Version: 1.0.0
 * 
 * DESCRIPTION:
 * Advanced remote controller with dual joysticks, wireless telemetry,
 * and comprehensive real-time monitoring for quadcopter control.
 * 
 * PIN CONFIGURATION:
 * - NRF24L01: CE=D9, CSN=D10, MOSI=D11, MISO=D12, SCK=D13
 * - Left Joystick: Vertical=A0 (Throttle), Horizontal=A1 (Yaw)
 * - Right Joystick: Vertical=A2 (Pitch), Horizontal=A3 (Roll)
 * - Button 1: D4 (Calibration)
 * - Button 2: D5 (Motor Arm)
 * - Switch 1: D2 (Altitude Hold)
 * - Switch 2: D3 (Kill Switch - Arming)
 * 
 * ============================================================================
 */

#include <SPI.h>
#include <RF24.h>

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================
// NRF24L01 Wireless Module
#define NRF_CE_PIN          9
#define NRF_CSN_PIN         10

// Joysticks (Analog)
#define THROTTLE_PIN        A0  // Left Vertical
#define YAW_PIN             A1  // Left Horizontal
#define PITCH_PIN           A2  // Right Vertical
#define ROLL_PIN            A3  // Right Horizontal

// Buttons (Active LOW with internal pullup)
#define BUTTON_CALIBRATE    4   // Button 1
#define BUTTON_MOTOR_ARM    5   // Button 2

// Switches (Toggle switches to GND)
#define SWITCH_ALT_HOLD     2   // Switch 1
#define SWITCH_KILL         3   // Switch 2 (Arming)

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================
// NRF24L01 Settings
#define NRF_CHANNEL         103
#define NRF_PAYLOAD_SIZE    32

// Joystick Calibration
#define STICK_MIN           0
#define STICK_MAX           1023
#define STICK_CENTER        512
#define STICK_DEADBAND      20

// Output ranges
#define OUTPUT_MIN          1000
#define OUTPUT_MAX          2000
#define OUTPUT_CENTER       1500

// Communication
#define TX_INTERVAL_MS      20          // 50Hz transmission rate
#define TELEMETRY_TIMEOUT   500         // ms

// ============================================================================
// DATA STRUCTURES
// ============================================================================
// Control data sent to FC
struct ControlData {
  uint16_t throttle;      // 1000-2000
  uint16_t yaw;           // 1000-2000
  uint16_t pitch;         // 1000-2000
  uint16_t roll;          // 1000-2000
  
  bool calibrate;         // Calibration trigger
  bool motorArm;          // Motor arming
  bool altitudeHold;      // Altitude hold mode
  bool killSwitch;        // Emergency disarm
  
  uint8_t checksum;       // Data integrity
};

// Telemetry received from FC
struct TelemetryData {
  float batteryVoltage;
  int16_t gyroX, gyroY, gyroZ;
  int16_t accelX, accelY, accelZ;
  float roll, pitch, yaw;
  bool armed;
  bool calibrated;
  uint8_t checksum;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// NRF24L01 Addresses
const byte txAddress[6] = "DRONE";
const byte rxAddress[6] = "REMOT";

// Control and telemetry
ControlData controlData;
TelemetryData telemetry;

// Input states
uint16_t rawThrottle, rawYaw, rawPitch, rawRoll;
bool buttonCalibrate, buttonMotorArm;
bool switchAltHold, switchKill;

// Previous button states (for edge detection)
bool lastButtonCalibrate = false;
bool lastButtonMotorArm = false;

// Communication status
bool linkActive = false;
unsigned long lastTelemetryTime = 0;
unsigned long lastTxTime = 0;

// Statistics
unsigned long txCount = 0;
unsigned long rxCount = 0;
unsigned long txFailCount = 0;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void setupNRF24();
void readInputs();
uint16_t applyDeadband(uint16_t value, uint16_t center, uint8_t deadband);
uint16_t mapJoystick(uint16_t value, bool isCentered);
void buildControlPacket();
void transmitData();
void receiveTelemetry();
void displayStatus();
void displayTelemetry();
uint8_t calculateChecksum(uint8_t* data, uint8_t len);

// ============================================================================
// SETUP FUNCTION
// ============================================================================
void setup() {
  // Serial communication
  Serial.begin(115200);
  delay(100);
  
  Serial.println(F(""));
  Serial.println(F("╔════════════════════════════════════════════════════╗"));
  Serial.println(F("║   PROFESSIONAL QUADCOPTER REMOTE CONTROLLER       ║"));
  Serial.println(F("║   Version 1.0.0                                    ║"));
  Serial.println(F("╚════════════════════════════════════════════════════╝"));
  Serial.println(F(""));
  
  // Configure input pins
  pinMode(BUTTON_CALIBRATE, INPUT_PULLUP);
  pinMode(BUTTON_MOTOR_ARM, INPUT_PULLUP);
  pinMode(SWITCH_ALT_HOLD, INPUT_PULLUP);
  pinMode(SWITCH_KILL, INPUT_PULLUP);
  
  // Initialize analog reference
  analogReference(DEFAULT);
  
  // Initialize NRF24L01
  Serial.print(F("[INIT] NRF24L01 Radio... "));
  setupNRF24();
  Serial.println(F("OK"));
  
  // Initial control values (safe defaults)
  controlData.throttle = OUTPUT_MIN;
  controlData.yaw = OUTPUT_CENTER;
  controlData.pitch = OUTPUT_CENTER;
  controlData.roll = OUTPUT_CENTER;
  controlData.calibrate = false;
  controlData.motorArm = false;
  controlData.altitudeHold = false;
  controlData.killSwitch = false;
  
  Serial.println(F("[INIT] System Ready"));
  Serial.println(F(""));
  Serial.println(F("════════════════════════════════════════════════════"));
  Serial.println(F("  CONTROLS:"));
  Serial.println(F("  - Left Stick:  Throttle (V) | Yaw (H)"));
  Serial.println(F("  - Right Stick: Pitch (V)    | Roll (H)"));
  Serial.println(F("  - Button 1 (D4): Gyro Calibration"));
  Serial.println(F("  - Button 2 (D5): Motor Arming"));
  Serial.println(F("  - Switch 1 (D2): Altitude Hold"));
  Serial.println(F("  - Switch 2 (D3): Kill Switch (Arming)"));
  Serial.println(F("════════════════════════════════════════════════════"));
  Serial.println(F(""));
  
  delay(1000);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Read all inputs
  readInputs();
  
  // Build control packet
  buildControlPacket();
  
  // Transmit at fixed interval
  if (millis() - lastTxTime >= TX_INTERVAL_MS) {
    transmitData();
    lastTxTime = millis();
  }
  
  // Check for telemetry
  receiveTelemetry();
  
  // Update link status
  if (millis() - lastTelemetryTime > TELEMETRY_TIMEOUT) {
    linkActive = false;
  }
  
  // Display status on serial monitor (1Hz update)
  static unsigned long displayTimer = 0;
  if (millis() - displayTimer >= 1000) {
    displayStatus();
    displayTimer = millis();
  }
}

// ============================================================================
// NRF24L01 INITIALIZATION
// ============================================================================
void setupNRF24() {
  if (!radio.begin()) {
    Serial.println(F("FAILED!"));
    Serial.println(F("[ERROR] NRF24L01 not detected!"));
    Serial.println(F("[ERROR] Check wiring and power"));
    while (1) {
      delay(1000);
    }
  }
  
  radio.setChannel(NRF_CHANNEL);
  radio.setPALevel(RF24_PA_MAX);          // Maximum power for PA+LNA
  radio.setDataRate(RF24_250KBPS);        // Low rate = better range
  radio.setPayloadSize(NRF_PAYLOAD_SIZE);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setAutoAck(true);
  radio.setRetries(5, 15);                // 5*250μs delay, 15 retries
  
  radio.openWritingPipe(txAddress);
  radio.openReadingPipe(1, rxAddress);
  radio.startListening();
}

// ============================================================================
// READ ALL INPUTS
// ============================================================================
void readInputs() {
  // Read joysticks (analog values)
  rawThrottle = analogRead(THROTTLE_PIN);
  rawYaw = analogRead(YAW_PIN);
  rawPitch = analogRead(PITCH_PIN);
  rawRoll = analogRead(ROLL_PIN);
  
  // Read buttons (active LOW)
  buttonCalibrate = !digitalRead(BUTTON_CALIBRATE);
  buttonMotorArm = !digitalRead(BUTTON_MOTOR_ARM);
  
  // Read switches (active LOW)
  switchAltHold = !digitalRead(SWITCH_ALT_HOLD);
  switchKill = !digitalRead(SWITCH_KILL);
}

// ============================================================================
// APPLY DEADBAND TO CENTERED STICKS
// ============================================================================
uint16_t applyDeadband(uint16_t value, uint16_t center, uint8_t deadband) {
  if (abs(value - center) < deadband) {
    return center;
  }
  return value;
}

// ============================================================================
// MAP JOYSTICK TO OUTPUT RANGE
// ============================================================================
uint16_t mapJoystick(uint16_t value, bool isCentered) {
  if (isCentered) {
    // Centered stick (Yaw, Pitch, Roll)
    value = applyDeadband(value, STICK_CENTER, STICK_DEADBAND);
    return map(value, STICK_MIN, STICK_MAX, OUTPUT_MIN, OUTPUT_MAX);
  } else {
    // Non-centered stick (Throttle)
    return map(value, STICK_MIN, STICK_MAX, OUTPUT_MIN, OUTPUT_MAX);
  }
}

// ============================================================================
// BUILD CONTROL PACKET
// ============================================================================
void buildControlPacket() {
  // Map joystick values
  controlData.throttle = mapJoystick(rawThrottle, false);
  controlData.yaw = mapJoystick(rawYaw, true);
  controlData.pitch = mapJoystick(rawPitch, true);
  controlData.roll = mapJoystick(rawRoll, true);
  
  // Button states (edge detection for calibration)
  if (buttonCalibrate && !lastButtonCalibrate) {
    controlData.calibrate = true;
  } else {
    controlData.calibrate = false;
  }
  lastButtonCalibrate = buttonCalibrate;
  
  // Motor arm (level triggered)
  controlData.motorArm = buttonMotorArm;
  
  // Switch states
  controlData.altitudeHold = switchAltHold;
  controlData.killSwitch = switchKill;
  
  // Calculate checksum
  controlData.checksum = calculateChecksum((uint8_t*)&controlData, sizeof(controlData) - 1);
}

// ============================================================================
// TRANSMIT DATA TO FLIGHT CONTROLLER
// ============================================================================
void transmitData() {
  radio.stopListening();
  
  bool success = radio.write(&controlData, sizeof(controlData));
  
  if (success) {
    txCount++;
    linkActive = true;
  } else {
    txFailCount++;
    linkActive = false;
  }
  
  radio.startListening();
}

// ============================================================================
// RECEIVE TELEMETRY FROM FLIGHT CONTROLLER
// ============================================================================
void receiveTelemetry() {
  if (radio.available()) {
    radio.read(&telemetry, sizeof(telemetry));
    
    // Verify checksum
    uint8_t calcChecksum = calculateChecksum((uint8_t*)&telemetry, sizeof(telemetry) - 1);
    if (calcChecksum == telemetry.checksum) {
      lastTelemetryTime = millis();
      linkActive = true;
      rxCount++;
    }
  }
}

// ============================================================================
// DISPLAY STATUS ON SERIAL MONITOR
// ============================================================================
void displayStatus() {
  // Clear screen (ANSI escape codes)
  Serial.print(F("\033[2J\033[H"));
  
  // Header
  Serial.println(F("╔════════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║           QUADCOPTER REMOTE CONTROLLER - LIVE STATUS              ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════════╝"));
  Serial.println(F(""));
  
  // Communication Status
  Serial.println(F("┌─── COMMUNICATION STATUS ───────────────────────────────────────────┐"));
  Serial.print(F("│ Link Status: "));
  if (linkActive) {
    Serial.println(F("✓ CONNECTED                                              │"));
  } else {
    Serial.println(F("✗ NO SIGNAL                                              │"));
  }
  Serial.print(F("│ Channel: "));
  Serial.print(NRF_CHANNEL);
  Serial.print(F(" | TX Count: "));
  Serial.print(txCount);
  Serial.print(F(" | RX Count: "));
  Serial.print(rxCount);
  Serial.print(F(" | Fails: "));
  Serial.println(txFailCount);
  Serial.print(F("│ Success Rate: "));
  if (txCount > 0) {
    float successRate = 100.0 * (txCount - txFailCount) / txCount;
    Serial.print(successRate, 1);
    Serial.println(F("%                                            │"));
  } else {
    Serial.println(F("N/A                                                     │"));
  }
  Serial.println(F("└────────────────────────────────────────────────────────────────────┘"));
  Serial.println(F(""));
  
  // Control Inputs
  Serial.println(F("┌─── CONTROL INPUTS ─────────────────────────────────────────────────┐"));
  Serial.print(F("│ THROTTLE: "));
  Serial.print(controlData.throttle);
  Serial.print(F(" │ YAW:   "));
  Serial.print(controlData.yaw);
  Serial.print(F(" │ PITCH: "));
  Serial.print(controlData.pitch);
  Serial.print(F(" │ ROLL:  "));
  Serial.println(controlData.roll);
  
  // Progress bars for sticks
  Serial.print(F("│ THR ["));
  int thrBar = map(controlData.throttle, OUTPUT_MIN, OUTPUT_MAX, 0, 20);
  for (int i = 0; i < 20; i++) Serial.print(i < thrBar ? "█" : "░");
  Serial.println(F("]                                   │"));
  
  Serial.print(F("│ YAW ["));
  int yawBar = map(controlData.yaw, OUTPUT_MIN, OUTPUT_MAX, 0, 20);
  for (int i = 0; i < 20; i++) {
    if (i == 10) Serial.print("│");
    else Serial.print(i < yawBar ? (yawBar > 10 ? "█" : "░") : (yawBar < 10 ? "█" : "░"));
  }
  Serial.println(F("]                                   │"));
  
  Serial.print(F("│ PIT ["));
  int pitBar = map(controlData.pitch, OUTPUT_MIN, OUTPUT_MAX, 0, 20);
  for (int i = 0; i < 20; i++) {
    if (i == 10) Serial.print("│");
    else Serial.print(i < pitBar ? (pitBar > 10 ? "█" : "░") : (pitBar < 10 ? "█" : "░"));
  }
  Serial.println(F("]                                   │"));
  
  Serial.print(F("│ ROL ["));
  int rolBar = map(controlData.roll, OUTPUT_MIN, OUTPUT_MAX, 0, 20);
  for (int i = 0; i < 20; i++) {
    if (i == 10) Serial.print("│");
    else Serial.print(i < rolBar ? (rolBar > 10 ? "█" : "░") : (rolBar < 10 ? "█" : "░"));
  }
  Serial.println(F("]                                   │"));
  Serial.println(F("└────────────────────────────────────────────────────────────────────┘"));
  Serial.println(F(""));
  
  // Buttons and Switches
  Serial.println(F("┌─── BUTTONS & SWITCHES ─────────────────────────────────────────────┐"));
  Serial.print(F("│ Calibration Button: "));
  Serial.print(buttonCalibrate ? "[PRESSED]" : "[      ]");
  Serial.print(F("  │  Motor Arm Button: "));
  Serial.println(buttonMotorArm ? "[PRESSED]      │" : "[      ]       │");
  
  Serial.print(F("│ Altitude Hold:      "));
  Serial.print(switchAltHold ? "[ON ]" : "[OFF]");
  Serial.print(F("       │  Kill Switch:      "));
  Serial.println(switchKill ? "[ARMED]        │" : "[DISARMED]     │");
  Serial.println(F("└────────────────────────────────────────────────────────────────────┘"));
  Serial.println(F(""));
  
  // Drone Telemetry
  if (linkActive) {
    Serial.println(F("┌─── DRONE TELEMETRY ────────────────────────────────────────────────┐"));
    Serial.print(F("│ Status: "));
    if (telemetry.armed) {
      Serial.print(F("⚠ ARMED & FLYING"));
    } else if (telemetry.calibrated) {
      Serial.print(F("✓ READY"));
    } else {
      Serial.print(F("⚠ CALIBRATION REQUIRED"));
    }
    Serial.println(F("                                       │"));
    
    Serial.print(F("│ Battery: "));
    Serial.print(telemetry.batteryVoltage, 1);
    Serial.print(F("V"));
    if (telemetry.batteryVoltage < 10.5) {
      Serial.print(F(" [LOW!]"));
    }
    Serial.println(F("                                                    │"));
    
    Serial.print(F("│ Attitude: Roll="));
    Serial.print(telemetry.roll, 1);
    Serial.print(F("° Pitch="));
    Serial.print(telemetry.pitch, 1);
    Serial.print(F("° Yaw="));
    Serial.print(telemetry.yaw, 1);
    Serial.println(F("°                   │"));
    
    Serial.print(F("│ Gyro: X="));
    Serial.print(telemetry.gyroX);
    Serial.print(F(" Y="));
    Serial.print(telemetry.gyroY);
    Serial.print(F(" Z="));
    Serial.print(telemetry.gyroZ);
    Serial.println(F("                                       │"));
    
    Serial.println(F("└────────────────────────────────────────────────────────────────────┘"));
  } else {
    Serial.println(F("┌─── DRONE TELEMETRY ────────────────────────────────────────────────┐"));
    Serial.println(F("│                                                                    │"));
    Serial.println(F("│                    ✗ NO TELEMETRY DATA RECEIVED                   │"));
    Serial.println(F("│                                                                    │"));
    Serial.println(F("│  Please check:                                                     │"));
    Serial.println(F("│  1. Flight controller is powered on                                │"));
    Serial.println(F("│  2. NRF24L01 modules are properly connected                        │"));
    Serial.println(F("│  3. Both devices are on the same channel (103)                     │"));
    Serial.println(F("│                                                                    │"));
    Serial.println(F("└────────────────────────────────────────────────────────────────────┘"));
  }
  
  Serial.println(F(""));
  
  // Warnings
  if (telemetry.armed) {
    Serial.println(F("┌────────────────────────────────────────────────────────────────────┐"));
    Serial.println(F("│                        ⚠ WARNING ⚠                                │"));
    Serial.println(F("│                    MOTORS ARE ARMED!                               │"));
    Serial.println(F("│              Keep clear of propellers!                             │"));
    Serial.println(F("└────────────────────────────────────────────────────────────────────┘"));
  }
  
  if (!linkActive) {
    Serial.println(F("┌────────────────────────────────────────────────────────────────────┐"));
    Serial.println(F("│                        ⚠ WARNING ⚠                                │"));
    Serial.println(F("│                   NO COMMUNICATION LINK                            │"));
    Serial.println(F("│                  Failsafe may be active                            │"));
    Serial.println(F("└────────────────────────────────────────────────────────────────────┘"));
  }
  
  if (!telemetry.calibrated && linkActive) {
    Serial.println(F("┌────────────────────────────────────────────────────────────────────┐"));
    Serial.println(F("│                          NOTICE                                    │"));
    Serial.println(F("│              Press BUTTON 1 to calibrate gyro                      │"));
    Serial.println(F("│          (Place drone on level surface first)                      │"));
    Serial.println(F("└────────────────────────────────────────────────────────────────────┘"));
  }
  
  Serial.println(F(""));
  Serial.print(F("Last Update: "));
  Serial.print(millis() / 1000);
  Serial.println(F("s"));
}

// ============================================================================
// CHECKSUM CALCULATION
// ============================================================================
uint8_t calculateChecksum(uint8_t* data, uint8_t len) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// ============================================================================
// END OF REMOTE CONTROLLER
// ============================================================================
