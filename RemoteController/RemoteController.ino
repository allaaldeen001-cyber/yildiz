/*
 * Professional Drone Remote Controller
 * Arduino Nano + NRF24L01 + Joysticks + Switches
 * 
 * Author: UAV Embedded Systems
 * Description: Professional RC transmitter with real-time telemetry display,
 *              dual joysticks, buttons, and safety switches
 */

// ============================================================================
// INCLUDES
// ============================================================================
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
// NRF24L01
#define NRF_CE_PIN        9
#define NRF_CSN_PIN       10

// Joysticks (Analog inputs)
#define THROTTLE_PIN      A0    // Left Vertical
#define YAW_PIN           A1    // Left Horizontal
#define PITCH_PIN         A2    // Right Vertical
#define ROLL_PIN          A3    // Right Horizontal

// Switches
#define SW_ALTHOLD_PIN    2     // SW_1: Altitude Hold
#define SW_ARM_PIN        3     // SW_2: Arm/Disarm Kill Switch

// Buttons
#define BTN_CALIB_PIN     4     // BTN_1: Gyro Calibration
#define BTN_ESC_PIN       5     // BTN_2: ESC Calibration

// ============================================================================
// CONSTANTS
// ============================================================================
#define NRF_CHANNEL       103
#define SERIAL_BAUD       115200
#define UPDATE_RATE       50      // Hz (20ms)
#define DISPLAY_RATE      5       // Hz (200ms)

// Joystick calibration
#define STICK_MIN         0
#define STICK_MAX         1023
#define STICK_CENTER      512
#define DEADZONE          30

// Output ranges
#define THROTTLE_MIN      1000
#define THROTTLE_MAX      2000
#define CONTROL_RANGE     500     // ±500 for pitch/roll/yaw

// ============================================================================
// DATA STRUCTURES
// ============================================================================
// Data sent to Flight Controller
struct RCData {
  uint16_t throttle;    // 1000-2000
  int16_t yaw;          // -500 to +500
  int16_t pitch;        // -500 to +500
  int16_t roll;         // -500 to +500
  bool armSwitch;       // SW_2: Arm/Disarm
  bool altHoldSwitch;   // SW_1: Altitude hold
  bool calibButton;     // BTN_1: Calibration
  bool escButton;       // BTN_2: ESC calibration
  uint8_t checksum;
};

// Telemetry data received from FC
struct TelemetryData {
  float batteryVoltage;
  float altitude;
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  bool armed;
  bool calibrated;
  uint8_t errorCode;
  uint8_t checksum;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
// NRF24L01
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte txAddress[6] = "DRONE";
const byte rxAddress[6] = "REMOT";

// Data packets
RCData rcData;
TelemetryData telemetry;

// Input states
int throttleRaw, yawRaw, pitchRaw, rollRaw;
bool armSwitch, altHoldSwitch;
bool calibButton, escButton;
bool lastCalibButton = false;
bool lastEscButton = false;

// Connection status
bool connected = false;
unsigned long lastAckTime = 0;

// Timing
unsigned long lastTxTime = 0;
unsigned long lastDisplayTime = 0;

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // Initialize input pins
  pinMode(SW_ALTHOLD_PIN, INPUT_PULLUP);
  pinMode(SW_ARM_PIN, INPUT_PULLUP);
  pinMode(BTN_CALIB_PIN, INPUT_PULLUP);
  pinMode(BTN_ESC_PIN, INPUT_PULLUP);
  
  // Print header
  Serial.println(F("\n\n"));
  Serial.println(F("╔════════════════════════════════════════╗"));
  Serial.println(F("║   DRONE REMOTE CONTROLLER v1.0        ║"));
  Serial.println(F("║   Professional UAV Control System     ║"));
  Serial.println(F("╚════════════════════════════════════════╝"));
  Serial.println();
  
  // Initialize NRF24L01
  Serial.print(F("Initializing NRF24L01... "));
  if (!initNRF()) {
    Serial.println(F("FAILED!"));
    Serial.println(F("Check wiring and reset!"));
    while(1);
  }
  Serial.println(F("OK"));
  
  Serial.print(F("RF Channel: "));
  Serial.println(NRF_CHANNEL);
  Serial.println(F("Waiting for Flight Controller..."));
  Serial.println();
  
  delay(500);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Read all inputs
  readInputs();
  
  // Handle button debouncing and actions
  handleButtons();
  
  // Prepare RC data packet
  prepareRCData();
  
  // Transmit data at fixed rate
  if (millis() - lastTxTime >= (1000 / UPDATE_RATE)) {
    transmitData();
    lastTxTime = millis();
  }
  
  // Receive telemetry
  receiveTelemetry();
  
  // Update serial display
  if (millis() - lastDisplayTime >= (1000 / DISPLAY_RATE)) {
    displayStatus();
    lastDisplayTime = millis();
  }
  
  // Check connection timeout
  if (millis() - lastAckTime > 1000 && connected) {
    connected = false;
  }
}

// ============================================================================
// NRF24L01 FUNCTIONS
// ============================================================================
bool initNRF() {
  if (!radio.begin()) {
    return false;
  }
  
  radio.setChannel(NRF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setRetries(5, 15);
  
  radio.openWritingPipe(txAddress);
  radio.openReadingPipe(1, rxAddress);
  
  return true;
}

uint8_t calculateChecksum(uint8_t* data, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len - 1; i++) {
    sum += data[i];
  }
  return sum;
}

bool validateChecksum(uint8_t* data, uint8_t len) {
  return (calculateChecksum(data, len) == data[len - 1]);
}

void transmitData() {
  radio.stopListening();
  
  bool success = radio.write(&rcData, sizeof(rcData));
  
  if (success) {
    if (!connected) {
      connected = true;
      Serial.println(F("\n>>> CONNECTED TO FLIGHT CONTROLLER <<<\n"));
    }
    lastAckTime = millis();
  }
  
  radio.startListening();
}

void receiveTelemetry() {
  if (radio.available()) {
    radio.read(&telemetry, sizeof(telemetry));
    
    if (validateChecksum((uint8_t*)&telemetry, sizeof(telemetry))) {
      // Telemetry data is valid
      lastAckTime = millis();
    }
  }
}

// ============================================================================
// INPUT FUNCTIONS
// ============================================================================
void readInputs() {
  // Read analog joysticks
  throttleRaw = analogRead(THROTTLE_PIN);
  yawRaw = analogRead(YAW_PIN);
  pitchRaw = analogRead(PITCH_PIN);
  rollRaw = analogRead(ROLL_PIN);
  
  // Read switches (active LOW with pullup)
  armSwitch = !digitalRead(SW_ARM_PIN);
  altHoldSwitch = !digitalRead(SW_ALTHOLD_PIN);
  
  // Read buttons (active LOW with pullup)
  calibButton = !digitalRead(BTN_CALIB_PIN);
  escButton = !digitalRead(BTN_ESC_PIN);
}

void handleButtons() {
  // Debounce calibration button
  static unsigned long lastCalibPress = 0;
  if (calibButton && !lastCalibButton && (millis() - lastCalibPress > 500)) {
    Serial.println(F("\n>>> CALIBRATION REQUESTED <<<"));
    lastCalibPress = millis();
  }
  lastCalibButton = calibButton;
  
  // Debounce ESC button
  static unsigned long lastEscPress = 0;
  if (escButton && !lastEscButton && (millis() - lastEscPress > 500)) {
    Serial.println(F("\n>>> ESC CALIBRATION REQUESTED <<<"));
    lastEscPress = millis();
  }
  lastEscButton = escButton;
}

void prepareRCData() {
  // Throttle: 0-1023 -> 1000-2000 (no deadzone on throttle)
  rcData.throttle = map(throttleRaw, STICK_MIN, STICK_MAX, THROTTLE_MIN, THROTTLE_MAX);
  rcData.throttle = constrain(rcData.throttle, THROTTLE_MIN, THROTTLE_MAX);
  
  // Yaw: apply deadzone and map to ±500
  if (abs(yawRaw - STICK_CENTER) < DEADZONE) {
    rcData.yaw = 0;
  } else {
    rcData.yaw = map(yawRaw, STICK_MIN, STICK_MAX, -CONTROL_RANGE, CONTROL_RANGE);
    rcData.yaw = constrain(rcData.yaw, -CONTROL_RANGE, CONTROL_RANGE);
  }
  
  // Pitch: apply deadzone and map to ±500
  if (abs(pitchRaw - STICK_CENTER) < DEADZONE) {
    rcData.pitch = 0;
  } else {
    rcData.pitch = map(pitchRaw, STICK_MIN, STICK_MAX, -CONTROL_RANGE, CONTROL_RANGE);
    rcData.pitch = constrain(rcData.pitch, -CONTROL_RANGE, CONTROL_RANGE);
  }
  
  // Roll: apply deadzone and map to ±500
  if (abs(rollRaw - STICK_CENTER) < DEADZONE) {
    rcData.roll = 0;
  } else {
    rcData.roll = map(rollRaw, STICK_MIN, STICK_MAX, -CONTROL_RANGE, CONTROL_RANGE);
    rcData.roll = constrain(rcData.roll, -CONTROL_RANGE, CONTROL_RANGE);
  }
  
  // Switches and buttons
  rcData.armSwitch = armSwitch;
  rcData.altHoldSwitch = altHoldSwitch;
  rcData.calibButton = calibButton;
  rcData.escButton = escButton;
  
  // Calculate checksum
  rcData.checksum = calculateChecksum((uint8_t*)&rcData, sizeof(rcData));
}

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================
void displayStatus() {
  // Clear screen (ANSI escape code)
  Serial.write(27);       // ESC
  Serial.print(F("[2J")); // Clear screen
  Serial.write(27);       // ESC
  Serial.print(F("[H"));  // Home cursor
  
  // Header
  Serial.println(F("╔════════════════════════════════════════════════════════╗"));
  Serial.println(F("║        PROFESSIONAL DRONE REMOTE CONTROLLER           ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════╝"));
  Serial.println();
  
  // Connection status
  Serial.print(F("  CONNECTION: "));
  if (connected) {
    Serial.println(F("✓ LINKED"));
  } else {
    Serial.println(F("✗ SEARCHING..."));
  }
  Serial.println();
  
  // Flight Controller Status
  Serial.println(F("┌─── FLIGHT CONTROLLER STATUS ───────────────────────────┐"));
  if (connected) {
    Serial.print(F("│  Armed:      "));
    Serial.println(telemetry.armed ? F("YES ⚠")  : F("NO"));
    
    Serial.print(F("│  Calibrated: "));
    Serial.println(telemetry.calibrated ? F("YES ✓") : F("NO ✗"));
    
    Serial.print(F("│  Battery:    "));
    Serial.print(telemetry.batteryVoltage, 1);
    Serial.println(F(" V"));
    
    Serial.print(F("│  Altitude:   "));
    Serial.print(telemetry.altitude, 1);
    Serial.println(F(" m"));
    
    Serial.print(F("│  Attitude:   Roll="));
    Serial.print(telemetry.roll);
    Serial.print(F("° Pitch="));
    Serial.print(telemetry.pitch);
    Serial.print(F("° Yaw="));
    Serial.print(telemetry.yaw);
    Serial.println(F("°"));
  } else {
    Serial.println(F("│  No telemetry data available"));
  }
  Serial.println(F("└────────────────────────────────────────────────────────┘"));
  Serial.println();
  
  // Control Inputs
  Serial.println(F("┌─── CONTROL INPUTS ─────────────────────────────────────┐"));
  
  // Throttle
  Serial.print(F("│  Throttle:   "));
  Serial.print(rcData.throttle);
  Serial.print(F(" "));
  printBar(rcData.throttle, THROTTLE_MIN, THROTTLE_MAX, 20);
  Serial.println();
  
  // Yaw
  Serial.print(F("│  Yaw:        "));
  if (rcData.yaw >= 0) Serial.print(F(" "));
  Serial.print(rcData.yaw);
  Serial.print(F(" "));
  printBarCentered(rcData.yaw, CONTROL_RANGE, 20);
  Serial.println();
  
  // Pitch
  Serial.print(F("│  Pitch:      "));
  if (rcData.pitch >= 0) Serial.print(F(" "));
  Serial.print(rcData.pitch);
  Serial.print(F(" "));
  printBarCentered(rcData.pitch, CONTROL_RANGE, 20);
  Serial.println();
  
  // Roll
  Serial.print(F("│  Roll:       "));
  if (rcData.roll >= 0) Serial.print(F(" "));
  Serial.print(rcData.roll);
  Serial.print(F(" "));
  printBarCentered(rcData.roll, CONTROL_RANGE, 20);
  Serial.println();
  
  Serial.println(F("└────────────────────────────────────────────────────────┘"));
  Serial.println();
  
  // Switches and Buttons
  Serial.println(F("┌─── SWITCHES & BUTTONS ─────────────────────────────────┐"));
  
  Serial.print(F("│  SW_1 (Alt Hold):  "));
  Serial.println(altHoldSwitch ? F("ON  ✓") : F("OFF"));
  
  Serial.print(F("│  SW_2 (Arm/Kill):  "));
  Serial.println(armSwitch ? F("ON  ⚠") : F("OFF"));
  
  Serial.print(F("│  BTN_1 (Calib):    "));
  Serial.println(calibButton ? F("PRESSED") : F("Released"));
  
  Serial.print(F("│  BTN_2 (ESC Cal):  "));
  Serial.println(escButton ? F("PRESSED") : F("Released"));
  
  Serial.println(F("└────────────────────────────────────────────────────────┘"));
  Serial.println();
  
  // Instructions
  if (!connected) {
    Serial.println(F("  ⚠ Waiting for Flight Controller connection..."));
  } else if (!telemetry.calibrated) {
    Serial.println(F("  ⚠ Press BTN_1 to calibrate gyroscope"));
  } else if (!telemetry.armed && armSwitch) {
    Serial.println(F("  ⚠ Lower throttle to arm!"));
  } else if (!telemetry.armed) {
    Serial.println(F("  ✓ Ready to arm - Flip SW_2"));
  } else {
    Serial.println(F("  ✓ ARMED - Fly safe!"));
  }
  
  Serial.println();
  Serial.print(F("  Update Rate: "));
  Serial.print(UPDATE_RATE);
  Serial.print(F(" Hz | Channel: "));
  Serial.println(NRF_CHANNEL);
}

void printBar(int value, int minVal, int maxVal, int width) {
  int filled = map(value, minVal, maxVal, 0, width);
  filled = constrain(filled, 0, width);
  
  Serial.print(F("["));
  for (int i = 0; i < width; i++) {
    if (i < filled) {
      Serial.print(F("█"));
    } else {
      Serial.print(F("░"));
    }
  }
  Serial.print(F("]"));
}

void printBarCentered(int value, int range, int width) {
  int center = width / 2;
  int pos = map(value, -range, range, 0, width);
  pos = constrain(pos, 0, width);
  
  Serial.print(F("["));
  for (int i = 0; i < width; i++) {
    if (i == center) {
      Serial.print(F("|"));
    } else if ((value > 0 && i > center && i <= pos) || 
               (value < 0 && i < center && i >= pos)) {
      Serial.print(F("█"));
    } else {
      Serial.print(F("░"));
    }
  }
  Serial.print(F("]"));
}
