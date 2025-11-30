/*
 * =====================================================================
 * PROFESSIONAL QUADCOPTER DRONE - REMOTE CONTROLLER
 * =====================================================================
 * 
 * Hardware: Arduino Nano
 * Author: Professional Embedded Systems Engineer
 * Version: 1.0
 * Date: 2025
 * 
 * Description:
 *   Professional remote controller for quadcopter drone system with
 *   NRF24L01 wireless communication, 4-channel control, and safety features.
 * 
 * Pin Configuration:
 *   NRF24L01: CE=D9, CSN=D10, SCK=D13, MOSI=D11, MISO=D12
 *   Left Joystick: Throttle=A0, Yaw=A1
 *   Right Joystick: Pitch=A2, Roll=A3
 *   Buttons: BTN1=D4, BTN2=D5, BTN3=D6
 *   Switches: SW1=D2, SW2=D3
 * 
 * =====================================================================
 */

#include <SPI.h>
#include <RF24.h>

// =====================================================================
// CONFIGURATION
// =====================================================================

// NRF24L01 Configuration
#define NRF_CE_PIN      9
#define NRF_CSN_PIN     10
#define NRF_CHANNEL     103
#define NRF_PA_LEVEL    RF24_PA_MAX
#define NRF_DATA_RATE   RF24_250KBPS

// Joystick Pins
#define THROTTLE_PIN    A0  // Left Joystick Vertical
#define YAW_PIN         A1  // Left Joystick Horizontal
#define PITCH_PIN       A2  // Right Joystick Vertical
#define ROLL_PIN        A3  // Right Joystick Horizontal

// Button Pins
#define BTN_CALIBRATE   4   // Button 1: Gyro Calibration
#define BTN_ESC_CAL     5   // Button 2: ESC Calibration
#define BTN_MOTOR_TEST  6   // Button 3: Motor Test

// Switch Pins
#define SW_ALT_HOLD     2   // Switch 1: Altitude Hold
#define SW_ARM          3   // Switch 2: Arming/Kill Switch

// Communication Settings
#define PACKET_SIZE     32
#define SEND_INTERVAL   20  // 50Hz update rate
#define ACK_TIMEOUT     500 // ms
#define LINK_TIMEOUT    1000 // ms

// Joystick Calibration
#define STICK_CENTER    512
#define STICK_DEADBAND  10
#define STICK_MIN       0
#define STICK_MAX       1023

// =====================================================================
// DATA STRUCTURES
// =====================================================================

// Packet structure for RC -> FC communication
struct RC_Data {
  uint16_t throttle;      // 1000-2000
  uint16_t yaw;           // 1000-2000
  uint16_t pitch;         // 1000-2000
  uint16_t roll;          // 1000-2000
  
  // Control flags
  uint8_t armed:1;        // Bit 0: Armed status
  uint8_t altHold:1;      // Bit 1: Altitude hold
  uint8_t calibrate:1;    // Bit 2: Calibration request
  uint8_t escCal:1;       // Bit 3: ESC calibration
  uint8_t motorTest:1;    // Bit 4: Motor test
  uint8_t reserved:3;     // Bits 5-7: Reserved
  
  uint16_t checksum;      // Data integrity check
};

// Acknowledgment packet from FC -> RC
struct FC_Ack {
  uint8_t status;         // Flight controller status
  uint8_t calResult;      // Calibration result
  int16_t batteryVoltage; // Battery voltage (mV)
  uint8_t rssi;           // Signal strength
  uint16_t checksum;      // Data integrity check
};

// Status codes
enum FC_Status {
  FC_IDLE = 0,
  FC_CALIBRATING,
  FC_ARMED,
  FC_FLYING,
  FC_ERROR
};

enum Cal_Result {
  CAL_NONE = 0,
  CAL_SUCCESS,
  CAL_FAILED,
  CAL_IN_PROGRESS
};

// =====================================================================
// GLOBAL OBJECTS & VARIABLES
// =====================================================================

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const uint64_t pipeOut = 0xE8E8F0F0E1LL;  // TX address
const uint64_t pipeIn = 0xE8E8F0F0E2LL;   // RX address (for ACK)

RC_Data rcData;
FC_Ack fcAck;

// Timing variables
unsigned long lastSendTime = 0;
unsigned long lastLinkTime = 0;
bool linkActive = false;

// Button state tracking
bool btn1State = false, btn1LastState = false;
bool btn2State = false, btn2LastState = false;
bool btn3State = false, btn3LastState = false;

// Joystick calibration offsets
int16_t throttleOffset = 0;
int16_t yawOffset = 0;
int16_t pitchOffset = 0;
int16_t rollOffset = 0;

// =====================================================================
// SETUP FUNCTION
// =====================================================================

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  initializePins();
  
  // Initialize NRF24L01
  if (!initializeRadio()) {
    Serial.println(F("❌ NRF24L01 initialization FAILED!"));
    while (1) {
      delay(1000);
    }
  }
  
  // Calibrate joysticks to center
  calibrateJoysticks();
  
  // Initialize RC data with safe defaults
  initializeRCData();
  
  Serial.println(F("================================"));
  Serial.println(F("  DRONE REMOTE CONTROLLER v1.0"));
  Serial.println(F("================================"));
  Serial.println(F("✓ Remote Controller Ready"));
  Serial.println(F("✓ Waiting for Flight Controller..."));
  Serial.println(F("================================\n"));
  
  delay(500);
}

// =====================================================================
// MAIN LOOP
// =====================================================================

void loop() {
  unsigned long currentTime = millis();
  
  // Read all inputs
  readJoysticks();
  readButtons();
  readSwitches();
  
  // Send data at fixed interval
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentTime;
    
    // Update checksum
    rcData.checksum = calculateChecksum((uint8_t*)&rcData, sizeof(RC_Data) - 2);
    
    // Send data and receive ACK
    bool success = sendData();
    
    if (success) {
      linkActive = true;
      lastLinkTime = currentTime;
      processAcknowledgment();
    } else {
      if (currentTime - lastLinkTime > LINK_TIMEOUT) {
        linkActive = false;
      }
    }
  }
  
  // Display status on serial monitor
  static unsigned long lastDisplayTime = 0;
  if (currentTime - lastDisplayTime >= 200) {  // 5Hz display update
    lastDisplayTime = currentTime;
    displayStatus();
  }
}

// =====================================================================
// INITIALIZATION FUNCTIONS
// =====================================================================

void initializePins() {
  // Configure button pins with internal pull-ups
  pinMode(BTN_CALIBRATE, INPUT_PULLUP);
  pinMode(BTN_ESC_CAL, INPUT_PULLUP);
  pinMode(BTN_MOTOR_TEST, INPUT_PULLUP);
  
  // Configure switch pins with internal pull-ups
  pinMode(SW_ALT_HOLD, INPUT_PULLUP);
  pinMode(SW_ARM, INPUT_PULLUP);
  
  // Joystick pins are analog inputs (no pinMode needed)
}

bool initializeRadio() {
  if (!radio.begin()) {
    return false;
  }
  
  // Configure radio
  radio.setChannel(NRF_CHANNEL);
  radio.setPALevel(NRF_PA_LEVEL);
  radio.setDataRate(NRF_DATA_RATE);
  radio.setAutoAck(true);
  radio.setRetries(3, 5);  // 3 retries, 5*250us = 1.25ms delay
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  
  // Open pipes
  radio.openWritingPipe(pipeOut);
  radio.openReadingPipe(1, pipeIn);
  
  radio.stopListening();  // TX mode
  
  return true;
}

void calibrateJoysticks() {
  Serial.println(F("Calibrating joysticks to center..."));
  
  // Take multiple samples and average
  long throttleSum = 0, yawSum = 0, pitchSum = 0, rollSum = 0;
  const int samples = 50;
  
  for (int i = 0; i < samples; i++) {
    throttleSum += analogRead(THROTTLE_PIN);
    yawSum += analogRead(YAW_PIN);
    pitchSum += analogRead(PITCH_PIN);
    rollSum += analogRead(ROLL_PIN);
    delay(10);
  }
  
  // Calculate offsets from ideal center (512)
  throttleOffset = STICK_CENTER - (throttleSum / samples);
  yawOffset = STICK_CENTER - (yawSum / samples);
  pitchOffset = STICK_CENTER - (pitchSum / samples);
  rollOffset = STICK_CENTER - (rollSum / samples);
  
  Serial.println(F("✓ Joystick calibration complete"));
}

void initializeRCData() {
  rcData.throttle = 1000;  // Min throttle
  rcData.yaw = 1500;       // Center
  rcData.pitch = 1500;     // Center
  rcData.roll = 1500;      // Center
  rcData.armed = 0;
  rcData.altHold = 0;
  rcData.calibrate = 0;
  rcData.escCal = 0;
  rcData.motorTest = 0;
  rcData.reserved = 0;
  rcData.checksum = 0;
}

// =====================================================================
// INPUT READING FUNCTIONS
// =====================================================================

void readJoysticks() {
  // Read raw values
  int throttleRaw = analogRead(THROTTLE_PIN) + throttleOffset;
  int yawRaw = analogRead(YAW_PIN) + yawOffset;
  int pitchRaw = analogRead(PITCH_PIN) + pitchOffset;
  int rollRaw = analogRead(ROLL_PIN) + rollOffset;
  
  // Apply deadband and constraints
  throttleRaw = constrain(throttleRaw, STICK_MIN, STICK_MAX);
  yawRaw = applyDeadband(yawRaw, STICK_CENTER, STICK_DEADBAND);
  pitchRaw = applyDeadband(pitchRaw, STICK_CENTER, STICK_DEADBAND);
  rollRaw = applyDeadband(rollRaw, STICK_CENTER, STICK_DEADBAND);
  
  // Map to PWM range (1000-2000)
  rcData.throttle = map(throttleRaw, STICK_MIN, STICK_MAX, 1000, 2000);
  rcData.yaw = map(yawRaw, STICK_MIN, STICK_MAX, 1000, 2000);
  rcData.pitch = map(pitchRaw, STICK_MIN, STICK_MAX, 1000, 2000);
  rcData.roll = map(rollRaw, STICK_MIN, STICK_MAX, 1000, 2000);
  
  // Constrain to valid range
  rcData.throttle = constrain(rcData.throttle, 1000, 2000);
  rcData.yaw = constrain(rcData.yaw, 1000, 2000);
  rcData.pitch = constrain(rcData.pitch, 1000, 2000);
  rcData.roll = constrain(rcData.roll, 1000, 2000);
}

int applyDeadband(int value, int center, int deadband) {
  if (abs(value - center) < deadband) {
    return center;
  }
  return constrain(value, STICK_MIN, STICK_MAX);
}

void readButtons() {
  // Read button states (active LOW with pull-up)
  btn1State = !digitalRead(BTN_CALIBRATE);
  btn2State = !digitalRead(BTN_ESC_CAL);
  btn3State = !digitalRead(BTN_MOTOR_TEST);
  
  // Detect button press (rising edge)
  if (btn1State && !btn1LastState) {
    rcData.calibrate = 1;
  } else {
    rcData.calibrate = 0;
  }
  
  if (btn2State && !btn2LastState) {
    rcData.escCal = 1;
  } else {
    rcData.escCal = 0;
  }
  
  if (btn3State && !btn3LastState) {
    rcData.motorTest = 1;
  } else {
    rcData.motorTest = 0;
  }
  
  // Update last states
  btn1LastState = btn1State;
  btn2LastState = btn2State;
  btn3LastState = btn3State;
}

void readSwitches() {
  // Read switch states (active LOW with pull-up)
  rcData.altHold = !digitalRead(SW_ALT_HOLD);
  rcData.armed = !digitalRead(SW_ARM);
}

// =====================================================================
// COMMUNICATION FUNCTIONS
// =====================================================================

bool sendData() {
  // Send data with ACK
  radio.startListening();
  delayMicroseconds(100);
  radio.stopListening();
  
  bool success = radio.write(&rcData, sizeof(RC_Data));
  
  if (success && radio.isAckPayloadAvailable()) {
    // Read ACK payload
    radio.read(&fcAck, sizeof(FC_Ack));
    
    // Verify checksum
    uint16_t calcChecksum = calculateChecksum((uint8_t*)&fcAck, sizeof(FC_Ack) - 2);
    if (calcChecksum == fcAck.checksum) {
      return true;
    }
  }
  
  return success;
}

void processAcknowledgment() {
  // Process FC status and feedback
  // This can be expanded for additional features
  
  // Handle calibration result display
  static uint8_t lastCalResult = CAL_NONE;
  if (fcAck.calResult != lastCalResult) {
    lastCalResult = fcAck.calResult;
    
    switch (fcAck.calResult) {
      case CAL_SUCCESS:
        Serial.println(F("\n✓✓ CALIBRATION SUCCESS ✓✓\n"));
        break;
      case CAL_FAILED:
        Serial.println(F("\n❌ CALIBRATION FAILED ❌\n"));
        break;
      case CAL_IN_PROGRESS:
        Serial.println(F("\n⏳ Calibrating... Keep drone steady!\n"));
        break;
    }
  }
}

uint16_t calculateChecksum(uint8_t* data, size_t length) {
  uint16_t sum = 0;
  for (size_t i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum;
}

// =====================================================================
// DISPLAY FUNCTION
// =====================================================================

void displayStatus() {
  Serial.println(F("\n╔═══════════════════════════════════════════════╗"));
  Serial.println(F("║      REMOTE CONTROLLER STATUS MONITOR         ║"));
  Serial.println(F("╚═══════════════════════════════════════════════╝"));
  
  // Connection status
  Serial.print(F("📡 NRF24L01: "));
  if (linkActive) {
    Serial.println(F("✓ LINKED   [OK]"));
  } else {
    Serial.println(F("✗ NO LINK  [SEARCHING...]"));
  }
  
  // Flight Controller Status
  Serial.print(F("🎯 FC Status: "));
  switch (fcAck.status) {
    case FC_IDLE:
      Serial.println(F("IDLE"));
      break;
    case FC_CALIBRATING:
      Serial.println(F("CALIBRATING"));
      break;
    case FC_ARMED:
      Serial.println(F("ARMED - READY"));
      break;
    case FC_FLYING:
      Serial.println(F("FLYING"));
      break;
    case FC_ERROR:
      Serial.println(F("ERROR"));
      break;
    default:
      Serial.println(F("UNKNOWN"));
  }
  
  Serial.println(F("\n--- CONTROL INPUTS ---"));
  
  // Joystick values
  Serial.print(F("🕹️  Throttle: "));
  Serial.print(rcData.throttle);
  Serial.print(F(" | Yaw: "));
  Serial.println(rcData.yaw);
  
  Serial.print(F("🕹️  Pitch:    "));
  Serial.print(rcData.pitch);
  Serial.print(F(" | Roll: "));
  Serial.println(rcData.roll);
  
  Serial.println(F("\n--- SWITCHES & BUTTONS ---"));
  
  // Switches
  Serial.print(F("🔘 Altitude Hold (SW1): "));
  Serial.println(rcData.altHold ? F("[ON]") : F("[OFF]"));
  
  Serial.print(F("🔘 Arming Switch (SW2): "));
  Serial.println(rcData.armed ? F("[ARMED] ⚠️") : F("[DISARMED]"));
  
  // Buttons
  Serial.print(F("🔳 BTN1 (Cal): "));
  Serial.print(btn1State ? F("[PRESSED]") : F("[   ]"));
  Serial.print(F(" | BTN2 (ESC): "));
  Serial.print(btn2State ? F("[PRESSED]") : F("[   ]"));
  Serial.print(F(" | BTN3 (Test): "));
  Serial.println(btn3State ? F("[PRESSED]") : F("[   ]"));
  
  // Battery and RSSI
  Serial.println(F("\n--- TELEMETRY ---"));
  Serial.print(F("🔋 Battery: "));
  Serial.print(fcAck.batteryVoltage / 1000.0, 2);
  Serial.print(F("V | RSSI: "));
  Serial.print(fcAck.rssi);
  Serial.println(F("%"));
  
  // Calibration result
  if (fcAck.calResult != CAL_NONE) {
    Serial.print(F("📊 Last Calibration: "));
    switch (fcAck.calResult) {
      case CAL_SUCCESS:
        Serial.println(F("✓ SUCCESS"));
        break;
      case CAL_FAILED:
        Serial.println(F("✗ FAILED"));
        break;
      case CAL_IN_PROGRESS:
        Serial.println(F("⏳ IN PROGRESS..."));
        break;
    }
  }
  
  Serial.println(F("═══════════════════════════════════════════════\n"));
}

// =====================================================================
// END OF REMOTE CONTROLLER CODE
// =====================================================================
