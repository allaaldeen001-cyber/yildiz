/*
 * ============================================================================
 * PROFESSIONAL QUADCOPTER REMOTE CONTROLLER FIRMWARE
 * ============================================================================
 * Target: Arduino Nano
 * Communication: NRF24L01 PA+LNA
 * 
 * Features:
 * - Dual joystick control (throttle, yaw, pitch, roll)
 * - Arm/Disarm kill switch
 * - Altitude hold toggle
 * - Calibration trigger button
 * - ESC calibration + motor test trigger
 * - Real-time telemetry display
 * - Link status monitoring
 * - Works with or without serial monitor
 * 
 * Author: UAV Embedded Systems Engineer
 * ============================================================================
 */

#include <SPI.h>
#include <RF24.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define NRF_CE_PIN      9
#define NRF_CSN_PIN     10

// Joysticks (Analog)
#define JOYSTICK_THROTTLE_PIN   A0  // Left V
#define JOYSTICK_YAW_PIN        A1  // Left H
#define JOYSTICK_PITCH_PIN      A2  // Right V
#define JOYSTICK_ROLL_PIN       A3  // Right H

// Buttons (Digital - Active LOW with internal pullup)
#define BUTTON1_PIN     4   // Calibration trigger
#define BUTTON2_PIN     5   // ESC calibration + motor test

// Switches (Digital - Active LOW with internal pullup)
#define SWITCH_ALTHOLD_PIN  2   // SW_1: Altitude Hold ON/OFF
#define SWITCH_ARM_PIN      3   // SW_2: ARM/DISARM kill switch

// ============================================================================
// COMMUNICATION STRUCTURES (MUST MATCH FC)
// ============================================================================
struct RC_Data {
  uint16_t throttle;      // 1000-2000
  uint16_t yaw;           // 1000-2000
  uint16_t pitch;         // 1000-2000
  uint16_t roll;          // 1000-2000
  uint8_t  armed;         // 0=disarmed, 1=armed
  uint8_t  altHold;       // 0=off, 1=on
  uint8_t  button1;       // Calibration trigger
  uint8_t  button2;       // ESC calibration + motor test
  uint32_t timestamp;     // For timeout detection
};

struct FC_Telemetry {
  float roll;             // degrees
  float pitch;            // degrees
  float yaw;              // degrees
  float altitude;         // meters
  uint8_t calibrated;     // 0=not calibrated, 1=calibrated
  uint8_t linked;         // 0=no link, 1=linked
  uint16_t loopTime;      // microseconds
};

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte txAddress[6] = "FC001";
const byte rxAddress[6] = "RC001";

RC_Data rcData = {1000, 1500, 1500, 1500, 0, 0, 0, 0, 0};
FC_Telemetry telemetry = {0, 0, 0, 0, 0, 0, 0};

// ============================================================================
// JOYSTICK CALIBRATION & FILTERING
// ============================================================================
struct JoystickCalibration {
  int center;
  int deadzone;
  int min;
  int max;
};

JoystickCalibration joyThrottle = {512, 20, 0, 1023};
JoystickCalibration joyYaw      = {512, 30, 0, 1023};
JoystickCalibration joyPitch    = {512, 30, 0, 1023};
JoystickCalibration joyRoll     = {512, 30, 0, 1023};

// Exponential filter for smooth readings
struct ExpoFilter {
  float alpha;
  float filtered;
};

ExpoFilter filterThrottle = {0.3, 512};
ExpoFilter filterYaw      = {0.3, 512};
ExpoFilter filterPitch    = {0.3, 512};
ExpoFilter filterRoll     = {0.3, 512};

// ============================================================================
// TIMING VARIABLES
// ============================================================================
unsigned long lastSendTime = 0;
unsigned long lastTelemetryReceived = 0;
unsigned long lastDisplayUpdate = 0;

const unsigned long SEND_INTERVAL = 20;        // 50Hz transmission rate
const unsigned long DISPLAY_INTERVAL = 200;    // 5Hz display update
const unsigned long LINK_TIMEOUT = 500;        // 500ms timeout

bool linkActive = false;
bool serialAvailable = false;

// ============================================================================
// BUTTON DEBOUNCING
// ============================================================================
struct ButtonState {
  uint8_t pin;
  bool currentState;
  bool lastState;
  unsigned long lastDebounceTime;
  unsigned long debounceDelay;
};

ButtonState button1 = {BUTTON1_PIN, HIGH, HIGH, 0, 50};
ButtonState button2 = {BUTTON2_PIN, HIGH, HIGH, 0, 50};

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void calibrateJoysticks();
float applyExpoFilter(ExpoFilter &filter, int rawValue);
int readJoystick(int pin, JoystickCalibration &cal, ExpoFilter &filter);
bool readButton(ButtonState &btn);
bool readSwitch(int pin);
void updateRCData();
void displayStatus();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  // Wait briefly for serial, but don't block if not connected
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 1000));
  
  serialAvailable = (bool)Serial;
  
  if (serialAvailable) {
    Serial.println(F("========================================"));
    Serial.println(F("  DRONE REMOTE CONTROLLER - BOOT"));
    Serial.println(F("========================================"));
  }
  
  // Configure pins
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(SWITCH_ALTHOLD_PIN, INPUT_PULLUP);
  pinMode(SWITCH_ARM_PIN, INPUT_PULLUP);
  
  // Initialize NRF24L01
  if (serialAvailable) Serial.println(F("Initializing NRF24L01..."));
  
  if (!radio.begin()) {
    if (serialAvailable) {
      Serial.println(F("ERROR: NRF24L01 initialization FAILED!"));
      Serial.println(F("Check wiring:"));
      Serial.println(F("  CE  -> D9"));
      Serial.println(F("  CSN -> D10"));
      Serial.println(F("  MOSI -> D11"));
      Serial.println(F("  MISO -> D12"));
      Serial.println(F("  SCK -> D13"));
    }
    while (1); // Halt
  }
  
  // Configure radio
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(103);
  radio.setRetries(5, 5);
  radio.enableAckPayload();
  radio.openWritingPipe(txAddress);
  radio.openReadingPipe(1, rxAddress);
  radio.startListening();
  
  if (serialAvailable) {
    Serial.println(F("NRF24L01 initialized successfully"));
    Serial.println(F("Channel: 103"));
    Serial.println(F("Data Rate: 250KBPS"));
    Serial.println(F("PA Level: MAX"));
  }
  
  // Calibrate joysticks (find center points)
  if (serialAvailable) Serial.println(F("\nCalibrating joysticks..."));
  calibrateJoysticks();
  
  if (serialAvailable) {
    Serial.println(F("Joystick calibration complete"));
    Serial.println(F("\n========================================"));
    Serial.println(F("  SYSTEM READY"));
    Serial.println(F("========================================"));
    Serial.println();
    displayStatus();
  }
  
  lastSendTime = millis();
  lastDisplayUpdate = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long currentTime = millis();
  
  // Update RC data from inputs
  updateRCData();
  
  // Transmit RC data at fixed rate (50Hz)
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentTime;
    
    rcData.timestamp = millis();
    
    radio.stopListening();
    bool success = radio.write(&rcData, sizeof(RC_Data));
    radio.startListening();
    
    if (success) {
      linkActive = true;
    }
    
    // Check for timeout
    if (currentTime - lastTelemetryReceived > LINK_TIMEOUT) {
      linkActive = false;
    }
  }
  
  // Receive telemetry from FC
  if (radio.available()) {
    radio.read(&telemetry, sizeof(FC_Telemetry));
    lastTelemetryReceived = millis();
    linkActive = true;
  }
  
  // Update display (if serial available)
  if (serialAvailable && (currentTime - lastDisplayUpdate >= DISPLAY_INTERVAL)) {
    lastDisplayUpdate = currentTime;
    displayStatus();
  }
}

// ============================================================================
// JOYSTICK CALIBRATION
// ============================================================================
void calibrateJoysticks() {
  // Read center positions (assumes sticks are centered)
  delay(500); // Wait for stable readings
  
  int samples = 50;
  long sumThrottle = 0, sumYaw = 0, sumPitch = 0, sumRoll = 0;
  
  for (int i = 0; i < samples; i++) {
    sumThrottle += analogRead(JOYSTICK_THROTTLE_PIN);
    sumYaw      += analogRead(JOYSTICK_YAW_PIN);
    sumPitch    += analogRead(JOYSTICK_PITCH_PIN);
    sumRoll     += analogRead(JOYSTICK_ROLL_PIN);
    delay(10);
  }
  
  joyYaw.center   = sumYaw / samples;
  joyPitch.center = sumPitch / samples;
  joyRoll.center  = sumRoll / samples;
  
  // Throttle doesn't have a center (it's a slider)
  joyThrottle.center = 512;
  
  // Initialize filters
  filterThrottle.filtered = analogRead(JOYSTICK_THROTTLE_PIN);
  filterYaw.filtered      = joyYaw.center;
  filterPitch.filtered    = joyPitch.center;
  filterRoll.filtered     = joyRoll.center;
  
  if (serialAvailable) {
    Serial.print(F("  Yaw center: "));    Serial.println(joyYaw.center);
    Serial.print(F("  Pitch center: ")); Serial.println(joyPitch.center);
    Serial.print(F("  Roll center: "));  Serial.println(joyRoll.center);
  }
}

// ============================================================================
// EXPONENTIAL FILTER
// ============================================================================
float applyExpoFilter(ExpoFilter &filter, int rawValue) {
  filter.filtered = filter.alpha * rawValue + (1.0 - filter.alpha) * filter.filtered;
  return filter.filtered;
}

// ============================================================================
// READ JOYSTICK WITH CALIBRATION
// ============================================================================
int readJoystick(int pin, JoystickCalibration &cal, ExpoFilter &filter) {
  int raw = analogRead(pin);
  float filtered = applyExpoFilter(filter, raw);
  
  int value;
  
  if (pin == JOYSTICK_THROTTLE_PIN) {
    // Throttle: full range mapping (no center, no deadzone on edges)
    value = map(filtered, cal.min, cal.max, 1000, 2000);
  } else {
    // Other axes: apply deadzone around center
    if (abs(filtered - cal.center) < cal.deadzone) {
      filtered = cal.center;
    }
    
    if (filtered < cal.center) {
      value = map(filtered, cal.min, cal.center, 1000, 1500);
    } else {
      value = map(filtered, cal.center, cal.max, 1500, 2000);
    }
  }
  
  return constrain(value, 1000, 2000);
}

// ============================================================================
// READ BUTTON WITH DEBOUNCING
// ============================================================================
bool readButton(ButtonState &btn) {
  int reading = digitalRead(btn.pin);
  
  if (reading != btn.lastState) {
    btn.lastDebounceTime = millis();
  }
  
  if ((millis() - btn.lastDebounceTime) > btn.debounceDelay) {
    if (reading != btn.currentState) {
      btn.currentState = reading;
    }
  }
  
  btn.lastState = reading;
  
  // Return true if button is pressed (active LOW)
  return (btn.currentState == LOW);
}

// ============================================================================
// READ SWITCH
// ============================================================================
bool readSwitch(int pin) {
  // Switches are active LOW (pressed = LOW)
  return (digitalRead(pin) == LOW);
}

// ============================================================================
// UPDATE RC DATA
// ============================================================================
void updateRCData() {
  // Read joysticks
  rcData.throttle = readJoystick(JOYSTICK_THROTTLE_PIN, joyThrottle, filterThrottle);
  rcData.yaw      = readJoystick(JOYSTICK_YAW_PIN, joyYaw, filterYaw);
  rcData.pitch    = readJoystick(JOYSTICK_PITCH_PIN, joyPitch, filterPitch);
  rcData.roll     = readJoystick(JOYSTICK_ROLL_PIN, joyRoll, filterRoll);
  
  // Read switches
  rcData.armed   = readSwitch(SWITCH_ARM_PIN) ? 1 : 0;
  rcData.altHold = readSwitch(SWITCH_ALTHOLD_PIN) ? 1 : 0;
  
  // Read buttons
  rcData.button1 = readButton(button1) ? 1 : 0;
  rcData.button2 = readButton(button2) ? 1 : 0;
}

// ============================================================================
// DISPLAY STATUS ON SERIAL MONITOR
// ============================================================================
void displayStatus() {
  // Clear screen (ANSI escape code)
  Serial.write(27);       // ESC
  Serial.print(F("[2J")); // Clear screen
  Serial.write(27);       // ESC
  Serial.print(F("[H"));  // Home cursor
  
  Serial.println(F("╔════════════════════════════════════════════════════════════════╗"));
  Serial.println(F("║          DRONE REMOTE CONTROLLER - STATUS DISPLAY             ║"));
  Serial.println(F("╠════════════════════════════════════════════════════════════════╣"));
  
  // Link status
  Serial.print(F("║ LINK STATUS:  "));
  if (linkActive) {
    Serial.print(F("✓ CONNECTED    "));
  } else {
    Serial.print(F("✗ NO LINK      "));
  }
  Serial.println(F("                                  ║"));
  
  // Calibration status
  Serial.print(F("║ CALIBRATION:  "));
  if (telemetry.calibrated) {
    Serial.print(F("✓ COMPLETE     "));
  } else {
    Serial.print(F("✗ NOT CALIBRATED"));
  }
  Serial.println(F("                                 ║"));
  
  Serial.println(F("╠════════════════════════════════════════════════════════════════╣"));
  
  // Joystick values
  Serial.print(F("║ THROTTLE: "));
  Serial.print(rcData.throttle);
  Serial.print(F("    YAW: "));
  Serial.print(rcData.yaw);
  Serial.print(F("                              ║"));
  Serial.println();
  
  Serial.print(F("║ PITCH:    "));
  Serial.print(rcData.pitch);
  Serial.print(F("    ROLL: "));
  Serial.print(rcData.roll);
  Serial.print(F("                             ║"));
  Serial.println();
  
  Serial.println(F("╠════════════════════════════════════════════════════════════════╣"));
  
  // Switches
  Serial.print(F("║ ARM SWITCH (SW_2):      "));
  Serial.print(rcData.armed ? F("[ARMED]  ") : F("[DISARMED]"));
  Serial.println(F("                      ║"));
  
  Serial.print(F("║ ALT HOLD SWITCH (SW_1): "));
  Serial.print(rcData.altHold ? F("[ON]     ") : F("[OFF]    "));
  Serial.println(F("                      ║"));
  
  Serial.println(F("╠════════════════════════════════════════════════════════════════╣"));
  
  // Buttons
  Serial.print(F("║ BUTTON 1 (Calibration): "));
  Serial.print(rcData.button1 ? F("[PRESSED]") : F("[-------]"));
  Serial.println(F("                      ║"));
  
  Serial.print(F("║ BUTTON 2 (ESC Cal/Test): "));
  Serial.print(rcData.button2 ? F("[PRESSED]") : F("[-------]"));
  Serial.println(F("                    ║"));
  
  Serial.println(F("╠════════════════════════════════════════════════════════════════╣"));
  
  // Telemetry from FC
  if (linkActive) {
    Serial.print(F("║ ATTITUDE:  Roll: "));
    Serial.print(telemetry.roll, 1);
    Serial.print(F("°  Pitch: "));
    Serial.print(telemetry.pitch, 1);
    Serial.print(F("°  Yaw: "));
    Serial.print(telemetry.yaw, 1);
    Serial.print(F("°"));
    
    // Padding
    int len = 20 + String(telemetry.roll, 1).length() + 
              String(telemetry.pitch, 1).length() + 
              String(telemetry.yaw, 1).length() + 6;
    for (int i = len; i < 63; i++) Serial.print(F(" "));
    Serial.println(F("║"));
    
    Serial.print(F("║ ALTITUDE:  "));
    Serial.print(telemetry.altitude, 2);
    Serial.print(F(" m"));
    
    len = 12 + String(telemetry.altitude, 2).length() + 2;
    for (int i = len; i < 63; i++) Serial.print(F(" "));
    Serial.println(F("║"));
    
    Serial.print(F("║ FC LOOP TIME: "));
    Serial.print(telemetry.loopTime);
    Serial.print(F(" µs"));
    
    len = 15 + String(telemetry.loopTime).length() + 3;
    for (int i = len; i < 63; i++) Serial.print(F(" "));
    Serial.println(F("║"));
  } else {
    Serial.println(F("║ TELEMETRY:  Waiting for FC connection...                      ║"));
  }
  
  Serial.println(F("╠════════════════════════════════════════════════════════════════╣"));
  Serial.println(F("║ CONTROLS:                                                      ║"));
  Serial.println(F("║   Left Stick:  Throttle (V) | Yaw (H)                         ║"));
  Serial.println(F("║   Right Stick: Pitch (V) | Roll (H)                           ║"));
  Serial.println(F("║   SW_2 (D3):   ARM/DISARM Kill Switch                         ║"));
  Serial.println(F("║   SW_1 (D2):   Altitude Hold ON/OFF                           ║"));
  Serial.println(F("║   Button_1:    Trigger IMU Calibration                        ║"));
  Serial.println(F("║   Button_2:    ESC Calibration + Motor Test                   ║"));
  Serial.println(F("╚════════════════════════════════════════════════════════════════╝"));
  Serial.println();
}
