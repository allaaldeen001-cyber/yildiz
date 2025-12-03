/*
 * ============================================================================
 * QUADCOPTER REMOTE CONTROLLER
 * ============================================================================
 * Target: Arduino Nano (ATmega328P)
 * 
 * WIRING TABLE:
 * ============================================================================
 * COMPONENT            | PIN    | NOTES
 * ---------------------|--------|----------------------------------------
 * Throttle Stick (V)   | A0     | Left stick vertical
 * Yaw Stick (H)        | A1     | Left stick horizontal
 * Pitch Stick (V)      | A2     | Right stick vertical
 * Roll Stick (H)       | A3     | Right stick horizontal
 * Arm Switch           | D2     | Toggle switch (active LOW)
 * Mode Switch          | D3     | Toggle switch (active LOW)
 * NRF24L01 CE          | D9     | Radio Chip Enable
 * NRF24L01 CSN         | D10    | Radio Chip Select
 * NRF24L01 MOSI        | D11    | SPI
 * NRF24L01 MISO        | D12    | SPI
 * NRF24L01 SCK         | D13    | SPI
 * LED Status           | D4     | Status indicator
 * Buzzer (optional)    | D5     | Link loss alarm
 * 
 * CONTROLS:
 * ============================================================================
 * Left Stick Vertical: Throttle (1000-2000)
 * Left Stick Horizontal: Yaw (-500 to +500)
 * Right Stick Vertical: Pitch (-500 to +500)
 * Right Stick Horizontal: Roll (-500 to +500)
 * 
 * Switch 1 (D2): ARM/DISARM
 *   - LOW (switch ON) = Armed
 *   - HIGH (switch OFF) = Disarmed
 * 
 * Switch 2 (D3): Flight Mode
 *   - LOW (switch ON) = Altitude Hold
 *   - HIGH (switch OFF) = Stabilize
 * 
 * LIBRARIES REQUIRED:
 * - SPI (built-in)
 * - RF24 (https://github.com/nRF24/RF24)
 * 
 * ============================================================================
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define STICK_THROTTLE      A0
#define STICK_YAW           A1
#define STICK_PITCH         A2
#define STICK_ROLL          A3

#define SWITCH_ARM          2
#define SWITCH_MODE         3

#define LED_PIN             4
#define BUZZER_PIN          5

#define NRF_CE_PIN          9
#define NRF_CSN_PIN         10

// ============================================================================
// CONFIGURATION
// ============================================================================
#define TRANSMIT_RATE_HZ    50        // 50Hz transmission
#define TRANSMIT_INTERVAL   (1000 / TRANSMIT_RATE_HZ)

#define STICK_DEADZONE      20        // ADC units
#define FILTER_ALPHA        0.3       // Low-pass filter coefficient

// ============================================================================
// DATA STRUCTURES
// ============================================================================
struct RadioPacket {
  uint16_t throttle;    // 1000-2000
  int16_t  roll;        // -500 to +500
  int16_t  pitch;       // -500 to +500
  int16_t  yaw;         // -500 to +500
  uint8_t  armSwitch;   // 0=disarmed, 1=armed
  uint8_t  modeSwitch;  // 0=stabilize, 1=altitude hold
  uint32_t timestamp;   // For timeout detection
};

struct TelemetryPacket {
  float    roll;        // degrees
  float    pitch;       // degrees
  float    yaw;         // degrees
  float    altitude;    // meters
  float    battery;     // volts
  uint8_t  flightMode;  // 0=disarmed, 1=stabilize, 2=alt_hold, 3=landing
  uint16_t loopTime;    // microseconds
};

// ============================================================================
// GLOBAL OBJECTS AND VARIABLES
// ============================================================================
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const uint64_t radioAddress = 0xF0F0F0F0E1LL;

RadioPacket txData = {1000, 0, 0, 0, 0, 0, 0};
TelemetryPacket rxTelemetry = {0, 0, 0, 0, 0, 0, 0};

// Joystick calibration
struct StickCalibration {
  int center;
  int min;
  int max;
};

StickCalibration calThrottle = {512, 0, 1023};
StickCalibration calYaw      = {512, 0, 1023};
StickCalibration calPitch    = {512, 0, 1023};
StickCalibration calRoll     = {512, 0, 1023};

// Filtered stick values
float filteredThrottle = 512;
float filteredYaw = 512;
float filteredPitch = 512;
float filteredRoll = 512;

// Timing
unsigned long lastTransmit = 0;
unsigned long lastTelemetry = 0;
unsigned long lastLinkCheck = 0;

bool linkActive = false;
bool telemetryReceived = false;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void initRadio();
void calibrateSticks();
void readSticks();
void transmitData();
void receiveTelemetry();
void checkLink();
void updateLED();
float applyLowPassFilter(float current, float new_value, float alpha);
int mapStick(int raw, StickCalibration &cal, bool isThrottle);
void displayStatus();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SWITCH_ARM, INPUT_PULLUP);
  pinMode(SWITCH_MODE, INPUT_PULLUP);
  
  Serial.println(F("============================================"));
  Serial.println(F("  QUADCOPTER REMOTE CONTROLLER v2.0"));
  Serial.println(F("============================================"));
  
  // Startup beep
  tone(BUZZER_PIN, 1500, 100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  tone(BUZZER_PIN, 2000, 100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize radio
  initRadio();
  
  // Calibrate sticks
  calibrateSticks();
  
  Serial.println(F("System Ready"));
  Serial.println(F("============================================"));
  Serial.println(F("Controls:"));
  Serial.println(F("  Left Stick V:  Throttle"));
  Serial.println(F("  Left Stick H:  Yaw"));
  Serial.println(F("  Right Stick V: Pitch"));
  Serial.println(F("  Right Stick H: Roll"));
  Serial.println(F("  Switch 1 (D2): ARM/DISARM"));
  Serial.println(F("  Switch 2 (D3): Stabilize/Altitude Hold"));
  Serial.println(F("============================================"));
  
  tone(BUZZER_PIN, 2500, 200);
  
  lastTransmit = millis();
  lastLinkCheck = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();
  
  // Read stick inputs
  readSticks();
  
  // Transmit at fixed rate
  if (now - lastTransmit >= TRANSMIT_INTERVAL) {
    lastTransmit = now;
    transmitData();
  }
  
  // Check for telemetry
  receiveTelemetry();
  
  // Check link status
  if (now - lastLinkCheck >= 1000) {
    lastLinkCheck = now;
    checkLink();
  }
  
  // Update status LED
  updateLED();
  
  // Display status (every 200ms)
  static unsigned long lastDisplay = 0;
  if (now - lastDisplay >= 200) {
    lastDisplay = now;
    displayStatus();
  }
}

// ============================================================================
// RADIO INITIALIZATION
// ============================================================================
void initRadio() {
  Serial.println(F("Initializing NRF24L01..."));
  
  if (!radio.begin()) {
    Serial.println(F("ERROR: NRF24L01 initialization failed!"));
    Serial.println(F("Check wiring:"));
    Serial.println(F("  CE  -> D9"));
    Serial.println(F("  CSN -> D10"));
    Serial.println(F("  MOSI -> D11"));
    Serial.println(F("  MISO -> D12"));
    Serial.println(F("  SCK -> D13"));
    
    // Error indication
    while(1) {
      tone(BUZZER_PIN, 500, 200);
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }
  
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(103);
  radio.setRetries(3, 5);
  radio.openWritingPipe(radioAddress);
  radio.stopListening();
  
  Serial.println(F("NRF24L01 initialized"));
  Serial.println(F("  Channel: 103"));
  Serial.println(F("  Data Rate: 250KBPS"));
  Serial.println(F("  PA Level: MAX"));
}

// ============================================================================
// STICK CALIBRATION
// ============================================================================
void calibrateSticks() {
  Serial.println(F("Calibrating sticks..."));
  Serial.println(F("Center all sticks and hold for 2 seconds..."));
  
  delay(2000);
  
  int samples = 100;
  long sumYaw = 0, sumPitch = 0, sumRoll = 0;
  
  for (int i = 0; i < samples; i++) {
    sumYaw += analogRead(STICK_YAW);
    sumPitch += analogRead(STICK_PITCH);
    sumRoll += analogRead(STICK_ROLL);
    delay(10);
  }
  
  calYaw.center = sumYaw / samples;
  calPitch.center = sumPitch / samples;
  calRoll.center = sumRoll / samples;
  
  // Initialize filters
  filteredThrottle = analogRead(STICK_THROTTLE);
  filteredYaw = calYaw.center;
  filteredPitch = calPitch.center;
  filteredRoll = calRoll.center;
  
  Serial.println(F("Calibration complete:"));
  Serial.print(F("  Yaw center: ")); Serial.println(calYaw.center);
  Serial.print(F("  Pitch center: ")); Serial.println(calPitch.center);
  Serial.print(F("  Roll center: ")); Serial.println(calRoll.center);
  
  tone(BUZZER_PIN, 2000, 200);
}

// ============================================================================
// READ STICKS
// ============================================================================
void readSticks() {
  // Read raw values
  int rawThrottle = analogRead(STICK_THROTTLE);
  int rawYaw = analogRead(STICK_YAW);
  int rawPitch = analogRead(STICK_PITCH);
  int rawRoll = analogRead(STICK_ROLL);
  
  // Apply low-pass filter
  filteredThrottle = applyLowPassFilter(filteredThrottle, rawThrottle, FILTER_ALPHA);
  filteredYaw = applyLowPassFilter(filteredYaw, rawYaw, FILTER_ALPHA);
  filteredPitch = applyLowPassFilter(filteredPitch, rawPitch, FILTER_ALPHA);
  filteredRoll = applyLowPassFilter(filteredRoll, rawRoll, FILTER_ALPHA);
  
  // Map to output ranges
  txData.throttle = mapStick((int)filteredThrottle, calThrottle, true);
  txData.yaw = mapStick((int)filteredYaw, calYaw, false);
  txData.pitch = mapStick((int)filteredPitch, calPitch, false);
  txData.roll = mapStick((int)filteredRoll, calRoll, false);
  
  // Read switches (active LOW)
  txData.armSwitch = (digitalRead(SWITCH_ARM) == LOW) ? 1 : 0;
  txData.modeSwitch = (digitalRead(SWITCH_MODE) == LOW) ? 1 : 0;
  
  // Update timestamp
  txData.timestamp = millis();
}

// ============================================================================
// MAP STICK
// ============================================================================
int mapStick(int raw, StickCalibration &cal, bool isThrottle) {
  if (isThrottle) {
    // Throttle: 0-1023 -> 1000-2000
    return map(raw, cal.min, cal.max, 1000, 2000);
  } else {
    // Other axes with center and deadzone
    if (abs(raw - cal.center) < STICK_DEADZONE) {
      return 0;
    }
    
    if (raw < cal.center) {
      return map(raw, cal.min, cal.center, -500, 0);
    } else {
      return map(raw, cal.center, cal.max, 0, 500);
    }
  }
}

// ============================================================================
// LOW-PASS FILTER
// ============================================================================
float applyLowPassFilter(float current, float new_value, float alpha) {
  return alpha * new_value + (1.0 - alpha) * current;
}

// ============================================================================
// TRANSMIT DATA
// ============================================================================
void transmitData() {
  bool success = radio.write(&txData, sizeof(RadioPacket));
  
  if (success) {
    linkActive = true;
    
    // Check if there's an ACK payload (telemetry)
    if (radio.available()) {
      radio.read(&rxTelemetry, sizeof(TelemetryPacket));
      telemetryReceived = true;
      lastTelemetry = millis();
    }
  } else {
    linkActive = false;
  }
}

// ============================================================================
// RECEIVE TELEMETRY
// ============================================================================
void receiveTelemetry() {
  // Telemetry is received via ACK payload in transmitData()
  // This function is kept for future expansion
}

// ============================================================================
// CHECK LINK STATUS
// ============================================================================
void checkLink() {
  if (millis() - lastTelemetry > 2000) {
    linkActive = false;
    
    // Alarm if armed
    if (txData.armSwitch == 1) {
      tone(BUZZER_PIN, 500, 100);
      Serial.println(F("WARNING: Link lost while armed!"));
    }
  }
}

// ============================================================================
// UPDATE LED
// ============================================================================
void updateLED() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  
  if (linkActive) {
    // Solid on when link is good
    digitalWrite(LED_PIN, HIGH);
  } else {
    // Fast blink when no link
    if (millis() - lastBlink > 200) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlink = millis();
    }
  }
}

// ============================================================================
// DISPLAY STATUS
// ============================================================================
void displayStatus() {
  // Clear screen
  Serial.write(27);
  Serial.print(F("[2J"));
  Serial.write(27);
  Serial.print(F("[H"));
  
  Serial.println(F("========================================"));
  Serial.println(F("  REMOTE CONTROLLER STATUS"));
  Serial.println(F("========================================"));
  
  // Link status
  Serial.print(F("Link: "));
  if (linkActive) {
    Serial.println(F("CONNECTED"));
  } else {
    Serial.println(F("NO LINK"));
  }
  
  Serial.println(F("----------------------------------------"));
  
  // Control inputs
  Serial.print(F("Throttle: ")); Serial.print(txData.throttle); Serial.println(F(" us"));
  Serial.print(F("Roll:     ")); Serial.println(txData.roll);
  Serial.print(F("Pitch:    ")); Serial.println(txData.pitch);
  Serial.print(F("Yaw:      ")); Serial.println(txData.yaw);
  
  Serial.println(F("----------------------------------------"));
  
  // Switches
  Serial.print(F("ARM Switch:  "));
  if (txData.armSwitch) {
    Serial.println(F("ARMED"));
  } else {
    Serial.println(F("DISARMED"));
  }
  
  Serial.print(F("Mode Switch: "));
  if (txData.modeSwitch) {
    Serial.println(F("ALT HOLD"));
  } else {
    Serial.println(F("STABILIZE"));
  }
  
  Serial.println(F("----------------------------------------"));
  
  // Telemetry
  if (telemetryReceived) {
    Serial.println(F("Flight Controller Telemetry:"));
    Serial.print(F("  Roll:     ")); Serial.print(rxTelemetry.roll, 1); Serial.println(F(" deg"));
    Serial.print(F("  Pitch:    ")); Serial.print(rxTelemetry.pitch, 1); Serial.println(F(" deg"));
    Serial.print(F("  Yaw:      ")); Serial.print(rxTelemetry.yaw, 1); Serial.println(F(" deg"));
    Serial.print(F("  Altitude: ")); Serial.print(rxTelemetry.altitude, 2); Serial.println(F(" m"));
    Serial.print(F("  Battery:  ")); Serial.print(rxTelemetry.battery, 1); Serial.println(F(" V"));
    
    Serial.print(F("  State:    "));
    switch(rxTelemetry.flightMode) {
      case 0: Serial.println(F("DISARMED")); break;
      case 1: Serial.println(F("STABILIZE")); break;
      case 2: Serial.println(F("ALT HOLD")); break;
      case 3: Serial.println(F("LANDING")); break;
      default: Serial.println(F("UNKNOWN")); break;
    }
    
    Serial.print(F("  Loop:     ")); Serial.print(rxTelemetry.loopTime); Serial.println(F(" us"));
  } else {
    Serial.println(F("No telemetry received yet"));
  }
  
  Serial.println(F("========================================"));
}
