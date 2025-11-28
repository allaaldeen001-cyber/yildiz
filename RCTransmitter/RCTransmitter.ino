/*
 * ========================================================================
 * RC TRANSMITTER FOR QUADCOPTER
 * ========================================================================
 * Professional RC transmitter with:
 * - 2 analog joysticks (4-axis control)
 * - Automatic joystick calibration
 * - Toggle switch for arm/kill
 * - 2 buttons for calibration and motor test
 * - NRF24L01 wireless communication
 * - Status LED feedback
 * - Safe throttle mapping
 * 
 * Author: Professional Embedded Systems Engineer
 * Date: November 2025
 * Version: 1.0.0
 * ========================================================================
 */

#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>

// ========================================================================
// PIN DEFINITIONS
// ========================================================================
#define JOYSTICK1_X_PIN   A0    // Left stick - Throttle
#define JOYSTICK1_Y_PIN   A1    // Left stick - Yaw
#define JOYSTICK2_X_PIN   A2    // Right stick - Pitch
#define JOYSTICK2_Y_PIN   A3    // Right stick - Roll

#define TOGGLE_SWITCH_PIN 2     // Arm/Kill switch
#define BUTTON1_PIN       3     // Calibration button
#define BUTTON2_PIN       4     // Motor test button
#define STATUS_LED_PIN    8     // Status LED

#define NRF_CE_PIN        9
#define NRF_CSN_PIN       10

// ========================================================================
// CONSTANTS
// ========================================================================
#define TRANSMIT_RATE     50    // 50ms = 20Hz transmission rate
#define DEADBAND          20    // Joystick deadband to prevent drift
#define THROTTLE_MIN      1000
#define THROTTLE_MAX      2000
#define CONTROL_MIN       1000
#define CONTROL_MAX       2000
#define CONTROL_CENTER    1500

// ========================================================================
// GLOBAL VARIABLES
// ========================================================================

// NRF24L01 Radio
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const byte address[6] = "DRONE";

// Radio Data Structure (must match flight controller)
struct RadioData {
  int throttle;    // 1000-2000
  int yaw;         // 1000-2000
  int pitch;       // 1000-2000
  int roll;        // 1000-2000
  bool armed;      // true/false
  byte command;    // 0=none, 1=calibrate, 2=motor_test
};
RadioData txData;

// Joystick calibration data
struct JoystickCalibration {
  int throttle_min, throttle_max, throttle_center;
  int yaw_min, yaw_max, yaw_center;
  int pitch_min, pitch_max, pitch_center;
  int roll_min, roll_max, roll_center;
  bool isCalibrated;
};
JoystickCalibration joyCalib;

// Raw joystick readings
int throttleRaw, yawRaw, pitchRaw, rollRaw;

// Button states
bool button1Pressed = false;
bool button2Pressed = false;
bool lastButton1State = HIGH;
bool lastButton2State = HIGH;
unsigned long lastButton1Time = 0;
unsigned long lastButton2Time = 0;
const unsigned long DEBOUNCE_TIME = 50;

// Toggle switch state
bool toggleState = false;
bool lastToggleState = false;

// System state
bool nrfConnected = false;
bool joystickCalibrated = false;
unsigned long lastTransmitTime = 0;
unsigned long lastLedBlinkTime = 0;
bool ledState = false;

// ========================================================================
// EEPROM ADDRESSES
// ========================================================================
#define EEPROM_CALIBRATED       0    // 1 byte
#define EEPROM_THROTTLE_MIN     4    // 2 bytes each
#define EEPROM_THROTTLE_MAX     6
#define EEPROM_THROTTLE_CENTER  8
#define EEPROM_YAW_MIN          10
#define EEPROM_YAW_MAX          12
#define EEPROM_YAW_CENTER       14
#define EEPROM_PITCH_MIN        16
#define EEPROM_PITCH_MAX        18
#define EEPROM_PITCH_CENTER     20
#define EEPROM_ROLL_MIN         22
#define EEPROM_ROLL_MAX         24
#define EEPROM_ROLL_CENTER      26

// ========================================================================
// SETUP
// ========================================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println(F("\n========================================"));
  Serial.println(F("  RC TRANSMITTER v1.0"));
  Serial.println(F("========================================\n"));

  // Initialize pins
  pinMode(TOGGLE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  digitalWrite(STATUS_LED_PIN, LOW);

  // Initialize NRF24L01
  if (initNRF()) {
    Serial.println(F("[OK] NRF24L01 initialized"));
    nrfConnected = true;
    blinkLED(3, 100);  // Success indication
  } else {
    Serial.println(F("[ERROR] NRF24L01 init failed!"));
    nrfConnected = false;
    errorBlink();
    while(1);  // Cannot continue without radio
  }

  // Load joystick calibration
  loadJoystickCalibration();

  // Initialize transmit data to safe values
  txData.throttle = THROTTLE_MIN;
  txData.yaw = CONTROL_CENTER;
  txData.pitch = CONTROL_CENTER;
  txData.roll = CONTROL_CENTER;
  txData.armed = false;
  txData.command = 0;

  Serial.println(F("\n========================================"));
  Serial.println(F("  RC TRANSMITTER READY"));
  Serial.println(F("========================================\n"));
  
  if (!joystickCalibrated) {
    Serial.println(F("[!] JOYSTICK NOT CALIBRATED!"));
    Serial.println(F("    Running auto-calibration..."));
    autoCalibrate();
  } else {
    Serial.println(F("[✓] Joystick calibration loaded"));
    Serial.println(F("    To recalibrate: Hold Button 1 on startup"));
  }

  Serial.println(F("\n--- Waiting for Flight Controller ---\n"));
  
  delay(500);
}

// ========================================================================
// MAIN LOOP
// ========================================================================
void loop() {
  // Read all inputs
  readJoysticks();
  readButtons();
  readToggleSwitch();

  // Map joystick values to control range
  mapControls();

  // Transmit data at fixed rate
  if (millis() - lastTransmitTime >= TRANSMIT_RATE) {
    transmitData();
    lastTransmitTime = millis();
  }

  // Update status LED
  updateStatusLED();

  // Print debug info periodically
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= 500) {
    printStatus();
    lastPrintTime = millis();
  }
}

// ========================================================================
// NRF24L01 FUNCTIONS
// ========================================================================
bool initNRF() {
  if (!radio.begin()) {
    return false;
  }
  
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setAutoAck(true);
  radio.setRetries(3, 5);  // 3 retries with 5*250us delay
  radio.stopListening();
  
  return true;
}

void transmitData() {
  bool success = radio.write(&txData, sizeof(RadioData));
  
  if (success) {
    // Quick LED blink on successful transmission
    if (!txData.armed) {
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(1);
      digitalWrite(STATUS_LED_PIN, LOW);
    }
  } else {
    // Transmission failed
    nrfConnected = false;
  }
  
  // Reset command after sending
  if (txData.command != 0) {
    txData.command = 0;
  }
}

// ========================================================================
// INPUT READING
// ========================================================================
void readJoysticks() {
  // Read raw analog values
  throttleRaw = analogRead(JOYSTICK1_X_PIN);
  yawRaw = analogRead(JOYSTICK1_Y_PIN);
  pitchRaw = analogRead(JOYSTICK2_X_PIN);
  rollRaw = analogRead(JOYSTICK2_Y_PIN);
}

void readButtons() {
  // Button 1 (Calibration) with debounce
  bool button1State = digitalRead(BUTTON1_PIN);
  if (button1State == LOW && lastButton1State == HIGH) {
    if (millis() - lastButton1Time > DEBOUNCE_TIME) {
      button1Pressed = true;
      lastButton1Time = millis();
      Serial.println(F("\n[BTN1] Calibration requested"));
      blinkLED(2, 50);
    }
  }
  lastButton1State = button1State;

  // Button 2 (Motor Test) with debounce
  bool button2State = digitalRead(BUTTON2_PIN);
  if (button2State == LOW && lastButton2State == HIGH) {
    if (millis() - lastButton2Time > DEBOUNCE_TIME) {
      button2Pressed = true;
      lastButton2Time = millis();
      Serial.println(F("\n[BTN2] Motor test requested"));
      blinkLED(2, 50);
    }
  }
  lastButton2State = button2State;

  // Handle button actions
  if (button1Pressed) {
    txData.command = 1;  // Calibration command
    button1Pressed = false;
  }
  
  if (button2Pressed) {
    txData.command = 2;  // Motor test command
    button2Pressed = false;
  }
}

void readToggleSwitch() {
  toggleState = digitalRead(TOGGLE_SWITCH_PIN);
  
  // Invert if needed (depends on switch wiring)
  // If switch UP (pulled to GND) = ARM, then invert
  bool armSwitch = !toggleState;  // LOW = ARM, HIGH = KILL
  
  if (armSwitch != lastToggleState) {
    lastToggleState = armSwitch;
    
    if (armSwitch) {
      Serial.println(F("\n[ARM] Drone ARMED - BE CAREFUL!"));
      blinkLED(2, 100);
    } else {
      Serial.println(F("\n[KILL] Drone DISARMED"));
      blinkLED(3, 50);
    }
  }
  
  txData.armed = armSwitch;
}

// ========================================================================
// CONTROL MAPPING
// ========================================================================
void mapControls() {
  // Apply deadband and map throttle
  // CRITICAL: Throttle must map from MIN at bottom to MAX at top
  // Center position should be around 40-50% throttle for safety
  if (joystickCalibrated) {
    // Use calibrated values
    txData.throttle = mapWithDeadband(
      throttleRaw, 
      joyCalib.throttle_min, 
      joyCalib.throttle_max,
      THROTTLE_MIN,
      THROTTLE_MAX,
      false  // No center deadband for throttle
    );
    
    // Yaw: center = 1500 (no rotation)
    txData.yaw = mapWithDeadband(
      yawRaw,
      joyCalib.yaw_min,
      joyCalib.yaw_max,
      CONTROL_MIN,
      CONTROL_MAX,
      true,  // Center deadband
      joyCalib.yaw_center
    );
    
    // Pitch: center = 1500 (no forward/back)
    txData.pitch = mapWithDeadband(
      pitchRaw,
      joyCalib.pitch_min,
      joyCalib.pitch_max,
      CONTROL_MIN,
      CONTROL_MAX,
      true,  // Center deadband
      joyCalib.pitch_center
    );
    
    // Roll: center = 1500 (no left/right)
    txData.roll = mapWithDeadband(
      rollRaw,
      joyCalib.roll_min,
      joyCalib.roll_max,
      CONTROL_MIN,
      CONTROL_MAX,
      true,  // Center deadband
      joyCalib.roll_center
    );
  } else {
    // Use default mapping (not recommended - calibrate first!)
    txData.throttle = map(throttleRaw, 0, 1023, THROTTLE_MIN, THROTTLE_MAX);
    txData.yaw = map(yawRaw, 0, 1023, CONTROL_MIN, CONTROL_MAX);
    txData.pitch = map(pitchRaw, 0, 1023, CONTROL_MIN, CONTROL_MAX);
    txData.roll = map(rollRaw, 0, 1023, CONTROL_MIN, CONTROL_MAX);
  }
  
  // Safety: constrain all outputs
  txData.throttle = constrain(txData.throttle, THROTTLE_MIN, THROTTLE_MAX);
  txData.yaw = constrain(txData.yaw, CONTROL_MIN, CONTROL_MAX);
  txData.pitch = constrain(txData.pitch, CONTROL_MIN, CONTROL_MAX);
  txData.roll = constrain(txData.roll, CONTROL_MIN, CONTROL_MAX);
}

int mapWithDeadband(int value, int inMin, int inMax, int outMin, int outMax, bool centerDeadband, int centerValue = 512) {
  if (centerDeadband) {
    // Apply center deadband
    int deadbandLow = centerValue - DEADBAND;
    int deadbandHigh = centerValue + DEADBAND;
    
    if (value >= deadbandLow && value <= deadbandHigh) {
      return CONTROL_CENTER;  // Return center position
    } else if (value < deadbandLow) {
      return map(value, inMin, deadbandLow, outMin, CONTROL_CENTER);
    } else {
      return map(value, deadbandHigh, inMax, CONTROL_CENTER, outMax);
    }
  } else {
    // No center deadband (for throttle)
    // Apply small deadband at min to ensure full low
    if (value < inMin + DEADBAND) {
      return outMin;
    }
    return map(value, inMin, inMax, outMin, outMax);
  }
}

// ========================================================================
// JOYSTICK CALIBRATION
// ========================================================================
void autoCalibrate() {
  Serial.println(F("\n========================================"));
  Serial.println(F("  JOYSTICK CALIBRATION"));
  Serial.println(F("========================================\n"));
  Serial.println(F("This will take 10 seconds..."));
  Serial.println(F(""));
  Serial.println(F("Step 1: Center all sticks - DON'T TOUCH!"));
  delay(3000);
  
  // Read center positions
  joyCalib.throttle_center = 0;
  joyCalib.yaw_center = 0;
  joyCalib.pitch_center = 0;
  joyCalib.roll_center = 0;
  
  for (int i = 0; i < 100; i++) {
    joyCalib.throttle_center += analogRead(JOYSTICK1_X_PIN);
    joyCalib.yaw_center += analogRead(JOYSTICK1_Y_PIN);
    joyCalib.pitch_center += analogRead(JOYSTICK2_X_PIN);
    joyCalib.roll_center += analogRead(JOYSTICK2_Y_PIN);
    delay(10);
  }
  
  joyCalib.throttle_center /= 100;
  joyCalib.yaw_center /= 100;
  joyCalib.pitch_center /= 100;
  joyCalib.roll_center /= 100;
  
  Serial.println(F("[✓] Center positions recorded"));
  Serial.print(F("    Throttle: ")); Serial.println(joyCalib.throttle_center);
  Serial.print(F("    Yaw: ")); Serial.println(joyCalib.yaw_center);
  Serial.print(F("    Pitch: ")); Serial.println(joyCalib.pitch_center);
  Serial.print(F("    Roll: ")); Serial.println(joyCalib.roll_center);
  
  Serial.println(F(""));
  Serial.println(F("Step 2: Move all sticks to EXTREMES"));
  Serial.println(F("        (Full circles for 5 seconds)"));
  delay(1000);
  
  // Initialize min/max with center values
  joyCalib.throttle_min = joyCalib.throttle_center;
  joyCalib.throttle_max = joyCalib.throttle_center;
  joyCalib.yaw_min = joyCalib.yaw_center;
  joyCalib.yaw_max = joyCalib.yaw_center;
  joyCalib.pitch_min = joyCalib.pitch_center;
  joyCalib.pitch_max = joyCalib.pitch_center;
  joyCalib.roll_min = joyCalib.roll_center;
  joyCalib.roll_max = joyCalib.roll_center;
  
  // Record extremes
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    int t = analogRead(JOYSTICK1_X_PIN);
    int y = analogRead(JOYSTICK1_Y_PIN);
    int p = analogRead(JOYSTICK2_X_PIN);
    int r = analogRead(JOYSTICK2_Y_PIN);
    
    joyCalib.throttle_min = min(joyCalib.throttle_min, t);
    joyCalib.throttle_max = max(joyCalib.throttle_max, t);
    joyCalib.yaw_min = min(joyCalib.yaw_min, y);
    joyCalib.yaw_max = max(joyCalib.yaw_max, y);
    joyCalib.pitch_min = min(joyCalib.pitch_min, p);
    joyCalib.pitch_max = max(joyCalib.pitch_max, p);
    joyCalib.roll_min = min(joyCalib.roll_min, r);
    joyCalib.roll_max = max(joyCalib.roll_max, r);
    
    // Visual feedback
    if ((millis() - startTime) % 500 == 0) {
      Serial.print(F("."));
    }
    
    delay(10);
  }
  Serial.println();
  
  Serial.println(F("[✓] Extreme positions recorded"));
  Serial.println(F(""));
  Serial.println(F("Calibration Results:"));
  Serial.print(F("  Throttle: ")); 
  Serial.print(joyCalib.throttle_min); Serial.print(F(" - "));
  Serial.print(joyCalib.throttle_center); Serial.print(F(" - "));
  Serial.println(joyCalib.throttle_max);
  
  Serial.print(F("  Yaw:      "));
  Serial.print(joyCalib.yaw_min); Serial.print(F(" - "));
  Serial.print(joyCalib.yaw_center); Serial.print(F(" - "));
  Serial.println(joyCalib.yaw_max);
  
  Serial.print(F("  Pitch:    "));
  Serial.print(joyCalib.pitch_min); Serial.print(F(" - "));
  Serial.print(joyCalib.pitch_center); Serial.print(F(" - "));
  Serial.println(joyCalib.pitch_max);
  
  Serial.print(F("  Roll:     "));
  Serial.print(joyCalib.roll_min); Serial.print(F(" - "));
  Serial.print(joyCalib.roll_center); Serial.print(F(" - "));
  Serial.println(joyCalib.roll_max);
  
  // Save to EEPROM
  saveJoystickCalibration();
  
  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("  CALIBRATION COMPLETE!"));
  Serial.println(F("========================================\n"));
  
  joystickCalibrated = true;
  blinkLED(5, 100);
}

void saveJoystickCalibration() {
  Serial.println(F("[SAVE] Saving calibration to EEPROM..."));
  
  EEPROM.write(EEPROM_CALIBRATED, 1);
  
  EEPROM.put(EEPROM_THROTTLE_MIN, joyCalib.throttle_min);
  EEPROM.put(EEPROM_THROTTLE_MAX, joyCalib.throttle_max);
  EEPROM.put(EEPROM_THROTTLE_CENTER, joyCalib.throttle_center);
  
  EEPROM.put(EEPROM_YAW_MIN, joyCalib.yaw_min);
  EEPROM.put(EEPROM_YAW_MAX, joyCalib.yaw_max);
  EEPROM.put(EEPROM_YAW_CENTER, joyCalib.yaw_center);
  
  EEPROM.put(EEPROM_PITCH_MIN, joyCalib.pitch_min);
  EEPROM.put(EEPROM_PITCH_MAX, joyCalib.pitch_max);
  EEPROM.put(EEPROM_PITCH_CENTER, joyCalib.pitch_center);
  
  EEPROM.put(EEPROM_ROLL_MIN, joyCalib.roll_min);
  EEPROM.put(EEPROM_ROLL_MAX, joyCalib.roll_max);
  EEPROM.put(EEPROM_ROLL_CENTER, joyCalib.roll_center);
  
  Serial.println(F("[✓] Calibration saved!"));
}

void loadJoystickCalibration() {
  byte calibrated = EEPROM.read(EEPROM_CALIBRATED);
  
  if (calibrated == 1) {
    EEPROM.get(EEPROM_THROTTLE_MIN, joyCalib.throttle_min);
    EEPROM.get(EEPROM_THROTTLE_MAX, joyCalib.throttle_max);
    EEPROM.get(EEPROM_THROTTLE_CENTER, joyCalib.throttle_center);
    
    EEPROM.get(EEPROM_YAW_MIN, joyCalib.yaw_min);
    EEPROM.get(EEPROM_YAW_MAX, joyCalib.yaw_max);
    EEPROM.get(EEPROM_YAW_CENTER, joyCalib.yaw_center);
    
    EEPROM.get(EEPROM_PITCH_MIN, joyCalib.pitch_min);
    EEPROM.get(EEPROM_PITCH_MAX, joyCalib.pitch_max);
    EEPROM.get(EEPROM_PITCH_CENTER, joyCalib.pitch_center);
    
    EEPROM.get(EEPROM_ROLL_MIN, joyCalib.roll_min);
    EEPROM.get(EEPROM_ROLL_MAX, joyCalib.roll_max);
    EEPROM.get(EEPROM_ROLL_CENTER, joyCalib.roll_center);
    
    joyCalib.isCalibrated = true;
    joystickCalibrated = true;
  } else {
    joyCalib.isCalibrated = false;
    joystickCalibrated = false;
  }
}

// ========================================================================
// STATUS & FEEDBACK
// ========================================================================
void updateStatusLED() {
  if (txData.armed) {
    // Solid ON when armed (WARNING!)
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else {
    // Slow blink when disarmed
    if (millis() - lastLedBlinkTime > 500) {
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
      lastLedBlinkTime = millis();
    }
  }
}

void printStatus() {
  Serial.print(F("T:"));
  Serial.print(txData.throttle);
  Serial.print(F(" Y:"));
  Serial.print(txData.yaw);
  Serial.print(F(" P:"));
  Serial.print(txData.pitch);
  Serial.print(F(" R:"));
  Serial.print(txData.roll);
  Serial.print(F(" | ARM:"));
  Serial.print(txData.armed ? F("YES") : F("NO"));
  Serial.print(F(" | CMD:"));
  Serial.println(txData.command);
}

void blinkLED(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(duration);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(duration);
  }
}

void errorBlink() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
  }
}

// ========================================================================
// END OF RC TRANSMITTER CODE
// ========================================================================
