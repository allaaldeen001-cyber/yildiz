/*
 * Quadcopter Remote Control (RC)
 * Hardware: Arduino Nano, NRF24L01, 2x Joystick, Toggle Switch, 2x Push Button, Status LED
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pin Definitions
#define JOYSTICK1_X_PIN A0  // Left Stick X (Yaw)
#define JOYSTICK1_Y_PIN A1  // Left Stick Y (Throttle)
#define JOYSTICK2_X_PIN A2  // Right Stick X (Roll)
#define JOYSTICK2_Y_PIN A3  // Right Stick Y (Pitch)
#define TOGGLE_SWITCH_PIN 2
#define BUTTON1_PIN 4      // Calibration button
#define BUTTON2_PIN 5      // Motor start button
#define STATUS_LED_PIN 13

// NRF24L01 Setup
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

// Radio Control Data Structure (must match Flight Controller)
struct RCData {
  uint16_t throttle;  // 0-2000
  uint16_t yaw;       // 0-2000
  uint16_t pitch;     // 0-2000
  uint16_t roll;      // 0-2000
  bool button1;       // Calibration button
  bool button2;       // Motor start button
  bool toggleSwitch;  // Arm/Kill switch
  bool isConnected;
};

RCData rcData;

// Joystick Calibration Data
struct JoystickCal {
  uint16_t centerX;
  uint16_t centerY;
  uint16_t minX;
  uint16_t maxX;
  uint16_t minY;
  uint16_t maxY;
};

JoystickCal leftStickCal = {512, 512, 0, 1023, 0, 1023};
JoystickCal rightStickCal = {512, 512, 0, 1023, 0, 1023};

// Button state tracking
bool button1State = false;
bool button1LastState = false;
bool button2State = false;
bool button2LastState = false;
bool toggleLastState = false;

// Timing
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 4000; // 250Hz (4ms)

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== Quadcopter RC Controller ==="));
  
  // Initialize pins
  pinMode(JOYSTICK1_X_PIN, INPUT);
  pinMode(JOYSTICK1_Y_PIN, INPUT);
  pinMode(JOYSTICK2_X_PIN, INPUT);
  pinMode(JOYSTICK2_Y_PIN, INPUT);
  pinMode(TOGGLE_SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Initialize NRF24L01
  Serial.print(F("Initializing NRF24L01... "));
  if (radio.begin()) {
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    radio.stopListening();
    Serial.println(F("OK"));
    Serial.print(F("NRF Channel: "));
    Serial.println(radio.getChannel());
  } else {
    Serial.println(F("FAILED"));
  }
  
  // Calibrate joysticks
  Serial.println(F("\nCalibrating joysticks..."));
  Serial.println(F("Move joysticks to all extremes, then center them"));
  delay(2000);
  calibrateJoysticks();
  Serial.println(F("Joystick calibration complete"));
  
  // Initialize RC data
  rcData.throttle = 1000;
  rcData.yaw = 1000;
  rcData.pitch = 1000;
  rcData.roll = 1000;
  rcData.button1 = false;
  rcData.button2 = false;
  rcData.toggleSwitch = false;
  rcData.isConnected = true;
  
  Serial.println(F("\n=== RC Controller Ready ==="));
  Serial.println(F("Controls:"));
  Serial.println(F("  Left Stick: Throttle (Y) / Yaw (X)"));
  Serial.println(F("  Right Stick: Pitch (Y) / Roll (X)"));
  Serial.println(F("  Button 1: Calibration"));
  Serial.println(F("  Button 2: Motor Start"));
  Serial.println(F("  Toggle: Arm/Kill Switch"));
  
  // Blink LED to indicate ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
  }
}

void loop() {
  unsigned long currentTime = micros();
  
  // Read inputs at 250Hz
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentTime;
    
    // Read joysticks
    readJoysticks();
    
    // Read buttons
    readButtons();
    
    // Read toggle switch
    readToggleSwitch();
    
    // Send data to flight controller
    sendRadioData();
    
    // Update status LED
    updateStatusLED();
  }
}

void calibrateJoysticks() {
  // Find min/max/center for each joystick
  uint16_t leftXMin = 1023, leftXMax = 0, leftYMin = 1023, leftYMax = 0;
  uint16_t rightXMin = 1023, rightXMax = 0, rightYMin = 1023, rightYMax = 0;
  
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) { // 5 seconds to move joysticks
    uint16_t leftX = analogRead(JOYSTICK1_X_PIN);
    uint16_t leftY = analogRead(JOYSTICK1_Y_PIN);
    uint16_t rightX = analogRead(JOYSTICK2_X_PIN);
    uint16_t rightY = analogRead(JOYSTICK2_Y_PIN);
    
    if (leftX < leftXMin) leftXMin = leftX;
    if (leftX > leftXMax) leftXMax = leftX;
    if (leftY < leftYMin) leftYMin = leftY;
    if (leftY > leftYMax) leftYMax = leftY;
    
    if (rightX < rightXMin) rightXMin = rightX;
    if (rightX > rightXMax) rightXMax = rightX;
    if (rightY < rightYMin) rightYMin = rightY;
    if (rightY > rightYMax) rightYMax = rightY;
    
    delay(10);
  }
  
  // Wait for joysticks to center
  Serial.println(F("Center joysticks now..."));
  delay(3000);
  
  // Read center positions
  leftStickCal.centerX = analogRead(JOYSTICK1_X_PIN);
  leftStickCal.centerY = analogRead(JOYSTICK1_Y_PIN);
  rightStickCal.centerX = analogRead(JOYSTICK2_X_PIN);
  rightStickCal.centerY = analogRead(JOYSTICK2_Y_PIN);
  
  // Set ranges with some margin
  leftStickCal.minX = leftXMin;
  leftStickCal.maxX = leftXMax;
  leftStickCal.minY = leftYMin;
  leftStickCal.maxY = leftYMax;
  
  rightStickCal.minX = rightXMin;
  rightStickCal.maxX = rightXMax;
  rightStickCal.minY = rightYMin;
  rightStickCal.maxY = rightYMax;
  
  Serial.print(F("Left Stick Center: X="));
  Serial.print(leftStickCal.centerX);
  Serial.print(F(" Y="));
  Serial.println(leftStickCal.centerY);
  Serial.print(F("Right Stick Center: X="));
  Serial.print(rightStickCal.centerX);
  Serial.print(F(" Y="));
  Serial.println(rightStickCal.centerY);
}

void readJoysticks() {
  // Read raw values
  uint16_t leftXRaw = analogRead(JOYSTICK1_X_PIN);
  uint16_t leftYRaw = analogRead(JOYSTICK1_Y_PIN);
  uint16_t rightXRaw = analogRead(JOYSTICK2_X_PIN);
  uint16_t rightYRaw = analogRead(JOYSTICK2_Y_PIN);
  
  // Map to 1000-2000 range with dead zone around center
  // IMPORTANT: Center position = 1000 (minimum throttle), not 1500!
  // This solves the dangerous throttle issue
  
  // Left Stick: Y = Throttle, X = Yaw
  // Throttle: Center (1000) = no throttle, Up (2000) = max throttle
  uint16_t throttleRaw = leftYRaw;
  if (throttleRaw < leftStickCal.centerY) {
    // Below center = minimum throttle (1000)
    rcData.throttle = 1000;
  } else {
    // Above center: map from center to max = 1000 to 2000
    throttleRaw = constrain(throttleRaw, leftStickCal.centerY, leftStickCal.maxY);
    rcData.throttle = map(throttleRaw, leftStickCal.centerY, leftStickCal.maxY, 1000, 2000);
  }
  
  // Yaw: Center = 1000 (no rotation), Left/Right = 1000-2000
  int16_t yawOffset = (int16_t)leftXRaw - (int16_t)leftStickCal.centerX;
  if (abs(yawOffset) < 10) { // Dead zone
    rcData.yaw = 1000;
  } else {
    if (yawOffset < 0) {
      // Left: 1000 to 500 (reverse)
      rcData.yaw = map(leftXRaw, leftStickCal.minX, leftStickCal.centerX, 500, 1000);
    } else {
      // Right: 1000 to 2000
      rcData.yaw = map(leftXRaw, leftStickCal.centerX, leftStickCal.maxX, 1000, 2000);
    }
  }
  
  // Right Stick: Y = Pitch, X = Roll
  // Pitch: Center = 1000 (level), Up/Down = 1000-2000
  int16_t pitchOffset = (int16_t)rightYRaw - (int16_t)rightStickCal.centerY;
  if (abs(pitchOffset) < 10) { // Dead zone
    rcData.pitch = 1000;
  } else {
    if (pitchOffset < 0) {
      // Forward (stick up): 1000 to 2000
      rcData.pitch = map(rightYRaw, rightStickCal.minY, rightStickCal.centerY, 2000, 1000);
    } else {
      // Backward (stick down): 1000 to 500
      rcData.pitch = map(rightYRaw, rightStickCal.centerY, rightStickCal.maxY, 1000, 500);
    }
  }
  
  // Roll: Center = 1000 (level), Left/Right = 1000-2000
  int16_t rollOffset = (int16_t)rightXRaw - (int16_t)rightStickCal.centerX;
  if (abs(rollOffset) < 10) { // Dead zone
    rcData.roll = 1000;
  } else {
    if (rollOffset < 0) {
      // Left: 1000 to 500
      rcData.roll = map(rightXRaw, rightStickCal.minX, rightStickCal.centerX, 500, 1000);
    } else {
      // Right: 1000 to 2000
      rcData.roll = map(rightXRaw, rightStickCal.centerX, rightStickCal.maxX, 1000, 2000);
    }
  }
  
  // Ensure values are in valid range
  rcData.throttle = constrain(rcData.throttle, 1000, 2000);
  rcData.yaw = constrain(rcData.yaw, 500, 2000);
  rcData.pitch = constrain(rcData.pitch, 500, 2000);
  rcData.roll = constrain(rcData.roll, 500, 2000);
}

void readButtons() {
  // Read button states (inverted because of INPUT_PULLUP)
  button1State = !digitalRead(BUTTON1_PIN);
  button2State = !digitalRead(BUTTON2_PIN);
  
  // Detect button press (edge detection)
  if (button1State && !button1LastState) {
    rcData.button1 = true;
    digitalWrite(STATUS_LED_PIN, HIGH); // Blink on press
    delay(50);
    digitalWrite(STATUS_LED_PIN, LOW);
  } else {
    rcData.button1 = false;
  }
  
  if (button2State && !button2LastState) {
    rcData.button2 = true;
    digitalWrite(STATUS_LED_PIN, HIGH); // Blink on press
    delay(50);
    digitalWrite(STATUS_LED_PIN, LOW);
  } else {
    rcData.button2 = false;
  }
  
  button1LastState = button1State;
  button2LastState = button2State;
}

void readToggleSwitch() {
  // Toggle switch: LOW = Kill, HIGH = Arm (inverted because of INPUT_PULLUP)
  rcData.toggleSwitch = !digitalRead(TOGGLE_SWITCH_PIN);
  
  // Blink LED when state changes
  if (rcData.toggleSwitch != toggleLastState) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    toggleLastState = rcData.toggleSwitch;
  }
}

void sendRadioData() {
  rcData.isConnected = true;
  bool success = radio.write(&rcData, sizeof(RCData));
  
  if (!success) {
    // Retry once
    delayMicroseconds(100);
    radio.write(&rcData, sizeof(RCData));
  }
}

void updateStatusLED() {
  // Blink LED periodically to show operation
  static unsigned long lastBlinkTime = 0;
  static bool ledState = false;
  
  if (millis() - lastBlinkTime > 1000) { // Blink every second
    ledState = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState);
    lastBlinkTime = millis();
  }
}
