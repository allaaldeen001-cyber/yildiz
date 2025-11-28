/*
 * Quadcopter Remote Control
 * Hardware:
 * - Arduino Nano
 * - NRF24L01 (Radio Module)
 * - 2x Joystick (Analog)
 * - Toggle Switch (Digital)
 * - 2x Push Button (Digital)
 * - Status LED
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pin Definitions
#define STATUS_LED 2
#define TOGGLE_SWITCH 3
#define BUTTON1 4
#define BUTTON2 5

// Joystick Pins
#define LEFT_JOYSTICK_X A0   // Yaw
#define LEFT_JOYSTICK_Y A1   // Throttle
#define RIGHT_JOYSTICK_X A2  // Roll
#define RIGHT_JOYSTICK_Y A3  // Pitch

// NRF24L01 Pins
#define CE_PIN 7
#define CSN_PIN 8

// Radio Setup
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// Data Structure for Radio Communication
struct RadioData {
  uint16_t throttle;  // 1000-2000 (center: 1500)
  uint16_t yaw;       // 1000-2000 (center: 1500)
  uint16_t pitch;     // 1000-2000 (center: 1500)
  uint16_t roll;      // 1000-2000 (center: 1500)
  bool button1;       // Calibration button
  bool button2;       // Motor start button
  bool toggleSwitch;  // Arming/Kill switch
  uint8_t channel;    // NRF channel number
};

RadioData txData;

// Joystick Calibration Values
// These will be calibrated on first run
int leftXMin = 0, leftXMax = 1023, leftXCenter = 512;
int leftYMin = 0, leftYMax = 1023, leftYCenter = 512;
int rightXMin = 0, rightXMax = 1023, rightXCenter = 512;
int rightYMin = 0, rightYMax = 1023, rightYCenter = 512;

bool calibrationMode = true;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_DURATION = 5000; // 5 seconds

void setup() {
  Serial.begin(9600);
  Serial.println("=== Quadcopter Remote Control ===");
  
  // Initialize Pins
  pinMode(STATUS_LED, OUTPUT);
  pinMode(TOGGLE_SWITCH, INPUT_PULLUP);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  
  // Initialize Joystick Pins (Analog - no setup needed)
  
  // Initialize Radio
  Serial.print("Initializing NRF24L01... ");
  if (radio.begin()) {
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    radio.stopListening();
    Serial.println("OK");
    Serial.print("Channel: ");
    Serial.println(radio.getChannel());
  } else {
    Serial.println("FAILED!");
    while(1) delay(10);
  }
  
  // Calibrate Joysticks
  Serial.println("\n=== Joystick Calibration ===");
  Serial.println("Please center all joysticks and keep them still for 5 seconds...");
  calibrateJoysticks();
  Serial.println("Calibration complete!");
  
  // Initialize data structure
  txData.throttle = 1500;  // Center position
  txData.yaw = 1500;
  txData.pitch = 1500;
  txData.roll = 1500;
  txData.button1 = false;
  txData.button2 = false;
  txData.toggleSwitch = false;
  txData.channel = radio.getChannel();
  
  Serial.println("\n=== Remote Control Ready ===");
  Serial.println("Controls:");
  Serial.println("  Left Stick:  Up/Down = Throttle, Left/Right = Yaw");
  Serial.println("  Right Stick: Up/Down = Pitch, Left/Right = Roll");
  Serial.println("  Toggle Switch: ARM/KILL");
  Serial.println("  Button 1: Calibration");
  Serial.println("  Button 2: Motor Start");
  Serial.println("\nStarting transmission...");
}

void loop() {
  // Read Joysticks
  readJoysticks();
  
  // Read Buttons and Switch
  txData.button1 = !digitalRead(BUTTON1);  // Inverted because of pull-up
  txData.button2 = !digitalRead(BUTTON2);
  txData.toggleSwitch = !digitalRead(TOGGLE_SWITCH);
  
  // Update channel number
  txData.channel = radio.getChannel();
  
  // Send Data
  bool result = radio.write(&txData, sizeof(txData));
  
  // Blink LED on successful transmission or button press
  static unsigned long lastBlink = 0;
  if (result || txData.button1 || txData.button2) {
    if (millis() - lastBlink > 100) {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      lastBlink = millis();
    }
  } else {
    digitalWrite(STATUS_LED, LOW);
  }
  
  delay(5); // ~200Hz transmission rate
}

void readJoysticks() {
  // Read raw analog values
  int leftXRaw = analogRead(LEFT_JOYSTICK_X);
  int leftYRaw = analogRead(LEFT_JOYSTICK_Y);
  int rightXRaw = analogRead(RIGHT_JOYSTICK_X);
  int rightYRaw = analogRead(RIGHT_JOYSTICK_Y);
  
  // Map to 1000-2000 range with center at 1500
  // Left Stick: X = Yaw, Y = Throttle
  txData.yaw = mapJoystick(leftXRaw, leftXMin, leftXMax, leftXCenter);
  txData.throttle = mapJoystick(leftYRaw, leftYMin, leftYMax, leftYCenter);
  
  // Right Stick: X = Roll, Y = Pitch
  txData.roll = mapJoystick(rightXRaw, rightXMin, rightXMax, rightXCenter);
  txData.pitch = mapJoystick(rightYRaw, rightYMin, rightYMax, rightYCenter);
  
  // Constrain to valid range
  txData.throttle = constrain(txData.throttle, 1000, 2000);
  txData.yaw = constrain(txData.yaw, 1000, 2000);
  txData.pitch = constrain(txData.pitch, 1000, 2000);
  txData.roll = constrain(txData.roll, 1000, 2000);
}

uint16_t mapJoystick(int rawValue, int minVal, int maxVal, int centerVal) {
  // Map joystick to 1000-2000 range with center at 1500
  // This fixes the dangerous center point issue
  
  if (rawValue < centerVal) {
    // Below center: map from min to center (1000-1500)
    return map(rawValue, minVal, centerVal, 1000, 1500);
  } else if (rawValue > centerVal) {
    // Above center: map from center to max (1500-2000)
    return map(rawValue, centerVal, maxVal, 1500, 2000);
  } else {
    // Exactly at center
    return 1500;
  }
}

void calibrateJoysticks() {
  calibrationStartTime = millis();
  
  // Sample joystick positions for 5 seconds
  int samples = 0;
  long leftXSum = 0, leftYSum = 0, rightXSum = 0, rightYSum = 0;
  
  int leftXMinTemp = 1023, leftXMaxTemp = 0;
  int leftYMinTemp = 1023, leftYMaxTemp = 0;
  int rightXMinTemp = 1023, rightXMaxTemp = 0;
  int rightYMinTemp = 1023, rightYMaxTemp = 0;
  
  while (millis() - calibrationStartTime < CALIBRATION_DURATION) {
    int leftX = analogRead(LEFT_JOYSTICK_X);
    int leftY = analogRead(LEFT_JOYSTICK_Y);
    int rightX = analogRead(RIGHT_JOYSTICK_X);
    int rightY = analogRead(RIGHT_JOYSTICK_Y);
    
    // Track min/max for range
    if (leftX < leftXMinTemp) leftXMinTemp = leftX;
    if (leftX > leftXMaxTemp) leftXMaxTemp = leftX;
    if (leftY < leftYMinTemp) leftYMinTemp = leftY;
    if (leftY > leftYMaxTemp) leftYMaxTemp = leftY;
    if (rightX < rightXMinTemp) rightXMinTemp = rightX;
    if (rightX > rightXMaxTemp) rightXMaxTemp = rightX;
    if (rightY < rightYMinTemp) rightYMinTemp = rightY;
    if (rightY > rightYMaxTemp) rightYMaxTemp = rightY;
    
    // Sum for center calculation
    leftXSum += leftX;
    leftYSum += leftY;
    rightXSum += rightX;
    rightYSum += rightY;
    samples++;
    
    // Blink LED during calibration
    digitalWrite(STATUS_LED, (millis() % 200) < 100);
    
    delay(10);
  }
  
  // Calculate centers and ranges
  leftXCenter = leftXSum / samples;
  leftYCenter = leftYSum / samples;
  rightXCenter = rightXSum / samples;
  rightYCenter = rightYSum / samples;
  
  // Set min/max with some margin
  leftXMin = leftXMinTemp - 10;
  leftXMax = leftXMaxTemp + 10;
  leftYMin = leftYMinTemp - 10;
  leftYMax = leftYMaxTemp + 10;
  rightXMin = rightXMinTemp - 10;
  rightXMax = rightXMaxTemp + 10;
  rightYMin = rightYMinTemp - 10;
  rightYMax = rightYMaxTemp + 10;
  
  // Constrain to valid ADC range
  leftXMin = constrain(leftXMin, 0, 1023);
  leftXMax = constrain(leftXMax, 0, 1023);
  leftYMin = constrain(leftYMin, 0, 1023);
  leftYMax = constrain(leftYMax, 0, 1023);
  rightXMin = constrain(rightXMin, 0, 1023);
  rightXMax = constrain(rightXMax, 0, 1023);
  rightYMin = constrain(rightYMin, 0, 1023);
  rightYMax = constrain(rightYMax, 0, 1023);
  
  digitalWrite(STATUS_LED, LOW);
  
  // Print calibration results
  Serial.println("\nCalibration Results:");
  Serial.print("Left X:  Center="); Serial.print(leftXCenter);
  Serial.print(" Range=["); Serial.print(leftXMin); Serial.print("-"); Serial.print(leftXMax); Serial.println("]");
  Serial.print("Left Y:  Center="); Serial.print(leftYCenter);
  Serial.print(" Range=["); Serial.print(leftYMin); Serial.print("-"); Serial.print(leftYMax); Serial.println("]");
  Serial.print("Right X: Center="); Serial.print(rightXCenter);
  Serial.print(" Range=["); Serial.print(rightXMin); Serial.print("-"); Serial.print(rightXMax); Serial.println("]");
  Serial.print("Right Y: Center="); Serial.print(rightYCenter);
  Serial.print(" Range=["); Serial.print(rightYMin); Serial.print("-"); Serial.print(rightYMax); Serial.println("]");
}
