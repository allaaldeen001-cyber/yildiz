/*
 * RC Controller for Drone Flight System
 * Hardware: Arduino Nano, NRF24L01, 2x Joysticks, 2x Buttons, 2x Switches
 * 
 * Pin Mapping:
 * - NRF24L01: CE=9, CSN=10
 * - Joystick Left (Throttle/Yaw):
 *   - A0 = Throttle (Up/Down)
 *   - A1 = Yaw (Left/Right)
 * - Joystick Right (Pitch/Roll):
 *   - A2 = Pitch (Forward/Backward)
 *   - A3 = Roll (Left/Right)
 * - Buttons:
 *   - D4 = Button 1 (Calibration)
 *   - D5 = Button 2 (Smooth Motor Start)
 * - Switches:
 *   - D3 = Switch 1 (Arm/Disarm: 1=Disarmed, 0=Armed)
 *   - D2 = Switch 2 (Altitude Hold: 1=Off, 0=On)
 * - LED: D6 (Communication status)
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

RF24 radio(9, 10);  // CE, CSN
const uint64_t pipe = 0xF0F0F0F0E1LL;

// Analog pins for joysticks
const int THROTTLE_PIN = A0;  // Left stick - Up/Down
const int YAW_PIN = A1;        // Left stick - Left/Right
const int PITCH_PIN = A2;      // Right stick - Up/Down
const int ROLL_PIN = A3;       // Right stick - Left/Right

// Digital pins for buttons and switches
const int BUTTON1_PIN = 4;     // Calibration button
const int BUTTON2_PIN = 5;     // Smooth motor start button
const int SWITCH1_PIN = 3;     // Arm/Disarm switch (1=disarmed, 0=armed)
const int SWITCH2_PIN = 2;     // Altitude hold switch (1=off, 0=on)
const int LED_PIN = 6;         // Status LED

// Calibration values (adjust these after testing your joysticks)
float scaleRoll = 0.1;
float calRoll = -512;
float offsetRoll = 0;

float scalePitch = -0.1;       // Negative because pitch is inverted
float calPitch = -512;
float offsetPitch = 0;

float scaleYaw = -0.1;
float calYaw = -512;
float offsetYaw = 0;

float scaleThrust = 1.5;
float calThrust = -512;
float offsetThrust = 1000;     // Minimum throttle value

// Smoothing filters
Smoothed <float> smoothThrottle;
Smoothed <float> smoothYaw;
Smoothed <float> smoothPitch;
Smoothed <float> smoothRoll;

// Communication status
unsigned long lastAckTime = 0;
unsigned long lastSendTime = 0;
bool communicationOK = false;
int failedPackets = 0;

struct Package
{
  int   thrust = 0;
  float x = 0;        // Roll
  float y = 0;        // Pitch
  float z = 0;        // Yaw
  int   id = 0;
  bool  but1 = 1;     // Calibration button (0=pressed)
  bool  but2 = 1;     // Smooth start button (0=pressed)
  bool  switch1 = 1;  // Arm/Disarm (1=disarmed, 0=armed)
  bool  switch2 = 1;  // Altitude hold (1=off, 0=on)
};

Package package;
int packetID = 0;

// Function prototypes
void readJoysticks();
void readButtons();
void printPackage();
void updateLED();

void setup() {
  // Configure button and switch pins
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(57600);
  Serial.println("=== RC Controller Initializing ===");
  
  // Initialize NRF24L01 with ACK
  radio.begin();
  radio.setAutoAck(true);         // Enable AUTO ACK
  radio.enableAckPayload();
  radio.setRetries(5, 15);        // 5 * 250us delay, 15 retries
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(108);          // Same channel as FC
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  Serial.println("NRF24L01 initialized with ACK enabled");
  
  // Initialize smoothing filters
  smoothThrottle.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothYaw.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothPitch.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothRoll.begin(SMOOTHED_EXPONENTIAL, 5);
  
  // Startup blink
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  Serial.println("RC Controller Ready");
  Serial.println("Searching for drone...");
}

void loop() {
  readJoysticks();
  readButtons();
  
  // Map joystick values to control inputs
  package.x = (smoothRoll.get() + calRoll) * scaleRoll + offsetRoll;        // Roll
  package.y = (smoothPitch.get() + calPitch) * scalePitch + offsetPitch;    // Pitch (inverted)
  package.z = (smoothYaw.get() + calYaw) * scaleYaw + offsetYaw;           // Yaw
  package.thrust = (smoothThrottle.get() + calThrust) * scaleThrust + offsetThrust;  // Throttle (inverted)
  
  // Ensure thrust is within valid range
  package.thrust = constrain(package.thrust, 1000, 2000);
  
  package.id = packetID++;
  
  // Send packet and check ACK
  bool success = radio.write(&package, sizeof(package));
  
  if (success) {
    communicationOK = true;
    failedPackets = 0;
    lastAckTime = millis();
  } else {
    failedPackets++;
    if (failedPackets > 10) {
      communicationOK = false;
      Serial.println("Communication lost!");
    }
  }
  
  updateLED();
  printPackage();
  
  // Control loop at ~50Hz
  delay(20);
}

void readJoysticks() {
  // Read raw values
  int rawThrottle = analogRead(THROTTLE_PIN);
  int rawYaw = analogRead(YAW_PIN);
  int rawPitch = analogRead(PITCH_PIN);
  int rawRoll = analogRead(ROLL_PIN);
  
  // IMPORTANT: Invert throttle and pitch (1023 - value)
  rawThrottle = 1023 - rawThrottle;
  rawPitch = 1023 - rawPitch;
  
  // Apply smoothing
  smoothThrottle.add(rawThrottle);
  smoothYaw.add(rawYaw);
  smoothPitch.add(rawPitch);
  smoothRoll.add(rawRoll);
}

void readButtons() {
  // Read buttons and switches (active LOW)
  package.but1 = digitalRead(BUTTON1_PIN);
  package.but2 = digitalRead(BUTTON2_PIN);
  package.switch1 = digitalRead(SWITCH1_PIN);
  package.switch2 = digitalRead(SWITCH2_PIN);
  
  // Safety check: warn if armed
  if (package.switch1 == 0) {
    // Armed state - blink LED rapidly
    static unsigned long lastWarnBlink = 0;
    if (millis() - lastWarnBlink > 200) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      lastWarnBlink = millis();
    }
  }
}

void updateLED() {
  if (communicationOK) {
    // Slow blink = good connection
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      lastBlink = millis();
    }
  } else {
    // Fast blink = no connection
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 100) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      lastBlink = millis();
    }
  }
}

void printPackage() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {  // Print every 500ms
    Serial.print("Thrust: ");
    Serial.print(package.thrust);
    Serial.print("\t Roll: ");
    Serial.print(package.x);
    Serial.print("\t Pitch: ");
    Serial.print(package.y);
    Serial.print("\t Yaw: ");
    Serial.print(package.z);
    Serial.print("\t Armed: ");
    Serial.print(package.switch1 == 0 ? "YES" : "NO");
    Serial.print("\t AltHold: ");
    Serial.print(package.switch2 == 0 ? "ON" : "OFF");
    Serial.print("\t Link: ");
    Serial.println(communicationOK ? "OK" : "LOST");
    lastPrint = millis();
  }
}
