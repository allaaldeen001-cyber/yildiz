/**
 * ============================================================================
 * DIY DRONE REMOTE CONTROLLER
 * ============================================================================
 * 
 * Hardware: Arduino Nano, NRF24L01, 2x Analog Joysticks, Buttons, Switches
 * 
 * PIN MAPPING:
 * -----------------------------------------
 * NRF24L01 Radio:
 *   D9  = CE
 *   D10 = CSN
 *   D11 = MOSI (hardware SPI)
 *   D12 = MISO (hardware SPI)
 *   D13 = SCK  (hardware SPI)
 * 
 * Joysticks (Analog):
 *   A0  = Left Y-axis  (Throttle)  - UP = increase, DOWN = decrease
 *   A1  = Left X-axis  (Yaw)       - LEFT = CCW, RIGHT = CW
 *   A2  = Right Y-axis (Pitch)     - UP = forward, DOWN = backward
 *   A3  = Right X-axis (Roll)      - LEFT = left, RIGHT = right
 * 
 * Buttons & Switches (directly mapped to FC pins):
 *   D2  = Switch 2 (Altitude Hold) - maps to FC A2
 *   D3  = Switch 1 (Arm/Disarm)    - maps to FC D2
 *   D4  = Button 1 (Calibration)   - maps to FC A0
 *   D5  = Button 2 (Motor Start)   - maps to FC A1
 * 
 * Note: Button/Switch states are transmitted to FC via radio.
 *       FC handles the actual pin assignments internally.
 * 
 * ============================================================================
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// NRF24L01 Radio
#define NRF_CE_PIN    9
#define NRF_CSN_PIN   10
#define NRF_CHANNEL   108  // Must match Flight Controller

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const uint64_t pipe = 0xF0F0F0F0E1LL;

// Joystick Pins
#define JOY_L_Y_PIN   A0  // Throttle (Left stick vertical)
#define JOY_L_X_PIN   A1  // Yaw (Left stick horizontal)
#define JOY_R_Y_PIN   A2  // Pitch (Right stick vertical)
#define JOY_R_X_PIN   A3  // Roll (Right stick horizontal)

// Button & Switch Pins
#define SWITCH_ALT_HOLD_PIN   2   // Altitude Hold Switch
#define SWITCH_ARM_PIN        3   // Arm/Disarm Switch
#define BTN_CALIBRATE_PIN     4   // Calibration Button
#define BTN_MOTOR_START_PIN   5   // Motor Start Button

// ============================================================================
// JOYSTICK CALIBRATION
// ============================================================================

// Scale factors (adjust for your joysticks)
float scaleX = 0.1;       // Roll sensitivity
float scaleY = -0.1;      // Pitch sensitivity (inverted)
float scaleZ = -0.1;      // Yaw sensitivity
float scaleThrust = 1.5;  // Throttle sensitivity

// Center calibration (512 = center for 10-bit ADC)
float calX = -512;        // Roll center offset
float calY = -512;        // Pitch center offset
float calZ = -512;        // Yaw center offset
float calThrust = -500;   // Throttle offset (slightly biased)

// Output offsets
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;
float offsetThrust = 1300;  // Base throttle value

// Joystick deadzone
#define DEADZONE 20

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct Package {
  int   thrust = 0;     // Throttle value (1000-2000)
  float x = 0;          // Roll command
  float y = 0;          // Pitch command
  float z = 0;          // Yaw command
  int   id = 0;         // Packet ID (for debugging)
  bool  but1 = 1;       // Calibration button (1=released, 0=pressed)
  bool  but2 = 1;       // Motor start button (1=released, 0=pressed)
  bool  switch1 = 1;    // Arm switch (1=disarmed, 0=armed)
  bool  switch2 = 1;    // Altitude hold (1=off, 0=on)
};

Package package;

// ============================================================================
// SMOOTHING FILTERS
// ============================================================================

Smoothed<float> smoothThrottle;
Smoothed<float> smoothRoll;
Smoothed<float> smoothPitch;
Smoothed<float> smoothYaw;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

int packetID = 0;

// Raw joystick values
float rawThrottle, rawYaw, rawPitch, rawRoll;

// Smoothed joystick values
float smoothedThrottle, smoothedYaw, smoothedPitch, smoothedRoll;

// Button/Switch states
bool but1, but2, switch1, switch2;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void setupRadio();
void readJoysticks();
void readButtons();
void applyDeadzone(float &value);
void buildPackage();
void transmitPackage();
void printDebug();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(57600);
  Serial.println(F("=== Drone Remote Controller ==="));
  
  // Configure button/switch pins with pull-ups
  pinMode(SWITCH_ALT_HOLD_PIN, INPUT_PULLUP);
  pinMode(SWITCH_ARM_PIN, INPUT_PULLUP);
  pinMode(BTN_CALIBRATE_PIN, INPUT_PULLUP);
  pinMode(BTN_MOTOR_START_PIN, INPUT_PULLUP);
  
  // Initialize radio
  setupRadio();
  
  // Initialize smoothing filters
  // Using exponential smoothing for responsive but smooth control
  smoothThrottle.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothRoll.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothPitch.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothYaw.begin(SMOOTHED_EXPONENTIAL, 5);
  
  Serial.println(F("Ready to transmit"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Read all inputs
  readJoysticks();
  readButtons();
  
  // Build and send packet
  buildPackage();
  transmitPackage();
  
  // Debug output
  printDebug();
  
  // Small delay for stability (approximately 100Hz update rate)
  delay(10);
}

// ============================================================================
// RADIO SETUP
// ============================================================================

void setupRadio() {
  radio.begin();
  radio.setChannel(NRF_CHANNEL);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  Serial.print(F("Radio initialized on channel "));
  Serial.println(NRF_CHANNEL);
}

// ============================================================================
// INPUT READING
// ============================================================================

void readJoysticks() {
  // Read raw analog values
  // Note: Some joysticks may need inversion based on wiring
  
  // Left stick - Throttle (Y) and Yaw (X)
  rawThrottle = max(analogRead(JOY_L_Y_PIN), 0);  // Ensure positive
  rawYaw = analogRead(JOY_L_X_PIN);
  
  // Right stick - Pitch (Y) and Roll (X)
  // Pitch is typically inverted (pushing forward = positive pitch = fly forward)
  rawPitch = 1023 - analogRead(JOY_R_Y_PIN);  // Invert for intuitive control
  rawRoll = analogRead(JOY_R_X_PIN);
  
  // Add to smoothing filters
  smoothThrottle.add(rawThrottle);
  smoothRoll.add(rawRoll);
  smoothPitch.add(rawPitch);
  smoothYaw.add(rawYaw);
  
  // Get smoothed values
  smoothedThrottle = smoothThrottle.get();
  smoothedRoll = smoothRoll.get();
  smoothedPitch = smoothPitch.get();
  smoothedYaw = smoothYaw.get();
}

void readButtons() {
  // Read button and switch states
  // INPUT_PULLUP means pressed = LOW, released = HIGH
  but1 = digitalRead(BTN_CALIBRATE_PIN);      // Calibration button
  but2 = digitalRead(BTN_MOTOR_START_PIN);    // Motor start button
  switch1 = digitalRead(SWITCH_ARM_PIN);      // Arm switch
  switch2 = digitalRead(SWITCH_ALT_HOLD_PIN); // Altitude hold switch
}

void applyDeadzone(float &value) {
  // Apply deadzone around center (512)
  if (value > (512 - DEADZONE) && value < (512 + DEADZONE)) {
    value = 512;
  }
}

// ============================================================================
// PACKAGE BUILDING
// ============================================================================

void buildPackage() {
  // Apply deadzone to directional controls
  float rollVal = smoothedRoll;
  float pitchVal = smoothedPitch;
  float yawVal = smoothedYaw;
  
  applyDeadzone(rollVal);
  applyDeadzone(pitchVal);
  applyDeadzone(yawVal);
  
  // Calculate control values
  // Format: (raw + calibration_offset) * scale + offset
  package.x = (rollVal + calX) * scaleX + offsetX;           // Roll
  package.y = (pitchVal + calY) * scaleY + offsetY;          // Pitch
  package.z = (yawVal + calZ) * scaleZ + offsetZ;            // Yaw
  package.thrust = (smoothedThrottle + calThrust) * scaleThrust + offsetThrust;
  
  // Constrain thrust to valid range
  package.thrust = constrain(package.thrust, 1000, 2000);
  
  // If throttle is at minimum, set to zero (used for connection detection)
  if (smoothedThrottle < 50) {
    package.thrust = 0;
  }
  
  // Set button/switch states
  package.but1 = but1;
  package.but2 = but2;
  package.switch1 = switch1;
  package.switch2 = switch2;
  
  // Increment packet ID
  package.id = packetID++;
  if (packetID > 30000) packetID = 0;
}

// ============================================================================
// TRANSMISSION
// ============================================================================

void transmitPackage() {
  radio.write(&package, sizeof(package));
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

void printDebug() {
  // Print control values
  Serial.print(F("T:"));
  Serial.print(package.thrust);
  Serial.print(F("\tY:"));
  Serial.print(package.z, 1);
  Serial.print(F("\tP:"));
  Serial.print(package.y, 1);
  Serial.print(F("\tR:"));
  Serial.print(package.x, 1);
  
  // Print button states
  Serial.print(F("\t|"));
  Serial.print(switch1 ? F("DIS") : F("ARM"));
  Serial.print(F("|"));
  Serial.print(switch2 ? F("---") : F("ALT"));
  Serial.print(F("|"));
  Serial.print(but1 ? F("-") : F("C"));
  Serial.print(but2 ? F("-") : F("M"));
  Serial.println(F("|"));
}
