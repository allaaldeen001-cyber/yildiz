/*
 * Professional UAV Remote Controller
 * Arduino Nano based Remote Controller with NRF24L01
 * 
 * Hardware Configuration:
 * - NRF24L01 PA+LNA: CE=D9, CSN=D10
 * - Left Joystick: V=A0, H=A1
 * - Right Joystick: V=A2, H=A3
 * - Button 1 (D4): Calibration
 * - Button 2 (D5): Motor On/ESC Calibration
 * - Switch 1 (D2): Position Hold (Pin1=D2, Pin2/3=GND)
 * - Switch 2 (D3): Arming/Kill Switch (Pin1=D3, Pin2/3=GND)
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pin Definitions
#define NRF_CE_PIN 9
#define NRF_CSN_PIN 10

// Joystick Pins
#define JOYSTICK_LEFT_V_PIN A0   // Throttle
#define JOYSTICK_LEFT_H_PIN A1   // Yaw
#define JOYSTICK_RIGHT_V_PIN A2  // Pitch
#define JOYSTICK_RIGHT_H_PIN A3  // Roll

// Button and Switch Pins
#define BUTTON_1_PIN 4  // Calibration
#define BUTTON_2_PIN 5  // Motor On/ESC Calibration
#define SWITCH_1_PIN 2  // Position Hold
#define SWITCH_2_PIN 3  // Arming/Kill Switch

// NRF24L01 Configuration
#define NRF_CHANNEL 103
#define NRF_ADDRESS "FC001"

// Joystick Calibration
#define JOYSTICK_DEADZONE 10
#define JOYSTICK_MIN 0
#define JOYSTICK_MAX 1023
#define JOYSTICK_CENTER 512

// Communication Data Structure (must match FC)
struct RCData {
  uint16_t throttle;    // A0: 0-1023
  uint16_t yaw;         // A1: 0-1023
  uint16_t pitch;       // A2: 0-1023
  uint16_t roll;        // A3: 0-1023
  bool button1;         // D4: Calibration
  bool button2;         // D5: Motor On/ESC Calibration
  bool switch1;         // D2: Position Hold
  bool switch2;         // D3: Arming/Kill Switch
  uint32_t timestamp;
};

struct FCData {
  float roll;
  float pitch;
  float yaw;
  float altitude;
  bool calibrated;
  bool armed;
  bool motors_on;
  uint32_t timestamp;
};

// Global Objects
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
RCData rcData;
FCData fcData;

// State Variables
bool communicationLinked = false;
unsigned long lastCommTime = 0;
unsigned long lastSerialUpdate = 0;
bool lastButton1State = false;
bool lastButton2State = false;
bool lastSwitch1State = false;
bool lastSwitch2State = false;

// Joystick Calibration Values
int leftVMin = 0, leftVMax = 1023, leftVCenter = 512;
int leftHMin = 0, leftHMax = 1023, leftHCenter = 512;
int rightVMin = 0, rightVMax = 1023, rightVCenter = 512;
int rightHMin = 0, rightHMax = 1023, rightHCenter = 512;

void setup() {
  Serial.begin(115200);
  Serial.println("=== Remote Controller Initializing ===");
  
  // Initialize pins
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(SWITCH_1_PIN, INPUT_PULLUP);
  pinMode(SWITCH_2_PIN, INPUT_PULLUP);
  
  // Initialize joystick pins (analog, no setup needed)
  
  // Initialize NRF24L01
  Serial.print("Initializing NRF24L01... ");
  if (radio.begin()) {
    radio.setChannel(NRF_CHANNEL);
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setAutoAck(true);
    radio.enableAckPayload();
    radio.setRetries(5, 15);
    radio.openWritingPipe(NRF_ADDRESS);
    radio.stopListening();
    Serial.println("OK");
    Serial.print("Channel: ");
    Serial.println(NRF_CHANNEL);
  } else {
    Serial.println("FAILED");
    while(1) delay(1000);
  }
  
  // Initialize RC Data
  memset(&rcData, 0, sizeof(rcData));
  
  // Calibrate joysticks (read center positions)
  delay(500);
  calibrateJoysticks();
  
  Serial.println("=== Remote Controller Ready ===");
  Serial.println("Waiting for Flight Controller...");
  Serial.println();
  Serial.println("=== Serial Monitor Display ===");
  Serial.println("Format: [Status] | Throttle | Yaw | Pitch | Roll | B1 | B2 | SW1 | SW2 | FC Data");
  Serial.println("-------------------------------------------------------------------------------");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read joystick inputs
  readJoysticks();
  
  // Read buttons and switches
  readButtonsSwitches();
  
  // Update timestamp
  rcData.timestamp = currentTime;
  
  // Send data to Flight Controller
  bool txResult = false;
  radio.stopListening();
  txResult = radio.write(&rcData, sizeof(RCData));
  
  if (txResult) {
    // Check for ACK payload (FC data)
    if (radio.isAckPayloadAvailable()) {
      radio.read(&fcData, sizeof(FCData));
      communicationLinked = true;
      lastCommTime = currentTime;
    }
  }
  
  // Check communication timeout
  if (currentTime - lastCommTime > 1000) {
    communicationLinked = false;
  }
  
  // Update serial monitor display (every 200ms)
  if (currentTime - lastSerialUpdate > 200) {
    updateSerialDisplay();
    lastSerialUpdate = currentTime;
  }
  
  delay(10); // Small delay for stability
}

void readJoysticks() {
  // Read raw analog values
  int leftVRaw = analogRead(JOYSTICK_LEFT_V_PIN);
  int leftHRaw = analogRead(JOYSTICK_LEFT_H_PIN);
  int rightVRaw = analogRead(JOYSTICK_RIGHT_V_PIN);
  int rightHRaw = analogRead(JOYSTICK_RIGHT_H_PIN);
  
  // Apply deadzone and map to 0-1023
  rcData.throttle = applyDeadzone(leftVRaw, leftVCenter);
  rcData.yaw = applyDeadzone(leftHRaw, leftHCenter);
  rcData.pitch = applyDeadzone(rightVRaw, rightVCenter);
  rcData.roll = applyDeadzone(rightHRaw, rightHCenter);
  
  // Invert throttle (joystick up = increase throttle)
  // Most joysticks: up = low value, down = high value
  rcData.throttle = 1023 - rcData.throttle;
}

int applyDeadzone(int rawValue, int center) {
  int value = rawValue;
  
  // Apply deadzone
  if (abs(value - center) < JOYSTICK_DEADZONE) {
    return center;
  }
  
  // Map to full range
  if (value < center) {
    return map(value, 0, center - JOYSTICK_DEADZONE, 0, center);
  } else {
    return map(value, center + JOYSTICK_DEADZONE, 1023, center, 1023);
  }
}

void readButtonsSwitches() {
  // Read buttons (inverted because of INPUT_PULLUP)
  // Button pressed = LOW (connected to GND), so invert to get true when pressed
  rcData.button1 = !digitalRead(BUTTON_1_PIN);
  rcData.button2 = !digitalRead(BUTTON_2_PIN);
  
  // Read switches
  // Wiring: Pin1 → D2/D3 (with INPUT_PULLUP), Pin2/3 → GND
  // When switch is ON (position "1"): Pin1 not connected to GND → reads HIGH
  // When switch is OFF (position "0"): Pin1 connected to GND → reads LOW
  // So: HIGH = ON, LOW = OFF (no inversion needed)
  rcData.switch1 = digitalRead(SWITCH_1_PIN);  // Position Hold (HIGH = ON)
  rcData.switch2 = digitalRead(SWITCH_2_PIN);  // Arming/Kill Switch (HIGH = ON)
  
  // Detect button presses for serial feedback
  if (rcData.button1 && !lastButton1State) {
    Serial.println("[ACTION] Button 1 pressed - Calibration requested");
  }
  if (rcData.button2 && !lastButton2State) {
    Serial.println("[ACTION] Button 2 pressed - ESC Calibration/Motor On");
  }
  if (rcData.switch1 != lastSwitch1State) {
    Serial.print("[ACTION] Switch 1 changed - Position Hold: ");
    Serial.println(rcData.switch1 ? "ON" : "OFF");
  }
  if (rcData.switch2 != lastSwitch2State) {
    Serial.print("[ACTION] Switch 2 changed - Arming: ");
    Serial.println(rcData.switch2 ? "ARMED" : "DISARMED");
  }
  
  lastButton1State = rcData.button1;
  lastButton2State = rcData.button2;
  lastSwitch1State = rcData.switch1;
  lastSwitch2State = rcData.switch2;
}

void calibrateJoysticks() {
  Serial.println("Calibrating joysticks (center positions)...");
  
  // Read center positions (assuming joysticks are centered at startup)
  leftVCenter = analogRead(JOYSTICK_LEFT_V_PIN);
  leftHCenter = analogRead(JOYSTICK_LEFT_H_PIN);
  rightVCenter = analogRead(JOYSTICK_RIGHT_V_PIN);
  rightHCenter = analogRead(JOYSTICK_RIGHT_H_PIN);
  
  Serial.print("Left V Center: ");
  Serial.println(leftVCenter);
  Serial.print("Left H Center: ");
  Serial.println(leftHCenter);
  Serial.print("Right V Center: ");
  Serial.println(rightVCenter);
  Serial.print("Right H Center: ");
  Serial.println(rightHCenter);
  Serial.println("Calibration complete");
}

void updateSerialDisplay() {
  // Clear line and print status
  Serial.print("\r");
  
  // Communication status
  if (communicationLinked) {
    Serial.print("[LINKED] ");
  } else {
    Serial.print("[NO LINK] ");
  }
  
  // RC Inputs
  Serial.print("T:");
  Serial.print(rcData.throttle);
  Serial.print(" Y:");
  Serial.print(rcData.yaw);
  Serial.print(" P:");
  Serial.print(rcData.pitch);
  Serial.print(" R:");
  Serial.print(rcData.roll);
  
  // Buttons and Switches
  Serial.print(" | B1:");
  Serial.print(rcData.button1 ? "ON" : "OFF");
  Serial.print(" B2:");
  Serial.print(rcData.button2 ? "ON" : "OFF");
  Serial.print(" SW1:");
  Serial.print(rcData.switch1 ? "ON" : "OFF");
  Serial.print(" SW2:");
  Serial.print(rcData.switch2 ? "ON" : "OFF");
  
  // FC Data (if available)
  if (communicationLinked) {
    Serial.print(" | FC: ");
    Serial.print("Cal:");
    Serial.print(fcData.calibrated ? "YES" : "NO");
    Serial.print(" Arm:");
    Serial.print(fcData.armed ? "YES" : "NO");
    Serial.print(" Motor:");
    Serial.print(fcData.motors_on ? "ON" : "OFF");
    Serial.print(" Roll:");
    Serial.print(fcData.roll, 1);
    Serial.print(" Pitch:");
    Serial.print(fcData.pitch, 1);
    Serial.print(" Alt:");
    Serial.print(fcData.altitude, 1);
  }
  
  // Print newline periodically for readability
  static int lineCount = 0;
  if (lineCount++ > 10) {
    Serial.println();
    lineCount = 0;
  }
}
