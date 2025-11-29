/*
 * Professional UAV Remote Controller
 * Arduino Nano based Remote Controller with NRF24L01, Joysticks, Buttons, and Switches
 * 
 * Hardware Connections:
 * - NRF24L01: CE=D9, CSN=D10
 * - Left Joystick: V=A0, H=A1
 * - Right Joystick: V=A2, H=A3
 * - Button_1: D4 (Calibration)
 * - Button_2: D5 (ESC Calibration/Motor On)
 * - Switch_1: D2 (Altitude Hold)
 * - Switch_2: D3 (Arming/Kill Switch)
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Pin Definitions
#define NRF_CE_PIN 9
#define NRF_CSN_PIN 10

// Joystick Pins
#define LEFT_JOYSTICK_V A0   // Throttle (Up/Down)
#define LEFT_JOYSTICK_H A1   // Yaw (Left/Right)
#define RIGHT_JOYSTICK_V A2  // Pitch (Up/Down)
#define RIGHT_JOYSTICK_H A3  // Roll (Left/Right)

// Button and Switch Pins
#define BUTTON_1_PIN 4       // Calibration
#define BUTTON_2_PIN 5       // ESC Calibration/Motor On
#define SWITCH_1_PIN 2       // Altitude Hold
#define SWITCH_2_PIN 3       // Arming/Kill Switch

// Communication
#define NRF_CHANNEL 103
#define NRF_ADDRESS 0xF0F0F0F0E1LL

// Joystick Calibration
#define JOYSTICK_MIN 0
#define JOYSTICK_MAX 1023
#define JOYSTICK_CENTER 512
#define JOYSTICK_DEADZONE 20

// Timing
#define SERIAL_UPDATE_INTERVAL 100  // milliseconds
#define DATA_SEND_INTERVAL 10       // milliseconds

// RF24 object
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// Data structures
struct RCData {
  uint16_t throttle;  // 1000-2000
  int16_t yaw;        // -500 to +500
  int16_t pitch;      // -500 to +500
  int16_t roll;       // -500 to +500
  uint8_t button1;    // Calibration
  uint8_t button2;    // ESC calibration
  uint8_t switch1;    // Altitude hold
  uint8_t switch2;    // Arming/Kill switch
  uint8_t checksum;
};

struct FCData {
  uint8_t status;      // 0=disarmed, 1=armed, 2=calibrating, 3=error
  uint8_t link_status; // 0=no link, 1=linked
  float pitch;
  float roll;
  float yaw;
  uint8_t calibration_status; // 0=not calibrated, 1=calibrated, 2=error
};

RCData rcData;
FCData fcData;

// State variables
bool linkStatus = false;
unsigned long lastSerialUpdate = 0;
unsigned long lastDataSend = 0;
bool button1State = false;
bool button1PrevState = false;
bool button2State = false;
bool button2PrevState = false;

// Joystick calibration values
int leftJoyVMin = 0, leftJoyVMax = 1023, leftJoyVCenter = 512;
int leftJoyHMin = 0, leftJoyHMax = 1023, leftJoyHCenter = 512;
int rightJoyVMin = 0, rightJoyVMax = 1023, rightJoyVCenter = 512;
int rightJoyHMin = 0, rightJoyHMax = 1023, rightJoyHCenter = 512;

void setup() {
  Serial.begin(115200);
  Serial.println("Remote Controller Initializing...");

  // Initialize pins
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(SWITCH_1_PIN, INPUT_PULLUP);
  pinMode(SWITCH_2_PIN, INPUT_PULLUP);

  // Initialize NRF24L01
  if (!radio.begin()) {
    Serial.println("NRF24L01 initialization failed!");
    while(1);
  }
  
  radio.setChannel(NRF_CHANNEL);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(15, 15);  // Max retries for ACK
  radio.setAutoAck(true);
  radio.openWritingPipe(NRF_ADDRESS);
  radio.stopListening();
  
  Serial.println("NRF24L01 initialized on channel " + String(NRF_CHANNEL));
  Serial.println("Remote Controller Ready!");
  Serial.println("\n=== Remote Controller Status Monitor ===");
  Serial.println("Link Status | Armed | Calibrated | Pitch | Roll | Yaw | Throttle");
  Serial.println("------------------------------------------------------------");

  // Calibrate joysticks
  calibrateJoysticks();
  
  // Initialize data structure
  memset(&rcData, 0, sizeof(rcData));
  memset(&fcData, 0, sizeof(fcData));
}

void loop() {
  // Read inputs
  readJoysticks();
  readButtons();
  readSwitches();

  // Calculate checksum
  calculateChecksum();

  // Send data to FC
  sendData();

  // Update serial monitor
  updateSerialMonitor();

  // Small delay for stability
  delay(5);
}

void calibrateJoysticks() {
  Serial.println("Calibrating joysticks...");
  Serial.println("Please center all joysticks and wait 3 seconds...");
  delay(3000);
  
  // Read center values
  leftJoyVCenter = analogRead(LEFT_JOYSTICK_V);
  leftJoyHCenter = analogRead(LEFT_JOYSTICK_H);
  rightJoyVCenter = analogRead(RIGHT_JOYSTICK_V);
  rightJoyHCenter = analogRead(RIGHT_JOYSTICK_H);
  
  // Set min/max ranges (with some margin)
  leftJoyVMin = leftJoyVCenter - 400;
  leftJoyVMax = leftJoyVCenter + 400;
  leftJoyHMin = leftJoyHCenter - 400;
  leftJoyHMax = leftJoyHCenter + 400;
  
  rightJoyVMin = rightJoyVCenter - 400;
  rightJoyVMax = rightJoyVCenter + 400;
  rightJoyHMin = rightJoyHCenter - 400;
  rightJoyHMax = rightJoyHCenter + 400;
  
  Serial.println("Joystick calibration complete!");
  Serial.println("Left V Center: " + String(leftJoyVCenter));
  Serial.println("Left H Center: " + String(leftJoyHCenter));
  Serial.println("Right V Center: " + String(rightJoyVCenter));
  Serial.println("Right H Center: " + String(rightJoyHCenter));
}

void readJoysticks() {
  // Read raw values
  int leftV = analogRead(LEFT_JOYSTICK_V);
  int leftH = analogRead(LEFT_JOYSTICK_H);
  int rightV = analogRead(RIGHT_JOYSTICK_V);
  int rightH = analogRead(RIGHT_JOYSTICK_H);
  
  // Throttle: Left Joystick Vertical (A0)
  // Up = increase throttle, Down = decrease throttle
  // Inverted because joystick up typically reads lower values
  int throttleRaw = map(leftV, leftJoyVMin, leftJoyVMax, 1023, 0);
  throttleRaw = constrain(throttleRaw, 0, 1023);
  rcData.throttle = map(throttleRaw, 0, 1023, 1000, 2000);
  
  // Apply deadzone
  if (abs(throttleRaw - 512) < JOYSTICK_DEADZONE) {
    rcData.throttle = 1000;  // Idle throttle
  }
  
  // Yaw: Left Joystick Horizontal (A1)
  // Left = counter-clockwise, Right = clockwise
  int yawRaw = map(leftH, leftJoyHMin, leftJoyHMax, -500, 500);
  yawRaw = constrain(yawRaw, -500, 500);
  
  // Apply deadzone
  if (abs(yawRaw) < JOYSTICK_DEADZONE) {
    yawRaw = 0;
  }
  rcData.yaw = yawRaw;
  
  // Pitch: Right Joystick Vertical (A2)
  // Up = forward, Down = backward
  int pitchRaw = map(rightV, rightJoyVMin, rightJoyVMax, 500, -500);
  pitchRaw = constrain(pitchRaw, -500, 500);
  
  // Apply deadzone
  if (abs(pitchRaw) < JOYSTICK_DEADZONE) {
    pitchRaw = 0;
  }
  rcData.pitch = pitchRaw;
  
  // Roll: Right Joystick Horizontal (A3)
  // Left = tilt left, Right = tilt right
  int rollRaw = map(rightH, rightJoyHMin, rightJoyHMax, -500, 500);
  rollRaw = constrain(rollRaw, -500, 500);
  
  // Apply deadzone
  if (abs(rollRaw) < JOYSTICK_DEADZONE) {
    rollRaw = 0;
  }
  rcData.roll = rollRaw;
}

void readButtons() {
  // Read button states (inverted because of pull-up)
  button1PrevState = button1State;
  button1State = !digitalRead(BUTTON_1_PIN);
  
  button2PrevState = button2State;
  button2State = !digitalRead(BUTTON_2_PIN);
  
  // Update data structure (momentary buttons)
  rcData.button1 = button1State ? 1 : 0;
  rcData.button2 = button2State ? 1 : 0;
}

void readSwitches() {
  // Read switch states (inverted because pin2,3 connected to GND)
  // When switch is "1" (ON), pin reads LOW (connected to GND via pin2,3)
  // When switch is "0" (OFF), pin reads HIGH (pulled up, not connected to GND)
  rcData.switch1 = digitalRead(SWITCH_1_PIN) ? 0 : 1;  // Altitude Hold (inverted)
  rcData.switch2 = digitalRead(SWITCH_2_PIN) ? 0 : 1;  // Arming/Kill Switch (inverted)
}

void calculateChecksum() {
  uint8_t checksum = 0;
  uint8_t* data = (uint8_t*)&rcData;
  for (int i = 0; i < sizeof(rcData) - 1; i++) {
    checksum ^= data[i];
  }
  rcData.checksum = checksum;
}

void sendData() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastDataSend >= DATA_SEND_INTERVAL) {
    // Stop listening to receive ACK
    radio.stopListening();
    
    // Send data
    bool sent = radio.write(&rcData, sizeof(rcData));
    
    if (sent) {
      linkStatus = true;
      
      // Try to receive response from FC
      radio.startListening();
      delayMicroseconds(200);
      
      if (radio.available()) {
        uint8_t bytes = radio.getPayloadSize();
        if (bytes == sizeof(FCData)) {
          radio.read(&fcData, sizeof(FCData));
        }
      }
      radio.stopListening();
    } else {
      linkStatus = false;
    }
    
    lastDataSend = currentTime;
  }
}

void updateSerialMonitor() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSerialUpdate >= SERIAL_UPDATE_INTERVAL) {
    // Clear line and print status
    Serial.print("\r");
    
    // Link Status
    if (linkStatus) {
      Serial.print("  LINKED   | ");
    } else {
      Serial.print(" NO LINK   | ");
    }
    
    // Armed Status
    if (fcData.status == 1) {
      Serial.print(" ARMED  | ");
    } else {
      Serial.print("DISARMED| ");
    }
    
    // Calibration Status
    if (fcData.calibration_status == 1) {
      Serial.print("   YES    | ");
    } else if (fcData.calibration_status == 2) {
      Serial.print("  ERROR   | ");
    } else {
      Serial.print("   NO     | ");
    }
    
    // Pitch
    Serial.print(String(fcData.pitch, 1));
    if (fcData.pitch >= 0) Serial.print(" ");
    Serial.print(" | ");
    
    // Roll
    Serial.print(String(fcData.roll, 1));
    if (fcData.roll >= 0) Serial.print(" ");
    Serial.print(" | ");
    
    // Yaw
    Serial.print(String(fcData.yaw, 1));
    if (fcData.yaw >= 0) Serial.print(" ");
    Serial.print(" | ");
    
    // Throttle
    Serial.print(String(rcData.throttle));
    Serial.print("      ");
    
    // Print button and switch status on new line
    Serial.print("\nButtons: B1=");
    Serial.print(rcData.button1 ? "ON " : "OFF");
    Serial.print(" B2=");
    Serial.print(rcData.button2 ? "ON " : "OFF");
    Serial.print(" | Switches: SW1=");
    Serial.print(rcData.switch1 ? "ON " : "OFF");
    Serial.print(" SW2=");
    Serial.print(rcData.switch2 ? "ON " : "OFF");
    Serial.print("                    ");
    
    lastSerialUpdate = currentTime;
  }
}
