/*
 * DRONE REMOTE CONTROLLER
 * Hardware: Arduino Nano + NRF24L01 + Joysticks
 * 
 * Pin Configuration:
 * - NRF24L01: CE=D9, CSN=D10
 * - Joysticks:
 *   - A0: Left Y (Throttle)
 *   - A1: Left X (Yaw)
 *   - A2: Right Y (Pitch)
 *   - A3: Right X (Roll)
 * - Buttons:
 *   - D4: Button 1 (Calibration)
 *   - D5: Button 2 (Smooth Motor Start / Arm)
 * - Switches:
 *   - D3: Switch 1 (Arm/Disarm)
 *   - D2: Switch 2 (Altitude Hold)
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

RF24 radio(9, 10);  // CE, CSN
const uint64_t pipe = 0xF0F0F0F0E1LL;

// Smoothing filters for joysticks
Smoothed <float> smoothThrottle;
Smoothed <float> smoothRoll;
Smoothed <float> smoothYaw;
Smoothed <float> smoothPitch;

// Joystick Calibration
float scaleX = 0.1;
float calX = -527;
float offsetX = 0;

float scaleY = -0.1;
float calY = -507;
float offsetY = 0;

float scaleZ = -0.1;
float calZ = -512;
float offsetZ = 0;

float scaleThrust = 1.5;
float calThrust = -500;
float offsetThrust = 1300;

// Analog Pins
const int YL_pin = 0;  // Throttle
const int XL_pin = 1;  // Yaw
const int YR_pin = 2;  // Pitch
const int XR_pin = 3;  // Roll

// Digital Pins
const int SWITCH2 = 2;  // Altitude Hold
const int SWITCH1 = 3;  // Arm/Disarm
const int BUT1 = 4;     // Calibration
const int BUT2 = 5;     // Smooth Motor Start / Arm

// Communication Package
struct Package {
  int   thrust = 0;
  float x = 0;      // Roll
  float y = 0;      // Pitch
  float z = 0;      // Yaw
  int   id = 0;
  bool  but1 = 1;   // Button 1: Calibration
  bool  but2 = 1;   // Button 2: Smooth Start / Arm
  bool  switch1 = 1;  // Switch 1: Arm/Disarm
  bool  switch2 = 1;  // Switch 2: Altitude Hold
};

Package package;

// Variables
int ID = 0;
float xr, yr, xl, yl;
unsigned long lastTransmitTime = 0;
bool transmitSuccess = false;

//===================== FUNCTION PROTOTYPES =====================
void readJoyStick();
void printPackage();
void initializeNRF();

//===================== SETUP =====================
void setup() {
  // Initialize buttons and switches with pull-up resistors
  pinMode(SWITCH2, INPUT_PULLUP);
  pinMode(SWITCH1, INPUT_PULLUP);
  pinMode(BUT1, INPUT_PULLUP);
  pinMode(BUT2, INPUT_PULLUP);
  
  Serial.begin(57600);
  Serial.println("=== DRONE REMOTE CONTROLLER ===");
  
  // Initialize smoothing filters (exponential smoothing)
  smoothThrottle.begin(SMOOTHED_EXPONENTIAL, 2);
  smoothRoll.begin(SMOOTHED_EXPONENTIAL, 2);
  smoothYaw.begin(SMOOTHED_EXPONENTIAL, 2);
  smoothPitch.begin(SMOOTHED_EXPONENTIAL, 2);
  
  // Initialize NRF24L01
  initializeNRF();
  
  Serial.println("=== READY TO TRANSMIT ===");
  Serial.println("Controls:");
  Serial.println("  Left Stick Y  = Throttle");
  Serial.println("  Left Stick X  = Yaw");
  Serial.println("  Right Stick Y = Pitch");
  Serial.println("  Right Stick X = Roll");
  Serial.println("  Switch D3     = Arm/Disarm (0=Armed, 1=Disarmed)");
  Serial.println("  Switch D2     = Altitude Hold");
  Serial.println("  Button D4     = Calibration (when disarmed)");
  Serial.println("  Button D5     = Smooth Motor Start (when armed)");
  Serial.println("              or Long Press = Arm (when disarmed)");
}

//===================== MAIN LOOP =====================
void loop() {
  readJoyStick();
  
  // Prepare package
  package.x = (xr + calX) * scaleX + offsetX;  // Roll
  package.y = (yr + calY) * scaleY + offsetY;  // Pitch
  package.z = (xl + calZ) * scaleZ + offsetZ;  // Yaw
  package.thrust = (yl + calThrust) * scaleThrust + offsetThrust;
  package.id = ID++;
  
  // Read buttons and switches
  package.but1 = digitalRead(BUT1);
  package.but2 = digitalRead(BUT2);
  package.switch1 = digitalRead(SWITCH1);
  package.switch2 = digitalRead(SWITCH2);
  
  // Transmit package
  transmitSuccess = radio.write(&package, sizeof(package));
  
  if (transmitSuccess) {
    lastTransmitTime = millis();
  }
  
  // Print status every 100ms
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 100) {
    printPackage();
    lastPrintTime = millis();
  }
  
  delay(7);  // ~140Hz update rate
}

//===================== NRF INITIALIZATION =====================
void initializeNRF() {
  Serial.println("Initializing NRF24L01...");
  
  if (!radio.begin()) {
    Serial.println("NRF24L01 initialization FAILED!");
    while(1) {
      Serial.println("Check NRF24L01 wiring!");
      delay(1000);
    }
  }
  
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(108);
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  Serial.println("NRF24L01 OK - Transmitting on channel 108");
}

//===================== READ JOYSTICKS =====================
void readJoyStick() {
  // Read raw values
  float rawRoll = analogRead(XR_pin);
  float rawYaw = analogRead(XL_pin);
  float rawPitch = 1023 - analogRead(YR_pin);  // Inverted
  float rawThrottle = max(analogRead(YL_pin), 0);  // Ensure non-negative
  
  // Apply smoothing
  smoothRoll.add(rawRoll);
  smoothYaw.add(rawYaw);
  smoothPitch.add(rawPitch);
  smoothThrottle.add(rawThrottle);
  
  // Get smoothed values
  xr = smoothRoll.get();
  xl = smoothYaw.get();
  yr = smoothPitch.get();
  yl = smoothThrottle.get();
}

//===================== PRINT PACKAGE INFO =====================
void printPackage() {
  // Status indicator
  if (millis() - lastTransmitTime < 100) {
    Serial.print("[TX OK] ");
  } else {
    Serial.print("[TX FAIL] ");
  }
  
  // Thrust
  Serial.print("Thr:");
  Serial.print(package.thrust);
  Serial.print(" ");
  
  // Roll, Pitch, Yaw
  Serial.print("R:");
  Serial.print(package.x, 1);
  Serial.print(" ");
  Serial.print("P:");
  Serial.print(package.y, 1);
  Serial.print(" ");
  Serial.print("Y:");
  Serial.print(package.z, 1);
  Serial.print(" ");
  
  // Switches and buttons
  Serial.print("Arm:");
  Serial.print(!package.switch1);  // 0=Armed, 1=Disarmed
  Serial.print(" ");
  Serial.print("Alt:");
  Serial.print(!package.switch2);
  Serial.print(" ");
  Serial.print("Cal:");
  Serial.print(!package.but1);
  Serial.print(" ");
  Serial.print("Start:");
  Serial.print(!package.but2);
  
  Serial.println();
}
