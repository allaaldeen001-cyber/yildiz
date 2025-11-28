/*
 * ============================================================
 *            QUADCOPTER RC TRANSMITTER - FIXED VERSION
 * ============================================================
 * 
 * FIXES APPLIED:
 * - Throttle inversion fixed (UP = More Power)
 * - Enhanced communication status display
 * - Improved signal filtering
 * - Better radio reliability
 * 
 * HARDWARE:
 * - Arduino Nano
 * - nRF24L01+ module (CE=9, CSN=10)
 * - 2x Joysticks (Left: Throttle/Yaw, Right: Pitch/Roll)
 * - Optional: 10uF capacitor on NRF power pins
 * 
 * PIN CONNECTIONS:
 * - A0: Left Joystick Y (Throttle)
 * - A1: Left Joystick X (Yaw)
 * - A2: Right Joystick Y (Pitch)
 * - A3: Right Joystick X (Roll)
 * - D2: Switch 2 (Altitude Hold)
 * - D3: Switch 1 (Arm/Disarm Safety)
 * - D4: Button 1 (Calibration)
 * - D5: Button 2 (Arming)
 * - D9: NRF CE
 * - D10: NRF CSN
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Smoothed.h>

// ================================================================
//                      SMOOTHING FILTERS
// ================================================================
Smoothed<float> smoothThrottle;
Smoothed<float> smoothRoll;
Smoothed<float> smoothYaw;
Smoothed<float> smoothPitch;

// ================================================================
//                      NRF24 RADIO SETUP
// ================================================================
RF24 radio(9, 10);  // CE, CSN pins
const uint64_t pipe = 0xF0F0F0F0E1LL;

// ================================================================
//                      CALIBRATION VALUES
// ================================================================
// Roll (Right Stick X) - Adjust calX for center position
float scaleX =  0.1;
float calX   = -512;   // Adjust if roll drifts at center
float offsetX = 0;

// Pitch (Right Stick Y) - Adjust calY for center position
float scaleY = -0.1;
float calY   = -512;   // Adjust if pitch drifts at center
float offsetY = 0;

// Yaw (Left Stick X) - Adjust calZ for center position
float scaleZ = -0.1;
float calZ   = -512;   // Adjust if yaw drifts at center
float offsetZ = 0;

// ================================================================
//                      JOYSTICK PINS
// ================================================================
const int PIN_THROTTLE = A0;  // Left Y - Throttle
const int PIN_YAW      = A1;  // Left X - Yaw
const int PIN_PITCH    = A2;  // Right Y - Pitch  
const int PIN_ROLL     = A3;  // Right X - Roll

// Button/Switch Pins
const int PIN_SWITCH2  = 2;   // Altitude Hold
const int PIN_SWITCH1  = 3;   // Arm Safety
const int PIN_BUTTON1  = 4;   // Calibration
const int PIN_BUTTON2  = 5;   // Arming

// ================================================================
//                      VARIABLES
// ================================================================
int packetID = 0;
float rawRoll, rawPitch;
float rawYaw, rawThrottle;
float filteredRoll, filteredPitch, filteredYaw, filteredThrottle;
bool button1, button2, switch1, switch2;
bool txStatus = false;

// Communication statistics
unsigned long txSuccess = 0;
unsigned long txFailed = 0;
unsigned long lastStatusPrint = 0;
const unsigned long STATUS_INTERVAL = 250;  // Print every 250ms

// Throttle deadzone at bottom (prevents accidental arming)
const int THROTTLE_DEADZONE = 50;

// ================================================================
//                      DATA PACKAGE
// ================================================================
struct Package {
  int   thrust = 1000;
  float x = 0;        // Roll
  float y = 0;        // Pitch
  float z = 0;        // Yaw
  int   id = 0;
  bool  but1 = 1;
  bool  but2 = 1;
  bool  switch1 = 1;
  bool  switch2 = 1;
};

Package package;

// ================================================================
//                      FUNCTION PROTOTYPES
// ================================================================
void readJoysticks();
void processInputs();
void sendPackage();
void printStatus();
void printDetailedStatus();

// ================================================================
//                      SETUP
// ================================================================
void setup() {
  // Configure input pins with pull-up resistors
  pinMode(PIN_SWITCH2, INPUT_PULLUP);
  pinMode(PIN_SWITCH1, INPUT_PULLUP);
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);

  Serial.begin(57600);
  Serial.println(F(""));
  Serial.println(F("========================================"));
  Serial.println(F("   QUADCOPTER RC TRANSMITTER v2.0"));
  Serial.println(F("   Throttle Direction: FIXED"));
  Serial.println(F("========================================"));
  
  // Initialize NRF24 Radio
  if (!radio.begin()) {
    Serial.println(F("[ERROR] Radio hardware not responding!"));
    Serial.println(F("Check wiring: CE=9, CSN=10, MOSI=11, MISO=12, SCK=13"));
    while (1) {
      delay(1000);  // Halt if radio fails
    }
  }
  
  radio.setChannel(108);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.setRetries(5, 15);
  radio.setDataRate(RF24_250KBPS);
  
  // Power level: Start LOW for testing, increase for range
  // Options: RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  // TIP: Add 10uF capacitor to NRF module before using MAX
  radio.setPALevel(RF24_PA_LOW);
  
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  // Initialize smoothing filters
  smoothThrottle.begin(SMOOTHED_EXPONENTIAL, 10);
  smoothRoll.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothYaw.begin(SMOOTHED_EXPONENTIAL, 5);
  smoothPitch.begin(SMOOTHED_EXPONENTIAL, 5);
  
  Serial.println(F("[OK] Radio initialized - Channel 108"));
  Serial.println(F("[OK] Smoothing filters ready"));
  Serial.println(F(""));
  Serial.println(F("CONTROLS:"));
  Serial.println(F("  Left Stick Y  = Throttle (UP = More Power)"));
  Serial.println(F("  Left Stick X  = Yaw"));
  Serial.println(F("  Right Stick Y = Pitch"));
  Serial.println(F("  Right Stick X = Roll"));
  Serial.println(F("  Button 1      = Calibrate gyro (hold 2s)"));
  Serial.println(F("  Button 2      = Arm/Disarm (hold 2s)"));
  Serial.println(F("  Switch 1      = Safety disarm"));
  Serial.println(F("  Switch 2      = Altitude hold mode"));
  Serial.println(F("========================================"));
  Serial.println(F(""));
  
  delay(500);
}

// ================================================================
//                      MAIN LOOP
// ================================================================
void loop() {
  readJoysticks();
  processInputs();
  sendPackage();
  printStatus();
  
  delay(5);  // ~200Hz update rate
}

// ================================================================
//                      READ JOYSTICKS
// ================================================================
void readJoysticks() {
  // Read buttons and switches (LOW = pressed/active)
  button1 = digitalRead(PIN_BUTTON1);
  button2 = digitalRead(PIN_BUTTON2);
  switch1 = digitalRead(PIN_SWITCH1);
  switch2 = digitalRead(PIN_SWITCH2);

  // Read analog joystick values (0-1023)
  rawRoll     = analogRead(PIN_ROLL);
  rawYaw      = analogRead(PIN_YAW);
  rawPitch    = 1023 - analogRead(PIN_PITCH);  // Invert if needed
  
  // ============================================================
  // THROTTLE FIX: Invert so UP = Higher value
  // ============================================================
  // If your throttle was backwards, this fixes it:
  rawThrottle = 1023 - analogRead(PIN_THROTTLE);
  
  // If throttle is STILL backwards after this change, 
  // remove the "1023 -" part above.
  
  // Apply smoothing filters
  smoothRoll.add(rawRoll);
  smoothYaw.add(rawYaw);
  smoothPitch.add(rawPitch);
  smoothThrottle.add(rawThrottle);

  filteredRoll     = smoothRoll.get();
  filteredYaw      = smoothYaw.get();
  filteredPitch    = smoothPitch.get();
  filteredThrottle = smoothThrottle.get();
}

// ================================================================
//                      PROCESS INPUTS
// ================================================================
void processInputs() {
  // Calculate Roll, Pitch, Yaw with calibration offsets
  package.x = (filteredRoll + calX) * scaleX + offsetX;
  package.y = (filteredPitch + calY) * scaleY + offsetY;
  package.z = (filteredYaw + calZ) * scaleZ + offsetZ;
  
  // ============================================================
  // THROTTLE PROCESSING
  // ============================================================
  // Map 0-1023 analog input to 1000-2000 microseconds for ESC
  // 1000 = Motor Off, 2000 = Full Power
  
  int throttleValue = (int)filteredThrottle;
  
  // Apply deadzone at the bottom to prevent accidental motor spin
  if (throttleValue < THROTTLE_DEADZONE) {
    throttleValue = 0;
  }
  
  // Map to ESC range (1000-2000)
  package.thrust = map(throttleValue, 0, 1023, 1000, 2000);
  
  // Safety constrain
  package.thrust = constrain(package.thrust, 1000, 2000);
  
  // Assign buttons/switches
  package.but1 = button1;
  package.but2 = button2;
  package.switch1 = switch1;
  package.switch2 = switch2;
  
  // Increment packet ID for tracking
  package.id = packetID++;
  if (packetID > 32000) packetID = 0;
}

// ================================================================
//                      SEND PACKAGE
// ================================================================
void sendPackage() {
  txStatus = radio.write(&package, sizeof(package));
  
  if (txStatus) {
    txSuccess++;
  } else {
    txFailed++;
  }
}

// ================================================================
//                      PRINT STATUS (Clean Display)
// ================================================================
void printStatus() {
  // Only print at specified interval to avoid serial flooding
  if (millis() - lastStatusPrint < STATUS_INTERVAL) {
    return;
  }
  lastStatusPrint = millis();
  
  // Calculate success rate
  unsigned long totalPackets = txSuccess + txFailed;
  float successRate = (totalPackets > 0) ? (100.0 * txSuccess / totalPackets) : 0;
  
  // Clear line and print status
  Serial.print(F("\r"));  // Carriage return for same-line update
  
  // Throttle bar visualization
  Serial.print(F("THR:"));
  int barLength = map(package.thrust, 1000, 2000, 0, 10);
  Serial.print(F("["));
  for (int i = 0; i < 10; i++) {
    if (i < barLength) Serial.print(F("="));
    else Serial.print(F(" "));
  }
  Serial.print(F("] "));
  
  // Numeric values
  Serial.print(package.thrust);
  Serial.print(F(" | Y:"));
  Serial.print(package.z, 1);
  Serial.print(F(" R:"));
  Serial.print(package.x, 1);
  Serial.print(F(" P:"));
  Serial.print(package.y, 1);
  
  // Switch states
  Serial.print(F(" | SW1:"));
  Serial.print(switch1 ? F("OFF") : F("ON "));
  Serial.print(F(" SW2:"));
  Serial.print(switch2 ? F("OFF") : F("ON "));
  
  // Communication status with visual indicator
  Serial.print(F(" | LINK:"));
  if (txStatus) {
    Serial.print(F("[OK]  "));
  } else {
    Serial.print(F("[LOST]"));
  }
  
  // Success rate
  Serial.print(F(" "));
  Serial.print(successRate, 1);
  Serial.print(F("%"));
  
  // Armed status warning
  if (switch1 == 0 && package.thrust > 1050) {
    Serial.print(F(" !ARMED!"));
  }
  
  Serial.println();
}

// ================================================================
//                      DETAILED STATUS (for debugging)
// ================================================================
void printDetailedStatus() {
  Serial.println(F("\n===== DETAILED STATUS ====="));
  
  Serial.print(F("Raw Throttle: "));
  Serial.println(rawThrottle);
  
  Serial.print(F("Filtered Throttle: "));
  Serial.println(filteredThrottle);
  
  Serial.print(F("Output Thrust: "));
  Serial.println(package.thrust);
  
  Serial.print(F("Roll (X): "));
  Serial.print(package.x);
  Serial.print(F("  Pitch (Y): "));
  Serial.print(package.y);
  Serial.print(F("  Yaw (Z): "));
  Serial.println(package.z);
  
  Serial.print(F("Packets Sent: "));
  Serial.print(txSuccess);
  Serial.print(F("  Failed: "));
  Serial.println(txFailed);
  
  Serial.println(F("===========================\n"));
}
