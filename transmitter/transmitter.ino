/*
 * Drone Transmitter (Remote Controller)
 * TMRh20 RF24 Library - NO ACK Mode for Low Latency
 * 
 * Hardware:
 * - Arduino Nano/Uno
 * - NRF24L01+ module (with PA+LNA recommended)
 * - 2x Joysticks (4 analog inputs)
 * - 2x Switches/Potentiometers for AUX channels
 * 
 * Wiring NRF24L01 to Arduino:
 * VCC  -> 3.3V (IMPORTANT: NOT 5V!)
 * GND  -> GND
 * CE   -> Pin 9
 * CSN  -> Pin 10
 * SCK  -> Pin 13
 * MOSI -> Pin 11
 * MISO -> Pin 12
 * IRQ  -> Not connected
 * 
 * TIP: Add 10-100uF capacitor between VCC and GND on NRF24L01!
 */

#include <SPI.h>
#include <RF24.h>
#include "../common/RF24_Config.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Joystick pins (analog)
#define THROTTLE_PIN  A0  // Left stick vertical
#define YAW_PIN       A1  // Left stick horizontal
#define PITCH_PIN     A2  // Right stick vertical
#define ROLL_PIN      A3  // Right stick horizontal

// Auxiliary channels
#define AUX1_PIN      A4  // Arm switch or potentiometer
#define AUX2_PIN      A5  // Mode switch or potentiometer

// Status LED
#define LED_PIN       LED_BUILTIN

// ============================================================================
// JOYSTICK CALIBRATION VALUES
// ============================================================================

// Throttle (0-1023 raw -> 0-1000 output)
#define THROTTLE_MIN    0
#define THROTTLE_MAX    1023
#define THROTTLE_DEAD   10

// Roll/Pitch/Yaw center and deadband
#define STICK_CENTER    512
#define STICK_DEADBAND  20
#define STICK_MIN       0
#define STICK_MAX       1023

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// RF24 radio object
RF24 radio(TX_CE_PIN, TX_CSN_PIN);

// Control data to send
ControlData controlData;

// Timing
unsigned long lastTransmitTime = 0;
const unsigned long transmitInterval = 1000 / TX_FREQUENCY_HZ;  // 10ms for 100Hz

// LED blinking
unsigned long lastLedToggle = 0;
bool ledState = false;
bool radioInitialized = false;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  Serial.println(F("=== Drone Transmitter ==="));
  Serial.println(F("TMRh20 RF24 Library - NO ACK Mode"));
  
  // Setup LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize joystick pins as inputs
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(YAW_PIN, INPUT);
  pinMode(PITCH_PIN, INPUT);
  pinMode(ROLL_PIN, INPUT);
  pinMode(AUX1_PIN, INPUT);
  pinMode(AUX2_PIN, INPUT);
  
  // Initialize radio
  initRadio();
  
  // Initialize control data to safe values
  controlData.throttle = 0;
  controlData.roll = 0;
  controlData.pitch = 0;
  controlData.yaw = 0;
  controlData.aux1 = 0;  // Disarmed
  controlData.aux2 = 0;
  
  Serial.println(F("Transmitter ready!"));
  Serial.print(F("Packet size: "));
  Serial.print(sizeof(ControlData));
  Serial.println(F(" bytes"));
  Serial.print(F("TX Rate: "));
  Serial.print(TX_FREQUENCY_HZ);
  Serial.println(F(" Hz"));
}

// ============================================================================
// RADIO INITIALIZATION
// ============================================================================

void initRadio() {
  Serial.println(F("Initializing NRF24L01..."));
  
  // Start radio
  if (!radio.begin()) {
    Serial.println(F("ERROR: Radio not responding!"));
    Serial.println(F("Check wiring and power supply."));
    radioInitialized = false;
    return;
  }
  
  // Configure for low latency drone control
  radio.setChannel(RF_CHANNEL);
  radio.setDataRate(RF_DATA_RATE);
  radio.setPALevel(RF_PA_LEVEL);
  
  // Disable auto-acknowledgment for lowest latency
  // This is KEY for drone control - no waiting for ACK!
  radio.setAutoAck(false);
  
  // Disable retries (not needed without ACK)
  radio.setRetries(0, 0);
  
  // Set payload size (fixed size is faster than dynamic)
  radio.setPayloadSize(sizeof(ControlData));
  
  // Open writing pipe
  radio.openWritingPipe(RADIO_ADDRESS);
  
  // Stop listening (we're transmitting)
  radio.stopListening();
  
  // Power up radio
  radio.powerUp();
  delay(5);  // Let radio stabilize
  
  radioInitialized = true;
  
  // Print radio configuration
  Serial.println(F("Radio initialized successfully!"));
  Serial.print(F("Channel: "));
  Serial.println(RF_CHANNEL);
  Serial.print(F("Data Rate: "));
  Serial.println(RF_DATA_RATE == RF24_2MBPS ? "2Mbps" : 
                 RF_DATA_RATE == RF24_1MBPS ? "1Mbps" : "250Kbps");
  Serial.println(F("Auto-ACK: DISABLED (low latency mode)"));
  
  // Debug info
  radio.printDetails();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long currentTime = millis();
  
  // Transmit at fixed rate
  if (currentTime - lastTransmitTime >= transmitInterval) {
    lastTransmitTime = currentTime;
    
    // Read joysticks and update control data
    readJoysticks();
    
    // Calculate and set checksum
    setControlChecksum(controlData);
    
    // Transmit data
    if (radioInitialized) {
      transmitData();
    }
    
    // Debug output (every 500ms to not flood serial)
    static unsigned long lastDebug = 0;
    if (currentTime - lastDebug >= 500) {
      lastDebug = currentTime;
      printDebugInfo();
    }
  }
  
  // Blink LED to show status
  updateStatusLed(currentTime);
}

// ============================================================================
// JOYSTICK READING
// ============================================================================

void readJoysticks() {
  // Read raw analog values
  int rawThrottle = analogRead(THROTTLE_PIN);
  int rawYaw = analogRead(YAW_PIN);
  int rawPitch = analogRead(PITCH_PIN);
  int rawRoll = analogRead(ROLL_PIN);
  int rawAux1 = analogRead(AUX1_PIN);
  int rawAux2 = analogRead(AUX2_PIN);
  
  // Process throttle (0-1000, no center)
  // Reverse if needed: rawThrottle = 1023 - rawThrottle;
  controlData.throttle = mapThrottle(rawThrottle);
  
  // Process roll (-500 to +500, with center and deadband)
  // Reverse if needed: rawRoll = 1023 - rawRoll;
  controlData.roll = mapStick(rawRoll);
  
  // Process pitch (-500 to +500, with center and deadband)
  // Reverse if needed: rawPitch = 1023 - rawPitch;
  controlData.pitch = mapStick(rawPitch);
  
  // Process yaw (-500 to +500, with center and deadband)
  // Reverse if needed: rawYaw = 1023 - rawYaw;
  controlData.yaw = mapStick(rawYaw);
  
  // Process aux channels (0-255)
  controlData.aux1 = map(rawAux1, 0, 1023, 0, 255);
  controlData.aux2 = map(rawAux2, 0, 1023, 0, 255);
}

// Map throttle with deadband at bottom
uint16_t mapThrottle(int raw) {
  // Apply constraints
  raw = constrain(raw, THROTTLE_MIN, THROTTLE_MAX);
  
  // Small deadband at bottom
  if (raw < THROTTLE_DEAD) {
    return 0;
  }
  
  // Map to 0-1000
  return map(raw, THROTTLE_DEAD, THROTTLE_MAX, 0, 1000);
}

// Map stick with center and deadband
int16_t mapStick(int raw) {
  // Calculate deviation from center
  int deviation = raw - STICK_CENTER;
  
  // Apply deadband
  if (abs(deviation) < STICK_DEADBAND) {
    return 0;
  }
  
  // Map to -500 to +500
  if (deviation > 0) {
    return map(deviation, STICK_DEADBAND, STICK_CENTER, 0, 500);
  } else {
    return map(deviation, -STICK_CENTER, -STICK_DEADBAND, -500, 0);
  }
}

// ============================================================================
// DATA TRANSMISSION
// ============================================================================

void transmitData() {
  // Send data (non-blocking in NO ACK mode)
  bool success = radio.write(&controlData, sizeof(ControlData));
  
  // Note: In NO ACK mode, write() always returns true
  // This is normal - we sacrifice delivery confirmation for speed
  (void)success;  // Suppress unused warning
}

// ============================================================================
// STATUS LED
// ============================================================================

void updateStatusLed(unsigned long currentTime) {
  unsigned long blinkInterval;
  
  if (!radioInitialized) {
    // Fast blink = radio error
    blinkInterval = 100;
  } else {
    // Slow blink = normal operation
    blinkInterval = 500;
  }
  
  if (currentTime - lastLedToggle >= blinkInterval) {
    lastLedToggle = currentTime;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

void printDebugInfo() {
  Serial.print(F("THR:"));
  Serial.print(controlData.throttle);
  Serial.print(F(" R:"));
  Serial.print(controlData.roll);
  Serial.print(F(" P:"));
  Serial.print(controlData.pitch);
  Serial.print(F(" Y:"));
  Serial.print(controlData.yaw);
  Serial.print(F(" A1:"));
  Serial.print(controlData.aux1);
  Serial.print(F(" A2:"));
  Serial.println(controlData.aux2);
}
