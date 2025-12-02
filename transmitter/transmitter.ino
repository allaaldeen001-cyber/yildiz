/*
 * DRONE TRANSMITTER (Controller)
 * ==============================
 * Using TMRh20 RF24 Library with ACK enabled for reliable communication
 * 
 * Hardware:
 * - Arduino Nano/Uno
 * - NRF24L01+ module
 * - 2x Joysticks (4 analog inputs)
 * - Optional: Switches for flight modes
 * 
 * NRF24L01 Wiring:
 * CE  -> D9
 * CSN -> D10
 * MOSI -> D11
 * MISO -> D12
 * SCK -> D13
 * VCC -> 3.3V (IMPORTANT: Use external 3.3V regulator with capacitor!)
 * GND -> GND
 */

#include <SPI.h>
#include <RF24.h>

// NRF24L01 Configuration
#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

// Communication pipes (must match receiver)
const byte address[6] = "DRON1";

// Joystick pins
#define THROTTLE_PIN A0  // Left stick vertical
#define YAW_PIN A1       // Left stick horizontal
#define PITCH_PIN A2     // Right stick vertical
#define ROLL_PIN A3      // Right stick horizontal

// Optional switches
#define ARM_SWITCH 7     // Arming switch
#define MODE_SWITCH 8    // Flight mode switch

// Control data structure (must match receiver)
struct ControlData {
  uint16_t throttle;   // 1000-2000 (PWM-like values)
  uint16_t yaw;        // 1000-2000
  uint16_t pitch;      // 1000-2000
  uint16_t roll;       // 1000-2000
  uint8_t arm;         // 0=disarmed, 1=armed
  uint8_t mode;        // 0=stabilize, 1=acro
  uint32_t timestamp;  // For latency checking
} controlData;

// Telemetry data from drone
struct TelemetryData {
  float battery;       // Battery voltage
  int16_t rssi;        // Signal strength
  uint8_t status;      // Flight status
  uint32_t timestamp;  // Response time
} telemetryData;

// Status tracking
unsigned long lastTransmit = 0;
unsigned long lastReceive = 0;
const unsigned long TX_INTERVAL = 20;  // 50Hz update rate
unsigned long packetsLost = 0;
unsigned long packetsSent = 0;

void setup() {
  Serial.begin(115200);
  Serial.println(F("=== DRONE TRANSMITTER ==="));
  Serial.println(F("Using TMRh20 RF24 Library"));
  Serial.println(F("ACK Enabled for Reliability"));
  
  // Initialize pins
  pinMode(ARM_SWITCH, INPUT_PULLUP);
  pinMode(MODE_SWITCH, INPUT_PULLUP);
  
  // Initialize NRF24L01
  if (!radio.begin()) {
    Serial.println(F("ERROR: NRF24L01 not found!"));
    while (1) {
      // Blink LED or halt
      delay(1000);
    }
  }
  
  // Configure for optimal drone control
  radio.setPALevel(RF24_PA_MAX);        // Maximum power for range
  radio.setDataRate(RF24_250KBPS);      // 250kbps for better range and reliability
  radio.setChannel(108);                // Channel 108 (2.508 GHz) - less interference
  radio.setRetries(3, 5);               // 3x250us delay, 5 retries (fast retransmit)
  radio.enableAckPayload();             // Enable ACK payloads for telemetry
  radio.enableDynamicPayloads();        // Dynamic payload sizes
  radio.setAutoAck(true);               // Enable auto-acknowledgment (CRITICAL)
  radio.setCRCLength(RF24_CRC_16);      // 16-bit CRC for error detection
  
  // Open writing pipe
  radio.openWritingPipe(address);
  radio.stopListening();                // TX mode
  
  Serial.println(F("NRF24L01 Initialized Successfully!"));
  Serial.println(F("Configuration:"));
  Serial.println(F("  - PA Level: MAX"));
  Serial.println(F("  - Data Rate: 250kbps"));
  Serial.println(F("  - Channel: 108"));
  Serial.println(F("  - Auto ACK: ENABLED"));
  Serial.println(F("  - CRC: 16-bit"));
  Serial.println(F(""));
  Serial.println(F("Controls:"));
  Serial.println(F("  Left Stick: Throttle (V) / Yaw (H)"));
  Serial.println(F("  Right Stick: Pitch (V) / Roll (H)"));
  Serial.println(F(""));
  
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read controls at fixed rate (50Hz)
  if (currentTime - lastTransmit >= TX_INTERVAL) {
    lastTransmit = currentTime;
    
    // Read joysticks (0-1023) and map to 1000-2000
    controlData.throttle = map(analogRead(THROTTLE_PIN), 0, 1023, 1000, 2000);
    controlData.yaw = map(analogRead(YAW_PIN), 0, 1023, 1000, 2000);
    controlData.pitch = map(analogRead(PITCH_PIN), 0, 1023, 1000, 2000);
    controlData.roll = map(analogRead(ROLL_PIN), 0, 1023, 1000, 2000);
    
    // Apply deadband (center stick = 1500 ± 10)
    controlData.yaw = applyDeadband(controlData.yaw, 1500, 10);
    controlData.pitch = applyDeadband(controlData.pitch, 1500, 10);
    controlData.roll = applyDeadband(controlData.roll, 1500, 10);
    
    // Read switches
    controlData.arm = !digitalRead(ARM_SWITCH);   // Inverted (pullup)
    controlData.mode = !digitalRead(MODE_SWITCH);
    
    // Add timestamp
    controlData.timestamp = currentTime;
    
    // Transmit data with ACK
    bool success = radio.write(&controlData, sizeof(controlData));
    packetsSent++;
    
    if (success) {
      // Check if ACK payload available (telemetry from drone)
      if (radio.available()) {
        radio.read(&telemetryData, sizeof(telemetryData));
        lastReceive = currentTime;
        
        // Calculate round-trip time
        uint32_t rtt = currentTime - telemetryData.timestamp;
        
        // Print telemetry periodically
        static unsigned long lastPrint = 0;
        if (currentTime - lastPrint >= 500) {  // Every 500ms
          lastPrint = currentTime;
          Serial.print(F("Battery: ")); Serial.print(telemetryData.battery, 2); Serial.print(F("V | "));
          Serial.print(F("RTT: ")); Serial.print(rtt); Serial.print(F("ms | "));
          Serial.print(F("Lost: ")); Serial.print(packetsLost); Serial.print(F("/"));
          Serial.print(packetsSent); Serial.print(F(" ("));
          Serial.print((float)packetsLost / packetsSent * 100, 1); Serial.println(F("%)"));
        }
      }
    } else {
      // Transmission failed
      packetsLost++;
      
      // Warning if connection lost
      if (currentTime - lastReceive > 1000) {
        Serial.println(F("WARNING: NO CONNECTION TO DRONE!"));
      }
    }
  }
  
  // Print control values periodically for debugging
  static unsigned long lastDebug = 0;
  if (currentTime - lastDebug >= 1000) {  // Every 1 second
    lastDebug = currentTime;
    Serial.print(F("THR:")); Serial.print(controlData.throttle);
    Serial.print(F(" YAW:")); Serial.print(controlData.yaw);
    Serial.print(F(" PIT:")); Serial.print(controlData.pitch);
    Serial.print(F(" ROL:")); Serial.print(controlData.roll);
    Serial.print(F(" ARM:")); Serial.print(controlData.arm);
    Serial.print(F(" MODE:")); Serial.println(controlData.mode);
  }
}

// Apply deadband to center sticks
uint16_t applyDeadband(uint16_t value, uint16_t center, uint16_t deadband) {
  if (abs((int)value - (int)center) < deadband) {
    return center;
  }
  return value;
}
