/*
 * Drone Transmitter Code
 * Using TMRh20 RF24 Library for nRF24L01
 * ACK enabled for perfect communication reliability
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Define radio pins
#define CE_PIN 9
#define CSN_PIN 10

// Create RF24 object
RF24 radio(CE_PIN, CSN_PIN);

// Radio pipe address (must match receiver)
const byte address[6] = "00001";

// Control structure for drone commands
struct DroneCommand {
  uint16_t throttle;    // 0-1000 (0-100%)
  int16_t pitch;        // -500 to +500 (forward/backward)
  int16_t roll;         // -500 to +500 (left/right)
  int16_t yaw;          // -500 to +500 (rotate)
  uint8_t mode;         // Flight mode
  uint8_t checksum;     // Data integrity check
};

DroneCommand command;
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 20; // 50Hz update rate (20ms)

// Calculate checksum
uint8_t calculateChecksum(DroneCommand* cmd) {
  uint8_t sum = 0;
  uint8_t* data = (uint8_t*)cmd;
  for (int i = 0; i < sizeof(DroneCommand) - 1; i++) {
    sum ^= data[i];
  }
  return sum;
}

void setup() {
  Serial.begin(115200);
  
  // Initialize radio
  if (!radio.begin()) {
    Serial.println(F("Radio hardware not responding!"));
    while (1) {} // Hold in infinite loop
  }
  
  // Configure radio for optimal drone communication
  radio.setPALevel(RF24_PA_MAX);           // Maximum power for range
  radio.setDataRate(RF24_250KBPS);         // Lower data rate = better range/reliability
  radio.setChannel(76);                    // Channel 76 (2.476 GHz)
  radio.setRetries(3, 5);                  // Retry 3 times with 5*250us delay (ACK enabled)
  radio.setAutoAck(true);                  // Enable ACK for perfect communication
  radio.setCRCLength(RF24_CRC_16);         // 16-bit CRC for error detection
  radio.setAddressWidth(5);                // 5-byte address width
  
  // Open writing pipe
  radio.openWritingPipe(address);
  
  // Stop listening (transmitter mode)
  radio.stopListening();
  
  // Print configuration
  Serial.println(F("=== Drone Transmitter Initialized ==="));
  Serial.print(F("Data Rate: "));
  Serial.println(F("250KBPS"));
  Serial.print(F("Power Level: "));
  Serial.println(F("MAX"));
  Serial.print(F("ACK: "));
  Serial.println(F("ENABLED"));
  Serial.print(F("Update Rate: "));
  Serial.print(1000/SEND_INTERVAL);
  Serial.println(F(" Hz"));
  Serial.println(F("Ready to send commands..."));
  
  // Initialize command structure
  command.throttle = 0;
  command.pitch = 0;
  command.roll = 0;
  command.yaw = 0;
  command.mode = 0;
}

void loop() {
  unsigned long currentTime = millis();
  
  // Send at fixed interval for smooth control
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentTime;
    
    // Read control inputs (replace with your actual input reading code)
    // For now, using example values - replace with joystick/controller reading
    readControlInputs();
    
    // Calculate checksum
    command.checksum = calculateChecksum(&command);
    
    // Send command with ACK
    bool success = radio.write(&command, sizeof(DroneCommand));
    
    // Monitor connection quality
    if (!success) {
      Serial.println(F("TX Failed - No ACK received"));
    }
    
    // Optional: Print status every second
    static unsigned long lastPrint = 0;
    if (currentTime - lastPrint >= 1000) {
      lastPrint = currentTime;
      Serial.print(F("Throttle: "));
      Serial.print(command.throttle);
      Serial.print(F(" | Pitch: "));
      Serial.print(command.pitch);
      Serial.print(F(" | Roll: "));
      Serial.print(command.roll);
      Serial.print(F(" | Yaw: "));
      Serial.println(command.yaw);
    }
  }
}

void readControlInputs() {
  // TODO: Replace with your actual input reading code
  // Example: Read from joysticks, potentiometers, or wireless controller
  
  // Example values (replace with actual readings):
  // command.throttle = map(analogRead(A0), 0, 1023, 0, 1000);
  // command.pitch = map(analogRead(A1), 0, 1023, -500, 500);
  // command.roll = map(analogRead(A2), 0, 1023, -500, 500);
  // command.yaw = map(analogRead(A3), 0, 1023, -500, 500);
  
  // For testing, you can use serial input or keep current values
}
