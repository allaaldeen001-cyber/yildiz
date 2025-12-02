/*
 * Drone Receiver Code
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

// Radio pipe address (must match transmitter)
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

DroneCommand receivedCommand;
unsigned long lastReceiveTime = 0;
const unsigned long TIMEOUT_MS = 500; // Consider connection lost after 500ms

// Motor pins (adjust based on your ESC/motor setup)
#define MOTOR_FL 3  // Front Left
#define MOTOR_FR 5  // Front Right
#define MOTOR_BL 6  // Back Left
#define MOTOR_BR 9  // Back Right

// Motor mixing constants
#define MIN_THROTTLE 0      // Minimum throttle value
#define MAX_THROTTLE 1000   // Maximum throttle value
#define MIN_PWM 0           // Minimum PWM output (adjust for your ESC)
#define MAX_PWM 255         // Maximum PWM output (adjust for your ESC)
#define MIX_SCALE 0.3       // Mixing scale factor (30% of control input affects motors)

// Calculate checksum
uint8_t calculateChecksum(DroneCommand* cmd) {
  uint8_t sum = 0;
  uint8_t* data = (uint8_t*)cmd;
  for (int i = 0; i < sizeof(DroneCommand) - 1; i++) {
    sum ^= data[i];
  }
  return sum;
}

// Safety: Stop all motors
void stopAllMotors() {
  analogWrite(MOTOR_FL, MIN_PWM);
  analogWrite(MOTOR_FR, MIN_PWM);
  analogWrite(MOTOR_BL, MIN_PWM);
  analogWrite(MOTOR_BR, MIN_PWM);
}

// Calculate motor speeds based on control inputs (Quadcopter X configuration)
// Motor layout:
//     FL    FR
//       \  /
//        \/
//        /\
//       /  \
//     BL    BR
void calculateMotorSpeeds(uint16_t throttle, int16_t pitch, int16_t roll, int16_t yaw) {
  // Safety: If throttle is zero, stop all motors
  if (throttle == 0) {
    stopAllMotors();
    return;
  }
  
  // Map throttle to PWM range (0-1000 -> MIN_PWM-MAX_PWM)
  float baseSpeed = map(throttle, MIN_THROTTLE, MAX_THROTTLE, MIN_PWM, MAX_PWM);
  
  // Normalize control inputs (-500 to +500 -> -1.0 to +1.0)
  float pitchNorm = (float)pitch / 500.0;
  float rollNorm = (float)roll / 500.0;
  float yawNorm = (float)yaw / 500.0;
  
  // Apply mixing algorithm for X-configuration quadcopter
  // Each motor gets base throttle plus corrections for attitude control
  float fl = baseSpeed + (pitchNorm * MIX_SCALE * baseSpeed) - (rollNorm * MIX_SCALE * baseSpeed) - (yawNorm * MIX_SCALE * baseSpeed);
  float fr = baseSpeed + (pitchNorm * MIX_SCALE * baseSpeed) + (rollNorm * MIX_SCALE * baseSpeed) + (yawNorm * MIX_SCALE * baseSpeed);
  float bl = baseSpeed - (pitchNorm * MIX_SCALE * baseSpeed) - (rollNorm * MIX_SCALE * baseSpeed) + (yawNorm * MIX_SCALE * baseSpeed);
  float br = baseSpeed - (pitchNorm * MIX_SCALE * baseSpeed) + (rollNorm * MIX_SCALE * baseSpeed) - (yawNorm * MIX_SCALE * baseSpeed);
  
  // Constrain values to valid PWM range
  int flPWM = constrain((int)fl, MIN_PWM, MAX_PWM);
  int frPWM = constrain((int)fr, MIN_PWM, MAX_PWM);
  int blPWM = constrain((int)bl, MIN_PWM, MAX_PWM);
  int brPWM = constrain((int)br, MIN_PWM, MAX_PWM);
  
  // Write to motors
  analogWrite(MOTOR_FL, flPWM);
  analogWrite(MOTOR_FR, frPWM);
  analogWrite(MOTOR_BL, blPWM);
  analogWrite(MOTOR_BR, brPWM);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize motor pins
  pinMode(MOTOR_FL, OUTPUT);
  pinMode(MOTOR_FR, OUTPUT);
  pinMode(MOTOR_BL, OUTPUT);
  pinMode(MOTOR_BR, OUTPUT);
  
  // Safety: Start with motors stopped
  stopAllMotors();
  
  // Initialize radio
  if (!radio.begin()) {
    Serial.println(F("Radio hardware not responding!"));
    while (1) {} // Hold in infinite loop
  }
  
  // Configure radio for optimal drone communication (MUST match transmitter)
  radio.setPALevel(RF24_PA_MAX);           // Maximum power for range
  radio.setDataRate(RF24_250KBPS);         // Lower data rate = better range/reliability
  radio.setChannel(76);                    // Channel 76 (2.476 GHz)
  radio.setRetries(3, 5);                  // Retry 3 times with 5*250us delay (ACK enabled)
  radio.setAutoAck(true);                  // Enable ACK for perfect communication
  radio.setCRCLength(RF24_CRC_16);         // 16-bit CRC for error detection
  radio.setAddressWidth(5);                // 5-byte address width
  
  // Open reading pipe
  radio.openReadingPipe(0, address);
  
  // Start listening (receiver mode)
  radio.startListening();
  
  // Print configuration
  Serial.println(F("=== Drone Receiver Initialized ==="));
  Serial.print(F("Data Rate: "));
  Serial.println(F("250KBPS"));
  Serial.print(F("Power Level: "));
  Serial.println(F("MAX"));
  Serial.print(F("ACK: "));
  Serial.println(F("ENABLED"));
  Serial.println(F("Waiting for commands..."));
  
  // Initialize command structure
  receivedCommand.throttle = 0;
  receivedCommand.pitch = 0;
  receivedCommand.roll = 0;
  receivedCommand.yaw = 0;
  receivedCommand.mode = 0;
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check for incoming data
  if (radio.available()) {
    // Read the data
    radio.read(&receivedCommand, sizeof(DroneCommand));
    
    // Verify checksum
    uint8_t calculatedChecksum = calculateChecksum(&receivedCommand);
    if (calculatedChecksum == receivedCommand.checksum) {
      // Valid data received
      lastReceiveTime = currentTime;
      
      // Apply motor control
      calculateMotorSpeeds(
        receivedCommand.throttle,
        receivedCommand.pitch,
        receivedCommand.roll,
        receivedCommand.yaw
      );
      
      // Optional: Print status every second
      static unsigned long lastPrint = 0;
      if (currentTime - lastPrint >= 1000) {
        lastPrint = currentTime;
        Serial.print(F("Throttle: "));
        Serial.print(receivedCommand.throttle);
        Serial.print(F(" | Pitch: "));
        Serial.print(receivedCommand.pitch);
        Serial.print(F(" | Roll: "));
        Serial.print(receivedCommand.roll);
        Serial.print(F(" | Yaw: "));
        Serial.print(receivedCommand.yaw);
        Serial.print(F(" | Signal: OK"));
        Serial.println();
      }
    } else {
      // Checksum error - ignore packet
      Serial.println(F("Checksum error - packet ignored"));
    }
  }
  
  // Safety: Stop motors if no signal received for too long
  if (currentTime - lastReceiveTime > TIMEOUT_MS) {
    if (lastReceiveTime > 0) { // Only stop if we've received data before
      stopAllMotors();
      static unsigned long lastTimeoutMsg = 0;
      if (currentTime - lastTimeoutMsg >= 1000) {
        Serial.println(F("Signal lost - Motors stopped for safety"));
        lastTimeoutMsg = currentTime;
      }
    }
  }
}
