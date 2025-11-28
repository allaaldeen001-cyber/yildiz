/*
 * DIY Drone Remote Controller
 * 
 * Hardware: Arduino Nano, NRF24L01, 2x Joysticks, Buttons, Switches
 * 
 * PIN ASSIGNMENTS:
 * ================
 * NRF24L01:
 *   D9  = CE
 *   D10 = CSN
 *   D11 = MOSI (SPI)
 *   D12 = MISO (SPI)
 *   D13 = SCK (SPI)
 * 
 * Joysticks:
 *   A0 = Throttle (Left Y-axis) - UP increases thrust
 *   A1 = Yaw (Left X-axis) - LEFT = CCW, RIGHT = CW
 *   A2 = Pitch (Right Y-axis) - UP = forward
 *   A3 = Roll (Right X-axis) - LEFT = tilt left
 * 
 * Buttons (directly on RC - mirrors info to FC):
 *   D4 = Button 1 (Calibration request)
 *   D5 = Button 2 (Smooth motor start request)
 * 
 * Switches:
 *   D3 = Switch 1 (Arm/Disarm: HIGH=Disarmed, LOW=Armed)
 *   D2 = Switch 2 (Altitude Hold: LOW=Active)
 * 
 * Outputs (Optional):
 *   D6 = LED (connection status)
 *   D7 = Buzzer (connection/ACK feedback)
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

// ==================== PIN DEFINITIONS ====================
// NRF24L01 Pins
const int PIN_NRF_CE  = 9;
const int PIN_NRF_CSN = 10;

// Joystick Pins
const int PIN_THROTTLE = A0;  // Left Y
const int PIN_YAW      = A1;  // Left X
const int PIN_PITCH    = A2;  // Right Y
const int PIN_ROLL     = A3;  // Right X

// Button/Switch Pins
const int PIN_BUTTON1  = 4;   // Calibration
const int PIN_BUTTON2  = 5;   // Motor start
const int PIN_SWITCH1  = 3;   // Arm/Disarm
const int PIN_SWITCH2  = 2;   // Altitude Hold

// Output Pins (optional)
const int PIN_LED     = 6;
const int PIN_BUZZER  = 7;

// ==================== NRF24L01 CONFIG ====================
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);
const uint64_t PIPE_ADDRESS = 0xF0F0F0F0E1LL;
const uint8_t NRF_CHANNEL = 108;  // Must match Flight Controller

// ==================== DATA STRUCTURES ====================
struct ControlPackage {
  int16_t thrust = 0;
  float x = 0;        // Roll
  float y = 0;        // Pitch  
  float z = 0;        // Yaw
  uint16_t id = 0;
  bool but1 = 1;      // Calibration button
  bool but2 = 1;      // Motor start button
  bool switch1 = 1;   // Arm switch
  bool switch2 = 1;   // Altitude hold
};

struct AckPackage {
  uint16_t lastId = 0;
  bool armed = false;
  bool altitudeHold = false;
  uint8_t batteryPercent = 100;
  int16_t currentAltitude = 0;
};

ControlPackage txPackage;
AckPackage ackPackage;

// ==================== JOYSTICK CALIBRATION ====================
// These values may need adjustment for your specific joysticks
// Center values (when joystick is at rest)
float calRoll     = -512;   // Right X center
float calPitch    = -507;   // Right Y center
float calYaw      = -512;   // Left X center
float calThrottle = -500;   // Left Y minimum (throttle doesn't center)

// Scaling factors
float scaleRoll     = 0.1;
float scalePitch    = -0.1;  // Inverted
float scaleYaw      = -0.1;  // Inverted
float scaleThrottle = 1.5;

// Offsets
float offsetRoll     = 0;
float offsetPitch    = 0;
float offsetYaw      = 0;
float offsetThrottle = 1300;

// ==================== SMOOTHING FILTERS ====================
Smoothed<float> smoothThrottle;
Smoothed<float> smoothYaw;
Smoothed<float> smoothPitch;
Smoothed<float> smoothRoll;

// ==================== STATE VARIABLES ====================
uint16_t packetId = 0;
bool connected = false;
unsigned long lastAckTime = 0;
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 10;  // 100 Hz transmit rate
const unsigned long ACK_TIMEOUT = 1000;  // 1 second

// Raw joystick readings
int rawThrottle, rawYaw, rawPitch, rawRoll;
float filteredThrottle, filteredYaw, filteredPitch, filteredRoll;

// ==================== FUNCTION PROTOTYPES ====================
void initializeHardware();
void initializeNRF();
void readJoysticks();
void readButtons();
void sendPackage();
void processAck();
void updateStatus();
void printDebug();

// ==================== SETUP ====================
void setup() {
  Serial.begin(57600);
  Serial.println(F("=== Drone Remote Controller ==="));
  
  initializeHardware();
  initializeNRF();
  
  // Startup indication
  if (PIN_BUZZER > 0) {
    tone(PIN_BUZZER, 1500, 200);
  }
  if (PIN_LED > 0) {
    digitalWrite(PIN_LED, HIGH);
    delay(200);
    digitalWrite(PIN_LED, LOW);
  }
  
  Serial.println(F("Remote Ready"));
}

// ==================== MAIN LOOP ====================
void loop() {
  readJoysticks();
  readButtons();
  
  // Send at regular intervals
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    sendPackage();
    lastSendTime = millis();
  }
  
  // Check for ACK
  processAck();
  
  // Update connection status
  updateStatus();
  
  // Debug output
  printDebug();
}

// ==================== INITIALIZATION ====================
void initializeHardware() {
  // Configure button/switch pins
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);
  pinMode(PIN_SWITCH1, INPUT_PULLUP);
  pinMode(PIN_SWITCH2, INPUT_PULLUP);
  
  // Configure output pins
  if (PIN_LED > 0) {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
  }
  if (PIN_BUZZER > 0) {
    pinMode(PIN_BUZZER, OUTPUT);
  }
  
  // Initialize smoothing filters with exponential smoothing
  smoothThrottle.begin(SMOOTHED_EXPONENTIAL, 3);
  smoothYaw.begin(SMOOTHED_EXPONENTIAL, 3);
  smoothPitch.begin(SMOOTHED_EXPONENTIAL, 3);
  smoothRoll.begin(SMOOTHED_EXPONENTIAL, 3);
  
  Serial.println(F("Hardware initialized"));
}

void initializeNRF() {
  radio.begin();
  radio.setChannel(NRF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);           // Enable ACK
  radio.enableAckPayload();         // Enable ACK payload
  radio.setRetries(5, 15);          // 5x250us delay, 15 retries
  radio.openWritingPipe(PIPE_ADDRESS);
  radio.stopListening();            // Transmitter mode
  
  Serial.println(F("NRF24L01 initialized with ACK"));
  Serial.print(F("Channel: "));
  Serial.println(NRF_CHANNEL);
}

// ==================== INPUT READING ====================
void readJoysticks() {
  // Read raw analog values
  rawThrottle = analogRead(PIN_THROTTLE);
  rawYaw      = analogRead(PIN_YAW);
  rawPitch    = 1023 - analogRead(PIN_PITCH);  // Invert for correct direction
  rawRoll     = analogRead(PIN_ROLL);
  
  // Ensure throttle doesn't go negative
  rawThrottle = max(rawThrottle, 0);
  
  // Apply smoothing
  smoothThrottle.add(rawThrottle);
  smoothYaw.add(rawYaw);
  smoothPitch.add(rawPitch);
  smoothRoll.add(rawRoll);
  
  // Get filtered values
  filteredThrottle = smoothThrottle.get();
  filteredYaw      = smoothYaw.get();
  filteredPitch    = smoothPitch.get();
  filteredRoll     = smoothRoll.get();
  
  // Apply calibration and scaling
  txPackage.thrust = (int16_t)((filteredThrottle + calThrottle) * scaleThrottle + offsetThrottle);
  txPackage.z      = (filteredYaw + calYaw) * scaleYaw + offsetYaw;
  txPackage.y      = (filteredPitch + calPitch) * scalePitch + offsetPitch;
  txPackage.x      = (filteredRoll + calRoll) * scaleRoll + offsetRoll;
  
  // Constrain thrust to valid range
  txPackage.thrust = constrain(txPackage.thrust, 1000, 2000);
}

void readButtons() {
  txPackage.but1    = digitalRead(PIN_BUTTON1);   // Calibration (active LOW)
  txPackage.but2    = digitalRead(PIN_BUTTON2);   // Motor start (active LOW)
  txPackage.switch1 = digitalRead(PIN_SWITCH1);   // Arm (LOW=Armed)
  txPackage.switch2 = digitalRead(PIN_SWITCH2);   // Alt hold (LOW=Active)
}

// ==================== COMMUNICATION ====================
void sendPackage() {
  // Increment packet ID
  txPackage.id = packetId++;
  
  // Transmit package
  bool success = radio.write(&txPackage, sizeof(txPackage));
  
  if (success) {
    // Check if ACK payload is available
    if (radio.isAckPayloadAvailable()) {
      radio.read(&ackPackage, sizeof(ackPackage));
      lastAckTime = millis();
      
      if (!connected) {
        connected = true;
        Serial.println(F("Connected to Flight Controller!"));
        if (PIN_BUZZER > 0) {
          tone(PIN_BUZZER, 2000, 100);
          delay(150);
          tone(PIN_BUZZER, 2500, 100);
        }
      }
    }
  }
}

void processAck() {
  // ACK is already processed in sendPackage()
  // This function can be used for additional ACK processing
  
  if (connected && ackPackage.lastId > 0) {
    // Validate ACK corresponds to our packet
    // Could implement packet loss detection here
  }
}

void updateStatus() {
  // Check connection timeout
  if (connected && (millis() - lastAckTime > ACK_TIMEOUT)) {
    connected = false;
    Serial.println(F("Connection lost!"));
    if (PIN_BUZZER > 0) {
      tone(PIN_BUZZER, 500, 500);
    }
  }
  
  // Update LED status
  if (PIN_LED > 0) {
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    
    if (connected) {
      // Solid LED when connected and armed
      if (ackPackage.armed) {
        digitalWrite(PIN_LED, HIGH);
      } else {
        // Slow blink when connected but disarmed
        if (millis() - lastBlink > 500) {
          ledState = !ledState;
          digitalWrite(PIN_LED, ledState);
          lastBlink = millis();
        }
      }
    } else {
      // Fast blink when disconnected
      if (millis() - lastBlink > 100) {
        ledState = !ledState;
        digitalWrite(PIN_LED, ledState);
        lastBlink = millis();
      }
    }
  }
}

// ==================== DEBUG OUTPUT ====================
void printDebug() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 250) return;
  lastPrint = millis();
  
  Serial.print(F("T:"));
  Serial.print(txPackage.thrust);
  Serial.print(F(" Y:"));
  Serial.print(txPackage.z, 1);
  Serial.print(F(" P:"));
  Serial.print(txPackage.y, 1);
  Serial.print(F(" R:"));
  Serial.print(txPackage.x, 1);
  Serial.print(F(" | S1:"));
  Serial.print(txPackage.switch1);
  Serial.print(F(" S2:"));
  Serial.print(txPackage.switch2);
  Serial.print(F(" B1:"));
  Serial.print(txPackage.but1);
  Serial.print(F(" B2:"));
  Serial.print(txPackage.but2);
  
  if (connected) {
    Serial.print(F(" | FC:"));
    Serial.print(ackPackage.armed ? F("ARM") : F("DSRM"));
    Serial.print(F(" Bat:"));
    Serial.print(ackPackage.batteryPercent);
    Serial.print(F("% Alt:"));
    Serial.print(ackPackage.currentAltitude);
    Serial.print(F("cm"));
  } else {
    Serial.print(F(" | DISCONNECTED"));
  }
  
  Serial.println();
}
