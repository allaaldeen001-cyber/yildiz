/*
 * ═══════════════════════════════════════════════════════════════════════════
 * QUADCOPTER REMOTE CONTROLLER
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * Hardware: Arduino Nano
 * Update Rate: 250Hz (4ms per cycle)
 * 
 * Components:
 * - NRF24L01+ (Radio) - CE:D9, CSN:D10
 * - Left Joystick - VRy:A0 (Throttle), VRx:A1 (Yaw)
 * - Right Joystick - VRy:A2 (Pitch), VRx:A3 (Roll)
 * - 4x Push Buttons - D4, D5, D6, D7
 * - 2x Toggle Switches - D2, D3
 * 
 * Button Functions:
 * - Button 1 (D4): Calibrate sensors
 * - Button 2 (D5): Motor test
 * - Button 3 (D6): Smooth landing
 * - Button 4 (D7): Smooth takeoff
 * 
 * Switch Functions:
 * - SW1 (D2): Altitude Hold ON/OFF
 * - SW2 (D3): ANGLE/ACRO mode
 * 
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <SPI.h>
#include <RF24.h>

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════

// Radio
#define RADIO_CE_PIN    9   // NRF24L01 CE
#define RADIO_CSN_PIN   10  // NRF24L01 CSN

// Joysticks (Analog)
#define THROTTLE_PIN    A0  // Left joystick Y (up/down)
#define YAW_PIN         A1  // Left joystick X (left/right)
#define PITCH_PIN       A2  // Right joystick Y (forward/back)
#define ROLL_PIN        A3  // Right joystick X (left/right)

// Buttons (Active LOW with internal pullup)
#define BTN1_PIN        4   // Calibrate
#define BTN2_PIN        5   // Motor Test
#define BTN3_PIN        6   // Landing
#define BTN4_PIN        7   // Takeoff

// Toggle Switches (Active LOW)
#define SW1_PIN         2   // Altitude Hold
#define SW2_PIN         3   // ANGLE/ACRO mode

// ═══════════════════════════════════════════════════════════════════════════
// RADIO DATA STRUCTURE (must match Flight Controller)
// ═══════════════════════════════════════════════════════════════════════════
struct RadioPacket {
  int16_t throttle;   // 0-1000
  int16_t yaw;        // -500 to +500
  int16_t pitch;      // -500 to +500
  int16_t roll;       // -500 to +500
  uint8_t sw1;        // HIGH or LOW
  uint8_t sw2;        // HIGH or LOW
  uint8_t btn1;       // HIGH or LOW
  uint8_t btn2;       // HIGH or LOW
  uint8_t btn3;       // HIGH or LOW
  uint8_t btn4;       // HIGH or LOW
};

// ═══════════════════════════════════════════════════════════════════════════
// GLOBAL OBJECTS & VARIABLES
// ═══════════════════════════════════════════════════════════════════════════
RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
const uint64_t radioAddress = 0xF0F0F0F0E1LL;

RadioPacket txData;

// Joystick calibration (center points)
int throttleCenter = 512;
int yawCenter = 512;
int pitchCenter = 512;
int rollCenter = 512;

// Joystick deadzone
const int DEADZONE = 20;

// Timing
unsigned long loopTimer = 0;
unsigned long currentTime = 0;

// Status
bool radioOK = true;
unsigned long lastSuccessfulSend = 0;

// ═══════════════════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);
  
  Serial.println(F("\n╔════════════════════════════════════════════════════╗"));
  Serial.println(F("║      QUADCOPTER REMOTE CONTROLLER - 250Hz TX      ║"));
  Serial.println(F("╚════════════════════════════════════════════════════╝"));
  
  // Initialize digital inputs with pullup
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(BTN3_PIN, INPUT_PULLUP);
  pinMode(BTN4_PIN, INPUT_PULLUP);
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);
  
  // Initialize radio
  initRadio();
  
  // Calibrate joysticks
  calibrateJoysticks();
  
  Serial.println(F("\n✅ REMOTE CONTROLLER READY!"));
  Serial.println(F("   Transmitting at 250Hz...\n"));
  
  // Print control mapping
  printControlMapping();
  
  // Start loop timer
  loopTimer = micros();
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP - 250Hz (4ms)
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
  currentTime = millis();
  
  // 1. READ JOYSTICKS (0.2ms)
  readJoysticks();
  
  // 2. READ BUTTONS & SWITCHES (0.1ms)
  readButtons();
  
  // 3. TRANSMIT DATA (0.5ms)
  transmitData();
  
  // 4. DEBUG OUTPUT (every 100ms)
  static unsigned long lastDebug = 0;
  if (currentTime - lastDebug >= 100) {
    printDebug();
    lastDebug = currentTime;
  }
  
  // 5. MAINTAIN 250Hz LOOP RATE
  while (micros() - loopTimer < 4000); // Wait for 4ms total
  loopTimer = micros();
}

// ═══════════════════════════════════════════════════════════════════════════
// INITIALIZATION
// ═══════════════════════════════════════════════════════════════════════════

void initRadio() {
  if (!radio.begin()) {
    Serial.println(F("❌ Radio initialization FAILED!"));
    Serial.println(F("   Check: VCC=3.3V, 10µF capacitor, wiring"));
    Serial.println(F("   CE=D9, CSN=D10, MOSI=D11, MISO=D12, SCK=D13"));
    while (1) { delay(1000); }
  }
  
  // OPTIMIZED SETTINGS FOR DRONE CONTROL
  radio.openWritingPipe(radioAddress);
  radio.openReadingPipe(1, radioAddress);  // For ACK payloads
  
  // Power: MAX for best range
  radio.setPALevel(RF24_PA_MAX);
  
  // Data rate: 250kbps = longest range, most reliable
  radio.setDataRate(RF24_250KBPS);
  
  // Channel: 108 (same as FC)
  radio.setChannel(108);
  
  // Auto-ACK: ENABLED (wait for acknowledgment)
  radio.setAutoAck(true);
  
  // Retry settings: Aggressive retries for critical control data
  radio.setRetries(5, 15);  // 5*250µs delay, 15 retries
  
  // Payload size: Fixed for speed
  radio.setPayloadSize(sizeof(RadioPacket));
  
  // CRC: 2 bytes for reliability
  radio.setCRCLength(RF24_CRC_16);
  
  // Dynamic payloads: DISABLED for speed
  radio.disableDynamicPayloads();
  
  // ACK payloads: ENABLED (receive telemetry from FC)
  radio.enableAckPayload();
  
  // TX mode
  radio.stopListening();
  
  Serial.println(F("✅ Radio initialized (2.4GHz, 250kbps, ACK ON)"));
  Serial.print(F("   Writing to address: 0x"));
  Serial.println((unsigned long)radioAddress, HEX);
  Serial.println(F("   Waiting for ACK from Flight Controller..."));
}

void calibrateJoysticks() {
  Serial.println(F("⏳ Calibrating joysticks (center sticks)..."));
  
  delay(1000); // Wait for user to center sticks
  
  // Read center positions (average 50 samples)
  long sumThrottle = 0, sumYaw = 0, sumPitch = 0, sumRoll = 0;
  const int samples = 50;
  
  for (int i = 0; i < samples; i++) {
    sumThrottle += analogRead(THROTTLE_PIN);
    sumYaw += analogRead(YAW_PIN);
    sumPitch += analogRead(PITCH_PIN);
    sumRoll += analogRead(ROLL_PIN);
    delay(10);
  }
  
  throttleCenter = sumThrottle / samples;
  yawCenter = sumYaw / samples;
  pitchCenter = sumPitch / samples;
  rollCenter = sumRoll / samples;
  
  Serial.print(F("   Center points: Throttle="));
  Serial.print(throttleCenter);
  Serial.print(F(" Yaw="));
  Serial.print(yawCenter);
  Serial.print(F(" Pitch="));
  Serial.print(pitchCenter);
  Serial.print(F(" Roll="));
  Serial.println(rollCenter);
  Serial.println(F("✅ Joystick calibration complete"));
}

// ═══════════════════════════════════════════════════════════════════════════
// INPUT READING
// ═══════════════════════════════════════════════════════════════════════════

void readJoysticks() {
  // Read raw analog values (0-1023)
  int throttleRaw = analogRead(THROTTLE_PIN);
  int yawRaw = analogRead(YAW_PIN);
  int pitchRaw = analogRead(PITCH_PIN);
  int rollRaw = analogRead(ROLL_PIN);
  
  // THROTTLE: 0-1000 (no centering, full range)
  txData.throttle = map(throttleRaw, 0, 1023, 0, 1000);
  txData.throttle = constrain(txData.throttle, 0, 1000);
  
  // YAW: -500 to +500 (centered)
  int yawOffset = yawRaw - yawCenter;
  if (abs(yawOffset) < DEADZONE) yawOffset = 0;
  txData.yaw = map(yawOffset, -512, 512, -500, 500);
  txData.yaw = constrain(txData.yaw, -500, 500);
  
  // PITCH: -500 to +500 (centered, inverted for natural control)
  int pitchOffset = pitchRaw - pitchCenter;
  if (abs(pitchOffset) < DEADZONE) pitchOffset = 0;
  txData.pitch = map(pitchOffset, -512, 512, 500, -500); // Inverted
  txData.pitch = constrain(txData.pitch, -500, 500);
  
  // ROLL: -500 to +500 (centered)
  int rollOffset = rollRaw - rollCenter;
  if (abs(rollOffset) < DEADZONE) rollOffset = 0;
  txData.roll = map(rollOffset, -512, 512, -500, 500);
  txData.roll = constrain(txData.roll, -500, 500);
}

void readButtons() {
  // Buttons are active LOW with pullup
  txData.btn1 = digitalRead(BTN1_PIN);
  txData.btn2 = digitalRead(BTN2_PIN);
  txData.btn3 = digitalRead(BTN3_PIN);
  txData.btn4 = digitalRead(BTN4_PIN);
  
  // Switches are active LOW
  txData.sw1 = digitalRead(SW1_PIN);
  txData.sw2 = digitalRead(SW2_PIN);
}

// ═══════════════════════════════════════════════════════════════════════════
// RADIO TRANSMISSION
// ═══════════════════════════════════════════════════════════════════════════

void transmitData() {
  // Send packet and wait for ACK
  bool success = radio.write(&txData, sizeof(RadioPacket));
  
  if (success) {
    radioOK = true;
    lastSuccessfulSend = currentTime;
    
    // Optional: Read telemetry from ACK payload
    if (radio.isAckPayloadAvailable()) {
      // Uncomment if FC is sending telemetry
      /*
      struct TelemetryPacket {
        float batteryVoltage;
        float altitude;
        uint8_t armed;
      } telemetry;
      
      radio.read(&telemetry, sizeof(TelemetryPacket));
      
      // Display telemetry
      Serial.print(F(" | Batt:"));
      Serial.print(telemetry.batteryVoltage, 1);
      Serial.print(F("V | Alt:"));
      Serial.print(telemetry.altitude, 0);
      Serial.print(F("cm"));
      */
    }
  } else {
    // Transmission failed
    // Check if connection lost for >500ms
    if (currentTime - lastSuccessfulSend > 500) {
      radioOK = false;
      
      // Warning every 2 seconds
      static unsigned long lastWarning = 0;
      if (currentTime - lastWarning > 2000) {
        Serial.println();
        Serial.println(F("⚠️  WARNING: No ACK from Flight Controller!"));
        Serial.println(F("   1. Check FC is powered on"));
        Serial.println(F("   2. Check distance (move closer)"));
        Serial.println(F("   3. Check nRF24 antennas are parallel"));
        Serial.println(F("   4. Check 10µF capacitor on both modules"));
        lastWarning = currentTime;
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// DEBUG & STATUS
// ═══════════════════════════════════════════════════════════════════════════

void printDebug() {
  Serial.print(F("TX: "));
  Serial.print(radioOK ? F("✅ OK") : F("❌ FAIL"));
  
  Serial.print(F(" | Throttle:"));
  Serial.print(txData.throttle);
  
  Serial.print(F(" | Yaw:"));
  Serial.print(txData.yaw);
  
  Serial.print(F(" | Pitch:"));
  Serial.print(txData.pitch);
  
  Serial.print(F(" | Roll:"));
  Serial.print(txData.roll);
  
  Serial.print(F(" | SW1:"));
  Serial.print(txData.sw1 == LOW ? F("ALT_HOLD") : F("OFF"));
  
  Serial.print(F(" | SW2:"));
  Serial.print(txData.sw2 == LOW ? F("ACRO") : F("ANGLE"));
  
  // Show button presses
  if (txData.btn1 == LOW) Serial.print(F(" [CALIB]"));
  if (txData.btn2 == LOW) Serial.print(F(" [TEST]"));
  if (txData.btn3 == LOW) Serial.print(F(" [LAND]"));
  if (txData.btn4 == LOW) Serial.print(F(" [TAKEOFF]"));
  
  Serial.println();
}

void printControlMapping() {
  Serial.println(F("\n╔════════════════════════════════════════════════════╗"));
  Serial.println(F("║                  CONTROL MAPPING                   ║"));
  Serial.println(F("╠════════════════════════════════════════════════════╣"));
  Serial.println(F("║ LEFT JOYSTICK:                                     ║"));
  Serial.println(F("║   • Y-axis (A0) → THROTTLE (up/down)               ║"));
  Serial.println(F("║   • X-axis (A1) → YAW (rotate left/right)          ║"));
  Serial.println(F("║                                                    ║"));
  Serial.println(F("║ RIGHT JOYSTICK:                                    ║"));
  Serial.println(F("║   • Y-axis (A2) → PITCH (forward/back)             ║"));
  Serial.println(F("║   • X-axis (A3) → ROLL (strafe left/right)         ║"));
  Serial.println(F("║                                                    ║"));
  Serial.println(F("║ BUTTONS:                                           ║"));
  Serial.println(F("║   • Button 1 (D4) → Calibrate Sensors              ║"));
  Serial.println(F("║   • Button 2 (D5) → Motor Test                     ║"));
  Serial.println(F("║   • Button 3 (D6) → Smooth Landing                 ║"));
  Serial.println(F("║   • Button 4 (D7) → Smooth Takeoff                 ║"));
  Serial.println(F("║                                                    ║"));
  Serial.println(F("║ SWITCHES:                                          ║"));
  Serial.println(F("║   • SW1 (D2) → Altitude Hold ON/OFF                ║"));
  Serial.println(F("║   • SW2 (D3) → ANGLE mode / ACRO mode              ║"));
  Serial.println(F("╚════════════════════════════════════════════════════╝\n"));
}
