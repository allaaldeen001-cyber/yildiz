/*
 * QUADCOPTER REMOTE CONTROLLER
 * Target: Arduino Nano
 * Radio: NRF24L01 (Channel 103)
 * 
 * Controls:
 * - A0: Throttle (Left Vertical)
 * - A1: Yaw (Left Horizontal)
 * - A2: Pitch (Right Vertical)
 * - A3: Roll (Right Horizontal)
 * - D2: Switch 2 (Altitude Hold)
 * - D3: Switch 1 (Arm/Disarm)
 * - D4: Button 1 (Calibration)
 * - D5: Button 2 (Arming)
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define NRF_CE_PIN  9
#define NRF_CSN_PIN 10

#define JOYSTICK_THROTTLE_PIN  A0
#define JOYSTICK_YAW_PIN       A1
#define JOYSTICK_PITCH_PIN     A2
#define JOYSTICK_ROLL_PIN      A3

#define BUTTON1_PIN   4
#define BUTTON2_PIN   5
#define SWITCH1_PIN   3
#define SWITCH2_PIN   2

// ============================================================================
// DATA STRUCTURES
// ============================================================================
struct Package {
  int   thrust;
  float x;
  float y;
  float z;
  int   id;
  bool  but1;
  bool  but2;
  bool  switch1;
  bool  switch2;
};

Package package;

// ============================================================================
// GLOBALS
// ============================================================================
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
const uint64_t pipe = 0xF0F0F0F0E1LL;

// Joystick Calibration
struct JoystickCal {
  int center;
  int deadzone;
};

JoystickCal calThrottle = {512, 10};
JoystickCal calYaw      = {512, 20};
JoystickCal calPitch    = {512, 20};
JoystickCal calRoll     = {512, 20};

// Timing
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 20;

// Low-pass filter
float filterAlpha = 0.3;
float filteredThrottle = 512;
float filteredYaw = 512;
float filteredPitch = 512;
float filteredRoll = 512;

// Button debouncing
struct ButtonState {
  uint8_t pin;
  bool current;
  bool last;
  unsigned long lastDebounce;
  unsigned long debounceDelay;
};

ButtonState btn1 = {BUTTON1_PIN, HIGH, HIGH, 0, 50};
ButtonState btn2 = {BUTTON2_PIN, HIGH, HIGH, 0, 50};

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================
void calibrateJoysticks();
float applyFilter(float filtered, int raw, float alpha);
int mapJoystick(int raw, JoystickCal &cal, bool isThrottle);
bool readButton(ButtonState &btn);
bool readSwitch(int pin);
void updatePackage();
void displayStatus();

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(57600);
  
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);
  
  Serial.println("Remote Controller Initializing...");
  
  if (!radio.begin()) {
    Serial.println("ERROR: NRF24L01 failed to initialize");
    Serial.println("Check wiring:");
    Serial.println("  CE  -> D9");
    Serial.println("  CSN -> D10");
    Serial.println("  MOSI -> D11");
    Serial.println("  MISO -> D12");
    Serial.println("  SCK -> D13");
    while (1);
  }
  
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(103);
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  Serial.println("NRF24L01 initialized");
  Serial.println("Channel: 103");
  Serial.println("Data Rate: 250KBPS");
  
  calibrateJoysticks();
  
  package.thrust = 0;
  package.x = 0;
  package.y = 0;
  package.z = 0;
  package.id = 0;
  package.but1 = 1;
  package.but2 = 1;
  package.switch1 = 1;
  package.switch2 = 1;
  
  Serial.println("Remote Controller Ready");
  Serial.println("=========================================");
  
  lastSendTime = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long currentTime = millis();
  
  updatePackage();
  
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentTime;
    
    bool success = radio.write(&package, sizeof(Package));
    
    if (success) {
      Serial.print("TX OK | T:");
      Serial.print(package.thrust);
      Serial.print(" X:");
      Serial.print(package.x, 1);
      Serial.print(" Y:");
      Serial.print(package.y, 1);
      Serial.print(" Z:");
      Serial.print(package.z, 1);
      Serial.print(" | SW1:");
      Serial.print(package.switch1 ? "OFF" : "ON");
      Serial.print(" SW2:");
      Serial.print(package.switch2 ? "OFF" : "ON");
      Serial.print(" | B1:");
      Serial.print(package.but1 ? "UP" : "DN");
      Serial.print(" B2:");
      Serial.println(package.but2 ? "UP" : "DN");
    } else {
      Serial.println("TX FAILED");
    }
  }
}

// ============================================================================
// CALIBRATION
// ============================================================================
void calibrateJoysticks() {
  Serial.println("Calibrating joysticks...");
  Serial.println("Ensure sticks are centered");
  
  delay(1000);
  
  int samples = 50;
  long sumYaw = 0, sumPitch = 0, sumRoll = 0;
  
  for (int i = 0; i < samples; i++) {
    sumYaw += analogRead(JOYSTICK_YAW_PIN);
    sumPitch += analogRead(JOYSTICK_PITCH_PIN);
    sumRoll += analogRead(JOYSTICK_ROLL_PIN);
    delay(10);
  }
  
  calYaw.center = sumYaw / samples;
  calPitch.center = sumPitch / samples;
  calRoll.center = sumRoll / samples;
  
  filteredYaw = calYaw.center;
  filteredPitch = calPitch.center;
  filteredRoll = calRoll.center;
  filteredThrottle = analogRead(JOYSTICK_THROTTLE_PIN);
  
  Serial.print("  Yaw center: ");
  Serial.println(calYaw.center);
  Serial.print("  Pitch center: ");
  Serial.println(calPitch.center);
  Serial.print("  Roll center: ");
  Serial.println(calRoll.center);
  Serial.println("Calibration complete");
}

// ============================================================================
// FILTER
// ============================================================================
float applyFilter(float filtered, int raw, float alpha) {
  return alpha * raw + (1.0 - alpha) * filtered;
}

// ============================================================================
// MAP JOYSTICK
// ============================================================================
int mapJoystick(int raw, JoystickCal &cal, bool isThrottle) {
  if (isThrottle) {
    return map(raw, 0, 1023, 0, 1000);
  } else {
    if (abs(raw - cal.center) < cal.deadzone) {
      return 0;
    }
    
    if (raw < cal.center) {
      return map(raw, 0, cal.center, -100, 0);
    } else {
      return map(raw, cal.center, 1023, 0, 100);
    }
  }
}

// ============================================================================
// READ BUTTON
// ============================================================================
bool readButton(ButtonState &btn) {
  int reading = digitalRead(btn.pin);
  
  if (reading != btn.last) {
    btn.lastDebounce = millis();
  }
  
  if ((millis() - btn.lastDebounce) > btn.debounceDelay) {
    if (reading != btn.current) {
      btn.current = reading;
    }
  }
  
  btn.last = reading;
  return (btn.current == LOW);
}

// ============================================================================
// READ SWITCH
// ============================================================================
bool readSwitch(int pin) {
  return (digitalRead(pin) == LOW);
}

// ============================================================================
// UPDATE PACKAGE
// ============================================================================
void updatePackage() {
  int rawThrottle = analogRead(JOYSTICK_THROTTLE_PIN);
  int rawYaw = analogRead(JOYSTICK_YAW_PIN);
  int rawPitch = analogRead(JOYSTICK_PITCH_PIN);
  int rawRoll = analogRead(JOYSTICK_ROLL_PIN);
  
  filteredThrottle = applyFilter(filteredThrottle, rawThrottle, filterAlpha);
  filteredYaw = applyFilter(filteredYaw, rawYaw, filterAlpha);
  filteredPitch = applyFilter(filteredPitch, rawPitch, filterAlpha);
  filteredRoll = applyFilter(filteredRoll, rawRoll, filterAlpha);
  
  package.thrust = mapJoystick((int)filteredThrottle, calThrottle, true);
  package.z = mapJoystick((int)filteredYaw, calYaw, false);
  package.y = mapJoystick((int)filteredPitch, calPitch, false);
  package.x = mapJoystick((int)filteredRoll, calRoll, false);
  
  package.switch1 = readSwitch(SWITCH1_PIN) ? 0 : 1;
  package.switch2 = readSwitch(SWITCH2_PIN) ? 0 : 1;
  
  package.but1 = readButton(btn1) ? 0 : 1;
  package.but2 = readButton(btn2) ? 0 : 1;
  
  package.id = 1;
}
