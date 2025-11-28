/*
 * Professional Remote Controller Firmware
 * Arduino Nano + NRF24L01
 * 
 * Features:
 * - Joystick input reading
 * - Button and switch handling
 * - NRF communication with ACK
 * - Serial monitor output
 * - Calibration and motor control commands
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "DroneProtocol.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
#define PIN_NRF_CE     9
#define PIN_NRF_CSN    10

// Joysticks (Analog inputs)
#define PIN_THROTTLE   A0  // Left joystick vertical
#define PIN_YAW        A1  // Left joystick horizontal
#define PIN_PITCH      A2  // Right joystick vertical
#define PIN_ROLL       A3  // Right joystick horizontal

// Buttons
#define PIN_BUTTON1    4   // Calibration button
#define PIN_BUTTON2    5   // Motor ON + ESC calibration

// Switches
#define PIN_SW1        2   // Altitude Hold ON/OFF
#define PIN_SW2        3   // ARM / DISARM (Kill Switch)

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================
RF24 radio(PIN_NRF_CE, PIN_NRF_CSN);

// ============================================================================
// DATA STRUCTURES
// ============================================================================
RC_Command rc_cmd;
FC_Telemetry fc_telemetry;

// Joystick calibration values
struct JoystickCal {
  int16_t min_val;
  int16_t max_val;
  int16_t center_val;
} throttle_cal, yaw_cal, pitch_cal, roll_cal;

// System state
bool link_active = false;
unsigned long last_link_time = 0;
unsigned long last_telemetry_time = 0;

// Button debouncing
unsigned long last_button1_time = 0;
unsigned long last_button2_time = 0;
uint8_t last_button1_state = 0;
uint8_t last_button2_state = 0;

// ============================================================================
// JOYSTICK READING FUNCTIONS
// ============================================================================
int16_t readJoystick(uint8_t pin, JoystickCal* cal) {
  int16_t raw = analogRead(pin);
  
  // Map to 1000-2000 range
  int16_t value;
  if (raw < cal->center_val) {
    // Below center
    value = map(raw, cal->min_val, cal->center_val, 1000, 1500);
  } else {
    // Above center
    value = map(raw, cal->center_val, cal->max_val, 1500, 2000);
  }
  
  return constrain(value, 1000, 2000);
}

void calibrateJoysticks() {
  Serial.println("Calibrating joysticks...");
  Serial.println("Move all joysticks to extremes, then center");
  delay(2000);
  
  // Read min/max values
  int16_t throttle_min = 1023, throttle_max = 0;
  int16_t yaw_min = 1023, yaw_max = 0;
  int16_t pitch_min = 1023, pitch_max = 0;
  int16_t roll_min = 1023, roll_max = 0;
  
  unsigned long start_time = millis();
  while (millis() - start_time < 5000) {
    int16_t t = analogRead(PIN_THROTTLE);
    int16_t y = analogRead(PIN_YAW);
    int16_t p = analogRead(PIN_PITCH);
    int16_t r = analogRead(PIN_ROLL);
    
    if (t < throttle_min) throttle_min = t;
    if (t > throttle_max) throttle_max = t;
    if (y < yaw_min) yaw_min = y;
    if (y > yaw_max) yaw_max = y;
    if (p < pitch_min) pitch_min = p;
    if (p > pitch_max) pitch_max = p;
    if (r < roll_min) roll_min = r;
    if (r > roll_max) roll_max = r;
    
    delay(10);
  }
  
  // Set calibration values
  throttle_cal.min_val = throttle_min;
  throttle_cal.max_val = throttle_max;
  throttle_cal.center_val = (throttle_min + throttle_max) / 2;
  
  yaw_cal.min_val = yaw_min;
  yaw_cal.max_val = yaw_max;
  yaw_cal.center_val = (yaw_min + yaw_max) / 2;
  
  pitch_cal.min_val = pitch_min;
  pitch_cal.max_val = pitch_max;
  pitch_cal.center_val = (pitch_min + pitch_max) / 2;
  
  roll_cal.min_val = roll_min;
  roll_cal.max_val = roll_max;
  roll_cal.center_val = (roll_min + roll_max) / 2;
  
  Serial.println("Calibration complete!");
  Serial.print("Throttle: "); Serial.print(throttle_cal.min_val); 
  Serial.print("-"); Serial.print(throttle_cal.center_val);
  Serial.print("-"); Serial.println(throttle_cal.max_val);
}

// ============================================================================
// BUTTON DEBOUNCING
// ============================================================================
uint8_t readButton(uint8_t pin, unsigned long* last_time, uint8_t* last_state) {
  uint8_t current_state = digitalRead(pin);
  unsigned long current_time = millis();
  
  // Debounce: require stable state for 50ms
  if (current_state != *last_state) {
    *last_time = current_time;
    *last_state = current_state;
    return 0; // State changed, wait for stability
  }
  
  if (current_time - *last_time > 50) {
    return current_state; // Stable state
  }
  
  return 0;
}

// ============================================================================
// SWITCH READING
// ============================================================================
uint8_t readSwitch(uint8_t pin) {
  // Switches: pin1 = signal, pin2/3 to GND
  // When switch is ON, pin reads HIGH (1)
  // When switch is OFF, pin reads LOW (0)
  return digitalRead(pin);
}

// ============================================================================
// NRF COMMUNICATION
// ============================================================================
void initNRF() {
  radio.begin();
  radio.setChannel(NRF_CHANNEL);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.openWritingPipe(0xF0F0F0F0E1LL);  // RC sends to FC
  radio.openReadingPipe(1, 0xF0F0F0F0D2LL); // RC receives from FC
  radio.stopListening();
}

bool sendCommand() {
  // Prepare command
  rc_cmd.throttle = readJoystick(PIN_THROTTLE, &throttle_cal);
  rc_cmd.yaw = readJoystick(PIN_YAW, &yaw_cal);
  rc_cmd.pitch = readJoystick(PIN_PITCH, &pitch_cal);
  rc_cmd.roll = readJoystick(PIN_ROLL, &roll_cal);
  
  // Read buttons with debouncing
  rc_cmd.button1 = readButton(PIN_BUTTON1, &last_button1_time, &last_button1_state);
  rc_cmd.button2 = readButton(PIN_BUTTON2, &last_button2_time, &last_button2_state);
  
  // Read switches
  rc_cmd.sw1 = readSwitch(PIN_SW1);
  rc_cmd.sw2 = readSwitch(PIN_SW2);
  
  // Calculate checksum
  rc_cmd.checksum = calculateRC_Checksum(&rc_cmd);
  
  // Send command
  bool result = radio.write(&rc_cmd, sizeof(RC_Command));
  
  if (result) {
    last_link_time = millis();
    link_active = true;
  } else {
    // Check for link loss
    if (millis() - last_link_time > 500) {
      link_active = false;
    }
  }
  
  return result;
}

bool receiveTelemetry() {
  radio.startListening();
  
  if (radio.available()) {
    radio.read(&fc_telemetry, sizeof(FC_Telemetry));
    
    // Validate checksum
    if (validateChecksum((uint8_t*)&fc_telemetry, sizeof(FC_Telemetry))) {
      last_telemetry_time = millis();
      radio.stopListening();
      return true;
    }
  }
  
  radio.stopListening();
  return false;
}

// ============================================================================
// SERIAL MONITOR OUTPUT
// ============================================================================
void printStatus() {
  static unsigned long last_print = 0;
  
  if (millis() - last_print < 200) return; // Update every 200ms
  last_print = millis();
  
  Serial.println("\n=== RC STATUS ===");
  Serial.print("Link: ");
  Serial.println(link_active ? "CONNECTED" : "LOST");
  
  Serial.print("Joysticks - T:");
  Serial.print(rc_cmd.throttle);
  Serial.print(" Y:");
  Serial.print(rc_cmd.yaw);
  Serial.print(" P:");
  Serial.print(rc_cmd.pitch);
  Serial.print(" R:");
  Serial.println(rc_cmd.roll);
  
  Serial.print("Buttons - B1:");
  Serial.print(rc_cmd.button1);
  Serial.print(" B2:");
  Serial.println(rc_cmd.button2);
  
  Serial.print("Switches - SW1(AltHold):");
  Serial.print(rc_cmd.sw1);
  Serial.print(" SW2(ARM):");
  Serial.println(rc_cmd.sw2);
  
  if (link_active && fc_telemetry.link_status) {
    Serial.println("\n=== FC TELEMETRY ===");
    Serial.print("Roll: ");
    Serial.print(fc_telemetry.roll);
    Serial.print("° Pitch: ");
    Serial.print(fc_telemetry.pitch);
    Serial.print("° Yaw Rate: ");
    Serial.print(fc_telemetry.yaw_rate);
    Serial.println("°/s");
    
    Serial.print("Altitude: ");
    Serial.print(fc_telemetry.altitude);
    Serial.print("m Vertical Speed: ");
    Serial.print(fc_telemetry.vertical_speed);
    Serial.println("m/s");
    
    Serial.print("Armed: ");
    Serial.print(fc_telemetry.armed ? "YES" : "NO");
    Serial.print(" AltHold: ");
    Serial.print(fc_telemetry.altitude_hold ? "ON" : "OFF");
    Serial.print(" Calibrated: ");
    Serial.println(fc_telemetry.calibration_status ? "YES" : "NO");
  }
  
  Serial.println("==================\n");
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Remote Controller Initializing...");
  
  // Initialize pins
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);
  pinMode(PIN_SW1, INPUT_PULLUP);
  pinMode(PIN_SW2, INPUT_PULLUP);
  
  // Initialize analog pins (no need for pinMode on analog pins)
  
  // Initialize NRF
  initNRF();
  
  // Initialize joystick calibration (default values)
  // These will be calibrated on first run
  throttle_cal.min_val = 0;
  throttle_cal.max_val = 1023;
  throttle_cal.center_val = 512;
  
  yaw_cal.min_val = 0;
  yaw_cal.max_val = 1023;
  yaw_cal.center_val = 512;
  
  pitch_cal.min_val = 0;
  pitch_cal.max_val = 1023;
  pitch_cal.center_val = 512;
  
  roll_cal.min_val = 0;
  roll_cal.max_val = 1023;
  roll_cal.center_val = 512;
  
  // Auto-calibrate joysticks on startup
  calibrateJoysticks();
  
  Serial.println("Remote Controller Ready!");
  Serial.println("Controls:");
  Serial.println("  Left Stick: Throttle (V), Yaw (H)");
  Serial.println("  Right Stick: Pitch (V), Roll (H)");
  Serial.println("  Button 1: Calibration");
  Serial.println("  Button 2: Motor ON + ESC Cal");
  Serial.println("  SW1: Altitude Hold");
  Serial.println("  SW2: ARM/DISARM");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Send command to FC (50 Hz)
  static unsigned long send_timer = 0;
  if (millis() - send_timer >= 20) {
    send_timer = millis();
    sendCommand();
  }
  
  // Receive telemetry from FC (50 Hz)
  static unsigned long recv_timer = 0;
  if (millis() - recv_timer >= 20) {
    recv_timer = millis();
    receiveTelemetry();
  }
  
  // Print status to serial monitor
  printStatus();
  
  // Small delay to prevent watchdog issues
  delay(1);
}
