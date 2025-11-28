/*
 * Remote Controller (RC) - Arduino Nano
 * 
 * Hardware:
 * - NRF24L01: CE=D9, CSN=D10
 * - Joysticks:
 *   A0 = Throttle (Left: Up/Down)
 *   A1 = Yaw (Left: Left/Right)
 *   A2 = Pitch (Right: Up/Down)
 *   A3 = Roll (Right: Left/Right)
 * - Buttons/Switches: Not used on RC (handled on FC)
 */

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

Smoothed <float> degerexpo;
Smoothed <float> degerxrexpo;
Smoothed <float> degerxlexpo;
Smoothed <float> degeryrexpo;

RF24 radio(9, 10);
const uint64_t pipe = 0xF0F0F0F0E1LL;

// Calibration values (adjust these for your joysticks)
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

// Joystick pins
const int THROTTLE_PIN = 0;  // A0 - Left joystick Up/Down
const int YAW_PIN = 1;       // A1 - Left joystick Left/Right
const int PITCH_PIN = 2;     // A2 - Right joystick Up/Down
const int ROLL_PIN = 3;      // A3 - Right joystick Left/Right

int ID = 0;
float xr, yr;  // Right joystick (Roll, Pitch)
float xl, yl;  // Left joystick (Yaw, Throttle)
float smoothyl, smoothxr, smoothxl, smoothyr;

struct Package {
  int   thrust = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  int   id = 0;
  bool  but1 = 1;
  bool  but2 = 1;
  bool  switch1 = 1;
  bool  switch2 = 1;
};

Package package;

void setup() {
  Serial.begin(57600);
  
  // Initialize radio
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  // Initialize smoothing filters
  degerexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degerxrexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degerxlexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degeryrexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  
  Serial.println("RC Controller initialized");
}

void loop() {
  readJoyStick();
  
  // Map joystick values to control signals
  // Right joystick: Roll (x) and Pitch (y)
  package.x = (xr + calX) * scaleX + offsetX;
  package.y = (yr + calY) * scaleY + offsetY;
  
  // Left joystick: Yaw (z) and Throttle
  package.z = (xl + calZ) * scaleZ + offsetZ;
  package.thrust = (yl + calThrust) * scaleThrust + offsetThrust;
  
  // Limit thrust range
  if (package.thrust < 1000) package.thrust = 1000;
  if (package.thrust > 2000) package.thrust = 2000;
  
  package.id = ID;
  ID++;
  
  // Buttons and switches are handled on FC board
  package.but1 = 1;
  package.but2 = 1;
  package.switch1 = 1;
  package.switch2 = 1;
  
  // Send package
  radio.write(&package, sizeof(package));
  
  printPackage();
  delay(10);  // ~100 Hz update rate
}

void readJoyStick() {
  // Read raw analog values
  // Throttle: A0 (inverted: UP = higher value)
  smoothyl = max(analogRead(THROTTLE_PIN), 0);
  
  // Yaw: A1 (Left/Right)
  smoothxl = analogRead(YAW_PIN);
  
  // Pitch: A2 (inverted: UP = higher value)
  smoothyr = 1023 - analogRead(PITCH_PIN);
  
  // Roll: A3 (Left/Right)
  smoothxr = analogRead(ROLL_PIN);
  
  // Apply exponential smoothing
  degerexpo.add(smoothyl);
  degerxlexpo.add(smoothxl);
  degeryrexpo.add(smoothyr);
  degerxrexpo.add(smoothxr);
  
  // Get smoothed values
  yl = degerexpo.get();
  xl = degerxlexpo.get();
  yr = degeryrexpo.get();
  xr = degerxrexpo.get();
}

void printPackage() {
  Serial.print("Thrust: ");
  Serial.print(package.thrust);
  Serial.print("\tRoll: ");
  Serial.print(package.x);
  Serial.print("\tPitch: ");
  Serial.print(package.y);
  Serial.print("\tYaw: ");
  Serial.println(package.z);
}
