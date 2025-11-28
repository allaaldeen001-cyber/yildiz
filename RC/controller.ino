#include  <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

Smoothed <float> degerexpo;
Smoothed <float> degerxrexpo;
Smoothed <float> degerxlexpo;
Smoothed <float> degeryrexpo;

RF24 radio(9, 10);

const uint64_t pipe = 0xF0F0F0F0E1LL;

// Joystick calibration values
float scaleX =  0.1;
float calX =    -527;
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

// Joystick pin mappings according to requirements
const int THROTTLE_PIN = 0;  // A0 - Throttle (L: Up/Down)
const int YAW_PIN = 1;       // A1 - Yaw (L: Left/Right)
const int PITCH_PIN = 2;     // A2 - Pitch (R: Up/Down)
const int ROLL_PIN = 3;      // A3 - Roll (R: Left/Right)

int ID = 0;
float xr, yr;  // Roll and Pitch (right joystick)
float xl, yl;  // Yaw and Throttle (left joystick)
float smoothyl, smoothxr, smoothxl, smoothyr;
bool but1, but2, switch1, switch2;

void readJoyStick();
void printPackage();

struct Package
{
  int   thrust = 0;
  float   x = 0;
  float   y = 0;
  float   z = 0;
  int  id = 0;
  bool but1 = 1;
  bool but2 = 1;
  bool switch1 = 1;
  bool switch2 = 1;
};

Package package;

void setup() {
  // Note: Physical buttons/switches on RC are not used in this implementation
  // They would be read here if needed for future expansion
  
  Serial.begin(57600);
  
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

  // Map joystick values to package
  // x = Roll (right joystick left/right) -> A3
  // y = Pitch (right joystick up/down) -> A2 (inverted: 1023 - value)
  // z = Yaw (left joystick left/right) -> A1
  // thrust = Throttle (left joystick up/down) -> A0 (inverted: 1023 - value)
  
  package.x =       (xr + calX) * scaleX + offsetX;        // Roll
  package.y =       (yr + calY) * scaleY + offsetY;        // Pitch (inverted in readJoyStick)
  package.z =       (xl + calZ) * scaleZ + offsetZ;        // Yaw
  package.thrust =  (yl + calThrust) * scaleThrust + offsetThrust;  // Throttle (inverted in readJoyStick)
  
  package.id = ID;
  ID++;
  
  // Default button/switch states (can be modified if physical buttons added)
  package.but1 = but1;
  package.but2 = but2;
  package.switch1 = switch1;
  package.switch2 = switch2;
  
  radio.write(&package, sizeof(package));
  printPackage();
  
  delay(7);  // ~140Hz update rate
}

void readJoyStick()
{
  // Read analog joystick values
  // Note: For pull-up joysticks, values may need inversion
  
  // Throttle: A0 - Left joystick Up/Down (inverted: UP increases thrust)
  smoothyl = 1023 - analogRead(THROTTLE_PIN);
  smoothyl = max(smoothyl, 0);
  
  // Yaw: A1 - Left joystick Left/Right
  smoothxl = analogRead(YAW_PIN);
  
  // Pitch: A2 - Right joystick Up/Down (inverted: UP tilts forward)
  smoothyr = 1023 - analogRead(PITCH_PIN);
  
  // Roll: A3 - Right joystick Left/Right
  smoothxr = analogRead(ROLL_PIN);
  
  // Apply exponential smoothing
  degerexpo.add(smoothyl);
  degerxrexpo.add(smoothxr);
  degerxlexpo.add(smoothxl);
  degeryrexpo.add(smoothyr);

  yl = degerexpo.get();   // Throttle
  xr = degerxrexpo.get(); // Roll
  xl = degerxlexpo.get(); // Yaw
  yr = degeryrexpo.get(); // Pitch
  
  // Default button/switch states (set to 1 = inactive)
  but1 = 1;
  but2 = 1;
  switch1 = 1;
  switch2 = 1;
}

void printPackage()
{
  Serial.print("Thrust: ");
  Serial.print(package.thrust);
  Serial.print("\tYaw: ");
  Serial.print(package.z);
  Serial.print("\tRoll: ");
  Serial.print(package.x);
  Serial.print("\tPitch: ");
  Serial.println(package.y);
}
