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

// Joystick pin definitions according to requirements
const int THROTTLE_PIN = 0;  // A0 - Throttle (L: Up/Down)
const int YAW_PIN = 1;       // A1 - Yaw (L: Left/Right)
const int PITCH_PIN = 2;     // A2 - Pitch (R: Up/Down)
const int ROLL_PIN = 3;       // A3 - Roll (R: Left/Right)

int ID = 0;
float xr, yr;      // Roll (xr) and Pitch (yr)
float xl, yl;      // Yaw (xl) and Throttle (yl)
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
  // Button and switch pins (if needed on RC)
  pinMode(2, INPUT_PULLUP); //Switch 2
  pinMode(3, INPUT_PULLUP); //Switch 1
  pinMode(4, INPUT_PULLUP); //Buton 1
  pinMode(5, INPUT_PULLUP); //Buton 2

  Serial.begin(57600);
  
  // Radio setup with ACK enabled
  radio.begin();
  radio.setAutoAck(true);  // Enable ACK for communication confirmation
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  // Initialize smoothing filters
  degerexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degerxrexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degerxlexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degeryrexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  
  Serial.println("RC Controller Initialized");
}

void loop() {
  readJoyStick();

  // Map joystick values to package
  // Roll (xr) -> package.x
  // Pitch (yr) -> package.y  
  // Yaw (xl) -> package.z
  // Throttle (yl) -> package.thrust
  
  package.x =       (xr + calX) * scaleX + offsetX;
  package.y =       (yr + calY) * scaleY + offsetY;
  package.z =       (xl + calZ) * scaleZ + offsetZ;
  package.thrust =  (yl + calThrust) * scaleThrust + offsetThrust;
  
  package.id = ID;
  ID++;
  
  package.but1 = but1;
  package.but2 = but2;
  package.switch1 = switch1;
  package.switch2 = switch2;
  
  // Send with ACK confirmation
  bool success = radio.write(&package, sizeof(package));
  
  if (success) {
    // Packet sent successfully with ACK
  } else {
    // Retry sending
    delay(1);
    radio.write(&package, sizeof(package));
  }
  
  printPackage();
  delay(7);  // ~140Hz update rate
}

void readJoyStick()
{
  but1 = digitalRead(4);
  but2 = digitalRead(5);
  switch1 = digitalRead(3);
  switch2 = digitalRead(2);

  // Read joystick values
  // Throttle: A0 (UP increases, DOWN decreases) - inverted: 1023 - value
  smoothyl = max(analogRead(THROTTLE_PIN), 0);
  
  // Yaw: A1 (LEFT/RIGHT)
  smoothxl = analogRead(YAW_PIN);
  
  // Pitch: A2 (UP/DOWN) - inverted: 1023 - value
  smoothyr = 1023 - analogRead(PITCH_PIN);
  
  // Roll: A3 (LEFT/RIGHT)
  smoothxr = analogRead(ROLL_PIN);

  // Apply exponential smoothing
  degerexpo.add(smoothyl);
  degerxrexpo.add(smoothxr);
  degerxlexpo.add(smoothxl);
  degeryrexpo.add(smoothyr);

  xr = degerxrexpo.get();  // Roll
  xl = degerxlexpo.get();  // Yaw
  yr = degeryrexpo.get();  // Pitch
  yl = degerexpo.get();    // Throttle
}

void printPackage()
{
  Serial.print("Thrust: ");
  Serial.print(package.thrust);
  Serial.print("\tRoll: ");
  Serial.print(package.x);
  Serial.print("\tPitch: ");
  Serial.print(package.y);
  Serial.print("\tYaw: ");
  Serial.print(package.z);
  Serial.print("\tID: ");
  Serial.println(package.id);
}
