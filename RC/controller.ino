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

//Joystick calibration values
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

//Joystick pins
const int XL_pin = 1;  //A1 - Yaw (Left joystick Left/Right)
const int YL_pin = 0;  //A0 - Throttle (Left joystick Up/Down)

const int XR_pin = 3;  //A3 - Roll (Right joystick Left/Right)
const int YR_pin = 2;  //A2 - Pitch (Right joystick Up/Down)

//Button/Switch pins
const int BUTTON1_PIN = 4;   //Button 1
const int BUTTON2_PIN = 5;   //Button 2 (Arming)
const int SWITCH1_PIN = 3;   //Switch 1 (Arm/Disarm)
const int SWITCH2_PIN = 2;   //Switch 2 (Altitude Hold)

int ID = 0;
float xr, yr;  //Right joystick (Roll, Pitch)
float xl, yl;  //Left joystick (Yaw, Throttle)
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
  //Pin configuration
  pinMode(SWITCH2_PIN, INPUT_PULLUP);  //Switch 2 (Altitude Hold)
  pinMode(SWITCH1_PIN, INPUT_PULLUP);  //Switch 1 (Arm/Disarm)
  pinMode(BUTTON1_PIN, INPUT_PULLUP); //Button 1
  pinMode(BUTTON2_PIN, INPUT_PULLUP); //Button 2 (Arming)

  Serial.begin(57600);
  
  //Initialize radio
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  Serial.println("RC Controller initialized");
  Serial.println("Waiting for connection...");
  
  //Initialize smoothing filters
  degerexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degerxrexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degerxlexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  degeryrexpo.begin(SMOOTHED_EXPONENTIAL, 1);
  
  delay(1000);
}

void loop() {
  readJoyStick();

  //Map joystick values to control signals
  //Right joystick X = Roll, Right joystick Y = Pitch
  //Left joystick X = Yaw, Left joystick Y = Throttle
  package.x = (xr + calX) * scaleX + offsetX;           //Roll
  package.y = (yr + calY) * scaleY + offsetY;           //Pitch
  package.z = (xl + calZ) * scaleZ + offsetZ;           //Yaw
  package.thrust = (yl + calThrust) * scaleThrust + offsetThrust;  //Throttle
  
  //Limit thrust range
  if (package.thrust < 1000) package.thrust = 1000;
  if (package.thrust > 2000) package.thrust = 2000;
  
  package.id = ID;
  ID++;
  if (ID > 1000) ID = 0;
  
  //Read buttons and switches (inverted because INPUT_PULLUP)
  package.but1 = !digitalRead(BUTTON1_PIN);
  package.but2 = !digitalRead(BUTTON2_PIN);
  package.switch1 = !digitalRead(SWITCH1_PIN);
  package.switch2 = !digitalRead(SWITCH2_PIN);
  
  //Send package
  radio.write(&package, sizeof(package));
  
  printPackage();
  
  delay(7);  //Approximately 140 Hz
}

void readJoyStick() {
  //Read buttons and switches
  but1 = !digitalRead(BUTTON1_PIN);
  but2 = !digitalRead(BUTTON2_PIN);
  switch1 = !digitalRead(SWITCH1_PIN);
  switch2 = !digitalRead(SWITCH2_PIN);

  //Read analog joystick values
  //Throttle: A0 (Left joystick Up/Down) - UP increases, DOWN decreases
  smoothyl = analogRead(YL_pin);  //Throttle
  
  //Yaw: A1 (Left joystick Left/Right)
  smoothxl = analogRead(XL_pin);  //Yaw
  
  //Pitch: A2 (Right joystick Up/Down) - UP tilts forward, DOWN tilts backward
  smoothyr = 1023 - analogRead(YR_pin);  //Pitch (inverted)
  
  //Roll: A3 (Right joystick Left/Right)
  smoothxr = analogRead(XR_pin);  //Roll

  //Apply smoothing filters
  degerexpo.add(smoothyl);
  degerxrexpo.add(smoothxr);
  degerxlexpo.add(smoothxl);
  degeryrexpo.add(smoothyr);

  //Get smoothed values
  xr = degerxrexpo.get();
  xl = degerxlexpo.get();
  yr = degeryrexpo.get();
  yl = degerexpo.get();
}

void printPackage() {
  Serial.print("Thrust: ");
  Serial.print(package.thrust);
  Serial.print("\tRoll: ");
  Serial.print(package.x);
  Serial.print("\tPitch: ");
  Serial.print(package.y);
  Serial.print("\tYaw: ");
  Serial.print(package.z);
  Serial.print("\tB1: ");
  Serial.print(package.but1);
  Serial.print("\tB2: ");
  Serial.print(package.but2);
  Serial.print("\tS1: ");
  Serial.print(package.switch1);
  Serial.print("\tS2: ");
  Serial.println(package.switch2);
}
