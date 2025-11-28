#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Smoothed.h>

// ============ PIN CONFIGURATION ============
// NRF24L01: CE=D9, CSN=D10
RF24 radio(9, 10);

// Joystick pins
const int THROTTLE_PIN = A0;  // Left stick - Up/Down
const int YAW_PIN = A1;        // Left stick - Left/Right
const int PITCH_PIN = A2;      // Right stick - Up/Down
const int ROLL_PIN = A3;       // Right stick - Left/Right

// Optional status LED
const int STATUS_LED = 8;
// ===========================================

const uint64_t pipe = 0xF0F0F0F0E1LL;

// Joystick calibration values
float scaleRoll = 0.1;
float calRoll = -527;
float offsetRoll = 0;

float scalePitch = -0.1;
float calPitch = -507;
float offsetPitch = 0;

float scaleYaw = -0.1;
float calYaw = -512;
float offsetYaw = 0;

float scaleThrust = 1.5;
float calThrust = -500;
float offsetThrust = 1300;

int ID = 0;
float roll, pitch, yaw, throttle;
float smoothRoll, smoothPitch, smoothYaw, smoothThrottle;

// Smoothing filters
Smoothed <float> filterThrottle;
Smoothed <float> filterRoll;
Smoothed <float> filterYaw;
Smoothed <float> filterPitch;

struct Package
{
  int   thrust = 0;
  float x = 0;      // Roll
  float y = 0;      // Pitch
  float z = 0;      // Yaw
  int   id = 0;
  bool  but1 = 1;
  bool  but2 = 1;
  bool  switch1 = 1;
  bool  switch2 = 1;
};

Package package;

void readJoystick();
void printPackage();

void setup() {
  Serial.begin(57600);
  
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // Initialize NRF24L01 with ACK enabled
  radio.begin();
  radio.setAutoAck(true);  // ACK enabled
  radio.enableAckPayload();
  radio.setRetries(5, 15);  // 5 retries, 15*250µs delay
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  // Initialize smoothing filters
  filterThrottle.begin(SMOOTHED_EXPONENTIAL, 1);
  filterRoll.begin(SMOOTHED_EXPONENTIAL, 1);
  filterYaw.begin(SMOOTHED_EXPONENTIAL, 1);
  filterPitch.begin(SMOOTHED_EXPONENTIAL, 1);
  
  Serial.println("RC Controller Ready");
  
  // Blink LED to indicate ready
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
  }
}

void loop() {
  readJoystick();

  // Map joystick values to package
  // Roll = Right stick Left/Right (A3)
  // Pitch = Right stick Up/Down (A2)
  // Yaw = Left stick Left/Right (A1)
  // Throttle = Left stick Up/Down (A0)
  
  package.x = (roll + calRoll) * scaleRoll + offsetRoll;        // Roll
  package.y = (pitch + calPitch) * scalePitch + offsetPitch;    // Pitch
  package.z = (yaw + calYaw) * scaleYaw + offsetYaw;            // Yaw
  package.thrust = (throttle + calThrust) * scaleThrust + offsetThrust;
  package.id = ID;
  ID++;
  
  // Transmit with ACK
  bool success = radio.write(&package, sizeof(package));
  
  // Indicate transmission status on LED
  if (success) {
    digitalWrite(STATUS_LED, HIGH);
  } else {
    digitalWrite(STATUS_LED, LOW);
  }
  
  printPackage();
  
  delay(10);  // ~100Hz update rate
}

void readJoystick()
{
  // Read raw values
  smoothRoll = analogRead(ROLL_PIN);           // A3 - Right stick X
  smoothPitch = 1023 - analogRead(PITCH_PIN);  // A2 - Right stick Y (inverted)
  smoothYaw = analogRead(YAW_PIN);             // A1 - Left stick X
  smoothThrottle = max(analogRead(THROTTLE_PIN), 0);  // A0 - Left stick Y
  
  // Apply smoothing
  filterThrottle.add(smoothThrottle);
  filterRoll.add(smoothRoll);
  filterYaw.add(smoothYaw);
  filterPitch.add(smoothPitch);
  
  throttle = filterThrottle.get();
  roll = filterRoll.get();
  yaw = filterYaw.get();
  pitch = filterPitch.get();
}

void printPackage()
{
  // Print for debugging
  Serial.print("Throttle: ");
  Serial.print(package.thrust);
  Serial.print("\t Yaw: ");
  Serial.print(package.z);
  Serial.print("\t Roll: ");
  Serial.print(package.x);
  Serial.print("\t Pitch: ");
  Serial.print(package.y);
  Serial.print("\t ID: ");
  Serial.println(package.id);
}
