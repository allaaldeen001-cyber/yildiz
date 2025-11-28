/* RC TRANSMITTER CODE - IMPROVED VERSION
   Fixed: Throttle inversion issue (throttle stick UP now increases speed)
   Fixed: Enhanced communication status display
   Fixed: Better calibration and smoothing
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Smoothed.h>

Smoothed <float> degerexpo;
Smoothed <float> degerxrexpo;
Smoothed <float> degerxlexpo;
Smoothed <float> degeryrexpo;

// CE, CSN pins
RF24 radio(9, 10);

const uint64_t pipe = 0xF0F0F0F0E1LL;

// Calibration Values
float scaleX =  0.1;
float calX =    -527;
float offsetX = 0;

float scaleY = -0.1;
float calY = -507;
float offsetY = 0;

float scaleZ = -0.1;
float calZ = -512;
float offsetZ = 0;

// Pins
const int XL_pin = 1; // A1
const int YL_pin = 0; // A0 (Throttle)
const int XR_pin = 3; // A3
const int YR_pin = 2; // A2

int ID = 0;
float xr, yr;
float xl, yl;
float smoothyl, smoothxr, smoothxl, smoothyr;
bool but1, but2, switch1, switch2;
bool txStatus = false;

// Communication Statistics
unsigned long packetsSent = 0;
unsigned long packetsLost = 0;
unsigned long lastStatusPrint = 0;
unsigned long lastPacketTime = 0;
bool linkStatus = false;

void readJoyStick();
void printPackage();
void printStatus();

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
  pinMode(2, INPUT_PULLUP); //Switch 2
  pinMode(3, INPUT_PULLUP); //Switch 1
  pinMode(4, INPUT_PULLUP); //Buton 1
  pinMode(5, INPUT_PULLUP); //Buton 2

  Serial.begin(57600);
  
  // --- NRF Setup ---
  radio.begin();
  radio.setChannel(108);            
  radio.setAutoAck(true);            
  radio.enableDynamicPayloads();    
  radio.setRetries(5, 15);          
  radio.setDataRate(RF24_250KBPS);  
  
  // CHANGED: Reduced power to LOW to prevent link loss (Brownouts)
  // If you solder a 10uF capacitor to the NRF module, you can try MAX again.
  radio.setPALevel(RF24_PA_LOW);    
  
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  // Smoothing Setup - Increased smoothing for better stability
  degerexpo.begin(SMOOTHED_EXPONENTIAL, 7); 
  degerxrexpo.begin(SMOOTHED_EXPONENTIAL, 7);
  degerxlexpo.begin(SMOOTHED_EXPONENTIAL, 7);
  degeryrexpo.begin(SMOOTHED_EXPONENTIAL, 7);
  
  Serial.println("========================================");
  Serial.println("RC TRANSMITTER READY");
  Serial.println("Channel: 108 | PA Level: LOW");
  Serial.println("========================================");
  Serial.println("Waiting for joystick input...");
  Serial.println();
  
  delay(500);
}

void loop() {
  readJoyStick();

  package.x =        (xr + calX) * scaleX + offsetX;
  package.y =        (yr + calY) * scaleY + offsetY;
  package.z =        (xl + calZ) * scaleZ + offsetZ;
  
  // --- FIXED THROTTLE LOGIC ---
  // FIXED: Inverted mapping so UP stick = HIGH speed
  // Mapping: 0 input -> 2000 output (Max), 1023 input -> 1000 output (Min)
  // This ensures pushing throttle stick UP increases drone speed
  package.thrust = map(yl, 0, 1023, 2000, 1000);

  // Safety constrain
  package.thrust = constrain(package.thrust, 1000, 2000);

  package.id = ID;
  ID++;
  
  package.but1 = but1;
  package.but2 = but2;
  package.switch1 = switch1;
  package.switch2 = switch2;

  // Send packet
  txStatus = radio.write(&package, sizeof(package));
  
  // Update statistics
  if (txStatus) {
    packetsSent++;
    linkStatus = true;
    lastPacketTime = millis();
  } else {
    packetsLost++;
    // Check if link is lost (no successful packet for 500ms)
    if (millis() - lastPacketTime > 500) {
      linkStatus = false;
    }
  }
  
  // Print status every 500ms
  if (millis() - lastStatusPrint > 500) {
    printStatus();
    lastStatusPrint = millis();
  }
  
  delay(5); 
}

void readJoyStick()
{
  but1 = digitalRead(4);
  but2 = digitalRead(5);
  switch1 = digitalRead(3);
  switch2 = digitalRead(2);

  smoothxr = analogRead(XR_pin);
  smoothxl = analogRead(XL_pin);
  
  // Pitch/Roll/Yaw reading
  smoothyr = 1023 - analogRead(YR_pin);
  
  // THROTTLE READING
  // Read raw value (0-1023)
  // The inversion/scaling is handled in loop() via map()
  smoothyl = analogRead(YL_pin); 

  degerexpo.add(smoothyl);
  degerxrexpo.add(smoothxr);
  degerxlexpo.add(smoothxl);
  degeryrexpo.add(smoothyr);

  xr = degerxrexpo.get();
  xl = degerxlexpo.get();
  yr = degeryrexpo.get();
  yl = degerexpo.get();
}

void printStatus()
{
  // Clear previous line (for cleaner output)
  Serial.print("\r");
  
  // Calculate link quality percentage
  unsigned long totalPackets = packetsSent + packetsLost;
  float linkQuality = 0;
  if (totalPackets > 0) {
    linkQuality = (float(packetsSent) / float(totalPackets)) * 100.0;
  }
  
  // Status display
  Serial.print("T:"); Serial.print(package.thrust);
  Serial.print(" | Y:"); Serial.print(package.z, 2); 
  Serial.print(" | R:"); Serial.print(package.x, 2); 
  Serial.print(" | P:"); Serial.print(package.y, 2);
  Serial.print(" | Link: ");
  
  if (linkStatus) {
    Serial.print("OK");
  } else {
    Serial.print("LOST");
  }
  
  Serial.print(" | Quality: ");
  Serial.print(linkQuality, 1);
  Serial.print("%");
  
  Serial.print(" | Sent: ");
  Serial.print(packetsSent);
  
  Serial.print(" | Lost: ");
  Serial.print(packetsLost);
  
  // Print button/switch status
  Serial.print(" | B1:");
  Serial.print(but1 ? "OFF" : "ON");
  Serial.print(" B2:");
  Serial.print(but2 ? "OFF" : "ON");
  Serial.print(" S1:");
  Serial.print(switch1 ? "OFF" : "ON");
  Serial.print(" S2:");
  Serial.print(switch2 ? "OFF" : "ON");
  
  Serial.print("      "); // Padding to clear any leftover characters
}

void printPackage()
{
  // Detailed package info (called less frequently)
  Serial.println();
  Serial.println("=== PACKAGE DETAILS ===");
  Serial.print("Thrust: "); Serial.println(package.thrust);
  Serial.print("Roll (X): "); Serial.println(package.x, 3);
  Serial.print("Pitch (Y): "); Serial.println(package.y, 3);
  Serial.print("Yaw (Z): "); Serial.println(package.z, 3);
  Serial.print("Packet ID: "); Serial.println(package.id);
  Serial.println("======================");
}
