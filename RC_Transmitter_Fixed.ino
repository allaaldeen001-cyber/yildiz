/* RC TRANSMITTER CODE - FIXED & ENHANCED
   Fixed: Throttle inversion (UP = More Power)
   Added: Enhanced communication status display
   Added: Real-time signal quality monitoring
   Added: Packet loss tracking
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

// Communication monitoring
unsigned long lastSuccessTime = 0;
int successCount = 0;
int failCount = 0;
int totalPackets = 0;
float successRate = 100.0;
unsigned long statsResetTime = 0;

void readJoyStick();
void printPackage();
void updateCommStats();
void printDetailedStatus();

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
  
  // CHANGED: Using LOW power to prevent link loss (Brownouts)
  // If you solder a 10uF capacitor to the NRF module, you can try MAX
  radio.setPALevel(RF24_PA_LOW);    
  
  radio.openWritingPipe(pipe);
  radio.stopListening();
  
  // Smoothing Setup
  degerexpo.begin(SMOOTHED_EXPONENTIAL, 5); 
  degerxrexpo.begin(SMOOTHED_EXPONENTIAL, 5);
  degerxlexpo.begin(SMOOTHED_EXPONENTIAL, 5);
  degeryrexpo.begin(SMOOTHED_EXPONENTIAL, 5);
  
  Serial.println(F("========================================"));
  Serial.println(F("  QUADCOPTER RC TRANSMITTER - READY"));
  Serial.println(F("========================================"));
  Serial.println(F("Channel: 108 | Power: LOW | Rate: 250kbps"));
  Serial.println(F("Throttle: FIXED (UP = More Power)"));
  Serial.println(F("========================================"));
  Serial.println();
  
  statsResetTime = millis();
}

void loop() {
  readJoyStick();

  package.x =        (xr + calX) * scaleX + offsetX;
  package.y =        (yr + calY) * scaleY + offsetY;
  package.z =        (xl + calZ) * scaleZ + offsetZ;
  
  // === FIXED THROTTLE: UP = More Power ===
  // Swapped 2000 and 1000 to fix inversion
  package.thrust = map(yl, 0, 1023, 2000, 1000);
  
  // Safety constrain
  package.thrust = constrain(package.thrust, 1000, 2000);

  package.id = ID;
  ID++;
  
  package.but1 = but1;
  package.but2 = but2;
  package.switch1 = switch1;
  package.switch2 = switch2;

  // Send
  txStatus = radio.write(&package, sizeof(package));
  
  // Update communication statistics
  updateCommStats();
  
  // Print status every cycle
  printPackage();
  
  // Detailed status every 2 seconds
  if (millis() % 2000 < 10) {
    printDetailedStatus();
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
  // Raw value (0-1023) - mapping handled in loop()
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

void updateCommStats() {
  totalPackets++;
  
  if (txStatus) {
    successCount++;
    lastSuccessTime = millis();
  } else {
    failCount++;
  }
  
  // Calculate success rate
  if (totalPackets > 0) {
    successRate = (successCount * 100.0) / totalPackets;
  }
  
  // Reset stats every 10 seconds to show current performance
  if (millis() - statsResetTime > 10000) {
    successCount = 0;
    failCount = 0;
    totalPackets = 0;
    statsResetTime = millis();
  }
}

void printPackage()
{
  // Compact single-line status
  Serial.print(F("T:"));
  Serial.print(package.thrust);
  Serial.print(F(" | Y:"));
  Serial.print(package.z, 1); 
  Serial.print(F(" | R:"));
  Serial.print(package.x, 1); 
  Serial.print(F(" | P:"));
  Serial.print(package.y, 1); 
  
  Serial.print(F(" | Link:"));
  if(txStatus) {
    Serial.print(F("✓OK"));
  } else {
    Serial.print(F("✗LOST"));
  }
  
  Serial.print(F(" | Rate:"));
  Serial.print(successRate, 0);
  Serial.print(F("%"));
  
  // Connection quality indicator
  unsigned long timeSinceSuccess = millis() - lastSuccessTime;
  if (timeSinceSuccess > 1000) {
    Serial.print(F(" [WARNING: No link >1s]"));
  } else if (successRate < 90) {
    Serial.print(F(" [WEAK]"));
  } else if (successRate >= 98) {
    Serial.print(F(" [EXCELLENT]"));
  }
  
  Serial.println();
}

void printDetailedStatus() {
  Serial.println();
  Serial.println(F("========== LINK STATUS =========="));
  Serial.print(F("Success Rate: "));
  Serial.print(successRate, 1);
  Serial.println(F("%"));
  Serial.print(F("Total Packets: "));
  Serial.println(totalPackets);
  Serial.print(F("Success: "));
  Serial.print(successCount);
  Serial.print(F(" | Failed: "));
  Serial.println(failCount);
  
  Serial.println(F("========== CONTROLS =========="));
  Serial.print(F("Throttle: "));
  Serial.print(package.thrust);
  Serial.print(F(" (Raw: "));
  Serial.print(yl, 0);
  Serial.println(F(")"));
  
  Serial.print(F("Roll: "));
  Serial.print(package.x, 2);
  Serial.print(F(" | Pitch: "));
  Serial.print(package.y, 2);
  Serial.print(F(" | Yaw: "));
  Serial.println(package.z, 2);
  
  Serial.print(F("Arm Switch: "));
  Serial.print(switch1 ? F("OFF") : F("ON"));
  Serial.print(F(" | Hold: "));
  Serial.print(switch2 ? F("OFF") : F("ON"));
  Serial.print(F(" | Btn1: "));
  Serial.print(but1 ? F("UP") : F("DOWN"));
  Serial.print(F(" | Btn2: "));
  Serial.println(but2 ? F("UP") : F("DOWN"));
  Serial.println(F("================================="));
  Serial.println();
}
