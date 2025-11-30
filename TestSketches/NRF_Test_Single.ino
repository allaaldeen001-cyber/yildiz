/*
 * NRF24L01 Single Module Test
 * Upload this to test if NRF module is detected
 * 
 * FLIGHT CONTROLLER: Change CE to 4
 * REMOTE CONTROLLER: Change CE to 9
 */

#include <SPI.h>
#include <RF24.h>

// CHANGE THIS:
// Flight Controller: CE = 4
// Remote Controller: CE = 9
#define CE_PIN 4    // ← CHANGE TO 9 FOR REMOTE!
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F(""));
  Serial.println(F("================================"));
  Serial.println(F("   NRF24L01 MODULE TEST"));
  Serial.println(F("================================"));
  Serial.println(F(""));
  
  Serial.print(F("Testing with CE=D"));
  Serial.print(CE_PIN);
  Serial.print(F(", CSN=D"));
  Serial.println(CSN_PIN);
  Serial.println(F(""));
  
  Serial.println(F("Initializing NRF24L01..."));
  
  if (!radio.begin()) {
    Serial.println(F(""));
    Serial.println(F("❌❌❌ FAILED ❌❌❌"));
    Serial.println(F("NRF24L01 NOT DETECTED!"));
    Serial.println(F(""));
    Serial.println(F("CHECK:"));
    Serial.println(F("1. 10μF capacitor between VCC-GND"));
    Serial.println(F("2. VCC connected to 3.3V (NOT 5V!)"));
    Serial.println(F("3. GND connected"));
    Serial.println(F("4. CE → D4 (FC) or D9 (RC)"));
    Serial.println(F("5. CSN → D10"));
    Serial.println(F("6. MOSI → D11"));
    Serial.println(F("7. MISO → D12"));
    Serial.println(F("8. SCK → D13"));
    Serial.println(F(""));
    Serial.println(F("Module may be defective!"));
    while(1) {
      delay(1000);
    }
  }
  
  Serial.println(F(""));
  Serial.println(F("✅✅✅ SUCCESS! ✅✅✅"));
  Serial.println(F("NRF24L01 DETECTED!"));
  Serial.println(F(""));
  
  // Configure for testing
  radio.setChannel(103);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  
  Serial.println(F("Module Details:"));
  Serial.println(F("-------------------"));
  radio.printDetails();
  Serial.println(F(""));
  
  Serial.println(F("✅ Hardware test PASSED!"));
  Serial.println(F("Wiring is correct."));
  Serial.println(F(""));
}

void loop() {
  static unsigned long count = 0;
  
  Serial.print(F("✅ NRF24 is working! ("));
  Serial.print(++count);
  Serial.println(F(")"));
  
  delay(2000);
}
