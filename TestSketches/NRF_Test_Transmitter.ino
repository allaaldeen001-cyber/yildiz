/*
 * NRF24L01 Transmitter Test
 * Upload this to REMOTE CONTROLLER Arduino
 * 
 * Sends test messages to Flight Controller
 */

#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);  // CE=D9, CSN=D10 for Remote Controller

const byte address[6] = "DRONE";

unsigned long txCount = 0;
unsigned long txSuccess = 0;
unsigned long txFail = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F(""));
  Serial.println(F("================================"));
  Serial.println(F("  TRANSMITTER TEST (REMOTE)"));
  Serial.println(F("================================"));
  Serial.println(F(""));
  
  if (!radio.begin()) {
    Serial.println(F("❌ NRF24L01 FAILED!"));
    Serial.println(F("Check wiring and capacitor!"));
    while(1);
  }
  
  Serial.println(F("✅ NRF24L01 detected"));
  
  radio.setChannel(103);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.openWritingPipe(address);
  radio.stopListening();
  
  Serial.println(F(""));
  Serial.println(F("Configuration:"));
  Serial.println(F("  Channel: 103"));
  Serial.println(F("  Power: MAX"));
  Serial.println(F("  Data Rate: 250kbps"));
  Serial.println(F("  Address: DRONE"));
  Serial.println(F(""));
  Serial.println(F("Ready to transmit!"));
  Serial.println(F("Sending test messages..."));
  Serial.println(F(""));
}

void loop() {
  char message[32];
  sprintf(message, "Test #%lu", txCount);
  
  bool ok = radio.write(&message, sizeof(message));
  
  txCount++;
  if (ok) {
    txSuccess++;
    Serial.print(F("✅ TX #"));
    Serial.print(txCount);
    Serial.print(F(" - SUCCESS"));
  } else {
    txFail++;
    Serial.print(F("❌ TX #"));
    Serial.print(txCount);
    Serial.print(F(" - FAILED"));
  }
  
  float successRate = 100.0 * txSuccess / txCount;
  Serial.print(F(" | Success Rate: "));
  Serial.print(successRate, 1);
  Serial.print(F("% ("));
  Serial.print(txSuccess);
  Serial.print(F("/"));
  Serial.print(txCount);
  Serial.println(F(")"));
  
  delay(1000);
  
  // Status every 10 messages
  if (txCount % 10 == 0) {
    Serial.println(F(""));
    Serial.println(F("--- STATUS ---"));
    Serial.print(F("Total Sent: "));
    Serial.println(txCount);
    Serial.print(F("Successful: "));
    Serial.println(txSuccess);
    Serial.print(F("Failed: "));
    Serial.println(txFail);
    Serial.print(F("Success Rate: "));
    Serial.print(successRate, 1);
    Serial.println(F("%"));
    Serial.println(F(""));
    
    if (successRate < 50) {
      Serial.println(F("⚠️ WARNING: Low success rate!"));
      Serial.println(F("Check:"));
      Serial.println(F("1. Receiver is powered on"));
      Serial.println(F("2. Both modules have capacitors"));
      Serial.println(F("3. Modules not too far apart"));
      Serial.println(F("4. No metal blocking signal"));
      Serial.println(F(""));
    }
  }
}
