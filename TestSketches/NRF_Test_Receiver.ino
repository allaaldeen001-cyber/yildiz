/*
 * NRF24L01 Receiver Test
 * Upload this to FLIGHT CONTROLLER Arduino
 * 
 * Receives test messages from Remote Controller
 */

#include <SPI.h>
#include <RF24.h>

RF24 radio(4, 10);  // CE=D4, CSN=D10 for Flight Controller

const byte address[6] = "DRONE";

unsigned long rxCount = 0;
unsigned long lastRxTime = 0;
unsigned long noSignalTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F(""));
  Serial.println(F("================================"));
  Serial.println(F("  RECEIVER TEST (FLIGHT CTRL)"));
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
  radio.openReadingPipe(1, address);
  radio.startListening();
  
  Serial.println(F(""));
  Serial.println(F("Configuration:"));
  Serial.println(F("  Channel: 103"));
  Serial.println(F("  Power: MAX"));
  Serial.println(F("  Data Rate: 250kbps"));
  Serial.println(F("  Address: DRONE"));
  Serial.println(F(""));
  Serial.println(F("Listening for messages..."));
  Serial.println(F("(Make sure Transmitter is running!)"));
  Serial.println(F(""));
  
  lastRxTime = millis();
}

void loop() {
  if (radio.available()) {
    char message[32] = "";
    radio.read(&message, sizeof(message));
    
    rxCount++;
    lastRxTime = millis();
    noSignalTime = 0;
    
    Serial.print(F("✅ RX #"));
    Serial.print(rxCount);
    Serial.print(F(" - Received: '"));
    Serial.print(message);
    Serial.println(F("'"));
    
    // Status every 10 messages
    if (rxCount % 10 == 0) {
      Serial.println(F(""));
      Serial.println(F("--- STATUS ---"));
      Serial.print(F("Total Received: "));
      Serial.println(rxCount);
      Serial.println(F("✅ Communication working!"));
      Serial.println(F(""));
    }
    
  } else {
    // No data available
    if (millis() - lastRxTime > 5000 && noSignalTime == 0) {
      noSignalTime = millis();
      Serial.println(F(""));
      Serial.println(F("⚠️ NO SIGNAL for 5 seconds!"));
      Serial.println(F(""));
      Serial.println(F("CHECK:"));
      Serial.println(F("1. Is Transmitter powered on?"));
      Serial.println(F("2. Is Transmitter sketch uploaded?"));
      Serial.println(F("3. Both modules have 10μF capacitors?"));
      Serial.println(F("4. Modules close together (1-2 meters)?"));
      Serial.println(F("5. Same channel (103) on both?"));
      Serial.println(F(""));
    }
    
    delay(100);
  }
}
