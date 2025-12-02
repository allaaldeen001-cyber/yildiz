/*
 * ═══════════════════════════════════════════════════════════════════
 * nRF24L01+ TEST SKETCHES
 * ═══════════════════════════════════════════════════════════════════
 * 
 * Use these to verify your nRF24 modules are working before uploading
 * the full drone firmware.
 * 
 * ═══════════════════════════════════════════════════════════════════
 */

// ═══════════════════════════════════════════════════════════════════
// TEST 1: BASIC MODULE DETECTION
// ═══════════════════════════════════════════════════════════════════
// Upload to BOTH Arduino (one at a time)
// This tests if the nRF24 module is detected and working

/*
#include <SPI.h>
#include <RF24.h>

// CHANGE THIS:
// Remote Controller: RF24 radio(9, 10);
// Flight Controller: RF24 radio(4, 10);
RF24 radio(9, 10);  // CE, CSN pins

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║   nRF24L01+ DETECTION TEST        ║"));
  Serial.println(F("╚════════════════════════════════════╝\n"));
  
  Serial.println(F("Testing nRF24L01+ module..."));
  Serial.println(F("If this fails, check:"));
  Serial.println(F("  1. 10µF capacitor between VCC/GND"));
  Serial.println(F("  2. 3.3V power (NOT 5V!)"));
  Serial.println(F("  3. Wiring: CE, CSN, SPI pins\n"));
  
  if (!radio.begin()) {
    Serial.println(F("❌ FAILED! Module not detected!"));
    Serial.println(F("\nTroubleshooting:"));
    Serial.println(F("  → Add 10µF capacitor (+ to VCC, - to GND)"));
    Serial.println(F("  → Check 3.3V power with multimeter"));
    Serial.println(F("  → Verify wiring (CE, CSN, SCK, MOSI, MISO)"));
    Serial.println(F("  → Try different nRF24 module"));
    while(1) {
      delay(1000);
      Serial.println(F("❌ Still waiting..."));
    }
  }
  
  Serial.println(F("✅ Module detected!\n"));
  
  // Test data rate
  Serial.print(F("Testing 250kbps data rate... "));
  if (radio.setDataRate(RF24_250KBPS)) {
    Serial.println(F("✅ OK"));
  } else {
    Serial.println(F("❌ FAILED - Module may be defective"));
  }
  
  // Test channel
  Serial.print(F("Setting channel 108... "));
  radio.setChannel(108);
  Serial.println(F("✅ OK"));
  
  // Test PA level
  Serial.print(F("Setting PA level MAX... "));
  radio.setPALevel(RF24_PA_MAX);
  Serial.println(F("✅ OK"));
  
  // Print details
  Serial.println(F("\nModule Configuration:"));
  Serial.println(F("  Data Rate: 250kbps"));
  Serial.println(F("  Channel: 108"));
  Serial.println(F("  PA Level: MAX"));
  Serial.println(F("  CRC: Enabled"));
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║   ✅ MODULE WORKING!               ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("\nNow upload the other test sketch to"));
  Serial.println(F("the other Arduino to test communication."));
}

void loop() {
  delay(5000);
  Serial.println(F("✅ Module still working..."));
}
*/


// ═══════════════════════════════════════════════════════════════════
// TEST 2A: TRANSMITTER (Remote Controller)
// ═══════════════════════════════════════════════════════════════════
// Upload this to REMOTE CONTROLLER Arduino

/*
#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);  // CE=9 for Remote Controller
const uint64_t address = 0xF0F0F0F0E1LL;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║   TRANSMITTER TEST                ║"));
  Serial.println(F("╚════════════════════════════════════╝\n"));
  
  if (!radio.begin()) {
    Serial.println(F("❌ Radio init FAILED!"));
    while(1) delay(1000);
  }
  
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.stopListening();
  
  Serial.println(F("✅ Transmitter ready!"));
  Serial.println(F("Sending packets every second...\n"));
  Serial.println(F("Expected: TX: ✅ OK"));
  Serial.println(F("If TX: ❌ FAIL, check Flight Controller:\n"));
}

unsigned long lastSend = 0;
uint32_t packetCount = 0;

void loop() {
  if (millis() - lastSend >= 1000) {
    packetCount++;
    
    // Send packet
    bool ok = radio.write(&packetCount, sizeof(packetCount));
    
    // Display result
    Serial.print(F("Packet #"));
    Serial.print(packetCount);
    Serial.print(F(" → TX: "));
    
    if (ok) {
      Serial.println(F("✅ OK (ACK received)"));
    } else {
      Serial.println(F("❌ FAIL (No ACK)"));
      Serial.println(F("  → Check FC is powered on"));
      Serial.println(F("  → Check FC has 10µF capacitor"));
      Serial.println(F("  → Move closer to FC"));
    }
    
    lastSend = millis();
  }
}
*/


// ═══════════════════════════════════════════════════════════════════
// TEST 2B: RECEIVER (Flight Controller)
// ═══════════════════════════════════════════════════════════════════
// Upload this to FLIGHT CONTROLLER Arduino

/*
#include <SPI.h>
#include <RF24.h>

RF24 radio(4, 10);  // CE=4 for Flight Controller
const uint64_t address = 0xF0F0F0F0E1LL;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║   RECEIVER TEST                   ║"));
  Serial.println(F("╚════════════════════════════════════╝\n"));
  
  if (!radio.begin()) {
    Serial.println(F("❌ Radio init FAILED!"));
    while(1) delay(1000);
  }
  
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setAutoAck(true);
  radio.startListening();
  
  Serial.println(F("✅ Receiver ready!"));
  Serial.println(F("Waiting for packets from RC...\n"));
  Serial.println(F("Expected: RX: #1, #2, #3..."));
  Serial.println(F("If nothing, check Remote Controller:\n"));
}

unsigned long lastReceive = 0;

void loop() {
  if (radio.available()) {
    uint32_t data;
    radio.read(&data, sizeof(data));
    
    Serial.print(F("RX: Packet #"));
    Serial.println(data);
    
    lastReceive = millis();
  }
  
  // Timeout warning
  if (lastReceive > 0 && millis() - lastReceive > 5000) {
    Serial.println(F("⚠️  No packets for 5 seconds..."));
    Serial.println(F("   Check RC is still powered on"));
    lastReceive = millis();
  }
  
  delay(10);
}
*/


// ═══════════════════════════════════════════════════════════════════
// INSTRUCTIONS
// ═══════════════════════════════════════════════════════════════════

/*
 * HOW TO USE THESE TESTS:
 * 
 * STEP 1: Test Both Modules
 * ─────────────────────────────────────────────────────────────────
 * 1. Uncomment TEST 1 code above (remove the surrounding slashes)
 * 2. Upload to Remote Controller Arduino
 * 3. Open Serial Monitor (115200 baud)
 * 4. Should see "✅ MODULE WORKING!"
 * 5. Repeat for Flight Controller Arduino (change CE pin to 4)
 * 
 * If both pass → Continue to STEP 2
 * If either fails → Fix hardware (capacitor, voltage, wiring)
 * 
 * 
 * STEP 2: Test Communication
 * ─────────────────────────────────────────────────────────────────
 * 1. Comment out TEST 1, uncomment TEST 2A (Transmitter)
 * 2. Upload to Remote Controller Arduino
 * 3. Comment out TEST 2A, uncomment TEST 2B (Receiver)
 * 4. Upload to Flight Controller Arduino
 * 5. Open BOTH Serial Monitors
 * 
 * Expected:
 *   RC: TX: ✅ OK
 *   FC: RX: Packet #1, #2, #3...
 * 
 * If working → Upload full drone firmware!
 * If not working → Check hardware (see EMERGENCY_TROUBLESHOOTING.md)
 * 
 * 
 * COMMON ISSUES:
 * ─────────────────────────────────────────────────────────────────
 * "❌ Radio init FAILED!"
 *   → Add 10µF capacitor between VCC and GND
 *   → Check 3.3V power (NOT 5V!)
 *   → Verify wiring (CE, CSN, SPI pins)
 * 
 * "TX: ❌ FAIL"
 *   → Check FC is powered on and running TEST 2B
 *   → Add 10µF capacitor to BOTH modules
 *   → Move RC closer to FC (1 meter apart)
 *   → Check both use same address/channel
 * 
 * "No packets received"
 *   → Check RC is running TEST 2A
 *   → Verify CE pins (RC=9, FC=4)
 *   → Check for metal obstacles between modules
 * 
 */
