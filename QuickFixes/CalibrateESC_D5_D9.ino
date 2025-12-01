/*
 * ESC CALIBRATION - For Motors D5 and D9
 * 
 * This will calibrate the ESCs on pins D5 (Front Right) and D9 (Rear Left)
 * 
 * Instructions:
 * 1. Upload this sketch
 * 2. Open Serial Monitor (115200 baud)
 * 3. Type 'H' and press Enter
 * 4. CONNECT BATTERY (ESCs will beep rapidly)
 * 5. Type 'L' and press Enter
 * 6. ESCs beep confirmation = CALIBRATED!
 * 7. Re-run DiagnosticTool to verify
 */

#include <Servo.h>

Servo esc5, esc9;  // Front Right and Rear Left

void setup() {
  Serial.begin(115200);
  
  // Wait for Serial Monitor
  while (!Serial && millis() < 3000);
  
  Serial.println(F("\n\n"));
  Serial.println(F("╔═══════════════════════════════════════╗"));
  Serial.println(F("║   ESC CALIBRATION - D5 and D9         ║"));
  Serial.println(F("╚═══════════════════════════════════════╝"));
  Serial.println();
  Serial.println(F("Calibrating:"));
  Serial.println(F("  D5 = Front Right (FR)"));
  Serial.println(F("  D9 = Rear Left (RL)"));
  Serial.println();
  Serial.println(F("⚠️  REMOVE PROPELLERS! ⚠️"));
  Serial.println();
  Serial.println(F("INSTRUCTIONS:"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("1. Type 'H' and press Enter"));
  Serial.println(F("2. Wait for 'CONNECT BATTERY NOW'"));
  Serial.println(F("3. Connect battery (ESCs beep rapidly)"));
  Serial.println(F("4. Type 'L' and press Enter"));
  Serial.println(F("5. ESCs beep 2-3 times = SUCCESS!"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println();
  Serial.println(F("Waiting for commands..."));
  Serial.println();
  
  // Attach ESCs
  esc5.attach(5);
  esc9.attach(9);
  
  // Set to neutral
  esc5.writeMicroseconds(1500);
  esc9.writeMicroseconds(1500);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    // HIGH calibration point
    if (c == 'H' || c == 'h') {
      Serial.println();
      Serial.println(F(">>> STEP 1: Setting HIGH point (2000µs)"));
      Serial.println(F(">>> "));
      Serial.println(F(">>> ⚡ CONNECT BATTERY NOW! ⚡"));
      Serial.println(F(">>> "));
      Serial.println(F(">>> You should hear rapid beeping from ESCs"));
      Serial.println(F(">>> Then type 'L' and press Enter"));
      Serial.println();
      
      esc5.writeMicroseconds(2000);
      esc9.writeMicroseconds(2000);
    }
    
    // LOW calibration point
    if (c == 'L' || c == 'l') {
      Serial.println();
      Serial.println(F(">>> STEP 2: Setting LOW point (1000µs)"));
      Serial.println(F(">>> "));
      Serial.println(F(">>> ESCs should beep 2-3 times"));
      Serial.println();
      
      esc5.writeMicroseconds(1000);
      esc9.writeMicroseconds(1000);
      
      delay(2000);
      
      Serial.println(F("✓ CALIBRATION COMPLETE!"));
      Serial.println();
      Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
      Serial.println(F("NEXT STEPS:"));
      Serial.println(F("1. Disconnect battery"));
      Serial.println(F("2. Upload DiagnosticTool.ino"));
      Serial.println(F("3. Verify D5 and D9 now spin properly"));
      Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
      Serial.println();
      Serial.println(F("Press 'T' to TEST motors now (with battery)"));
      Serial.println();
    }
    
    // TEST mode
    if (c == 'T' || c == 't') {
      Serial.println();
      Serial.println(F(">>> TEST MODE: Spinning motors"));
      Serial.println();
      
      // Test D5
      Serial.println(F("Testing D5 (Front Right)..."));
      for (int i = 1000; i <= 1200; i += 10) {
        esc5.writeMicroseconds(i);
        delay(20);
      }
      delay(1000);
      esc5.writeMicroseconds(1000);
      delay(500);
      Serial.println(F("  Did D5 spin? (should be smooth)"));
      Serial.println();
      
      // Test D9
      Serial.println(F("Testing D9 (Rear Left)..."));
      for (int i = 1000; i <= 1200; i += 10) {
        esc9.writeMicroseconds(i);
        delay(20);
      }
      delay(1000);
      esc9.writeMicroseconds(1000);
      delay(500);
      Serial.println(F("  Did D9 spin? (should be smooth)"));
      Serial.println();
      
      // Test both
      Serial.println(F("Testing BOTH together..."));
      for (int i = 1000; i <= 1200; i += 10) {
        esc5.writeMicroseconds(i);
        esc9.writeMicroseconds(i);
        delay(20);
      }
      delay(1000);
      esc5.writeMicroseconds(1000);
      esc9.writeMicroseconds(1000);
      Serial.println(F("  Did both spin smoothly?"));
      Serial.println();
      
      Serial.println(F("✓ TEST COMPLETE"));
      Serial.println();
      Serial.println(F("If motors work → Upload FlightController.ino"));
      Serial.println(F("If still not working → Check wiring"));
      Serial.println();
    }
  }
}
