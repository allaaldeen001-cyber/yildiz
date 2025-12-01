/*
 * DIAGNOSTIC TOOL - Find Problems Fast
 * 
 * This sketch tests ALL systems and shows what's working/broken
 * 
 * Upload this FIRST to diagnose issues!
 */

#include <Wire.h>
#include <Servo.h>

// Pin definitions
#define MOTOR_FL 3
#define MOTOR_FR 5
#define MOTOR_RR 6
#define MOTOR_RL 9
#define BUZZER_PIN 8
#define LED_PIN 7

#define MPU6050_ADDR 0x68

Servo motorFL, motorFR, motorRR, motorRL;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Wait max 3 seconds for Serial
  
  Serial.println(F("\n\n\n"));
  Serial.println(F("╔═══════════════════════════════════════╗"));
  Serial.println(F("║    QUADCOPTER DIAGNOSTIC TOOL         ║"));
  Serial.println(F("║    Finding Problems...                ║"));
  Serial.println(F("╚═══════════════════════════════════════╝"));
  Serial.println();
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Test 1: Buzzer and LED
  testBuzzerLED();
  
  // Test 2: I2C Scanner
  testI2C();
  
  // Test 3: MPU6050
  testMPU6050();
  
  // Test 4: Motors
  testMotors();
  
  // Summary
  printSummary();
  
  Serial.println(F("\n=== DIAGNOSTIC COMPLETE ==="));
  Serial.println(F("Scroll up to see all results!"));
  Serial.println();
}

void loop() {
  // Live MPU6050 monitoring
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    readAndPrintMPU();
    lastPrint = millis();
  }
}

// ====================================
// TEST 1: BUZZER AND LED
// ====================================
void testBuzzerLED() {
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("TEST 1: BUZZER AND LED"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  
  // LED test
  Serial.print(F("Testing LED (pin D7)... "));
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  delay(200);
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
  Serial.println(F("✓ Should have blinked"));
  
  // Buzzer test
  Serial.print(F("Testing Buzzer (pin D8)... "));
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println(F("✓ Should have beeped"));
  Serial.println();
}

// ====================================
// TEST 2: I2C SCANNER
// ====================================
void testI2C() {
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("TEST 2: I2C BUS SCAN"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  
  Wire.begin();
  Wire.setClock(400000);
  
  Serial.println(F("Scanning I2C bus (addresses 0x01-0x7F)..."));
  Serial.println();
  
  int devicesFound = 0;
  
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print(F("✓ FOUND device at 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.print(address, HEX);
      
      if (address == 0x68 || address == 0x69) {
        Serial.print(F("  ← MPU6050!"));
      }
      Serial.println();
      devicesFound++;
    }
  }
  
  Serial.println();
  if (devicesFound == 0) {
    Serial.println(F("✗ ERROR: No I2C devices found!"));
    Serial.println(F("  Check MPU6050 wiring:"));
    Serial.println(F("    SDA → A4"));
    Serial.println(F("    SCL → A5"));
    Serial.println(F("    VCC → 3.3V"));
    Serial.println(F("    GND → GND"));
  } else {
    Serial.print(F("Found "));
    Serial.print(devicesFound);
    Serial.println(F(" device(s)"));
  }
  Serial.println();
}

// ====================================
// TEST 3: MPU6050 DETAILED TEST
// ====================================
void testMPU6050() {
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("TEST 3: MPU6050 SENSOR"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x75); // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1, true);
  
  if (Wire.available()) {
    byte whoAmI = Wire.read();
    Serial.print(F("WHO_AM_I register: 0x"));
    Serial.println(whoAmI, HEX);
    
    if (whoAmI == 0x68) {
      Serial.println(F("✓ MPU6050 identified correctly!"));
    } else {
      Serial.println(F("✗ WARNING: Unexpected WHO_AM_I value"));
    }
  } else {
    Serial.println(F("✗ ERROR: Cannot read from MPU6050!"));
    Serial.println(F("  MPU6050 not responding!"));
    return;
  }
  
  // Wake up MPU6050
  Serial.print(F("Waking up MPU6050... "));
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Wake up
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println(F("✓ OK"));
  } else {
    Serial.println(F("✗ FAILED"));
    return;
  }
  
  delay(100);
  
  // Read sensor data
  Serial.println(F("\nReading sensor data (move MPU to see changes):"));
  Serial.println(F("--------------------------------------------------"));
  
  for (int i = 0; i < 5; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);
    
    if (Wire.available() == 14) {
      int16_t accelX = Wire.read() << 8 | Wire.read();
      int16_t accelY = Wire.read() << 8 | Wire.read();
      int16_t accelZ = Wire.read() << 8 | Wire.read();
      int16_t temp = Wire.read() << 8 | Wire.read();
      int16_t gyroX = Wire.read() << 8 | Wire.read();
      int16_t gyroY = Wire.read() << 8 | Wire.read();
      int16_t gyroZ = Wire.read() << 8 | Wire.read();
      
      Serial.print(F("Sample "));
      Serial.print(i + 1);
      Serial.print(F(": AX="));
      Serial.print(accelX);
      Serial.print(F(" AY="));
      Serial.print(accelY);
      Serial.print(F(" AZ="));
      Serial.print(accelZ);
      Serial.print(F(" | GX="));
      Serial.print(gyroX);
      Serial.print(F(" GY="));
      Serial.print(gyroY);
      Serial.print(F(" GZ="));
      Serial.println(gyroZ);
      
      // Check if data looks reasonable
      if (accelX == 0 && accelY == 0 && accelZ == 0 && 
          gyroX == 0 && gyroY == 0 && gyroZ == 0) {
        Serial.println(F("✗ ERROR: All zeros! MPU6050 not working!"));
        break;
      } else if (i == 4) {
        Serial.println(F("✓ MPU6050 reading data correctly!"));
      }
    } else {
      Serial.println(F("✗ ERROR: Cannot read sensor data!"));
      break;
    }
    
    delay(200);
  }
  Serial.println();
}

// ====================================
// TEST 4: MOTOR TEST
// ====================================
void testMotors() {
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("TEST 4: MOTOR/ESC TEST"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("⚠️  REMOVE PROPELLERS! ⚠️"));
  Serial.println();
  Serial.println(F("Testing each motor individually..."));
  Serial.println();
  
  // Attach all motors
  motorFL.attach(MOTOR_FL);
  motorFR.attach(MOTOR_FR);
  motorRR.attach(MOTOR_RR);
  motorRL.attach(MOTOR_RL);
  
  // Initialize to min
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorRR.writeMicroseconds(1000);
  motorRL.writeMicroseconds(1000);
  
  delay(1000);
  
  // Test FL
  Serial.println(F("Testing Front Left (D3)..."));
  Serial.println(F("  Should spin for 2 seconds"));
  for (int speed = 1000; speed <= 1200; speed += 10) {
    motorFL.writeMicroseconds(speed);
    delay(20);
  }
  delay(1000);
  motorFL.writeMicroseconds(1000);
  delay(500);
  Serial.println(F("  ✓ Did it spin? (Y/N)"));
  Serial.println();
  
  // Test FR
  Serial.println(F("Testing Front Right (D5)..."));
  Serial.println(F("  Should spin for 2 seconds"));
  for (int speed = 1000; speed <= 1200; speed += 10) {
    motorFR.writeMicroseconds(speed);
    delay(20);
  }
  delay(1000);
  motorFR.writeMicroseconds(1000);
  delay(500);
  Serial.println(F("  ✓ Did it spin? (Y/N)"));
  Serial.println();
  
  // Test RR
  Serial.println(F("Testing Rear Right (D6)..."));
  Serial.println(F("  Should spin for 2 seconds"));
  for (int speed = 1000; speed <= 1200; speed += 10) {
    motorRR.writeMicroseconds(speed);
    delay(20);
  }
  delay(1000);
  motorRR.writeMicroseconds(1000);
  delay(500);
  Serial.println(F("  ✓ Did it spin? (Y/N)"));
  Serial.println();
  
  // Test RL
  Serial.println(F("Testing Rear Left (D9)..."));
  Serial.println(F("  Should spin for 2 seconds"));
  for (int speed = 1000; speed <= 1200; speed += 10) {
    motorRL.writeMicroseconds(speed);
    delay(20);
  }
  delay(1000);
  motorRL.writeMicroseconds(1000);
  delay(500);
  Serial.println(F("  ✓ Did it spin? (Y/N)"));
  Serial.println();
  
  // Test all together
  Serial.println(F("Testing ALL motors together..."));
  Serial.println(F("  All should spin for 2 seconds"));
  for (int speed = 1000; speed <= 1200; speed += 10) {
    motorFL.writeMicroseconds(speed);
    motorFR.writeMicroseconds(speed);
    motorRR.writeMicroseconds(speed);
    motorRL.writeMicroseconds(speed);
    delay(20);
  }
  delay(1000);
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorRR.writeMicroseconds(1000);
  motorRL.writeMicroseconds(1000);
  
  Serial.println(F("  ✓ Did all 4 motors spin? (Y/N)"));
  Serial.println();
}

// ====================================
// SUMMARY
// ====================================
void printSummary() {
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("DIAGNOSTIC SUMMARY"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println();
  Serial.println(F("Check the results above and note:"));
  Serial.println();
  Serial.println(F("BUZZER/LED:"));
  Serial.println(F("  - Did LED blink? _____"));
  Serial.println(F("  - Did buzzer beep? _____"));
  Serial.println();
  Serial.println(F("I2C BUS:"));
  Serial.println(F("  - Found MPU6050 at 0x68? _____"));
  Serial.println();
  Serial.println(F("MPU6050:"));
  Serial.println(F("  - Reading data? _____"));
  Serial.println(F("  - Data changes when moved? _____"));
  Serial.println();
  Serial.println(F("MOTORS (which ones spun?):"));
  Serial.println(F("  - Front Left (FL, D3)? _____"));
  Serial.println(F("  - Front Right (FR, D5)? _____"));
  Serial.println(F("  - Rear Right (RR, D6)? _____"));
  Serial.println(F("  - Rear Left (RL, D9)? _____"));
  Serial.println();
  Serial.println(F("NOW: Live MPU6050 monitoring below..."));
  Serial.println(F("     Tilt drone to see values change!"));
  Serial.println();
}

// ====================================
// LIVE MPU MONITORING
// ====================================
void readAndPrintMPU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  if (Wire.available() == 14) {
    int16_t accelX = Wire.read() << 8 | Wire.read();
    int16_t accelY = Wire.read() << 8 | Wire.read();
    int16_t accelZ = Wire.read() << 8 | Wire.read();
    int16_t temp = Wire.read() << 8 | Wire.read();
    int16_t gyroX = Wire.read() << 8 | Wire.read();
    int16_t gyroY = Wire.read() << 8 | Wire.read();
    int16_t gyroZ = Wire.read() << 8 | Wire.read();
    
    // Calculate angles
    float accelRoll = atan2(accelY, accelZ) * 57.2958;
    float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958;
    
    Serial.print(F("LIVE: Roll="));
    Serial.print(accelRoll, 1);
    Serial.print(F("° Pitch="));
    Serial.print(accelPitch, 1);
    Serial.print(F("° | Gyro X="));
    Serial.print(gyroX);
    Serial.print(F(" Y="));
    Serial.print(gyroY);
    Serial.print(F(" Z="));
    Serial.print(gyroZ);
    Serial.println(F("  ← TILT TO SEE CHANGE"));
  } else {
    Serial.println(F("✗ MPU6050 not responding!"));
  }
}
