/*
 * STABILIZATION TEST - Verify MPU6050 Response
 * 
 * This will show you EXACTLY what the IMU detects and how motors respond
 * 
 * Upload this, ARM, then tilt the drone and watch Serial Monitor!
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

// IMU data
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float angleRoll, anglePitch;
float gyroRollCal, gyroPitchCal, gyroYawCal;

// PID outputs
float pidRoll = 0, pidPitch = 0;

// Motor speeds
int motorFLSpeed, motorFRSpeed, motorRRSpeed, motorRLSpeed;

// State
bool armed = false;
int baseThrottle = 1100;  // Base hover throttle for testing

// PID gains (same as flight controller)
#define PID_KP 1.4
#define PID_KD 18.0

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Serial.println(F("\n\n"));
  Serial.println(F("╔═══════════════════════════════════════╗"));
  Serial.println(F("║   STABILIZATION TEST TOOL             ║"));
  Serial.println(F("║   Verify IMU → Motor Response         ║"));
  Serial.println(F("╚═══════════════════════════════════════╝"));
  Serial.println();
  
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initialize motors
  motorFL.attach(MOTOR_FL);
  motorFR.attach(MOTOR_FR);
  motorRR.attach(MOTOR_RR);
  motorRL.attach(MOTOR_RL);
  
  motorFL.writeMicroseconds(1000);
  motorFR.writeMicroseconds(1000);
  motorRR.writeMicroseconds(1000);
  motorRL.writeMicroseconds(1000);
  
  // Initialize MPU6050
  Wire.begin();
  Wire.setClock(400000);
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  // Configure gyro and accel
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);  // ±500°/s
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);  // ±8g
  Wire.endTransmission();
  
  Serial.println(F("[OK] Motors and MPU6050 initialized"));
  Serial.println();
  Serial.println(F("CALIBRATING... Keep drone LEVEL and STILL!"));
  
  beep(1, 500);
  calibrate();
  beep(2, 100);
  
  Serial.println(F("[OK] Calibration complete!"));
  Serial.println();
  Serial.println(F("INSTRUCTIONS:"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println(F("1. Type 'A' to ARM (motors spin at 1100)"));
  Serial.println(F("2. Tilt drone and watch motor response"));
  Serial.println(F("3. Type 'D' to DISARM"));
  Serial.println(F("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"));
  Serial.println();
  Serial.println(F("⚠️  REMOVE PROPELLERS! ⚠️"));
  Serial.println();
}

void loop() {
  // Read IMU
  readIMU();
  calculateAngles();
  
  // Check for commands
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == 'A' || c == 'a') {
      armed = true;
      beep(1, 100);
      Serial.println(F("\n>>> ARMED - Motors at 1100, stabilization ON"));
      Serial.println(F(">>> Tilt drone now and watch the response!\n"));
    }
    
    if (c == 'D' || c == 'd') {
      armed = false;
      beep(2, 100);
      Serial.println(F("\n>>> DISARMED - Motors stopped\n"));
    }
  }
  
  if (armed) {
    // Calculate PID
    calculatePID();
    
    // Mix motors
    mixMotors();
    
    // Update motors
    motorFL.writeMicroseconds(motorFLSpeed);
    motorFR.writeMicroseconds(motorFRSpeed);
    motorRR.writeMicroseconds(motorRRSpeed);
    motorRL.writeMicroseconds(motorRLSpeed);
    
    // Print debug info
    printDebug();
    
  } else {
    // Disarmed - stop motors
    motorFL.writeMicroseconds(1000);
    motorFR.writeMicroseconds(1000);
    motorRR.writeMicroseconds(1000);
    motorRL.writeMicroseconds(1000);
  }
  
  delay(50);  // 20Hz for easy reading
}

void calibrate() {
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  
  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);
    
    gyroX = (Wire.read() << 8 | Wire.read());
    gyroY = (Wire.read() << 8 | Wire.read());
    gyroZ = (Wire.read() << 8 | Wire.read());
    
    gyroXSum += gyroX;
    gyroYSum += gyroY;
    gyroZSum += gyroZ;
    
    delay(3);
  }
  
  gyroRollCal = gyroXSum / 1000.0;
  gyroPitchCal = gyroYSum / 1000.0;
  gyroYawCal = gyroZSum / 1000.0;
}

void readIMU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  accelX = (Wire.read() << 8 | Wire.read());
  accelY = (Wire.read() << 8 | Wire.read());
  accelZ = (Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read();  // Skip temp
  gyroX = (Wire.read() << 8 | Wire.read());
  gyroY = (Wire.read() << 8 | Wire.read());
  gyroZ = (Wire.read() << 8 | Wire.read());
  
  // Apply calibration
  gyroX -= gyroRollCal;
  gyroY -= gyroPitchCal;
  gyroZ -= gyroYawCal;
}

void calculateAngles() {
  // Calculate angles from accelerometer
  float accelRoll = atan2(accelY, accelZ) * 57.2958;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958;
  
  // Complementary filter (simplified for testing)
  angleRoll = accelRoll;
  anglePitch = accelPitch;
}

void calculatePID() {
  // Simple PD controller for testing
  // Target angle = 0 (level)
  
  float rollError = 0 - angleRoll;
  float pitchError = 0 - anglePitch;
  
  float gyroRollRate = gyroX / 65.5;
  float gyroPitchRate = gyroY / 65.5;
  
  pidRoll = (PID_KP * rollError) + (PID_KD * (-gyroRollRate / 65.5));
  pidPitch = (PID_KP * pitchError) + (PID_KD * (-gyroPitchRate / 65.5));
  
  // Limit outputs
  pidRoll = constrain(pidRoll, -400, 400);
  pidPitch = constrain(pidPitch, -400, 400);
}

void mixMotors() {
  // X configuration motor mixing
  motorFLSpeed = baseThrottle - pidPitch + pidRoll;
  motorFRSpeed = baseThrottle - pidPitch - pidRoll;
  motorRRSpeed = baseThrottle + pidPitch - pidRoll;
  motorRLSpeed = baseThrottle + pidPitch + pidRoll;
  
  // Constrain
  motorFLSpeed = constrain(motorFLSpeed, 1000, 2000);
  motorFRSpeed = constrain(motorFRSpeed, 1000, 2000);
  motorRRSpeed = constrain(motorRRSpeed, 1000, 2000);
  motorRLSpeed = constrain(motorRLSpeed, 1000, 2000);
}

void printDebug() {
  static unsigned long lastPrint = 0;
  
  if (millis() - lastPrint > 200) {  // Print every 200ms
    Serial.print(F("TILT: "));
    
    // Show tilt direction
    if (anglePitch > 5) {
      Serial.print(F("NOSE UP   "));
    } else if (anglePitch < -5) {
      Serial.print(F("NOSE DOWN "));
    } else {
      Serial.print(F("LEVEL     "));
    }
    
    Serial.print(F(" | "));
    
    if (angleRoll > 5) {
      Serial.print(F("RIGHT "));
    } else if (angleRoll < -5) {
      Serial.print(F("LEFT  "));
    } else {
      Serial.print(F("LEVEL "));
    }
    
    // Show angles
    Serial.print(F(" | Ang P:"));
    Serial.print(anglePitch, 1);
    Serial.print(F("° R:"));
    Serial.print(angleRoll, 1);
    Serial.print(F("°"));
    
    // Show PID
    Serial.print(F(" | PID P:"));
    Serial.print((int)pidPitch);
    Serial.print(F(" R:"));
    Serial.print((int)pidRoll);
    
    // Show motors with visual indicators
    Serial.print(F(" | Motors:"));
    Serial.print(F(" FL:"));
    Serial.print(motorFLSpeed);
    if (motorFLSpeed > baseThrottle + 50) Serial.print(F("↑"));
    else if (motorFLSpeed < baseThrottle - 50) Serial.print(F("↓"));
    else Serial.print(F("═"));
    
    Serial.print(F(" FR:"));
    Serial.print(motorFRSpeed);
    if (motorFRSpeed > baseThrottle + 50) Serial.print(F("↑"));
    else if (motorFRSpeed < baseThrottle - 50) Serial.print(F("↓"));
    else Serial.print(F("═"));
    
    Serial.print(F(" RR:"));
    Serial.print(motorRRSpeed);
    if (motorRRSpeed > baseThrottle + 50) Serial.print(F("↑"));
    else if (motorRRSpeed < baseThrottle - 50) Serial.print(F("↓"));
    else Serial.print(F("═"));
    
    Serial.print(F(" RL:"));
    Serial.print(motorRLSpeed);
    if (motorRLSpeed > baseThrottle + 50) Serial.print(F("↑"));
    else if (motorRLSpeed < baseThrottle - 50) Serial.print(F("↓"));
    else Serial.print(F("═"));
    
    Serial.println();
    
    // Interpretation
    if (anglePitch < -10) {
      Serial.println(F("  → Nose DOWN detected. Rear should be FASTER (↑), Front SLOWER (↓)"));
    } else if (anglePitch > 10) {
      Serial.println(F("  → Nose UP detected. Front should be FASTER (↑), Rear SLOWER (↓)"));
    }
    
    if (angleRoll < -10) {
      Serial.println(F("  → Tilted LEFT. Right side should be FASTER (↑), Left SLOWER (↓)"));
    } else if (angleRoll > 10) {
      Serial.println(F("  → Tilted RIGHT. Left side should be FASTER (↑), Right SLOWER (↓)"));
    }
    
    Serial.println();
    lastPrint = millis();
  }
}

void beep(int count, int duration) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) delay(duration);
  }
}
