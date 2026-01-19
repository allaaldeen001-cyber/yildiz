/*
  ===========================================
  ESC CALIBRATION TOOL
  ===========================================
  
  This tool calibrates all 4 ESCs at once to ensure
  they respond identically to throttle commands.
  
  IMPORTANT: Remove propellers before calibration!
  
  Instructions:
  1. Connect battery with throttle stick at MAXIMUM
  2. ESCs will beep to confirm high throttle
  3. Move throttle stick to MINIMUM
  4. ESCs will beep to confirm low throttle
  5. Calibration complete!
  
  Motor Pins:
  - M1 (FL) = Pin 4
  - M2 (FR) = Pin 5
  - M3 (RR) = Pin 6
  - M4 (RL) = Pin 7
  
  Throttle Input: Pin 10
*/

// Motor pins
#define MOTOR_FL 4
#define MOTOR_FR 5
#define MOTOR_RR 6
#define MOTOR_RL 7

// Throttle input pin
#define THROTTLE_PIN 10

// Status LED
#define LED_PIN 13

// ESC pulse widths
#define ESC_MIN 1000
#define ESC_MAX 2000

void setup() {
  Serial.begin(115200);
  Serial.println(F("==================================="));
  Serial.println(F("      ESC CALIBRATION TOOL"));
  Serial.println(F("==================================="));
  Serial.println();
  Serial.println(F("WARNING: Remove all propellers!"));
  Serial.println();
  Serial.println(F("Instructions:"));
  Serial.println(F("1. Disconnect battery"));
  Serial.println(F("2. Move throttle to MAXIMUM"));
  Serial.println(F("3. Connect battery"));
  Serial.println(F("4. Wait for ESC beeps"));
  Serial.println(F("5. Move throttle to MINIMUM"));
  Serial.println(F("6. Wait for confirmation beeps"));
  Serial.println();
  
  // Configure pins
  pinMode(MOTOR_FL, OUTPUT);
  pinMode(MOTOR_FR, OUTPUT);
  pinMode(MOTOR_RR, OUTPUT);
  pinMode(MOTOR_RL, OUTPUT);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  Serial.println(F("Reading throttle input..."));
  Serial.println(F("Move throttle stick to see values"));
  Serial.println();
}

void loop() {
  // Read throttle from receiver
  int throttle = pulseIn(THROTTLE_PIN, HIGH, 30000);
  
  // Validate throttle reading
  if (throttle < 900 || throttle > 2100) {
    throttle = ESC_MIN;  // Default to minimum if no signal
  }
  
  // Pass throttle directly to all ESCs
  sendToAllMotors(throttle);
  
  // Print throttle value for debugging
  static unsigned long last_print = 0;
  if (millis() - last_print > 200) {
    Serial.print(F("Throttle: "));
    Serial.print(throttle);
    
    if (throttle > 1900) {
      Serial.println(F(" <- HIGH (connect battery now)"));
      digitalWrite(LED_PIN, HIGH);
    } else if (throttle < 1100) {
      Serial.println(F(" <- LOW (calibration position)"));
      digitalWrite(LED_PIN, LOW);
    } else {
      Serial.println();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Blink
    }
    
    last_print = millis();
  }
}

void sendToAllMotors(int pulseWidth) {
  // Send identical pulse to all motors simultaneously
  unsigned long start = micros();
  
  digitalWrite(MOTOR_FL, HIGH);
  digitalWrite(MOTOR_FR, HIGH);
  digitalWrite(MOTOR_RR, HIGH);
  digitalWrite(MOTOR_RL, HIGH);
  
  while (micros() - start < pulseWidth);
  
  digitalWrite(MOTOR_FL, LOW);
  digitalWrite(MOTOR_FR, LOW);
  digitalWrite(MOTOR_RR, LOW);
  digitalWrite(MOTOR_RL, LOW);
  
  // Wait for next cycle (50Hz typical ESC update rate)
  while (micros() - start < 20000);
}
