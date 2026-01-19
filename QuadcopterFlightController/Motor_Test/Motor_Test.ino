/*
  ===========================================
  MOTOR TEST TOOL
  ===========================================
  
  This tool tests each motor individually to verify:
  - Motor rotation direction
  - Motor responsiveness
  - ESC functionality
  
  WARNING: Remove all propellers before testing!
  
  Commands via Serial Monitor:
  - '1' = Test Motor 1 (Front-Left)
  - '2' = Test Motor 2 (Front-Right)
  - '3' = Test Motor 3 (Rear-Right)
  - '4' = Test Motor 4 (Rear-Left)
  - 'a' = Test ALL motors
  - 's' = STOP all motors
  - '+' = Increase test throttle
  - '-' = Decrease test throttle
  
  Expected rotation (X configuration):
  - M1 (FL) = Counter-Clockwise
  - M2 (FR) = Clockwise
  - M3 (RR) = Counter-Clockwise
  - M4 (RL) = Clockwise
*/

// Motor pins
#define MOTOR_FL 4   // M1 - Front-Left
#define MOTOR_FR 5   // M2 - Front-Right
#define MOTOR_RR 6   // M3 - Rear-Right
#define MOTOR_RL 7   // M4 - Rear-Left

#define LED_PIN 13

// ESC parameters
#define ESC_MIN 1000
#define ESC_MAX 2000
#define ESC_IDLE 1100

// Test throttle (adjustable)
int test_throttle = 1200;
int throttle_step = 50;

// Motor states
int motor1_speed = ESC_MIN;
int motor2_speed = ESC_MIN;
int motor3_speed = ESC_MIN;
int motor4_speed = ESC_MIN;

void setup() {
  Serial.begin(115200);
  
  Serial.println(F("==========================================="));
  Serial.println(F("         MOTOR TEST TOOL"));
  Serial.println(F("==========================================="));
  Serial.println();
  Serial.println(F("WARNING: REMOVE ALL PROPELLERS!"));
  Serial.println();
  
  // Configure pins
  pinMode(MOTOR_FL, OUTPUT);
  pinMode(MOTOR_FR, OUTPUT);
  pinMode(MOTOR_RR, OUTPUT);
  pinMode(MOTOR_RL, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize ESCs
  Serial.println(F("Initializing ESCs..."));
  for (int i = 0; i < 100; i++) {
    sendMotorSignals();
    delay(10);
  }
  Serial.println(F("ESCs initialized."));
  Serial.println();
  
  printMenu();
}

void loop() {
  // Handle serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    handleCommand(cmd);
  }
  
  // Continuously send motor signals
  sendMotorSignals();
}

void handleCommand(char cmd) {
  switch (cmd) {
    case '1':
      stopAllMotors();
      motor1_speed = test_throttle;
      Serial.println(F("Testing Motor 1 (Front-Left) - Should spin CCW"));
      break;
      
    case '2':
      stopAllMotors();
      motor2_speed = test_throttle;
      Serial.println(F("Testing Motor 2 (Front-Right) - Should spin CW"));
      break;
      
    case '3':
      stopAllMotors();
      motor3_speed = test_throttle;
      Serial.println(F("Testing Motor 3 (Rear-Right) - Should spin CCW"));
      break;
      
    case '4':
      stopAllMotors();
      motor4_speed = test_throttle;
      Serial.println(F("Testing Motor 4 (Rear-Left) - Should spin CW"));
      break;
      
    case 'a':
    case 'A':
      motor1_speed = test_throttle;
      motor2_speed = test_throttle;
      motor3_speed = test_throttle;
      motor4_speed = test_throttle;
      Serial.println(F("Testing ALL motors"));
      break;
      
    case 's':
    case 'S':
      stopAllMotors();
      Serial.println(F("All motors STOPPED"));
      break;
      
    case '+':
    case '=':
      test_throttle += throttle_step;
      if (test_throttle > 1600) test_throttle = 1600;  // Safety limit
      Serial.print(F("Test throttle: "));
      Serial.println(test_throttle);
      break;
      
    case '-':
    case '_':
      test_throttle -= throttle_step;
      if (test_throttle < ESC_IDLE) test_throttle = ESC_IDLE;
      Serial.print(F("Test throttle: "));
      Serial.println(test_throttle);
      break;
      
    case 'h':
    case 'H':
    case '?':
      printMenu();
      break;
      
    case 'r':
    case 'R':
      // Rotation test - sequence through all motors
      Serial.println(F("Rotation sequence test..."));
      rotationTest();
      break;
  }
}

void stopAllMotors() {
  motor1_speed = ESC_MIN;
  motor2_speed = ESC_MIN;
  motor3_speed = ESC_MIN;
  motor4_speed = ESC_MIN;
}

void rotationTest() {
  Serial.println(F("Motor 1 (Front-Left)..."));
  for (int i = 0; i < 100; i++) {
    motor1_speed = test_throttle;
    sendMotorSignals();
    delay(10);
  }
  stopAllMotors();
  delay(500);
  
  Serial.println(F("Motor 2 (Front-Right)..."));
  for (int i = 0; i < 100; i++) {
    motor2_speed = test_throttle;
    sendMotorSignals();
    delay(10);
  }
  stopAllMotors();
  delay(500);
  
  Serial.println(F("Motor 3 (Rear-Right)..."));
  for (int i = 0; i < 100; i++) {
    motor3_speed = test_throttle;
    sendMotorSignals();
    delay(10);
  }
  stopAllMotors();
  delay(500);
  
  Serial.println(F("Motor 4 (Rear-Left)..."));
  for (int i = 0; i < 100; i++) {
    motor4_speed = test_throttle;
    sendMotorSignals();
    delay(10);
  }
  stopAllMotors();
  
  Serial.println(F("Rotation test complete."));
}

void sendMotorSignals() {
  unsigned long start = micros();
  
  digitalWrite(MOTOR_FL, HIGH);
  digitalWrite(MOTOR_FR, HIGH);
  digitalWrite(MOTOR_RR, HIGH);
  digitalWrite(MOTOR_RL, HIGH);
  
  unsigned long m1_end = start + motor1_speed;
  unsigned long m2_end = start + motor2_speed;
  unsigned long m3_end = start + motor3_speed;
  unsigned long m4_end = start + motor4_speed;
  
  while (digitalRead(MOTOR_FL) || digitalRead(MOTOR_FR) || 
         digitalRead(MOTOR_RR) || digitalRead(MOTOR_RL)) {
    unsigned long now = micros();
    if (now >= m1_end) digitalWrite(MOTOR_FL, LOW);
    if (now >= m2_end) digitalWrite(MOTOR_FR, LOW);
    if (now >= m3_end) digitalWrite(MOTOR_RR, LOW);
    if (now >= m4_end) digitalWrite(MOTOR_RL, LOW);
  }
  
  // Wait to maintain ~50Hz update rate
  while (micros() - start < 4000);
}

void printMenu() {
  Serial.println(F("==========================================="));
  Serial.println(F("Commands:"));
  Serial.println(F("  1 = Test Motor 1 (Front-Left)"));
  Serial.println(F("  2 = Test Motor 2 (Front-Right)"));
  Serial.println(F("  3 = Test Motor 3 (Rear-Right)"));
  Serial.println(F("  4 = Test Motor 4 (Rear-Left)"));
  Serial.println(F("  a = Test ALL motors"));
  Serial.println(F("  s = STOP all motors"));
  Serial.println(F("  + = Increase throttle"));
  Serial.println(F("  - = Decrease throttle"));
  Serial.println(F("  r = Rotation sequence test"));
  Serial.println(F("  h = Show this menu"));
  Serial.println(F("==========================================="));
  Serial.print(F("Current test throttle: "));
  Serial.println(test_throttle);
  Serial.println();
}
