/**
 * ============================================================================
 *          QUADCOPTER MOTOR TEST UTILITY
 * ============================================================================
 * 
 * USE THIS TO:
 *   1. Test individual motors
 *   2. Find motor speed differences
 *   3. Verify motor/ESC health
 *   4. Calculate correct trim values
 *   5. Verify motor direction (CW/CCW)
 * 
 * SAFETY:
 *   - REMOVE PROPELLERS before testing!
 *   - Secure the drone
 *   - Keep hands away from motors
 * 
 * MOTOR LAYOUT (X-configuration):
 *        FRONT
 *   FL(CCW)  FR(CW)
 *       X
 *   RL(CW)   RR(CCW)
 * 
 * ============================================================================
 */

#include <Servo.h>

// Pin definitions - match your flight controller
#define PIN_MOTOR_FL        3
#define PIN_MOTOR_FR        5
#define PIN_MOTOR_RL        6
#define PIN_MOTOR_RR        9

#define ESC_MIN             1000
#define ESC_MAX             2000

Servo escFL, escFR, escRL, escRR;

int currentTest = 0;
int testSpeed = 1200;  // Start at low speed
bool testRunning = false;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println(F("\n=========================================="));
    Serial.println(F("     MOTOR TEST UTILITY"));
    Serial.println(F("=========================================="));
    Serial.println(F("\n*** REMOVE ALL PROPELLERS! ***\n"));
    
    // Attach ESCs
    escFL.attach(PIN_MOTOR_FL, ESC_MIN, ESC_MAX);
    escFR.attach(PIN_MOTOR_FR, ESC_MIN, ESC_MAX);
    escRL.attach(PIN_MOTOR_RL, ESC_MIN, ESC_MAX);
    escRR.attach(PIN_MOTOR_RR, ESC_MIN, ESC_MAX);
    
    // Initialize to minimum
    stopAllMotors();
    
    Serial.println(F("ESCs initialized. Waiting for arming..."));
    Serial.println(F("(ESCs may need 2-3 seconds to arm)\n"));
    delay(3000);
    
    printMenu();
}

void stopAllMotors() {
    escFL.writeMicroseconds(ESC_MIN);
    escFR.writeMicroseconds(ESC_MIN);
    escRL.writeMicroseconds(ESC_MIN);
    escRR.writeMicroseconds(ESC_MIN);
    testRunning = false;
}

void printMenu() {
    Serial.println(F("\n--- MOTOR TEST MENU ---"));
    Serial.println(F("Commands:"));
    Serial.println(F("  1 = Test FL (Front-Left, CCW)"));
    Serial.println(F("  2 = Test FR (Front-Right, CW)"));
    Serial.println(F("  3 = Test RL (Rear-Left, CW)"));
    Serial.println(F("  4 = Test RR (Rear-Right, CCW)"));
    Serial.println(F("  5 = Test ALL motors"));
    Serial.println(F("  6 = Speed comparison test"));
    Serial.println(F("  0 = STOP all motors"));
    Serial.println(F("  + = Increase speed"));
    Serial.println(F("  - = Decrease speed"));
    Serial.print(F("\nCurrent speed: ")); Serial.println(testSpeed);
    Serial.println();
}

void setMotor(int motor, int speed) {
    switch (motor) {
        case 1: escFL.writeMicroseconds(speed); break;
        case 2: escFR.writeMicroseconds(speed); break;
        case 3: escRL.writeMicroseconds(speed); break;
        case 4: escRR.writeMicroseconds(speed); break;
    }
}

void testSingleMotor(int motor) {
    stopAllMotors();
    delay(100);
    
    const char* names[] = {"", "FL (Front-Left, CCW)", "FR (Front-Right, CW)", 
                           "RL (Rear-Left, CW)", "RR (Rear-Right, CCW)"};
    
    Serial.print(F("\nTesting: ")); Serial.println(names[motor]);
    Serial.print(F("Speed: ")); Serial.println(testSpeed);
    Serial.println(F("Verify:"));
    Serial.println(F("  - Motor spins"));
    Serial.println(F("  - Direction is correct (CW/CCW)"));
    Serial.println(F("  - No unusual vibration or noise"));
    Serial.println(F("Press 0 to stop, +/- to adjust speed\n"));
    
    setMotor(motor, testSpeed);
    currentTest = motor;
    testRunning = true;
}

void testAllMotors() {
    Serial.println(F("\nTesting ALL motors at same speed"));
    Serial.print(F("Speed: ")); Serial.println(testSpeed);
    Serial.println(F("Listen for any motor that sounds different!"));
    Serial.println(F("Press 0 to stop\n"));
    
    escFL.writeMicroseconds(testSpeed);
    escFR.writeMicroseconds(testSpeed);
    escRL.writeMicroseconds(testSpeed);
    escRR.writeMicroseconds(testSpeed);
    
    currentTest = 5;
    testRunning = true;
}

void runSpeedComparisonTest() {
    Serial.println(F("\n=== SPEED COMPARISON TEST ==="));
    Serial.println(F("This test runs each motor for 3 seconds."));
    Serial.println(F("Listen and compare the sound/speed.\n"));
    
    const char* names[] = {"FL", "FR", "RL", "RR"};
    Servo* motors[] = {&escFL, &escFR, &escRL, &escRR};
    
    for (int i = 0; i < 4; i++) {
        Serial.print(F("Testing ")); Serial.print(names[i]);
        Serial.print(F(" at ")); Serial.print(testSpeed); Serial.println(F("us..."));
        
        motors[i]->writeMicroseconds(testSpeed);
        delay(3000);
        motors[i]->writeMicroseconds(ESC_MIN);
        delay(500);
    }
    
    Serial.println(F("\n=== TEST COMPLETE ==="));
    Serial.println(F("Which motor seemed slowest?"));
    Serial.println(F("Increase TRIM for that motor in the FC code."));
    Serial.println(F("Typical increment: +20 to +50\n"));
    
    printMenu();
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case '0':
                stopAllMotors();
                Serial.println(F("*** ALL MOTORS STOPPED ***"));
                printMenu();
                break;
                
            case '1':
            case '2':
            case '3':
            case '4':
                testSingleMotor(cmd - '0');
                break;
                
            case '5':
                testAllMotors();
                break;
                
            case '6':
                stopAllMotors();
                runSpeedComparisonTest();
                break;
                
            case '+':
            case '=':
                testSpeed += 50;
                if (testSpeed > 1800) testSpeed = 1800;
                Serial.print(F("Speed: ")); Serial.println(testSpeed);
                if (testRunning && currentTest <= 4) {
                    setMotor(currentTest, testSpeed);
                } else if (testRunning && currentTest == 5) {
                    testAllMotors();
                }
                break;
                
            case '-':
            case '_':
                testSpeed -= 50;
                if (testSpeed < 1050) testSpeed = 1050;
                Serial.print(F("Speed: ")); Serial.println(testSpeed);
                if (testRunning && currentTest <= 4) {
                    setMotor(currentTest, testSpeed);
                } else if (testRunning && currentTest == 5) {
                    testAllMotors();
                }
                break;
                
            case 'h':
            case 'H':
            case '?':
                printMenu();
                break;
        }
    }
    
    // Safety timeout - stop after 30 seconds
    static uint32_t lastActivity = 0;
    if (testRunning) {
        if (millis() - lastActivity > 30000) {
            stopAllMotors();
            Serial.println(F("\n*** SAFETY TIMEOUT - Motors stopped ***"));
            printMenu();
        }
    } else {
        lastActivity = millis();
    }
}
