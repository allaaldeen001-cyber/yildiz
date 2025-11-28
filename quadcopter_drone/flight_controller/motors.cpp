/*
 * ============================================================================
 * MOTOR/ESC CONTROL - IMPLEMENTATION
 * ============================================================================
 */

#include "motors.h"
#include <Servo.h>

// Servo objects for ESC control
static Servo escMotor1;
static Servo escMotor2;
static Servo escMotor3;
static Servo escMotor4;

// ============================================================================
// MotorController Implementation
// ============================================================================

MotorController::MotorController() {
    motor1 = motor2 = motor3 = motor4 = ESC_MIN_PULSE;
    armed = false;
    calibrated = false;
}

void MotorController::init() {
    // Attach ESC signals using Servo library
    // Servo library can output 1000-2000µs pulses at ~50Hz
    escMotor1.attach(MOTOR1_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    escMotor2.attach(MOTOR2_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    escMotor3.attach(MOTOR3_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    escMotor4.attach(MOTOR4_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    
    // Initialize all motors to minimum
    setAllMotors(ESC_MIN_PULSE);
    update();
    
    Serial.println(F("Motors initialized"));
}

void MotorController::calibrateESCs() {
    Serial.println(F(""));
    Serial.println(F("╔════════════════════════════════════════╗"));
    Serial.println(F("║        ESC CALIBRATION PROCEDURE       ║"));
    Serial.println(F("╠════════════════════════════════════════╣"));
    Serial.println(F("║ 1. Disconnect battery from ESCs        ║"));
    Serial.println(F("║ 2. This will now send MAX throttle     ║"));
    Serial.println(F("║ 3. Connect battery (ESCs will beep)    ║"));
    Serial.println(F("║ 4. Wait for calibration beeps          ║"));
    Serial.println(F("║ 5. MIN throttle will be sent           ║"));
    Serial.println(F("╚════════════════════════════════════════╝"));
    Serial.println(F(""));
    Serial.println(F("Sending MAX throttle signal..."));
    
    // Send maximum throttle
    setAllMotors(ESC_MAX_PULSE);
    update();
    
    // Wait for user to connect battery
    delay(7000);  // Give time to connect battery and hear beeps
    
    Serial.println(F("Sending MIN throttle signal..."));
    
    // Send minimum throttle
    setAllMotors(ESC_MIN_PULSE);
    update();
    
    delay(3000);  // Wait for ESCs to confirm
    
    calibrated = true;
    Serial.println(F("ESC calibration complete!"));
    Serial.println(F("You should have heard confirmation beeps."));
}

void MotorController::arm() {
    if (!armed) {
        Serial.println(F("ARMING MOTORS..."));
        
        // Send arming signal
        setAllMotors(ESC_ARM_PULSE);
        update();
        delay(100);
        
        armed = true;
        Serial.println(F("MOTORS ARMED!"));
    }
}

void MotorController::disarm() {
    Serial.println(F("DISARMING MOTORS..."));
    
    // Cut all motors
    setAllMotors(ESC_MIN_PULSE);
    update();
    
    armed = false;
    Serial.println(F("MOTORS DISARMED"));
}

void MotorController::emergencyStop() {
    // Immediately cut all motors - no gradual decrease
    motor1 = motor2 = motor3 = motor4 = ESC_MIN_PULSE;
    
    escMotor1.writeMicroseconds(ESC_MIN_PULSE);
    escMotor2.writeMicroseconds(ESC_MIN_PULSE);
    escMotor3.writeMicroseconds(ESC_MIN_PULSE);
    escMotor4.writeMicroseconds(ESC_MIN_PULSE);
    
    armed = false;
    
    Serial.println(F("!!! EMERGENCY STOP !!!"));
}

void MotorController::setMotor(uint8_t motor, uint16_t value) {
    value = constrain(value, ESC_MIN_PULSE, ESC_MAX_PULSE);
    
    switch (motor) {
        case 1: motor1 = value; break;
        case 2: motor2 = value; break;
        case 3: motor3 = value; break;
        case 4: motor4 = value; break;
    }
}

void MotorController::setAllMotors(uint16_t value) {
    value = constrain(value, ESC_MIN_PULSE, ESC_MAX_PULSE);
    motor1 = motor2 = motor3 = motor4 = value;
}

void MotorController::mix(uint16_t throttle, float roll, float pitch, float yaw) {
    // Only allow mixing if armed
    if (!armed) {
        setAllMotors(ESC_MIN_PULSE);
        return;
    }
    
    // If throttle is below deadzone, motors off
    if (throttle < THROTTLE_DEADZONE) {
        setAllMotors(ESC_MIN_PULSE);
        return;
    }
    
    /*
     * Motor Mixing (X-Configuration):
     *
     *       Throttle    Roll     Pitch     Yaw
     * M1:     +          -         +        +     (Front-Right, CCW)
     * M2:     +          -         -        -     (Rear-Right, CW)
     * M3:     +          +         -        +     (Rear-Left, CCW)
     * M4:     +          +         +        -     (Front-Left, CW)
     *
     * Roll: positive = roll right (left side up)
     * Pitch: positive = pitch forward (nose down)
     * Yaw: positive = rotate clockwise
     */
    
    int32_t m1 = throttle - roll + pitch + yaw;  // Front-Right
    int32_t m2 = throttle - roll - pitch - yaw;  // Rear-Right
    int32_t m3 = throttle + roll - pitch + yaw;  // Rear-Left
    int32_t m4 = throttle + roll + pitch - yaw;  // Front-Left
    
    // Constrain to valid ESC range
    motor1 = constrainMotor(m1);
    motor2 = constrainMotor(m2);
    motor3 = constrainMotor(m3);
    motor4 = constrainMotor(m4);
}

void MotorController::smoothTest(uint16_t targetSpeed, uint16_t duration) {
    Serial.println(F("Smooth motor test starting..."));
    
    if (!armed) {
        Serial.println(F("Cannot test - motors not armed!"));
        return;
    }
    
    targetSpeed = constrain(targetSpeed, ESC_MIN_PULSE, 1300);  // Limit for safety
    
    uint16_t startTime = millis();
    uint16_t currentSpeed = ESC_MIN_PULSE;
    uint16_t halfDuration = duration / 2;
    
    // Ramp up
    Serial.println(F("Ramping up..."));
    while (millis() - startTime < halfDuration) {
        float progress = (float)(millis() - startTime) / halfDuration;
        currentSpeed = ESC_MIN_PULSE + (targetSpeed - ESC_MIN_PULSE) * progress;
        setAllMotors(currentSpeed);
        update();
        delay(20);
    }
    
    // Hold at target
    setAllMotors(targetSpeed);
    update();
    delay(500);
    
    // Ramp down
    Serial.println(F("Ramping down..."));
    startTime = millis();
    while (millis() - startTime < halfDuration) {
        float progress = (float)(millis() - startTime) / halfDuration;
        currentSpeed = targetSpeed - (targetSpeed - ESC_MIN_PULSE) * progress;
        setAllMotors(currentSpeed);
        update();
        delay(20);
    }
    
    // Stop
    setAllMotors(ESC_MIN_PULSE);
    update();
    
    Serial.println(F("Motor test complete"));
}

void MotorController::update() {
    // Write current motor values to ESCs
    escMotor1.writeMicroseconds(motor1);
    escMotor2.writeMicroseconds(motor2);
    escMotor3.writeMicroseconds(motor3);
    escMotor4.writeMicroseconds(motor4);
}

bool MotorController::areMotorsRunning() {
    return armed && (motor1 > ESC_IDLE_PULSE || motor2 > ESC_IDLE_PULSE || 
                     motor3 > ESC_IDLE_PULSE || motor4 > ESC_IDLE_PULSE);
}

uint16_t MotorController::constrainMotor(int32_t value) {
    if (value < ESC_MIN_PULSE) return ESC_MIN_PULSE;
    if (value > ESC_MAX_PULSE) return ESC_MAX_PULSE;
    return (uint16_t)value;
}

// ============================================================================
// PWMOutput Implementation (alternative to Servo library)
// ============================================================================

bool PWMOutput::initialized = false;

void PWMOutput::init() {
    if (initialized) return;
    
    // Configure Timer1 for higher resolution PWM
    // This gives us better control over ESC timing
    
    // For now, we use the Servo library which is more compatible
    initialized = true;
}

void PWMOutput::write(uint8_t pin, uint16_t microseconds) {
    // This would use direct timer manipulation for more precise timing
    // For simplicity, we use Servo library in this implementation
}
