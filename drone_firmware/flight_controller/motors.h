/**
 * ============================================================================
 * MOTOR CONTROLLER AND MIXER
 * ============================================================================
 * 
 * Handles motor output generation with:
 * - X-configuration quadcopter mixing
 * - PWM output via Timer1 and Timer2
 * - Soft start/stop for safety
 * - Motor test sequences
 * 
 * Motor Layout (X-configuration, viewed from above):
 * 
 *        FRONT
 *     FL     FR
 *      \   /
 *       \ /
 *       /\
 *      /  \
 *     RL   RR
 *        REAR
 * 
 * FL (Front-Left):  CCW, +Roll, +Pitch, -Yaw
 * FR (Front-Right): CW,  -Roll, +Pitch, +Yaw
 * RR (Rear-Right):  CCW, -Roll, -Pitch, -Yaw
 * RL (Rear-Left):   CW,  +Roll, -Pitch, +Yaw
 * 
 * ============================================================================
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// MOTOR INDEX DEFINITIONS
// ============================================================================

#define MOTOR_FL    0   // Front-Left
#define MOTOR_FR    1   // Front-Right
#define MOTOR_RR    2   // Rear-Right
#define MOTOR_RL    3   // Rear-Left
#define NUM_MOTORS  4

// ============================================================================
// MOTOR CONTROLLER CLASS
// ============================================================================

class MotorController {
public:
    // Motor output values (1000-2000)
    uint16_t motorOutput[NUM_MOTORS];
    
    // Throttle command
    uint16_t throttleCommand;
    
    // State
    bool armed;
    bool motorsEnabled;
    
    /**
     * Initialize motor controller
     */
    void begin() {
        // Configure motor pins as outputs
        pinMode(PIN_MOTOR_FL, OUTPUT);
        pinMode(PIN_MOTOR_FR, OUTPUT);
        pinMode(PIN_MOTOR_RR, OUTPUT);
        pinMode(PIN_MOTOR_RL, OUTPUT);
        
        // Initialize outputs to minimum
        for (int i = 0; i < NUM_MOTORS; i++) {
            motorOutput[i] = MOTOR_MIN;
        }
        
        // Write initial values
        writeMotors();
        
        armed = false;
        motorsEnabled = false;
        throttleCommand = MOTOR_MIN;
        
        // Configure Timer1 for higher resolution PWM (pins 9, 10)
        // Note: Pin 10 used for NRF CSN, only pin 9 uses Timer1
        // Timer1: 16-bit, pins 9 & 10
        // We'll use standard analogWrite for simplicity
        
        // Configure Timer2 for pins 3 and 11
        // Using default settings for ESC compatibility
    }
    
    /**
     * Arm the motors
     */
    void arm() {
        if (!armed) {
            armed = true;
            motorsEnabled = true;
            
            // Soft start - briefly pulse all motors
            for (int i = 0; i < NUM_MOTORS; i++) {
                motorOutput[i] = MOTOR_IDLE;
            }
            writeMotors();
        }
    }
    
    /**
     * Disarm the motors
     */
    void disarm() {
        armed = false;
        motorsEnabled = false;
        
        // Immediately stop all motors
        for (int i = 0; i < NUM_MOTORS; i++) {
            motorOutput[i] = MOTOR_MIN;
        }
        writeMotors();
    }
    
    /**
     * Emergency stop - immediate motor cutoff
     */
    void emergencyStop() {
        armed = false;
        motorsEnabled = false;
        
        // Direct register writes for fastest response
        analogWrite(PIN_MOTOR_FL, 0);
        analogWrite(PIN_MOTOR_FR, 0);
        analogWrite(PIN_MOTOR_RR, 0);
        analogWrite(PIN_MOTOR_RL, 0);
        
        for (int i = 0; i < NUM_MOTORS; i++) {
            motorOutput[i] = MOTOR_MIN;
        }
    }
    
    /**
     * Mix control inputs to motor outputs
     * 
     * @param throttle  Throttle command (1000-2000)
     * @param roll      Roll control (-400 to +400)
     * @param pitch     Pitch control (-400 to +400)
     * @param yaw       Yaw control (-400 to +400)
     */
    void mix(uint16_t throttle, float roll, float pitch, float yaw) {
        throttleCommand = throttle;
        
        if (!armed || !motorsEnabled) {
            for (int i = 0; i < NUM_MOTORS; i++) {
                motorOutput[i] = MOTOR_MIN;
            }
            return;
        }
        
        // Apply throttle limit
        if (throttle > THROTTLE_LIMIT) {
            throttle = THROTTLE_LIMIT;
        }
        
        // Ensure minimum throttle when armed
        if (throttle < MOTOR_IDLE) {
            throttle = MOTOR_IDLE;
        }
        
        // X-configuration mixing
        // FL: +Roll, +Pitch, -Yaw (CCW)
        // FR: -Roll, +Pitch, +Yaw (CW)
        // RR: -Roll, -Pitch, -Yaw (CCW)
        // RL: +Roll, -Pitch, +Yaw (CW)
        
        float fl = throttle + roll + pitch - yaw;
        float fr = throttle - roll + pitch + yaw;
        float rr = throttle - roll - pitch - yaw;
        float rl = throttle + roll - pitch + yaw;
        
        // Find highest motor value for dynamic throttle limiting
        float maxMotor = max(max(fl, fr), max(rr, rl));
        float minMotor = min(min(fl, fr), min(rr, rl));
        
        // Scale down if any motor exceeds maximum
        if (maxMotor > MOTOR_MAX) {
            float scale = (float)(MOTOR_MAX - throttle) / (maxMotor - throttle);
            fl = throttle + (fl - throttle) * scale;
            fr = throttle + (fr - throttle) * scale;
            rr = throttle + (rr - throttle) * scale;
            rl = throttle + (rl - throttle) * scale;
        }
        
        // Ensure minimum motor value
        motorOutput[MOTOR_FL] = constrain((uint16_t)fl, MOTOR_IDLE, MOTOR_MAX);
        motorOutput[MOTOR_FR] = constrain((uint16_t)fr, MOTOR_IDLE, MOTOR_MAX);
        motorOutput[MOTOR_RR] = constrain((uint16_t)rr, MOTOR_IDLE, MOTOR_MAX);
        motorOutput[MOTOR_RL] = constrain((uint16_t)rl, MOTOR_IDLE, MOTOR_MAX);
    }
    
    /**
     * Write motor outputs to ESCs
     */
    void writeMotors() {
        // Convert 1000-2000 to analogWrite range (0-255)
        // Standard ESC expects ~1000-2000µs pulses
        // Using analogWrite with appropriate scaling
        
        if (motorsEnabled && armed) {
            // Map 1000-2000 to ~125-250 for ~500Hz PWM
            analogWrite(PIN_MOTOR_FL, mapMotorToPWM(motorOutput[MOTOR_FL]));
            analogWrite(PIN_MOTOR_FR, mapMotorToPWM(motorOutput[MOTOR_FR]));
            analogWrite(PIN_MOTOR_RR, mapMotorToPWM(motorOutput[MOTOR_RR]));
            analogWrite(PIN_MOTOR_RL, mapMotorToPWM(motorOutput[MOTOR_RL]));
        } else {
            analogWrite(PIN_MOTOR_FL, mapMotorToPWM(MOTOR_MIN));
            analogWrite(PIN_MOTOR_FR, mapMotorToPWM(MOTOR_MIN));
            analogWrite(PIN_MOTOR_RR, mapMotorToPWM(MOTOR_MIN));
            analogWrite(PIN_MOTOR_RL, mapMotorToPWM(MOTOR_MIN));
        }
    }
    
    /**
     * ESC calibration sequence
     * Run with props removed!
     */
    void calibrateESC(void (*buzzerCallback)(int)) {
        // Step 1: Send maximum throttle
        for (int i = 0; i < NUM_MOTORS; i++) {
            motorOutput[i] = MOTOR_MAX;
        }
        writeMotorsRaw();
        buzzerCallback(1);  // Beep to indicate max throttle
        delay(3000);
        
        // Step 2: Send minimum throttle
        for (int i = 0; i < NUM_MOTORS; i++) {
            motorOutput[i] = MOTOR_MIN;
        }
        writeMotorsRaw();
        buzzerCallback(2);  // Two beeps to indicate min throttle
        delay(2000);
        
        // Step 3: Wait for ESC confirmation
        delay(3000);
        
        buzzerCallback(3);  // Three beeps to indicate completion
    }
    
    /**
     * Motor test sequence
     * Spins each motor individually
     */
    void motorTest(void (*buzzerCallback)(int), uint16_t testThrottle = 1200) {
        // Ensure safe test throttle
        testThrottle = constrain(testThrottle, MOTOR_MIN, MOTOR_IDLE + 200);
        
        // Test each motor
        for (int motor = 0; motor < NUM_MOTORS; motor++) {
            // All motors off
            for (int i = 0; i < NUM_MOTORS; i++) {
                motorOutput[i] = MOTOR_MIN;
            }
            writeMotorsRaw();
            delay(500);
            
            // Beep to indicate which motor
            buzzerCallback(motor + 1);
            delay(500);
            
            // Ramp up the motor
            for (uint16_t pwm = MOTOR_MIN; pwm <= testThrottle; pwm += 10) {
                motorOutput[motor] = pwm;
                writeMotorsRaw();
                delay(20);
            }
            
            // Hold for observation
            delay(1000);
            
            // Ramp down
            for (uint16_t pwm = testThrottle; pwm >= MOTOR_MIN; pwm -= 10) {
                motorOutput[motor] = pwm;
                writeMotorsRaw();
                delay(20);
            }
            
            motorOutput[motor] = MOTOR_MIN;
            writeMotorsRaw();
            delay(500);
        }
        
        // All done
        buzzerCallback(5);
    }
    
    /**
     * Get motor output as 0-255 for telemetry
     */
    uint8_t getMotorPercent(uint8_t motor) {
        if (motor >= NUM_MOTORS) return 0;
        return map(motorOutput[motor], MOTOR_MIN, MOTOR_MAX, 0, 255);
    }
    
private:
    /**
     * Map motor value (1000-2000) to PWM duty cycle (0-255)
     */
    uint8_t mapMotorToPWM(uint16_t motorValue) {
        // Standard ESC calibration: 1000µs = off, 2000µs = full
        // With ~490Hz PWM (2040µs period), we need:
        // 1000µs = ~49% duty = 125
        // 2000µs = ~98% duty = 250
        return map(motorValue, MOTOR_MIN, MOTOR_MAX, 125, 250);
    }
    
    /**
     * Write motors directly (for calibration/test)
     */
    void writeMotorsRaw() {
        analogWrite(PIN_MOTOR_FL, mapMotorToPWM(motorOutput[MOTOR_FL]));
        analogWrite(PIN_MOTOR_FR, mapMotorToPWM(motorOutput[MOTOR_FR]));
        analogWrite(PIN_MOTOR_RR, mapMotorToPWM(motorOutput[MOTOR_RR]));
        analogWrite(PIN_MOTOR_RL, mapMotorToPWM(motorOutput[MOTOR_RL]));
    }
};

#endif // MOTORS_H
