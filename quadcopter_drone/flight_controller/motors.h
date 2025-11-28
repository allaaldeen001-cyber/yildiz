/*
 * ============================================================================
 * MOTOR/ESC CONTROL
 * ============================================================================
 * ESC control using PWM signals
 * Motor mixing for X-configuration quadcopter
 * ============================================================================
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "config.h"

/*
 * Motor Layout (X-Configuration, view from above):
 *
 *         FRONT
 *           ^
 *     M4    |    M1
 *      \    |    /
 *       \   |   /
 *        \  |  /
 *         \ | /
 *          \|/
 *    <------+------> RIGHT
 *          /|\
 *         / | \
 *        /  |  \
 *       /   |   \
 *      /    |    \
 *     M3    |    M2
 *         REAR
 *
 * Motor directions:
 * M1 (Front-Right): Counter-Clockwise (CCW)
 * M2 (Rear-Right):  Clockwise (CW)
 * M3 (Rear-Left):   Counter-Clockwise (CCW)
 * M4 (Front-Left):  Clockwise (CW)
 *
 * Mixing:
 *       Throttle    Roll     Pitch     Yaw
 * M1:     +          -         +        +
 * M2:     +          -         -        -
 * M3:     +          +         -        +
 * M4:     +          +         +        -
 */

class MotorController {
public:
    // Constructor
    MotorController();
    
    // Initialization
    void init();
    void calibrateESCs();
    
    // Motor control
    void arm();
    void disarm();
    void emergencyStop();
    
    // Set motor values directly (1000-2000)
    void setMotor(uint8_t motor, uint16_t value);
    void setAllMotors(uint16_t value);
    
    // Motor mixing - converts throttle/pitch/roll/yaw to motor values
    void mix(uint16_t throttle, float roll, float pitch, float yaw);
    
    // Smooth motor test (gradual spin-up)
    void smoothTest(uint16_t targetSpeed, uint16_t duration);
    
    // Write values to ESCs
    void update();
    
    // Getters
    uint16_t getMotor1() { return motor1; }
    uint16_t getMotor2() { return motor2; }
    uint16_t getMotor3() { return motor3; }
    uint16_t getMotor4() { return motor4; }
    bool isArmed() { return armed; }
    
    // Status
    bool areMotorsRunning();

private:
    // Motor PWM values (1000-2000 microseconds)
    uint16_t motor1;  // Front-Right
    uint16_t motor2;  // Rear-Right
    uint16_t motor3;  // Rear-Left
    uint16_t motor4;  // Front-Left
    
    // Armed state
    bool armed;
    
    // ESC calibration flag
    bool calibrated;
    
    // Write PWM to a specific motor
    void writeMotor(uint8_t pin, uint16_t value);
    
    // Constrain motor value to safe range
    uint16_t constrainMotor(int32_t value);
};

// ============================================================================
// PWM Output using Timer1 and Timer2
// More precise than analogWrite for ESC control
// ============================================================================

class PWMOutput {
public:
    static void init();
    static void write(uint8_t pin, uint16_t microseconds);
    
private:
    static bool initialized;
};

#endif // MOTORS_H
