/**
 * ============================================================================
 * BUZZER CONTROLLER
 * ============================================================================
 * 
 * Non-blocking buzzer control for status indication
 * 
 * ============================================================================
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>
#include "config.h"

// ============================================================================
// BUZZER PATTERNS
// ============================================================================

#define PATTERN_NONE        0
#define PATTERN_LINK_OK     1   // Short double beep
#define PATTERN_CALIB_OK    2   // Two short beeps
#define PATTERN_CALIB_FAIL  3   // One long 7-second beep
#define PATTERN_ARM         4   // Rising tone
#define PATTERN_DISARM      5   // Falling tone
#define PATTERN_LOW_BATT    6   // Rapid beeping
#define PATTERN_FAILSAFE    7   // Continuous alarm
#define PATTERN_MOTOR_TEST  8   // Motor number beeps

// ============================================================================
// BUZZER CONTROLLER CLASS
// ============================================================================

class BuzzerController {
public:
    /**
     * Initialize buzzer
     */
    void begin() {
        pinMode(PIN_BUZZER, OUTPUT);
        digitalWrite(PIN_BUZZER, LOW);
        
        currentPattern = PATTERN_NONE;
        patternStep = 0;
        stepStartTime = 0;
        isOn = false;
    }
    
    /**
     * Update buzzer state (call in main loop)
     */
    void update() {
        if (currentPattern == PATTERN_NONE) {
            return;
        }
        
        uint32_t now = millis();
        uint32_t elapsed = now - stepStartTime;
        
        switch (currentPattern) {
            case PATTERN_LINK_OK:
                playLinkOK(elapsed);
                break;
                
            case PATTERN_CALIB_OK:
                playCalibOK(elapsed);
                break;
                
            case PATTERN_CALIB_FAIL:
                playCalibFail(elapsed);
                break;
                
            case PATTERN_ARM:
                playArm(elapsed);
                break;
                
            case PATTERN_DISARM:
                playDisarm(elapsed);
                break;
                
            case PATTERN_LOW_BATT:
                playLowBatt(elapsed);
                break;
                
            case PATTERN_FAILSAFE:
                playFailsafe(elapsed);
                break;
                
            case PATTERN_MOTOR_TEST:
                playMotorTest(elapsed);
                break;
        }
    }
    
    /**
     * Play a pattern
     */
    void play(uint8_t pattern) {
        currentPattern = pattern;
        patternStep = 0;
        stepStartTime = millis();
        repeatCount = 0;
        
        // Start with buzzer on for most patterns
        if (pattern != PATTERN_NONE) {
            buzzerOn();
        }
    }
    
    /**
     * Play motor test beeps (1-5 beeps)
     */
    void playMotorBeeps(uint8_t count) {
        motorBeepCount = count;
        play(PATTERN_MOTOR_TEST);
    }
    
    /**
     * Stop any playing pattern
     */
    void stop() {
        currentPattern = PATTERN_NONE;
        buzzerOff();
    }
    
    /**
     * Check if a pattern is playing
     */
    bool isPlaying() const {
        return currentPattern != PATTERN_NONE;
    }
    
    /**
     * Simple blocking beep (for initialization)
     */
    void beep(uint16_t durationMs) {
        buzzerOn();
        delay(durationMs);
        buzzerOff();
    }
    
    /**
     * Multiple blocking beeps
     */
    void beeps(uint8_t count, uint16_t onMs = 100, uint16_t offMs = 100) {
        for (uint8_t i = 0; i < count; i++) {
            beep(onMs);
            if (i < count - 1) {
                delay(offMs);
            }
        }
    }
    
private:
    uint8_t currentPattern;
    uint8_t patternStep;
    uint32_t stepStartTime;
    bool isOn;
    uint8_t repeatCount;
    uint8_t motorBeepCount;
    
    void buzzerOn() {
        digitalWrite(PIN_BUZZER, HIGH);
        isOn = true;
    }
    
    void buzzerOff() {
        digitalWrite(PIN_BUZZER, LOW);
        isOn = false;
    }
    
    void nextStep() {
        patternStep++;
        stepStartTime = millis();
    }
    
    // Pattern: Short double beep for link OK
    void playLinkOK(uint32_t elapsed) {
        switch (patternStep) {
            case 0:  // First beep
                if (elapsed >= 50) { buzzerOff(); nextStep(); }
                break;
            case 1:  // Gap
                if (elapsed >= 50) { buzzerOn(); nextStep(); }
                break;
            case 2:  // Second beep
                if (elapsed >= 50) { stop(); }
                break;
        }
    }
    
    // Pattern: Two beeps for calibration OK
    void playCalibOK(uint32_t elapsed) {
        switch (patternStep) {
            case 0:  // First beep
                if (elapsed >= 200) { buzzerOff(); nextStep(); }
                break;
            case 1:  // Gap
                if (elapsed >= 200) { buzzerOn(); nextStep(); }
                break;
            case 2:  // Second beep
                if (elapsed >= 200) { stop(); }
                break;
        }
    }
    
    // Pattern: Long 7-second beep for calibration fail
    void playCalibFail(uint32_t elapsed) {
        if (elapsed >= 7000) {
            stop();
        }
    }
    
    // Pattern: Rising tone for arm
    void playArm(uint32_t elapsed) {
        switch (patternStep) {
            case 0:
                if (elapsed >= 100) { buzzerOff(); nextStep(); }
                break;
            case 1:
                if (elapsed >= 50) { buzzerOn(); nextStep(); }
                break;
            case 2:
                if (elapsed >= 100) { buzzerOff(); nextStep(); }
                break;
            case 3:
                if (elapsed >= 50) { buzzerOn(); nextStep(); }
                break;
            case 4:
                if (elapsed >= 200) { stop(); }
                break;
        }
    }
    
    // Pattern: Falling tone for disarm
    void playDisarm(uint32_t elapsed) {
        switch (patternStep) {
            case 0:
                if (elapsed >= 200) { buzzerOff(); nextStep(); }
                break;
            case 1:
                if (elapsed >= 50) { buzzerOn(); nextStep(); }
                break;
            case 2:
                if (elapsed >= 100) { stop(); }
                break;
        }
    }
    
    // Pattern: Rapid beeping for low battery
    void playLowBatt(uint32_t elapsed) {
        if (elapsed >= 100) {
            if (isOn) {
                buzzerOff();
            } else {
                buzzerOn();
            }
            stepStartTime = millis();
            repeatCount++;
            
            if (repeatCount >= 10) {
                stop();
            }
        }
    }
    
    // Pattern: Continuous alternating for failsafe
    void playFailsafe(uint32_t elapsed) {
        if (elapsed >= 200) {
            if (isOn) {
                buzzerOff();
            } else {
                buzzerOn();
            }
            stepStartTime = millis();
        }
        // Failsafe pattern continues until explicitly stopped
    }
    
    // Pattern: Motor test beeps
    void playMotorTest(uint32_t elapsed) {
        if (patternStep < motorBeepCount * 2) {
            if (patternStep % 2 == 0) {
                // Beep on
                if (elapsed >= 150) {
                    buzzerOff();
                    nextStep();
                }
            } else {
                // Gap
                if (elapsed >= 100) {
                    buzzerOn();
                    nextStep();
                }
            }
        } else {
            stop();
        }
    }
};

#endif // BUZZER_H
