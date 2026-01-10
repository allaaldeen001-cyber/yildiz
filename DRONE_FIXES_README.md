# Quadcopter Flight Controller - Issues Found & Fixes Applied

## Summary of Analysis

I analyzed the drone flight controller code and found several **critical issues** that were causing poor auto-leveling and potential PWM timing problems.

---

## 🔴 CRITICAL ISSUE #1: PWM Timer Conflicts

### Problem
The original code used these motor pins:
- `PIN_MOTOR_FL = 3` (Timer 2) ✓
- `PIN_MOTOR_FR = 5` (Timer 0) ❌ **CONFLICT!**
- `PIN_MOTOR_RL = 6` (Timer 0) ❌ **CONFLICT!**
- `PIN_MOTOR_RR = 9` (Timer 1) ✓

**Pins 5 and 6 share Timer 0 with Arduino's `millis()` and `delay()` functions!**

When using the Servo library on Timer 0 pins, it can cause:
- Timing jitter in PWM signals
- Motor stuttering/twitching
- Unpredictable `millis()` behavior
- Unstable flight

### Fix Applied
Changed motor pins to avoid Timer 0:
```cpp
// FIXED pin assignments
#define PIN_MOTOR_FL        3     // Timer 2
#define PIN_MOTOR_FR        9     // Timer 1
#define PIN_MOTOR_RL        10    // Timer 1
#define PIN_MOTOR_RR        11    // Timer 2
```

Also changed RF CSN and Buzzer pins to accommodate the new motor pins:
```cpp
#define PIN_RF_CSN          8     // Changed from 10
#define PIN_BUZZER          2     // Changed from 8
```

---

## 🔴 CRITICAL ISSUE #2: Low-Pass Filter Applied BACKWARDS

### Problem
The original low-pass filter was implemented incorrectly:
```cpp
// ORIGINAL (WRONG)
float lowPassFilter(float current, float newValue, float alpha) {
    return alpha * current + (1.0f - alpha) * newValue;
}
```

With `GYRO_LPF_ALPHA = 0.7`, this means:
- 70% of OLD value retained
- 30% of NEW value applied

**This creates massive lag!** The sensor readings barely track actual movement.

### Fix Applied
```cpp
// FIXED - now alpha controls how fast we track NEW values
float lowPassFilter(float current, float newValue, float alpha) {
    return current + alpha * (newValue - current);
    // Equivalent to: (1-alpha)*current + alpha*newValue
}
```

And adjusted alpha values to be meaningful:
```cpp
#define GYRO_LPF_ALPHA      0.5f    // Was 0.7 (inverted!) 
#define ACCEL_LPF_ALPHA     0.3f    // Was 0.85 (inverted!)
#define MOTOR_LPF_ALPHA     0.4f    // Was 0.85 (inverted!)
```

---

## 🔴 CRITICAL ISSUE #3: Complementary Filter Too Gyro-Heavy

### Problem
```cpp
#define COMP_FILTER_ALPHA   0.996f  // 99.6% gyro, 0.4% accel
```

This means:
- Gyro is trusted 99.6%
- Accelerometer is only 0.4% influence
- Drift correction is **extremely slow**
- The drone will slowly drift off level and barely correct

### Fix Applied
```cpp
#define COMP_FILTER_ALPHA   0.98f   // 98% gyro, 2% accel
```

2% accelerometer influence provides:
- Still filters out vibrations
- But corrects drift 5x faster
- Better self-leveling response

---

## 🟡 ISSUE #4: Excessive Angle Smoothing (Double/Triple Filtering)

### Problem
The angles were being smoothed multiple times:
1. First: Gyro/Accel filtered with LPF
2. Second: Complementary filter
3. Third: Additional `rollSmooth`/`pitchSmooth` filter with `ANGLE_LPF_ALPHA = 0.9`

This created compounding lag - the drone sees movement way after it happens!

### Fix Applied
Removed the extra angle smoothing:
```cpp
#define ANGLE_LPF_ALPHA     0.0f    // DISABLED - was 0.9
```

The complementary filter output is already smooth enough.

---

## 🟡 ISSUE #5: Gyro Deadband Too Large

### Problem
```cpp
#define GYRO_DEADBAND       0.3f    // deg/sec
```

Any rotation under 0.3°/sec was ignored. This prevented the fine corrections needed for stable hover.

### Fix Applied
```cpp
#define GYRO_DEADBAND       0.1f    // Allow finer corrections
```

---

## 🟡 ISSUE #6: PID Gains Too Weak for Self-Leveling

### Problem
```cpp
#define PID_ROLL_KI         0.002f   // Way too weak!
#define PID_I_MAX           25.0f    // Too restrictive
```

The integral term was too weak to correct steady-state errors. The drone would hover at a slight angle forever.

### Fix Applied
```cpp
#define PID_ROLL_KP         1.8f    // Was 1.2
#define PID_ROLL_KI         0.015f  // Was 0.002 (7.5x increase)
#define PID_ROLL_KD         0.8f    // Was 0.5
#define PID_I_MAX           50.0f   // Was 25
```

---

## ✅ NEW FEATURE: Drone Direction Indicator

Added direction indication to the serial monitor output. The drone now shows its orientation:

### Visual Indicator Format: `DIR[pitch|roll|yaw]`
```
^  = Nose UP
v  = Nose DOWN
-  = Pitch level

<  = Tilted LEFT
>  = Tilted RIGHT
|  = Roll level

\  = Spinning LEFT (CCW)
/  = Spinning RIGHT (CW)
o  = No spin
```

### Example Output:
```
ARM DIR[v>o] G:1.0x R:8.2 P:-12.3 Yr:0.0 |PID R:45 P:-67 |M:1234,1156,1298,1189
```
This shows: Nose DOWN, tilting RIGHT, not spinning

### Threshold Settings:
```cpp
#define DIR_THRESHOLD_TILT  5.0f   // 5° to show tilt direction
#define DIR_THRESHOLD_YAW   20.0f  // 20°/sec to show spin
```

---

## Files Created

| File | Description |
|------|-------------|
| `drone_flight_controller.ino` | Original code (unchanged) |
| `remote_controller.ino` | Original remote code (unchanged) |
| `drone_flight_controller_fixed.ino` | **Fixed version with all improvements** |

---

## Hardware Wiring Changes Required

If using the fixed code, update your wiring:

| Component | Old Pin | New Pin |
|-----------|---------|---------|
| Motor FR ESC | D5 | **D9** |
| Motor RL ESC | D6 | **D10** |
| Motor RR ESC | D9 | **D11** |
| NRF24 CSN | D10 | **D8** |
| Buzzer | D8 | **D2** |

---

## Recommended Tuning Process

After flashing the fixed code:

1. **Verify calibration** - Place on flat surface, check serial output shows near 0° roll/pitch
2. **Test direction indicator** - Tilt drone by hand, verify correct direction shows
3. **Motor spin test** - Briefly arm with props OFF, verify all 4 motors spin
4. **Hover test** - With props, gentle hover to test self-leveling
5. **PID tuning** - If still oscillating or sluggish:
   - Oscillation → Reduce P and D
   - Sluggish leveling → Increase P and I
   - Overshooting → Increase D

---

## Summary of Changes

| Parameter | Original | Fixed | Why |
|-----------|----------|-------|-----|
| Motor Pins | 3,5,6,9 | 3,9,10,11 | Avoid Timer 0 |
| LPF Function | Inverted | Corrected | Was causing lag |
| COMP_FILTER_ALPHA | 0.996 | 0.98 | Better drift correction |
| ANGLE_LPF_ALPHA | 0.9 | 0.0 | Remove extra lag |
| GYRO_DEADBAND | 0.3 | 0.1 | Allow fine corrections |
| PID_ROLL_KI | 0.002 | 0.015 | Stronger leveling |
| PID_I_MAX | 25 | 50 | More correction authority |

