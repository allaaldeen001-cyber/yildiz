# Quadcopter Flight Controller - Bug Analysis and Fixes

## Critical Issues Found

### 1. **MOTOR MIXING SIGN ERROR** (Main cause of RR drift)

**Location:** `updateMotors()` function

**Original Code:**
```cpp
int16_t fl = baseThr + (int16_t)rollPID + (int16_t)pitchPID - (int16_t)yawPID;
int16_t fr = baseThr - (int16_t)rollPID + (int16_t)pitchPID + (int16_t)yawPID;
int16_t rl = baseThr + (int16_t)rollPID - (int16_t)pitchPID + (int16_t)yawPID;
int16_t rr = baseThr - (int16_t)rollPID - (int16_t)pitchPID - (int16_t)yawPID;
```

**Problem Analysis:**
- When the drone tilts right (positive roll angle), the PID output becomes negative
- With the original mixing: FL gets `+rollPID` (negative value) = decrease
- But FL should INCREASE to correct a right tilt!
- This causes the drone to amplify the error instead of correcting it
- Result: drift toward RR (right-rear)

**Fix:**
```cpp
// Roll: + = increase left motors (FL, RL), - = increase right motors (FR, RR)
// Pitch: + = increase rear motors (RL, RR), - = increase front motors (FL, FR)
int16_t fl = baseThr + rollMix - pitchMix - yawMix;
int16_t fr = baseThr - rollMix - pitchMix + yawMix;
int16_t rl = baseThr + rollMix + pitchMix + yawMix;
int16_t rr = baseThr - rollMix + pitchMix - yawMix;
```

### 2. **DOUBLE AXIS INVERSION**

**Location:** `updateAngles()` and `updatePID()`

**Problem:**
- Pitch rate was inverted in `updateAngles()` with `PITCH_INVERT`
- Then inverted again in `updatePID()` with `adjPitchRate = pitchRate * PITCH_INVERT`
- Double inversion = no inversion, breaking the control loop

**Fix:**
- Removed redundant inversions
- Created clear axis sign constants (`ROLL_SIGN`, `PITCH_SIGN`, `YAW_SIGN`)
- Apply signs consistently in `readIMU()` only

### 3. **MOTOR TRIM IMBALANCE**

**Original Values:**
```cpp
#define TRIM_FL +40
#define TRIM_FR +60
#define TRIM_RL +110  // Very high!
#define TRIM_RR +60
```

**Problem:**
- The large RL trim (+110) was likely compensating for the PID sign bug
- This masks the real problem and creates asymmetric thrust

**Fix:**
- Reset all trims to 0
- Created separate `ROLL_TRIM` and `PITCH_TRIM` for CG offset compensation
- Trims should be calibrated AFTER fixing PID issues

### 4. **ESC IDLE SPEED TOO HIGH**

**Original:** `ESC_IDLE = 1300`

**Problem:**
- High idle speed reduces control authority at low throttle
- Motors spin too fast when attempting gentle liftoff

**Fix:**
- Reduced to `ESC_IDLE = 1150`
- Added minimum throttle enforcement when armed for better control

### 5. **INTEGRAL ANTI-WINDUP ISSUES**

**Original:**
```cpp
if (abs(error) > 0.5f && angleKi > 0) {
    state.angleIntegral += error * dt;
    state.angleIntegral = constrainFloat(...);
}
```

**Problem:**
- Integral only accumulates above threshold
- No decay when near target
- Can cause slow oscillations due to accumulated integral

**Fix:**
```cpp
if (abs(error) > 0.3f && angleKi > 0) {
    state.angleIntegral += error * dt;
    state.angleIntegral = constrainFloat(...);
} else {
    state.angleIntegral *= 0.99f;  // Decay when near target
}
```

### 6. **D-TERM FILTERING INSUFFICIENT**

**Original:** `D_TERM_LPF_ALPHA = 0.2f`

**Problem:**
- High alpha means less filtering
- Gyro noise amplified by derivative term causing vibrations

**Fix:**
- Reduced to `D_TERM_LPF_ALPHA = 0.15f`
- Also reduced `OUTPUT_RATE_LIMIT` from 30 to 20 for smoother response

### 7. **GYRO BIAS SETTLING**

**Problem:**
- No check that gyro bias has settled before arming
- Rapid movements during startup could corrupt bias estimate

**Fix:**
- Added `biasSettleTime` check (5 seconds minimum)
- Slowed bias adaptation rate from 0.01 to 0.005
- Initialize PID state with current rates when arming

## Other Improvements

### Better Calibration
- Increased calibration passes from 4 to 6
- Extended calibration sample time
- More detailed calibration output

### Improved Timing Safety
```cpp
if (dt > 0.01f) dt = 0.002f;  // Reset if too large
if (dt < 0.0001f) dt = 0.002f;  // Reset if too small
```

### Enhanced Debug Output
- Added flight mode to debug output
- Added motor balance calculation (L-R and F-B sums)
- Cleaner formatting

### LED Indication
- Added fast blink pattern when no radio connected
- Helps diagnose connection issues

## Tuning Recommendations

### If Still Oscillating:
1. Reduce `RATE_ROLL_KP` (start at 0.3, work up)
2. Increase `RATE_ROLL_KD` (try 0.03-0.05)
3. Reduce `ANGLE_ROLL_KP` (try 2.0)

### If Still Drifting:
1. Check `ROLL_SIGN` and `PITCH_SIGN` match your MPU6050 orientation
2. Verify accelerometer calibration is accurate
3. Adjust `ROLL_TRIM` and `PITCH_TRIM` for CG offset

### Motor Sign Check Procedure:
1. Arm with throttle at minimum
2. Tilt drone right by hand
3. Left motors (FL, RL) should spin faster
4. If right motors spin faster, flip `ROLL_SIGN`

## Summary of Changes

| Parameter | Original | Fixed V1 | Fixed V2 | Reason |
|-----------|----------|----------|----------|--------|
| Motor mixing signs | Inconsistent | Corrected | - | Fix drift direction |
| TRIM values | 40/60/110/60 | 0/0/0/0 | - | Remove compensation |
| ESC_IDLE | 1300 | 1150 | - | Better low-throttle control |
| RC_ROLL_SIGN | -1.0 | 1.0 | **-1.0** | Fix inverted roll control |
| RC_YAW_SIGN | -1.0 | -1.0 | **1.0** | Fix inverted yaw control |
| ANGLE_ROLL_KP | 3.0 | 2.5 | **1.5** | Reduce shaking |
| RATE_ROLL_KP | 0.5 | 0.6 | **0.25** | Main cause of shaking |
| RATE_ROLL_KD | 0.015 | 0.025 | **0.008** | D amplifies noise |
| D_TERM_LPF_ALPHA | 0.2 | 0.15 | **0.08** | More filtering |
| GYRO_LPF_ALPHA | 0.5 | 0.4 | **0.3** | More filtering |
| MOTOR_LPF_ALPHA | 0.4 | 0.3 | **0.2** | Smoother motors |
| OUTPUT_RATE_LIMIT | 30 | 20 | **15** | Smoother response |
| All I gains | Various | Small | **0** | Start without I term |

---

## V2 Fixes (Shaking and Inverted Controls)

### Issue: Inverted Roll and Yaw Controls
- Stick left → drone went right
- Yaw stick right → drone rotated left

**Fix:** Flipped `RC_ROLL_SIGN` and `RC_YAW_SIGN`

### Issue: Drone Shaking
Shaking is caused by PID overcorrection. The control loop corrects too aggressively, overshoots, corrects back, overshoots again = oscillation/shaking.

**Fixes:**
1. Reduced `RATE_KP` from 0.6 to 0.25 (most important)
2. Reduced `RATE_KD` from 0.025 to 0.008 (D amplifies gyro noise)
3. Reduced `ANGLE_KP` from 2.5 to 1.5
4. Set all I gains to 0 (add later if needed)
5. Increased filtering on gyro, D-term, and motors

---

## V3 Fixes (Motor Mixing - RR Drift)

### Issue: Drone drifts toward RR (rear-right)

The motor mixing signs were **ALL WRONG** for all three axes.

### Root Cause Analysis

When the PID calculates a correction:
- **Tilted RIGHT** (positive roll) → error = 0 - positive = **negative** → rollPID = **negative**

With the OLD mixing:
```cpp
fl = baseThr + rollMix  // + negative = DECREASE FL
```

But FL is on the **LEFT** side and should **INCREASE** to correct a right tilt!

### The Fix

**OLD (WRONG) mixing:**
```cpp
fl = baseThr + rollMix - pitchMix - yawMix
fr = baseThr - rollMix - pitchMix + yawMix
rl = baseThr + rollMix + pitchMix + yawMix
rr = baseThr - rollMix + pitchMix - yawMix
```

**NEW (CORRECT) mixing:**
```cpp
fl = baseThr - rollMix + pitchMix + yawMix  // Left, Front, CCW
fr = baseThr + rollMix + pitchMix - yawMix  // Right, Front, CW
rl = baseThr - rollMix - pitchMix - yawMix  // Left, Rear, CW
rr = baseThr + rollMix - pitchMix + yawMix  // Right, Rear, CCW
```

### Verification Logic

| Condition | PID Output | Correct Response |
|-----------|------------|------------------|
| Tilt RIGHT | rollPID negative | FL,RL increase (left side up) |
| Nose DOWN | pitchPID positive | FL,FR increase (front up) |
| Yaw CW command | yawPID positive | FL,RR increase (CCW motors) |

### Testing Feature Added

When **disarmed**, the serial monitor now shows **simulated motor values**. 
Manually tilt the drone and verify:
- Tilt RIGHT → FL, RL values should increase
- Tilt FORWARD → RL, RR values should increase
- If opposite, check `ROLL_SIGN` and `PITCH_SIGN` constants
