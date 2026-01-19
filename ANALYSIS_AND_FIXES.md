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

| Parameter | Original | Fixed | Reason |
|-----------|----------|-------|--------|
| Motor mixing signs | Inconsistent | Corrected | Fix drift direction |
| TRIM values | 40/60/110/60 | 0/0/0/0 | Remove compensation |
| ESC_IDLE | 1300 | 1150 | Better low-throttle control |
| D_TERM_LPF_ALPHA | 0.2 | 0.15 | More filtering |
| OUTPUT_RATE_LIMIT | 30 | 20 | Smoother response |
| GYRO_LPF_ALPHA | 0.5 | 0.4 | More filtering |
| Axis inversions | Inconsistent | Unified | Correct stabilization |
