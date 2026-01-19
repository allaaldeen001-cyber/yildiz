# Quadcopter Flight Controller Tuning Guide

## Issues Fixed in This Version

### 1. RR Motor Drift / Slow RR Motor
**Problem:** Drone drifts toward RR direction, RR motor stays on ground while others lift.

**Root Causes Found:**
- Motor mixing had **inverted roll signs** - when trying to correct roll, it made it worse
- RR motor trim was too low (+60, now increased to +120)
- No motor scaling compensation

**Fixes Applied:**
```cpp
// CORRECTED motor mixing (was wrong before):
int16_t fl = baseThr - rollPID + pitchPID + yawPID;  // Changed sign
int16_t fr = baseThr + rollPID + pitchPID - yawPID;  // Changed sign
int16_t rl = baseThr - rollPID - pitchPID - yawPID;  // Changed sign
int16_t rr = baseThr + rollPID - pitchPID + yawPID;  // Changed sign

// Increased RR trim
#define TRIM_RR  +120  // Was +60

// Added motor scaling
#define MOTOR_SCALE_RR  1.05f  // 5% boost
```

**If Still Drifting:**
1. Increase `TRIM_RR` further (try +150, +180)
2. Increase `MOTOR_SCALE_RR` (try 1.10f, 1.15f)
3. Check RR motor/ESC/propeller for mechanical issues

---

### 2. Oscillation Problems
**Problem:** Drone oscillates/wobbles during flight.

**Root Causes:**
- P gains too high causing overcorrection
- D-term reacting to setpoint changes (derivative kick)
- Insufficient filtering

**Fixes Applied:**
```cpp
// Reduced P gains
#define RATE_ROLL_KP   0.35f   // Was 0.5
#define ANGLE_ROLL_KP  2.5f    // Was 3.0

// Increased D for damping
#define RATE_ROLL_KD   0.025f  // Was 0.015

// More aggressive filtering
#define SETPOINT_LPF_ALPHA  0.2f   // Was 0.3
#define D_TERM_LPF_ALPHA    0.15f  // Was 0.2
#define OUTPUT_RATE_LIMIT   20.0f  // Was 30

// Derivative on MEASUREMENT (not error)
float rateDelta = (currentRate - state.prevRate) / dt;
float rateD_raw = -rateKd * rateDelta;  // Negative sign!
```

**Tuning Steps if Still Oscillating:**
1. Reduce `RATE_ROLL_KP` (try 0.25, 0.20)
2. Increase `RATE_ROLL_KD` (try 0.03, 0.035)
3. Reduce `ANGLE_ROLL_KP` (try 2.0, 1.8)
4. Reduce `OUTPUT_RATE_LIMIT` (try 15.0, 10.0)

---

### 3. RC Stick Direction
**Problem:** Stick movements don't match drone response.

**Expected Behavior (Mode 2):**
- Pitch stick UP → Nose goes DOWN (negative pitch command)
- Roll stick RIGHT → Right side goes DOWN (positive roll command)
- Yaw stick RIGHT → Drone rotates CLOCKWISE (positive yaw)
- Throttle UP → Motors speed up

**Configuration in FC:**
```cpp
// Axis inversions (adjust based on IMU mounting)
#define PITCH_INVERT    -1.0f
#define ROLL_INVERT      1.0f

// RC inversions (adjust based on your transmitter)
#define RC_ROLL_INVERT      -1.0f
#define RC_PITCH_INVERT     1.0f
#define RC_YAW_INVERT       -1.0f
```

**Configuration in Remote:**
```cpp
#define THROTTLE_DIRECTION  +1
#define YAW_DIRECTION       +1
#define PITCH_DIRECTION     -1  // UP stick = negative value
#define ROLL_DIRECTION      +1
```

---

## Motor Mixing Reference (X-Quad)

```
        FRONT
   FL(CCW)  FR(CW)      Motor directions (viewed from above)
       ╲  ╱
        ╳
       ╱  ╲
   RL(CW)  RR(CCW)
```

### Correct Mixing Formula:
```cpp
// Positive roll = right side down = MORE power to FR, RR
// Positive pitch = nose down = MORE power to FL, FR
// Positive yaw = clockwise = MORE power to CCW (FL, RR)

FL = Throttle - Roll + Pitch + Yaw   (CCW)
FR = Throttle + Roll + Pitch - Yaw   (CW)
RL = Throttle - Roll - Pitch - Yaw   (CW)
RR = Throttle + Roll - Pitch + Yaw   (CCW)
```

---

## PID Tuning Procedure

### Step 1: Start Safe
```cpp
#define ANGLE_ROLL_KP  2.0f
#define ANGLE_ROLL_KI  0.0f
#define RATE_ROLL_KP   0.25f
#define RATE_ROLL_KI   0.0f
#define RATE_ROLL_KD   0.02f
```

### Step 2: Tune Rate P
1. Arm and hover
2. Give small roll/pitch inputs
3. If slow response: increase RATE_KP by 0.05
4. If oscillating: decrease RATE_KP by 0.05
5. Target: Quick response without oscillation

### Step 3: Tune Rate D
1. With Rate P set, observe high-frequency vibration
2. If vibrating: increase RATE_KD by 0.005
3. If sluggish/laggy: decrease RATE_KD by 0.005
4. Target: Smooth, damped response

### Step 4: Tune Angle P
1. Make larger angle corrections
2. If slow to level: increase ANGLE_KP by 0.5
3. If overshooting level: decrease ANGLE_KP by 0.5
4. Target: Quick return to level without overshoot

### Step 5: Add I-term (optional)
Only add if drone drifts at hover:
```cpp
#define ANGLE_ROLL_KI  0.02f  // Start very small
```

---

## Diagnostic Checklist

### Before First Flight:
- [ ] All propellers removed for motor test
- [ ] Each motor spins in correct direction
- [ ] Motors respond to correct stick direction
- [ ] ESCs calibrated
- [ ] Battery fully charged
- [ ] No loose wires

### Motor Test (use motor_test_utility.ino):
1. Test each motor individually
2. Listen for speed differences
3. Check rotation direction
4. Verify no vibration/noise

### If Motor is Slow:
1. Check propeller (damaged, wrong direction?)
2. Check motor (bearing noise?)
3. Check ESC calibration
4. Increase TRIM for that motor
5. Increase MOTOR_SCALE for that motor

### If Drifting:
1. Verify level calibration
2. Check for bent motor mount
3. Verify center of gravity
4. Adjust trim values
5. Check PID tuning

---

## Common Problems & Solutions

| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| Drifts one direction | Motor trim wrong | Increase trim for that direction |
| Fast oscillation | Rate P too high | Decrease RATE_KP |
| Slow oscillation | Angle P too high | Decrease ANGLE_KP |
| Vibration | D-term noise | Decrease RATE_KD or increase D filtering |
| Won't level | I-term too low | Add small ANGLE_KI |
| Sluggish | P too low | Increase ANGLE_KP and/or RATE_KP |
| One motor slow | Mechanical issue | Check motor/ESC, increase trim |
| Wrong stick direction | Inversion wrong | Flip RC_XXX_INVERT sign |

---

## Safe Testing Protocol

1. **Remove props** - Test motors, verify directions
2. **Props on, tethered** - Test at low throttle, verify response
3. **First hover** - Very gentle, ready to disarm
4. **Tune in small steps** - One parameter at a time
5. **Land between changes** - Don't tune in flight

---

## Motor Trim Values Quick Reference

If your drone pulls in a direction, adjust these:

| Direction of Pull | Increase Trim For |
|-------------------|-------------------|
| Front-Left | FR, RR |
| Front-Right | FL, RL |
| Rear-Left | FR, RR |
| Rear-Right | FL, RL |
| Forward | RL, RR |
| Backward | FL, FR |

Current trim values:
```cpp
#define TRIM_FL  +30
#define TRIM_FR  +50
#define TRIM_RL  +80
#define TRIM_RR  +120  // Increased for slow motor
```
