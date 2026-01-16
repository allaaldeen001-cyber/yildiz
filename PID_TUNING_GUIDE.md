# Quadcopter PID Tuning Guide - Eliminating Oscillation

## Understanding Your Oscillation Problem

Your oscillation cycle:
```
Drone tilts left → PID overcorrects right → Overshoots right → 
PID overcorrects left → Overshoots left → Repeat (oscillation!)
```

This happens because:
1. **P gain too high** - The proportional response is too aggressive
2. **Insufficient damping** - Not enough D term to slow down the correction
3. **Single-loop PID** - No rate feedback to sense how fast it's moving

## Solution: Cascaded PID Architecture

The new code uses a **cascaded PID** (also called dual-loop or angle+rate PID):

```
         ┌─────────────┐         ┌─────────────┐
Stick →  │ ANGLE LOOP  │ → Rate →│  RATE LOOP  │ → Motors
         │ (outer, slow)│ Target │ (inner, fast) │
         └─────────────┘         └─────────────┘
                ↑                        ↑
           Current Angle            Gyro Rate
```

### Why This Stops Oscillation:

1. **Angle Loop** (outer) - Slow, converts angle error to target rate
   - "I'm tilted 10° left, so I want to rotate right at 30°/sec"
   
2. **Rate Loop** (inner) - Fast, controls motors based on gyro
   - "I'm rotating at 40°/sec but want 30°/sec, reduce motors"
   - **This is the key!** It knows HOW FAST it's moving, not just position

With single-loop PID, the controller only knows position. With cascaded, it also knows velocity - providing natural damping.

## Anti-Oscillation Techniques in the New Code

### 1. Derivative on Measurement (Not Error)

**Problem with classic D term:**
```cpp
// CLASSIC (problematic):
float D = Kd * (error - prevError) / dt;
// When setpoint changes suddenly, D spikes! ("derivative kick")
```

**Solution:**
```cpp
// NEW (anti-oscillation):
float D = -Kd * (measurement - prevMeasurement) / dt;
// Only reacts to actual drone movement, not setpoint changes
```

### 2. Setpoint Filtering

Smooth the target angle to prevent sudden changes:
```cpp
filteredSetpoint = filteredSetpoint + 0.3 * (target - filteredSetpoint);
```

### 3. Output Rate Limiting

Prevent sudden motor changes:
```cpp
// Max 30 units change per 2ms loop cycle
if (newOutput - prevOutput > 30) newOutput = prevOutput + 30;
```

## Tuning Procedure

### Step 1: Start with Safe Gains (Already Set)

The new code starts with conservative gains:
```cpp
// Outer loop (angle)
ANGLE_ROLL_KP = 3.0   // Angle error to rate
ANGLE_ROLL_KI = 0.0   // Start at zero!

// Inner loop (rate) - These are critical!
RATE_ROLL_KP = 0.5    // Rate error to motor
RATE_ROLL_KI = 0.0    // Start at zero!
RATE_ROLL_KD = 0.015  // Damping
```

### Step 2: Test and Observe

1. Arm and apply ~40% throttle (hover)
2. Tilt the drone slightly by hand and release
3. Observe the response:

| Observation | Problem | Fix |
|-------------|---------|-----|
| Fast oscillation (vibration) | Rate P too high | Reduce `RATE_ROLL_KP` by 20% |
| Slow oscillation (wobble) | Angle P too high | Reduce `ANGLE_ROLL_KP` by 20% |
| Slow to level | All P too low | Increase `RATE_ROLL_KP` by 10% |
| Overshoots then settles | Need more D | Increase `RATE_ROLL_KD` by 20% |
| Drifts slowly | Need I term | Add small `RATE_ROLL_KI` (0.01) |

### Step 3: Tune Rate Loop First (Inner Loop)

**Always tune the inner loop before the outer loop!**

1. Set `ANGLE_ROLL_KP = 1.0` (low, so rate loop dominates)
2. Increase `RATE_ROLL_KP` until you see slight oscillation
3. Reduce by 20-30%
4. Increase `RATE_ROLL_KD` until oscillation stops completely
5. The drone should now resist rotation smoothly

### Step 4: Tune Angle Loop (Outer Loop)

1. Increase `ANGLE_ROLL_KP` until drone returns to level quickly
2. If it overshoots, reduce or add more D to rate loop
3. Target: Quick return to level without overshoot

### Step 5: Add I Term Last (If Needed)

Only add I term if the drone drifts or doesn't hold level:
```cpp
RATE_ROLL_KI = 0.01  // Start very small!
```

## Gain Adjustment Reference

### If Still Oscillating (Fast Vibration):

```cpp
// Reduce rate P
#define RATE_ROLL_KP   0.3f  // Was 0.5
#define RATE_PITCH_KP  0.3f

// Increase rate D
#define RATE_ROLL_KD   0.025f  // Was 0.015
#define RATE_PITCH_KD  0.025f
```

### If Still Oscillating (Slow Wobble):

```cpp
// Reduce angle P
#define ANGLE_ROLL_KP  2.0f  // Was 3.0
#define ANGLE_PITCH_KP 2.0f
```

### If Too Sluggish:

```cpp
// Increase rate P carefully
#define RATE_ROLL_KP   0.7f  // Was 0.5
#define RATE_PITCH_KP  0.7f

// And/or increase angle P
#define ANGLE_ROLL_KP  4.0f  // Was 3.0
#define ANGLE_PITCH_KP 4.0f
```

### If Overshooting Then Settling:

```cpp
// Increase D term
#define RATE_ROLL_KD   0.03f  // Was 0.015
#define RATE_PITCH_KD  0.03f
```

## Filtering Adjustments

### If Still Vibrating After PID Tuning:

```cpp
// More aggressive D-term filtering
#define D_TERM_LPF_ALPHA    0.1f  // Was 0.2 (lower = more filtering)

// Slower motor response
#define MOTOR_LPF_ALPHA     0.3f  // Was 0.4

// Stricter output rate limit
#define OUTPUT_RATE_LIMIT   20.0f  // Was 30
```

### If Too Slow to Respond:

```cpp
// Less filtering
#define D_TERM_LPF_ALPHA    0.3f  // Was 0.2
#define MOTOR_LPF_ALPHA     0.5f  // Was 0.4
#define OUTPUT_RATE_LIMIT   40.0f // Was 30
```

## Common Mistakes to Avoid

1. **Don't add I term first** - I term causes slow oscillation if P and D aren't tuned
2. **Don't tune both axes at once** - Tune roll first, then copy to pitch
3. **Don't make big changes** - Adjust gains by 10-20% at a time
4. **Don't skip the rate loop** - Always tune inner loop before outer
5. **Don't test at full throttle** - Tune at hover throttle (~40-50%)

## Emergency Fixes

### Drone Flips Instantly on Takeoff
- Motor directions wrong
- Props on wrong motors
- PID signs inverted (check ROLL_INVERT, PITCH_INVERT)

### Drone Oscillates Only at High Throttle
- Reduce `RATE_ROLL_KP` by 30%
- Reduce `ESC_MAX_THROTTLE` temporarily

### Drone Oscillates Only When Moving Stick
- Increase `SETPOINT_LPF_ALPHA` (more setpoint filtering)
- This smooths out your stick input

## Debug Using Serial Output

The serial output shows real-time PID state:
```
ARM | R:2.3 P:-1.1 | Rate:5,-3 | PID:12,-8 | M:1350,1340,1360,1345 | D:15,-10
     │     │            │           │              │                    │
     │     │            │           │              │                    └─ Motor differential
     │     │            │           │              └─ Motor values (µs)
     │     │            │           └─ PID output (roll, pitch)
     │     │            └─ Gyro rate (deg/sec)
     │     └─ Current angles (degrees)
     └─ Flight state
```

**Signs of oscillation in debug:**
- PID output constantly swinging positive/negative
- Rate values alternating rapidly
- Motors values rapidly changing

**Signs of good tuning:**
- PID output steady when level
- Quick, clean response when tilted
- Motors stable during hover
