# 🎚️ PID Tuning Guide

## Overview

This guide explains the PID control system and provides step-by-step tuning instructions.

---

## PID Controller Basics

### What is PID?

**PID** = **P**roportional + **I**ntegral + **D**erivative

```
Output = Kp × Error + Ki × ∫Error dt + Kd × dError/dt
```

### PID Terms Explained

| Term | What it does | Analogy |
|------|-------------|---------|
| **P** (Proportional) | React to current error | "How far off am I right now?" |
| **I** (Integral) | Correct accumulated error | "How long have I been off?" |
| **D** (Derivative) | Predict future error | "How fast am I moving toward target?" |

---

## Cascaded PID Architecture

This flight controller uses **cascaded (nested) PID loops** like Betaflight:

```
┌─────────────────────────────────────────────────────────┐
│                     OUTER LOOP                          │
│  Target Angle → [Angle PID] → Target Rate              │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│                     INNER LOOP                          │
│  Target Rate → [Rate PID] → Motor Output                │
└─────────────────────────────────────────────────────────┘
```

### Why Cascaded?

- **Better stability**: Rate loop dampens oscillations
- **Better control**: Angle loop provides smooth leveling
- **Professional behavior**: Same as DJI, Betaflight, PX4

---

## Default PID Values

These are **pre-tuned** values that should work for most 250-450mm quadcopters:

### Rate PID (Inner Loop)

```cpp
// Roll Rate
pidRateRoll.Kp = 0.65;
pidRateRoll.Ki = 0.35;
pidRateRoll.Kd = 0.018;
pidRateRoll.maxI = 150.0;

// Pitch Rate
pidRatePitch.Kp = 0.65;
pidRatePitch.Ki = 0.35;
pidRatePitch.Kd = 0.018;
pidRatePitch.maxI = 150.0;

// Yaw Rate
pidYaw.Kp = 0.8;
pidYaw.Ki = 0.3;
pidYaw.Kd = 0.005;
pidYaw.maxI = 100.0;
```

### Angle PID (Outer Loop)

```cpp
// Roll Angle
pidAngleRoll.Kp = 4.0;
pidAngleRoll.Ki = 0.0;  // Usually not needed
pidAngleRoll.Kd = 0.0;  // Usually not needed

// Pitch Angle
pidAnglePitch.Kp = 4.0;
pidAnglePitch.Ki = 0.0;
pidAnglePitch.Kd = 0.0;
```

### Altitude PID

```cpp
pidAltitude.Kp = 4.5;
pidAltitude.Ki = 0.15;
pidAltitude.Kd = 3.5;
pidAltitude.maxI = 200.0;
```

---

## Tuning Steps

### Prerequisites

Before tuning:
- [ ] Props ON (tuning without props is not accurate)
- [ ] Secure the drone (use a test stand or soft tether)
- [ ] Open Serial Monitor (watch for oscillations)
- [ ] Have a way to quickly disarm (remove throttle stick)

---

## Step 1: Tune Rate PID (Roll)

This is the **most important** PID to tune!

### 1.1 Start with P-only

Set initial values:
```cpp
pidRateRoll.Kp = 0.5;
pidRateRoll.Ki = 0.0;  // Start with I disabled
pidRateRoll.Kd = 0.0;  // Start with D disabled
```

Upload and test:
1. Arm drone
2. Give slight throttle
3. Tilt drone with roll stick
4. Observe response

**Too low Kp**: Drone doesn't correct, drifts  
**Good Kp**: Drone resists tilt, returns to level  
**Too high Kp**: Fast oscillations (buzzing sound)

**Action**: Increase Kp in steps of 0.1 until you see **slight oscillation**.

### 1.2 Add D term (damping)

Once you have slight oscillation:
```cpp
pidRateRoll.Kd = 0.015;  // Start here
```

**Effect of D**:
- Reduces oscillation
- Makes response smoother
- Too much = sluggish response

**Action**: Increase Kd until oscillations stop.

### 1.3 Add I term (steady-state correction)

Now add integral:
```cpp
pidRateRoll.Ki = 0.3;  // Start here
```

**Effect of I**:
- Corrects small drifts
- Holds angle precisely
- Too much = slow wobbles

**Action**: Increase Ki until drone holds position perfectly.

### 1.4 Final Roll Rate Values

Example final values:
```cpp
pidRateRoll.Kp = 0.65;
pidRateRoll.Ki = 0.35;
pidRateRoll.Kd = 0.018;
```

---

## Step 2: Tune Rate PID (Pitch)

**Repeat the same process for Pitch**:

Start with:
```cpp
pidRatePitch.Kp = pidRateRoll.Kp;  // Copy roll values
pidRatePitch.Ki = pidRateRoll.Ki;
pidRatePitch.Kd = pidRateRoll.Kd;
```

Test pitch axis:
1. Arm drone
2. Push pitch stick forward/back
3. Observe response

Usually pitch and roll have **same values**, but adjust if needed:
- Pitch more sluggish? → Increase Kp
- Pitch oscillates more? → Increase Kd

---

## Step 3: Tune Rate PID (Yaw)

Yaw is different (slower, less responsive):

Start with:
```cpp
pidYaw.Kp = 0.8;   // Higher than roll/pitch
pidYaw.Ki = 0.3;
pidYaw.Kd = 0.005; // Much lower than roll/pitch
```

Test yaw:
1. Arm drone
2. Push yaw stick left/right
3. Drone should rotate smoothly

**Too low Kp**: Slow rotation, doesn't track stick  
**Too high Kp**: Jerky rotation  
**Too high Kd**: Yaw feels "sticky"

---

## Step 4: Tune Angle PID (Roll & Pitch)

Angle PID is **easier** - usually just tune Kp:

Start with:
```cpp
pidAngleRoll.Kp = 4.0;
pidAnglePitch.Kp = 4.0;
```

Test in ANGLE mode:
1. Arm drone
2. Tilt drone with stick
3. Release stick → drone should return to level

**Too low Kp**: Slow return to level  
**Good Kp**: Quick return, no overshoot  
**Too high Kp**: Overshoots, wobbles

**Note**: Ki and Kd are usually **0** for angle PID (inner rate loop handles damping).

---

## Step 5: Tune Altitude PID

Test in ALT_HOLD or during landing:

Start with:
```cpp
pidAltitude.Kp = 4.5;
pidAltitude.Ki = 0.15;
pidAltitude.Kd = 3.5;
```

Test altitude hold:
1. Arm drone
2. Switch to ALT_HOLD mode
3. Drone should hover at constant altitude

**Too low Kp**: Drone drifts up/down slowly  
**Good Kp**: Holds altitude, small corrections  
**Too high Kp**: Bounces up and down (oscillates)

**D term** is critical for altitude:
- Too low Kd → Oscillates (bobbing motion)
- Good Kd → Smooth, damped response
- Too high Kd → Over-damped, sluggish

**I term** corrects for battery voltage drop:
- Too low Ki → Slowly drifts down as battery drains
- Good Ki → Holds altitude precisely
- Too high Ki → Slow oscillations (period ~5-10 seconds)

---

## Observing PID Behavior

### What to Watch

#### Visual Observation

| Symptom | Cause | Fix |
|---------|-------|-----|
| Fast oscillation (10-20 Hz) | Kp too high | Reduce Kp |
| Slow oscillation (1-2 Hz) | Ki too high | Reduce Ki |
| Overshoots target | Kd too low | Increase Kd |
| Sluggish response | Kp too low or Kd too high | Increase Kp or reduce Kd |
| Drifts slowly | Ki too low | Increase Ki |

#### Serial Monitor

Watch telemetry output:
```
Mode:ANGLE | R:2.3 P:-1.5 | Alt:150cm V:0cm/s | M:1520,1515,1510,1525
```

**Good signs**:
- Roll/Pitch near 0° when hovering
- Altitude stable (±5 cm)
- Motor speeds similar (±20 µs)

**Bad signs**:
- Roll/Pitch oscillating ±5° or more → Reduce Kp or increase Kd
- Altitude drifting continuously → Increase Ki
- Motor speeds wildly different → Check frame balance

#### Audio Feedback

- **Smooth hum**: Good tuning ✅
- **High-pitched buzz**: Kp too high ❌
- **Pulsing sound**: Ki too high ❌
- **One motor louder**: Physical imbalance (not PID issue)

---

## Advanced Tuning

### Integral Anti-Windup

Integral windup happens when error accumulates during extreme conditions (e.g., full throttle).

The code implements **integral clamping**:
```cpp
pid->integral += error * dt;
pid->integral = constrain(pid->integral, -pid->maxI, pid->maxI);
```

If you see **slow oscillations**, reduce `maxI`:
```cpp
pidRateRoll.maxI = 100.0;  // Reduce from 150.0
```

### Feed-Forward (Optional)

Not implemented by default, but you can add:
```cpp
float feedforward = targetRate * 0.1;  // 10% feed-forward
output = P + I + D + feedforward;
```

Benefits:
- Faster response to stick input
- Reduces tracking error

---

## PID Tuning Checklist

### Before Flight
- [ ] Default PID values uploaded
- [ ] Sensors calibrated (gyro + barometer)
- [ ] Props balanced and secure
- [ ] Frame rigid (no loose parts)

### Tuning Sequence
- [ ] Step 1: Rate PID - Roll (P → D → I)
- [ ] Step 2: Rate PID - Pitch (copy roll, adjust if needed)
- [ ] Step 3: Rate PID - Yaw (higher Kp, lower Kd)
- [ ] Step 4: Angle PID (usually just Kp)
- [ ] Step 5: Altitude PID (Kp → Kd → Ki)

### Validation
- [ ] Hover test (30 seconds, stable altitude)
- [ ] Angle mode test (release sticks → returns to level)
- [ ] Acro mode test (smooth rate response)
- [ ] Altitude hold test (no bobbing)
- [ ] Landing test (smooth descent, no drops)

---

## Quick Reference: PID Effects

### Proportional (Kp)

| Change | Effect |
|--------|--------|
| Increase Kp | Faster response, more oscillation |
| Decrease Kp | Slower response, less oscillation |

### Integral (Ki)

| Change | Effect |
|--------|--------|
| Increase Ki | Less drift, more slow oscillation |
| Decrease Ki | More drift, less slow oscillation |

### Derivative (Kd)

| Change | Effect |
|--------|--------|
| Increase Kd | More damping, less overshoot, might get sluggish |
| Decrease Kd | Less damping, more overshoot, might oscillate |

---

## Common Problems

### Problem: Oscillates on one axis only

**Cause**: Motor imbalance or frame flex

**Solution**:
1. Check motor screws are tight
2. Check arm is not cracked
3. Balance props
4. If still oscillates, reduce Kp on that axis only

---

### Problem: Slow bobbing up and down

**Cause**: Altitude Ki too high

**Solution**:
```cpp
pidAltitude.Ki = 0.1;  // Reduce from 0.15
```

---

### Problem: Altitude drops slowly over time

**Cause**: Altitude Ki too low (can't compensate for battery voltage drop)

**Solution**:
```cpp
pidAltitude.Ki = 0.2;  // Increase from 0.15
```

---

### Problem: Feels sluggish, doesn't respond to stick

**Cause**: Kp too low or Kd too high

**Solution**:
```cpp
pidRateRoll.Kp = 0.75;  // Increase from 0.65
pidRateRoll.Kd = 0.015; // Reduce from 0.018
```

---

## PID Tuning Tools

### In-Code Tuning

Edit values in `FlightController.ino`:
```cpp
#define RATE_ROLL_KP  0.65f  // ← Change this
#define RATE_ROLL_KI  0.35f
#define RATE_ROLL_KD  0.018f
```

Upload after each change.

### Real-Time Tuning (Advanced)

Add serial commands to adjust PIDs without re-uploading:

```cpp
// In loop()
if (Serial.available()) {
  char cmd = Serial.read();
  if (cmd == '+') pidRateRoll.Kp += 0.05;
  if (cmd == '-') pidRateRoll.Kp -= 0.05;
  Serial.print("Kp: ");
  Serial.println(pidRateRoll.Kp);
}
```

Send `+` or `-` from Serial Monitor to adjust Kp in real-time.

---

## Summary

### Tuning Order

1. **Rate PID** (most important)
   - Roll: P → D → I
   - Pitch: Copy roll, adjust
   - Yaw: Higher P, lower D

2. **Angle PID** (easier)
   - Just tune Kp (usually 3-5)

3. **Altitude PID** (for landing)
   - Kp for response
   - Kd for damping (critical!)
   - Ki for steady-state

### Golden Rules

✅ **Tune one parameter at a time**  
✅ **Make small changes** (0.05-0.1 increments)  
✅ **Test after each change** (30 second hover minimum)  
✅ **Start conservative** (can always increase)  
✅ **Safety first** (test stand or soft tether)  

### Expected Results

After tuning:
- **Stable hover** with minimal corrections
- **Returns to level** smoothly when stick released
- **Holds altitude** within ±5 cm
- **Smooth landing** with no drops or bounces

**Good luck tuning! 🎚️**
