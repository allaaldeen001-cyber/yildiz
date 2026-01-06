# PID Tuning Guide for Quadcopter

This guide explains how to diagnose and fix common flight stability problems.

---

## Understanding PID

```
PID Output = (Kp × Error) + (Ki × ∫Error) + (Kd × dError/dt)

Kp = Proportional - How hard to push back against error (NOW)
Ki = Integral     - Accumulated error correction (PAST)
Kd = Derivative   - Dampening/prediction (FUTURE)
```

**Simple analogy:**
- **Kp** = How hard you push a shopping cart to correct course
- **Ki** = Noticing you've been drifting left for a while, so push right more
- **Kd** = Seeing a turn coming and preparing for it

---

## Problem 1: Fast Shaking / Vibration / Jitter

### What it looks like:
```
                    High frequency oscillation
    ─────────────────────────────────────────────────
         /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
    ─────────────────────────────────────────────────
                    Target (level)
```

The quad vibrates rapidly, motors sound "angry" or make a buzzing noise. Very fast oscillations (10-50 Hz).

### Causes:
1. **Kd too HIGH** - Derivative amplifies sensor noise
2. **Noisy sensors** - Vibration from motors reaching IMU
3. **Loop rate issues** - Inconsistent timing

### Solutions:

**Try in this order:**

| Solution | How to do it | Why it works |
|----------|--------------|--------------|
| 1. Reduce Kd | Decrease by 20-30% | Less noise amplification |
| 2. Add D-term filtering | Already in code (0.7 alpha) | Smooths derivative |
| 3. Soft-mount IMU | Use foam/rubber standoffs | Reduces vibration to sensor |
| 4. Check props | Balance props, check for damage | Reduces vibration source |

**Code change:**
```cpp
// In quadcopter_fc.ino, find and reduce Kd:
PID pidRoll(4.0f, 0.02f, 1.5f);   // Original
PID pidRoll(4.0f, 0.02f, 1.0f);   // Try reducing Kd (third value)
```

---

## Problem 2: Slow Wobbling / Low Frequency Oscillation

### What it looks like:
```
                    Low frequency oscillation (1-5 Hz)
    ─────────────────────────────────────────────────
              /‾‾‾\      /‾‾‾\      /‾‾‾\
             /     \    /     \    /     \
    ────────/───────\──/───────\──/───────\──────────
                     \/         \/         \
                                            Target
```

The quad slowly rocks back and forth. You can see it wobble with your eyes. Period of 0.2-1 second.

### Causes:
1. **Kp too HIGH** - Overcorrecting
2. **Kd too LOW** - Not enough dampening
3. **Ki too HIGH** - Integral windup causing overshoot

### Solutions:

**Try in this order:**

| Solution | How to do it | Why it works |
|----------|--------------|--------------|
| 1. Increase Kd | Increase by 20-30% | More dampening |
| 2. Reduce Kp | Decrease by 10-20% | Less aggressive correction |
| 3. Reduce Ki | Decrease by 50% | Less integral buildup |

**Code change:**
```cpp
// Original
PID pidRoll(4.0f, 0.02f, 1.5f);

// Option A: More dampening (increase Kd)
PID pidRoll(4.0f, 0.02f, 2.0f);

// Option B: Less aggressive (reduce Kp)
PID pidRoll(3.2f, 0.02f, 1.5f);

// Option C: Both
PID pidRoll(3.5f, 0.02f, 2.0f);
```

---

## Problem 3: Constant Drift / Won't Hold Level

### What it looks like:
```
                    Steady state error
    ─────────────────────────────────────────────────
                                           Actual
    ──────────────────────────────────────/──────────
                                         /
    ────────────────────────────────────/─────────────
                                       /
    ──────────────────────────────────/───────────────
                                      Target (level)
```

The quad doesn't return to exactly level. Always tilted slightly in one direction. Gets worse over time.

### Causes:
1. **Ki too LOW** - Not correcting accumulated error
2. **IMU not calibrated** - Sensor offset
3. **Physical imbalance** - Weight distribution, motor thrust differences

### Solutions:

**Try in this order:**

| Solution | How to do it | Why it works |
|----------|--------------|--------------|
| 1. Increase Ki | Increase by 50-100% | Corrects steady-state error |
| 2. Recalibrate IMU | Press calibration button | Zeros sensor offsets |
| 3. Check balance | Ensure battery centered | Physical cause |

**Code change:**
```cpp
// Original
PID pidRoll(4.0f, 0.02f, 1.5f);

// Increase Ki (second value)
PID pidRoll(4.0f, 0.04f, 1.5f);   // Doubled Ki

// If still drifting, try more
PID pidRoll(4.0f, 0.06f, 1.5f);   // 3x Ki
```

**Important:** Don't increase Ki too much or you'll get slow oscillation!

---

## Problem 4: Overshoots / Bounces Past Target

### What it looks like:
```
    Command: "Go to level"
    
                   Overshoot!
                      /\
                     /  \
                    /    \    Overshoot again
                   /      \      /\
    ──────────────/────────\────/──\────────────────
                            \  /    \____ Finally settles
                             \/
    ────────────────────────────────────────────────
                    Target (level)
```

When you let go of stick, the quad goes past level, comes back, goes past again, eventually settles.

### Causes:
1. **Kp too HIGH** - Too aggressive
2. **Kd too LOW** - Not predicting/dampening enough
3. **Ki too HIGH** - Integral windup pushing past target

### Solutions:

**Try in this order:**

| Solution | How to do it | Why it works |
|----------|--------------|--------------|
| 1. Increase Kd | Increase by 30-50% | Predicts overshoot, slows down |
| 2. Reduce Kp | Decrease by 20% | Less aggressive push |
| 3. Reduce Ki | Decrease by 30% | Less integral windup |

**Code change:**
```cpp
// Original
PID pidRoll(4.0f, 0.02f, 1.5f);

// Best fix: More Kd (dampening)
PID pidRoll(4.0f, 0.02f, 2.2f);

// Alternative: Less Kp
PID pidRoll(3.2f, 0.02f, 1.5f);

// Or both
PID pidRoll(3.5f, 0.015f, 2.0f);
```

---

## Problem 5: Sluggish Response / Feels Heavy

### What it looks like:
```
    Command: "Tilt right!"
    
    ────────────────────────────────────────────────
                    Target (desired angle)
    ─────────────────────────────────/──────────────
                                    /
                                   /
                                  /   Slow response
                                 /
                                /
    ───────────────────────────/────────────────────
    Start                      Takes forever!
```

The quad responds slowly to stick inputs. Feels "mushy" or "floaty". Doesn't hold position in wind.

### Causes:
1. **Kp too LOW** - Not pushing hard enough against error
2. **Heavy quad** - Not enough authority
3. **Weak motors/props** - Physical limitation

### Solutions:

**Try in this order:**

| Solution | How to do it | Why it works |
|----------|--------------|--------------|
| 1. Increase Kp | Increase by 20-30% | More responsive |
| 2. Increase Ki slightly | Increase by 20% | Better tracking |
| 3. Increase Kd slightly | Increase by 10% | Keep it stable |

**Code change:**
```cpp
// Original
PID pidRoll(4.0f, 0.02f, 1.5f);

// More responsive
PID pidRoll(5.0f, 0.02f, 1.5f);   // 25% more Kp

// Even more responsive (may need more Kd too)
PID pidRoll(5.5f, 0.025f, 1.8f);
```

**Warning:** If you increase Kp too much, you'll get oscillation!

---

## Quick Reference Chart

| Problem | Kp | Ki | Kd |
|---------|----|----|----| 
| Fast shaking | - | - | ⬇️ REDUCE |
| Slow wobbling | ⬇️ reduce | ⬇️ reduce | ⬆️ INCREASE |
| Constant drift | - | ⬆️ INCREASE | - |
| Overshoots | ⬇️ reduce | ⬇️ reduce | ⬆️ INCREASE |
| Sluggish | ⬆️ INCREASE | ⬆️ slight | ⬆️ slight |

---

## Systematic Tuning Procedure

### Step 1: Start Fresh
```cpp
// Start with conservative values
PID pidRoll(2.0f, 0.0f, 0.0f);   // Kp only, no Ki, no Kd
PID pidPitch(2.0f, 0.0f, 0.0f);
```

### Step 2: Tune Kp First
1. Arm and hover
2. Give small roll/pitch inputs
3. **If sluggish:** Increase Kp by 0.5
4. **If oscillating:** Reduce Kp by 0.5
5. Find the point where it JUST starts to oscillate
6. Reduce Kp by 20% from that point

```cpp
// Example: Oscillation started at Kp=5.0
// Final Kp = 5.0 × 0.8 = 4.0
PID pidRoll(4.0f, 0.0f, 0.0f);
```

### Step 3: Add Kd for Dampening
1. Start with Kd = Kp × 0.3
2. **If still oscillating:** Increase Kd
3. **If shaking/jittery:** Reduce Kd
4. Goal: Smooth, no oscillation

```cpp
// Kp=4.0, start Kd at 4.0×0.3=1.2
PID pidRoll(4.0f, 0.0f, 1.2f);

// Adjust as needed
PID pidRoll(4.0f, 0.0f, 1.5f);  // If oscillating, more Kd
```

### Step 4: Add Ki for Drift Correction
1. Start with Ki = 0.01
2. **If drifting:** Increase Ki slowly
3. **If slow oscillation starts:** Reduce Ki
4. Goal: Returns to level, no wobble

```cpp
// Add small Ki
PID pidRoll(4.0f, 0.01f, 1.5f);

// If still drifting
PID pidRoll(4.0f, 0.02f, 1.5f);
```

### Step 5: Fine Tune
1. Test in different conditions
2. Make small adjustments (10-20% at a time)
3. Tune pitch the same way
4. Yaw usually needs less aggressive values

---

## Recommended Starting Values by Quad Size

### 250mm (Mini Quad)
```cpp
PID pidRoll(4.5f, 0.03f, 1.8f);
PID pidPitch(4.5f, 0.03f, 1.8f);
PID pidYaw(4.0f, 0.02f, 0.0f);
```

### 450mm (Medium Quad)
```cpp
PID pidRoll(3.5f, 0.02f, 1.5f);
PID pidPitch(3.5f, 0.02f, 1.5f);
PID pidYaw(3.0f, 0.01f, 0.0f);
```

### 550mm+ (Large Quad)
```cpp
PID pidRoll(2.5f, 0.015f, 1.2f);
PID pidPitch(2.5f, 0.015f, 1.2f);
PID pidYaw(2.0f, 0.01f, 0.0f);
```

---

## Altitude Hold PID Tuning

The altitude PID has different characteristics:

```cpp
PID pidAlt(50.0f, 0.5f, 30.0f);
```

### Problems and Fixes:

| Problem | Symptom | Fix |
|---------|---------|-----|
| Bouncing up/down | Oscillates in altitude | Reduce Kp, increase Kd |
| Won't hold altitude | Drifts up or down | Increase Ki |
| Slow altitude response | Takes forever to climb | Increase Kp |
| Overshoots target alt | Goes past then comes back | Increase Kd |

---

## Safety Tips for Tuning

1. **Remove props first** for initial tests (check motor response direction)
2. **Secure the quad** or hold it loosely for first powered tests
3. **Tune outdoors** with lots of space
4. **Start with low throttle** (just barely hovering)
5. **Make small changes** (10-20% at a time)
6. **Keep notes** of what values you tried
7. **Have a way to disarm quickly** (kill switch!)

---

## Code Location for PID Values

In `quadcopter_fc.ino`, find these lines near the end of the variable declarations:

```cpp
// PID Controllers - ADJUST THESE VALUES
PID pidRoll(4.0f, 0.02f, 1.5f);    // Kp, Ki, Kd for Roll
PID pidPitch(4.0f, 0.02f, 1.5f);   // Kp, Ki, Kd for Pitch  
PID pidYaw(3.0f, 0.01f, 0.0f);     // Kp, Ki, Kd for Yaw
PID pidAlt(50.0f, 0.5f, 30.0f);    // Kp, Ki, Kd for Altitude
```

After changing values:
1. Save the file
2. Upload to drone
3. Test carefully
4. Repeat until good!

---

## Visual Summary

```
    TOO MUCH                JUST RIGHT               TOO LITTLE
    
Kp: ~~~~~~~ Oscillation     ──────── Stable         ~~~~~~~~ Sluggish
    
Kd: ∿∿∿∿∿∿∿ Shaking         ──────── Smooth         ~~~~~~~~ Wobble
    
Ki: ~~~~~~~ Slow wobble     ──────── Level          ↗↗↗↗↗↗↗ Drift
```

---

Good luck tuning! Remember: Small changes, test often, be patient! 🚁
