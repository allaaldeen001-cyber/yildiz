# 🎛️ PID Tuning Guide - Quadcopter Drone

## Table of Contents
- [Understanding PID Control](#understanding-pid-control)
- [Tuning Prerequisites](#tuning-prerequisites)
- [Tuning Process](#tuning-process)
- [Advanced Tuning](#advanced-tuning)
- [Common Issues](#common-issues)

---

## 🧠 Understanding PID Control

### What is PID?

PID is a control algorithm that calculates corrections to keep your drone stable. It continuously measures errors and adjusts motor speeds.

**Real-world analogy**: Imagine driving a car:
- **P** (Proportional): How hard you turn the wheel when you drift
- **I** (Integral): Correcting for wind pushing you sideways
- **D** (Derivative): How smoothly you return to center

### Cascaded PID System (Betaflight Style)

Your flight controller uses **two PID loops** working together:

```
USER INPUT (RC Stick)
    ↓
┌─────────────────────────────────────┐
│  OUTER LOOP: ANGLE PID              │
│  Input: Desired angle (degrees)     │
│  Feedback: Accelerometer angle      │
│  Output: Desired rate (deg/s)       │
└─────────────────────────────────────┘
    ↓
┌─────────────────────────────────────┐
│  INNER LOOP: RATE PID               │
│  Input: Desired rate (deg/s)        │
│  Feedback: Gyroscope rate           │
│  Output: Motor corrections          │
└─────────────────────────────────────┘
    ↓
MOTOR MIXING → ESCs → Motors
```

**Why two loops?**
- **Outer loop (Angle)**: Provides stability, auto-leveling
- **Inner loop (Rate)**: Provides agility, fast response
- **Together**: Professional flight characteristics!

---

## 📋 Tuning Prerequisites

### Before You Start

⚠️ **CRITICAL SAFETY**:
1. **REMOVE ALL PROPELLERS** during tuning
2. Secure drone to prevent damage
3. Have emergency disarm ready
4. Wear safety glasses

### Required Tools

1. **Serial Monitor** (Arduino IDE, 115200 baud)
2. **Notebook** for recording values
3. **Flat workspace** with good lighting
4. **Spare propellers** (you might break some!)

### Frame Setup Checklist

✅ All motors spin freely  
✅ ESCs calibrated  
✅ Battery fully charged  
✅ Gyro calibrated on level surface  
✅ Motor directions correct (FL/RR = CCW, FR/RL = CW)  
✅ Propellers removed  

---

## 🎯 Tuning Process

### Overview

You will tune **3 PID controllers** in this order:

1. **RATE PID** (most important) - Gyro response
2. **ANGLE PID** - Auto-leveling strength
3. **ALTITUDE PID** - Height control

### STEP 1: Rate PID Tuning (Inner Loop)

This is the **most critical** tuning step. Rate PID controls how quickly the drone responds to rotation commands.

#### 1.1 Setup

```cpp
// In FlightController.ino, find these lines:

pidRateRoll.Kp = 0.0;  // Start at zero
pidRateRoll.Ki = 0.0;
pidRateRoll.Kd = 0.0;
pidRateRoll.maxI = 100;

// Same for pitch and yaw
```

#### 1.2 Tune P Term (Proportional)

**Goal**: Find Kp that makes drone respond quickly but oscillates slightly

**Procedure**:
1. Set `Kp = 0.3`
2. Switch to **ACRO mode** (SW2 = OFF)
3. ARM drone (props off!)
4. Tilt frame by hand in roll direction
5. Watch Serial Monitor for motor response
6. **Increase Kp by 0.1** until:
   - Motors respond immediately to tilt
   - You see slight oscillation in motor speeds
7. **Reduce Kp by 20%** to eliminate oscillation

**Example progression**:
```
Kp = 0.3 → Motors barely respond
Kp = 0.5 → Getting better
Kp = 0.7 → Good response
Kp = 0.9 → Starting to oscillate
Kp = 0.8 ✅ FINAL VALUE (0.9 - 20%)
```

**Symptoms**:
- **Too low**: Sluggish response, drone drifts
- **Too high**: High-frequency oscillations, motor twitching
- **Perfect**: Quick response, no oscillation

#### 1.3 Tune D Term (Derivative)

**Goal**: Dampen oscillations and smooth response

**Procedure**:
1. Start with `Kd = 0.005`
2. Increase by 0.005 until oscillations disappear
3. Too much D → sluggish, delayed response
4. Typical range: `0.010 - 0.030`

**Example**:
```
Kd = 0.000 → Oscillates slightly
Kd = 0.010 → Smoother
Kd = 0.015 ✅ FINAL VALUE (no oscillation)
Kd = 0.030 → Too sluggish
```

#### 1.4 Tune I Term (Integral)

**Goal**: Eliminate steady-state error and drift

**Procedure**:
1. Start with `Ki = 0.1`
2. Hover in place (with props on, outdoor test!)
3. If drifts slowly → increase Ki
4. If oscillates slowly → reduce Ki
5. Typical range: `0.2 - 0.6`

**Example**:
```
Ki = 0.0 → Drifts with wind
Ki = 0.2 → Slight drift correction
Ki = 0.4 ✅ FINAL VALUE (holds position)
Ki = 0.8 → Slow oscillations
```

#### 1.5 Repeat for All Axes

Tune **Roll**, **Pitch**, and **Yaw** separately:
- Roll and Pitch usually have identical values
- Yaw typically has lower P (less sensitive)

**Typical values**:
```cpp
// Roll & Pitch
pidRateRoll.Kp = 0.8;
pidRateRoll.Ki = 0.4;
pidRateRoll.Kd = 0.015;

pidRatePitch.Kp = 0.8;
pidRatePitch.Ki = 0.4;
pidRatePitch.Kd = 0.015;

// Yaw (less aggressive)
pidRateYaw.Kp = 0.8;
pidRateYaw.Ki = 0.4;
pidRateYaw.Kd = 0.015;
```

---

### STEP 2: Angle PID Tuning (Outer Loop)

Now that Rate PID is tuned, configure the Angle PID for auto-leveling.

#### 2.1 Setup

```cpp
pidAngleRoll.Kp = 0.0;  // Start at zero
pidAngleRoll.Ki = 0.0;  // Usually stays at 0
pidAngleRoll.Kd = 0.0;  // Usually stays at 0
```

#### 2.2 Tune Angle P Term

**Goal**: Find Kp that returns drone to level quickly

**Procedure**:
1. Set `Kp = 2.0`
2. Switch to **ANGLE mode** (SW2 = ON)
3. ARM drone (props on, outdoor test!)
4. Tilt drone ~15° in roll
5. Release sticks to center
6. Watch how quickly it levels

**Increase Kp** until:
- Drone returns to level quickly (1-2 seconds)
- No overshoot or bouncing
- Feels "locked in place"

**Example progression**:
```
Kp = 2.0 → Slowly returns to level
Kp = 3.0 → Faster return
Kp = 3.5 ✅ FINAL VALUE (quick, no overshoot)
Kp = 5.0 → Overshoots, bounces
```

**Typical values**:
```cpp
pidAngleRoll.Kp = 3.5;
pidAngleRoll.Ki = 0.0;  // Leave at 0
pidAngleRoll.Kd = 0.0;  // Leave at 0

pidAnglePitch.Kp = 3.5;
pidAnglePitch.Ki = 0.0;
pidAnglePitch.Kd = 0.0;
```

**Note**: Usually, Angle PID only needs the P term. The I and D terms are rarely used.

---

### STEP 3: Altitude PID Tuning

Finally, tune the altitude hold system.

#### 3.1 Setup

```cpp
pidAltitude.Kp = 0.0;
pidAltitude.Ki = 0.0;
pidAltitude.Kd = 0.0;
pidAltitude.maxI = 200;
```

#### 3.2 Tune Altitude P Term

**Goal**: Find Kp that maintains altitude without bouncing

**Procedure**:
1. Set `Kp = 3.0`
2. Enable **ALTITUDE HOLD** (SW1 = ON)
3. Takeoff manually to ~1.5m
4. Observe altitude stability
5. **Increase Kp** until:
   - Altitude stays locked (±10cm)
   - No bouncing up/down

**Example**:
```
Kp = 3.0 → Drifts slowly
Kp = 5.0 ✅ FINAL VALUE (stable)
Kp = 8.0 → Bounces up/down
```

#### 3.3 Tune Altitude D Term (Velocity Damping)

**Goal**: Dampen vertical oscillations

**Procedure**:
1. Start with `Kd = 2.0`
2. Push drone down gently, release
3. Should return smoothly without bouncing
4. **Increase Kd** to eliminate bounce

**Example**:
```
Kd = 0.0 → Bounces after push
Kd = 2.0 → Better
Kd = 3.0 ✅ FINAL VALUE (smooth recovery)
Kd = 5.0 → Too sluggish, drifts
```

#### 3.4 Tune Altitude I Term

**Goal**: Eliminate steady altitude drift

**Procedure**:
1. Start with `Ki = 0.1`
2. Hover for 30 seconds
3. If drifts up/down → increase Ki
4. If oscillates → reduce Ki

**Example**:
```
Ki = 0.0 → Drifts down slowly
Ki = 0.2 ✅ FINAL VALUE (holds altitude)
Ki = 0.5 → Slow oscillations
```

**Typical values**:
```cpp
pidAltitude.Kp = 5.0;
pidAltitude.Ki = 0.2;
pidAltitude.Kd = 3.0;
pidAltitude.maxI = 200;
```

---

## 🚀 Advanced Tuning

### Feed-Forward (Not Implemented Yet)

Feed-forward improves responsiveness by anticipating movements. Consider adding in future versions:

```cpp
float feedForward = setpointRate * 0.1; // 10% of desired rate
output += feedForward;
```

### Dynamic Filtering

Reduce propwash oscillations with notch filters (future enhancement):

```cpp
// Notch filter at motor frequency
if (gyroFrequency == motorRPM) {
  applyNotchFilter();
}
```

### Throttle Curve

Non-linear throttle response for better low-throttle control:

```cpp
// Exponential throttle
int adjustedThrottle = throttle * throttle / 1000;
```

---

## 🔧 Common Issues & Solutions

### Issue 1: Oscillations at Hover

**Symptoms**: Drone bounces/vibrates when hovering

**Solutions**:
1. **Reduce P gain** (Rate PID) by 20%
2. **Increase D gain** (Rate PID) by 50%
3. Check for:
   - Loose motor mounts
   - Bent propellers
   - Vibrations in frame

**Example fix**:
```cpp
// Before
pidRateRoll.Kp = 1.0;  // Too high
pidRateRoll.Kd = 0.01; // Too low

// After
pidRateRoll.Kp = 0.8;  // Reduced
pidRateRoll.Kd = 0.015; // Increased
```

---

### Issue 2: Drifts Slowly in One Direction

**Symptoms**: Drone moves left/right/forward/back without stick input

**Solutions**:
1. **Recalibrate gyro** (Button 1)
2. **Increase I gain** (Rate PID)
3. Check accelerometer calibration

**Example fix**:
```cpp
// Before
pidRateRoll.Ki = 0.1;  // Too low

// After
pidRateRoll.Ki = 0.4;  // Increased
```

---

### Issue 3: Altitude Bounces Up/Down

**Symptoms**: In ALTITUDE HOLD, drone bounces

**Solutions**:
1. **Reduce P gain** (Altitude PID)
2. **Increase D gain** (Altitude PID)
3. Check MS5611 mounting (away from propwash)

**Example fix**:
```cpp
// Before
pidAltitude.Kp = 8.0;  // Too high
pidAltitude.Kd = 1.0;  // Too low

// After
pidAltitude.Kp = 5.0;  // Reduced
pidAltitude.Kd = 3.0;  // Increased
```

---

### Issue 4: Sluggish Response

**Symptoms**: Drone reacts slowly to stick input

**Solutions**:
1. **Increase P gain** (Rate PID)
2. **Reduce D gain** (Rate PID)
3. Check battery voltage (low voltage = slow response)

**Example fix**:
```cpp
// Before
pidRateRoll.Kp = 0.5;  // Too low
pidRateRoll.Kd = 0.03; // Too high

// After
pidRateRoll.Kp = 0.8;  // Increased
pidRateRoll.Kd = 0.015; // Reduced
```

---

### Issue 5: Flips on Takeoff

**Symptoms**: Drone immediately flips when armed

**Causes**:
1. **Wrong motor direction**
2. **Wrong motor position**
3. **Gyro not calibrated**

**Solutions**:
1. Check motor rotation:
   - FL (D3) = CCW
   - FR (D5) = CW
   - RR (D6) = CCW
   - RL (D9) = CW
2. Verify X-configuration wiring
3. Recalibrate gyro on flat surface

---

## 📊 PID Tuning Cheat Sheet

| Symptom | P | I | D |
|---------|---|---|---|
| **Oscillates rapidly** | ↓ | - | ↑ |
| **Drifts slowly** | - | ↑ | - |
| **Slow response** | ↑ | - | - |
| **Overshoots target** | - | - | ↑ |
| **Won't hold position** | - | ↑ | - |
| **Sluggish, delayed** | ↑ | - | ↓ |
| **Bounces/vibrates** | ↓ | - | ↑ |

**Legend**: ↑ = Increase, ↓ = Decrease, - = No change

---

## 🎓 Understanding Each Term

### P (Proportional) - "How hard to push"

- **Effect**: Immediate correction proportional to error
- **Too low**: Slow response, drifts
- **Too high**: Oscillations, unstable
- **Sweet spot**: Quick response, no oscillation

**Equation**: `P_output = Kp * error`

---

### I (Integral) - "Remember past errors"

- **Effect**: Accumulates error over time, eliminates drift
- **Too low**: Steady-state error, drifts
- **Too high**: Slow oscillations, overshoot
- **Sweet spot**: Holds position perfectly

**Equation**: `I_output = Ki * Σ(error * dt)`

**Anti-windup**: Limits integral buildup:
```cpp
integral = constrain(integral, -maxI, maxI);
```

---

### D (Derivative) - "Predict future error"

- **Effect**: Dampens oscillations, smooths response
- **Too low**: Oscillates, bounces
- **Too high**: Sluggish, ignores quick changes
- **Sweet spot**: Smooth, damped response

**Equation**: `D_output = Kd * (error - last_error) / dt`

**Better**: Derivative on measurement (avoids "kick"):
```cpp
D_output = Kd * (last_input - current_input) / dt
```

---

## 📈 Tuning Strategy Summary

### Quick Start (5 minutes)

1. **Rate PID**: Start with default values (Kp=0.8, Ki=0.4, Kd=0.015)
2. **Angle PID**: Start with Kp=3.5
3. **Altitude PID**: Start with Kp=5.0, Ki=0.2, Kd=3.0
4. **Test flight**: If stable, you're done!
5. **Fine-tune**: Adjust based on issues above

### Full Tuning (30 minutes)

1. **Rate P**: 0 → 1.0, find oscillation point, reduce 20%
2. **Rate D**: 0 → 0.03, eliminate oscillations
3. **Rate I**: 0 → 0.6, eliminate drift
4. **Angle P**: 2.0 → 5.0, fast return to level
5. **Altitude P**: 3.0 → 8.0, stable hover
6. **Altitude D**: 1.0 → 5.0, smooth recovery
7. **Altitude I**: 0 → 0.5, eliminate drift

---

## 🛠️ Tools for Analysis

### Serial Monitor Output

Watch these values during tuning:
```
Mode:ANGLE | Armed:YES | RC:OK
Roll:2.3 | Pitch:-1.5 | Alt:150cm
Motors:1245,1255,1240,1250
```

**What to look for**:
- **Roll/Pitch**: Should be near 0° when hovering
- **Alt**: Should be stable (±5cm)
- **Motors**: Should be similar (±20µs difference)

### Black Box Logging (Future Enhancement)

Record flight data to SD card for analysis:
- Gyro rates
- PID outputs
- Motor speeds
- Altitude

---

## 🎯 Final Recommendations

### For Beginners

Use conservative default values:
```cpp
// RATE PID
Kp = 0.6, Ki = 0.3, Kd = 0.012

// ANGLE PID
Kp = 3.0

// ALTITUDE PID
Kp = 4.0, Ki = 0.15, Kd = 2.5
```

### For Racing/Aerobatics

Use aggressive values:
```cpp
// RATE PID
Kp = 1.2, Ki = 0.6, Kd = 0.025

// ANGLE PID (not used in ACRO)
Kp = 5.0

// ALTITUDE PID (not used in racing)
Kp = 7.0, Ki = 0.3, Kd = 4.0
```

### For Smooth Video

Use damped values:
```cpp
// RATE PID
Kp = 0.7, Ki = 0.5, Kd = 0.020

// ANGLE PID
Kp = 3.0

// ALTITUDE PID
Kp = 5.0, Ki = 0.2, Kd = 3.5
```

---

## ✅ Tuning Checklist

Before declaring tuning complete:

- [ ] Hover in ANGLE mode for 30 seconds (stable?)
- [ ] Push drone by hand, returns to center? (no overshoot?)
- [ ] Fly figure-8 pattern (smooth?)
- [ ] Fly in wind (holds position with I term?)
- [ ] Altitude hold for 30 seconds (±10cm?)
- [ ] Fast stick movements (no oscillation?)
- [ ] Emergency stop works?
- [ ] Failsafe disarms?

---

**Happy Tuning! 🎛️**

*Remember: PID tuning is an iterative process. Start conservative, fly safe, and tune incrementally!*
