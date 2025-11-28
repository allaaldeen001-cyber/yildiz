# PID Tuning Guide

## Complete Guide to Tuning Your Drone's Control Loops

This guide provides systematic procedures for tuning all PID controllers in the flight control system.

---

## 📋 Table of Contents

1. [Understanding the Control Architecture](#understanding-the-control-architecture)
2. [Pre-Tuning Requirements](#pre-tuning-requirements)
3. [Rate PID Tuning (Inner Loop)](#rate-pid-tuning-inner-loop)
4. [Angle PID Tuning (Outer Loop)](#angle-pid-tuning-outer-loop)
5. [Yaw Rate Tuning](#yaw-rate-tuning)
6. [Altitude PID Tuning](#altitude-pid-tuning)
7. [Advanced Tuning Tips](#advanced-tuning-tips)
8. [Common Problems & Solutions](#common-problems--solutions)

---

## 🎯 Understanding the Control Architecture

### Cascade PID Overview

```
                    TUNING ORDER
                    ════════════

  Step 1: Rate PID (Inner Loop)
  ─────────────────────────────
  • Must be tuned FIRST
  • Runs at 400 Hz
  • Controls angular velocity
  • Rejects disturbances quickly

          ┌─────────────────┐
          │    RATE PID     │
          │   P, I, D       │
          │                 │
  Rate ───│► Error ─► Out ──│──► Motor
  Cmd     │                 │    Mixer
          │ ◄─── Gyro ◄─────│
  Current └─────────────────┘
  Rate

  Step 2: Angle PID (Outer Loop)
  ──────────────────────────────
  • Tune AFTER rate loops
  • Runs at 100 Hz
  • Controls attitude (tilt angle)
  • Generates rate setpoints

          ┌─────────────────┐
          │   ANGLE PID     │
          │   P, I, D       │
          │                 │
  Angle ──│► Error ─► Out ──│──► Rate
  Cmd     │                 │    Setpoint
          │ ◄─── AHRS ◄─────│
  Current └─────────────────┘
  Angle
```

### What Each Gain Does

| Gain | Effect | Too Low | Too High |
|------|--------|---------|----------|
| **P** | Proportional response | Sluggish, drifty | Oscillation, jitter |
| **I** | Eliminates steady-state error | Slow correction, drift | Slow oscillation, overshoot |
| **D** | Dampens oscillation | Overshoots, bouncy | Vibration, noise sensitivity |

---

## 🔧 Pre-Tuning Requirements

### Essential Setup

1. **Props Removed** - Initial tuning should be done with props OFF
2. **Secure Mount** - Drone secured but free to rotate on one axis
3. **Serial Debug Enabled** - Uncomment `#define DEBUG_PID` in config.h
4. **Fresh Battery** - Full charge for consistent power
5. **Calm Environment** - No wind, stable surface

### Initial Values (Starting Point)

```cpp
// config.h - START HERE

// Rate PID - Start conservative
#define PID_ROLL_RATE_KP    0.5f   // Start at 50% of default
#define PID_ROLL_RATE_KI    0.0f   // Start with zero
#define PID_ROLL_RATE_KD    0.0f   // Start with zero

#define PID_PITCH_RATE_KP   0.5f
#define PID_PITCH_RATE_KI   0.0f
#define PID_PITCH_RATE_KD   0.0f

// Angle PID - Start conservative  
#define PID_ROLL_ANGLE_KP   3.0f   // Start at 50% of default
#define PID_ROLL_ANGLE_KI   0.0f
#define PID_ROLL_ANGLE_KD   0.0f
```

---

## 🔄 Rate PID Tuning (Inner Loop)

### Step-by-Step Procedure

#### 1. Tune Roll Rate First

```
ROLL RATE P TUNING
──────────────────

a) Set I and D to 0
b) Start with P = 0.3

c) Give quick roll input, observe response:

   Low P (sluggish):          Good P:              High P (oscillating):
   
   Input ──┐                  Input ──┐            Input ──┐
           │                          │                    │
   Rate  ──┴──────────       Rate ────┘────       Rate ────┘╱╲╱╲──
         slow response              quick stop           oscillation

d) Increase P until oscillation starts, then reduce by 20%

e) Target: Quick response, minimal overshoot
```

#### 2. Add Roll Rate D

```
ROLL RATE D TUNING
──────────────────

a) Keep optimal P, I still at 0
b) Start with D = 0.01

c) Increase D until:
   • Oscillation from P test is damped
   • Response feels "crisp"
   
d) Watch for vibration - if motors buzz, reduce D

e) Typical values: D = 0.02 to 0.05
```

#### 3. Add Roll Rate I

```
ROLL RATE I TUNING
──────────────────

a) Keep optimal P and D
b) Start with I = 0.1

c) Test by holding constant stick:
   • Does it drift? Increase I
   • Does it oscillate slowly? Decrease I

d) Typical values: I = 0.2 to 0.5
```

#### 4. Repeat for Pitch Rate

- Same procedure as roll
- Values usually similar to roll (symmetric quad)
- May need slight adjustment for CG offset

### Rate PID Reference Values

| Frame Type | P | I | D |
|------------|---|---|---|
| Small (3") | 0.5-0.8 | 0.2-0.4 | 0.02-0.04 |
| Medium (5") | 0.6-0.9 | 0.3-0.5 | 0.02-0.05 |
| Large (7"+) | 0.4-0.7 | 0.2-0.4 | 0.03-0.06 |

---

## 📐 Angle PID Tuning (Outer Loop)

### Prerequisites

- Rate PIDs must be tuned first!
- Drone should be stable with stick inputs

### Step-by-Step Procedure

#### 1. Tune Angle P

```
ANGLE P TUNING
──────────────

a) Set angle I and D to 0
b) Start with P = 2.0

c) Give angle command, observe:

   Low P:                    Good P:                High P:
   ┌────                     ┌────                  ┌─╱╲─
   │                         │                      │
   │    slow lean            │   quick response     │  oscillation
   └────────                 └────────              └────────

d) Increase P until oscillation, reduce by 30%

e) Target: Snappy response, holds angle well
```

#### 2. Add Angle I (Optional)

```
ANGLE I TUNING
──────────────

a) Usually not needed for angle loops
b) If drone drifts when trying to hover:
   - Add small I (0.01-0.05)
   - Too much causes slow wallowing

c) Better solution: Calibrate accelerometer properly
```

#### 3. Angle D is Usually Zero

- Angle D is rarely needed
- Rate D already provides damping
- Can add 0.1-0.5 if needed for very slow frames

### Angle PID Reference Values

| Responsiveness | P | I | D |
|----------------|---|---|---|
| Beginner | 3.0-4.0 | 0.01 | 0 |
| Normal | 4.0-5.0 | 0.02 | 0 |
| Sport | 5.0-6.0 | 0.02 | 0 |
| Acro | 6.0-8.0 | 0.03 | 0 |

---

## 🔄 Yaw Rate Tuning

### Special Considerations

- Yaw only has rate control (no angle loop)
- Yaw authority is typically lower than roll/pitch
- Higher P needed due to lower motor differential

### Procedure

```
YAW RATE TUNING
───────────────

a) Start values: P=1.5, I=0.3, D=0

b) Give yaw input, observe:
   • Slow rotation: Increase P
   • Oscillation/overshoot: Decrease P, add I

c) Yaw bounce test:
   • Quick yaw, let go
   • Should stop without bounce
   • Add I if it drifts back

d) Typical values:
   P = 1.5 - 3.0
   I = 0.3 - 0.8  
   D = 0 (usually not needed)
```

---

## 📊 Altitude PID Tuning

### Cascade Structure

```
ALTITUDE CONTROL CASCADE
────────────────────────

                 Alt Error    Vel Cmd     Vel Error    Throttle
Alt Setpoint ───►[ALT PID]───►   ───────►[VEL PID]───► Adjustment
                     ▲                        ▲
                     │                        │
                Alt Estimate           Vel Estimate
                 (Barometer)          (Baro + Accel)
```

### Altitude PID (Outer)

```
a) Set to: P=0.3, I=0, D=0

b) Enable altitude hold, observe:
   • Sinks/rises slowly: Increase P
   • Bounces up/down: Decrease P

c) Add I (0.01-0.05) to eliminate steady offset

d) Add D (0.05-0.15) to reduce overshoot on transitions
```

### Velocity PID (Inner)

```
a) Set to: P=0.1, I=0, D=0

b) Command altitude change, observe:
   • Slow climb/descent: Increase P
   • Throttle oscillation: Decrease P

c) Add I (0.01-0.03) for consistent climb rates

d) D usually not needed
```

### Altitude Reference Values

| Parameter | Conservative | Normal | Aggressive |
|-----------|--------------|--------|------------|
| Alt P | 0.3 | 0.5 | 0.8 |
| Alt I | 0.01 | 0.02 | 0.03 |
| Alt D | 0.05 | 0.1 | 0.15 |
| Vel P | 0.1 | 0.15 | 0.2 |
| Vel I | 0.01 | 0.02 | 0.03 |

---

## 🎓 Advanced Tuning Tips

### TPA (Throttle PID Attenuation)

At high throttle, motors have more authority. Consider reducing D at high throttle:

```cpp
// In pid.h - optional enhancement
float throttleScale = 1.0f - (throttle - 1500) * 0.001f;
dTerm *= constrain(throttleScale, 0.5f, 1.0f);
```

### Feed-Forward

For sharper response, add feed-forward to rate controller:

```cpp
float feedForward = stickDerivative * FF_GAIN;
output = pTerm + iTerm + dTerm + feedForward;
```

### Filtering

If motors are hot or making noise:
1. Reduce D term
2. Increase `GYRO_LPF_ALPHA` in config.h
3. Check for vibration sources

---

## 🔧 Common Problems & Solutions

### Problem: Oscillation at Hover

```
Symptoms: Drone wobbles/shakes when hovering
Cause: P too high or D too low

Fix:
1. Reduce Rate P by 10-20%
2. Increase Rate D by 10-20%
3. Check for loose components causing vibration
```

### Problem: Slow/Sluggish Response

```
Symptoms: Drone feels "floaty", slow to respond
Cause: P too low

Fix:
1. Increase Rate P by 20%
2. Increase Angle P by 20%
3. Verify loop timing is correct
```

### Problem: Bounces Back After Maneuver

```
Symptoms: Overshoots and oscillates after stick movement
Cause: I too high or D too low

Fix:
1. Reduce Rate I by 20%
2. Increase Rate D slightly
3. Check Angle P isn't too high
```

### Problem: Drift in One Direction

```
Symptoms: Drone drifts even with centered sticks
Cause: Accelerometer calibration or I term

Fix:
1. Recalibrate on perfectly level surface
2. Add small Angle I (0.01-0.02)
3. Check for motor/ESC issues
```

### Problem: Yaw Wobble

```
Symptoms: Drone wobbles during yaw rotation
Cause: Yaw P too high or motor timing issues

Fix:
1. Reduce Yaw P by 20%
2. Check motor direction/timing
3. Verify ESC calibration
```

### Problem: Altitude Porpoising

```
Symptoms: Bounces up/down in altitude hold
Cause: Alt PID too aggressive

Fix:
1. Reduce Alt P by 30%
2. Reduce Vel P by 30%
3. Increase baro filter (BARO_LPF_ALPHA)
```

---

## 📝 Tuning Log Template

```
Date: ___________
Battery: _________
Weather: _________

Rate PID - Roll
  P: _____ → _____
  I: _____ → _____
  D: _____ → _____
  Notes: _________________

Rate PID - Pitch
  P: _____ → _____
  I: _____ → _____
  D: _____ → _____
  Notes: _________________

Angle PID - Roll
  P: _____ → _____
  I: _____ → _____
  Notes: _________________

Angle PID - Pitch
  P: _____ → _____
  I: _____ → _____
  Notes: _________________

Yaw Rate PID
  P: _____ → _____
  I: _____ → _____
  Notes: _________________

Altitude PID
  Alt P: _____ → _____
  Alt I: _____ → _____
  Vel P: _____ → _____
  Notes: _________________

Overall Feel:
  □ Sluggish  □ Responsive  □ Twitchy
  □ Stable    □ Drifty      □ Bouncy
  
Next Steps:
_________________________________
_________________________________
```

---

## ✅ Final Checklist

After tuning, verify:

- [ ] Drone responds equally to all directions
- [ ] No oscillation at any throttle level
- [ ] Smooth altitude hold (if enabled)
- [ ] Quick yaw without wobble
- [ ] No motor heating or noise
- [ ] Stable hover with hands off
- [ ] Good response to stick inputs
- [ ] Clean landing without bounce

---

*Happy Tuning! Remember: Small changes, one parameter at a time.*
