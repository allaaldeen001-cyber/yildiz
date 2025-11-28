# PID Tuning Guide - Professional Drone Flight Controller

## Overview

This guide provides step-by-step procedures for tuning the PID controllers in the flight control system. Proper tuning is critical for stable, responsive flight.

---

## Tuning Philosophy

### Cascade Control Tuning Order

**CRITICAL:** Always tune in this order:
1. **Rate PIDs** (inner loop) - Most important
2. **Angle PIDs** (outer loop) - Depends on rate PIDs
3. **Altitude PID** - Depends on angle control

### General Principles

1. **Start Conservative:** Begin with lower gains, increase gradually
2. **One Axis at a Time:** Tune roll, then pitch, then yaw separately
3. **Small Increments:** Change gains by 10-20% at a time
4. **Test Thoroughly:** After each change, test hover and maneuvers
5. **Document Changes:** Keep notes of what works

---

## Rate PID Tuning (Inner Loop)

### Purpose
Controls angular rates directly using gyroscope feedback. This is the foundation of stable flight.

### Default Values
```
Roll Rate:  Kp=0.8, Ki=0.0, Kd=0.05
Pitch Rate: Kp=0.8, Ki=0.0, Kd=0.05
Yaw Rate:   Kp=1.2, Ki=0.0, Kd=0.1
```

### Tuning Procedure

#### Step 1: Initial Setup
1. Set all gains to default values
2. Ensure IMU is calibrated
3. Test on bench (no props!) first

#### Step 2: Tune Roll Rate PID

**A. Increase Kp (Proportional)**
- Start with Kp = 0.5
- Increase by 0.1 increments
- Test: Quick roll stick input → observe response
- **Goal:** Fast response without oscillation
- **Stop when:** Slight overshoot appears

**B. Add Kd (Derivative)**
- Start with Kd = 0.02
- Increase by 0.01 increments
- **Goal:** Eliminate overshoot and oscillation
- **Stop when:** Smooth, damped response

**C. Ki (Integral)**
- Usually NOT needed for rate control
- Only add if persistent steady-state error
- Start with Ki = 0.01
- **Warning:** Can cause wind-up and instability

#### Step 3: Tune Pitch Rate PID
- Use same procedure as roll
- Values should be similar (symmetrical quad)

#### Step 4: Tune Yaw Rate PID
- Yaw typically needs higher Kp (more authority)
- Start with Kp = 1.2
- Yaw is less critical, can tolerate more oscillation

### Symptoms and Solutions

| Symptom | Cause | Solution |
|---------|-------|----------|
| Sluggish response | Kp too low | Increase Kp |
| Oscillation | Kp too high | Decrease Kp |
| Overshoot | Kd too low | Increase Kd |
| Slow response | Kd too high | Decrease Kd |
| Jittery | Kd too high | Decrease Kd |
| Steady-state error | Need Ki | Add small Ki |

### Testing Checklist

- [ ] Quick stick input → immediate response
- [ ] Release stick → returns to center smoothly
- [ ] No oscillation after input
- [ ] No overshoot
- [ ] Stable hover (no drift)

---

## Angle PID Tuning (Outer Loop)

### Purpose
Converts pilot angle commands to rate setpoints. Provides smooth, intuitive control.

### Default Values
```
Roll Angle:  Kp=3.0, Ki=0.0, Kd=0.0
Pitch Angle: Kp=3.0, Ki=0.0, Kd=0.0
```

### Tuning Procedure

#### Step 1: Tune Roll Angle PID

**A. Increase Kp**
- Start with Kp = 2.0
- Increase by 0.5 increments
- Test: Command 10° roll → observe tracking
- **Goal:** Reaches commanded angle quickly without overshoot
- **Stop when:** Overshoot appears

**B. Ki and Kd**
- Usually NOT needed for angle control
- Angle PIDs are simpler than rate PIDs
- Only add if specific issues arise

#### Step 2: Tune Pitch Angle PID
- Use same procedure as roll
- Should match roll values

### Symptoms and Solutions

| Symptom | Cause | Solution |
|---------|-------|----------|
| Slow to reach angle | Kp too low | Increase Kp |
| Overshoot | Kp too high | Decrease Kp |
| Oscillation | Kp too high | Decrease Kp |

### Testing Checklist

- [ ] Command 10° roll → reaches angle smoothly
- [ ] Release stick → returns to level
- [ ] No overshoot
- [ ] Feels "locked in" during hover

---

## Altitude PID Tuning

### Purpose
Maintains commanded altitude using throttle adjustment.

### Default Values
```
Altitude: Kp=0.5, Ki=0.1, Kd=0.05
```

### Tuning Procedure

#### Step 1: Tune Kp (Proportional)

**A. Increase Kp**
- Start with Kp = 0.3
- Increase by 0.1 increments
- Test: Enable altitude hold, disturb drone
- **Goal:** Quick correction without oscillation
- **Stop when:** Altitude oscillates

#### Step 2: Tune Ki (Integral)

**B. Add Ki**
- Start with Ki = 0.05
- Increase by 0.05 increments
- **Purpose:** Eliminates steady-state error
- **Warning:** Too high causes wind-up
- **Stop when:** Altitude maintains without drift

#### Step 3: Tune Kd (Derivative)

**C. Add Kd**
- Start with Kd = 0.02
- Increase by 0.01 increments
- **Purpose:** Dampens overshoot
- **Stop when:** Smooth altitude transitions

### Symptoms and Solutions

| Symptom | Cause | Solution |
|---------|-------|----------|
| Slow correction | Kp too low | Increase Kp |
| Oscillation | Kp too high | Decrease Kp |
| Altitude drift | Ki too low | Increase Ki |
| Wind-up | Ki too high | Decrease Ki |
| Overshoot | Kd too low | Increase Kd |

### Testing Checklist

- [ ] Enable altitude hold → maintains altitude
- [ ] Disturb drone → returns to altitude smoothly
- [ ] No oscillation
- [ ] No drift over time
- [ ] Smooth transitions when enabling/disabling

---

## AHRS Filter Tuning

### Mahony Filter Parameters

**Default Values:**
```
Kp_Mahony = 2.0
Ki_Mahony = 0.005
```

### Tuning Procedure

#### Kp_Mahony (Proportional Gain)

**Purpose:** How quickly filter corrects attitude using accelerometer

**Tuning:**
- **Too Low (< 1.0):** Slow correction, attitude drifts
- **Too High (> 4.0):** Noisy attitude, over-correction
- **Sweet Spot:** 1.5 - 2.5

**Test:**
- Tilt drone manually
- Observe attitude reading
- Should track quickly but smoothly

#### Ki_Mahony (Integral Gain)

**Purpose:** Handles long-term gyro drift

**Tuning:**
- **Too Low (< 0.001):** Yaw drift over time
- **Too High (> 0.01):** Causes instability
- **Sweet Spot:** 0.003 - 0.007

**Test:**
- Hover for 30 seconds
- Observe yaw drift
- Should remain stable

### Symptoms and Solutions

| Symptom | Cause | Solution |
|---------|-------|----------|
| Attitude drifts | Ki too low | Increase Ki_Mahony |
| Noisy attitude | Kp too high | Decrease Kp_Mahony |
| Slow correction | Kp too low | Increase Kp_Mahony |
| Yaw drift | Ki too low | Increase Ki_Mahony |

---

## Complete Tuning Workflow

### Phase 1: Bench Testing (No Props!)

1. **Power On**
   - Verify all systems initialize
   - Check sensor readings
   - Verify NRF link

2. **Calibration**
   - Perform IMU calibration
   - Perform ESC calibration
   - Verify beep sequences

3. **Motor Test**
   - Test each motor individually
   - Verify correct direction
   - Check PWM range

4. **Control Test**
   - Move joysticks
   - Observe motor responses
   - Verify motor mixing

### Phase 2: Hover Tuning

1. **First Hover**
   - Low altitude (30 cm)
   - Test basic stability
   - Observe behavior

2. **Rate PID Tuning**
   - Tune roll rate first
   - Then pitch rate
   - Finally yaw rate
   - Goal: Stable hover, responsive

3. **Angle PID Tuning**
   - Tune roll angle
   - Then pitch angle
   - Goal: Smooth angle tracking

4. **Altitude PID Tuning**
   - Enable altitude hold
   - Tune for stability
   - Goal: Maintains altitude

### Phase 3: Flight Testing

1. **Basic Maneuvers**
   - Forward/backward flight
   - Left/right flight
   - Yaw rotation
   - Observe stability

2. **Aggressive Maneuvers**
   - Quick direction changes
   - Rapid altitude changes
   - Test limits

3. **Failsafe Testing**
   - Test link loss
   - Test tilt limit
   - Verify safety systems

---

## Tuning Tips

### Do's

✅ **Do** tune one axis at a time  
✅ **Do** make small incremental changes  
✅ **Do** test thoroughly after each change  
✅ **Do** document your changes  
✅ **Do** start with conservative values  
✅ **Do** test in safe, open areas  
✅ **Do** use a spotter for first flights  

### Don'ts

❌ **Don't** tune multiple axes simultaneously  
❌ **Don't** make large gain changes  
❌ **Don't** skip testing steps  
❌ **Don't** ignore safety procedures  
❌ **Don't** tune without props first  
❌ **Don't** fly near people or property  

---

## Recommended PID Values by Quad Size

### Small Quad (250-350mm)
```
Rate PIDs:    Kp=0.6-0.8, Kd=0.03-0.05
Angle PIDs:   Kp=2.5-3.0
Altitude PID: Kp=0.4-0.6, Ki=0.08-0.12, Kd=0.03-0.05
```

### Medium Quad (400-500mm)
```
Rate PIDs:    Kp=0.8-1.0, Kd=0.05-0.08
Angle PIDs:   Kp=3.0-4.0
Altitude PID: Kp=0.5-0.7, Ki=0.1-0.15, Kd=0.05-0.08
```

### Large Quad (550mm+)
```
Rate PIDs:    Kp=1.0-1.2, Kd=0.08-0.12
Angle PIDs:   Kp=4.0-5.0
Altitude PID: Kp=0.6-0.8, Ki=0.15-0.2, Kd=0.08-0.12
```

**Note:** These are starting points. Actual values depend on:
- Motor/propeller combination
- Frame stiffness
- Weight distribution
- Battery voltage

---

## Troubleshooting Tuning Issues

### Problem: Drone Oscillates Constantly

**Possible Causes:**
1. Rate PID Kp too high
2. Mechanical vibration
3. Loose components

**Solutions:**
1. Reduce rate PID Kp by 20%
2. Check prop balance
3. Tighten all screws
4. Add vibration dampening

### Problem: Slow Response to Commands

**Possible Causes:**
1. Rate PID Kp too low
2. ESC calibration issue
3. Motor/prop mismatch

**Solutions:**
1. Increase rate PID Kp by 20%
2. Recalibrate ESCs
3. Check motor power

### Problem: Altitude Hold Oscillates

**Possible Causes:**
1. Altitude PID Kp too high
2. Barometer noise
3. Throttle response too fast

**Solutions:**
1. Reduce altitude PID Kp
2. Check barometer mounting
3. Add low-pass filter to altitude

### Problem: Yaw Drift

**Possible Causes:**
1. No magnetometer (normal)
2. Motor imbalance
3. Prop imbalance

**Solutions:**
1. Use yaw stick to correct (normal)
2. Check motor directions
3. Balance props
4. Consider adding magnetometer

---

## Advanced Tuning Techniques

### Dynamic Tuning

Adjust gains based on flight mode:
- **Acro Mode:** Higher rate gains for aggressive flight
- **Stabilized Mode:** Lower angle gains for smooth flight
- **Altitude Hold:** Optimized altitude PID

### Adaptive Tuning

Some advanced systems adjust gains based on:
- Battery voltage (lower voltage = lower gains)
- Flight speed (higher speed = different gains)
- Altitude (different gains at different altitudes)

**Note:** Current firmware uses fixed gains. Adaptive tuning requires additional code.

---

## Conclusion

Proper PID tuning is essential for stable, responsive flight. Follow this guide systematically, test thoroughly, and always prioritize safety. Remember: **Better to be slightly under-tuned than over-tuned.**

Good luck and fly safe! 🚁
