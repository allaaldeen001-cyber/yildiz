# Quadcopter PID Tuning Guide

## Table of Contents
1. [Understanding Your Issues](#understanding-your-issues)
2. [Pre-Flight Checklist](#pre-flight-checklist)
3. [PID Basics](#pid-basics)
4. [Tuning Process](#tuning-process)
5. [Troubleshooting Common Problems](#troubleshooting-common-problems)
6. [Advanced Tuning](#advanced-tuning)

---

## Understanding Your Issues

### Issue 1: Drone Drifts Toward RR (Rear-Right)

**Possible Causes:**
1. **IMU Calibration Error** - The gyroscope/accelerometer offsets are incorrect
2. **Physical CG Offset** - Center of gravity is not centered
3. **Motor/ESC Mismatch** - Motors or ESCs have different characteristics
4. **Propeller Issues** - Damaged, bent, or mismatched props
5. **Frame Twist** - Frame is not perfectly square

**Solutions:**
```cpp
// 1. Run MPU6050_Calibration sketch and copy the offsets
gyro_x_cal = YOUR_VALUE;
gyro_y_cal = YOUR_VALUE;
gyro_z_cal = YOUR_VALUE;

// 2. Add trim offsets to compensate for physical imbalance
// In calculateAngles() or calculateSetpoints():
angle_roll -= 0.5;   // Adjust this value (+ or - degrees)
angle_pitch -= 0.3;  // Adjust this value (+ or - degrees)
```

### Issue 2: Motors Not at Same Speed

**Possible Causes:**
1. **ESCs Not Calibrated** - Run ESC_Calibration sketch
2. **Different Motor KV** - Ensure all motors are identical
3. **PID Fighting Imbalance** - This is normal behavior when PID is working
4. **Wiring Differences** - Longer wires = more resistance

**Solutions:**
1. Calibrate all ESCs simultaneously
2. Check motor specifications match
3. Measure motor current draw at same throttle
4. Use the Motor_Test sketch to verify response

### Issue 3: Stability and Auto-Leveling

**The original code had NO stability control!** It was just displaying angles - not a flight controller.

The new code includes:
- Full PID control for Roll, Pitch, and Yaw
- Auto-level mode using angle feedback
- Rate mode for acrobatic flying
- Complementary filter for stable angle estimation

---

## Pre-Flight Checklist

Before attempting to fly, verify ALL of the following:

### 1. Hardware Checks
- [ ] All propellers securely attached
- [ ] Propeller rotation direction correct (CW/CCW as per diagram)
- [ ] Battery fully charged
- [ ] All connections secure
- [ ] Frame is rigid and square
- [ ] Center of gravity is centered

### 2. Software Checks
- [ ] MPU6050 calibrated (gyro offsets applied)
- [ ] ESCs calibrated (all respond identically)
- [ ] Receiver inputs working (check Serial Monitor)
- [ ] Arm/Disarm working
- [ ] Motor test completed (all spin correctly)

### 3. Motor Rotation Direction
```
       FRONT
    M1       M2
  (CCW)     (CW)
      \     /
       \   /
        \ /
        / \
       /   \
      /     \
  (CW)     (CCW)
    M4       M3
       REAR
```

**Verify with Motor_Test sketch before flying!**

---

## PID Basics

### What Each Term Does

**P (Proportional):**
- Reacts to current error
- Higher P = Faster response, but can oscillate
- Too high = Oscillation, too low = Sluggish

**I (Integral):**
- Accumulates past error
- Corrects steady-state drift
- Higher I = Fights wind/imbalance better
- Too high = Slow oscillation, overshoot

**D (Derivative):**
- Predicts future error
- Dampens oscillation, smooths response
- Higher D = More damping
- Too high = Vibration, jittering

### Default PID Values
```cpp
// Rate PID (inner loop)
float pid_p_gain_roll = 1.3;
float pid_i_gain_roll = 0.04;
float pid_d_gain_roll = 18.0;

float pid_p_gain_yaw = 4.0;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.0;

// Level PID (outer loop for auto-level)
float pid_p_gain_level = 3.0;
float pid_i_gain_level = 0.0;
```

---

## Tuning Process

### Step 1: Start Safe
1. Remove propellers
2. Connect to Serial Monitor
3. Verify gyro readings are stable (~0 when still)
4. Verify receiver inputs respond correctly

### Step 2: Set Ultra-Conservative Gains
```cpp
pid_p_gain_roll = 0.5;
pid_i_gain_roll = 0.0;
pid_d_gain_roll = 0.0;

pid_p_gain_pitch = 0.5;
pid_i_gain_pitch = 0.0;
pid_d_gain_pitch = 0.0;

pid_p_gain_yaw = 1.0;
pid_i_gain_yaw = 0.0;
pid_d_gain_yaw = 0.0;
```

### Step 3: Tune P Gain First
1. Secure drone loosely (can rotate but won't fly away)
2. Arm and apply ~30% throttle
3. Tilt drone and release - observe response
4. **Increase P** until you see oscillation
5. **Reduce P by 20-30%** from oscillation point

### Step 4: Add D Gain
1. With P set, add D gain
2. Start with D = P × 10 (rough estimate)
3. Increase D until oscillation dampens
4. If you get jitter/vibration, reduce D

### Step 5: Add I Gain (Small!)
1. Add very small I gain (0.01-0.05)
2. Test: drone should return to level after disturbance
3. Too much I = slow "wobble" or overshoot
4. I fights steady drift but causes problems if too high

### Step 6: Tune Yaw
1. Yaw typically needs more P than roll/pitch
2. Usually little or no D needed for yaw
3. Small I helps maintain heading

### Step 7: Tune Level Mode (if using)
1. Level mode uses angle, not rate
2. Tune pid_p_gain_level for self-leveling strength
3. Higher = snappier return to level
4. Too high = oscillation around level point

---

## Troubleshooting Common Problems

### Problem: High-Frequency Oscillation/Vibration
**Cause:** D gain too high, or vibration from motors
**Solution:**
- Reduce D gain
- Add vibration dampening to IMU mount
- Check for loose propellers

### Problem: Slow Oscillation (1-2 Hz)
**Cause:** P gain too high, or I gain accumulating
**Solution:**
- Reduce P gain
- Reduce I gain
- Reset I term on landing

### Problem: Drone Drifts in One Direction
**Cause:** IMU offset, CG offset, or motor imbalance
**Solution:**
- Re-calibrate IMU
- Add trim offset in code
- Check motor/ESC calibration
- Verify CG is centered

### Problem: Toilet Bowl Effect (Circling)
**Cause:** Yaw axis affecting roll/pitch, compass issue
**Solution:**
- Check motor rotation directions
- Verify prop placement (CW vs CCW)
- Tune yaw independently
- Check for magnetometer interference

### Problem: Drone Flips on Takeoff
**Cause:** Motor/prop configuration wrong, or PID sign wrong
**Solution:**
- Verify motor rotation directions with Motor_Test
- Check prop placement matches motor direction
- Verify PID output signs in motor mixing
- Start with props off, check that tilting drone increases correct motors

### Problem: Sluggish Response
**Cause:** P gain too low, or filter too aggressive
**Solution:**
- Increase P gain
- Decrease complementary filter alpha
- Check loop frequency is stable at 250Hz

### Problem: Drone Won't Arm
**Cause:** Safety checks failing
**Solution:**
- Ensure throttle is at minimum
- Move yaw stick fully right (for >1 second)
- Check receiver connections
- Verify receiver input values in Serial Monitor

---

## Advanced Tuning

### Adjusting the Complementary Filter
```cpp
#define COMPLEMENTARY_ALPHA 0.9996
```
- Higher (0.999+) = Trust gyro more (good for vibration)
- Lower (0.95) = Trust accelerometer more (faster settling)

### Battery Voltage Compensation
Uncomment in code to compensate for battery sag:
```cpp
if (battery_voltage < 11.0 && battery_voltage > 6.0) {
  motor_fl += (12.0 - battery_voltage) * 20;
  // ... same for other motors
}
```

### Loop Frequency
Default is 250Hz. Can increase to 500Hz for better response:
```cpp
#define LOOP_FREQUENCY 500
```
Note: Ensure loop completes in time - monitor via Serial.

### Expo/Rates for RC Input
Add exponential curve for better control feel:
```cpp
float applyExpo(float input, float expo) {
  return input * (1.0 - expo) + (input * input * input) * expo;
}
```

---

## Quick Reference: Tuning Direction

| Symptom | P | I | D |
|---------|---|---|---|
| Oscillation (fast) | ↓ | - | ↑ |
| Oscillation (slow) | ↓ | ↓ | - |
| Sluggish response | ↑ | - | - |
| Drifts over time | - | ↑ | - |
| Vibration/jitter | - | - | ↓ |
| Overshoot | ↓ | ↓ | ↑ |
| Doesn't return to level | - | ↑ | - |

---

## Safety Reminders

1. **ALWAYS remove propellers when tuning on the bench**
2. **Fly in a large open area, away from people**
3. **Have a spotter**
4. **Don't fly over your maximum skill level**
5. **Start with low throttle and conservative gains**
6. **If in doubt, disarm immediately**

Good luck with your tuning! Start conservative and make small changes.
