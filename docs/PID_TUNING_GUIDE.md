# PID Tuning Guide for Drone Flight Controller

## 📚 PID Control Basics

### What is PID?

**PID** stands for **Proportional-Integral-Derivative** control. It's a feedback control system that:
- Measures the error (difference between desired and actual state)
- Calculates a correction using three terms
- Applies the correction to minimize error

### PID Formula
```
Output = (Kp × Error) + (Ki × ∫Error·dt) + (Kd × dError/dt)
```

Where:
- **Kp** (Proportional): Immediate response to current error
- **Ki** (Integral): Corrects accumulated error over time
- **Kd** (Derivative): Dampens rapid changes (predicts future error)

---

## 🎯 Understanding Each Term

### Proportional (P)
- **Effect**: Direct response to error
- **Too High**: Oscillations, instability
- **Too Low**: Sluggish response, won't reach setpoint
- **Analogy**: Steering wheel sensitivity

**Symptoms:**
- ↑ P too high → Rapid oscillations/vibrations
- ↓ P too low → Slow, mushy response

---

### Integral (I)
- **Effect**: Eliminates steady-state error (drift)
- **Too High**: Overshooting, slow oscillations
- **Too Low**: Drone drifts slowly over time
- **Analogy**: Automatic trim adjustment

**Symptoms:**
- ↑ I too high → Overshoots target, wobbles
- ↓ I too low → Drifts with wind, doesn't hold position

⚠️ **Warning**: I-term can cause instability if not limited (anti-windup)

---

### Derivative (D)
- **Effect**: Dampens oscillations, smooths response
- **Too High**: Jittery, noisy (amplifies sensor noise)
- **Too Low**: Overshoots, bounces
- **Analogy**: Shock absorber

**Symptoms:**
- ↑ D too high → Twitchy, nervous behavior
- ↓ D too low → Bouncy, overshoots

---

## 🛠️ Tuning Process

### Safety First!
✓ Remove propellers for initial tests
✓ Secure drone to prevent flyaways
✓ Test in open area away from obstacles
✓ Start with low throttle (hover only)
✓ Have emergency kill switch ready (SW_2)

---

## 📋 Step-by-Step Tuning

### Method: Ziegler-Nichols (Modified for Quadcopters)

#### Step 0: Reset to Safe Defaults
```cpp
// In FlightController.ino:
#define KP_ROLL   1.0
#define KI_ROLL   0.0   // Start with zero
#define KD_ROLL   0.0   // Start with zero

#define KP_PITCH  1.0
#define KI_PITCH  0.0
#define KD_PITCH  0.0

#define KP_YAW    2.0
#define KI_YAW    0.0
#define KD_YAW    0.0
```

---

### Step 1: Tune Proportional (P)

**Goal**: Find maximum P before oscillations start

**Procedure:**
1. Set I = 0, D = 0
2. Set P = 1.0
3. Arm drone, increase throttle to hover
4. Observe behavior:
   - Stable? → Increase P by 0.2
   - Oscillating? → Decrease P by 0.2
5. Repeat until drone **barely** oscillates
6. **Reduce P by 20%** for safety margin

**Example:**
```
Test 1: P=1.0 → Sluggish → Increase
Test 2: P=1.5 → Better → Increase
Test 3: P=2.0 → Good → Increase
Test 4: P=2.5 → Slight oscillation → Decrease
Test 5: P=2.2 → Perfect
Final: P=1.8 (2.2 × 0.8 for margin)
```

**Optimal P Value**: Typically **1.5 - 3.0** for most drones

---

### Step 2: Tune Derivative (D)

**Goal**: Dampen oscillations and improve smoothness

**Procedure:**
1. Keep P from Step 1
2. Set D = P × 10 (starting point)
3. Test flight:
   - Smooth? → Done with D
   - Still oscillates? → Increase D by 20%
   - Too stiff/jittery? → Decrease D by 20%
4. Find sweet spot

**Example:**
```
P = 1.8 (from Step 1)
Test 1: D = 18.0 → Good dampening
Test 2: D = 22.0 → Smoother
Test 3: D = 26.0 → Too stiff
Final: D = 22.0
```

**Optimal D Value**: Typically **10-20× P value**

---

### Step 3: Tune Integral (I)

**Goal**: Eliminate drift without causing overshoot

**Procedure:**
1. Keep P and D from Steps 1-2
2. Start with I = 0.01
3. Hover drone for 30+ seconds
4. Observe:
   - Drifts slowly? → Increase I
   - Overshoots after movement? → Decrease I
   - Wobbles slowly? → Decrease I
5. Very small adjustments (±0.005)

**Example:**
```
Test 1: I = 0.01 → Still drifts
Test 2: I = 0.03 → Reduced drift
Test 3: I = 0.05 → Perfect, no drift
Test 4: I = 0.08 → Overshoots
Final: I = 0.05
```

**Optimal I Value**: Typically **0.02 - 0.10**

⚠️ **Caution**: I-term can cause instability. Use sparingly!

---

### Step 4: Fine-Tuning

**Test Maneuvers:**
1. **Hover Test**: 60 seconds stable hover
2. **Pitch Test**: Quick forward/backward
3. **Roll Test**: Quick left/right
4. **Yaw Test**: 360° rotation
5. **Figure-8**: Smooth continuous movement
6. **Altitude Hold**: If using barometer

**Adjust based on:**
- Still oscillates? → Increase D or decrease P
- Slow response? → Increase P
- Drifts over time? → Increase I (carefully)
- Overshoots? → Increase D or decrease P

---

## 🎚️ Recommended Starting Values

### Conservative (Stable, Safe)
```cpp
#define KP_ROLL   1.0
#define KI_ROLL   0.02
#define KD_ROLL   10.0

#define KP_PITCH  1.0
#define KI_PITCH  0.02
#define KD_PITCH  10.0

#define KP_YAW    2.0
#define KI_YAW    0.01
#define KD_YAW    0.0
```
**Use for**: First flights, testing, beginners

---

### Default (Balanced)
```cpp
#define KP_ROLL   1.5
#define KI_ROLL   0.05
#define KD_ROLL   15.0

#define KP_PITCH  1.5
#define KI_PITCH  0.05
#define KD_PITCH  15.0

#define KP_YAW    3.0
#define KI_YAW    0.02
#define KD_YAW    0.0
```
**Use for**: Normal flying, general purpose

---

### Aggressive (Responsive)
```cpp
#define KP_ROLL   2.5
#define KI_ROLL   0.08
#define KD_ROLL   25.0

#define KP_PITCH  2.5
#define KI_PITCH  0.08
#define KD_PITCH  25.0

#define KP_YAW    4.0
#define KI_YAW    0.03
#define KD_YAW    0.0
```
**Use for**: Experienced pilots, racing, acrobatics

---

## 🔍 Troubleshooting PID Issues

### Problem: Rapid Oscillations (High Frequency)

**Cause**: P too high or D too low

**Solution:**
```cpp
KP_ROLL -= 0.3;  // Decrease P
KD_ROLL += 5.0;  // Increase D
```

**Visual**: Drone vibrates/shakes rapidly (>5 Hz)

---

### Problem: Slow Wobbles (Low Frequency)

**Cause**: I too high

**Solution:**
```cpp
KI_ROLL -= 0.02;  // Decrease I
```

**Visual**: Drone sways slowly back and forth (<1 Hz)

---

### Problem: Overshooting

**Cause**: P too high, D too low, or I too high

**Solution:**
```cpp
KP_ROLL -= 0.2;  // Decrease P
KD_ROLL += 3.0;  // Increase D
KI_ROLL -= 0.01; // Decrease I
```

**Visual**: Drone tilts past desired angle then corrects

---

### Problem: Sluggish Response

**Cause**: P too low

**Solution:**
```cpp
KP_ROLL += 0.3;  // Increase P
```

**Visual**: Drone feels "mushy", slow to respond to stick input

---

### Problem: Drift (slowly moves in one direction)

**Cause**: I too low or gyro calibration issue

**Solution:**
```cpp
KI_ROLL += 0.01;  // Increase I
```

Or re-calibrate gyro on level surface.

**Visual**: Drone slowly drifts left/right/forward/backward

---

### Problem: Jittery, Nervous Behavior

**Cause**: D too high (amplifying sensor noise)

**Solution:**
```cpp
KD_ROLL -= 5.0;  // Decrease D
```

**Visual**: Motors constantly adjusting, twitchy

---

## 📐 Altitude Hold PID Tuning

Altitude hold uses barometric pressure sensor (MS5611).

### Starting Values
```cpp
#define KP_ALT 2.0
#define KI_ALT 0.1
#define KD_ALT 1.5
```

### Tuning Altitude Hold

**Step 1: Test Current Settings**
- Enable altitude hold at 2m height
- Release throttle stick
- Observe:
  - Maintains altitude? → Good
  - Drifts up/down slowly? → Need more I
  - Oscillates up/down? → Too much P or I

**Step 2: Adjust P**
- Too low: Drifts significantly, slow to correct
- Too high: Bounces up and down

**Step 3: Adjust I**
- Eliminates long-term drift
- Be careful: too much causes instability

**Step 4: Adjust D**
- Smooths altitude changes
- Reduces bouncing

**Optimal Ranges:**
- KP_ALT: 1.5 - 3.0
- KI_ALT: 0.05 - 0.2
- KD_ALT: 1.0 - 3.0

---

## 🎯 Advanced Tuning Tips

### 1. Tune Roll and Pitch Separately
Although usually identical, your drone may be asymmetric:
```cpp
#define KP_ROLL   1.5
#define KP_PITCH  1.7  // Slightly different if needed
```

### 2. Yaw Usually Needs Different Values
Yaw dynamics are different from roll/pitch:
- Higher P (2-4x)
- Lower or zero D
- Very low I

### 3. Check Complementary Filter
Affects angle estimation:
```cpp
// In calculateAngles():
angleRoll = 0.98 * (gyro) + 0.02 * (accel);
```

Adjust ratio:
- More gyro (0.99/0.01): Less drift, more noise
- More accel (0.96/0.04): More drift correction, slower response

### 4. Loop Timing Matters
Current code: 250 Hz (4ms loop time)
- Faster loop → Can use higher PID gains
- Slower loop → Need lower PID gains

### 5. Anti-Windup Protection
Already implemented:
```cpp
rollErrorSum = constrain(rollErrorSum, -400, 400);
```

Prevents integral term from growing too large.

### 6. Battery Voltage Compensation (Advanced)
Motor response changes as battery drains. Advanced users can add:
```cpp
float batteryFactor = batteryVoltage / 11.1; // Normalize to 3S nominal
baseThrottle *= batteryFactor;
```

---

## 📊 PID Tuning Cheat Sheet

| Symptom | Probable Cause | Action |
|---------|----------------|--------|
| Fast oscillations (buzz) | P too high | Decrease P by 0.2-0.5 |
| Slow wobbles | I too high | Decrease I by 0.01-0.02 |
| Overshooting | P high, D low | Decrease P, increase D |
| Sluggish, mushy | P too low | Increase P by 0.2-0.5 |
| Drifts over time | I too low | Increase I by 0.01-0.02 |
| Jittery, twitchy | D too high | Decrease D by 2-5 |
| Bouncy landing | D too low | Increase D by 2-5 |
| Won't hold angle | P too low | Increase P |

---

## 🧪 Testing Procedure

### Pre-Flight PID Test
1. **Bench Test (No Props)**:
   - Tilt drone by hand
   - Motors should spin to correct
   - Response should be smooth, not jerky

2. **Tethered Test**:
   - Secure drone with string
   - Arm and slowly increase throttle
   - Observe stability before full flight

3. **Hover Test**:
   - Fly 1m high, release sticks
   - Should hover relatively stable
   - Minor drift is acceptable

4. **Response Test**:
   - Quick stick input then release
   - Should return to level quickly
   - No overshooting or oscillations

### Flight Test Checklist
- [ ] Stable hover for 60 seconds
- [ ] Quick pitch forward/back
- [ ] Quick roll left/right
- [ ] 360° yaw rotation
- [ ] Figure-8 pattern
- [ ] Altitude changes
- [ ] Emergency stop test

---

## 📝 Logging PID Changes

Keep a tuning log:

```
Date: 2025-11-29
Test #3
Changes: Increased KP_ROLL from 1.5 to 1.8
Result: Better response, slight oscillation at high throttle
Next: Try KD_ROLL = 18 (was 15)
Battery: 3S 2200mAh at 80%
Weather: Calm, indoors
```

This helps track what works and what doesn't.

---

## 🎓 Learning Resources

### Understanding PID:
- **Video**: "PID Controller Explained" (YouTube)
- **Interactive**: PID Simulator tools online
- **Book**: "PID Control System Design" (basics)

### Drone-Specific:
- **Betaflight PID Tuning**: Similar concepts
- **ArduPilot Tuning Guide**: More advanced
- **RCGroups Forums**: Real-world experiences

---

## ⚠️ Safety Reminders

1. **Always start conservative** - Can always increase gains
2. **Test incrementally** - Small changes, one at a time
3. **Use kill switch** - Have SW_2 ready for emergency disarm
4. **Open area** - Test away from obstacles and people
5. **Backup values** - Note working PID values before changing
6. **Vibration check** - Ensure drone is mechanically sound first
7. **Propeller balance** - Unbalanced props cause oscillations
8. **Fresh battery** - Low battery affects performance

---

## 🎯 Quick Start for Impatient Users

**Want to skip tuning? Use these proven values:**

```cpp
// Copy these to FlightController.ino:
#define KP_ROLL   1.5
#define KI_ROLL   0.05
#define KD_ROLL   15.0

#define KP_PITCH  1.5
#define KI_PITCH  0.05
#define KD_PITCH  15.0

#define KP_YAW    3.0
#define KI_YAW    0.02
#define KD_YAW    0.0

#define KP_ALT    2.0
#define KI_ALT    0.1
#define KD_ALT    1.5
```

**Should work for:**
- 250-450mm quadcopter frames
- 1000-2200kV motors
- 5-6 inch propellers
- 3S-4S LiPo batteries

**May need tuning if:**
- Larger/smaller frame
- Different motor/prop combination
- Heavy payload (camera, etc.)
- Different center of gravity

---

**Good luck with tuning! Patience and incremental changes = success!** 🚁
