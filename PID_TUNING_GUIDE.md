# 🎛️ PID TUNING GUIDE FOR QUADCOPTER

## Understanding PID Control

A PID (Proportional-Integral-Derivative) controller is the brain that keeps your drone stable. This guide will help you tune your quadcopter for perfect flight characteristics.

---

## 📊 What is PID?

### The Three Components

#### P - Proportional
- **What it does**: Reacts to current error
- **Effect**: Stronger correction for larger errors
- **Analogy**: Like pushing harder when further from target
- **Too high**: Oscillations and instability
- **Too low**: Sluggish, drifts easily

#### I - Integral
- **What it does**: Corrects persistent offset
- **Effect**: Eliminates steady-state error
- **Analogy**: Like adding more force if not reaching target
- **Too high**: Overshoot and slow oscillations
- **Too low**: Drone drifts slowly over time

#### D - Derivative
- **What it does**: Dampens rapid changes
- **Effect**: Smooths out corrections
- **Analogy**: Like applying brakes when approaching target
- **Too high**: Slow response, reduced corrections
- **Too low**: Overshoots and oscillations

---

## 🎯 Default PID Values

### Current Settings in Code

```cpp
// Roll and Pitch (Stabilization)
#define PID_ROLL_KP       1.5f
#define PID_ROLL_KI       0.05f
#define PID_ROLL_KD       18.0f

#define PID_PITCH_KP      1.5f
#define PID_PITCH_KI      0.05f
#define PID_PITCH_KD      18.0f

// Yaw (Rotation)
#define PID_YAW_KP        3.0f
#define PID_YAW_KI        0.02f
#define PID_YAW_KD        0.0f

// Altitude (Future Feature)
#define PID_ALT_KP        2.0f
#define PID_ALT_KI        0.1f
#define PID_ALT_KD        1.5f
```

These values are **conservative** and should fly well for most setups.

---

## 🔧 When to Tune PID

### Signs You Need Tuning

#### Oscillations (Vibrations)
- **Symptom**: Drone shakes or wobbles
- **Cause**: P or D gain too high
- **Fix**: Reduce P gain by 10-20%

#### Sluggish Response
- **Symptom**: Drone responds slowly to stick inputs
- **Cause**: P gain too low or D gain too high
- **Fix**: Increase P gain by 10-20%

#### Drift
- **Symptom**: Drone slowly drifts in one direction
- **Cause**: I gain too low or IMU calibration issue
- **Fix**: Increase I gain slightly or recalibrate IMU

#### Overshoot
- **Symptom**: Drone tips too far when correcting
- **Cause**: P too high or D too low
- **Fix**: Reduce P gain or increase D gain

---

## 📝 Tuning Process

### Step-by-Step Tuning

#### Preparation
1. **Remove propellers** for safety
2. **Secure drone** on soft surface or use test stand
3. **Open serial monitor** to see motor outputs
4. **Have kill switch ready**
5. **Make small changes** (10% at a time)

---

### Phase 1: Tune Roll Axis

#### Step 1: Start with P Gain
1. Set initial values:
   ```cpp
   PID_ROLL_KP = 0.5f
   PID_ROLL_KI = 0.0f
   PID_ROLL_KD = 0.0f
   ```

2. Upload code and arm drone

3. Apply roll input (tilt drone manually or use stick)

4. Increase P gain until drone responds quickly:
   ```cpp
   PID_ROLL_KP = 0.7f  // Test
   PID_ROLL_KP = 1.0f  // Test
   PID_ROLL_KP = 1.3f  // Test
   PID_ROLL_KP = 1.5f  // Test
   ```

5. **Stop when**:
   - Drone starts to oscillate (shake)
   - Reduce P by 20% from oscillation point
   - Example: Oscillates at 2.0, use 1.6

#### Step 2: Add D Gain
1. Start with D = P × 10:
   ```cpp
   PID_ROLL_KD = 16.0f  // If P = 1.6
   ```

2. Increase D gain until oscillations disappear:
   ```cpp
   PID_ROLL_KD = 18.0f  // Test
   PID_ROLL_KD = 20.0f  // Test
   PID_ROLL_KD = 22.0f  // Test
   ```

3. **Stop when**:
   - Drone responds smoothly
   - No oscillations or vibrations

#### Step 3: Add I Gain
1. Start with very small I value:
   ```cpp
   PID_ROLL_KI = 0.01f
   ```

2. Gradually increase:
   ```cpp
   PID_ROLL_KI = 0.03f  // Test
   PID_ROLL_KI = 0.05f  // Test
   PID_ROLL_KI = 0.08f  // Test
   ```

3. **Stop when**:
   - Drone holds level without drift
   - Before slow oscillations appear

#### Step 4: Test Flight
1. **Install propellers**
2. **Arm drone**
3. **Gradually increase throttle** to 50%
4. **Observe behavior**:
   - [ ] No oscillations
   - [ ] Responds quickly to roll stick
   - [ ] Returns to level when stick centered
   - [ ] No drift

5. **If problems**:
   - Oscillates: Reduce P by 10%
   - Sluggish: Increase P by 10%
   - Drifts: Increase I by 20%

---

### Phase 2: Tune Pitch Axis

**Repeat Phase 1 process for Pitch**

Use same values as Roll for symmetrical frame:
```cpp
PID_PITCH_KP = PID_ROLL_KP
PID_PITCH_KI = PID_ROLL_KI
PID_PITCH_KD = PID_ROLL_KD
```

If frame is asymmetrical (different weight distribution):
- Tune pitch independently
- Follow same steps as Roll tuning

---

### Phase 3: Tune Yaw Axis

#### Yaw is Different
- Uses **rate control** (not angle control)
- Usually higher P gain
- Little or no D gain needed
- Very low I gain

#### Tuning Steps
1. Start with P gain:
   ```cpp
   PID_YAW_KP = 2.0f
   PID_YAW_KI = 0.0f
   PID_YAW_KD = 0.0f
   ```

2. Increase P until drone rotates smoothly:
   ```cpp
   PID_YAW_KP = 3.0f  // Test
   PID_YAW_KP = 4.0f  // Test
   ```

3. Add small I gain to prevent drift:
   ```cpp
   PID_YAW_KI = 0.02f
   ```

4. Yaw usually doesn't need D gain:
   ```cpp
   PID_YAW_KD = 0.0f
   ```

5. **Test**:
   - [ ] Smooth rotation left/right
   - [ ] No twitching or oscillations
   - [ ] Holds heading when stick centered

---

## 🎮 Advanced Tuning

### Rate vs Level Mode

#### Level Mode (Current Implementation)
- Drone self-levels when sticks centered
- Uses angle as setpoint
- Easier for beginners
- More stable

#### Rate/Acro Mode (Advanced)
- No self-leveling
- Uses rotation rate as setpoint
- More responsive
- For acrobatics

**To implement Rate mode:**
```cpp
// Change in calculatePID():
// Instead of angle setpoint:
float desiredRollRate = map(receivedData.roll, 1000, 2000, -200, 200);
pidRoll.output = computePID(&pidRoll, desiredRollRate, rollRate, dt);
```

---

### PID Filtering

#### Low-Pass Filter for D Term
Reduces noise in derivative calculation:

```cpp
// Add to PIDController struct:
float lastFiltered;
float alpha;  // 0.7 = gentle filter

// In computePID():
float derivative = (error - pid->lastError) / dt;
derivative = pid->alpha * derivative + (1 - pid->alpha) * pid->lastFiltered;
pid->lastFiltered = derivative;
float D = pid->kD * derivative;
```

---

### Dynamic PID Scaling

#### Throttle-Based Scaling
Adjust PID gains based on throttle:

```cpp
// Higher throttle = more aerodynamic authority
float throttleScale = map(receivedData.throttle, 1000, 2000, 0.7, 1.3);
float scaledP = pidRoll.kP * throttleScale;
```

---

## 📈 Data Logging for Tuning

### What to Log
- Desired angle vs actual angle
- Motor outputs
- PID outputs (P, I, D components)
- Response time

### Add Logging to Code

```cpp
void printPIDData() {
  Serial.print(F("Roll: "));
  Serial.print(F("Desired=")); Serial.print(desiredRoll);
  Serial.print(F(" Actual=")); Serial.print(roll);
  Serial.print(F(" Error=")); Serial.print(desiredRoll - roll);
  Serial.print(F(" P=")); Serial.print(pidRoll.kP * (desiredRoll - roll));
  Serial.print(F(" I=")); Serial.print(pidRoll.kI * pidRoll.integral);
  Serial.print(F(" D=")); Serial.print(pidRoll.kD * (error - pidRoll.lastError) / dt);
  Serial.print(F(" Output=")); Serial.println(pidRoll.output);
}
```

---

## 🔍 Troubleshooting PID Issues

### Issue 1: Constant Oscillation
**Symptoms**: Drone shakes continuously
**Cause**: P gain too high
**Solution**:
```cpp
PID_ROLL_KP = PID_ROLL_KP * 0.7;  // Reduce by 30%
```

---

### Issue 2: Slow Oscillation (Bouncing)
**Symptoms**: Drone tips back and forth slowly
**Cause**: I gain too high
**Solution**:
```cpp
PID_ROLL_KI = PID_ROLL_KI * 0.5;  // Reduce by 50%
```

---

### Issue 3: High-Frequency Vibration
**Symptoms**: Fast buzzing/vibrating
**Causes**:
- D gain too high
- Motor noise
- Propeller imbalance

**Solutions**:
1. Reduce D gain:
   ```cpp
   PID_ROLL_KD = PID_ROLL_KD * 0.8;
   ```

2. Add D-term filtering (see Advanced Tuning)

3. Balance propellers

4. Check motor bearings

---

### Issue 4: Sluggish Response
**Symptoms**: Drone responds slowly to inputs
**Cause**: P gain too low or D gain too high
**Solution**:
```cpp
PID_ROLL_KP = PID_ROLL_KP * 1.2;  // Increase by 20%
// OR
PID_ROLL_KD = PID_ROLL_KD * 0.8;  // Reduce D by 20%
```

---

### Issue 5: Toilet Bowl Effect (Spiraling)
**Symptoms**: Drone slowly spirals when hovering
**Causes**:
- I gain too high
- IMU drift
- Compass interference (if using magnetometer)

**Solutions**:
1. Reduce I gain
2. Recalibrate IMU
3. Keep electronics away from motors

---

### Issue 6: Flip on Takeoff
**Symptoms**: Drone immediately flips over
**Causes**:
- Wrong motor directions
- Wrong propeller positions
- Inverted PID outputs

**Solutions**:
1. Check motor rotation (see motor test)
2. Verify propeller CW/CCW
3. Check motor mixing formula

---

## 🎯 Fine-Tuning for Flight Styles

### Smooth/Cinematic Flying
```cpp
// Lower P and D for smooth movements
PID_ROLL_KP = 1.0f;
PID_ROLL_KD = 12.0f;
PID_ROLL_KI = 0.08f;  // Higher I for drift correction
```

### Aggressive/Sport Flying
```cpp
// Higher P and D for fast response
PID_ROLL_KP = 2.5f;
PID_ROLL_KD = 25.0f;
PID_ROLL_KI = 0.03f;  // Lower I to prevent overshoot
```

### Windy Conditions
```cpp
// Higher I to fight wind
PID_ROLL_KI = 0.12f;
// Slightly higher P
PID_ROLL_KP = 1.8f;
```

---

## 📊 PID Value Examples

### Example 1: 450mm Frame, 1000KV Motors
```cpp
#define PID_ROLL_KP       1.5f
#define PID_ROLL_KI       0.05f
#define PID_ROLL_KD       18.0f
#define PID_YAW_KP        3.0f
#define PID_YAW_KI        0.02f
#define PID_YAW_KD        0.0f
```

### Example 2: 250mm Frame, 2300KV Motors (Racing)
```cpp
#define PID_ROLL_KP       2.8f
#define PID_ROLL_KI       0.08f
#define PID_ROLL_KD       28.0f
#define PID_YAW_KP        5.0f
#define PID_YAW_KI        0.05f
#define PID_YAW_KD        0.0f
```

### Example 3: 650mm Frame, 700KV Motors (Heavy Lift)
```cpp
#define PID_ROLL_KP       0.8f
#define PID_ROLL_KI       0.03f
#define PID_ROLL_KD       12.0f
#define PID_YAW_KP        2.0f
#define PID_YAW_KI        0.01f
#define PID_YAW_KD        0.0f
```

---

## 🛠️ Tools for Tuning

### Serial Monitor Method (Current)
- Print PID values to serial
- Adjust in code
- Re-upload firmware
- Simple but slow

### Better Method: Real-Time Tuning
Add serial commands to adjust PIDs without re-uploading:

```cpp
void handleSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    
    if (cmd.startsWith("RP=")) {
      pidRoll.kP = cmd.substring(3).toFloat();
      Serial.print(F("Roll P set to: ")); Serial.println(pidRoll.kP);
    }
    else if (cmd.startsWith("RI=")) {
      pidRoll.kI = cmd.substring(3).toFloat();
    }
    else if (cmd.startsWith("RD=")) {
      pidRoll.kD = cmd.substring(3).toFloat();
    }
    // Add similar for pitch and yaw
  }
}
```

**Usage**: Type `RP=1.8` in serial monitor to set Roll P to 1.8

---

## 📝 PID Tuning Checklist

### Before Tuning
- [ ] Drone mechanically sound
- [ ] Motors spinning correctly
- [ ] Propellers balanced
- [ ] IMU calibrated properly
- [ ] Battery fully charged
- [ ] Safe environment for testing

### During Tuning
- [ ] Start with P gain only
- [ ] Make small changes (10-20%)
- [ ] Test one axis at a time
- [ ] Document all changes
- [ ] Keep kill switch ready

### After Tuning
- [ ] Verify all axes stable
- [ ] Test in different conditions
- [ ] Save PID values
- [ ] Document final settings
- [ ] Backup code

---

## 🎓 Learning Resources

### Understanding PID
- PID Theory: https://en.wikipedia.org/wiki/PID_controller
- Multirotor PID: https://oscarliang.com/quadcopter-pid-explained-tuning/

### Video Tutorials
- Joshua Bardwell (YouTube): PID tuning series
- Quadcopter 101 (YouTube): Flight controller tuning

---

## 💡 Pro Tips

1. **One change at a time**: Never adjust multiple PIDs simultaneously
2. **Keep notes**: Document every change and result
3. **Start conservative**: It's easier to add gains than recover from a crash
4. **Weather matters**: Wind affects tuning significantly
5. **Battery voltage**: PID performance changes with battery charge
6. **Propeller condition**: Worn props affect PID response
7. **Test incrementally**: Don't jump straight to aggressive values
8. **Use test stand**: Safer than mid-air tuning
9. **Video review**: Record flights to analyze behavior
10. **Community help**: Share data on RC forums for advice

---

## ⚠️ Safety Reminders

- **Always test without propellers first**
- **Keep kill switch accessible**
- **Small changes = safer tuning**
- **Crash = reset to safe values**
- **Never tune near people or obstacles**

---

## 🎯 Goal: Perfect PID

**You've achieved good PID tuning when:**
- Drone hovers rock-solid with no drift
- Responds quickly to stick inputs
- Returns to level smoothly
- No oscillations or vibrations
- Flies predictably in light wind
- You feel confident in control

**Happy tuning! 🎮🚁**

---

**Document Version**: 1.0.0
**Last Updated**: November 2025
