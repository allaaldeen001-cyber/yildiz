# PID Tuning Guide for Yildiz Drone

## Understanding PID Control

PID (Proportional-Integral-Derivative) control keeps your drone stable by correcting errors.

### P (Proportional)
- **What it does:** Reacts to current error
- **Effect:** Larger error = stronger correction
- **Too high:** Oscillations, bouncing
- **Too low:** Slow response, drifting

### I (Integral)
- **What it does:** Eliminates steady-state error
- **Effect:** Accumulates error over time
- **Too high:** Slow oscillations, overshooting
- **Too low:** Constant drift, wind compensation fails

### D (Derivative)
- **What it does:** Dampens rapid changes
- **Effect:** Predicts future error
- **Too high:** Jittery, noisy, hot motors
- **Too low:** Overshooting, bouncing

## Step-by-Step Tuning Process

### Step 1: Safety First

1. **Remove propellers** for initial testing
2. Secure drone to prevent movement
3. Have arm switch easily accessible
4. Keep battery charged (low voltage affects performance)

### Step 2: Reset PID Values

Start with conservative values:
```cpp
pidRollP = 0.5;
pidRollI = 0.0;
pidRollD = 0.0;

pidPitchP = 0.5;
pidPitchI = 0.0;
pidPitchD = 0.0;

pidYawP = 1.0;
pidYawI = 0.0;
pidYawD = 0.0;
```

### Step 3: Tune P Gain

1. **Start with Roll P = 0.5**
2. Arm drone (motors should spin slowly)
3. Tilt drone manually (props off!)
4. Motors should react to correct the tilt
5. **Increase P by 0.2** until you see:
   - Motors respond strongly
   - Slight oscillations when released
6. **Reduce P by 20-30%** from oscillation point
7. Repeat for Pitch P

**Example progression:**
- P=0.5: Too slow
- P=1.0: Better
- P=1.5: Good response
- P=2.0: Starting to oscillate ← TOO HIGH!
- **Final: P=1.4** (30% below oscillation)

### Step 4: Tune D Gain

1. Keep P at value from Step 3
2. **Start with D = P × 10** (e.g., if P=1.4, try D=14)
3. Arm and tilt drone
4. **Increase D** until oscillations are damped
5. Too much D = jittery/noisy
6. Find sweet spot where tilt response is smooth

**Example progression:**
- D=0: Bouncy after tilt
- D=10: Still bouncy
- D=15: Smoother
- D=20: Very smooth ← GOOD!
- D=25: Jittery ← TOO HIGH!
- **Final: D=18**

### Step 5: Test Flight (Hover)

1. **Install propellers** (correct direction!)
2. Check motor directions again
3. Find open area (grass recommended)
4. Arm and slowly increase throttle
5. Hover at 1-2 feet

**What to look for:**
- ✅ Stable hover = Good tune!
- ⚠️ Oscillations = Reduce P or increase D
- ⚠️ Drifts = Need I term
- ⚠️ Very unstable = Reduce all gains by 50%

### Step 6: Tune I Gain

⚠️ **Only add I if drone drifts during hover**

1. **Start with I = P × 0.02** (e.g., if P=1.4, try I=0.03)
2. Hover and observe
3. I eliminates constant drift
4. **Increase slowly** (0.01 at a time)
5. Too much I = slow wobbles

**Example:**
- I=0: Constant drift to one side
- I=0.02: Less drift
- I=0.04: No drift ← GOOD!
- I=0.08: Slow oscillations ← TOO HIGH!
- **Final: I=0.04**

### Step 7: Fine Tuning

After successful hover:

1. Try aggressive movements
2. Punch throttle up/down
3. Roll/pitch quickly
4. Observe behavior:
   - Oscillates? → Reduce P or increase D
   - Overshoots? → Increase D or reduce I
   - Sluggish? → Increase P
   - Drifts? → Increase I

## Recommended Starting Values

### Lightweight Drone (250mm, <500g)
```cpp
pidRollP = 1.0;
pidRollI = 0.03;
pidRollD = 15.0;

pidPitchP = 1.0;
pidPitchI = 0.03;
pidPitchD = 15.0;

pidYawP = 2.0;
pidYawI = 0.02;
pidYawD = 0.0;
```

### Medium Drone (450mm, 500-1000g)
```cpp
pidRollP = 1.3;
pidRollI = 0.04;
pidRollD = 18.0;

pidPitchP = 1.3;
pidPitchI = 0.04;
pidPitchD = 18.0;

pidYawP = 2.5;
pidYawI = 0.03;
pidYawD = 0.0;
```

### Heavy Drone (>1000g)
```cpp
pidRollP = 1.5;
pidRollI = 0.05;
pidRollD = 20.0;

pidPitchP = 1.5;
pidPitchI = 0.05;
pidPitchD = 20.0;

pidYawP = 3.0;
pidYawI = 0.04;
pidYawD = 0.0;
```

## Common Problems and Solutions

### Problem: Rapid Oscillations (Bouncing)
**Cause:** P too high  
**Solution:**
1. Reduce P by 30%
2. If still oscillating, reduce D by 20%
3. Retest

### Problem: Slow Oscillations (Wobbling)
**Cause:** I too high  
**Solution:**
1. Reduce I by 50%
2. If no I term, reduce P by 20%

### Problem: Jittery/Noisy
**Cause:** D too high  
**Solution:**
1. Reduce D by 30%
2. Check motor/prop balance
3. Check frame rigidity

### Problem: Drifts in One Direction
**Cause:** I too low or motor imbalance  
**Solution:**
1. Increase I by 0.01
2. Check motor thrust (swap motors)
3. Check prop condition

### Problem: Sluggish Response
**Cause:** P too low  
**Solution:**
1. Increase P by 20%
2. Test for oscillations
3. Adjust D if needed

### Problem: Tilts After Quick Movements
**Cause:** D too low  
**Solution:**
1. Increase D by 20%
2. Check for overshooting
3. Balance with P

## Advanced Tuning Tips

### Complementary Filter Tuning

Located in `calculateAngles()`:
```cpp
// Default: 98% gyro, 2% accelerometer
angleX = angleX * 0.98 + accelAngleX * 0.02;
```

- **More gyro** (0.99/0.01): Faster response, drifts over time
- **More accel** (0.95/0.05): More stable, sluggish
- **Default (0.98/0.02)**: Good balance for most drones

### Loop Rate Optimization

Default: 250Hz (4ms loop)
```cpp
while (micros() - loopTimer < 4000);  // 250Hz
```

- **Faster** (500Hz): Better response, more processing
- **Slower** (125Hz): Less processing, may be unstable
- **Recommended:** Keep at 250Hz

### Rate Limits

Adjust max angles for your comfort:
```cpp
// Conservative: ±20°
pidRollSetpoint = (controlData.roll - 1500) * 20.0 / 500.0;

// Default: ±30°
pidRollSetpoint = (controlData.roll - 1500) * 30.0 / 500.0;

// Aggressive: ±45°
pidRollSetpoint = (controlData.roll - 1500) * 45.0 / 500.0;
```

## Tuning Checklist

Before each tuning session:

- [ ] Battery fully charged
- [ ] Propellers balanced
- [ ] Motors spinning freely
- [ ] Frame tight (no loose parts)
- [ ] Gyro calibrated (on level surface)
- [ ] Safe flying area selected
- [ ] Spare props available
- [ ] Fire extinguisher nearby (LiPo safety)

## Logging for Analysis

Add to loop for detailed logging:
```cpp
if (armed) {
  Serial.print(angleX); Serial.print(",");
  Serial.print(pidRollSetpoint); Serial.print(",");
  Serial.print(pidRollOutput); Serial.print(",");
  Serial.println(motor1Speed);
}
```

Use Arduino Serial Plotter to visualize!

## When to Stop Tuning

✅ You're done when:
- Drone hovers stable without input
- Returns to level quickly when tilted
- No oscillations or wobbles
- Responds smoothly to stick inputs
- Can handle wind gusts

⚠️ Don't over-tune:
- "Good enough" is better than perfect
- Over-tuning can make it worse
- Some environmental factors can't be tuned out

## Final Tips

1. **Tune incrementally** - Small changes (10-20% at a time)
2. **Test thoroughly** - 30 seconds hover minimum per change
3. **Document values** - Write down what works
4. **Weather matters** - Wind requires higher gains
5. **Battery voltage** - Performance changes as battery drains
6. **Props condition** - Replace damaged props immediately
7. **Take breaks** - Frustration leads to mistakes
8. **Be patient** - Good tuning takes time

---

**Happy tuning and safe flights! 🚁**
