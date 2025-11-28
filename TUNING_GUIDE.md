# PID Tuning Guide for Quadcopter Flight Controller

## 📋 Table of Contents

- [Understanding the Control System](#understanding-the-control-system)
- [Default PID Values](#default-pid-values)
- [Tuning Philosophy](#tuning-philosophy)
- [Step-by-Step Tuning Procedure](#step-by-step-tuning-procedure)
- [Rate PID Tuning](#rate-pid-tuning)
- [Angle PID Tuning](#angle-pid-tuning)
- [Altitude PID Tuning](#altitude-pid-tuning)
- [Troubleshooting](#troubleshooting)
- [Advanced Tuning Tips](#advanced-tuning-tips)

---

## 🎯 Understanding the Control System

This flight controller uses a **cascade control architecture** with three main loops:

### 1. Rate Loop (Inner Loop - 250Hz)
- **Purpose**: Stabilize angular velocities (roll rate, pitch rate, yaw rate)
- **Input**: Desired angular rates (rad/s)
- **Feedback**: Gyroscope measurements
- **Output**: Motor commands
- **Characteristics**: Fast response, direct control authority

### 2. Angle Loop (Outer Loop - 100Hz)
- **Purpose**: Achieve desired tilt angles (roll, pitch)
- **Input**: Target angles from RC sticks (degrees)
- **Feedback**: Attitude from AHRS filter
- **Output**: Desired rates for inner loop
- **Characteristics**: Smooth tracking, self-leveling

### 3. Altitude Loop (25Hz)
- **Purpose**: Maintain target altitude
- **Input**: Target height (meters)
- **Feedback**: Kalman-filtered barometric altitude
- **Output**: Throttle adjustment
- **Characteristics**: Slower response, altitude hold mode

**Cascade Structure**:
```
RC Sticks → Angle PID → Rate PID → Motor Mixing → Motors
                ↑           ↑
              AHRS       Gyro
              
Barometer → Altitude PID → Climb Rate PID → Throttle Adjust
                ↑                 ↑
            Kalman Filter    Vertical Velocity
```

---

## 📊 Default PID Values

### Rate PIDs (FlightController_FC.ino - Lines ~156-159)

```cpp
// Roll Rate PID
PID_Coefficients rateRollPID = {
  .kp = 1.5,     // Proportional gain
  .ki = 0.05,    // Integral gain
  .kd = 0.01,    // Derivative gain
  .iMax = 100.0  // Integrator limit
};

// Pitch Rate PID
PID_Coefficients ratePitchPID = {
  .kp = 1.5,
  .ki = 0.05,
  .kd = 0.01,
  .iMax = 100.0
};

// Yaw Rate PID
PID_Coefficients rateYawPID = {
  .kp = 2.0,
  .ki = 0.1,
  .kd = 0.0,     // No derivative for yaw
  .iMax = 100.0
};
```

### Angle PIDs (Lines ~166-168)

```cpp
// Roll Angle PID
PID_Coefficients angleRollPID = {
  .kp = 3.5,
  .ki = 0.0,     // No integral for angle loop
  .kd = 0.0,     // No derivative (handled by rate loop)
  .iMax = 0.0
};

// Pitch Angle PID
PID_Coefficients anglePitchPID = {
  .kp = 3.5,
  .ki = 0.0,
  .kd = 0.0,
  .iMax = 0.0
};
```

### Altitude PIDs (Lines ~175-177)

```cpp
// Altitude PID (height → desired climb rate)
PID_Coefficients altitudePID = {
  .kp = 2.0,
  .ki = 0.5,
  .kd = 1.0,
  .iMax = 50.0
};

// Climb Rate PID (velocity → throttle adjustment)
PID_Coefficients climbRatePID = {
  .kp = 30.0,
  .ki = 5.0,
  .kd = 5.0,
  .iMax = 200.0
};
```

---

## 🧠 Tuning Philosophy

### General Principles

1. **Tune from inner to outer loop**: Rate PIDs first, then angle PIDs, then altitude PIDs
2. **Start conservative**: Begin with low gains, increase gradually
3. **One axis at a time**: Tune roll first, then copy to pitch, tune yaw separately
4. **Test incrementally**: Make small changes (10-20%), test, observe, repeat
5. **Safety first**: Always test with propellers OFF initially

### PID Term Functions

- **P (Proportional)**: 
  - Corrects error magnitude
  - Too high → oscillations
  - Too low → slow response, drifts
  
- **I (Integral)**:
  - Eliminates steady-state error
  - Too high → overshoot, oscillations
  - Too low → persistent drift
  
- **D (Derivative)**:
  - Dampens oscillations, predicts future error
  - Too high → noise amplification, jittery
  - Too low → overshoots, oscillations

---

## 🛠️ Step-by-Step Tuning Procedure

### Prerequisites

- ✅ Drone fully assembled, all components secure
- ✅ Propellers **REMOVED** (initial testing)
- ✅ IMU calibration complete (Button_1)
- ✅ ESC calibration complete (Button_2)
- ✅ Serial monitor connected to FC for debugging
- ✅ Battery fully charged

### Safety Setup

1. **Secure the drone**:
   - Option A: Remove propellers (safest for initial tuning)
   - Option B: Use propeller guards
   - Option C: Tether with string (for flight testing)

2. **Prepare workspace**:
   - Clear area, no obstacles
   - Soft landing surface (foam/grass)
   - Fire extinguisher nearby (LiPo safety)

3. **Monitoring**:
   - Keep RC in hand, thumb on SW_2 (kill switch)
   - Watch for smoke, unusual sounds, excessive heat

---

## 🎚️ Rate PID Tuning

### Goal
Achieve fast, stable response to angular rate commands without oscillations.

### Procedure

#### Step 1: Tune Rate P (Proportional)

1. **Set initial values**:
   ```cpp
   rateRollPID.kp = 1.0;
   rateRollPID.ki = 0.0;  // Disable integral
   rateRollPID.kd = 0.0;  // Disable derivative
   ```

2. **Test**:
   - Arm drone (propellers OFF)
   - Tilt drone by hand (roll axis)
   - Observe motor response in serial monitor
   - Motors should counter the tilt

3. **Increase Kp**:
   - Increase by 0.2-0.5 at a time
   - Repeat test
   - Continue until motors respond quickly and stop without oscillating
   - If oscillations occur, reduce Kp by 20%

4. **Target behavior**:
   - Quick correction
   - Stops at neutral position
   - No bouncing or oscillations

**Typical Range**: Kp = 1.0 - 3.0

#### Step 2: Tune Rate D (Derivative)

1. **Add derivative**:
   ```cpp
   rateRollPID.kd = 0.01;
   ```

2. **Test**:
   - Repeat tilt test
   - Kd should dampen any remaining oscillations

3. **Increase Kd** if:
   - Still oscillating slightly
   - Overshoots and bounces back

4. **Reduce Kd** if:
   - Response becomes sluggish
   - Jittery/twitchy behavior

**Typical Range**: Kd = 0.005 - 0.05

#### Step 3: Tune Rate I (Integral)

1. **Add integral**:
   ```cpp
   rateRollPID.ki = 0.02;
   ```

2. **Test with propellers ON** (low throttle):
   - Arm drone
   - Slowly increase throttle to just before liftoff
   - Hold drone gently, let it try to self-level
   - Release - should return to level without drift

3. **Increase Ki** if:
   - Drone drifts slowly in one direction
   - Doesn't fully correct to level

4. **Reduce Ki** if:
   - Overshoot and oscillations
   - Slow, large amplitude oscillations

**Typical Range**: Ki = 0.01 - 0.1

#### Step 4: Repeat for Pitch

Copy roll PID values to pitch (usually identical):
```cpp
ratePitchPID.kp = rateRollPID.kp;
ratePitchPID.ki = rateRollPID.ki;
ratePitchPID.kd = rateRollPID.kd;
```

#### Step 5: Tune Yaw Separately

Yaw typically needs higher P and I:
```cpp
rateYawPID.kp = 2.0;  // Higher than roll/pitch
rateYawPID.ki = 0.1;  // Higher Ki for heading hold
rateYawPID.kd = 0.0;  // Usually no D term
```

Test by spinning drone on yaw axis (hand-held).

---

## 🎯 Angle PID Tuning

### Goal
Smooth, predictable angle tracking from RC stick inputs.

### Procedure

#### Step 1: Set Conservative Values

```cpp
angleRollPID.kp = 2.0;
angleRollPID.ki = 0.0;  // Usually not needed
angleRollPID.kd = 0.0;  // Damping from rate loop
```

#### Step 2: Test with Propellers ON

1. **Hover test**:
   - Arm drone
   - Slowly increase throttle to lift off
   - Hover at 30cm height
   - Release sticks to center - should self-level

2. **Increase Kp** if:
   - Slow return to level
   - Drifts off angle
   - Sluggish response to stick inputs

3. **Reduce Kp** if:
   - Overshoots level position
   - Oscillates around level
   - Too aggressive/hard to control

**Typical Range**: Kp = 2.0 - 5.0

#### Step 3: Flight Test

1. Fly figure-8 patterns
2. Observe:
   - Does it track stick inputs smoothly?
   - Does it overshoot target angles?
   - Does it oscillate when releasing sticks?

3. Fine-tune Kp accordingly

#### Step 4: Copy to Pitch

```cpp
anglePitchPID.kp = angleRollPID.kp;
```

**Note**: Angle loop typically does NOT need Ki or Kd. Rate loop provides damping.

---

## 🎈 Altitude PID Tuning

### Goal
Stable altitude hold without oscillations or drift.

### Prerequisites

- ✅ Rate and angle PIDs tuned
- ✅ Stable hover achieved
- ✅ MS5611 barometer working (check serial output)

### Procedure

#### Step 1: Baseline Test

1. **Disable altitude hold**:
   - SW_1 = OFF
   - Manual throttle control

2. **Find hover throttle**:
   - Slowly increase throttle until hovering
   - Note throttle value (should be 1400-1600)
   - This is your hover point

3. **Verify barometer**:
   - Check FC serial output for altitude reading
   - Should be stable (±0.1m variation)

#### Step 2: Tune Altitude PID (Height → Velocity)

1. **Start conservative**:
   ```cpp
   altitudePID.kp = 1.0;
   altitudePID.ki = 0.1;
   altitudePID.kd = 0.5;
   ```

2. **Enable altitude hold**:
   - Hover at 1-2m height
   - Toggle SW_1 = ON
   - Observe behavior

3. **Increase Kp** if:
   - Altitude drifts up/down slowly
   - Slow correction to disturbances

4. **Reduce Kp** if:
   - Altitude oscillates (up-down bouncing)
   - Overcorrects to disturbances

5. **Add Ki**:
   - Eliminates steady drift
   - Start low (0.1), increase if drift persists

6. **Add Kd**:
   - Dampens altitude oscillations
   - Start with 0.5-1.0

**Typical Range**: Kp = 1.0 - 3.0, Ki = 0.1 - 1.0, Kd = 0.5 - 2.0

#### Step 3: Tune Climb Rate PID (Velocity → Throttle)

1. **Start conservative**:
   ```cpp
   climbRatePID.kp = 20.0;
   climbRatePID.ki = 2.0;
   climbRatePID.kd = 3.0;
   ```

2. **Test**:
   - Enable altitude hold
   - Push down on drone gently
   - Release - should return to target altitude

3. **Increase Kp** if:
   - Slow climb/descent response
   - Takes long time to correct altitude

4. **Reduce Kp** if:
   - Aggressive throttle changes
   - Altitude overshoots

5. **Tune Ki**:
   - Handles steady disturbances (wind, battery voltage drop)
   - Too high → oscillations

6. **Tune Kd**:
   - Smooths throttle response
   - Prevents aggressive climbs/descents

**Typical Range**: Kp = 20.0 - 50.0, Ki = 2.0 - 10.0, Kd = 3.0 - 10.0

#### Step 4: Integration Test

1. Enable altitude hold at 2m
2. Observe for 30 seconds:
   - ✅ Altitude stable (±0.2m)
   - ✅ No oscillations
   - ✅ No drift up/down

3. Push drone down 0.5m, release:
   - ✅ Returns to target altitude smoothly
   - ✅ No overshoot

4. Change throttle stick slightly:
   - ✅ Altitude adjusts proportionally
   - ✅ Returns to hold mode when stick centered

---

## 🔧 Troubleshooting

### Problem: High-Frequency Oscillations (Fast Vibrations)

**Symptoms**: Rapid shaking, buzzing sound

**Causes**:
- Rate P gain too high
- Rate D gain too high
- Mechanical vibrations amplified by PIDs

**Solutions**:
1. Reduce `rateRollPID.kp` and `ratePitchPID.kp` by 20%
2. Reduce `rateRollPID.kd` and `ratePitchPID.kd` by 50%
3. Check for loose components (motors, ESCs, frame)
4. Add vibration dampening (foam, rubber mounts)

---

### Problem: Low-Frequency Oscillations (Slow Wobbling)

**Symptoms**: Drone rocks back and forth slowly (~1 Hz)

**Causes**:
- Rate I gain too high
- Angle P gain too high
- Insufficient rate P gain

**Solutions**:
1. Reduce `rateRollPID.ki` and `ratePitchPID.ki` by 30%
2. Reduce `angleRollPID.kp` and `anglePitchPID.kp` by 20%
3. If still present, increase rate P slightly

---

### Problem: Drifts in One Direction

**Symptoms**: Drone slowly drifts roll/pitch despite level trim

**Causes**:
- IMU not calibrated correctly
- Rate I gain too low
- Mechanical imbalance (CG offset)

**Solutions**:
1. Recalibrate IMU (Button_1) on level surface
2. Increase `rateRollPID.ki` or `ratePitchPID.ki` slightly
3. Check CG (center of gravity) - should be centered
4. Check motor thrust balance

---

### Problem: Overshoots and Bounces Back

**Symptoms**: Tilt past target angle, overcorrect, repeat

**Causes**:
- Angle P gain too high
- Rate D gain too low

**Solutions**:
1. Reduce `angleRollPID.kp` by 15-20%
2. Increase `rateRollPID.kd` by 10-20%

---

### Problem: Sluggish Response

**Symptoms**: Slow to respond to stick inputs, lazy corrections

**Causes**:
- Rate P gain too low
- Angle P gain too low
- Excessive filtering (MPU6050 DLPF)

**Solutions**:
1. Increase `rateRollPID.kp` by 20%
2. Increase `angleRollPID.kp` by 20%
3. Check DLPF setting in MPU6050 config (44Hz is good balance)

---

### Problem: Altitude Oscillates (Up/Down Bouncing)

**Symptoms**: Altitude hold causes bouncing motion

**Causes**:
- Altitude P gain too high
- Climb rate P gain too high
- Barometer noise

**Solutions**:
1. Reduce `altitudePID.kp` by 30%
2. Reduce `climbRatePID.kp` by 20%
3. Check barometer placement (away from propwash)
4. Increase Kalman filter measurement noise `R` (line ~195)

---

### Problem: Altitude Drifts Up/Down

**Symptoms**: Slowly climbs or descends in altitude hold

**Causes**:
- Altitude I gain too low
- Climb rate I gain too low
- Barometer drift (temperature changes)

**Solutions**:
1. Increase `altitudePID.ki` slightly
2. Increase `climbRatePID.ki` slightly
3. Check base altitude (should update on ground before flight)

---

## 🚀 Advanced Tuning Tips

### 1. Iterative Tuning Method

Don't try to perfect each PID in one session. Use this cycle:

```
1. Get "good enough" rate PIDs (stable hover)
2. Tune angle PIDs (smooth flight)
3. Fly for 5-10 flights, observe behavior
4. Revisit rate PIDs with better understanding
5. Fine-tune all loops
6. Repeat
```

### 2. Data Logging (Optional)

If you add SD card logging to FC:

- Log gyro rates, desired rates, PID outputs
- Plot in Excel/Python after flight
- Identify overshoot, oscillations, lag
- Adjust PIDs based on data

### 3. Gain Scheduling (Advanced)

For aggressive pilots:

- Increase rate gains at high stick deflections
- Reduce gains near center stick (smooth control)
- Requires code modification (not included in base firmware)

### 4. Feed-Forward Control (Advanced)

- Add RC stick input directly to motor commands
- Reduces latency, improves responsiveness
- Requires careful tuning to avoid instability

### 5. Notch Filters (Advanced)

If persistent oscillations at specific frequency:

- Add digital notch filter to gyro data
- Targets specific resonance frequency
- Requires FFT analysis to identify frequency

### 6. Battery Voltage Compensation

As battery drains, motors lose power:

- Monitor battery voltage (analog pin)
- Increase PID outputs as voltage drops
- Maintains consistent response throughout flight

---

## 📝 Tuning Log Template

Keep a tuning log to track changes:

```
Date: ______
Flight #: ____
Battery: ____

Rate PIDs:
  Roll:  Kp=____ Ki=____ Kd=____
  Pitch: Kp=____ Ki=____ Kd=____
  Yaw:   Kp=____ Ki=____ Kd=____

Angle PIDs:
  Roll:  Kp=____ Ki=____ Kd=____
  Pitch: Kp=____ Ki=____ Kd=____

Observations:
- ____________________________________________
- ____________________________________________
- ____________________________________________

Changes for next flight:
- ____________________________________________
- ____________________________________________
```

---

## ✅ Final Validation

Before considering tuning complete, verify:

- ✅ Stable hover hands-off (no stick input)
- ✅ Smooth response to gentle stick inputs
- ✅ Quick recovery from aggressive maneuvers
- ✅ No oscillations at any throttle level
- ✅ Altitude hold stable for 30+ seconds
- ✅ No drift in any axis
- ✅ Comfortable to fly, predictable behavior

---

## 🎓 Summary

**Tuning Order**: Rate P → Rate D → Rate I → Angle P → Altitude PIDs

**Golden Rules**:
1. Start low, increase gradually
2. Test after every change
3. Safety first (always ready to disarm)
4. One parameter at a time
5. Log your changes

**Typical Tuning Time**: 5-10 hours of test flights

**Patience is key!** Proper PID tuning is an iterative process. Don't rush it.

---

**Good luck with your tuning! 🚁**
