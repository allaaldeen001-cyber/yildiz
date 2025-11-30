# PID Tuning Guide

Professional guide for tuning PID controllers on your quadcopter drone.

---

## What is PID?

**PID** stands for **P**roportional, **I**ntegral, **D**erivative. It's a control algorithm that keeps your drone stable by continuously calculating error corrections.

### The Three Terms

1. **P (Proportional)**: Reacts to current error
   - Higher P = stronger reaction to tilting
   - Too high = oscillations
   - Too low = sluggish response

2. **I (Integral)**: Reacts to accumulated error over time
   - Higher I = corrects drift over time
   - Too high = overshooting and bouncing
   - Too low = drone drifts

3. **D (Derivative)**: Reacts to rate of error change
   - Higher D = dampens oscillations
   - Too high = jittery response
   - Too low = overshooting

---

## Default PID Values

The code comes with default values optimized for a standard 450mm quadcopter:

### Roll & Pitch PID
```cpp
Kp = 2.0
Ki = 0.02
Kd = 15.0
```

### Yaw PID
```cpp
Kp = 3.0
Ki = 0.02
Kd = 0.0
```

These values work for:
- **Frame Size**: 450mm
- **Total Weight**: 800-1200g
- **Motor KV**: 1000-2300
- **Props**: 10-12 inch

---

## When to Tune PID

Tune PID if you experience:
- ❌ Oscillations (shaking/wobbling)
- ❌ Sluggish response to controls
- ❌ Drone drifts in one direction
- ❌ Overshooting when leveling
- ❌ Different size/weight drone

---

## How to Modify PID Values

### Location in Code

Open `FlightController/FlightController.ino` and find these lines:

```cpp
// PID Tuning - Roll & Pitch
#define KP_ANGLE        2.0
#define KI_ANGLE        0.02
#define KD_ANGLE        15.0

// PID Tuning - Yaw
#define KP_YAW          3.0
#define KI_YAW          0.02
#define KD_YAW          0.0
```

Change these values, save, and re-upload to your Flight Controller.

---

## Step-by-Step Tuning Process

### Safety First!
- ⚠️ **Remove propellers during initial tuning**
- ⚠️ **Use a test stand or secure the drone**
- ⚠️ **Have kill switch ready (SW2)**
- ⚠️ **Start with small changes (±0.1)**

### Step 1: Start with Default Values

Upload code with default values and test fly in open area.

### Step 2: Tune P (Proportional)

**Goal**: Find minimum P that provides adequate response.

1. Start with default `KP_ANGLE = 2.0`
2. Fly and observe:
   - **Too High**: Oscillations (shaking)
   - **Too Low**: Sluggish, slow to level
3. Adjust:
   - If oscillating: **Decrease** by 0.2
   - If sluggish: **Increase** by 0.2
4. Repeat until smooth response without oscillations

**Typical Range**: 1.5 - 3.5

### Step 3: Tune D (Derivative)

**Goal**: Dampen oscillations from P term.

1. Start with default `KD_ANGLE = 15.0`
2. Fly and observe:
   - **Too High**: Jittery, vibrating
   - **Too Low**: Oscillations after movements
3. Adjust:
   - If jittery: **Decrease** by 2.0
   - If oscillating: **Increase** by 2.0
4. Find sweet spot that dampens without jitter

**Typical Range**: 10.0 - 25.0

### Step 4: Tune I (Integral)

**Goal**: Eliminate long-term drift.

1. Start with default `KI_ANGLE = 0.02`
2. Fly in no-wind conditions
3. Hover and observe:
   - **Too High**: Bouncing, overshooting
   - **Too Low**: Slowly drifts in one direction
4. Adjust:
   - If bouncing: **Decrease** by 0.005
   - If drifting: **Increase** by 0.005

**Typical Range**: 0.01 - 0.05

⚠️ **Note**: I term is most sensitive. Make very small changes!

### Step 5: Tune Yaw

Yaw tuning is separate:

1. Test yaw response (left joystick horizontal)
2. Adjust `KP_YAW`:
   - Increase if yaw is too slow
   - Decrease if yaw oscillates
3. `KD_YAW` is usually 0 (disabled)
4. Adjust `KI_YAW` if yaw drifts over time

**Typical Range**: 
- `KP_YAW`: 2.5 - 4.0
- `KI_YAW`: 0.01 - 0.03

---

## Tuning for Different Drone Sizes

### Small Drone (250mm)
```cpp
KP_ANGLE = 1.5
KI_ANGLE = 0.015
KD_ANGLE = 12.0

KP_YAW = 2.5
KI_YAW = 0.015
KD_YAW = 0.0
```

### Medium Drone (450mm) - DEFAULT
```cpp
KP_ANGLE = 2.0
KI_ANGLE = 0.02
KD_ANGLE = 15.0

KP_YAW = 3.0
KI_YAW = 0.02
KD_YAW = 0.0
```

### Large Drone (650mm+)
```cpp
KP_ANGLE = 2.8
KI_ANGLE = 0.03
KD_ANGLE = 20.0

KP_YAW = 3.5
KI_YAW = 0.025
KD_YAW = 0.0
```

---

## Advanced: PID Limits

You can also adjust maximum PID output:

```cpp
#define PID_ROLL_MAX    400
#define PID_PITCH_MAX   400
#define PID_YAW_MAX     400
```

- Increase for more aggressive corrections (racing drones)
- Decrease for smoother, gentler flight (aerial photography)

---

## Common Issues & Solutions

### Issue 1: Oscillations (Shaking)

**Symptoms**: Drone shakes rapidly, especially when hovering

**Solutions**:
1. Decrease `KP_ANGLE` by 0.2
2. Increase `KD_ANGLE` by 2.0
3. Check for:
   - Loose screws
   - Bent propellers
   - Vibrations from motors

### Issue 2: Sluggish Response

**Symptoms**: Drone is slow to respond to stick inputs

**Solutions**:
1. Increase `KP_ANGLE` by 0.2
2. Increase max rate values:
```cpp
#define MAX_ROLL_RATE   240.0  // was 180.0
#define MAX_PITCH_RATE  240.0
```

### Issue 3: Drifting

**Symptoms**: Drone slowly drifts in one direction even with centered sticks

**Solutions**:
1. Recalibrate gyro (Button 1)
2. Increase `KI_ANGLE` by 0.005
3. Check for:
   - Unbalanced propellers
   - Center of gravity offset
   - Wind conditions

### Issue 4: Overshooting

**Symptoms**: Drone goes past level position and bounces back

**Solutions**:
1. Decrease `KP_ANGLE` by 0.2
2. Increase `KD_ANGLE` by 2.0
3. Decrease `KI_ANGLE` by 0.005

### Issue 5: Toilet Bowl Effect

**Symptoms**: Drone circles in hover, getting wider over time

**Solutions**:
1. Decrease `KI_ANGLE` by 0.01
2. Reduce `KI_YAW` by 0.005
3. Recalibrate gyro on flat surface

---

## Tuning Workflow Diagram

```
START with default values
         ↓
    Test fly
         ↓
    Oscillations? ──YES→ Decrease P, Increase D
         ↓ NO
    Sluggish? ──YES→ Increase P
         ↓ NO
    Drifting? ──YES→ Increase I
         ↓ NO
    ✓ PERFECT!
```

---

## Data Logging for Advanced Tuning

For precise tuning, you can log PID data:

1. Add Serial output in the loop:
```cpp
Serial.print(imuData.angleX);
Serial.print(",");
Serial.print(pidRoll.lastError);
Serial.println();
```

2. Use Arduino Serial Plotter to visualize
3. Look for smooth curves without oscillations

---

## Safety Reminders

- ✅ Always test changes incrementally
- ✅ Keep a log of PID values that work
- ✅ Have a backup set of known-good values
- ✅ Test in calm wind conditions
- ✅ Start with short test flights (30 seconds)
- ✅ Increase test duration gradually

---

## PID Tuning Checklist

Before declaring PID tuned:

- [ ] Drone hovers steadily without oscillations
- [ ] Stick inputs produce smooth, predictable movements
- [ ] Drone returns to level quickly after stick release
- [ ] No drifting in calm wind
- [ ] Yaw rotations are smooth and controllable
- [ ] No overshooting when leveling
- [ ] Stable flight for 5+ minutes

---

## Further Reading

- PID Control Theory: https://en.wikipedia.org/wiki/PID_controller
- Betaflight PID Tuning: https://betaflight.com/docs/tuning/
- Quadcopter Dynamics: https://www.youtube.com/results?search_query=quadcopter+pid+tuning

---

**Last Updated**: 2025-11-30
