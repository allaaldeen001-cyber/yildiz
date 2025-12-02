# 🛬 Smooth Automatic Landing System

## Overview

This landing system implements a **state machine-based controlled descent** with MS5611 barometer feedback, touchdown detection, and safe motor cutoff sequence.

---

## Landing State Machine

```
┌─────────────────┐
│   LANDING_IDLE  │  (Normal flight)
└────────┬────────┘
         │ Button 3 pressed
         ▼
┌─────────────────┐
│   INITIATED     │  (Record start altitude, set ground ref)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│   DESCENDING    │  (Smooth S-curve descent profile)
└────────┬────────┘  - Descent rate: 0 → 50 cm/s (smooth ramp)
         │            - Max rate: 50 cm/s
         │            - Barometer feedback controls throttle
         │
         │ Altitude < 50cm
         ▼
┌─────────────────┐
│  NEAR_GROUND    │  (Extra caution)
└────────┬────────┘  - Descent rate: 15 cm/s (30% of max)
         │            - Tight angle limits (±15°)
         │            - Increased sensor monitoring
         │
         │ TOUCHDOWN DETECTED
         ▼
┌─────────────────┐
│   TOUCHDOWN     │  (Contact confirmed)
└────────┬────────┘  - Motors: LANDING_SAFE_IDLE (1050µs)
         │            - Attitude control still active
         │            - Wait 1 second for stability
         │
         ▼
┌─────────────────┐
│   SAFE_IDLE     │  (Minimal thrust)
└────────┬────────┘  - Motors: 1000-1050µs
         │            - Wait 0.5 seconds
         │
         ▼
┌─────────────────┐
│    COMPLETE     │  (Disarm & stop)
└─────────────────┘  - Motors: OFF (1000µs)
                      - Armed: FALSE
                      - Return to IDLE state
```

---

## Touchdown Detection Criteria

The system detects touchdown using **multiple sensors** for reliability:

### Primary Criteria (ALL must be true):
1. **Altitude below threshold**: `altitude < 15cm` (above ground level)
2. **Low vertical velocity**: `|velocity| < 10 cm/s` (not falling fast)

### Secondary Criteria (OR condition):
3. **Accelerometer impact**: `Z-accel > 11 m/s²` (>1.1g indicates ground contact)

### Failsafe Trigger:
4. **Time-based**: If barometer fails, touchdown after 10 seconds of descent

```cpp
// Touchdown detection code
bool altitudeTouchdown = (altitudeAGL < LANDING_TOUCHDOWN_ALTITUDE);  // < 15cm
bool velocityLow = (abs(verticalVelocity) < LANDING_VELOCITY_THRESHOLD);  // < 10 cm/s
bool accelImpact = (sensors.accelZ > 11.0f);  // > 1.1g

if ((altitudeTouchdown && velocityLow) || accelImpact || baroFailsafe) {
  landingState = LANDING_TOUCHDOWN;
  // Set ground reference
  groundReferenceAltitude = sensors.altitude;
}
```

---

## Descent Profile

### Smooth S-Curve Algorithm

The descent rate follows an **ease-in-out curve** for smooth acceleration and deceleration:

```cpp
// S-curve formula (0 to 1 over 5 seconds)
float progress = min(1.0f, timeSinceLanding / 5.0f);
float smoothFactor = 3.0f * progress * progress - 2.0f * progress * progress * progress;
targetDescentRate = LANDING_DESCENT_RATE_MAX * smoothFactor;  // 0 → 50 cm/s
```

**Graph**:
```
Descent Rate (cm/s)
50 │         ╭───────────
   │       ╭─╯
   │     ╭─╯
25 │   ╭─╯
   │ ╭─╯
0  │─╯
   └─────────────────────► Time (s)
   0   1   2   3   4   5
```

### Why S-Curve?
- **Linear descent**: Jerky start/stop (bad)
- **S-curve descent**: Smooth acceleration → constant speed → smooth deceleration (good!)

---

## Throttle Control During Landing

### Altitude PID Feedback Loop

```cpp
// Target altitude decreases over time
targetAltitude = landingStartAltitude - (descentRate * elapsedTime);

// PID calculates throttle adjustment
float altitudeError = targetAltitude - currentAltitude;
float throttleCorrection = pidAltitude.compute(altitudeError);

// Base throttle around hover point (1500µs)
baseThrottle = 1500 + throttleCorrection;
baseThrottle = constrain(baseThrottle, 1100, 1900);
```

### Motor Mixing with Minimum Spin

```cpp
// X-configuration mixing
motorFL = baseThrottle - pidPitch + pidRoll - pidYaw;
motorFR = baseThrottle - pidPitch - pidRoll + pidYaw;
motorRR = baseThrottle + pidPitch - pidRoll - pidYaw;
motorRL = baseThrottle + pidPitch + pidRoll + pidYaw;

// CRITICAL: Apply minimum throttle (prevents motor stop)
if (landingState == DESCENDING || landingState == NEAR_GROUND) {
  motorFL = max(motorFL, LANDING_IDLE_THROTTLE);  // 1100µs
  motorFR = max(motorFR, LANDING_IDLE_THROTTLE);
  motorRR = max(motorRR, LANDING_IDLE_THROTTLE);
  motorRL = max(motorRL, LANDING_IDLE_THROTTLE);
}
```

**Why minimum throttle?**
- Motors never stop during descent
- Attitude control remains effective
- Prevents sudden drops

---

## Failsafe Modes

### 1. Barometer Failure Failsafe

If MS5611 fails or provides invalid data:

```cpp
if (baroFailsafe) {
  Serial.println("⚠️ BAROMETER FAILSAFE - Using time-based descent");
  
  // Very slow descent (30% of normal)
  targetDescentRate = LANDING_DESCENT_RATE_MAX * 0.3f;  // 15 cm/s
  
  // Touchdown after 10 seconds
  if (timeSinceLanding > 10.0f) {
    landingState = LANDING_TOUCHDOWN;
  }
}
```

**Detection criteria**:
- MS5611 initialization fails
- Read errors for > 500ms
- Pressure readings out of valid range (300-1100 mbar)

### 2. IMU Failure Failsafe

If MPU6050 fails:

```cpp
if (!sensors.mpuValid || millis() - sensors.lastMPURead > SENSOR_TIMEOUT_MS) {
  Serial.println("❌ IMU FAILURE - EMERGENCY STOP");
  armed = false;
  failsafeMode();  // Continuous beeping + LED flashing
}
```

### 3. Radio Signal Loss Failsafe

If radio disconnects:

```cpp
if (millis() - lastRadioTime > FAILSAFE_TIMEOUT_MS) {
  Serial.println("❌ RADIO SIGNAL LOST - FAILSAFE");
  armed = false;
  currentMode = MODE_DISARMED;
  // Motors stop
}
```

---

## Ground Reference Tracking

The system maintains a **ground reference altitude** for accurate height-above-ground:

### Initialization
```cpp
// On takeoff (Button 4)
groundReferenceAltitude = sensors.altitude;  // Set current altitude as ground
groundReferenceSet = true;

// On landing initiation (Button 3)
if (!groundReferenceSet) {
  groundReferenceAltitude = 0;  // Will be updated at touchdown
}
```

### Usage
```cpp
// Calculate altitude above ground level
float altitudeAGL = sensors.altitude - groundReferenceAltitude;

// Example:
// sensors.altitude = 150cm (raw barometer)
// groundReferenceAltitude = 5cm (ground pressure offset)
// altitudeAGL = 145cm (actual height above ground)
```

### Recalibration
```cpp
// Update ground reference at touchdown
if (landingState == LANDING_TOUCHDOWN) {
  groundReferenceAltitude = sensors.altitude;
  groundReferenceSet = true;
}
```

---

## Vertical Velocity Estimation

Used for touchdown detection and descent rate monitoring:

```cpp
// Calculate velocity from altitude change
float dt = (currentTime - lastAltitudeTime) / 1000.0f;  // seconds
verticalVelocity = (currentAltitude - lastAltitude) / dt;  // cm/s

// Low-pass filter to reduce noise
static float filteredVelocity = 0;
filteredVelocity = 0.8f * filteredVelocity + 0.2f * verticalVelocity;
verticalVelocity = filteredVelocity;
```

**Velocity signs**:
- Positive: Rising
- Negative: Descending
- Near zero: Hovering or touchdown

---

## Safety Limits

### Maximum Descent Rate
```cpp
#define LANDING_DESCENT_RATE_MAX  50.0f  // cm/s
```
Prevents dangerously fast descent.

### Maximum Tilt During Landing
```cpp
#define LANDING_MAX_TILT  15.0f  // degrees
```
Limits roll/pitch to prevent aggressive maneuvers near ground.

### Touchdown Altitude Threshold
```cpp
#define LANDING_TOUCHDOWN_ALTITUDE  15.0f  // cm
```
Below this altitude, touchdown is considered.

### Idle Throttle Levels
```cpp
#define LANDING_IDLE_THROTTLE  1100  // µs (during descent, attitude control active)
#define LANDING_SAFE_IDLE      1050  // µs (after touchdown, minimal thrust)
```

---

## Integration with Stabilization

The landing system works **seamlessly with the attitude controller**:

1. **Roll/Pitch stabilization continues** throughout landing
2. **Angle mode enforced** (auto-level) with tilt limits
3. **Yaw control active** (pilot can rotate during descent)
4. **Altitude PID overrides throttle** (replaces manual stick input)

```cpp
// During landing, altitude PID sets base throttle
if (currentMode == MODE_LANDING) {
  float targetAltitude = computeLandingTarget();
  baseThrottle = altitudePID.compute(targetAltitude, currentAltitude);
}

// Attitude PID still runs normally
pidRoll = computePID(targetRoll, actualRoll);
pidPitch = computePID(targetPitch, actualPitch);
pidYaw = computePID(targetYaw, actualYaw);

// Mix into motors
motorFL = baseThrottle - pidPitch + pidRoll - pidYaw;
// ... etc
```

---

## Telemetry Output

During landing, Serial Monitor shows:

```
═══════════════════════════════════════════════════════════
🛬 AUTOMATIC LANDING INITIATED
═══════════════════════════════════════════════════════════
  Current altitude: 148.5 cm
  State: INITIATED → DESCENDING

Mode:LAND | R:2.3 P:-1.5 | Alt:132cm V:-42cm/s | LS:DESC | M:1320,1315,1310,1325

  State: DESCENDING → NEAR GROUND

Mode:LAND | R:0.8 P:-0.2 | Alt:42cm V:-18cm/s | LS:NEAR | M:1180,1175,1172,1178

✅ TOUCHDOWN DETECTED!
  Altitude: 12.3 cm | Velocity: 8.5 cm/s
  State: TOUCHDOWN → SAFE IDLE

Mode:LAND | R:0.1 P:0.3 | Alt:0cm V:-2cm/s | LS:SAFE | M:1050,1048,1052,1051

✅ LANDING COMPLETE - DISARMED
═══════════════════════════════════════════════════════════
```

---

## Parameter Tuning

### Descent Rate
```cpp
#define LANDING_DESCENT_RATE_MAX  50.0f  // cm/s
```
- **Faster descent** (60-80 cm/s): Quicker landing, less smooth
- **Slower descent** (30-40 cm/s): Smoother, more time

### Touchdown Altitude
```cpp
#define LANDING_TOUCHDOWN_ALTITUDE  15.0f  // cm
```
- **Higher** (20-25 cm): Earlier touchdown detection, might bounce
- **Lower** (10-12 cm): More accurate, might land hard

### Idle Throttle
```cpp
#define LANDING_IDLE_THROTTLE  1100  // µs
```
- **Higher** (1150 µs): More control authority, slower descent
- **Lower** (1080 µs): Faster descent, less control

### Altitude PID
```cpp
pidAltitude.Kp = 4.5;   // Response to altitude error
pidAltitude.Ki = 0.15;  // Correction for steady-state error
pidAltitude.Kd = 3.5;   // Damping (reduces oscillation)
```

**Tuning guide**:
- **Oscillates**: Reduce Kp, increase Kd
- **Slow response**: Increase Kp
- **Steady-state error**: Increase Ki (but not too much!)

---

## Test Checklist

### Pre-Landing Test
- [ ] Barometer calibrated (ground = 0 cm)
- [ ] Serial Monitor shows valid altitude readings
- [ ] Drone hovers stably in ALT_HOLD mode
- [ ] Radio signal strong
- [ ] Open area with flat ground

### Landing Test Sequence

1. **Takeoff** (Button 4)
   - Drone rises to 150 cm
   - Enters ALT_HOLD automatically
   - Check Serial: `Mode:ALT_H | Alt:150cm`

2. **Initiate Landing** (Button 3)
   - Check Serial: `🛬 AUTOMATIC LANDING INITIATED`
   - State: `INITIATED → DESCENDING`

3. **Observe Descent**
   - Smooth descent, no jerking
   - Motors audibly spinning (not silent)
   - Drone remains level (±15° max tilt)
   - Serial shows: `Alt:XXXcm V:-YYcm/s | LS:DESC`

4. **Near Ground Transition**
   - At ~50 cm: `State: DESCENDING → NEAR GROUND`
   - Descent slows noticeably
   - Serial: `LS:NEAR`

5. **Touchdown**
   - Gentle contact with ground
   - Serial: `✅ TOUCHDOWN DETECTED!`
   - Motors reduce to low idle
   - Serial: `LS:SAFE`

6. **Complete**
   - Motors stop after 0.5s
   - 3 beeps
   - Serial: `✅ LANDING COMPLETE - DISARMED`

### Expected vs Actual

| Parameter | Expected | How to Check |
|-----------|----------|--------------|
| Descent rate | 30-50 cm/s | Serial: `V:-XX cm/s` |
| Touchdown altitude | < 15 cm | Serial at touchdown |
| Vertical velocity at touchdown | < 10 cm/s | Serial: `V:-X cm/s` |
| Total landing time | 3-5 seconds | Stopwatch |
| Motors spinning throughout | YES | Listen for motor sound |
| Drone tilt during descent | < 15° | Visual observation |

---

## Common Issues & Solutions

### Issue: Drone drops suddenly
**Cause**: Motors stopping mid-descent

**Solution**: Check minimum throttle is applied:
```cpp
applyMinimumThrottle(LANDING_IDLE_THROTTLE);  // Should be in code
```

---

### Issue: Never detects touchdown
**Cause**: Altitude threshold too low or velocity too strict

**Solution**: Adjust thresholds:
```cpp
#define LANDING_TOUCHDOWN_ALTITUDE  20.0f  // Increase from 15
#define LANDING_VELOCITY_THRESHOLD  15.0f  // Increase from 10
```

---

### Issue: Bounces after touchdown
**Cause**: Motors still at high throttle

**Solution**: Verify state transitions in Serial Monitor. Should see:
```
TOUCHDOWN → SAFE IDLE (motors reduce)
SAFE IDLE → COMPLETE (motors stop)
```

---

### Issue: Descends too fast
**Cause**: Altitude PID too aggressive

**Solution**: Reduce PID gains:
```cpp
pidAltitude.Kp = 3.5;  // Reduce from 4.5
pidAltitude.Kd = 4.5;  // Increase from 3.5 (more damping)
```

---

### Issue: Barometer shows "FAILSAFE"
**Cause**: MS5611 not connected or faulty

**Solution**:
1. Check I2C wiring (SDA/SCL to A4/A5)
2. Check MS5611 power (3.3V or 5V depending on module)
3. Test with I2C scanner sketch
4. Landing will use time-based fallback (10s descent)

---

## Summary

The landing system provides:

✅ **Smooth descent** with S-curve profile  
✅ **Altitude feedback** from MS5611 barometer  
✅ **Touchdown detection** (altitude + velocity + accel)  
✅ **Safe motor sequence** (idle → safe idle → stop)  
✅ **Failsafe modes** (barometer, IMU, radio)  
✅ **Continuous stabilization** throughout descent  
✅ **Ground reference tracking** for accurate height  

**Result**: Professional-grade automatic landing with no sudden drops or hard touchdowns!
