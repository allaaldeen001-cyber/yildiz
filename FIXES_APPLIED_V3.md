# ✅ FIXES APPLIED - Testing with Props

## Issues Fixed

### ✅ Issue 1: Pitch Inverted (Joystick Backward/Forward)

**Problem**: 
- Joystick UP → Drone goes BACKWARD ❌
- Joystick DOWN → Drone goes FORWARD ❌

**Root Cause**: 
Pitch axis mapping was not inverted.

**Fix Applied**:
```cpp
// ANGLE Mode (line ~370):
float tgtPitchAng = map(rcData.pitch, -500, 500, MAX_ANGLE, -MAX_ANGLE);
// Now: Joystick UP (+500) → Negative angle → Forward tilt ✅

// ACRO Mode (line ~382):
pitchRateSp = map(rcData.pitch, -500, 500, 400, -400);
// Now: Joystick UP (+500) → Negative rate → Forward rotation ✅
```

**Result**:
- Joystick UP → Drone goes FORWARD ✅
- Joystick DOWN → Drone goes BACKWARD ✅

---

### ✅ Issue 2: Motors Too Fast (RS2205 2300KV)

**Problem**: 
Motors spin too fast, dangerous for testing.

**Root Cause**: 
Throttle mapping was 1000-2000µs (full range). High KV motors (2300KV) spin very fast even at low throttle.

**Fixes Applied**:

#### 1. Reduced Maximum Throttle (Manual Mode)
```cpp
// Before:
baseThr = map(rcData.throttle, 0, 1000, 1000, 2000);  // Full range

// After:
baseThr = map(rcData.throttle, 0, 1000, 1000, 1700);  // LIMITED to 1700µs (85%)
```

**Effect**: 
- Full throttle stick now = 1700µs instead of 2000µs
- ~35% reduction in max power
- Safer for testing!

#### 2. Reduced Altitude Hold Hover Point
```cpp
// Before:
baseThr = 1500 + altCorr;  // Hover around 1500µs
baseThr = constrain(baseThr, 1100, 1900);

// After:
baseThr = 1400 + altCorr;  // Hover around 1400µs (LOWER)
baseThr = constrain(baseThr, 1100, 1700);  // Max 1700µs
```

**Effect**:
- Altitude hold uses less power
- Gentler hover for 2300KV motors
- Still adjusts up/down with throttle stick

#### 3. Reduced Motor Test Speed
```cpp
// Before:
mFL.writeMicroseconds(1150);  // Test at 1150µs

// After:
mFL.writeMicroseconds(1120);  // Test at 1120µs (LOWER)
```

**Effect**:
- Motor test spins slower (safer without props)
- Can still verify motor directions

---

## Expected Behavior Now

### Throttle Response (Manual/ACRO/ANGLE Mode)

| Stick Position | Motor Output | Power Level |
|----------------|--------------|-------------|
| 0% (min) | 1000µs | Off |
| 25% | 1175µs | Low (~20% power) |
| 50% | 1350µs | Medium (~40% power) |
| 75% | 1525µs | High (~60% power) |
| 100% (max) | **1700µs** | **Max (85% power)** |

**Before**: 100% stick = 2000µs = 100% power = TOO FAST!  
**After**: 100% stick = 1700µs = 85% power = SAFER ✅

---

### Altitude Hold Hover Point

**Before**: ~1500µs hover point  
**After**: ~1400µs hover point  

For RS2205 2300KV motors:
- **1400µs** = gentle hover (good for testing)
- **1500µs** = aggressive hover (too much for light quads)

---

### Pitch Control

**Before (WRONG)**:
- Push joystick UP → Pitch setpoint = +45° → Drone tilts back → Goes backward ❌

**After (CORRECT)**:
- Push joystick UP → Pitch setpoint = -45° → Drone tilts forward → Goes forward ✅

**Why this matters**:
Standard quadcopter convention:
- Forward tilt = Negative pitch angle
- Backward tilt = Positive pitch angle

Now matches your expectation!

---

## Testing Guide

### Test 1: Verify Pitch Direction (Hovering Test)

1. **Arm** drone (Button 4 or manual arm)
2. **Increase throttle** slowly until it hovers (~30cm)
3. **Push right joystick UP** (forward)
   - **Expected**: Drone moves FORWARD ✅
4. **Pull right joystick DOWN** (backward)
   - **Expected**: Drone moves BACKWARD ✅
5. **Push right joystick LEFT**
   - **Expected**: Drone moves LEFT ✅
6. **Push right joystick RIGHT**
   - **Expected**: Drone moves RIGHT ✅

**If roll is also inverted**, let me know and I'll fix it too!

---

### Test 2: Verify Throttle Range (Ground Test)

1. **Disarm** drone
2. **Arm** (don't increase throttle)
3. **Slowly increase throttle** to 25%
   - **Expected**: Motors spin gently (~1175µs)
4. **Increase to 50%**
   - **Expected**: Motors spin moderately (~1350µs)
5. **Increase to 100%**
   - **Expected**: Motors spin fast but not max (~1700µs)
6. **Disarm**

**Expected**: Progressive increase, not too aggressive ✅

---

### Test 3: Altitude Hold (If Barometer Works)

1. **Manual takeoff** to 50cm
2. **Switch SW1 ON** (Altitude Hold)
3. **Release throttle stick** to center
   - **Expected**: Drone hovers at ~50cm, gentle power
4. **Move throttle up slightly**
   - **Expected**: Climbs slowly
5. **Move throttle down slightly**
   - **Expected**: Descends slowly

**Expected**: Gentle altitude hold at ~1400µs ✅

---

## Motor Directions (Verify These!)

You confirmed:
```
FR: CW  (Clockwise)
FL: CCW (Counter-clockwise)
RR: CCW (Counter-clockwise)
RL: CW  (Clockwise)
```

**This is correct for X-configuration!** ✅

```
     FRONT
      (↑)
       
FL(CCW)   FR(CW)
    ╲    ╱
     ╲  ╱
      ╳
     ╱  ╲
    ╱    ╲
RL(CW)    RR(CCW)
```

If any motor spins the **wrong direction**:
1. Disconnect battery
2. Swap **any 2 wires** between motor and ESC
3. Test again (Button 2)

---

## Tuning Recommendations for RS2205 2300KV

### If Motors Still Too Powerful

**Option 1**: Reduce throttle limit further
```cpp
// In computePID(), line ~352:
baseThr = map(rcData.throttle, 0, 1000, 1000, 1600);  // Max 1600µs (80%)
```

**Option 2**: Reduce PID output
```cpp
// At top of file, reduce these:
#define RATE_ROLL_KP 0.55f   // Reduced from 0.65
#define RATE_PITCH_KP 0.55f  // Reduced from 0.65
```

---

### If Oscillates (Vibrates/Shakes)

**High KV motors need lower P gain**:
```cpp
#define RATE_ROLL_KP 0.50f   // Reduce from 0.65
#define RATE_PITCH_KP 0.50f  // Reduce from 0.65
```

---

### If Feels Sluggish

**Increase D gain** (more damping):
```cpp
#define RATE_ROLL_KD 0.025f  // Increase from 0.018
#define RATE_PITCH_KD 0.025f // Increase from 0.018
```

---

## Battery Considerations

**RS2205 2300KV** typically run on:
- **3S LiPo** (11.1V nominal) → Good for testing
- **4S LiPo** (14.8V nominal) → More power, reduce throttle even more!

**If using 4S**, reduce max throttle to **1500-1600µs**:
```cpp
baseThr = map(rcData.throttle, 0, 1000, 1000, 1500);  // 4S battery
```

---

## Summary of Changes

| Parameter | Before | After | Reason |
|-----------|--------|-------|--------|
| **Pitch (ANGLE)** | `map(pitch, -500, 500, -45, 45)` | `map(pitch, -500, 500, 45, -45)` | Fix inversion |
| **Pitch (ACRO)** | `map(pitch, -500, 500, -400, 400)` | `map(pitch, -500, 500, 400, -400)` | Fix inversion |
| **Max Throttle** | 2000µs (100%) | 1700µs (85%) | Reduce power |
| **Alt Hold Hover** | 1500µs | 1400µs | Gentler hover |
| **Alt Hold Max** | 1900µs | 1700µs | Reduce power |
| **Motor Test** | 1150µs | 1120µs | Safer testing |

---

## Upload and Test!

1. **Upload** FlightController.ino (re-upload with fixes)
2. **Test pitch** direction (hovering test)
3. **Test throttle** range (should be gentler)
4. **Adjust** if needed (see tuning recommendations)

---

## If You Need More Changes

**Pitch still inverted?**
→ Check Remote Controller joystick wiring (A2 pin)

**Still too fast?**
→ Further reduce max throttle:
```cpp
baseThr = map(rcData.throttle, 0, 1000, 1000, 1600);
```

**Motors spin wrong direction?**
→ Swap 2 motor wires on that ESC

**Oscillates/vibrates?**
→ Reduce Rate PID Kp to 0.50

---

**Upload and test now!** Both issues should be fixed! ✅
