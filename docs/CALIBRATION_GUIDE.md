# Calibration and Setup Guide

Complete calibration procedures for the Professional Arduino Nano Drone System.

---

## 📋 Pre-Flight Checklist

Before any calibration or flight:

- [ ] All hardware connections verified (see WIRING_DIAGRAM.md)
- [ ] **Propellers REMOVED** (install only after all testing complete)
- [ ] Battery fully charged
- [ ] Firmware uploaded to both Arduino Nano boards
- [ ] Serial monitor working at 115200 baud
- [ ] Clear workspace on level surface

---

## 🔄 System Startup Sequence

Follow this exact sequence every time you fly:

### Step 1: Power Up Remote Controller
1. **Connect battery to Remote Controller**
2. **Open Serial Monitor** (115200 baud)
3. **Verify display shows:**
   - "PROFESSIONAL QUADCOPTER REMOTE CONTROLLER"
   - NRF24L01 initialized OK
   - System Ready

**Expected Output:**
```
╔════════════════════════════════════════════════════╗
║   PROFESSIONAL QUADCOPTER REMOTE CONTROLLER       ║
║   Version 1.0.0                                    ║
╚════════════════════════════════════════════════════╝

[INIT] NRF24L01 Radio... OK
[INIT] System Ready
```

### Step 2: Power Up Flight Controller
1. **Connect battery to Flight Controller** (via ESC BEC or separate power)
2. **Listen for startup beep** (single short beep)
3. **Wait 2-3 seconds** for system initialization
4. **Listen for ready beeps** (two short beeps)

**Expected Beep Pattern:**
- 1 beep: Power on
- (2-3 second pause)
- 2 beeps: System ready

### Step 3: Verify Communication Link
1. **Check Remote Controller serial monitor**
   - Should show "✓ CONNECTED" under Communication Status
   - Success rate should be >95%
2. **Check Flight Controller Status LED (D7)**
   - Should blink slowly (every 500ms) when linked
   - Fast blinking = not calibrated
   - No blinking = no communication

**Serial Monitor Should Show:**
```
┌─── COMMUNICATION STATUS ───────────────────────────┐
│ Link Status: ✓ CONNECTED                          │
│ Channel: 103 | TX Count: 50 | RX Count: 5 | Fails: 0
│ Success Rate: 100.0%                               │
└────────────────────────────────────────────────────┘
```

---

## 🎯 Gyroscope Calibration

**CRITICAL:** Must be performed before every flight session!

### Procedure:

1. **Place drone on completely level surface**
   - Use table or floor (not on carpet or soft surface)
   - Ensure no vibration or movement
   - No wind or fans nearby

2. **Ensure Kill Switch (SW_2) is in "ARMED" position (up/1)**
   - Remote controller switch D3 should be open (not grounded)

3. **Press and release BUTTON_1 (Calibration Button) on Remote**

4. **Keep drone absolutely still for 4-5 seconds**
   - Do not touch drone
   - No vibrations on table
   - Status LED will flash rapidly during calibration

5. **Listen for result:**
   - **SUCCESS**: 2 short beeps (beep-beep)
   - **FAILURE**: 1 long beep (7 seconds)

6. **Check serial monitor for confirmation:**

**Successful Calibration:**
```
┌─── DRONE TELEMETRY ────────────────────────────────┐
│ Status: ✓ READY                                    │
│ Battery: 11.1V                                     │
│ Attitude: Roll=0.2° Pitch=-0.1° Yaw=0.0°          │
│ Gyro: X=15 Y=-8 Z=3                                │
└────────────────────────────────────────────────────┘
```

### Troubleshooting Calibration:

| Problem | Cause | Solution |
|---------|-------|----------|
| 1 long beep (failure) | Excessive drift detected | - Check for vibrations<br>- Ensure stable surface<br>- Check MPU6050 connection |
| No beeps | Button not registered | - Check Button_1 wiring<br>- Verify in serial monitor button state |
| Unstable after calibration | Surface not level | - Use bubble level app on phone<br>- Recalibrate on flat surface |

---

## ⚙️ ESC Calibration

**NOTE:** Only needed ONCE when setting up new ESCs or after ESC firmware update!

### Purpose:
- Synchronizes ESC throttle range with flight controller output
- Ensures all motors respond identically
- Prevents motor desync issues

### Prerequisites:
- **PROPELLERS REMOVED!**
- Gyro calibration completed
- Battery connected and fully charged

### Procedure:

1. **Verify switch positions:**
   - **SW_1 (Altitude Hold)**: Set to "0" (down/grounded on D2)
   - **SW_2 (Kill Switch)**: Set to "1" (up/armed)

2. **Press and hold BUTTON_2 (Motor Arm Button) for 1 second**

3. **Release button and wait**
   - System enters ESC calibration mode
   - 3 rapid beeps confirm entry

4. **Automatic sequence begins:**
   - Each motor will spin up one by one:
     1. Front-Left (FL)
     2. Front-Right (FR)
     3. Rear-Right (RR)
     4. Rear-Left (RL)
   - Each motor ramps up smoothly, then back down
   - Single beep after each motor
   - Total time: ~15 seconds

5. **Completion:**
   - 2 final beeps (beep-beep)
   - Serial monitor shows "ESC Calibration Complete"

### Expected Motor Sequence:

```
    FRONT
  1→ FL    FR ←2
      \    /
       \  /
        \/
        /\
       /  \
      /    \
  4→ RL    RR ←3
      REAR

Order: FL → FR → RR → RL
```

### Verify Correct Operation:

After ESC calibration, each motor should:
- ✓ Start smoothly (no stuttering)
- ✓ Spin quietly without unusual noise
- ✓ Accelerate and decelerate smoothly
- ✓ Stop completely when commanded

### Common ESC Issues:

| Problem | Cause | Solution |
|---------|-------|----------|
| Motor beeps continuously | ESC not calibrated | Redo ESC calibration |
| Motor doesn't spin | Bad connection | Check signal wire, power |
| Motor stutters | Low battery, bad ESC | Charge battery, replace ESC |
| Wrong direction | Motor wiring reversed | Swap any 2 of 3 motor wires |

---

## 🔧 Joystick Calibration (Optional)

Joysticks are auto-calibrated in firmware, but you can verify:

### Check Joystick Centers:

1. **Open serial monitor on Remote Controller**
2. **Leave joysticks at center (not touching)**
3. **Check values in "CONTROL INPUTS" section:**
   - Throttle: ~1000-1050 (at minimum)
   - Yaw: ~1480-1520
   - Pitch: ~1480-1520
   - Roll: ~1480-1520

4. **Move each stick to extremes:**
   - Minimum: ~1000
   - Maximum: ~2000

### If Values are Incorrect:

The firmware has a 20-point deadband around center. If your joysticks are:
- **Too sensitive**: Increase `STICK_DEADBAND` in RemoteController.ino (line 45)
- **Not responsive**: Decrease `STICK_DEADBAND`
- **Reversed**: Swap connections or modify mapping in code

---

## ⚠️ Arming Sequence

After calibration, to arm the drone:

### Requirements to Arm:
1. ✓ Gyroscope calibrated (2 beeps after Button_1)
2. ✓ Kill Switch (SW_2) in "ARMED" position
3. ✓ Throttle stick at minimum (<1050)
4. ✓ Communication link active

### Arming Steps:

1. **Verify all requirements met** (check serial monitor)

2. **Press BUTTON_2 (Motor Arm)** on remote

3. **Listen for 3 rapid beeps** = ARMED

4. **Status LED goes solid ON**

5. **Serial monitor shows:**
```
┌─── DRONE TELEMETRY ────────────────────────────────┐
│ Status: ⚠ ARMED & FLYING                          │
```

### Disarming:

**Method 1: Kill Switch (Emergency)**
- Flip SW_2 to "0" position → Immediate disarm

**Method 2: Low Throttle**
- Lower throttle to minimum for 2 seconds

**Method 3: Release Arm Button**
- Release BUTTON_2 (if using toggle mode)

### Failsafe Disarm:

System automatically disarms if:
- Signal lost for >1 second
- Battery voltage too low (if monitoring enabled)
- MPU6050 sensor failure

---

## 🧪 Pre-Flight Test Procedure

**With propellers REMOVED:**

### Test 1: Communication
- [ ] Power both systems
- [ ] Serial shows "CONNECTED"
- [ ] LED blinks on FC
- [ ] Telemetry data updating

### Test 2: Calibration
- [ ] Gyro calibration successful (2 beeps)
- [ ] Angles near 0° when level
- [ ] ESC calibration if needed

### Test 3: Controls
- [ ] Move throttle → value changes 1000-2000
- [ ] Move yaw → value changes 1000-2000
- [ ] Move pitch → value changes 1000-2000
- [ ] Move roll → value changes 1000-2000
- [ ] All inputs show in serial monitor

### Test 4: Switches
- [ ] Toggle SW_1 → serial shows status change
- [ ] Toggle SW_2 → serial shows armed/disarmed
- [ ] Press Button_1 → calibration starts
- [ ] Press Button_2 → (when armed) motors respond

### Test 5: Motors (NO PROPS!)
- [ ] Arm system
- [ ] Slowly increase throttle
- [ ] All 4 motors spin
- [ ] Motors stop when throttle lowered
- [ ] Disarm successfully

### Test 6: Stability Check
- [ ] Arm system
- [ ] Gently tilt FC forward → pitch angle changes
- [ ] Gently tilt FC left → roll angle changes
- [ ] Rotate FC → yaw rate changes
- [ ] Values return to ~0 when level

---

## 🚁 First Flight Procedure

**Only after all tests pass!**

### Preparation:

1. **Install propellers** (correct rotation and orientation!)
   - FL: Counter-clockwise prop
   - FR: Clockwise prop
   - RR: Counter-clockwise prop
   - RL: Clockwise prop

2. **Choose safe location:**
   - Open area, no obstacles within 10 meters
   - No people or animals nearby
   - Soft grass (in case of crash)
   - No wind or very light wind
   - Good lighting

3. **Final checks:**
   - Battery fully charged
   - Propellers tight but not over-tightened
   - Frame secure
   - All wires secure (no loose connections)

### Flight Test:

1. **Perform calibration** (Button_1)
   - Wait for 2 beeps

2. **Arm drone** (Button_2)
   - Stand back 2-3 meters

3. **Slowly increase throttle to 30-40%**
   - Drone should lift gently
   - Hover at ~0.5 meters

4. **Test stability:**
   - Hold hover for 5-10 seconds
   - Observe if drone drifts
   - Check if drone oscillates

5. **Test controls (at low altitude):**
   - Small pitch input → should move forward/backward
   - Small roll input → should move left/right
   - Small yaw input → should rotate
   - All movements should be smooth

6. **Land:**
   - Slowly reduce throttle
   - Let drone settle on ground
   - Reduce throttle to minimum
   - Disarm

---

## 📊 PID Tuning (Advanced)

If drone is unstable, you may need to tune PID values:

### Default Values (in FlightController.ino):
```cpp
Roll/Pitch:  Kp=1.3,  Ki=0.04,  Kd=18.0
Yaw:         Kp=2.0,  Ki=0.02,  Kd=0.0
```

### Symptoms and Solutions:

| Symptom | Likely Cause | Adjustment |
|---------|--------------|------------|
| Oscillates rapidly | Kp too high | Decrease Kp by 0.1 |
| Drifts slowly | Ki too low | Increase Ki by 0.01 |
| Overshoots corrections | Kd too low | Increase Kd by 1.0 |
| Sluggish response | Kp too low | Increase Kp by 0.1 |
| Wobbles at hover | Kd too high | Decrease Kd by 1.0 |

### Tuning Procedure:

1. Start with Kp only (set Ki=0, Kd=0)
2. Increase Kp until drone oscillates
3. Reduce Kp by 20%
4. Add Kd to dampen oscillations
5. Add small Ki to eliminate steady-state error
6. Test and iterate

---

## 🔋 Battery and Power Management

### Battery Selection:
- **Recommended**: 3S LiPo (11.1V nominal)
- **Capacity**: 1500-2200mAh for 5-7 minute flight time
- **Discharge rate**: 25C minimum

### Battery Safety:
- Never discharge below 3.3V per cell (9.9V for 3S)
- Store at 3.8V per cell (storage charge)
- Use LiPo bag for charging and storage
- Never leave charging unattended

### Low Battery Indication:
Currently the firmware shows 11.1V placeholder. To add real monitoring:
1. Add voltage divider to analog pin (A6 or A7)
2. Modify telemetry section to read voltage
3. Add low voltage alarm (buzzer pattern)

---

## 📝 Calibration Log Template

Keep a log of each calibration session:

```
Date: _______________
Battery Voltage: _____ V
Temperature: _____ °C
Location: _____________

Gyro Calibration:
- Success: [ ] Yes [ ] No
- Offsets: X=____ Y=____ Z=____
- Attempts: ____

ESC Calibration:
- Performed: [ ] Yes [ ] No
- All motors OK: [ ] Yes [ ] No

Flight Test:
- Hover stable: [ ] Yes [ ] No
- Control response: [ ] Good [ ] Needs tuning
- Flight time: ____ minutes

Notes:
_______________________________
_______________________________
_______________________________
```

---

## 🆘 Emergency Procedures

### If Drone Becomes Unstable in Flight:
1. **Immediately flip Kill Switch (SW_2)** → Motors stop
2. **Or** lower throttle to minimum

### If Communication Lost:
- Drone will automatically disarm after 1 second
- Motors stop, drone will fall (land softly if low altitude)

### If Motor Stops Mid-Flight:
- **DO NOT** try to recover
- **Flip Kill Switch immediately**
- Let drone land (crash)
- Inspect for damage before next flight

---

## ✅ Daily Pre-Flight Checklist

Before every flight session:

- [ ] Visual inspection (no loose wires, props secure)
- [ ] Battery charged (>11.4V for 3S)
- [ ] Propellers undamaged
- [ ] Gyro calibration performed
- [ ] Communication link verified
- [ ] All controls responding correctly
- [ ] Kill switch tested
- [ ] Clear flight area
- [ ] Weather suitable (no rain, light wind)

---

**Remember**: Safety first! When in doubt, don't fly. Always be ready to disarm immediately.

