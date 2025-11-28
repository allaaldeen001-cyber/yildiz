# 🎯 Complete Calibration Guide

## Why Calibration is Critical

Proper calibration ensures:
- **Stable hover** without drifting
- **Accurate angle measurements**
- **Smooth motor response**
- **Safe flight characteristics**

**Without proper calibration, your drone WILL NOT fly well!**

---

## 📋 Pre-Calibration Checklist

Before calibrating:
- [ ] All hardware properly connected
- [ ] Batteries fully charged
- [ ] Code uploaded to both Arduino boards
- [ ] Serial monitor ready (57600 baud)
- [ ] Drone on perfectly flat, stable surface
- [ ] No vibrations or wind
- [ ] Props OFF for safety (during most calibrations)

---

## 🔧 Step-by-Step Calibration

### Stage 1: ESC Calibration (One-Time Setup)

**Purpose**: Ensures all motors respond identically to throttle commands.

**When**: First time setup, or after changing ESCs.

**Steps**:

1. **Remove propellers** for safety
2. **Disconnect battery** from drone
3. **Power on RC transmitter**
4. **Move throttle stick to MAXIMUM** (top position)
5. **Connect battery** to flight controller
6. **Listen for beep sequence** from ESCs (usually 3 beeps)
7. **Wait 2 seconds**
8. **Move throttle to MINIMUM** (bottom position)
9. **ESCs will beep again** (confirming calibration)
10. **Disconnect battery**
11. **Return throttle to minimum**

**Expected Result**: All 4 ESCs beep in unison.

**Troubleshooting**:
- If only some ESCs beep: Check connections
- If no beeps: Verify ESC power and signal wires
- If motors spin during calibration: DISCONNECT IMMEDIATELY, check wiring

---

### Stage 2: Gyro Calibration (Automatic at Startup)

**Purpose**: Zeroes out gyroscope bias for accurate angle measurement.

**When**: Automatically runs every time you power on the drone.

**What to Do**:

1. **Place drone on flat surface** (table, floor)
2. **Ensure it's level** (use spirit level if available)
3. **Power on flight controller**
4. **DO NOT MOVE** the drone for 3-5 seconds
5. **Watch serial monitor** for "Gyro Calibration Complete!"

**Serial Monitor Output**:
```
Starting Gyro Calibration...
Keep drone FLAT and STILL...
........
✓ Gyro calibration VALID
Gyro offsets -> X:-23.5 Y:15.2 Z:8.3
```

**Good Calibration Signs**:
- ✅ "Gyro calibration VALID" message
- ✅ Offset values typically -200 to +200
- ✅ No error messages

**Bad Calibration Signs**:
- ❌ "Gyro calibration may be poor!"
- ❌ Offset values >1000 or <-1000
- ❌ Drone was moved during calibration

**If Calibration Fails**:
1. Power off drone
2. Check MPU6050 connections (SDA, SCL, VCC, GND)
3. Ensure surface is truly flat
4. Remove any vibration sources
5. Try again

---

### Stage 3: Level Calibration (Manual - Critical!)

**Purpose**: Teaches the drone what "level" means for stable hover.

**When**: 
- First time setup
- After crashes
- If drone drifts left/right/forward/back
- At least once per week of flying

**Steps**:

1. **Power on RC transmitter**
2. **Power on flight controller**
3. **Place drone on perfectly FLAT surface**
   - Use a spirit level to verify
   - Table or smooth floor works best
   - NOT on carpet or uneven ground
4. **Open serial monitor** (57600 baud) on flight controller
5. **Verify "Radio: ✓ OK"** (link established)
6. **Hold Button 1 on RC for 2 seconds**
7. **Wait for buzzer beep**
8. **DO NOT MOVE** drone during calibration
9. **Watch progress dots** in serial monitor
10. **Wait for completion buzzer** (two beeps)
11. **Check serial monitor** for success message

**Serial Monitor Output**:
```
========================================
    STARTING LEVEL CALIBRATION
========================================
Starting LEVEL calibration...
Place drone on FLAT surface!
..............
✓ Level offsets -> X:0.123 Y:-0.087
✓ Calibration saved to EEPROM
========================================
```

**Good Calibration Values**:
- ✅ X and Y offsets between -2.0 and +2.0
- ✅ Close to zero (within ±0.5 is excellent)
- ✅ "Calibration saved to EEPROM" message

**Bad Calibration Values**:
- ❌ X or Y offset >5.0 or <-5.0
- ❌ Values change drastically each time
- ❌ Error messages

**If Level Calibration Seems Wrong**:
1. Verify surface is truly flat (use level)
2. Check motor mounts are level
3. Ensure frame is not bent
4. Repeat calibration 2-3 times, use most consistent result
5. Check MPU6050 is mounted flat on board

---

### Stage 4: Verification (Test Before Flying!)

**Purpose**: Confirm all calibrations are working correctly.

**Steps**:

1. **Arm the drone** (Button 2, 2 seconds) with props OFF
2. **Very slowly increase throttle** to ~30%
3. **All motors should spin at same speed**
4. **Tilt drone left** - Right motors should speed up
5. **Tilt drone right** - Left motors should speed up
6. **Tilt drone forward** - Rear motors should speed up
7. **Tilt drone backward** - Front motors should speed up
8. **Disarm** (Button 2, 2 seconds)

**What to Look For**:
- ✅ Motors respond to tilt in correct direction
- ✅ Motor response is proportional (more tilt = more correction)
- ✅ Response is smooth, not jerky
- ✅ All motors return to equal speed when level

**If Responses Are Wrong**:
- Check motor order (FL=3, FR=5, RR=6, RL=9)
- Verify motor rotation directions
- Recalibrate level (Stage 3)
- Check PID signs (should not need changing)

---

## 🎓 Understanding Calibration Values

### Gyro Offsets
These compensate for manufacturing variations in the MPU6050.

**Typical Values**: -200 to +200 (raw sensor units)

**Example**:
```
Gyro offsets -> X:-23.5 Y:15.2 Z:8.3
```
This means the gyro reads -23.5 on X-axis when stationary.

### Level Offsets
These compensate for sensor mounting angle.

**Typical Values**: -2.0 to +2.0 (degrees)

**Example**:
```
Level offsets -> X:0.123 Y:-0.087
```
This means the sensor is mounted 0.123° off-level on roll axis.

**Perfect Mounting**: Would give offsets of 0.000, 0.000
**Real World**: Usually within ±0.5°
**Acceptable**: Within ±2.0°
**Concerning**: >±5.0° (may indicate bent frame or bad sensor)

---

## 🔄 When to Re-Calibrate

### Gyro Calibration
- **Automatic every startup** - No manual action needed
- Happens in ~3 seconds

### Level Calibration
Re-calibrate if:
- Drone drifts consistently in one direction
- After any crash or hard landing
- Frame was disassembled/reassembled
- New motors or propellers installed
- Weekly for best performance
- After flight controller was removed/reinstalled

### ESC Calibration
Re-calibrate if:
- Replacing ESCs
- Motors spin at noticeably different speeds at same throttle
- After ESC firmware update
- Once per year for maintenance

---

## 🎯 Advanced Calibration Tips

### 1. Perfect Level Surface
- Use a bubble level to verify table/floor
- Glass table works best (rigid + flat)
- Avoid carpet or soft surfaces
- Temperature changes can affect floor flatness

### 2. Eliminate Vibrations
- Turn off fans, AC, washing machines
- Don't touch table during calibration
- Keep people/pets away
- Close windows (wind vibration)

### 3. Temperature Stabilization
- Let drone sit for 5 minutes after power-on
- MPU6050 drifts when temperature changes
- Fly in similar temperature to calibration

### 4. Multiple Calibrations
- Run level calibration 3 times
- Take average if values differ
- Consistency indicates good calibration

### 5. Backup Your Calibration
After good calibration, write down values:
```
Date: ___________
Gyro X: _______ Y: _______ Z: _______
Level X: _______ Y: _______
```
You can manually restore these if needed.

---

## 🔍 Troubleshooting Calibration Issues

### Issue: "Gyro calibration may be poor!"

**Possible Causes**:
- Drone moved during calibration
- Surface not level
- MPU6050 connection loose
- Defective MPU6050

**Solutions**:
1. Power off and restart
2. Use different, flatter surface
3. Check I2C connections (SDA, SCL)
4. Test MPU6050 with standalone sketch
5. Replace MPU6050 if defective

---

### Issue: Drone drifts even after level calibration

**Possible Causes**:
- Calibration surface wasn't level
- Motor mounts not level
- Bent frame
- Wind/air currents
- One motor weaker than others

**Solutions**:
1. Recalibrate on different surface
2. Use spirit level to check motor arms
3. Inspect frame for bends
4. Test indoors (no wind)
5. Check motor current draw (should be equal)
6. Balance propellers

---

### Issue: Motors don't respond correctly to tilt

**Possible Causes**:
- Wrong motor order
- Wrong rotation direction
- Incorrect frame type in code
- PID signs inverted

**Solutions**:
1. Verify motor order: FL=3, FR=5, RR=6, RL=9
2. Check motor rotation (props should pull air down)
3. Confirm X-configuration in code
4. Swap PID signs if needed (shouldn't be necessary)

---

### Issue: Calibration values change drastically each time

**Possible Causes**:
- MPU6050 not firmly mounted
- Loose connections
- Vibrations during calibration
- Defective sensor

**Solutions**:
1. Secure MPU6050 with double-sided foam tape
2. Check all wire connections
3. Isolate from vibrations
4. Replace MPU6050

---

## 📊 Calibration Quality Indicators

### Excellent Calibration ✅
- Level offsets: ±0.5°
- Gyro offsets: <±100
- Drone hovers with minimal stick input
- No drift in any direction
- Smooth, stable flight

### Good Calibration ✓
- Level offsets: ±1.0°
- Gyro offsets: <±200
- Minor drift, easily corrected
- Stable flight overall

### Acceptable Calibration ⚠
- Level offsets: ±2.0°
- Gyro offsets: <±500
- Noticeable drift
- Requires constant correction
- **Recommend re-calibrating**

### Poor Calibration ❌
- Level offsets: >±5.0°
- Gyro offsets: >±1000
- Severe drift
- Unstable, hard to control
- **DO NOT FLY - Recalibrate or check hardware**

---

## 🎓 Calibration Best Practices

1. **Calibrate in flying environment**
   - If you fly outdoors, calibrate outdoors
   - Temperature affects sensors

2. **Warm up period**
   - Let electronics stabilize for 2-3 minutes
   - Especially in cold weather

3. **Consistent surface**
   - Use same calibration spot each time
   - Mark the spot for consistency

4. **Regular maintenance**
   - Weekly level calibration
   - After every crash
   - After any maintenance

5. **Document your settings**
   - Keep log of calibration dates
   - Note values for reference
   - Track performance over time

6. **Test before flying**
   - Always verify with tilt test
   - Check serial monitor output
   - Arm with props OFF first

---

## ✅ Calibration Checklist

Use this before every flight session:

- [ ] Powered on in level position (gyro auto-calibrates)
- [ ] Level calibration done within last week
- [ ] Verification test passed (tilt response correct)
- [ ] No error messages in serial monitor
- [ ] "Radio: ✓ OK" showing
- [ ] ESCs calibrated (one-time, unless replaced)
- [ ] All motors spin at same speed at hover throttle
- [ ] Battery fully charged
- [ ] Weather suitable (no strong wind for beginners)

---

## 🎯 Final Calibration Tips

**Remember**:
- **More samples = Better calibration** (code uses 1500-2000 samples)
- **Stability is key** (don't rush, let it complete)
- **EEPROM saves it** (survives power cycles)
- **Re-calibrate after crashes** (impact can shift sensors)
- **Test before flying** (verification saves crashes)

**The extra 5 minutes for proper calibration can save hours of troubleshooting!**

---

**Your drone is now calibrated and ready for stable flight! 🚁**
