# Testing and Calibration Procedures

## 📋 Table of Contents

- [Overview](#overview)
- [Pre-Flight Checklist](#pre-flight-checklist)
- [Bench Testing Procedures](#bench-testing-procedures)
- [Calibration Procedures](#calibration-procedures)
- [Motor Direction Verification](#motor-direction-verification)
- [ESC Calibration](#esc-calibration)
- [Failsafe Testing](#failsafe-testing)
- [First Flight Test](#first-flight-test)
- [Troubleshooting Failed Tests](#troubleshooting-failed-tests)

---

## 🎯 Overview

This document provides comprehensive testing procedures to ensure your quadcopter is safe and ready to fly. **Follow these steps in order** before attempting your first flight.

### Testing Phases

1. **Bench Tests** - Verify electronics without propellers
2. **Calibration** - Configure sensors and ESCs
3. **Motor Tests** - Verify correct motor operation
4. **Failsafe Tests** - Ensure safety systems work
5. **First Flight** - Controlled maiden flight

**⚠️ SAFETY FIRST**: Always remove propellers during bench testing!

---

## ✅ Pre-Flight Checklist

### Hardware Inspection

- [ ] All screws tight (frame, motors, ESCs)
- [ ] No loose wires or dangling components
- [ ] Battery secured with strap/velcro
- [ ] Propellers NOT installed (bench testing)
- [ ] No damaged components (cracked frame, bent motor shaft)
- [ ] NRF24L01 antennas clear of carbon fiber
- [ ] ESC wires properly insulated (no shorts)
- [ ] Battery connector polarity correct (red=+, black=-)

### Power System

- [ ] Battery fully charged (12.6V for 3S LiPo)
- [ ] Battery voltage displayed correctly (if using voltage monitor)
- [ ] BEC providing 5V to Arduino Nano
- [ ] All grounds connected (common ground)
- [ ] No reversed polarity anywhere

### Software

- [ ] FlightController_FC.ino uploaded to FC Arduino
- [ ] RemoteController_RC.ino uploaded to RC Arduino
- [ ] RF24 library installed (v1.4.7+)
- [ ] Serial monitors work (115200 baud)
- [ ] No compilation errors or warnings

---

## 🔬 Bench Testing Procedures

### Test 1: Power-On Test

**Goal**: Verify all components power on without magic smoke.

**Procedure**:

1. **Disconnect battery** from FC
2. **Connect RC** to USB power
   - Serial monitor should show RC boot sequence
   - "NRF24L01 initialized successfully"
   - Status display showing joystick values

3. **Connect FC** to USB power (NO battery yet)
   - Serial monitor should show FC boot sequence
   - "MPU6050 OK", "MS5611 OK", "NRF24L01 OK"
   - Base altitude displayed

4. **Check LED**:
   - FC LED (D7) should blink when RC is powered on
   - Indicates NRF link established

5. **Disconnect USB** from FC

6. **Connect battery** to FC (propellers still OFF)
   - Listen for 2 beeps (startup complete)
   - LED should blink (link active)

**Expected Results**:
- ✅ No smoke, sparks, or burning smell
- ✅ Both systems boot successfully
- ✅ NRF link established (LED blinking)
- ✅ Serial outputs show valid sensor data

**If Failed**: See [Troubleshooting](#troubleshooting-failed-tests)

---

### Test 2: Communication Test

**Goal**: Verify reliable RC → FC communication.

**Procedure**:

1. Power on RC and FC (previous test)
2. Open both serial monitors (RC and FC)

3. **RC Serial Monitor**:
   - Check "LINK STATUS: ✓ CONNECTED"
   - Move joysticks - values should change (1000-2000)
   - Toggle switches - states should update
   - Press buttons - status should show [PRESSED]

4. **FC Serial Monitor** (if you added telemetry debug):
   - Should show received RC data
   - Timestamp should update continuously

5. **Move away test**:
   - Walk 10m away with RC
   - Link should remain stable
   - If using PA+LNA modules, range should be 100m+

6. **Interference test**:
   - Power on WiFi router nearby
   - Enable Bluetooth on phone
   - Link should remain stable (channel 103 is relatively clear)

**Expected Results**:
- ✅ Link status shows CONNECTED continuously
- ✅ Joystick values update in real-time
- ✅ No dropped packets or link resets
- ✅ Range adequate for intended flight area

---

### Test 3: IMU Response Test

**Goal**: Verify MPU6050 is reading correctly.

**Procedure**:

1. Power on FC (via USB or battery)
2. Open FC serial monitor
3. **Tilt test**:
   - Tilt FC forward - pitch should increase
   - Tilt FC backward - pitch should decrease
   - Tilt FC left - roll should decrease (negative)
   - Tilt FC right - roll should increase (positive)
   - Rotate FC - yaw should change

4. **Return to level**:
   - Place FC flat on table
   - Roll and pitch should return to ~0° (±5° tolerance before calibration)

5. **Static test**:
   - Leave FC stationary for 10 seconds
   - Attitude values should be stable (not drifting rapidly)

**Expected Results**:
- ✅ Tilt directions match expected behavior
- ✅ Attitude returns to ~0° when level
- ✅ No rapid drift (< 1°/sec before calibration)
- ✅ Values stable when stationary

**If Failed**: 
- Check MPU6050 wiring (SDA=A4, SCL=A5)
- Verify I2C communication (should see "MPU6050 OK" at boot)
- Try different MPU6050 module (some are faulty)

---

### Test 4: Barometer Test

**Goal**: Verify MS5611 altitude readings.

**Procedure**:

1. Power on FC
2. Open serial monitor
3. Check "Base Altitude" value at boot (should be reasonable, e.g., 100-2000m)

4. **Altitude change test**:
   - Note current altitude
   - Lift FC 1 meter higher
   - Altitude should increase by ~1.0m (±0.3m tolerance)
   - Lower FC back down
   - Altitude should return to original value

5. **Stability test**:
   - Place FC on table for 30 seconds
   - Altitude should vary by < 0.5m
   - No rapid jumps or glitches

**Expected Results**:
- ✅ Base altitude is reasonable (not 0 or 9999)
- ✅ Altitude changes proportionally to height change
- ✅ Altitude stable when stationary
- ✅ No error messages ("MS5611 OK" at boot)

**If Failed**:
- Check MS5611 wiring (I2C: SDA=A4, SCL=A5)
- Verify MS5611 is not faulty (common issue)
- Ensure MS5611 pressure port is not blocked
- Check for correct I2C address (0x77 or 0x76)

---

## 🎯 Calibration Procedures

### Calibration 1: IMU Calibration

**Goal**: Zero out gyro/accel offsets for accurate attitude estimation.

**Procedure**:

1. **Prepare surface**:
   - Find perfectly level surface (use spirit level)
   - Table, floor, or workbench
   - Surface should be stable (no vibrations)

2. **Power on systems**:
   - RC on first
   - FC on second
   - Wait for link (LED blinking)

3. **Position drone**:
   - Place drone on level surface
   - Ensure it's not tilted in any axis
   - Do NOT touch drone during calibration

4. **Trigger calibration**:
   - Press **Button_1** on RC
   - FC will collect 2000 samples (~4 seconds)
   - Serial monitor shows progress dots

5. **Listen for result**:
   - **2 short beeps**: ✅ Success
   - **1 long beep (7 sec)**: ❌ Failed

6. **Verify**:
   - RC serial monitor shows "CALIBRATION: ✓ COMPLETE"
   - FC serial monitor shows gyro/accel offsets

**Expected Offsets**:
- Accel: ±2.0 m/s² (reasonable)
- Gyro: ±0.1 rad/s (reasonable)

**If Failed**:
- Offsets will be HUGE (>5.0 for accel, >0.5 for gyro)
- Drone was moving or tilted during calibration
- Surface was not level
- Vibrations from nearby equipment
- **Solution**: Repeat calibration on stable, level surface

**Recalibration**:
- Recalibrate before every flight session
- Recalibrate if drone was transported or handled roughly
- Recalibrate if behavior seems off

---

### Calibration 2: Joystick Calibration (Automatic)

**Goal**: Find center points and deadzones for joysticks.

**Procedure**:

1. **Before powering on RC**:
   - Center both joysticks (neutral position)
   - Release joysticks (should spring to center)

2. **Power on RC**:
   - RC automatically calibrates during boot
   - Takes 50 samples over 0.5 seconds
   - Serial monitor shows center values

3. **Verify**:
   - RC serial monitor shows calibration values
   - Yaw/Pitch/Roll centers should be ~512 (±50)
   - Move joysticks - values should change to 1000-2000

4. **Test deadzone**:
   - Release joysticks to center
   - Values should jump to exactly 1500 (deadzone applied)
   - Small movements near center should be ignored

**Expected Results**:
- ✅ Center values ~512 (analog range 0-1023)
- ✅ Full stick deflection gives 1000-2000 range
- ✅ Deadzone eliminates stick drift at center

**If Failed**:
- Joystick centers are way off (< 400 or > 600)
- Joysticks may be damaged or low quality
- Try different joystick modules
- Manually adjust `joyYaw.center`, etc. in RC code

---

## 🔄 Motor Direction Verification

### Test: Individual Motor Spin Direction

**Goal**: Ensure each motor spins in correct direction for X configuration.

**Setup**:
- ⚠️ **REMOVE ALL PROPELLERS** before this test!
- Power on RC and FC
- Complete IMU calibration first

**Procedure**:

1. **Trigger motor test**:
   - Ensure **SW_2 (ARM) is OFF**
   - Press **Button_2** on RC
   - FC enters ESC calibration + motor test mode

2. **ESC calibration sequence** (automatic):
   - All ESCs receive full throttle (2000) for 3 seconds
   - Then minimum throttle (1000) for 2 seconds
   - Listen for ESC beeps (confirms calibration)

3. **Motor test sequence**:
   - Each motor spins individually for 1.5 seconds
   - Order: FL → FR → RR → RL
   - Observe spin direction of each motor

4. **Expected directions** (looking down at drone):

```
        FRONT
          ↑
    FL(⟲)    FR(⟳)
       \ X /
       / X \
    RL(⟳)    RR(⟲)
          ↓
        BACK

FL: Counter-clockwise (CW in diagram = wrong!)
FR: Clockwise
RR: Counter-clockwise
RL: Clockwise
```

**How to Check**:
- Look at motor from above
- Note which way it spins
- Compare to diagram above

**Correcting Wrong Direction**:

If a motor spins the wrong way:

**Option A: Swap any 2 motor wires**
- Disconnect motor from ESC
- Swap any 2 of the 3 wires
- Reconnect
- Retest

**Option B: Change BLHeli ESC direction** (if ESCs support it)
- Use BLHeli configurator
- Connect ESC to PC
- Toggle "Reversed" setting
- Reflash ESC

**Verification**:
- Re-run motor test (Button_2)
- All motors should now spin correct direction
- 3 beeps at end indicates test complete

---

## ⚙️ ESC Calibration

### Full ESC Calibration (if motors don't spin)

**When to do this**:
- First time setup
- After changing ESCs
- Motors don't respond to throttle
- Motor startup is inconsistent

**Procedure** (Manual):

1. **Power OFF FC** (disconnect battery)

2. **Connect FC to computer** (USB)

3. **Open FC serial monitor** (115200 baud)

4. **Modify code temporarily** (optional for advanced users):
   ```cpp
   // In setup(), add:
   motorFL = motorFR = motorRR = motorRL = MOTOR_MAX; // 2000
   writeMotors();
   delay(3000);
   motorFL = motorFR = motorRR = motorRL = MOTOR_MIN; // 1000
   writeMotors();
   delay(2000);
   ```

5. **Upload modified code**

6. **Connect battery while code uploads**:
   - ESCs will receive full throttle immediately
   - ESCs beep rapidly (programming mode)
   - After 3 seconds, throttle drops to minimum
   - ESCs confirm calibration with beep sequence

7. **Remove temporary code**, re-upload normal firmware

**Easier Method** (Recommended):

- Just use **Button_2** on RC (as described above)
- Automatic ESC calibration + motor test

---

## 🛡️ Failsafe Testing

### Test 1: Link Loss Failsafe

**Goal**: Verify motors cut when NRF link is lost.

**Setup**:
- Propellers OFF
- Drone armed and motors spinning (low throttle)

**Procedure**:

1. **Arm drone**:
   - SW_2 = ON
   - Increase throttle slightly (motors spinning)

2. **Simulate link loss**:
   - **Option A**: Turn OFF RC
   - **Option B**: Remove power from RC NRF24L01
   - **Option C**: Cover RC antenna with aluminum foil

3. **Observe**:
   - FC should detect timeout after 500ms
   - Motors should cut immediately
   - LED (D7) should turn OFF (no link)

4. **Restore link**:
   - Turn RC back ON (or remove foil)
   - LED should blink (link restored)

**Expected Results**:
- ✅ Motors cut within 1 second of link loss
- ✅ LED turns OFF (no link)
- ✅ Link restores automatically when RC back on
- ✅ Must re-arm to spin motors again (safety)

---

### Test 2: Disarm Switch (Kill Switch)

**Goal**: Verify SW_2 immediately stops motors.

**Setup**:
- Propellers OFF
- Drone armed and motors spinning

**Procedure**:

1. **Arm drone**:
   - SW_2 = ON
   - Increase throttle (motors spinning)

2. **Toggle SW_2 to OFF**:
   - Flip switch immediately

3. **Observe**:
   - Motors should cut instantly (< 100ms)
   - RC status shows "[DISARMED]"

4. **Try to spin motors**:
   - SW_2 still OFF
   - Increase throttle
   - Motors should NOT spin

5. **Re-arm**:
   - SW_2 = ON
   - Increase throttle
   - Motors should spin again

**Expected Results**:
- ✅ Motors stop immediately when disarmed
- ✅ Motors cannot spin while disarmed
- ✅ Can re-arm and resume control

---

### Test 3: Uncalibrated Safety

**Goal**: Verify motors blocked until calibration complete.

**Procedure**:

1. **Power cycle FC** (motors forget calibration status)

2. **Try to arm**:
   - SW_2 = ON
   - Increase throttle

3. **Observe**:
   - Motors should NOT spin
   - RC status shows "CALIBRATION: ✗ NOT CALIBRATED"

4. **Calibrate** (Button_1)

5. **Try again**:
   - SW_2 = ON
   - Increase throttle
   - Motors should now spin

**Expected Results**:
- ✅ Motors blocked before calibration
- ✅ Motors allowed after calibration
- ✅ Safety prevents flying with bad sensor data

---

## 🚁 First Flight Test

### Pre-Flight Setup

- [ ] All previous tests passed
- [ ] Propellers installed **correctly** (CW on CW motors, CCW on CCW motors)
- [ ] Propellers tight (but not over-tightened)
- [ ] Battery fully charged (12.6V)
- [ ] Flight area clear (no people, obstacles)
- [ ] Soft landing surface (grass preferred)
- [ ] RC kill switch (SW_2) easily accessible

### Flight Test 1: Hover Test

**Goal**: Verify stable hover without propellers flying off.

**Procedure**:

1. **Position drone**:
   - On flat ground
   - At least 3m away from you
   - Orient FRONT facing away

2. **Power on sequence**:
   - RC on
   - FC on (2 beeps)
   - Verify link (LED blinking)
   - SW_2 = OFF

3. **Calibrate**:
   - Press Button_1
   - Wait for 2 beeps (success)

4. **Arm**:
   - SW_2 = ON
   - Throttle to minimum

5. **Slowly increase throttle**:
   - Gradual stick movement
   - Watch for liftoff (~1400-1600 throttle)
   - Lift to 30cm height

6. **Hover for 10 seconds**:
   - Sticks centered (except throttle)
   - Observe stability
   - Listen for unusual sounds

7. **Land**:
   - Slowly reduce throttle
   - Touchdown gently
   - SW_2 = OFF immediately

**Expected Behavior**:
- ✅ Smooth liftoff, no flips
- ✅ Stable hover without drifting
- ✅ Responds to small stick inputs
- ✅ No vibrations or oscillations
- ✅ Props don't fly off

**If Problems**: See [Troubleshooting Flight Issues](#troubleshooting-flight-issues)

---

### Flight Test 2: Control Response Test

**Goal**: Verify all control axes work correctly.

**Procedure**:

1. **Hover at 1m height**

2. **Test Roll**:
   - Push right stick RIGHT
   - Drone should tilt RIGHT and move right
   - Release stick - drone should level out

3. **Test Pitch**:
   - Push right stick UP
   - Drone should tilt FORWARD and move forward
   - Release stick - drone should level out

4. **Test Yaw**:
   - Push left stick LEFT
   - Drone should rotate counter-clockwise
   - Release stick - rotation should stop

5. **Test Throttle**:
   - Increase left stick UP
   - Drone should climb
   - Decrease left stick DOWN
   - Drone should descend

6. **Combinations**:
   - Try gentle figure-8 pattern
   - Test all controls simultaneously

**Expected Behavior**:
- ✅ All axes respond correctly (no reversed channels)
- ✅ Self-levels when sticks released
- ✅ Smooth control, no abrupt movements
- ✅ Predictable behavior

---

### Flight Test 3: Altitude Hold Test

**Goal**: Verify altitude hold mode works.

**Procedure**:

1. **Hover at 1.5m height** (manual throttle)

2. **Enable altitude hold**:
   - SW_1 = ON
   - Release throttle stick to center

3. **Observe for 30 seconds**:
   - Drone should maintain height
   - Slight variations (±0.3m) are normal

4. **Push down test**:
   - Gently push down on drone (hand)
   - Release
   - Should return to target altitude

5. **Pitch/Roll test**:
   - While altitude hold active
   - Use pitch/roll to fly around
   - Altitude should stay constant

6. **Disable altitude hold**:
   - SW_1 = OFF
   - Resume manual throttle control

**Expected Behavior**:
- ✅ Altitude stable in hold mode
- ✅ Returns to target after disturbance
- ✅ Pitch/roll control still works
- ✅ Smooth transition in/out of altitude hold

**If Failed**: See [TUNING_GUIDE.md](TUNING_GUIDE.md) - Altitude PID section

---

## 🔧 Troubleshooting Failed Tests

### Problem: No NRF Link (LED not blinking)

**Check**:
1. NRF24L01 wiring (CE, CSN, MOSI, MISO, SCK)
2. NRF24L01 power (3.3V, NOT 5V!)
3. Capacitor across NRF24L01 power pins (10µF)
4. Both NRF modules on same channel (103)
5. Try different NRF24L01 modules (some are DOA)

---

### Problem: IMU Calibration Always Fails

**Check**:
1. Surface is actually level (use spirit level)
2. No vibrations (turn off nearby equipment)
3. Drone is stationary (don't touch during calibration)
4. MPU6050 wiring correct (I2C: SDA=A4, SCL=A5)
5. Try different MPU6050 module

**Debug**:
- Check FC serial output for offset values
- If offsets > 5.0 (accel) or > 0.5 (gyro), something is wrong

---

### Problem: Motors Don't Spin

**Check**:
1. ESC calibration complete (Button_2)
2. IMU calibration complete (Button_1)
3. SW_2 = ON (armed)
4. Throttle above minimum (> 1100)
5. ESC BEC providing 5V to Arduino
6. ESC signal wires connected (FL=D3, FR=D5, RR=D6, RL=D9)

**Debug**:
- Test ESCs with servo tester
- Check PWM signals with oscilloscope/logic analyzer

---

### Problem: Drone Flips on Takeoff

**Causes**:
1. Motor spinning wrong direction
2. Propeller on wrong motor (CW prop on CCW motor)
3. Motor wired to wrong pin
4. IMU mounted upside-down or rotated

**Solution**:
- Verify motor directions (see Motor Direction Verification)
- Check propeller orientation
- Re-check ESC wiring

---

### Problem: Oscillations During Flight

**Causes**:
- PID gains too high (usually rate P or D)
- Mechanical vibrations
- Loose components

**Solution**:
- See [TUNING_GUIDE.md](TUNING_GUIDE.md)
- Reduce rate PID gains by 20%
- Tighten all screws
- Add vibration dampening

---

### Problem: Drifts in One Direction

**Causes**:
- IMU not calibrated correctly
- CG (center of gravity) offset
- Motor thrust imbalance
- Wind

**Solution**:
- Recalibrate IMU on level surface
- Check CG is centered
- Increase rate I gain slightly

---

## 📊 Test Results Log

Keep a log of test results:

```
Date: ___________

Pre-Flight Checklist: [ ] Pass [ ] Fail
  Notes: _________________________________

Bench Test 1 (Power-On): [ ] Pass [ ] Fail
  Notes: _________________________________

Bench Test 2 (Communication): [ ] Pass [ ] Fail
  Link Range: ________m

Bench Test 3 (IMU Response): [ ] Pass [ ] Fail
  Notes: _________________________________

Bench Test 4 (Barometer): [ ] Pass [ ] Fail
  Base Altitude: ________m

IMU Calibration: [ ] Pass [ ] Fail
  Accel Offsets: X=____ Y=____ Z=____
  Gyro Offsets:  X=____ Y=____ Z=____

Motor Direction Test: [ ] Pass [ ] Fail
  FL: [ ] CW [ ] CCW
  FR: [ ] CW [ ] CCW
  RR: [ ] CW [ ] CCW
  RL: [ ] CW [ ] CCW

Failsafe Test (Link Loss): [ ] Pass [ ] Fail
Failsafe Test (Kill Switch): [ ] Pass [ ] Fail
Failsafe Test (Uncalibrated): [ ] Pass [ ] Fail

First Flight (Hover): [ ] Pass [ ] Fail
  Hover Throttle: ________
  Notes: _________________________________

First Flight (Control Response): [ ] Pass [ ] Fail
  Notes: _________________________________

First Flight (Altitude Hold): [ ] Pass [ ] Fail
  Notes: _________________________________

Overall Status: [ ] READY TO FLY [ ] NEEDS WORK
```

---

## ✅ Final Validation

Before regular flights, ensure:

- [ ] All bench tests passed
- [ ] IMU calibration successful
- [ ] Motor directions verified
- [ ] ESC calibration complete
- [ ] All failsafe tests passed
- [ ] First hover test stable
- [ ] Control response correct
- [ ] Altitude hold working (if using)
- [ ] No loose components
- [ ] Propellers secure and undamaged

**Only proceed to regular flights after ALL tests pass!**

---

## 🎓 Summary

Testing order:
1. Power-on → Communication → IMU → Barometer
2. Calibrate IMU
3. Verify motor directions
4. Calibrate ESCs
5. Test failsafes
6. First flight (hover → control → altitude hold)

**Time Required**: 2-3 hours for complete testing

**Safety Reminder**: 
- Propellers OFF for ALL bench tests
- Kill switch always ready during flights
- Test in open area away from people

---

**Good luck and fly safe! 🚁**
