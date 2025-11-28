# Testing Procedures Guide

## Professional Drone Engineer Workflow

Complete testing procedures for validating your drone system before and during flight operations.

---

## 📋 Table of Contents

1. [Test Equipment Required](#test-equipment-required)
2. [Bench Tests (Props OFF)](#bench-tests-props-off)
3. [Communication Tests](#communication-tests)
4. [Sensor Validation](#sensor-validation)
5. [Motor Direction Tests](#motor-direction-tests)
6. [Control Direction Tests](#control-direction-tests)
7. [Failsafe Testing](#failsafe-testing)
8. [Pre-Flight Checklist](#pre-flight-checklist)
9. [First Flight Procedure](#first-flight-procedure)
10. [Post-Flight Inspection](#post-flight-inspection)

---

## 🧰 Test Equipment Required

| Equipment | Purpose |
|-----------|---------|
| Multimeter | Voltage/continuity checks |
| USB Cable | Programming and monitoring |
| Fully Charged Battery | Consistent power |
| Level Surface | Calibration accuracy |
| Prop Balancer | Vibration reduction |
| Computer with Serial Monitor | Debug output |
| Safety Glasses | Eye protection |
| Fire Extinguisher | LiPo safety |

---

## 🔧 Bench Tests (Props OFF)

### ⚠️ SAFETY: ALL BENCH TESTS MUST BE DONE WITH PROPS REMOVED

### Test 1: Power-On Self Test (POST)

```
PROCEDURE:
─────────

1. Connect battery with props REMOVED

2. Observe LED (D7):
   □ Single blink → Power OK
   □ Continuous blink → Waiting for link
   
3. Listen for buzzer:
   □ Single short beep → Initialization OK
   □ Double beep → Ready
   □ Multiple beeps → Error (count beeps for error code)

4. Check serial output (115200 baud):
   □ "FC: Initializing..."
   □ "FC: MPU6050 initialized"
   □ "FC: MS5611 initialized"
   □ "FC: NRF24 initialized"
   □ "FC: Ready"

PASS CRITERIA:
  ✓ All initialization messages present
  ✓ No error beeps
  ✓ LED blinks for link
```

### Test 2: Sensor Verification

```
PROCEDURE:
─────────

1. Enable DEBUG_SENSORS in config.h
2. Upload and open Serial Monitor

3. With drone level and stationary:
   □ Roll:  -2° to +2° (should read ~0°)
   □ Pitch: -2° to +2° (should read ~0°)
   □ Yaw:   Any value (no absolute reference)

4. Tilt drone 45° left:
   □ Roll should read ~45°

5. Tilt drone 45° forward:
   □ Pitch should read ~45°

6. Rotate drone 90° clockwise:
   □ Yaw should change by ~90°

7. Check barometer:
   □ Altitude reading stable (±5cm)
   □ Blow on sensor gently: altitude should change

PASS CRITERIA:
  ✓ Angles within ±5° of actual
  ✓ Barometer responds to pressure change
  ✓ Values stable when stationary
```

### Test 3: Calibration Test

```
PROCEDURE:
─────────

1. Place drone on perfectly level surface
2. Power on FC and RC
3. Wait for link (LED blinking fast)
4. Ensure ARM switch (SW2) is OFF
5. Press Button 1 on RC

6. Observe:
   □ Serial shows "Starting IMU calibration..."
   □ Wait ~2 seconds
   □ Two beeps → Calibration SUCCESS
   
   OR
   
   □ One long 7-second beep → Calibration FAILED
   □ Check serial for reason:
     - "Motion detected" → Drone moved during cal
     - "Values out of range" → Sensor issue

7. After successful calibration:
   □ Serial shows gyro offsets
   □ Offset values typically -50 to +50

PASS CRITERIA:
  ✓ Calibration succeeds on level surface
  ✓ Fails appropriately if moved during cal
  ✓ Gyro offsets are reasonable
```

---

## 📡 Communication Tests

### Test 4: Link Establishment

```
PROCEDURE:
─────────

1. Power on RC first
2. Open Serial Monitor on RC (115200 baud)
3. Observe: "LINK" column shows "LOST"

4. Power on FC
5. Wait up to 5 seconds

6. Observe:
   □ RC shows "LINK: OK"
   □ RC LED becomes solid
   □ FC LED blinks fast
   □ Link quality shown in telemetry

PASS CRITERIA:
  ✓ Link established within 5 seconds
  ✓ Link quality > 80%
  ✓ Both LEDs indicate link status correctly
```

### Test 5: Range Test

```
PROCEDURE:
─────────

1. Establish link at close range
2. Walk away with RC while monitoring FC LED

3. Check at various distances:
   □ 10m  → Still linked
   □ 50m  → Still linked
   □ 100m → Still linked (outdoor)
   
4. Monitor link quality in serial output
5. Note distance where quality drops below 50%

PASS CRITERIA:
  ✓ Reliable link at intended operating distance
  ✓ Clean failsafe when out of range
  ✓ Link re-establishes when in range
```

### Test 6: Telemetry Verification

```
PROCEDURE:
─────────

1. With link established, monitor RC serial output

2. Verify telemetry updates:
   □ Roll/Pitch/Yaw angles displayed
   □ Altitude value present
   □ Loop time shown (~2500 µs typical)
   □ Motor percentages all zero (not armed)

3. Tilt FC board:
   □ Telemetry angles should change
   
4. Raise/lower FC board:
   □ Altitude value should change

PASS CRITERIA:
  ✓ Telemetry updates at ~10 Hz
  ✓ Values match actual orientation
  ✓ All fields populated correctly
```

---

## 🔧 Motor Direction Tests

### ⚠️ PERFORM WITH PROPS REMOVED

### Test 7: Individual Motor Test

```
MOTOR LAYOUT (X-Config, viewed from above):
──────────────────────────────────────────

                FRONT
           FL (CCW)    FR (CW)
              ╲          ╱
               ╲        ╱
                ╲      ╱
                 ╲    ╱
                  ╲  ╱
                   ╲╱
                   ╱╲
                  ╱  ╲
                 ╱    ╲
                ╱      ╲
               ╱        ╲
              ╱          ╲
           RL (CW)     RR (CCW)
                REAR

PROCEDURE:
─────────

1. Ensure ARM switch is OFF
2. Press Button 2 on RC

3. Motors will spin one at a time:
   □ Beep 1 → FL spins → Mark direction
   □ Beep 2 → FR spins → Mark direction
   □ Beep 3 → RR spins → Mark direction
   □ Beep 4 → RL spins → Mark direction

4. Verify directions:
   □ FL = CCW (counter-clockwise from above)
   □ FR = CW  (clockwise from above)
   □ RR = CCW (counter-clockwise from above)
   □ RL = CW  (clockwise from above)

FIXING WRONG DIRECTION:
  • Swap any two of the three motor wires
  • Or change direction in ESC settings (BLHeli)

PASS CRITERIA:
  ✓ All four motors spin
  ✓ Spin directions match diagram
  ✓ No unusual motor sounds
```

### Test 8: ESC Calibration

```
PROCEDURE:
─────────

⚠️ PROPS MUST BE REMOVED

1. Power on RC only
2. Turn on FC while holding Button 2
3. Within 3 seconds, press Button 2

4. Sequence:
   □ Beep 1 → ESCs receiving max throttle
   □ Wait 3 seconds
   □ Beep 2 → ESCs receiving min throttle
   □ ESCs should beep confirmation tones
   □ Beep 3 → Calibration complete

5. Power cycle FC to exit calibration mode

PASS CRITERIA:
  ✓ ESC confirmation tones heard
  ✓ All motors respond equally after cal
```

---

## 🎮 Control Direction Tests

### Test 9: Control Response Test

```
PROCEDURE:
─────────

⚠️ PROPS REMOVED, MOTORS WILL SPIN

1. ARM the drone (SW2 ON, throttle low)
2. Listen for ARM beep
3. Monitor serial output for motor percentages

4. Minimum throttle test:
   □ Motors should idle at ~5%
   
5. Control tests (observe motor speed changes):

   ROLL RIGHT (stick right):
   □ FL & RL speed up
   □ FR & RR slow down
   
   ROLL LEFT (stick left):
   □ FR & RR speed up
   □ FL & RL slow down
   
   PITCH FORWARD (stick up):
   □ RL & RR speed up
   □ FL & FR slow down
   
   PITCH BACK (stick down):
   □ FL & FR speed up
   □ RL & RR slow down
   
   YAW RIGHT (left stick right):
   □ FL & RR speed up (CCW motors)
   □ FR & RL slow down (CW motors)
   
   YAW LEFT (left stick left):
   □ FR & RL speed up (CW motors)
   □ FL & RR slow down (CCW motors)

6. DISARM (SW2 OFF)

PASS CRITERIA:
  ✓ All control directions correct
  ✓ Response is smooth
  ✓ Returns to idle when stick centered
```

### Test 10: Attitude Response Test

```
PROCEDURE:
─────────

1. ARM the drone (props OFF)
2. Hold FC in hands, throttle at ~30%

3. Tilt drone left:
   □ Right motors should speed up
   □ Drone tries to level itself
   
4. Tilt drone forward:
   □ Rear motors should speed up
   □ Drone tries to level itself

5. Tilt drone in each direction:
   □ Response should be immediate
   □ Correction in correct direction
   □ No oscillation

6. DISARM

PASS CRITERIA:
  ✓ Self-leveling response correct
  ✓ Quick response without oscillation
  ✓ No unexpected behavior
```

---

## 🚨 Failsafe Testing

### Test 11: Link Loss Failsafe

```
PROCEDURE:
─────────

⚠️ PERFORM IN SAFE AREA

1. ARM the drone (props OFF or in safe enclosure)
2. Increase throttle to ~30%
3. Motors should spin

4. Power OFF the RC transmitter

5. Observe within 500ms:
   □ Motors should stop immediately
   □ Buzzer sounds failsafe alarm
   □ FC LED rapid blink
   □ Serial shows "FAILSAFE - Link lost!"

6. Power ON RC transmitter

7. Observe:
   □ Link re-establishes
   □ Failsafe clears (buzzer stops)
   □ Drone remains DISARMED (must re-arm)

PASS CRITERIA:
  ✓ Motors stop within 1 second of link loss
  ✓ Failsafe indication clear
  ✓ Must manually re-arm after failsafe
```

### Test 12: Kill Switch Test

```
PROCEDURE:
─────────

1. ARM the drone
2. Increase throttle to ~40%
3. Motors spinning

4. Flip SW2 to OFF (DISARM/KILL)

5. Observe:
   □ Motors stop IMMEDIATELY
   □ Disarm beep sounds
   □ LED shows not armed

6. Try to increase throttle:
   □ Motors should NOT respond

7. Re-ARM (SW2 ON):
   □ Only works with throttle LOW
   □ Arm beep sounds
   □ Motors resume at idle

PASS CRITERIA:
  ✓ Instant motor stop on kill switch
  ✓ Cannot accidentally throttle up when disarmed
  ✓ Re-arm requires throttle low
```

### Test 13: Tilt Protection Test

```
PROCEDURE:
─────────

1. ARM the drone (props OFF)
2. Hold FC, throttle at ~30%

3. Slowly tilt past 60°

4. Observe:
   □ Motors should cut immediately
   □ Failsafe buzzer sounds
   □ "FAILSAFE - Excessive tilt!" in serial

5. Return to level:
   □ Failsafe clears
   □ Must re-arm

PASS CRITERIA:
  ✓ Triggers at approximately 60° tilt
  ✓ Immediate motor cutoff
  ✓ Clear recovery path
```

---

## ✅ Pre-Flight Checklist

```
BEFORE EACH FLIGHT
══════════════════

MECHANICAL
□ Frame tight, no loose screws
□ Props secure and undamaged  
□ Props on correct motors (CCW/CW)
□ Battery secure
□ All wires tucked and secured

ELECTRICAL
□ Battery fully charged
□ All connections solid
□ No exposed wires
□ ESC wires away from props

SENSORS
□ Calibration successful today
□ No sensor errors on power-up
□ Attitude reading correct

COMMUNICATION
□ RC battery charged
□ Link established
□ Link quality > 80%
□ Telemetry updating

CONTROLS
□ All stick movements correct
□ Arm/disarm working
□ Alt-hold switch works (if using)

ENVIRONMENT
□ Weather suitable
□ Area clear of people
□ Away from airports/restricted zones
□ Spotter available (recommended)

SAFETY
□ Know kill switch location
□ Emergency plan if control lost
□ First aid kit nearby
□ Fire extinguisher available
```

---

## 🚁 First Flight Procedure

### Pre-Hover Tests (On Ground)

```
1. Place drone on level ground
2. Move 5 meters away
3. ARM the drone
4. Slowly raise throttle to 30%
5. Observe:
   □ Drone wants to lift evenly
   □ No tipping tendency
   □ Throttle response smooth
6. Lower throttle, DISARM
```

### First Hover

```
1. With props ON, ARM
2. Slowly raise throttle until light on skids
3. Maintain 0.5m hover
4. Check:
   □ Stable hover (may need trim)
   □ No toilet-bowl oscillation  
   □ Responds to stick inputs correctly
5. Practice landing gently
6. Repeat until comfortable
```

### First Controlled Flight

```
1. Hover at 1m
2. Small roll left, return to center
3. Small roll right, return to center
4. Small pitch forward, return
5. Small pitch back, return
6. Small yaw left, return
7. Small yaw right, return
8. Land and assess
```

---

## 🔍 Post-Flight Inspection

```
AFTER EACH FLIGHT
═════════════════

IMMEDIATE
□ DISARM confirmed
□ Props stopped
□ Battery disconnect

INSPECTION
□ Props - any chips or cracks?
□ Motors - any heat (excessive = problem)
□ Frame - any damage?
□ Battery - any puffing?
□ Wires - any burns or melting?

LOGGING
□ Flight duration
□ Any issues observed
□ Battery voltage after flight
□ Changes needed?
```

---

## 📋 Troubleshooting Reference

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| No power-on LED | Power connection | Check battery/wiring |
| Sensors not init | I2C issue | Check wiring, addresses |
| No link | NRF issue | Check wiring, channel |
| Motor won't spin | ESC issue | Recalibrate ESC |
| Wrong motor dir | Wire swap | Swap 2 motor wires |
| Unstable hover | PID tuning | Follow tuning guide |
| Drifts in flight | Calibration | Recalibrate on level surface |
| Toilet bowl | AHRS issue | Check for magnetic interference |
| Failsafe random | Signal issue | Check antenna, range |

---

*Safe Flying! Always respect the machine and your surroundings.*
