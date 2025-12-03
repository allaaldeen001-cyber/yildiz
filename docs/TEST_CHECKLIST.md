# ✅ Pre-Flight Test Checklist & Safety Guide

## Safety First! ⚠️

### Critical Safety Rules

1. **NEVER** test with props on indoors
2. **ALWAYS** remove props for motor tests
3. **ALWAYS** test in open area (>10m clearance all directions)
4. **ALWAYS** have a spotter during first flights
5. **NEVER** fly near people, animals, or property
6. **ALWAYS** check weather (no wind >15 km/h for first flights)
7. **ALWAYS** have a way to quickly disarm (remove throttle stick)

### Emergency Procedures

| Emergency | Action |
|-----------|--------|
| Loss of control | Drop throttle to zero → auto-disarm |
| Motor failure | Drop throttle → land immediately |
| Radio failure | Auto-disarm after 1 second |
| Fly-away | Drop throttle, approach carefully after landing |
| Smoke/fire | Drop throttle, disconnect battery, move away |

---

## Hardware Checks

### Before Every Flight

- [ ] **Battery voltage** >11.1V (3S) or >14.8V (4S)
- [ ] **Props secure** and correctly oriented
  - FL & RR: CCW props
  - FR & RL: CW props
- [ ] **Motor screws tight** (no wobble)
- [ ] **Frame screws tight** (no loose arms)
- [ ] **Wiring secure** (no exposed wires, good solder joints)
- [ ] **Antenna positioned** (perpendicular to flight direction)
- [ ] **SD card removed** (can cause noise in sensors)

### Props Orientation Check

```
   FRONT
    (↑)
     
FL  (CCW)  FR  (CW)
     ╲    ╱
      ╲  ╱
       ╳
      ╱  ╲
     ╱    ╲
RL  (CW)   RR  (CCW)
```

**How to check**:
1. View prop from top
2. FL & RR: Leading edge rotates counter-clockwise
3. FR & RL: Leading edge rotates clockwise

---

## Software Checks

### Initial Setup (One Time)

- [ ] **Libraries installed**:
  - Adafruit_MPU6050
  - Adafruit_Sensor
  - MS5611 (Rob Tillaart)
  - RF24 (TMRh20)
- [ ] **Code compiles** without errors
- [ ] **FlightController uploaded** to FC Arduino
- [ ] **RemoteController uploaded** to RC Arduino
- [ ] **Serial Monitor works** (115200 baud)

### Before Every Flight

- [ ] **Serial Monitor check** (FC):
  ```
  ✅ MPU6050 initialized
  ✅ MS5611 initialized
  ✅ Radio initialized
  ✅ Initialization complete!
  📡 Waiting for radio connection...
  ```

- [ ] **Radio connection** confirmed:
  ```
  ✅ Radio connected
  Throttle: 500 | Roll: 0 | Pitch: 0 | Yaw: 0
  ```

- [ ] **Sensor readings** valid:
  ```
  Mode:DISARM | R:0.2 P:-0.5 | Alt:0cm V:0cm/s
  ```
  - Roll/Pitch within ±2° when level
  - Altitude = 0 ± 10 cm

- [ ] **Calibration** done (if moved location or temp changed):
  - Press Button 1 (Calibrate)
  - Keep drone level for 3 seconds
  - Check Serial: `✅ Calibration complete`

---

## Ground Tests (Props OFF)

### Test 1: Motor Direction Test

**Purpose**: Verify correct motor directions and ESC wiring

**Procedure**:
1. **Remove ALL propellers** ⚠️
2. Power on FC + RC
3. Keep drone disarmed
4. Press **Button 2** (Motor Test)
5. Each motor spins for 2 seconds

**Expected results**:
- Motor FL (D3): Spins alone for 2s
- Motor FR (D5): Spins alone for 2s
- Motor RR (D6): Spins alone for 2s
- Motor RL (D9): Spins alone for 2s
- 2 beeps at end

**Check**: View motor from top, verify rotation:
- FL & RR: Counter-clockwise (CCW)
- FR & RL: Clockwise (CW)

**If wrong direction**: Swap any 2 motor wires on that ESC

---

### Test 2: Radio Range Test

**Purpose**: Verify radio communication range

**Procedure**:
1. Power on FC + RC
2. Walk away with RC while watching FC Serial Monitor
3. Note distance when "RADIO SIGNAL LOST" appears

**Expected**: >20 meters indoors, >50 meters outdoors

**If fails**:
- Check nRF24L01+ has 10µF capacitor
- Check antenna is not damaged
- Check 3.3V power to nRF24L01+

---

### Test 3: Control Input Test

**Purpose**: Verify all sticks, buttons, and switches work

**Procedure**:
1. Power on FC + RC
2. Open RC Serial Monitor (if available) or FC telemetry
3. Move each control and verify response:

| Control | Expected Serial Output |
|---------|------------------------|
| Throttle up | `Throttle: 1000` |
| Throttle down | `Throttle: 0` |
| Roll right | `Roll: +500` |
| Roll left | `Roll: -500` |
| Pitch forward | `Pitch: +500` |
| Pitch back | `Pitch: -500` |
| Yaw right | `Yaw: +500` |
| Yaw left | `Yaw: -500` |
| SW1 toggle | `SW1: ON` / `SW1: OFF` |
| SW2 toggle | `SW2: ANGLE` / `SW2: ACRO` |
| Button 1 press | `🔧 Calibrating sensors...` |
| Button 2 press | `🔊 MOTOR TEST` |

---

## First Flight Tests (Props ON)

### Safety Setup

- [ ] **Location**: Open field, >10m from obstacles
- [ ] **Weather**: Calm (<10 km/h wind), no rain
- [ ] **Spotter**: Someone watching for issues
- [ ] **Landing pad**: Flat, level surface marked
- [ ] **Emergency plan**: Know how to disarm (throttle to zero)

---

### Test 4: Hover Test (Manual)

**Purpose**: Verify basic flight stability

**Procedure**:
1. Place drone on level ground
2. Stand 5m away
3. Power on FC + RC
4. Verify Serial: `Mode:DISARM`
5. Set SW2 to **ANGLE mode** (auto-level)
6. Set SW1 **OFF** (manual throttle)
7. **Slowly** increase throttle until drone lifts (~30cm)
8. Hold hover for 10 seconds
9. **Slowly** decrease throttle to land
10. Remove throttle to disarm

**Expected**:
- Drone lifts off smoothly
- Hovers relatively level (±10° tilt)
- Responds to small stick inputs
- Lands gently

**If issues**:
- **One motor slow**: Check ESC calibration
- **Oscillates fast**: Reduce rate Kp (see PID guide)
- **Drifts horizontally**: Check center of gravity (battery position)
- **Wobbles slowly**: Reduce rate Ki

---

### Test 5: Altitude Hold Test

**Purpose**: Verify barometer and altitude PID

**Procedure**:
1. Hover manually at 50cm (from Test 4)
2. Set SW1 **ON** → **ALT_HOLD mode**
3. Release throttle stick to center
4. Observe drone for 30 seconds

**Expected**:
- Drone maintains altitude ±10 cm
- No bobbing or bouncing
- Throttle stick adjusts target altitude slowly

**If issues**:
- **Drifts up/down slowly**: Adjust altitude Ki (see PID guide)
- **Bounces up/down**: Reduce altitude Kp, increase Kd
- **Doesn't hold altitude**: Check barometer in Serial Monitor

---

### Test 6: Smooth Takeoff Test

**Purpose**: Verify automatic takeoff system

**Procedure**:
1. Place drone on level ground
2. Ensure disarmed (`Mode:DISARM`)
3. Press **Button 4** (Smooth Takeoff)
4. Watch Serial Monitor

**Expected Serial output**:
```
🚁 AUTOMATIC TAKEOFF - ARM + LAUNCH
  Target: 150 cm
[Beep]

Mode:TKOFF | Alt:30cm V:+25cm/s
Mode:TKOFF | Alt:75cm V:+28cm/s
Mode:TKOFF | Alt:120cm V:+22cm/s
Mode:TKOFF | Alt:148cm V:+8cm/s

✅ Takeoff complete → ALT HOLD
[2 beeps]

Mode:ALT_H | Alt:150cm V:0cm/s
```

**Expected behavior**:
- Smooth acceleration (no jerk)
- Steady climb to 150 cm
- Smooth deceleration near target
- Stable hover at 150 cm

**If issues**:
- **Jerky takeoff**: Check altitude PID Kd
- **Overshoots target**: Increase altitude Kd
- **Too slow**: Increase `TAKEOFF_CLIMB_RATE` in code

---

### Test 7: Smooth Landing Test

**Purpose**: Verify automatic landing system (main feature!)

**Procedure**:
1. Drone hovering at 150 cm (from Test 6)
2. Press **Button 3** (Smooth Landing)
3. Watch Serial Monitor
4. Observe drone descent

**Expected Serial output**:
```
🛬 AUTOMATIC LANDING INITIATED
  Current altitude: 148.5 cm
  State: INITIATED → DESCENDING

Mode:LAND | Alt:132cm V:-42cm/s | LS:DESC
Mode:LAND | Alt:98cm V:-45cm/s | LS:DESC
Mode:LAND | Alt:42cm V:-18cm/s | LS:NEAR

✅ TOUCHDOWN DETECTED!
  Altitude: 12.3 cm | Velocity: 8.5 cm/s
  State: TOUCHDOWN → SAFE IDLE

Mode:LAND | Alt:0cm V:-2cm/s | LS:SAFE

✅ LANDING COMPLETE - DISARMED
[3 beeps]
```

**Expected behavior**:
- **Smooth descent** (no sudden drops)
- **Motors audible** throughout (never silent until touchdown)
- **Level attitude** (tilt <15°)
- **Deceleration near ground** (<50 cm)
- **Gentle touchdown** (no bounce)
- **Motors stop** after 0.5s safe idle
- **Auto-disarm** with 3 beeps

**Critical checks**:
- [ ] Descent rate 30-50 cm/s (not too fast)
- [ ] Motors spinning throughout descent (listen!)
- [ ] Transition to NEAR_GROUND at ~50 cm
- [ ] Touchdown detection at <15 cm
- [ ] No hard impact

**If issues**:
- **Drops suddenly**: Check minimum throttle in code
- **Too fast**: Reduce `LANDING_DESCENT_RATE_MAX`
- **Bounces**: Verify SAFE_IDLE state in Serial
- **Never lands**: Check barometer readings
- **Oscillates during descent**: Reduce altitude Kp, increase Kd

---

## Advanced Tests (After Successful Basic Tests)

### Test 8: Failsafe Test

**Purpose**: Verify radio failsafe works

**Procedure**:
1. Hover at 50 cm in ANGLE mode
2. Turn off RC transmitter
3. Observe drone

**Expected**:
- Drone disarms within 1 second
- Falls to ground (low height!)
- Serial shows: `❌ RADIO SIGNAL LOST - FAILSAFE!`

**Important**: Do at LOW altitude only!

---

### Test 9: Multiple Landing Cycles

**Purpose**: Verify system reliability

**Procedure**:
1. Repeat Tests 6 & 7 five times:
   - Button 4: Takeoff
   - Button 3: Land
   - Wait 10 seconds
   - Repeat

**Expected**:
- Consistent behavior each time
- No degradation in performance
- Ground reference remains accurate

---

### Test 10: Wind Test

**Purpose**: Verify landing in wind

**Procedure**:
1. Wait for mild wind day (10-15 km/h)
2. Perform normal takeoff and landing
3. Observe altitude hold stability

**Expected**:
- Altitude hold still works (±20 cm acceptable)
- Landing slightly less smooth but still safe
- No sudden drops

**Note**: Do NOT fly in wind >20 km/h!

---

## Troubleshooting Failed Tests

### Drone won't arm

**Checks**:
- [ ] Battery voltage >11.1V
- [ ] Radio connected (Serial shows `✅ Radio connected`)
- [ ] Sensors initialized (no error messages)
- [ ] Not already armed

---

### Motors don't spin

**Checks**:
- [ ] ESCs powered (battery connected)
- [ ] ESC signal wires connected to D3, D5, D6, D9
- [ ] ESCs calibrated (some ESCs need calibration)
- [ ] Motor wires connected to ESCs

**ESC Calibration** (if needed):
1. Disconnect battery
2. Throttle stick to MAX
3. Connect battery (ESCs beep)
4. Throttle stick to MIN (ESCs beep)
5. Done

---

### One motor spins wrong direction

**Fix**:
1. Disconnect battery
2. Swap any 2 wires between motor and ESC
3. Reconnect battery
4. Test again (Button 2)

---

### Altitude shows wrong value

**Checks**:
- [ ] MS5611 connected to I2C (A4/A5)
- [ ] MS5611 has correct power (3.3V or 5V)
- [ ] Calibration done (Button 1)

**Recalibrate**:
1. Place drone on ground
2. Press Button 1
3. Wait 3 seconds
4. Altitude should be ~0 cm

---

### Drone tilts to one side

**Checks**:
- [ ] Battery centered (CoG in middle)
- [ ] Frame is level (use bubble level)
- [ ] Calibration done on level surface
- [ ] Motor thrust equal (check props are same pitch)

---

### Landing drops suddenly

**Cause**: Motors stopping mid-descent (CRITICAL BUG)

**Fix**:
Check code has minimum throttle:
```cpp
if (landingState == LANDING_DESCENDING || landingState == LANDING_NEAR_GROUND) {
  applyMinimumThrottle(LANDING_IDLE_THROTTLE);  // 1100
}
```

If missing, motors can go below idle and stop!

---

## Post-Flight Checks

After every flight:

- [ ] **Battery voltage** checked (stop if <11.0V)
- [ ] **Motors temperature** checked (should be warm, not hot)
- [ ] **Props inspected** (no cracks, chips, or bends)
- [ ] **Screws checked** (vibrations can loosen over time)
- [ ] **Serial log reviewed** (any error messages?)
- [ ] **Flight notes** recorded (tuning changes, issues observed)

---

## Flight Log Template

Keep a log of every flight for troubleshooting:

```
Date: ___________
Flight #: _____
Battery: ______ V (start) → ______ V (end)
Flight time: _____ minutes

Tests performed:
[ ] Hover test
[ ] Altitude hold
[ ] Takeoff
[ ] Landing

Observations:
- Stability: ___________________
- Landing quality: _____________
- Issues: _____________________

PID changes:
- Rate Roll Kp: _____
- Rate Roll Ki: _____
- Rate Roll Kd: _____
- Altitude Kp: ______
- Altitude Kd: ______

Notes:
_________________________________
_________________________________
```

---

## Summary

### Mandatory Tests Before First Flight

1. ✅ **Motor direction test** (props off)
2. ✅ **Radio range test**
3. ✅ **Control input test**
4. ✅ **Manual hover test** (props on, 10 seconds)

### Mandatory Tests for Landing System

5. ✅ **Altitude hold test** (30 seconds stable)
6. ✅ **Smooth takeoff test** (automated rise to 150 cm)
7. ✅ **Smooth landing test** (automated descent with no drops)

### Success Criteria

For landing system to be validated:
- [ ] **5 successful landings** in a row
- [ ] **No motor cutoffs** during descent
- [ ] **Touchdown < 15 cm altitude**
- [ ] **Vertical velocity < 10 cm/s** at touchdown
- [ ] **No bounces** or hard impacts
- [ ] **Consistent behavior** each time

---

## Safety Summary

### Before EVERY flight:

⚠️ **Battery charged**  
⚠️ **Props secure and correct direction**  
⚠️ **Open area with clearance**  
⚠️ **Serial Monitor checked (no errors)**  
⚠️ **Radio connection confirmed**  
⚠️ **Emergency procedure known**  

### During flight:

⚠️ **Keep line of sight**  
⚠️ **Monitor battery voltage**  
⚠️ **Ready to disarm** (throttle to zero)  
⚠️ **Fly conservatively**  

### After flight:

⚠️ **Disarm before approaching**  
⚠️ **Disconnect battery**  
⚠️ **Check for damage**  
⚠️ **Record flight notes**  

---

**Fly safe! 🚁**
