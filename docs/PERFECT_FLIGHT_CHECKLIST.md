# ✈️ Perfect Flight Checklist

**Pre-flight, flight, and post-flight procedures for perfect drone operation**

---

## 📋 Pre-Flight Checklist

### Before Power-On

**Hardware Inspection** (5 minutes):
- [ ] Frame: No cracks or damage
- [ ] Props: Tight, no cracks, correct orientation
  - FL: CCW (unmarked or "A")
  - FR: CW (marked "R" or "B")
  - RR: CCW (unmarked or "A")
  - RL: CW (marked "R" or "B")
- [ ] Motors: Spin freely by hand, no grinding
- [ ] ESCs: Securely mounted, no loose wires
- [ ] Battery: Voltage >11.0V per cell (11.1V = 3.7V × 3)
- [ ] Wiring: All connections tight, no exposed wire
- [ ] nRF24L01+: 10µF capacitor visible on both modules
- [ ] Sensors: MPU6050 and MS5611 securely mounted

**Environment Check**:
- [ ] Open area: 10+ meters clear radius
- [ ] No people or animals nearby
- [ ] No obstacles overhead (trees, wires)
- [ ] Weather: Calm, wind <10 km/h
- [ ] Legal to fly: Not near airport, not in restricted zone

**Equipment Ready**:
- [ ] Spare propellers (you'll break some!)
- [ ] Fully charged battery
- [ ] USB cable (for quick reconfig if needed)
- [ ] Laptop with Arduino IDE (optional)

---

### After Power-On

**Remote Controller** (RC):
1. Power on RC first (always RC before FC!)
2. Center both joysticks
3. Open Serial Monitor (115200 baud)
4. Wait for "REMOTE CONTROLLER READY!"
5. Check Serial shows:
   ```
   TX: ✅ OK
   Throttle:500 | Yaw:0 | Pitch:0 | Roll:0
   ```
6. Move sticks, values should change smoothly
7. Press buttons, should show button names
8. Toggle switches, should show mode changes

**Flight Controller** (FC):
1. Place FC on flat, level surface
2. **Keep completely still!** (gyro calibration)
3. Power on FC
4. Wait for startup sequence:
   ```
   ✅ Motors initialized
   ✅ Radio initialized (2.4GHz, 250kbps, ACK ON)
   ✅ MPU6050 initialized (DLPF=21Hz)
   ✅ MS5611 barometer initialized
   ⏳ Calibrating gyro... DONE
   ⏳ Calibrating altitude... DONE
   ✅ SYSTEM READY!
   ```
5. Listen for 2 beeps (ready signal)
6. LED should be solid ON
7. Check Serial shows:
   ```
   Mode:ANGLE | Armed:NO | RC:OK
   Roll:0.0 | Pitch:0.0 | Alt:0cm
   ```

**Connection Test**:
- [ ] RC Serial shows "TX: ✅ OK" continuously
- [ ] FC Serial shows "RC:OK" continuously
- [ ] Move RC sticks → FC Serial values change
- [ ] No warning beeps from FC

---

### Calibration Check

**If needed, recalibrate**:

1. **Press Button 1** (Calibrate)
2. FC must be on flat surface
3. **Do not touch or move!**
4. Wait for calibration complete
5. Check Serial:
   ```
   ⏳ Calibrating gyro... DONE
   ⏳ Calibrating altitude... DONE
   ```
6. Verify angles near zero:
   ```
   Roll:0.2 | Pitch:-0.1
   ```
7. Tilt FC, angles should change

**Good calibration**:
- Roll/Pitch: ±0.5° when level
- Changes smoothly when tilted
- No drift over time

**Bad calibration**:
- Roll/Pitch: >2° when level
- Jumpy values
- Drifts slowly
- **Solution**: Recalibrate, check surface is level

---

### Motor Test (Props OFF!)

**⚠️ PROPELLERS MUST BE OFF!**

1. Press **Button 2** (Motor Test)
2. All 4 motors should spin briefly (~1 second)
3. Check spin direction:
   - FL (D3): Counter-clockwise
   - FR (D5): Clockwise
   - RR (D6): Counter-clockwise
   - RL (D9): Clockwise
4. Motors should sound smooth (no grinding)
5. No ESC beeping errors

**If wrong direction**:
- Swap any 2 of the 3 motor wires to ESC
- Test again with Button 2

---

### Final Pre-Flight

**Safety Setup**:
- [ ] Propellers ON and tight
- [ ] Battery secured (no movement)
- [ ] All people 5+ meters away
- [ ] Clear flight path (no obstacles)
- [ ] Know emergency landing button (Button 3)

**Switch Configuration**:
- [ ] SW1 = OFF (manual throttle)
- [ ] SW2 = ON (ANGLE mode - auto-level)
- [ ] Throttle stick = DOWN (minimum)

**Ready to Fly**:
- [ ] FC Serial: "RC:OK", "Armed:NO"
- [ ] RC Serial: "TX: ✅ OK"
- [ ] No beeping from FC
- [ ] LED solid ON
- [ ] Area clear

---

## 🚁 Flight Procedures

### Takeoff (Automatic)

**Press Button 4** (Auto Takeoff):

1. Button pressed → FC arms automatically
2. Beep (armed)
3. Motors spin up smoothly
4. Drone rises at 80 cm/s
5. Reaches 150cm (1.5m)
6. Transitions to Altitude Hold
7. Hovers hands-free!

**Serial Monitor shows**:
```
Mode:TAKEOFF | Armed:YES | RC:OK
Alt:40cm → 80cm → 120cm → 150cm
Mode:ALT_HOLD | Armed:YES
```

**What to watch**:
- Smooth ascent (not jerky)
- Stays centered (not drifting)
- Reaches 1.5m and stops
- Hovers stable

**If something goes wrong**:
- Press **Button 3** immediately (emergency landing)
- OR: Throttle to minimum, wait for failsafe

---

### Manual Flight (ANGLE Mode)

**Right Stick** (Pitch & Roll):
- Forward → Drone tilts forward, moves forward
- Back → Drone tilts back, moves back
- Right → Drone tilts right, moves right
- Left → Drone tilts left, moves left
- **Center stick → Drone levels and stops**

**Left Stick** (Throttle & Yaw):
- Up → Climbs
- Down → Descends
- Right → Rotates clockwise
- Left → Rotates counter-clockwise

**Practice Maneuvers**:

1. **Hover in place** (5 minutes)
   - Center all sticks
   - Small corrections only
   - Goal: Stay in 1m radius

2. **Forward/back** (5 minutes)
   - Pitch stick forward → move 5m
   - Center stick → stop
   - Pitch stick back → return
   - Repeat

3. **Left/right** (5 minutes)
   - Roll stick right → move 5m
   - Center stick → stop
   - Roll stick left → return
   - Repeat

4. **Yaw rotations** (5 minutes)
   - Yaw stick right → rotate 90°
   - Center stick → stop rotation
   - Yaw stick left → rotate back
   - Repeat

5. **Figure-8 pattern** (10 minutes)
   - Combine pitch + roll + yaw
   - Smooth transitions
   - Maintain altitude

---

### Altitude Hold Mode

**Enable SW1** (Altitude Hold ON):

1. Drone locks current altitude
2. Mode changes to ALT_HOLD
3. Throttle stick now adjusts target altitude:
   - Stick up → Climbs slowly (10cm/s)
   - Stick center → Holds altitude perfectly
   - Stick down → Descends slowly

**Benefits**:
- Hands-free hovering
- Wind compensation automatic
- Easier to fly
- Focus on position control only

**Best for**:
- Aerial photography
- Learning advanced maneuvers
- Long flights (less fatigue)

---

### Landing (Automatic)

**Press Button 3** (Auto Landing):

1. Drone starts descent
2. Descends at 50 cm/s (gentle)
3. Altitude decreases: 150 → 100 → 50 → 10cm
4. At 10cm → motors disarm automatically
5. 3 beeps (safe landing)

**Serial Monitor shows**:
```
Mode:LANDING | Armed:YES
Alt:150cm → 100cm → 50cm → 10cm
Mode:ANGLE | Armed:NO
✅ Landing complete, DISARMED
```

**Landing Tips**:
- Press Button 3 when above landing spot
- Drone descends straight down
- Lands gently (no bounce)
- Automatically disarms

---

## 🔧 In-Flight Troubleshooting

### Drone Drifts in One Direction

**Cause**: Wind or gyro drift

**Solution**:
1. Enable Altitude Hold (SW1 = ON)
2. Or: Recalibrate after landing (Button 1)

---

### Drone Oscillates/Vibrates

**Cause**: PID too aggressive

**Quick Fix**:
1. Land immediately (Button 3)
2. Reduce P gain in code:
   ```cpp
   pidRateRoll.Kp = 0.5; // Was 0.65
   ```
3. Upload and test again

---

### Loss of Control

**Cause**: Radio signal lost

**What happens**:
1. FC detects no signal for 1 second
2. Automatic failsafe triggered
3. Motors disarm
4. Drone falls (controlled descent)
5. FC beeps rapidly

**Prevention**:
- Keep drone in sight
- Don't fly too far
- Check battery before flight
- Ensure nRF24 modules have 10µF capacitors

---

### Motors Stop Mid-Flight

**Causes**:
1. **Failsafe triggered** → Radio lost
2. **Battery dead** → Voltage too low
3. **Throttle too low** → Increase throttle

**Emergency**:
- Press Button 4 immediately (auto takeoff)
- If that fails, prepare for crash landing
- Mark location to retrieve

---

## 📊 Post-Flight Checklist

### Immediate After Landing

1. **Disarm**: Should be automatic (Button 3)
2. **Disconnect battery**: Wait 30 seconds
3. **Check temperature**: Motors should be warm, not hot
4. **Inspect props**: Look for cracks or chips
5. **Check frame**: No new damage

### Review Flight Data

**Serial Monitor** (last 20 lines):
- [ ] No "FAILSAFE" messages
- [ ] "RC:OK" throughout flight
- [ ] Roll/Pitch angles reasonable (±30°)
- [ ] Altitude stable in ALT_HOLD mode
- [ ] Motor speeds balanced (within 50µs)

**Example good data**:
```
Mode:ANGLE | Armed:YES | RC:OK
Roll:2.3 | Pitch:-1.5 | Alt:150cm
Motors:1245,1255,1240,1250
```

**Example bad data** (needs fixing):
```
Mode:ANGLE | Armed:YES | RC:LOST  ← Radio issue!
Roll:15.2 | Pitch:8.3 | Alt:150cm  ← Oscillating!
Motors:1400,1100,1400,1100         ← Unbalanced!
```

---

### Maintenance

**After Every Flight**:
- [ ] Check prop tightness
- [ ] Wipe dust off sensors
- [ ] Check for loose wires

**After 10 Flights**:
- [ ] Tighten all screws
- [ ] Check motor bearings (spin by hand)
- [ ] Inspect solder joints
- [ ] Clean ESCs (remove dust)

**After 50 Flights**:
- [ ] Replace props (even if look OK)
- [ ] Check motor shaft for wear
- [ ] Inspect frame for cracks
- [ ] Re-solder weak connections

---

## 🎯 Flight Time Tracking

**Battery**: 2200mAh 3S LiPo

**Expected flight time**:
- **ANGLE mode (hovering)**: 8-10 minutes
- **ANGLE mode (moving)**: 6-8 minutes
- **ACRO mode (aggressive)**: 4-6 minutes
- **ALT HOLD mode**: 7-9 minutes

**Battery voltage**:
- **Fully charged**: 12.6V (4.2V per cell)
- **Nominal**: 11.1V (3.7V per cell)
- **Land now**: 10.5V (3.5V per cell)
- **NEVER below**: 9.0V (3.0V per cell)

**Voltage check**:
```cpp
// Add to Flight Controller code:
float batteryVoltage = analogRead(A6) * (11.1 / 1023.0);
Serial.print("Battery: ");
Serial.print(batteryVoltage, 1);
Serial.println("V");

if (batteryVoltage < 10.5) {
  Serial.println("⚠️  LOW BATTERY - LAND NOW!");
  beep(3);
}
```

---

## 🎓 Progression Path

### Week 1: Basic Flying
- [ ] Auto takeoff/landing (Button 4 & 3)
- [ ] Hover in place (1m radius)
- [ ] Forward/back 5m
- [ ] Left/right 5m
- [ ] Yaw rotations 360°

### Week 2: Advanced Control
- [ ] Figure-8 pattern
- [ ] Altitude Hold mode
- [ ] Manual takeoff (no Button 4)
- [ ] Manual landing (no Button 3)
- [ ] Fly in light wind (5 km/h)

### Week 3: Mastery
- [ ] Complex patterns (orbit, spiral)
- [ ] Low altitude flight (1m)
- [ ] Fast forward flight
- [ ] Emergency procedures
- [ ] Try ACRO mode (carefully!)

---

## ✅ Perfect Flight Definition

A perfect flight has:

**Takeoff**:
✅ Smooth ascent to 1.5m
✅ No drift or oscillation
✅ Transitions to stable hover

**Flight**:
✅ Responsive to controls
✅ Returns to level when sticks centered
✅ Altitude Hold maintains ±10cm
✅ No unexpected movements
✅ Radio connection 100%

**Landing**:
✅ Gentle descent
✅ Lands on target spot
✅ Auto-disarms safely
✅ No hard impact

**Post-Flight**:
✅ No errors in Serial Monitor
✅ Battery voltage >10.5V
✅ Props intact
✅ No loose wires

---

## 🏆 Flight Skills Checklist

Mark your progress:

**Beginner** (Week 1):
- [ ] First successful takeoff
- [ ] Hover 30 seconds
- [ ] Safe landing
- [ ] 5 flights without crash

**Intermediate** (Week 2):
- [ ] Hover 5 minutes
- [ ] Figure-8 pattern
- [ ] Altitude Hold mastered
- [ ] 20 flights without crash

**Advanced** (Week 3+):
- [ ] Manual takeoff/landing
- [ ] Complex patterns
- [ ] ACRO mode basics
- [ ] 50+ flights

**Expert** (Month 2+):
- [ ] ACRO flips and rolls
- [ ] FPV flying (if added camera)
- [ ] Custom PID tuning
- [ ] 100+ flights

---

**Safe Flying! ✈️**

*Always prioritize safety, follow procedures, and have fun!*
