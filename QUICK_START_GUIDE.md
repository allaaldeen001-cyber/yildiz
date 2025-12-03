# Quick Start Guide - Quadcopter Flight Controller

## 5-Minute Setup

### Step 1: Hardware Assembly (30 min)

1. **Mount Flight Controller** on drone frame (use foam for vibration dampening)
2. **Connect ESCs** to pins D3, D5, D6, D9 (see wiring table)
3. **Connect MPU6050** to I2C (A4=SDA, A5=SCL)
4. **Connect MS5611** to I2C (same bus, 3.3V power)
5. **Connect NRF24L01** to SPI with 3.3V regulator + capacitors
6. **Connect buttons** to D4, A0, A1, A2
7. **Connect potentiometers** to A6, A7
8. **Connect buzzer** to D7

### Step 2: Software Upload (5 min)

1. **Install libraries** via Arduino Library Manager:
   - RF24
   - MS5611 (by Jarzebski)

2. **Upload Flight Controller**:
   - Open `FlightController_Complete.ino`
   - Board: Arduino Nano
   - Processor: ATmega328P (Old Bootloader) if using clone
   - Upload

3. **Upload Remote Controller**:
   - Open `RemoteController_Complete.ino`
   - Same board settings
   - Upload to second Arduino

### Step 3: First-Time Calibration (2 min)

1. **Place drone on level surface**
2. **Open Serial Monitor** (115200 baud)
3. **Press Button 1 (D4)** on Flight Controller
4. **Wait for beeps**: System calibrates for 10 seconds
5. **Done**: "Calibration saved to EEPROM" appears

### Step 4: Motor Direction Test (2 min)

**REMOVE ALL PROPELLERS**

1. **Press Button 2 (A0)** on Flight Controller
2. **Watch motors spin** one at a time:
   - Front Left → Front Right → Rear Left → Rear Right
3. **Verify directions**:
   - FL: Clockwise
   - FR: Counter-Clockwise
   - RL: Counter-Clockwise
   - RR: Clockwise
4. **If wrong**: Swap any two motor wires

### Step 5: First Flight (10 min)

1. **Install propellers** (correct orientation)
2. **Power on Remote Controller** first
3. **Power on Flight Controller**
4. **Verify link**: Remote Serial shows "CONNECTED"
5. **Place on level ground** for 30 seconds
6. **ARM**: Switch 1 to ON position
7. **Slowly increase throttle**
8. **Lift off** to 1 meter height
9. **Release sticks**: Drone should stay roughly level
10. **Land**: Reduce throttle slowly

---

## Control Reference Card

### Sticks (Remote Controller)

```
     LEFT STICK              RIGHT STICK
   
       [T]                      [P]
        ↑                        ↑
    [Y] ← → [Y]              [R] ← → [R]
        ↓                        ↓
       [T]                      [P]

T = Throttle (up/down)
Y = Yaw (rotate)
P = Pitch (forward/back)
R = Roll (left/right)
```

### Switches

- **Switch 1 (D2)**: ARM/DISARM (master kill switch)
- **Switch 2 (D3)**: STABILIZE / ALTITUDE HOLD

### Flight Controller Buttons

- **Button 1 (D4)**: Calibrate all sensors (ground only)
- **Button 2 (A0)**: Motor direction test (no props)
- **Button 3 (A1)**: Auto takeoff to +1 meter
- **Button 4 (A2)**: Auto landing with touchdown detection

### Potentiometers

- **Pot 1 (A6)**: P-Gain (0.5 to 5.0) - Response speed
- **Pot 2 (A7)**: D-Gain (5.0 to 30.0) - Stability

---

## Flight Modes Quick Reference

### DISARMED
- Motors: OFF
- LED: Fast blink
- Switch 1: OFF position

### ARMED IDLE
- Motors: Low idle (1050 µs)
- LED: Slow blink
- Throttle: Below 1100

### STABILIZE
- Manual throttle control
- Auto-levels when sticks centered
- Switch 2: OFF (Stabilize)

### ALTITUDE HOLD
- Maintains current altitude
- Throttle stick: Climb/descend rate
- Switch 2: ON (Alt Hold)
- Requires: MS5611 calibrated

---

## Emergency Procedures

### Uncontrolled Flight
1. **Switch 1 to OFF** (disarm)
2. Clear the area
3. Wait for motors to stop

### Radio Link Lost
- Auto-landing activates (if barometer valid)
- OR motors cut immediately
- Buzzer sounds continuously

### Excessive Tilt
- Motors cut automatically
- Requires: Switch 1 OFF then ON to recover

---

## Tuning in 3 Minutes

### Default Settings (Start Here)
- Pot 1: Middle position (P ≈ 2.5)
- Pot 2: Middle position (D ≈ 17.5)

### If Drone Oscillates (Shakes Fast)
- Turn Pot 1 **counter-clockwise** (decrease P)

### If Drone Responds Slowly
- Turn Pot 1 **clockwise** (increase P)

### If Drone Bounces After Movements
- Turn Pot 2 **clockwise** (increase D)

### If Drone Feels Sluggish
- Turn Pot 2 **counter-clockwise** (decrease D)

---

## Pre-Flight Checklist

- [ ] Battery fully charged
- [ ] Propellers tight, correct orientation
- [ ] Calibration done (within last week)
- [ ] Remote shows "CONNECTED"
- [ ] Clear area (10m radius minimum)
- [ ] Switch 1 OFF (disarmed)
- [ ] Potentiometers at middle position

---

## Troubleshooting (1 Minute Fixes)

### No Radio Link
- Check NRF24L01 has 3.3V power
- Add 10µF capacitor across NRF VCC/GND
- Re-upload both sketches

### Motors Don't Spin
- Switch 1 to ON (arm)
- Throttle above 1100
- Check ESC power from battery
- Verify calibration complete

### Unstable Flight
- Decrease Pot 1 (P-gain)
- Increase Pot 2 (D-gain)
- Re-calibrate on level surface
- Check motor screws tight

### Altitude Hold Jumps
- Cover MS5611 with foam (block light)
- Re-calibrate barometer (Button 1)
- Avoid rapid stick movements

---

## Serial Monitor Commands

Open Serial Monitor at **115200 baud** to see:

```
State:2 | R:1.2 P:-0.5 Y:45.3 | Alt:0.52 Tgt:0.50 | M:1445,1448,1442,1450 | PID_P:2.50 D:17.5 | FS:0
```

**Meaning**:
- State: Flight mode (0-6)
- R/P/Y: Roll, Pitch, Yaw angles
- Alt: Current altitude (m)
- Tgt: Target altitude (m)
- M: Motor PWM values (µs)
- PID_P/D: Current P and D gains
- FS: Failsafe active (0=no, 1=yes)

---

## Safety Rules (MEMORIZE)

1. **NEVER** arm with propellers near people
2. **ALWAYS** remove propellers for testing
3. **NEVER** fly indoors until experienced
4. **ALWAYS** test in open area (10m+ clear)
5. **NEVER** reach over spinning propellers
6. **ALWAYS** disarm before handling

---

## Daily Operation

### Morning Flight:

1. Check battery voltage
2. Power on remote
3. Power on drone
4. Verify link
5. ARM
6. Takeoff
7. Tune PID if needed
8. Enjoy flight
9. Land (Button 4 for auto-landing)
10. DISARM
11. Power off

---

## Progressive Learning Path

### Week 1: Stabilize Mode
- Practice hovering
- Learn stick coordination
- Tune PID gains
- Master takeoff/landing

### Week 2: Altitude Hold
- Enable Switch 2
- Practice with auto-altitude
- Use Button 3 (auto takeoff)
- Use Button 4 (auto landing)

### Week 3: Advanced
- Longer flights
- Windy conditions
- Figure-8 patterns
- Emergency procedures

---

## Important Numbers

### Motor PWM
- OFF: 1000 µs
- Armed Idle: 1050 µs
- Hover: ~1450 µs
- Max: 2000 µs

### Stick Ranges
- Throttle: 1000-2000 µs
- Roll/Pitch/Yaw: -500 to +500

### Frequencies
- Main loop: 250 Hz (4 ms)
- Radio: 50 Hz (20 ms)
- Barometer: 50 Hz (20 ms)

### Safety Limits
- Max tilt: 45°
- Landing tilt: 15°
- Radio timeout: 1 second
- Descent rate: 0.2 m/s

---

## When Things Go Wrong

### Problem: Won't ARM
**Fix**: Press Button 1 to calibrate

### Problem: Drifts in one direction
**Fix**: Re-calibrate on level surface

### Problem: Flips on takeoff
**Fix**: Motor direction wrong, swap two wires

### Problem: Won't take off
**Fix**: Increase throttle past 1100

### Problem: Altitude hold doesn't work
**Fix**: Cover MS5611 with foam, re-calibrate

---

## Contact for Help

1. Check Serial Monitor for error messages
2. Read full README_COMPLETE_SYSTEM.md
3. Review CHANGES_AND_FIXES.md
4. Check wiring against pin tables

---

## Quick Specifications

- **Loop Rate**: 250 Hz (4 ms)
- **Attitude Accuracy**: ±1° steady-state
- **Altitude Accuracy**: ±20 cm (calm conditions)
- **Radio Range**: 100+ meters (line of sight)
- **Battery**: 3S LiPo (11.1V)
- **Motors**: RS2205 2300KV
- **Props**: 5045 (5 inch)
- **Weight**: ~300g without battery
- **Flight Time**: ~10 minutes (2200mAh battery)

---

## Success Indicators

You're flying well when:
- Drone hovers hands-off for 10+ seconds
- Gentle stick movements produce smooth response
- Landing is controlled (not a crash)
- Serial Monitor shows stable loop times
- PID gains are optimized for your build
- Radio link stays solid at 50+ meters

---

## Next Steps

After mastering basics:
1. Practice different flight patterns
2. Try manual landing vs auto-landing
3. Test in light wind (< 10 mph)
4. Experiment with PID tuning
5. Consider adding GPS for position hold

---

**Remember**: Start slow, practice often, safety first.

**Enjoy your flights!**

---

*Quick Start Guide v2.0*
*Compatible with Flight Controller v2.0*
