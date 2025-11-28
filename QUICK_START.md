# ⚡ Quick Start Guide

**Get flying in 30 minutes!**

This guide assumes you have all components wired correctly. See [WIRING_DIAGRAM.md](WIRING_DIAGRAM.md) if not.

---

## ✅ Pre-Flight Setup (5 minutes)

### 1. Install Arduino Libraries

Open Arduino IDE → Tools → Manage Libraries

Install these:
- `RF24` by TMRh20
- `MS5611` by Rob Tillaart  
- `Smoothed` by Matthew Fryer

---

### 2. Upload Code (5 minutes)

#### Flight Controller
```
1. Open: Drone_Flight_Controller/Drone_Flight_Controller.ino
2. Board: Arduino Nano
3. Processor: ATmega328P (Old Bootloader if needed)
4. Port: Select your Arduino
5. Click Upload ↑
6. Wait for "Done uploading"
```

#### RC Controller
```
1. Open: RC_Controller/RC_Controller.ino
2. Board: Arduino Nano
3. Port: Select your Arduino
4. Click Upload ↑
5. Wait for "Done uploading"
```

**Verify:** Open Serial Monitor (57600 baud) - you should see startup messages

---

## 🔗 First Connection (2 minutes)

### Power-On Sequence

**CRITICAL: Always power RC first!**

1. **Power ON RC Controller**
   - Connect USB or battery
   - LED blinks rapidly
   - Serial shows "Searching for drone..."

2. **Power ON Flight Controller**  
   - Connect main battery
   - Hear 3 ascending beeps
   - Wait 3 seconds

3. **Confirm Link**
   - Hear 2 high-pitched beeps
   - RC LED changes to slow blink
   - Serial shows "RC LINKED!"

**If no link:** See [troubleshooting](#troubleshooting-quick-fixes) below

---

## 🎯 Calibration (5 minutes)

**IMPORTANT: Do this on level surface, drone completely still!**

### Step-by-Step

1. **Verify Disarmed**
   - Switch 1 on RC = UP position (1)
   - LED on FC stays ON continuously

2. **Start Calibration**
   - Press and hold **Button 1** on RC
   - Hold for 2 seconds
   - Release when you hear beeps

3. **Wait for Completion**
   - Keep drone absolutely still!
   - You'll hear: Beep → pause → beep → pause → final beep
   - Serial shows "Calibration complete"

4. **Verify**
   - Serial shows calibration values
   - Example: `X=0.45 Y=-0.32`
   - If values >5.0, repeat calibration

**Done!** Calibration is saved permanently.

---

## 🚁 First Flight (10 minutes)

### Pre-Flight Checks

- [ ] Battery fully charged (>11.1V)
- [ ] All props tight
- [ ] Clear 5m radius area
- [ ] Calibration completed
- [ ] RC battery good

### Motor Direction Test

**IMPORTANT: Remove propellers for this test!**

1. **Arm the Drone**
   - Move Switch 1 DOWN (0 = armed)
   - Hold 2 seconds
   - Hear confirmation beep
   - LED turns OFF (now blinks on signals)

2. **Run Smooth Motor Test**
   - Press **Button 2** once
   - All motors spin slowly
   - Watch for 5 seconds

3. **Verify Directions**
   ```
        FRONT
         ↑
    
    [FL]   [FR]
      ⟲     ⟳
       \   /
        \ /
         X
        / \
       /   \
      ⟳     ⟲
    [RL]   [RR]
   
   ⟳ = Clockwise
   ⟲ = Counter-clockwise
   ```

4. **Fix Wrong Direction**
   - Swap any 2 of the 3 motor wires to ESC
   - Re-test

5. **Disarm**
   - Switch 1 UP (1 = disarmed)
   - LED turns ON

### Install Propellers

**CRITICAL: Match prop rotation to motor direction!**

```
Props are marked:
- CW (Clockwise) - Install on FR and RL motors
- CCW (Counter-clockwise) - Install on FL and RR motors

Tip: CCW props often have a dot or letter marking
```

### Takeoff!

1. **Clear Area**
   - No people within 10m
   - No obstacles overhead
   - Outdoor or large room

2. **Arm**
   - Switch 1 DOWN
   - Throttle to minimum

3. **Hover Test**
   - Slowly increase throttle
   - Lift off 10cm
   - Hold for 5 seconds
   - Slowly reduce throttle
   - Land

4. **If Stable → Continue Flying!**
   - Practice hovering at knee height
   - Small movements only
   - Don't go above head height until confident

5. **If Unstable → Disarm & Troubleshoot**
   - See [troubleshooting](#troubleshooting-quick-fixes)

---

## 🎮 Flight Controls

```
LEFT STICK              RIGHT STICK

    UP                      UP
  Throttle                Pitch
  (Climb)               (Forward)
     ↑                      ↑
 ←       →              ←       →
 Yaw     Yaw            Roll    Roll
(CCW)   (CW)           (Left)  (Right)
     ↓                      ↓
  Throttle                Pitch
 (Descend)              (Backward)
```

### Tips for Beginners

1. **Start Slow**
   - Hover at 30cm height
   - Practice for 5+ flights before going higher

2. **Small Movements**
   - Gentle stick inputs
   - Return sticks to center often

3. **Face One Direction**
   - Don't change your orientation
   - Front of drone = front to you

4. **Emergency Stop**
   - Flip Switch 1 UP immediately
   - Drone drops but safe

---

## 🔧 Troubleshooting Quick Fixes

### Motors Don't Spin

**Check:**
1. Is Switch 1 DOWN (armed)?
2. Is throttle >1000?
3. Are ESC signal wires connected? (D3, D5, D6, D9)
4. Battery voltage >11V?

**Try:**
```
1. Disarm (Switch 1 UP)
2. Disconnect battery
3. Wait 10 seconds
4. Reconnect battery
5. Re-arm
```

---

### No RC Link

**Check:**
1. Did you power RC first?
2. Is NRF24L01 getting 3.3V (not 5V)?
3. Is 10µF capacitor on NRF24L01?

**Try:**
```
1. Power off both devices
2. Power ON RC controller
3. Wait 5 seconds
4. Power ON flight controller
5. Wait for beeps
```

**Still no link?**
- Swap NRF24L01 modules (one may be bad)
- Check wiring: CE, CSN, MOSI, MISO, SCK

---

### Drone Flips on Takeoff

**Causes:**
1. ❌ Motor direction wrong
2. ❌ Props installed backward
3. ❌ ESC not calibrated

**Fix:**
```
1. Remove props
2. Run motor test (Button 2)
3. Verify rotation matches diagram
4. If wrong: swap 2 motor wires
5. Re-install props (check CW/CCW marking)
6. Try again
```

---

### Drone Drifts Heavily

**Causes:**
1. ❌ Not calibrated on level surface
2. ❌ Props damaged/unbalanced
3. ❌ Frame flexing

**Fix:**
```
1. Place drone on perfectly level surface (use spirit level)
2. Re-calibrate (Button 1, hold 2s)
3. Check all props for cracks
4. Balance props with prop balancer
5. Tighten all frame screws
```

---

### Altitude Hold Doesn't Work

**Check:**
1. Activate only at hover throttle (1400-1450)
2. MS5611 calibrated? (part of Button 1 calibration)
3. Barometer not covered or blocked?

**Try:**
```
1. Hover at 1m height
2. Throttle to center position
3. Flip Switch 2 DOWN
4. Hear confirmation beep
5. Drone should maintain altitude
```

---

### Communication Lost in Flight

**Symptoms:**
- Buzzer sounds
- Motors stop
- Drone falls

**Causes:**
- RC battery low
- Too far away (>100m)
- Interference

**Recovery:**
```
1. Move RC closer
2. Check RC battery
3. Wait - system auto-recovers when signal returns
4. May need to re-arm
```

---

## 📊 Normal Operating Values

Check Serial Monitor (57600 baud):

```
╔═══════════════════════════════════════════════════╗
║ Parameter          Normal Range        Units      ║
╠═══════════════════════════════════════════════════╣
║ Pressure           950-1050            hPa        ║
║ Roll/Pitch Angle   -30 to +30          degrees    ║
║ Yaw Angle          -180 to +180        degrees    ║
║ Motor Values       1050-1700           µs         ║
║ Thrust Input       1000-2000           µs         ║
║ Loop Time          ~7 ms               ms         ║
╚═══════════════════════════════════════════════════╝
```

**If values outside range:** Check sensors and calibration

---

## 🆘 Emergency Procedures

### In-Flight Emergency

**Problem:** Losing control

**Action:**
1. **Flip Switch 1 UP immediately** (disarm)
2. Drone will drop - be ready
3. Check for damage before next flight

---

### Flyaway

**Problem:** Drone flying away uncontrollably

**Action:**
1. **Flip Switch 1 UP** (disarm)
2. Don't chase - let it crash
3. Next time: Check compass, GPS (if added)

---

### Low Battery

**Warning:** Drone sluggish, won't climb

**Action:**
1. Land immediately
2. Don't push throttle - may damage battery
3. Recharge before next flight
4. Consider low-voltage alarm

---

## 📱 Status Indicators

### LED Patterns

| Pattern | Meaning |
|---------|---------|
| 🔴 Solid ON | Disarmed (safe) |
| 🟢 Slow blink (500ms) | Armed, RC connected |
| 🟡 Fast blink (100ms) | RC disconnected |
| 🔵 Very fast blink | Altitude hold active |

### Buzzer Sounds

| Sound | Meaning |
|-------|---------|
| 3 ascending beeps | Startup OK |
| 2 high beeps | RC linked |
| 1 beep pattern | Calibration |
| 1 long beep | Armed |
| Rapid beeping | ERROR / Kill switch |

---

## 🎯 Next Steps

### After Successful First Flight

1. **Practice Hovering**
   - 10 flights at knee height
   - Focus on stability

2. **Learn Orientation**
   - Hover with front facing you
   - Then try sideways
   - Never fly towards yourself until confident!

3. **Try Altitude Hold**
   - Hover at 1m
   - Engage Switch 2
   - Practice with it on

4. **Tune PID (Advanced)**
   - See USER_MANUAL.md
   - Adjust if oscillating or sluggish

5. **Join Community**
   - r/Multicopter
   - DIYDrones.com
   - Share your build!

---

## 📚 More Information

| Document | When to Read |
|----------|--------------|
| [WIRING_DIAGRAM.md](WIRING_DIAGRAM.md) | Building/troubleshooting connections |
| [USER_MANUAL.md](USER_MANUAL.md) | Complete operation guide |
| [PARTS_LIST.md](PARTS_LIST.md) | Shopping for components |
| [README.md](README.md) | Project overview |

---

## ⚠️ Safety Reminders

- 🚫 Never touch spinning propellers
- 🚫 Never fly near people/animals
- 🚫 Never fly above your skill level
- 🚫 Never fly in bad weather
- ✅ Always have clear exit plan
- ✅ Always wear eye protection
- ✅ Always check battery before flight
- ✅ Always disarm after landing

---

## 🎉 Success Checklist

Mark off as you complete:

- [ ] Libraries installed
- [ ] Code uploaded to both Arduinos
- [ ] RC and FC linked successfully
- [ ] Calibration completed
- [ ] Motor directions verified
- [ ] Propellers installed correctly
- [ ] First hover successful
- [ ] Smooth landing achieved

**Congratulations! You're now a drone pilot! 🚁**

---

**Questions?** Check [USER_MANUAL.md](USER_MANUAL.md) for detailed info.

**Happy Flying! ✈️**
