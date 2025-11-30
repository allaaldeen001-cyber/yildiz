# Button Functions Guide

## 🎮 Complete Control Layout

### Joysticks (2x - 4 axes total)

#### Left Stick (Throttle + Yaw)
```
        UP
     Throttle
    (Altitude)
        |
   L ---+--- R
    Yaw   Yaw
   (Rotate)
        |
       DOWN
    Throttle
```

- **Vertical Axis (A0)** → Throttle
  - Up = Increase altitude
  - Down = Decrease altitude
  - Center = Hover (maintain altitude in flight)

- **Horizontal Axis (A1)** → Yaw
  - Left = Rotate counter-clockwise
  - Right = Rotate clockwise
  - Center = No rotation

#### Right Stick (Pitch + Roll)
```
        UP
       Pitch
     (Forward)
        |
   L ---+--- R
   Roll   Roll
  (Left) (Right)
        |
       DOWN
       Pitch
    (Backward)
```

- **Vertical Axis (A2)** → Pitch
  - Up = Tilt forward (move forward)
  - Down = Tilt backward (move backward)
  - Center = No tilt

- **Horizontal Axis (A3)** → Roll
  - Left = Tilt left (strafe left)
  - Right = Tilt right (strafe right)
  - Center = Level

---

## 🔘 Toggle Switches (2x)

### SW1 (D2): ARM / DISARM
**Primary safety control**

| Position | Function | Description |
|----------|----------|-------------|
| **OFF** | DISARM | Motors locked, cannot spin |
| **ON** | ARM | Motors enabled (if throttle low) |

**Safety Rules:**
- ✅ Can only ARM when throttle is at minimum (<5%)
- ✅ Automatically DISARMS on signal loss (failsafe)
- ✅ Cannot ARM during motor test mode
- ✅ Arming turns off buzzer automatically

**Usage:**
1. Throttle to minimum
2. Flip SW1 to ON
3. Listen for 1 beep = Armed
4. LED stays solid = Ready to fly

---

### SW2 (D3): Flight Mode
**Select control behavior**

| Position | Mode | Description |
|----------|------|-------------|
| **OFF** | **ACRO** | Manual rate control (advanced) |
| **ON** | **ANGLE** | Auto-level (beginner) |

#### ANGLE Mode (Auto-Level) ✅ Recommended
**SW2 = ON**

```
Stick Position = Desired Tilt Angle
- Self-levels when sticks centered
- Maximum tilt: ±50 degrees
- Easier to control
- Perfect for beginners
```

**How it works:**
- Right stick right → Drone tilts 20° right
- Release stick → Drone returns to level
- MPU6050 + PID keeps drone stable
- Complementary filter prevents drift

**Best for:**
- Learning to fly
- Stable video recording
- Precise positioning
- Calm flight

#### ACRO Mode (Rate Control) ⚠️ Advanced
**SW2 = OFF**

```
Stick Position = Rotation Rate
- No self-leveling
- Maximum rate: ±100 deg/s
- Full manual control
- For experienced pilots
```

**How it works:**
- Right stick right → Drone rolls right continuously
- Release stick → Rotation stops, but stays tilted
- You must manually level the drone
- High agility, requires skill

**Best for:**
- Experienced pilots
- Aerobatics and tricks
- Fast flying
- Racing

---

## 🔴 Buttons (4x)

### Button 1 (D4): Calibration
**Recalibrate sensors on demand**

**Function:**
- Press to recalibrate gyro and accelerometer
- Useful if drone drifts or feels unstable
- Runs same calibration as power-on

**How to use:**
1. **DISARM** the drone (SW1 = OFF)
2. Place drone on **level surface**
3. Press **Button 1**
4. Hear 1 beep
5. **Don't move drone** for 6 seconds
6. LED blinks during calibration
7. Hear 2 beeps when complete

**When to use:**
- Drone drifts to one side
- After temperature changes
- After flying in different location
- If angles seem off in Serial Monitor

**Safety:**
- ⚠️ Only works when DISARMED
- ⚠️ Drone must be completely still
- ⚠️ Must be on level surface

---

### Button 2 (D5): Motor Test
**Safe motor spin test without flying**

**Function:**
- Hold button to spin all motors
- Speed gradually increases while held
- Release to stop
- Tests motors, ESCs, and propeller directions

**How to use:**
1. **REMOVE PROPELLERS!** ⚠️
2. **DISARM** the drone
3. **Hold Button 2**
4. Hear 1 beep = Test started
5. Motors spin at 1000µs initially
6. Speed increases to 1400µs while held
7. **Release Button 2**
8. Hear 1 beep = Test stopped
9. Motors stop

**Serial Monitor shows:**
```
Button 2: Motor test START
MOTOR_TEST | Mot FL:1100 FR:1100 RR:1100 RL:1100
MOTOR_TEST | Mot FL:1200 FR:1200 RR:1200 RL:1200
Button 2: Motor test STOP
```

**What to check:**
- ✅ All 4 motors spin
- ✅ Motors spin smoothly (no grinding)
- ✅ All motors spin same direction as config
  - FL: Counter-clockwise
  - FR: Clockwise
  - RR: Counter-clockwise
  - RL: Clockwise
- ✅ No excessive vibration
- ✅ ESCs respond properly

**Safety:**
- ⚠️ **ALWAYS REMOVE PROPELLERS**
- ⚠️ Only works when DISARMED
- ⚠️ Cannot ARM during motor test
- ⚠️ Keep fingers away from motors

---

### Button 3 (D6): Buzzer Beep
**Find your drone when lost**

**Function:**
- Toggle buzzer on/off
- Rapid beeping pattern (100ms on, 100ms off)
- Helps locate drone in grass, bushes, trees
- Automatically turns off when arming

**How to use:**
1. Press **Button 3** once
2. Buzzer starts beeping rapidly
3. Serial Monitor shows: `Button 3: Buzzer ON (find mode)`
4. Walk toward beeping sound
5. Press **Button 3** again to stop
6. Or flip SW1 to ARM (auto-stops)

**Beep pattern:**
```
BEEP-pause-BEEP-pause-BEEP-pause...
(100ms on, 100ms off, repeating)
```

**Serial Monitor shows:**
```
Button 3: Buzzer ON (find mode)
DISARM | Mode:--- | ... | BUZZ:ON
Button 3: Buzzer OFF
```

**When to use:**
- Drone landed in tall grass
- Can't see drone after crash
- Drone stuck in bush or tree
- Lost orientation of drone position

**Note:**
- Works when ARMED or DISARMED
- Automatically stops when arming
- Can toggle on/off anytime

---

### Button 4 (D7): Reserved
**Available for future features**

**Potential uses:**
- GPS return-to-home (if GPS added later)
- Rate profile switching
- Headless mode toggle
- Flip mode enable
- Camera trigger
- Custom function

**Current status:**
- Button is wired and functional
- Can be read in code
- Not assigned to any function yet
- Add your own feature!

---

## 🎯 Quick Reference Table

| Control | Function | When Active | Notes |
|---------|----------|-------------|-------|
| **Left Stick V** | Throttle | Always | Altitude control |
| **Left Stick H** | Yaw | Always | Rotation left/right |
| **Right Stick V** | Pitch | Always | Forward/backward |
| **Right Stick H** | Roll | Always | Left/right tilt |
| **SW1** | ARM/DISARM | Always | Main safety switch |
| **SW2** | ANGLE/ACRO | When armed | Flight mode select |
| **BTN1** | Calibrate | When disarmed | Recalibrate sensors |
| **BTN2** | Motor Test | When disarmed | Safe motor spin test |
| **BTN3** | Buzzer | Always | Find drone beeper |
| **BTN4** | Reserved | - | Future use |

---

## 📋 Common Use Cases

### Pre-Flight Checks
1. Power on remote controller
2. Power on flight controller
3. Wait for calibration (2 beeps)
4. Check connection (✓CONN in remote)
5. **Press BTN2** (motor test) - verify all motors
6. **Release BTN2**
7. Flip **SW1** to ARM
8. Throttle up slightly - ready to fly!

### Drift Calibration
1. Land and **DISARM** (SW1 OFF)
2. Place on level surface
3. **Press BTN1** (calibration)
4. Wait 6 seconds (don't move!)
5. Hear 2 beeps = done
6. **ARM** and test again

### Lost Drone in Grass
1. **Press BTN3** (buzzer on)
2. Walk toward beeping
3. Find drone
4. **Press BTN3** (buzzer off)
5. Check for damage
6. Fly again!

### Motor Direction Check
1. **REMOVE PROPELLERS** ⚠️
2. **DISARM** drone
3. **Hold BTN2** (motor test)
4. Observe motor rotation:
   - FL & RR should spin CCW
   - FR & RL should spin CW
5. **Release BTN2**
6. Fix any wrong directions (swap motor wires)

### Switching Flight Modes
**In air:**
1. Currently in ANGLE mode (stable)
2. Flip **SW2** to OFF → Switches to ACRO
3. Drone no longer self-levels!
4. Flip **SW2** to ON → Back to ANGLE
5. Drone levels itself

---

## 🎓 Flight Mode Comparison

| Feature | ANGLE Mode | ACRO Mode |
|---------|------------|-----------|
| **Self-leveling** | ✅ Yes | ❌ No |
| **Stick = Angle** | ✅ Yes | ❌ No |
| **Stick = Rate** | ❌ No | ✅ Yes |
| **Max tilt** | ±50° | Unlimited |
| **Returns to level** | ✅ Yes | ❌ No |
| **Drift prevention** | ✅ Yes | ⚠️ Manual |
| **Difficulty** | ⭐ Easy | ⭐⭐⭐ Hard |
| **Best for** | Learning | Tricks |
| **Recommended** | Beginners | Experts |

---

## 💡 Pro Tips

### Button Combinations
- **BTN1 + BTN3**: You could add "panic beep" mode
- **BTN2 long press**: Could add ESC calibration mode
- **BTN4**: Add your favorite feature!

### Flight Mode Tips
**ANGLE Mode:**
- Perfect for first 10-20 flights
- Learn throttle control here
- Practice hovering in place
- Build confidence with auto-level

**ACRO Mode:**
- Only try after mastering ANGLE
- Practice at high altitude first
- Start with gentle inputs
- Remember: no auto-level!

### Button Best Practices
- **BTN1**: Calibrate on flat ground only
- **BTN2**: Always remove props first!
- **BTN3**: Leave on if drone out of sight
- Don't press multiple buttons at once

---

## 🔧 Serial Monitor Output

### Remote Controller
```
=== CONTROLS ===
Left Stick:
  Vertical   -> Throttle (altitude)
  Horizontal -> Yaw (rotate left/right)
...

T:1000 R:0 P:0 Y:0 | SW1:SAFE SW2:ACRO | BTN:- | ✓CONN
T:1000 R:0 P:0 Y:0 | SW1:ARM SW2:ANGLE | BTN:- | ✓CONN
T:1000 R:0 P:0 Y:0 | SW1:SAFE SW2:ACRO | BTN:CAL | ✓CONN
T:1000 R:0 P:0 Y:0 | SW1:SAFE SW2:ACRO | BTN:TEST | ✓CONN
T:1000 R:0 P:0 Y:0 | SW1:SAFE SW2:ACRO | BTN:BUZZ | ✓CONN
```

### Flight Controller
```
DISARM | Mode:--- | Ang R:0.0 P:0.0 | Mot FL:1000 FR:1000 RR:1000 RL:1000

Button 1: Starting calibration...
[OK] Calibration complete!

ARMED | Mode:ANGLE | Ang R:5.2 P:-2.1 | PID R:73 P:-29 | Mot FL:1173 FR:1027 RR:1027 RL:1173

MOTOR_TEST | Mot FL:1100 FR:1100 RR:1100 RL:1100
MOTOR_TEST | Mot FL:1200 FR:1200 RR:1200 RL:1200

DISARM | Mode:--- | ... | BUZZ:ON
```

---

## ⚠️ Safety Reminders

### Always:
- ✅ Remove propellers for motor tests
- ✅ Keep throttle low when arming
- ✅ Calibrate on level surface
- ✅ Test buttons before flight
- ✅ Know your flight mode (check SW2)

### Never:
- ❌ Motor test with propellers on
- ❌ Calibrate while moving
- ❌ ARM during motor test
- ❌ Fly without knowing which mode you're in
- ❌ Switch to ACRO mode on first flight

---

## 🎯 Troubleshooting Buttons

### Button 1 (Calibration) not working
**Problem:** Press button but nothing happens

**Solutions:**
- Ensure drone is DISARMED (SW1 = OFF)
- Check button wiring (D4 to GND)
- Check Serial Monitor for "Button 1: Starting calibration..."
- Verify button connected with internal pullup

### Button 2 (Motor Test) not starting
**Problem:** Hold button but motors don't spin

**Solutions:**
- Ensure drone is DISARMED
- Cannot run during armed state
- Check button wiring (D5 to GND)
- ESCs must be powered and calibrated
- Check Serial Monitor for "Motor test START"

### Button 3 (Buzzer) not beeping
**Problem:** Press button but no sound

**Solutions:**
- Check buzzer wiring (D8)
- Verify active buzzer (not passive)
- Check power to buzzer
- Look for "BUZZ:ON" in Serial Monitor
- Test buzzer with startup beeps

### Button 4 not responding
**Problem:** Button 4 doesn't do anything

**Solutions:**
- This is normal! Button 4 is reserved
- Not currently assigned to any function
- Can add your own feature in code
- Button is wired and functional

---

## 📚 Related Documentation

- **Control theory:** See `README.md` for PID explanation
- **Wiring:** See `WIRING_DIAGRAMS.md` for button connections
- **Flying:** See `OPERATION_GUIDE.md` for flight procedures
- **Testing:** See `HOW_TO_TEST_STABILIZATION.md` for verification

---

**Master these controls and you'll have complete command of your quadcopter! 🚁✨**
