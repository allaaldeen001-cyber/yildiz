# Quick Start Guide - 15 Minutes to First Flight

## ⚡ Prerequisites

**Hardware Ready:**
- ✅ All components wired (see WIRING_GUIDE.md)
- ✅ Battery fully charged
- ✅ Propellers attached correctly (remove for initial tests!)

**Software Ready:**
- ✅ Arduino IDE installed
- ✅ Required libraries installed (see LIBRARIES_INSTALLATION.md)
- ✅ Code uploaded to both Arduino Nanos

**Not ready?** Read the full manuals first!

---

## 🚀 5-Minute Upload

### Step 1: Upload Flight Controller Code (2 min)

1. Open Arduino IDE
2. File → Open → `FlightController/FlightController.ino`
3. Connect Flight Controller Arduino Nano via USB
4. Tools → Board → **Arduino Nano**
5. Tools → Processor → **ATmega328P (Old Bootloader)**
6. Tools → Port → Select COM port
7. Click **Upload** (→) button
8. Wait for "Done uploading"

**Success**: Should see:
```
Sketch uses 20832 bytes (67%) of program storage space.
Done uploading.
```

**Problem?** Try different processor option or check USB cable.

---

### Step 2: Upload Remote Controller Code (2 min)

1. File → Open → `RemoteController/RemoteController.ino`
2. Connect Remote Controller Arduino Nano via USB
3. Verify board settings (same as above)
4. Click **Upload** (→) button
5. Wait for "Done uploading"

**Success**: Open Serial Monitor (Ctrl+Shift+M), set to **115200 baud**

Should see:
```
╔════════════════════════════════════════╗
║   DRONE REMOTE CONTROLLER v1.0        ║
║   Professional UAV Control System     ║
╚════════════════════════════════════════╝

Initializing NRF24L01... OK
RF Channel: 103
```

---

## 🔧 2-Minute Hardware Check

### Flight Controller

1. **Remove USB**, connect battery
2. Listen for: **beep-beep** (startup sound)
3. LED (D7) should be **OFF** (no RC connection yet)

**No beeps?**
- Check battery voltage (should be >11.1V)
- Verify ESC BEC providing 5V to Arduino
- Check buzzer wiring (D8)

---

### Remote Controller

1. **Keep USB connected** (Serial Monitor open)
2. **Or** disconnect USB and connect 9V battery
3. Serial Monitor shows: "Waiting for Flight Controller..."
4. Move joysticks - check RC display updates

---

## 🔗 1-Minute Connection Test

### Power On Both Boards

**Important Order:**
1. RC ON first
2. FC ON second

### Check Connection

**Flight Controller:**
- LED (D7) starts **blinking** (1Hz)
- Beep-beep sound

**Remote Controller Serial Monitor:**
```
>>> CONNECTED TO FLIGHT CONTROLLER <<<

  CONNECTION: ✓ LINKED

┌─── FLIGHT CONTROLLER STATUS ───────────┐
│  Armed:      NO
│  Calibrated: NO ✗
│  Battery:    11.2 V
```

**Not connected?**
- Check NRF24L01 power (3.3V)
- Add 100µF capacitor to NRF24L01
- Move devices closer (<1m for initial test)
- See TROUBLESHOOTING.md

---

## 🎯 2-Minute Calibration

### Gyro Calibration

**CRITICAL**: Place drone on **perfectly level surface**

1. Ensure **SW_2 (Arm Switch)** is **ON** (position 1)
2. Press **Button_1** (Calibration button)
3. Wait 6-8 seconds
4. Listen for result:
   - ✅ **Two short beeps** = SUCCESS
   - ❌ **One long beep (7s)** = FAILED

**If failed:**
- Verify surface is truly level
- Don't touch drone during calibration
- Wait 30 seconds and retry

**Serial Monitor shows:**
```
>>> CALIBRATION REQUESTED <<<

✓ Calibration SUCCESS!
Gyro offsets: X=0.23 Y=-0.15 Z=0.08

│  Calibrated: YES ✓
```

---

## 🧪 3-Minute Motor Test

### ESC Calibration (First Time Only)

**⚠️ REMOVE PROPELLERS FIRST!**

1. Set **SW_1 (Alt Hold)** to **OFF** (position 0)
2. Press **Button_2** (ESC Calibration button)
3. Listen: **Three beeps** = started
4. Each motor spins one by one:
   - Front Left → beep
   - Front Right → beep
   - Rear Right → beep
   - Rear Left → beep
5. Final: **beep-beep-BEEEEP** = complete

**Check:**
- All 4 motors spun?
- Direction correct? (see motor configuration below)
- Smooth acceleration?

---

## 🛫 First Flight (2 minutes)

### Pre-Flight Final Checks

- [ ] **Propellers ON** (correct direction!)
- [ ] Battery >11.1V
- [ ] SW_2 (Arm) is **OFF** (position 0)
- [ ] Throttle stick **DOWN**
- [ ] Clear 10m area
- [ ] Emergency plan ready

### Motor Configuration
```
     FRONT
      ↑
  FL     FR
   ↺     ↻     ↺ = Counter-Clockwise
    \   /      ↻ = Clockwise
     \ /
      X
     / \
    /   \
   ↻     ↺
  RL     RR
```

---

### Arming

1. **Lower throttle** to minimum (left stick down)
2. Serial Monitor shows: `Throttle: 1000-1100`
3. Flip **SW_2 (Arm)** to **ON**
4. Listen: **beep-beep** = ARMED
5. Serial Monitor: `Armed: YES ⚠`

**Can't arm?**
- Lower throttle more
- Check calibration completed
- Verify battery voltage

---

### Takeoff

1. **Slowly** push throttle up (left stick)
2. Drone should lift off smoothly at ~50% throttle
3. Hover at 0.5-1m height
4. **Release stick** - drone should stay relatively stable
5. Practice for 30 seconds

**Controls:**
- Left stick UP/DOWN = Climb/Descend
- Left stick LEFT/RIGHT = Rotate (Yaw)
- Right stick UP/DOWN = Forward/Backward (Pitch)
- Right stick LEFT/RIGHT = Slide left/right (Roll)

---

### Landing

1. Reduce throttle gradually
2. Let drone settle on ground
3. Throttle to minimum
4. Flip **SW_2 (Arm)** to **OFF**
5. Listen: **beep** = disarmed

---

## 🎮 Control Reference

### Left Joystick (Throttle + Yaw)
```
        UP
    Altitude+
         |
LEFT --- --- RIGHT
 CCW          CW
         |
       DOWN
    Altitude-
```

### Right Joystick (Pitch + Roll)
```
        UP
     Forward
         |
LEFT --- --- RIGHT
Slide-L    Slide-R
         |
       DOWN
    Backward
```

---

## 🚨 Emergency Stop

**ANYTIME**: Flip **SW_2** to **OFF**
- Motors stop immediately
- Drone will fall!
- Use only in emergency

---

## ✅ Post-Flight Checklist

- [ ] Disarm (SW_2 OFF)
- [ ] Disconnect battery
- [ ] Check frame for damage
- [ ] Inspect propellers
- [ ] Charge battery to storage voltage (3.8V/cell)

---

## 📊 Expected Performance

**First Flight:**
- Hover time: 8-12 minutes (depends on battery)
- Control range: 50-100m (NRF24L01 PA+LNA)
- Stability: Should hover with minor drift
- Response: Smooth, not too aggressive

**If unstable:**
- Re-calibrate on level surface
- Check propeller balance
- Reduce vibrations (foam mounting)
- Tune PID (see PID_TUNING_GUIDE.md)

---

## 🎯 Next Steps

### Test Flight #2-5: Learn Controls
- Practice hover
- Forward/backward flight
- Left/right flight
- Yaw rotation
- Figure-8 patterns

### Test Flight #6-10: Advanced Features
- Altitude hold (SW_1 ON)
- Higher altitude flights
- Faster maneuvers
- Emergency procedures

### Advanced:
- PID tuning (PID_TUNING_GUIDE.md)
- Add FPV camera
- Add GPS module
- Data logging

---

## 🆘 Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| No connection | Check NRF24L01 power (3.3V), add capacitor |
| Can't calibrate | Ensure perfectly level surface |
| Can't arm | Lower throttle, check calibration |
| Motors don't spin | Check ESC connections, re-do ESC cal |
| Drone flips on takeoff | Wrong propeller direction/placement |
| Oscillates/vibrates | Reduce vibrations, tune PID |
| Drifts badly | Re-calibrate gyro, check level surface |

**Full troubleshooting**: See TROUBLESHOOTING.md

---

## 📚 Documentation Index

1. **README.md** - Project overview
2. **QUICK_START.md** - ⭐ You are here
3. **WIRING_GUIDE.md** - Hardware connections
4. **LIBRARIES_INSTALLATION.md** - Software setup
5. **OPERATION_MANUAL.md** - Detailed operation guide
6. **PID_TUNING_GUIDE.md** - Performance tuning
7. **TROUBLESHOOTING.md** - Problem solving

---

## ⚠️ Safety Reminder

- ✓ Always fly in open areas
- ✓ Keep away from people and obstacles
- ✓ Monitor battery voltage
- ✓ Have emergency kill switch ready
- ✓ Follow local UAV regulations
- ✓ Never fly near airports
- ✓ Respect privacy

---

**Ready to Fly? Let's go! 🚁**

**Questions?** Check the full documentation or TROUBLESHOOTING.md

**First flight successful?** Congratulations! Now practice and have fun!
