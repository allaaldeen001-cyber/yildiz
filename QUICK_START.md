# Quick Start Guide

Get your drone flying in 30 minutes! ⚡

---

## ⚡ Fast Track Setup

### Prerequisites
- All hardware assembled (see WIRING_DIAGRAM.md)
- Arduino IDE installed
- **PROPELLERS REMOVED!**

---

## 📥 Step 1: Install Software (5 minutes)

### 1.1 Download Arduino IDE
- Go to: https://www.arduino.cc/en/software
- Download and install for your OS
- Open Arduino IDE

### 1.2 Install RF24 Library
1. Open Arduino IDE
2. Click: `Sketch` → `Include Library` → `Manage Libraries`
3. Search: "RF24"
4. Install: **RF24 by TMRh20**
5. Close Library Manager

---

## 📤 Step 2: Upload Firmware (10 minutes)

### 2.1 Upload to Flight Controller

1. **Connect Flight Controller Arduino to computer** (USB cable)

2. **Configure Arduino IDE:**
   - `Tools` → `Board` → `Arduino Nano`
   - `Tools` → `Processor` → Try "ATmega328P" first
   - `Tools` → `Port` → Select correct port (COM3, /dev/ttyUSB0, etc.)

3. **Open Flight Controller code:**
   - `File` → `Open`
   - Navigate to: `FlightController/FlightController.ino`

4. **Upload:**
   - Click Upload button (→) or press Ctrl+U
   - Wait for "Done uploading"

   **If upload fails:**
   - Try `Tools` → `Processor` → `ATmega328P (Old Bootloader)`
   - Check USB cable (must be data cable, not power-only)

5. **Verify:**
   - Open Serial Monitor (Ctrl+Shift+M)
   - Set baud rate: **115200**
   - Should see: "=== Flight Controller Ready ==="

6. **Disconnect Flight Controller**

### 2.2 Upload to Remote Controller

1. **Connect Remote Controller Arduino to computer**

2. **Open Remote Controller code:**
   - `File` → `Open`
   - Navigate to: `RemoteController/RemoteController.ino`

3. **Upload** (same process as above)

4. **Verify:**
   - Open Serial Monitor (115200 baud)
   - Should see nice formatted display with controls

5. **Keep Remote connected** to computer (for monitoring)

---

## 🔌 Step 3: Power Up (2 minutes)

### 3.1 Power Remote Controller
- Already powered via USB
- Check serial monitor shows system ready

### 3.2 Power Flight Controller
- **Ensure PROPELLERS REMOVED!**
- Connect battery to ESCs or separate power supply
- Should hear: 1 short beep, then 2 short beeps
- ESCs will beep/chirp (musical tones = normal)

---

## 🎯 Step 4: Verify Communication (3 minutes)

### Check Link Status

Look at Remote Controller serial monitor:

```
┌─── COMMUNICATION STATUS ───────────┐
│ Link Status: ✓ CONNECTED           │
│ Channel: 103 | Success Rate: 98%   │
└────────────────────────────────────┘
```

**If shows "✗ NO SIGNAL":**
1. Check Flight Controller has power
2. Verify both NRF24L01 modules have 10μF capacitor
3. Check wiring (see WIRING_DIAGRAM.md)
4. See TROUBLESHOOTING.md

**If CONNECTED:**
✓ Proceed to next step!

---

## 🔧 Step 5: Calibrate Sensors (5 minutes)

### 5.1 Gyro Calibration

1. **Place drone on flat, level surface** (table, not carpet)
2. **Don't touch for next 5 seconds**
3. **Set Kill Switch (SW_2) to "1" (armed position)**
4. **Press Button_1 on remote**
5. **Wait** - LED on drone will flash
6. **Listen for result:**
   - ✓ **2 beeps** = Success! Continue.
   - ✗ **1 long beep** = Failed. Try again on more stable surface.

7. **Check serial monitor:**
```
┌─── DRONE TELEMETRY ────────────────┐
│ Status: ✓ READY                    │
│ Attitude: Roll=0.1° Pitch=-0.2°    │
└────────────────────────────────────┘
```

**Calibration complete when angles near 0° and says "READY"**

---

## 🧪 Step 6: Test Controls (5 minutes)

### 6.1 Test Inputs

**Watch serial monitor while moving controls:**

1. **Throttle stick** (left vertical):
   - Move up/down
   - Value should change 1000-2000

2. **Yaw stick** (left horizontal):
   - Move left/right
   - Value should change 1000-2000, centered at 1500

3. **Pitch stick** (right vertical):
   - Move up/down
   - Value should change, centered at 1500

4. **Roll stick** (right horizontal):
   - Move left/right
   - Value should change, centered at 1500

5. **All switches and buttons:**
   - Toggle each switch - status should change
   - Press each button - status should show pressed

**All controls working?** ✓ Continue!

### 6.2 Test Motors

**⚠️ CRITICAL: PROPELLERS MUST BE REMOVED!**

1. **Set switches:**
   - SW_1 (Altitude Hold): **OFF** (0)
   - SW_2 (Kill Switch): **ON** (1)

2. **Throttle stick to minimum** (all the way down)

3. **Press and hold Button_2** (Motor Arm)
   - Should hear 3 beeps
   - Serial shows "⚠ ARMED & FLYING"

4. **SLOWLY** increase throttle
   - All 4 motors should start spinning
   - Increase gradually - motors speed up together

5. **Check motor directions** (NO PROPS!)
   ```
        FRONT
     FL ↻  ↺ FR
       \    /
        \  /
         \/
         /\
        /  \
       /    \
     RL ↺  ↻ RR
        REAR
   
   ↻ = Counter-Clockwise
   ↺ = Clockwise
   ```

6. **Lower throttle to minimum**

7. **Disarm:** Toggle SW_2 to OFF

**All motors spin correctly?** ✓ Ready for props!

---

## 🚁 Step 7: First Flight! (Setup 5 min, Flying ∞)

### 7.1 Install Propellers

**Match propeller type to motor rotation:**

- **Front-Left (FL)**: Counter-clockwise prop
- **Front-Right (FR)**: Clockwise prop  
- **Rear-Right (RR)**: Counter-clockwise prop
- **Rear-Left (RL)**: Clockwise prop

**Tip:** Props are marked "CW" or "CCW" on the blade.

**Check:** Leading edge (thicker edge) points forward (direction of travel).

### 7.2 Pre-Flight Checklist

- [ ] Battery fully charged (>11.4V for 3S)
- [ ] All propellers tight (hand-tight, not over-torqued)
- [ ] No loose wires
- [ ] Clear area (5+ meters radius, no people)
- [ ] Soft ground (grass recommended for first flights)

### 7.3 Final Calibration

1. Place drone on ground (level surface)
2. Power on remote, then drone
3. Press Button_1 (calibration)
4. Wait for 2 beeps (success)

### 7.4 Arm and Fly!

1. **Stand 2-3 meters away**

2. **Verify:**
   - SW_2 (Kill Switch) = ON
   - Throttle = minimum
   - Serial shows "READY"

3. **Arm:** Press Button_2
   - 3 beeps confirm armed
   - ⚠️ Props will spin at low throttle!

4. **Takeoff:**
   - Slowly increase throttle to 30-40%
   - Drone should gently lift off
   - Hover at 0.5-1 meter height

5. **Test Stability:**
   - Hold hover for 10 seconds
   - Observe if drone drifts or oscillates
   - If stable: ✓ Success!

6. **Test Controls** (gentle inputs!):
   - Small pitch forward → should move forward
   - Small roll left → should slide left
   - Small yaw left → should rotate left

7. **Land:**
   - Slowly reduce throttle
   - Let drone settle
   - Reduce throttle to minimum
   - **Disarm:** Toggle SW_2 to OFF (or lower throttle for 2 sec)

---

## 🎉 Congratulations!

You've successfully built and flown your Arduino drone!

---

## ⚠️ Common First Flight Issues

| Problem | Solution |
|---------|----------|
| **Drone flips on takeoff** | STOP! Props on wrong motors or backwards. See TROUBLESHOOTING.md |
| **One motor doesn't spin** | Check ESC connection, see TROUBLESHOOTING.md |
| **Drone drifts heavily** | Recalibrate gyro on perfectly level surface |
| **Oscillates/shakes** | Lower PID gains (see CALIBRATION_GUIDE.md) |
| **Doesn't take off** | Increase throttle more, check battery voltage |

---

## 📚 Next Steps

### Learn to Fly:
1. Practice hovering (hardest skill!)
2. Forward/backward flight
3. Left/right flight
4. Rotation (yaw)
5. Figure-8 patterns
6. Smooth landing

### Upgrades:
- Add FPV camera
- GPS position hold
- Altitude sensor (barometer)
- LED lights for visibility
- Battery voltage monitoring
- Longer flight time (bigger battery/efficient props)

### Advanced Features:
- Implement altitude hold (SW_1 currently unused)
- Add waypoint navigation
- Implement return-to-home
- Add telemetry display (OLED screen)

---

## 📖 Full Documentation

For detailed information, see:
- **README.md** - Project overview
- **WIRING_DIAGRAM.md** - Complete wiring schematics
- **CALIBRATION_GUIDE.md** - Detailed calibration procedures
- **TROUBLESHOOTING.md** - Problem solving
- **PARTS_LIST.md** - Shopping guide

---

## 🆘 Need Help?

Check TROUBLESHOOTING.md for:
- Communication problems
- Motor issues
- Calibration failures
- Flight instability
- And much more!

---

## ⚡ Quick Reference Card

**Print this and keep with your drone:**

```
┌─────────────────────────────────────────────┐
│         QUICK REFERENCE CARD                │
├─────────────────────────────────────────────┤
│ STARTUP:                                    │
│  1. Power remote (USB)                      │
│  2. Power drone (battery)                   │
│  3. Verify "CONNECTED"                      │
│  4. Press BTN1 (calibrate)                  │
│  5. Wait for 2 beeps                        │
│                                             │
│ ARM:                                        │
│  1. SW2 = ON                                │
│  2. Throttle = MIN                          │
│  3. Press BTN2                              │
│  4. Hear 3 beeps                            │
│                                             │
│ EMERGENCY DISARM:                           │
│  → Flip SW2 to OFF immediately!             │
│                                             │
│ CONTROLS:                                   │
│  Left Stick:  Throttle (V) | Yaw (H)        │
│  Right Stick: Pitch (V) | Roll (H)          │
│                                             │
│ SWITCHES:                                   │
│  SW1: Altitude Hold (future feature)        │
│  SW2: Kill Switch (MUST BE ON to fly)       │
│                                             │
│ BUTTONS:                                    │
│  BTN1: Calibrate gyro                       │
│  BTN2: Arm motors                           │
└─────────────────────────────────────────────┘
```

---

**Happy Flying! 🚁 Stay Safe!**

