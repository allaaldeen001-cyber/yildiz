# Quick Start Guide

## 🚀 Get Flying in 30 Minutes

This is a condensed quick-start guide. For complete documentation, see [README.md](README.md).

---

## ⚡ 1. Hardware Setup (10 minutes)

### Flight Controller Connections

```
Arduino Nano Connections:
├─ MPU6050:  SDA→A4, SCL→A5, INT→D2
├─ MS5611:   SDA→A4, SCL→A5 (shared I2C)
├─ NRF24L01: CE→D4, CSN→D10, SPI pins (D11/D12/D13)
├─ ESCs:     FL→D3, FR→D5, RR→D6, RL→D9
├─ Buzzer:   Signal→D8
├─ LED:      Positive→D7 (with 220Ω resistor)
└─ Power:    5V from BEC, common GND
```

### Remote Controller Connections

```
Arduino Nano Connections:
├─ NRF24L01:     CE→D9, CSN→D10, SPI pins (D11/D12/D13)
├─ Joysticks:    Left V→A0, Left H→A1, Right V→A2, Right H→A3
├─ Buttons:      Button1→D4, Button2→D5 (with internal pullup)
├─ Switches:     SW1→D2, SW2→D3 (with internal pullup)
└─ Power:        USB or 9V battery
```

**Critical**: NRF24L01 needs **3.3V** (not 5V!) - use adapter with regulator!

---

## 💾 2. Upload Firmware (5 minutes)

### Install Library

Arduino IDE → Tools → Manage Libraries → Search **"RF24"** → Install

### Upload to FC

1. Open `FlightController_FC.ino`
2. Tools → Board → **Arduino Nano**
3. Tools → Processor → **ATmega328P (Old Bootloader)**
4. Select COM port
5. Click **Upload**

### Upload to RC

1. Open `RemoteController_RC.ino`
2. Same board/processor settings
3. Click **Upload**

---

## ✅ 3. Pre-Flight Checklist (5 minutes)

**Hardware**:
- [ ] All wires secure, no shorts
- [ ] Propellers **REMOVED** (for initial testing!)
- [ ] Battery charged (12.6V for 3S)
- [ ] Motors mounted correctly (check rotation direction later)

**Software**:
- [ ] Both firmwares uploaded successfully
- [ ] No compilation errors
- [ ] Serial monitors work (115200 baud)

---

## 🧪 4. Calibration (5 minutes)

### Step 1: Power On

1. Turn ON **Remote Controller** (RC)
2. Turn ON **Flight Controller** (FC)
3. Listen for **2 beeps** from FC
4. Check FC **LED blinks** (indicates NRF link)

### Step 2: IMU Calibration

1. Place drone on **perfectly level surface**
2. **SW_2 switch = OFF** (disarmed)
3. Press **Button_1** on RC
4. Wait 5 seconds (don't touch drone!)
5. Listen for result:
   - **2 short beeps** = ✅ Success
   - **1 long beep** = ❌ Failed (try again on level surface)

### Step 3: ESC Calibration + Motor Test

1. **VERIFY PROPELLERS ARE OFF!**
2. Press **Button_2** on RC
3. ESC calibration sequence runs automatically
4. Motors test individually: FL → FR → RR → RL
5. **Check rotation directions** (looking from above):

```
     FRONT
FL(⟳) - FR(⟲)
  \ X /
  / X \
RL(⟲) - RR(⟳)
     BACK
     
⟳ = Clockwise
⟲ = Counter-clockwise
```

**If wrong direction**: Swap any 2 motor wires, repeat Button_2 test

---

## 🎮 5. First Flight (5 minutes)

### Step 1: Install Propellers

- **CW propellers** on **CW motors** (FL, RR)
- **CCW propellers** on **CCW motors** (FR, RL)
- Tighten securely (but don't over-tighten)

### Step 2: Flight Area

- Open space (at least 5m x 5m)
- No people, obstacles, or pets
- Grass surface (soft landing)
- No wind (for first flight)

### Step 3: Takeoff

1. Place drone on ground, **FRONT facing away**
2. Stand 3m behind drone
3. **SW_1 = OFF** (altitude hold disabled for first flight)
4. **SW_2 = ON** (armed)
5. Slowly increase **left stick UP** (throttle)
6. Liftoff at ~1500 throttle
7. Hover at **30cm height** for 10 seconds
8. Gently reduce throttle to land
9. **SW_2 = OFF** immediately after landing

### Step 4: Test Controls

If hover is stable:

- **Right stick RIGHT** → Drone tilts right (roll)
- **Right stick UP** → Drone tilts forward (pitch)
- **Left stick LEFT** → Drone rotates CCW (yaw)
- **Left stick UP** → Drone climbs (throttle)

---

## 🎛️ Control Summary

| Input | Function | Direction |
|-------|----------|-----------|
| **Left Stick V** | Throttle | Up=climb, Down=descend |
| **Left Stick H** | Yaw | Left=CCW rotate, Right=CW rotate |
| **Right Stick V** | Pitch | Up=forward, Down=backward |
| **Right Stick H** | Roll | Left=left tilt, Right=right tilt |
| **SW_2 (D3)** | ARM/DISARM | ON=armed, OFF=kill motors |
| **SW_1 (D2)** | Altitude Hold | ON=enabled, OFF=manual |
| **Button_1** | Calibrate IMU | Press once |
| **Button_2** | ESC Cal + Motor Test | Press once (disarmed) |

---

## 🛡️ Safety Features

### Automatic Failsafes

1. **Link Loss** → Motors cut after 500ms
2. **Disarm Switch** → SW_2=OFF instantly kills motors
3. **Uncalibrated** → Motors blocked until IMU calibrated
4. **Max Tilt** → ±30° limit
5. **Throttle Cap** → 65% maximum thrust

### Emergency Stop

**Flip SW_2 to OFF** → Motors stop immediately

---

## 🔧 Troubleshooting Quick Fixes

| Problem | Solution |
|---------|----------|
| **No NRF link (LED off)** | Check NRF24L01 power (3.3V!), add capacitor, check wiring |
| **Calibration fails** | Ensure level surface, don't touch drone during calibration |
| **Motors don't spin** | Complete Button_1 and Button_2 calibrations, SW_2=ON, throttle>1100 |
| **Flips on takeoff** | Wrong motor direction or propeller orientation |
| **Oscillations** | Reduce PID gains (see TUNING_GUIDE.md) |
| **Drifts** | Recalibrate IMU (Button_1) on level surface |

---

## 📚 Next Steps

- **Tune PIDs**: See [TUNING_GUIDE.md](TUNING_GUIDE.md) for optimal performance
- **Full Testing**: See [TESTING_PROCEDURES.md](TESTING_PROCEDURES.md) for comprehensive tests
- **Learn Architecture**: See [README.md](README.md) for system details

---

## ⚠️ Safety Warnings

1. **NEVER arm with propellers attached until calibration complete**
2. **Always remove propellers during bench testing**
3. **Keep fingers away from spinning propellers**
4. **Fly in open areas away from people**
5. **Monitor battery voltage** (land at 10.5V for 3S)
6. **Practice emergency disarm** (SW_2 toggle)

---

## 🎓 Learning Progression

### Beginner (Flights 1-5)
- Master stable hover
- Practice gentle movements
- Emergency disarm drills

### Intermediate (Flights 6-15)
- Figure-8 patterns
- Forward flight
- Altitude hold mode (SW_1=ON)

### Advanced (Flights 16+)
- PID tuning for your preferences
- Aggressive maneuvers
- Long-range flights (monitor battery!)

---

**Ready to fly? Follow steps 1-5 above and you'll be airborne in 30 minutes!** 🚁

For complete documentation, see [README.md](README.md).
