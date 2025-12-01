# ⚡ Quick Start Guide - Quadcopter Drone

**Get flying in 30 minutes!**

---

## 📦 What You Need

### Hardware Checklist

**Flight Controller**:
- [ ] Arduino Nano
- [ ] MPU6050 (gyro + accel)
- [ ] MS5611 (barometer)
- [ ] nRF24L01+ radio
- [ ] 4x Brushless motors + ESCs
- [ ] Passive buzzer
- [ ] LED
- [ ] 3S LiPo battery (11.1V)

**Remote Controller**:
- [ ] Arduino Nano
- [ ] nRF24L01+ radio
- [ ] 2x Analog joysticks
- [ ] 4x Push buttons
- [ ] 2x Toggle switches
- [ ] 9V battery or 3x AA

**Tools**:
- [ ] Soldering iron
- [ ] Wire strippers
- [ ] Multimeter
- [ ] USB cable (for programming)

---

## 🚀 30-Minute Setup

### STEP 1: Install Software (5 minutes)

1. **Download Arduino IDE**
   - Get from: https://www.arduino.cc/
   - Version 1.8.19 or newer

2. **Install Libraries**
   - Open Arduino IDE
   - Go to: Sketch → Include Library → Manage Libraries
   - Install these:
     - `Adafruit MPU6050`
     - `Adafruit Unified Sensor`
     - `MS5611` by Rob Tillaart
     - `RF24` by TMRh20

3. **Download This Project**
   - Clone or download repository
   - Extract to your Arduino folder

**✅ Checkpoint**: Libraries installed, no errors

---

### STEP 2: Build Remote Controller (10 minutes)

**Why RC first?** Easier to build, lets you test radio immediately

**Wiring**:
```
Arduino Nano → Components
──────────────────────────
A0 → Left Joystick VRy (Throttle)
A1 → Left Joystick VRx (Yaw)
A2 → Right Joystick VRy (Pitch)
A3 → Right Joystick VRx (Roll)

D4 → Button 1 (other side → GND)
D5 → Button 2 (other side → GND)
D6 → Button 3 (other side → GND)
D7 → Button 4 (other side → GND)

D2 → Switch 1 (other side → GND)
D3 → Switch 2 (other side → GND)

D9  → nRF24L01+ CE
D10 → nRF24L01+ CSN
D11 → nRF24L01+ MOSI
D12 → nRF24L01+ MISO
D13 → nRF24L01+ SCK
3.3V → nRF24L01+ VCC (+ 10µF capacitor to GND!)
GND → nRF24L01+ GND

5V → Joystick VCC (both)
GND → Joystick GND (both)
```

**Upload Code**:
1. Open `RemoteController/RemoteController.ino`
2. Select: Tools → Board → Arduino Nano
3. Select: Tools → Processor → ATmega328P (Old Bootloader)
4. Select correct COM port
5. Click Upload

**Test**:
1. Open Serial Monitor (115200 baud)
2. Center joysticks
3. Should see: "REMOTE CONTROLLER READY!"
4. Move sticks → values change
5. Press buttons → shows button names

**✅ Checkpoint**: RC transmits, Serial shows joystick values

---

### STEP 3: Build Flight Controller (10 minutes)

**Wiring**:
```
Arduino Nano → Components
──────────────────────────
D3 → Front-Left Motor ESC signal
D5 → Front-Right Motor ESC signal
D6 → Rear-Right Motor ESC signal
D9 → Rear-Left Motor ESC signal

D7 → LED + (with 220Ω resistor to GND)
D8 → Buzzer + (other side to GND)

D4  → nRF24L01+ CE
D10 → nRF24L01+ CSN
D11 → nRF24L01+ MOSI
D12 → nRF24L01+ MISO
D13 → nRF24L01+ SCK
3.3V → nRF24L01+ VCC (+ 10µF capacitor!)
GND → nRF24L01+ GND

A4 → MPU6050 SDA + MS5611 SDA
A5 → MPU6050 SCL + MS5611 SCL
5V → MPU6050 VCC + MS5611 VCC
GND → MPU6050 GND + MS5611 GND

VIN → Battery + (11.1V, through ESC BEC)
GND → Battery -
```

**⚠️ IMPORTANT**: 
- Remove propellers before testing!
- Double-check motor wiring
- Add 10µF capacitor to nRF24L01+

**Upload Code**:
1. Open `FlightController/FlightController.ino`
2. Same board settings as RC
3. Click Upload

**Test**:
1. Place on flat surface
2. Power on
3. Open Serial Monitor (115200 baud)
4. Should see:
   ```
   ✅ Motors initialized
   ✅ Radio initialized
   ✅ MPU6050 initialized
   ✅ MS5611 barometer initialized
   ⏳ Calibrating gyro... DONE
   ⏳ Calibrating altitude... DONE
   ✅ SYSTEM READY!
   ```
5. Should hear 2 beeps

**✅ Checkpoint**: FC initializes, receives RC data, Serial shows "RC:OK"

---

### STEP 4: Motor Test (5 minutes)

**PROPELLERS MUST BE OFF!**

1. **Check motor directions**:
   - Press Button 2 on RC (Motor Test)
   - All 4 motors should spin briefly
   - If not → check wiring

2. **Configure motor rotation**:
   ```
   Required:
   FL (D3) = Counter-Clockwise
   FR (D5) = Clockwise
   RR (D6) = Counter-Clockwise
   RL (D9) = Clockwise
   ```
   - To reverse: Swap any 2 motor wires to ESC

3. **Calibrate ESCs** (if needed):
   ```
   a. Disconnect battery
   b. Set all motors to 2000µs in code (max)
   c. Connect battery → ESCs beep
   d. Set all motors to 1000µs (min)
   e. ESCs beep confirmation
   ```

**✅ Checkpoint**: All motors spin correctly, directions verified

---

### STEP 5: First Flight Test (propellers ON)

**⚠️ SAFETY**:
- Fly outdoors in open area
- Keep 5+ meters from people
- Wear safety glasses
- Have emergency stop ready (Button 3)

**Pre-flight checklist**:
- [ ] All wiring secure
- [ ] Battery charged (>11V)
- [ ] Propellers tight and correct orientation
- [ ] RC powered on, joysticks centered
- [ ] FC calibrated (gyro + altitude)
- [ ] Serial Monitor shows "RC:OK"
- [ ] Clear area (no people/obstacles)

**Flight procedure**:

1. **Place drone on flat ground**

2. **Configure switches**:
   - SW1 = OFF (manual throttle)
   - SW2 = ON (ANGLE mode)

3. **Automatic Takeoff**:
   - Press Button 4 (Takeoff)
   - Drone arms automatically
   - Rises to 1.5m
   - Hovers in place!

4. **Manual Control**:
   - Right stick right → Moves right
   - Right stick forward → Moves forward
   - Left stick left/right → Rotates
   - Throttle up/down → Adjusts altitude

5. **Automatic Landing**:
   - Press Button 3 (Landing)
   - Descends gently
   - Disarms at ground
   - 3 beeps = safe!

**✅ Checkpoint**: Successful takeoff, hover, and landing!

---

## 🎯 Next Steps

### If Everything Works:

1. **Practice basic flying**:
   - Hover in place (5 minutes)
   - Figure-8 pattern
   - Forward/back, left/right
   - Yaw rotations

2. **Try ALTITUDE HOLD**:
   - Enable SW1 (ON)
   - Drone maintains altitude automatically
   - Easier to fly!

3. **Tune PID values**:
   - See `docs/PID_TUNING.md`
   - Adjust for your specific frame/motors

### If Something's Wrong:

**Drone flips on takeoff**:
- Check motor directions (see Step 4)
- Recalibrate gyro (Button 1)

**Won't ARM**:
- Check radio connection (Serial: "RC:OK")
- Lower throttle stick

**Drifts in one direction**:
- Recalibrate gyro on flat surface
- Tune PID values

**Full troubleshooting**: See `docs/TROUBLESHOOTING.md`

---

## 📚 Learn More

### Documentation

- **README.md**: Complete overview
- **docs/PID_TUNING.md**: Detailed tuning guide
- **docs/FLIGHT_MODES.md**: All 5 flight modes explained
- **docs/TROUBLESHOOTING.md**: Fix common issues
- **LIBRARIES.txt**: Library installation details

### Control Mapping Reference

**Left Joystick**:
- Up/Down = Throttle (altitude)
- Left/Right = Yaw (rotation)

**Right Joystick**:
- Up/Down = Pitch (forward/back)
- Left/Right = Roll (strafe left/right)

**Buttons**:
- Button 1 = Calibrate sensors
- Button 2 = Motor test
- Button 3 = Auto landing
- Button 4 = Auto takeoff

**Switches**:
- SW1 = Altitude Hold ON/OFF
- SW2 = ANGLE/ACRO mode

---

## ⚠️ Safety Reminders

1. **Always remove propellers** when testing indoors
2. **Fly in open areas** away from people
3. **Check battery voltage** before each flight
4. **Inspect hardware** after crashes
5. **Follow local drone regulations**

---

## 🎓 Tips for Success

### For Beginners:

1. **Start with ANGLE mode** (auto-leveling)
2. **Practice hovering** before flying around
3. **Use ALTITUDE HOLD** to reduce workload
4. **Fly in calm weather** (no wind)
5. **Keep flights short** (2-3 minutes) while learning

### For Advanced:

1. **Try ACRO mode** for aerobatics
2. **Tune PID values** for your specific setup
3. **Add FPV camera** for first-person flying
4. **Modify code** to add features
5. **Share your improvements** with community!

---

## 📊 Expected Performance

With default settings:
- **Flight time**: 5-10 minutes (depends on battery)
- **Range**: 100-500 meters (visual line of sight)
- **Max tilt**: ±25° (ANGLE mode)
- **Max rate**: ±300°/s (ACRO mode)
- **Altitude accuracy**: ±10cm (with MS5611)
- **Control latency**: <20ms (250Hz loop)

---

## ✅ Success Criteria

You'll know your build is successful when:

- [ ] Remote controller transmits reliably
- [ ] Flight controller receives commands
- [ ] All 4 motors spin correctly
- [ ] Gyro and altitude calibrate without errors
- [ ] Drone takes off smoothly (Button 4)
- [ ] Hovers stably in place
- [ ] Responds to stick inputs
- [ ] Lands safely (Button 3)
- [ ] No oscillations or vibrations

**If all checked → congratulations! 🎉**

---

## 🆘 Emergency Contacts

**If you get stuck**:

1. **Read troubleshooting guide**: `docs/TROUBLESHOOTING.md`
2. **Check Serial Monitor**: Often shows error messages
3. **Double-check wiring**: 90% of issues are wiring
4. **Test components individually**: Isolate the problem
5. **Ask for help**: Include Serial output and video

---

**Happy Flying! 🚁**

*Remember: Building a quadcopter takes patience. Don't rush, follow instructions carefully, and prioritize safety!*
