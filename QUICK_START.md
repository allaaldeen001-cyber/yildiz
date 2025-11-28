# 🚀 QUICK START GUIDE - Quadcopter Drone

## ⚡ Fast Setup (30 minutes)

### 1. Upload Firmware (5 minutes)

#### Flight Controller
1. Open Arduino IDE
2. Load `FlightController/FlightController.ino`
3. Select **Board**: Arduino Nano
4. Select **Processor**: ATmega328P (Old Bootloader)
5. Select correct **Port**
6. Click **Upload**

#### RC Transmitter
1. Open Arduino IDE
2. Load `RCTransmitter/RCTransmitter.ino`
3. Select **Board**: Arduino Nano
4. Select **Processor**: ATmega328P (Old Bootloader)
5. Select correct **Port**
6. Click **Upload**

---

### 2. Required Libraries
Install via Arduino Library Manager (Sketch → Include Library → Manage Libraries):

| Library | Author | Version |
|---------|--------|---------|
| RF24 | TMRh20 | Latest |
| MPU6050 | Electronic Cats | Latest |
| MS5611 | Rob Tillaart | Latest |
| Servo | Arduino | Built-in |
| Wire | Arduino | Built-in |

---

### 3. Hardware Connections

#### Flight Controller

```
┌─────────────────────────────────────────────┐
│           FLIGHT CONTROLLER                  │
│                                              │
│  Arduino Nano                                │
│  ┌──────────────────────────┐               │
│  │  D2  →  LED              │               │
│  │  D3  →  ESC Motor 1 (FR) │  ◄── 1000-2000µs PWM
│  │  D4  →  ESC Motor 2 (RR) │  ◄── 1000-2000µs PWM
│  │  D5  →  ESC Motor 3 (RL) │  ◄── 1000-2000µs PWM
│  │  D6  →  ESC Motor 4 (FL) │  ◄── 1000-2000µs PWM
│  │  D7  →  Buzzer (+)        │               │
│  │  D8  →  Status LED (+)    │               │
│  │  D9  →  NRF24 CE          │               │
│  │  D10 →  NRF24 CSN         │               │
│  │  D11 →  NRF24 MOSI        │               │
│  │  D12 →  NRF24 MISO        │               │
│  │  D13 →  NRF24 SCK         │               │
│  │  A4  →  SDA (MPU+MS5611)  │               │
│  │  A5  →  SCL (MPU+MS5611)  │               │
│  │  3.3V → NRF24 VCC         │  ⚠️ NOT 5V!   │
│  │  5V  →  MPU6050, MS5611   │               │
│  │  GND →  All GND           │               │
│  └──────────────────────────┘               │
│                                              │
│  Motor Layout (X-Frame):                    │
│         FRONT                                │
│     M4 ↺    ↻ M1                            │
│        \    /                                │
│         \  /                                 │
│          \/                                  │
│          /\                                  │
│         /  \                                 │
│        /    \                                │
│     M3 ↻    ↺ M2                            │
│         REAR                                 │
│                                              │
│  ↻ = CW (Clockwise)                         │
│  ↺ = CCW (Counter-Clockwise)                │
└─────────────────────────────────────────────┘
```

#### RC Transmitter

```
┌─────────────────────────────────────────────┐
│           RC TRANSMITTER                     │
│                                              │
│  Arduino Nano                                │
│  ┌──────────────────────────┐               │
│  │  D2  →  Toggle Switch     │  ◄── Arm/Kill │
│  │  D3  →  Button 1          │  ◄── Calibrate│
│  │  D4  →  Button 2          │  ◄── Motor Test│
│  │  D8  →  Status LED (+)    │               │
│  │  D9  →  NRF24 CE          │               │
│  │  D10 →  NRF24 CSN         │               │
│  │  D11 →  NRF24 MOSI        │               │
│  │  D12 →  NRF24 MISO        │               │
│  │  D13 →  NRF24 SCK         │               │
│  │  A0  →  Joystick 1 VRx    │  ◄── Throttle │
│  │  A1  →  Joystick 1 VRy    │  ◄── Yaw      │
│  │  A2  →  Joystick 2 VRx    │  ◄── Pitch    │
│  │  A3  →  Joystick 2 VRy    │  ◄── Roll     │
│  │  5V  →  Joysticks VCC     │               │
│  │  3.3V → NRF24 VCC         │  ⚠️ NOT 5V!   │
│  │  GND →  All GND           │               │
│  └──────────────────────────┘               │
│                                              │
│  Joystick Layout:                            │
│  ┌────────┐        ┌────────┐               │
│  │  LEFT  │        │ RIGHT  │               │
│  │ STICK  │        │ STICK  │               │
│  │        │        │        │               │
│  │ Up: ▲  │        │ Up: ▲  │               │
│  │ Throttle│        │ Pitch  │               │
│  │        │        │        │               │
│  │L◄──┼──►R│        │L◄──┼──►R│               │
│  │   Yaw  │        │  Roll  │               │
│  └────────┘        └────────┘               │
└─────────────────────────────────────────────┘
```

---

### 4. Step-by-Step Flight Preparation

#### ⚠️ SAFETY FIRST: REMOVE PROPELLERS FOR ALL TESTING!

#### Step 1: Power On (30 seconds)
1. **Power RC Transmitter** first (via USB or battery)
   - Status LED should blink slowly
   - Serial monitor shows: "RC TRANSMITTER READY"
   - Joystick auto-calibration runs on first boot
   
2. **Power Flight Controller** (via LiPo or USB)
   - You'll hear 2 beeps
   - Serial monitor shows initialization
   - Wait for NRF connection
   - **You'll hear 2 more beeps** when connected ✓

**Serial Monitor Shows:**
```
[✓] NRF CONNECTION ESTABLISHED!
WAITING FOR RC CONNECTION...
Step 1: Power on RC Transmitter ✓
Step 2: Set toggle switch to KILL position
```

---

#### Step 2: Safety Check (10 seconds)
1. **Toggle switch to KILL/DISARM position** (DOWN)
   - Status LED blinks slowly
   - Serial confirms: "KILL mode"

**Serial Monitor Shows:**
```
[KILL] Drone DISARMED
Step 3: Press Button 1 to calibrate
```

---

#### Step 3: Calibration (3 minutes)
⚠️ **CRITICAL**: Place drone on **LEVEL SURFACE**!

1. **Press Button 1** on RC transmitter
   - LED blinks twice
   - Calibration starts automatically

2. **Wait for calibration sequence:**
   - ESC calibration (30 seconds)
   - MPU6050 calibration (10 seconds) - **DON'T MOVE!**
   - MS5611 calibration (5 seconds)
   - Saves to EEPROM

3. **Listen for completion beeps**: 3 short + 1 long

**Serial Monitor Shows:**
```
[CAL] Calibrating ESCs...
[✓] ESC calibration complete!
[CAL] Calibrating MPU6050...
[✓] MPU6050 calibration complete!
[CAL] Calibrating MS5611...
[✓] MS5611 calibration complete!
[✓] Calibration saved!

========================================
  CALIBRATION COMPLETE!
========================================
Next step: ARM the drone
```

---

#### Step 4: Arm Drone (5 seconds)
1. **Toggle switch to ARM position** (UP)
   - Status LED turns **SOLID ON**
   - 2 quick beeps
   - Serial confirms: "DRONE ARMED"

⚠️ **WARNING**: Motors can now spin!

**Serial Monitor Shows:**
```
[ARM] Drone ARMED - BE CAREFUL!
[✓] DRONE ARMED!
    Gradually increase throttle
Step 5: Test motors with Button 2
```

---

#### Step 5: Motor Test (10 seconds)
⚠️ **REMOVE PROPELLERS FIRST!**

1. **Press Button 2** on RC transmitter
   - Motors spin up gradually to low speed
   - Run for 3 seconds
   - Spin down gradually

2. **Verify all 4 motors spin:**
   - Motor 1 (Front-Right)
   - Motor 2 (Rear-Right)
   - Motor 3 (Rear-Left)
   - Motor 4 (Front-Left)

**Serial Monitor Shows:**
```
[TEST] Running motor test...
[✓] Motor test complete!

========================================
  DRONE READY TO FLY!
========================================
Flight Tips:
1. Start with SMALL throttle increases
2. Keep stick movements SMOOTH
3. Practice hovering before moving
4. KILL SWITCH ready at all times!
```

---

#### Step 6: Install Propellers (5 minutes)
**Only after successful motor test!**

1. **Match propeller rotation:**
   - Motor 1 (FR): CW propeller ↻
   - Motor 2 (RR): CCW propeller ↺
   - Motor 3 (RL): CW propeller ↻
   - Motor 4 (FL): CCW propeller ↺

2. **Secure propellers tightly**
   - Use propeller nuts/bolts
   - Check for cracks or damage
   - Ensure proper orientation

3. **Final safety check:**
   - All screws tight
   - Battery secure
   - Wires clear of propellers
   - Frame solid with no loose parts

---

### 5. First Flight (10 minutes)

#### Pre-Flight Checklist
- [ ] Open area with no people/obstacles
- [ ] Battery fully charged (>11.1V)
- [ ] Propellers installed correctly
- [ ] Kill switch tested
- [ ] Weather calm (no wind)
- [ ] Safe distance (10 feet minimum)

#### First Takeoff
1. **Place drone on flat ground**
2. **Arm drone** (toggle switch UP)
3. **Slowly increase throttle** to 40-50%
4. **Drone should hover** at about 1m height
5. **Make small corrections** with right stick
6. **Practice hovering** for 30 seconds
7. **Reduce throttle** slowly to land
8. **Disarm** (toggle switch DOWN)

#### Control Tips
| Stick | Action | Result |
|-------|--------|--------|
| Left Up | Increase throttle | Climb |
| Left Down | Decrease throttle | Descend |
| Left Left/Right | Yaw | Rotate left/right |
| Right Up | Pitch forward | Fly forward |
| Right Down | Pitch back | Fly backward |
| Right Left/Right | Roll | Strafe left/right |

---

### 6. Telemetry Data

During flight, serial monitor displays real-time data:

```
=== FLIGHT DATA ===
Throttle: 1500 | Yaw: 1500 | Pitch: 1500 | Roll: 1500
Alt: 1.25m | Climb: 0.02m/s
IMU: Pitch=0.5° Roll=-0.3° Yaw=45.2°
Motors: [1450, 1480, 1470, 1460]
NRF: Ch108 | Signal: GOOD
```

**What to monitor:**
- **Throttle**: Should be 1000-2000 (1500 = hover)
- **Altitude**: Current height above takeoff point
- **IMU angles**: Tilt of the drone (should be near 0° when hovering)
- **Motors**: Individual motor speeds (should be balanced)
- **Signal**: Must show "GOOD" for safe flight

---

## 🛠️ Troubleshooting

### NRF Not Connecting
**Symptoms**: No connection beeps, LED blinks fast
**Solutions**:
1. Check NRF24 power is 3.3V (NOT 5V!)
2. Add 10µF capacitor across NRF VCC/GND
3. Use shorter wires (<10cm)
4. Verify SPI pins (MOSI, MISO, SCK, CE, CSN)

### Motors Not Spinning
**Symptoms**: Armed but no motor movement
**Solutions**:
1. Re-run ESC calibration
2. Check ESC power (11.1V LiPo connected)
3. Verify PWM pins D3-D6
4. Check arm switch is in ARM position

### Drone Not Stable
**Symptoms**: Flips, wobbles, won't hover
**Solutions**:
1. Re-calibrate IMU on **perfectly level** surface
2. Check motor directions match code
3. Verify propeller rotation directions
4. Check frame is rigid (no flex)
5. Balance propellers

### Joystick Center Issues
**Symptoms**: Drone drifts when sticks centered
**Solutions**:
1. Re-run joystick calibration on RC
2. Center all sticks before calibration
3. Move sticks to full extremes during calibration
4. Check analog pins A0-A3

---

## ⚠️ EMERGENCY PROCEDURES

### If Drone Goes Out of Control
1. **Toggle switch to KILL** immediately
2. All motors stop instantly
3. Check for damage before restarting

### If Signal Lost
- Drone enters **FAILSAFE** mode automatically
- Throttle reduces gradually
- Attempts to land safely
- Status LED blinks rapidly

### If Battery Low
- Land immediately when voltage <10.5V
- Don't fly until recharged
- Never over-discharge LiPo batteries!

---

## 📈 PID Tuning (Advanced)

Default PID values are conservative. To tune:

1. **Test one axis at a time** (Roll → Pitch → Yaw)
2. **Start with P gain:**
   - Too low = sluggish response
   - Too high = oscillations
3. **Add D gain** to dampen oscillations
4. **Add I gain** last, for drift correction

**Edit in code:**
```cpp
#define PID_ROLL_KP       1.5f   // Increase for faster response
#define PID_ROLL_KI       0.05f  // Increase to fix drift
#define PID_ROLL_KD       18.0f  // Increase to reduce oscillations
```

---

## 🎓 Learning Path

### Beginner (Day 1-7)
- Master hovering in place
- Learn basic throttle control
- Practice slow movements
- Build confidence with kill switch

### Intermediate (Week 2-4)
- Forward/backward flight
- Left/right strafing
- Smooth turns with yaw
- Figure-8 patterns

### Advanced (Month 2+)
- Altitude hold tuning
- Acrobatic maneuvers
- Long-range flight
- FPV camera integration

---

## 📞 Support

**Issues?** Check:
1. Serial monitor error messages
2. LED blink patterns
3. Buzzer beep codes
4. README troubleshooting section

**Success?** Happy flying! 🚁✨

---

**Created by**: Professional Embedded Systems Engineer
**Version**: 1.0.0
**License**: Open Source Educational Use
