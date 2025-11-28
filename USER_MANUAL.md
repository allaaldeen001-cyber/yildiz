# 🚁 DIY DRONE FLIGHT CONTROL SYSTEM - USER MANUAL

## 📋 TABLE OF CONTENTS
1. [System Overview](#system-overview)
2. [Hardware Setup](#hardware-setup)
3. [Wiring Guide](#wiring-guide)
4. [Software Upload](#software-upload)
5. [Calibration Procedure](#calibration-procedure)
6. [Flight Operations](#flight-operations)
7. [LED & Buzzer Signals](#led--buzzer-signals)
8. [Troubleshooting](#troubleshooting)
9. [Safety Guidelines](#safety-guidelines)

---

## 🎯 SYSTEM OVERVIEW

This drone system features:
- ✅ **Stabilized flight** with MPU6050 gyro/accelerometer
- ✅ **Altitude hold** with MS5611 barometer
- ✅ **NRF24L01 radio** with ACK for reliable communication
- ✅ **Smooth motor start** for testing
- ✅ **Safety features**: auto-disarm, signal loss protection, tilt angle limits
- ✅ **Visual feedback** via LED and audible feedback via buzzer

---

## 🔧 HARDWARE SETUP

### Flight Controller (FC) Components
- Arduino Nano
- NRF24L01 radio module (with 10µF capacitor!)
- MPU6050 gyro/accelerometer
- MS5611 barometer
- 4x ESCs + Brushless motors
- Buzzer (active or passive)
- LED
- 2x Push buttons
- 2x Switches (SPDT)
- Voltage divider for battery monitoring

### Remote Controller (RC) Components
- Arduino Nano
- NRF24L01 radio module (with 10µF capacitor!)
- 2x 2-axis joysticks
- Optional LED for link status

---

## 🔌 WIRING GUIDE

### Flight Controller Wiring

#### Power
```
Battery → Voltage Regulator (7-12V) → Arduino VIN
Battery → ESC Power Lines
GND common to all components
```

#### NRF24L01 Module
```
NRF24     Arduino Nano
VCC   →   3.3V (NOT 5V!)
GND   →   GND
CE    →   D2
CSN   →   D10
MOSI  →   D11
MISO  →   D12
SCK   →   D13

⚠️ CRITICAL: Add 10µF capacitor between VCC and GND at NRF module!
```

#### MPU6050 Sensor
```
MPU6050   Arduino Nano
VCC   →   5V
GND   →   GND
SDA   →   A4
SCL   →   A5
```

#### MS5611 Barometer
```
MS5611    Arduino Nano
VCC   →   5V (or 3.3V)
GND   →   GND
SDA   →   A4 (same as MPU)
SCL   →   A5 (same as MPU)
```

#### Motors (ESC Signal Wires)
```
ESC Signal    Arduino Nano
Front Left  → D3
Front Right → D5
Rear Right  → D6
Rear Left   → D9

Motor Layout (X configuration):
    FL(D3)  FR(D5)
        \ X /
        / X \
    RL(D9)  RR(D6)
```

#### Buttons & Switches
```
Component               Arduino Pin    Connection
Calibration Button   →  A6          →  Button to GND (internal pullup)
Smooth Start Button  →  A7          →  Button to GND (internal pullup)
Arm/Disarm Switch    →  D4          →  Switch: LOW=Armed, HIGH=Disarmed
Altitude Hold Switch →  D7          →  Switch: LOW=Active, HIGH=Off
```

#### Outputs
```
Buzzer   →  D8  →  Buzzer+ (Buzzer- to GND)
LED      →  A3  →  LED+ (with 220Ω resistor to GND)
```

#### Battery Monitor
```
Battery+ → R1 (1.5kΩ) → A0 → R2 (1kΩ) → GND

This divider reads up to 12.5V battery voltage
```

---

### Remote Controller Wiring

#### NRF24L01 Module
```
NRF24     Arduino Nano
VCC   →   3.3V
GND   →   GND
CE    →   D9
CSN   →   D10
MOSI  →   D11
MISO  →   D12
SCK   →   D13

⚠️ Add 10µF capacitor between VCC and GND!
```

#### Joysticks
```
Left Stick (Throttle & Yaw):
  X-axis (Yaw)      →  A1
  Y-axis (Throttle) →  A0
  VCC               →  5V
  GND               →  GND

Right Stick (Pitch & Roll):
  X-axis (Roll)     →  A3
  Y-axis (Pitch)    →  A2
  VCC               →  5V
  GND               →  GND
```

#### Status LED (Optional)
```
LED  →  D8  →  220Ω resistor  →  GND
```

---

## 💻 SOFTWARE UPLOAD

### Required Libraries
Install these via Arduino Library Manager:
1. `RF24` by TMRh20
2. `Smoothed` by Matthew Fryer
3. `MS5611` by Rob Tillaart

### Upload Steps

1. **Upload to Flight Controller:**
   - Open `Drone_Flight_Control.ino`
   - Select Board: "Arduino Nano"
   - Select Processor: "ATmega328P (Old Bootloader)" or "ATmega328P"
   - Select correct COM port
   - Click Upload
   - Also upload: `Barometer.ino`, `Kalman_Filter.ino` (as tabs)

2. **Upload to Remote Controller:**
   - Open `Controller.ino`
   - Select Board: "Arduino Nano"
   - Select correct COM port
   - Click Upload

---

## 🎯 CALIBRATION PROCEDURE

### ⚠️ CRITICAL: ALWAYS CALIBRATE BEFORE FIRST FLIGHT!

### Step-by-Step Calibration

1. **Power ON the Remote Controller first**
   - Wait for LED blinks (3 times)
   - Keep it powered on

2. **Power ON the Flight Controller**
   - You should hear: beep-beep-beeeep (startup tones)
   - Wait 2-5 seconds
   - You should hear: beep-beep (link confirmation)
   - LED on FC blinks when link is established

3. **Ensure Arm Switch is DISARMED (D4 = HIGH)**
   - LED should stay ON continuously = disarmed state
   - This is a safety feature

4. **Place drone on LEVEL SURFACE**
   - Must be perfectly flat
   - No vibrations

5. **Press and HOLD Calibration Button (A6) for 2 seconds**
   - You'll hear: beep-pause-beep
   - Wait... (takes ~10-15 seconds)
   - Calibration in progress
   - You'll hear: beep-beep (confirmation)
   - LED blinks twice

6. **Calibration Complete!**
   - Check Serial Monitor (57600 baud) for "Calibration complete!"
   - Ground pressure value is displayed

### What Gets Calibrated:
- ✅ MPU6050 X/Y axis offsets (saved to EEPROM)
- ✅ MS5611 ground pressure (saved to EEPROM)
- ✅ Gyro drift compensation

### Re-calibration Required:
- After moving to different altitude
- If drone drifts without input
- After firmware changes

---

## 🚁 FLIGHT OPERATIONS

### Pre-Flight Checklist
- [ ] Both batteries fully charged
- [ ] All wiring secure
- [ ] Propellers undamaged and tight
- [ ] Calibration completed on level surface
- [ ] Remote controller powered on
- [ ] Clear flying area (no obstacles/people)

---

### STEP-BY-STEP FLIGHT PROCEDURE

#### 1. POWER UP SEQUENCE
```
a) Power ON Remote Controller
b) Wait for LED blinks
c) Power ON Flight Controller
d) Wait for link confirmation beeps
e) LED on FC should blink = good signal
```

#### 2. PRE-ARM CHECKS
```
a) Arm Switch should be DISARMED (D4 = HIGH)
b) LED stays ON = disarmed (safe)
c) Altitude Hold Switch should be OFF (D7 = HIGH)
d) Throttle stick at minimum
```

#### 3. ARMING THE DRONE
```
a) Set Arm Switch to ARMED position (D4 = LOW)
b) LED stops being always-on, now blinks with signals
c) Motors are still off (safe)
d) Keep throttle at minimum
```

#### 4. SMOOTH MOTOR START (RECOMMENDED!)
```
a) Press Smooth Start Button (A7)
b) You'll hear a beep
c) Motors will ramp up slowly: 1000 → 1100 PWM
d) Verify ALL 4 motors spin correctly
e) Check rotation directions:
   - Front Left: CCW
   - Front Right: CW
   - Rear Left: CW
   - Rear Right: CCW
f) If any motor wrong, DISARM immediately and fix!
```

#### 5. TAKEOFF
```
a) Slowly increase throttle (left stick up)
b) Drone should lift smoothly
c) Use right stick for pitch/roll
d) Use left stick left/right for yaw
e) Start low (hover at 1m height)
```

#### 6. FLYING
```
Throttle (A0):  Up = increase thrust
                Down = decrease thrust

Yaw (A1):       Left = rotate CCW
                Right = rotate CW

Pitch (A2):     Up = tilt forward (fly forward)
                Down = tilt back

Roll (A3):      Left = tilt left
                Right = tilt right
```

#### 7. ALTITUDE HOLD MODE (OPTIONAL)
```
a) While hovering at desired height
b) Flip Altitude Hold Switch (D7) to LOW
c) Drone will maintain current altitude
d) Throttle stick centered (1400-1450 range)
e) Small throttle adjustments change altitude slowly
f) Pitch/roll/yaw work normally
g) To disable: flip switch back to HIGH
```

#### 8. LANDING
```
a) Turn OFF altitude hold if active
b) Bring drone to hover
c) Slowly decrease throttle
d) Let it settle gently
e) Throttle to minimum
```

#### 9. DISARM
```
a) Set Arm Switch to DISARMED (D4 = HIGH)
b) Motors stop immediately
c) LED stays ON = disarmed
d) Safe to approach
```

#### 10. POWER DOWN
```
a) Disconnect flight controller battery
b) Power off remote controller
```

---

## 🔔 LED & BUZZER SIGNALS

### Buzzer Signals

| Pattern | Meaning |
|---------|---------|
| Beep-Beep-Beeeep (startup) | Flight controller booting |
| Beep-Beep (short, at startup) | Radio link established ✅ |
| Beep-pause-Beep | Calibration started |
| Beep-Beep (after calibration) | Calibration complete ✅ |
| Single Beep (when arming) | Smooth motor start activated |
| Repeating beep every 2 sec | KILL SWITCH ACTIVE ⚠️ |
| Long beep after signal loss recovery | Signal restored ✅ |

### LED Signals

| LED Behavior | Meaning |
|--------------|---------|
| **Always ON** | Disarmed (safe mode) ✅ |
| **Blinking (fast)** | Armed & receiving RC signals ✅ |
| **OFF** | No RC signal / error ⚠️ |
| **2 blinks during calibration** | Calibration success ✅ |

---

## 🚨 SAFETY FEATURES

### Auto-Disarm Conditions

1. **Signal Loss (3+ seconds)**
   - Motors stop immediately
   - Buzzer repeats every 2 seconds
   - LED off
   - To recover: restore RC signal, wait for beep

2. **Excessive Tilt Angle (>30°)**
   - If drone tilts beyond 30° on X or Y axis
   - Motors stop immediately
   - Prevents crash/flip
   - Requires manual reset

3. **Arm Switch to DISARMED**
   - Immediate motor stop
   - LED stays on

### Manual Kill Switch
- Flip Arm Switch to DISARMED at any time
- Motors stop within 1 control loop (7ms)

---

## 🛠 TROUBLESHOOTING

### Problem: No radio link (no beeps at startup)

**Solutions:**
- Check NRF24L01 wiring (CE, CSN, SPI pins)
- Verify 10µF capacitor on NRF power
- Check 3.3V power to NRF (NOT 5V!)
- Ensure both FC and RC have same `pipe` address
- Try moving antennas apart (interference)
- Re-upload firmware to both boards

---

### Problem: Drone drifts in one direction

**Solutions:**
- Recalibrate on perfectly level surface
- Check if frame is bent
- Verify all motors spin freely
- Check propeller balance
- Adjust PID values (advanced)

---

### Problem: Motors don't start

**Solutions:**
- Check Arm Switch is in ARMED position (D4 = LOW)
- Verify ESC signal wires (D3, D5, D6, D9)
- Ensure ESCs are calibrated (separate procedure)
- Check motor power connections
- Use smooth motor start to test

---

### Problem: One motor doesn't spin

**Solutions:**
- Check ESC signal wire connection
- Verify motor is not damaged
- Check ESC power and ground
- Swap ESC to test if ESC is faulty
- Check Arduino pin (might be damaged)

---

### Problem: Altitude hold doesn't work

**Solutions:**
- Verify MS5611 wiring (I2C on A4/A5)
- Check MS5611 I2C address (0x77 or 0x76)
- Recalibrate (includes barometer)
- Ensure throttle in 1400-1450 range
- Check switch wiring (D7)

---

### Problem: Calibration button doesn't work

**Solutions:**
- Can only calibrate when DISARMED (D4 = HIGH)
- Hold button for full 2 seconds
- Check button wiring to A6
- Verify button pulls pin to GND
- Check Serial Monitor for messages

---

### Problem: LED doesn't blink when armed

**Solutions:**
- Check LED wiring to A3
- Verify RC signals are being received
- Check Serial Monitor for debug info
- Ensure 220Ω resistor in series with LED

---

### Problem: Buzzer doesn't work

**Solutions:**
- Check buzzer polarity (if polarized)
- Verify connection to D8
- Try different buzzer (may be faulty)
- Check Serial Monitor - code might be working

---

## ⚠️ SAFETY GUIDELINES

### ❌ NEVER DO THIS:
1. ❌ Fly near people or animals
2. ❌ Fly indoors without propeller guards
3. ❌ Touch spinning propellers
4. ❌ Fly with low battery
5. ❌ Skip calibration
6. ❌ Fly in rain or strong wind
7. ❌ Use damaged propellers
8. ❌ Arm the drone while holding it

### ✅ ALWAYS DO THIS:
1. ✅ Calibrate before each flight session
2. ✅ Test motor directions before flight
3. ✅ Use smooth motor start to verify function
4. ✅ Keep spare propellers
5. ✅ Fly in open area
6. ✅ Monitor battery voltage
7. ✅ Disarm immediately if anything seems wrong
8. ✅ Remove propellers when testing/debugging

---

## 🔋 BATTERY MANAGEMENT

### Monitoring Voltage
- Voltage divider on A0 monitors battery
- For 3S LiPo (11.1V nominal):
  - Fully charged: 12.6V
  - Storage: 11.4V
  - Low: 10.5V
  - Critical: 9.9V (land immediately!)

### Code to Read Voltage:
Add to `Print()` function:
```cpp
calculate_battery();
Serial.print("Battery: ");
Serial.print(vin);
Serial.println("V");
```

---

## 📊 PID TUNING (ADVANCED)

Default PID values (in code):
```cpp
const float kp = 2;        // Proportional
const float ki = 0.0001;   // Integral
const float kd = 0.5;      // Derivative
```

### If drone oscillates (shakes):
- Decrease `kp` (try 1.5)
- Decrease `kd` (try 0.3)

### If drone responds slowly:
- Increase `kp` (try 2.5)
- Increase `kd` (try 0.7)

### If drone drifts over time:
- Increase `ki` (try 0.0002)
- Recalibrate first!

---

## 📞 SUPPORT & MODIFICATIONS

### Code Structure
- `Drone_Flight_Control.ino` - Main FC code
- `Barometer.ino` - Altitude hold logic
- `Kalman_Filter.ino` - Sensor fusion for altitude
- `Controller.ino` - RC transmitter code
- `Gyro.h` / `Gyro.cpp` - MPU6050 interface

### Debug Mode
Open Serial Monitor (57600 baud) on FC to see:
- Sensor readings
- PID values
- Motor outputs
- System status

Send 'd' character to get detailed debug output.

---

## 🎓 LEARNING RESOURCES

### Understanding the Code
1. **PID Control**: How the drone self-stabilizes
2. **Kalman Filter**: Sensor fusion for accurate altitude
3. **Complementary Filter**: Gyro + Accelerometer fusion
4. **NRF24L01 Protocol**: Wireless communication with ACK

### Recommended Next Steps
1. Add GPS module for position hold
2. Implement return-to-home
3. Add FPV camera
4. Tune PID for your specific frame
5. Add telemetry (battery, altitude to RC)

---

## ✈️ HAPPY FLYING!

**Remember**: Practice in open areas, start slow, and always prioritize safety!

For issues or questions, check Serial Monitor debug output first.

**Good luck with your DIY drone project! 🚁**
