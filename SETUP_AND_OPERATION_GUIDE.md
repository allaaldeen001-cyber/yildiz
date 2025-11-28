# 🚁 DIY Drone Flight Controller - Complete Setup & Operation Guide

## 📋 Table of Contents
1. [System Overview](#system-overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Wiring Diagrams](#wiring-diagrams)
4. [Software Installation](#software-installation)
5. [Initial Setup & Calibration](#initial-setup--calibration)
6. [Flight Operations](#flight-operations)
7. [Control Reference](#control-reference)
8. [Safety Features](#safety-features)
9. [Troubleshooting](#troubleshooting)

---

## 🔧 System Overview

This DIY drone system consists of two main components:
- **Flight Controller (FC)**: Arduino Nano with MPU6050, MS5611, and NRF24L01
- **Remote Controller (RC)**: Arduino Nano with NRF24L01 and dual joysticks

### Key Features
✅ PID-based flight stabilization  
✅ Barometric altitude hold  
✅ Wireless control via NRF24L01  
✅ Comprehensive safety features  
✅ Visual and audio feedback  
✅ EEPROM calibration storage  

---

## 🛠 Hardware Requirements

### Flight Controller Board
| Component | Model | Quantity |
|-----------|-------|----------|
| Microcontroller | Arduino Nano | 1 |
| IMU/Gyroscope | MPU6050 | 1 |
| Barometer | MS5611 | 1 |
| RF Module | NRF24L01 | 1 |
| ESCs | 30A (minimum) | 4 |
| Brushless Motors | 1000-2200 KV | 4 |
| Buzzer | Active Buzzer | 1 |
| LED | 5mm LED + 220Ω resistor | 1 |
| Battery | 3S LiPo (11.1V) | 1 |

### Remote Controller Board
| Component | Model | Quantity |
|-----------|-------|----------|
| Microcontroller | Arduino Nano | 1 |
| RF Module | NRF24L01 | 1 |
| Joysticks | Dual-axis analog | 2 |
| Switches | Toggle switches | 2 |
| Push Buttons | Momentary buttons | 2 |
| Battery | 9V or 2S LiPo | 1 |

---

## 🔌 Wiring Diagrams

### Flight Controller Connections

#### Arduino Nano → NRF24L01
```
NRF24L01    Arduino Nano
--------    ------------
VCC    →    3.3V
GND    →    GND
CE     →    D4
CSN    →    D10
SCK    →    D13
MOSI   →    D11
MISO   →    D12
```

#### Arduino Nano → MPU6050
```
MPU6050     Arduino Nano
-------     ------------
VCC    →    5V
GND    →    GND
SCL    →    A5
SDA    →    A4
```

#### Arduino Nano → MS5611
```
MS5611      Arduino Nano
------      ------------
VCC    →    3.3V or 5V (check module)
GND    →    GND
SCL    →    A5 (shared with MPU6050)
SDA    →    A4 (shared with MPU6050)
```

#### Arduino Nano → Motors (ESCs)
```
Motor Position    ESC Signal Pin
--------------    --------------
Front Left   →    D3
Front Right  →    D5
Rear Right   →    D6
Rear Left    →    D9
```

#### Arduino Nano → Outputs
```
Component    Arduino Pin
---------    -----------
Buzzer  →    D8
LED     →    D7 (via 220Ω resistor)
```

**⚠️ IMPORTANT:** 
- ESCs must have separate power supply (LiPo battery)
- Connect ALL grounds together (Arduino, ESCs, battery)
- NRF24L01 requires stable 3.3V - use capacitor (10µF) if needed

---

### Remote Controller Connections

#### Arduino Nano → NRF24L01
```
NRF24L01    Arduino Nano
--------    ------------
VCC    →    3.3V
GND    →    GND
CE     →    D9
CSN    →    D10
SCK    →    D13
MOSI   →    D11
MISO   →    D12
```

#### Arduino Nano → Joysticks
```
Joystick      Arduino Pin    Function
--------      -----------    --------
Left Y   →    A0             Throttle (Up/Down)
Left X   →    A1             Yaw (Rotate)
Right Y  →    A2             Pitch (Forward/Back)
Right X  →    A3             Roll (Left/Right)
```

#### Arduino Nano → Controls
```
Control         Arduino Pin
-------         -----------
Switch 1   →    D3 (Arm/Disarm)
Switch 2   →    D2 (Altitude Hold)
Button 1   →    D4 (Calibration)
Button 2   →    D5 (Motor Start/Arm)
```

**Note:** All buttons and switches use internal pull-up resistors (no external resistors needed).

---

## 💾 Software Installation

### Required Libraries

Install these libraries via Arduino IDE Library Manager:

1. **Servo** (Built-in)
2. **SPI** (Built-in)
3. **Wire** (Built-in)
4. **EEPROM** (Built-in)
5. **RF24** by TMRh20
6. **Smoothed** by Matthew Fryer
7. **MS5611** by Rob Tillaart

### Installation Steps

1. Open Arduino IDE
2. Go to **Tools → Manage Libraries**
3. Search and install:
   - `RF24`
   - `Smoothed`
   - `MS5611`

### Upload Code

#### Flight Controller
1. Open `Drone_Flight_Control.ino`
2. Ensure all 3 files are in same folder:
   - `Drone_Flight_Control.ino`
   - `Barometer.ino`
   - `Kalman_Filter.ino`
3. Also include `Gyro.h` and `Gyro.cpp` in the same folder
4. Select **Board:** Arduino Nano
5. Select **Processor:** ATmega328P (Old Bootloader) - if using clone
6. Upload to Flight Controller Arduino

#### Remote Controller
1. Open `Controller.ino`
2. Select **Board:** Arduino Nano
3. Select **Processor:** ATmega328P (Old Bootloader) - if using clone
4. Upload to Remote Controller Arduino

---

## 🎯 Initial Setup & Calibration

### Step-by-Step First Setup

#### 1. **Power Up Sequence**

```
1. Power ON Remote Controller first
2. Wait for Serial message: "READY TO TRANSMIT"
3. Power ON Flight Controller
4. Listen for startup beeps (3 tones)
5. Watch for "RC CONNECTED" message
```

**Expected Behavior:**
- RC powers up, NRF initializes
- FC powers up with 3 beeps
- Within 1-2 seconds: connection beep on FC
- LED on FC stays solid (disarmed state)

---

#### 2. **Initial Calibration (MANDATORY)**

**Prerequisites:**
- Drone must be on flat, stable surface
- Switch 1 (Arm) must be OFF (position = 1 = Disarmed)
- Keep drone completely still during calibration

**Calibration Procedure:**

```
1. Ensure Switch 1 is in DISARM position (1)
2. Press and HOLD Button 1 (Calibration) for 3 seconds
3. Release button when you hear the first beep
4. Keep drone PERFECTLY STILL for ~10 seconds
5. You'll hear:
   - Short beep (starting MPU6050 calibration)
   - Long beep (starting MS5611 calibration)
   - Two quick beeps (calibration complete)
```

**Serial Monitor Output:**
```
=== STARTING FULL CALIBRATION ===
Calibrating MPU6050... Keep drone stable!
MPU6050 Cal: X=-2.34 Y=1.87
Calibrating MS5611 barometer...
Ground Pressure: 101325.5
=== CALIBRATION COMPLETE ===
```

**⚠️ Calibration Troubleshooting:**
- If drone drifts during flight → Recalibrate on level surface
- Calibrate in same location where you'll fly
- Recalibrate if you move to significantly different altitude

---

#### 3. **Motor Direction Check**

Before first flight, verify all motors spin in correct direction:

```
Motor Layout (viewed from top):
    FRONT
  FL    FR
    \ /
    / \
  RL    RR
    REAR

Rotation Direction:
  FL → CCW (Counter-clockwise)
  FR → CW  (Clockwise)
  RL → CW  (Clockwise)
  RR → CCW (Counter-clockwise)
```

**Motor Test Procedure:**

```
1. Remove propellers (IMPORTANT!)
2. Ensure Switch 1 is DISARM (1)
3. Long-press Button 2 for 3 seconds to ARM
4. Press Button 2 briefly for Smooth Motor Start
5. Motors will ramp up slowly, hold for 2 seconds, then ramp down
6. Verify each motor spins correctly
```

**If motor direction is wrong:**
- Swap any two of the three motor wires

---

## 🎮 Flight Operations

### Pre-Flight Checklist

- [ ] Battery fully charged (>11.4V for 3S)
- [ ] All propellers securely attached (correct rotation!)
- [ ] RC battery charged
- [ ] Clear flight area (minimum 5m radius)
- [ ] Calibration completed recently
- [ ] Switch 1 in DISARM position
- [ ] Throttle stick at minimum

---

### Arming Procedure

**Method 1: Using Button 2 (Recommended)**
```
1. Set Switch 1 to DISARM position (1)
2. Press and HOLD Button 2 for 3 seconds
3. Release when you hear beep and LED blinks 3 times
4. Drone is now ARMED
5. LED will blink when receiving commands
```

**Method 2: Using Switch 1 (Direct)**
```
1. Move Switch 1 to ARM position (0)
2. Drone arms immediately
3. BE READY - motors will respond to throttle!
```

**⚠️ ARMED STATE INDICATORS:**
- LED blinks when receiving RC data
- Motors at idle speed (1050 μs)
- Responsive to joystick commands

---

### First Flight

1. **Arm the drone** (see above)
2. **Slowly increase throttle** until drone is light on ground
3. **Continue increasing** until drone lifts off (~6 inches)
4. **Hover in place** - practice throttle control
5. **Small stick movements** - test roll, pitch, yaw
6. **Land gently** by reducing throttle gradually
7. **Disarm** using Switch 1

**First Flight Tips:**
- Start on calm day (no wind)
- Practice in open space
- Have assistant ready to disarm if needed
- Expect to trim using calibration if drift occurs

---

### Altitude Hold Mode

**Activation:**
```
1. Fly drone to desired altitude
2. Set throttle to middle position (1400-1450 range)
3. Activate Switch 2 (Altitude Hold)
4. Drone will maintain current altitude
```

**Using Altitude Hold:**
- **Throttle centered (1425):** Hold altitude
- **Throttle up (>1450):** Climb slowly
- **Throttle down (<1400):** Descend slowly
- **Roll/Pitch/Yaw:** Work normally

**Deactivate:** Turn off Switch 2

**Note:** Altitude hold works best above 2 meters height

---

### Disarming

**Quick Disarm (Emergency):**
- Flip Switch 1 to DISARM position (1)
- Motors stop immediately

**Normal Landing Disarm:**
- Land drone gently
- Reduce throttle to minimum
- Flip Switch 1 to DISARM
- LED turns solid ON

---

## 🕹 Control Reference

### Joystick Functions

| Stick | Direction | Function | Effect |
|-------|-----------|----------|--------|
| **Left Y** | UP | Increase Throttle | Ascend |
| **Left Y** | DOWN | Decrease Throttle | Descend |
| **Left X** | LEFT | Yaw Left | Rotate CCW |
| **Left X** | RIGHT | Yaw Right | Rotate CW |
| **Right Y** | UP | Pitch Forward | Fly Forward |
| **Right Y** | DOWN | Pitch Back | Fly Backward |
| **Right X** | LEFT | Roll Left | Fly Left |
| **Right X** | RIGHT | Roll Right | Fly Right |

---

### Button & Switch Functions

| Control | Position/Action | Function | When Active |
|---------|----------------|----------|-------------|
| **Switch 1** | 0 (Down) | ARMED | Anytime |
| **Switch 1** | 1 (Up) | DISARMED | Anytime |
| **Switch 2** | 0 (Down) | Altitude Hold ON | When armed + throttle mid |
| **Switch 2** | 1 (Up) | Altitude Hold OFF | Anytime |
| **Button 1** | Hold 2s | Full Calibration | When DISARMED only |
| **Button 2** | Hold 3s | ARM Drone | When DISARMED only |
| **Button 2** | Press briefly | Smooth Motor Test | When ARMED only |

---

## 🛡 Safety Features

### Automatic Safety Systems

#### 1. **Maximum Tilt Angle Limit (30°)**
- If drone tilts beyond 30° on any axis
- Motors cut immediately (Kill Switch activated)
- Prevents flip-over crashes
- **Recovery:** Right drone, wait 2 seconds for beep, then can re-arm

#### 2. **Communication Loss Protection**
- If RC signal lost for 3 seconds
- Automatic motor cutoff
- Buzzer sounds alarm (1000 Hz)
- **Recovery:** Restore RC connection, wait for beep

#### 3. **Arming Interlocks**
- Cannot calibrate while armed
- Cannot smooth-start while disarmed
- Prevents accidental motor activation

#### 4. **LED Status Indicators**
- **Solid ON:** Disarmed (safe state)
- **Blinking:** Armed and receiving data
- **Fast Blink:** Error/Kill Switch

#### 5. **Buzzer Alerts**
| Beep Pattern | Meaning |
|--------------|---------|
| 3 rising tones | Startup successful |
| Single 2000 Hz | RC connected |
| Repeating 1000 Hz | Kill switch active |
| 3 quick beeps | Calibration complete |
| 1500 Hz | Armed/Disarmed |

---

## 🔍 Troubleshooting

### Communication Issues

#### **Problem:** FC never gets "RC CONNECTED"

**Solutions:**
1. Check NRF24L01 wiring (CE, CSN, power)
2. Add 10µF capacitor across NRF power pins
3. Verify both use channel 108
4. Ensure pipe address matches: `0xF0F0F0F0E1LL`
5. Try different NRF24L01 modules (common failure)

#### **Problem:** Connection drops frequently

**Solutions:**
1. Add capacitor to NRF VCC/GND
2. Keep NRF modules away from motors/ESCs
3. Shorten antenna distance if too far (>50m)
4. Check for interference (WiFi routers, etc.)

---

### Calibration Issues

#### **Problem:** Drone drifts in one direction

**Solutions:**
1. Recalibrate on FLAT surface
2. Ensure no vibration during calibration
3. Check motor mounting (loose screws cause vibration)
4. Verify propellers balanced

#### **Problem:** Calibration doesn't save

**Solutions:**
1. Wait for completion beeps
2. Don't power off immediately after calibration
3. Check EEPROM library installed
4. Try manual EEPROM reset (re-upload code)

---

### Flight Issues

#### **Problem:** Motors don't spin when armed

**Solutions:**
1. Check ESC wiring (signal pins correct?)
2. Verify ESCs calibrated (separate ESC calibration procedure)
3. Ensure battery voltage >10.5V
4. Test with smooth motor start function

#### **Problem:** Drone flips immediately on takeoff

**Solutions:**
1. **CHECK MOTOR DIRECTIONS FIRST** (most common!)
2. Verify motor/ESC mapping (FL, FR, RL, RR)
3. Check propeller installation (CW vs CCW props)
4. Recalibrate on level surface

#### **Problem:** Altitude hold doesn't work

**Solutions:**
1. Throttle must be 1400-1450 range
2. Fly above 2 meters altitude
3. Verify MS5611 initialized (check Serial)
4. Recalibrate MS5611 ground pressure

#### **Problem:** Uncontrollable yaw drift

**Solutions:**
1. Normal if compass not used (slight drift expected)
2. Press Button 2 briefly to reset yaw to zero
3. Adjust `sensiZ` parameter if too sensitive

---

### Hardware Issues

#### **Problem:** ESCs beeping continuously

**Solutions:**
1. ESCs not calibrated - run ESC calibration:
   ```
   1. Disconnect signal wires from Arduino
   2. Power Arduino via USB
   3. Connect all ESC signals together
   4. Upload ESC calibration sketch
   5. Follow prompts
   ```

#### **Problem:** Buzzer too loud/annoying

**Solutions:**
1. Add resistor in series (100-470Ω)
2. Comment out non-critical beeps in code
3. Replace with quieter buzzer

---

### Emergency Procedures

#### **Kill Switch Activated**
1. **Don't panic** - motors cut for safety
2. Right drone if tilted
3. Wait 2 seconds for beeps to stop
4. Check what triggered it (tilt? signal loss?)
5. Fix issue
6. Re-arm when ready

#### **Flyaway/Loss of Control**
1. Flip Switch 1 to DISARM immediately
2. If no response, cut RC power (forces signal loss)
3. Drone will kill switch after 3 seconds

#### **Crash Landing**
1. Disarm immediately (Switch 1)
2. Disconnect battery
3. Check for damage:
   - Broken propellers
   - Loose wires
   - Bent motor shafts
4. Test motors without props before next flight

---

## 📊 Advanced Configuration

### PID Tuning

**Default Values (in code):**
```cpp
kp = 2.0     // Proportional
ki = 0.0001  // Integral
kd = 0.5     // Derivative
kpZ = 2.0    // Yaw
```

**Tuning Process:**
1. Start with defaults
2. If oscillations: **reduce kp**
3. If sluggish: **increase kp**
4. If drift: **increase ki** (small increments!)
5. If twitchy: **increase kd**

### Altitude PID Tuning

```cpp
pid_p_gain_altitude = 14.0
pid_i_gain_altitude = 2.0
pid_d_gain_altitude = 7.5
pid_max_altitude = 400
```

---

## 📝 Specifications

| Parameter | Value |
|-----------|-------|
| Loop Frequency | 140 Hz |
| Communication Rate | 250 kbps |
| Max Tilt Angle | 30° |
| Max Thrust | 1700 µs (adjustable) |
| Armed Idle | 1050 µs |
| Altitude Hold Range | 1400-1450 µs |
| Signal Loss Timeout | 3 seconds |

---

## 🆘 Support & Contact

### Serial Monitor Debug

Enable debugging mode:
```cpp
debugging(true);  // In setup()
```

Baud rate: **57600**

### Common Serial Messages

| Message | Meaning |
|---------|---------|
| `*** RC CONNECTED ***` | Communication established |
| `*** ARMED ***` | Motors armed |
| `*** DISARMED ***` | Motors disarmed |
| `KILL SWITCH: ...` | Emergency stop triggered |
| `Motor test complete` | Smooth start finished |

---

## ⚖️ Legal & Safety Notice

**WARNING:** This is an experimental DIY project.

- Fly in open areas away from people
- Follow local drone regulations
- Never fly near airports
- Respect privacy
- You are responsible for safe operation
- Test thoroughly before adding camera/payload
- Use at your own risk

**Recommended Safety Equipment:**
- Safety goggles
- Propeller guards
- Fire-safe LiPo charging bag
- Voltage alarm for battery

---

## 📚 Additional Resources

### Recommended Reading
- PID Control Theory
- LiPo Battery Safety
- ESC Calibration Guides
- Arduino Programming Basics

### Useful Tools
- LiPo voltage checker
- Propeller balancer
- Vibration isolation mounts
- Heat shrink tubing

---

**Version:** 1.0  
**Last Updated:** November 2025  
**Tested On:** Arduino Nano (ATmega328P)

**Happy Flying! 🚁**
