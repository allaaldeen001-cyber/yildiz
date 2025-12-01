# 🚁 Quadcopter Drone Project - Arduino Nano

**Professional flight controller with Betaflight-style PID control, altitude hold, and autonomous takeoff/landing**

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Wiring Diagrams](#wiring-diagrams)
- [Software Installation](#software-installation)
- [Calibration & Setup](#calibration--setup)
- [Flight Modes](#flight-modes)
- [Control Mapping](#control-mapping)
- [Safety Features](#safety-features)
- [PID Tuning](#pid-tuning)
- [Troubleshooting](#troubleshooting)

---

## 🎯 Overview

This is a **complete quadcopter system** featuring:

- **Dual Arduino Nano Setup**: Separate Flight Controller and Remote Controller
- **Betaflight-Style PID**: Professional cascaded PID control (used by 95% of racing drones)
- **250Hz Control Loop**: Ultra-responsive flight characteristics
- **Altitude Hold**: Maintains height automatically using MS5611 barometer
- **Autonomous Takeoff/Landing**: One-button operation
- **5 Flight Modes**: ANGLE, ACRO, ALTITUDE HOLD, TAKEOFF, LANDING
- **Wireless Control**: 2.4GHz nRF24L01+ radio (100-1000m range)
- **Sensor Fusion**: Complementary filter combining gyro and accelerometer data

---

## ✨ Features

### **Stabilization**
- ✅ Betaflight-style cascaded PID (Outer: Angle, Inner: Rate)
- ✅ Complementary filter (98% gyro, 2% accel)
- ✅ 250Hz gyro rate control
- ✅ Motor mixing for X-configuration

### **Altitude Control**
- ✅ MS5611 barometer (10cm resolution)
- ✅ Ground level calibration
- ✅ Altitude Hold mode with PID
- ✅ Vertical velocity damping

### **Flight Modes**
- ✅ **ANGLE**: Auto-level, beginner-friendly
- ✅ **ACRO**: Rate control, advanced aerobatics
- ✅ **ALTITUDE HOLD**: Maintains height automatically
- ✅ **SMOOTH TAKEOFF**: Auto ARM + rise to 1.5m
- ✅ **SMOOTH LANDING**: Auto descent + safe touchdown

### **Safety**
- ✅ Failsafe (auto-disarm on signal loss after 1 second)
- ✅ Throttle limits and motor constraints
- ✅ Calibration checks on startup
- ✅ Audio/visual feedback (buzzer + LED)

---

## 🛠 Hardware Requirements

### **Flight Controller (Main Quadcopter)**

| Component | Model | Pins | Purpose |
|-----------|-------|------|---------|
| **Microcontroller** | Arduino Nano | - | Main brain |
| **Gyro/Accel** | MPU6050 | SDA, SCL, INT→D2 | Measures tilt & rotation |
| **Barometer** | MS5611 | SDA, SCL | Measures altitude |
| **Radio RX** | NRF24L01+ | CE→D4, CSN→D10 | Receives commands |
| **Motors** | 4x Brushless + ESC | D3, D5, D6, D9 | Thrust control |
| **Buzzer** | Passive Buzzer | D8 | Audio alerts |
| **LED** | Standard LED | D7 | Status indicator |
| **Battery** | 3S LiPo (11.1V) | - | Power source |

### **Remote Controller (Handheld)**

| Component | Pins | Purpose |
|-----------|------|---------|
| **Microcontroller** | Arduino Nano | Main controller |
| **Radio TX** | NRF24L01+ (CE→D9, CSN→D10) | Transmits commands |
| **Left Joystick** | A0 (Throttle), A1 (Yaw) | Altitude & rotation |
| **Right Joystick** | A2 (Pitch), A3 (Roll) | Forward/back, left/right |
| **Buttons** | D4, D5, D6, D7 | Calibrate, Test, Land, Takeoff |
| **Switches** | D2, D3 | Altitude Hold, ANGLE/ACRO |
| **Battery** | 9V or 3x AA | Power source |

---

## 🔌 Wiring Diagrams

### **Flight Controller Wiring**

```
╔═══════════════════════════════════════════════════════════════╗
║                    FLIGHT CONTROLLER                          ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  ┌─────────────┐                                              ║
║  │ ARDUINO NANO│                                              ║
║  │             │                                              ║
║  │ D3  ────────┼────► Front-Left Motor (ESC Signal)          ║
║  │ D5  ────────┼────► Front-Right Motor (ESC Signal)         ║
║  │ D6  ────────┼────► Rear-Right Motor (ESC Signal)          ║
║  │ D9  ────────┼────► Rear-Left Motor (ESC Signal)           ║
║  │             │                                              ║
║  │ D7  ────────┼────► LED (+ 220Ω resistor to GND)           ║
║  │ D8  ────────┼────► Buzzer (+ to D8, - to GND)             ║
║  │             │                                              ║
║  │ D4  ────────┼────► NRF24L01 CE                            ║
║  │ D10 ────────┼────► NRF24L01 CSN                           ║
║  │ D11 ────────┼────► NRF24L01 MOSI                          ║
║  │ D12 ────────┼────► NRF24L01 MISO                          ║
║  │ D13 ────────┼────► NRF24L01 SCK                           ║
║  │             │                                              ║
║  │ A4  ────────┼────► MPU6050 SDA (+ MS5611 SDA)             ║
║  │ A5  ────────┼────► MPU6050 SCL (+ MS5611 SCL)             ║
║  │ D2  ────────┼────► MPU6050 INT (optional)                 ║
║  │             │                                              ║
║  │ 5V  ────────┼────► NRF24L01 VCC, MPU6050 VCC, MS5611 VCC  ║
║  │ GND ────────┼────► All GND connections                    ║
║  │ VIN ────────┼────► 11.1V from LiPo (or 7-12V regulated)   ║
║  └─────────────┘                                              ║
║                                                               ║
╠═══════════════════════════════════════════════════════════════╣
║  MOTOR CONFIGURATION (X-Pattern):                             ║
║                                                               ║
║          FRONT                                                ║
║         FL   FR      FL = Front-Left (D3)                     ║
║          \ X /       FR = Front-Right (D5)                    ║
║           X          RR = Rear-Right (D6)                     ║
║          / X \       RL = Rear-Left (D9)                      ║
║         RL   RR                                               ║
║                                                               ║
║  Motor Rotation:                                              ║
║   FL: CCW  FR: CW                                             ║
║   RL: CW   RR: CCW                                            ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
```

### **Remote Controller Wiring**

```
╔═══════════════════════════════════════════════════════════════╗
║                    REMOTE CONTROLLER                          ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║  ┌─────────────┐                                              ║
║  │ ARDUINO NANO│                                              ║
║  │             │                                              ║
║  │ A0  ────────┼────► Left Joystick Y (Throttle)             ║
║  │ A1  ────────┼────► Left Joystick X (Yaw)                  ║
║  │ A2  ────────┼────► Right Joystick Y (Pitch)               ║
║  │ A3  ────────┼────► Right Joystick X (Roll)                ║
║  │             │                                              ║
║  │ D4  ────────┼────► Button 1 (Calibrate) + pullup          ║
║  │ D5  ────────┼────► Button 2 (Motor Test) + pullup         ║
║  │ D6  ────────┼────► Button 3 (Landing) + pullup            ║
║  │ D7  ────────┼────► Button 4 (Takeoff) + pullup            ║
║  │             │                                              ║
║  │ D2  ────────┼────► Switch 1 (Altitude Hold) + pullup      ║
║  │ D3  ────────┼────► Switch 2 (ANGLE/ACRO) + pullup         ║
║  │             │                                              ║
║  │ D9  ────────┼────► NRF24L01 CE                            ║
║  │ D10 ────────┼────► NRF24L01 CSN                           ║
║  │ D11 ────────┼────► NRF24L01 MOSI                          ║
║  │ D12 ────────┼────► NRF24L01 MISO                          ║
║  │ D13 ────────┼────► NRF24L01 SCK                           ║
║  │             │                                              ║
║  │ 5V  ────────┼────► NRF24L01 VCC, Joystick VCC             ║
║  │ GND ────────┼────► All GND connections                    ║
║  │ VIN ────────┼────► 9V Battery or 3x AA                    ║
║  └─────────────┘                                              ║
║                                                               ║
╠═══════════════════════════════════════════════════════════════╣
║  JOYSTICK WIRING:                                             ║
║                                                               ║
║  Left Joystick:         Right Joystick:                       ║
║   • VCC → 5V             • VCC → 5V                           ║
║   • GND → GND            • GND → GND                          ║
║   • VRx → A1 (Yaw)       • VRx → A3 (Roll)                    ║
║   • VRy → A0 (Throttle)  • VRy → A2 (Pitch)                   ║
║                                                               ║
║  BUTTON WIRING (all buttons use internal pullup):            ║
║   • One side → Digital Pin                                    ║
║   • Other side → GND                                          ║
║   • Active LOW (pressed = 0, released = 1)                    ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
```

### **NRF24L01+ Pinout (Both Controllers)**

```
    ┌─────────────┐
    │   NRF24L01+ │
    │             │
    │  1 GND      │ → GND
    │  2 VCC      │ → 3.3V (use capacitor!)
    │  3 CE       │ → D4 (FC) / D9 (RC)
    │  4 CSN      │ → D10 (both)
    │  5 SCK      │ → D13 (both)
    │  6 MOSI     │ → D11 (both)
    │  7 MISO     │ → D12 (both)
    │  8 IRQ      │ → Not connected
    └─────────────┘

⚠️  IMPORTANT: Add 10µF capacitor between VCC and GND!
    NRF24L01+ draws high current bursts during transmission.
```

---

## 💻 Software Installation

### **Required Libraries**

Install these libraries via Arduino IDE Library Manager:

1. **Adafruit MPU6050** (by Adafruit)
2. **Adafruit Unified Sensor** (by Adafruit)
3. **MS5611** (by Rob Tillaart)
4. **RF24** (by TMRh20)

### **Installation Steps**

1. **Clone or Download** this repository
2. **Open Arduino IDE** (version 1.8.19 or newer)
3. **Install libraries**:
   - Go to `Sketch → Include Library → Manage Libraries`
   - Search and install each library listed above
4. **Upload Flight Controller**:
   - Open `FlightController/FlightController.ino`
   - Select board: `Tools → Board → Arduino Nano`
   - Select processor: `Tools → Processor → ATmega328P (Old Bootloader)` *
   - Upload to FC Arduino
5. **Upload Remote Controller**:
   - Open `RemoteController/RemoteController.ino`
   - Upload to RC Arduino

**Note**: If upload fails, try "ATmega328P" without "Old Bootloader"

---

## ⚙️ Calibration & Setup

### **Initial Setup Sequence**

1. **Power on Remote Controller first**
   - Center both joysticks during startup
   - Wait for "REMOTE CONTROLLER READY!" message
   - Check Serial Monitor (115200 baud) for status

2. **Power on Flight Controller**
   - Place drone on flat, level surface
   - **Keep it completely still** during calibration
   - Wait for 2 beeps (ready signal)
   - Check Serial Monitor for:
     ```
     ✅ Motors initialized
     ✅ Radio initialized
     ✅ MPU6050 initialized
     ✅ MS5611 barometer initialized
     ✅ SYSTEM READY!
     ```

3. **Manual Recalibration (if needed)**
   - Press **Button 1** on RC to recalibrate sensors
   - Drone must be on flat surface and still
   - Wait for 2 beeps when complete

---

## 🎮 Flight Modes

### **1. ANGLE Mode (Default)** 🟢 *Recommended for Beginners*

**How to activate**: SW2 = ON, SW1 = OFF

**Behavior**:
- Auto-levels when sticks centered
- Maximum tilt: ±25°
- Prevents flipping
- Best for stable flight and learning

**Use case**: First flights, photography, stable hovering

---

### **2. ACRO Mode** 🔴 *Advanced Users Only*

**How to activate**: SW2 = OFF, SW1 = OFF

**Behavior**:
- No auto-leveling
- Direct rate control (up to 300°/s)
- Can flip and roll
- Requires skill to control

**Use case**: Aerobatics, racing, advanced maneuvers

---

### **3. ALTITUDE HOLD Mode** 🔵

**How to activate**: SW1 = ON (overrides SW2)

**Behavior**:
- Maintains current altitude automatically
- Throttle stick adjusts target altitude (±10cm/s)
- Auto-levels like ANGLE mode
- Compensates for wind

**Use case**: Easy flying, aerial photography, hands-free hovering

**Example**:
```
1. Enable SW1 at 150cm altitude
2. Drone locks altitude at 150cm
3. Move throttle up → climbs slowly
4. Center throttle → holds new altitude
5. Wind gust → auto-compensates
```

---

### **4. TAKEOFF Mode** 🚀

**How to activate**: Press Button 4 (D7)

**Automatic sequence**:
1. Arms motors
2. Spins up smoothly
3. Rises at 80cm/s
4. Reaches 150cm (1.5m)
5. Transitions to ALTITUDE HOLD
6. Hovers hands-free!

**Duration**: ~2 seconds

---

### **5. LANDING Mode** 🛬

**How to activate**: Press Button 3 (D6)

**Automatic sequence**:
1. Descends at 50cm/s (gentle)
2. Altitude decreases smoothly
3. Reaches 10cm → disarms
4. 3 beeps (safe landing)

**Duration**: ~3 seconds

---

## 🕹 Control Mapping

### **Left Joystick**

| Axis | Control | Range | Function |
|------|---------|-------|----------|
| **Y (A0)** | THROTTLE | 0-1000 | Altitude (up/down) |
| **X (A1)** | YAW | ±200°/s | Rotation (spin left/right) |

### **Right Joystick**

| Axis | Control | Range | Function |
|------|---------|-------|----------|
| **Y (A2)** | PITCH | ±25° | Movement (forward/back) |
| **X (A3)** | ROLL | ±25° | Movement (left/right strafe) |

### **Buttons**

| Button | Pin | Function | Usage |
|--------|-----|----------|-------|
| **Button 1** | D4 | **Calibrate** | Recalibrate gyro + barometer |
| **Button 2** | D5 | **Motor Test** | Briefly spin motors (disarmed only) |
| **Button 3** | D6 | **Landing** | Auto descent + safe touchdown |
| **Button 4** | D7 | **Takeoff** | Auto ARM + rise to 1.5m |

### **Switches**

| Switch | Pin | Position | Mode |
|--------|-----|----------|------|
| **SW1** | D2 | OFF | Manual throttle |
| **SW1** | D2 | ON | **Altitude Hold** (auto-maintain height) |
| **SW2** | D3 | OFF | **ACRO** mode (rate control) |
| **SW2** | D3 | ON | **ANGLE** mode (auto-level) |

---

## 🛡 Safety Features

### **Failsafe**
- **Trigger**: Radio signal lost for >1 second
- **Action**: Auto-disarms motors immediately
- **Indicator**: Rapid beeps, "FAILSAFE" on Serial

### **Throttle Limits**
- Minimum: 1000µs (motors off)
- Maximum: 2000µs (full throttle)
- Armed throttle: >1050µs to spin

### **Motor Constraints**
- Individual motor speed limited to 1000-2000µs
- Prevents over-speeding
- Ensures stability

### **Audio Alerts**
- **1 beep**: Armed
- **2 beeps**: Calibration complete / Ready
- **3 beeps**: Safe landing complete
- **5 rapid beeps**: Error or failsafe

### **LED Status**
- **Solid**: System ready
- **Blinking**: Initializing
- **Off**: Not powered or error

---

## 🎛 PID Tuning

### **What is PID?**

PID (Proportional-Integral-Derivative) is the algorithm that keeps your drone stable. Think of it like a driver correcting steering:
- **P** (Proportional): How hard to correct (stiffness)
- **I** (Integral): Corrects drift over time (eliminates wind drift)
- **D** (Derivative): Dampens oscillations (smoothness)

### **Default PID Values** (Tuned for 250mm frame, 1500Kv motors)

**RATE PID (Inner Loop - Gyro Response)**

```cpp
// Roll & Pitch Rate
pidRateRoll.Kp = 0.8;
pidRateRoll.Ki = 0.4;
pidRateRoll.Kd = 0.015;

pidRatePitch.Kp = 0.8;
pidRatePitch.Ki = 0.4;
pidRatePitch.Kd = 0.015;

// Yaw Rate
pidRateYaw.Kp = 0.8;
pidRateYaw.Ki = 0.4;
pidRateYaw.Kd = 0.015;
```

**ANGLE PID (Outer Loop - Attitude Control)**

```cpp
pidAngleRoll.Kp = 3.5;
pidAngleRoll.Ki = 0.0;  // Usually 0
pidAngleRoll.Kd = 0.0;  // Usually 0

pidAnglePitch.Kp = 3.5;
pidAnglePitch.Ki = 0.0;
pidAnglePitch.Kd = 0.0;
```

**ALTITUDE PID**

```cpp
pidAltitude.Kp = 5.0;
pidAltitude.Ki = 0.2;
pidAltitude.Kd = 3.0;  // Velocity damping
```

### **How to Tune**

⚠️ **WARNING**: Always tune with propellers OFF first!

**Step 1: Rate PID (Most Important)**

1. **Set all to zero** except `Kp = 0.5`
2. **Increase P** until drone responds quickly but oscillates slightly
3. **Add D** (`Kd = 0.01`) to dampen oscillations
4. **Add I** (`Ki = 0.2`) to eliminate drift
5. **Fine-tune** until smooth and responsive

**Step 2: Angle PID**

1. Start with `Kp = 3.0`
2. Increase until drone returns to level quickly
3. Too high → oscillates, Too low → slow response

**Step 3: Altitude PID**

1. Hover in ALTITUDE HOLD mode
2. If bounces → reduce `Kp` or increase `Kd`
3. If drifts → increase `Ki`
4. If slow response → increase `Kp`

### **Tuning Symptoms**

| Problem | Solution |
|---------|----------|
| Oscillates rapidly | Reduce `Kp` or increase `Kd` |
| Drifts slowly | Increase `Ki` |
| Slow response | Increase `Kp` |
| Overshoots target | Increase `Kd` |
| Won't hover in place | Increase `Ki` |

---

## 🔧 Troubleshooting

### **Motors don't spin**
- Check ESC calibration (send 2000µs, then 1000µs)
- Verify motor connections (D3, D5, D6, D9)
- Ensure throttle >50 and armed

### **Drone flips on takeoff**
- Wrong motor rotation direction (FL/RR = CCW, FR/RL = CW)
- Wrong motor position (check X-configuration)
- Recalibrate gyro (Button 1)

### **Radio signal lost**
- Check nRF24L01+ power (use 10µF capacitor!)
- Verify both use same address (0xF0F0F0F0E1LL)
- Check antenna orientation

### **Drifts in ANGLE mode**
- Recalibrate gyro on flat surface
- Adjust complementary filter (98% gyro / 2% accel)
- Check `Ki` term (eliminate steady-state error)

### **Altitude bounces up/down**
- Reduce `Kp` in altitude PID
- Increase `Kd` for damping
- Check MS5611 connection

### **Won't ARM**
- Check throttle is <100
- Verify radio connection
- Check Serial Monitor for errors

---

## 📊 Performance Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Loop Rate** | 250Hz (4ms) | Betaflight standard for Arduino |
| **Attitude Update** | 250Hz | MPU6050 reading rate |
| **Altitude Update** | 50Hz | MS5611 reading rate |
| **Radio Update** | 250Hz | Command transmission |
| **Sensor Fusion** | Complementary Filter | 98% gyro, 2% accel |
| **Altitude Resolution** | 10cm | MS5611 precision |
| **Angle Precision** | 0.1° | MPU6050 + filter |
| **Max Tilt (ANGLE)** | ±50° | Configurable |
| **Max Rate (ACRO)** | ±500°/s | Configurable |
| **Takeoff Speed** | 80cm/s | Smooth ascent |
| **Landing Speed** | 50cm/s | Gentle descent |
| **Takeoff Height** | 150cm (1.5m) | Configurable |
| **Max Altitude** | 500cm (5m) | Software limited |
| **Radio Range** | 100-1000m | Depends on nRF24 version |
| **Failsafe Timeout** | 1 second | Auto-disarm on signal loss |

---

## 📝 Flight Sequence Example

### **Typical Flight**

```
PHASE 1: STARTUP (5 seconds)
──────────────────────────────────
1. Power on RC, center sticks
2. Power on FC, keep still
3. Wait for 2 beeps (ready!)

PHASE 2: SMOOTH TAKEOFF (2 seconds)
──────────────────────────────────
4. Place drone on flat ground
5. Press Button 4
6. Drone arms, spins up, rises to 1.5m
7. Transitions to ALTITUDE HOLD
8. Hovering hands-free!

PHASE 3: FLYING
──────────────────────────────────
9. Right stick right → Rolls right, moves right
10. Right stick forward → Pitches forward, moves forward
11. Left stick right → Yaws right, rotates clockwise
12. Throttle up/down → Adjusts altitude slowly
13. Sticks centered → Hovers in place

PHASE 4: SMOOTH LANDING (3 seconds)
──────────────────────────────────
14. Press Button 3
15. Descends gently at 50cm/s
16. Reaches 10cm → auto-disarms
17. 3 beeps (safe landing!)
```

---

## 📚 Additional Resources

- **Betaflight Documentation**: [betaflight.com](https://betaflight.com)
- **PID Tuning Guide**: See `docs/PID_TUNING.md`
- **Flight Modes Guide**: See `docs/FLIGHT_MODES.md`
- **Wiring Photos**: See `docs/wiring/`

---

## 🤝 Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request

---

## ⚠️ Disclaimer

**SAFETY WARNING**: Quadcopters are potentially dangerous. Always:
- Fly in open areas away from people
- Remove propellers during testing
- Wear safety glasses
- Follow local drone regulations
- Never fly near airports
- Inspect hardware before each flight

**The author is not responsible for any damage, injury, or loss caused by this project.**

---

## 📄 License

MIT License - See LICENSE file for details

---

## 🙏 Acknowledgments

- Betaflight team for PID algorithms
- Adafruit for sensor libraries
- Arduino community for support

---

**Happy Flying! 🚁**
