# Professional Arduino Drone Flight Controller & RC System

## Complete Engineering Firmware Design

A production-quality quadcopter flight control system designed for Arduino Nano, featuring professional cascade PID control, sensor fusion, and reliable NRF24L01 communication.

![Drone System Architecture](docs/images/architecture.png)

---

## 📋 Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Pin Mappings](#pin-mappings)
4. [Features](#features)
5. [Installation](#installation)
6. [Operation Guide](#operation-guide)
7. [Control Architecture](#control-architecture)
8. [Safety Features](#safety-features)
9. [Tuning Guide](#tuning-guide)
10. [Testing Procedures](#testing-procedures)
11. [Troubleshooting](#troubleshooting)

---

## 🎯 System Overview

This firmware implements a complete drone control system consisting of:

- **Flight Controller (FC)**: Arduino Nano-based stabilization system with AHRS, cascade PID, and altitude hold
- **Remote Controller (RC)**: Arduino Nano-based transmitter with 100Hz update rate and telemetry display

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                        REMOTE CONTROLLER                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────────┐ │
│  │ Joystick │  │ Joystick │  │ Buttons  │  │    Toggle Switches   │ │
│  │   Left   │  │  Right   │  │  1 & 2   │  │   ARM    ALT HOLD    │ │
│  │ Thr/Yaw  │  │ Pit/Roll │  │          │  │                      │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └──────────┬───────────┘ │
│       │             │             │                    │             │
│       └─────────────┴─────────────┴────────────────────┘             │
│                              │                                       │
│                     ┌────────▼────────┐                             │
│                     │  Arduino Nano   │                             │
│                     │   + NRF24L01    │                             │
│                     └────────┬────────┘                             │
│                              │                                       │
└──────────────────────────────┼───────────────────────────────────────┘
                               │ 2.4GHz
                               │ 100 Hz
                               │ Bidirectional
                               ▼
┌──────────────────────────────────────────────────────────────────────┐
│                        FLIGHT CONTROLLER                              │
├──────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌────────────────┐     ┌────────────────────────────────────────┐   │
│  │   NRF24L01     │────▶│           Arduino Nano                 │   │
│  │   PA+LNA       │◀────│                                        │   │
│  └────────────────┘     │  ┌─────────┐  ┌─────────────────────┐  │   │
│                         │  │  AHRS   │  │   Cascade PID       │  │   │
│  ┌────────────────┐     │  │ Mahony  │  │                     │  │   │
│  │    MPU6050     │────▶│  │ Filter  │─▶│ Angle→Rate→Output   │  │   │
│  │   Gyro+Accel   │     │  └─────────┘  └─────────────────────┘  │   │
│  └────────────────┘     │                         │               │   │
│                         │  ┌─────────────────────▼───────────┐   │   │
│  ┌────────────────┐     │  │         Motor Mixer             │   │   │
│  │    MS5611      │────▶│  │    X-Configuration Quad         │   │   │
│  │   Barometer    │     │  └─────────────────────────────────┘   │   │
│  └────────────────┘     │                │                        │   │
│                         └────────────────┼────────────────────────┘   │
│                                          │                            │
│    ┌─────────────┬─────────────┬─────────┴──────────┬─────────────┐  │
│    ▼             ▼             ▼                    ▼             │  │
│  ┌───┐        ┌───┐        ┌───┐                ┌───┐            │  │
│  │FL │        │FR │        │RR │                │RL │            │  │
│  │ESC│        │ESC│        │ESC│                │ESC│            │  │
│  └─┬─┘        └─┬─┘        └─┬─┘                └─┬─┘            │  │
│    │            │            │                    │              │  │
│    ▼            ▼            ▼                    ▼              │  │
│   🔄CCW        🔄CW         🔄CCW                🔄CW             │  │
│                                                                   │  │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 🔧 Hardware Requirements

### Flight Controller

| Component | Specification | Purpose |
|-----------|--------------|---------|
| Arduino Nano | ATmega328P 16MHz | Main processor |
| NRF24L01 PA+LNA | 2.4GHz, +20dBm | Long-range communication |
| MPU6050 | 6-DOF IMU | Attitude sensing |
| MS5611 | Barometer, 10cm resolution | Altitude sensing |
| ESCs (x4) | 20-30A, BLHeli | Motor control |
| Buzzer | 5V active | Status indication |
| LED | 3mm/5mm | Link status |

### Remote Controller

| Component | Specification | Purpose |
|-----------|--------------|---------|
| Arduino Nano | ATmega328P 16MHz | Main processor |
| NRF24L01 PA+LNA | 2.4GHz, +20dBm | Long-range communication |
| Joysticks (x2) | Dual-axis analog | Throttle/Yaw, Pitch/Roll |
| Push Buttons (x2) | Momentary | Calibration, Motor test |
| Toggle Switches (x2) | SPDT | Arm, Altitude hold |

---

## 📌 Pin Mappings

### Flight Controller Pinout

```
Arduino Nano Flight Controller
─────────────────────────────────

                    ┌─────────────┐
                    │  USB PORT   │
                    └──────┬──────┘
              D13 ●────────┼────────● D12
         (SCK) D13 │       │       │ D11 (MOSI)
              D12 ●────────┼────────● D10 ─── NRF CSN
        (MISO) D12 │       │       │ D9 ──── Motor RL
              D11 ●────────┼────────● D8 ──── Buzzer
        (MOSI) D11 │       │       │ D7 ──── Status LED
               D10 ●───────┼────────● D6 ──── Motor RR
       (NRF CSN)   │       │       │ D5 ──── Motor FR
               D9 ●────────┼────────● D4 ──── NRF CE
       (Motor RL)  │       │       │ D3 ──── Motor FL
               D8 ●────────┼────────● D2 ──── MPU INT
        (Buzzer)   │       │       │
               D7 ●────────┼────────● GND
     (Status LED)  │       │       │
                   │       │       │ RST
               D6 ●────────┼────────● RX0
       (Motor RR)  │       │       │ TX1
               D5 ●────────┼────────●
       (Motor FR)  │       │       │
               D4 ●────────┼────────● VIN
        (NRF CE)   │       │       │
               D3 ●────────┼────────● GND
       (Motor FL)  │       │       │
               D2 ●────────┼────────● 5V
       (MPU INT)   │       │       │
               GND ●───────┼────────● A7
                   │       │       │
               RST ●───────┼────────● A6
                   │       │       │
               A0 ●────────┼────────● A5 ──── I2C SCL
                   │       │       │        (MPU6050, MS5611)
               A1 ●────────┼────────● A4 ──── I2C SDA
                   │       │       │        (MPU6050, MS5611)
               A2 ●────────┼────────● A3
                   │       │       │
               A3 ●────────┼────────● 3.3V ── NRF VCC
                   │       │       │
                   └───────┴───────┘

I2C Bus (A4=SDA, A5=SCL):
  • MPU6050: Address 0x68
  • MS5611:  Address 0x77

SPI Bus (D11=MOSI, D12=MISO, D13=SCK):
  • NRF24L01 PA+LNA
```

### Remote Controller Pinout

```
Arduino Nano Remote Controller
───────────────────────────────

Pin Assignments:
  • D2  - Switch 1 (Altitude Hold)
  • D3  - Switch 2 (ARM/Disarm)
  • D4  - Button 1 (Calibration)
  • D5  - Button 2 (Motor Test)
  • D9  - NRF CE
  • D10 - NRF CSN
  • D11 - MOSI (SPI)
  • D12 - MISO (SPI)
  • D13 - SCK (SPI) / LED

  • A0  - Left Joystick Vertical (Throttle)
  • A1  - Left Joystick Horizontal (Yaw)
  • A2  - Right Joystick Vertical (Pitch)
  • A3  - Right Joystick Horizontal (Roll)
```

---

## ✨ Features

### Flight Controller Features

- **400 Hz Main Loop** - Professional-grade control rate
- **Mahony AHRS** - Quaternion-based attitude estimation
- **Cascade PID Control** - Angle → Rate architecture
- **Altitude Hold** - Barometer + accelerometer fusion
- **Fail-safe System** - Automatic motor cutoff on link loss
- **Tilt Protection** - Emergency stop at 60° tilt
- **Calibration System** - Gyro/accel calibration with validation
- **ESC Calibration** - Built-in ESC setup sequence
- **Motor Test Mode** - Individual motor spin-up

### Remote Controller Features

- **100 Hz Update Rate** - Low-latency control
- **Bidirectional Telemetry** - Real-time FC status
- **Serial Monitor Display** - Comprehensive status output
- **Button Debouncing** - Reliable input handling
- **Joystick Filtering** - Smooth control response
- **Link Quality Indication** - LED status feedback

---

## 💻 Installation

### Prerequisites

1. **Arduino IDE** (1.8.x or 2.x)
2. **RF24 Library** by TMRh20
   ```
   Tools → Manage Libraries → Search "RF24" → Install
   ```

### Upload Procedure

1. **Flight Controller**
   ```bash
   cd drone_firmware/flight_controller
   # Open flight_controller.ino in Arduino IDE
   # Select Board: Arduino Nano
   # Select Processor: ATmega328P (or ATmega328P Old Bootloader)
   # Select Port: Your COM port
   # Upload
   ```

2. **Remote Controller**
   ```bash
   cd drone_firmware/remote_controller
   # Open remote_controller.ino in Arduino IDE
   # Upload with same settings
   ```

---

## 🎮 Operation Guide

### Startup Sequence

```
1. Power ON Remote Controller
   └── LED blinks → Searching for FC

2. Power ON Flight Controller
   └── Single beep → Initializing
   └── Double beep → Ready

3. Wait for Link
   └── FC LED blinks fast → Linked
   └── RC LED solid → Linked

4. Calibration (Required first time)
   └── Ensure drone is LEVEL and STATIONARY
   └── Ensure SW2 (ARM) is OFF
   └── Press Button 1
   └── Wait for 2 beeps → Success
   └── (1 long beep = Failed, retry)

5. ESC Calibration (First time only)
   └── Ensure props are REMOVED
   └── Ensure SW2 (ARM) is OFF
   └── Press Button 2
   └── Wait for calibration sequence

6. Arming
   └── Throttle stick LOW
   └── Switch SW2 to ARM position
   └── Rising beep → Armed
   └── LED solid → Ready to fly

7. Flying
   └── Increase throttle gently
   └── Use right stick for pitch/roll
   └── Use left stick horizontal for yaw

8. Altitude Hold (Optional)
   └── Set desired altitude
   └── Switch SW1 to ON
   └── Throttle stick controls climb rate

9. Disarming
   └── Land and reduce throttle
   └── Switch SW2 to DISARM position
   └── Falling beep → Disarmed
```

### Control Mapping

```
LEFT JOYSTICK                    RIGHT JOYSTICK
─────────────                    ──────────────
      ▲                                ▲
      │ Throttle UP                    │ Pitch FWD
      │                                │
 ◄────┼────►  Yaw               ◄────┼────►  Roll
 CCW      CW                   Left      Right
      │                                │
      │ Throttle DOWN                  │ Pitch BACK
      ▼                                ▼
```

---

## 🔬 Control Architecture

### Cascade PID Structure

```
                    ┌───────────────────────────────────────────────┐
                    │              CASCADE PID CONTROLLER            │
                    └───────────────────────────────────────────────┘

  Stick Input     ┌─────────────────┐      ┌─────────────────┐
  (Angle Cmd) ───▶│   ANGLE PID     │─────▶│    RATE PID     │─────▶ Motor
                  │  (Outer Loop)   │ Rate │   (Inner Loop)  │       Output
  Current    ────▶│   100 Hz        │ Cmd  │    400 Hz       │◀──── Current
  Angle          └─────────────────┘      └─────────────────┘       Rate


  Example: Roll Control
  ─────────────────────

  Roll Stick ──▶ [ANGLE PID] ──▶ Roll Rate Cmd ──▶ [RATE PID] ──▶ Motor Mix
      │               │                                  │
      │         Angle Error                         Rate Error
      │               │                                  │
      ▼               ▼                                  ▼
  ±30° max    Roll Angle (AHRS)              Roll Rate (Gyro)
```

### AHRS (Mahony Filter)

The Mahony filter provides robust attitude estimation by fusing gyroscope and accelerometer data:

```
                    MAHONY AHRS FILTER
                    ──────────────────

  Gyroscope ────┐
  (Angular     │     ┌──────────────────────────────────────┐
   Rate)       ├────▶│                                      │
               │     │    Quaternion Integration            │
               │     │                                      │
               │     │    q̇ = ½ q ⊗ [0, ω]                 │
  Accelerometer│     │                                      │
  (Gravity     ├────▶│    With correction from              │
   Vector)     │     │    accelerometer reference           │
               │     │                                      │
               │     │    ω_corrected = ω + Kp*e + Ki*∫e    │
               │     │                                      │
               │     └────────────────┬─────────────────────┘
               │                      │
               │                      ▼
               │              ┌───────────────┐
               │              │  Quaternion   │
               │              │  q0,q1,q2,q3  │
               │              └───────┬───────┘
               │                      │
               │                      ▼
               │              ┌───────────────┐
               │              │ Euler Angles  │
               │              │ Roll, Pitch,  │
               │              │    Yaw        │
               │              └───────────────┘


  Parameters:
    Kp = 10.0   (Proportional gain - responsiveness)
    Ki = 0.005  (Integral gain - drift correction)
```

### Altitude Estimation

```
              ALTITUDE SENSOR FUSION
              ──────────────────────

  MS5611 Barometer          MPU6050 Vertical Accel
  (Low noise, slow)         (High noise, fast)
         │                          │
         │                          │
         ▼                          ▼
  ┌──────────────┐          ┌──────────────┐
  │  Low-Pass    │          │  High-Pass   │
  │   Filter     │          │   Filter     │
  │  α = 0.98    │          │  α = 0.02    │
  └──────┬───────┘          └──────┬───────┘
         │                          │
         └──────────┬───────────────┘
                    │
                    ▼
           ┌────────────────┐
           │  Complementary │
           │     Filter     │
           │                │
           │  h = α*h_baro  │
           │    + (1-α)*∫∫a │
           └────────┬───────┘
                    │
                    ▼
           ┌────────────────┐
           │   Estimated    │
           │   Altitude     │
           │   Velocity     │
           └────────────────┘
```

### Motor Mixing (X-Configuration)

```
               QUADCOPTER MOTOR MIXING
               ───────────────────────

                    FRONT
                      ▲
             FL (CCW) │ FR (CW)
                 ╲    │    ╱
                  ╲   │   ╱
                   ╲  │  ╱
                    ╲ │ ╱
               ──────●──────
                    ╱ │ ╲
                   ╱  │  ╲
                  ╱   │   ╲
                 ╱    │    ╲
             RL (CW)  │  RR (CCW)
                      ▼
                    REAR


  Motor Mixing Equations:
  ───────────────────────

  FL = Throttle + Roll + Pitch - Yaw   (CCW)
  FR = Throttle - Roll + Pitch + Yaw   (CW)
  RR = Throttle - Roll - Pitch - Yaw   (CCW)
  RL = Throttle + Roll - Pitch + Yaw   (CW)

  Where:
    Throttle = 1000-2000 (limited to 65% = 1650)
    Roll     = -400 to +400 (from PID)
    Pitch    = -400 to +400 (from PID)
    Yaw      = -400 to +400 (from PID)
```

---

## 🛡️ Safety Features

| Feature | Description | Action |
|---------|-------------|--------|
| **Link Failsafe** | No packets for 500ms | Motor cutoff |
| **Tilt Protection** | >60° roll or pitch | Motor cutoff |
| **Throttle Limit** | Maximum 65% throttle | Prevents runaway |
| **Arm Interlock** | Throttle must be low | Prevents accidental arm |
| **Kill Switch** | SW2 disarm | Immediate motor stop |
| **Calibration Check** | Must calibrate before arm | Ensures valid sensor data |

---

## 🎛️ Tuning Guide

See [docs/TUNING_GUIDE.md](docs/TUNING_GUIDE.md) for detailed tuning instructions.

### Quick Reference - Default PID Values

```cpp
// Rate PID (Inner Loop) - 400 Hz
Roll Rate:   Kp=0.7,  Ki=0.3,  Kd=0.03
Pitch Rate:  Kp=0.7,  Ki=0.3,  Kd=0.03
Yaw Rate:    Kp=2.0,  Ki=0.5,  Kd=0.0

// Angle PID (Outer Loop) - 100 Hz
Roll Angle:  Kp=4.5,  Ki=0.02, Kd=0.0
Pitch Angle: Kp=4.5,  Ki=0.02, Kd=0.0

// Altitude PID - 50 Hz
Altitude:    Kp=0.5,  Ki=0.02, Kd=0.1
V.Velocity:  Kp=0.15, Ki=0.02, Kd=0.01
```

---

## 🧪 Testing Procedures

See [docs/TESTING_GUIDE.md](docs/TESTING_GUIDE.md) for complete testing procedures.

### Pre-Flight Checklist

- [ ] Props removed for initial testing
- [ ] Battery fully charged
- [ ] All connections secure
- [ ] RC linked (solid LED)
- [ ] Calibration successful
- [ ] Motor direction verified
- [ ] Control direction verified
- [ ] Failsafe tested

---

## 📁 Project Structure

```
drone_firmware/
├── README.md                    # This file
├── shared/
│   └── protocol.h               # Communication protocol definitions
├── flight_controller/
│   ├── flight_controller.ino    # Main FC sketch
│   ├── config.h                 # FC configuration
│   ├── mpu6050.h               # IMU driver
│   ├── ms5611.h                # Barometer driver
│   ├── ahrs.h                  # Mahony AHRS filter
│   ├── pid.h                   # PID controller
│   ├── motors.h                # Motor mixer
│   ├── altitude.h              # Altitude controller
│   ├── nrf_comm.h              # NRF24 communication
│   └── buzzer.h                # Buzzer controller
├── remote_controller/
│   ├── remote_controller.ino    # Main RC sketch
│   └── config.h                 # RC configuration
└── docs/
    ├── TUNING_GUIDE.md          # PID tuning guide
    ├── TESTING_GUIDE.md         # Testing procedures
    └── WIRING_DIAGRAM.md        # Detailed wiring
```

---

## ⚠️ Disclaimer

**This is experimental firmware for educational purposes.** Flying drones can be dangerous. Always:

- Test with propellers removed first
- Fly in open areas away from people
- Follow local regulations
- Use appropriate safety equipment
- Never fly over crowds or near airports

---

## 📄 License

MIT License - See LICENSE file for details.

---

## 🤝 Contributing

Contributions welcome! Please submit issues and pull requests.

---

*Built with ❤️ for the drone community*
