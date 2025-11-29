# Professional Drone System

## Arduino Nano Based Quadcopter with Custom Remote Controller

A professional-grade UAV embedded system featuring a custom flight controller and remote controller, both based on Arduino Nano. The system includes full PID stabilization, barometric altitude hold, and robust NRF24L01 communication with ACK-based telemetry.

---

## 📋 Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Components](#hardware-components)
3. [Wiring Diagrams](#wiring-diagrams)
4. [Software Architecture](#software-architecture)
5. [Installation](#installation)
6. [Operation Guide](#operation-guide)
7. [Safety Features](#safety-features)
8. [Troubleshooting](#troubleshooting)
9. [PID Tuning](#pid-tuning)

---

## 🚁 System Overview

### Features

- **250Hz Flight Control Loop** - Smooth, responsive stabilization
- **Complementary Filter** - Sensor fusion for accurate attitude estimation
- **PID Control** - Independent Roll, Pitch, Yaw, and Altitude control
- **Altitude Hold** - MS5611 barometer-based position hold
- **50Hz Communication** - Reliable NRF24L01 link with ACK payloads
- **Safety Systems** - Multiple failsafes including kill switch and angle limits
- **Real-time Telemetry** - Live drone status on RC serial monitor

### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     REMOTE CONTROLLER                            │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────┐  │
│  │  Joysticks  │───►│Arduino Nano │───►│ NRF24L01 PA+LNA     │  │
│  │  Buttons    │    │             │◄───│ (Transmitter)       │  │
│  │  Switches   │    └─────────────┘    └──────────┬──────────┘  │
│  └─────────────┘                                  │              │
└───────────────────────────────────────────────────│──────────────┘
                                                    │ 2.4GHz
                                                    │ Channel 103
                                                    ▼
┌───────────────────────────────────────────────────│──────────────┐
│                     FLIGHT CONTROLLER             │              │
│  ┌─────────────────────┐    ┌─────────────────────┴──────────┐  │
│  │ NRF24L01 PA+LNA     │───►│        Arduino Nano            │  │
│  │ (Receiver)          │◄───│                                │  │
│  └─────────────────────┘    │  ┌──────┐  ┌──────┐  ┌──────┐  │  │
│                             │  │ PID  │  │Sensor│  │Motor │  │  │
│  ┌─────────────┐            │  │Control│  │Fusion│  │Mixer │  │  │
│  │   MPU6050   │───────────►│  └──────┘  └──────┘  └──────┘  │  │
│  │   MS5611    │            │                                │  │
│  └─────────────┘            └────────────────┬───────────────┘  │
│                                              │                   │
│  ┌───────────────────────────────────────────┴───────────────┐  │
│  │                      ESC x 4                              │  │
│  │     FL (D3)      FR (D5)      RR (D6)      RL (D9)        │  │
│  └───────────────────────────────────────────────────────────┘  │
│                              │                                   │
│  ┌───────────────────────────┴───────────────────────────────┐  │
│  │                     MOTORS x 4                            │  │
│  │    Front-Left   Front-Right   Rear-Right   Rear-Left      │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔧 Hardware Components

### Flight Controller Board

| Component | Quantity | Description |
|-----------|----------|-------------|
| Arduino Nano | 1 | ATmega328P, 16MHz |
| NRF24L01 PA+LNA | 1 | Long-range 2.4GHz transceiver |
| MPU6050 | 1 | 6-DOF IMU (Gyro + Accelerometer) |
| MS5611 | 1 | High-precision barometer |
| Active Buzzer | 1 | 5V, for audio feedback |
| LED | 1 | 5mm, status indicator |
| ESC | 4 | 30A BLHeli, with BEC |
| Brushless Motor | 4 | 2212 1000KV or similar |

### Remote Controller Board

| Component | Quantity | Description |
|-----------|----------|-------------|
| Arduino Nano | 1 | ATmega328P, 16MHz |
| NRF24L01 PA+LNA | 1 | Long-range 2.4GHz transceiver |
| Joystick Module | 2 | Dual-axis analog joysticks |
| Push Button | 2 | Momentary, normally open |
| Toggle Switch | 2 | SPST or SPDT, 3-pin |

---

## 📐 Wiring Diagrams

### Flight Controller Wiring

```
                        ARDUINO NANO
                    ┌─────────────────┐
                    │     USB         │
                    │    ┌───┐        │
              D13 ──┤    │   │        ├── D12
    NRF_SCK ──────►─┤D13 │   │     D11├──────►── NRF_MOSI
    NRF_MISO ◄──────┤D12 │   │     D10├──────►── NRF_CSN
                    │D11 └───┘      D9├──────►── MOTOR_RL (PWM)
                    │D10           D8 ├──────►── BUZZER
                    │D9            D7 ├──────►── STATUS_LED
                    │D8            D6 ├──────►── MOTOR_RR (PWM)
                    │D7            D5 ├──────►── MOTOR_FR (PWM)
                    │D6            D4 ├──────►── NRF_CE
                    │D5            D3 ├──────►── MOTOR_FL (PWM)
                    │D4            D2 ├──◄───── MPU6050_INT
                    │D3            GND├──────── GND
                    │D2            RST│
                    │GND           RX0│
                    │RST           TX1│
                    │              5V ├──────── VCC (5V)
                    │5V            A7 │
                    │A7            A6 │
                    │A6            A5 ├──────►── I2C_SCL (MPU/MS5611)
                    │A5            A4 ├──◄────► I2C_SDA (MPU/MS5611)
                    │A4            A3 │
                    │A3            A2 │
                    │A2            A1 │
                    │A1            A0 │
                    │A0            REF│
                    │              3V3├──────── NRF24L01 VCC (3.3V!)
                    └─────────────────┘


NRF24L01 PA+LNA MODULE (Flight Controller)
┌───────────────────────────┐
│      ┌─────────────┐      │
│      │   ANTENNA   │      │
│      └─────────────┘      │
│                           │
│  GND ──── 1 ○   ○ 2 ──── VCC (3.3V!)
│  CE  ──── 3 ○   ○ 4 ──── CSN
│  SCK ──── 5 ○   ○ 6 ──── MOSI
│  MISO ─── 7 ○   ○ 8 ──── IRQ (not used)
│                           │
└───────────────────────────┘

Pin Connections:
  1 (GND)  → Arduino GND
  2 (VCC)  → Arduino 3.3V (IMPORTANT: NOT 5V!)
  3 (CE)   → Arduino D4
  4 (CSN)  → Arduino D10
  5 (SCK)  → Arduino D13
  6 (MOSI) → Arduino D11
  7 (MISO) → Arduino D12
  8 (IRQ)  → Not connected


MPU6050 MODULE
┌─────────────────┐
│  VCC ── 5V      │
│  GND ── GND     │
│  SCL ── A5      │
│  SDA ── A4      │
│  INT ── D2      │
│  (Other pins NC)│
└─────────────────┘


MS5611 MODULE
┌─────────────────┐
│  VCC ── 5V      │
│  GND ── GND     │
│  SCL ── A5      │ (shared with MPU6050)
│  SDA ── A4      │ (shared with MPU6050)
└─────────────────┘


ESC/MOTOR CONNECTIONS
┌────────────────────────────────────────────┐
│                                            │
│   FL Motor (D3)           FR Motor (D5)    │
│       ↖                       ↗            │
│         \\    FRONT     //                 │
│           \\           //                  │
│             \\   ▲   //                    │
│               \\   //                      │
│                 \\ //                      │
│                  X                         │
│                 // \\                      │
│               //     \\                    │
│             //         \\                  │
│           //     REAR    \\                │
│         //                 \\              │
│       ↙                       ↘            │
│   RL Motor (D9)           RR Motor (D6)    │
│                                            │
│   Motor Rotation:                          │
│   FL = CW  (Clockwise)                     │
│   FR = CCW (Counter-Clockwise)             │
│   RR = CW  (Clockwise)                     │
│   RL = CCW (Counter-Clockwise)             │
│                                            │
└────────────────────────────────────────────┘

ESC Connection:
  Each ESC:
  - Signal wire (white/yellow) → Arduino PWM pin
  - GND wire (black/brown) → Arduino GND
  - Power wires → LiPo Battery
  - Motor wires → Brushless Motor (3 wires)
```

### Remote Controller Wiring

```
                        ARDUINO NANO
                    ┌─────────────────┐
                    │     USB         │
                    │    ┌───┐        │
              D13 ──┤    │   │        ├── D12
    NRF_SCK ──────►─┤D13 │   │     D11├──────►── NRF_MOSI
    NRF_MISO ◄──────┤D12 │   │     D10├──────►── NRF_CSN
                    │D11 └───┘      D9├──────►── NRF_CE
                    │D10           D8 │
                    │D9            D7 │
                    │D8            D6 │
                    │D7            D5 ├──◄───── BTN_MOTORS (Pull-up)
                    │D6            D4 ├──◄───── BTN_CALIBRATE (Pull-up)
                    │D5            D3 ├──◄───── SW_ARM (Pull-up)
                    │D4            D2 ├──◄───── SW_ALT_HOLD (Pull-up)
                    │D3            GND├──────── GND
                    │D2            RST│
                    │GND           RX0│
                    │RST           TX1│
                    │              5V ├──────── VCC (5V)
                    │5V            A7 │
                    │A7            A6 │
                    │A6            A5 │
                    │A5            A4 │
                    │A4            A3 ├──◄───── JOY_RIGHT_H (Roll)
                    │A3            A2 ├──◄───── JOY_RIGHT_V (Pitch)
                    │A2            A1 ├──◄───── JOY_LEFT_H (Yaw)
                    │A1            A0 ├──◄───── JOY_LEFT_V (Throttle)
                    │A0            REF│
                    │              3V3├──────── NRF24L01 VCC (3.3V!)
                    └─────────────────┘


NRF24L01 PA+LNA MODULE (Remote Controller)
┌───────────────────────────┐
│      ┌─────────────┐      │
│      │   ANTENNA   │      │
│      └─────────────┘      │
│                           │
│  GND ──── 1 ○   ○ 2 ──── VCC (3.3V!)
│  CE  ──── 3 ○   ○ 4 ──── CSN
│  SCK ──── 5 ○   ○ 6 ──── MOSI
│  MISO ─── 7 ○   ○ 8 ──── IRQ (not used)
│                           │
└───────────────────────────┘

Pin Connections:
  1 (GND)  → Arduino GND
  2 (VCC)  → Arduino 3.3V (IMPORTANT: NOT 5V!)
  3 (CE)   → Arduino D9
  4 (CSN)  → Arduino D10
  5 (SCK)  → Arduino D13
  6 (MOSI) → Arduino D11
  7 (MISO) → Arduino D12
  8 (IRQ)  → Not connected


JOYSTICK MODULES
┌─────────────────────────────────────────────────┐
│                                                 │
│   LEFT JOYSTICK              RIGHT JOYSTICK     │
│   (Throttle/Yaw)             (Pitch/Roll)       │
│                                                 │
│   ┌─────────┐                ┌─────────┐        │
│   │  VCC────┼── 5V           │  VCC────┼── 5V   │
│   │  GND────┼── GND          │  GND────┼── GND  │
│   │  VRx───┼── A1 (Yaw)     │  VRx───┼── A3    │
│   │  VRy───┼── A0 (Throt)   │  VRy───┼── A2    │
│   │  SW ───┼── NC           │  SW ───┼── NC    │
│   └─────────┘                └─────────┘        │
│                                                 │
│   Control Mapping:                              │
│   A0: Throttle (Up = Climb, Down = Descend)     │
│   A1: Yaw (Left = CCW, Right = CW)              │
│   A2: Pitch (Up = Forward, Down = Backward)     │
│   A3: Roll (Left = Left, Right = Right)         │
│                                                 │
└─────────────────────────────────────────────────┘


BUTTONS AND SWITCHES
┌─────────────────────────────────────────────────┐
│                                                 │
│   PUSH BUTTONS (Momentary)                      │
│                                                 │
│   Button 1 (Calibration)   Button 2 (Motors)   │
│   ┌─────┐                  ┌─────┐              │
│   │     │                  │     │              │
│   │  ○──┼── D4             │  ○──┼── D5         │
│   │  ○──┼── GND            │  ○──┼── GND        │
│   └─────┘                  └─────┘              │
│                                                 │
│   (Internal pull-ups enabled, active LOW)       │
│                                                 │
├─────────────────────────────────────────────────┤
│                                                 │
│   TOGGLE SWITCHES (SPST or SPDT)                │
│                                                 │
│   Switch 1 (Alt Hold)      Switch 2 (Arm)      │
│   ┌─────────┐              ┌─────────┐          │
│   │    ○────┼── D2         │    ○────┼── D3     │
│   │    ○────┼── GND        │    ○────┼── GND    │
│   │   [○]───┼── GND        │   [○]───┼── GND    │
│   └─────────┘              └─────────┘          │
│                                                 │
│   If using SPDT (3-pin) switch:                 │
│   - Center pin → Arduino Digital Pin            │
│   - Both outer pins → GND                       │
│                                                 │
│   Switch UP = Signal HIGH (via pull-up)         │
│   Switch DOWN = Signal LOW (grounded)           │
│                                                 │
└─────────────────────────────────────────────────┘
```

---

## 💻 Software Architecture

### Flight Controller State Machine

```
┌──────────────────────────────────────────────────────────────────┐
│                    FLIGHT CONTROLLER STATES                       │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌────────────┐                                                 │
│   │   INIT     │ ─► Initialize sensors, radio, motors            │
│   └─────┬──────┘                                                 │
│         ▼                                                        │
│   ┌────────────────┐                                             │
│   │ WAIT_CONNECTION│ ─► LED blinks slowly (500ms)                │
│   └───────┬────────┘                                             │
│           ▼ (NRF connected)                                      │
│   ┌────────────┐                                                 │
│   │ CONNECTED  │ ─► LED blinks medium (250ms)                    │
│   └─────┬──────┘                                                 │
│         │ (BTN1 pressed)                                         │
│         ▼                                                        │
│   ┌────────────────┐                                             │
│   │  CALIBRATING   │ ─► LED blinks fast (100ms)                  │
│   │   (IMU Cal)    │     Keep drone still for 4 seconds          │
│   └───────┬────────┘                                             │
│           │ Success: 2 beeps                                     │
│           │ Fail: 1 long beep (7s)                               │
│           ▼                                                      │
│   ┌────────────────┐                                             │
│   │ ESC_CALIBRATING│ ─► (if SW1=OFF, BTN2 pressed)               │
│   │  (Motor Test)  │     Tests each motor sequentially           │
│   └───────┬────────┘                                             │
│           ▼                                                      │
│   ┌────────────┐                                                 │
│   │   READY    │ ─► LED slow pulse (1000ms)                      │
│   │            │    Press BTN2 to enable motors                  │
│   └─────┬──────┘                                                 │
│         │ (SW2=ARM + motors enabled)                             │
│         ▼                                                        │
│   ┌────────────┐                                                 │
│   │   ARMED    │ ─► LED blinks (500ms)                           │
│   │            │    Ready to fly!                                │
│   └─────┬──────┘                                                 │
│         │ (SW2=DISARM)                                           │
│         ▼                                                        │
│   ┌────────────┐                                                 │
│   │  DISARMED  │ ─► Motors stop immediately                      │
│   │ (Kill SW)  │    Returns to READY state                       │
│   └────────────┘                                                 │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

### Control Loop (250Hz)

```
┌─────────────────────────────────────────────────────────────────┐
│                    MAIN CONTROL LOOP                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   1. Read NRF24 ──► Receive commands from RC                    │
│         │                                                       │
│         ▼                                                       │
│   2. Read MPU6050 ──► Get raw gyro/accel data                   │
│         │                                                       │
│         ▼                                                       │
│   3. Complementary Filter ──► Calculate pitch/roll/yaw angles   │
│         │                  │                                    │
│         │                  │  angle = 0.98 * (angle + gyro*dt)  │
│         │                  │        + 0.02 * accel_angle        │
│         ▼                                                       │
│   4. Read MS5611 ──► Get altitude (every 10 loops, ~25Hz)       │
│         │                                                       │
│         ▼                                                       │
│   5. PID Control ──► Calculate corrections                      │
│         │           │                                           │
│         │           │  Roll PID:  Kp=1.3, Ki=0.04, Kd=18        │
│         │           │  Pitch PID: Kp=1.3, Ki=0.04, Kd=18        │
│         │           │  Yaw PID:   Kp=4.0, Ki=0.02, Kd=0         │
│         │           │  Alt PID:   Kp=0.7, Ki=0.01, Kd=25        │
│         ▼                                                       │
│   6. Motor Mixer ──► Calculate individual motor speeds          │
│         │           │                                           │
│         │           │  FL = throttle + pitch + roll + yaw       │
│         │           │  FR = throttle + pitch - roll - yaw       │
│         │           │  RR = throttle - pitch - roll + yaw       │
│         │           │  RL = throttle - pitch + roll - yaw       │
│         ▼                                                       │
│   7. Update ESCs ──► Write PWM (1000-2000µs)                    │
│         │                                                       │
│         ▼                                                       │
│   8. Send Telemetry ──► ACK payload with drone status           │
│         │                                                       │
│         ▼                                                       │
│   9. Wait ──► Maintain 4ms loop time (250Hz)                    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📦 Installation

### Required Libraries

Install via Arduino Library Manager:

1. **RF24** by TMRh20 - NRF24L01 communication
2. **Servo** - Built-in, for ESC control
3. **Wire** - Built-in, for I2C communication

### Steps

1. **Clone/Download** this repository

2. **Install Libraries**
   - Open Arduino IDE
   - Go to Sketch → Include Library → Manage Libraries
   - Search and install "RF24"

3. **Upload Flight Controller Code**
   - Open `flight_controller/flight_controller.ino`
   - Select Board: "Arduino Nano"
   - Select Processor: "ATmega328P" or "ATmega328P (Old Bootloader)"
   - Select Port
   - Upload

4. **Upload Remote Controller Code**
   - Open `remote_controller/remote_controller.ino`
   - Select Board: "Arduino Nano"
   - Upload

5. **Important: Copy Common Files**
   - Ensure the `common/config.h` file is accessible
   - The sketches reference `../common/config.h`
   - Alternative: Copy `config.h` to each sketch folder

---

## 🎮 Operation Guide

### Startup Sequence

```
┌─────────────────────────────────────────────────────────────────┐
│                    STARTUP CHECKLIST                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  □ 1. Place drone on flat, level surface                        │
│                                                                 │
│  □ 2. Ensure propellers are removed for initial testing!        │
│                                                                 │
│  □ 3. Set SW2 (Kill Switch) to OFF position                     │
│                                                                 │
│  □ 4. Power ON Remote Controller                                │
│       - Open Serial Monitor (115200 baud)                       │
│       - Verify "DISCONNECTED" status                            │
│                                                                 │
│  □ 5. Power ON Flight Controller (connect LiPo)                 │
│       - Wait for 1 beep                                         │
│       - LED should start blinking                               │
│                                                                 │
│  □ 6. Verify Connection                                         │
│       - RC shows "CONNECTED"                                    │
│       - FC LED blinks faster                                    │
│       - 2 short beeps confirm link                              │
│                                                                 │
│  □ 7. Calibrate IMU                                             │
│       - Keep drone perfectly still                              │
│       - Press BTN1 (Calibration)                                │
│       - Wait 4 seconds                                          │
│       - Success: 2 beeps                                        │
│       - Failure: 1 long beep (7 seconds) → retry                │
│                                                                 │
│  □ 8. ESC Calibration (First time or after changes)             │
│       - Ensure SW1 (Alt Hold) is OFF                            │
│       - Press BTN2 (Motors)                                     │
│       - Each motor spins briefly in sequence                    │
│       - Special beep pattern confirms completion                │
│                                                                 │
│  □ 9. Enable Motors                                             │
│       - Press BTN2 (Motors) again                               │
│       - RC shows [MOT] status                                   │
│                                                                 │
│  □ 10. ARM the Drone                                            │
│        - Move throttle to MINIMUM                               │
│        - Set SW2 (Arm) to ON position                           │
│        - 1 beep confirms armed                                  │
│        - RC shows [ARMED] status                                │
│                                                                 │
│  ■ 11. FLY!                                                     │
│        - Slowly increase throttle                               │
│        - Use right stick for pitch/roll                         │
│        - Use left stick for yaw                                 │
│                                                                 │
│  ⚠ EMERGENCY: Set SW2 to OFF = IMMEDIATE STOP                   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Control Reference

```
┌─────────────────────────────────────────────────────────────────┐
│                    CONTROL MAPPING                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│        LEFT STICK                    RIGHT STICK                │
│        ┌───────┐                     ┌───────┐                  │
│        │   ↑   │ Climb               │   ↑   │ Forward          │
│        │       │                     │       │                  │
│        │← ─○─ →│ Yaw CCW/CW          │← ─○─ →│ Roll L/R         │
│        │       │                     │       │                  │
│        │   ↓   │ Descend             │   ↓   │ Backward         │
│        └───────┘                     └───────┘                  │
│                                                                 │
│    ┌─────────────────────────────────────────────────────────┐  │
│    │  BUTTONS & SWITCHES                                     │  │
│    ├─────────────────────────────────────────────────────────┤  │
│    │  BTN1 (D4): Calibrate gyroscope/accelerometer          │  │
│    │  BTN2 (D5): Enable motors / ESC calibration            │  │
│    │  SW1  (D2): Altitude Hold ON/OFF                       │  │
│    │  SW2  (D3): ARM (ON) / DISARM-Kill Switch (OFF)        │  │
│    └─────────────────────────────────────────────────────────┘  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🛡️ Safety Features

### Built-in Protections

| Feature | Description |
|---------|-------------|
| **Max Angle Limit** | 30° tilt limit prevents flipping |
| **Throttle Cap** | 65% maximum power limit |
| **Kill Switch** | SW2 immediately disarms motors |
| **Connection Loss** | Auto-disarm if signal lost >500ms |
| **Arm Safety** | Cannot arm with throttle above minimum |
| **Calibration Check** | Must calibrate before arming |
| **ESC Test** | Sequential motor test before flight |

### Emergency Procedures

```
⚠️ EMERGENCY STOP:
   → Flip SW2 (Kill Switch) to OFF
   → All motors stop immediately
   
⚠️ LOSS OF CONTROL:
   → Reduce throttle to minimum
   → Flip SW2 to OFF
   
⚠️ SIGNAL LOSS:
   → Drone auto-disarms after 500ms
   → Motors stop automatically
```

---

## 🔧 Troubleshooting

### Common Issues

| Problem | Possible Cause | Solution |
|---------|----------------|----------|
| NRF24 not responding | Wrong wiring | Check SPI connections |
| | Power issue | Add 10µF capacitor near VCC/GND |
| | Wrong voltage | Must be 3.3V, NOT 5V! |
| No connection | Wrong channel | Both must use channel 103 |
| | Distance too far | Start close, then test range |
| MPU6050 error | I2C address wrong | Check for 0x68 or 0x69 |
| | Wiring issue | Verify SDA/SCL connections |
| Unstable flight | PID needs tuning | See PID Tuning section |
| | Vibration | Use rubber damping mounts |
| | Props wrong | Check CW/CCW prop placement |
| Motors don't spin | Not armed | Follow startup sequence |
| | ESC not calibrated | Run ESC calibration |
| | Throttle not at min | Lower throttle before arm |
| Drift in hover | IMU not calibrated | Recalibrate on level surface |
| | Accelerometer offset | Check mounting level |

### LED Status Indicators

| Pattern | State |
|---------|-------|
| Slow blink (500ms) | Waiting for connection |
| Medium blink (250ms) | Connected, not calibrated |
| Fast blink (100ms) | Calibrating |
| Slow pulse (1000ms) | Ready, waiting to arm |
| Steady blink (500ms) | Armed, ready to fly |

### Buzzer Codes

| Pattern | Meaning |
|---------|---------|
| 1 short beep | Startup complete |
| 2 short beeps | Connection established |
| 2 beeps | Calibration successful |
| 1 long beep (7s) | Calibration failed |
| Short-Short-Long | ESC calibration complete |
| 1 beep | Armed |
| 2 quick beeps | Disarmed |

---

## 🎯 PID Tuning

### Current Default Values

```cpp
// Roll PID
Kp = 1.3    // Proportional gain
Ki = 0.04   // Integral gain
Kd = 18.0   // Derivative gain

// Pitch PID
Kp = 1.3
Ki = 0.04
Kd = 18.0

// Yaw PID
Kp = 4.0
Ki = 0.02
Kd = 0.0

// Altitude PID
Kp = 0.7
Ki = 0.01
Kd = 25.0
```

### Tuning Guide

1. **Start with P only** (set I and D to 0)
   - Increase Kp until drone oscillates
   - Reduce Kp by 20%

2. **Add Derivative**
   - Increase Kd until oscillation stops
   - This dampens quick movements

3. **Add Integral**
   - Slowly increase Ki
   - Helps eliminate steady-state error
   - Too much = slow oscillation

4. **Test in order**: Roll → Pitch → Yaw → Altitude

### Tuning Tips

- Make small changes (10-20%)
- Test outdoors in calm conditions
- Start with conservative values
- Tune without propellers first (motor response)
- Always have kill switch ready!

---

## 📝 File Structure

```
drone_system/
├── common/
│   └── config.h           # Shared configuration and packet structures
├── flight_controller/
│   └── flight_controller.ino    # Main flight controller firmware
├── remote_controller/
│   └── remote_controller.ino    # Remote controller firmware
└── README.md              # This documentation
```

---

## 📄 License

This project is open source for educational and personal use.

**⚠️ DISCLAIMER:** Flying drones carries inherent risks. Always follow local regulations, fly safely, and use appropriate safety equipment. The authors are not responsible for any damages or injuries resulting from the use of this system.

---

## 🔮 Future Improvements

- [ ] GPS position hold
- [ ] Return to home function
- [ ] Battery voltage monitoring with low-battery warning
- [ ] SD card logging
- [ ] Bluetooth configuration app
- [ ] OLED display for remote controller
- [ ] Auto-landing feature
- [ ] Waypoint navigation

---

**Happy Flying! 🚁**
