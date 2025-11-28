# Arduino Nano Quadcopter Drone

A complete DIY quadcopter drone system using Arduino Nano with full stabilization, calibration, and wireless control.

## 🚁 Project Overview

This project implements a fully functional quadcopter drone with:
- **PID-based flight stabilization** using MPU6050 IMU
- **Altitude hold capability** using MS5611 barometer
- **Wireless control** via NRF24L01 2.4GHz transceiver
- **Comprehensive calibration system** for IMU, ESC, and joysticks
- **Safety features** including arm/disarm switch and kill switch
- **User-guided setup** via Serial Monitor

## 📦 Hardware Components

### Flight Controller Board
| Component | Quantity | Description |
|-----------|----------|-------------|
| Arduino Nano | 1 | Main microcontroller (ATmega328P) |
| NRF24L01 | 1 | 2.4GHz wireless transceiver |
| MPU6050 | 1 | 6-axis IMU (Gyro + Accelerometer) |
| MS5611 | 1 | Barometric pressure sensor |
| ESC | 4 | Electronic Speed Controllers (30A recommended) |
| Buzzer | 1 | Active buzzer for audio feedback |
| LED | 2 | Status LEDs |

### RC Transmitter Board
| Component | Quantity | Description |
|-----------|----------|-------------|
| Arduino Nano | 1 | Main microcontroller |
| NRF24L01 | 1 | 2.4GHz wireless transceiver |
| Joystick | 2 | Analog joysticks (KY-023) |
| Toggle Switch | 1 | Arm/Disarm switch |
| Push Button | 2 | Calibration and motor test buttons |
| LED | 1 | Status LED |

## 🎮 Control Layout

```
    LEFT JOYSTICK                    RIGHT JOYSTICK
         ▲                                ▲
    Throttle UP                      Pitch Forward
         │                                │
    ◄────┼────►                      ◄────┼────►
   Yaw   │   Yaw                   Roll   │   Roll
   Left  │  Right                  Left   │  Right
         │                                │
         ▼                                ▼
    Throttle DOWN                   Pitch Backward
```

### Control Functions:
- **Left Stick Y-axis (Throttle)**: Controls altitude (UP = ascend, DOWN = descend)
- **Left Stick X-axis (Yaw)**: Rotates drone (LEFT = CCW, RIGHT = CW)
- **Right Stick Y-axis (Pitch)**: Forward/backward tilt (UP = forward, DOWN = backward)
- **Right Stick X-axis (Roll)**: Left/right tilt (LEFT = strafe left, RIGHT = strafe right)

## 🔌 Wiring Diagrams

### Flight Controller Wiring

```
ARDUINO NANO FLIGHT CONTROLLER
═══════════════════════════════════════════════════════════════

                        ┌─────────────────┐
                        │   ARDUINO NANO  │
                        │                 │
              ┌─────────┤ D2          D13 ├─────────┐
              │         │ D3          D12 ├───────┐ │
              │    ┌────┤ D4          D11 ├─────┐ │ │
              │    │    │ D5          D10 ├───┐ │ │ │
              │    │    │ D6           D9 ├─┐ │ │ │ │
              │    │    │ D7           D8 ├─┼─┼─┼─┼─┼── BUZZER
              │    │    │              A0 │ │ │ │ │ │
              │    │    │              A1 │ │ │ │ │ │
              │    │    │              A2 │ │ │ │ │ │
              │    │    │              A3 │ │ │ │ │ │
              │    │    │ A4(SDA)      A4 ├─┼─┼─┼─┼─┼── I2C SDA
              │    │    │ A5(SCL)      A5 ├─┼─┼─┼─┼─┼── I2C SCL
              │    │    │              A6 │ │ │ │ │ │
              │    │    │              A7 │ │ │ │ │ │
              │    │    │ VIN         GND │ │ │ │ │ │
              │    │    │ GND          5V │ │ │ │ │ │
              │    │    │ RST         3V3 ├─┼─┼─┼─┼─┼── NRF24 VCC
              │    │    └─────────────────┘ │ │ │ │ │
              │    │                        │ │ │ │ │
              │    │    NRF24L01:           │ │ │ │ │
              │    │    CE  ← D4            │ │ │ │ │
              │    │    CSN ← D7            │ │ │ │ │
              │    └────SCK ← D13 ──────────┼─┼─┼─┼─┘
              │         MOSI← D11 ──────────┼─┼─┘ │
              │         MISO← D12 ──────────┼─┘   │
              │                             │     │
              │    ESC SIGNALS:             │     │
              │    Motor 1 (Front-Right) ← D3     │
              │    Motor 2 (Rear-Right)  ← D9 ────┘
              │    Motor 3 (Rear-Left)   ← D10
              │    Motor 4 (Front-Left)  ← D11 (shared with MOSI, use D6)
              │
              │    LEDS:
              │    Status LED ← D2
              └────ARM LED    ← D5

I2C BUS (A4=SDA, A5=SCL):
├── MPU6050 (Address: 0x68)
└── MS5611  (Address: 0x77)
```

### RC Transmitter Wiring

```
ARDUINO NANO RC TRANSMITTER
═══════════════════════════════════════════════════════════════

                        ┌─────────────────┐
                        │   ARDUINO NANO  │
                        │                 │
    STATUS LED ─────────┤ D2          D13 ├───── NRF24 SCK
    BTN1 (CAL) ─────────┤ D3          D12 ├───── NRF24 MISO
    BTN2 (MOTOR)────────┤ D4          D11 ├───── NRF24 MOSI
    ARM SWITCH ─────────┤ D5          D10 ├───── NRF24 CSN
                        │ D6           D9 ├───── NRF24 CE
                        │ D7           D8 │
    LEFT JOY Y ─────────┤ A0          A7  │
    LEFT JOY X ─────────┤ A1          A6  │
    RIGHT JOY Y ────────┤ A2          A5  │
    RIGHT JOY X ────────┤ A3          A4  │
                        │ VIN         GND │
                        │ GND          5V │
                        │ RST         3V3 ├───── NRF24 VCC
                        └─────────────────┘

JOYSTICK CONNECTIONS:
┌─────────────────────────────────────────┐
│ LEFT JOYSTICK          RIGHT JOYSTICK   │
│ VRx → A1 (Yaw)         VRx → A3 (Roll)  │
│ VRy → A0 (Throttle)    VRy → A2 (Pitch) │
│ GND → GND              GND → GND        │
│ +5V → 5V               +5V → 5V         │
│ SW  → (not used)       SW  → (not used) │
└─────────────────────────────────────────┘
```

### Motor Configuration (X-Configuration)

```
        FRONT
          │
    M4    │    M1
     ╲    │    ╱
      ╲   │   ╱
       ╲  │  ╱
        ╲ │ ╱
         ╲│╱
          ●────────► RIGHT
         ╱│╲
        ╱ │ ╲
       ╱  │  ╲
      ╱   │   ╲
     ╱    │    ╲
    M3    │    M2
          │
        REAR

Motor Rotation:
M1 (Front-Right): CCW ↺
M2 (Rear-Right):  CW  ↻
M3 (Rear-Left):   CCW ↺
M4 (Front-Left):  CW  ↻
```

## 🔧 Installation

### Required Libraries
Install these libraries via Arduino IDE Library Manager:

```
- RF24 (by TMRh20) - NRF24L01 communication
- Wire - I2C communication (built-in)
- EEPROM - Data storage (built-in)
```

### Upload Instructions

1. **Flight Controller:**
   - Open `flight_controller/flight_controller.ino`
   - Select Board: "Arduino Nano"
   - Select Processor: "ATmega328P" or "ATmega328P (Old Bootloader)"
   - Upload

2. **RC Transmitter:**
   - Open `rc_transmitter/rc_transmitter.ino`
   - Select Board: "Arduino Nano"
   - Upload

## 🎯 Calibration Process

### Step-by-Step Guide (via Serial Monitor)

1. **Power On Both Units**
   - Open Serial Monitor (115200 baud)
   - Wait for "NRF Connected!" message
   - Buzzer will beep when connected

2. **Set Kill Switch**
   - Toggle ARM switch to OFF (kill switch mode)
   - Serial Monitor will confirm: "Kill switch confirmed"

3. **Calibration (Press Button 1)**
   - Place drone on flat surface
   - Press BTN1 to start calibration
   - IMU calibration: ~5 seconds (keep drone still)
   - ESC calibration: ~10 seconds
   - Joystick calibration: Move sticks to extremes
   - Settings saved to EEPROM

4. **Arm the Drone**
   - Toggle ARM switch to ON
   - Serial Monitor: "DRONE ARMED!"

5. **Motor Test (Press Button 2)**
   - Press BTN2 for smooth motor spin-up
   - Verify all motors spin correctly

6. **Ready to Fly!**
   - Serial Monitor shows live telemetry
   - Slowly increase throttle to take off

## 📊 Telemetry Display

```
╔════════════════════════════════════════╗
║         QUADCOPTER TELEMETRY           ║
╠════════════════════════════════════════╣
║ THR: 1200  YAW: 1500  PIT: 1500  ROL: 1500 ║
║ ALT: 2.5m  BAT: 11.2V  RSSI: -45dBm    ║
║ GYRO: X:0.5 Y:-0.2 Z:0.1               ║
║ PID:  P:1.2 I:0.01 D:15                ║
║ STATUS: ARMED | FLYING | CH:76         ║
╚════════════════════════════════════════╝
```

## ⚠️ Safety Features

1. **Arm/Disarm Switch**: Physical toggle prevents accidental motor activation
2. **Kill Switch**: Instantly stops all motors
3. **Failsafe**: Motors stop if RC signal lost for >500ms
4. **Throttle Lock**: Throttle must be at minimum to arm
5. **Calibration Required**: Won't arm without proper calibration
6. **Low Throttle Protection**: Minimum throttle = motors off (not idle)

## 🔄 PID Tuning

Default PID values (adjustable in code):

| Axis  | P    | I     | D    |
|-------|------|-------|------|
| Roll  | 1.2  | 0.02  | 18   |
| Pitch | 1.2  | 0.02  | 18   |
| Yaw   | 2.0  | 0.02  | 0    |

### Tuning Tips:
1. Start with P only (I=0, D=0)
2. Increase P until oscillation, then reduce by 20%
3. Add D to dampen oscillation
4. Add small I to eliminate drift

## 📁 File Structure

```
quadcopter_drone/
├── flight_controller/
│   ├── flight_controller.ino    # Main flight controller code
│   ├── config.h                 # Pin definitions & settings
│   ├── mpu6050.h               # IMU sensor handler
│   ├── mpu6050.cpp
│   ├── ms5611.h                # Barometer handler
│   ├── ms5611.cpp
│   ├── pid.h                   # PID controller
│   ├── pid.cpp
│   ├── motors.h                # ESC/Motor control
│   ├── motors.cpp
│   ├── nrf_comm.h              # NRF24L01 communication
│   ├── nrf_comm.cpp
│   └── calibration.h           # Calibration routines
│
├── rc_transmitter/
│   ├── rc_transmitter.ino      # Main RC code
│   ├── config.h                # Pin definitions
│   └── nrf_comm.h              # Communication protocol
│
├── docs/
│   └── wiring_diagrams.md      # Detailed wiring info
│
└── README.md                   # This file
```

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| NRF not connecting | Check wiring, add 10µF capacitor to NRF VCC |
| Motors not spinning | Verify ESC calibration, check signal wires |
| Drone unstable | Re-calibrate IMU on flat surface |
| Drifting | Adjust PID values, check propeller balance |
| Short range | Use NRF24L01+PA+LNA module with antenna |

## 📜 License

This project is open source. Use at your own risk.

## ⚡ Disclaimer

**Flying drones can be dangerous!** Always:
- Fly in open areas away from people
- Use propeller guards when learning
- Check local regulations
- Never fly near airports
- Maintain line of sight
