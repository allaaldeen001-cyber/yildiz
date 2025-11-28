# DIY Drone Flight Control System

A complete Arduino-based quadcopter flight control system with NRF24L01 radio communication, MPU6050 gyroscope/accelerometer, MS5611 barometer for altitude hold, and a custom joystick-based remote controller.

## 📋 Table of Contents

- [System Overview](#system-overview)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Library Dependencies](#library-dependencies)
- [Installation](#installation)
- [Joystick Control Mapping](#joystick-control-mapping)
- [Operation Guide](#operation-guide)
- [Safety Features](#safety-features)
- [PID Tuning Guide](#pid-tuning-guide)
- [Troubleshooting](#troubleshooting)

---

## 🚁 System Overview

This project consists of two main components:

1. **Flight Controller (FC)** - Mounted on the drone, handles stabilization, motor control, and altitude hold
2. **Remote Controller (RC)** - Handheld transmitter with joysticks, buttons, and switches

### Features

- ✅ PID-based attitude stabilization
- ✅ Complementary filter for sensor fusion (Gyro + Accelerometer)
- ✅ Altitude hold using MS5611 barometer with Kalman filter
- ✅ 2.4GHz NRF24L01 radio communication
- ✅ Smooth motor start feature
- ✅ Calibration storage in EEPROM
- ✅ Safety angle limits (30° max tilt)
- ✅ Radio link loss detection
- ✅ LED status indicators and buzzer feedback

---

## 🔧 Hardware Requirements

### Flight Controller

| Component | Quantity | Notes |
|-----------|----------|-------|
| Arduino Nano | 1 | ATmega328P, 5V |
| NRF24L01 | 1 | 2.4GHz radio module |
| MPU6050 | 1 | 6-axis gyro/accelerometer |
| MS5611 | 1 | Barometric pressure sensor |
| ESC | 4 | Electronic Speed Controllers |
| Brushless Motors | 4 | Match with ESCs |
| Buzzer | 1 | Active, 5V |
| LED | 1 | 5mm, any color |
| Resistors | 2 | 1.5kΩ and 1kΩ (voltage divider) |
| Toggle Switch | 2 | SPST |
| Push Button | 2 | Momentary |

### Remote Controller

| Component | Quantity | Notes |
|-----------|----------|-------|
| Arduino Nano | 1 | ATmega328P, 5V |
| NRF24L01 | 1 | 2.4GHz radio module |
| Joystick Module | 2 | Dual-axis analog |
| Toggle Switch | 2 | SPST |
| Push Button | 2 | Momentary |

---

## 📌 Pin Configuration

### Flight Controller Pinout

```
Arduino Nano Pin     Function              Connection
─────────────────────────────────────────────────────────
D2                   Altitude Hold SW      Switch to GND
D3 (PWM)             ESC Front Left        ESC signal wire
D4                   NRF24L01 CE           NRF CE pin
D5 (PWM)             ESC Front Right       ESC signal wire
D6 (PWM)             ESC Rear Right        ESC signal wire
D7                   Status LED            LED + resistor
D8                   Buzzer                Buzzer +
D9 (PWM)             ESC Rear Left         ESC signal wire
D10                  NRF24L01 CSN          NRF CSN pin
D11                  NRF24L01 MOSI         NRF MOSI pin
D12                  NRF24L01 MISO         NRF MISO pin
D13                  NRF24L01 SCK          NRF SCK pin
A0                   Battery Voltage       Voltage divider
A1                   Calibration Button    Button to GND
A2                   Motor Start Button    Button to GND
A3                   Arm/Disarm Switch     Switch to GND
A4 (SDA)             I2C Data              MPU6050 & MS5611
A5 (SCL)             I2C Clock             MPU6050 & MS5611
VCC                  Power                 5V supply
GND                  Ground                Common ground
```

### Remote Controller Pinout

```
Arduino Nano Pin     Function              Connection
─────────────────────────────────────────────────────────
D2                   Altitude Hold SW      Switch to GND
D3                   Arm/Disarm Switch     Switch to GND
D4                   Calibration Button    Button to GND
D5                   Motor Start Button    Button to GND
D9                   NRF24L01 CE           NRF CE pin
D10                  NRF24L01 CSN          NRF CSN pin
D11                  NRF24L01 MOSI         NRF MOSI pin
D12                  NRF24L01 MISO         NRF MISO pin
D13                  NRF24L01 SCK          NRF SCK pin
A0                   Throttle (Left Y)     Joystick Y
A1                   Yaw (Left X)          Joystick X
A2                   Pitch (Right Y)       Joystick Y
A3                   Roll (Right X)        Joystick X
VCC                  Power                 5V supply
GND                  Ground                Common ground
```

### Motor Layout (X Configuration)

```
       FRONT
    FL ╲   ╱ FR
        ╲ ╱
         ╳
        ╱ ╲
    RL ╱   ╲ RR
       REAR

FL = Front Left  (D3) - CCW
FR = Front Right (D5) - CW
RL = Rear Left   (D9) - CW
RR = Rear Right  (D6) - CCW
```

---

## 📚 Library Dependencies

Install these libraries via Arduino Library Manager:

```
- RF24 by TMRh20
- Servo (built-in)
- Wire (built-in)
- EEPROM (built-in)
- Smoothed by Matthew Fryer
- MS5611 by Rob Tillaart
```

---

## 🔌 Installation

### 1. Wire the Hardware

#### Flight Controller
1. Connect NRF24L01 to SPI pins (see pinout above)
2. Connect MPU6050 to I2C (A4=SDA, A5=SCL, VCC=3.3V, GND)
3. Connect MS5611 to I2C (same bus as MPU6050)
4. Connect ESCs to PWM pins D3, D5, D6, D9
5. Connect buzzer to D8, LED to D7
6. Connect buttons/switches to analog pins with pull-ups

#### Remote Controller
1. Connect NRF24L01 to SPI pins
2. Connect joysticks to analog pins A0-A3
3. Connect buttons/switches to D2-D5 with pull-ups

### 2. Upload the Code

1. Open Arduino IDE
2. Install required libraries
3. Open `FlightController/FlightController.ino`
4. Select board: "Arduino Nano" (ATmega328P)
5. Upload to Flight Controller
6. Open `RemoteController/RemoteController.ino`
7. Upload to Remote Controller

### 3. Initial Calibration

1. Place drone on a level surface
2. Power on RC first, then FC
3. Wait for link confirmation (beep)
4. Ensure Arm Switch is OFF (disarmed)
5. Press Calibration Button for 2+ seconds
6. Wait for confirmation beeps
7. Calibration is saved to EEPROM

---

## 🎮 Joystick Control Mapping

```
+─────────────────────+    +─────────────────────+
│     LEFT STICK      │    │    RIGHT STICK      │
│                     │    │                     │
│         ↑           │    │         ↑           │
│    THROTTLE UP      │    │    PITCH FORWARD    │
│                     │    │                     │
│  ← YAW   ●   YAW →  │    │  ← ROLL   ●  ROLL → │
│   CCW        CW     │    │   LEFT       RIGHT  │
│                     │    │                     │
│    THROTTLE DOWN    │    │    PITCH BACKWARD   │
│         ↓           │    │         ↓           │
+─────────────────────+    +─────────────────────+
```

| Control | Joystick | Direction | Action |
|---------|----------|-----------|--------|
| Throttle | Left Y | Up | Increase motor power |
| Throttle | Left Y | Down | Decrease motor power |
| Yaw | Left X | Left | Rotate counterclockwise |
| Yaw | Left X | Right | Rotate clockwise |
| Pitch | Right Y | Up | Tilt forward (fly forward) |
| Pitch | Right Y | Down | Tilt backward (fly backward) |
| Roll | Right X | Left | Tilt left (fly left) |
| Roll | Right X | Right | Tilt right (fly right) |

---

## 📖 Operation Guide

### Startup Sequence

```
1. ⚡ Power ON Remote Controller first
2. ⚡ Power ON Flight Controller
3. 🔊 Listen for link confirmation beep
4. ✅ Verify LED blinks when moving sticks
```

### Control Buttons & Switches

| Control | Position/Action | Function |
|---------|-----------------|----------|
| **Arm Switch** | ON (1) | DISARMED - motors disabled, LED stays ON |
| **Arm Switch** | OFF (0) | ARMED - ready for flight |
| **Calibration Button** | Hold 2s | Calibrate MPU6050 & MS5611 (when disarmed) |
| **Motor Start Button** | Hold 2s | Smooth motor ramp-up (when armed) |
| **Altitude Hold Switch** | ON | Enable altitude hold mode |
| **Altitude Hold Switch** | OFF | Normal flight mode |

### Full Flight Workflow

```
┌──────────────────────────────────────────────────────────┐
│  1. POWER ON                                             │
│     └─ RC first, then FC                                 │
│     └─ Wait for link beep                                │
├──────────────────────────────────────────────────────────┤
│  2. PRE-FLIGHT CHECKS                                    │
│     └─ Arm Switch = OFF (disarmed)                       │
│     └─ LED should be solid ON                            │
│     └─ Throttle at minimum                               │
├──────────────────────────────────────────────────────────┤
│  3. CALIBRATION (if needed)                              │
│     └─ Place on level surface                            │
│     └─ Hold Calibration Button 2+ seconds                │
│     └─ Wait for confirmation beeps                       │
├──────────────────────────────────────────────────────────┤
│  4. ARMING                                               │
│     └─ Set Arm Switch = ON (armed)                       │
│     └─ Hold Motor Start Button 2+ seconds                │
│     └─ Motors will ramp up slowly                        │
│     └─ Verify all motors spin correctly                  │
├──────────────────────────────────────────────────────────┤
│  5. TAKEOFF                                              │
│     └─ Slowly increase throttle                          │
│     └─ Use right stick for pitch/roll                    │
│     └─ Use left stick (X) for yaw                        │
├──────────────────────────────────────────────────────────┤
│  6. ALTITUDE HOLD (optional)                             │
│     └─ Reach desired altitude                            │
│     └─ Set throttle to ~1400-1450 range                  │
│     └─ Enable Altitude Hold Switch                       │
│     └─ Drone maintains altitude automatically            │
├──────────────────────────────────────────────────────────┤
│  7. LANDING                                              │
│     └─ Disable Altitude Hold Switch                      │
│     └─ Slowly reduce throttle                            │
│     └─ Touch down gently                                 │
│     └─ Set Arm Switch = OFF immediately                  │
└──────────────────────────────────────────────────────────┘
```

---

## 🛡️ Safety Features

### Angle Limit Protection
- Maximum tilt angle: **30 degrees**
- If exceeded, motors cut immediately
- Prevents flips and crashes

### Radio Link Loss
- Timeout: **3 seconds**
- Motors cut on signal loss
- Buzzer alarm sounds
- Recovers when signal restored

### LED Status Indicators

| LED State | Meaning |
|-----------|---------|
| Solid ON | Disarmed (safe) |
| Fast blink | Receiving radio data |
| Slow blink | Armed, no data |
| OFF | Error state |

### Buzzer Codes

| Pattern | Meaning |
|---------|---------|
| 1-2-3 rising tones | Startup complete |
| Two quick beeps | Calibration started |
| Rising success tone | Calibration/operation complete |
| Long single beep | Armed |
| Repeated beeps | Kill switch / error |

---

## 🎚️ PID Tuning Guide

### Default PID Values

```cpp
const float KP = 2.0;      // Proportional
const float KI = 0.0001;   // Integral
const float KD = 0.5;      // Derivative
const float KP_Z = 2.0;    // Yaw proportional
```

### Tuning Process

1. **Start with P only** (set I and D to 0)
   - Increase P until drone oscillates
   - Reduce P by 20%

2. **Add D term**
   - Increase D to dampen oscillations
   - Too much D causes jittering

3. **Add I term (optional)**
   - Very small values (0.0001-0.001)
   - Corrects steady-state drift

### Altitude Hold PID

```cpp
float pid_p_gain_altitude = 14.0;
float pid_i_gain_altitude = 2.0;
float pid_d_gain_altitude = 7.5;
int pid_max_altitude = 400;
```

---

## 🔧 Troubleshooting

### No Radio Link

- Verify both use same `PIPE_ADDRESS` and `NRF_CHANNEL`
- Check NRF24L01 power supply (needs stable 3.3V)
- Add capacitor (10-100μF) across NRF power pins
- Reduce distance and try again

### Motors Don't Spin

1. Check ESC calibration
2. Verify Arm Switch is in ARMED position (0)
3. Hold Motor Start Button for 2+ seconds
4. Check ESC signal wire connections

### Drone Tips Over

1. Recalibrate on level surface
2. Check motor/propeller direction
3. Verify ESC connections match motor layout
4. Tune PID parameters

### Altitude Hold Doesn't Work

1. Verify MS5611 is connected properly
2. Check I2C address (default 0x77)
3. Ensure throttle is in 1400-1450 range
4. Enable Altitude Hold Switch

### Drift/Not Level

1. Recalibrate gyroscope
2. Check for vibrations
3. Ensure drone is stationary during calibration
4. Verify accelerometer calibration values

---

## 📁 Project Structure

```
/workspace/
├── FlightController/
│   ├── FlightController.ino   # Main flight controller code
│   ├── Barometer.ino          # Altitude hold logic
│   ├── KalmanFilter.ino       # Sensor fusion
│   ├── Gyro.h                 # Gyro class header
│   └── Gyro.cpp               # Gyro implementation
├── RemoteController/
│   └── RemoteController.ino   # RC transmitter code
└── README.md                  # This file
```

---

## ⚠️ Disclaimer

**Flying drones can be dangerous.** Always:

- Fly in open areas away from people
- Check local regulations
- Use propeller guards when learning
- Have a spotter present
- Start with low throttle
- Test calibration before each flight

---

## 📝 License

This project is provided as-is for educational purposes. Use at your own risk.

---

## 🙏 Acknowledgments

- RF24 library by TMRh20
- MS5611 library by Rob Tillaart
- PID control concepts from various open-source flight controllers
