# DIY Drone Flight Control System

A complete Arduino-based drone flight controller and remote control system featuring:
- **PID-based attitude stabilization** with MPU6050 gyroscope/accelerometer
- **Altitude hold** using MS5611 barometer with Kalman filtering
- **Reliable RF communication** using NRF24L01 with ACK payloads
- **Safety features** including maximum tilt limits and communication timeout protection

## 📁 Project Structure

```
├── FlightController/
│   ├── FlightController.ino   # Main flight controller code
│   ├── Gyro.h                 # MPU6050 library header
│   └── Gyro.cpp               # MPU6050 library implementation
├── RemoteController/
│   └── RemoteController.ino   # Remote transmitter code
└── README.md
```

## 🛠️ Hardware Requirements

### Flight Controller (FC)
- Arduino Nano
- NRF24L01 wireless module
- MPU6050 gyroscope/accelerometer
- MS5611 barometer
- 4x ESC (Electronic Speed Controllers)
- 4x Brushless motors
- LED
- Buzzer
- Voltage divider for battery monitoring

### Remote Controller (RC)
- Arduino Nano
- NRF24L01 wireless module
- 2x Analog joysticks
- 2x Momentary push buttons
- 2x Toggle switches
- LED (optional)
- Buzzer (optional)

## 📌 Wiring Diagrams

### Flight Controller Wiring

```
ARDUINO NANO PIN ASSIGNMENTS - FLIGHT CONTROLLER
================================================

                    +-----+
         +----------| USB |----------+
         |          +-----+          |
    D1/TX|                           |VIN ← Battery (7-12V)
    D0/RX|                           |GND ← Common Ground
   RESET|                           |RESET
     GND|                           |5V
      D2|← Altitude Hold Switch     |A7
      D3|→ ESC Front Left (PWM)     |A6
      D4|→ NRF24L01 CE              |A5 → I2C SCL (MPU6050, MS5611)
      D5|→ ESC Front Right (PWM)    |A4 → I2C SDA (MPU6050, MS5611)
      D6|→ ESC Rear Right (PWM)     |A3 ← Arm/Disarm Switch
      D7|→ Status LED               |A2 ← Smooth Start Button
      D8|→ Buzzer                   |A1 ← Calibration Button
      D9|→ ESC Rear Left (PWM)      |A0 ← Battery Voltage
     D10|→ NRF24L01 CSN             |AREF
     D11|→ NRF24L01 MOSI            |3V3 → NRF24L01 VCC
     D12|← NRF24L01 MISO            |
     D13|→ NRF24L01 SCK             |
         +---------------------------+

NRF24L01 Module:
┌─────────────────────┐
│  GND  VCC           │
│   1    2   (3.3V!)  │
│  CE   CSN           │
│   3    4            │
│  SCK  MOSI          │
│   5    6            │
│  MISO IRQ           │
│   7    8 (unused)   │
└─────────────────────┘
Connections:
  VCC  → Arduino 3.3V
  GND  → Arduino GND
  CE   → D4
  CSN  → D10
  SCK  → D13
  MOSI → D11
  MISO → D12

MPU6050 Module:
  VCC → Arduino 5V
  GND → Arduino GND
  SCL → A5
  SDA → A4

MS5611 Module:
  VCC → Arduino 5V (or 3.3V depending on module)
  GND → Arduino GND
  SCL → A5
  SDA → A4

ESC Connections:
  Front Left (FL)  → D3
  Front Right (FR) → D5
  Rear Right (RR)  → D6
  Rear Left (RL)   → D9
  (All ESC signal wires also need common GND)

Switches/Buttons (all with INPUT_PULLUP - connect to GND when active):
  Calibration Button   → A1 (hold 2s when disarmed)
  Smooth Start Button  → A2 (press when armed)
  Arm/Disarm Switch    → A3 (HIGH=Disarmed, LOW=Armed)
  Altitude Hold Switch → D2 (LOW=Active)

LED and Buzzer:
  LED+    → D7 (with 220-470Ω resistor to GND)
  Buzzer+ → D8

Battery Voltage Monitor (voltage divider):
  Battery+ → R1 (1.5kΩ) → A0 → R2 (1kΩ) → GND
  (Supports up to 12.5V with 5V Arduino ADC)
```

### Remote Controller Wiring

```
ARDUINO NANO PIN ASSIGNMENTS - REMOTE CONTROLLER
=================================================

                    +-----+
         +----------| USB |----------+
         |          +-----+          |
    D1/TX|                           |VIN ← Battery (7-12V)
    D0/RX|                           |GND ← Common Ground
   RESET|                           |RESET
     GND|                           |5V
      D2|← Switch 2 (Alt Hold)      |A7
      D3|← Switch 1 (Arm/Disarm)    |A6
      D4|← Button 1 (Calibration)   |A5
      D5|← Button 2 (Motor Start)   |A4
      D6|→ LED (optional)           |A3 ← Right Joystick X (Roll)
      D7|→ Buzzer (optional)        |A2 ← Right Joystick Y (Pitch)
      D8|                           |A1 ← Left Joystick X (Yaw)
      D9|→ NRF24L01 CE              |A0 ← Left Joystick Y (Throttle)
     D10|→ NRF24L01 CSN             |AREF
     D11|→ NRF24L01 MOSI            |3V3 → NRF24L01 VCC
     D12|← NRF24L01 MISO            |
     D13|→ NRF24L01 SCK             |
         +---------------------------+

Joystick Connections:
  Left Joystick (Throttle/Yaw):
    VCC → 5V
    GND → GND
    VRy → A0 (Throttle - vertical)
    VRx → A1 (Yaw - horizontal)
    SW  → (optional button, unused)
  
  Right Joystick (Pitch/Roll):
    VCC → 5V
    GND → GND
    VRy → A2 (Pitch - vertical)
    VRx → A3 (Roll - horizontal)
    SW  → (optional button, unused)

Buttons/Switches (INPUT_PULLUP - connect to GND when active):
  Button 1 (Calibration)   → D4
  Button 2 (Motor Start)   → D5
  Switch 1 (Arm/Disarm)    → D3 (HIGH=Disarmed, LOW=Armed)
  Switch 2 (Altitude Hold) → D2 (LOW=Active)
```

## 📚 Required Libraries

Install these libraries via Arduino IDE Library Manager:

1. **RF24** by TMRh20 - NRF24L01 radio communication
2. **Smoothed** by Matthew Fryer - Value smoothing/filtering
3. **MS5611** by Rob Tillaart - Barometer library

The `Gyro.h` and `Gyro.cpp` files are included in the project.

## ⚙️ Configuration

### Joystick Calibration (Remote Controller)

Edit these values in `RemoteController.ino` to match your joysticks:

```cpp
// Center values (when joystick is at rest)
float calRoll     = -512;   // Right X center
float calPitch    = -507;   // Right Y center
float calYaw      = -512;   // Left X center
float calThrottle = -500;   // Left Y minimum

// Adjust scaling if controls feel too sensitive/sluggish
float scaleRoll     = 0.1;
float scalePitch    = -0.1;
float scaleYaw      = -0.1;
float scaleThrottle = 1.5;
```

### PID Tuning (Flight Controller)

Adjust in `FlightController.ino`:

```cpp
// Attitude PID (roll/pitch)
const float KP = 2.0;
const float KI = 0.0001;
const float KD = 0.5;
const float KP_Z = 2.0;  // Yaw

// Altitude PID
float pidAltP = 14.0;
float pidAltI = 2.0;
float pidAltD = 7.5;
```

### Safety Limits

```cpp
const int THRUST_MAX = 1700;        // Maximum throttle PWM
const float MAX_TILT_ANGLE = 30.0;  // Safety cutoff angle (degrees)
const float COMM_TIMEOUT = 3.0;     // Seconds before emergency stop
```

## 🎮 Control Layout

```
LEFT JOYSTICK                    RIGHT JOYSTICK
=============                    ==============
     UP                               UP
      ↑                                ↑
      │ Increase                       │ Pitch Forward
      │ Throttle                       │ (Fly Forward)
      │                                │
LEFT ←┼→ RIGHT                  LEFT ←┼→ RIGHT
  Yaw │ Yaw                     Roll  │ Roll
  CCW │ CW                      Left  │ Right
      │                                │
      ↓ Decrease                       ↓ Pitch Back
      │ Throttle                       │ (Fly Backward)
     DOWN                            DOWN
```

## 📖 User Workflow

### First-Time Setup

1. **Upload Code**
   - Upload `FlightController.ino` to the FC Arduino
   - Upload `RemoteController.ino` to the RC Arduino

2. **Connect Hardware**
   - Wire everything according to the diagrams above
   - Ensure all ESCs are connected to a common ground
   - **DO NOT** connect propellers yet!

3. **ESC Calibration** (if needed)
   - Most ESCs need throttle range calibration
   - Consult your ESC documentation

### Normal Operation

1. **Power ON the Remote Controller first**
   - Wait for startup beep

2. **Ensure Arm Switch is in DISARMED position** (HIGH)
   - LED on FC stays ON continuously when disarmed

3. **Power ON the Flight Controller**
   - Wait for startup beep sequence
   - Connection confirmed by additional beep when NRF links

4. **Calibrate Sensors** (recommended before each flight)
   - Place drone on flat, level surface
   - Ensure Arm Switch = DISARMED
   - Press and HOLD Calibration Button (A1) for 2+ seconds
   - Wait for beep + LED flash confirmation

5. **Arm the System**
   - Move Arm Switch to ARMED position (LOW)
   - LED starts blinking

6. **Smooth Motor Start**
   - Press Motor Start Button (A2)
   - Motors will ramp up gradually
   - Verify all motors spin correctly
   - Beep confirms ready state

7. **Fly!**
   - Use joysticks to control
   - Throttle up to take off
   - Toggle Altitude Hold Switch for automatic altitude maintenance

8. **Landing**
   - Reduce throttle to descend
   - Once landed, move Arm Switch to DISARMED
   - Motors stop immediately

### Altitude Hold Mode

When Altitude Hold Switch (D2) is active AND throttle is between 1400-1450:
- Drone maintains current altitude automatically
- Throttle stick now adjusts altitude setpoint
- Moving throttle above 1450 = climb
- Moving throttle below 1400 = descend

## 🛡️ Safety Features

| Feature | Description |
|---------|-------------|
| **Max Tilt Protection** | Automatic motor cutoff if tilt exceeds 30° |
| **Communication Timeout** | Emergency stop after 3 seconds of no signal |
| **Arm Switch** | Physical switch prevents accidental startup |
| **Disarm Indicator** | LED stays ON when disarmed as warning |
| **Smooth Motor Start** | Gradual motor ramp-up to verify operation |

## 🔧 Troubleshooting

### No Communication Between RC and FC

1. Verify both are on same channel (default: 108)
2. Check NRF24L01 wiring, especially VCC (must be 3.3V!)
3. Add 10-100µF capacitor between VCC and GND on NRF modules
4. Try lower power level if too close: `RF24_PA_MIN`

### Unstable Flight / Oscillations

1. Reduce PID gains (especially P and D)
2. Ensure calibration was done on level surface
3. Check for vibration issues on MPU6050 mounting
4. Verify motor/prop balance

### Motors Don't Spin

1. Verify Arm Switch is in ARMED position
2. Check ESC signal connections
3. Ensure ESCs are calibrated
4. Verify battery is connected and charged

### Altitude Hold Drift

1. Allow MS5611 to warm up (1-2 minutes)
2. Recalibrate on ground before flight
3. Adjust altitude PID gains
4. Shield MS5611 from prop wash

## 📊 Motor Configuration (X Quadcopter)

```
        FRONT
    FL ↺     ↻ FR
      \     /
       \   /
        \ /
         X
        / \
       /   \
      /     \
    RL ↻     ↺ RR
        BACK

↺ = Counter-Clockwise (CCW)
↻ = Clockwise (CW)

Motor Mixing:
FL = Thrust - Roll + Pitch - Yaw
FR = Thrust + Roll + Pitch + Yaw
RL = Thrust - Roll - Pitch + Yaw
RR = Thrust + Roll - Pitch - Yaw
```

## 📝 License

This project is open source. Use at your own risk. Always follow local regulations for drone operation and never fly over people or restricted areas.

## ⚠️ Disclaimer

Building and flying drones involves inherent risks. Ensure you:
- Test without propellers first
- Fly in open areas away from people
- Follow all local aviation regulations
- Use appropriate safety equipment
- Have adequate insurance if required
