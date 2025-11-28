# Arduino Quadcopter Drone Project

A complete, professional quadcopter drone system with flight controller and RC transmitter using Arduino Nano.

## 🚁 System Overview

This project implements a fully-functional quadcopter drone with:
- **Stable flight control** using PID algorithms
- **6-axis IMU stabilization** (MPU6050)
- **Barometric altitude hold** (MS5611)
- **Wireless control** via NRF24L01 (2.4GHz)
- **Comprehensive calibration** system
- **Safety features** and failsafes

---

## 📦 Hardware Components

### Flight Controller Board
| Component | Description | Pin Connection |
|-----------|-------------|----------------|
| Arduino Nano | Main microcontroller | - |
| NRF24L01 | 2.4GHz wireless receiver | CE=9, CSN=10, MOSI=11, MISO=12, SCK=13 |
| MPU6050 | IMU (gyro + accel) | SDA=A4, SCL=A5 |
| MS5611 | Barometric pressure sensor | SDA=A4, SCL=A5 |
| 4x ESC | Motor speed controllers | D3, D4, D5, D6 (PWM) |
| Buzzer | Audio feedback | D7 |
| Status LED | Visual indicator | D8 |
| LED | Main indicator | D2 |

### RC Transmitter Board
| Component | Description | Pin Connection |
|-----------|-------------|----------------|
| Arduino Nano | Main microcontroller | - |
| NRF24L01 | 2.4GHz wireless transmitter | CE=9, CSN=10, MOSI=11, MISO=12, SCK=13 |
| Joystick 1 (Left) | Throttle + Yaw | VRx=A0, VRy=A1 |
| Joystick 2 (Right) | Pitch + Roll | VRx=A2, VRy=A3 |
| Toggle Switch | Arm/Kill switch | D2 |
| Push Button 1 | Calibration trigger | D3 (with pullup) |
| Push Button 2 | Motor test | D4 (with pullup) |
| Status LED | Visual feedback | D8 |

---

## 🔌 Wiring Diagrams

### Flight Controller Wiring

```
Arduino Nano (Flight Controller)
┌─────────────────────┐
│  D2  → LED          │
│  D3  → ESC Motor 1  │  (Front-Right)
│  D4  → ESC Motor 2  │  (Rear-Right)
│  D5  → ESC Motor 3  │  (Rear-Left)
│  D6  → ESC Motor 4  │  (Front-Left)
│  D7  → Buzzer       │
│  D8  → Status LED   │
│  D9  → NRF24 CE     │
│  D10 → NRF24 CSN    │
│  D11 → NRF24 MOSI   │
│  D12 → NRF24 MISO   │
│  D13 → NRF24 SCK    │
│  A4  → MPU6050 SDA  │  (also MS5611 SDA)
│  A5  → MPU6050 SCL  │  (also MS5611 SCL)
│  3.3V → NRF24 VCC   │
│  GND → All GND      │
└─────────────────────┘

Motor Configuration (X-Frame):
        FRONT
    M4      M1
      \    /
       \  /
        \/
        /\
       /  \
      /    \
    M3      M2
```

### RC Transmitter Wiring

```
Arduino Nano (RC Transmitter)
┌─────────────────────┐
│  D2  → Toggle Switch│
│  D3  → Button 1     │  (Calibration)
│  D4  → Button 2     │  (Motor Test)
│  D8  → Status LED   │
│  D9  → NRF24 CE     │
│  D10 → NRF24 CSN    │
│  D11 → NRF24 MOSI   │
│  D12 → NRF24 MISO   │
│  D13 → NRF24 SCK    │
│  A0  → Joystick1 X  │  (Throttle)
│  A1  → Joystick1 Y  │  (Yaw)
│  A2  → Joystick2 X  │  (Pitch)
│  A3  → Joystick2 Y  │  (Roll)
│  3.3V → NRF24 VCC   │
│  5V  → Joysticks    │
│  GND → All GND      │
└─────────────────────┘
```

---

## 📚 Required Libraries

Install these libraries via Arduino IDE Library Manager:

1. **RF24** by TMRh20 - NRF24L01 communication
2. **MPU6050** by Electronic Cats - IMU sensor
3. **MS5611** by Rob Tillaart - Barometric sensor
4. **I2Cdev** by Jeff Rowberg - I2C communication helper
5. **Wire** - Built-in I2C library

---

## 🛠️ Installation & Setup

### 1. Hardware Assembly
- Assemble the quadcopter frame with motors at each corner
- Mount the flight controller in the center
- Connect ESCs to motors and flight controller
- Wire all sensors according to the wiring diagram above
- Ensure proper power distribution (11.1V LiPo recommended)

### 2. Software Upload
1. Open Arduino IDE
2. Install required libraries
3. Upload `FlightController/FlightController.ino` to the drone's Arduino
4. Upload `RCTransmitter/RCTransmitter.ino` to the remote's Arduino

### 3. Pre-Flight Calibration (Critical!)
Follow the serial monitor guidance system:

1. **Power on RC Transmitter** - Wait for initialization
2. **Power on Flight Controller** - Wait for NRF connection
3. **Set toggle switch to KILL position** - Disarms the drone
4. **Press Button 1** - Starts calibration sequence:
   - ESC calibration (throttle range)
   - IMU calibration (place drone level!)
   - MS5611 calibration (altitude baseline)
   - Settings saved to EEPROM
5. **Arm the drone** - Toggle switch to ARM position
6. **Test motors** - Press Button 2 for smooth motor test
7. **Ready to fly!** - Follow throttle guidelines

---

## 🎮 Control Guide

### Left Joystick (Throttle & Yaw)
- **Up/Down**: Altitude control (throttle)
  - Up = Increase altitude
  - Down = Decrease altitude
  - **Center position = Hover throttle (~40-60%)**
- **Left/Right**: Rotation (yaw)
  - Left = Rotate counter-clockwise
  - Right = Rotate clockwise

### Right Joystick (Pitch & Roll)
- **Up/Down**: Forward/Backward (pitch)
  - Up = Fly forward
  - Down = Fly backward
  - Center = Hover
- **Left/Right**: Strafe (roll)
  - Left = Fly left
  - Right = Fly right
  - Center = Hover

### Toggle Switch
- **UP** = ARM (enable motors)
- **DOWN** = KILL (disable motors immediately)

### Button 1 (Calibration)
- Initiates full calibration routine
- Must be done with drone level and stationary
- Only press when toggle is in KILL position

### Button 2 (Motor Test)
- Spins motors at low speed for testing
- Safe for bench testing
- Drone must be armed first

---

## 🚦 LED & Buzzer Indicators

### Flight Controller
| Pattern | Meaning |
|---------|---------|
| Buzzer 2 beeps | NRF connected successfully |
| Status LED solid | Armed and ready |
| Status LED slow blink | Disarmed |
| Status LED fast blink | Calibration mode |
| Buzzer continuous | Error/Warning |

### RC Transmitter
| Pattern | Meaning |
|---------|---------|
| LED blink on button press | Command sent successfully |
| LED fast blink | No connection to flight controller |
| LED solid | Connected and transmitting |

---

## 📊 Serial Monitor Data

The flight controller outputs real-time telemetry:

```
=== QUADCOPTER FLIGHT DATA ===
Throttle: 1500 | Yaw: 1500 | Pitch: 1500 | Roll: 1500
Altitude: 1.25m | Climb Rate: 0.02m/s
IMU: Pitch=0.5° Roll=-0.3° Yaw=45.2°
NRF Channel: 108 | Signal: GOOD
Motors: [1450, 1480, 1470, 1460]
Battery: 11.4V
```

---

## ⚠️ Safety Guidelines

### CRITICAL SAFETY RULES
1. **Always remove propellers** during initial testing and calibration
2. **Test throttle response** before attaching propellers
3. **Verify kill switch works** before every flight
4. **Calibrate before first flight** and after any crashes
5. **Keep safe distance** - minimum 10 feet from drone
6. **Check battery voltage** before each flight (min 10.5V)
7. **Fly in open areas** away from people and obstacles

### Pre-Flight Checklist
- [ ] Propellers secure and undamaged
- [ ] Battery fully charged and connected
- [ ] Frame and arms tight with no loose parts
- [ ] All wires secured away from propellers
- [ ] Calibration completed recently
- [ ] Kill switch tested and working
- [ ] Clear flight area

---

## 🔧 Tuning PID Parameters

The flight controller uses PID control for stabilization. Default values are conservative for stability. Advanced users can tune via serial commands or code modification:

### Default PID Values
```cpp
// Roll/Pitch PID
kP = 1.5
kI = 0.05
kD = 18.0

// Yaw PID
kP = 3.0
kI = 0.02
kD = 0.0
```

### Tuning Guide
1. Start with default values
2. If drone oscillates, reduce P gain
3. If drone drifts slowly, increase I gain
4. If drone is sluggish, reduce D gain
5. Test one axis at a time (roll, then pitch, then yaw)

---

## 🐛 Troubleshooting

### NRF24L01 Connection Issues
- Check 3.3V power (not 5V!)
- Add 10µF capacitor across VCC/GND
- Use short wires (<10cm)
- Verify SPI pins connected correctly

### Motors Not Spinning
- Check ESC calibration
- Verify PWM signals (D3-D6)
- Ensure arm switch is in ARM position
- Check serial monitor for error messages

### Drone Not Stable
- Re-calibrate IMU on level surface
- Check motor directions (props spinning correct way)
- Verify motor mounting positions match code
- Tune PID values

### Altitude Drift
- Re-calibrate MS5611 at launch altitude
- Check for air leaks in barometer
- Reduce altitude PID gains

---

## 📈 Advanced Features

### Altitude Hold Mode
- Automatically maintains set altitude using MS5611
- Enable/disable via serial command: `ALTHLD=1`

### Failsafe Mode
- Activates if NRF signal lost for >1 second
- Gradually reduces throttle and attempts landing
- Always lands in last known safe attitude

### Acro Mode (Future)
- Rate mode for advanced pilots
- Full manual control, no auto-leveling
- Enable via serial command: `ACRO=1`

---

## 🔋 Power Requirements

### Flight Controller
- 5V for Arduino (via ESC BEC or separate BEC)
- 3.3V for NRF24L01 (from Arduino regulator)
- ESCs powered by main LiPo (11.1V 3S recommended)

### RC Transmitter
- 5V via USB or battery
- Can use 2S LiPo or 4x AA batteries

### Recommended Battery
- **LiPo 3S 11.1V, 2200mAh, 25C minimum**
- Flight time: 8-12 minutes depending on weight

---

## 📝 License & Credits

This project is open-source and free to use for educational purposes.

**Created by:** Professional Embedded Systems Engineer
**Date:** November 2025
**Version:** 1.0.0

---

## 🤝 Support

For issues, improvements, or questions:
1. Check troubleshooting section first
2. Review serial monitor for error messages
3. Verify all wiring connections
4. Re-run calibration routine

**Happy Flying! 🚁**

---

## ⚡ Quick Start Guide

1. **Upload firmware** to both Arduinos
2. **Wire components** per diagrams
3. **Power on RC**, then **power on drone**
4. **Follow serial monitor** step-by-step instructions
5. **Calibrate** with Button 1 (drone level!)
6. **Test motors** with Button 2 (no props!)
7. **Arm drone** with toggle switch
8. **Gradually increase throttle** and fly!

**Remember: Start with LOW throttle and make small stick movements!**
