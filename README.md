# Professional Drone System - Arduino Nano

A professional UAV embedded system with flight controller and remote controller boards using Arduino Nano.

## 🚁 System Overview

This project implements a complete quadcopter drone system with:
- **Flight Controller Board**: Handles stabilization, sensor fusion, motor control
- **Remote Controller Board**: Transmits pilot commands and displays telemetry
- **Bidirectional NRF24L01 Communication**: Reliable ACK-based data transmission

## 📋 Hardware Components

### Flight Controller Board
| Component | Pin | Description |
|-----------|-----|-------------|
| Arduino Nano | - | Main processor |
| NRF24L01 PA+LNA | CE: D4, CSN: D10 | 2.4GHz transceiver |
| MPU6050 | INT: D2, I2C: A4/A5 | 6-axis IMU |
| MS5611 | I2C: A4/A5 | Barometric pressure sensor |
| Buzzer | D8 | Audio feedback |
| Status LED | D7 | System status indicator |
| Motor FL (Front Left) | D3 | PWM output |
| Motor FR (Front Right) | D5 | PWM output |
| Motor RR (Rear Right) | D6 | PWM output |
| Motor RL (Rear Left) | D9 | PWM output |

### Remote Controller Board
| Component | Pin | Description |
|-----------|-----|-------------|
| Arduino Nano | - | Main processor |
| NRF24L01 PA+LNA | CE: D9, CSN: D10 | 2.4GHz transceiver |
| Left Joystick V | A0 | Throttle |
| Left Joystick H | A1 | Yaw |
| Right Joystick V | A2 | Pitch |
| Right Joystick H | A3 | Roll |
| Button 1 | D4 | Gyro Calibration |
| Button 2 | D5 | ESC Calibration |
| Switch 1 | D2 | Altitude Hold |
| Switch 2 | D3 | Arm/Disarm |

## 🎮 Control Mapping

### Left Joystick
- **Vertical (A0 - Throttle)**: UP = Climb, DOWN = Descend
- **Horizontal (A1 - Yaw)**: LEFT = Rotate CCW, RIGHT = Rotate CW

### Right Joystick
- **Vertical (A2 - Pitch)**: UP = Forward, DOWN = Backward
- **Horizontal (A3 - Roll)**: LEFT = Slide Left, RIGHT = Slide Right

### Switches & Buttons
- **Switch 2 (D3)**: Arm/Disarm kill switch
- **Switch 1 (D2)**: Altitude hold mode
- **Button 1 (D4)**: Gyro calibration
- **Button 2 (D5)**: ESC calibration

## 🚀 Operation Procedure

1. **Power On RC**: Turn on remote controller first
2. **Power On FC**: Turn on flight controller
3. **Check Connection**: LED (D7) blinks when NRF link established
4. **Set Arming Switch**: Ensure SW_2 is at position "1"
5. **Gyro Calibration**: Press Button_1
   - Success: Buzzer beeps twice
   - Failure: Buzzer beeps continuously for 7 seconds
6. **ESC Calibration**: Set SW_1 to "0", press Button_2
   - Motors spin one by one smoothly
   - Buzzer confirms with distinct sound pattern
7. **Fly**: Use joysticks to control the drone

## 🛡️ Safety Features

- **Max Angle Limit**: 30° maximum tilt (prevents aggressive maneuvers)
- **Throttle Cap**: 65% maximum power (prevents sudden jumps)
- **Failsafe**: Auto-disarm on signal loss
- **Kill Switch**: Immediate motor shutdown via SW_2
- **Pre-flight Checks**: Mandatory calibration before arming

## 📡 Communication Protocol

- **Channel**: 103
- **Data Rate**: 250kbps
- **ACK Enabled**: Yes (ensures reliable data transmission)
- **Packet Rate**: 50Hz (20ms interval)
- **Telemetry**: Bidirectional (RC ← → FC)

## 📊 Features

### Flight Controller
- ✅ Multi-loop PID control (Roll, Pitch, Yaw)
- ✅ Complementary filter for sensor fusion
- ✅ Barometric altitude hold
- ✅ Low-pass filtering for smooth control
- ✅ Automatic failsafe and disarm
- ✅ Pre-flight calibration routines
- ✅ Real-time telemetry transmission

### Remote Controller
- ✅ Joystick input with dead zones
- ✅ Button debouncing
- ✅ Serial monitor display of all parameters
- ✅ Connection status monitoring
- ✅ Telemetry reception and display
- ✅ One-button calibration commands

## 📦 Required Libraries

Install these libraries via Arduino IDE Library Manager:

```
- RF24 (NRF24L01 communication)
- Wire (I2C communication)
- MPU6050_light or Adafruit_MPU6050
- MS5611 (Barometric sensor)
- Servo (ESC control)
```

## 🔧 Installation

### Quick Installation (5 minutes)

1. **Install Arduino IDE** (version 1.8.13 or newer)
2. **Install Required Libraries**:
   - RF24 (by TMRh20)
   - Wire, SPI, Servo (built-in)
   - See [LIBRARIES_INSTALLATION.md](docs/LIBRARIES_INSTALLATION.md) for details

3. **Upload Code**:
   - Upload `FlightController/FlightController.ino` to FC Arduino Nano
   - Upload `RemoteController/RemoteController.ino` to RC Arduino Nano

4. **Wire Components**: Follow [WIRING_GUIDE.md](docs/WIRING_GUIDE.md)

5. **First Flight**: See [QUICK_START.md](docs/QUICK_START.md)

### 📚 Complete Documentation

| Document | Description |
|----------|-------------|
| [QUICK_START.md](docs/QUICK_START.md) | 15-minute guide to first flight |
| [WIRING_GUIDE.md](docs/WIRING_GUIDE.md) | Complete wiring instructions |
| [OPERATION_MANUAL.md](docs/OPERATION_MANUAL.md) | Detailed operation procedures |
| [PID_TUNING_GUIDE.md](docs/PID_TUNING_GUIDE.md) | Performance optimization |
| [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Problem-solving guide |
| [LIBRARIES_INSTALLATION.md](docs/LIBRARIES_INSTALLATION.md) | Software setup |
| [BILL_OF_MATERIALS.md](docs/BILL_OF_MATERIALS.md) | Parts list & pricing |
| [PIN_MAPPING.md](docs/PIN_MAPPING.md) | Pin assignment reference |
| [ADVANCED_FEATURES.md](docs/ADVANCED_FEATURES.md) | Future enhancements |
| [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) | Complete project overview |

## 📝 Serial Monitor (RC Board)

The remote controller provides real-time feedback via serial monitor (115200 baud):

```
=== DRONE REMOTE CONTROLLER ===
Link: CONNECTED
Armed: NO
Throttle: 1000 | Yaw: 0
Pitch: 0 | Roll: 0
SW1 (AltHold): OFF | SW2 (Arm): OFF
BTN1 (Cal): Released | BTN2 (ESC): Released
Battery: 11.2V | Altitude: 0.0m
=====================================
```

## ⚠️ Important Notes

- Always perform gyro calibration on level surface
- Keep drone on flat ground during calibration
- Test all controls before first flight
- Start with low throttle and gradual movements
- Never fly near people or obstacles during testing

## 🎯 PID Tuning

Default PID values are conservative for safety. Tune for your specific setup:

```cpp
// In FlightController.ino
#define KP_ROLL   1.5
#define KI_ROLL   0.05
#define KD_ROLL   15.0
// Adjust based on flight characteristics
```

## 📄 License

This project is open-source. Use at your own risk.

## ⚠️ Disclaimer

This is an experimental drone system. Always follow local regulations and safety guidelines when operating UAVs.

---
**Built with ❤️ for the maker community**
