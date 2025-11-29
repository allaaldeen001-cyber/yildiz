# Yildiz Drone System

Professional Arduino Nano Quadcopter Flight Controller & Remote Controller

## Overview

This repository contains a complete drone control system including:

- **Flight Controller** - Arduino Nano based with MPU6050 IMU, PID stabilization, and NRF24L01 communication
- **Remote Controller** - Dual joystick transmitter with live telemetry display

## Quick Links

- [📖 Full Documentation](drone_system/README.md)
- [🔧 Wiring Guide](drone_system/docs/WIRING.md)
- [📋 Quick Reference](drone_system/docs/QUICK_REFERENCE.md)

## Project Structure

```
drone_system/
├── flight_controller/     # Flight controller Arduino code
├── remote_controller/     # Remote controller Arduino code
├── shared/               # Common protocol definitions
└── docs/                 # Documentation and wiring guides
```

## Features

- ✅ 6-axis IMU stabilization with complementary filter
- ✅ PID control (Roll, Pitch, Yaw)
- ✅ 250Hz control loop
- ✅ NRF24L01 PA+LNA communication with ACK payloads
- ✅ Real-time telemetry via Serial Monitor
- ✅ Safety features (kill switch, failsafe, angle limits)
- ✅ ESC calibration mode

## Hardware

### Flight Controller
- Arduino Nano
- NRF24L01 PA+LNA (CE:D4, CSN:D10)
- MPU6050 (INT:D2, I2C:A4/A5)
- 4x ESC+Motors (FL:D3, FR:D5, RR:D6, RL:D9)
- Buzzer (D8), LED (D7)

### Remote Controller
- Arduino Nano
- NRF24L01 PA+LNA (CE:D9, CSN:D10)
- 2x Joysticks (A0-A3)
- 2x Buttons (D4, D5)
- 2x Switches (D2, D3)

## Getting Started

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Install RF24 library by TMRh20
3. Upload `flight_controller.ino` to drone's Arduino
4. Upload `remote_controller.ino` to transmitter's Arduino
5. Follow the [flight procedure](drone_system/README.md#-operating-procedure)

## License

MIT License - Open source for educational and hobby use.

---

**⚠️ WARNING: Drones are dangerous! Always remove propellers during testing. Follow local regulations.**
