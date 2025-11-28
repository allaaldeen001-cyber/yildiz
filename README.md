# DIY Drone Flight Control System

A complete Arduino-based quadcopter flight control system with remote controller.

## 📁 Project Structure

```
/workspace/
├── FC/                          # Flight Controller code
│   ├── Drone_Flight_control.ino # Main FC program
│   ├── Barometer.ino            # MS5611 altitude hold
│   ├── kalman_filter.ino        # Kalman filter for altitude
│   ├── Gyro.h                   # MPU6050 header
│   └── Gyro.cpp                 # MPU6050 implementation
├── RC/                          # Remote Controller code
│   └── controller.ino           # RC transmitter program
├── SETUP_GUIDE.md               # Detailed setup instructions
└── README.md                    # This file
```

## 🎯 Features

- **6-DOF Flight Control**: Roll, Pitch, Yaw, and Throttle
- **Altitude Hold**: MS5611 barometer-based altitude stabilization
- **Safety Features**: 
  - Maximum tilt angle limit (30°)
  - Kill switch (disarm)
  - No-data protection
  - Smooth motor start
- **Calibration**: MPU6050 and MS5611 calibration
- **LED Status**: Visual feedback for system state
- **Buzzer Alerts**: Audio feedback for operations

## 🚀 Quick Start

1. **Hardware Setup**: Connect components according to `SETUP_GUIDE.md`
2. **Install Libraries**: Install required Arduino libraries (see SETUP_GUIDE.md)
3. **Upload RC Code**: Upload `RC/controller.ino` to RC Arduino
4. **Upload FC Code**: Upload all files in `FC/` folder to FC Arduino
5. **Calibrate**: Follow calibration procedure in SETUP_GUIDE.md
6. **Test**: Remove propellers, arm system, test smooth start

## 📖 Documentation

See `SETUP_GUIDE.md` for:
- Complete hardware connections
- Library installation
- Step-by-step setup
- User workflow
- Configuration parameters
- Troubleshooting guide

## ⚠️ Safety First

- **ALWAYS** remove propellers during testing
- **ALWAYS** test in open area away from people
- **ALWAYS** ensure system is disarmed before handling
- **NEVER** fly near people or obstacles

## 🔧 Hardware Requirements

### Flight Controller:
- Arduino Nano
- MPU6050 (Gyroscope/Accelerometer)
- MS5611 (Barometer)
- NRF24L01 (Radio module)
- 4x ESC + Motors
- Buzzer, LED, Switches, Buttons

### Remote Controller:
- Arduino Nano
- NRF24L01 (Radio module)
- 2x Joysticks (4-axis)

## 📝 License

This project is provided as-is for educational and DIY purposes.

---

For detailed setup instructions, see `SETUP_GUIDE.md`
