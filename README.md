# 🚁 DIY Drone Flight Controller System

A complete Arduino-based quadcopter flight controller with PID stabilization, altitude hold, and wireless control.

## 📦 Project Structure

```
/workspace/
├── Flight_Controller/          # Flight Controller code (Arduino Nano)
│   ├── Drone_Flight_Control.ino   # Main flight controller
│   ├── Barometer.ino              # MS5611 altitude hold
│   ├── Kalman_Filter.ino          # Altitude smoothing
│   ├── Gyro.h                     # MPU6050 interface header
│   └── Gyro.cpp                   # MPU6050 implementation
│
├── Remote_Controller/          # Remote Controller code (Arduino Nano)
│   └── Controller.ino             # RC transmitter
│
├── SETUP_AND_OPERATION_GUIDE.md  # Complete setup instructions
└── README.md                      # This file
```

## ✨ Features

### Flight Control
- ✅ **PID Stabilization** - 3-axis gyroscope-based stabilization
- ✅ **Altitude Hold** - Barometric pressure-based altitude lock
- ✅ **Wireless Control** - NRF24L01 2.4GHz communication
- ✅ **140Hz Loop Rate** - Fast and responsive control

### Safety Systems
- ✅ **30° Max Tilt Angle** - Prevents flip-over crashes
- ✅ **Signal Loss Protection** - Auto-cutoff after 3 seconds
- ✅ **Kill Switch** - Immediate motor stop on critical errors
- ✅ **Arming Interlocks** - Prevents accidental activation

### User Features
- ✅ **Dual Calibration** - MPU6050 + MS5611 combined calibration
- ✅ **Smooth Motor Start** - Pre-flight motor verification
- ✅ **LED Status Indicators** - Visual feedback system
- ✅ **Buzzer Alerts** - Audio feedback for all states
- ✅ **EEPROM Storage** - Calibration persistence

## 🎯 Quick Start

### 1. Hardware Assembly
- See [SETUP_AND_OPERATION_GUIDE.md](SETUP_AND_OPERATION_GUIDE.md#wiring-diagrams) for complete wiring

### 2. Software Upload
```bash
# Install Arduino IDE libraries:
- RF24 (by TMRh20)
- Smoothed (by Matthew Fryer)  
- MS5611 (by Rob Tillaart)

# Upload Flight_Controller/Drone_Flight_Control.ino to FC Arduino
# Upload Remote_Controller/Controller.ino to RC Arduino
```

### 3. First Flight
```
1. Power ON Remote Controller first
2. Power ON Flight Controller → wait for connection beep
3. Press Button 1 (hold 2s) → Full calibration
4. Long-press Button 2 (3s) → ARM
5. Slowly increase throttle → Lift off!
```

## 📖 Documentation

**Complete setup guide:** [SETUP_AND_OPERATION_GUIDE.md](SETUP_AND_OPERATION_GUIDE.md)

Includes:
- Detailed wiring diagrams
- Step-by-step calibration
- Flight procedures
- Control reference
- Troubleshooting guide
- Safety information

## 🔧 Hardware Requirements

### Flight Controller
- Arduino Nano
- MPU6050 (Gyroscope/Accelerometer)
- MS5611 (Barometer)
- NRF24L01 (RF transceiver)
- 4× ESCs (30A minimum)
- 4× Brushless motors
- Buzzer + LED
- 3S LiPo battery

### Remote Controller
- Arduino Nano
- NRF24L01
- 2× Dual-axis joysticks
- 2× Toggle switches
- 2× Push buttons

## 🎮 Control Layout

### Joysticks
```
     LEFT STICK              RIGHT STICK
        ↑                        ↑
   Throttle UP              Pitch Forward
        |                        |
   ←----+----→            ←------+------→
   Yaw  |  Yaw            Roll   |   Roll
      Left Right          Left   |  Right
        |                        |
   Throttle DOWN           Pitch Back
        ↓                        ↓
```

### Buttons & Switches
- **Switch 1 (D3):** Arm/Disarm (0=Armed, 1=Disarmed)
- **Switch 2 (D2):** Altitude Hold
- **Button 1 (D4):** Full Calibration (when disarmed)
- **Button 2 (D5):** Smooth Motor Start (armed) or Long-press to Arm

## 🛡 Safety Features

| Feature | Threshold | Action |
|---------|-----------|--------|
| Max Tilt Angle | 30° | Immediate motor cutoff |
| Signal Loss | 3 seconds | Auto-disarm |
| Low Battery | <10.5V | Warning (add voltage sensor) |
| Armed LED | Always ON | Visual disarm warning |

## 🔍 System Specifications

| Parameter | Value |
|-----------|-------|
| Control Loop Rate | 140 Hz |
| RF Data Rate | 250 kbps |
| RF Channel | 108 |
| Max Thrust | 1700 µs (adjustable) |
| Armed Idle Speed | 1050 µs |
| Altitude Hold Range | 1400-1450 µs throttle |

## 📊 PID Parameters

### Attitude Control
```cpp
kp = 2.0        // Proportional gain
ki = 0.0001     // Integral gain
kd = 0.5        // Derivative gain
kpZ = 2.0       // Yaw proportional gain
```

### Altitude Control
```cpp
pid_p_gain_altitude = 14.0
pid_i_gain_altitude = 2.0
pid_d_gain_altitude = 7.5
pid_max_altitude = 400
```

## 🐛 Troubleshooting

### Common Issues

**No RC Connection:**
- Check NRF24L01 wiring (especially power!)
- Add 10µF capacitor across NRF VCC/GND
- Verify both devices on channel 108

**Drone Drifts:**
- Recalibrate on flat surface
- Check propeller balance
- Verify motor mounting (vibration)

**Immediate Flip on Takeoff:**
- ⚠️ **Check motor rotation directions!** (most common)
- Verify motor position mapping
- Confirm CW/CCW propeller placement

**Altitude Hold Unstable:**
- Fly above 2 meters altitude
- Keep throttle in 1400-1450 range
- Recalibrate MS5611 ground pressure

See full troubleshooting guide in [SETUP_AND_OPERATION_GUIDE.md](SETUP_AND_OPERATION_GUIDE.md#troubleshooting)

## 🔨 Customization

### Adjusting Sensitivity
Edit in `Drone_Flight_Control.ino`:
```cpp
float sensiX = -0.45;      // Roll sensitivity
float sensiY = 0.45;       // Pitch sensitivity
float sensiZ = -0.01;      // Yaw sensitivity
float sensiThrust = 1.1;   // Throttle scaling
```

### Changing Max Angle
```cpp
int maxAngle = 30;  // Maximum tilt angle in degrees
```

### Disabling Kill Switch
```cpp
bool killAngle = false;  // Disable angle-based kill switch
```

## 📝 Serial Monitor

Enable debug output:
```cpp
debugging(true);  // In setup()
```

Baud rate: **57600**

### Key Messages
- `*** RC CONNECTED ***` - Communication established
- `*** ARMED ***` - Motors ready
- `=== CALIBRATION COMPLETE ===` - Cal finished
- `KILL SWITCH: ...` - Emergency stop reason

## ⚠️ Safety Warning

**This is experimental DIY hardware. Use at your own risk!**

- Always test motors WITHOUT propellers first
- Fly in open areas away from people/property
- Follow local drone regulations
- Use proper LiPo safety procedures
- Wear safety goggles during testing
- Have fire extinguisher nearby when charging

## 🔗 Pin Reference

### Flight Controller Pin Map
```
D3  → Front Left Motor ESC
D4  → NRF24L01 CE
D5  → Front Right Motor ESC
D6  → Rear Right Motor ESC
D7  → Status LED (+ 220Ω resistor)
D8  → Buzzer
D9  → Rear Left Motor ESC
D10 → NRF24L01 CSN
D11 → NRF24L01 MOSI
D12 → NRF24L01 MISO
D13 → NRF24L01 SCK
A4  → MPU6050 & MS5611 SDA
A5  → MPU6050 & MS5611 SCL
```

### Remote Controller Pin Map
```
D2  → Switch 2 (Altitude Hold)
D3  → Switch 1 (Arm/Disarm)
D4  → Button 1 (Calibration)
D5  → Button 2 (Motor Start/Arm)
D9  → NRF24L01 CE
D10 → NRF24L01 CSN
D11 → NRF24L01 MOSI
D12 → NRF24L01 MISO
D13 → NRF24L01 SCK
A0  → Left Joystick Y (Throttle)
A1  → Left Joystick X (Yaw)
A2  → Right Joystick Y (Pitch)
A3  → Right Joystick X (Roll)
```

## 🎓 Learning Resources

This project demonstrates:
- PID control theory
- Sensor fusion (gyro + accelerometer)
- Kalman filtering
- Wireless communication protocols
- Real-time embedded systems
- Motor control (PWM/ESC)

## 📜 License

Open source - feel free to modify and improve!

## 🙏 Credits

Based on community drone projects with improvements:
- Enhanced safety features
- Dual calibration system
- Improved altitude hold
- Better user feedback
- Comprehensive documentation

## 📧 Support

For issues and questions:
1. Check [SETUP_AND_OPERATION_GUIDE.md](SETUP_AND_OPERATION_GUIDE.md#troubleshooting)
2. Verify wiring against diagrams
3. Test with Serial Monitor (57600 baud)
4. Ensure all libraries installed correctly

---

**Happy Flying! 🚁**

*Stay safe, fly responsibly, and enjoy your DIY drone!*
