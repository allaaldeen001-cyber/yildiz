# DIY Drone Flight Control System

A complete Arduino-based quadcopter flight control system with NRF24L01 wireless communication, MPU6050 gyroscope, MS5611 barometric pressure sensor, and PID-based flight stabilization.

## 🚁 Features

- **Wireless Control**: NRF24L01 2.4GHz radio communication with ACK confirmation
- **Flight Stabilization**: PID controller for roll, pitch, and yaw
- **Altitude Hold**: MS5611 barometric pressure sensor with Kalman filtering
- **Safety Features**: 
  - Maximum tilt angle limit (30°)
  - No-data timeout protection
  - Arming/disarming system
  - Smooth motor start
- **Visual Feedback**: LED status indicators and buzzer alerts

## 📁 Project Structure

```
.
├── FC/                          # Flight Controller code
│   ├── Drone_Flight_control.ino # Main FC program
│   ├── Barometer.ino            # Altitude hold logic
│   ├── kalman_filter.ino        # Kalman filter for altitude
│   ├── Gyro.h                   # Gyroscope class header
│   └── Gyro.cpp                 # Gyroscope class implementation
├── RC/                          # Remote Controller code
│   └── controller.ino           # RC transmitter program
├── SETUP_INSTRUCTIONS.md        # Detailed setup guide
└── README.md                    # This file
```

## 🔌 Quick Pin Reference

### Flight Controller
| Component | Pin | Description |
|-----------|-----|-------------|
| NRF24L01 CE | D4 | Radio chip enable |
| NRF24L01 CSN | D10 | Radio chip select |
| Motor FL | D3 | Front Left ESC |
| Motor FR | D5 | Front Right ESC |
| Motor RR | D6 | Rear Right ESC |
| Motor RL | D9 | Rear Left ESC |
| Calibration Button | A1 | Sensor calibration |
| Smooth Start Button | A2 | Smooth motor ramp-up |
| Arm Switch | A3 | Arm/Disarm (HIGH=disarmed) |
| Altitude Hold Switch | D2 | Altitude hold toggle |
| LED | D7 | Status indicator |
| Buzzer | D8 | Audio feedback |
| MPU6050 | A4/A5 | I2C (SDA/SCL) |
| MS5611 | A4/A5 | I2C (SDA/SCL) |

### Remote Controller
| Component | Pin | Description |
|-----------|-----|-------------|
| NRF24L01 CE | D9 | Radio chip enable |
| NRF24L01 CSN | D10 | Radio chip select |
| Throttle | A0 | Left stick Y-axis |
| Yaw | A1 | Left stick X-axis |
| Pitch | A2 | Right stick Y-axis |
| Roll | A3 | Right stick X-axis |

## ⚡ Quick Start

1. **Install Libraries**:
   - RF24 by TMRh20
   - Smoothed by Sofian Audry
   - MS5611 library

2. **Upload FC Code**:
   - Open `FC/Drone_Flight_control.ino` in Arduino IDE
   - Upload to Flight Controller Arduino Nano

3. **Upload RC Code**:
   - Open `RC/controller.ino` in Arduino IDE
   - Upload to Remote Controller Arduino Nano

4. **Power On**:
   - Power RC first, then FC
   - Wait for 3 beeps (NRF link confirmed)

5. **Calibrate**:
   - Ensure disarmed (LED ON)
   - Press Calibration Button (A1) for 2+ seconds
   - Wait for confirmation beeps

6. **Arm & Fly**:
   - Set Arm Switch (A3) to LOW (armed)
   - Press Smooth Start Button (A2) to test motors
   - Use joysticks to control

## 📖 Documentation

See [SETUP_INSTRUCTIONS.md](SETUP_INSTRUCTIONS.md) for:
- Detailed wiring diagrams
- Calibration procedures
- Troubleshooting guide
- Safety checklist
- PID tuning guide

## ⚙️ Configuration

### PID Gains (in `Drone_Flight_control.ino`)
```cpp
const float kp = 2;        // Proportional gain
const float ki = 0.0001;   // Integral gain
const float kd = 0.5;      // Derivative gain
const float kpZ = 2;      // Yaw gain
```

### Altitude Hold PID (in `Drone_Flight_control.ino`)
```cpp
float pid_p_gain_altitude = 14.0;
float pid_i_gain_altitude = 2.0;
float pid_d_gain_altitude = 7.5;
```

### Control Sensitivity (in `Drone_Flight_control.ino`)
```cpp
float sensiX = -0.45;      // Roll sensitivity
float sensiY = 0.45;       // Pitch sensitivity
float sensiZ = -0.01;      // Yaw sensitivity
float sensiThrust = 1.1;   // Throttle sensitivity
```

## 🛡️ Safety Features

- **30° Maximum Tilt**: Automatic motor kill if exceeded
- **No Data Timeout**: Motors stop after 3 seconds of lost signal
- **Arming Required**: Motors disabled until armed
- **Thrust Limiting**: Maximum thrust capped at 1700
- **Visual Warnings**: LED indicates system status

## 🔧 Troubleshooting

**NRF Not Connecting?**
- Verify 3.3V power (not 5V!)
- Check antenna connections
- Ensure same pipe address on both devices

**Motors Not Spinning?**
- Check ESC connections
- Verify arming status
- Test with Smooth Start button

**Gyro Drift?**
- Recalibrate sensors
- Ensure level surface during calibration

See [SETUP_INSTRUCTIONS.md](SETUP_INSTRUCTIONS.md) for detailed troubleshooting.

## 📝 License

This project is provided as-is for educational and DIY purposes.

## ⚠️ Disclaimer

**WARNING**: Always test without propellers first! This is experimental software. Use at your own risk. Ensure all safety checks pass before flying.

## 🤝 Contributing

Improvements and bug fixes are welcome! Please test thoroughly before submitting changes.

---

**Happy Flying! 🚁**
