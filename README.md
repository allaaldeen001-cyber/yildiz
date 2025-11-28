# 🚁 DIY Drone Flight Control System

A complete Arduino-based quadcopter flight controller with altitude hold, stabilization, and wireless control.

## ✨ Features

- ✅ **Self-stabilization** using MPU6050 (gyro + accelerometer)
- ✅ **Altitude hold** with MS5611 barometer and Kalman filtering
- ✅ **Reliable wireless control** via NRF24L01 with ACK
- ✅ **Smooth motor start** for safe testing
- ✅ **Safety features**: signal loss protection, auto-disarm, tilt angle limits (30°)
- ✅ **LED status indicators** and buzzer feedback
- ✅ **One-button calibration** for MPU6050 + MS5611
- ✅ **PID control** for stable flight

---

## 🛠 Hardware Requirements

### Flight Controller (FC)
- Arduino Nano
- NRF24L01 radio module
- MPU6050 gyro/accelerometer
- MS5611 barometer
- 4× Brushless motors + ESCs
- Active buzzer
- LED
- 2× Push buttons (calibration, motor start)
- 2× SPDT switches (arm, altitude hold)
- Voltage divider (battery monitor)

### Remote Controller (RC)
- Arduino Nano
- NRF24L01 radio module
- 2× 2-axis analog joysticks
- Optional: Status LED

### Additional Items
- LiPo battery (3S recommended)
- Quadcopter frame
- Propellers
- Wires, connectors, heat shrink

---

## 📁 Project Structure

```
/workspace/
├── Drone_Flight_Control.ino    # Main FC firmware
├── Barometer.ino                # Altitude hold logic
├── Kalman_Filter.ino            # Sensor fusion for altitude
├── Gyro.h                       # MPU6050 interface header
├── Gyro.cpp                     # MPU6050 implementation
├── Controller.ino               # RC transmitter firmware
├── PIN_MAPPING.md               # Hardware pin assignments
├── USER_MANUAL.md               # Complete user guide
└── README.md                    # This file
```

---

## 🚀 Quick Start

### 1. Install Libraries

Open Arduino IDE → Library Manager → Install:
- `RF24` by TMRh20
- `Smoothed` by Matthew Fryer
- `MS5611` by Rob Tillaart

### 2. Wire the Hardware

See **[PIN_MAPPING.md](PIN_MAPPING.md)** for complete wiring diagrams.

**⚠️ CRITICAL:** Add 10µF capacitor across NRF24L01 power pins!

### 3. Upload Firmware

**Flight Controller:**
1. Open `Drone_Flight_Control.ino` in Arduino IDE
2. Ensure `Barometer.ino`, `Kalman_Filter.ino`, `Gyro.h`, `Gyro.cpp` are in the same folder
3. Board: Arduino Nano
4. Upload

**Remote Controller:**
1. Open `Controller.ino`
2. Board: Arduino Nano
3. Upload

### 4. Calibrate

1. Power RC first, then FC
2. Wait for link confirmation beeps
3. Place drone on level surface
4. Press calibration button (A6) for 2 seconds
5. Wait for completion beeps

### 5. Test & Fly

1. Remove propellers
2. Arm the system (switch D4 to LOW)
3. Press smooth start button (A7)
4. Verify all motors spin correctly
5. Disarm, install propellers
6. Follow flight procedures in **[USER_MANUAL.md](USER_MANUAL.md)**

---

## 🎮 Control Layout

### Remote Joysticks

```
LEFT STICK (A0, A1):          RIGHT STICK (A2, A3):
     Throttle                      Pitch
        ↑                            ↑
Yaw ←   ● → Yaw          Roll ←      ● → Roll
        ↓                            ↓
     Throttle                      Pitch
```

### Switches & Buttons
- **D4 Switch**: Arm (LOW) / Disarm (HIGH)
- **D7 Switch**: Altitude Hold (LOW) / Manual (HIGH)
- **A6 Button**: Calibrate sensors (hold 2s, when disarmed)
- **A7 Button**: Smooth motor start (when armed)

---

## 🔔 Status Indicators

### LED (A3)
| Behavior | Meaning |
|----------|---------|
| Always ON | Disarmed (safe) |
| Blinking | Armed & receiving signals |
| OFF | Signal loss |

### Buzzer (D8)
| Pattern | Meaning |
|---------|---------|
| Beep-Beep-Beeeep | Startup |
| Beep-Beep | Link established |
| Beep-pause-Beep | Calibration started |
| Repeating beep | Kill switch active |

---

## ⚙️ Pin Assignments (FC)

### Core Functions
- **D2**: NRF CE
- **D10**: NRF CSN
- **D11-D13**: SPI (NRF)
- **A4-A5**: I2C (MPU6050 + MS5611)

### Motors (PWM)
- **D3**: Front Left
- **D5**: Front Right
- **D6**: Rear Right
- **D9**: Rear Left

### Inputs
- **A6**: Calibration button
- **A7**: Smooth start button
- **D4**: Arm/disarm switch
- **D7**: Altitude hold switch

### Outputs
- **D8**: Buzzer
- **A3**: Status LED
- **A0**: Battery voltage

See **[PIN_MAPPING.md](PIN_MAPPING.md)** for full details.

---

## 🔧 Configuration Options

Edit these in `Drone_Flight_Control.ino`:

```cpp
// PID Tuning
const float kp = 2;           // Proportional gain
const float ki = 0.0001;      // Integral gain
const float kd = 0.5;         // Derivative gain

// Safety Limits
int maxAngle = 30;            // Max tilt angle (degrees)
int maxThrust = 1700;         // Max motor output (1000-2000)

// Control Frequency
float hz = 140;               // Control loop frequency (Hz)

// Sensitivity
float sensiX = -0.45;         // Roll sensitivity
float sensiY = 0.45;          // Pitch sensitivity
float sensiZ = -0.01;         // Yaw sensitivity
```

---

## 🛡️ Safety Features

1. **Auto-disarm on signal loss** (>3 seconds)
2. **Auto-disarm on excessive tilt** (>30°)
3. **Manual kill switch** (arm switch)
4. **Smooth motor start** (verify before flight)
5. **Visual disarm indicator** (LED always on)
6. **Audible warnings** (buzzer patterns)

---

## 📚 Documentation

- **[USER_MANUAL.md](USER_MANUAL.md)** - Complete setup, calibration, and flight guide
- **[PIN_MAPPING.md](PIN_MAPPING.md)** - Hardware wiring diagrams
- Code comments throughout source files

---

## 🐛 Troubleshooting

### No radio link
- Check NRF24L01 wiring and power (3.3V!)
- Add 10µF capacitor if missing
- Verify SPI connections

### Drone drifts
- Recalibrate on level surface
- Check propeller balance
- Verify frame is not bent

### Motors don't start
- Check arm switch position
- Use smooth motor start feature
- Verify ESC calibration

See **[USER_MANUAL.md](USER_MANUAL.md)** for complete troubleshooting guide.

---

## 🎯 Flight Workflow

```
1. Power RC → Power FC
2. Wait for link beeps ✅
3. Calibrate (if needed)
4. Place in open area
5. Remove propellers (for first test)
6. Arm system (D4 = LOW)
7. Smooth motor start (A7 button)
8. Verify all motors spin correctly
9. Disarm
10. Install propellers
11. Arm again
12. Fly!
```

---

## 🔋 Battery Warning

- Monitor voltage via A0 (voltage divider)
- For 3S LiPo:
  - ✅ Full: 12.6V
  - ✅ Normal: 11.1V
  - ⚠️ Low: 10.5V
  - 🚨 Critical: 9.9V - **LAND NOW!**

---

## 📊 Advanced Features

### Altitude Hold
- Uses MS5611 barometer
- Kalman filter for smooth altitude estimation
- PID control maintains set altitude
- Activate with D7 switch
- Works best at 1400-1450 throttle range

### Complementary Filter
- Fuses gyro and accelerometer data
- 99% gyro, 1% accelerometer
- Corrects gyro drift over time
- Stable angle estimation

---

## 🧪 Testing Checklist

Before first flight:
- [ ] All connections secure
- [ ] NRF24L01 has 10µF capacitor
- [ ] Battery fully charged
- [ ] Calibration completed
- [ ] LED indicates link status
- [ ] Buzzer sounds at startup
- [ ] Arm/disarm switch works
- [ ] Smooth motor start tested (no props!)
- [ ] All 4 motors spin correct direction
- [ ] Motor directions verified:
  - FL: CCW, FR: CW
  - RL: CW, RR: CCW
- [ ] Kill switch tested
- [ ] Open flying area prepared

---

## 🎓 Learning & Modification

### Understanding the Code
- **PID loops**: `calculatePID()` in main file
- **Altitude control**: `calculate_pressure()` in Barometer.ino
- **Kalman filter**: `KalmanPosVel()` in Kalman_Filter.ino
- **Sensor fusion**: `calculateAngle()` in Gyro.cpp

### Potential Improvements
- Add GPS for position hold
- Implement return-to-home
- Add telemetry to RC display
- Implement waypoint navigation
- Add FPV camera
- Create mobile app control

---

## ⚠️ Safety Warning

**THIS IS AN EXPERIMENTAL FLYING DEVICE**

- Always fly in open areas away from people
- Never fly near airports or restricted zones
- Follow local drone regulations
- Spinning propellers are dangerous
- LiPo batteries can be hazardous if damaged
- Use at your own risk

---

## 📞 Support

For issues:
1. Check Serial Monitor (57600 baud) for debug output
2. Review USER_MANUAL.md troubleshooting section
3. Verify all wiring against PIN_MAPPING.md
4. Test components individually

---

## 📜 License

This project is provided as-is for educational purposes.

---

## 🙏 Credits

- MPU6050 filtering based on Joop Brokking's YMFC project
- Kalman filter implementation adapted from quadcopter community
- NRF24L01 library by TMRh20

---

## ✈️ Ready to Fly?

Read **[USER_MANUAL.md](USER_MANUAL.md)** for complete instructions!

**Happy flying! 🚁**
