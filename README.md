# Professional Arduino Nano Drone System

A professional-grade quadcopter flight control system built with Arduino Nano, featuring robust PID control, wireless communication, and comprehensive safety features.

## 📋 System Overview

This project implements a complete drone system with two main components:
- **Flight Controller (FC)**: Manages flight stabilization, motor control, and sensor fusion
- **Remote Controller (RC)**: Wireless transmitter for pilot control and system monitoring

## 🔧 Hardware Components

### Flight Controller Board
| Component | Pin Assignment | Description |
|-----------|---------------|-------------|
| Arduino Nano | - | Main processor |
| NRF24L01 PA+LNA | CE: D4, CSN: D10 | Wireless transceiver |
| MPU6050 | INT: D2, I2C: A4(SDA)/A5(SCL) | 6-axis IMU |
| Buzzer | D8 | Audio feedback |
| Status LED | D7 | Visual indicators |
| Motor FL (Front-Left) | D3 (PWM) | ESC signal |
| Motor FR (Front-Right) | D5 (PWM) | ESC signal |
| Motor RR (Rear-Right) | D6 (PWM) | ESC signal |
| Motor RL (Rear-Left) | D9 (PWM) | ESC signal |

### Remote Controller Board
| Component | Pin Assignment | Description |
|-----------|---------------|-------------|
| Arduino Nano | - | Main processor |
| NRF24L01 PA+LNA | CE: D9, CSN: D10 | Wireless transceiver |
| Left Joystick (Throttle/Yaw) | V: A0, H: A1 | Vertical: Throttle, Horizontal: Yaw |
| Right Joystick (Pitch/Roll) | V: A2, H: A3 | Vertical: Pitch, Horizontal: Roll |
| Calibration Button | D4 | Gyro calibration trigger |
| Motor Arm Button | D5 | ESC/Motor arming |
| Altitude Hold Switch | D2 | Position hold mode |
| Kill Switch | D3 | Emergency disarm |

## 🚁 Flight Controls

### Left Joystick
- **Vertical (A0 - Throttle)**: UP = Climb (increase thrust), DOWN = Descend (decrease thrust)
- **Horizontal (A1 - Yaw)**: LEFT = Rotate CCW, RIGHT = Rotate CW

### Right Joystick
- **Vertical (A2 - Pitch)**: UP = Forward tilt, DOWN = Backward tilt
- **Horizontal (A3 - Roll)**: LEFT = Tilt left, RIGHT = Tilt right

### Switches & Buttons
- **SW_2 (D3)**: Kill Switch - Immediate disarm when set to 0
- **SW_1 (D2)**: Altitude Hold - Enable/disable position hold mode
- **Button_1 (D4)**: Gyro Calibration - Calibrate IMU sensors
- **Button_2 (D5)**: Motor Arming - Enable motor control

## 🔄 Startup Sequence

1. **Power ON Remote Controller** - Wait for initialization
2. **Power ON Flight Controller** - System boot and sensor check
3. **Communication Link** - Status LED (D7) blinks to confirm NRF24L01 connection
4. **Pre-flight Checks**:
   - Ensure Kill Switch (SW_2) is in position "1" (armed position)
   - Ensure Altitude Hold (SW_1) is in position "0" (disabled)
5. **Gyro Calibration**:
   - Press Button_1 (D4)
   - Keep drone on level surface
   - **Success**: 2 short beeps
   - **Failure**: 1 long beep (7 seconds)
6. **ESC Calibration** (first-time setup):
   - Set SW_1 to "0"
   - Press Button_2 (D5)
   - Motors will spin up one by one smoothly
   - Confirmation beep pattern (different from calibration)
7. **Ready to Fly** - Use joysticks to control the drone

## 🛡️ Safety Features

1. **Maximum Angle Limit**: 30° tilt limit prevents aggressive maneuvers
2. **Throttle Cap**: Internal 65% power limit prevents sudden altitude surges
3. **Failsafe Mode**: Auto-disarm on signal loss
4. **Kill Switch**: Instant motor cutoff for emergencies
5. **Low Throttle Disarm**: Prevents accidental arming
6. **Calibration Verification**: Ensures proper sensor initialization

## 📡 Communication Protocol

- **Wireless Protocol**: NRF24L01 with Enhanced ShockBurst™
- **Channel**: 103 (2.503 GHz)
- **Data Rate**: 250kbps for maximum range
- **Power Level**: PA_MAX for PA+LNA modules
- **ACK Packets**: Enabled for reliable bidirectional communication
- **Packet Rate**: 50 Hz (20ms update interval)
- **Payload**: 32 bytes (control data + telemetry)

## 📊 Serial Monitor Output (RC)

The Remote Controller provides real-time feedback via serial monitor (115200 baud):
- Communication status (LINKED / NO SIGNAL)
- Joystick values (Throttle, Yaw, Pitch, Roll)
- Button states (Calibration, Arming)
- Switch positions (Altitude Hold, Kill Switch)
- Calibration results
- Battery voltage (if available)
- Signal strength indicators

**Note**: Serial monitor is optional - the system operates fully autonomously.

## 🎯 PID Tuning Parameters

Default PID values optimized for stability:
- **Roll/Pitch**: Kp=1.3, Ki=0.04, Kd=18.0
- **Yaw**: Kp=2.0, Ki=0.02, Kd=0.0
- **Loop Rate**: 250 Hz for stable control

## 📂 Project Structure

```
/workspace/
├── FlightController/
│   └── FlightController.ino    # FC firmware
├── RemoteController/
│   └── RemoteController.ino    # RC firmware
├── docs/
│   ├── WIRING_DIAGRAM.md       # Connection diagrams
│   ├── CALIBRATION_GUIDE.md    # Setup procedures
│   └── TROUBLESHOOTING.md      # Common issues
└── README.md                    # This file
```

## 🔌 Required Libraries

Install via Arduino IDE Library Manager:
- **NRF24L01**: RF24 by TMRh20
- **MPU6050**: MPU6050 by Electronic Cats or Adafruit MPU6050
- **I2C**: Wire (built-in)

## 🚀 Getting Started

1. Install required libraries in Arduino IDE
2. Upload `FlightController.ino` to FC Arduino Nano
3. Upload `RemoteController.ino` to RC Arduino Nano
4. Connect hardware according to wiring diagrams
5. Follow startup sequence above
6. Calibrate sensors before first flight
7. Test in safe environment

## ⚠️ Important Notes

- **First Flight**: Test in open area with no obstacles
- **Battery**: Use appropriate LiPo battery with voltage monitoring
- **Props**: Install propellers AFTER all testing is complete
- **Calibration**: Required after each power cycle for best performance
- **Failsafe**: Understand kill switch operation before flight

## 📝 License

This project is provided as-is for educational and hobbyist purposes.

## 🙏 Credits

Developed as a professional UAV embedded system following industry best practices.

---
**⚡ Safety First**: Always follow local drone regulations and fly responsibly!
