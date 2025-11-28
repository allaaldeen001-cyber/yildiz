# Arduino Quadcopter Drone Project

A complete quadcopter drone system with Arduino Nano-based flight controller and remote control.

## Features

- **Stable Flight Control**: PID-based stabilization using MPU6050 IMU
- **Altitude Hold**: Barometric pressure sensor (MS5611) for altitude measurement
- **Wireless Control**: NRF24L01 2.4GHz radio communication
- **Safety Features**: 
  - Kill switch/arming toggle
  - Radio connection monitoring
  - Automatic kill on radio loss
- **Calibration System**: 
  - IMU calibration
  - Barometric sensor calibration
  - ESC calibration
  - Joystick calibration
- **User Guidance**: Serial monitor step-by-step setup instructions
- **Flight Data Display**: Real-time telemetry (throttle, pitch, roll, yaw, altitude)

## Hardware Components

### Flight Controller
- Arduino Nano
- NRF24L01 (Radio Module)
- MPU6050 (IMU - Gyroscope + Accelerometer)
- MS5611 (Barometric Pressure Sensor)
- 4x ESC (Electronic Speed Controllers)
- Buzzer
- Status LED

### Remote Control
- Arduino Nano
- NRF24L01 (Radio Module)
- 2x Joystick (Analog)
- Toggle Switch (Arming/Kill)
- 2x Push Button
- Status LED

## Control Scheme

### Left Joystick
- **Up/Down**: Throttle (Altitude control)
- **Left/Right**: Yaw (Rotation)

### Right Joystick
- **Up/Down**: Pitch (Forward/Backward)
- **Left/Right**: Roll (Left/Right strafe)

### Buttons
- **Button 1**: Calibration (IMU, MS5611, ESC)
- **Button 2**: Smooth motor start test

### Toggle Switch
- **OFF Position**: Kill switch (safety mode)
- **ON Position**: Armed (ready to fly)

## Setup Instructions

1. **Wire the Hardware**
   - Follow the wiring guide in `WIRING_GUIDE.md`
   - Double-check all connections, especially NRF24L01 (3.3V only!)

2. **Install Libraries**
   - Open Arduino IDE
   - Install required libraries (see WIRING_GUIDE.md)

3. **Upload Code**
   - Upload `FC_Quadcopter.ino` to Flight Controller Arduino
   - Upload `RC_Quadcopter.ino` to Remote Control Arduino

4. **Initial Setup Sequence**
   - Power on Remote Control first
   - Power on Flight Controller
   - Open Serial Monitor (9600 baud) on Flight Controller
   - Follow the step-by-step instructions shown in Serial Monitor

5. **Calibration Process**
   - Keep drone level and still
   - Press Button 1 on remote control
   - Wait for calibration to complete

6. **Pre-Flight Checks**
   - Verify kill switch works (toggle OFF)
   - Arm drone (toggle ON)
   - Press Button 2 for motor test (propellers removed!)
   - Check all motors spin smoothly

7. **First Flight**
   - Attach propellers (correct rotation!)
   - Arm drone
   - Start with low throttle
   - Make small adjustments

## Safety Warnings

⚠️ **IMPORTANT SAFETY NOTES:**

1. **Always remove propellers** during initial testing and calibration
2. **Test kill switch** before first flight
3. **Start with low throttle** on first flights
4. **Keep drone level** during calibration
5. **Check motor rotation** before attaching propellers
6. **Use proper battery** with appropriate voltage regulator
7. **NRF24L01 requires 3.3V** - use voltage regulator!

## Joystick Center Point Fix

The code fixes the dangerous joystick center point issue:
- **Old (Dangerous)**: Center = 1000, Max = 2000
- **New (Safe)**: Center = 1500, Range = 1000-2000

This ensures the drone doesn't have power when joysticks are centered.

## PID Tuning

Default PID values are set for basic stability. You may need to tune these for your specific drone:

```cpp
// In FC_Quadcopter.ino
float Kp_pitch = 1.5, Ki_pitch = 0.05, Kd_pitch = 0.3;
float Kp_roll = 1.5, Ki_roll = 0.05, Kd_roll = 0.3;
float Kp_yaw = 1.0, Ki_yaw = 0.01, Kd_yaw = 0.2;
```

Tuning tips:
- Increase **Kp** if drone responds too slowly
- Increase **Kd** if drone oscillates
- Increase **Ki** if drone drifts

## Serial Monitor Output

The Flight Controller provides:
- Setup guidance (step-by-step)
- Connection status
- Calibration progress
- Flight data (throttle, yaw, pitch, roll, altitude, angles)

Example flight data:
```
T:1500 | Y:1500 | P:1500 | R:1500 | Alt:0.00m | Ch:76 | Ang:0.5/-0.3/12.1
```

## Troubleshooting

See `WIRING_GUIDE.md` for detailed troubleshooting tips.

Common issues:
- **No NRF connection**: Check 3.3V power, SPI connections
- **IMU errors**: Verify I2C connections, check pull-up resistors
- **Unstable flight**: Recalibrate, check PID values, verify motor directions

## File Structure

```
.
├── FC_Quadcopter.ino      # Flight Controller code
├── RC_Quadcopter.ino      # Remote Control code
├── WIRING_GUIDE.md        # Detailed wiring instructions
└── README.md              # This file
```

## License

This project is provided as-is for educational purposes. Use at your own risk.

## Credits

Designed for Arduino-based quadcopter systems with professional embedded system practices.
