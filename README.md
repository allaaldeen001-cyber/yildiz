# DIY Drone Flight Control System

Complete Arduino-based drone flight control system with NRF24L01 radio communication, MPU6050 gyroscope, MS5611 barometer, and joystick-based RC controller.

## 📋 System Overview

### Flight Controller (FC) Hardware
- **Microcontroller**: Arduino Nano
- **Radio**: NRF24L01 (CE: D4, CSN: D10)
- **IMU**: MPU6050 (I2C)
- **Barometer**: MS5611 (I2C, Address: 0x77)
- **Motors (ESC pins)**:
  - D3 = Front Left (FL)
  - D5 = Front Right (FR)
  - D6 = Rear Right (RR)
  - D9 = Rear Left (RL)
- **Inputs**:
  - A1 = Calibration Button (local FC button)
  - A2 = Smooth Motor-Start Button (local FC button)
  - **Note**: Arm/Disarm and Altitude-Hold switches are on RC controller (sent via radio)
  - **Note**: D4 is used for radio CE pin, D5 is used for Front Right motor ESC
- **Outputs**:
  - D8 = Buzzer
  - D7 = Status LED

### Remote Controller (RC) Hardware
- **Microcontroller**: Arduino Nano
- **Radio**: NRF24L01 (CE: D9, CSN: D10)
- **Joysticks**:
  - A0 = Throttle (Left: Up/Down)
  - A1 = Yaw (Left: Left/Right)
  - A2 = Pitch (Right: Up/Down)
  - A3 = Roll (Right: Left/Right)
- **Inputs**:
  - D4 = Button 1
  - D5 = Button 2 (Arming)
  - D3 = Switch 1 (Arm/Disarm: 1=Disarmed, 0=Armed)
  - D2 = Switch 2 (Altitude Hold)

## 🎮 Control Logic

### Joystick Functions
- **Throttle (A0)**: UP → Increase thrust (up to 2000), DOWN → Decrease thrust (down to 1000)
- **Yaw (A1)**: LEFT → Rotate CCW, RIGHT → Rotate CW
- **Pitch (A2)**: UP → Tilt forward (fly forward), DOWN → Tilt backward
- **Roll (A3)**: LEFT → Tilt left, RIGHT → Tilt right

## 🛠️ Required Libraries

Install the following Arduino libraries:
1. **Servo** (Built-in)
2. **SPI** (Built-in)
3. **Wire** (Built-in)
4. **EEPROM** (Built-in)
5. **Smoothed** - Install from Arduino Library Manager
6. **RF24** - Install from Arduino Library Manager
7. **nRF24L01** - Usually comes with RF24 library
8. **MS5611** - Install from Arduino Library Manager or use: https://github.com/jarzebski/Arduino-MS5611

## 📦 File Structure

```
/workspace/
├── FC/
│   ├── Drone_Flight_control.ino  (Main FC code)
│   ├── Gyro.h                    (Gyroscope header)
│   └── Gyro.cpp                  (Gyroscope implementation)
├── RC/
│   └── controller.ino            (RC controller code)
└── README.md                     (This file)
```

## 🔧 Setup Instructions

### 1. Hardware Connections

#### Flight Controller:
- Connect NRF24L01: CE→D4, CSN→D10, MOSI→D11, MISO→D12, SCK→D13
- Connect MPU6050: SDA→A4, SCL→A5, VCC→5V, GND→GND
- Connect MS5611: SDA→A4, SCL→A5, VCC→5V, GND→GND
- Connect ESCs to pins D3, D5, D6, D9
- Connect Buzzer to D8
- Connect LED to D7
- Connect buttons/switches as specified

#### Remote Controller:
- Connect NRF24L01: CE→D9, CSN→D10, MOSI→D11, MISO→D12, SCK→D13
- Connect joysticks to analog pins A0-A3
- Connect buttons/switches to digital pins D2-D5

### 2. Software Setup

1. **Install Arduino IDE** (version 1.8.x or later)
2. **Install Required Libraries**:
   - Open Arduino IDE
   - Go to Sketch → Include Library → Manage Libraries
   - Search and install: "Smoothed", "RF24", "MS5611"
3. **Upload Code**:
   - Open `FC/Drone_Flight_control.ino` in Arduino IDE
   - Select Board: Arduino Nano
   - Select correct COM port
   - Upload to FC board
   - Repeat for `RC/controller.ino` on RC board

### 3. Calibration

1. **Power ON**: RC first, then FC
2. **Wait for link confirmation**: Buzzer beeps twice when radio link is established
3. **Calibrate**:
   - On RC: Set Switch 1 (D3) to Disarmed position (1) - FC LED should be ON
   - On FC: Press Button A1 (Calibration) and hold for 2 seconds
   - Wait for beep + LED flash confirmation
   - System calibrates both MPU6050 and MS5611

### 4. Arming and Flight

1. **Arm the system**:
   - On RC: Set Switch 1 (D3) to Armed position (0)
   - On RC: Press and hold Button 2 (D5) for 2 seconds to arm
   - FC LED will blink when armed

2. **Smooth Motor Start**:
   - On FC: Press Button A2 (Smooth Motor Start) when armed
   - Motors will ramp up smoothly
   - Verify all motors spin correctly

3. **Flight**:
   - Use joysticks to control the drone
   - On RC: Switch 2 (D2) enables altitude-hold mode
   - Maximum tilt angle is limited to 30° for safety

## ⚠️ Safety Features

- **Maximum Tilt Angle**: 30° limit prevents dangerous angles
- **Kill Switch**: Automatic shutdown if tilt exceeds limits
- **No Data Timeout**: Motors stop if radio link is lost for >3 seconds
- **Disarm Protection**: LED stays ON when disarmed to warn user
- **Calibration Protection**: Can only calibrate when disarmed

## 🔍 LED Behavior

- **ON continuously**: System is disarmed (D3 = 1)
- **Blinking**: RC signal received and system is armed
- **OFF**: No RC signal or system error

## 📊 Serial Monitor

Open Serial Monitor at 57600 baud to view:
- Pressure readings
- Armed status
- Angle measurements (X, Y)
- Debug information

## 🐛 Troubleshooting

### Radio Link Issues:
- Ensure both boards are powered
- Check NRF24L01 connections
- Verify CE and CSN pins are correct
- Try different power levels in code (RF24_PA_LOW/MED/HIGH)

### Calibration Issues:
- Ensure system is disarmed (D3 = 1)
- Keep drone level during calibration
- Wait for confirmation beeps

### Motor Issues:
- Check ESC connections
- Verify motor pin assignments
- Ensure ESCs are calibrated separately
- Check battery voltage

### Gyroscope Issues:
- Verify I2C connections (SDA, SCL)
- Check MPU6050 power supply
- Ensure proper I2C address (0x68)

## 📝 Code Customization

### PID Tuning:
Adjust in `Drone_Flight_control.ino`:
```cpp
const float kp = 2;        // Proportional gain
const float ki = 0.0001;   // Integral gain
const float kd = 0.5;      // Derivative gain
```

### Sensitivity Adjustment:
```cpp
float sensiX = -0.45;      // Roll sensitivity
float sensiY = 0.45;       // Pitch sensitivity
float sensiZ = -0.01;      // Yaw sensitivity
float sensiThrust = 1.1;   // Throttle sensitivity
```

### Safety Limits:
```cpp
int maxAngle = 30;         // Maximum tilt angle (degrees)
int maxThrust = 1700;      // Maximum throttle limit
```

## 📄 License

This project is provided as-is for educational and DIY purposes.

## 🙏 Credits

Based on Arduino drone flight control systems with enhancements for safety and usability.
