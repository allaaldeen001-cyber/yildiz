# DIY Drone Flight Control System

A complete Arduino-based flight control system for quadcopter drones using NRF24L01 radio communication, MPU6050 IMU, and MS5611 barometer.

## 📁 Project Structure

```
├── FlightController/
│   ├── FlightController.ino   # Main flight controller code
│   ├── Gyro.h                 # MPU6050 library header
│   └── Gyro.cpp               # MPU6050 library implementation
├── RemoteController/
│   └── RemoteController.ino   # Remote controller code
└── README.md                  # This file
```

## 🔧 Hardware Requirements

### Flight Controller (FC)
- Arduino Nano
- NRF24L01 2.4GHz Radio Module
- MPU6050 6-axis IMU (Gyroscope + Accelerometer)
- MS5611 Barometric Pressure Sensor
- 4x ESCs (Electronic Speed Controllers)
- 4x Brushless Motors
- Buzzer
- LED
- Voltage divider for battery monitoring

### Remote Controller (RC)
- Arduino Nano
- NRF24L01 2.4GHz Radio Module
- 2x Analog Joysticks
- 2x Push Buttons
- 2x Toggle Switches

## 📌 Pin Mappings

### Flight Controller Pin Configuration

| Pin | Function | Description |
|-----|----------|-------------|
| **ESC Motors (PWM)** | | |
| D3 | ESC_FL | Front Left Motor |
| D5 | ESC_FR | Front Right Motor |
| D6 | ESC_RR | Rear Right Motor |
| D9 | ESC_RL | Rear Left Motor |
| **NRF24L01 Radio** | | |
| D4 | CE | Chip Enable |
| D10 | CSN | Chip Select |
| D11 | MOSI | SPI Data Out |
| D12 | MISO | SPI Data In |
| D13 | SCK | SPI Clock |
| **I2C Sensors** | | |
| A4 | SDA | I2C Data (MPU6050, MS5611) |
| A5 | SCL | I2C Clock (MPU6050, MS5611) |
| **Inputs** | | |
| D2 | ARM_SWITCH | Arm/Disarm (1=Disarm, 0=Arm) |
| A0 | CALIBRATION_BTN | Calibration Button |
| A1 | MOTOR_START_BTN | Smooth Motor Start Button |
| A2 | ALTITUDE_HOLD | Altitude Hold Switch |
| **Outputs** | | |
| D7 | LED | Status LED |
| D8 | BUZZER | Audio Feedback |
| **Analog** | | |
| A3 | BATTERY | Battery Voltage Monitor |

### Remote Controller Pin Configuration

| Pin | Function | Description |
|-----|----------|-------------|
| **NRF24L01 Radio** | | |
| D9 | CE | Chip Enable |
| D10 | CSN | Chip Select |
| D11 | MOSI | SPI Data Out |
| D12 | MISO | SPI Data In |
| D13 | SCK | SPI Clock |
| **Joysticks (Analog)** | | |
| A0 | LEFT_Y | Throttle (Up/Down) |
| A1 | LEFT_X | Yaw (Left/Right) |
| A2 | RIGHT_Y | Pitch (Forward/Backward) |
| A3 | RIGHT_X | Roll (Left/Right) |
| **Buttons & Switches** | | |
| D2 | SWITCH_ALT_HOLD | Altitude Hold Toggle |
| D3 | SWITCH_ARM | Arm/Disarm Toggle |
| D4 | BTN_CALIBRATE | Calibration Button |
| D5 | BTN_MOTOR_START | Smooth Motor Start |

## 🎮 Control Logic

### Joystick Functions

| Joystick | Direction | Action |
|----------|-----------|--------|
| **Left Y (A0)** | UP | Increase thrust (up to 2000) |
| | DOWN | Decrease thrust (down to 1000) |
| **Left X (A1)** | LEFT | Rotate Counter-Clockwise (Yaw) |
| | RIGHT | Rotate Clockwise (Yaw) |
| **Right Y (A2)** | UP | Tilt forward (fly forward) |
| | DOWN | Tilt backward (fly backward) |
| **Right X (A3)** | LEFT | Tilt left (roll left) |
| | RIGHT | Tilt right (roll right) |

### Button Functions

| Button/Switch | State | Action |
|---------------|-------|--------|
| **Arm Switch (D3)** | 1 (Up) | Disarmed - Motors OFF |
| | 0 (Down) | Armed - Motors enabled |
| **Altitude Hold (D2)** | 1 (Up) | Manual throttle control |
| | 0 (Down) | Altitude hold active |
| **Calibration (D4)** | Hold 2s | Calibrate MPU6050 & MS5611 |
| **Motor Start (D5)** | Press | Smooth motor spin-up test |

## 🔄 System Workflow

### Startup Sequence

1. **Power ON the Remote Controller first**
2. **Power ON the Flight Controller**
3. Wait for link confirmation (buzzer beeps, LED blinks)
4. Ensure Arm Switch is in DISARM position (D3 = 1)
5. LED stays ON continuously when disarmed (safety warning)

### Calibration Procedure

1. Place drone on a **flat, level surface**
2. Ensure Arm Switch is DISARMED (D3 = 1)
3. Press and hold Calibration Button (D4) for **2 seconds**
4. Wait for confirmation:
   - Short beep → Long beep
   - LED flashes
   - Final confirmation beep
5. Calibration values saved to EEPROM

### Arming Procedure

1. Ensure calibration is complete
2. Move throttle stick to **minimum position**
3. Flip Arm Switch to ARM (D3 = 0)
4. Single beep confirms arming
5. LED starts blinking with radio signal

### Smooth Motor Start Test

1. With drone armed
2. Press Motor Start Button (D5)
3. Motors ramp up slowly over 2 seconds
4. Use this to verify all motors are spinning correctly
5. Beep confirms test complete

### Flying

1. Slowly increase throttle to take off
2. Use right stick for directional control
3. Use left stick X-axis for yaw (rotation)
4. Maximum tilt angle limited to 30° for safety

### Altitude Hold

1. While flying at desired altitude
2. Set throttle to ~50% (neutral)
3. Flip Altitude Hold Switch ON (D2 = 0)
4. Drone maintains current altitude
5. Throttle adjusts altitude up/down when moved

### Landing & Disarming

1. Reduce throttle gradually
2. Land gently
3. Reduce throttle to minimum
4. Flip Arm Switch to DISARM (D3 = 1)
5. Motors stop immediately

## ⚡ LED Behavior Summary

| State | LED Behavior |
|-------|--------------|
| Disarmed | Solid ON (warning) |
| Armed, receiving signal | Blinking |
| Armed, no signal | OFF |
| Calibrating | Flashing pattern |
| Link established | Double blink |

## 🔊 Buzzer Codes

| Pattern | Meaning |
|---------|---------|
| Rising 3-tone | Startup complete |
| Double beep | Radio link established |
| Short-Long beep | Calibration started |
| High beep | Calibration/Action complete |
| Long low beep (repeating) | Error/Kill switch active |
| Single medium beep | State change (arm/disarm) |

## ⚠️ Safety Features

1. **Maximum Tilt Angle**: 30° - Motors cut if exceeded
2. **Radio Timeout**: Motors stop after 3 seconds of no signal
3. **Disarm LED Warning**: LED stays on when disarmed
4. **Calibration Lock**: Can only calibrate when disarmed
5. **Smooth Motor Start**: Gradual spin-up for motor testing

## 🔧 Configuration Parameters

### PID Tuning (FlightController.ino)

```cpp
const float kp = 2.0;      // Proportional gain
const float ki = 0.0001;   // Integral gain
const float kd = 0.5;      // Derivative gain
const float kpZ = 2.0;     // Yaw proportional gain
```

### Altitude Hold PID

```cpp
float pid_p_gain_altitude = 14.0;
float pid_i_gain_altitude = 2.0;
float pid_d_gain_altitude = 7.5;
int pid_max_altitude = 400;
```

### Control Sensitivity

```cpp
float sensiX = -0.45;      // Roll sensitivity
float sensiY = 0.45;       // Pitch sensitivity
float sensiZ = -0.01;      // Yaw sensitivity
float sensiThrust = 1.1;   // Throttle sensitivity
```

## 📚 Required Libraries

Install these libraries via Arduino IDE Library Manager:

1. **RF24** - NRF24L01 radio communication
2. **Servo** - ESC control
3. **Wire** - I2C communication
4. **EEPROM** - Calibration storage
5. **Smoothed** - Signal filtering
6. **MS5611** - Barometer interface

## 🔌 Wiring Diagrams

### NRF24L01 Module

```
NRF24L01    Arduino
--------    -------
VCC    -->  3.3V (NOT 5V!)
GND    -->  GND
CE     -->  D4 (FC) / D9 (RC)
CSN    -->  D10
MOSI   -->  D11
MISO   -->  D12
SCK    -->  D13
IRQ    -->  Not connected
```

### MPU6050 Module

```
MPU6050    Arduino
-------    -------
VCC    -->  5V
GND    -->  GND
SCL    -->  A5
SDA    -->  A4
```

### MS5611 Module

```
MS5611     Arduino
------     -------
VCC    -->  3.3V
GND    -->  GND
SCL    -->  A5
SDA    -->  A4
```

### ESC Connections

```
ESC        Arduino
---        -------
Signal -->  D3/D5/D6/D9
GND    -->  GND (common ground)
VCC    -->  (ESC powered by battery)
```

## 🔍 Troubleshooting

### No Radio Link
- Check NRF24L01 is powered with 3.3V (not 5V)
- Verify both FC and RC use same channel (108)
- Add 10µF capacitor across NRF24L01 VCC-GND
- Check antenna orientation

### Unstable Flight
- Recalibrate on level surface
- Check propeller direction and mounting
- Reduce PID gains
- Check for vibrations affecting sensors

### Motors Not Spinning
- Check arm switch position
- Verify ESC calibration
- Check motor/ESC connections
- Listen for ESC beep codes

### Altitude Hold Issues
- Ensure MS5611 is working (check serial output)
- Recalibrate to set ground pressure
- Check for air currents affecting barometer

## 📝 Motor Configuration

```
    Front
  FL    FR
   \  /
    \/    (X-frame)
    /\
   /  \
  RL    RR
    Back

FL: Front Left  (D3) - CW rotation
FR: Front Right (D5) - CCW rotation
RL: Rear Left   (D9) - CCW rotation
RR: Rear Right  (D6) - CW rotation
```

## 📄 License

This project is open source. Feel free to modify and use for personal projects.

## 🙏 Acknowledgments

Based on community DIY drone projects with improvements for stability, safety, and ease of use.
