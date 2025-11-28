# DIY Drone Flight Control System - Setup Guide

## 📋 System Overview

This is a complete Arduino-based drone flight control system with:
- **Flight Controller (FC)**: Arduino Nano with MPU6050, MS5611, NRF24L01
- **Remote Controller (RC)**: Arduino Nano with NRF24L01 and joysticks

## 🔌 Hardware Connections

### Flight Controller (FC) Board

#### Arduino Nano Pin Connections:
- **NRF24L01 Radio Module**
  - CE → D4
  - CSN → D10
  - MOSI → D11
  - MISO → D12
  - SCK → D13
  - VCC → 3.3V (use voltage regulator)
  - GND → GND

- **MPU6050 Gyroscope/Accelerometer**
  - SDA → A4
  - SCL → A5
  - VCC → 5V
  - GND → GND

- **MS5611 Barometer**
  - SDA → A4 (shared with MPU6050)
  - SCL → A5 (shared with MPU6050)
  - VCC → 5V
  - GND → GND

- **ESC Motors**
  - Front Left (FL) → D3
  - Front Right (FR) → D5
  - Rear Right (RR) → D6
  - Rear Left (RL) → D9

- **Physical Controls** (using analog pins to avoid conflicts)
  - Calibration Button → A0 (with pull-up resistor)
  - Smooth Start Button → A1 (with pull-up resistor)
  - Arm/Disarm Switch → A2 (with pull-up resistor)
  - Altitude Hold Switch → A3 (with pull-up resistor)
  
  **Note**: If you prefer to use D2-D5 as specified in requirements, you'll need to:
  - Move NRF24L01 CE from D4 to another pin (e.g., D11)
  - Move ESC pins to different PWM pins
  - Update pin definitions in code accordingly
  - See PIN_MAPPING.md for details

- **Outputs**
  - Buzzer → D8
  - Status LED → D7

- **Battery Voltage Monitor** (optional)
  - Voltage divider → A0

### Remote Controller (RC) Board

#### Arduino Nano Pin Connections:
- **NRF24L01 Radio Module**
  - CE → D9
  - CSN → D10
  - MOSI → D11
  - MISO → D12
  - SCK → D13
  - VCC → 3.3V (use voltage regulator)
  - GND → GND

- **Joysticks**
  - Throttle (Left Up/Down) → A0
  - Yaw (Left Left/Right) → A1
  - Pitch (Right Up/Down) → A2
  - Roll (Right Left/Right) → A3

## 📦 Required Libraries

Install these libraries in Arduino IDE (Sketch → Include Library → Manage Libraries):

1. **Servo** (built-in)
2. **SPI** (built-in)
3. **Wire** (built-in)
4. **EEPROM** (built-in)
5. **Smoothed** - Search for "Smoothed" by David Lloyd
6. **RF24** - Search for "RF24" by TMRh20
7. **nRF24L01** - Usually comes with RF24 library
8. **MS5611** - Search for "MS5611" barometer library

### Manual Library Installation (if needed):
- Download libraries from GitHub if not available in Library Manager
- Place in Arduino/libraries folder

## 🚀 Setup Instructions

### Step 1: Install Arduino IDE
1. Download and install Arduino IDE (1.8.x or 2.x)
2. Install Arduino AVR Boards support (Tools → Board → Boards Manager)

### Step 2: Upload RC Controller Code
1. Open `RC/controller.ino` in Arduino IDE
2. Select Board: **Arduino Nano**
3. Select Processor: **ATmega328P (Old Bootloader)** or **ATmega328P**
4. Select Port: Your RC Arduino COM port
5. Click Upload
6. Open Serial Monitor (57600 baud) to verify operation

### Step 3: Upload Flight Controller Code
1. Open `FC/Drone_Flight_control.ino` in Arduino IDE
2. Ensure all files in FC folder are in the same directory:
   - `Drone_Flight_control.ino`
   - `Barometer.ino`
   - `kalman_filter.ino`
   - `Gyro.h`
   - `Gyro.cpp`
3. Select Board: **Arduino Nano**
4. Select Processor: **ATmega328P (Old Bootloader)** or **ATmega328P**
5. Select Port: Your FC Arduino COM port
6. Click Upload
7. Open Serial Monitor (57600 baud) to monitor status

### Step 4: Calibrate Joysticks (RC)
1. Power on RC controller
2. Open Serial Monitor
3. Center all joysticks
4. Note the values displayed
5. Adjust `calX`, `calY`, `calZ`, `calThrust` in `controller.ino` to center values
6. Re-upload if needed

## 🎮 User Workflow

### Initial Setup (First Time):
1. **Power ON RC first**, then FC (important for link establishment)
2. Wait for link confirmation beep from FC buzzer
3. **Ensure Arm Switch D3 = 1 (DISARMED)** - LED should stay ON
4. Press **Calibration Button D4** (hold for 2+ seconds)
   - Buzzer will beep twice
   - LED will blink
   - Calibration completes (MPU6050 + MS5611)
5. Wait for final confirmation beep

### Pre-Flight Check:
1. Verify Arm Switch D3 = 1 (DISARMED) - LED ON
2. Check all connections
3. Ensure propellers are removed for first test!

### Arming and Flying:
1. Set **Arm Switch D3 = 0 (ARMED)** - LED starts blinking
2. Press **Smooth Start Button D5** to verify motors
   - Motors ramp up smoothly over 2 seconds
   - Verify all 4 motors spin correctly
   - Motors return to idle after test
3. Use joysticks to control:
   - **Left Stick Up/Down**: Throttle
   - **Left Stick Left/Right**: Yaw (rotation)
   - **Right Stick Up/Down**: Pitch (forward/backward)
   - **Right Stick Left/Right**: Roll (left/right)
4. **Altitude Hold**: Set **Switch D2 = 0** to enable
   - Maintains current altitude automatically
   - Adjust with throttle stick

### Safety Features:
- **Maximum Tilt Angle**: 30° (safety limit)
- **Kill Switch**: Arm Switch D3 = 1 (disarms immediately)
- **No Data Protection**: Motors stop if RC link lost for 3+ seconds
- **Angle Protection**: Motors stop if tilt exceeds 30°

## ⚙️ Configuration Parameters

### Flight Controller Tuning (in `Drone_Flight_control.ino`):

**PID Gains:**
```cpp
const float kp = 2;        // Proportional gain
const float ki = 0.0001;   // Integral gain
const float kd = 0.5;      // Derivative gain
const float kpZ = 2;       // Z-axis gain
```

**Control Sensitivity:**
```cpp
float sensiX = -0.45;      // Roll sensitivity
float sensiY = 0.45;       // Pitch sensitivity
float sensiZ = -0.01;      // Yaw sensitivity
float sensiThrust = 1.1;   // Throttle sensitivity
```

**Motor Limits:**
```cpp
int pMAX = 2000;           // Maximum PWM
int pMIN = 1000;           // Minimum PWM
int MINarmed = 1050;       // Armed minimum
int maxThrust = 1700;      // Thrust limit
```

**Safety:**
```cpp
int maxAngle = 30;         // Maximum tilt angle (degrees)
```

### Remote Controller Tuning (in `controller.ino`):

**Joystick Calibration:**
```cpp
float calX = -527;         // Roll center offset
float calY = -507;         // Pitch center offset
float calZ = -512;         // Yaw center offset
float calThrust = -500;    // Throttle center offset
```

## 🔧 Troubleshooting

### No RC Link:
- Check NRF24L01 connections (CE, CSN pins)
- Verify both boards use same pipe address: `0xF0F0F0F0E1LL`
- Ensure 3.3V power supply for NRF24L01 (not 5V!)
- Check antenna connections

### Motors Not Spinning:
- Verify ESC connections and power
- Check arm switch position (must be 0 = armed)
- Ensure calibration completed successfully
- Check serial monitor for error messages

### Unstable Flight:
- Re-calibrate MPU6050 (Button D4 when disarmed)
- Adjust PID gains (start with lower values)
- Check propellers are balanced and correctly mounted
- Verify motor rotation directions

### Altitude Hold Not Working:
- Verify MS5611 connections
- Check barometer calibration
- Ensure switch D2 is set correctly
- Throttle must be between 1400-1450 for altitude hold to engage

## 📝 Notes

- **Always power RC first**, then FC
- **Remove propellers** during initial testing
- **Test in open area** away from people and obstacles
- **Start with low throttle** and gradually increase
- **LED behavior**:
  - ON continuously = Disarmed
  - Blinking = Armed and receiving RC signals
- **Buzzer codes**:
  - Startup sequence = System ready
  - Single beep = Link confirmed
  - Double beep = Calibration started
  - Triple beep = Calibration complete
  - Continuous beeps = Error/Kill switch activated

## ⚠️ Safety Warnings

1. **NEVER** test with propellers attached indoors
2. **ALWAYS** ensure disarmed (LED ON) before handling
3. **VERIFY** motor directions before first flight
4. **TEST** in open area with no people nearby
5. **CHECK** all connections before each flight
6. **MONITOR** battery voltage during flight
7. **RESPECT** maximum tilt angle limit (30°)

## 📚 Additional Resources

- Arduino Nano Pinout: https://www.arduino.cc/en/uploads/Main/ArduinoNanoManual23.pdf
- MPU6050 Datasheet: Search online for MPU-6050 datasheet
- MS5611 Datasheet: Search online for MS5611 datasheet
- NRF24L01 Guide: https://nrf24.github.io/RF24/

---

**Happy Flying! 🚁**
