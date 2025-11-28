# DIY Drone Flight Control System

Complete Arduino-based drone flight control system with NRF24L01 wireless communication, MPU6050 gyroscope/accelerometer, MS5611 barometer for altitude hold, and joystick-based remote control.

## 📋 System Overview

### Flight Controller (FC) Board
- **Microcontroller**: Arduino Nano
- **Wireless**: NRF24L01 (CE=D4, CSN=D10)
- **IMU**: MPU6050 (I2C)
- **Barometer**: MS5611 (I2C, address 0x77)
- **Motors**: 4x ESC on pins D3, D5, D6, D9
- **Controls**: Buttons D4 (Calibration), D5 (Motor Start), Switches D3 (Arm), D2 (Altitude Hold)
- **Outputs**: Buzzer D8, LED D7

### Remote Controller (RC) Board
- **Microcontroller**: Arduino Nano
- **Wireless**: NRF24L01 (CE=D9, CSN=D10)
- **Joysticks**: 4x analog inputs (A0-A3)

## 🔌 Hardware Connections

### Flight Controller

#### NRF24L01 Module
```
NRF24L01 → Arduino Nano
VCC      → 3.3V
GND      → GND
CE       → D4
CSN      → D10
SCK      → D13
MOSI     → D11
MISO     → D12
```

#### MPU6050
```
MPU6050 → Arduino Nano
VCC     → 5V
GND     → GND
SDA     → A4
SCL     → A5
```

#### MS5611
```
MS5611 → Arduino Nano
VCC    → 5V
GND    → GND
SDA    → A4
SCL    → A5
```

#### ESCs (Electronic Speed Controllers)
```
ESC Front Left  → D3
ESC Front Right → D5
ESC Rear Right  → D6
ESC Rear Left   → D9
```

#### Controls
**NOTE**: Due to pin conflicts (D3/D4/D5 used for motors and NRF), buttons/switches moved to analog pins:
```
Calibration Button  → A0 (with pullup, can use as digital input)
Motor Start Button  → A1 (with pullup, can use as digital input)
Arm Switch          → A2 (with pullup, 1=Disarmed, 0=Armed)
Altitude Hold Switch → A3 (with pullup)
```

#### Outputs
```
Buzzer → D8
LED    → D7
```

### Remote Controller

#### NRF24L01 Module
```
NRF24L01 → Arduino Nano
VCC      → 3.3V
GND      → GND
CE       → D9
CSN      → D10
SCK      → D13
MOSI     → D11
MISO     → D12
```

#### Joysticks
```
Left Joystick Y (Throttle) → A0
Left Joystick X (Yaw)      → A1
Right Joystick Y (Pitch)   → A2
Right Joystick X (Roll)    → A3
```

## 📦 Required Libraries

Install these libraries via Arduino Library Manager:

1. **Servo** (built-in)
2. **SPI** (built-in)
3. **Wire** (built-in)
4. **EEPROM** (built-in)
5. **nRF24L01** - RF24 library by TMRh20
6. **Smoothed** - For signal smoothing
7. **MS5611** - MS5611 barometer library

### Installation
```
Arduino IDE → Sketch → Include Library → Manage Libraries
Search and install:
- RF24 (by TMRh20)
- Smoothed
- MS5611
```

## 🚀 Setup Instructions

### 1. Upload Code

#### Flight Controller
1. Open `FC/Drone_Flight_control.ino` in Arduino IDE
2. Ensure all files in `FC/` folder are in the same directory
3. Select board: **Arduino Nano**
4. Select correct COM port
5. Upload

#### Remote Controller
1. Open `RC/controller.ino` in Arduino IDE
2. Select board: **Arduino Nano**
3. Select correct COM port
4. Upload

### 2. Calibrate Joysticks

Open Serial Monitor (57600 baud) on the RC controller and adjust calibration values in `controller.ino`:

```cpp
float scaleX = 0.1;      // Adjust for roll sensitivity
float calX = -527;       // Adjust for roll center
float scaleY = -0.1;     // Adjust for pitch sensitivity
float calY = -507;       // Adjust for pitch center
float scaleZ = -0.1;     // Adjust for yaw sensitivity
float calZ = -512;       // Adjust for yaw center
float scaleThrust = 1.5; // Adjust for throttle sensitivity
float calThrust = -500;  // Adjust for throttle center
```

### 3. Initial Setup Workflow

1. **Power ON RC first**, then FC
2. Wait for NRF link confirmation (buzzer beeps on FC)
3. **Ensure Arm Switch D3 = 1 (DISARMED)** - LED should be ON
4. **Press Button D4** to calibrate:
   - Calibrates MPU6050 gyro offsets
   - Sets MS5611 ground pressure
   - Confirmation: Buzzer beeps + LED flashes
5. **Set Arm Switch D3 = 0 (ARMED)** - LED turns OFF
6. **Press Button D5** for smooth motor start:
   - Motors ramp up gradually to verify all spin correctly
   - Release button to stop
7. **Use joysticks** to control normally
8. **Use Switch D2** to enable altitude hold mode

## 🎮 Control Functions

### Joystick Mapping

#### Left Joystick
- **Up/Down (A0)**: Throttle (increase/decrease motor power)
- **Left/Right (A1)**: Yaw (rotate clockwise/counter-clockwise)

#### Right Joystick
- **Up/Down (A2)**: Pitch (tilt forward/backward)
- **Left/Right (A3)**: Roll (tilt left/right)

### Switch Functions

#### Arm Switch (D3)
- **1 (HIGH)**: Disarmed - Motors disabled, LED ON
- **0 (LOW)**: Armed - Motors enabled, LED OFF

#### Altitude Hold Switch (D2)
- **0 (LOW)**: Altitude hold active (when armed and in flight)
- **1 (HIGH)**: Manual altitude control

### Button Functions

#### Calibration Button (D4)
- Press and hold for 2 seconds (only when disarmed)
- Calibrates MPU6050 and MS5611
- Stores calibration in EEPROM

#### Motor Start Button (D5)
- Press when armed to smoothly ramp up motors
- Useful for verifying all motors spin correctly
- Release to stop

## ⚙️ Tuning Parameters

### PID Tuning (in `Drone_Flight_control.ino`)

```cpp
const float kp = 2.0;        // Proportional gain
const float ki = 0.0001;     // Integral gain
const float kd = 0.5;         // Derivative gain
const float kpZ = 2.0;       // Yaw proportional gain
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
float sensiX = -0.45;        // Roll sensitivity
float sensiY = 0.45;         // Pitch sensitivity
float sensiZ = -0.01;        // Yaw sensitivity
float sensiThrust = 1.1;     // Throttle sensitivity
```

### Motor Limits

```cpp
int pMAX = 2000;      // Maximum motor power
int pMIN = 1000;      // Minimum motor power
int MINarmed = 1050;  // Minimum when armed
int maxThrust = 1700; // Thrust limit
```

### Safety Limits

```cpp
int maxAngle = 30;    // Maximum tilt angle (degrees)
bool killAngle = true; // Enable angle-based kill switch
```

## 🛡️ Safety Features

1. **Kill Switch**: Automatically stops motors if:
   - Radio signal lost for >3 seconds
   - Tilt angle exceeds 30 degrees (if enabled)

2. **Arming System**: Motors only operate when explicitly armed

3. **Angle Limits**: Maximum tilt angle protection

4. **LED Indicators**:
   - ON continuously when disarmed
   - Blinks when receiving radio data
   - Flashes during calibration

5. **Buzzer Alerts**:
   - Startup sequence
   - NRF link confirmation
   - Calibration confirmation
   - Kill switch activation

## 🔧 Troubleshooting

### No Radio Communication
- Check NRF24L01 connections (especially 3.3V power)
- Verify both FC and RC use same pipe address
- Ensure antennas are connected
- Check Serial Monitor for "Radio OK" message

### Motors Not Spinning
- Verify Arm Switch is set to 0 (armed)
- Check ESC connections and calibration
- Ensure throttle is above minimum (1000)
- Check kill switch status

### Unstable Flight
- Recalibrate MPU6050 (Button D4)
- Adjust PID gains
- Check motor mounting and propellers
- Verify ESC calibration

### Altitude Hold Not Working
- Calibrate MS5611 (part of Button D4 calibration)
- Check MS5611 connections
- Verify switch D2 is set correctly
- Adjust altitude PID gains

## 📝 File Structure

```
workspace/
├── FC/
│   ├── Drone_Flight_control.ino  (Main FC code)
│   ├── Barometer.ino             (Altitude hold)
│   ├── kalman_filter.ino         (Altitude filtering)
│   ├── Gyro.h                    (Gyro class header)
│   └── Gyro.cpp                  (Gyro class implementation)
├── RC/
│   └── controller.ino            (Remote controller code)
└── README.md                     (This file)
```

## ⚠️ Important Notes

1. **Always power ON RC before FC** to establish communication
2. **Keep Arm Switch in DISARMED position** during setup and calibration
3. **Calibrate on level surface** for accurate readings
4. **Test motors individually** before first flight
5. **Start with low throttle** and gradually increase
6. **Maximum tilt angle is 30°** for safety
7. **LED stays ON when disarmed** as a visual warning

## 📚 Additional Resources

- MPU6050 Datasheet
- MS5611 Datasheet
- NRF24L01 Datasheet
- ESC Calibration Guides

## 🔄 Version History

- **v1.0**: Initial release with full flight control, altitude hold, and safety features

---

**⚠️ WARNING**: This is a DIY project. Always follow safety guidelines when working with drones. Test in a safe, open area. Ensure proper propeller guards and safety equipment.
