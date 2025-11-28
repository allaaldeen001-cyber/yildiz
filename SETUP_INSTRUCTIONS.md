# DIY Drone Flight Control System - Setup Instructions

## 📋 System Overview

This drone system consists of two main components:
1. **Flight Controller (FC)** - Arduino Nano with sensors and motor control
2. **Remote Controller (RC)** - Arduino Nano with joysticks for manual control

## 🔧 Hardware Components

### Flight Controller (FC) Board
- **Arduino Nano**
- **NRF24L01** (2.4GHz Radio)
  - CE Pin: D4
  - CSN Pin: D10
  - MOSI: D11
  - MISO: D12
  - SCK: D13
- **MPU6050** (Gyroscope/Accelerometer)
  - SDA: A4
  - SCL: A5
- **MS5611** (Barometric Pressure Sensor)
  - SDA: A4
  - SCL: A5
  - Address: 0x77
- **ESCs & Motors**:
  - D3 = Front Left (FL)
  - D5 = Front Right (FR)
  - D6 = Rear Right (RR)
  - D9 = Rear Left (RL)
- **Buttons & Switches**:
  - A1 = Calibration Button
  - A2 = Smooth Motor-Start Button
  - A3 = Arm/Disarm Switch (HIGH=disarmed, LOW=armed)
  - D2 = Altitude-Hold Switch
- **Outputs**:
  - D7 = Status LED
  - D8 = Buzzer

### Remote Controller (RC) Board
- **Arduino Nano**
- **NRF24L01** (2.4GHz Radio)
  - CE Pin: D9
  - CSN Pin: D10
- **Joysticks**:
  - A0 = Throttle (Left stick Up/Down)
  - A1 = Yaw (Left stick Left/Right)
  - A2 = Pitch (Right stick Up/Down)
  - A3 = Roll (Right stick Left/Right)
- **Buttons & Switches** (Optional, sent via radio):
  - D2 = Switch 2
  - D3 = Switch 1
  - D4 = Button 1
  - D5 = Button 2

## 📦 Required Libraries

Install these libraries in Arduino IDE (Sketch → Include Library → Manage Libraries):

1. **Servo** (Built-in)
2. **SPI** (Built-in)
3. **Wire** (Built-in)
4. **EEPROM** (Built-in)
5. **RF24** by TMRh20
   - Search: "RF24"
   - Install: "RF24" by TMRh20
6. **Smoothed** by Sofian Audry
   - Search: "Smoothed"
   - Install: "Smoothed" by Sofian Audry
7. **MS5611**
   - Search: "MS5611"
   - Install: "MS5611" library

## 🔌 Wiring Instructions

### Flight Controller Wiring

#### NRF24L01 Module
```
NRF24L01 → Arduino Nano
VCC      → 3.3V (IMPORTANT: Use 3.3V, not 5V!)
GND      → GND
CE       → D4
CSN      → D10
SCK      → D13
MOSI     → D11
MISO     → D12
```

#### MPU6050 Module
```
MPU6050 → Arduino Nano
VCC     → 5V
GND     → GND
SDA     → A4
SCL     → A5
```

#### MS5611 Module
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

All ESCs:
Red wire   → Battery +
Black wire → Battery -
Signal     → Respective Arduino pin
```

#### Buttons & Switches
```
Calibration Button (A1):
  One terminal → A1
  Other terminal → GND

Smooth Start Button (A2):
  One terminal → A2
  Other terminal → GND

Arm/Disarm Switch (A3):
  One terminal → A3
  Other terminal → GND
  (When switch is closed/LOW = Armed, Open/HIGH = Disarmed)

Altitude Hold Switch (D2):
  One terminal → D2
  Other terminal → GND
```

#### LED & Buzzer
```
LED (D7):
  Anode (+) → D7 (via 220Ω resistor)
  Cathode (-) → GND

Buzzer (D8):
  Positive → D8
  Negative → GND
```

### Remote Controller Wiring

#### NRF24L01 Module
```
NRF24L01 → Arduino Nano
VCC      → 3.3V (IMPORTANT: Use 3.3V!)
GND      → GND
CE       → D9
CSN      → D10
SCK      → D13
MOSI     → D11
MISO     → D12
```

#### Joysticks
```
Left Joystick:
  VRx (X-axis) → A1 (Yaw)
  VRy (Y-axis) → A0 (Throttle)
  VCC → 5V
  GND → GND

Right Joystick:
  VRx (X-axis) → A3 (Roll)
  VRy (Y-axis) → A2 (Pitch)
  VCC → 5V
  GND → GND
```

## 💻 Software Setup

### Step 1: Install Arduino IDE
Download and install Arduino IDE from: https://www.arduino.cc/en/software

### Step 2: Install Required Libraries
1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search and install each library listed above

### Step 3: Upload Code to Flight Controller
1. Connect Arduino Nano (FC) to computer via USB
2. Select board: **Tools → Board → Arduino Nano**
3. Select port: **Tools → Port → [Your COM port]**
4. Open `FC/Drone_Flight_control.ino`
5. Ensure all related files are in the same folder:
   - `Drone_Flight_control.ino`
   - `Barometer.ino`
   - `kalman_filter.ino`
   - `Gyro.h`
   - `Gyro.cpp`
6. Click **Upload**

### Step 4: Upload Code to Remote Controller
1. Connect Arduino Nano (RC) to computer via USB
2. Select board: **Tools → Board → Arduino Nano**
3. Select port: **Tools → Port → [Your COM port]**
4. Open `RC/controller.ino`
5. Click **Upload**

## 🚀 First-Time Setup & Calibration

### Power-On Sequence
1. **Power ON the Remote Controller FIRST**
2. Wait 2-3 seconds
3. **Power ON the Flight Controller**
4. Wait for NRF link confirmation (3 beeps from buzzer)

### Calibration Procedure
1. Ensure **Arm Switch (A3) is HIGH (disarmed)** - LED should be ON continuously
2. Place drone on **level surface**
3. Press and hold **Calibration Button (A1)** for 2+ seconds
4. Wait for calibration sequence:
   - Short beep + LED blink
   - Longer beep + LED blink
   - High beep + LED blink (calibration complete)
5. Calibration stores MPU6050 and MS5611 offsets in EEPROM

### Arming the Drone
1. Ensure drone is on **level, safe surface**
2. Ensure **no props are attached** for first test
3. Set **Arm Switch (A3) to LOW (armed)** - LED will start blinking
4. Press and hold **Smooth Start Button (A2)** to ramp up motors smoothly
5. Verify all motors spin correctly
6. Release Smooth Start Button to stop motors

## 🎮 Control Functions

### Joystick Controls
- **Throttle (A0)**: Up = Increase thrust, Down = Decrease thrust
- **Yaw (A1)**: Left = Rotate CCW, Right = Rotate CW
- **Pitch (A2)**: Up = Tilt forward, Down = Tilt backward
- **Roll (A3)**: Left = Tilt left, Right = Tilt right

### Switch Functions
- **Arm/Disarm (A3)**: 
  - HIGH (disarmed) = Motors disabled, LED ON continuously
  - LOW (armed) = Motors enabled, LED blinks on RC signal
- **Altitude Hold (D2)**:
  - LOW = Altitude hold active (maintains current altitude)
  - HIGH = Manual altitude control

## ⚠️ Safety Features

1. **Maximum Tilt Angle**: 30° - Drone will kill motors if exceeded
2. **No Data Timeout**: If RC signal lost for 3+ seconds, motors stop
3. **Arming Required**: Motors only work when armed
4. **Thrust Limiting**: Maximum thrust limited to 1700 for safety
5. **LED Warning**: LED stays ON when disarmed to warn user

## 🔍 Troubleshooting

### NRF24L01 Not Connecting
- Verify both modules use **3.3V** (not 5V!)
- Check antenna connections
- Ensure both FC and RC use same pipe address (0xF0F0F0F0E1LL)
- Try increasing PA level: `radio.setPALevel(RF24_PA_HIGH)`

### Motors Not Spinning
- Check ESC connections
- Verify ESCs are calibrated (some ESCs need calibration)
- Ensure drone is armed (Arm Switch LOW)
- Check motor values in Serial Monitor

### Gyro Drift
- Recalibrate MPU6050 (press Calibration Button when disarmed)
- Ensure drone is on level surface during calibration
- Check for vibrations affecting sensor

### Altitude Hold Not Working
- Verify MS5611 is connected correctly
- Check Serial Monitor for pressure readings
- Ensure Altitude Hold Switch is LOW
- Throttle must be between 1400-1450 for altitude hold to engage

## 📊 Serial Monitor Debugging

Open Serial Monitor at **57600 baud** to see:
- Pressure readings
- Armed status
- Switch states
- Motor values (if debugging enabled)

## 📝 Pin Conflict Resolution

**Note**: Original requirements specified D3, D4, D5 for buttons/switches, but these conflict with motor pins (D3, D5) and NRF CE pin (D4). The code uses:
- **A1** for Calibration Button (instead of D4)
- **A2** for Smooth Start Button (instead of D5)
- **A3** for Arm/Disarm Switch (instead of D3)
- **D2** for Altitude Hold Switch (as specified)

## 🎯 Flight Checklist

Before each flight:
- [ ] Calibrate sensors (when disarmed)
- [ ] Verify all motors spin correctly
- [ ] Check battery voltage
- [ ] Ensure NRF link is established (3 beeps)
- [ ] Test controls on ground first
- [ ] Start with low throttle
- [ ] Keep altitude hold switch OFF for first flights

## 📚 Additional Notes

- **Communication**: ACK (Acknowledgment) is enabled for reliable NRF communication
- **Update Rate**: ~140Hz control loop
- **PID Tuning**: PID gains can be adjusted in `Drone_Flight_control.ino`:
  - `kp`, `ki`, `kd` for attitude control
  - `pid_p_gain_altitude`, `pid_i_gain_altitude`, `pid_d_gain_altitude` for altitude hold

---

**⚠️ WARNING**: Always test without propellers first! Ensure all safety checks pass before flying.
