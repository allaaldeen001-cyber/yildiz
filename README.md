# Arduino Quadcopter Drone - Complete Flight System

A professional-grade quadcopter flight control system using Arduino Nano, featuring PID stabilization, sensor fusion, and comprehensive safety features.

## Hardware Components

### Flight Controller Board
- **Arduino Nano** - Main flight controller
- **NRF24L01** - 2.4GHz wireless communication module
- **4x ESC** - Electronic Speed Controllers for brushless motors
- **Buzzer** - Audio feedback for connection status
- **MPU6050** - 6-axis IMU (gyroscope + accelerometer)
- **MS5611** - Barometric pressure sensor for altitude
- **LED** - General status indicator
- **Status LED** - Communication status indicator

### RC Controller Board
- **Arduino Nano** - Remote control unit
- **NRF24L01** - 2.4GHz wireless communication module
- **2x Joystick** - Dual-axis analog joysticks
- **Toggle Switch** - Arm/Kill switch
- **2x Push Button** - Calibration and motor start buttons
- **Status LED** - Operation confirmation LED

## Wiring Diagrams

### Flight Controller Wiring

```
Arduino Nano Pin Connections:
─────────────────────────────────
ESC1 (Motor 1 - Front Left)  → Pin 3
ESC2 (Motor 2 - Front Right) → Pin 5
ESC3 (Motor 3 - Back Left)   → Pin 6
ESC4 (Motor 4 - Back Right)  → Pin 9
Buzzer                        → Pin 10
Status LED                    → Pin 11
LED                           → Pin 12

NRF24L01 Module:
─────────────────────────────────
VCC      → 3.3V (use voltage regulator!)
GND      → GND
CE       → Pin 7
CSN      → Pin 8
SCK      → Pin 13 (SPI)
MOSI     → Pin 11 (SPI)
MISO     → Pin 12 (SPI)

MPU6050 Sensor:
─────────────────────────────────
VCC      → 5V
GND      → GND
SDA      → A4 (I2C)
SCL      → A5 (I2C)

MS5611 Sensor:
─────────────────────────────────
VCC      → 5V
GND      → GND
SDA      → A4 (I2C)
SCL      → A5 (I2C)
```

### RC Controller Wiring

```
Arduino Nano Pin Connections:
─────────────────────────────────
Left Joystick X (Yaw)     → A0
Left Joystick Y (Throttle)→ A1
Right Joystick X (Roll)   → A2
Right Joystick Y (Pitch)  → A3
Toggle Switch             → Pin 2 (with pull-up)
Button 1 (Calibration)    → Pin 4 (with pull-up)
Button 2 (Motor Start)    → Pin 5 (with pull-up)
Status LED                → Pin 13

NRF24L01 Module:
─────────────────────────────────
VCC      → 3.3V (use voltage regulator!)
GND      → GND
CE       → Pin 7
CSN      → Pin 8
SCK      → Pin 13 (SPI)
MOSI     → Pin 11 (SPI)
MISO     → Pin 12 (SPI)
```

## Motor Configuration (X-Frame)

```
    Motor 1 (Front Left)    Motor 2 (Front Right)
              \     /
               \   /
                \ /
                 X
                / \
               /   \
              /     \
    Motor 3 (Back Left)     Motor 4 (Back Right)
```

**Motor Rotation:**
- Motor 1 & 4: Clockwise
- Motor 2 & 3: Counter-clockwise

## Required Libraries

Install these libraries via Arduino Library Manager:

1. **RF24** by TMRh20 - For NRF24L01 communication
2. **Servo** - Built-in Arduino library for ESC control
3. **Adafruit MPU6050** - For MPU6050 sensor
4. **Adafruit Sensor** - Required by MPU6050 library
5. **Adafruit MS5611** - For MS5611 barometric sensor
6. **Wire** - Built-in I2C library
7. **SPI** - Built-in SPI library
8. **EEPROM** - Built-in for storing calibration data

## Installation Instructions

### 1. Install Arduino IDE
Download and install Arduino IDE from https://www.arduino.cc/

### 2. Install Required Libraries
1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search and install:
   - `RF24` by TMRh20
   - `Adafruit MPU6050`
   - `Adafruit Sensor`
   - `Adafruit MS5611`

### 3. Upload Code
1. Connect Flight Controller Arduino Nano via USB
2. Select **Tools → Board → Arduino Nano**
3. Select correct **Port**
4. Open `FlightController.ino`
5. Click **Upload**

6. Repeat for RC Controller:
   - Connect RC Arduino Nano via USB
   - Open `RCController.ino`
   - Click **Upload**

## Setup and Calibration Procedure

### Step-by-Step Flight Setup

1. **Power On Both Systems**
   - Connect Flight Controller to battery (via ESC power)
   - Power RC Controller (USB or battery)
   - Open Serial Monitor (115200 baud) for Flight Controller

2. **Wait for Radio Connection**
   ```
   Serial Monitor will show:
   "Waiting for RC connection..."
   "=== RC CONNECTED ===" (Buzzer will beep)
   ```

3. **Set Kill Switch**
   - Set toggle switch to **KILL position** (OFF)
   - Serial Monitor will confirm: "Kill switch confirmed"

4. **Calibrate Sensors**
   - Press **Button 1** on RC Controller
   - Keep drone **perfectly still** during calibration
   - Serial Monitor will show calibration progress:
     - MPU6050 calibration (1000 samples)
     - MS5611 calibration (100 samples)
     - ESC calibration (follow prompts)

5. **ESC Calibration**
   - When prompted: **Disconnect battery**
   - Press any key in Serial Monitor
   - **Connect battery** - you'll hear ESC beeps
   - ESCs will calibrate automatically

6. **Arm the Drone**
   - Set toggle switch to **ARM position** (ON)
   - Serial Monitor confirms: "=== DRONE ARMED ==="

7. **Start Motors Smoothly**
   - Press **Button 2** on RC Controller
   - Motors will spin at idle speed (smooth, not flying)
   - Serial Monitor confirms: "=== DRONE READY TO FLY ==="

8. **Fly!**
   - Increase throttle slowly to take off
   - Use joysticks to control flight
   - Serial Monitor shows real-time flight data

## Control Scheme

### Left Joystick
- **Up/Down (Y-axis)**: Throttle
  - Center = 1000 (no throttle - SAFE!)
  - Up = 1000-2000 (increasing throttle)
  - Down = 1000 (minimum)
- **Left/Right (X-axis)**: Yaw (rotation)
  - Center = 1000 (no rotation)
  - Left = 500-1000 (counter-clockwise)
  - Right = 1000-2000 (clockwise)

### Right Joystick
- **Up/Down (Y-axis)**: Pitch (forward/backward)
  - Center = 1000 (level)
  - Up = 1000-2000 (nose down, forward)
  - Down = 1000-500 (nose up, backward)
- **Left/Right (X-axis)**: Roll (sideways)
  - Center = 1000 (level)
  - Left = 500-1000 (tilt left)
  - Right = 1000-2000 (tilt right)

### Buttons
- **Button 1**: Calibration (IMU, MS5611, ESC)
- **Button 2**: Smooth motor start

### Toggle Switch
- **OFF (Kill)**: Emergency stop, motors disabled
- **ON (Arm)**: System armed, ready for flight

## Safety Features

1. **Throttle Dead Zone**: Joystick center = 1000 (no throttle), preventing accidental takeoff
2. **Kill Switch**: Immediate motor shutdown via toggle switch
3. **Radio Loss Protection**: Motors automatically stop if radio signal lost (>500ms)
4. **State Machine**: Prevents motors from starting without proper calibration and arming sequence
5. **Motor Limits**: All motor outputs constrained to 1000-2000 microseconds

## Serial Monitor Output

### Setup Phase
```
=== Quadcopter Flight Controller ===
Initializing...
Initializing MPU6050... OK
Initializing MS5611... OK
Initializing NRF24L01... OK
NRF Channel: 76

=== Setup Complete ===
Waiting for RC connection...
```

### Flight Phase
```
State: 6 | Throttle: 1200 | Yaw: 1000 | Pitch: 1050 | Roll: 1000 | Altitude: 1.2m | Pitch Angle: 2.5° | Roll Angle: -1.1° | NRF: OK | Ch: 76
```

## PID Tuning

Default PID values (can be adjusted in code):
- **Pitch PID**: Kp=2.0, Ki=0.5, Kd=0.3
- **Roll PID**: Kp=2.0, Ki=0.5, Kd=0.3
- **Yaw PID**: Kp=1.0, Ki=0.2, Kd=0.1

To tune:
1. Start with default values
2. If drone oscillates: Reduce Kp
3. If slow response: Increase Kp
4. If drift: Increase Ki
5. If overshoot: Increase Kd

## Troubleshooting

### Radio Not Connecting
- Check NRF24L01 wiring (especially 3.3V power)
- Verify both modules use same channel (76)
- Ensure antennas are connected
- Check distance (max ~100m line of sight)

### Motors Not Spinning
- Verify ESC calibration completed
- Check battery connection
- Ensure drone is armed (toggle switch ON)
- Verify Button 2 was pressed for motor start

### Drone Unstable
- Recalibrate MPU6050 (keep drone still)
- Check motor connections and rotation direction
- Verify propellers are correctly installed
- Adjust PID values

### Altitude Reading Incorrect
- Recalibrate MS5611
- Check sensor wiring
- Verify sensor is not blocked

## Technical Specifications

- **Control Loop Frequency**: 250Hz (4ms)
- **Serial Output Frequency**: 10Hz (100ms)
- **Radio Update Rate**: 250Hz
- **ESC PWM Frequency**: 50Hz (20ms period)
- **PWM Range**: 1000-2000 microseconds
- **Radio Channel**: 76 (2.476 GHz)
- **Max Range**: ~100 meters (line of sight)

## Calibration Data Storage

All calibration data is stored in EEPROM:
- IMU offsets (6 values)
- ESC min/max values (8 values)
- Sea level pressure (1 value)

Data persists across power cycles. To recalibrate, press Button 1 again.

## Important Notes

⚠️ **WARNING**: 
- Always keep kill switch accessible
- Test in open area away from people
- Start with low throttle
- Ensure propellers are secure
- Use proper battery protection (fuses, etc.)

🔧 **Joystick Calibration Solution**:
The code solves the dangerous throttle center issue by:
- Mapping joystick center position to 1000 (minimum throttle)
- Only increasing throttle when joystick is moved UP from center
- Implementing dead zone around center position
- This ensures center = safe, no accidental takeoff

## License

This project is provided as-is for educational and hobby purposes. Use at your own risk.

## Support

For issues or questions:
1. Check Serial Monitor output for error messages
2. Verify all wiring connections
3. Ensure libraries are correctly installed
4. Test components individually before full integration

---

**Fly safely and enjoy your quadcopter!** 🚁
