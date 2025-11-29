# Professional UAV Drone System

A complete Arduino Nano-based drone system consisting of a Flight Controller (FC) and Remote Controller (RC) with professional-grade features including PID control, sensor fusion, and safety mechanisms.

## System Overview

### Flight Controller Board
- **Microcontroller**: Arduino Nano
- **Wireless**: NRF24L01 PA+LNA module
- **IMU**: MPU6050 (6-axis accelerometer/gyroscope)
- **Barometer**: MS5611
- **Feedback**: Buzzer (D8) and Status LED (D7)
- **Motors**: 4x ESC + Brushless Motors

### Remote Controller Board
- **Microcontroller**: Arduino Nano
- **Wireless**: NRF24L01 PA+LNA module
- **Inputs**: 2x Joysticks (4-axis), 2x Push Buttons, 2x Toggle Switches
- **Feedback**: Serial Monitor Display

## Hardware Connections

### Flight Controller Wiring

#### NRF24L01 PA+LNA Module
```
NRF24L01    Arduino Nano
VCC    →    3.3V
GND    →    GND
CE     →    D4
CSN    →    D10
SCK    →    D13
MOSI   →    D11
MISO   →    D12
```

#### MPU6050
```
MPU6050     Arduino Nano
VCC    →    5V
GND    →    GND
SDA    →    A4 (SDA)
SCL    →    A5 (SCL)
INT    →    D2
```

#### MS5611 Barometer
```
MS5611      Arduino Nano
VCC    →    5V
GND    →    GND
SDA    →    A4 (SDA)
SCL    →    A5 (SCL)
```

#### ESCs and Motors
```
ESC         Arduino Nano    Motor Position
FL ESC Signal  →  D3       Front Left
FR ESC Signal  →  D5       Front Right
RR ESC Signal  →  D6       Rear Right
RL ESC Signal  →  D9       Rear Left

All ESC Power: Connect to Battery (BEC for Arduino power)
All ESC Ground: Connect to GND
```

#### Buzzer
```
Buzzer      Arduino Nano
Positive →  D8
Negative →  GND
```

#### Status LED
```
LED         Arduino Nano
Anode   →   D7 (via 220Ω resistor)
Cathode →   GND
```

### Remote Controller Wiring

#### NRF24L01 PA+LNA Module
```
NRF24L01    Arduino Nano
VCC    →    3.3V
GND    →    GND
CE     →    D9
CSN    →    D10
SCK    →    D13
MOSI   →    D11
MISO   →    D12
```

#### Left Joystick
```
Joystick    Arduino Nano    Function
V (Vertical)   →  A0       Throttle (Up/Down)
H (Horizontal) →  A1       Yaw (Left/Right)
VCC        →  5V
GND        →  GND
```

#### Right Joystick
```
Joystick    Arduino Nano    Function
V (Vertical)   →  A2       Pitch (Forward/Backward)
H (Horizontal) →  A3       Roll (Left/Right)
VCC        →  5V
GND        →  GND
```

#### Buttons
```
Button      Arduino Nano    Function
Button 1    →  D4          Calibration
Button 2    →  D5          ESC Calibration / Motor On
(Use internal pull-up: connect button between pin and GND)
```

#### Toggle Switches
```
Switch      Arduino Nano    Function
Switch 1    →  D2          Position Hold
Switch 2    →  D3          Arming / Kill Switch

Wiring: Pin1 → Digital Pin (D2 or D3)
        Pin2 → GND
        Pin3 → GND
(When switch is ON: Pin1 connected to switch, reads HIGH)
(When switch is OFF: Pin1 connected to GND, reads LOW)
```

## Required Libraries

Install the following libraries via Arduino IDE Library Manager:

1. **RF24** by TMRh20
   - Search: "RF24"
   - Install: "RF24" by TMRh20

2. **Servo** (Built-in Arduino Library)
   - Usually included with Arduino IDE

3. **PID** by Brett Beauregard
   - Search: "PID"
   - Install: "PID Library" by Brett Beauregard

4. **Wire** (Built-in Arduino Library)
   - Usually included with Arduino IDE

5. **SPI** (Built-in Arduino Library)
   - Usually included with Arduino IDE

### MPU6050 Library Options
You can use either:
- **I2Cdev** library by Jeff Rowberg (includes MPU6050)
- **Adafruit MPU6050** library

For this code, we use direct I2C communication, so no additional library is strictly required, but you may want to use a library for easier integration.

### MS5611 Library Options
- **SparkFun MS5611** library
- Or use direct I2C communication (as implemented)

## Installation Steps

1. **Install Arduino IDE** (version 1.8.x or later)

2. **Install Required Libraries**
   - Open Arduino IDE
   - Go to Sketch → Include Library → Manage Libraries
   - Search and install: RF24, PID

3. **Upload Flight Controller Code**
   - Open `FlightController.ino`
   - Select Board: Tools → Board → Arduino Nano
   - Select Processor: Tools → Processor → ATmega328P (Old Bootloader) or ATmega328P
   - Select Port: Tools → Port → (your COM port)
   - Click Upload

4. **Upload Remote Controller Code**
   - Open `RemoteController.ino`
   - Repeat steps above for RC board
   - Note: Different COM port if using separate USB connection

## System Operation

### Startup Sequence

1. **Power On Remote Controller First**
   - Connect RC to USB or power supply
   - Open Serial Monitor (115200 baud)
   - Wait for "Remote Controller Ready" message

2. **Power On Flight Controller**
   - Connect FC to USB or battery
   - Wait for "Flight Controller Ready" message
   - Status LED should blink when communication is established

3. **Verify Communication**
   - Status LED (D7) on FC should blink rapidly when linked
   - Serial Monitor on RC should show "[LINKED]" status

### Calibration Procedure

1. **Ensure Arming Switch (SW2) is ON**
   - Toggle Switch 2 to position "1" (HIGH)

2. **Perform IMU Calibration**
   - Press Button 1 on Remote Controller
   - Keep drone perfectly still during calibration (~4 seconds)
   - **Success**: Buzzer beeps twice (200ms each)
   - **Failure**: Buzzer beeps once for 7 seconds

3. **ESC Calibration** (Optional but Recommended)
   - Set Switch 1 (Position Hold) to OFF ("0")
   - Press Button 2 on Remote Controller
   - ESCs will calibrate automatically:
     - 2 seconds at maximum throttle
     - 1 second at minimum throttle
     - Motors spin one by one for testing
   - **Confirmation**: Buzzer beeps three times (different from calibration beep)

### Flying Procedure

1. **Pre-Flight Checklist**
   - ✓ Communication linked (LED blinking)
   - ✓ Arming switch (SW2) is ON
   - ✓ Calibration completed successfully
   - ✓ ESC calibration completed (if performed)
   - ✓ Propellers attached correctly
   - ✓ Clear flying area

2. **Arming**
   - Ensure Switch 2 (Arming) is ON
   - Drone is now armed and ready

3. **Takeoff**
   - Slowly increase throttle (Left Joystick UP)
   - Drone should lift off smoothly

4. **Flight Controls**
   - **Left Joystick**:
     - UP/DOWN: Throttle (climb/descend)
     - LEFT/RIGHT: Yaw (rotate left/right)
   - **Right Joystick**:
     - UP/DOWN: Pitch (forward/backward)
     - LEFT/RIGHT: Roll (tilt left/right)

5. **Position Hold** (Optional)
   - Toggle Switch 1 ON to enable altitude hold
   - Drone will maintain current altitude using barometer

6. **Emergency Stop**
   - Toggle Switch 2 OFF (Kill Switch)
   - All motors stop immediately
   - Buzzer beeps once (500ms)

## Safety Features

1. **Maximum Angle Limit**: 30 degrees
   - Prevents flipping and ensures stable flight
   - Automatically limits pitch and roll commands

2. **Throttle Limiting**: 65% maximum power
   - Prevents sudden altitude changes
   - Reduces risk of loss of control

3. **Kill Switch**: Immediate motor shutdown
   - Switch 2 OFF disarms and stops all motors
   - Critical safety feature

4. **Communication Timeout**: Auto-disarm
   - If communication lost, motors remain off
   - LED blinks slowly when no communication

5. **Calibration Requirement**: Must calibrate before flight
   - Prevents flight with incorrect sensor readings
   - Buzzer feedback confirms calibration status

## Serial Monitor Display (Remote Controller)

The RC Serial Monitor shows real-time status:

```
[LINKED] T:512 Y:512 P:512 R:512 | B1:OFF B2:OFF SW1:OFF SW2:ON | FC: Cal:YES Arm:YES Motor:ON Roll:2.1 Pitch:-1.5 Alt:0.5
```

**Status Indicators:**
- `[LINKED]` / `[NO LINK]`: Communication status
- `T/Y/P/R`: Throttle, Yaw, Pitch, Roll values (0-1023)
- `B1/B2`: Button states
- `SW1/SW2`: Switch states
- `FC:`: Flight Controller data
  - `Cal`: Calibration status
  - `Arm`: Arming status
  - `Motor`: Motor status
  - `Roll/Pitch`: Current angles (degrees)
  - `Alt`: Altitude (meters)

## Troubleshooting

### Communication Issues
- **No LED blinking**: Check NRF24L01 connections, ensure both boards powered
- **Intermittent connection**: Check antenna connections, reduce distance
- **Wrong channel**: Verify both boards use channel 103

### Calibration Issues
- **Calibration fails**: Ensure drone is perfectly still, check MPU6050 connections
- **No beep**: Check buzzer connections, verify D8 pin

### Motor Issues
- **Motors don't spin**: Check ESC connections, verify power supply
- **Motors spin erratically**: Recalibrate ESCs, check signal wires
- **One motor doesn't work**: Check individual ESC and motor connections

### Sensor Issues
- **Drone drifts**: Recalibrate IMU, check for vibrations
- **Altitude hold not working**: Check MS5611 connections, verify I2C communication

## Technical Specifications

### Communication
- **Protocol**: NRF24L01 2.4GHz
- **Channel**: 103
- **Data Rate**: 250kbps
- **Range**: ~1000m (with PA+LNA module)
- **ACK**: Enabled for reliable communication

### Control Loop
- **Update Rate**: ~200Hz (5ms loop)
- **PID Control**: Roll, Pitch, Yaw, Altitude
- **Sensor Fusion**: Complementary filter for attitude estimation

### ESC Control
- **PWM Frequency**: 50Hz (20ms period)
- **Pulse Width**: 1000-2000 microseconds
- **Calibration**: Automatic via Button 2

## Code Structure

### FlightController.ino
- Main flight control loop
- Sensor reading and fusion
- PID control implementation
- Motor mixing algorithm
- Safety checks and limits

### RemoteController.ino
- Joystick reading and processing
- Button/switch debouncing
- NRF24L01 communication
- Serial monitor display
- User interface

## Development Notes

- **PID Tuning**: Adjust PID gains in FlightController.ino for your specific drone
- **Motor Mixing**: Modify motor mixing algorithm for different frame configurations
- **Safety Limits**: Adjust MAX_ANGLE_DEGREES and MAX_THROTTLE_PERCENT as needed
- **Calibration**: Increase CALIBRATION_SAMPLES for more accurate calibration

## License

This project is provided as-is for educational and development purposes.

## Safety Warning

⚠️ **IMPORTANT SAFETY NOTICES**:

1. **Always test in a safe, open area** away from people and obstacles
2. **Wear safety glasses** when testing motors
3. **Remove propellers** during initial testing and calibration
4. **Start with low throttle** and gradually increase
5. **Keep kill switch accessible** at all times
6. **Check all connections** before each flight
7. **Follow local regulations** regarding drone operation
8. **This is a development system** - use at your own risk

## Support

For issues and questions:
1. Check Serial Monitor output for error messages
2. Verify all connections match wiring diagrams
3. Ensure all libraries are installed correctly
4. Test each component individually before full system test

---

**Professional UAV Embedded System**  
*Developed for Arduino Nano Platform*
