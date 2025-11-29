# Professional UAV Drone System

A complete Arduino Nano-based drone system with Flight Controller and Remote Controller boards, featuring NRF24L01 wireless communication, MPU6050 IMU, PID control, and comprehensive safety features.

## System Overview

This project consists of two main components:
1. **Flight Controller (FC)** - Controls the drone's flight, motors, and stability
2. **Remote Controller (RC)** - Transmits pilot commands and receives telemetry

## Hardware Requirements

### Flight Controller Board
- Arduino Nano
- NRF24L01 PA+LNA module (CE: D4, CSN: D10)
- MPU6050 IMU (INT: D2, SDA: A4, SCL: A5)
- Buzzer (D8)
- Status LED (D7)
- 4x ESC + Motors:
  - Front Left (FL): D3
  - Front Right (FR): D5
  - Rear Right (RR): D6
  - Rear Left (RL): D9

### Remote Controller Board
- Arduino Nano
- NRF24L01 PA+LNA module (CE: D9, CSN: D10)
- 2x Analog Joysticks:
  - Left Joystick: V: A0, H: A1
  - Right Joystick: V: A2, H: A3
- Push Buttons:
  - Button_1: D4 (Calibration)
  - Button_2: D5 (ESC Calibration/Motor On)
- Toggle Switches:
  - SW_1: D2 (Altitude Hold/Position Hold)
  - SW_2: D3 (Arming/Kill Switch)
  - Switch wiring: Pin1 → D2/D3, Pin2&3 → GND

## Wiring Diagrams

### Flight Controller Connections

```
Arduino Nano          NRF24L01
-----------          ---------
D4 (CE)      ------> CE
D10 (CSN)    ------> CSN
D11 (MOSI)   ------> MOSI
D12 (MISO)   ------> MISO
D13 (SCK)    ------> SCK
3.3V         ------> VCC
GND          ------> GND

Arduino Nano          MPU6050
-----------          -------
A4 (SDA)     ------> SDA
A5 (SCL)     ------> SCL
D2           ------> INT
3.3V         ------> VCC
GND          ------> GND

Arduino Nano          ESCs
-----------          ----
D3           ------> FL ESC Signal
D5           ------> FR ESC Signal
D6           ------> RR ESC Signal
D9           ------> RL ESC Signal
5V           ------> ESC BEC (if needed)
GND          ------> ESC GND

Arduino Nano          Peripherals
-----------          -----------
D7           ------> LED (with 220Ω resistor)
D8           ------> Buzzer (with transistor if needed)
```

### Remote Controller Connections

```
Arduino Nano          NRF24L01
-----------          ---------
D9 (CE)      ------> CE
D10 (CSN)    ------> CSN
D11 (MOSI)   ------> MOSI
D12 (MISO)   ------> MISO
D13 (SCK)    ------> SCK
3.3V         ------> VCC
GND          ------> GND

Arduino Nano          Joysticks
-----------          ---------
A0           ------> Left Joystick V (Throttle)
A1           ------> Left Joystick H (Yaw)
A2           ------> Right Joystick V (Pitch)
A3           ------> Right Joystick H (Roll)
5V           ------> Joystick VCC
GND          ------> Joystick GND

Arduino Nano          Buttons/Switches
-----------          ----------------
D4           ------> Button_1 (with pull-up)
D5           ------> Button_2 (with pull-up)
D2           ------> SW_1 Pin1 (with pull-up)
D3           ------> SW_2 Pin1 (with pull-up)
GND          ------> SW_1 Pin2&3, SW_2 Pin2&3
```

## Software Setup

### Required Libraries

Install the following libraries via Arduino IDE Library Manager:

1. **RF24** by TMRh20
   - Tools → Manage Libraries → Search "RF24"
   - Install "RF24" by TMRh20

2. **Servo** (usually included with Arduino IDE)

### Installation Steps

1. **Install Arduino IDE** (version 1.8.x or later)

2. **Install Libraries**:
   - Open Arduino IDE
   - Go to Sketch → Include Library → Manage Libraries
   - Search and install "RF24" library

3. **Upload Flight Controller Code**:
   - Open `FlightController/FlightController.ino`
   - Select Board: Arduino Nano
   - Select Processor: ATmega328P (Old Bootloader) or ATmega328P
   - Select Port: (your FC Arduino port)
   - Click Upload

4. **Upload Remote Controller Code**:
   - Open `RemoteController/RemoteController.ino`
   - Select Board: Arduino Nano
   - Select Processor: ATmega328P (Old Bootloader) or ATmega328P
   - Select Port: (your RC Arduino port)
   - Click Upload

## Communication Protocol

- **NRF Channel**: 103
- **Data Rate**: 250KBPS
- **Power Level**: MAX
- **ACK Enabled**: Yes (15 retries)
- **Update Rate**: ~100Hz (10ms intervals)

### Data Structures

**RCData** (Remote Controller → Flight Controller):
- `throttle`: 1000-2000 (microseconds)
- `yaw`: -500 to +500
- `pitch`: -500 to +500
- `roll`: -500 to +500
- `button1`: Calibration trigger
- `button2`: ESC calibration trigger
- `switch1`: Altitude hold enable
- `switch2`: Arming/Kill switch
- `checksum`: XOR checksum

**FCData** (Flight Controller → Remote Controller):
- `status`: 0=disarmed, 1=armed, 2=calibrating, 3=error
- `link_status`: 0=no link, 1=linked
- `pitch`: Current pitch angle (degrees)
- `roll`: Current roll angle (degrees)
- `yaw`: Current yaw angle (degrees)
- `calibration_status`: 0=not calibrated, 1=calibrated, 2=error

## Operation Manual

### Initial Setup Sequence

1. **Power On Remote Controller First**
   - Connect RC Arduino to power/USB
   - Wait for initialization

2. **Power On Flight Controller**
   - Connect FC Arduino to power/USB
   - Wait for initialization beep

3. **Verify Communication**
   - Check LED on FC (D7):
     - **Blinking slowly (500ms)**: Linked ✓
     - **Blinking fast (100ms)**: No link ✗
   - Check RC Serial Monitor:
     - Should show "LINKED" status
     - Displays real-time telemetry

4. **Arming Sequence**
   - Ensure SW_2 (Kill Switch) is ON (position "1")
   - Press Button_1 to calibrate gyro
   - Wait for 2 beeps (success) or 1 long beep (error)
   - If calibration failed, repeat step 4

5. **ESC Calibration** (First time only or after ESC changes)
   - Set SW_1 to OFF (position "0")
   - Press and hold Button_2
   - Motors will calibrate and test one by one
   - Listen for ESC calibration beep pattern
   - Release Button_2

6. **Arming for Flight**
   - Ensure SW_2 is ON (position "1")
   - System will auto-arm when:
     - Calibrated ✓
     - Link established ✓
     - Kill switch ON ✓
   - Check Serial Monitor: Status should show "ARMED"

### Flight Controls

#### Left Joystick
- **Up/Down (A0 - Throttle)**:
  - **UP**: Increase motor speed (1000-2000), drone climbs
  - **DOWN**: Decrease motor speed, drone descends/stops
- **Left/Right (A1 - Yaw)**:
  - **LEFT**: Rotate counter-clockwise (nose turns left)
  - **RIGHT**: Rotate clockwise (nose turns right)

#### Right Joystick
- **Up/Down (A2 - Pitch)**:
  - **UP**: Tilt forward, drone flies forward
  - **DOWN**: Tilt backward, drone flies backward
- **Left/Right (A3 - Roll)**:
  - **LEFT**: Tilt left, drone slides left
  - **RIGHT**: Tilt right, drone slides right

#### Switches & Buttons
- **SW_1 (D2)**: Altitude/Position Hold
  - **ON (1)**: Activates position hold mode
  - **OFF (0)**: Manual control mode
- **SW_2 (D3)**: Arming/Kill Switch
  - **ON (1)**: Allows arming (when conditions met)
  - **OFF (0)**: Immediate disarm/kill switch
- **Button_1 (D4)**: Gyro Calibration
  - Press to start 3-second calibration
- **Button_2 (D5)**: ESC Calibration
  - Press (with SW_1 OFF) to calibrate ESCs

## Safety Features

1. **Maximum Angle Limit**: 30 degrees tilt angle
   - Prevents flipping and ensures stable flight
   - Motors automatically disarm if exceeded

2. **Throttle Cap**: 65% maximum power
   - Prevents sudden altitude gain
   - Reduces risk of loss of control

3. **Kill Switch**: SW_2 OFF position
   - Immediate motor shutdown
   - Emergency stop capability

4. **Link Monitoring**: 200ms timeout
   - Auto-disarm on communication loss
   - LED indicates link status

5. **Calibration Required**: Must calibrate before arming
   - Ensures accurate IMU readings
   - Prevents drift and instability

## Serial Monitor Output

### Remote Controller Serial Monitor

The RC serial monitor displays:
- **Link Status**: LINKED / NO LINK
- **Armed Status**: ARMED / DISARMED
- **Calibration Status**: YES / NO / ERROR
- **Pitch/Roll/Yaw**: Current angles (degrees)
- **Throttle**: Current throttle value (1000-2000)
- **Button States**: B1, B2 status
- **Switch States**: SW1, SW2 status

Example output:
```
Link Status | Armed | Calibrated | Pitch | Roll | Yaw | Throttle
------------------------------------------------------------
  LINKED   | ARMED  |   YES    |  2.3 | -1.1 | 45.2 | 1200
Buttons: B1=OFF B2=OFF | Switches: SW1=OFF SW2=ON
```

## Troubleshooting

### No Communication Link
- Check NRF24L01 connections (CE, CSN, MOSI, MISO, SCK)
- Verify both modules are on channel 103
- Check power supply (3.3V stable)
- Ensure antennas are connected (for PA+LNA modules)
- Check Serial Monitor for error messages

### Calibration Fails
- Ensure drone is stationary during calibration
- Check MPU6050 connections (SDA, SCL, INT)
- Verify MPU6050 power (3.3V)
- Try recalibrating multiple times
- Check Serial Monitor for error details

### Motors Don't Spin
- Verify ESC connections to correct pins
- Check ESC power supply
- Ensure ESC calibration completed
- Verify arming status (SW_2 must be ON)
- Check throttle is above minimum (1000)

### Unstable Flight
- Recalibrate gyro (Button_1)
- Check motor mounting and propellers
- Verify PID constants (adjust if needed)
- Ensure balanced propellers
- Check for vibrations affecting IMU

### Position Hold Not Working
- Ensure SW_1 is ON (position "1")
- System must be armed and calibrated
- Position hold maintains position when joysticks centered
- Manual input acts as trim adjustment

## PID Tuning

Default PID values are conservative for stability. To tune:

**FlightController.ino** - Adjust these constants:
```cpp
#define KP_ROLL 1.0      // Increase for faster response
#define KI_ROLL 0.05     // Increase to reduce steady-state error
#define KD_ROLL 0.3      // Increase to reduce oscillations

#define KP_PITCH 1.0     // Same as roll
#define KI_PITCH 0.05
#define KD_PITCH 0.3

#define KP_YAW 1.5       // Usually higher for yaw
#define KI_YAW 0.02
#define KD_YAW 0.1
```

**Tuning Tips**:
- Start with small changes (±0.1)
- Test in safe, open area
- Increase P for responsiveness
- Increase D to reduce overshoot
- Increase I to eliminate steady-state error
- Too high values cause oscillations

## Technical Specifications

- **Control Loop Frequency**: 250Hz (4ms)
- **IMU Update Rate**: 250Hz
- **Communication Rate**: ~100Hz (10ms)
- **Max Tilt Angle**: ±30°
- **Max Throttle**: 65% (1300/2000)
- **Min Throttle**: 0% (1000/2000)
- **Motor Mixing**: X configuration
- **IMU Filter**: Complementary filter
- **Communication**: NRF24L01 with ACK

## Motor Configuration (X Layout)

```
    FL  ←→  FR
     \      /
      \    /
       \  /
        X
       /  \
      /    \
     /      \
    RL  ←→  RR
```

- **FL** (Front Left): D3
- **FR** (Front Right): D5
- **RR** (Rear Right): D6
- **RL** (Rear Left): D9

## License

This project is provided as-is for educational and hobby purposes. Use at your own risk. Always follow local regulations regarding drone operation.

## Author Notes

- Designed for Arduino Nano (ATmega328P)
- Optimized for stability and safety
- Professional UAV embedded system architecture
- Suitable for learning and small-scale drone projects

## Support

For issues or questions:
1. Check Serial Monitor output for error messages
2. Verify all connections match wiring diagrams
3. Ensure libraries are correctly installed
4. Review troubleshooting section

---

**⚠️ SAFETY WARNING**: Always test in a safe, open area. Keep hands and body away from propellers. This system has safety features but should be used responsibly. Follow all local drone regulations.
