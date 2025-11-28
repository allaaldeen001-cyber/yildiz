# Quadcopter Drone - Arduino Nano Flight Controller

A complete quadcopter flight control system with RC transmitter for Arduino Nano.

## 🚀 Features

- **Stabilized Flight** - PID-controlled roll, pitch, and yaw
- **Beginner Mode** - Limited tilt angle (30°) for safe learning
- **Auto-Calibration** - Gyroscope and accelerometer calibration with EEPROM storage
- **Safety Features** - Kill switch, angle limits, radio failsafe
- **Ground Proximity Warning** - Ultrasonic sensor support
- **Real-time Status** - Serial monitor communication display

## 📦 Hardware Requirements

### Flight Controller
| Component | Specifications |
|-----------|---------------|
| Arduino Nano | ATmega328P |
| MPU6050 | 6-axis IMU (Gyro + Accelerometer) |
| nRF24L01+ | 2.4GHz radio module |
| ESC x4 | Electronic Speed Controllers |
| Brushless Motors x4 | Matched to your frame |
| Buzzer | 5V active buzzer |
| LED | Status indicator |
| HC-SR04 | Ultrasonic sensor (optional) |

### RC Transmitter
| Component | Specifications |
|-----------|---------------|
| Arduino Nano | ATmega328P |
| nRF24L01+ | 2.4GHz radio module |
| Joystick x2 | Dual-axis analog joysticks |
| Switches x2 | SPDT toggle switches |
| Buttons x2 | Momentary push buttons |

## 🔌 Wiring Diagrams

### Flight Controller Pinout

```
Arduino Nano Pin    ->  Component
-----------------------------------------
D3                  ->  ESC Front Left (Signal)
D4                  ->  nRF24L01 CE
D5                  ->  ESC Front Right (Signal)
D6                  ->  ESC Rear Right (Signal)
D7                  ->  LED
D8                  ->  Buzzer
D9                  ->  ESC Rear Left (Signal)
D10                 ->  nRF24L01 CSN
D11                 ->  nRF24L01 MOSI
D12                 ->  nRF24L01 MISO
D13                 ->  nRF24L01 SCK
A0                  ->  Battery Voltage Divider
A1                  ->  Ultrasonic TRIG
A2                  ->  Ultrasonic ECHO
A4                  ->  MPU6050 SDA
A5                  ->  MPU6050 SCL
3.3V                ->  nRF24L01 VCC (Use regulator!)
GND                 ->  Common Ground
```

### RC Transmitter Pinout

```
Arduino Nano Pin    ->  Component
-----------------------------------------
A0                  ->  Left Joystick Y (Throttle)
A1                  ->  Left Joystick X (Yaw)
A2                  ->  Right Joystick Y (Pitch)
A3                  ->  Right Joystick X (Roll)
D2                  ->  Switch 2 (Altitude Hold)
D3                  ->  Switch 1 (Arm Safety)
D4                  ->  Button 1 (Calibration)
D5                  ->  Button 2 (Arming)
D9                  ->  nRF24L01 CE
D10                 ->  nRF24L01 CSN
D11                 ->  nRF24L01 MOSI
D12                 ->  nRF24L01 MISO
D13                 ->  nRF24L01 SCK
3.3V                ->  nRF24L01 VCC
GND                 ->  Common Ground
```

## 🛠️ Motor Configuration (X Layout)

```
        FRONT
     FL      FR
       \    /
        \  /
     CCW  \/  CW
          /\
        /    \
     CW /      \ CCW
     RL        RR
        REAR

FL (Front Left)  = Counter-Clockwise propeller
FR (Front Right) = Clockwise propeller
RL (Rear Left)   = Clockwise propeller
RR (Rear Right)  = Counter-Clockwise propeller
```

## 📚 Required Libraries

Install via Arduino Library Manager:

```
- RF24 by TMRh20
- Smoothed by Matthew Fryer
- Servo (built-in)
- Wire (built-in)
- EEPROM (built-in)
- SPI (built-in)
```

## ⚙️ Setup Instructions

### Step 1: Flash the Code

1. Open `RC_Transmitter/RC_Transmitter.ino` in Arduino IDE
2. Select "Arduino Nano" as board (ATmega328P processor)
3. Upload to transmitter Arduino

4. Open `Flight_Controller/Flight_Controller.ino` in Arduino IDE
5. Upload to flight controller Arduino

### Step 2: ESC Calibration (Do Once)

Before first flight, calibrate your ESCs:

1. **Disconnect** the battery from the drone
2. Move the throttle stick to **maximum** position
3. Connect the battery (you'll hear beeps from ESCs)
4. Move throttle to **minimum** position
5. Wait for confirmation beeps
6. ESCs are now calibrated

### Step 3: Gyro Calibration

The gyroscope calibrates automatically on power-up:

1. Power on the flight controller
2. **Keep the drone completely still** for 5 seconds
3. Wait for the completion beep

### Step 4: Level Calibration

To calibrate the accelerometer for level flight:

1. Place drone on a **perfectly flat surface**
2. Power on and wait for gyro calibration
3. **Hold Button 1** for 2 seconds
4. Wait for beep confirmation
5. Calibration is saved to EEPROM

## 🎮 Controls

| Control | Action |
|---------|--------|
| Left Stick UP/DOWN | Throttle (UP = More Power) |
| Left Stick LEFT/RIGHT | Yaw rotation |
| Right Stick UP/DOWN | Pitch (forward/backward) |
| Right Stick LEFT/RIGHT | Roll (left/right) |
| Button 1 (hold 2s) | Calibrate gyro & level |
| Button 2 (hold 2s) | Arm / Disarm motors |
| Switch 1 | Safety disarm (flip = motors off) |
| Switch 2 | Altitude hold mode (if enabled) |

## 📊 Serial Monitor Output

### Transmitter Output
Open Serial Monitor at **57600 baud** to see:

```
THR:[=====     ] 1500 | Y:0.0 R:0.0 P:0.0 | SW1:OFF SW2:OFF | LINK:[OK] 98.5%
```

- **THR**: Throttle bar and value (1000-2000)
- **Y/R/P**: Yaw, Roll, Pitch commands
- **SW1/SW2**: Switch states
- **LINK**: Communication status and success rate

### Flight Controller Output
```
A:YES | R:0.5 P:-0.3 Y:12.5 | THR:1450 | M:1400/1380/1420/1390 | LINK:OK
```

- **A**: Armed status
- **R/P/Y**: Current roll, pitch, yaw angles
- **THR**: Throttle input
- **M**: Motor values (FL/FR/RL/RR)
- **LINK**: Radio connection status

## 🔧 Troubleshooting

### Throttle Goes DOWN When Stick Goes UP

The code now inverts throttle automatically. If it's still backwards:

In `RC_Transmitter.ino`, find this line:
```cpp
rawThrottle = 1023 - analogRead(PIN_THROTTLE);
```

Change to:
```cpp
rawThrottle = analogRead(PIN_THROTTLE);
```

### Drone Drifts Without Input

1. Place drone on flat surface
2. Hold Button 1 for 2 seconds to recalibrate
3. Check that propellers are balanced
4. Verify motors are all same specs

### Radio Connection Lost

1. Check nRF24L01 wiring (especially 3.3V power!)
2. Add 10µF capacitor between VCC and GND on nRF module
3. Reduce power level if testing close range
4. Ensure both devices use same channel (108)

### Drone Flips on Takeoff

1. Check motor rotation directions
2. Verify propeller types (CW vs CCW)
3. Confirm motor wiring matches pin definitions
4. Reduce PID gains:
   ```cpp
   #define PID_ROLL_KP    1.0  // Start lower
   #define PID_PITCH_KP   1.0
   ```

### ESCs Not Responding

1. Calibrate ESCs (see Step 2 above)
2. Check signal wire connections
3. Verify ESC ground is connected to Arduino ground
4. Test with Servo library sweep example

## ⚠️ Safety Warnings

1. **REMOVE PROPELLERS** when testing/tuning indoors
2. Never arm with propellers unless ready to fly
3. Always use Switch 1 as a kill switch
4. Keep throttle at minimum before arming
5. Fly in open areas away from people
6. Have a spotter when learning
7. Check battery voltage before each flight

## 🎯 PID Tuning Guide

Start with conservative values and increase gradually:

### Step 1: Set All Gains to Zero
```cpp
#define PID_ROLL_KP    0
#define PID_ROLL_KI    0
#define PID_ROLL_KD    0
```

### Step 2: Tune P Gain
- Increase slowly until drone responds to tilts
- Stop when it oscillates, then reduce by 20%

### Step 3: Tune D Gain
- Add D to dampen oscillations
- Start at 10x your P value

### Step 4: Tune I Gain (Optional)
- Add small I to eliminate drift
- Too much causes slow wobble

### Recommended Starting Values
```cpp
// Conservative (stable, but slow response)
#define PID_ROLL_KP    1.2
#define PID_ROLL_KI    0.01
#define PID_ROLL_KD    10.0

// Aggressive (faster response, may oscillate)
#define PID_ROLL_KP    2.5
#define PID_ROLL_KI    0.03
#define PID_ROLL_KD    20.0
```

## 📁 File Structure

```
/workspace/
├── README.md                    # This file
├── RC_Transmitter/
│   └── RC_Transmitter.ino      # Transmitter code
└── Flight_Controller/
    └── Flight_Controller.ino   # Flight controller code
```

## 📝 Version History

- **v2.0** - Fixed throttle direction, improved calibration, better PID, enhanced status display
- **v1.0** - Initial release

## 📄 License

Open source for personal/educational use.

---

**Happy Flying! 🚁**
