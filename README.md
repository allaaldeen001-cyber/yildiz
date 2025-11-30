# Professional Arduino Nano Quadcopter Drone System

A complete professional-grade quadcopter drone system using Arduino Nano with flight controller and remote controller boards.

## 🚁 System Overview

This project implements a full-featured quadcopter drone system with:
- **Flight Controller (FC)**: IMU-based stabilization with PID control
- **Remote Controller (RC)**: 4-channel control with NRF24L01 wireless communication
- **Safety Features**: Failsafe, angle limits, throttle caps, arming system
- **Professional Features**: ESC calibration, gyro calibration, altitude hold capability

---

## 📋 Hardware Components

### A) Flight Controller Board
| Component | Pin Assignment | Description |
|-----------|---------------|-------------|
| Arduino Nano | - | Main microcontroller |
| NRF24L01 PA+LNA | CE: D4, CSN: D10 | 2.4GHz wireless transceiver |
| MPU6050 | INT: D2, I2C: A4(SDA), A5(SCL) | 6-axis IMU sensor |
| Buzzer | D8 | Audio feedback |
| Status LED | D7 | Visual feedback |
| Motor FL (Front-Left) | D3 (PWM) | Front-left motor ESC |
| Motor FR (Front-Right) | D5 (PWM) | Front-right motor ESC |
| Motor RR (Rear-Right) | D6 (PWM) | Rear-right motor ESC |
| Motor RL (Rear-Left) | D9 (PWM) | Rear-left motor ESC |

### B) Remote Controller Board
| Component | Pin Assignment | Description |
|-----------|---------------|-------------|
| Arduino Nano | - | Main microcontroller |
| NRF24L01 PA+LNA | CE: D9, CSN: D10 | 2.4GHz wireless transceiver |
| Left Joystick | V: A0 (Throttle), H: A1 (Yaw) | Throttle & Yaw control |
| Right Joystick | V: A2 (Pitch), H: A3 (Roll) | Pitch & Roll control |
| Button 1 | D4 | Gyro calibration trigger |
| Button 2 | D5 | ESC calibration trigger |
| Button 3 | D6 | Motor spin test |
| Switch 1 | D2 | Altitude Hold toggle |
| Switch 2 | D3 | Arming/Kill switch |

---

## 🎮 Control Mapping

### Left Joystick
- **Vertical (A0 - Throttle)**:
  - Push UP: Increases altitude (1000-2000 PWM)
  - Push DOWN: Decreases altitude
- **Horizontal (A1 - Yaw)**:
  - Push LEFT: Rotate counter-clockwise
  - Push RIGHT: Rotate clockwise

### Right Joystick
- **Vertical (A2 - Pitch)**:
  - Push UP: Tilt forward (fly forward)
  - Push DOWN: Tilt backward (fly backward)
- **Horizontal (A3 - Roll)**:
  - Push LEFT: Tilt left (fly left)
  - Push RIGHT: Tilt right (fly right)

### Buttons & Switches
- **Button 1 (D4)**: Gyro calibration - Press when drone is on flat surface
- **Button 2 (D5)**: ESC calibration - Calibrates motor speed controllers
- **Button 3 (D6)**: Motor test - Spins all motors at minimum speed
- **Switch 1 (D2)**: Altitude Hold - Enable position hold mode
- **Switch 2 (D3)**: Arming Switch - Kill switch for safety

---

## 🚀 Operation Procedure

### Initial Setup
1. **Power ON Remote Controller** - Wait for initialization
2. **Power ON Flight Controller** - Wait for initialization
3. **Verify Communication** - Status LED (D7) should blink indicating NRF link
4. **Set Arming Switch (SW_2) to "1"** - Enable arming mode

### Calibration Sequence
5. **Place drone on flat, level surface**
6. **Press Button 1 (Gyro Calibration)**:
   - Keep drone stationary during calibration
   - Success: Buzzer beeps **TWICE**
   - Failure: Buzzer beeps **ONCE for 7 seconds**

### ESC Calibration (First-time setup)
7. **Set Switch 1 (SW_1) to "0"**
8. **Press Button 2 (ESC Calibration)**:
   - Motors will spin up one by one smoothly
   - Buzzer confirms with unique tone pattern
   - Wait for all 4 motors to complete sequence

### Pre-Flight Check
9. **Press Button 3 (Motor Test)**: All motors spin at minimum speed
10. **Verify all motors spinning correctly**
11. **Check joystick response** on Serial Monitor

### Flight
12. **Increase throttle gradually** - Left joystick up
13. **Use right joystick for directional control**
14. **Use left joystick horizontal for yaw rotation**
15. **To land**: Reduce throttle slowly to zero
16. **Emergency Stop**: Flip SW_2 to "0" (DISARM)

---

## 🛡️ Safety Features

1. **Maximum Angle Limit**: ±30 degrees (prevents aggressive maneuvers)
2. **Throttle Cap**: 65% maximum power (prevents excessive acceleration)
3. **Failsafe Mode**: Auto-disarm on communication loss (>1000ms)
4. **Arming System**: Requires deliberate arming to start motors
5. **Low Throttle Arming**: Can only arm when throttle is below 10%
6. **Kill Switch**: Instant disarm via SW_2
7. **IMU Stability**: Optimized PID tuning for smooth flight
8. **Battery Protection**: Visual/audio warnings for low voltage

---

## 📚 Required Libraries

Install these libraries via Arduino IDE Library Manager:

```
- RF24 by TMRh20 (v1.4.8 or later)
- MPU6050 by Electronic Cats (v1.5.0 or later)
- I2Cdev (included with MPU6050)
- Wire (built-in)
```

---

## 🔧 Installation & Upload

### Remote Controller
1. Open `RemoteController/RemoteController.ino` in Arduino IDE
2. Select **Board**: "Arduino Nano"
3. Select **Processor**: "ATmega328P (Old Bootloader)" or "ATmega328P"
4. Select correct **Port**
5. Click **Upload**

### Flight Controller
1. Open `FlightController/FlightController.ino` in Arduino IDE
2. Select **Board**: "Arduino Nano"
3. Select **Processor**: "ATmega328P (Old Bootloader)" or "ATmega328P"
4. Select correct **Port**
5. Click **Upload**

---

## 🔌 Wiring Diagrams

### Flight Controller Wiring

```
MPU6050 → Arduino Nano
  VCC   →   5V
  GND   →   GND
  SCL   →   A5
  SDA   →   A4
  INT   →   D2

NRF24L01 → Arduino Nano
  VCC   →   3.3V (use external 3.3V regulator recommended)
  GND   →   GND
  CE    →   D4
  CSN   →   D10
  SCK   →   D13
  MOSI  →   D11
  MISO  →   D12

ESC Connections:
  FL ESC Signal → D3
  FR ESC Signal → D5
  RR ESC Signal → D6
  RL ESC Signal → D9

Buzzer (+) → D8
Buzzer (-) → GND

LED (+) → D7 (with 220Ω resistor)
LED (-) → GND
```

### Remote Controller Wiring

```
NRF24L01 → Arduino Nano
  VCC   →   3.3V (use external 3.3V regulator recommended)
  GND   →   GND
  CE    →   D9
  CSN   →   D10
  SCK   →   D13
  MOSI  →   D11
  MISO  →   D12

Left Joystick:
  VRx (H) → A1 (Yaw)
  VRy (V) → A0 (Throttle)
  VCC     → 5V
  GND     → GND

Right Joystick:
  VRx (H) → A3 (Roll)
  VRy (V) → A2 (Pitch)
  VCC     → 5V
  GND     → GND

Button 1 → D4 → GND (with 10kΩ pull-up)
Button 2 → D5 → GND (with 10kΩ pull-up)
Button 3 → D6 → GND (with 10kΩ pull-up)

Switch 1 → D2 (pin 1), GND (pins 2&3)
Switch 2 → D3 (pin 1), GND (pins 2&3)
```

---

## 📊 Serial Monitor Output

### Remote Controller Serial Monitor (115200 baud)
```
=== RC STATUS ===
NRF: LINKED ✓
Throttle: 1234
Yaw: 512
Pitch: 498
Roll: 505
SW1: ON  SW2: ARMED
BTN1: 0  BTN2: 0  BTN3: 0
Calibration: SUCCESS
```

---

## ⚙️ PID Tuning (Advanced)

Default PID values in FlightController.ino:
```cpp
// Roll & Pitch PID
Kp_angle = 2.0
Ki_angle = 0.02
Kd_angle = 15.0

// Yaw PID
Kp_yaw = 3.0
Ki_yaw = 0.02
Kd_yaw = 0.0
```

Adjust these values in the code for different drone weights and sizes.

---

## 🐛 Troubleshooting

| Issue | Solution |
|-------|----------|
| NRF not connecting | Check 3.3V power supply, use external regulator, verify wiring |
| Motors not spinning | Check ESC calibration, verify PWM connections, check arming switch |
| Drone drifts | Perform gyro calibration on flat surface |
| Unstable flight | Adjust PID values, check motor directions, verify propeller orientation |
| No buzzer sound | Check buzzer polarity, verify D8 connection |
| LED not blinking | Check LED polarity, verify 220Ω resistor |

---

## 📐 Motor Layout & Rotation

```
    FRONT
  FL ↻   FR ↺
     \ X /
     / X \
  RL ↺   RR ↻
    REAR

FL = Front Left (D3)  - Counter-clockwise
FR = Front Right (D5) - Clockwise
RR = Rear Right (D6)  - Counter-clockwise
RL = Rear Left (D9)   - Clockwise
```

---

## 📝 License

This project is open-source for educational and non-commercial use.

## ⚠️ Safety Disclaimer

**IMPORTANT**: Drone operation can be dangerous. Always:
- Remove propellers during testing
- Operate in open areas away from people
- Follow local drone regulations
- Use proper battery protection
- Never fly indoors initially
- Wear safety glasses

---

## 👨‍💻 Author

Professional Embedded Systems Engineer
Quadcopter Drone System v1.0

---

**Happy Flying! 🚁**
