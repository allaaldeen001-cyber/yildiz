# Quadcopter Drone Flight Controller - Improved Version

## 🚁 Overview
This project contains improved RC Transmitter and Flight Controller code for an Arduino Nano-based quadcopter drone with enhanced stability, calibration, and communication monitoring.

## ✅ Key Improvements Made

### 1. **Fixed Throttle Inversion Issue**
   - **Problem**: Moving throttle stick UP caused drone speed to go LOW
   - **Solution**: Inverted throttle mapping in RC transmitter
   - **Change**: `map(yl, 0, 1023, 2000, 1000)` instead of `map(yl, 0, 1023, 1000, 2000)`
   - **Result**: Throttle stick UP now correctly increases drone speed

### 2. **Enhanced Calibration**
   - Increased gyro calibration samples from 1500 to 2000
   - Improved calibration routine with better averaging
   - Added EEPROM storage for calibration values
   - Better initialization sequence for stable startup

### 3. **Improved Stability**
   - Optimized PID parameters:
     - `kp`: 2.0 → 2.2 (faster response)
     - `ki`: 0.0001 → 0.00015 (better steady-state)
     - `kd`: 0.5 → 0.6 (better damping)
     - `kpZ`: 2.0 → 2.5 (improved yaw control)
   - Added integral anti-windup protection
   - Increased smoothing filters (5 → 7 samples)
   - Better sensor fusion

### 4. **Communication Status Display**
   - Real-time link quality percentage
   - Packet sent/received counters
   - Packet loss detection
   - Link status indicator (OK/LOST)
   - Button and switch status display

### 5. **Safety Features**
   - Max angle limited to **30 degrees** (beginner mode)
   - Kill switch protection
   - Failsafe on radio loss
   - Ground proximity warning
   - Improved arming/disarming logic

## 📁 Files

- **RC_Transmitter.ino** - Remote controller code for Arduino Nano
- **Flight_Controller.ino** - Flight controller code for Arduino Nano

## 🔌 Hardware Connections

### RC Transmitter
- **NRF24L01**: CE=Pin 9, CSN=Pin 10
- **Joysticks**: 
  - Left Stick X (Roll): A1
  - Left Stick Y (Throttle): A0
  - Right Stick X (Yaw): A3
  - Right Stick Y (Pitch): A2
- **Buttons/Switches**:
  - Button 1: Pin 4 (Calibration)
  - Button 2: Pin 5 (Arm/Disarm)
  - Switch 1: Pin 3 (Disarm)
  - Switch 2: Pin 2 (Altitude Hold)

### Flight Controller
- **NRF24L01**: CE=Pin 4, CSN=Pin 10
- **MPU6050**: I2C (SDA/SCL)
- **MS5611 Barometer**: I2C (Address 0x77)
- **Ultrasonic**: Trig=A1, Echo=A2
- **ESCs**:
  - Front Left: Pin 3
  - Front Right: Pin 5
  - Rear Right: Pin 6
  - Rear Left: Pin 9
- **Buzzer**: Pin 8
- **LED**: Pin 7

## 🚀 Setup Instructions

### 1. Upload RC Transmitter Code
1. Open `RC_Transmitter.ino` in Arduino IDE
2. Install required libraries:
   - RF24 (by TMRh20)
   - Smoothed
3. Select board: Arduino Nano
4. Upload to transmitter Arduino

### 2. Upload Flight Controller Code
1. Open `Flight_Controller.ino` in Arduino IDE
2. Install required libraries:
   - RF24 (by TMRh20)
   - Smoothed
   - MS5611
   - Servo (built-in)
   - Wire (built-in)
   - SPI (built-in)
   - EEPROM (built-in)
3. Select board: Arduino Nano
4. Upload to flight controller Arduino

### 3. Calibration Process
1. **Power on both devices**
2. **Keep drone perfectly level and still**
3. **Press Button 1** on transmitter for 2 seconds
4. Wait for calibration beep (high tone)
5. Calibration values are saved to EEPROM

### 4. Arming Sequence
1. Ensure drone is on level surface
2. **Press Button 2** on transmitter for 2 seconds
3. LED will turn on and buzzer beeps
4. Drone is now ARMED
5. To disarm, press Button 2 again or toggle Switch 1

## 📊 Serial Monitor Output

### RC Transmitter Output
```
========================================
RC TRANSMITTER READY
Channel: 108 | PA Level: LOW
========================================

T:1500 | Y:0.00 | R:0.00 | P:0.00 | Link: OK | Quality: 98.5% | Sent: 1234 | Lost: 18 | B1:OFF B2:OFF S1:OFF S2:OFF
```

### Flight Controller Output
```
========================================
FLIGHT CONTROLLER INITIALIZING...
========================================
Motors: OK
Radio: OK (Channel 108)
Loaded calibration from EEPROM - X: -0.123 Y: 0.456
Calibrating Gyro... (Please keep drone still)
Gyro: Calibrated
Barometer: OK
========================================
READY FOR FLIGHT
Max Angle: 30 degrees
Waiting for RC signal...
========================================

Status: DISARMED | Link: OK | Quality: 98.5% | Rx: 1234 | Drop: 18 | Thrust: 1000 | Error X: 0.12 Y: -0.05 Z: 0.00 | Alt: 25cm
```

## 🎮 Controls

- **Left Stick Y (Throttle)**: Up = Increase speed, Down = Decrease speed
- **Right Stick X (Roll)**: Left/Right tilt
- **Right Stick Y (Pitch)**: Forward/Backward tilt
- **Left Stick X (Yaw)**: Rotate left/right
- **Button 1**: Calibration (hold 2 seconds)
- **Button 2**: Arm/Disarm (hold 2 seconds)
- **Switch 1**: Emergency Disarm
- **Switch 2**: Altitude Hold (when throttle is in middle range)

## ⚙️ Tuning Parameters

### PID Tuning (in Flight_Controller.ino)
```cpp
const float kp = 2.2;      // Proportional gain
const float ki = 0.00015;  // Integral gain
const float kd = 0.6;      // Derivative gain
const float kpZ = 2.5;     // Yaw gain
```

### Sensitivity Tuning
```cpp
float sensiX = -0.45;  // Roll sensitivity
float sensiY =  0.45;  // Pitch sensitivity
float sensiZ = -0.01;  // Yaw sensitivity
```

### Safety Limits
```cpp
int maxAngle = 30;        // Maximum tilt angle (degrees)
int maxThrust = 1850;     // Maximum throttle value
int MINarmed = 1050;      // Minimum throttle when armed
```

## 🔧 Troubleshooting

### Throttle Still Inverted?
- Check if joystick wiring is correct
- Verify analog reading direction
- Try swapping the mapping values: `map(yl, 0, 1023, 2000, 1000)` ↔ `map(yl, 0, 1023, 1000, 2000)`

### Poor Stability?
1. Recalibrate gyro (Button 1, keep drone level)
2. Check if drone is balanced
3. Adjust PID values gradually
4. Ensure propellers are correctly mounted

### Communication Issues?
- Check NRF24L01 power supply (add 10uF capacitor)
- Verify channel matches (108)
- Check antenna connections
- Reduce distance between transmitter and receiver

### Drone Won't Arm?
- Ensure throttle is at minimum (1000)
- Check if kill switch is active
- Verify radio link is established
- Check for error messages in serial monitor

## 📝 Notes

- **Always test in a safe, open area**
- **Start with low throttle** until comfortable
- **Keep maxAngle at 30 degrees** for beginner mode
- **Monitor serial output** for system status
- **Calibrate before each flight session**
- **Check battery voltage** before flying

## 🎯 Flight Checklist

- [ ] Both devices powered on
- [ ] Serial monitor shows "Link: OK"
- [ ] Gyro calibrated (Button 1)
- [ ] Drone on level surface
- [ ] Throttle at minimum
- [ ] Props clear of obstacles
- [ ] Battery fully charged
- [ ] Ready to arm (Button 2)

## ⚠️ Safety Warnings

1. **Always disarm** when not flying
2. **Keep hands clear** of propellers
3. **Test in open area** away from people
4. **Monitor battery** voltage
5. **Respect max angle** limit (30 degrees)
6. **Emergency disarm** with Switch 1 if needed

## 📈 Performance Improvements

- **Stability**: Improved PID tuning reduces oscillations
- **Calibration**: Better gyro calibration for accurate level flight
- **Communication**: Enhanced monitoring helps diagnose issues
- **Safety**: 30-degree limit prevents dangerous tilts
- **Responsiveness**: Optimized filters for smoother control

---

**Ready to Fly!** 🚁✨

Make sure to follow all safety guidelines and test thoroughly before attempting aggressive maneuvers.
