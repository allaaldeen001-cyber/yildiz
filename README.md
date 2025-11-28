# Quadcopter Drone Flight System - Enhanced Edition

## 🚁 Overview
This is a complete Arduino Nano-based quadcopter flight controller system with enhanced stability, comprehensive calibration, and real-time monitoring.

## ✅ What's Fixed & Improved

### 1. **THROTTLE INVERSION FIXED** ✓
- **Problem**: Moving throttle stick UP decreased motor speed
- **Solution**: Corrected mapping in RC transmitter code
- **Result**: Stick UP = More Power (as expected)

### 2. **Enhanced Calibration System** ✓
- Multi-stage calibration with validation
- 2000 samples for gyro calibration (vs 1500 before)
- 1500 samples for level calibration (vs 1000 before)
- Automatic validation checks
- EEPROM storage with integrity checking

### 3. **Improved Stability** ✓
- **Max Angle**: 30° (safe for beginners)
- **Optimized PID values**:
  - P: 1.8 (was 2.0) - Smoother response
  - I: 0.0002 (was 0.0001) - Better position hold
  - D: 0.45 (was 0.5) - Less oscillation
- **Better filtering**: Increased low-pass thresholds
- **Anti-windup**: Prevents integral term from growing too large
- **Improved complementary filter**: 98%/2% gyro/accel (was 99%/1%)

### 4. **Communication Monitoring** ✓
- Real-time packet success rate
- Connection quality indicators (EXCELLENT/WEAK/LOST)
- Detailed statistics every 2 seconds
- Visual warnings for poor signal

### 5. **Safety Features** ✓
- Ultrasonic ground proximity warning (<40cm)
- 30-degree angle limit with kill switch
- 3-second failsafe timeout
- Emergency disarm switch
- Startup validation tests

---

## 📋 Hardware Requirements

### Flight Controller (Drone):
- **Microcontroller**: Arduino Nano
- **IMU**: MPU6050 (I2C: SDA, SCL)
- **Barometer**: MS5611 (I2C: 0x77)
- **Radio**: nRF24L01+ (CE=4, CSN=10)
- **Ultrasonic**: HC-SR04 (Trig=A1, Echo=A2)
- **ESCs**: 4x connected to pins 3, 5, 6, 9
- **Buzzer**: Pin 8
- **LED**: Pin 7
- **Battery Monitor**: Pin A0 (voltage divider)

### Remote Controller (Transmitter):
- **Microcontroller**: Arduino Nano
- **Radio**: nRF24L01+ (CE=9, CSN=10)
- **Joysticks**: 
  - Left Stick: A0 (Throttle), A1 (Yaw)
  - Right Stick: A2 (Pitch), A3 (Roll)
- **Buttons**: Pins 4, 5
- **Switches**: Pins 2, 3

### Power:
- **Flight Controller**: 5V from BEC or regulator
- **ESCs**: 3S or 4S LiPo battery
- **RC Transmitter**: 5V (USB or batteries)

---

## 🔧 Installation & Setup

### Step 1: Install Libraries
Install these Arduino libraries via Library Manager:

```
- RF24 by TMRh20
- Smoothed by Matthew Fryer
- MS5611 by Rob Tillaart (for barometer)
- Wire (built-in)
- EEPROM (built-in)
- Servo (built-in)
- SPI (built-in)
```

### Step 2: Upload Code
1. **RC Transmitter**: Upload `RC_Transmitter_Fixed.ino` to transmitter Arduino
2. **Flight Controller**: Upload `Flight_Controller_Enhanced.ino` to drone Arduino

### Step 3: Calibration (CRITICAL!)

#### A. ESC Calibration (One-time)
Before first flight, calibrate ESCs:
1. Disconnect battery from drone
2. Set transmitter throttle to MAXIMUM
3. Connect battery to drone (ESCs will beep)
4. Wait for confirmation beep
5. Move throttle to MINIMUM
6. ESCs will beep again (calibrated)
7. Disconnect battery

#### B. Gyro & Level Calibration
1. Power on both RC transmitter and drone
2. **Place drone on perfectly FLAT surface**
3. **Hold Button 1 for 2 seconds**
4. Wait for calibration (you'll see dots in serial monitor)
5. Buzzer will beep twice when done
6. Calibration is saved to EEPROM automatically

---

## 🎮 Controls

### RC Transmitter Controls:

| Control | Function | Notes |
|---------|----------|-------|
| **Left Stick Up/Down** | Throttle | UP = More Power |
| **Left Stick Left/Right** | Yaw (Rotation) | Turn left/right |
| **Right Stick Up/Down** | Pitch | Forward/Backward |
| **Right Stick Left/Right** | Roll | Strafe left/right |
| **Button 1** (Hold 2s) | Calibrate Level | Must be on flat surface |
| **Button 2** (Hold 2s) | Arm/Disarm | Toggle motors on/off |
| **Switch 1** | Emergency Disarm | Instant motor stop |
| **Switch 2** | Altitude Hold | Active at 1400-1450 throttle |

### Safety Notes:
- **Always arm with low throttle**
- **Test controls before takeoff**
- **Keep Switch 1 accessible for emergency stop**

---

## 📊 Serial Monitor Output

### RC Transmitter Monitor (57600 baud):
```
========================================
  QUADCOPTER RC TRANSMITTER - READY
========================================
Channel: 108 | Power: LOW | Rate: 250kbps
Throttle: FIXED (UP = More Power)
========================================

T:1234 | Y:0.5 | R:-2.3 | P:1.8 | Link:✓OK | Rate:100% [EXCELLENT]

========== LINK STATUS ==========
Success Rate: 99.8%
Total Packets: 1234
Success: 1232 | Failed: 2
========== CONTROLS ==========
Throttle: 1234 (Raw: 512)
Roll: -2.30 | Pitch: 1.80 | Yaw: 0.50
Arm Switch: OFF | Hold: OFF | Btn1: UP | Btn2: UP
=================================
```

### Flight Controller Monitor (57600 baud):
```
========================================
   QUADCOPTER FLIGHT CONTROLLER
   Enhanced Stability Edition
========================================
✓ Ultrasonic sensor initialized
✓ Motors attached and armed
✓ Radio initialized (Channel 108)
Loading calibration from EEPROM...
Starting Gyro Calibration...
Keep drone FLAT and STILL...
.......
✓ Gyro calibration VALID
Gyro offsets -> X:-23.5 Y:15.2 Z:8.3
✓ Barometer ready

========================================
         STARTUP COMPLETE
========================================
Safety Features:
  • Max Angle: 30 degrees
  • Failsafe: 3 seconds
  • Ground proximity warning

Controls:
  • Button 1 (2s hold): Calibrate level
  • Button 2 (2s hold): Arm/Disarm
  • Switch 1: Emergency disarm
  • Switch 2: Altitude hold (1400-1450)
========================================

>>> READY TO FLY <<<

========== FLIGHT STATUS ==========
Status: ARMED ✓
Angles -> R:2.3° P:-1.5° Y:45.2°
Errors -> R:0.12° P:-0.08° Y:0.00°
Targets -> R:0.0° P:0.0° Y:0.0°
Throttle: 1450 | Motors: FL:1455 FR:1448 RL:1445 RR:1452
Radio: ✓ OK (142 pkts/s)
Ground: 85 cm
===================================
```

---

## 🔍 Troubleshooting

### Issue: "No RC signal detected!"
**Solutions**:
- Check both transmitter and receiver are powered
- Verify nRF24L01+ modules are properly connected
- Ensure both use same channel (108)
- Try adding 10µF capacitor to nRF24 VCC/GND
- Check PA level (LOW on transmitter, MAX on receiver)

### Issue: "Gyro calibration may be poor!"
**Solutions**:
- Place drone on perfectly flat, stable surface
- Don't move drone during calibration
- Avoid vibrations during calibration
- If problem persists, check MPU6050 connections

### Issue: Drone drifts in one direction
**Solutions**:
- Recalibrate level (Button 1, 2s hold)
- Ensure drone is on flat surface during calibration
- Check motor mounting (all motors should be level)
- Verify propellers are installed correctly

### Issue: Oscillations/Vibrations in hover
**Solutions**:
- Reduce P gain (try 1.5 instead of 1.8)
- Reduce D gain (try 0.3 instead of 0.45)
- Check propeller balance
- Ensure ESCs are calibrated
- Dampen vibrations to flight controller

### Issue: Won't arm
**Solutions**:
- Check Switch 1 is in "ON" position
- Verify radio link is active
- Ensure throttle is below 1100
- Check serial monitor for error messages
- Try recalibrating

### Issue: Motors spin at different speeds
**Solutions**:
- Recalibrate ESCs (see Step 3A)
- Check motor rotation directions
- Verify ESC connections
- Ensure battery is fully charged

### Issue: Link quality poor (<90%)
**Solutions**:
- Reduce distance between RC and drone
- Remove obstacles between RC and drone
- Add 10µF capacitor to nRF24L01+
- Check antenna orientation
- Try different PA level (try MAX if you have capacitor)

---

## ⚙️ PID Tuning (Advanced)

If you want to customize flight characteristics:

### In `Flight_Controller_Enhanced.ino`, find:
```cpp
const float kp = 1.8;    // Proportional
const float ki = 0.0002; // Integral
const float kd = 0.45;   // Derivative
const float kpZ = 1.5;   // Yaw
```

### Tuning Guide:
1. **Too Sluggish**: Increase `kp` by 0.2
2. **Too Aggressive/Oscillates**: Decrease `kp` by 0.2
3. **Drifts Over Time**: Increase `ki` by 0.0001
4. **High-frequency vibrations**: Decrease `kd` by 0.05
5. **Slow yaw response**: Increase `kpZ` by 0.2

**Always make small changes and test!**

---

## 🛡️ Safety Checklist

Before **EVERY** flight:

- [ ] Propellers securely attached and undamaged
- [ ] Battery fully charged
- [ ] ESCs properly connected
- [ ] RC link strong (>95% success rate)
- [ ] Gyro calibration valid
- [ ] Level calibration recent (<1 week)
- [ ] Clear area for flying (no people/obstacles)
- [ ] Emergency disarm switch accessible
- [ ] Test all controls at low throttle before takeoff

---

## 📈 Performance Specifications

| Parameter | Value |
|-----------|-------|
| Loop Rate | 140 Hz |
| Radio Update | ~200 Hz |
| Max Angle | 30° (configurable) |
| Failsafe Timeout | 3 seconds |
| Ground Warning | <40 cm |
| Throttle Range | 1000-2000 µs |
| Radio Channel | 108 |
| Radio Data Rate | 250 kbps |

---

## 🎯 Flight Modes

### 1. **Manual Mode** (Default)
- Direct stick control
- Max 30° tilt angle
- Self-leveling disabled

### 2. **Altitude Hold Mode**
- Activated: Switch 2 ON + Throttle 1400-1450
- Maintains current altitude
- Stick up/down adjusts altitude setpoint
- Uses barometer + Kalman filter

---

## 📝 Configuration Options

### In `Flight_Controller_Enhanced.ino`:

```cpp
// === SAFETY ===
int maxAngle = 30;           // Max tilt angle (degrees)
bool killAngle = true;       // Enable angle limiter
int maxThrust = 1850;        // Max throttle value

// === SENSITIVITY ===
float sensiX = -0.3;         // Roll sensitivity
float sensiY = 0.3;          // Pitch sensitivity
float sensiZ = -0.008;       // Yaw sensitivity

// === FILTERING ===
int lowPassX = 8;            // Roll deadband
int lowPassY = 8;            // Pitch deadband
int lowPassZ = 12;           // Yaw deadband
```

---

## 🐛 Debugging

### Enable Verbose Output:
In `setup()`, change:
```cpp
debugging(false);  // Change to true for debug mode
```

### Watch Serial Monitor:
- **Flight Controller**: See angles, errors, motor speeds
- **RC Transmitter**: See stick values, link quality

---

## 📚 Additional Resources

### Learn More:
- **PID Tuning**: Search "quadcopter PID tuning guide"
- **MPU6050**: Calibration and setup guides
- **nRF24L01+**: Range and reliability improvements
- **ESC Calibration**: Detailed video tutorials online

### Recommended Upgrades:
1. Add GPS module for position hold
2. Add FPV camera for first-person view
3. Upgrade to more powerful flight controller (Pixhawk, etc.)
4. Add telemetry for real-time data logging

---

## ⚠️ Important Notes

1. **Always test in open area away from people**
2. **Start with low throttle when learning**
3. **Keep propeller guards if possible**
4. **Monitor battery voltage (avoid over-discharge)**
5. **Practice on simulator before real flight**
6. **Local drone laws may apply - check regulations**

---

## 🎓 Quick Start Guide

### First Flight Procedure:

1. **Charge battery fully**
2. **Power on RC transmitter** (wait for "RC Ready")
3. **Place drone on flat surface**
4. **Power on drone** (wait for startup beeps)
5. **Verify "Radio: ✓ OK" in drone serial monitor**
6. **Calibrate if needed** (Button 1, 2 seconds)
7. **Hold Button 2 for 2 seconds** (arm motors)
8. **Slowly increase throttle** to test response
9. **Lift off gently** (50-60% throttle)
10. **Practice hover before moving around**

**Emergency**: Flip Switch 1 or Button 2 to disarm immediately!

---

## 📞 Support

If you encounter issues:
1. Check serial monitor for error messages
2. Review troubleshooting section above
3. Verify all wiring connections
4. Test components individually
5. Recalibrate everything

---

## 📄 License & Credits

This enhanced flight controller is based on open-source quadcopter designs with significant improvements for stability and safety.

**Key Improvements**:
- Fixed throttle inversion
- Enhanced calibration system
- Improved PID tuning
- Real-time monitoring
- Better safety features

---

## ✨ Summary of Enhancements

| Feature | Before | After |
|---------|--------|-------|
| Throttle Direction | ❌ Inverted | ✅ Correct |
| Max Angle | 180° (unsafe) | 30° (safe) |
| Calibration Samples | 1000-1500 | 1500-2000 |
| Calibration Validation | ❌ None | ✅ Automatic |
| Link Monitoring | Basic | Detailed stats |
| PID Tuning | Aggressive | Smooth & stable |
| Integral Anti-windup | ❌ None | ✅ Implemented |
| Ground Warning | ❌ None | ✅ Ultrasonic |
| Serial Output | Minimal | Comprehensive |
| Startup Tests | ❌ None | ✅ Full validation |

---

## 🚀 Ready to Fly!

Your drone is now equipped with:
- ✅ Correct throttle mapping
- ✅ Enhanced stability
- ✅ Comprehensive monitoring
- ✅ Multiple safety features
- ✅ Professional-grade calibration

**Happy Flying! 🚁**

*Remember: Safety first, practice makes perfect!*
