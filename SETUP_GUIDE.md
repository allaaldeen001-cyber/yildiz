# Quick Setup Guide

## ⚡ Fast Start Checklist

### Before First Use

1. **Hardware Assembly**
   - [ ] Solder all connections according to pinout
   - [ ] Verify NRF24L01 has 3.3V power (NOT 5V!)
   - [ ] Check all I2C connections (MPU6050, MS5611)
   - [ ] Verify ESC connections and power
   - [ ] Test buzzer and LED connections

2. **Software Setup**
   - [ ] Install Arduino IDE (1.8.x or 2.x)
   - [ ] Install required libraries:
     - RF24 by TMRh20
     - Smoothed
     - MS5611 (check library compatibility)
   - [ ] Upload FC code to Flight Controller
   - [ ] Upload RC code to Remote Controller

3. **Calibration**
   - [ ] Power ON RC first, wait 2 seconds
   - [ ] Power ON FC, wait for beep sequence
   - [ ] Verify NRF link (buzzer beeps)
   - [ ] Ensure Arm Switch (A2) = HIGH (disarmed)
   - [ ] Place drone on level surface
   - [ ] Press Calibration Button (A0) for 2 seconds
   - [ ] Wait for confirmation beeps

4. **Pre-Flight Check**
   - [ ] Arm Switch (A2) = LOW (armed)
   - [ ] Press Motor Start Button (A1)
   - [ ] Verify all 4 motors spin correctly
   - [ ] Check motor rotation direction
   - [ ] Verify propellers are mounted correctly

## 🔧 Pin Conflict Resolution

**Important**: Original pin assignments had conflicts. Buttons/switches moved to analog pins:

| Function | Original Pin | New Pin | Reason |
|----------|--------------|---------|--------|
| Calibration Button | D4 | **A0** | D4 used for NRF CE |
| Motor Start Button | D5 | **A1** | D5 used for FR motor |
| Arm Switch | D3 | **A2** | D3 used for FL motor |
| Altitude Hold Switch | D2 | **A3** | Available pin |

**Note**: Analog pins (A0-A3) can be used as digital inputs with `digitalRead()`.

## 📡 Radio Setup

### NRF24L01 Configuration
- **Data Rate**: 250KBPS (for range)
- **Power Level**: LOW (adjust if needed)
- **Auto ACK**: Disabled
- **Pipe Address**: 0xF0F0F0F0E1LL (must match on both FC and RC)

### Troubleshooting Radio
1. Check 3.3V power to NRF24L01
2. Verify CE and CSN pins
3. Ensure antennas are connected
4. Check Serial Monitor for "Radio OK" message
5. Try different power levels if range is poor

## 🎛️ Joystick Calibration

Open Serial Monitor on RC (57600 baud) and adjust values in `controller.ino`:

```cpp
// Center positions (when joysticks are centered)
float calX = -527;    // Roll center
float calY = -507;    // Pitch center  
float calZ = -512;    // Yaw center
float calThrust = -500; // Throttle center

// Sensitivity (how much movement per joystick unit)
float scaleX = 0.1;   // Roll sensitivity
float scaleY = -0.1;  // Pitch sensitivity
float scaleZ = -0.1;  // Yaw sensitivity
float scaleThrust = 1.5; // Throttle sensitivity
```

**Calibration Method**:
1. Center all joysticks
2. Read Serial Monitor values
3. Adjust `cal*` values until outputs are near 0
4. Test full range and adjust `scale*` values

## 🚁 First Flight

1. **Safe Area**: Open field, no obstacles
2. **Arm Drone**: Set Arm Switch (A2) = LOW
3. **Motor Start**: Press Button (A1), verify motors
4. **Takeoff**: Gradually increase throttle
5. **Hover**: Practice maintaining stable hover
6. **Landing**: Reduce throttle, disarm when landed

## ⚠️ Safety Reminders

- **Always disarm** when not flying
- **LED ON** = Disarmed (safe)
- **LED OFF** = Armed (motors can spin)
- **Maximum tilt**: 30° (safety limit)
- **Kill switch**: Activates if radio lost >3 seconds
- **Test motors** before each flight
- **Check battery** voltage before flight

## 🔍 Common Issues

### Motors Don't Spin
- Check Arm Switch position (must be LOW/0)
- Verify ESC calibration
- Check throttle is above minimum
- Ensure no kill switch active

### Unstable Flight
- Recalibrate MPU6050
- Check motor/propeller balance
- Verify ESC connections
- Adjust PID gains if needed

### No Radio Communication
- Power RC before FC
- Check NRF24L01 connections
- Verify 3.3V power
- Check antenna connections
- Try different power level

### Altitude Hold Not Working
- Calibrate MS5611 (part of calibration)
- Check MS5611 I2C connection
- Verify Altitude Hold Switch (A3) position
- Adjust altitude PID gains

## 📊 Serial Monitor Output

### Flight Controller
- `actual_pressure`: Raw barometer reading
- `actual_pressure_2`: Filtered altitude velocity
- `armed`: Arm status (0/1)
- `thrust`: Current throttle value

### Remote Controller
- `Thrust`: Throttle value (1000-2000)
- `Roll`: Roll input (-100 to +100)
- `Pitch`: Pitch input (-100 to +100)
- `Yaw`: Yaw input (-100 to +100)

## 🎯 Next Steps

1. Fine-tune PID gains for your specific drone
2. Adjust control sensitivities
3. Calibrate ESCs if needed
4. Test altitude hold in safe conditions
5. Practice flight maneuvers gradually

---

**Remember**: Start slow, test thoroughly, and always prioritize safety!
