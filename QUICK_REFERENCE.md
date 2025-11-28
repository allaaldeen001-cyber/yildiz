# Quick Reference Guide

## 🎮 Control Functions

### Flight Controller Physical Controls

| Control | Pin | Function |
|---------|-----|----------|
| **Calibration Button** | A0 | Press and hold 2+ seconds when DISARMED to calibrate MPU6050 + MS5611 |
| **Smooth Start Button** | A1 | Press when ARMED to test motors (ramps up over 2 seconds) |
| **Arm/Disarm Switch** | A2 | HIGH (open) = Disarmed, LOW (closed) = Armed |
| **Altitude Hold Switch** | A3 | LOW (closed) = Active, HIGH (open) = Inactive |

### Remote Controller Joysticks

| Joystick | Pin | Function |
|----------|-----|----------|
| **Throttle** | A0 | Left stick Up/Down - Controls motor speed |
| **Yaw** | A1 | Left stick Left/Right - Rotates drone |
| **Pitch** | A2 | Right stick Up/Down - Tilts forward/backward |
| **Roll** | A3 | Right stick Left/Right - Tilts left/right |

## 🔄 System States

### LED Status Indicators
- **LED ON (solid)**: System DISARMED - Safe to handle
- **LED Blinking**: System ARMED and receiving RC signals
- **LED OFF**: System ARMED but no RC signal

### Buzzer Codes
- **Startup sequence** (3 beeps): System initialized
- **Single beep**: RC link confirmed
- **Double beep**: Calibration started
- **Triple beep**: Calibration complete
- **Continuous beeps**: Error/Kill switch activated

## 📋 Pre-Flight Checklist

1. ✅ Power ON RC controller first
2. ✅ Power ON Flight Controller
3. ✅ Wait for link confirmation beep
4. ✅ Verify LED is ON (disarmed)
5. ✅ Press Calibration Button (hold 2+ seconds)
6. ✅ Wait for calibration complete beep
7. ✅ Verify all connections secure
8. ✅ **REMOVE PROPELLERS** for first test
9. ✅ Set Arm Switch to ARMED (LED starts blinking)
10. ✅ Press Smooth Start Button to test motors
11. ✅ Verify all 4 motors spin correctly
12. ✅ Re-attach propellers (if motors OK)
13. ✅ Test in open area away from people

## ⚠️ Safety Rules

1. **ALWAYS** remove propellers during testing
2. **ALWAYS** ensure disarmed (LED ON) before handling
3. **ALWAYS** test in open area away from people
4. **NEVER** exceed 30° tilt angle (safety limit)
5. **NEVER** fly near people or obstacles
6. **ALWAYS** power RC first, then FC

## 🔧 Common Operations

### Calibration Procedure
1. Ensure system is DISARMED (LED ON)
2. Press and hold Calibration Button (A0) for 2+ seconds
3. Wait for double beep (calibration started)
4. Keep drone level and still
5. Wait for triple beep (calibration complete)

### Smooth Motor Start Test
1. Arm the system (Arm Switch to ARMED)
2. Press Smooth Start Button (A1)
3. Motors ramp up smoothly over 2 seconds
4. Verify all 4 motors spin correctly
5. Motors return to idle automatically

### Altitude Hold
1. Arm the system
2. Take off manually
3. Set throttle to mid-range (1400-1450)
4. Activate Altitude Hold Switch (A3 to LOW)
5. Drone maintains current altitude
6. Adjust altitude with throttle stick

## 🐛 Troubleshooting Quick Fixes

| Problem | Solution |
|---------|----------|
| No RC link | Check NRF24L01 connections, verify 3.3V power |
| Motors not spinning | Check arm switch, verify calibration completed |
| Unstable flight | Re-calibrate, check PID gains, verify propellers |
| Altitude hold not working | Check MS5611 connections, verify throttle in range |
| LED not blinking | Check RC link, verify arm switch position |

## 📞 Pin Conflicts Resolution

If you need to use different pins, see `PIN_MAPPING.md` for alternatives.

**Current pin assignments avoid conflicts:**
- NRF24L01: D4 (CE), D10 (CSN)
- ESC Motors: D3, D5, D6, D9
- Physical Controls: A0-A3 (analog pins as digital inputs)
- Outputs: D7 (LED), D8 (Buzzer)

---

For detailed information, see `SETUP_GUIDE.md`
