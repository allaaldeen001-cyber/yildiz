# Quick Start Guide

## Step-by-Step Setup

### 1. Hardware Assembly
- Follow `WIRING_GUIDE.md` for detailed wiring
- **CRITICAL**: NRF24L01 needs 3.3V (use voltage regulator!)
- Double-check all connections before powering on

### 2. Software Setup
1. Install Arduino IDE (1.8.x or 2.x)
2. Install required libraries (see `LIBRARY_NOTES.md`)
3. Upload `FC_Quadcopter.ino` to Flight Controller
4. Upload `RC_Quadcopter.ino` to Remote Control

### 3. Initial Power-On Sequence

**Remote Control:**
1. Power on Remote Control first
2. Wait for joystick calibration (5 seconds - keep joysticks centered)
3. Status LED should blink when transmitting

**Flight Controller:**
1. Power on Flight Controller
2. Open Serial Monitor (9600 baud)
3. Follow on-screen instructions

### 4. Setup Procedure (via Serial Monitor)

The Flight Controller will guide you through:

1. **NRF Connection**
   - Wait for "NRF Connection: SUCCESS"
   - Buzzer will beep once

2. **Kill Switch**
   - Set toggle switch to OFF (kill switch mode)
   - Wait for confirmation beep

3. **Calibration**
   - Keep drone **LEVEL and STILL**
   - Press Button 1 on remote
   - Wait for calibration to complete (~10 seconds)
   - Calibration saves automatically to EEPROM

4. **Arming**
   - Set toggle switch to ON (arm position)
   - Wait for confirmation beeps

5. **Motor Test**
   - **PROPELLERS REMOVED!**
   - Press Button 2 on remote
   - Motors will spin smoothly to test
   - Return to minimum after test

6. **Ready to Fly**
   - Serial Monitor will show "DRONE READY TO FLY"
   - Flight data will start displaying

### 5. Pre-Flight Checklist

- [ ] All propellers removed during testing
- [ ] Kill switch tested (toggle OFF stops motors)
- [ ] Calibration completed successfully
- [ ] Motor test completed (Button 2)
- [ ] All motors spin in correct direction
- [ ] Propellers attached (correct rotation!)
- [ ] Battery fully charged
- [ ] Flight area clear
- [ ] Radio connection stable (status LED blinking)

### 6. First Flight

1. **Arm drone** (toggle switch ON)
2. **Start with low throttle** (left stick up slightly)
3. **Make small adjustments** to pitch/roll
4. **Use kill switch immediately** if drone becomes unstable
5. **Land safely** by reducing throttle

### 7. Flight Data Display

Serial Monitor shows:
```
T:1500 | Y:1500 | P:1500 | R:1500 | Alt:0.00m | Ch:76 | Ang:0.5/-0.3/12.1
```

- **T**: Throttle (1000-2000, center 1500)
- **Y**: Yaw (1000-2000, center 1500)
- **P**: Pitch (1000-2000, center 1500)
- **R**: Roll (1000-2000, center 1500)
- **Alt**: Altitude in meters
- **Ch**: NRF channel number
- **Ang**: Current angles (Pitch/Roll/Yaw in degrees)

## Control Reference

### Left Joystick
- **Up**: Increase throttle (go up)
- **Down**: Decrease throttle (go down)
- **Left**: Rotate counter-clockwise (yaw left)
- **Right**: Rotate clockwise (yaw right)

### Right Joystick
- **Up**: Pitch forward (fly forward)
- **Down**: Pitch backward (fly backward)
- **Left**: Roll left (strafe left)
- **Right**: Roll right (strafe right)

### Buttons
- **Button 1**: Calibration (IMU, MS5611, ESC)
- **Button 2**: Smooth motor start test

### Toggle Switch
- **OFF**: Kill switch (safety mode, motors disabled)
- **ON**: Armed (ready to fly)

## Safety Features

✅ **Automatic Kill on Radio Loss**: If radio connection is lost for >500ms, kill switch activates
✅ **Center Point Safety**: Joysticks centered = 1500 (no motor power)
✅ **Kill Switch**: Immediate motor stop
✅ **Calibration Required**: Drone won't fly until calibrated
✅ **Motor Test Mode**: Test motors without flying

## Troubleshooting

### No NRF Connection
- Check 3.3V power to NRF24L01
- Verify SPI connections
- Check both modules use same channel (76)
- Ensure antennas connected

### Calibration Fails
- Keep drone perfectly level and still
- Don't move during calibration
- Check sensor connections (I2C)

### Unstable Flight
- Recalibrate IMU (keep level)
- Check motor rotation directions
- Verify propeller installation
- Adjust PID values if needed

### Motors Don't Start
- Check ESC connections
- Verify ESC calibration
- Check ESC power supply
- Ensure drone is armed

## Important Notes

⚠️ **Always test with propellers removed first!**
⚠️ **Use kill switch if drone becomes unstable**
⚠️ **Start with low throttle on first flights**
⚠️ **Keep drone level during calibration**
⚠️ **NRF24L01 requires 3.3V - use voltage regulator!**

## Next Steps

- Tune PID values for your specific drone
- Adjust motor mixing if needed
- Add additional safety features
- Implement flight modes (if desired)

For detailed information, see:
- `README.md` - Full documentation
- `WIRING_GUIDE.md` - Wiring instructions
- `LIBRARY_NOTES.md` - Library installation
