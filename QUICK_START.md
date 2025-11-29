# Quick Start Guide

## Pre-Flight Checklist

### Hardware Setup
- [ ] Flight Controller assembled and wired correctly
- [ ] Remote Controller assembled and wired correctly
- [ ] Both boards powered (USB or battery)
- [ ] NRF24L01 modules with antennas attached
- [ ] Propellers removed for initial testing
- [ ] ESCs connected to battery (if using battery power)

### Software Setup
- [ ] Arduino IDE installed
- [ ] Required libraries installed (RF24, PID)
- [ ] FlightController.ino uploaded to FC board
- [ ] RemoteController.ino uploaded to RC board
- [ ] Serial Monitor open on RC (115200 baud)

## Startup Sequence

1. **Power On Remote Controller**
   ```
   - Connect RC to USB/power
   - Open Serial Monitor (115200 baud)
   - Wait for "Remote Controller Ready"
   ```

2. **Power On Flight Controller**
   ```
   - Connect FC to USB/battery
   - Wait for "Flight Controller Ready"
   - Status LED (D7) should start blinking
   ```

3. **Verify Communication**
   ```
   - Check Serial Monitor shows "[LINKED]"
   - Status LED on FC blinks rapidly when linked
   - If "[NO LINK]", check NRF24L01 connections
   ```

## Calibration Procedure

### Step 1: IMU Calibration
1. Ensure Switch 2 (Arming) is ON
2. Place drone on level surface
3. Press Button 1 (Calibration)
4. Keep drone perfectly still for ~4 seconds
5. **Success**: Two beeps (200ms each)
6. **Failure**: One long beep (7 seconds)

### Step 2: ESC Calibration (Optional)
1. Set Switch 1 (Position Hold) to OFF
2. Press Button 2 (ESC Calibration)
3. Wait for automatic sequence:
   - 2 seconds at max throttle
   - 1 second at min throttle
   - Motors spin one by one
4. **Confirmation**: Three beeps (100ms each)

## Flying

### Arming
- Toggle Switch 2 (Arming) to ON
- Drone is now armed (motors will spin with throttle)

### Controls
- **Left Joystick**:
  - UP: Increase throttle (climb)
  - DOWN: Decrease throttle (descend)
  - LEFT: Rotate CCW (yaw left)
  - RIGHT: Rotate CW (yaw right)

- **Right Joystick**:
  - UP: Tilt forward (fly forward)
  - DOWN: Tilt backward (fly backward)
  - LEFT: Tilt left (slide left)
  - RIGHT: Tilt right (slide right)

### Position Hold
- Toggle Switch 1 ON to enable altitude hold
- Drone maintains current altitude using barometer

### Emergency Stop
- Toggle Switch 2 OFF immediately
- All motors stop
- Buzzer beeps once

## Troubleshooting

### No Communication
- Check NRF24L01 wiring (CE, CSN, SPI pins)
- Verify both boards powered
- Check channel is 103 on both boards
- Move boards closer together

### Calibration Fails
- Ensure drone is perfectly still
- Check MPU6050 connections (SDA, SCL, INT)
- Verify I2C communication

### Motors Don't Spin
- Check ESC connections
- Verify power supply adequate
- Check signal wires (D3, D5, D6, D9)
- Recalibrate ESCs

### Drone Unstable
- Recalibrate IMU
- Check for vibrations (loose parts)
- Verify propellers installed correctly
- Check motor rotation directions

## Safety Reminders

⚠️ **ALWAYS**:
- Test without propellers first
- Start with low throttle
- Keep kill switch accessible
- Fly in open area away from people
- Check all connections before flight

## Serial Monitor Output

Watch Serial Monitor for real-time status:
```
[LINKED] T:512 Y:512 P:512 R:512 | B1:OFF B2:OFF SW1:OFF SW2:ON | FC: Cal:YES Arm:YES Motor:ON Roll:2.1 Pitch:-1.5 Alt:0.5
```

- `[LINKED]`: Communication status
- `T/Y/P/R`: Joystick values (0-1023)
- `B1/B2`: Button states
- `SW1/SW2`: Switch states
- `FC:`: Flight Controller data

## Next Steps

1. Test communication and calibration
2. Test motors individually (without propellers)
3. First flight: Low altitude, open area
4. Gradually increase altitude and complexity
5. Tune PID values if needed (see main README)

---

**Remember**: Safety first! Always test thoroughly before full flight.
