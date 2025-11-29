# Quick Start Guide - UAV Drone System

## Fast Setup Checklist

### Hardware Checklist
- [ ] 2x Arduino Nano boards
- [ ] 2x NRF24L01 PA+LNA modules
- [ ] 1x MPU6050 IMU
- [ ] 4x ESCs + Motors
- [ ] 2x Analog Joysticks
- [ ] 2x Push Buttons
- [ ] 2x Toggle Switches
- [ ] 1x Buzzer
- [ ] 1x LED + 220Ω resistor
- [ ] Jumper wires
- [ ] Power supplies (5V for Arduino, appropriate for ESCs)

### Software Setup
1. [ ] Install Arduino IDE
2. [ ] Install RF24 library (Tools → Manage Libraries → Search "RF24")
3. [ ] Upload FlightController.ino to FC Arduino
4. [ ] Upload RemoteController.ino to RC Arduino

### First Flight Sequence

```
1. Power ON Remote Controller
   └─> Wait for initialization

2. Power ON Flight Controller
   └─> Listen for beep (initialization complete)
   └─> Check LED: Should blink slowly when linked

3. Verify Communication
   └─> Open RC Serial Monitor (115200 baud)
   └─> Should see "LINKED" status

4. Calibrate Gyro
   └─> Ensure SW_2 (Kill Switch) is ON
   └─> Press Button_1
   └─> Wait 3 seconds
   └─> Listen: 2 beeps = success, 1 long beep = error

5. ESC Calibration (First time only)
   └─> Set SW_1 to OFF
   └─> Press Button_2
   └─> Motors calibrate and test one by one
   └─> Listen for ESC calibration beep pattern

6. Arm for Flight
   └─> Ensure SW_2 is ON
   └─> System auto-arms when ready
   └─> Check Serial Monitor: "ARMED" status

7. Take Off
   └─> Slowly increase throttle (Left joystick UP)
   └─> Use right joystick for pitch/roll
   └─> Use left joystick left/right for yaw
```

## Pin Reference

### Flight Controller
| Component | Pin | Notes |
|-----------|-----|-------|
| NRF24L01 CE | D4 | |
| NRF24L01 CSN | D10 | |
| MPU6050 INT | D2 | |
| MPU6050 SDA | A4 | |
| MPU6050 SCL | A5 | |
| Buzzer | D8 | |
| LED | D7 | With 220Ω resistor |
| Motor FL | D3 | |
| Motor FR | D5 | |
| Motor RR | D6 | |
| Motor RL | D9 | |

### Remote Controller
| Component | Pin | Notes |
|-----------|-----|-------|
| NRF24L01 CE | D9 | |
| NRF24L01 CSN | D10 | |
| Left Joystick V | A0 | Throttle |
| Left Joystick H | A1 | Yaw |
| Right Joystick V | A2 | Pitch |
| Right Joystick H | A3 | Roll |
| Button_1 | D4 | Calibration |
| Button_2 | D5 | ESC Calibration |
| SW_1 | D2 | Altitude Hold |
| SW_2 | D3 | Arming/Kill Switch |

## Control Reference

### Joystick Mapping
- **Left Stick UP**: Increase throttle (climb)
- **Left Stick DOWN**: Decrease throttle (descend)
- **Left Stick LEFT**: Yaw left (counter-clockwise)
- **Left Stick RIGHT**: Yaw right (clockwise)
- **Right Stick UP**: Pitch forward (fly forward)
- **Right Stick DOWN**: Pitch backward (fly backward)
- **Right Stick LEFT**: Roll left (slide left)
- **Right Stick RIGHT**: Roll right (slide right)

### Button Functions
- **Button_1**: Start gyro calibration (3 seconds)
- **Button_2**: ESC calibration + motor test

### Switch Functions
- **SW_1 ON**: Enable position/altitude hold
- **SW_1 OFF**: Manual control mode
- **SW_2 ON**: Allow arming (when conditions met)
- **SW_2 OFF**: Kill switch (immediate disarm)

## Safety Checklist

Before each flight:
- [ ] Verify communication link (LED blinking slowly)
- [ ] Calibrate gyro (Button_1)
- [ ] Check kill switch position (SW_2 ON)
- [ ] Verify arming status in Serial Monitor
- [ ] Test motors at low throttle
- [ ] Ensure clear flight area
- [ ] Check propellers are secure
- [ ] Verify battery levels

## Troubleshooting Quick Fixes

| Problem | Solution |
|---------|----------|
| No link | Check NRF24L01 connections, power supply |
| Calibration fails | Keep drone still, check MPU6050 connections |
| Motors don't spin | Check ESC connections, verify arming |
| Unstable flight | Recalibrate gyro, check propellers |
| Serial Monitor blank | Check baud rate (115200), USB connection |

## Emergency Procedures

**Immediate Stop**: Flip SW_2 to OFF (kill switch)
**Loss of Control**: Kill switch immediately
**Communication Loss**: System auto-disarms after 200ms

## Important Notes

- Always test in open, safe area
- Keep hands away from propellers
- Start with low throttle
- Position hold (SW_1) maintains position when joysticks centered
- Throttle is capped at 65% for safety
- Maximum tilt angle is 30° (auto-disarm if exceeded)

---

For detailed information, see README.md
