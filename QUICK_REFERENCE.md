# Quick Reference Card

## Pin Assignments

### Flight Controller
```
D2  → MPU6050 INT
D3  → ESC FL
D4  → NRF24L01 CE
D5  → ESC FR
D6  → ESC RR
D7  → Status LED
D8  → Buzzer
D9  → ESC RL
D10 → NRF24L01 CSN
A4  → I2C SDA
A5  → I2C SCL
```

### Remote Controller
```
D2  → SW1 (Altitude Hold)
D3  → SW2 (ARM/DISARM)
D4  → Button 1 (Calibration)
D5  → Button 2 (Motor ON/ESC Cal)
D9  → NRF24L01 CE
D10 → NRF24L01 CSN
A0  → Throttle
A1  → Yaw
A2  → Pitch
A3  → Roll
```

## Control Mapping

### Joysticks
- **Left Stick V (A0):** Throttle (1000-2000)
- **Left Stick H (A1):** Yaw (1000-2000)
- **Right Stick V (A2):** Pitch (1000-2000)
- **Right Stick H (A3):** Roll (1000-2000)

### Buttons & Switches
- **Button 1 (D4):** IMU Calibration
- **Button 2 (D5):** ESC Calibration + Motor Test
- **SW1 (D2):** Altitude Hold ON/OFF
- **SW2 (D3):** ARM/DISARM (Kill Switch)

## Startup Sequence

1. Power on RC
2. Power on FC
3. Wait for LED blink (link established)
4. Set SW2 = ON (ARM)
5. Press Button 1 → Wait for 2 beeps (IMU cal)
6. Press Button 2 → Wait for 3 beeps (ESC cal)
7. Ready to fly!

## Default PID Values

### Rate PIDs (400 Hz)
```
Roll:  Kp=0.8, Ki=0.0, Kd=0.05
Pitch: Kp=0.8, Ki=0.0, Kd=0.05
Yaw:   Kp=1.2, Ki=0.0, Kd=0.1
```

### Angle PIDs (50 Hz)
```
Roll:  Kp=3.0, Ki=0.0, Kd=0.0
Pitch: Kp=3.0, Ki=0.0, Kd=0.0
```

### Altitude PID (25 Hz)
```
Kp=0.5, Ki=0.1, Kd=0.05
```

## Safety Limits

- **Max Tilt:** ±30°
- **Max Throttle:** 65%
- **Link Timeout:** 500 ms
- **PWM Range:** 1000-2000 μs

## Loop Rates

- **AHRS:** 400 Hz
- **Rate PID:** 400 Hz
- **Angle PID:** 50 Hz
- **Altitude:** 25 Hz
- **Communication:** 50 Hz

## NRF24L01 Settings

- **Channel:** 103
- **Data Rate:** 250 kbps
- **Power:** MAX
- **ACK:** Enabled
- **Retries:** 5

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No link | Check NRF wiring, channel |
| Oscillation | Reduce rate PID Kp |
| Slow response | Increase rate PID Kp |
| Altitude drift | Increase altitude PID Ki |
| Yaw drift | Normal (no magnetometer) |

## Emergency Procedures

1. **Disarm Immediately:** Set SW2 = OFF
2. **Link Lost:** FC auto-disarms after 500ms
3. **Tilt Limit:** FC auto-disarms if >30°
4. **Motor Stop:** All motors to MIN_PWM

## File Structure

```
/workspace/
├── FlightController.ino    # FC firmware
├── RemoteController.ino     # RC firmware
├── DroneProtocol.h          # Communication protocol
├── README.md                # Complete documentation
├── TUNING_GUIDE.md          # PID tuning guide
├── SETUP_GUIDE.md           # Installation guide
├── SCHEMATICS.md            # Hardware schematics
└── QUICK_REFERENCE.md       # This file
```

## Important Notes

⚠️ **ALWAYS test without props first!**  
⚠️ **Follow local drone regulations**  
⚠️ **Fly in safe, open areas**  
⚠️ **Use proper safety equipment**  
⚠️ **Test failsafe before first flight**

---

**Version:** 1.0  
**For detailed information, see README.md**
