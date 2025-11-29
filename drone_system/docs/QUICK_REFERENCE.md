# Quick Reference Card

## Pin Assignments

### Flight Controller (FC)
```
┌──────────────────────────────────────┐
│ NRF24L01     │ CE:D4, CSN:D10, SPI   │
│ MPU6050      │ SDA:A4, SCL:A5, INT:D2│
│ Motor FL     │ D3 (CW)               │
│ Motor FR     │ D5 (CCW)              │
│ Motor RR     │ D6 (CW)               │
│ Motor RL     │ D9 (CCW)              │
│ Buzzer       │ D8                    │
│ Status LED   │ D7                    │
└──────────────────────────────────────┘
```

### Remote Controller (RC)
```
┌──────────────────────────────────────┐
│ NRF24L01     │ CE:D9, CSN:D10, SPI   │
│ Left Joy V   │ A0 (Throttle)         │
│ Left Joy H   │ A1 (Yaw)              │
│ Right Joy V  │ A2 (Pitch)            │
│ Right Joy H  │ A3 (Roll)             │
│ Button 1     │ D4 (Calibration)      │
│ Button 2     │ D5 (Motor On/Arm)     │
│ Switch 1     │ D2 (Altitude Hold)    │
│ Switch 2     │ D3 (Kill Switch)      │
└──────────────────────────────────────┘
```

---

## LED Status Codes

| Pattern | Meaning |
|---------|---------|
| Slow blink (1Hz) | Waiting for connection / Idle |
| Fast blink (4Hz) | Calibrating / No signal |
| Double blink | Armed, ready to fly |
| Solid ON | Flying |
| OFF | Error or powered off |

---

## Buzzer Codes

| Sound | Meaning |
|-------|---------|
| 2 short beeps | IMU calibration success |
| 1 long beep (7s) | IMU calibration failed |
| Rising 3 tones | ESC calibration complete |
| 2 quick beeps | Armed |
| 1 low beep | Disarmed |
| Repeating beep | Error / Low battery |

---

## Flight Checklist

### Pre-Flight
- [ ] Propellers removed (for testing)
- [ ] Battery charged
- [ ] All connections secure
- [ ] Drone on level surface
- [ ] Clear area around drone

### Startup Sequence
1. [ ] Power ON Remote Controller
2. [ ] Power ON Flight Controller
3. [ ] Verify LED connection (slow blink → pattern change)
4. [ ] Set SW2 (Kill) = OFF
5. [ ] Press BTN1 (Calibrate IMU)
6. [ ] Wait for 2 beeps (success)
7. [ ] Set SW1 = ON
8. [ ] Set SW2 = ON
9. [ ] Throttle to MIN
10. [ ] Press BTN2 (Arm)
11. [ ] Wait for double beep

### Flying
- [ ] Increase throttle slowly
- [ ] Make small control inputs
- [ ] Maintain visual contact
- [ ] Land before battery is depleted

### Emergency
- **KILL SWITCH (SW2 → OFF)** - Instant motor cutoff
- Land immediately if behavior is erratic
- If drone flies away, kill switch ASAP

---

## Joystick Calibration

If joysticks are drifting or off-center, adjust in `remote_controller/config.h`:

```cpp
// Adjust deadband (higher = less sensitive center)
#define JOYSTICK_DEADBAND   20

// Adjust stick ranges if not 0-1023
#define THROTTLE_STICK_MIN  0
#define THROTTLE_STICK_MAX  1023
```

---

## PID Quick Tune

### Symptoms & Fixes

| Symptom | Adjustment |
|---------|------------|
| Oscillating/vibrating | Decrease P or increase D |
| Slow response | Increase P |
| Drifting over time | Increase I (small amounts) |
| Overshooting | Increase D |
| Wobbly at high speed | Decrease I |

### Default Values
```cpp
Roll/Pitch: P=1.3, I=0.04, D=15.0
Yaw:        P=4.0, I=0.02, D=0.0
```

---

## Failsafe Behavior

| Condition | Action |
|-----------|--------|
| Signal lost > 500ms | Gradual throttle reduction |
| Signal lost + Throttle=0 | Auto disarm |
| Kill switch OFF | Immediate disarm |
| Excessive tilt > 30° | Limit applied |

---

## Serial Monitor Commands

Open at **115200 baud** for live telemetry.

The display auto-updates with:
- Connection status
- Control inputs (visual bars)
- Button/switch states
- Drone angles
- Motor PWM values
- Signal quality

---

## Common Issues

| Problem | Solution |
|---------|----------|
| No connection | Check NRF24 wiring, add capacitor |
| IMU cal fails | Keep drone still, level surface |
| Motors don't spin | Run ESC calibration |
| Drone flips | Check motor order/direction |
| Drifts sideways | Level during IMU calibration |
| Radio interference | Change NRF_CHANNEL |

---

## Parts List

### Flight Controller
- 1× Arduino Nano
- 1× NRF24L01 PA+LNA
- 1× MPU6050
- 4× ESC (sized for motors)
- 4× Brushless motors
- 1× Active buzzer
- 1× LED + 330Ω resistor
- 1× 10-100µF capacitor

### Remote Controller
- 1× Arduino Nano
- 1× NRF24L01 PA+LNA
- 2× Joystick modules
- 2× Push buttons
- 2× Toggle switches (SPDT)
- 1× 10-100µF capacitor
- 1× Enclosure

---

## Motor Configuration (X-Quad)

```
      FRONT
        │
   FL ──┼── FR
   CW   │   CCW
        │
   RL ──┼── RR
  CCW   │   CW
        │
      REAR
```

**Propeller Direction:**
- FL & RR: Clockwise (CW) - "Normal" props
- FR & RL: Counter-Clockwise (CCW) - "Reverse" props
