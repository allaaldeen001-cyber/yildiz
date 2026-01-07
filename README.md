# Quadcopter Flight Controller & Remote - STABLE VERSION

Professional-grade Arduino-based quadcopter flight controller with advanced calibration and stability features.

## ✅ FIXES IN THIS VERSION

1. **Proper Iterative MPU6050 Calibration** - 6-pass calibration that finds accurate offsets
2. **Level Trim Compensation** - Automatically corrects for mounting angle errors
3. **Gyro Bias Tracking** - Continuously adapts to gyro drift before arming
4. **Heavy Vibration Filtering** - Rejects propeller noise effectively
5. **Level Check Before Arm** - Won't arm unless drone is level
6. **Correct Motor Mixing** - Fixed X-quad configuration

## Calibration Process

On every startup, the flight controller runs a comprehensive calibration:

```
========================================
   MPU6050 ADVANCED CALIBRATION
========================================
Place drone on FLAT LEVEL surface!
DO NOT MOVE during calibration!
Starting in 3 seconds...

Calibrating (this takes ~30 seconds)...
Pass 1/6 - Gyro err: 523 Accel err: 1245
Pass 2/6 - Gyro err: 89 Accel err: 342
Pass 3/6 - Gyro err: 24 Accel err: 156
Pass 4/6 - Gyro err: 8 Accel err: 87
Pass 5/6 - Gyro err: 3 Accel err: 45
Pass 6/6 - Gyro err: 1 Accel err: 23

--- CALIBRATION RESULTS ---
Accel (target 0,0,16384): 12, -8, 16378
Gyro (target 0,0,0): 0.2, -0.1, 0.3
Level offset (Roll, Pitch): 0.12°, -0.34°

*** CALIBRATION SUCCESS! ***
```

## Motor Layout (X-Configuration)

```
       FRONT
    FL(CCW)  FR(CW)
        ╲  ╱
         ╳
        ╱  ╲
    RL(CW)  RR(CCW)
       BACK
```

**Motor spin directions:**
- FL (Front-Left): Counter-Clockwise (CCW)
- FR (Front-Right): Clockwise (CW)
- RL (Rear-Left): Clockwise (CW)
- RR (Rear-Right): Counter-Clockwise (CCW)

## Hardware Requirements

### Flight Controller
| Component | Pin |
|-----------|-----|
| MPU6050 SDA | A4 |
| MPU6050 SCL | A5 |
| NRF24L01 CE | D4 |
| NRF24L01 CSN | D10 |
| ESC FL | D3 |
| ESC FR | D5 |
| ESC RL | D6 |
| ESC RR | D9 |
| Buzzer | D8 |
| LED | D7 |

### Remote Controller
| Component | Pin |
|-----------|-----|
| NRF24L01 CE | D9 |
| NRF24L01 CSN | D10 |
| Throttle Joystick | A0 |
| Yaw Joystick | A1 |
| Pitch Joystick | A2 |
| Roll Joystick | A3 |
| Gain POT | A6 |
| Angle POT | A7 |
| Arm Switch | D2 |
| Alt Hold Switch | D3 |
| Calibrate Button | D4 |
| Motor Test Button | D5 |
| Flight Mode Button | D6 |
| Rates Button | D7 |

## Pre-Flight Checklist

### 1. Power On Calibration
- Place drone on **perfectly flat, level surface**
- Power on and **DO NOT MOVE** for 30 seconds
- Wait for "CALIBRATION SUCCESS" beeps (2 short high beeps)
- If you hear 4 low beeps, recalibrate on a flatter surface

### 2. Verify Level Reading
Check Serial Monitor shows angles near 0:
```
DIS  G:1.0x R:0.1 P:-0.2 Yr:0.0
```
If R (roll) or P (pitch) show more than ±2°, recalibrate.

### 3. Verify Motor Directions (Props OFF!)
1. Arm the drone (throttle low, switch on)
2. Tilt drone RIGHT by hand → Left motors (FL, RL) should speed UP
3. Tilt drone FORWARD by hand → Rear motors (RL, RR) should speed UP
4. If opposite, your MPU6050 may be rotated - adjust code or mounting

### 4. Arm Requirements
The drone will only arm if:
- ✅ Throttle is at minimum
- ✅ Roll angle < 3°
- ✅ Pitch angle < 3°
- ✅ Calibration was successful
- ✅ Radio is connected

If arm fails, check Serial Monitor for reason.

## PID Tuning

The default PID values are conservative for first flight:
```cpp
Roll/Pitch: Kp=1.2, Ki=0.002, Kd=0.5
Yaw:        Kp=2.0, Ki=0.001, Kd=0.0
```

### Using the Gain POT
- Rotate to adjust all PID gains together (0.5x to 1.5x)
- Start at 50% (0.5x) and slowly increase
- If oscillating, reduce gain

### Using the Angle POT
- Adjusts maximum tilt angle (15° to 35°)
- Start low for safer testing

## Troubleshooting

### Drone still drifts when holding
1. **Recalibrate** - Make sure surface is truly flat
2. Check Serial Monitor for angle values - they should be near 0
3. If angles drift over time, there's a gyro bias issue
4. Try powering on, waiting 1 minute (don't move), then arm

### Oscillations with props
1. **Reduce PID gains** using the Gain POT
2. Add vibration damping to MPU6050 (foam mounting)
3. Balance propellers
4. Check for loose motor mounts

### Won't arm
Check Serial Monitor - it will tell you why:
```
ARM FAILED:
  - Throttle not low
  - Not level (R:5.2 P:-3.1)
  - Calibration invalid
```

### Calibration keeps failing
1. Surface not flat enough - use a spirit level to check
2. Moving during calibration - be patient!
3. MPU6050 hardware issue - check connections

## LED Status

| Pattern | Meaning |
|---------|---------|
| Solid ON | Armed |
| Fast blink (100ms) | Failsafe |
| Medium blink (200ms) | Calibration failed |
| Slow blink (500ms) | Disarmed, ready |

## Buzzer Codes

| Pattern | Meaning |
|---------|---------|
| 3 short high | Startup |
| 2 short high | Calibration success |
| 4 short low | Calibration failed |
| Rising tone | Armed |
| Long low | Disarmed |
| 3 quick beeps | Radio connected |
| Long low buzz | Failsafe / Radio lost |

## Libraries Required

Install via Arduino Library Manager:
- `RF24` by TMRh20
- `I2Cdev` by Jeff Rowberg
- `MPU6050` by Jeff Rowberg

## Safety Warning

⚠️ **PROPELLERS ARE DANGEROUS** ⚠️

- Always remove propellers when testing/debugging
- Never arm indoors near people
- Use a test stand for initial flights
- Keep fingers away from spinning props
- Ensure failsafe cuts motors on signal loss

## License

MIT License - Use at your own risk. Always fly safely and follow local regulations.
