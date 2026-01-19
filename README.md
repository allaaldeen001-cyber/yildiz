# Quadcopter Flight Controller - Arduino

A flight controller for X-configuration quadcopters with anti-oscillation features.

## Hardware Requirements

- Arduino (ATmega328P based)
- MPU6050 IMU (I2C)
- NRF24L01+ Radio module (SPI)
- 4x Brushless ESCs
- 4x Brushless Motors

## Pin Configuration

| Component | Pin |
|-----------|-----|
| Motor FL (Front-Left) | 3 |
| Motor FR (Front-Right) | 5 |
| Motor RL (Rear-Left) | 6 |
| Motor RR (Rear-Right) | 9 |
| NRF24 CE | 4 |
| NRF24 CSN | 10 |
| LED | 7 |
| Buzzer | 8 |

## Motor Layout (X-Configuration)

```
       FRONT
  FL(CCW)  FR(CW)
       X
  RL(CW)   RR(CCW)
```

## Features

- Cascaded PID control (angle loop → rate loop)
- Derivative-on-measurement to prevent D-kick
- Setpoint filtering for smooth control
- Output rate limiting
- Auto-leveling (Angle mode)
- Multiple flight modes (Angle, Horizon, Acro)
- Emergency disarm on excessive tilt/rotation
- Radio failsafe

## Files

- `quadcopter_fc_fixed.ino` - Main flight controller code with fixes
- `ANALYSIS_AND_FIXES.md` - Detailed bug analysis and fixes applied

## Setup Instructions

1. Install required libraries:
   - I2Cdev
   - MPU6050
   - RF24

2. Upload code to Arduino

3. Calibration procedure:
   - Place drone flat and level
   - Power on - automatic calibration runs
   - Wait for completion beeps

4. Arming:
   - Throttle low
   - Drone level
   - Flip arm switch
   - Listen for arm confirmation beeps

## Tuning

If experiencing oscillation:
1. Reduce `RATE_ROLL_KP` (default: 0.6)
2. Increase `RATE_ROLL_KD` (default: 0.025)
3. Reduce `ANGLE_ROLL_KP` (default: 2.5)

If experiencing drift:
1. Check axis sign constants match MPU6050 orientation
2. Verify accelerometer calibration
3. Adjust `ROLL_TRIM` and `PITCH_TRIM`

## Safety

- Always test with props removed first
- Keep throttle low during initial testing
- Use a tether for first flights
- Emergency disarm activates at >60° tilt
