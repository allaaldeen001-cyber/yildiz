# Yildiz - Arduino Nano Quadcopter Flight Controller

A DIY quadcopter flight controller for Arduino Nano with MPU6050 IMU and NRF24L01 radio.

## Features

- **Anti-Oscillation PID** - Cascaded PID architecture eliminates the classic tilt-overcorrect oscillation problem
- **500Hz Control Loop** - Fast IMU updates for responsive control
- **Safety Features** - Auto-disarm on 60° tilt, RF loss protection
- **Multiple Flight Modes** - Angle, Horizon, and Acro modes

## Files

- `quadcopter_fc_stable.ino` - Main flight controller code with anti-oscillation improvements
- `PID_TUNING_GUIDE.md` - Comprehensive guide for tuning PID gains

## Anti-Oscillation Improvements

The classic oscillation problem:
```
Drone tilts left → PID overcorrects → Overshoots right → Repeat
```

This is solved with:

1. **Cascaded PID** - Outer angle loop feeds inner rate loop
2. **Derivative on Measurement** - Eliminates D-term kick on setpoint changes  
3. **Setpoint Filtering** - Smooths target angle transitions
4. **Output Rate Limiting** - Prevents sudden motor changes

## Hardware Requirements

- Arduino Nano
- MPU6050 IMU
- NRF24L01+ Radio Module
- 4x ESCs and Brushless Motors
- Compatible RC Transmitter

## Motor Layout (X-Configuration)

```
       FRONT
  FL(CCW)  FR(CW)
      \  /
       \/
       /\
      /  \
  RL(CW)  RR(CCW)
```

## Quick Start

1. Connect hardware according to pin definitions in code
2. Upload `quadcopter_fc_stable.ino` to Arduino Nano
3. Place drone flat for calibration on power-up
4. Follow `PID_TUNING_GUIDE.md` if oscillation occurs

## PID Tuning

If you still experience oscillation after uploading the new code:

1. **Reduce `RATE_ROLL_KP`** from 0.5 to 0.3
2. **Increase `RATE_ROLL_KD`** from 0.015 to 0.025
3. **Reduce `ANGLE_ROLL_KP`** from 3.0 to 2.0

See `PID_TUNING_GUIDE.md` for detailed tuning instructions.
