# Yildiz Quadcopter Flight Controller

A complete Arduino-based quadcopter flight controller with PID stabilization and auto-leveling.

## Project Structure

```
QuadcopterFlightController/
├── QuadcopterFlightController.ino  # Main flight controller
├── ESC_Calibration/                # ESC calibration tool
├── MPU6050_Calibration/            # IMU calibration tool
├── Motor_Test/                     # Motor test utility
└── TUNING_GUIDE.md                 # Complete tuning guide
```

## Features

- **Full PID Control** - Pitch, Roll, and Yaw stabilization
- **Auto-Level Mode** - Automatic horizon leveling
- **Rate Mode** - Direct rate control for acrobatics
- **Complementary Filter** - Sensor fusion for stable angles
- **Motor Mixing** - X-configuration quadcopter support
- **Arm/Disarm** - Safety arm/disarm via stick commands
- **Calibration Tools** - ESC, IMU, and motor test utilities

## Hardware Requirements

- Arduino Nano/Uno
- MPU6050 IMU (I2C)
- 4x ESCs (PWM compatible)
- 4x Brushless Motors
- RC Receiver (PWM output)
- 3S/4S LiPo Battery

## Wiring

### MPU6050
| MPU6050 | Arduino |
|---------|---------|
| VCC     | 5V      |
| GND     | GND     |
| SDA     | A4      |
| SCL     | A5      |

### Motors (X-Configuration)
| Motor | Pin | Position     | Rotation |
|-------|-----|--------------|----------|
| M1    | 4   | Front-Left   | CCW      |
| M2    | 5   | Front-Right  | CW       |
| M3    | 6   | Rear-Right   | CCW      |
| M4    | 7   | Rear-Left    | CW       |

### Receiver
| Channel | Pin | Function |
|---------|-----|----------|
| CH1     | 8   | Roll     |
| CH2     | 9   | Pitch    |
| CH3     | 10  | Throttle |
| CH4     | 11  | Yaw      |
| CH5     | 12  | Aux/Mode |

## Quick Start

1. **Upload MPU6050_Calibration** - Calibrate your IMU
2. **Upload ESC_Calibration** - Calibrate all ESCs
3. **Upload Motor_Test** - Verify motor directions
4. **Upload QuadcopterFlightController** - Flash the flight controller
5. **Tune PID** - Follow TUNING_GUIDE.md

## Arm/Disarm

- **ARM**: Throttle low + Yaw stick right
- **DISARM**: Throttle low + Yaw stick left

## Issues Fixed

The original code was **NOT a flight controller** - it was just a tilt sensor displaying angles on an LCD. This new code provides:

1. **Drift Fix** - Proper IMU calibration and complementary filter
2. **Motor Speed Uniformity** - ESC calibration tools
3. **Stability & Auto-Leveling** - Full PID control system

## Safety Warnings

- **ALWAYS remove propellers when testing on bench**
- **Calibrate ESCs before first flight**
- **Start with conservative PID values**
- **Fly in open areas away from people**

## License

Open source for educational purposes.
