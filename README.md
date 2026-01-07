# Quadcopter Flight Controller & Remote

A professional-grade Arduino-based quadcopter flight controller with NRF24L01 radio communication.

## ⚠️ IMPORTANT FIX: Motor Mixing Signs

The original code had **inverted motor mixing signs** which caused the drone to flip on takeoff instead of self-leveling. This has been corrected in the `FlightController.ino`.

### What Was Wrong

The original motor mixing:
```cpp
// WRONG - Inverted signs cause flip!
int16_t fl = baseThr - rollPID + pitchPID - yawPID;
int16_t fr = baseThr + rollPID + pitchPID + yawPID;
int16_t rl = baseThr - rollPID - pitchPID + yawPID;
int16_t rr = baseThr + rollPID - pitchPID - yawPID;
```

### The Fix

Corrected motor mixing for standard X-quad configuration:
```cpp
// CORRECT - Proper self-leveling
int16_t fl = baseThr + rollPID + pitchPID + yawPID;
int16_t fr = baseThr - rollPID + pitchPID - yawPID;
int16_t rl = baseThr + rollPID - pitchPID - yawPID;
int16_t rr = baseThr - rollPID - pitchPID + yawPID;
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
- Arduino Nano
- MPU6050 IMU (I2C: SDA=A4, SCL=A5)
- NRF24L01 Radio (CE=D4, CSN=D10)
- 4x ESCs connected to:
  - D3: Front-Left (FL)
  - D5: Front-Right (FR)
  - D6: Rear-Left (RL)
  - D9: Rear-Right (RR)
- Buzzer: D8
- LED: D7

### Remote Controller
- Arduino Nano
- NRF24L01 Radio (CE=D9, CSN=D10)
- 2x Joysticks (A0-A3)
- 2x Potentiometers (A6, A7)
- 2x Toggle Switches (D2, D3)
- 4x Push Buttons (D4-D7)
- Built-in LED: D13

## Pre-Flight Checklist

### 1. Verify Motor Directions
Before flying, verify each motor spins in the correct direction:
- FL & RR: Counter-Clockwise (CCW)
- FR & RL: Clockwise (CW)

If a motor spins the wrong way, swap any two of its three ESC wires.

### 2. Verify Propeller Installation
- CCW motors (FL, RR): Use CCW propellers
- CW motors (FR, RL): Use CW propellers

### 3. MPU6050 Orientation
The MPU6050 should be mounted:
- Flat (chip facing up)
- X-axis pointing forward (toward the "front" of the quad)

### 4. Test Self-Leveling (Props Off!)
1. Power on with props removed
2. Connect remote and arm
3. Give a small amount of throttle
4. Tilt the drone by hand
5. Watch motor speeds in Serial Monitor:
   - Tilting RIGHT → Left motors (FL, RL) should speed UP
   - Tilting FORWARD → Rear motors (RL, RR) should speed UP

If motors respond opposite, your MPU6050 may be mounted differently. Adjust the mixing signs accordingly.

## Flight Modes

1. **ANGLE (Self-Level)**: Drone automatically levels when sticks are centered
2. **HORIZON**: Mix of angle and rate mode
3. **ACRO**: Full manual rate control (advanced users only)

## Potentiometer Functions

- **Gain POT (A6)**: Adjusts PID gains (0.5x to 1.5x)
- **Angle POT (A7)**: Adjusts max tilt angle (15° to 45°)

## Troubleshooting

### Drone flips on takeoff
1. ✅ **Most likely: Motor mixing signs are wrong** (fixed in this version)
2. Check motor spin directions
3. Check propeller orientation
4. Verify MPU6050 is level during calibration

### Drone drifts in one direction
1. Recalibrate MPU6050 (keep quad perfectly level)
2. Trim joystick centers
3. Check for bent motor mounts or unbalanced props

### Oscillations/vibrations
1. Reduce PID gains using the Gain POT
2. Check for loose components
3. Balance propellers
4. Add vibration dampening to MPU6050

## Libraries Required

Install these via Arduino Library Manager:
- `RF24` by TMRh20
- `I2Cdev` and `MPU6050` by Jeff Rowberg

## License

MIT License - Use at your own risk. Always fly safely and follow local regulations.
