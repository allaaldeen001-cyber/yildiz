# DIY NRF24L01 Drone Platform

This repository contains Arduino sketches for a DIY quadcopter based on an Arduino Nano flight controller (FC) and an Arduino Nano remote controller (RC). The system uses an NRF24L01 link with hardware acknowledgements, an MPU6050 IMU, and an MS5611 barometer for altitude hold.

## Layout

```
fc/
  Drone_Flight_control.ino   # Main FC sketch (PID, failsafe, indicators)
  Barometer.ino              # Altitude-hold & MS5611 logic
  kalman_filter.ino          # 1D position/velocity filter for the barometer
  Gyro.h / Gyro.cpp          # MPU6050 driver with calibration helpers
rc/
  controller.ino             # Joystick-driven transmitter with ACK handling
```

## Hardware Map

### Flight Controller (Arduino Nano)
- NRF24L01: CE `D4`, CSN `D10`, channel `108`
- ESC outputs: `D3`=FL, `D5`=FR, `D6`=RR, `D9`=RL
- MS5611 barometer (I2C)
- MPU6050 gyro/accel (I2C)
- Indicators: `D7` LED, `D8` buzzer
- Battery sense: `A0` via 1.5k/1k divider

### Remote Controller (Arduino Nano)
- NRF24L01: CE `D9`, CSN `D10`, channel `108`
- Joysticks:
  - `A0` throttle (1023 − raw, outputs 1000–2000 µs)
  - `A1` yaw (left stick, center at 512)
  - `A2` pitch (1023 − raw)
  - `A3` roll
- Switches/Buttons (input pull-ups, active LOW):
  - `D3` Arm/Disarm (1 = disarmed)
  - `D2` Altitude hold enable
  - `D4` Calibration (MPU + MS5611) — hold ≥2 s while disarmed
  - `D5` Smooth motor start — tap when armed

## Key Behaviors
- NRF link uses auto-ACK plus telemetry (battery, arm flag, altitude-hold state).
- FC buzzer + LED pulse once when the RC link is established.
- FC LED stays solid while disarmed, otherwise blinks briefly whenever a valid packet arrives.
- Calibration (button D4) runs only when the arm switch is in the disarmed position; it trims the MPU6050 and re-baselines the MS5611 before acknowledging with buzzer + LED pulses.
- Arm switch LOW (D3=0) requests arming. Smooth start (button D5) ramps motors from 1050 µs for ~1.5 s so you can confirm every motor spins before take-off.
- Altitude hold (switch D2 LOW) engages MS5611 control once throttle is between 1350–1650 µs; the Kalman filter output feeds a PID that biases thrust.
- Quad automatically kills output if tilt exceeds 30°, the link is lost (>0.5 s), or throttle data is missing for >3 s. Recovery requires RF link plus disarm.

## Recommended Power-On Workflow
1. **Power the RC first.** Confirm serial output shows healthy readings.
2. **Power the FC.** Wait for the startup tone/LED pulse, then the NRF link confirmation tone once packets arrive.
3. **Disarmed state (switch D3=1).** LED stays solid ON.
4. **Run calibration (optional but recommended):** hold button D4 for ~2 s until buzzer/LED acknowledge. Keep the quad level and still.
5. **Arm:** flip switch D3 to 0. The LED begins blinking on packet reception.
6. **Smooth motor start:** tap button D5 to ramp motors gently; verify all props spin.
7. **Take off:** use throttle/attitude sticks normally. Engage altitude hold by flipping switch D2 LOW while in the hover band.
8. **Disarm:** return switch D3 to 1 after landing. LED returns to solid ON.

## Building & Uploading
- Install the required Arduino libraries: `RF24`, `Smoothed`, `MS5611` (by BlueDot or equivalent).
- Open the FC folder in the Arduino IDE and compile/upload `Drone_Flight_control.ino` to the flight controller Nano.
- Open the RC folder and upload `controller.ino` to the transmitter Nano.
- Ensure both boards share a common ground when debugging via USB to avoid floating reference issues.

## Troubleshooting
- **No link/buzzer repeating:** verify both NRF24L01 modules share the same channel (108) and address, and that `setAutoAck(true)` is active on both boards.
- **LED never blinks while armed:** packets are not arriving; check RC power or antenna.
- **Altitude hold oscillates:** tune `pid_p_gain_altitude`, `pid_i_gain_altitude`, and `pid_d_gain_altitude` in `Barometer.ino`. Start with small adjustments.
- **Unexpected motor stop:** inspect serial output for `killSwitch` triggers (excess tilt, RF loss, or throttle drop) and correct the root cause before re-arming.
