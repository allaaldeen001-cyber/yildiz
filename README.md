# DIY NRF24L01 Drone

This repository contains a matched pair of Arduino sketches:

- `fc/` – flight controller firmware for an Arduino Nano that drives four ESCs, reads an MPU6050 + MS5611 stack, and keeps a bidirectional NRF24L01 link alive.
- `rc/` – handheld transmitter firmware for another Arduino Nano with two joysticks, two switches, and two momentary buttons.

Both sketches target the pinout and behaviour described in the project brief:

| Function | FC Pin | RC Pin |
|----------|--------|--------|
| NRF24L01 CE/CSN | D4 / D10 | D9 / D10 |
| ESC FL/FR/RR/RL | D3 / D5 / D6 / D9 | – |
| Buzzer / LED | D8 / D7 | – |
| Arm switch | via RC `switch1` (D3) | D3 |
| Altitude-hold switch | via RC `switch2` (D2) | D2 |
| Calibration button | via RC `but1` (D4) | D4 |
| Smooth start button | via RC `but2` (D5) | D5 |
| Joysticks | – | A0 (Throttle), A1 (Yaw), A2 (Pitch), A3 (Roll) |

## Key behaviour

- Stable NRF channel 76 with auto-ack + ack payload so both sides know link health. The FC beeps/blinks once whenever the link is established.
- RC button D4 triggers a combined MPU6050 + MS5611 calibration, but only when the arm switch is in the disarmed (HIGH) position.
- RC switch D3 controls arming (LOW = armed). LED D7 stays solid while disarmed and pulses briefly every time a command packet is received during flight.
- RC button D5 commands a smooth motor-start routine that gently ramps all ESCs to verify correct rotation before take-off.
- RC switch D2 toggles MS5611-based altitude hold. The FC keeps a Kalman-filtered vertical velocity estimate and limits corrective thrust to the 1400–1450 µs window, as in the reference implementation.
- Pitch/roll set-points are clamped to ±30° for safety, and the FC kills motors if the measured attitude exceeds that limit or if NRF packets stop arriving for >0.4 s.
- Ack payload streams battery voltage, altitude estimate, arming state, and altitude-hold status back to the RC console output.

## Building & flashing

1. Install the required Arduino libraries (`RF24`, `Smoothed`, `MS5611`, `Servo`) via the Arduino Library Manager.
2. Open `fc/Drone_Flight_control.ino` in the Arduino IDE, select *Arduino Nano* + the right processor/port, and upload.
3. Open `rc/controller/controller.ino` in another IDE window and upload it to the transmitter Nano.

## Operating workflow

1. Power the RC first, then the FC. Wait for the NRF link confirmation beep/LED blink on the FC.
2. With the arm switch HIGH (disarmed), hold Button D4 to calibrate the IMU + barometer until you hear the confirmation tone.
3. Move the arm switch LOW to arm. Press Button D5 once to run the smooth-start motor test.
4. Fly using the joysticks (Throttle uses `1023 - raw`, Pitch uses `1023 - raw`, Yaw/Roll follow direct readings). Keep throttle between 1000–2000 µs.
5. Flip Switch D2 LOW to enable altitude hold whenever you need barometric lock; flip it back HIGH to disable.
6. To land, disable altitude hold, reduce throttle, flip the arm switch HIGH, and power down FC → RC.
