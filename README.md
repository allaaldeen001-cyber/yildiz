# Build-and-Calibrate Arduino Quadcopter Drone

This workspace contains:

- `firmware/flight_controller/flight_controller.ino` – Arduino Nano flight controller firmware that fuses MPU6050 + MS5611 data, talks to an NRF24L01+ radio, guides you through calibration over Serial, and drives four ESCs with a simple PID loop.
- `firmware/radio_controller/radio_controller.ino` – Arduino Nano transmitter sketch with dual joysticks, a kill/arm toggle, calibration & spin-test buttons, and stick auto-calibration stored in EEPROM to avoid mid-stick throttle surprises.
- `docs/bringup-and-guidance.md` – complete wiring tables, calibration instructions, and the serial bring-up flow that the drone walks you through (link status, kill confirmation, Button 1 calibration, Button 2 smooth spin, ready-to-fly telemetry).

Flash the RC first, then the flight controller, connect via Serial Monitor at 115200 baud, and follow the on-screen prompts for a safe, repeatable calibration + arming sequence.
