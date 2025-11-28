# yildiz

DIY drone flight control stack for Arduino Nano, NRF24L01 telemetry, MPU6050 attitude sensing, MS5611 barometer based altitude hold, and an analog joystick RC handset.

## Repository layout

- `fc/` – flight controller sketch (`Drone_Flight_control.ino`) plus the supporting barometer and gyro sources.
- `rc/` – joystick-based remote controller (`controller.ino`).

Both sketches build with the standard Arduino toolchain. Place the contents of each folder inside its own Arduino sketch directory when flashing.

## Flight controller pin map (Arduino Nano)

| Function                         | Pin  | Notes |
| -------------------------------- | ---- | ----- |
| NRF24L01 CE / CSN                | D4 / D10 | Keep CE on D4 and CSN on D10 for minimal wiring. |
| ESC front-left / front-right     | D3 / D5 | Servo outputs (Timer2/Timer0). |
| ESC rear-right / rear-left       | D6 / D9 | Servo outputs (Timer0/Timer1). |
| Status LED / Buzzer              | D7 / D8 | LED blinks on signal, solid ON when disarmed. |
| Altitude-hold switch             | D2 | LOW = altitude hold enabled. |
| Battery monitor                  | A0 | Voltage divider input (R1=1.5 kΩ, R2=1 kΩ). |
| Arm switch / Calibration button / Smooth-start button | A1 / A2 / A3 | Analog pins are used to avoid clashes with the ESC and NRF pins. Wire the switches to ground so LOW = active. |

> The original request mapped the arm switch and buttons to D3–D5, but those pins are already occupied by the ESC outputs and NRF CE line. Assigning the physical controls to A1–A3 keeps the existing motor wiring untouched while still delivering the requested behaviors.

## Remote controller pin map

| Function             | Pin | Details |
| -------------------- | --- | ------- |
| NRF24L01 CE / CSN    | D9 / D10 | Matches the FC radio channel (90) and pipe address. |
| Switches (Arm / Alt) | D3 / D2 | Pulled up, connect to ground when active. |
| Buttons (Cal / Smooth start cue) | D4 / D5 | Pulled up, connect to ground. |
| Throttle / Yaw / Pitch / Roll | A0 / A1 / A2 / A3 | Throttle & pitch are inverted in firmware (`1023 - raw`). |

## Required Arduino libraries

- [RF24](https://github.com/nRF24/RF24)
- [Smoothed](https://github.com/MartinL1/Smoothed)
- [MS5611](https://github.com/RobTillaart/MS5611) (or an equivalent driver exposing `begin`, `read`, and `getPressure`)

Install these libraries through the Arduino Library Manager or as ZIP imports before compiling either sketch.

## Operating workflow

1. **Power sequence** – Turn on the RC handset first, then power the flight controller (FC). When the NRF link is established the FC buzzer beeps once and the LED flashes briefly.
2. **Disarmed state** – Keep the hardware arm switch (A1) HIGH (open). The LED stays solid ON whenever the system is disarmed.
3. **Calibration (button on A2)** – While disarmed, hold the calibration button for ≥1.5 s to recalibrate the MPU6050 and zero the MS5611 baseline. The buzzer plays a double beep and the LED blinks.
4. **Arming** – Flip the arm switch LOW. The LED will go off except for short flashes each time a radio packet arrives. Pitch/roll/yaw commands are limited to ±30° for safety.
5. **Smooth motor start (button on A3)** – With the system armed and throttle near idle, press the smooth-start button to ramp all four ESCs from 1050 µs to ~1170 µs. This confirms that every motor spins before throttle is raised.
6. **Normal flight** – Use the joysticks as follows:  
   - Throttle (A0, inverted) → 1000–2000 µs range.  
   - Yaw (A1) → package `z`.  
   - Pitch (A2, inverted) → package `y`.  
   - Roll (A3) → package `x`.
7. **Altitude hold (switch on D2)** – Toggle LOW to lock the MS5611 altitude controller. With the throttle hovering between ~1400–1450 µs, the FC engages the PID loop (P=14, I=2, D=7.5). Moving the throttle above/below that window injects a gentle climb/descent offset.
8. **Disarming** – Flip the arm switch HIGH or trigger the failsafe. Motors stop immediately and smooth-start is reset.

## Status indicators & failsafes

- **LED (D7)** – Solid ON when disarmed, short 60 ms flashes whenever a valid RC packet arrives while armed.
- **Link confirmation** – First valid packet after a dropout triggers a buzzer chirp + LED blink.
- **Kill logic** – Motors shut down if:  
  - RC data is absent for >3 s (kill code 2).  
  - Roll or pitch error exceeds ±30° while `killAngle` is enabled (kill code 1).  
  - The hardware arm switch returns to HIGH.
- **Recovery** – For kill code 2, restoring the radio link automatically clears the latch (audible beep). For kill code 1, flip the arm switch back to HIGH to acknowledge before re-arming.

## Build & flash notes

1. Open `fc/Drone_Flight_control.ino` in the Arduino IDE. The IDE automatically pulls in `Barometer.ino`, `kalman_filter.ino`, `Gyro.cpp`, and `Gyro.h`.
2. Select *Arduino Nano* and the correct processor/port, then upload.
3. Open `rc/controller.ino` in a separate Arduino IDE window, configure it for another Nano, and upload.

## Troubleshooting checklist

- **No NRF link tone** – Confirm both sketches use channel 90 and the pipe `0xF0F0F0F0E1LL`, check CE/CSN wiring (D4/D10 on FC, D9/D10 on RC), and power the RC before the FC.
- **Buttons unresponsive** – Verify the analog pins A1–A3 are wired through normally-open switches to ground. Because these inputs use the internal pull-ups, a LOW level is required to trigger each feature.
- **Altitude hold hunting** – Re-run the calibration (button on A2) to refresh gyro/baseline pressure, ensure the throttle is within 1400–1450 µs when engaging, and confirm the barometer foam isolation is intact.

By following this structure you get a predictable workflow: power RC → power FC → link tone → calibration → arm → smooth start → fly → enable altitude hold as needed.
