# Arduino Nano NRF24 Quadcopter

A complete end-to-end build for a stable brushed/brushless quadcopter that uses **two Arduino Nano boards** and an **nRF24L01+ link**. This repository contains both firmwares (flight controller + handheld radio) together with wiring, calibration, and guided bring-up notes so you can reproduce a reliable setup with the exact hardware listed in the brief.

> ⚠️ **Remove propellers during every calibration or bench test.** The workflow below assumes bare motors until the very final checks.

## Hardware Overview

### Flight Controller Stack
- Arduino Nano (ATmega328P)
- MPU6050 6-DoF IMU (I²C)
- MS5611 barometer (I²C)
- nRF24L01+ transceiver (SPI)
- 4 × ESCs + brushless motors (PWM from D3/D5/D6/D9)
- Active buzzer (D4)
- Status LED (D13)

### Handheld RC Transmitter
- Arduino Nano
- nRF24L01+ (SPI)
- 2 × joysticks
  - Left stick: throttle (vertical) + yaw (horizontal)
  - Right stick: pitch (vertical) + roll (horizontal)
- Toggle switch (Kill ⬇ / Arm ⬆)
- Button 1 → Calibration command
- Button 2 → Smooth idle spool command
- Status LED (blinks on each successful packet)

## Wiring Maps

### Flight Controller
| Signal | Nano Pin | Target | Notes |
| --- | --- | --- | --- |
| MPU6050 SDA | A4 | MPU6050 SDA | Share I²C with MS5611 |
| MPU6050 SCL | A5 | MPU6050 SCL | Pull-ups already on breakout |
| MS5611 SDA | A4 | MS5611 SDA | |
| MS5611 SCL | A5 | MS5611 SCL | |
| nRF24 CE | D7 | nRF24 CE | |
| nRF24 CSN | D8 | nRF24 CSN | D10 kept as OUTPUT to keep SPI master |
| nRF24 MOSI | D11 | nRF24 MOSI | Hardware SPI |
| nRF24 MISO | D12 | nRF24 MISO | Hardware SPI |
| nRF24 SCK | D13 | nRF24 SCK | Same pin also drives status LED (ok) |
| ESC Front-Left | D3 | ESC signal | Servo library (1000–2000 µs) |
| ESC Front-Right | D5 | ESC signal | |
| ESC Rear-Right | D6 | ESC signal | |
| ESC Rear-Left | D9 | ESC signal | |
| Buzzer + | D4 | Active buzzer | LOW = off |
| Status LED + | D13 | LED/buffer | Blink on packet receipt |
| 5V, GND | 5V, GND | All modules | Keep noise low, add decoupling |

### Radio Controller
| Signal | Nano Pin | Target | Notes |
| --- | --- | --- | --- |
| Joystick 1 vertical (Throttle) | A0 | Pot wiper | Remove return spring or keep stick pulled down when arming |
| Joystick 1 horizontal (Yaw) | A1 | Pot wiper | |
| Joystick 2 vertical (Pitch) | A2 | |
| Joystick 2 horizontal (Roll) | A3 | |
| Button 1 | D2 | Push button → GND | Active LOW, internal pull-up |
| Button 2 | D3 | Push button → GND | Active LOW |
| Kill/Arm toggle | D4 | SPDT to GND/5V | DOWN = GND (kill), UP = 5V (arm) |
| Status LED | D5 | LED + resistor | Blink feedback |
| nRF24 CE | D7 | Radio CE | |
| nRF24 CSN | D8 | Radio CSN | |
| nRF24 MOSI/MISO/SCK | D11/D12/D13 | Radio SPI | |
| 5V, GND | 5V, GND | All modules | Add 10 µF near radio |

## Firmware Layout

```
firmware/
├── flight_controller/
│   └── flight_controller.ino
└── radio_transmitter/
    └── radio_transmitter.ino
```

- `flight_controller.ino` implements the full stabilization stack, NRF link, buzzer/LED logic, ESC drive, PID loop, MS5611 altitude hold primitives, EEPROM-backed calibration, and the serial guide requested in the prompt.
- `radio_transmitter.ino` covers joystick calibration (including the throttle safety fix), command buttons, toggle switch handling, telemetry echo, and link feedback LED behavior.

## Flash + Bench Checklist

1. **Install libraries** in the Arduino IDE (or PlatformIO): `RF24`, `Adafruit_MPU6050`, `Adafruit_Sensor`, `MS5611`, and `Servo` (bundled).
2. **Flash the transmitter** first (`firmware/radio_transmitter/radio_transmitter.ino`).
   - Hold **Button 1 while powering** the RC to enter stick calibration. Move joysticks to every extreme, then release pitch/roll/yaw to center and keep throttle fully down until the LED stops flashing.
3. **Flash the flight controller** (`firmware/flight_controller/flight_controller.ino`). Leave props off.
4. **Open the FC Serial Monitor** at `115200` baud. This session walks you through the entire bring-up.
5. **Power both sides**. When the NRF link comes up, the FC buzzer chirps twice and the status LED toggles every packet.

## Guided Serial Workflow (per requirement)

The FC sketch prints each instruction only after the previous step is satisfied, so you cannot skip safety gates:

1. `Waiting for RC link…` → ensure both units are powered; NRF link success is confirmed with `RC link OK…` and a double chirp.
2. `Move toggle DOWN (kill)…` → set the transmitter switch to SAFE (kill). FC acknowledges before continuing.
3. `Press Button 1 for full calibration.` → Button 1 command launches IMU, MS5611, and ESC calibration. Results are stored in EEPROM on the FC, then it advances automatically.
4. `Flip toggle UP to arm…` → throttle must stay at the learned idle (≤1000 µs) or the FC refuses to arm.
5. `Press Button 2 for smooth idle spin.` → FC ramps motor outputs from 1000 µs to idle (~1120 µs) without sudden jumps.
6. `Motors spinning at idle. Drone ready for throttle inputs.` → at this point the Serial Monitor shows continuous telemetry lines with throttle/yaw/pitch/roll/altitude/link %, plus the current guide step ID. Any kill-switch change or link drop forces the workflow back to the appropriate step.

## Throttle Safety Fix

Cheap gimbals self-center around ~512 ADC counts, so naïvely mapping `0–1023 → 1000–2000` would indeed put the stick **center at ~1500 µs** and the bottom at ~1000 µs—meaning the quad would hover as soon as the stick springs to center. The transmitter firmware solves this by:

- Running an **EEPROM-backed calibration** that learns true minimum/maximum counts for each stick.
- Treating the throttle axis as **unipolar**: bottom position becomes `0.0`, and the expo/curve is only applied above that point.
- Enforcing a **deadband** (2 % of the range) so the FC always sees exactly `1000 µs` until you physically push the stick up.
- Requiring the kill switch to be SAFE and throttle to be at the learned minimum before arming.

## Calibration Details

- **RC joystick calibration** (optional but recommended anytime hardware changes): hold Button 1 while powering the transmitter, move joysticks to all extremes for ~5 s, release pitch/roll/yaw to the center, keep throttle down, and wait for the LED to stop blinking. Data is stored in EEPROM.
- **Flight controller calibration (Button 1 command)**:
  - Samples 2000 IMU readings to derive gyro + accelerometer biases (saved to EEPROM).
  - Samples the MS5611 to capture the current sea-level pressure reference.
  - Performs a full ESC calibration routine (max → min). Props **must** be removed.
- **Smooth spool (Button 2)** ramps motor outputs by 1.5 µs per loop until they reach idle, so ESCs never desync.

## Telemetry + Debugging

The FC pushes the following fields back through the NRF ACK payload (displayed on the RC serial output and the FC serial console):

- `Throttle` (raw µs command received)
- `Attitude` (roll/pitch/yaw ×10 for resolution)
- `Altitude` in meters (MS5611-derived)
- `Link%` based on NRF frame counters
- `Setup Step` (0–5) so you always know where you are in the guided sequence
- `Flags` bitmask (armed, motors idling)
- `Channel` confirmation (defaults to 108)

## Next Steps & Tuning

- Start with modest PID gains (already set) and record logs via the Serial Monitor before any aggressive maneuvers.
- Adjust `pidRoll`, `pidPitch`, `pidYaw` in `flight_controller.ino` for your airframe mass and prop size.
- Once stable, you can raise `radio.setDataRate` to `RF24_2MBPS`, but only after confirming link quality.
- Always re-run the RC stick calibration if you replace joysticks or notice throttle drift.

Happy flying, and stay safe! ✈️
