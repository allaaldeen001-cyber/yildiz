# DIY NRF24L01 Drone Stack

This workspace contains two Arduino sketches:

- `fc/Drone_Flight_control/` – quadcopter flight controller with NRF24L01, MPU6050, and MS5611 support.
- `rc/controller/` – joystick-based handheld transmitter that streams the control packet used by the FC.

Upload each sketch to its own Arduino Nano and keep the folder names unchanged so the Arduino IDE builds the multi-file projects correctly.

## Flight Controller Pin Map

| Subsystem | Pins | Notes |
| --- | --- | --- |
| NRF24L01 | CE `D4`, CSN `D10`, SPI `D11/D12/D13` | Channel 108 @ 250 kbps, fixed pipe `0xF0F0F0F0E1`. |
| Motors (PWM) | `D3` FL, `D5` FR, `D6` RR, `D9` RL | 1000–2000 µs range. |
| Sensors | I²C (`A4` SDA, `A5` SCL) for MPU6050 & MS5611 | Keep wires short and twisted. |
| Status LED / Buzzer | `D7` LED, `D8` buzzer | LED stays ON while disarmed, pulses on packet reception. |
| Battery monitor | `A0` via 1500 Ω / 1000 Ω divider | Update `R1/R2` in code if the divider changes. |
| Local controls | Arm switch `A1`, Alt-hold switch `A2`, Cal button `A3`, Smooth-start button `A6` | `A1–A3` use `INPUT_PULLUP`. `A6` is analog only: tie to VCC through 10 kΩ, pull to GND with the button. |

> **Why analog pins for the switches?** The ESC outputs already occupy `D3/D5/D6/D9` and the radio needs `D4/D10`. Adding dedicated switches there would clash with those peripherals, so the sketch exposes the control pins as constants at the top (`ARM_SWITCH_PIN`, etc.). If your wiring harness already uses other pins (e.g., `D3` for ARM), adjust those constants and recompile.

## Remote Controller Pin Map

| Item | Pin | Transformation |
| --- | --- | --- |
| Throttle (left Y) | `A0` | Inverted (`1023 - raw`) -> 1000–2000 µs. |
| Yaw (left X) | `A1` | Centered to ±500 units. |
| Pitch (right Y) | `A2` | Inverted to match “stick forward = forward flight”. |
| Roll (right X) | `A3` | Centered to ±500 units. |
| Buttons / switches | `D4` calibration request, `D5` smooth-start request, `D3` arm/disarm, `D2` altitude hold | Sent as part of the RF packet for optional remote overrides.

Both boards must share the same radio channel and pipe (already set to 108 / `0xF0F0F0F0E1LL`). Keep the transmit antenna at least 10 cm away from the FC to avoid brown-outs.

## Power-On & Flight Workflow

1. **Power the RC first.** Verify serial debug shows packets streaming (optional).
2. **Power the FC.** It plays the three-tone boot melody while initializing sensors.
3. **Watch for the link confirmation.** When the first RC packet arrives, the FC buzzer beeps once and LED D7 flashes.
4. **Disarmed state check.** Keep the arm switch (A1 input HIGH) in DISARM until you finish calibration.
5. **Calibration (Button on A3).** With the FC disarmed, hold the calibration button for a tap (edge-triggered). The FC:
   - Stops the motors.
   - Runs MPU6050 gyro bias averaging (stored in EEPROM).
   - Averages 50 MS5611 samples to lock the ground pressure baseline.
   - Signals completion with a high-pitched beep and LED flash.
6. **Arm (switch LOW).** Flip the arm switch so `A1` reads LOW. LED D7 will now pulse only when packets are received.
7. **Smooth motor start (button on A6).** Press the smooth-start button once. Motors ramp from 1000 µs to ~1250 µs over 1.5 s so you can confirm all props spin.
8. **Apply throttle and fly.** Throttle (RC A0) now controls 1000–2000 µs output. Use yaw (A1), pitch (A2), and roll (A3) as normal.
9. **Altitude hold (switch on A2).** Toggle LOW to enable the MS5611 PID. When throttle is centered (1400–1450 µs) the FC locks altitude and blends the Z PID output (`pid_output_altitude`) into the motor mix. Moving the stick above/below that window hands control back to you with a gentle manual throttle bias.
10. **Disarm to land.** Flip the arm switch back to HIGH; motors stop immediately and smooth-start must be re-triggered before the next flight.

## Built-In Safety & Indicators

- **Kill angle**: if roll or pitch error exceeds ±30°, the FC trips `killSwitch = 1`, cuts all motors, and emits a repeating 1 kHz alarm until you reset via link recovery.
- **Link loss**: >0.6 s without packets triggers `killSwitch = 2`. The FC waits for a stable RF link before re-enabling arming.
- **LED logic**: solid ON = disarmed; short pulse = packet received; dark = armed but no packets (failsafe will trigger shortly).
- **Buzzer events**: boot melody, link confirmation chirp, calibration beeps, smooth-start chirp, failsafe alarms.

## Customisation Notes

- All tunable constants (PID gains, smooth-start duration, radio channel, pin numbers) live near the top of `Drone_Flight_control.ino`.
- `SMOOTH_START_TARGET` controls the verification throttle level. Raise it if bigger props need more torque to spin.
- To remap the local switches back to different digital pins, edit `ARM_SWITCH_PIN`, `ALT_HOLD_SWITCH_PIN`, `CAL_BUTTON_PIN`, and `SMOOTH_BUTTON_PIN` and ensure those pins are not already tied to ESCs, the radio, or SPI/I²C busses.
- The RC sketch exposes the same `ControlPacket` struct; you can extend it with additional aux channels as long as both sides match the layout.

## Quick Verification Checklist

- `fc/Drone_Flight_control/` compiles for **Arduino Nano (ATmega328P)** with the Servo, RF24, Smoothed, and MS5611 libraries installed.
- `rc/controller/` compiles for another Nano, using the same libraries minus MS5611/Gyro.
- After flashing, open the FC serial monitor at 57600 baud to verify `actual_pressure` and `actual_pressure_2` streaming.
- Toggle each switch/button and confirm the expected buzzer/LED feedback before attempting a powered test.
- Always remove props when doing calibration or firmware updates.
