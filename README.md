# Professional Nano UAV Control Stack

This repository contains two Arduino Nano sketches that work together to form a professional-grade quadcopter control stack:

- `firmware/flight_controller/flight_controller.ino` – runs on the airframe Nano, closes the control loop, and drives the ESCs.
- `firmware/remote_controller/remote_controller.ino` – runs on the handheld Nano, merges pilot inputs with safety logic, and provides live telemetry over Serial.

Both boards communicate bidirectionally through NRF24L01+ PA/LNA modules on RF channel **103** with ACK payloads, giving deterministic command delivery plus instant feedback about link health, calibration status, and flight state.

## Hardware Summary

| Subsystem | Component | Pinout |
|-----------|-----------|--------|
| Flight Controller | Arduino Nano | MCU, 5 V logic domain |
|  | NRF24L01+ PA/LNA | CE → D4, CSN → D10, MOSI → D11, MISO → D12, SCK → D13, IRQ unused |
|  | MPU6050 IMU | SDA → A4, SCL → A5, INT → D2 |
|  | Status LED | D7 (active HIGH) |
|  | Buzzer | D8 (tone capable) |
|  | ESC outputs | FL → D3, FR → D5, RR → D6, RL → D9 (Servo PWM 1000–2000 µs) |
| Remote Controller | Arduino Nano | MCU |
|  | NRF24L01+ PA/LNA | CE → D9, CSN → D10 |
|  | Left stick (Throttle/Yaw) | V → A0, H → A1 |
|  | Right stick (Pitch/Roll) | V → A2, H → A3 |
|  | Button 1 (Calibrate) | D4 (ACTIVE LOW) |
|  | Button 2 (Arm / ESC Cal) | D5 (ACTIVE LOW) |
|  | Switch 1 (Alt‑hold latch) | D2 (ACTIVE LOW = ON) |
|  | Switch 2 (Kill / Arm gate) | D3 (ACTIVE LOW = ARMED) |

> **Switch wiring:** Use the built-in pullups (`pinMode(INPUT_PULLUP)`) and wire the switch poles to GND so that **LOW = ON**. The README follows the same “0 = engaged” convention described in the requirements.

## Firmware Features

- **ACK-secured NRF24 link** on channel 103, 250 kbps, PA MAX. LED D7 blinks rapidly the moment the first valid frame is acknowledged.
- **Structured control bus** with deadbanded sticks, dual-button safety logic, and explicit state bits for calibration, arming, ESC tuning, and altitude hold.
- **MPU6050-driven attitude loop** with tunable PID controllers (±30° pitch/roll cap and ±120°/s yaw cap). Throttle is internally capped at ≈65% (≤1650 µs) for safer indoor tuning.
- **Altitude-hold latch** (SW1) freezes the last throttle value so the pilot can trim hover thrust before hands-free maneuvers.
- **State-aware buzzer + LED UX:**
  - NRF link confirmation: 3× short LED blinks (requirement #3).
  - IMU calibration success: double 2.6 kHz chirp; failure: single 7 s 600 Hz tone.
  - ESC calibration cue: dual 1.2 kHz beeps at start, alternating 1.8/0.9 kHz finish tone.
  - Kill/failsafe: low warning chirps plus fast LED pulse.
- **Motor mixer** limits tilt to 30° and sequences ESC calibration “one by one” when SW1 is low and BTN2 is pressed, exactly matching the requested workflow.
- **Serial-based ground display** on the RC side prints link health, switch/button states, and live telemetry (`state`, `faultFlags`, `imuCal`, `escCal`, packet miss counter) so the pilot can verify the full stack without needing another display.

## Software Dependencies

Install the following Arduino libraries (all via Library Manager):

- [`RF24` by TMRh20](https://github.com/nRF24/RF24)
- [`MPU6050_light` by RFetick](https://github.com/rfetick/MPU6050_light)
- Standard `Servo`, `SPI`, and `Wire` libraries (bundled with the Arduino IDE)

## Building & Flashing

1. Open the Arduino IDE and select **Arduino Nano** with the correct processor/bootloader.
2. Install the required libraries listed above.
3. Open `firmware/flight_controller/flight_controller.ino` and compile/upload it to the airframe Nano.
4. Open `firmware/remote_controller/remote_controller.ino` and upload it to the handheld Nano.
5. Keep both boards powered via stable 5 V rails; the NRF24 PA/LNA modules need a solid 3.3 V regulator (≥200 mA) plus local decoupling (10 µF + 100 nF).

## Operating Procedure (matches the user flow)

1. **Power the remote controller first.** Open the Serial Monitor at 115200 baud to watch the status panel (optional but recommended).
2. **Power the flight controller.** Within a second the RC should report `LINK:OK` and LED D7 on the FC will blink three times to confirm NRF connectivity.
3. **Verify arming switch SW2 is at “1”** (physical ON direction → logic LOW). With SW2 HIGH the FC stays disarmed and `FAULT_KILL_SWITCH` is reported.
4. **Press Button 1 (D4) to trigger gyro calibration.** Keep the frame still. Two quick chirps indicate success; a single 7 s tone means the IMU failed to calibrate.
5. **Set SW1 to “0” (logic LOW) and press Button 2** to run the ESC calibration routine. Motors will ramp one at a time with a unique buzzer melody at the end.
6. **Return SW1 to “1” (logic HIGH) for normal flight.** Keep Button 2 pressed to request arming; the FC arms only if SW2 is ON, kill is not active, radio link is good, and the IMU succeeded.
7. **Use the sticks to fly:**
   - Throttle (A0) 1000–2000 µs mapped, internally limited to 65%.
   - Yaw (A1) rotates the craft ±120°/s.
   - Pitch (A2) and Roll (A3) tilt ±30°.
8. **Altitude hold (SW1 LOW)** freezes the current throttle reference so you can fine‑tune pitch/roll without accidental climbs. Toggle OFF to regain throttle control.
9. **Emergency:** flip SW2 HIGH (logic 1) at any time to kill the motors immediately. The FC also auto-failsafes if the radio link drops for >200 ms.

## Safety & Diagnostics

- **LED D7 patterns:** solid = armed, slow pulse = linked/disarmed, medium pulse = sensor issue, fast pulse = link loss/killed. Upon first link it blinks rapidly three times per requirement.
- **Buzzer cues:** documented above; all tones are unique so you can distinguish calibration, ESC tuning, arming, and fault states audibly.
- **Telemetry bits:** `faultFlags` enumerates link loss, IMU failure, kill switch state, and ongoing ESC calibration. `state` exposes the FC finite-state machine (DISARMED, CALIBRATING, ARMED, ESC_TUNING, FAILSAFE).
- **Throttle governor:** prevents full-power surges. Adjust `maxUs` in `mapThrottleToUs()` if higher thrust is needed after initial tuning.
- **PID tuning:** `pidPitch`, `pidRoll`, and `pidYaw` gains are at the top of the FC sketch for quick refinement. Start conservative when changing prop sizes or payloads.

## Next Steps

- Integrate a barometer or ToF sensor on the I²C bus if you need true altitude hold (the current implementation maintains throttle, not absolute height).
- Feed actual battery voltage into an analog pin and update `TelemetryFrame::batteryMvHundreds` for on-screen low-voltage alerts.
- Expand the RC UI with OLED/TFT if you need richer telemetry beyond the Serial console.
