# DIY NRF24L01 Drone Stack

This workspace contains two Arduino sketches that work together to run a DIY quadcopter with an NRF24L01 radio link, MPU6050 IMU, MS5611 barometer, joystick-based RC transmitter, dual switches, two buttons, a status LED, and a buzzer.

## Repository layout

- `fc/Firmware/` – Flight-controller sketch split into `Drone_Flight_control.ino`, `Barometer.ino`, `kalman_filter.ino`, `Gyro.h`, and `Gyro.cpp`.
- `rc/Controller/` – Remote-controller sketch that reads the joysticks, switches, and buttons, then transmits the `Package` payload to the FC.

## Hardware mapping

### Flight controller (Arduino Nano)
- NRF24L01: `CE -> D4`, `CSN -> D10`, powered from 3.3 V with a local decoupling cap.
- ESC outputs: `D3 (FL)`, `D5 (FR)`, `D6 (RR)`, `D9 (RL)` with 1000–2000 µs pulses.
- LED status: `D7` (active HIGH).
- Buzzer: `D8`.
- I2C sensors: MPU6050 + MS5611 on `A4/A5`.

### Remote controller (Arduino Nano)
- NRF24L01: `CE -> D9`, `CSN -> D10`.
- Joysticks: `A0 (Throttle)`, `A1 (Yaw)`, `A2 (Pitch)`, `A3 (Roll)`.
- Switches: `D3 (Arm/Disarm, 1=disarmed)`, `D2 (Altitude-hold enable)`.
- Buttons: `D4 (Calibration request)`, `D5 (Smooth motor-start)`.

## Feature summary

- Stable 250 kbps NRF link on channel 108 with LOW PA level for noise immunity.
- Link confirmation: when the FC receives the first valid packet it beeps and flashes the LED; a low tone plays if the link drops for >0.5 s.
- LED logic: solid ON while disarmed (Arm switch = 1). When armed, the LED flashes for ~120 ms after each control packet to confirm radio traffic.
- Safety envelope: command angles are clamped to ±30° and the controller trips `killSwitch` if measured roll/pitch exceed that limit.
- Calibration: Button D4 (held >2 s while disarmed) calibrates the MPU6050 offsets and the MS5611 ground pressure. Confirmation is a two-tone buzzer sweep plus LED flash.
- Arming: Switch D3=0 enables arming (provided link is alive and no kill condition). Switch D3=1 forces disarm and solid LED.
- Smooth-start: Once armed, press Button D5 to ramp all ESCs from 1000 µs to 1050 µs before normal throttle takes over.
- Altitude hold: Switch D2=0 engages the MS5611-based altitude PID when the throttle is near mid-stick. Manual throttle excursions temporarily bias the hold loop.
- Buzzer cues: startup fanfare, link acquired, arming, disarming, calibration, smooth-start complete, and failsafe/kill events.

## Uploading the sketches

1. Open the Arduino IDE (or CLI) and select "Arduino Nano" with the bootloader that matches your board.
2. Install the required libraries if you have not already:
   - `RF24` (TMRh20 fork)
   - `Smoothed`
   - `MS5611`
3. For the flight controller:
   - Open all files inside `fc/Firmware/` as a single sketch (Arduino treats `.ino` + `.cpp/.h` within the same folder as one build target).
   - Select the FC Nano serial port and upload.
4. For the remote controller:
   - Open `rc/Controller/controller.ino`.
   - Select the RC Nano serial port and upload.

## Pre-flight workflow

1. **Power on the RC first.** Verify the serial monitor prints joystick/packet data.
2. **Power on the FC.** You should hear the startup tones followed by a short beep/blink once the radio link is established.
3. **Disarmed state (Switch D3 = 1):** The LED stays solid ON. Move the sticks to confirm telemetry flashes on the RC.
4. **Calibration (optional but recommended when moving locations):**
   - Keep Switch D3 = 1 (disarmed).
   - Hold Button D4 for >2 s. The buzzer will emit a double tone while the FC calibrates the MPU6050 and averages the MS5611 baseline. Wait for the confirmation chirp before proceeding.
5. **Arming:** Flip Switch D3 to 0. You should hear the arming beep, but motors will still output only the idle minimum until smooth-start completes.
6. **Smooth-start:** Press Button D5 once. The FC ramps every ESC from 1000 µs to 1050 µs over ~0.3 s so you can visually confirm all motors spin. After the confirmation beep, throttle commands become active.
7. **Flight:**
   - Use the joysticks as usual (Throttle=A0, Yaw=A1, Pitch=A2, Roll=A3). The FC LED will blink briefly on each packet.
   - Flip Switch D2 to 0 at mid-throttle to enable altitude hold. Pushing throttle above ~1650 µs or below ~1350 µs temporarily biases the altitude PID for manual climbs/descents.
8. **Failsafe behavior:**
   - Loss of radio for >3 s sets `killSwitch=2`, stops the motors, and plays a repeating warning tone until the link returns.
   - Excessive roll/pitch (>30°) triggers `killSwitch=1`, halting the motors and beeping until the frame is level again.
9. **Shutdown:** Disarm via Switch D3=1, wait for the LED to turn solid, then power down the FC followed by the RC.

## Testing tips

- Keep the props off for the first bench test. Use the serial monitor on the FC to observe `actual_pressure`, link state, and arming flags.
- If the NRF link is noisy, add a 10 µF electrolytic capacitor across the NRF module's 3.3 V and GND pins.
- Re-run calibration whenever you move to a significantly different elevation or temperature.
- Use the RC serial output to confirm switch/button states before powering the FC.
