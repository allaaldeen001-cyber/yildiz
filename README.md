# Arduino Nano Quadcopter Flight Controller & RC Station

This project contains a complete open-source control stack for a 4-motor quadcopter built around dual Arduino Nano boards. One Nano acts as the flight controller (FC) with an MPU6050 IMU, NRF24L01 radio, buzzer, status LEDs, and 4 ESC outputs. The second Nano drives the handheld RC transmitter with dual joysticks, a toggle kill/arm switch, two push buttons, an NRF24L01, and a Nokia 5110 display that walks the pilot through every pre-flight step while showing live telemetry (throttle, roll, pitch, yaw, altitude, NRF channel, switch state).

```
firmware/
├── common/config.h                 # Shared radio channel, packet formats, GuideStep enum
├── flight_controller/flight_controller.ino   # PID-based FC firmware with IMU + ESC calibration
└── rc_transmitter/rc_transmitter.ino         # Handheld RC with display wizard + joystick calibration
```

---

## Hardware Summary

### Flight Controller Stack

| Subsystem | Details |
|-----------|---------|
| MCU | Arduino Nano (ATmega328P @ 16 MHz) |
| IMU | MPU6050 (I²C @ 400 kHz) |
| Radio | NRF24L01 (CE→D7, CSN→D8, hardware SPI) |
| ESC outputs | Servo PWM on D3 / D5 / D6 / D9 |
| Buzzer | D10 (active-high, non-blocking pattern driver) |
| Status LED | D4 blinks for link + packet acks |
| Armed LED | D2 solid when FC considers motors armed |
| Battery sense | A0 with 100 kΩ : 10 kΩ divider (max 25 V) |
| Power | 5 V BEC / regulator feeding Nano + sensors |

### RC Transmitter Stack

| Subsystem | Details |
|-----------|---------|
| MCU | Arduino Nano |
| Radio | NRF24L01 (CE→D8, CSN→D9, SPI pins 11/12/13) |
| Joysticks | Two 2-axis pots on A0..A3 (Throttle, Yaw, Pitch, Roll) |
| Toggle switch | D4 (INPUT_PULLUP) — KILL when LOW, ARM when HIGH |
| Button 1 | D2 — requests IMU+ESC calibration on FC |
| Button 2 | D3 — requests smooth idle spin check |
| Display | Nokia 5110 (Adafruit_PCD8544) via software SPI (CLK→A4, DIN→A5, DC→D5, CS→D6, RST→D7) |
| Power | 2S LiPo or 5 V USB, with NRF & LCD on 3.3 V regulator |

---

## Required Arduino Libraries

Install via **Tools → Manage Libraries…** (or `arduino-cli lib install`):

- `RF24` by TMRh20 (radio layer for both sketches)
- `Adafruit GFX Library` + `Adafruit PCD8544 Nokia 5110 LCD Library` (RC display)

Everything else relies on Arduino core libraries (`Wire`, `Servo`, `EEPROM`, `SPI`, `math`).

---

## Firmware Highlights

- **Shared radio protocol:** `config.h` defines a packed control frame (1000–2000 µs channels, kill switch, button states) plus a telemetry ACK payload (altitude, battery, link quality, GuideStep). Both sketches include it to guarantee identical packet layouts.
- **Complementary-filter IMU fusion:** The FC reads the MPU6050 at 400 Hz, removes saved gyro/accel biases, and blends gyro integration with accelerometer tilt for low-drift roll/pitch. Relative altitude is estimated from accelerometer Z projected onto the world frame (enough for “high altitude” readouts on the RC display).
- **PID attitude control:** Tunable PID loops (`rollPid`, `pitchPid`, `yawPid`) keep the quad level. Outputs are mixed into the four ESC channels with idle-throttle clamping so all motors ramp evenly. See `updateMotors()` inside `firmware/flight_controller/flight_controller.ino` for the full mixer.
- **Guided arming workflow:** Both FC and RC implement a `GuideStep` state machine (WAIT_LINK → REQUEST_KILL → REQUEST_CAL → CALIBRATING → REQUEST_ARM → REQUEST_IDLE_SPIN → READY → FLYING). The RC display shows the current step and a short instruction string, so the pilot performs actions in a safe order. The FC enforces the same state transitions internally before it ever raises motor outputs.
- **Joystick auto-calibration + center fix:** Holding **both buttons while powering the RC** launches a wizard that captures stick centers and extremes, saves them to EEPROM, and the firmware maps the axes with deadband/expo so the joystick center really corresponds to 1500 µs. Throttle is forced to 1000 µs whenever the kill switch is engaged, eliminating the “center equals 1000” hazard the user reported.
- **ESC + IMU calibration on demand:** Pressing Button 1 while the kill switch is in **KILL** sends a command to the FC. The FC averages 3000 IMU samples, saves gyro/acc biases + ESC range to EEPROM, then runs a classic ESC high/low calibration sequence (props OFF). Completion beeps (3×) and automatically advances the guide to the arming step.
- **Smooth idle spin check:** After arming, the FC waits for Button 2 to request an idle spin. Idle throttle ramps from 1000 µs to 1120 µs to verify all motors respond evenly before enabling full flight control. This satisfies the “smooth turn not flying” requirement.
- **Feedback everywhere:** The FC buzzer beeps twice on power, twice again on radio link, triple after calibration, and four short beeps on failsafe. The status LED blinks at 2 Hz when packets arrive and flashes briefly whenever either button press is acknowledged, letting you know the NRF link is healthy. The RC display shows NRF channel number, live stick values, altitude, switch state, and link state.

---

## Building & Flashing

1. **Clone / copy** this repo into your Arduino sketch folder or open the `.ino` files directly in the IDE.
2. **Install libraries** listed above on both development machines (FC & RC).
3. **Flight controller board**
   - Open `firmware/flight_controller/flight_controller.ino`.
   - Select **Board:** Arduino Nano (Old Bootloader if using a clone) and the correct serial port.
   - Verify/Upload.
4. **RC transmitter board**
   - Open `firmware/rc_transmitter/rc_transmitter.ino`.
   - Same board/port selection (second Nano).
   - Verify/Upload.
5. Power each board from USB once to populate EEPROM defaults (the RC wizard will launch automatically if calibration data is missing).

---

## RC Joystick Calibration (Fixes the center=1000 issue)

1. **Power off** the RC.
2. **Hold Button 1 and Button 2** (both tied to GND when pressed).
3. **Apply power**. The Nokia 5110 screen will show “Hold sticks centered…”.
4. Release the sticks and keep them centered until prompted (≈1.5 s).
5. When asked, **move every stick to all extremes** (full circles) for ~6 s.
6. Wait for “Stick cal saved” and release the buttons.
7. Calibration data is stored in the RC Nano’s EEPROM; repeat any time pots drift or you replace a joystick by holding both buttons during boot.

With valid calibration, throttle, roll, pitch, and yaw all map to 1500 µs at center, ±500 µs travel, and a 2 % deadband to kill noise. Throttle output is force-clamped to 1000 µs whenever the toggle switch is in KILL, so motors cannot spool up accidentally.

---

## Guided Startup Sequence (Display-driven)

The RC display always shows the NRF state, current `GuideStep`, and a concise instruction string. Follow it in order:

1. **WAIT_LINK / “Power FC+RC”** – Turn on both units. The FC buzzer gives two short beeps once the NRF link is alive.
2. **REQUEST_KILL / “Toggle→KILL”** – Move the toggle switch to the KILL (safe) position. The FC confirms and advances.
3. **REQUEST_CAL / “BTN1 → CAL”** – Press Button 1 to run the combined IMU + ESC calibration. Keep the drone flat and props off. The display shows “Calibrating…” while the FC averages sensors and pulses the ESCs high/low.
4. **REQUEST_ARM / “Toggle→ARM”** – After calibration beeps, flip the toggle to ARM. The FC lights the Armed LED and bumps the buzzer twice.
5. **REQUEST_IDLE_SPIN / “BTN2 → Idle”** – Press Button 2. Motors slowly ramp to 1120 µs for a smooth spin test without takeoff. If everything feels balanced, release.
6. **READY / “Ease throttle”** – Throttle is now live (still constrained by idle min). Raise it gently to lift. If you bring throttle back near idle, the guide reverts to READY; if you drop the toggle back to KILL the system returns to REQUEST_ARM and forces all motors off.
7. **FLYING / “Fly safe & chk”** – Once throttle exceeds idle, the display shows FLYING along with live telemetry. The FC will fall back to READY when throttle returns to idle, or to REQUEST_ARM if you hit the kill switch.

---

## LED & Buzzer Cheat Sheet

| Signal | Behavior | Meaning |
|--------|----------|---------|
| Buzzer (two short beeps) | On power & on NRF link | Successful startup / handshake |
| Buzzer (three beeps) | After Button 1 calibration | IMU + ESC calibration completed and saved |
| Buzzer (two medium beeps) | After Button 2 idle spin ramp | Motors spooled smoothly |
| Buzzer (four rapid beeps) | Link lost >300 ms | Failsafe triggered; motors forced to 1000 µs |
| Status LED (D4) | Slow blink at 2 Hz | NRF packets streaming |
| Status LED pulse | Brief HIGH when buttons pressed | RC command received/acknowledged |
| Armed LED (D2) | Solid HIGH | FC currently armed (toggle in ARM, guide past calibration) |

---

## Battery Sensing & Power Notes

- Wire a **100 kΩ (R1)** from battery positive to A0 and a **10 kΩ (R2)** from A0 to GND. The code assumes that divider to compute pack voltage. Adjust `DIVIDER_RATIO` in `flight_controller.ino` if you pick different resistors.
- Keep all ESC grounds tied to the Nano ground. Only power the NRF24L01 modules from a clean 3.3 V regulator (not the raw BEC unless it’s low-noise).
- During ESC calibration (Button 1) **remove props** and make sure the LiPo is fully charged; ESCs need to hear the 2 s high + 2 s low pulses without interruptions.

---

## Safety & Testing Checklist

1. **Bench first:** Power the FC with props removed. Check the Nokia display walks through kill → calibration → arm steps, and verify the FC buzzer + LEDs respond exactly when expected.
2. **Radio link:** Walk 10 m away and confirm link quality (`LQ%` on display) stays near 100. The FC status LED should keep blinking; if it turns solid off, inspect wiring/antennas.
3. **Sensor orientation:** Tilt the frame gently and watch the RC telemetry numbers change in the correct direction (roll positive when tilting right, etc.). Reverse ESC wiring if necessary.
4. **Idle spin test:** With props still off, press Button 2 after arming. Ensure all motors spin evenly and no ESC fails to start. The FC automatically refuses to fly until this step succeeds.
5. **Fail-safe drill:** Turn off the transmitter mid-idle spin; the FC should emit four quick beeps, set `GuideStep` back to WAIT_LINK, and drop all ESC outputs to 1000 µs.
6. **Live flight (outdoors, open area):** Reinstall props, repeat the guided flow, hover low, and tune PID gains in `flight_controller.ino` if necessary (`rollPid`, `pitchPid`, `yawPid`). Start with small adjustments (±0.2 on `kp`) and recompile.

---

## Future Extensions

- Integrate a barometer (BMP280) or GPS for true altitude vs. current IMU-derived “relative” estimate.
- Add magnetometer support for drift-free yaw.
- Implement rate/acro flight modes or expo settings adjustable from the RC display.
- Log telemetry frames to an SD card on the RC for post-flight analysis.

With the supplied code and workflow you can build a stable, step-by-step guided quadcopter experience that meets the user’s original request: automatic calibration, safe arming with kill switch enforcement, joystick calibration to eliminate the 1000 µs center issue, audible/visual feedback, and a Nokia 5110 screen that tells the pilot exactly what to do before every flight.
