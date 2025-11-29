# Professional Nano-Based Quadcopter Reference Platform

This repository contains a complete dual-Arduino UAV stack: a **flight controller (FC)** that closes the attitude/altitude loop on-board the airframe, and a **remote controller (RC)** handset that streams pilot inputs, shows link health on the serial monitor, and orchestrates safety/calibration workflows. Both sides communicate bidirectionally over high-power NRF24L01+ PA/LNA radios operating on channel **103** with ACK payloads so every command is positively confirmed.

> **Key goals**
> - Professional-grade pin mapping, state machines, and failsafes for the listed hardware.
> - Deterministic sensor fusion (MPU6050 IMU + MS5611 barometer), PID loops, and ESC mixing with 65% throttle cap & ±30° attitude limit.
> - Explicit workflows for calibration, ESC setup, arming, kill switch, and status/buzzer feedback.

---

## Repository Layout

```
platformio.ini          # Multi-env PlatformIO project targeting two Nano boards
src/
  ├─ common/           # Shared sensor & math drivers
  │    ├─ mpu6050.cpp
  │    └─ ms5611.cpp
  ├─ flight_controller/
  │    └─ main.cpp     # FC firmware
  └─ remote_controller/
       └─ main.cpp     # RC firmware
include/
  ├─ rf_protocol.h     # Shared radio structs / enums
  ├─ pid.h             # Lightweight PID helper
  ├─ filters.h         # Complementary attitude filter
  ├─ mpu6050.h
  └─ ms5611.h
```

Both environments build with the same shared headers via PlatformIO; you can also copy each `main.cpp` into an `.ino` sketch if you prefer the Arduino IDE (keep the `include/` directory as a sibling so the headers resolve).

---

## Hardware Summary

### Flight Controller Board (Arduino Nano)

| Peripheral | Pins | Notes |
|------------|------|-------|
| NRF24L01+ PA/LNA | CE:D4, CSN:D10 | SPI pins are D11-D13 (hardware). RF set to channel 103 w/ ACK payloads. |
| MPU6050 | I²C + INT:D2 | 500 dps / ±4 g, complementary filter fusion. |
| MS5611 Barometer | I²C | OSR 4096, 50 ms cadence for altitude hold. |
| ESC Outputs | D3 (FL), D5 (FR), D6 (RR), D9 (RL) | Servo library drives 1000–1650 µs (65% cap). |
| Status LED | D7 | Slow blink = searching, medium = linked, fast solid = armed. |
| Buzzer | D8 | Distinct patterns for calibration, faults, ESC routine. |

### Remote Controller Board (Arduino Nano)

| Control | Pins | Function |
|---------|------|----------|
| NRF24L01+ PA/LNA | CE:D9, CSN:D10 | Shares the same pipe/channel as FC. |
| Left Joystick | V:A0 (Throttle), H:A1 (Yaw) |
| Right Joystick | V:A2 (Pitch), H:A3 (Roll) |
| Button 1 | D4 | IMU calibration request. |
| Button 2 | D5 | Motor-on/arm request & ESC calibration trigger. |
| Switch 1 | D2 | Altitude-hold toggle (`1`=hold). Also used to enter ESC calibration when flipped to `0`. |
| Switch 2 | D3 | Kill/arming safety (`1`=allow arming, `0`=kill). Wired with other switch poles to GND per spec. |

> **Spec resolution:** The original brief had conflicting wording for switch roles. This implementation follows the numbered list: `SW1` toggles altitude hold (and gates ESC calibration when set to `0`), `SW2` is the kill/arming safety. Button 2 starts the motors (arming) but never disarms—flip `SW2` low to kill instantly.

---

## Communication & Control Architecture

- **RF transport:** NRF24 channel 103, 1 Mbps, auto-ACK with 16-byte command frames and 18-byte status payloads (`include/rf_protocol.h`). Retries are enabled (5×15) for resilience.
- **Link supervision:** The FC tracks the time of the last valid frame; >250 ms silence raises a radio error, disarms the motors, and plays a low-frequency alert. LED D7 reverts to the slow “searching” blink pattern until another ACKed frame arrives.
- **Command frame contents:** 1000–2000 µs throttle, ±500-unit attitude sticks (mapped to ±30°), ±500 yaw rate (mapped to ±120 °/s), switch/btn bitfields, and momentary flag bits for IMU calibration, ESC calibration, and arming requests.
- **Status payload:** Bit flags for link, calibration state, arming, altitude hold, kill state, and sensor faults plus roll/pitch (×10), yaw rate, altitude, loop time, and an error code (`NONE`, `IMU`, `BARO`, `LINK`, `WDOG`). The RC serial monitor shows this real-time “glass cockpit”.

---

## Flight Controller Firmware (`src/flight_controller/main.cpp`)

**Core features**
- Complementary filter fuses gyro & accel at 500 Hz, MS5611 altitude updates every 50 ms. PID loops (roll/pitch/yaw/altitude) are modular (`include/pid.h`).
- Motor mixer enforces ±30° command limit and a 65 % throttle ceiling (1650 µs) for safety. Slew-limited ESC calibration routine ramps each motor sequentially with a distinctive buzzer melody.
- Safety handling:
  - Kill switch low (`SW2=0`) or radio timeout → immediate disarm + alert tone.
  - Button 2 only arms when `SW2=1`, IMU + ESC calibrations are complete, and the FC is currently disarmed. Disarming must be done via the kill switch.
  - Altitude hold engages when `SW1=1`; target altitude is latched on the rising edge and controlled via the barometer PID.
- Calibration logic:
  - **IMU calibration (Button 1)** averages 1 500–2 000 samples, stores gyro/accel offsets, and waits for user confirmation. Success = double beep; failure = single 7 s tone.
  - **ESC calibration (Button 2 while `SW1=0`, `SW2=0`, disarmed)** sweeps each ESC from min to capped max, confirming with a different two-tone chirp.
- Diagnostics: D7 blink speeds communicate link/armed state, buzzer patterns differentiate calibration types, and `Serial` is available for deeper logging if needed.

---

## Remote Controller Firmware (`src/remote_controller/main.cpp`)

- Samples the joysticks at 50 Hz, applies dead-bands, and maps to FC command ranges.
- Debounces buttons/switches, emitting one-shot command flags:
  - Button 1 → IMU calibration flag.
  - Button 2 → (a) ESC calibration when **both** `SW1=0` and `SW2=0`, or (b) arm request when `SW2=1`.
- Serial monitor output every 250 ms doubles as an on-screen status panel:
  - Displays link state, raw input states, FC arming & mode flags, IMU/ESC calibration status, live altitude, attitude angles, and the latest FC error mnemonic.
  - The RC remains fully functional without the monitor, but the textual HUD is a convenient pre-flight checklist.

---

## Building & Uploading

1. **Install PlatformIO** (VS Code extension or CLI).
2. Connect the desired Nano over USB.
3. From the repo root run:

   ```bash
   # Flight controller (CE=D4 board)
   pio run -e flight_controller -t upload

   # Remote controller (CE=D9 board)
   pio run -e remote_controller -t upload
   ```

4. Use `pio device monitor -b 115200` on the RC port to view the live status panel.

> **Arduino IDE users:** Copy `src/flight_controller/main.cpp` (and the headers under `include/`) into an FC sketch folder, and the same for the RC. The code is 100 % Arduino API compatible; PlatformIO simply streamlines dependency management.

---

## Operational Checklist

1. **Power on the RC first**, verify the serial console shows `NO LINK`.
2. **Power the FC**. LED D7 will blink faster once the NRF link is established (ACK confirmed).
3. Ensure kill switch `SW2` is at `1` (safe for arming) and throttle is low.
4. Press **Button 1** to run IMU calibration. Wait for double-beep success (single long tone = retry while keeping the drone level).
5. To calibrate ESCs: set `SW1` to `0`, set `SW2` to `0`, then press **Button 2**. Motors ramp one by one and the buzzer plays the ESC tone when finished.
6. Flip `SW1` back to the desired position (0 = manual altitude, 1 = hold). Set `SW2=1`, then tap **Button 2** once to arm. Motors spool to idle and the LED goes into the fast blink mode.
7. Use the sticks to fly (throttle up/down, yaw on left stick horizontal, pitch on right vertical, roll on right horizontal). Altitude hold limits throttle adjustments to the barometer PID; manual mode obeys the throttle stick directly.
8. **Kill anytime** by flipping `SW2` to `0`. The FC disarms instantly, motors cut to 1000 µs, and the buzzer emits the kill tone.

---

## Safety & Extensions

- Keep propellers off during calibration steps, especially ESC calibration where motors spin automatically.
- The MS5611 altitude hold is tuned for gentle indoor/outdoor hovering; adjust PID gains in `main.cpp` if you add payload or change prop/motor sets.
- Battery voltage sensing can be added by populating `status_payload.battery_mv`. Tie a resistor divider into an analog pin and update the FC firmware.
- Consider logging telemetry over USB or adding an OLED to the RC by reusing the status payload.

Fly safe and iterate like a professional UAV engineer! ✈️
