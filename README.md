# Professional Arduino Nano UAV Stack

This workspace contains a two-board UAV reference implementation that matches the requested professional workflow:

- `firmware/flight_controller/flight_controller.ino` — Flight stack running on an Arduino Nano with MPU6050, NRF24L01 PA+LNA, buzzer, status LED, and four ESCs/motors (FL=D3, FR=D5, RR=D6, RL=D9).
- `firmware/remote_controller/remote_controller.ino` — Handheld radio with dual joysticks, two toggle switches (ALT HOLD, ARM/KILL), two momentary buttons, and an NRF24L01 PA+LNA.
- `firmware/shared/DroneLink.h` — Shared packet format, enums, and RF channel definition (channel 103).

Both sketches require the standard Arduino core plus the **RF24** and **MPU6050** libraries. Upload each `.ino` to its respective Nano from the Arduino IDE (or arduino-cli) after installing dependencies.

---

## Hardware Maps

### Flight Controller (FC)
| Peripheral | Pin |
|------------|-----|
| NRF24 CE / CSN | D4 / D10 |
| MPU6050 INT | D2 |
| Status LED | D7 (blinks fast when link OK) |
| Buzzer | D8 |
| ESC Front-Left / Front-Right / Rear-Right / Rear-Left | D3 / D5 / D6 / D9 |

### Remote Controller (RC)
| Peripheral | Pin |
|------------|-----|
| NRF24 CE / CSN | D9 / D10 |
| Left stick (Throttle / Yaw) | A0 / A1 |
| Right stick (Pitch / Roll) | A2 / A3 |
| Button 1 (Gyro calibration) | D4 |
| Button 2 (Motor arm + ESC mode) | D5 |
| Switch 1 (ALT HOLD enable, `1` = hold) | D2 |
| Switch 2 (ARM/KILL, `1` = arm allowed) | D3 |

All toggle switches use PIN → common, the remaining two switch pins to GND.

---

## Key Behaviors
- NRF channel fixed at **103** with auto-ack + ack payload telemetry to guarantee closed-loop comms.
- RC streams at 50 Hz and shows link state, button/switch states, and FC telemetry cues on the Serial Monitor (`115200` baud). The RC continues to operate even if the monitor is closed.
- FC status LED (D7): fast blink when link & armed, medium when link but disarmed, slow when failsafe/no link.
- Safety limits: throttle internally capped at **65 %**, pitch/roll commands capped at **±30°**, yaw limited to ±150 °/s, and hard failsafe after 350 ms without packets (motors drop to 1000 µs).
- Buzzer cues: double-chirp for successful IMU calibration, 7 s tone for failed calibration, short rising tone for ESC calibration start/end, short beep on arming/disarming.

---

## Operating Procedure
1. **Power the RC** (D2/D3 default HIGH). Confirm the Serial Monitor shows `Link: --` while FC is off.
2. **Power the FC.** Within ~1 s the RC should report `Link: OK` and the FC LED will enter the fast blink pattern (communication confirmed per requirement).
3. Ensure **Switch 2 (ARM/KILL)** is at `1` (lever open/high). If it is low, the FC remains disarmed regardless of button presses.
4. Place the aircraft on a level surface and press **Button 1** once to trigger IMU calibration. Keep the frame still:
   - Success: buzzer chirps twice, RC prints `[FC] Calibration OK`.
   - Failure: buzzer holds a single low tone for ~7 s, RC prints `[FC] Calibration FAIL`.
5. For ESC calibration / soft start, set **Switch 1 to `0`** (ALT HOLD off) and hold **Button 2** for >1.5 s. Each ESC ramps smoothly in sequence; the buzzer emits a distinctive double chirp when the sequence completes.
6. **Arming:** with IMU + ESC ready, Switch 2 high, and optional ALT HOLD state selected, tap **Button 2** (short press). Motors arm/disarm on every short press and the buzzer clicks once.
7. **Flying:** use joysticks with the following mapping:
   - A0: throttle (1000–2000 µs). UP = climb, DOWN = descend.
   - A1: yaw (LEFT = CCW, RIGHT = CW).
   - A2: pitch (UP = forward, DOWN = backward).
   - A3: roll (LEFT = left strafe, RIGHT = right strafe).
   - Switch 1 high enables throttle hold/position-hold assist (internally clamps throttle drift); flip to `0` for manual mode or when entering ESC calibration.
   - Switch 2 low instantly kills the motors (failsafe/kill switch).
8. Land, flip Switch 2 low to disarm, then power off FC followed by RC.

---

## Serial Monitor Reference (RC @ 115200 baud)
```
Link:OK | Seq:1234 | ArmSW:1 | AltHold:0 | Btn1:0 | Btn2:0 | IMU:ready | ESC:ready | Armed:1 | FState:3
```
- `Link`: `OK` when ack payloads arrive <350 ms apart; otherwise `--`.
- `ArmSW`/`AltHold`: live switch states.
- `Btn1/Btn2`: debounced button levels.
- `IMU/ESC/Armed/FState`: telemetry flags echoed from the FC (FlightState values align with `DroneLink::FlightState` enum).
- Additional `[FC] …` lines stream human-readable events such as calibration success/failure, ESC sequence transitions, and arming changes.

---

## Building & Deploying
1. Install Arduino libraries: `RF24` by TMRh20 and any MPU6050 helper (e.g., Jeff Rowberg’s).
2. Open each `.ino` in its own sketch folder (or use arduino-cli/PlatformIO) and select **Arduino Nano** with the correct processor.
3. Upload `flight_controller.ino` to the airframe Nano and `remote_controller.ino` to the handheld Nano.
4. Use the Serial Monitor on the RC side for runtime feedback; the FC emits optional debug logs over USB at 115200 baud if required.

Keep props removed during calibration and ESC sequencing. Re-enable props only after verifying all safety interlocks (Switch 2 high, Button 2 short press to arm, throttle minimum).
