## Build & Calibrate Arduino Quadcopter Drone

This guide accompanies the firmware inside `firmware/flight_controller` and `firmware/radio_controller`. Follow it end-to-end to wire the hardware, flash the sketches, and complete the guided bring-up over the Serial Monitor.

---

### 1. Bill of Materials

- **Flight controller**: Arduino Nano, MPU6050, MS5611, NRF24L01+, 4x ESC (PWM 50 Hz), 4x brushless motors, LiPo battery, buzzer, status LED, guide LED.
- **Radio controller**: Arduino Nano, NRF24L01+, two 2-axis joysticks, toggle (kill/arm), two momentary push buttons (Button 1 = full calibration, Button 2 = smooth spin test), status LED.

---

### 2. Wiring Overview

| Signal | Flight Controller Pin | Notes |
| --- | --- | --- |
| ESC1 PWM | `D3` | Front-right |
| ESC2 PWM | `D5` | Rear-right |
| ESC3 PWM | `D6` | Rear-left |
| ESC4 PWM | `D9` | Front-left |
| Buzzer | `D10` | Active buzzer, GND to ground |
| Status LED | `D12` | Blinks when RC buttons acknowledged |
| Guide LED | `D4` | Solid/flash to indicate guide state |
| NRF24 CE/CSN | `D7` / `D8` | MOSI `D11`, MISO `D12`, SCK `D13`, 3.3 V with decoupling |
| MPU6050 | I²C (`A4/A5`) | Add pull-ups if not on breakout |
| MS5611 | I²C (`A4/A5`) | Share bus with MPU6050 |

| Signal | Radio Controller Pin | Notes |
| --- | --- | --- |
| Left joystick vertical (Throttle) | `A0` |
| Left joystick horizontal (Yaw) | `A1` |
| Right joystick vertical (Pitch) | `A2` |
| Right joystick horizontal (Roll) | `A3` |
| Kill/Arm toggle | `D2` (pull-up enabled) | LOW = Kill |
| Button 1 (Calibrate) | `D4` (pull-up) |
| Button 2 (Smooth spin) | `D5` (pull-up) |
| Status LED | `D6` |
| NRF24 CE/CSN | `D9` / `D10` |

> **Power note:** Give both NRF modules a clean 3.3 V rail with at least a 47 µF capacitor directly on the breakout. Tie all grounds together, especially ESC ground, sensor ground, and Nano ground.

---

### 3. Firmware Flash Order

1. Install Arduino libraries: **RF24 (by TMRh20)**. Servo & Wire ship with the IDE.
2. Flash `firmware/radio_controller/radio_controller.ino` to the RC Nano.
3. Flash `firmware/flight_controller/flight_controller.ino` to the FC Nano.
4. Open Serial Monitor at **115200 baud** on the flight controller Nano (this provides the guided checklist).

---

### 4. Radio Controller Calibration Mode

To solve the “joystick center equals 1000 µs throttle” danger, the RC firmware enforces:

- A programmable idle zone (0–5% travel) that still outputs **1000 µs** even if the stick is near center.
- Stick calibration stored in EEPROM so the midpoints for roll/pitch/yaw always map to **1500 µs** and the throttle minimum is captured with the stick fully down.

**Entering calibration mode**

1. Hold **Button 1** and **Button 2** while powering the RC Nano.
2. Move both joysticks (all four axes) through their entire travel for ~5 s.
3. Release the buttons. The RC saves the min/center/max positions to EEPROM and reboots into normal mode.

You only need to repeat this when changing joysticks or after a major mechanical adjustment.

---

### 5. Serial Monitor Guided Bring-Up

The flight controller prints explicit instructions through the following states:

1. **Waiting for RC link** – plug in the RC, watch the buzzer double-beep when the NRF link comes up.
2. **Require Kill** – flip the toggle (RC) to the kill position so ESCs definitely stay at 1000 µs.
3. **Calibration (Button 1)** – press Button 1 on the RC to start:
   - MPU6050 gyro/accelerometer averaging.
   - MS5611 baseline capture for altitude.
   - ESC max/min calibration (props must be removed).
   - Calibration data is stored to EEPROM (survives reset).
4. **Arm Request** – flip the toggle to ARM; motors are still held off.
5. **Smooth Spin Test (Button 2)** – with props still off, press Button 2 to gently ramp motors up/down (verifies wiring and ESC order).
6. **Ready to Fly** – Serial Monitor starts streaming telemetry (`Throttle/Yaw/Pitch/Roll/Altitude/NRF link quality`). Advance throttle slowly to lift off.
7. **Flying** – the guide reminds you of stick roles. Tipping the kill switch immediately disarms and resets to step 4.

Every transition is echoed on the Serial Monitor, the guide LED, and (for Button presses) the RC status LED.

---

### 6. Safety & Checklist

- Remove propellers for **all** calibrations and the smooth spin test.
- Never arm indoors.
- Verify ESC directions and motor numbering before flight (adjust lead swap instead of software when possible).
- If the link drops for >300 ms, the FC automatically:
  - Cuts throttle to 1000 µs on every ESC.
  - Sounds a low-frequency buzzer alert.
  - Forces you back to the “Require Kill” step once the link returns.

---

### 7. Tuning Hooks

- PID gains are defined inside `runController()` (`flight_controller.ino`). Start with the provided conservative values and increase proportional gains gradually if the quad feels soft.
- To add altitude hold, extend `TelemetryPacket` and feed `altitudeCm` into a throttle PID (currently the code only reports it).

---

### 8. Troubleshooting Quick Notes

- **NRF link never comes up:** confirm both modules share the same `RADIO_CHANNEL` (90) and pipe (`"FCN01"`), use short SPI wires, and add decoupling caps.
- **Joystick center drifts:** rerun RC calibration mode (both buttons held during power-on) and ensure the joysticks are not contacting the case.
- **Serial guide stuck on “Require Kill”:** verify the toggle switch wiring; LOW must correspond to kill (armSwitch = 0).
- **Altitude noisy:** add foam to the MS5611 to isolate prop wash, or average more samples before using it for control.

---

With both sketches flashed and the checklist completed, the drone will arm only after the inertial/barometric calibration plus the smooth spin verification. This sequence produces a stable baseline before any PID tuning or additional features are layered on.
