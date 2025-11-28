# Arduino Nano Drone Flight Stack

Production-ready firmware for a quad-X drone built around two Arduino Nano boards, NRF24L01 PA+LNA radios, an MPU6050 IMU, and an MS5611 barometer. The repository contains both the flight-controller (FC) stack and the companion remote-controller (RC) firmware plus the shared radio protocol.

## Repository Layout
- `firmware/common/CommProtocol.h` – packed radio structs, checksums, bit masks, channel definition.
- `firmware/fc/nano_fc.ino` – flight controller firmware: sensor fusion, cascade control, failsafes, motor mixing.
- `firmware/rc/nano_rc.ino` – remote controller firmware: stick processing, mode switches, telemetry display.

## Radio Protocol (NRF24 channel 103, 250 kbps, auto-ack)
Both packets include a monotonically increasing `seq` field and a 16-bit additive checksum to reject corrupt payloads (payload size < 32 bytes so ACK payloads are supported).

### RC → FC (`RcToFcPacket`)
| Field | Bits | Notes |
| --- | --- | --- |
| `seq` | 16 | wraps at 65535, used for stale-packet rejection |
| `throttle` | 16 | 1000–2000 µs equivalent, internally capped to 65% thrust |
| `roll`, `pitch` | 16 each | desired angles in 0.1° increments (±30° range) |
| `yaw` | 16 | desired yaw rate in 0.1 °/s (±180 °/s) |
| `switches` | 8 | `SWITCH_ALT_HOLD` bit0, `SWITCH_ARM` bit1 |
| `buttons` | 8 | `BUTTON_CALIBRATE` bit0, `BUTTON_MOTOR_TEST` bit1 (momentary) |
| `aux` | 8 | reserved |
| `checksum` | 16 | sum of all previous bytes |

### FC → RC Telemetry (`FcToRcPacket`)
| Field | Bits | Notes |
| --- | --- | --- |
| `seq` | 16 | increments every ACK payload |
| `rollAngleDeg`, `pitchAngleDeg` | 16 each | fused attitude in 0.01° |
| `yawRateDps` | 16 | most recent yaw rate set-point (0.1 °/s) |
| `altitudeCm` | 16 | filtered altitude above take-off point |
| `batteryMv` | 16 | VBAT estimate (A6 via divider, configure ratio if needed) |
| `statusFlags` | 8 | `FC_STATUS_LINKED`, `ARMED`, `CALIBRATED`, `ALT_HOLD`, `FAILSAFE` |
| `linkQuality` | 8 | 0–100 (%) |
| `checksum` | 16 | additive checksum |

## Flight Controller Highlights (`firmware/fc/nano_fc.ino`)
- **Hardware map:** NRF24 (CE D4, CSN D10), MPU6050 on I²C (+ INT D2), MS5611 on I²C, buzzer D8, LED D7, ESC outputs FL D3 / FR D5 / RR D6 / RL D9, optional battery divider on A6.
- **Timing:**  
  - Sensor fusion + rate PID loop @ 250 Hz (4000 µs tick via `micros()` guard).  
  - Angle PID loop @ 100 Hz for set-point cascades.  
  - Altitude estimator + throttle loop @ 25 Hz.  
  - All loops non-blocking; MS5611 driver uses a state machine to avoid I²C stalls.
- **Filtering:** Mahony quaternion filter (Kp 3.0, Ki 0.03) fuses MPU6050 gyro/accel; vertical acceleration is rotated into earth frame and low-passed before blending with MS5611 altitude in a complementary filter.
- **Control stack:**  
  - Outer angle PID (roll/pitch) limits commands to ±30° and generates rate targets.  
  - Inner rate PID drives motor mixing (Quad-X: `throttle ± pitch ± roll ± yaw`).  
  - Yaw is rate-only (no magnetometer), so long-term drift is handled by pilot commands.  
  - Altitude hold: height PID → climb-rate → throttle PID. Throttle stick deflection while alt-hold is active shifts the target altitude smoothly (≈1 m/s per full-stick deflection).
- **Throttle management:** Commands are curved (quadratic) and clamped to 1650 µs (≈65% thrust) to leave headroom for PID authority. Hover throttle is auto-learned and feeds the altitude velocity loop.
- **Safety + workflow:**  
  - SW2 (ARM) must be HIGH, link alive <250 ms, throttle <1100 µs, and IMU calibrated before arming.  
  - Link watchdog (250 ms), button-triggered calibration disarms and averages 2000 samples; success = 2 short beeps, failure = one 7 s tone.  
  - Button_2 starts the ESC calibration + motor test sequence (max, min, then individual ramps) with a distinct triple beep (`buzzer.info`).  
  - Motor outputs hard-zeroed while calibrating, during failsafe, and whenever SW2 is LOW. LED on D7 blinks fast when linked, slow when waiting for link.  
  - Battery sense (A6) is optional; adjust `BATTERY_DIVIDER_RATIO` to match the actual resistor ladder.
- **Altitude estimator:** MS5611 pressure is converted to meters and fused with double-integrated vertical acceleration (gravity removed via the quaternion). Complementary gain (α=0.02) provides smooth but drift-free altitude for the hold loop.
- **Motor test & ESC calibration:** Button_2 (from RC) forces a sequence: all motors full for 2 s, all low for 2 s, then FL/FR/RR/RL ramps (smooth 1.5 s) to confirm spin direction without props.
- **Fail-safe hierarchy:** Kill order is (1) motor test / calibration, (2) SW2 low, (3) radio watchdog, (4) RC throttle kill (SW2 low also forces RC to send 1000 µs), ensuring motors never spin unintentionally.

### Initialization (matches the requested workflow)
1. Power the RC; it immediately transmits (throttle min, SW2 status).  
2. Power the FC; LED D7 slow-blinks until the NRF link locks, then fast-blinks.  
3. Set SW2=1 (RC switch HIGH). Press Button_1 to run gyro/accel calibration (two beeps = OK).  
4. Leave SW1=0, press Button_2 to run ESC calibration + motor test (triple beep + smooth ramps).  
5. Fly using sticks; SW1 toggles altitude hold, SW2 is the kill switch, Buttons remain momentary commands.

## Remote Controller Highlights (`firmware/rc/nano_rc.ino`)
- **Inputs:** Dual-axis joysticks (A0–A3) sampled 4× and averaged, mapped from raw ADC (80–980) into 1000–2000 µs equivalents.  
- **Switch logic:** SW1 (D2) toggles altitude hold bit, SW2 (D3) arms/disarms; both use `INPUT_PULLUP` so “1” = switch open/HIGH. SW2 LOW also forces throttle command to 1000 µs before packets are finalized.  
- **Buttons:** Button_1 (D4) requests IMU calibration; Button_2 (D5) requests ESC calibration + motor test. Active-low inputs debounced in software.  
- **Transmission:** 100 Hz update rate, NRF24 auto-ack with ACK payloads, sequence numbers, additive checksum. Link health is tracked and displayed; ack payloads carry telemetry without a separate downlink.  
- **Telemetry display:** Serial monitor (115200) prints every 100 ms: link state, switch/button states, FC arming + calibration state, altitude, roll, and telemetry age.

## Tuning Guide
| Loop | Axis | Kp | Ki | Kd | Notes |
| --- | --- | --- | --- | --- | --- |
| Rate | Roll/Pitch | 0.12 | 0.04 | 0.0008 | Increase Kp for snappier response; raise Ki in 0.01 steps if steady-state errors persist; D term damps overshoot. |
| Rate | Yaw | 0.18 | 0.02 | 0.0 | Yaw is rate-only; adjust Kp for desired spin speed, Ki for hold. |
| Angle | Roll/Pitch | 4.5 | 0.0 | 0.12 | Governs overall attitude stiffness; start at 4.5 and raise equally until oscillations appear. |
| Altitude (height) | Z | 2.0 | 0.5 | 0.0 | Height error to climb-rate; adjust Kp for responsiveness, Ki for long-term bias removal. |
| Altitude (velocity) | Z | 80.0 | 30.0 | 0.0 | Converts climb-rate command into throttle. Lower Kp if throttle oscillates; adjust Ki for sustained climb errors. |

Recommended tuning order:
1. Fly in manual (alt-hold off). Tune rate loop (P then D, then small Ki) for roll/pitch, then yaw.  
2. Tune angle loop gains equally so the craft tracks stick angles without oscillation.  
3. Enable altitude hold indoors with props removed (motor test mode) to verify transitions, then outdoors: raise height PID Kp to reach desired climb response, add Ki only if altitude drifts.  
4. Finally, adjust the velocity PID for throttle smoothness (reduce if motors chatter) and re-check hover throttle capture.

## Testing & Validation Procedure
1. **Bench link test:** Power RC + FC (no props). Confirm LED D7 fast-blinks and RC serial monitor shows `LINKED`. Move sticks and watch telemetry roll/pitch react.  
2. **Sensor calibration:** Place the airframe level, SW2=1, press Button_1. Expect two short beeps; if a 7 s tone occurs, check that the frame is perfectly still.  
3. **Motor/ESC test:** Remove props, set SW1=0, press Button_2. Observe the max/min pulses followed by individual ramps FL→FR→RR→RL. Verify spin direction matches quad-X conventions; re-wire ESCs if necessary.  
4. **Throttle + safety:** With SW2=0, confirm motors never spin even when throttle is raised (RC also transmits 1000 µs). Toggle SW2 to 1 and ensure arming only occurs when throttle <1100 µs and calibration succeeded.  
5. **Failsafe:** While armed (props still off), cut RC power; motors should stop within 250 ms and LED revert to slow blink. Repeat by forcing SW2=0 to verify kill switch.  
6. **Hover test:** Install props, arm, lift gently in manual mode, confirm attitude control. Engage altitude hold (SW1=1) at a safe height; throttle stick deflection should command smooth climb/descend while the base throttle remains limited.  
7. **Post-flight verification:** Check telemetry (battery voltage, status flags) and store tuning notes before further adjustments.

With these steps the FC/RC pair meet the requested workflow: RC-first power-on, confirmed link via LED and serial monitor, Button-driven calibration, ESC test, throttle-limited cascade loops, and robust safety fallbacks.
