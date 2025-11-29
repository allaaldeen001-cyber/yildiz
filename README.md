# Professional Nano-Based UAV Platform

This repository contains the full firmware and documentation for a two-board Arduino Nano drone platform:

- `flight_controller/flight_controller.ino` – sensor fusion, safety logic, and ESC control for the quad.
- `remote_controller/remote_controller.ino` – dual-stick radio handset with on-device telemetry/diagnostics.

The design targets a lightweight professional workflow: deterministic state machines, ACK-guarded RF messaging, barometric altitude support, and well-defined bring-up procedures.

## Hardware Summary

| Subsystem | Part | Notes |
|-----------|------|-------|
| MCU (FC + RC) | Arduino Nano (ATmega328P) | 16 MHz/5 V |
| Radio link | nRF24L01+ PA+LNA | Channel 103, 1 Mbps, auto-ack enabled |
| IMU | MPU6050 | DMP disabled; complementary filter used |
| Barometer | MS5611 | Oversampling ×128 for low-noise Z reference |
| Propulsion | 4× ESC + brushless motors | Driven with `Servo` pulses (1–2 ms) on D3/D5/D6/D9 |

### Flight Controller Pin Map

| Function | Pin |
|----------|-----|
| nRF24 CE / CSN | D4 / D10 |
| MPU6050 INT | D2 |
| Buzzer | D8 |
| Status LED | D7 |
| ESC Front-Left / Front-Right | D3 / D5 |
| ESC Rear-Right / Rear-Left | D6 / D9 |
| MS5611 | I²C (A4/A5) |

### Remote Controller Pin Map

| Function | Pin |
|----------|-----|
| nRF24 CE / CSN | D9 / D10 |
| Left stick V (Throttle) / H (Yaw) | A0 / A1 |
| Right stick V (Pitch) / H (Roll) | A2 / A3 |
| Button 1 – IMU calibration | D4 |
| Button 2 – Motor enable / ESC cal request | D5 |
| Switch 1 – Altitude hold gate | D2 |
| Switch 2 – Arm/Kill safety | D3 |

> **Note on conflicting specs:** The user brief references Switch 1 as both “altitude hold” and “kill.”  
> This firmware keeps Switch 2 (D3) as the hard kill/arming gate, while Switch 1 (D2) toggles the altitude hold controller.  
> The README and code annotate this choice clearly so you can swap behavior quickly if desired.

## Software Architecture

### RF Messaging

- Both boards share the 5-byte pipe address `0xE7E7E7E7E7`, channel 103, 1 Mbps, `RF24_PA_MAX`.
- Auto-ack with dynamic payload + ACK payloads is enabled. Every outbound control frame receives a telemetry ACK so the RC can print **Linked / Lost**, calibration results, and FC state.
- CRC16 (ANSI) is appended to each control frame to protect against single-packet corruption before the ACK is even considered.

#### Control Frame Layout (RC ➜ FC)

| Field | Type | Description |
|-------|------|-------------|
| `sequence` | `uint32_t` | Monotonic counter for drop detection |
| `throttle` | `uint16_t` | 1000–2000 μs request (internally capped at 65 %) |
| `pitch`, `roll`, `yaw` | `int16_t` | ±500 units (~±30 ° commands) |
| `flags` | `uint8_t` | Bit0: calibration, Bit1: motor enable, Bit2: ESC calibration |
| `switchMask` | `uint8_t` | Bit0: altitude hold, Bit1: kill safe (1 = run), Bit2: reserved |
| `crc` | `uint16_t` | ANSI CRC16 over preceding bytes |

#### Telemetry Frame Layout (FC ➜ RC ACK)

| Field | Type | Meaning |
|-------|------|---------|
| `fcState` | `uint8_t` | Enumerated `INIT/READY/ARMED/ESC_CAL/FAULT` |
| `linkQuality` | `uint8_t` | Rolling average of ACK success (0–100 %) |
| `calibrationCode` | `uint8_t` | 0 idle, 1 running, 2 success, 3 failed |
| `batteryMv` | `uint16_t` | Placeholder input (wire via analog divider when available) |
| `baroAltCm` | `int16_t` | Filtered altitude for HUD |
| `faultCode` | `uint8_t` | 0 OK, otherwise last failsafe reason |

### Flight Controller State Machine

```
INIT → (self-test OK) → READY
READY --calibration request--> CALIBRATING --success--> READY
READY --motor enable + kill safe--> ARMED
ARMED --ESC cal request--> ESC_CAL → READY
ARMED --kill / link loss / fault--> FAULT → READY
```

- LED D7 blinks at 5 Hz when packets are flowing; otherwise it slow-pulses to indicate link loss.
- Buzzer D8 patterns:
  - Calibration success: two short chirps.
  - Calibration failure: single 7 s tone.
  - ESC calibration: staircase chirp matching each motor ramp.

### Inner-Loop Control

- Complementary filter fuses MPU6050 gyro + accel to limit pitch/roll commands to ±30 °.
- Throttle commands are capped at 65 % internally (1650 µs) even if RC requests 2000.
- Altitude hold (`switchMask & ALT_HOLD`) engages a PI controller around the MS5611 altitude; disengage (Switch 1 “0”) before entering ESC calibration per the operations checklist.
- Hard kill (Switch 2 low) immediately disarms and commands 1000 µs to every ESC irrespective of RC throttle.
- Link-loss failsafe (<500 ms without packet) forces kill, sounds a fault beep, and sets LED to solid.

## Build & Flash

1. **Required Arduino Libraries**
   - [RF24](https://github.com/nRF24/RF24) by TMRh20.
   - [i2cdevlib MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050).
   - A lightweight MS5611 driver (e.g., [SparkFun MS5611](https://github.com/sparkfun/SparkFun_MS5611_Breakout_Board_Arduino_Library)).
2. Install the libraries via the Arduino Library Manager or as ZIPs.
3. Open each `.ino` file, select **Arduino Nano (ATmega328P, Old Bootloader)**, choose the matching serial port, and upload.
4. Provide a steady 5 V supply to both boards when flashing to avoid browning out the PA+LNA module.

## Configuration Knobs

- `flight_controller/flight_controller.ino`
  - `ENABLE_BATTERY_MONITOR`: set to `1` and wire the LiPo through a divider to `A6` if you want live battery voltage in the telemetry feed.
  - `kThrottleCapUs`, `kMaxTiltDeg`, and the PI gains (`kAltKp`, `kAltKi`) let you tune the safety envelope for your specific airframe.
- `remote_controller/remote_controller.ino`
  - `kLongPressMs` defines how long Button 2 must be held (with Switch 1 at “0”) to request the ESC calibration macro.

## Operating Checklist

1. Power the remote controller; verify the serial monitor (115200 baud) shows `Link: ---` and live switch/button states.
2. Power the flight controller. Within ~1 s the RC should show `Link: OK`, and the FC LED (D7) blinks to acknowledge radio sync.
3. Ensure Switch 2 (kill) is **HIGH** (physical toggle away from GND). The RC display will show `SAFE` or `ARMED`.
4. Press **Button 1** to start gyro/baro calibration. The RC logs the progress; upon success the FC buzzer chirps twice.
5. If the buzzer emits a single 7 s tone, calibration failed—check that the drone is perfectly still and retry.
6. Set Switch 1 LOW (disable altitude hold), then hold **Button 2** for roughly >1 s to initiate ESC calibration. The FC ramps each motor smoothly and emits a distinct staircase tone. When done, it returns to READY.
7. With Switch 1 HIGH (enable hold) if desired, re-press Button 2 to arm. Slowly advance the throttle (A0) and command yaw/pitch/roll as needed.
8. Flipping Switch 2 LOW instantly disarms (kill). Keep fingers on this switch whenever testing.

## Remote Controller Console Output

The RC periodically prints a compact HUD:

```
Link:OK RSSI:88%  Th:1230  Pr/Pi/Ro:0/2/-4
BTN cal:0 arm:1  SW alt:1 kill:0
FC ST:ARMED  Cal:OK  Alt:123cm  Fault:0
```

- `Link` flips to `---` if ACKs stop arriving. Throttle and stick values are bounded so you can diagnose sensor noise.
- Calibration / ESC results are echoed as they happen so the serial monitor acts as a field-side “screen,” but the RC logic does not depend on a tethered PC.

## Safety Envelope

- Max pitch/roll command: ±30 °.
- Max throttle output: 65 % (≈1650 µs). Raising the cap requires editing `kThrottleCapUs` inside the FC sketch.
- All critical transitions (calibration → ready, ready → armed) require explicit button presses **and** the kill switch being safe.
- A watchdog disarms if the sensor stack reports NaNs or if the IMU interrupt is lost.

## Next Steps

- Integrate GPS for full-position hold in place of the simple altitude gate.
- Add EEPROM-backed trim storage so joystick offsets persist across power cycles.
- Feed LiPo voltage into an analog divider and update `batteryMv` inside the telemetry struct for full health monitoring.