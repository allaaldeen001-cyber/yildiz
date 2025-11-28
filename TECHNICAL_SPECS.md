# Technical Specifications & Engineering Details

## 📋 System Architecture Overview

This document provides deep technical details about the flight controller firmware architecture, algorithms, and performance characteristics.

---

## 🔷 Flight Controller Specifications

### Processing & Performance

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Microcontroller** | ATmega328P | 16 MHz, 8-bit AVR |
| **Flash Memory** | 30.5 KB / 32 KB | ~95% utilization |
| **SRAM** | ~1.8 KB / 2 KB | ~90% utilization |
| **Rate Loop Frequency** | 250 Hz | 4ms period |
| **Angle Loop Frequency** | 100 Hz | 10ms period |
| **Altitude Loop Frequency** | 25 Hz | 40ms period |
| **Loop Time (typical)** | 2500-3500 µs | Varies with sensor I2C |
| **Loop Jitter** | ±500 µs | Acceptable for 250Hz |

### Sensor Configuration

#### MPU6050 (IMU)

| Parameter | Configuration | Register |
|-----------|---------------|----------|
| **Gyro Range** | ±500°/s | GYRO_CONFIG=0x08 |
| **Gyro Resolution** | 65.5 LSB/(°/s) | 16-bit ADC |
| **Accel Range** | ±8g | ACCEL_CONFIG=0x10 |
| **Accel Resolution** | 4096 LSB/g | 16-bit ADC |
| **DLPF Bandwidth** | 44 Hz | CONFIG=0x03 |
| **Sample Rate** | 250 Hz | SMPLRT_DIV=0x00 |
| **I2C Clock** | 400 kHz | Fast mode |

**Noise Characteristics**:
- Gyro noise density: ~0.005°/s/√Hz
- Accel noise density: ~300 µg/√Hz
- Effective after DLPF: ~0.02°/s, ~0.002g

#### MS5611 (Barometer)

| Parameter | Configuration |
|-----------|---------------|
| **Pressure Range** | 10-1200 mbar |
| **Pressure Resolution** | 0.012 mbar (OSR=4096) |
| **Altitude Resolution** | ~10 cm (at sea level) |
| **Conversion Time** | 9.04 ms (OSR=4096) |
| **Sample Rate** | 25 Hz |
| **I2C Clock** | 400 kHz |

**Altitude Formula**:
```
altitude = 44330 × (1 - (P/P0)^0.1903)
where P0 = 1013.25 hPa (sea level)
```

---

## 🧮 Attitude Estimation: Mahony AHRS Filter

### Algorithm Overview

The Mahony filter is a computationally efficient quaternion-based attitude estimator that fuses gyroscope and accelerometer data.

### Mathematical Model

**State**: Quaternion q = [q0, q1, q2, q3]

**Process Model** (gyroscope integration):
```
q̇ = 0.5 × q ⊗ [0, ωx, ωy, ωz]

where ⊗ is quaternion multiplication
ω = [ωx, ωy, ωz] = gyro rates (rad/s)
```

**Correction** (accelerometer feedback):
```
e = â × ĝ

where:
  â = normalized accelerometer vector
  ĝ = estimated gravity direction from quaternion
  e = error (cross product)
```

**Feedback Terms**:
```
Proportional: ω_corrected = ω + Kp × e
Integral:     ω_corrected = ω + Kp × e + Ki × ∫e dt
```

### Implementation Details

**Filter Gains**:
- `Kp = 2.0`: Proportional gain (responsiveness)
- `Ki = 0.01`: Integral gain (bias correction)

**Update Rate**: 250 Hz (4ms)

**Computational Cost**: ~800 µs per update

**Advantages**:
- No magnetometer needed (yaw drifts, but rate control compensates)
- No gimbal lock (quaternion representation)
- Low computational cost (suitable for 16 MHz AVR)
- Tunable via Kp/Ki gains

**Quaternion to Euler Conversion**:
```cpp
roll  = atan2(2(q0*q1 + q2*q3), 1 - 2(q1² + q2²))
pitch = asin(2(q0*q2 - q3*q1))
yaw   = atan2(2(q0*q3 + q1*q2), 1 - 2(q2² + q3²))
```

---

## 📊 1D Kalman Filter (Altitude Estimation)

### State-Space Model

**State Vector**:
```
x = [h, v]ᵀ

where:
  h = height (m)
  v = vertical velocity (m/s)
```

**State Transition** (prediction):
```
x_k = F × x_{k-1} + B × u

F = [1  dt]    B = [0.5×dt²]
    [0   1]        [  dt   ]

u = az (vertical acceleration from IMU, m/s²)
```

**Measurement Model**:
```
z = H × x + v

H = [1  0]  (barometer measures height directly)
v ~ N(0, R)  (measurement noise)
```

### Kalman Filter Equations

**Predict**:
```cpp
x̂_k⁻ = F × x̂_{k-1} + B × u
P_k⁻ = F × P_{k-1} × Fᵀ + Q
```

**Update**:
```cpp
K = P_k⁻ × Hᵀ × (H × P_k⁻ × Hᵀ + R)⁻¹  // Kalman gain
x̂_k = x̂_k⁻ + K × (z - H × x̂_k⁻)        // State update
P_k = (I - K × H) × P_k⁻                 // Covariance update
```

### Noise Parameters

```cpp
Q = [0.01   0  ]  // Process noise covariance
    [  0   0.1 ]

R = 0.5            // Measurement noise variance (barometer)
```

**Tuning**:
- Increase `Q` → Trust barometer more (responsive, but noisy)
- Increase `R` → Trust accelerometer more (smooth, but drifts)

**Performance**:
- Update rate: 25 Hz
- Altitude RMS error: ~0.2m (static)
- Velocity estimate lag: ~200ms

---

## 🎮 Cascade PID Control Architecture

### Control Hierarchy

```
┌─────────────────────────────────────────────────────────┐
│ SETPOINT (RC Input)                                     │
└────────────────┬────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────┐
│ ANGLE LOOP (100 Hz)                                     │
│ Input:  Target angle (degrees)                          │
│ Output: Desired angular rate (rad/s)                    │
│ PID:    P only (no I/D)                                 │
└────────────────┬────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────┐
│ RATE LOOP (250 Hz)                                      │
│ Input:  Desired angular rate (rad/s)                    │
│ Output: Motor commands                                  │
│ PID:    Full P+I+D                                      │
└────────────────┬────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────┐
│ MOTOR MIXING (X Configuration)                          │
└────────────────┬────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────┐
│ MOTORS (PWM 490 Hz)                                     │
└─────────────────────────────────────────────────────────┘
```

### PID Controller Implementation

**Discrete PID Algorithm**:
```cpp
P = Kp × error
I = Ki × Σ(error × dt)  [with anti-windup]
D = Kd × (error - error_prev) / dt

output = P + I + D
```

**Anti-Windup**:
```cpp
integral = constrain(integral, -iMax, +iMax)
```

Prevents integrator saturation when control authority is limited.

### Rate Loop (Inner Loop)

**Frequency**: 250 Hz (4ms period)

**Input**: Desired angular rates [rad/s]
- Roll rate:  ωx_des
- Pitch rate: ωy_des
- Yaw rate:   ωz_des

**Feedback**: Gyroscope measurements [rad/s]

**PID Gains** (default):

| Axis | Kp | Ki | Kd | iMax |
|------|----|----|----|----|
| Roll | 1.5 | 0.05 | 0.01 | 100 |
| Pitch | 1.5 | 0.05 | 0.01 | 100 |
| Yaw | 2.0 | 0.1 | 0.0 | 100 |

**Output**: Motor command adjustments [-∞, +∞]

**Bandwidth**: ~30 Hz (crossover frequency)

### Angle Loop (Outer Loop)

**Frequency**: 100 Hz (10ms period)

**Input**: Target angles from RC sticks [degrees]
- Roll angle:  ±30°
- Pitch angle: ±30°

**Feedback**: AHRS attitude estimate [degrees]

**PID Gains** (default):

| Axis | Kp | Ki | Kd |
|------|----|----|---|
| Roll | 3.5 | 0.0 | 0.0 |
| Pitch | 3.5 | 0.0 | 0.0 |

**Output**: Desired rates for inner loop [rad/s], constrained to ±3.0 rad/s

**Bandwidth**: ~5 Hz

**Note**: No I or D terms needed because:
- Rate loop provides damping (effective D term)
- Integral error accumulates in rate loop instead

### Altitude Control Loop

**Frequency**: 25 Hz (40ms period)

**Two-stage cascade**:

#### Stage 1: Altitude PID (Height → Velocity)

**Input**: Target altitude [m]

**Feedback**: Kalman-filtered altitude [m]

**PID Gains**:
- Kp = 2.0
- Ki = 0.5
- Kd = 1.0

**Output**: Desired climb rate [m/s], constrained to ±2.0 m/s

#### Stage 2: Climb Rate PID (Velocity → Throttle)

**Input**: Desired climb rate [m/s]

**Feedback**: Kalman-filtered velocity [m/s]

**PID Gains**:
- Kp = 30.0
- Ki = 5.0
- Kd = 5.0

**Output**: Throttle adjustment [PWM units], added to hover throttle (~1500)

**Hover Throttle**: Automatically determined by RC stick position when altitude hold is enabled

---

## 🚁 Motor Mixing (X Configuration)

### Quadcopter Layout

```
        FRONT (0°)
           ↑
      FL(⟳)  FR(⟲)
        \  X  /
        /  X  \
      RL(⟲)  RR(⟳)
           ↓
        BACK (180°)
```

### Mixing Matrix

```
          Throttle  Roll  Pitch  Yaw
FL (D3)  [   1      -1     +1    +1  ]
FR (D5)  [   1      +1     +1    -1  ]
RR (D6)  [   1      +1     -1    +1  ]
RL (D9)  [   1      -1     -1    -1  ]
```

### Motor Command Calculation

```cpp
motorFL = throttle - rollCmd + pitchCmd + yawCmd;
motorFR = throttle + rollCmd + pitchCmd - yawCmd;
motorRR = throttle + rollCmd - pitchCmd + yawCmd;
motorRL = throttle - rollCmd - pitchCmd - yawCmd;

// Constrain to valid PWM range
motorFL = constrain(motorFL, 1000, 2000);
motorFR = constrain(motorFR, 1000, 2000);
motorRR = constrain(motorRR, 1000, 2000);
motorRL = constrain(motorRL, 1000, 2000);
```

### Sign Convention

- **Roll positive** → Tilt right → Increase FR/RR, decrease FL/RL
- **Pitch positive** → Tilt forward → Increase FL/FR, decrease RR/RL
- **Yaw positive** → Rotate CW → Increase FL/RR (CW motors), decrease FR/RL (CCW motors)

### PWM Output

- **Range**: 1000-2000 µs (standard ESC range)
- **PWM Frequency**: 490 Hz (Arduino `analogWrite` default on pins 3,5,6,9)
- **Resolution**: 8-bit (0-255 mapped to 1000-2000)

**Note**: Some modern ESCs support higher resolution (11-bit) via special libraries, but 8-bit is sufficient for stable flight.

---

## 📡 NRF24L01 Communication Protocol

### Radio Configuration

| Parameter | Value |
|-----------|-------|
| **Frequency** | 2.503 GHz (Channel 103) |
| **Data Rate** | 250 kbps |
| **TX Power** | +20 dBm (with PA+LNA) |
| **Auto-Retry** | 5 retries, 1250 µs delay |
| **CRC** | 2 bytes |
| **Address Width** | 5 bytes |
| **Payload Size** | Fixed (20 bytes RC→FC, 18 bytes FC→RC) |

### Communication Timing

| Direction | Rate | Latency | Packet Loss |
|-----------|------|---------|-------------|
| RC → FC | 50 Hz (20ms) | 2-5ms | < 1% (with ACK) |
| FC → RC | 20 Hz (50ms) | 5-10ms | ~5% (telemetry, non-critical) |

### Data Structures

#### RC → FC (20 bytes)

```cpp
struct RC_Data {
  uint16_t throttle;    // Bytes 0-1
  uint16_t yaw;         // Bytes 2-3
  uint16_t pitch;       // Bytes 4-5
  uint16_t roll;        // Bytes 6-7
  uint8_t  armed;       // Byte 8
  uint8_t  altHold;     // Byte 9
  uint8_t  button1;     // Byte 10
  uint8_t  button2;     // Byte 11
  uint32_t timestamp;   // Bytes 12-15
  // 4 bytes padding (struct alignment)
};
```

#### FC → RC (18 bytes)

```cpp
struct FC_Telemetry {
  float    roll;        // Bytes 0-3
  float    pitch;       // Bytes 4-7
  float    yaw;         // Bytes 8-11
  float    altitude;    // Bytes 12-15
  uint8_t  calibrated;  // Byte 16
  uint8_t  linked;      // Byte 17
  uint16_t loopTime;    // Bytes 18-19
  // 2 bytes padding
};
```

### Failsafe Logic

**Timeout**: 500 ms (no packets received)

**Actions**:
1. Set `systemArmed = false`
2. Force all motors to `MOTOR_MIN` (1000)
3. Turn off status LED
4. Reset PID integrators

**Recovery**:
- Link automatically restored when packets resume
- Must re-arm manually (SW_2 toggle)
- All safety checks re-applied

---

## ⚙️ Safety Systems

### 1. Link Loss Detection

**Method**: Timestamp comparison
```cpp
if (millis() - lastNrfReceiveTime > NRF_TIMEOUT_MS) {
  systemArmed = false;
  // Cut motors
}
```

**Timeout**: 500 ms (configurable)

### 2. Calibration Enforcement

**Check**: 
```cpp
if (!calibrationComplete) {
  systemArmed = false;
  // Block motors
}
```

Prevents flight with uninitialized sensor offsets.

### 3. Maximum Tilt Angle

**Limit**: ±30° (configurable)

**Implementation**:
```cpp
targetRoll = constrain(targetRoll, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
targetPitch = constrain(targetPitch, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
```

Prevents loss of control from excessive angles.

### 4. Throttle Cap

**Limit**: 65% (configurable)

**Implementation**:
```cpp
throttleNormalized = constrain(throttleNormalized, 0.0, THROTTLE_CAP);
```

**Purpose**:
- Reserves headroom for PID control authority
- Prevents over-power on aggressive maneuvers

### 5. PID Integrator Anti-Windup

**Method**: Integral clamping
```cpp
state.integral = constrain(state.integral, -coeff.iMax, coeff.iMax);
```

**Purpose**:
- Prevents integrator saturation
- Maintains controllability during sustained errors

---

## 💾 Memory Optimization

### Flash Usage

| Component | Size (bytes) | % of 32 KB |
|-----------|--------------|------------|
| AHRS Filter | ~2,500 | 7.6% |
| PID Control | ~1,800 | 5.5% |
| Kalman Filter | ~1,200 | 3.7% |
| NRF24 Library | ~12,000 | 36.6% |
| Sensor I/O | ~3,500 | 10.7% |
| Calibration | ~2,000 | 6.1% |
| Motor Mixing | ~800 | 2.4% |
| Other | ~7,000 | 21.3% |
| **Total** | **~30,800** | **~94%** |

### SRAM Usage

| Component | Size (bytes) | % of 2 KB |
|-----------|--------------|-----------|
| Global Variables | ~600 | 29% |
| Stack | ~800 | 39% |
| NRF Buffers | ~200 | 10% |
| Available | ~448 | 22% |
| **Total** | **~2,048** | **100%** |

### Optimization Techniques

1. **Fixed-point arithmetic**: Not used (float required for AHRS)
2. **PROGMEM**: String literals stored in flash
3. **Struct packing**: Minimize alignment padding
4. **Local variables**: Prefer stack over global
5. **Loop unrolling**: Minimal (code size vs. speed tradeoff)

---

## ⚡ Power Consumption

### Flight Controller

| Component | Current (mA) | Notes |
|-----------|--------------|-------|
| Arduino Nano | 20 | 16 MHz, active |
| MPU6050 | 3.5 | Normal mode |
| MS5611 | 1.5 | Conversion mode |
| NRF24L01 (RX) | 45 | PA+LNA, RX mode |
| Buzzer (active) | 30 | Only during beeps |
| LED | 10 | 220Ω resistor |
| **Total (typical)** | **~80 mA** | Excluding ESCs |

### Remote Controller

| Component | Current (mA) |
|-----------|--------------|
| Arduino Nano | 20 |
| NRF24L01 (TX) | 115 | PA+LNA, TX mode |
| Joysticks | 5 | Potentiometer dividers |
| **Total** | **~140 mA** | USB or 9V battery |

---

## 📈 Performance Benchmarks

### Attitude Estimation Accuracy

| Condition | Roll Error | Pitch Error | Yaw Drift |
|-----------|------------|-------------|-----------|
| Static (level) | ±0.5° | ±0.5° | N/A |
| Slow tilt (10°/s) | ±1.0° | ±1.0° | N/A |
| Fast tilt (90°/s) | ±3.0° | ±3.0° | N/A |
| Yaw (no mag) | N/A | N/A | ~5°/min |

### Altitude Hold Performance

| Metric | Value |
|--------|-------|
| Altitude RMS error | ±0.2 m (calm air) |
| Altitude overshoot | < 0.3 m (step input) |
| Settling time | ~3 seconds |
| Vertical velocity error | ±0.1 m/s |

### Control Loop Performance

| Loop | Bandwidth | Phase Margin | Gain Margin |
|------|-----------|--------------|-------------|
| Rate Loop | ~30 Hz | 45° | 6 dB |
| Angle Loop | ~5 Hz | 60° | 12 dB |
| Altitude Loop | ~0.5 Hz | 50° | 8 dB |

---

## 🔬 Advanced Topics

### Future Enhancements

1. **GPS Integration**:
   - Position hold mode
   - Return-to-home
   - Waypoint navigation

2. **Magnetometer Fusion**:
   - Absolute yaw heading
   - Compass mode
   - Heading hold

3. **Optical Flow**:
   - Indoor position hold
   - Velocity estimation without GPS

4. **Telemetry Logging**:
   - SD card logging
   - Black box for crash analysis

5. **Advanced Filters**:
   - Extended Kalman Filter (full 6DOF state)
   - Complementary filter variants
   - Adaptive PID gains

6. **Multi-Rotor Support**:
   - Hexacopter (6 motors)
   - Octocopter (8 motors)
   - Configurable motor mixing

---

## 📚 References

### Academic Papers

1. Mahony, R., et al. "Nonlinear Complementary Filters on the Special Orthogonal Group." IEEE TAC, 2008.
2. Welch, G. & Bishop, G. "An Introduction to the Kalman Filter." UNC Chapel Hill, 2006.

### Datasheets

- [MPU6050 Product Specification](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MS5611 Barometric Pressure Sensor](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf)
- [NRF24L01+ Datasheet](https://www.sparkfun.com/datasheets/Components/SMD/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf)

### Software Libraries

- [RF24 Library by TMRh20](https://github.com/nRF24/RF24)
- [Arduino Wire Library](https://www.arduino.cc/en/Reference/Wire)

---

## 📊 Firmware Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-11-28 | Initial release with all features |

---

## 👤 Author

Developed by an expert UAV embedded systems engineer specializing in low-level flight control firmware for resource-constrained microcontrollers.

---

**For usage documentation, see [README.md](README.md)**
