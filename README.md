# Arduino Nano Quadcopter Flight Controller

A professional-grade, production-quality flight controller firmware for Arduino Nano-based quadcopter systems. This project prioritizes flight stability, deterministic timing, robust RF communication, and comprehensive safety systems.

## Table of Contents

- [System Overview](#system-overview)
- [Hardware Configuration](#hardware-configuration)
- [Software Architecture](#software-architecture)
- [Timing Architecture](#timing-architecture)
- [RF Communication Protocol](#rf-communication-protocol)
- [Sensor Fusion](#sensor-fusion)
- [PID Control System](#pid-control-system)
- [Safety Systems](#safety-systems)
- [Class Responsibility Breakdown](#class-responsibility-breakdown)
- [Critical Implementation Details](#critical-implementation-details)
- [Tuning Guide](#tuning-guide)
- [Known Pitfalls](#known-pitfalls)
- [Best Practices](#best-practices)

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         QUADCOPTER SYSTEM ARCHITECTURE                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────────────────┐              ┌──────────────────────────────────┐ │
│  │   REMOTE CONTROLLER  │   2.4 GHz    │      FLIGHT CONTROLLER           │ │
│  │   (Arduino Nano)     │◄────────────►│      (Arduino Nano)              │ │
│  │                      │   NRF24L01   │                                  │ │
│  │  ┌────────────────┐  │   50 Hz      │  ┌───────────┐  ┌─────────────┐  │ │
│  │  │ Joysticks (4x) │  │              │  │  MPU6050  │  │ ESC x4      │  │ │
│  │  │ A0-A3          │  │              │  │  I2C      │  │ D3,D5,D6,D9 │  │ │
│  │  └────────────────┘  │              │  │  1000 Hz  │  │ 250 Hz      │  │ │
│  │  ┌────────────────┐  │              │  └───────────┘  └─────────────┘  │ │
│  │  │ Switches (2x)  │  │              │        │              ▲          │ │
│  │  │ D2, D3         │  │              │        ▼              │          │ │
│  │  └────────────────┘  │              │  ┌───────────┐  ┌─────────────┐  │ │
│  │  ┌────────────────┐  │              │  │ Attitude  │  │ Motor Mixer │  │ │
│  │  │ Buttons (2x)   │  │              │  │ Estimator │  │ Quad-X      │  │ │
│  │  │ D4, D5         │  │              │  │ 500 Hz    │  │             │  │ │
│  │  └────────────────┘  │              │  └───────────┘  └─────────────┘  │ │
│  │  ┌────────────────┐  │              │        │              ▲          │ │
│  │  │ NRF24L01       │  │              │        ▼              │          │ │
│  │  │ CE:D9 CSN:D10  │  │              │  ┌─────────────────────────────┐ │ │
│  │  └────────────────┘  │              │  │      PID Controllers        │ │ │
│  └──────────────────────┘              │  │   Roll │ Pitch │ Yaw       │ │ │
│                                        │  │         250 Hz              │ │ │
│                                        │  └─────────────────────────────┘ │ │
│                                        │  ┌─────────────┐ ┌────────────┐  │ │
│                                        │  │ LED (D7)    │ │ Buzzer(D8) │  │ │
│                                        │  │ Status      │ │ Feedback   │  │ │
│                                        │  └─────────────┘ └────────────┘  │ │
│                                        └──────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Hardware Configuration

### Flight Controller (Arduino Nano)

| Component | Pin | Description |
|-----------|-----|-------------|
| MPU6050 SDA | A4 | I2C Data |
| MPU6050 SCL | A5 | I2C Clock |
| MPU6050 INT | D2 | Data Ready Interrupt |
| Motor FL (CCW) | D3 | Front-Left ESC (Timer2) |
| Motor FR (CW) | D5 | Front-Right ESC (Timer0) |
| Motor RL (CW) | D6 | Rear-Left ESC (Timer0) |
| Motor RR (CCW) | D9 | Rear-Right ESC (Timer1) |
| NRF24L01 CE | D4 | Chip Enable |
| NRF24L01 CSN | D10 | Chip Select |
| NRF24L01 SCK | D13 | SPI Clock |
| NRF24L01 MOSI | D11 | SPI Data Out |
| NRF24L01 MISO | D12 | SPI Data In |
| LED | D7 | Arm Status Indicator |
| Buzzer | D8 | Audio Feedback |

### Remote Controller (Arduino Nano)

| Component | Pin | Description |
|-----------|-----|-------------|
| NRF24L01 CE | D9 | Chip Enable |
| NRF24L01 CSN | D10 | Chip Select |
| Arm Switch | D2 | Toggle - Arm/Disarm |
| Aux Switch | D3 | Toggle - Auxiliary |
| Calibrate Button | D4 | Momentary - IMU Cal |
| Motor Test Button | D5 | Momentary - Motor Test |
| Throttle | A0 | Joystick Axis |
| Yaw | A1 | Joystick Axis |
| Pitch | A2 | Joystick Axis |
| Roll | A3 | Joystick Axis |
| Buzzer | D6 | Audio Feedback (Optional) |

### Motor Layout (Quad-X Configuration)

```
        FRONT
    FL (CCW)    FR (CW)
         \      /
          \    /
           \  /
            \/
            /\
           /  \
          /    \
         /      \
    RL (CW)    RR (CCW)
        REAR

Motor Mixing Matrix:
┌────────┬──────────┬───────┬───────┬──────┐
│ Motor  │ Throttle │ Roll  │ Pitch │ Yaw  │
├────────┼──────────┼───────┼───────┼──────┤
│ FL     │    +     │   -   │   +   │  -   │
│ FR     │    +     │   +   │   +   │  +   │
│ RL     │    +     │   -   │   -   │  +   │
│ RR     │    +     │   +   │   -   │  -   │
└────────┴──────────┴───────┴───────┴──────┘
```

---

## Software Architecture

### Design Principles

1. **Object-Oriented in Single File**: All classes defined in single `.ino` for Arduino IDE compatibility
2. **Non-Blocking Design**: No blocking delays in main loop; all timing via `micros()` scheduling
3. **Deterministic Timing**: Fixed-period task execution for consistent control behavior
4. **Fail-Safe Priority**: Safety checks execute before control calculations

### Class Hierarchy

```
FlightController (Main State Machine)
├── IMU
│   ├── Raw sensor reading
│   ├── Calibration management
│   └── Low-pass filtering
├── AttitudeEstimator
│   └── Complementary filter fusion
├── PIDController (x3)
│   ├── Roll (angle mode)
│   ├── Pitch (angle mode)
│   └── Yaw (rate mode)
├── RadioLink
│   ├── Packet reception
│   ├── Link quality monitoring
│   └── Failsafe detection
├── MotorMixer
│   ├── Quad-X mixing
│   └── ESC PWM generation
└── UserInterface
    ├── LED control
    └── Buzzer patterns
```

---

## Timing Architecture

### Frequency Allocation

```
┌──────────────────┬──────────┬────────────┬─────────────────────────────────┐
│ Subsystem        │ Frequency│ Period     │ Rationale                       │
├──────────────────┼──────────┼────────────┼─────────────────────────────────┤
│ IMU Sampling     │ 1000 Hz  │ 1.0 ms     │ Nyquist for 500Hz fusion        │
│ Attitude Fusion  │ 500 Hz   │ 2.0 ms     │ 2x PID rate for interpolation   │
│ PID Control      │ 250 Hz   │ 4.0 ms     │ Standard for multirotor         │
│ ESC Update       │ 250 Hz   │ 4.0 ms     │ Synchronized with PID           │
│ RF Reception     │ 50 Hz    │ 20.0 ms    │ Human input bandwidth           │
│ LED Update       │ 2 Hz     │ 500.0 ms   │ Visual status indication        │
└──────────────────┴──────────┴────────────┴─────────────────────────────────┘
```

### Timing Diagram

```
Time (ms): 0    1    2    3    4    5    6    7    8    9    10   ...  20
           |    |    |    |    |    |    |    |    |    |    |         |
IMU 1kHz:  ●────●────●────●────●────●────●────●────●────●────●────...──●
           |         |         |         |         |         |         |
ATT 500Hz: ●─────────●─────────●─────────●─────────●─────────●────...──●
           |                   |                   |                   |
PID 250Hz: ●───────────────────●───────────────────●──────────────...──●
           |                                                           |
RF 50Hz:   ●───────────────────────────────────────────────────────────●
           |                   |                   |                   |
ESC 250Hz: ●───────────────────●───────────────────●──────────────...──●
           (synchronized with PID output)

Within each 4ms PID cycle:
├── 4 IMU samples collected and averaged
├── 2 attitude estimates computed
├── 1 PID calculation (Roll, Pitch, Yaw)
└── 1 ESC update (4 motors)
```

### Why Timing Matters

**Mismatched frequencies cause:**

| Problem | Cause | Symptom |
|---------|-------|---------|
| Attitude drift | Inconsistent fusion dt | Slow roll/pitch in one direction |
| PID oscillation | Variable derivative dt | Motor twitching, "toilet bowl" |
| Control aliasing | IMU < 2x PID rate | Phase lag, sluggish response |
| ESC desync | PWM faster than ESC | Motor stuttering, sync loss |
| RF latency | Blocking operations | Delayed control response |

---

## RF Communication Protocol

### Design Decisions

#### ACK vs NO_ACK Mode: **NO_ACK Selected**

| Factor | ACK Mode | NO_ACK Mode |
|--------|----------|-------------|
| Latency | 3-5 ms | ~1 ms |
| Reliability | Guaranteed delivery | Best-effort |
| Complexity | Retry logic needed | Simple |
| For Control Systems | ❌ Old data useless | ✅ Fresh data priority |

**Rationale**: In real-time control, a 20ms-old packet arriving with ACK is less valuable than dropping it and using the next fresh packet. Failsafe handles sustained loss.

#### Channel Selection: **108 (2.508 GHz)**

- Above WiFi band (2.4-2.4835 GHz)
- Reduces interference in typical environments
- Within NRF24L01 range (2.4-2.525 GHz)

#### Data Rate: **2 Mbps**

- Shorter air time = less collision probability
- Trade-off: Slightly reduced range (acceptable for LOS operation)

### Packet Structure (16 bytes)

```
┌─────────┬─────────┬─────────┬─────────┬──────────┬──────────┬──────────┬──────────┬─────────┐
│ Byte    │ 0-1     │ 2-3     │ 4-5     │ 6-7      │ 8        │ 9        │ 10-13    │ 14-15   │
├─────────┼─────────┼─────────┼─────────┼──────────┼──────────┼──────────┼──────────┼─────────┤
│ Field   │Throttle │ Yaw     │ Pitch   │ Roll     │ Switches │ Checksum │ Sequence │Reserved │
│ Type    │ uint16  │ int16   │ int16   │ int16    │ uint8    │ uint8    │ uint32   │ uint16  │
│ Range   │ 0-1000  │±500     │±500     │±500      │ Bitfield │ XOR      │ Counter  │ Future  │
└─────────┴─────────┴─────────┴─────────┴──────────┴──────────┴──────────┴──────────┴─────────┘

Switches Bitfield:
┌─────┬─────────────────┐
│ Bit │ Function        │
├─────┼─────────────────┤
│ 0   │ Arm/Disarm      │
│ 1   │ Calibrate       │
│ 2   │ Motor Test      │
│ 3   │ Auxiliary       │
│ 4-7 │ Reserved        │
└─────┴─────────────────┘
```

### Failsafe Behavior

```
┌─────────────────────┬────────────────────┬─────────────────────────────────┐
│ Time Since Packet   │ Link State         │ Action                          │
├─────────────────────┼────────────────────┼─────────────────────────────────┤
│ 0-500 ms            │ CONNECTED          │ Normal operation                │
│ 500-1000 ms         │ DEGRADED           │ LED warning, reduced authority  │
│ >1000 ms            │ FAILSAFE           │ Motors cut, buzzer alarm        │
└─────────────────────┴────────────────────┴─────────────────────────────────┘
```

---

## Sensor Fusion

### Complementary Filter vs DMP

| Criterion | Complementary Filter | MPU6050 DMP |
|-----------|---------------------|-------------|
| Latency | <100 μs | 2-4 ms |
| Tunability | Full control | Black box |
| CPU Usage | ~50 μs/cycle | ~20 μs (HW) |
| Complexity | Simple | FIFO management |
| Failure Modes | Predictable | Unknown |
| Drift (no mag) | Yaw drifts | Yaw drifts |

**Decision: Complementary Filter**

For a 250 Hz PID loop (4ms period), DMP's 2-4ms latency consumes 50-100% of the control period. The complementary filter's sub-millisecond latency preserves control bandwidth.

### Filter Implementation

```
angle = α × (angle + gyro_rate × dt) + (1-α) × accel_angle

Where:
- α = 0.98 (gyro trust factor)
- dt = 2ms (fusion period)
- Time constant τ = dt/(1-α) ≈ 100ms
```

### Why These Parameters?

- **α = 0.98**: Trusts gyro for short-term, accel for long-term drift correction
- **τ ≈ 100ms**: Fast enough to correct drift before visible attitude error
- **500 Hz fusion**: Captures all gyro dynamics, averages accel noise

---

## PID Control System

### Control Architecture

```
                    ┌─────────────────────────────────────────────────────┐
                    │              ANGLE MODE CONTROL                      │
                    │                                                      │
   Stick Input      │    ┌─────────┐     ┌─────────┐     ┌──────────┐    │
  (±50° desired)───►│───►│  Error  │────►│   PID   │────►│  Motor   │───►│──► Motors
                    │    │ Calc    │     │  Roll   │     │  Mixer   │    │
   Current Angle────│───►│         │     │  Pitch  │     │          │    │
   (from IMU)       │    └─────────┘     └─────────┘     └──────────┘    │
                    │                                                      │
                    └─────────────────────────────────────────────────────┘

                    ┌─────────────────────────────────────────────────────┐
                    │               RATE MODE (YAW ONLY)                   │
                    │                                                      │
   Stick Input      │    ┌─────────┐     ┌─────────┐     ┌──────────┐    │
  (±180°/s desired)─│───►│  Error  │────►│   PID   │────►│  Motor   │───►│──► Motors
                    │    │ Calc    │     │   Yaw   │     │  Mixer   │    │
   Gyro Rate────────│───►│         │     │         │     │          │    │
   (from IMU)       │    └─────────┘     └─────────┘     └──────────┘    │
                    │                                                      │
                    └─────────────────────────────────────────────────────┘
```

### PID Features

1. **Derivative on Measurement** (not error)
   - Prevents derivative kick on setpoint changes
   - `d_term = Kd × -d(measurement)/dt`

2. **Integral Anti-Windup**
   - Clamps integral term to ±200
   - Back-calculation: reduces integral when output saturates

3. **Derivative Low-Pass Filter**
   - IIR filter with α = 0.7
   - Reduces noise amplification from derivative term

### Default Gains (Starting Point)

```
Roll/Pitch (Angle Mode):
  Kp = 4.0   (Proportional response to angle error)
  Ki = 0.02  (Integral to eliminate steady-state error)
  Kd = 1.5   (Derivative to damp oscillations)

Yaw (Rate Mode):
  Kp = 3.0
  Ki = 0.01
  Kd = 0.0   (Often zero for rate mode)
```

---

## Safety Systems

### Arming Requirements

All conditions must be met:

1. ✅ Arm switch transitions from OFF → ON (edge detection)
2. ✅ Throttle ≤ 5% (prevents sudden motor spin)
3. ✅ IMU calibrated
4. ✅ RF link connected
5. ✅ Not in calibration or motor test mode

### State Machine

```
┌────────────────────────────────────────────────────────────────────────┐
│                      FLIGHT CONTROLLER STATE MACHINE                    │
├────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│    ┌──────────┐                                                        │
│    │   INIT   │──────────────────────────────────┐                     │
│    └────┬─────┘                                  │                     │
│         │ Success                                │ Failure             │
│         ▼                                        ▼                     │
│    ┌──────────┐     Cal Button              ┌──────────┐              │
│    │ DISARMED │◄────────────────────────────│  ERROR   │              │
│    └────┬─────┘◄──────────────┐             └──────────┘              │
│         │                      │                                       │
│         │ Arm Switch ON        │ Arm Switch OFF                       │
│         │ Throttle Low         │                                       │
│         │ RF Connected         │                                       │
│         │ IMU Calibrated       │                                       │
│         ▼                      │                                       │
│    ┌──────────┐                │                                       │
│    │  ARMED   │────────────────┤                                       │
│    └────┬─────┘                │                                       │
│         │                      │                                       │
│         │ RF Timeout           │ RF Recovery                           │
│         ▼                      │ + Disarm                              │
│    ┌──────────┐                │                                       │
│    │ FAILSAFE │────────────────┘                                       │
│    └──────────┘                                                        │
│                                                                         │
└────────────────────────────────────────────────────────────────────────┘
```

### LED Behavior

| State | LED Pattern |
|-------|-------------|
| Initializing | Fast blink (100ms) |
| Disarmed | Slow blink (500ms) |
| Armed | Solid ON |
| Failsafe | Very fast blink (100ms) |
| Error | Very fast blink (50ms) |
| Calibrating | Fast blink (50ms) |

### Buzzer Patterns

| Event | Pattern |
|-------|---------|
| Button press | Single short beep (50ms) |
| RF paired | 5 beeps |
| Calibration start | Long low beep |
| Calibration done | Rising tone sequence |
| Armed | Two-tone rising |
| Disarmed | Single falling tone |
| Ready to fly | Four-tone rising |
| Failsafe | Long low alarm |
| Error | Three rapid beeps |

---

## Class Responsibility Breakdown

### IMU Class

**Purpose**: Hardware abstraction for MPU6050

**Responsibilities**:
- Initialize MPU6050 via I2C (400kHz)
- Configure DLPF (44Hz cutoff), ranges (±2g, ±250°/s)
- Burst-read sensor data (14 bytes in single transaction)
- Apply calibration offsets
- Software low-pass filtering (IIR)
- Manage calibration procedure

**Key Methods**:
```cpp
bool begin()           // Initialize sensor
bool readRaw()         // Burst read from I2C
void processData()     // Apply calibration, scaling, filtering
void calibrate()       // 2-second calibration routine
```

### AttitudeEstimator Class

**Purpose**: Fuse IMU data into attitude angles

**Responsibilities**:
- Implement complementary filter
- Convert body rates to Euler rate
- Handle gimbal lock near ±90° pitch
- Normalize yaw to ±180°

**Key Methods**:
```cpp
void update(const IMU::Data&)  // Run fusion algorithm
float getRoll()                 // Get roll angle (degrees)
float getPitch()                // Get pitch angle (degrees)
float getYaw()                  // Get yaw angle (degrees)
```

### PIDController Class

**Purpose**: Generic PID implementation with safety features

**Responsibilities**:
- Compute PID output from setpoint and measurement
- Derivative filtering
- Integral anti-windup (clamping + back-calculation)
- Output limiting

**Key Methods**:
```cpp
void setGains(float kp, float ki, float kd)
void setIntegralLimits(float min, float max)
void setOutputLimits(float min, float max)
float calculate(float setpoint, float measurement, float dt)
void reset()
```

### RadioLink Class

**Purpose**: NRF24L01 communication management

**Responsibilities**:
- Configure and manage NRF24L01
- Receive and validate packets (checksum)
- Track packet sequence for loss detection
- Manage link state machine
- Trigger failsafe on timeout

**Key Methods**:
```cpp
bool begin()                   // Initialize radio
bool update()                  // Check for packets, update state
LinkState getLinkState()       // Get current link status
const ControlPacket& getLastPacket()  // Get last valid packet
bool isFailsafe()              // Check if in failsafe
```

### MotorMixer Class

**Purpose**: Convert control outputs to motor commands

**Responsibilities**:
- Implement Quad-X mixing matrix
- Generate PWM signals for ESCs
- Enforce motor limits
- Arm/disarm control
- Motor test sequence

**Key Methods**:
```cpp
void begin()           // Attach ESC servos
void mix(throttle, roll, pitch, yaw)  // Calculate and output
void arm() / disarm()  // Motor enable control
void motorTest()       // Test sequence (PROPS OFF!)
```

### FlightController Class

**Purpose**: Main orchestrator and state machine

**Responsibilities**:
- Initialize all subsystems
- Manage timing for all tasks
- Implement state machine
- Coordinate safety checks
- Handle user inputs

**Key Methods**:
```cpp
void begin()     // Initialize everything
void update()    // Main loop - call as fast as possible
State getState() // Get current state
```

---

## Critical Implementation Details

### I2C Speed

```cpp
Wire.setClock(400000);  // 400kHz Fast Mode
```

At 100kHz (default), reading 14 bytes takes ~1.4ms, exceeding our 1ms IMU budget. 400kHz brings this to ~350μs.

### No Blocking Code

The only blocking operations occur during:
- Calibration (user-initiated, disarmed)
- Motor test (user-initiated, disarmed)
- Buzzer patterns (brief, acceptable)

Main loop runs continuously with timing gates:

```cpp
if (now - lastPidTime >= PID_PERIOD_US) {
    lastPidTime = now;
    // Execute PID
}
```

### Memory Usage

| Resource | Used | Available | Headroom |
|----------|------|-----------|----------|
| Flash | ~20 KB | 32 KB | 12 KB |
| RAM | ~1.2 KB | 2 KB | 800 B |
| Stack | ~200 B | - | Sufficient |

### Timer Conflicts

ESC PWM uses Servo library (Timer1). Avoid:
- `tone()` on pins 9, 10 (Timer1)
- `analogWrite()` on pins 9, 10 during flight

---

## Tuning Guide

### Ziegler-Nichols Method (Modified)

1. Set Ki = 0, Kd = 0
2. Increase Kp until consistent oscillation
3. Note ultimate gain Ku and period Tu
4. Calculate:
   - Kp = 0.6 × Ku
   - Ki = 2 × Kp / Tu
   - Kd = Kp × Tu / 8
5. Fine-tune from this starting point

### Typical Gain Ranges (250mm frame)

```
┌──────────┬─────────┬─────────┬─────────┐
│ Axis     │ Kp      │ Ki      │ Kd      │
├──────────┼─────────┼─────────┼─────────┤
│ Roll     │ 2.0-8.0 │ 0.01-0.1│ 0.5-3.0 │
│ Pitch    │ 2.0-8.0 │ 0.01-0.1│ 0.5-3.0 │
│ Yaw      │ 2.0-5.0 │ 0.01-0.1│ 0.0-1.0 │
└──────────┴─────────┴─────────┴─────────┘
```

### Tuning Symptoms

| Symptom | Likely Cause | Adjustment |
|---------|--------------|------------|
| Oscillation | Kp too high | Reduce Kp, increase Kd |
| Sluggish | Kp too low | Increase Kp |
| Drift | Ki too low | Increase Ki |
| Bounce-back | Ki too high | Reduce Ki |
| Noisy motors | Kd too high | Reduce Kd, check vibration |
| Toilet bowl | Timing issues | Check loop timing consistency |

---

## Known Pitfalls

### 1. Variable Loop Timing

**Problem**: Using `delay()` or blocking operations causes inconsistent PID timing.

**Symptom**: Derivative term causes oscillation.

**Solution**: Use `micros()` scheduling with fixed periods.

### 2. I2C Blocking on Error

**Problem**: I2C operations can hang if sensor disconnects.

**Solution**: Implement timeout in I2C operations, handle errors gracefully.

### 3. SPI Contention

**Problem**: RF and ESC updates can conflict on SPI bus.

**Symptom**: Corrupted packets, motor glitches.

**Solution**: Separate timing for RF (50Hz) and ESC (250Hz), use different timeslots.

### 4. Accelerometer Noise During Flight

**Problem**: Vibration causes accelerometer readings to be unreliable.

**Symptom**: Attitude drift during aggressive maneuvers.

**Solution**: 
- Increase complementary filter α (trust gyro more)
- Soft-mount IMU
- DLPF at 44Hz or lower

### 5. ESC Calibration

**Problem**: ESCs not calibrated to same throttle range.

**Symptom**: Quad tilts at hover throttle.

**Solution**: Calibrate all ESCs to same range (1000-2000μs).

### 6. Prop Direction

**Problem**: Props spinning wrong direction.

**Symptom**: Quad flips on takeoff.

**Solution**: Verify motor directions match mixing matrix (FL=CCW, FR=CW, etc.).

### 7. RF Antenna Orientation

**Problem**: Antenna perpendicular to receiver.

**Symptom**: Intermittent signal loss.

**Solution**: Keep antennas vertical, maintain line-of-sight.

---

## Best Practices

### Pre-Flight Checklist

1. ☐ Props secure, correct rotation
2. ☐ Battery charged, secured
3. ☐ IMU calibration performed on level surface
4. ☐ Throttle at minimum before powering on
5. ☐ Arm switch OFF before powering on
6. ☐ Visual check of all connections
7. ☐ Clear area (3m radius minimum)
8. ☐ Radio link verified (LED behavior)

### Code Quality

1. **No magic numbers**: All constants in named namespaces
2. **Explicit types**: Use `uint16_t`, `int32_t` instead of `int`
3. **Const correctness**: Mark read-only variables `const`
4. **Single responsibility**: Each class has one job
5. **Fail-safe defaults**: Motors OFF on any error

### Hardware Reliability

1. **Capacitor on NRF24L01**: 10-100μF across VCC-GND
2. **Ferrite beads**: On motor power leads
3. **Soft mounting**: Vibration isolation for IMU
4. **Proper wire gauge**: ESC leads appropriately sized
5. **Strain relief**: On all connectors

---

## Dependencies

- **RF24 Library**: TMRH20 fork (https://github.com/nRF24/RF24)
- **Wire Library**: Built-in Arduino I2C
- **Servo Library**: Built-in Arduino PWM
- **SPI Library**: Built-in Arduino SPI

Install via Arduino Library Manager:
```
RF24 by TMRh20
```

---

## Building and Uploading

1. Open `quadcopter_fc/quadcopter_fc.ino` in Arduino IDE
2. Select Board: "Arduino Nano"
3. Select Processor: "ATmega328P" (or "Old Bootloader" if needed)
4. Select Port
5. Upload

Repeat for `quadcopter_remote/quadcopter_remote.ino` on the remote controller Nano.

---

## License

This project is provided for educational purposes. Use at your own risk. Always prioritize safety when working with multirotors.

---

## Version History

- **1.0.0**: Initial release
  - Complete flight controller implementation
  - Remote controller implementation
  - Comprehensive documentation

---

*This firmware is designed for serious flight control applications. It is not a beginner tutorial. Ensure you understand the code before flying.*
