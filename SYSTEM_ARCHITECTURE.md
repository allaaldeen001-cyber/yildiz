# System Architecture

Visual overview of the Professional Arduino Nano Drone System.

---

## 🏗️ High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        DRONE SYSTEM OVERVIEW                         │
└─────────────────────────────────────────────────────────────────────┘

    ┌──────────────────────┐                    ┌──────────────────────┐
    │   REMOTE CONTROLLER  │                    │  FLIGHT CONTROLLER   │
    │                      │                    │                      │
    │  ┌────────────────┐  │   NRF24L01 2.4GHz │  ┌────────────────┐  │
    │  │  Arduino Nano  │  │ ◄────────────────►│  │  Arduino Nano  │  │
    │  │   + NRF24L01   │  │    Bidirectional  │  │   + NRF24L01   │  │
    │  └────────────────┘  │    50Hz Control   │  │   + MPU6050    │  │
    │          │           │   250Hz Telemetry  │  └────────────────┘  │
    │  ┌───────▼────────┐  │                    │          │           │
    │  │   INPUTS       │  │                    │  ┌───────▼────────┐  │
    │  │ • 2 Joysticks  │  │                    │  │ FLIGHT CONTROL │  │
    │  │ • 2 Buttons    │  │                    │  │   • PID Loop   │  │
    │  │ • 2 Switches   │  │                    │  │   • Sensors    │  │
    │  └────────────────┘  │                    │  │   • Safety     │  │
    │          │           │                    │  └───────┬────────┘  │
    │  ┌───────▼────────┐  │                    │          │           │
    │  │ SERIAL OUTPUT  │  │                    │  ┌───────▼────────┐  │
    │  │  • Dashboard   │  │                    │  │    4 MOTORS    │  │
    │  │  • Telemetry   │  │                    │  │  (ESC + BLDC)  │  │
    │  │  • Status      │  │                    │  └────────────────┘  │
    │  └────────────────┘  │                    │                      │
    └──────────────────────┘                    └──────────────────────┘
           Pilot Side                                  Drone Side
```

---

## 📡 Communication Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                     COMMUNICATION PROTOCOL                           │
└─────────────────────────────────────────────────────────────────────┘

REMOTE CONTROLLER                              FLIGHT CONTROLLER
─────────────────                              ─────────────────

Read Inputs:                                   Listen for packets
├─ Joysticks (A0-A3)                                    │
├─ Buttons (D4, D5)                                     │
└─ Switches (D2, D3)                                    │
        │                                               │
        ▼                                               │
Build Packet:                                           │
├─ Throttle: 1000-2000                                  │
├─ Yaw: 1000-2000                                       │
├─ Pitch: 1000-2000                                     │
├─ Roll: 1000-2000                                      │
├─ Buttons: bool                                        │
├─ Switches: bool                                       │
└─ Checksum: uint8                                      │
        │                                               │
        ▼                                               │
Transmit via NRF24                                      │
    (50Hz rate)                                         │
        │                                               │
        └──────────────────────────────────────────────►│
                        Control Data                    │
                                                        ▼
                                              Receive & Validate:
                                              ├─ Verify checksum
                                              ├─ Update lastRxTime
                                              └─ Extract commands
                                                        │
                                                        ▼
                                              Execute Flight Control:
                                              ├─ Read IMU (MPU6050)
                                              ├─ Calculate angles
                                              ├─ Compute PID
                                              ├─ Mix motors
                                              └─ Write ESCs (250Hz)
                                                        │
                                                        ▼
                                              Build Telemetry:
                                              ├─ Angles (roll/pitch/yaw)
                                              ├─ Gyro data
                                              ├─ Armed status
                                              ├─ Calibration status
                                              └─ Checksum
                                                        │
        ┌──────────────────────────────────────────────┘
        │                   Telemetry Data
        ▼
Receive Telemetry
        │
        ▼
Display on Serial:
├─ Communication status
├─ Control inputs
├─ Drone telemetry
└─ Warnings/alerts

```

---

## 🔄 Flight Controller State Machine

```
┌─────────────────────────────────────────────────────────────────────┐
│                    FLIGHT CONTROLLER STATES                          │
└─────────────────────────────────────────────────────────────────────┘

    ┌──────────────┐
    │   POWER ON   │
    └──────┬───────┘
           │
           ▼
    ┌──────────────┐
    │ INITIALIZE   │
    │ • NRF24L01   │
    │ • MPU6050    │
    │ • PID        │
    │ • Motors OFF │
    └──────┬───────┘
           │
           ▼
    ┌──────────────┐         Signal Lost
    │   WAITING    │◄─────────────────┐
    │ • LED: Fast  │                  │
    │ • Not Cal.   │                  │
    └──────┬───────┘                  │
           │                          │
           │ Button_1 Pressed         │
           ▼                          │
    ┌──────────────┐  Failed         │
    │ CALIBRATING  ├─────────┐       │
    │ • LED: Rapid │         │       │
    │ • 2000 Reads │         │       │
    └──────┬───────┘         │       │
           │                 │       │
           │ Success         │       │
           ▼                 ▼       │
    ┌──────────────┐  ┌─────────┐   │
    │  CALIBRATED  │  │  ERROR  │   │
    │ • LED: Slow  │  │ 1 Long  │   │
    │ • Ready      │  │  Beep   │   │
    └──────┬───────┘  └─────────┘   │
           │                         │
           │ Button_2 + Conditions   │
           ▼                         │
    ┌──────────────┐                │
    │    ARMED     │────────────────┤
    │ • LED: Solid │   Kill Switch  │
    │ • 3 Beeps    │   or Low Thr   │
    │ • PID Active │                │
    └──────┬───────┘                │
           │                         │
           │ Flying...               │
           ▼                         │
    ┌──────────────┐                │
    │   DISARMED   │────────────────┘
    │ • Motors OFF │    Timeout
    │ • 1 Beep     │
    └──────────────┘

```

---

## ⚙️ Control Loop Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│              FLIGHT CONTROLLER MAIN LOOP (250Hz)                     │
└─────────────────────────────────────────────────────────────────────┘

    START LOOP (every 4ms)
         │
         ▼
    ┌────────────────┐
    │  Timing Check  │ ◄──┐
    │ Wait for 4ms   │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │  Read MPU6050  │    │
    │ • Gyro XYZ     │    │
    │ • Accel XYZ    │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │ Sensor Fusion  │    │
    │ • Integrate    │    │
    │ • Filter       │    │
    │ • Angles       │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │ Check Radio RX │    │
    │ • New data?    │    │
    │ • Valid CRC?   │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │ Handle Inputs  │    │
    │ • Calibration  │    │
    │ • Arming       │    │
    │ • Failsafe     │    │
    └────────┬───────┘    │
             │            │
             ▼            │
       ┌─────────┐        │
       │ Armed?  │        │
       └────┬────┘        │
            │ No          │
            ├────────────►│ (Skip PID)
            │ Yes         │
            ▼             │
    ┌────────────────┐    │
    │  Compute PID   │    │
    │ • Roll         │    │
    │ • Pitch        │    │
    │ • Yaw          │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │  Motor Mixer   │    │
    │ FL = T-R-P-Y   │    │
    │ FR = T+R-P+Y   │    │
    │ RR = T+R+P-Y   │    │
    │ RL = T-R+P+Y   │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │ Apply Limits   │    │
    │ • Min: 1000    │    │
    │ • Max: 2000    │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │  Write Motors  │    │
    │ • PWM Output   │    │
    │ • All 4 ESCs   │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │ Send Telemetry │    │
    │ (every 10 loops)   │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │ Update LED/Beep│    │
    └────────┬───────┘    │
             │            │
             └────────────┘
        Loop repeats...

```

---

## 🎮 Remote Controller Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│              REMOTE CONTROLLER MAIN LOOP (50Hz)                      │
└─────────────────────────────────────────────────────────────────────┘

    START LOOP (every 20ms)
         │
         ▼
    ┌────────────────┐
    │  Read Inputs   │
    │ • ADC (A0-A3)  │ ◄──┐
    │ • Digital pins │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │ Apply Deadband │    │
    │ • Center sticks│    │
    │ • ±20 points   │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │ Map to Range   │    │
    │ 0-1023 →       │    │
    │ 1000-2000      │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │ Button Logic   │    │
    │ • Edge detect  │    │
    │ • Debounce     │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │ Build Packet   │    │
    │ • 32 bytes     │    │
    │ • Checksum     │    │
    └────────┬───────┘    │
             │            │
             ▼            │
       ┌─────────┐        │
       │20ms up? │        │
       └────┬────┘        │
            │ No          │
            ├────────────►│
            │ Yes         │
            ▼             │
    ┌────────────────┐    │
    │  Transmit NRF  │    │
    │ • Write packet │    │
    │ • Check ACK    │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │ Receive Telem. │    │
    │ • If available │    │
    │ • Verify CRC   │    │
    └────────┬───────┘    │
             │            │
             ▼            │
    ┌────────────────┐    │
    │ Update Display │    │
    │ (every 1 sec)  │    │
    │ • Connection   │    │
    │ • Controls     │    │
    │ • Telemetry    │    │
    └────────┬───────┘    │
             │            │
             └────────────┘
        Loop repeats...

```

---

## 🧮 PID Controller Detail

```
┌─────────────────────────────────────────────────────────────────────┐
│                      PID CONTROL ALGORITHM                           │
└─────────────────────────────────────────────────────────────────────┘

Input: Desired Angle (Setpoint)
       Current Angle (Input)
       Time Delta (dt)

    ┌──────────────────┐
    │  Error = SP - PV │  (Setpoint - Process Variable)
    └────────┬─────────┘
             │
     ┌───────┴────────┐
     │                │
     ▼                ▼
┌─────────┐      ┌─────────┐
│    P    │      │    I    │
│ Kp × e  │      │ Ki × Σe │
└────┬────┘      └────┬────┘
     │                │
     │           ┌────▼────────┐
     │           │ Anti-Windup │
     │           │ Limit: ±max │
     │           └────┬────────┘
     │                │
     │                │        ┌─────────┐
     │                │        │    D    │
     │                │        │ Kd×Δe/Δt│
     │                │        └────┬────┘
     │                │             │
     └────────┬───────┴─────────────┘
              │
              ▼
      ┌───────────────┐
      │  PID Output   │
      │  = P + I + D  │
      └───────┬───────┘
              │
              ▼
      ┌───────────────┐
      │ Limit Output  │
      │   ±400 max    │
      └───────┬───────┘
              │
              ▼
         Motor Mixer

Roll PID:    Kp=1.3,  Ki=0.04, Kd=18.0
Pitch PID:   Kp=1.3,  Ki=0.04, Kd=18.0
Yaw PID:     Kp=2.0,  Ki=0.02, Kd=0.0

```

---

## 🚁 Motor Mixing (X Configuration)

```
┌─────────────────────────────────────────────────────────────────────┐
│                        MOTOR MIXING LOGIC                            │
└─────────────────────────────────────────────────────────────────────┘

Inputs:
├─ Throttle: Base power (1000-2000)
├─ Roll PID: Left/Right correction (-400 to +400)
├─ Pitch PID: Forward/Back correction (-400 to +400)
└─ Yaw PID: Rotation correction (-400 to +400)

                    FRONT
              FL ↻        ↺ FR
                 \        /
                  \      /
                   \    /
                    \  /
                     \/
                     /\
                    /  \
                   /    \
                  /      \
                 /        \
              RL ↺        ↻ RR
                    REAR

Motor Equations:
────────────────
FL = Throttle - RollPID - PitchPID - YawPID
FR = Throttle + RollPID - PitchPID + YawPID
RR = Throttle + RollPID + PitchPID - YawPID
RL = Throttle - RollPID + PitchPID + YawPID

Example (hover with slight right tilt correction):
────────────────────────────────────────────────
Throttle = 1500 (mid-throttle)
RollPID = +50 (correct right tilt)
PitchPID = 0 (level)
YawPID = 0 (no rotation)

FL = 1500 - 50 - 0 - 0 = 1450 (slower)
FR = 1500 + 50 - 0 + 0 = 1550 (faster)
RR = 1500 + 50 + 0 - 0 = 1550 (faster)
RL = 1500 - 50 + 0 + 0 = 1450 (slower)

Result: Right side motors faster → drone tilts left → corrects right tilt

All outputs constrained to 1000-2000 range.

```

---

## 📊 Data Structures

```
┌─────────────────────────────────────────────────────────────────────┐
│                          DATA PACKETS                                │
└─────────────────────────────────────────────────────────────────────┘

CONTROL DATA (RC → FC)
──────────────────────
struct ControlData {
  uint16_t throttle;      // 1000-2000  [2 bytes]
  uint16_t yaw;           // 1000-2000  [2 bytes]
  uint16_t pitch;         // 1000-2000  [2 bytes]
  uint16_t roll;          // 1000-2000  [2 bytes]
  bool calibrate;         // Button 1   [1 byte]
  bool motorArm;          // Button 2   [1 byte]
  bool altitudeHold;      // Switch 1   [1 byte]
  bool killSwitch;        // Switch 2   [1 byte]
  uint8_t checksum;       // XOR check  [1 byte]
};                        // Total: 13 bytes


TELEMETRY DATA (FC → RC)
────────────────────────
struct TelemetryData {
  float batteryVoltage;   // Voltage    [4 bytes]
  int16_t gyroX;          // Raw gyro   [2 bytes]
  int16_t gyroY;          //            [2 bytes]
  int16_t gyroZ;          //            [2 bytes]
  int16_t accelX;         // Raw accel  [2 bytes]
  int16_t accelY;         //            [2 bytes]
  int16_t accelZ;         //            [2 bytes]
  float roll;             // Degrees    [4 bytes]
  float pitch;            // Degrees    [4 bytes]
  float yaw;              // Degrees    [4 bytes]
  bool armed;             // Status     [1 byte]
  bool calibrated;        // Status     [1 byte]
  uint8_t checksum;       // XOR check  [1 byte]
};                        // Total: 31 bytes


NRF24L01 CONFIGURATION
──────────────────────
Channel: 103 (2.503 GHz)
Data Rate: 250 kbps (long range)
Power: PA_MAX (+20dBm typical)
Payload: 32 bytes (max)
CRC: 2 bytes
Auto-ACK: Enabled
Retries: 15 × (5 × 250μs delay)
Address Width: 5 bytes
Pipes: 2 (bidirectional)

```

---

## 🔌 Hardware Connections

```
┌─────────────────────────────────────────────────────────────────────┐
│                   FLIGHT CONTROLLER BLOCK DIAGRAM                    │
└─────────────────────────────────────────────────────────────────────┘

                    ┌─────────────────────┐
                    │   ARDUINO NANO      │
                    │   ATmega328P        │
                    │   16MHz / 5V        │
                    └──────────┬──────────┘
                               │
        ┌──────────────────────┼──────────────────────┐
        │                      │                      │
    ┌───▼────┐           ┌─────▼────┐          ┌─────▼────┐
    │ NRF24  │           │ MPU6050  │          │   I/O    │
    │ Module │           │   IMU    │          │Buzzer/LED│
    └────────┘           └──────────┘          └──────────┘
        │                     │
        │ SPI                 │ I2C
        │ CE=D4               │ SDA=A4
        │ CSN=D10             │ SCL=A5
        │                     │ INT=D2
        │                     │
        └─────────┬───────────┘
                  │
          ┌───────▼────────┐
          │   CONTROL      │
          │   LOGIC        │
          │   • PID        │
          │   • Safety     │
          │   • Failsafe   │
          └───────┬────────┘
                  │
        ┌─────────┼─────────┐
        │         │         │
    ┌───▼──┐  ┌──▼──┐  ┌───▼──┐
    │ ESC1 │  │ESC2 │  │ ESC3 │  etc...
    │  FL  │  │ FR  │  │  RR  │
    └───┬──┘  └──┬──┘  └───┬──┘
        │        │          │
    ┌───▼──┐  ┌──▼──┐  ┌───▼──┐
    │Motor │  │Motor│  │Motor │
    │  1   │  │  2  │  │  3   │
    └──────┘  └─────┘  └──────┘

POWER FLOW:
───────────
LiPo 3S (11.1V) → 4× ESC → 4× Motors
                       │
                       └─→ BEC (5V) → Arduino Nano
                                    → MPU6050
                                    → Buzzer/LED

NRF24 powered by 3.3V rail with 10μF capacitor

```

---

## 📈 Performance Analysis

```
┌─────────────────────────────────────────────────────────────────────┐
│                       TIMING BREAKDOWN                               │
└─────────────────────────────────────────────────────────────────────┘

FLIGHT CONTROLLER (4ms loop = 250Hz)
────────────────────────────────────
MPU6050 Read (I2C):          ~0.5ms
Angle Calculation:           ~0.3ms
Radio Check:                 ~0.1ms
Input Processing:            ~0.1ms
PID Computation (3 axes):    ~0.4ms
Motor Mixing:                ~0.1ms
PWM Output:                  ~0.1ms
Telemetry (every 10 loops):  ~0.2ms
Misc/Overhead:               ~0.3ms
                            ──────
Idle Time:                   ~2.0ms (50% CPU usage)


REMOTE CONTROLLER (20ms loop = 50Hz)
────────────────────────────────────
ADC Reads (4 channels):      ~0.5ms
Digital Reads:               ~0.1ms
Data Processing:             ~0.2ms
Packet Building:             ~0.1ms
NRF Transmit:                ~1.0ms
Receive Check:               ~0.5ms
Serial Update (1Hz):         ~10ms (every 50 loops)
Misc/Overhead:               ~0.3ms
                            ──────
Idle Time:                   ~17.3ms (13% CPU usage)


LATENCY ANALYSIS
────────────────
RC Input → FC Action:
  RC Read:              0.5ms
  RC Process:           0.2ms
  NRF TX:               1.0ms
  Air Time (250kbps):   1.0ms
  FC Receive:           0.5ms
  FC Process:           0.5ms
  PID Compute:          0.4ms
  Motor Update:         0.1ms
                       ─────
  Total Latency:       ~4.2ms ✓ Excellent


RADIO PERFORMANCE
─────────────────
Packet Success Rate: >98% (typical)
Range (outdoor):     ~500m
Range (indoor):      ~100m
Retry Rate:          <2%

```

---

## 🎯 System Requirements

```
┌─────────────────────────────────────────────────────────────────────┐
│                    MINIMUM vs RECOMMENDED                            │
└─────────────────────────────────────────────────────────────────────┘

MINIMUM SETUP
─────────────
✓ Arduino Nano clone ($3)
✓ Basic NRF24L01 (with PA+LNA!)
✓ Cheap MPU6050 module
✓ 30A generic ESCs
✓ 1000KV motors
✓ Plastic frame
✓ 2200mAh battery
─────────────
Cost: ~$180
Performance: Basic stable flight


RECOMMENDED SETUP
─────────────────
✓ Genuine Arduino Nano or quality clone
✓ Quality NRF24L01 PA+LNA (tested)
✓ Proven MPU6050 (InvenSense)
✓ BLHeli ESCs 30A
✓ Quality 1100KV motors
✓ Carbon fiber frame
✓ 2200mAh quality LiPo
─────────────
Cost: ~$300
Performance: Smooth, reliable flight


PROFESSIONAL SETUP
──────────────────
✓ Arduino Nano Every (faster)
✓ Premium NRF24L01 modules
✓ MPU6050 with low drift
✓ High-end BLHeli_32 ESCs
✓ Premium motors (balanced)
✓ Quality CF frame with dampening
✓ 2500mAh high C-rating LiPo
─────────────
Cost: ~$450
Performance: Competition-grade

```

---

**This architecture is designed for:**
- ✅ Reliability
- ✅ Safety
- ✅ Expandability
- ✅ Educational value
- ✅ Professional performance

