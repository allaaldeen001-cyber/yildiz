# System Architecture Diagram

## Complete System Overview

```
┌──────────────────────────────────────────────────────────────────────────┐
│                    QUADCOPTER DRONE SYSTEM ARCHITECTURE                   │
└──────────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────┐         ┌─────────────────────────────────┐
│      REMOTE CONTROLLER          │  2.4GHz │       FLIGHT CONTROLLER         │
│         (TX Mode)               │ ◄──────►│          (RX Mode)              │
│                                 │  NRF24  │                                 │
│  ┌───────────────────────────┐ │         │  ┌───────────────────────────┐  │
│  │   ARDUINO NANO            │ │         │  │   ARDUINO NANO            │  │
│  │   ATmega328P @ 16MHz      │ │         │  │   ATmega328P @ 16MHz      │  │
│  └─────────┬─────────────────┘ │         │  └──────┬────────────────────┘  │
│            │                    │         │         │                        │
│  ┌─────────▼─────────┐         │         │  ┌──────▼──────────────────┐    │
│  │  NRF24L01 PA+LNA  │         │         │  │  NRF24L01 PA+LNA        │    │
│  │  CE:D9  CSN:D10   │         │         │  │  CE:D4  CSN:D10         │    │
│  │  Channel: 103     │         │         │  │  Channel: 103           │    │
│  └───────────────────┘         │         │  └─────────────────────────┘    │
│                                 │         │                                 │
│  ┌───────────────────────────┐ │         │  ┌─────────────────────────┐   │
│  │   JOYSTICKS               │ │         │  │   MPU6050 IMU           │   │
│  │                           │ │         │  │   6-axis Gyro+Accel     │   │
│  │  Left:  Throttle (A0)    │ │         │  │   INT:D2  SDA:A4        │   │
│  │         Yaw (A1)         │ │         │  │           SCL:A5        │   │
│  │                           │ │         │  │   250Hz Sample Rate     │   │
│  │  Right: Pitch (A2)       │ │         │  └─────────────────────────┘   │
│  │         Roll (A3)        │ │         │                                 │
│  └───────────────────────────┘ │         │  ┌─────────────────────────┐   │
│                                 │         │  │   BUZZER (D8)           │   │
│  ┌───────────────────────────┐ │         │  │   Audio Feedback        │   │
│  │   BUTTONS                 │ │         │  └─────────────────────────┘   │
│  │   BTN1: Calibrate   (D4) │ │         │                                 │
│  │   BTN2: ESC Cal     (D5) │ │         │  ┌─────────────────────────┐   │
│  │   BTN3: Motor Test  (D6) │ │         │  │   STATUS LED (D7)       │   │
│  └───────────────────────────┘ │         │  │   Link & Status         │   │
│                                 │         │  └─────────────────────────┘   │
│  ┌───────────────────────────┐ │         │                                 │
│  │   SWITCHES                │ │         │  ┌─────────────────────────┐   │
│  │   SW1: Alt Hold     (D2) │ │         │  │   MOTORS & ESCs         │   │
│  │   SW2: Arm/Disarm   (D3) │ │         │  │                         │   │
│  └───────────────────────────┘ │         │  │   FL (D3) ↻   FR (D5) ↺ │   │
│                                 │         │  │      \         /        │   │
│  ┌───────────────────────────┐ │         │  │       \   ^   /         │   │
│  │  POWER: 9V Battery        │ │         │  │        \ FWD /          │   │
│  └───────────────────────────┘ │         │  │       X  |  X           │   │
│                                 │         │  │        /   \            │   │
└─────────────────────────────────┘         │  │       /     \           │   │
                                             │  │   RL (D9) ↺   RR (D6) ↻ │   │
                                             │  └─────────────────────────┘   │
                                             │                                 │
                                             │  ┌─────────────────────────┐   │
                                             │  │  POWER: 3S LiPo 11.1V   │   │
                                             │  │  via ESC BEC            │   │
                                             │  └─────────────────────────┘   │
                                             └─────────────────────────────────┘
```

---

## Data Flow Diagram

```
┌──────────────────────────────────────────────────────────────────────────┐
│                           DATA FLOW OVERVIEW                              │
└──────────────────────────────────────────────────────────────────────────┘

REMOTE CONTROLLER                               FLIGHT CONTROLLER
─────────────────                               ─────────────────

  Read Inputs                                    
      ├─ Throttle (A0)         
      ├─ Yaw (A1)             
      ├─ Pitch (A2)           
      ├─ Roll (A3)            
      ├─ Buttons (D4,D5,D6)   
      └─ Switches (D2,D3)     
           │                                    
           ▼                                    
  Package RC_Data                              
      ├─ throttle: 1000-2000                   
      ├─ yaw: 1000-2000                        
      ├─ pitch: 1000-2000                      
      ├─ roll: 1000-2000                       
      ├─ armed: 0/1                            
      ├─ calibrate: 0/1                        
      └─ checksum                              
           │                                    
           ▼                                    
  ╔═══════════════════╗                        
  ║  NRF24L01 Send    ║ ─────────────────────> ╔═══════════════════╗
  ║  Channel 103      ║      2.4GHz            ║  NRF24L01 Receive ║
  ║  50Hz Update      ║ <───────────────────── ║  Channel 103      ║
  ╚═══════════════════╝      ACK Payload       ╚═══════════════════╝
           │                                              │
           │                                              ▼
           │                                       Verify Checksum
           │                                              │
           │                                              ▼
           │                                       Process Commands
           │                                              │
           │                                              ├─> Calibrate?
           │                                              ├─> ESC Cal?
           │                                              └─> Armed?
           │                                              │
           │                                              ▼
           │                                       ╔═══════════════════╗
           │                                       ║   Read MPU6050    ║
           │                                       ║   250Hz IMU Data  ║
           │                                       ╚═══════════════════╝
           │                                              │
           │                                              ├─> Gyro X,Y,Z
           │                                              └─> Accel X,Y,Z
           │                                              │
           │                                              ▼
           │                                       Complementary Filter
           │                                              │
           │                                              ├─> Angle X (Roll)
           │                                              └─> Angle Y (Pitch)
           │                                              │
           │                                              ▼
           │                                       ╔═══════════════════╗
           │                                       ║   PID Control     ║
           │                                       ║   Roll/Pitch/Yaw  ║
           │                                       ╚═══════════════════╝
           │                                              │
           │                                              ├─> Roll PID Out
           │                                              ├─> Pitch PID Out
           │                                              └─> Yaw PID Out
           │                                              │
           │                                              ▼
           │                                       ╔═══════════════════╗
           │                                       ║   Motor Mixing    ║
           │                                       ║   X Configuration ║
           │                                       ╚═══════════════════╝
           │                                              │
           │                                              ├─> FL Motor PWM
           │                                              ├─> FR Motor PWM
           │                                              ├─> RR Motor PWM
           │                                              └─> RL Motor PWM
           │                                              │
           │                                              ▼
           │                                       Send to ESCs (PWM)
           │                                              │
           │                                              ▼
           │                                       Build FC_Ack
           │                                              │
           │                                              ├─> Status
           │                                              ├─> Cal Result
           │                                              ├─> Battery V
           │                                              └─> RSSI
           │                                              │
           ▼◄─────────────────────────────────────────────┘
    Process Telemetry                            Send ACK Payload
           │
           ├─> Update Status
           ├─> Battery Level
           └─> Flight Mode
           │
           ▼
    ╔═══════════════════╗
    ║  Serial Monitor   ║
    ║  Status Display   ║
    ╚═══════════════════╝
```

---

## Control Loop Timing

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         TIMING DIAGRAM                                    │
└──────────────────────────────────────────────────────────────────────────┘

REMOTE CONTROLLER (50Hz = 20ms)
────────────────────────────────
│ 0ms                       20ms                       40ms                 │
├───────────────────────────┼───────────────────────────┼──────────────────►
│                           │                           │
│ Read Inputs               │ Read Inputs               │ Read Inputs
│ Package Data              │ Package Data              │ Package Data
│ Send via NRF              │ Send via NRF              │ Send via NRF
│ Display Status            │ Display Status            │ Display Status
└───────────────────────────┴───────────────────────────┴───────────────────


FLIGHT CONTROLLER (250Hz = 4ms)
────────────────────────────────
│ 0ms   4ms   8ms   12ms  16ms  20ms  24ms  28ms  32ms  36ms  40ms        │
├─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────────────►
│     │     │     │     │     │     │     │     │     │     │
│ Read│ Read│ Read│ Read│ Read│ Read│ Read│ Read│ Read│ Read│ Read
│ IMU │ IMU │ IMU │ IMU │ IMU │ IMU │ IMU │ IMU │ IMU │ IMU │ IMU
│ PID │ PID │ PID │ PID │ PID │ PID │ PID │ PID │ PID │ PID │ PID
│ Mix │ Mix │ Mix │ Mix │ Mix │ Mix │ Mix │ Mix │ Mix │ Mix │ Mix
│ Mtrs│ Mtrs│ Mtrs│ Mtrs│ Mtrs│ Mtrs│ Mtrs│ Mtrs│ Mtrs│ Mtrs│ Mtrs
└─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────────────

Note: RC data updates are received asynchronously and used in next cycle
```

---

## State Machine Diagram

```
┌──────────────────────────────────────────────────────────────────────────┐
│                    FLIGHT CONTROLLER STATE MACHINE                        │
└──────────────────────────────────────────────────────────────────────────┘

           ┌─────────────┐
           │   INIT      │  Power On, Initialize Hardware
           └──────┬──────┘
                  │
                  ▼
           ┌─────────────┐
     ┌─────┤   IDLE      │◄─────┐  Waiting for Commands
     │     └──────┬──────┘      │  Motors OFF
     │            │              │
     │            │ BTN1         │ Disarm (SW2=0)
     │            │ Pressed      │
     │            ▼              │
     │     ┌─────────────┐      │
     │     │ CALIBRATING │──────┤  Gyro Calibration
     │     └──────┬──────┘      │  8 seconds
     │            │              │
     │            │ Complete     │
     │            ▼              │
     │     Back to IDLE          │
     │            │              │
     │            │ SW2=1        │
     │            │ & Throttle   │
     │            │ < 10%        │
     │            ▼              │
     │     ┌─────────────┐      │
     └─────┤   ARMED     │──────┘  Motors Idle
           └──────┬──────┘         Ready to Fly
                  │
                  │ Throttle > 10%
                  ▼
           ┌─────────────┐
     ┌─────┤   FLYING    │◄─────┐  Active Flight
     │     └──────┬──────┘      │  PID Control Active
     │            │              │
     │            │ Throttle     │ Continue Flying
     │            │ < 10%        │
     │            └──────────────┘
     │
     │ Signal Loss
     │ > 1000ms
     │
     ▼
┌─────────────┐
│  FAILSAFE   │  Emergency Disarm
└──────┬──────┘  Motors OFF
       │          Alert Buzzer
       │
       ▼ Power Cycle
   Back to INIT
```

---

## Motor Mixing Formula

```
┌──────────────────────────────────────────────────────────────────────────┐
│                        MOTOR MIXING (X CONFIG)                            │
└──────────────────────────────────────────────────────────────────────────┘

Input Channels:
  T = Throttle   (1000-2000)
  R = Roll PID   (-400 to +400)
  P = Pitch PID  (-400 to +400)
  Y = Yaw PID    (-400 to +400)

Motor Outputs:
  FL = T + P + R - Y    (Front Left,  ↻ CCW)
  FR = T + P - R + Y    (Front Right, ↺ CW)
  RR = T - P - R - Y    (Rear Right,  ↻ CCW)
  RL = T - P + R + Y    (Rear Left,   ↺ CW)

Constrain each motor: 1000 ≤ Motor ≤ 2000

Visual Representation:
          FRONT (+Pitch)
               ↑
    FL ↻       │       FR ↺
      +P+R-Y   │   +P-R+Y
        \      │      /
         \     │     /
LEFT ←────┼────┼────┼────→ RIGHT
(-Roll)   │  \ │ /  │   (+Roll)
          │   \│/   │
          │    X    │
          │   /│\   │
          │  / │ \  │
        /     │     \
       /      │      \
     -P+R+Y   │   -P-R-Y
    RL ↺       │       RR ↻
               ↓
           REAR (-Pitch)

    ↻ = Counter-Clockwise
    ↺ = Clockwise
```

---

## PID Control Diagram

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         PID CONTROL LOOP                                  │
└──────────────────────────────────────────────────────────────────────────┘

                      ROLL AXIS EXAMPLE
                      ─────────────────

RC Input              Setpoint                        Plant
(Roll Stick)    ┌───►(Desired  ┌──────┐  Control  ┌──────────┐  Measured
1000-2000       │    Rate)     │ PID  ├──────────►│  DRONE   ├─┐ Angle/Rate
    │           │  -180 to     │ Ctrl │  Signal   │  (Motor  │ │
    │           │  +180 °/s    └──────┘           │  Mixing) │ │
    │           │      ▲                           └──────────┘ │
    ▼           │      │                                         │
┌────────┐      │      │                                         │
│ Map to │      │      │ Error                                   │
│ Rate   ├──────┘      │ ← ──────────────────────────────────────┘
└────────┘             │
                       │
                ┌──────┴─────────────────────────────┐
                │   PID Calculation                  │
                │                                    │
                │   P = Kp × Error                  │
                │   I = Ki × ∫Error dt              │
                │   D = Kd × dError/dt              │
                │                                    │
                │   Output = P + I + D              │
                └────────────────────────────────────┘

Same process applies to:
  - Pitch Axis (Forward/Backward)
  - Yaw Axis (Rotation)
```

---

## Safety Features Flowchart

```
┌──────────────────────────────────────────────────────────────────────────┐
│                        SAFETY CHECKS                                      │
└──────────────────────────────────────────────────────────────────────────┘

Every Loop Iteration:

    ┌─────────────────┐
    │  RC Link OK?    │
    └────────┬────────┘
             │
        ┌────┴────┐
        │   YES   │         NO
        │         │         │
        ▼         │         ▼
    Continue      │   ┌──────────────┐
                  │   │  FAILSAFE!   │
                  │   │  Disarm Now  │
                  │   └──────────────┘
                  │
                  ▼
    ┌─────────────────┐
    │  Armed Switch?  │
    └────────┬────────┘
             │
        ┌────┴────┐
        │   ON    │        OFF
        │         │         │
        ▼         │         ▼
    Continue      │   ┌──────────────┐
                  │   │  Disarm      │
                  │   │  Motors OFF  │
                  │   └──────────────┘
                  │
                  ▼
    ┌─────────────────┐
    │  Angle < 30°?   │
    └────────┬────────┘
             │
        ┌────┴────┐
        │   YES   │         NO
        │         │         │
        ▼         │         ▼
    Continue      │   ┌──────────────┐
                  │   │  Limit Angle │
                  │   │  to Max 30°  │
                  │   └──────────────┘
                  │
                  ▼
    ┌─────────────────┐
    │ Throttle < 65%? │
    └────────┬────────┘
             │
        ┌────┴────┐
        │   YES   │         NO
        │         │         │
        ▼         │         ▼
    Continue      │   ┌──────────────┐
                  │   │  Cap to 65%  │
                  │   └──────────────┘
                  │
                  ▼
    ┌─────────────────┐
    │  Update Motors  │
    └─────────────────┘
```

---

**Last Updated**: 2025-11-30
