# Professional Quadcopter Flight Control System

## 🚁 Complete Arduino-Based Drone System with Advanced Flight Control

A production-quality flight controller and remote control system designed for Arduino Nano, featuring professional cascade PID control, altitude hold, Mahony AHRS filter, and comprehensive safety systems.

---

## 📋 Table of Contents

- [System Overview](#system-overview)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configurations](#pin-configurations)
- [System Architecture](#system-architecture)
- [Software Features](#software-features)
- [Installation & Setup](#installation--setup)
- [Operation Workflow](#operation-workflow)
- [Control Mapping](#control-mapping)
- [Safety Features](#safety-features)
- [Communication Protocol](#communication-protocol)
- [Troubleshooting](#troubleshooting)

---

## 🔷 System Overview

This system consists of two main components:

### **Flight Controller (FC)**
- Attitude estimation using Mahony AHRS filter
- Cascade PID control (Rate + Angle loops)
- 1D Kalman filter for altitude estimation
- Altitude hold mode with smooth transitions
- Professional safety systems and failsafe
- Real-time telemetry transmission

### **Remote Controller (RC)**
- Dual joystick control (4 channels)
- Arm/disarm kill switch
- Altitude hold toggle
- Calibration triggers
- Real-time status display
- Robust NRF24L01 communication

---

## 🔧 Hardware Requirements

### Flight Controller Board

| Component | Model | Connection |
|-----------|-------|------------|
| **Microcontroller** | Arduino Nano | - |
| **IMU** | MPU6050 | I2C (A4=SDA, A5=SCL), INT→D2 |
| **Barometer** | MS5611 | I2C (A4=SDA, A5=SCL) |
| **Radio** | NRF24L01 PA+LNA | CE→D4, CSN→D10, SPI |
| **Buzzer** | Active Buzzer | D8 |
| **Status LED** | LED + 220Ω Resistor | D7 |
| **ESCs** | 4x BLHeli ESC | FL→D3, FR→D5, RR→D6, RL→D9 |
| **Power** | 5V BEC | 5V + GND to Nano |

### Remote Controller Board

| Component | Model | Connection |
|-----------|-------|------------|
| **Microcontroller** | Arduino Nano | - |
| **Radio** | NRF24L01 PA+LNA | CE→D9, CSN→D10, SPI |
| **Left Joystick** | 2-Axis Analog | V→A0 (Throttle), H→A1 (Yaw) |
| **Right Joystick** | 2-Axis Analog | V→A2 (Pitch), H→A3 (Roll) |
| **Button 1** | Push Button | D4 (Calibration) |
| **Button 2** | Push Button | D5 (ESC Cal/Motor Test) |
| **Switch 1** | Toggle Switch | D2 (Altitude Hold) |
| **Switch 2** | Toggle Switch | D3 (ARM/DISARM) |
| **Power** | USB or 9V Battery | Vin/5V + GND |

### Additional Components

- **Motors**: 4x brushless motors (e.g., 2204-2300KV)
- **Propellers**: 2x CW + 2x CCW (5-6 inch)
- **Battery**: 3S LiPo (11.1V, 1500-2200mAh)
- **Frame**: 250-300mm quadcopter frame

---

## 📌 Pin Configurations

### Flight Controller (FC) Pinout

```
Arduino Nano Pin Configuration:

Digital Pins:
  D2  ← MPU6050 INT (interrupt)
  D3  → ESC Front Left (PWM)
  D4  → NRF24L01 CE
  D5  → ESC Front Right (PWM)
  D6  → ESC Rear Right (PWM)
  D7  → Status LED
  D8  → Buzzer
  D9  → ESC Rear Left (PWM)
  D10 → NRF24L01 CSN
  D11 → NRF24L01 MOSI (SPI)
  D12 → NRF24L01 MISO (SPI)
  D13 → NRF24L01 SCK (SPI)

Analog Pins (I2C):
  A4 (SDA) ↔ MPU6050 SDA, MS5611 SDA
  A5 (SCL) ↔ MPU6050 SCL, MS5611 SCL

Power:
  5V  ← BEC 5V Output
  GND ← Common Ground
```

### Remote Controller (RC) Pinout

```
Arduino Nano Pin Configuration:

Digital Pins:
  D2  ← Switch 1 (Altitude Hold)
  D3  ← Switch 2 (ARM/DISARM)
  D4  ← Button 1 (Calibration)
  D5  ← Button 2 (ESC Cal/Motor Test)
  D9  → NRF24L01 CE
  D10 → NRF24L01 CSN
  D11 → NRF24L01 MOSI (SPI)
  D12 → NRF24L01 MISO (SPI)
  D13 → NRF24L01 SCK (SPI)

Analog Pins:
  A0 ← Left Joystick V (Throttle)
  A1 ← Left Joystick H (Yaw)
  A2 ← Right Joystick V (Pitch)
  A3 ← Right Joystick H (Roll)

Power:
  5V/Vin ← Power Source
  GND    ← Common Ground
```

**Note**: All switches and buttons use internal pull-up resistors (active LOW).

---

## 🏗️ System Architecture

### Control Loop Hierarchy

```
┌─────────────────────────────────────────────────────────────────┐
│                     FLIGHT CONTROLLER LOOPS                      │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ RATE LOOP (250 Hz - Inner Loop)                         │   │
│  │  • Read MPU6050 (gyro + accel)                          │   │
│  │  • Mahony AHRS filter → attitude quaternion             │   │
│  │  • Rate PID: gyro rates → motor commands                │   │
│  │  • Motor mixing (X configuration)                       │   │
│  │  • Safety checks + motor output                         │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              ↑                                    │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ ANGLE LOOP (100 Hz - Outer Loop)                        │   │
│  │  • Angle PID: target angles → desired rates             │   │
│  │  • Feeds into rate loop as setpoints                    │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              ↑                                    │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ ALTITUDE LOOP (25 Hz)                                   │   │
│  │  • Read MS5611 barometer                                │   │
│  │  • 1D Kalman filter (height + velocity)                 │   │
│  │  • Altitude PID → climb rate → throttle adjustment      │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                   │
├─────────────────────────────────────────────────────────────────┤
│  NRF COMMUNICATION (50 Hz TX, continuous RX)                     │
│  • Receive RC commands                                           │
│  • Transmit telemetry (attitude, altitude, status)              │
│  • Failsafe on link loss                                         │
└─────────────────────────────────────────────────────────────────┘
```

### Mahony AHRS Filter

The Mahony filter is a computationally efficient quaternion-based attitude estimator that fuses gyroscope and accelerometer data:

**Algorithm**:
1. **Predict**: Integrate gyroscope rates to update quaternion
2. **Correct**: Use accelerometer to estimate gravity direction
3. **Error Calculation**: Cross product between estimated and measured gravity
4. **Feedback**: Apply proportional + integral correction to gyro rates

**Advantages**:
- Low computational cost (suitable for Arduino Nano)
- No magnetometer required
- Stable attitude estimation without gimbal lock
- Configurable feedback gains (Kp = 2.0, Ki = 0.01)

### 1D Kalman Filter (Altitude Estimation)

Fuses barometric altitude with vertical acceleration for smooth altitude estimation:

**State Vector**: `[height, vertical_velocity]`

**Process Model**:
```
height_k = height_{k-1} + velocity_{k-1} * dt
velocity_k = velocity_{k-1} + accel_z * dt
```

**Measurement**: Barometric altitude (direct height measurement)

**Benefits**:
- Filters barometer noise
- Reduces lag using accelerometer
- Provides velocity estimate for climb rate control

### Cascade PID Control

#### Rate Loop (Inner Loop - 250Hz)
- **Input**: Desired angular rates (rad/s)
- **Feedback**: Gyroscope rates
- **Output**: Motor commands
- **Purpose**: Fast stabilization, direct control authority

#### Angle Loop (Outer Loop - 100Hz)
- **Input**: Desired angles (degrees) from RC sticks
- **Feedback**: Attitude from AHRS
- **Output**: Desired rates for inner loop
- **Purpose**: Smooth angle tracking, self-leveling

#### Altitude Loop (25Hz)
- **Input**: Target altitude (meters)
- **Feedback**: Kalman-filtered altitude
- **Output**: Desired climb rate → throttle adjustment
- **Purpose**: Maintain constant height, altitude hold mode

---

## 🚀 Software Features

### Flight Controller Features

✅ **Mahony AHRS Filter** - Quaternion-based attitude estimation  
✅ **Cascade PID Control** - Rate + Angle loops for stability  
✅ **1D Kalman Filter** - Optimal altitude estimation  
✅ **Altitude Hold Mode** - Hands-free height maintenance  
✅ **Professional Safety Systems** - Failsafe, kill switch, link timeout  
✅ **IMU Calibration** - Automatic gyro/accel offset calibration  
✅ **ESC Calibration** - Full-range ESC calibration routine  
✅ **Motor Test Sequence** - Individual motor verification  
✅ **Real-time Telemetry** - Attitude, altitude, status feedback  
✅ **Optimized Timing** - Fixed-rate loops using `micros()`  

### Remote Controller Features

✅ **Dual Joystick Control** - 4-channel control (throttle, yaw, pitch, roll)  
✅ **Exponential Filtering** - Smooth joystick readings  
✅ **Automatic Calibration** - Center-point detection  
✅ **Deadzone Handling** - Eliminates stick drift  
✅ **Button Debouncing** - Reliable button presses  
✅ **Link Monitoring** - Real-time connection status  
✅ **Serial Display** - Rich ANSI-formatted status screen  
✅ **Standalone Operation** - Works without serial monitor  

---

## 📥 Installation & Setup

### 1. Software Requirements

- **Arduino IDE** 1.8.19 or newer (or Arduino IDE 2.x)
- **Libraries**:
  - `RF24` by TMRh20 (v1.4.7+)
  - `Wire` (built-in)
  - `SPI` (built-in)

**Install RF24 Library**:
```
Arduino IDE → Tools → Manage Libraries → Search "RF24" → Install
```

### 2. Hardware Assembly

#### Flight Controller:
1. Mount MPU6050 and MS5611 on the same I2C bus
2. Connect NRF24L01 to SPI pins (use adapter with 3.3V regulator)
3. Connect 4 ESCs to designated PWM pins
4. Add buzzer and LED with appropriate resistors
5. Power from 5V BEC (isolated from ESC power rail)

#### Remote Controller:
1. Connect 2 analog joysticks to A0-A3
2. Wire buttons and switches with pull-up configuration
3. Connect NRF24L01 to SPI pins
4. Power via USB or 9V battery with regulator

### 3. Upload Firmware

**Flight Controller**:
```
1. Open FlightController_FC.ino in Arduino IDE
2. Select board: Tools → Board → Arduino Nano
3. Select processor: Tools → Processor → ATmega328P (Old Bootloader)
4. Select port: Tools → Port → [Your COM port]
5. Click Upload
```

**Remote Controller**:
```
1. Open RemoteController_RC.ino in Arduino IDE
2. Select board: Tools → Board → Arduino Nano
3. Select processor: Tools → Processor → ATmega328P (Old Bootloader)
4. Select port: Tools → Port → [Your COM port]
5. Click Upload
```

### 4. Pre-Flight Checks

✔️ **Verify Wiring**: Double-check all connections  
✔️ **Motor Direction**: Ensure motors spin in correct direction  
✔️ **Propeller Orientation**: CW props on CW motors, CCW props on CCW motors  
✔️ **Battery Voltage**: Fully charged 3S LiPo (12.6V)  
✔️ **Secure Mounting**: All components firmly attached  
✔️ **Clear Flight Area**: No obstacles, adequate space  

---

## 🎮 Operation Workflow

### Standard Flight Sequence

```
1. Power ON Remote Controller (RC)
   └─ Wait for NRF initialization (2 seconds)

2. Power ON Flight Controller (FC)
   └─ Listen for 2 beeps (startup complete)
   └─ LED (D7) should blink indicating NRF link

3. Verify Link Status
   └─ RC serial monitor shows "CONNECTED"
   └─ FC LED blinks steadily

4. **CRITICAL: Set ARM Switch (SW_2) to OFF/DISARM**
   └─ Ensures motors don't spin during calibration

5. Press Button_1 (Calibration)
   └─ Keep drone LEVEL and STATIONARY
   └─ Wait 4-5 seconds
   └─ Success: 2 short beeps
   └─ Failure: 1 long beep (7 seconds) → repeat step 5

6. Set Altitude Hold Switch (SW_1) to OFF

7. Press Button_2 (ESC Calibration + Motor Test)
   └─ ESC calibration sequence (high → low throttle)
   └─ Motor test: FL → FR → RR → RL
   └─ Verify correct motor spin directions
   └─ Listen for 3 confirmation beeps

8. ARM the Drone
   └─ Set SW_2 (ARM switch) to ON
   └─ Throttle stick to minimum position

9. Takeoff
   └─ Slowly increase throttle
   └─ Use right stick for pitch/roll control
   └─ Use left stick (horizontal) for yaw

10. (Optional) Enable Altitude Hold
    └─ Stabilize at desired height
    └─ Toggle SW_1 to ON
    └─ Drone maintains current altitude
    └─ Use pitch/roll/yaw normally

11. Landing
    └─ Disable altitude hold (SW_1 OFF)
    └─ Reduce throttle smoothly
    └─ Set SW_2 to OFF immediately after touchdown
```

---

## 🕹️ Control Mapping

### Joystick Controls

| Control | Stick | Direction | Function |
|---------|-------|-----------|----------|
| **Throttle** | Left V | Up/Down | Increase/Decrease altitude |
| **Yaw** | Left H | Left/Right | Rotate CCW/CW |
| **Pitch** | Right V | Up/Down | Forward/Backward |
| **Roll** | Right H | Left/Right | Tilt left/right |

### Switches

| Switch | Pin | State | Function |
|--------|-----|-------|----------|
| **SW_2** | D3 | ON (LOW) | ARM - Motors enabled |
| **SW_2** | D3 | OFF (HIGH) | DISARM - Kill motors immediately |
| **SW_1** | D2 | ON (LOW) | Altitude Hold enabled |
| **SW_1** | D2 | OFF (HIGH) | Altitude Hold disabled (manual throttle) |

### Buttons

| Button | Pin | Function |
|--------|-----|----------|
| **Button_1** | D4 | Trigger IMU calibration (gyro + accel) |
| **Button_2** | D5 | ESC calibration + motor test sequence |

**Note**: All switches/buttons are active LOW (pressed = LOW, released = HIGH)

---

## 🛡️ Safety Features

### Automatic Safety Systems

1. **NRF Link Timeout (500ms)**
   - Monitors last received packet timestamp
   - Automatically disarms and cuts motors if link lost
   - LED turns OFF to indicate loss of connection

2. **Arm/Disarm Kill Switch (SW_2)**
   - Hardware-level motor kill
   - Instantly disarms on switch toggle
   - Must be OFF during calibration

3. **Calibration Requirement**
   - Motors blocked until successful IMU calibration
   - Prevents flight with incorrect sensor offsets

4. **Maximum Tilt Angle (±30°)**
   - Software limit prevents extreme angles
   - Protects against loss of control

5. **Throttle Cap (65%)**
   - Limits maximum thrust to prevent over-power
   - Reserves headroom for control authority

6. **PID Integrator Anti-Windup**
   - Prevents integrator saturation
   - Maintains controllability during aggressive maneuvers

7. **Smooth Motor Startup**
   - Gradual throttle application
   - Prevents sudden torque spikes

### Emergency Procedures

**Loss of Control**:
```
1. Toggle SW_2 to DISARM immediately
2. All motors cut instantly
```

**Link Loss**:
```
1. Automatic failsafe activates (500ms timeout)
2. Motors cut automatically
3. Re-establish link before re-arming
```

**Failed Calibration**:
```
1. Listen for 1 long beep (7 seconds)
2. Ensure drone is level and stationary
3. Press Button_1 again
4. Do NOT attempt flight until calibration succeeds
```

---

## 📡 Communication Protocol

### NRF24L01 Configuration

- **Channel**: 103 (2.503 GHz)
- **Data Rate**: 250 kbps (long range)
- **PA Level**: MAX (+20dBm with PA+LNA)
- **Auto-Retry**: 5 retries, 5x250µs delay
- **Packet Size**: 20 bytes (RC→FC), 18 bytes (FC→RC)

### RC → FC Data Structure

```cpp
struct RC_Data {
  uint16_t throttle;      // 1000-2000 (PWM-style)
  uint16_t yaw;           // 1000-2000
  uint16_t pitch;         // 1000-2000
  uint16_t roll;          // 1000-2000
  uint8_t  armed;         // 0=disarmed, 1=armed
  uint8_t  altHold;       // 0=off, 1=on
  uint8_t  button1;       // Calibration trigger
  uint8_t  button2;       // ESC calibration
  uint32_t timestamp;     // millis() for timeout detection
};
```

### FC → RC Telemetry Structure

```cpp
struct FC_Telemetry {
  float roll;             // degrees (-180 to +180)
  float pitch;            // degrees (-90 to +90)
  float yaw;              // degrees (-180 to +180)
  float altitude;         // meters (relative to base altitude)
  uint8_t calibrated;     // 0=not calibrated, 1=calibrated
  uint8_t linked;         // 0=no link, 1=linked
  uint16_t loopTime;      // microseconds (rate loop execution time)
};
```

### Communication Timing

- **RC → FC**: 50 Hz (every 20ms)
- **FC → RC**: 20 Hz (every 50ms)
- **Link Timeout**: 500ms (failsafe activation)

---

## 🔧 Troubleshooting

### Common Issues

#### 1. **NRF24L01 Not Working**

**Symptoms**: "NRF24L01 FAILED!" message, no link

**Solutions**:
- Check power: NRF24L01 requires 3.3V (NOT 5V!)
- Use adapter board with voltage regulator and capacitor
- Verify SPI wiring: MOSI(D11), MISO(D12), SCK(D13), CE, CSN
- Add 10µF capacitor across NRF24L01 power pins
- Keep wires short (< 10cm)
- Test with different NRF24L01 module (some are defective)

#### 2. **Calibration Always Fails**

**Symptoms**: 1 long beep (7 seconds) after Button_1 press

**Solutions**:
- Ensure drone is on stable, level surface
- Avoid touching/moving drone during calibration
- Check MPU6050 connection (I2C: SDA=A4, SCL=A5)
- Upload FC firmware with serial monitor open to see offsets
- If offsets are huge (>2.0 for accel, >0.1 for gyro), IMU may be faulty

#### 3. **Drone Drifts/Oscillates**

**Symptoms**: Unstable flight, oscillations, drifts in one direction

**Solutions**:
- Recalibrate IMU (Button_1)
- Reduce rate PID gains (see `TUNING_GUIDE.md`)
- Check propeller direction and orientation
- Verify motor mounting is secure (no vibrations)
- Ensure CG (center of gravity) is centered
- Check for bent propellers or damaged motors

#### 4. **Motors Don't Spin**

**Symptoms**: Armed, but motors don't respond to throttle

**Solutions**:
- Verify ESC calibration (Button_2)
- Check ESC BEC power to Arduino (5V + GND)
- Verify ESC signal wires: FL=D3, FR=D5, RR=D6, RL=D9
- Test ESCs individually with servo tester
- Ensure throttle stick is above minimum (>1100)
- Check motor/ESC connections

#### 5. **Altitude Hold Doesn't Work**

**Symptoms**: Altitude hold enabled, but drone climbs/descends

**Solutions**:
- Verify MS5611 connection (I2C: SDA=A4, SCL=A5)
- Check baseline altitude in serial monitor
- Tune altitude PID gains (see `TUNING_GUIDE.md`)
- Ensure drone has sufficient thrust margin (hover < 60% throttle)
- Check for barometer interference (keep away from propwash)

#### 6. **RC Serial Monitor Shows Garbage**

**Symptoms**: Random characters, unreadable text

**Solutions**:
- Set serial monitor baud rate to **115200**
- Select "Newline" or "Both NL & CR" in serial monitor
- If ANSI codes visible, terminal doesn't support them (ignore)

---

## 📊 Default PID Values

### Rate PIDs (Inner Loop)

```cpp
Roll Rate:  Kp = 1.5, Ki = 0.05, Kd = 0.01
Pitch Rate: Kp = 1.5, Ki = 0.05, Kd = 0.01
Yaw Rate:   Kp = 2.0, Ki = 0.1,  Kd = 0.0
```

### Angle PIDs (Outer Loop)

```cpp
Roll Angle:  Kp = 3.5, Ki = 0.0, Kd = 0.0
Pitch Angle: Kp = 3.5, Ki = 0.0, Kd = 0.0
```

### Altitude PIDs

```cpp
Altitude:    Kp = 2.0,  Ki = 0.5, Kd = 1.0  (Height → Velocity)
Climb Rate:  Kp = 30.0, Ki = 5.0, Kd = 5.0  (Velocity → Throttle)
```

**Note**: See `TUNING_GUIDE.md` for detailed tuning instructions.

---

## 📐 Motor Layout (X Configuration)

```
        FRONT
          ↑
    FL         FR
     ⟲   X    ⟳
    RL         RR
          ↓
        BACK

FL = Front Left  (CW rotation)  - D3
FR = Front Right (CCW rotation) - D5
RR = Rear Right  (CW rotation)  - D6
RL = Rear Left   (CCW rotation) - D9

Propeller Orientation:
- CW motors  → CW propellers (pusher)
- CCW motors → CCW propellers (pusher)
```

---

## 📚 Additional Resources

- **TUNING_GUIDE.md** - Comprehensive PID tuning instructions
- **TESTING_PROCEDURES.md** - Bench tests, calibration, fail-safe verification
- **SCHEMATICS.md** - Detailed wiring diagrams (coming soon)

---

## ⚠️ Important Warnings

1. **NEVER arm the drone with propellers attached until ESC calibration and motor test are complete**
2. **ALWAYS remove propellers during bench testing and calibration**
3. **Test in open area away from people and obstacles**
4. **Monitor battery voltage** - land immediately if voltage drops below 10.5V (3S LiPo)
5. **Inspect propellers before each flight** - replace if damaged
6. **Do NOT fly indoors** without propeller guards
7. **Keep fingers away from propellers** at all times
8. **Ensure SW_2 is OFF before connecting battery**

---

## 📜 License

This project is released under the MIT License. Use at your own risk.

**Disclaimer**: This firmware is provided "as-is" without warranty. The author is not responsible for any damage, injury, or loss resulting from the use of this software. Fly responsibly and follow local regulations.

---

## 🤝 Contributing

Contributions, bug reports, and feature requests are welcome! Please open an issue or pull request on GitHub.

---

## ✨ Author

Developed by an expert UAV Embedded Systems Engineer specializing in low-level flight control firmware for resource-constrained microcontrollers.

**Contact**: [Open GitHub Issue for Support]

---

**Happy Flying! 🚁**
