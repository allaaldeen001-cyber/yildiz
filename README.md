# Quadcopter Drone - NRF24L01 Flight Control System

Complete Arduino-based quadcopter drone flight controller using **TMRh20 RF24 Library** with optimized NRF24L01 communication.

## 📡 Why NO ACK Mode for Drone Control?

For drone communication, we use **Auto-Acknowledgment DISABLED (NO ACK mode)**:

| Feature | With ACK | NO ACK (Used Here) |
|---------|----------|-------------------|
| Latency | Higher (waits for ACK) | **Lower (instant send)** |
| Reliability | Guaranteed delivery | Best-effort |
| Timing | Variable (retries) | **Consistent** |
| Use Case | Data logging, telemetry | **Real-time control** |

**Why NO ACK is better for drones:**
1. ✅ **Consistent timing** - No random delays from retries
2. ✅ **Lower latency** - Commands arrive faster
3. ✅ **Higher throughput** - More packets per second
4. ✅ **Fresh data priority** - New commands > old guaranteed commands
5. ✅ **Failsafe triggers faster** - Signal loss detected immediately

## 📁 Project Structure

```
├── common/
│   └── RF24_Config.h      # Shared radio configuration
├── transmitter/
│   └── transmitter.ino    # Remote controller code
├── receiver/
│   └── receiver.ino       # Flight controller code
└── README.md
```

## 🔧 Hardware Requirements

### Transmitter (Remote Controller)
- Arduino Nano/Uno
- NRF24L01+ module (PA+LNA version recommended)
- 2x Joysticks (PS2 style or similar)
- 2x Switches or potentiometers for AUX channels
- 10-100μF capacitor (for NRF24L01 power stability)

### Receiver (Drone)
- Arduino Nano/Uno/Pro Mini
- NRF24L01+ module (PA+LNA version recommended)
- MPU6050 IMU (Gyroscope + Accelerometer)
- 4x ESCs (Electronic Speed Controllers)
- 4x Brushless motors
- Drone frame (X configuration)
- LiPo Battery (3S 11.1V or 4S 14.8V)
- 10-100μF capacitor (for NRF24L01 power stability)

## 🔌 Wiring Diagrams

### NRF24L01 to Arduino (Both TX and RX)

```
NRF24L01        Arduino
┌─────────┐
│ VCC  ●──┼───── 3.3V (NOT 5V!)
│ GND  ●──┼───── GND
│ CE   ●──┼───── Pin 9
│ CSN  ●──┼───── Pin 10
│ SCK  ●──┼───── Pin 13
│ MOSI ●──┼───── Pin 11
│ MISO ●──┼───── Pin 12
│ IRQ  ●  │      (Not connected)
└─────────┘

⚠️ IMPORTANT: Add 10-100μF capacitor between VCC and GND!
```

### Transmitter Connections

```
Component           Arduino Pin
─────────────────────────────────
Throttle Joystick   A0 (vertical)
Yaw Joystick        A1 (horizontal)
Pitch Joystick      A2 (vertical)
Roll Joystick       A3 (horizontal)
AUX1 (Arm switch)   A4
AUX2 (Mode switch)  A5
```

### Receiver (Drone) Connections

```
MPU6050             Arduino
─────────────────────────────────
VCC                 5V
GND                 GND
SDA                 A4
SCL                 A5

ESCs                Arduino Pin
─────────────────────────────────
Motor 1 (FL, CCW)   Pin 3 (PWM)
Motor 2 (FR, CW)    Pin 5 (PWM)
Motor 3 (BL, CW)    Pin 6 (PWM)
Motor 4 (BR, CCW)   A0 (digital)
```

### Motor Layout (X Configuration - View from TOP)

```
          FRONT
     M1 (CCW)   M2 (CW)
         ╲     ╱
          ╲   ╱
           ╲ ╱
           ─┼─
           ╱ ╲
          ╱   ╲
         ╱     ╲
     M3 (CW)   M4 (CCW)
          BACK

CCW = Counter-Clockwise
CW  = Clockwise
```

## 📚 Required Libraries

Install these libraries in Arduino IDE (Sketch → Include Library → Manage Libraries):

1. **RF24 by TMRh20** - NRF24L01 communication
2. **Wire** - I2C for MPU6050 (built-in)
3. **Servo** - ESC control (built-in)

## 🚀 Setup Instructions

### Step 1: Install Libraries
1. Open Arduino IDE
2. Go to Sketch → Include Library → Manage Libraries
3. Search for "RF24" and install "RF24 by TMRh20"

### Step 2: Upload Transmitter Code
1. Connect Arduino (transmitter) to computer
2. Open `transmitter/transmitter.ino`
3. Select correct board and port
4. Upload

### Step 3: Upload Receiver Code
1. Connect Arduino (receiver/drone) to computer
2. **REMOVE PROPELLERS!**
3. Open `receiver/receiver.ino`
4. Select correct board and port
5. Upload

### Step 4: Calibrate Joysticks
1. Open Serial Monitor (115200 baud)
2. Move joysticks to all positions
3. Note the min/max/center values
4. Update values in `transmitter.ino` if needed

### Step 5: Calibrate Gyro
1. Place drone on a flat, level surface
2. Power on - gyro calibrates automatically
3. **DO NOT MOVE THE DRONE during calibration!**

### Step 6: ESC Calibration (One-time)
1. Disconnect battery from drone
2. Set transmitter throttle to maximum
3. Connect battery - ESCs will beep
4. Lower throttle to minimum - ESCs will confirm with beeps
5. Calibration complete

## 🎮 Controls

| Control | Function |
|---------|----------|
| Left Stick Up/Down | Throttle |
| Left Stick Left/Right | Yaw |
| Right Stick Up/Down | Pitch (forward/backward) |
| Right Stick Left/Right | Roll (left/right) |
| AUX1 Switch | Arm/Disarm (>50% = Armed) |
| AUX2 Switch | Flight Mode (<50% = Angle, >50% = Rate) |

### Arming Procedure
1. Throttle to minimum
2. AUX1 switch to ON position
3. Drone will arm (motors start spinning at idle)

### Disarming
1. AUX1 switch to OFF position
2. Motors stop immediately

## ⚙️ PID Tuning Guide

Default PID values are a starting point. Tune for your drone:

```cpp
// In receiver.ino - adjust these values

// Roll PID
float rollKp = 1.3;    // Increase if drone is sluggish
float rollKi = 0.04;   // Increase if drone drifts
float rollKd = 18.0;   // Increase if oscillating

// Pitch PID (usually same as roll)
float pitchKp = 1.3;
float pitchKi = 0.04;
float pitchKd = 18.0;

// Yaw PID
float yawKp = 4.0;     // Yaw responsiveness
float yawKi = 0.02;    // Yaw drift correction
float yawKd = 0.0;     // Usually 0 for yaw
```

### Tuning Process
1. **Start with P only** (set I and D to 0)
2. Increase P until oscillations appear
3. Reduce P by 20%
4. **Add D** to dampen oscillations
5. **Add small I** to eliminate steady-state error

## ⚠️ Safety Features

### Failsafe System
- **Signal Lost**: Throttle reduces gradually
- **Auto-Disarm**: After 2 seconds at zero throttle
- **Arm Safety**: Cannot arm if throttle is not at minimum
- **Checksum Validation**: Corrupted packets are ignored

### Signal Loss Timeout
Configurable in `RF24_Config.h`:
```cpp
#define SIGNAL_TIMEOUT_MS  200  // 200ms = failsafe trigger
```

## 🔧 Troubleshooting

### NRF24L01 Not Working
1. ✅ Check VCC is 3.3V (NOT 5V!)
2. ✅ Add 10-100μF capacitor between VCC and GND
3. ✅ Check all SPI connections
4. ✅ Try a different NRF24L01 module
5. ✅ Reduce PA level to RF24_PA_LOW for testing

### No Signal / Constant Failsafe
1. ✅ Ensure both TX and RX have same channel (RF_CHANNEL)
2. ✅ Ensure both have same address (RADIO_ADDRESS)
3. ✅ Ensure both have same data rate (RF_DATA_RATE)
4. ✅ Ensure both have Auto-ACK disabled

### Drone Flips on Takeoff
1. ✅ Check motor directions match layout diagram
2. ✅ Verify propeller rotation (CCW/CW)
3. ✅ Check motor number assignments in code
4. ✅ Calibrate ESCs

### Drone Drifts
1. ✅ Calibrate accelerometer offsets
2. ✅ Increase I gain (slowly)
3. ✅ Check for bent motor mounts
4. ✅ Balance propellers

### Oscillations
1. ✅ Reduce P gain
2. ✅ Increase D gain
3. ✅ Check for loose parts
4. ✅ Verify loop timing is stable

## 📊 Technical Specifications

| Parameter | Value |
|-----------|-------|
| Radio Frequency | 2.4GHz ISM Band |
| Channel | 100 (configurable) |
| Data Rate | 2Mbps |
| TX Power | 0dBm (PA_MAX) |
| TX Frequency | 100Hz |
| Control Loop | 250Hz |
| Signal Timeout | 200ms |
| Packet Size | 12 bytes |

## 📜 License

This project is open source. Use at your own risk.

## ⚠️ Disclaimer

Flying drones can be dangerous. Always:
- **REMOVE PROPELLERS** when testing/configuring
- Fly in open areas away from people
- Follow local drone regulations
- Use protective gear
- Never fly over crowds
- Maintain visual line of sight
