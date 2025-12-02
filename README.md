# Yildiz Drone Flight Controller

Professional quadcopter flight controller with NRF24L01+ wireless communication using TMRh20 library.

## Features

- ✅ **TMRh20 RF24 Library** - Industry-standard NRF24L01 driver
- ✅ **Auto-ACK Enabled** - Reliable packet delivery with automatic retransmission
- ✅ **250kbps Data Rate** - Best range and reliability
- ✅ **PID Flight Control** - Stable flight with tunable PID parameters
- ✅ **MPU6050 IMU** - 6-axis gyro + accelerometer for stabilization
- ✅ **Complementary Filter** - Smooth angle estimation
- ✅ **Failsafe** - Automatic motor cutoff on signal loss
- ✅ **Dual Flight Modes** - Stabilize (angle) and Acro (rate)
- ✅ **Real-time Telemetry** - Battery, signal strength, status
- ✅ **250Hz Loop Rate** - Fast and responsive flight control

## Why ACK is Enabled

For drone flight control, **AUTO-ACK is ESSENTIAL**:

| Feature | Benefit |
|---------|---------|
| **Packet Verification** | Ensures every command reaches the drone |
| **Auto-Retransmission** | Lost packets automatically resent (5 retries) |
| **Bidirectional Telemetry** | ACK payload sends battery/status back |
| **Safety Critical** | No lost control commands = safer flying |
| **Minimal Latency** | Only ~1-2ms overhead, negligible for flight |

**Without ACK**: Packets can be lost silently, leading to erratic behavior or crashes!

## Hardware Requirements

### Transmitter (Controller)
- Arduino Nano/Uno
- NRF24L01+ module with antenna
- 2x Analog joysticks
- 2x Switches (arm, mode)
- **External 3.3V regulator** with capacitors (100uF + 0.1uF)

### Receiver (Drone)
- Arduino Nano/Uno/Mega (Mega recommended)
- NRF24L01+ module with antenna
- MPU6050 IMU
- 4x ESCs (20A or higher)
- 4x Brushless motors (1000-2000KV)
- 4x Propellers (matching motor size)
- LiPo battery (3S or 4S)
- Battery voltage divider for monitoring
- Drone frame (250-450mm)

## Wiring Diagrams

### NRF24L01 Connections (Both TX and RX)
```
NRF24L01    Arduino
========    =======
CE      ->  D9
CSN     ->  D10
MOSI    ->  D11
MISO    ->  D12
SCK     ->  D13
VCC     ->  3.3V (MUST use external regulator!)
GND     ->  GND

CRITICAL: NRF24L01 needs clean 3.3V power!
- Use AMS1117-3.3V or similar regulator
- Add 100uF electrolytic + 0.1uF ceramic capacitors
- Poor power = unreliable communication
```

### MPU6050 Connections (Receiver only)
```
MPU6050     Arduino
========    =======
VCC     ->  5V
GND     ->  GND
SCL     ->  A5 (Uno) or D21 (Mega)
SDA     ->  A4 (Uno) or D20 (Mega)
```

### Motor Connections (Receiver)
```
Motor       ESC         Arduino Pin
=====       ===         ===========
M1 (FL)  -> ESC1    ->  D3  (CW rotation)
M2 (FR)  -> ESC2    ->  D5  (CCW rotation)
M3 (BR)  -> ESC3    ->  D6  (CW rotation)
M4 (BL)  -> ESC4    ->  D11 (CCW rotation)

Quadcopter X Layout:
      FRONT
    M1     M2
       \ /
       / \
    M4     M3
      BACK
```

### Transmitter Joystick Connections
```
Control     Arduino Pin
=======     ===========
Throttle -> A0 (Left stick vertical)
Yaw      -> A1 (Left stick horizontal)
Pitch    -> A2 (Right stick vertical)
Roll     -> A3 (Right stick horizontal)
Arm SW   -> D7 (with pullup)
Mode SW  -> D8 (with pullup)
```

## Installation

### 1. Install Arduino IDE
Download from: https://www.arduino.cc/en/software

### 2. Install Required Libraries

Open Arduino IDE -> Tools -> Manage Libraries, install:

```
1. RF24 by TMRh20 (version 1.4.8 or later)
2. Servo (built-in, no installation needed)
```

**Library Links:**
- RF24: https://github.com/nRF24/RF24
- Documentation: https://nrf24.github.io/RF24/

### 3. Upload Code

**Transmitter:**
1. Open `transmitter/transmitter.ino`
2. Select board: Tools -> Board -> Arduino Nano/Uno
3. Select port: Tools -> Port -> (your port)
4. Upload

**Receiver:**
1. Open `receiver/receiver.ino`
2. Select board: Tools -> Board -> Arduino Nano/Uno/Mega
3. Select port: Tools -> Port -> (your port)
4. Upload

## Configuration

### Radio Settings (Must Match!)

Both transmitter and receiver use these settings:

```cpp
PA Level:     RF24_PA_MAX      // Maximum power
Data Rate:    RF24_250KBPS    // Best range
Channel:      108              // 2.508 GHz
Retries:      3 x 250us delay, 5 retries
Auto ACK:     ENABLED          // Critical!
CRC:          16-bit           // Error detection
```

### PID Tuning (Receiver)

**Located in receiver.ino around line 100:**

```cpp
// TUNE THESE FOR YOUR DRONE!
float pidRollP = 1.3;
float pidRollI = 0.04;
float pidRollD = 18.0;

float pidPitchP = 1.3;
float pidPitchI = 0.04;
float pidPitchD = 18.0;

float pidYawP = 2.0;
float pidYawI = 0.02;
float pidYawD = 0.0;
```

**Tuning Process:**
1. Start with P=1.0, I=0, D=0
2. Increase P until oscillations start, then reduce by 20%
3. Increase D to dampen oscillations
4. Add small I to eliminate steady-state error
5. Test in small increments!

**Tuning Tips:**
- P too high: Oscillations
- P too low: Sluggish response
- I too high: Slow oscillations
- D too high: Jittery, noisy
- Start conservative, tune on bench before flying!

## First Flight Checklist

### Pre-Flight

- [ ] All wiring double-checked
- [ ] Propellers OFF for testing!
- [ ] Battery fully charged
- [ ] ESCs calibrated (send 1000us for 3 seconds)
- [ ] Gyro calibration done (drone on level surface)
- [ ] Transmitter shows good connection
- [ ] Motor direction tested (props off!)
- [ ] Failsafe tested (turn off TX, motors should stop)
- [ ] PID gains conservative (start low)

### Motor Direction Test

With propellers OFF:
1. Arm the drone (throttle low, arm switch on)
2. Slowly increase throttle
3. Check motor direction with finger (CAREFUL!)
   - M1 (FL): Clockwise
   - M2 (FR): Counter-clockwise
   - M3 (BR): Clockwise
   - M4 (BL): Counter-clockwise
4. Wrong direction? Swap any 2 motor wires on ESC

### First Hover Test

1. **Test in open area** away from people/objects
2. Start with low PID gains
3. Arm drone on level ground
4. Slowly increase throttle until it lifts (~1550-1600)
5. Hover at 1-2 feet for 10 seconds
6. If stable, continue testing
7. If oscillating, reduce P gain
8. If drifting, check motor/prop balance

## Flight Modes

### Stabilize Mode (Mode Switch OFF)
- **Self-leveling** - Returns to horizontal when sticks centered
- **Angle control** - Stick position = tilt angle (max ±30°)
- **Best for beginners**
- Easier to fly, more stable

### Acro Mode (Mode Switch ON)
- **Rate control** - Stick position = rotation speed
- **No self-leveling** - Manual control required
- **For experienced pilots**
- Allows flips and rolls

## Troubleshooting

### No Radio Connection

**Symptoms:** "NO CONNECTION TO DRONE" message

**Solutions:**
1. Check NRF24L01 wiring (especially CE/CSN)
2. Verify 3.3V power with multimeter
3. Add capacitors (100uF + 0.1uF) to NRF24L01 VCC
4. Check address matches in both codes: `"DRON1"`
5. Verify channel matches: `108`
6. Try different NRF24L01 module (can be faulty)
7. Reduce distance (start with 1 meter)

### Poor Connection / Packet Loss

**Solutions:**
1. Improve NRF24L01 power supply
2. Add shielding to reduce interference
3. Move away from WiFi routers
4. Use modules with external antenna
5. Check battery voltage (low voltage = weak signal)

### Oscillations During Flight

**Symptoms:** Rapid shaking/bouncing

**Solutions:**
1. Reduce P gain by 20-30%
2. Reduce D gain if very jittery
3. Check propeller balance
4. Verify motor mounting is tight
5. Check frame for flex
6. Reduce I gain if slow oscillations

### Drone Drifts

**Symptoms:** Moves in one direction at hover

**Solutions:**
1. Recalibrate gyro on level surface
2. Check motor thrust balance
3. Check propeller condition (damaged?)
4. Verify frame is not bent
5. Add I term to eliminate steady-state error
6. Check motor timing (ESC settings)

### Motors Don't Spin

**Solutions:**
1. Check arming: Throttle must be LOW + arm switch ON
2. Verify ESC calibration (1000us = off)
3. Check failsafe: Must have radio connection
4. Check motor wiring
5. Verify ESC power connection
6. Check Serial Monitor for status

### Failsafe Activates Randomly

**Solutions:**
1. Improve NRF24L01 power supply (most common!)
2. Add larger capacitors (470uF + 10uF)
3. Use separate regulator for NRF24L01
4. Check for loose connections
5. Reduce interference sources
6. Use better quality NRF24L01 modules

## Safety Guidelines

⚠️ **CRITICAL SAFETY RULES** ⚠️

1. **NEVER** test with propellers on until everything is verified
2. **ALWAYS** test failsafe before first flight
3. **NEVER** fly near people or animals
4. **ALWAYS** remove battery when working on electronics
5. **NEVER** fly indoors until experienced
6. **ALWAYS** have fire extinguisher ready (LiPo safety)
7. **NEVER** fly over crowds or property
8. **ALWAYS** follow local drone regulations
9. **NEVER** fly beyond visual range
10. **ALWAYS** have a kill switch ready (arm switch)

## Performance Specifications

| Specification | Value |
|--------------|-------|
| Control Loop Rate | 250 Hz (4ms) |
| Radio Update Rate | 50 Hz (20ms) |
| Radio Range | 100-300m (depends on environment) |
| Latency (RTT) | 2-5ms typical |
| Max Tilt Angle | ±30° (stabilize mode) |
| Max Rotation Rate | ±180°/s (acro mode) |
| Packet Loss | <0.1% (with good setup) |

## Communication Protocol

### Control Data (TX -> RX)
```
throttle: 1000-2000 (PWM microseconds)
yaw:      1000-2000 (1500 = center)
pitch:    1000-2000 (1500 = center)
roll:     1000-2000 (1500 = center)
arm:      0 or 1
mode:     0=stabilize, 1=acro
timestamp: milliseconds
```

### Telemetry Data (RX -> TX via ACK payload)
```
battery:   float (voltage)
rssi:      int16_t (signal strength)
status:    0=disarmed, 1=armed, 2=failsafe
timestamp: milliseconds
```

## Advanced Tuning

### Rate Scaling
Adjust max angles/rates in `calculatePID()`:
```cpp
// Max angle in stabilize mode (default ±30°)
pidRollSetpoint = (controlData.roll - 1500) * 30.0 / 500.0;

// Max rate in acro mode (default ±180°/s)
pidRollSetpoint = (controlData.roll - 1500) * 180.0 / 500.0;
```

### Deadband Adjustment
Transmitter joystick deadband (center ±10):
```cpp
controlData.yaw = applyDeadband(controlData.yaw, 1500, 10);
```

### Complementary Filter
Adjust sensor fusion in `calculateAngles()`:
```cpp
// 98% gyro, 2% accelerometer (default)
angleX = angleX * 0.98 + accelAngleX * 0.02;
```

## License

Open source - Use at your own risk!

## Disclaimer

⚠️ **Flying drones can be dangerous!**

- This is an educational project
- No warranty or guarantee of safety
- User assumes all risks
- Follow all local laws and regulations
- Get proper training before flying
- Consider insurance

**BUILD AND FLY AT YOUR OWN RISK!**

## Credits

- TMRh20 for excellent RF24 library
- Arduino community for resources and support

---

**Good luck and fly safe! 🚁**
