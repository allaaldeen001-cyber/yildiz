# Professional Arduino-Based Drone Flight Control System

## Overview

This is a complete, production-quality firmware implementation for a professional-grade quadcopter drone system consisting of:

1. **Flight Controller (FC)** - Arduino Nano-based flight control board
2. **Remote Controller (RC)** - Arduino Nano-based radio control transmitter

The system implements a cascade PID control architecture with AHRS attitude estimation, altitude hold, and comprehensive safety features.

---

## Hardware Architecture

### Flight Controller Board

| Component | Connection | Purpose |
|-----------|------------|---------|
| Arduino Nano | - | Main processor |
| NRF24L01 PA+LNA | CE→D4, CSN→D10 | Radio communication |
| MPU6050 | I2C, INT→D2 | IMU (gyro + accelerometer) |
| MS5611 | I2C | Barometric altimeter |
| Buzzer | D8 | Audio feedback |
| Status LED | D7 | Link status indicator |
| ESC FL | D3 | Front Left motor |
| ESC FR | D5 | Front Right motor |
| ESC RR | D6 | Rear Right motor |
| ESC RL | D9 | Rear Left motor |

### Remote Controller Board

| Component | Connection | Purpose |
|-----------|------------|---------|
| Arduino Nano | - | Main processor |
| NRF24L01 PA+LNA | CE→D9, CSN→D10 | Radio communication |
| Left Joystick V | A0 | Throttle control |
| Left Joystick H | A1 | Yaw control |
| Right Joystick V | A2 | Pitch control |
| Right Joystick H | A3 | Roll control |
| Button 1 | D4 | Calibration trigger |
| Button 2 | D5 | Motor ON + ESC calibration |
| SW1 | D2 | Altitude Hold toggle |
| SW2 | D3 | ARM/DISARM (Kill Switch) |

---

## System Architecture

### Control Loop Hierarchy

```
┌─────────────────────────────────────────┐
│         RC Commands (50 Hz)             │
│  Throttle, Roll, Pitch, Yaw, Switches   │
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│      Angle PID Loop (50 Hz)             │
│  Converts angle commands to rate SPs    │
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│      Rate PID Loop (400 Hz)             │
│  Controls angular rates (p, q, r)      │
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│      Motor Mixing (400 Hz)              │
│  Converts control outputs to PWM        │
└─────────────────────────────────────────┘
```

### Parallel Processing Loops

1. **AHRS Loop (400 Hz)** - Attitude estimation using Mahony filter
2. **Rate PID Loop (400 Hz)** - Inner control loop for angular rates
3. **Angle PID Loop (50 Hz)** - Outer control loop for attitude angles
4. **Altitude Loop (25 Hz)** - Altitude estimation and control
5. **Communication Loop (50 Hz)** - NRF24L01 data exchange

---

## Core Algorithms

### 1. AHRS (Attitude and Heading Reference System)

#### Mahony Filter Implementation

The Mahony filter fuses gyroscope and accelerometer data to estimate the drone's orientation using quaternions.

**Key Features:**
- **Update Rate:** 400 Hz
- **Filter Type:** Mahony complementary filter
- **Output:** Quaternion (q0, q1, q2, q3) and Euler angles (roll, pitch, yaw)

**Algorithm Flow:**
1. Read gyroscope rates (deg/s) and accelerometer (g)
2. Compute gravity vector error using accelerometer
3. Apply proportional-integral feedback to correct gyro drift
4. Integrate quaternion using corrected gyro rates
5. Normalize quaternion to prevent drift
6. Convert quaternion to Euler angles for control

**Tuning Parameters:**
- `Kp_Mahony = 2.0` - Proportional gain (higher = faster correction)
- `Ki_Mahony = 0.005` - Integral gain (handles long-term drift)

**Why Mahony over Madgwick?**
- Lower computational cost (better for Arduino Nano)
- More stable for aggressive maneuvers
- Sufficient accuracy for rate control

### 2. Cascade PID Control Architecture

#### Inner Loop: Rate Control (400 Hz)

Controls angular rates directly using gyroscope feedback.

**Controllers:**
- Roll Rate PID: Controls roll angular velocity
- Pitch Rate PID: Controls pitch angular velocity  
- Yaw Rate PID: Controls yaw angular velocity

**Default Tuning:**
```
Roll Rate:  Kp=0.8, Ki=0.0, Kd=0.05
Pitch Rate: Kp=0.8, Ki=0.0, Kd=0.05
Yaw Rate:   Kp=1.2, Ki=0.0, Kd=0.1
```

**Why High Frequency?**
- Direct rate control requires fast response
- Prevents oscillation and improves stability
- Matches gyroscope update rate

#### Outer Loop: Angle Control (50 Hz)

Converts pilot angle commands to rate setpoints.

**Controllers:**
- Roll Angle PID: Converts roll angle command to roll rate setpoint
- Pitch Angle PID: Converts pitch angle command to pitch rate setpoint

**Default Tuning:**
```
Roll Angle:  Kp=3.0, Ki=0.0, Kd=0.0
Pitch Angle: Kp=3.0, Ki=0.0, Kd=0.0
```

**Why Lower Frequency?**
- Angle changes are slower than rates
- Reduces computational load
- Prevents over-correction

**Control Flow:**
```
Pilot Command (angle) → Angle PID → Rate Setpoint → Rate PID → Motor Output
```

### 3. Altitude Estimation and Control

#### Altitude Estimation (Complementary Filter)

Fuses barometric pressure and vertical acceleration.

**Algorithm:**
1. Read barometric altitude (MS5611)
2. Transform accelerometer to world frame using quaternion
3. Extract vertical acceleration component
4. Complementary filter: `α * baro + (1-α) * (prev_alt + vel*dt)`
5. Estimate vertical velocity from acceleration integration

**Parameters:**
- `α = 0.98` - Trust barometer more (smooth but slow)
- Update rate: 25 Hz

**Why Complementary Filter?**
- Simple and computationally efficient
- Good balance between responsiveness and noise rejection
- No matrix operations (unlike Kalman filter)

#### Altitude Hold PID

Maintains commanded altitude using throttle adjustment.

**Controller:**
- Altitude PID: Controls altitude error → climb rate command

**Default Tuning:**
```
Altitude: Kp=0.5, Ki=0.1, Kd=0.05
```

**Operation:**
- When SW1 is ON: Maintains current altitude
- When SW1 is OFF: Manual throttle control
- Smooth transition between modes

### 4. Motor Mixing

Converts control commands to individual motor PWM signals.

#### X-Quad Configuration

```
    FL ──── FR
     │       │
     │   X   │
     │       │
    RL ──── RR
```

#### Mixing Formula

For X-quad configuration:

```
FL = throttle + roll + pitch - yaw
FR = throttle - roll + pitch + yaw
RR = throttle - roll - pitch - yaw
RL = throttle + roll - pitch + yaw
```

Where:
- `throttle`: Base thrust (0-1, limited to 65%)
- `roll`: Roll command (-1 to +1)
- `pitch`: Pitch command (-1 to +1)
- `yaw`: Yaw command (-1 to +1)

**Normalization:**
- All inputs normalized to [-1, 1] range
- Output constrained to [0, 0.65] (65% max throttle)
- Converted to PWM: 1000-2000 microseconds

**Why 65% Throttle Limit?**
- Safety margin for emergency maneuvers
- Prevents over-powering
- Allows headroom for control authority

---

## Communication Protocol

### NRF24L01 Configuration

- **Channel:** 103
- **Data Rate:** 250 kbps (reliable)
- **Power Level:** MAX
- **ACK Mode:** Enabled (reliable delivery)
- **Retries:** 5 attempts, 15 delay

### RC → FC Command Structure

```c
struct RC_Command {
  uint16_t throttle;    // 1000-2000
  uint16_t yaw;         // 1000-2000
  uint16_t pitch;       // 1000-2000
  uint16_t roll;        // 1000-2000
  uint8_t button1;      // Calibration
  uint8_t button2;      // Motor ON/ESC cal
  uint8_t sw1;          // Altitude Hold
  uint8_t sw2;          // ARM/DISARM
  uint8_t checksum;     // Data integrity
}
```

### FC → RC Telemetry Structure

```c
struct FC_Telemetry {
  float roll;           // Current roll angle (°)
  float pitch;          // Current pitch angle (°)
  float yaw_rate;       // Yaw rate (°/s)
  float altitude;       // Altitude (m)
  float vertical_speed; // Vertical speed (m/s)
  uint8_t armed;        // Armed status
  uint8_t altitude_hold; // Altitude hold active
  uint8_t link_status;  // Link status
  uint8_t calibration_status; // Calibration status
  uint8_t checksum;     // Data integrity
}
```

### Checksum Algorithm

Simple XOR checksum for data integrity:
```c
checksum = data[0] ^ data[1] ^ ... ^ data[n-1]
```

---

## Safety Systems

### 1. Maximum Tilt Angle Protection

**Limit:** ±30 degrees

**Action:**
- If roll or pitch exceeds ±30°, motors immediately disarm
- Prevents flip-over and damage

**Implementation:**
```c
if (abs(roll) > 30.0 || abs(pitch) > 30.0) {
  disarm_motors();
}
```

### 2. Throttle Limiting

**Limit:** 65% maximum throttle

**Purpose:**
- Prevents over-powering
- Maintains control authority
- Safety margin for emergencies

### 3. Link Loss Failsafe

**Timeout:** 500 ms

**Action:**
- If no RC command received for 500 ms:
  - System automatically disarms
  - Motors stop immediately
  - LED stops blinking

**Implementation:**
```c
if (millis() - last_link_time > 500) {
  link_active = false;
  system_armed = false;
}
```

### 4. ARM/DISARM Switch

**SW2 Function:**
- SW2 = 1: System armed (motors can run)
- SW2 = 0: System disarmed (motors disabled)

**Safety:**
- Motors cannot run unless SW2 is ON
- Immediate disarm on switch OFF

### 5. Smooth Motor Startup

**ESC Calibration:**
- Button 2 triggers calibration sequence
- Motors ramp up individually for testing
- Confirmation beeps indicate completion

---

## Calibration Procedures

### 1. IMU Calibration (Button 1)

**Purpose:** Calibrate gyroscope and accelerometer offsets

**Procedure:**
1. Place drone on level surface
2. Press Button 1 on RC
3. FC collects 1000 samples over 2 seconds
4. Calculates gyro offsets (should be ~0 when stationary)
5. Validates calibration (offsets must be < 50 deg/s)

**Success:** 2 short beeps
**Failure:** 1 long beep (7 seconds)

**Why Important:**
- Removes sensor bias
- Improves attitude estimation accuracy
- Prevents drift

### 2. ESC Calibration (Button 2)

**Purpose:** Calibrate ESC endpoints for proper PWM range

**Procedure:**
1. Press Button 2 on RC
2. FC sends MAX_PWM (2000 μs) to all ESCs
3. Wait 2 seconds
4. FC sends MIN_PWM (1000 μs) to all ESCs
5. Wait 2 seconds
6. Test each motor individually (1200 μs pulse)

**Confirmation:** 3 beeps

**Why Important:**
- Ensures ESCs understand full PWM range
- Synchronizes motor response
- Prevents motor startup issues

### 3. Joystick Calibration (Automatic)

**Procedure:**
- Performed automatically on RC startup
- Move all joysticks to extremes
- System records min/max/center values
- Calibration stored for session

---

## System Initialization Sequence

### Flight Controller Startup

1. **Hardware Init**
   - Initialize pins (LED, buzzer, ESC outputs)
   - Set motors to minimum PWM
   - Initialize I2C bus

2. **Sensor Init**
   - Initialize MPU6050 (verify connection)
   - Initialize MS5611 barometer
   - If sensor fails → long beep, halt

3. **NRF Init**
   - Configure NRF24L01
   - Set channel, power, data rate
   - Enable ACK mode
   - Start listening for RC commands

4. **Control Init**
   - Initialize PID controllers with default values
   - Initialize AHRS quaternion (no rotation)
   - Set timing variables

5. **Startup Complete**
   - 2 short beeps
   - LED ready to blink on link

### Remote Controller Startup

1. **Hardware Init**
   - Initialize pins (buttons, switches, joysticks)
   - Initialize NRF24L01

2. **Joystick Calibration**
   - Auto-calibrate joystick ranges
   - Store calibration values

3. **Ready State**
   - Begin sending commands
   - Monitor link status
   - Display status on serial monitor

---

## Operating Workflow

### Pre-Flight Checklist

1. **Power On**
   - Turn on RC first
   - Turn on FC second
   - Verify LED blinking (link established)

2. **Calibration**
   - Ensure drone on level surface
   - Press Button 1 → Wait for 2 beeps (IMU calibration)
   - Press Button 2 → Wait for 3 beeps (ESC calibration)

3. **Arming**
   - Set SW2 to ON (ARM position)
   - Verify armed status on serial monitor
   - Keep throttle at minimum

4. **Flight Mode**
   - Set SW1 to OFF (manual altitude control)
   - Ready for takeoff

### Flight Operations

1. **Takeoff**
   - Slowly increase throttle
   - Use pitch/roll to stabilize
   - Use yaw to control heading

2. **Altitude Hold (Optional)**
   - Set SW1 to ON
   - Release throttle stick
   - Drone maintains current altitude
   - Use pitch/roll/yaw for maneuvering

3. **Landing**
   - Reduce throttle gradually
   - Set SW2 to OFF (disarm) before touchdown
   - Motors stop immediately

---

## Tuning Guide

### PID Tuning Philosophy

**General Rules:**
1. Tune inner loops first (rate PIDs)
2. Then tune outer loops (angle PIDs)
3. Finally tune altitude control
4. Make small adjustments (10-20% at a time)
5. Test thoroughly after each change

### Rate PID Tuning

**Goal:** Responsive rate control without oscillation

**Procedure:**
1. Start with default values
2. Increase Kp until response is snappy but not oscillatory
3. Add Kd to dampen overshoot
4. Ki usually not needed for rate control

**Symptoms:**
- **Too Low Kp:** Sluggish response, slow correction
- **Too High Kp:** Oscillation, instability
- **Too Low Kd:** Overshoot, ringing
- **Too High Kd:** Slow response, jittery

**Test:** Hover and apply quick stick inputs, observe response

### Angle PID Tuning

**Goal:** Smooth angle tracking without overshoot

**Procedure:**
1. Start with Kp = 3.0
2. Increase Kp for faster angle response
3. Usually no Ki or Kd needed

**Symptoms:**
- **Too Low Kp:** Slow to reach commanded angle
- **Too High Kp:** Oscillation, overshoot

**Test:** Command specific angles, observe tracking

### Altitude PID Tuning

**Goal:** Stable altitude hold without drift

**Procedure:**
1. Start with default values
2. Increase Kp for faster correction
3. Add Ki to eliminate steady-state error
4. Add Kd to reduce overshoot

**Symptoms:**
- **Too Low Kp:** Slow altitude correction
- **Too High Kp:** Oscillation, hunting
- **Too Low Ki:** Altitude drift over time
- **Too High Ki:** Wind-up, instability

**Test:** Enable altitude hold, observe stability

### AHRS Filter Tuning

**Mahony Filter Parameters:**

- **Kp_Mahony:** Proportional gain (default: 2.0)
  - Higher = faster correction, more noise
  - Lower = smoother, slower correction

- **Ki_Mahony:** Integral gain (default: 0.005)
  - Handles long-term gyro drift
  - Usually keep low

**Tuning:**
- If attitude drifts: Increase Ki_Mahony
- If attitude noisy: Decrease Kp_Mahony
- If slow to correct: Increase Kp_Mahony

---

## Testing Procedures

### Bench Testing (No Props!)

1. **Power-On Test**
   - Verify all sensors initialize
   - Check NRF link establishment
   - Verify LED blinking

2. **Calibration Test**
   - Test IMU calibration (Button 1)
   - Test ESC calibration (Button 2)
   - Verify beep sequences

3. **Motor Direction Test**
   - Arm system (SW2 ON)
   - Test each motor individually
   - Verify correct rotation direction
   - **CRITICAL:** Ensure props are OFF!

4. **Control Response Test**
   - Move joysticks
   - Observe motor PWM changes on serial monitor
   - Verify correct motor mixing

5. **Failsafe Test**
   - Turn off RC
   - Verify FC disarms within 500 ms
   - Verify motors stop

### Flight Testing

1. **First Flight**
   - Open area, no obstacles
   - Low altitude hover
   - Test basic controls
   - Land immediately if unstable

2. **Tuning Flight**
   - Hover and observe behavior
   - Adjust PIDs incrementally
   - Test each axis independently

3. **Altitude Hold Test**
   - Enable altitude hold (SW1 ON)
   - Release throttle
   - Verify altitude maintenance
   - Test transitions

4. **Stress Test**
   - Aggressive maneuvers
   - Test failsafe (turn off RC)
   - Test tilt limit protection

---

## Troubleshooting

### Common Issues

**1. No Link Between RC and FC**
- Check NRF24L01 connections
- Verify channel matches (103)
- Check power supply (NRF needs stable 3.3V)
- Verify antenna connection

**2. Drone Oscillates**
- Rate PID Kp too high → Reduce Kp
- Rate PID Kd too low → Increase Kd
- Check motor/prop balance

**3. Slow Response**
- Rate PID Kp too low → Increase Kp
- Check ESC calibration
- Verify motor connections

**4. Altitude Hold Drifts**
- Altitude PID Ki too low → Increase Ki
- Check barometer readings
- Verify MS5611 connection

**5. Yaw Drift**
- Normal without magnetometer
- Use yaw stick to correct
- Consider adding magnetometer for future version

**6. Motors Don't Start**
- Check SW2 (ARM switch)
- Verify ESC calibration
- Check PWM output range
- Verify motor connections

---

## Performance Specifications

### Timing Performance

- **AHRS Update:** 400 Hz (2.5 ms period)
- **Rate PID:** 400 Hz (2.5 ms period)
- **Angle PID:** 50 Hz (20 ms period)
- **Altitude Control:** 25 Hz (40 ms period)
- **Communication:** 50 Hz (20 ms period)

### Control Limits

- **Max Tilt Angle:** ±30°
- **Max Throttle:** 65%
- **PWM Range:** 1000-2000 μs
- **Link Timeout:** 500 ms

### Sensor Specifications

- **MPU6050:** ±2000°/s gyro, ±16g accel
- **MS5611:** 10 cm altitude resolution
- **NRF24L01:** 250 kbps, up to 1000 m range (with PA+LNA)

---

## Code Structure

### Flight Controller Modules

1. **DroneProtocol.h** - Communication structures
2. **FlightController.ino** - Main FC firmware
   - AHRS filter
   - PID controllers
   - Motor mixing
   - Safety systems
   - Communication

### Remote Controller Modules

1. **DroneProtocol.h** - Communication structures (shared)
2. **RemoteController.ino** - Main RC firmware
   - Joystick reading
   - Button/switch handling
   - Communication
   - Serial output

---

## Future Enhancements

### Possible Improvements

1. **Magnetometer Integration**
   - Add HMC5883L or QMC5883L
   - Eliminate yaw drift
   - Full 9-DOF AHRS

2. **GPS Integration**
   - Position hold
   - Return-to-home
   - Waypoint navigation

3. **SD Card Logging**
   - Flight data recording
   - Post-flight analysis
   - Black box functionality

4. **Telemetry Display**
   - OLED screen on RC
   - Real-time flight data
   - Battery voltage monitoring

5. **Advanced Filters**
   - Extended Kalman Filter (EKF)
   - Better sensor fusion
   - Improved accuracy

---

## License and Disclaimer

**WARNING:** This firmware is for educational and research purposes. Flying drones can be dangerous. Always:
- Follow local regulations
- Fly in safe, open areas
- Use proper safety equipment
- Test thoroughly before flight
- Never fly near people or property

**Use at your own risk.**

---

## Author Notes

This firmware represents a complete, professional-grade implementation suitable for:
- Educational purposes
- Research and development
- Hobbyist projects
- Learning embedded systems

The code is optimized for Arduino Nano's limited resources while maintaining professional flight control standards.

---

## References

- Mahony, R., et al. "Complementary filter design on the special orthogonal group SO(3)"
- MultiWii flight control system (concept inspiration)
- Betaflight/INAV (architecture reference)
- Arduino libraries documentation

---

**Version:** 1.0  
**Last Updated:** 2024  
**Compatibility:** Arduino Nano, Arduino IDE 1.8+
