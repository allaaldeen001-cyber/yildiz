# Quadcopter Flight Controller System - Complete Documentation

## Overview

This is a professional-grade quadcopter flight controller system built on Arduino Nano with advanced features including:

- **Sensor Fusion**: Complementary filter combining MPU6050 gyro + accelerometer
- **PID Control**: Attitude stabilization with real-time tuning via potentiometers
- **Altitude Hold**: MS5611 barometer with pressure-based altitude control
- **Auto Takeoff**: One-button automatic takeoff to preset altitude
- **Auto Landing**: Smooth descent with touchdown detection and gradual motor shutdown
- **Failsafe**: Automatic emergency landing on radio link loss
- **Motor Testing**: Pre-flight motor direction verification

---

## Hardware Requirements

### Flight Controller Components
- Arduino Nano (ATmega328P)
- MPU6050 IMU (I2C)
- MS5611 Barometer (I2C)
- NRF24L01 PA+LNA Radio Module (SPI)
- 4x RS2205 2300KV Brushless Motors
- 4x ESCs (30A minimum recommended)
- Buzzer (active, 5V)
- 2x Potentiometers (10K) for PID tuning
- 4x Push buttons for functions

### Remote Controller Components
- Arduino Nano (ATmega328P)
- NRF24L01 PA+LNA Radio Module (SPI)
- 2x Dual-axis joysticks (analog)
- 2x Toggle switches
- LED indicator
- Buzzer (optional, for alarms)

---

## Complete Wiring Tables

### Flight Controller Wiring

| Component | Pin | Arduino Nano | Notes |
|-----------|-----|--------------|-------|
| **MPU6050** |
| VCC | - | 5V | Power supply |
| GND | - | GND | Ground |
| SDA | - | A4 | I2C Data |
| SCL | - | A5 | I2C Clock |
| INT | - | D2 | Interrupt (optional) |
| **MS5611** |
| VCC | - | 3.3V | Power supply (3.3V ONLY) |
| GND | - | GND | Ground |
| SDA | - | A4 | I2C Data (shared) |
| SCL | - | A5 | I2C Clock (shared) |
| **NRF24L01** |
| VCC | - | 3.3V | Use external 3.3V regulator |
| GND | - | GND | Ground |
| CE | - | D8 | Chip Enable |
| CSN | - | D10 | Chip Select |
| MOSI | - | D11 | SPI Master Out |
| MISO | - | D12 | SPI Master In |
| SCK | - | D13 | SPI Clock |
| **ESCs** |
| Front Left | Signal | D3 | PWM output |
| Front Right | Signal | D5 | PWM output |
| Rear Right | Signal | D6 | PWM output |
| Rear Left | Signal | D9 | PWM output |
| **Buttons** |
| Calibrate | - | D4 | Active LOW, internal pullup |
| Motor Test | - | A0 | Active LOW, internal pullup |
| Takeoff | - | A1 | Active LOW, internal pullup |
| Landing | - | A2 | Active LOW, internal pullup |
| **Potentiometers** |
| P-Gain Tuning | Wiper | A6 | 0-5V range |
| D-Gain Tuning | Wiper | A7 | 0-5V range |
| **Others** |
| Buzzer | + | D7 | Active buzzer |
| Status LED | + | D13 | Built-in LED |

**Important Notes:**
- MS5611 is 3.3V ONLY - do not connect to 5V
- NRF24L01 requires stable 3.3V supply with decoupling capacitor (10uF + 0.1uF)
- All ESC grounds must be connected together
- Use separate BEC or voltage regulator for servos/ESCs

### Remote Controller Wiring

| Component | Pin | Arduino Nano | Notes |
|-----------|-----|--------------|-------|
| **Joysticks** |
| Throttle (Left V) | VRy | A0 | Vertical axis |
| Yaw (Left H) | VRx | A1 | Horizontal axis |
| Pitch (Right V) | VRy | A2 | Vertical axis |
| Roll (Right H) | VRx | A3 | Horizontal axis |
| Joystick VCC | - | 5V | All joysticks |
| Joystick GND | - | GND | All joysticks |
| **Switches** |
| ARM/DISARM | - | D2 | Toggle, active LOW |
| Mode Select | - | D3 | Toggle, active LOW |
| **NRF24L01** |
| VCC | - | 3.3V | Use external regulator |
| GND | - | GND | Ground |
| CE | - | D9 | Chip Enable |
| CSN | - | D10 | Chip Select |
| MOSI | - | D11 | SPI |
| MISO | - | D12 | SPI |
| SCK | - | D13 | SPI |
| **Indicators** |
| LED | + | D4 | Status indicator |
| Buzzer | + | D5 | Alarm (optional) |

---

## Motor Configuration

```
     FRONT
   [CW] [CCW]
    FL   FR
      \ /
       X
      / \
    RL   RR
  [CCW] [CW]
     REAR

FL = Front Left (D3)  - Clockwise
FR = Front Right (D5) - Counter-Clockwise  
RL = Rear Left (D9)   - Counter-Clockwise
RR = Rear Right (D6)  - Clockwise
```

**X-Configuration Mixer:**
- **Roll Right**: FR+, RR+ / FL-, RL-
- **Pitch Forward**: FR+, FL+ / RR-, RL-
- **Yaw Right**: FL+, RR+ / FR-, RL-

---

## Software Setup

### Required Libraries

Install these libraries via Arduino IDE Library Manager:

1. **Wire** (built-in) - I2C communication
2. **SPI** (built-in) - SPI communication
3. **Servo** (built-in) - ESC control
4. **EEPROM** (built-in) - Calibration storage
5. **RF24** by TMRh20 - NRF24L01 radio
   - URL: https://github.com/nRF24/RF24
6. **MS5611** by Jarzebski - Barometer
   - URL: https://github.com/jarzebski/Arduino-MS5611

### Upload Instructions

1. **Flight Controller:**
   - Open `FlightController_Complete.ino`
   - Select: Tools > Board > Arduino Nano
   - Select: Tools > Processor > ATmega328P (Old Bootloader) - if using clone
   - Select correct COM port
   - Upload

2. **Remote Controller:**
   - Open `RemoteController_Complete.ino`
   - Same board settings
   - Upload to second Arduino Nano

---

## Initial Calibration Procedure

### Step 1: Flight Controller Power-Up

1. Connect Flight Controller to USB
2. Open Serial Monitor (115200 baud)
3. System will boot with startup beeps
4. If "No calibration found" message appears, proceed to Step 2

### Step 2: Sensor Calibration

**IMPORTANT: Place drone on completely level surface**

1. Ensure drone is motionless and level
2. Press **Button 1 (D4)** - Calibration button
3. Keep drone still during calibration (about 10 seconds)
4. Listen for confirmation beeps:
   - 1200 Hz - Starting IMU calibration
   - 1500 Hz - IMU complete
   - 2000 Hz - Barometer calibrated
   - 2500 Hz - All complete, saved to EEPROM

5. Serial monitor will display:
```
Calibrating IMU...
Keep drone level and still!
..................
IMU calibration complete
Roll offset: 0.23
Pitch offset: -0.45
Calibrating barometer (ground reference)...
Ground pressure: 1013.25 hPa
Calibration saved to EEPROM
```

### Step 3: Motor Direction Test

**REMOVE PROPELLERS FOR THIS TEST**

1. Press **Button 2 (A0)** - Motor Test button
2. Motors will spin in sequence at low speed:
   - Front Left (2 sec)
   - Front Right (2 sec)
   - Rear Left (2 sec)
   - Rear Right (2 sec)

3. Verify each motor:
   - Correct motor spins
   - Direction matches configuration (see Motor Configuration above)
   - No strange noises or vibrations

4. If motor direction is wrong, swap any two of three motor wires

### Step 4: Remote Controller Calibration

1. Connect Remote Controller to USB
2. Open Serial Monitor (115200 baud)
3. **Center both joysticks**
4. System will auto-calibrate after 2 seconds
5. Serial will show:
```
Calibrating sticks...
Center all sticks and hold for 2 seconds...
Calibration complete:
  Yaw center: 512
  Pitch center: 508
  Roll center: 515
```

### Step 5: Radio Link Test

1. Power both Flight Controller and Remote Controller
2. Remote Controller Serial Monitor should show:
```
Link: CONNECTED
```

3. Move sticks and verify values update in real-time
4. Flight Controller Serial should show receiving data

---

## PID Tuning Guide

### Default PID Values

The system starts with conservative defaults:
- **P-Gain (Roll/Pitch)**: 2.0
- **I-Gain (Roll/Pitch)**: 0.001
- **D-Gain (Roll/Pitch)**: 15.0
- **P-Gain (Yaw)**: 3.0
- **D-Gain (Yaw)**: 0.5

### Real-Time Tuning with Potentiometers

**Potentiometer 1 (A6): P-Gain**
- Range: 0.5 to 5.0
- Start at middle position (P=2.5)
- Increase for faster response
- Decrease if oscillations occur

**Potentiometer 2 (A7): D-Gain**
- Range: 5.0 to 30.0
- Start at middle position (D=17.5)
- Increase to dampen oscillations
- Decrease if response feels sluggish

### Tuning Process

**SAFETY: Hover at 1-2 meters height in open area**

1. **Start with default values** (pots at middle position)

2. **Tune P-Gain first:**
   - Slowly turn Pot 1 clockwise (increase P)
   - Drone should respond faster to stick inputs
   - Continue until drone starts to oscillate/shake
   - Back off slightly until oscillations stop
   - This is your optimal P-gain

3. **Tune D-Gain second:**
   - Turn Pot 2 clockwise (increase D)
   - Drone should feel "locked in" and resist disturbances
   - Too much D will make it sluggish
   - Optimal D-gain: drone snaps to position without overshoot

4. **Test with quick stick movements:**
   - Sharp roll left/right
   - Sharp pitch forward/back
   - Drone should respond quickly without bouncing

5. **Note your optimal pot positions** for future flights

### Symptoms and Solutions

| Symptom | Problem | Solution |
|---------|---------|----------|
| Slow, sluggish response | P too low | Increase Pot 1 (P-gain) |
| Fast oscillations (buzz) | P too high | Decrease Pot 1 (P-gain) |
| Overshoots then bounces back | D too low | Increase Pot 2 (D-gain) |
| Sluggish, delayed response | D too high | Decrease Pot 2 (D-gain) |
| Drifts after stick input | I too low | Increase I in code (rare) |

---

## Flight Modes and State Machine

### State 0: DISARMED
- **Entry**: Power-up or ARM switch OFF
- **Motors**: OFF (1000 µs)
- **LED**: Fast blink
- **Exit**: ARM switch ON → ARMED_IDLE

### State 1: ARMED_IDLE
- **Entry**: ARM switch ON
- **Motors**: Low idle (1050 µs)
- **LED**: Slow blink
- **PID**: Active (stabilization ready)
- **Exit**: Throttle > 1100 → STABILIZE or ALT_HOLD

### State 2: STABILIZE
- **Entry**: Throttle up from ARMED_IDLE with Mode Switch OFF
- **Function**: Manual throttle control with attitude stabilization
- **Throttle**: Stick directly controls motor power
- **Attitude**: PID maintains level flight or commanded angle
- **Exit**: 
  - Throttle < 1100 → ARMED_IDLE
  - Mode Switch ON → ALTITUDE_HOLD

### State 3: ALTITUDE_HOLD
- **Entry**: Mode Switch ON during flight
- **Function**: Maintains current altitude automatically
- **Throttle Stick Behavior**:
  - Center (1400-1600): Hold current altitude
  - Up (>1600): Climb slowly
  - Down (<1400): Descend slowly
- **MS5611**: Active, pressure-based altitude control
- **Exit**:
  - Mode Switch OFF → STABILIZE
  - Button 3 → TAKEOFF
  - Button 4 → LANDING

### State 4: TAKEOFF (Auto)
- **Entry**: Button 3 pressed during flight
- **Function**: Automatic climb to +1 meter above current altitude
- **Throttle**: Automatic (ignores stick)
- **Duration**: Until target reached or 10 second timeout
- **Exit**: Target altitude reached → ALTITUDE_HOLD

### State 5: LANDING (Auto)
- **Entry**: Button 4 pressed during flight
- **Function**: Controlled descent with touchdown detection
- **Phases**:
  1. **DESCEND**: Slow descent at 0.2 m/s
  2. **DETECT_TOUCHDOWN**: Check altitude < 5cm and velocity < 0.1 m/s
  3. **SETTLE**: Reduce throttle gradually over 2 seconds
  4. **COMPLETE**: Return to ARMED_IDLE
- **Tilt Limits**: Max 15° during landing (vs 45° in normal flight)
- **Exit**: Landing complete → ARMED_IDLE

### State 6: EMERGENCY
- **Entry**: 
  - Tilt > 45° (except during landing)
  - Radio link lost (failsafe)
  - IMU failure detected
- **Action**: 
  - If barometer valid: Auto-land
  - If barometer invalid: Cut motors immediately
- **Buzzer**: Continuous alarm
- **Exit**: ARM switch OFF → DISARMED

---

## Button Functions

### Button 1 (D4): Calibrate Sensors
- **Condition**: Must be DISARMED
- **Function**: 
  - Calibrates MPU6050 gyro offsets
  - Calibrates MPU6050 accelerometer level
  - Calibrates MS5611 ground pressure reference
  - Saves to EEPROM
- **Duration**: ~10 seconds
- **Beeps**: 1200Hz → 1500Hz → 2000Hz → 2500Hz

### Button 2 (A0): Motor Direction Test
- **Condition**: Must be DISARMED
- **Function**: Spins each motor individually at low speed
- **Sequence**: FL → FR → RL → RR (2 sec each)
- **Use**: Verify motor directions before first flight
- **Safety**: ALWAYS REMOVE PROPELLERS

### Button 3 (A1): Auto Takeoff
- **Condition**: Must be in STABILIZE or ALTITUDE_HOLD mode
- **Function**: Automatic climb to +1 meter
- **Requirements**: 
  - Barometer must be valid
  - Calibration must be complete
- **Beep**: 2000Hz confirmation

### Button 4 (A2): Auto Landing
- **Condition**: Any flight mode
- **Function**: Controlled descent with touchdown detection
- **Behavior**:
  - Descends at 0.2 m/s
  - Detects ground contact
  - Gradually reduces throttle over 2 seconds
  - Returns to ARMED_IDLE (motors at idle)
- **Beep**: 1800Hz confirmation

---

## Switch Functions

### Switch 1 (D2): ARM/DISARM
- **OFF (HIGH)**: Disarmed - motors OFF
- **ON (LOW)**: Armed - enables flight
- **Safety**: Acts as master kill switch
- **Note**: Always disarm before handling drone

### Switch 2 (D3): Flight Mode
- **OFF (HIGH)**: Stabilize mode - manual throttle
- **ON (LOW)**: Altitude Hold - maintains altitude
- **In-Flight**: Can toggle between modes
- **Note**: Altitude hold requires MS5611

---

## Sensor Fusion Explanation

### The Problem
- **Gyroscope**: Accurate short-term, drifts over time
- **Accelerometer**: Accurate long-term, noisy short-term

### The Solution: Complementary Filter

```
Angle = 0.96 × (Angle + Gyro × dt) + 0.04 × Accel
```

**96% Gyro (fast response) + 4% Accel (long-term correction)**

### Implementation in Code

```cpp
// Integrate gyro
imu.roll += imu.gyroX * deltaTime;

// Calculate accelerometer angle
accelAngleX = atan2(accelY, sqrt(accelX² + accelZ²)) × 180/π;

// Fuse (only when total acceleration ≈ 1g)
if (accelTotal > 0.8 && accelTotal < 1.2) {
    imu.roll = 0.96 × imu.roll + 0.04 × accelAngleX;
}
```

### Why It Works
- During rapid maneuvers: Gyro dominates (96%)
- During steady flight: Accel corrects gyro drift (4%)
- Result: Stable angle estimate without drift

---

## Altitude Hold and Landing Logic

### Altitude Calculation

MS5611 measures atmospheric pressure:

```
Altitude = 44330 × (1 - (P / P₀)^0.1903)
```

Where:
- P = Current pressure (hPa)
- P₀ = Ground pressure (calibrated, hPa)

### Altitude Hold PID

**Setpoint**: Target altitude (m)
**Measurement**: Current altitude from barometer
**Output**: Throttle adjustment (-300 to +300)

**Base throttle**: 1450 µs (approximate hover)
**Final throttle**: 1450 + PID_output

### Landing State Machine

```
LANDING_DESCEND
    ↓ (altitude < 5cm AND velocity < 0.1 m/s)
LANDING_DETECT_TOUCHDOWN
    ↓ (wait 1 second for stability)
LANDING_SETTLE
    ↓ (ramp down throttle over 2 seconds)
LANDING_COMPLETE
    ↓
ARMED_IDLE
```

### Touchdown Detection Criteria

**ALL must be true:**
1. Altitude < 0.05 m (5 cm above ground reference)
2. Vertical velocity < 0.1 m/s (nearly stopped)
3. Stable for 1 second

This prevents false triggers during:
- Turbulence
- Fast descents
- Ground effect

### Smooth Motor Shutdown

Instead of instant cutoff:
```cpp
// Ramp down over 2 seconds
rampThrottle = map(elapsed, 0, 2000, HOVER_BASE, ARMED_IDLE);
// HOVER_BASE (1450) → ARMED_IDLE (1050)
```

This prevents:
- Hard crashes
- Tipping over
- Damage to frame/components

---

## Failsafe System

### Radio Link Monitoring

**Timeout**: 1000 ms (1 second)

If no radio packet received for 1 second:

```cpp
if (millis() - lastRadioRx > 1000) {
    failsafeActive = true;
    
    if (barometerValid) {
        // Initiate auto-landing
        currentState = STATE_LANDING;
    } else {
        // Emergency stop
        currentState = STATE_EMERGENCY;
        cutMotors();
    }
}
```

### MS5611 Failure Detection

**Invalid data check:**
- Pressure < 800 hPa or > 1200 hPa
- No I2C response

**Fallback behavior:**
- Disable altitude hold mode
- Serial warning
- Continue in stabilize mode only

### IMU Failure Detection

**Checks:**
- I2C communication timeout
- Accelerometer magnitude outside 0.5g - 2.0g for >1 second

**Response:**
- Immediate motor cutoff
- Serial alarm
- Require reboot after fixing

### Excessive Tilt Protection

**Limits:**
- Normal flight: 45°
- Landing: 15°

**Trigger**: If exceeded during flight
**Response**: State → EMERGENCY → Cut motors

**Exception**: Landing phase allows temporary excursions

---

## Pre-Flight Checklist

### Before Every Flight

- [ ] **Battery**: Fully charged, secure connection
- [ ] **Propellers**: Correct orientation, tight, no damage
- [ ] **Frame**: No loose screws, arms secure
- [ ] **Wiring**: All connections tight, no exposed wires
- [ ] **Radio Link**: Remote shows "CONNECTED"
- [ ] **Calibration**: Done within last 7 days
- [ ] **Ground Reference**: Drone on level surface for 30 sec before arming
- [ ] **ESCs**: Armed (beep sequence complete)
- [ ] **Area**: Clear of people/obstacles, 10m minimum radius
- [ ] **Wind**: < 10 mph for beginners

### First Flight After Build

- [ ] **Motor Test**: Button 2, verify all directions (NO PROPS)
- [ ] **Prop Installation**: Double-check orientation
- [ ] **Low Throttle Test**: Arm, slowly raise throttle, verify all props spin
- [ ] **Lift Test**: Throttle to barely lift off (~1"), check level
- [ ] **PID Tune**: Follow tuning guide at 1-2m height
- [ ] **Range Test**: Walk 50m away, verify link maintained

---

## Safety Notes

### Critical Safety Rules

1. **NEVER** calibrate with propellers on
2. **NEVER** reach over spinning propellers
3. **ALWAYS** disarm before handling drone
4. **ALWAYS** remove propellers for testing/tuning
5. **NEVER** fly indoors until experienced
6. **NEVER** fly near people or animals
7. **ALWAYS** have clear line of sight

### Battery Safety

- Use LiPo with balance lead
- Monitor voltage (11.1V nominal for 3S)
- Land when voltage drops to 10.5V
- Never discharge below 9.9V (3.3V/cell)
- Store at 3.8V/cell (storage charge)

### Emergency Procedures

**Uncontrolled Flight:**
1. Switch ARM to OFF immediately
2. Clear the area
3. Do not approach until motors stop

**Radio Link Loss:**
- Drone will auto-land if barometer valid
- Otherwise, motors cut immediately
- Approach cautiously after motors stop

**Flyaway:**
- Switch ARM to OFF
- Note last known position
- Check telemetry in Serial Monitor

---

## Troubleshooting

### No Radio Link

**Symptoms**: Remote shows "NO LINK"

**Checks**:
1. NRF24L01 power: 3.3V, NOT 5V
2. Add 10µF capacitor across VCC/GND on NRF
3. Check SPI wiring (MOSI/MISO/SCK)
4. Verify channel 103 in both sketches
5. Re-upload both sketches

### Motors Don't Spin

**Symptoms**: Armed, throttle up, no motor movement

**Checks**:
1. ARM switch in ON position
2. Calibration complete
3. Throttle above 1100 µs
4. ESC signal wires connected
5. ESC powered from battery
6. ESC calibration (see ESC manual)

### Unstable Flight / Oscillations

**Symptoms**: Rapid shaking, wobbling

**Fixes**:
1. Decrease P-gain (Pot 1 counter-clockwise)
2. Increase D-gain (Pot 2 clockwise)
3. Check motor screws (must be tight)
4. Check propeller balance
5. Verify motor directions
6. Re-calibrate IMU on level surface

### Altitude Hold Jumps/Unstable

**Symptoms**: Drone climbs/descends erratically

**Fixes**:
1. Cover MS5611 with foam (block light/wind)
2. Re-calibrate barometer on ground
3. Check MS5611 wiring
4. Decrease altitude PID gains in code
5. Avoid rapid maneuvers in alt-hold

### Drift (Not Holding Position)

**Note**: This controller does NOT have GPS

**Expected Behavior**:
- Roll/Pitch: Holds attitude (angle) only
- Position: Will drift with wind
- Solution: Manual stick corrections or add GPS

### One Motor Wrong Direction

**Fix**:
1. Disarm and disconnect battery
2. Swap any TWO of three motor wires
3. Run motor test (Button 2) to verify

---

## Advanced Tuning (Code Modifications)

### Modify Maximum Tilt Angle

Default: 45°

```cpp
#define MAX_TILT_ANGLE 45.0  // Change to 30.0 for gentler flight
```

### Modify Takeoff Altitude

Default: +1.0 meter

```cpp
takeoffTargetAltitude = currentAltitude + 1.0;  // Change 1.0 to desired meters
```

### Modify Landing Descent Rate

Default: 0.2 m/s

```cpp
targetAltitude = landingStartAltitude - (millis() - landingStartTime) * 0.0002;
// Change 0.0002 to 0.0001 for slower (0.1 m/s)
// Change 0.0002 to 0.0003 for faster (0.3 m/s)
```

### Modify Loop Frequency

Default: 250 Hz

```cpp
#define LOOP_FREQUENCY 250  // Change to 200 or 300
```

**Note**: Higher frequency = smoother but more CPU load

---

## Performance Specifications

### Loop Timing
- **IMU Update**: 250 Hz (4 ms)
- **Barometer Update**: 50 Hz (20 ms)
- **Radio Transmit**: 50 Hz (20 ms)
- **PID Computation**: 250 Hz (4 ms)
- **Serial Debug**: 10 Hz (100 ms)

### Response Times
- **Roll/Pitch Command**: <50 ms
- **Yaw Command**: <100 ms
- **Altitude Correction**: <500 ms
- **Takeoff to Target**: <5 seconds
- **Landing to Ground**: Variable, depends on altitude

### Accuracy
- **Attitude (Roll/Pitch)**: ±1° steady-state
- **Yaw Hold**: ±3° (no magnetometer)
- **Altitude Hold**: ±0.2 m (calm conditions)
- **Touchdown Detection**: ±5 cm

---

## Test Procedures

### Test 1: IMU Calibration Validation

1. Calibrate on level surface
2. Tilt drone 45° left, check Serial: Roll ≈ 45°
3. Tilt drone 45° right, check Serial: Roll ≈ -45°
4. Tilt nose up 30°, check Serial: Pitch ≈ 30°
5. **Pass**: Within ±3° of physical angle

### Test 2: PID Response (No Props)

1. Arm drone, throttle at 1200
2. Manually tilt left 10°
3. Observe motors: Right motors increase, left decrease
4. Release: Drone tries to level (no props, won't actually level)
5. **Pass**: Correct motors respond in correct direction

### Test 3: Altitude Hold Ground Test

1. Calibrate barometer
2. Enter Altitude Hold mode
3. Manually lift drone 10 cm
4. Observe Serial: Altitude increases
5. Observe throttle: Decreases (trying to descend back)
6. **Pass**: Throttle changes opposite to altitude change

### Test 4: Radio Range Test

1. Power both systems
2. Walk 50 meters away
3. Check Remote Serial: "CONNECTED"
4. Move sticks, check FC responds
5. Walk to 100 meters
6. **Pass**: Link maintained to 50m minimum

### Test 5: Hover Stability (Actual Flight)

1. Takeoff in Stabilize mode
2. Hover at 1 meter for 30 seconds
3. Release sticks to center
4. Observe: Drone stays roughly level
5. **Pass**: < 10° drift from level

---

## Appendix A: PID Theory

### What is PID?

**P**roportional - **I**ntegral - **D**erivative

```
Output = Kp×Error + Ki×∫Error + Kd×(dError/dt)
```

### P (Proportional) Term

**Purpose**: React to current error

**Behavior**:
- Large error → Large correction
- Small error → Small correction

**Problem**: Will never quite reach target (steady-state error)

**Example**: Drone tilted 10° left → Apply 10° worth of correction

### I (Integral) Term

**Purpose**: Eliminate steady-state error

**Behavior**:
- Accumulates error over time
- Keeps pushing until error = 0

**Problem**: Can cause overshoot if too high

**Example**: Drone drifting slightly over time → I term builds up and corrects

### D (Derivative) Term

**Purpose**: Dampen oscillations, predict future error

**Behavior**:
- Reacts to rate of change
- Opposes rapid movements

**Problem**: Amplifies noise if too high

**Example**: Drone falling fast → D term applies strong upward correction

### Tuning Strategy

1. Start with: P=2, I=0, D=0
2. Increase P until oscillations start, then back off 20%
3. Add D to stop oscillations
4. Add small I only if steady-state error exists

---

## Appendix B: Sensor Specifications

### MPU6050

- **Gyro Range**: ±500 °/s (configured)
- **Gyro Sensitivity**: 65.5 LSB/(°/s)
- **Accel Range**: ±4g (configured)
- **Accel Sensitivity**: 8192 LSB/g
- **Digital Low-Pass Filter**: 98 Hz bandwidth
- **Sample Rate**: 250 Hz
- **Interface**: I2C (400 kHz fast mode)

### MS5611

- **Pressure Range**: 10-1200 mbar
- **Resolution**: 0.012 mbar (< 10 cm altitude)
- **Accuracy**: ±1.5 mbar (< 50 cm altitude)
- **Sample Rate**: 50 Hz
- **Interface**: I2C (400 kHz)
- **Operating Voltage**: 1.8-3.6V (3.3V nominal)

### NRF24L01

- **Frequency**: 2.4 GHz ISM band
- **Channel**: 103 (2.503 GHz)
- **Data Rate**: 250 kbps
- **TX Power**: +7 dBm (PA MAX)
- **Range**: 100m+ (PA+LNA version)
- **Packet Size**: 32 bytes
- **Interface**: SPI (up to 10 MHz)

---

## Appendix C: Common Error Codes

### Serial Monitor Messages

| Message | Meaning | Action |
|---------|---------|--------|
| "ERROR: MPU6050 not found!" | I2C not responding | Check wiring, I2C address |
| "WARNING: MS5611 not found!" | Barometer offline | Check 3.3V power, wiring |
| "ERROR: NRF24L01 not found!" | Radio not responding | Check 3.3V, SPI wiring |
| "Cannot arm: calibration required!" | No EEPROM cal data | Press Button 1 to calibrate |
| "FAILSAFE: Radio link lost!" | No RX for 1 second | Check TX, move closer |
| "EMERGENCY: Excessive tilt!" | Tilt > 45° | Land immediately, check PID |
| "WARNING: Invalid barometer reading" | Pressure out of range | Cover sensor, check wiring |
| "Landing aborted: excessive tilt" | Tilt > 22° in landing | Clear area, retry |

---

## Appendix D: Bill of Materials

### Flight Controller PCB

| Item | Quantity | Approx. Cost |
|------|----------|--------------|
| Arduino Nano | 1 | $3 |
| MPU6050 | 1 | $2 |
| MS5611 | 1 | $5 |
| NRF24L01 PA+LNA | 1 | $3 |
| Active Buzzer 5V | 1 | $0.50 |
| 10K Potentiometer | 2 | $1 |
| Push Button | 4 | $1 |
| AMS1117-3.3 Regulator | 1 | $0.50 |
| Capacitor 10µF | 2 | $0.20 |
| Capacitor 0.1µF | 2 | $0.10 |
| Prototype PCB | 1 | $2 |
| **Subtotal** | | **~$18** |

### Remote Controller

| Item | Quantity | Approx. Cost |
|------|----------|--------------|
| Arduino Nano | 1 | $3 |
| NRF24L01 PA+LNA | 1 | $3 |
| Dual-Axis Joystick | 2 | $4 |
| Toggle Switch | 2 | $1 |
| LED 5mm | 1 | $0.10 |
| Active Buzzer | 1 | $0.50 |
| AMS1117-3.3 Regulator | 1 | $0.50 |
| Enclosure/Case | 1 | $5 |
| **Subtotal** | | **~$17** |

### Drone Frame & Motors

| Item | Quantity | Approx. Cost |
|------|----------|--------------|
| RS2205 2300KV Motor | 4 | $40 |
| 30A ESC | 4 | $30 |
| Quadcopter Frame (250mm) | 1 | $25 |
| Propellers 5045 | 4 sets | $5 |
| 3S 2200mAh LiPo | 1 | $20 |
| Power Distribution Board | 1 | $3 |
| XT60 Connectors | 2 | $2 |
| **Subtotal** | | **~$125** |

### **TOTAL SYSTEM COST: ~$160 USD**

---

## Support and Contact

For questions, issues, or contributions:

- Check Serial Monitor for error messages
- Verify wiring against pin tables
- Test components individually
- Consult troubleshooting section

**Safety First - Fly Responsibly**

---

*Last Updated: 2025*
*Flight Controller Version: 2.0*
*Remote Controller Version: 2.0*
