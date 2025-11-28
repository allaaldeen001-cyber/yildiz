# Technical Reference

## Detailed Explanation of Filters, Control Loops, and Safety Logic

This document provides in-depth technical explanations of all algorithms and systems implemented in the flight controller firmware.

---

## 📋 Table of Contents

1. [Mahony AHRS Filter](#mahony-ahrs-filter)
2. [PID Control Theory](#pid-control-theory)
3. [Cascade Control Architecture](#cascade-control-architecture)
4. [Motor Mixing Mathematics](#motor-mixing-mathematics)
5. [Altitude Estimation](#altitude-estimation)
6. [Low-Pass Filters](#low-pass-filters)
7. [Safety System Logic](#safety-system-logic)
8. [Timing and Loop Structure](#timing-and-loop-structure)

---

## 🔄 Mahony AHRS Filter

### Overview

The Mahony filter is a complementary filter implemented in quaternion space. It fuses high-frequency gyroscope data with low-frequency accelerometer data to estimate attitude.

### Why Quaternions?

Quaternions avoid gimbal lock and provide smooth interpolation:

```
Quaternion: q = q₀ + q₁i + q₂j + q₃k

Where:
  q₀ = scalar (real) part
  q₁, q₂, q₃ = vector (imaginary) parts
  
Unit quaternion constraint: q₀² + q₁² + q₂² + q₃² = 1
```

### Algorithm Steps

```
MAHONY AHRS ALGORITHM
═════════════════════

Input: Gyroscope (ωx, ωy, ωz) in rad/s
       Accelerometer (ax, ay, az) in g
       Time step dt

1. NORMALIZE ACCELEROMETER
   ────────────────────────
   magnitude = √(ax² + ay² + az²)
   
   if magnitude is valid (0.1g to 2g):
       ax /= magnitude
       ay /= magnitude
       az /= magnitude

2. ESTIMATE GRAVITY DIRECTION FROM QUATERNION
   ──────────────────────────────────────────
   The current quaternion estimates where "down" is.
   Extract gravity vector from rotation matrix:
   
   vx = 2(q₁q₃ - q₀q₂)
   vy = 2(q₀q₁ + q₂q₃)
   vz = q₀² - ½ + q₃²

3. CALCULATE ERROR (Cross Product)
   ────────────────────────────────
   Error between measured and estimated gravity:
   
   ex = ay·vz - az·vy
   ey = az·vx - ax·vz
   ez = ax·vy - ay·vx

4. INTEGRAL FEEDBACK (Gyro Bias Compensation)
   ──────────────────────────────────────────
   integralFB += Ki · e · dt
   
   (with anti-windup limiting)

5. APPLY CORRECTIONS TO GYRO
   ─────────────────────────
   ω_corrected = ω + Kp·e + integralFB

6. INTEGRATE QUATERNION
   ─────────────────────
   Quaternion derivative:
   q̇ = ½ · q ⊗ [0, ωx, ωy, ωz]
   
   Update:
   q += q̇ · dt
   
   Normalize:
   q /= |q|

7. EXTRACT EULER ANGLES
   ─────────────────────
   roll  = atan2(2(q₀q₁ + q₂q₃), 1 - 2(q₁² + q₂²))
   pitch = asin(2(q₀q₂ - q₃q₁))
   yaw   = atan2(2(q₀q₃ + q₁q₂), 1 - 2(q₂² + q₃²))
```

### Tuning Parameters

| Parameter | Default | Effect |
|-----------|---------|--------|
| Kp | 10.0 | Higher = faster convergence, more noise sensitivity |
| Ki | 0.005 | Higher = faster bias compensation, risk of oscillation |

### Implementation Notes

```cpp
// Key code from ahrs.h

void updateMahony(float gx, float gy, float gz, 
                  float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    
    // Normalize accelerometer
    recipNorm = 1.0f / sqrt(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // Estimated direction of gravity (from quaternion)
    halfvx = q1*q3 - q0*q2;
    halfvy = q0*q1 + q2*q3;
    halfvz = q0*q0 - 0.5f + q3*q3;
    
    // Error (cross product)
    halfex = ay*halfvz - az*halfvy;
    halfey = az*halfvx - ax*halfvz;
    halfez = ax*halfvy - ay*halfvx;
    
    // Apply feedback
    gx += MAHONY_KP * halfex;
    gy += MAHONY_KP * halfey;
    gz += MAHONY_KP * halfez;
    
    // Integrate quaternion
    // ... (quaternion update math)
}
```

---

## 🎛️ PID Control Theory

### The PID Equation

```
                     t
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·(de/dt)
                     0

Where:
  u(t) = Control output
  e(t) = Error (setpoint - measurement)
  Kp   = Proportional gain
  Ki   = Integral gain
  Kd   = Derivative gain
```

### Discrete Implementation

```
PID DISCRETE FORM
═════════════════

At each time step n with sample time dt:

Error:
  e[n] = setpoint - measurement

Proportional Term:
  P = Kp · e[n]

Integral Term (Trapezoidal):
  integral += Ki · e[n] · dt
  I = integral
  
  With anti-windup:
  if (integral > I_max) integral = I_max
  if (integral < -I_max) integral = -I_max

Derivative Term (on measurement to avoid kick):
  derivative = -(measurement[n] - measurement[n-1]) / dt
  D = Kd · derivative
  
  With low-pass filter:
  D_filtered = α · D_prev + (1-α) · D

Output:
  output = P + I + D
  
  With output limiting:
  output = constrain(output, -out_max, out_max)
```

### Anti-Windup Strategies

```
ANTI-WINDUP MECHANISMS
══════════════════════

1. INTEGRAL CLAMPING
   ──────────────────
   Simply limit integral accumulator:
   
   integral = constrain(integral, -I_max, I_max)
   
   Simple but can cause sluggish response.

2. BACK-CALCULATION
   ─────────────────
   When output saturates, reduce integral:
   
   if (output > out_max) {
       excess = output - out_max;
       integral -= excess * Kb;  // Kb = back-calc gain
       output = out_max;
   }
   
   Provides smoother transitions.

3. CONDITIONAL INTEGRATION
   ───────────────────────
   Only integrate when not saturated:
   
   if (abs(output) < out_max) {
       integral += Ki * error * dt;
   }
```

### Derivative on Measurement

```
WHY DERIVATIVE ON MEASUREMENT?
══════════════════════════════

Problem with derivative on error:
────────────────────────────────

When setpoint changes suddenly:
  e[n]   = new_setpoint - measurement
  e[n-1] = old_setpoint - measurement
  
  de/dt ≈ (e[n] - e[n-1]) / dt
        = (new_setpoint - old_setpoint) / dt  ← HUGE!

This causes "derivative kick" - a spike in output.

Solution - derivative on measurement:
───────────────────────────────────

  d/dt(measurement) = (meas[n] - meas[n-1]) / dt
  
  D_term = -Kd · d/dt(measurement)

The negative sign is because:
  d/dt(error) = d/dt(setpoint) - d/dt(measurement)
  
If setpoint is constant:
  d/dt(error) = -d/dt(measurement)
```

---

## 🔄 Cascade Control Architecture

### Structure

```
CASCADE PID STRUCTURE
═════════════════════

                 OUTER LOOP              INNER LOOP
              (Angle Control)         (Rate Control)
              
Stick      ┌──────────────┐        ┌──────────────┐
Input  ───►│   ANGLE PID  │──Rate──►│   RATE PID   │──────► Motor
(angle)    │   (100 Hz)   │ Cmd    │   (400 Hz)   │        Output
           └──────┬───────┘        └──────┬───────┘
                  │                       │
                  │ Angle                 │ Angular
                  │ Feedback              │ Rate
                  │                       │ Feedback
           ┌──────┴───────┐        ┌──────┴───────┐
           │     AHRS     │        │    GYRO      │
           │   (Angles)   │        │   (Rates)    │
           └──────────────┘        └──────────────┘


WHY CASCADE?
════════════

1. BETTER DISTURBANCE REJECTION
   The inner loop rejects disturbances before they 
   significantly affect the outer loop.

2. EASIER TUNING
   Tune inner loop first (simpler dynamics)
   Then tune outer loop independently.

3. SEPARATE TIME SCALES
   Inner loop handles fast dynamics (rate changes)
   Outer loop handles slow dynamics (position/angle)

4. IMPROVED STABILITY
   Inner loop stabilizes the system first
   Outer loop provides setpoints to a stable inner system
```

### Loop Rates

```
LOOP RATE SELECTION
═══════════════════

Main Loop: 400 Hz (2500 µs period)
│
├── Rate PID:     Every loop     = 400 Hz
│                 (fastest for best disturbance rejection)
│
├── Angle PID:    Every 4 loops  = 100 Hz
│                 (slower dynamics, doesn't need 400 Hz)
│
├── Altitude:     Every 8 loops  = 50 Hz
│                 (barometer is slow, ~20-50 Hz max)
│
├── Barometer:    Every 20 loops = 20 Hz
│                 (MS5611 conversion time ~10ms per reading)
│
└── Telemetry:    Every 40 loops = 10 Hz
                  (don't need fast updates for display)
```

---

## ⚙️ Motor Mixing Mathematics

### X-Configuration Geometry

```
X-CONFIGURATION QUADCOPTER
══════════════════════════

Top View:

              +X (Forward/Pitch+)
                    ▲
                    │
        FL          │          FR
          ╲         │         ╱
            ╲  45°  │  45°  ╱
              ╲     │     ╱
                ╲   │   ╱
                  ╲ │ ╱
    +Y ◄───────────●───────────► -Y
    (Roll+)      ╱ │ ╲        (Roll-)
                ╱   │   ╲
              ╱     │     ╲
            ╱  45°  │  45°  ╲
          ╱         │         ╲
        RL          │          RR
                    │
                    ▼
              -X (Backward/Pitch-)


Motor Positions (normalized):
  FL: (+0.707, +0.707)  →  +Roll, +Pitch
  FR: (+0.707, -0.707)  →  -Roll, +Pitch
  RR: (-0.707, -0.707)  →  -Roll, -Pitch
  RL: (-0.707, +0.707)  →  +Roll, -Pitch
```

### Mixing Equations

```
MOTOR MIXING MATRIX
═══════════════════

For X-configuration:

┌────┐   ┌                        ┐ ┌─────────┐
│ FL │   │ +1   +1   +1   -1      │ │Throttle │
│ FR │ = │ +1   -1   +1   +1      │ │ Roll    │
│ RR │   │ +1   -1   -1   -1      │ │ Pitch   │
│ RL │   │ +1   +1   -1   +1      │ │ Yaw     │
└────┘   └                        ┘ └─────────┘

Expanded:
  FL = Throttle + Roll + Pitch - Yaw
  FR = Throttle - Roll + Pitch + Yaw
  RR = Throttle - Roll - Pitch - Yaw
  RL = Throttle + Roll - Pitch + Yaw


YAW MOMENT GENERATION
═════════════════════

Motor rotation creates torque reaction:
  CCW motor → CW torque on frame
  CW motor  → CCW torque on frame

For positive yaw (CW rotation of drone):
  Need net CCW torque on frame
  Speed up CW motors (FR, RL)
  Slow down CCW motors (FL, RR)
  
Hence Yaw signs: FL-, FR+, RR-, RL+
```

### Dynamic Throttle Scaling

```
THROTTLE PRIORITY MIXING
════════════════════════

Problem: Roll/Pitch/Yaw commands can push motors
         above maximum or below minimum.

Solution: Scale control commands to fit within limits
          while prioritizing throttle.

Algorithm:
──────────

1. Calculate raw motor values
   m[i] = throttle + mix[i] * controls

2. Find max and min motor values
   m_max = max(m[0], m[1], m[2], m[3])
   m_min = min(m[0], m[1], m[2], m[3])

3. If max exceeds limit, scale down controls:
   if (m_max > MOTOR_MAX) {
       scale = (MOTOR_MAX - throttle) / (m_max - throttle)
       for each motor:
           m[i] = throttle + (m[i] - throttle) * scale
   }

4. Ensure minimum motor value:
   for each motor:
       if (m[i] < MOTOR_IDLE) m[i] = MOTOR_IDLE

This maintains control authority while respecting limits.
```

---

## 📊 Altitude Estimation

### Sensor Fusion Challenge

```
ALTITUDE SENSING COMPARISON
═══════════════════════════

Barometer (MS5611):
  ✓ Absolute altitude reference
  ✓ Low drift over time
  ✗ Slow response (~10-50 Hz)
  ✗ Affected by wind, prop wash
  ✗ ~10cm resolution typical

Accelerometer (Vertical):
  ✓ Fast response (400 Hz)
  ✓ Good for short-term changes
  ✗ Drift when integrated (no absolute reference)
  ✗ Affected by attitude errors
  ✗ Noise accumulates in double integration
```

### Complementary Filter

```
COMPLEMENTARY FILTER FOR ALTITUDE
═════════════════════════════════

Concept: Use barometer for low-frequency (DC/slow)
         Use accelerometer for high-frequency (fast changes)

                 Barometer
                     │
              ┌──────▼──────┐
              │  Low-Pass   │
              │   Filter    │
              │  (α ≈ 0.98) │
              └──────┬──────┘
                     │
                     ▼
              ┌──────────────┐
              │      +       │──────► Altitude
              └──────────────┘        Estimate
                     ▲
                     │
              ┌──────┴──────┐
              │  High-Pass  │
              │   Filter    │
              │ (1-α ≈ 0.02)│
              └──────┬──────┘
                     │
              Accelerometer
              (double integrated)


Mathematical Form:
──────────────────

h_est[n] = α · h_baro[n] + (1-α) · (h_est[n-1] + v[n-1]·dt + ½·a·dt²)

Where:
  α = 0.98 (barometer weight)
  h_baro = barometer altitude
  v = estimated vertical velocity
  a = vertical acceleration (corrected for attitude)
  dt = time step

Velocity estimate:
  v[n] = v[n-1] + a·dt + K·(h_baro - h_pred)

The K term provides velocity correction from barometer.
```

### Vertical Acceleration Extraction

```
EXTRACTING VERTICAL ACCELERATION
════════════════════════════════

Problem: Accelerometer measures in BODY frame
         We need acceleration in WORLD frame (vertical)

Solution: Use rotation matrix from AHRS

Body frame acceleration: a_body = [ax, ay, az]

Rotation matrix from quaternion:
       ┌                                         ┐
  R =  │ 1-2(q₂²+q₃²)   2(q₁q₂-q₀q₃)   2(q₁q₃+q₀q₂) │
       │ 2(q₁q₂+q₀q₃)   1-2(q₁²+q₃²)   2(q₂q₃-q₀q₁) │
       │ 2(q₁q₃-q₀q₂)   2(q₂q₃+q₀q₁)   1-2(q₁²+q₂²) │
       └                                         ┘

World Z (vertical) acceleration:
  a_world_z = R[2][0]·ax + R[2][1]·ay + R[2][2]·az

Subtract gravity (1g pointing down):
  a_vertical = a_world_z - 1.0

Result is vertical acceleration in g units.
Convert to cm/s²: a_cm = a_vertical × 980.665
```

---

## 📉 Low-Pass Filters

### First-Order IIR Filter

```
EXPONENTIAL MOVING AVERAGE (EMA)
════════════════════════════════

Also called: First-order IIR, RC filter, Alpha filter

Equation:
  y[n] = α · y[n-1] + (1-α) · x[n]

Where:
  x[n] = new input sample
  y[n] = filtered output
  α = smoothing factor (0 to 1)

Properties:
  α close to 1: More smoothing, slower response
  α close to 0: Less smoothing, faster response

Cutoff Frequency:
  fc = fs × (1-α) / (2π × α)
  
Where fs = sample frequency


IMPLEMENTATION IN CODE
══════════════════════

// Simple implementation
float filter(float newValue, float oldFiltered, float alpha) {
    return alpha * oldFiltered + (1.0f - alpha) * newValue;
}

// Usage
gyroFiltered = filter(gyroRaw, gyroFiltered, 0.8);
```

### Filter Selection Guide

```
RECOMMENDED FILTER COEFFICIENTS
═══════════════════════════════

Signal              Alpha    Notes
───────────────     ─────    ─────
Gyroscope           0.8      Balance noise vs. responsiveness
Accelerometer       0.9      More smoothing needed
Barometer           0.7      Significant smoothing for noise
Motor output        0.0      No filtering (immediate response)
Derivative (PID)    0.7      Smooth derivative, reduce jitter

Choosing Alpha:
───────────────
Higher alpha (0.9+):
  • Slower response
  • Smoother output
  • More latency
  
Lower alpha (0.5-):
  • Faster response
  • More noise passes through
  • Less latency
```

---

## 🛡️ Safety System Logic

### State Machine

```
FLIGHT CONTROLLER STATE MACHINE
═══════════════════════════════

         ┌───────────────────────────────────────────┐
         │                                           │
         ▼                                           │
    ┌─────────┐     sensors OK      ┌───────────┐   │
    │  INIT   │────────────────────►│ PREFLIGHT │   │
    └─────────┘                     └─────┬─────┘   │
         │                                │         │
         │ sensor error                   │ arm     │
         ▼                                ▼         │
    ┌─────────┐                     ┌───────────┐   │
    │  ERROR  │◄────────────────────│   ARMED   │   │
    └─────────┘     error           └─────┬─────┘   │
         ▲                                │         │
         │                                │ throttle│
         │                                ▼         │
         │                          ┌───────────┐   │
         │                          │  FLYING   │   │
         │                          └─────┬─────┘   │
         │                                │         │
         │ cannot recover                 │ link lost
         │                                │ tilt > 60°
         │                                ▼         │
         │                          ┌───────────┐   │
         └──────────────────────────│ FAILSAFE  │───┘
                                    └───────────┘
                                          │
                                          │ link restored
                                          │ must re-arm
                                          ▼
                                    ┌───────────┐
                                    │ PREFLIGHT │
                                    └───────────┘
```

### Failsafe Logic

```
FAILSAFE CONDITIONS AND ACTIONS
═══════════════════════════════

1. LINK LOSS FAILSAFE
   ───────────────────
   Condition: No valid packet for > 500ms
   Action:    Immediate motor cutoff
   Recovery:  Link must be restored, must re-arm
   
   Code:
   if (millis() - lastPacketTime > FAILSAFE_TIMEOUT_MS) {
       motors.emergencyStop();
       systemState = STATE_FAILSAFE;
   }

2. TILT FAILSAFE
   ──────────────
   Condition: |roll| > 60° OR |pitch| > 60°
   Action:    Immediate motor cutoff
   Recovery:  Return to level, must re-arm
   
   Code:
   if (abs(ahrs.roll) > MAX_SAFE_TILT || 
       abs(ahrs.pitch) > MAX_SAFE_TILT) {
       motors.emergencyStop();
       systemState = STATE_FAILSAFE;
   }

3. KILL SWITCH (SW2)
   ──────────────────
   Condition: SW2 switched to OFF
   Action:    Immediate motor cutoff
   Recovery:  Switch SW2 ON with throttle low
   
   Code:
   if (!nrf.isArmed()) {
       motors.disarm();
   }

4. EMERGENCY FLAG
   ───────────────
   Condition: Emergency flag in packet
   Action:    Immediate motor cutoff
   Recovery:  Clear flag, must re-arm
```

### Arming Checks

```
ARMING REQUIREMENTS
═══════════════════

bool canArm() {
    // 1. No system error
    if (systemState == STATE_ERROR) return false;
    
    // 2. Communication link active
    if (!nrf.isLinked()) return false;
    
    // 3. IMU calibrated
    if (!imuCalibrated) return false;
    
    // 4. Throttle at minimum
    if (throttleCommand > THROTTLE_IDLE + 50) return false;
    
    // 5. Drone approximately level
    if (abs(ahrs.roll) > 10.0f) return false;
    if (abs(ahrs.pitch) > 10.0f) return false;
    
    return true;
}

These checks prevent:
  • Arming with sensors not ready
  • Arming without control link
  • Arming with throttle up (dangerous!)
  • Arming when tilted (would flip immediately)
```

---

## ⏱️ Timing and Loop Structure

### Main Loop Timing

```
MAIN LOOP STRUCTURE (2500 µs / 400 Hz)
══════════════════════════════════════

Time (µs)    Task
─────────    ────────────────────────────────
0            Loop start, record timestamp
             
50-150       Read IMU (I2C burst read)
             • 14 bytes via I2C @ 400kHz
             • ~150 µs typical
             
150-200      Check NRF for packets
             • Quick register check
             • Read if available
             
200-400      AHRS update
             • Mahony filter math
             • ~150-200 µs with float math
             
400-600      Control loops
             • Rate PID always
             • Angle PID every 4th loop
             • Altitude every 8th loop
             
600-800      Motor mixing
             • Mix calculations
             • PWM output
             
800-1000     Safety checks
             • Link timeout
             • Tilt check
             • Emergency flags
             
1000-1200    Housekeeping
             • Buzzer update
             • LED update
             • Telemetry build
             
1200-2500    Wait for next loop
             • Precise timing
             • Prevents overrun
             
2500         Next loop starts


TIMING ENFORCEMENT
══════════════════

uint32_t loopStartTime = micros();

// ... all processing ...

uint32_t elapsed = micros() - loopStartTime;
if (elapsed < LOOP_PERIOD_US) {
    delayMicroseconds(LOOP_PERIOD_US - elapsed);
}

This ensures consistent 400 Hz regardless of
processing time variations.
```

### Critical Timing Requirements

```
TIMING REQUIREMENTS
═══════════════════

Component          Requirement           Reason
─────────          ───────────           ──────
IMU Read           < 500 µs              Leave time for processing
AHRS Update        Every loop            Prevent integration drift  
Rate PID           Every loop (400 Hz)   Fast disturbance rejection
Angle PID          100 Hz minimum        Stable outer loop
Motor Update       > 250 Hz              ESC responsiveness
Barometer          20-50 Hz              Sensor conversion time
Communication      10-100 Hz             Latency vs. bandwidth
Failsafe Check     Every loop            Safety critical
```

---

## 📚 References

1. Mahony, R., Hamel, T., & Pflimlin, J. M. (2008). Nonlinear complementary filters on the special orthogonal group. IEEE Transactions on Automatic Control.

2. Madgwick, S. O. H. (2010). An efficient orientation filter for inertial and inertial/magnetic sensor arrays. Report x-io Technologies.

3. Åström, K. J., & Hägglund, T. (1995). PID Controllers: Theory, Design, and Tuning. ISA.

4. MS5611 Datasheet - Measurement Specialties

5. MPU-6050 Register Map - InvenSense

---

*For questions or improvements, please open an issue in the repository.*
