# Arduino Nano Quadcopter Drone Project

A complete DIY quadcopter drone project using Arduino Nano for both flight controller and remote controller.

## 🚁 Project Overview

This project implements a fully functional quadcopter with:
- **Auto-stabilization** using PID control
- **Complementary filter** for sensor fusion
- **Wireless control** via nRF24L01+ (up to 100m range)
- **Dual flight modes**: ANGLE (self-leveling) and ACRO (manual)
- **Altitude hold** using barometric pressure sensor
- **Failsafe protection** for signal loss

---

## 📦 Components Required

### A) Flight Controller
| Component | Purpose | Quantity |
|-----------|---------|----------|
| Arduino Nano | Main flight controller | 1 |
| MPU6050 | 6-axis gyro + accelerometer (tilt/rotation) | 1 |
| MS5611 | Barometric pressure sensor (altitude) | 1 |
| nRF24L01+ | 2.4GHz wireless receiver | 1 |
| Brushless Motors | 1000-1500 KV rating | 4 |
| ESC (Electronic Speed Controller) | 20-30A | 4 |
| LiPo Battery | 11.1V 3S 2200mAh | 1 |
| Buzzer | Status indication | 1 |
| LED | Visual feedback | 1 |

### B) Remote Controller
| Component | Purpose | Quantity |
|-----------|---------|----------|
| Arduino Nano | Remote controller brain | 1 |
| nRF24L01+ | 2.4GHz wireless transmitter | 1 |
| Dual-axis Joysticks | Flight control | 2 |
| Push Buttons | Function buttons | 4 |
| Toggle Switches | Mode switches | 2 |
| Battery | 9V or 3x AA | 1 |

---

## 🔌 Complete Wiring Diagrams

### Flight Controller Connections

```
Arduino Nano Flight Controller
================================

I2C Bus (Sensors):
  A4 (SDA) ──┬── MPU6050 SDA
             └── MS5611 SDA
  A5 (SCL) ──┬── MPU6050 SCL
             └── MS5611 SCL

Motor Outputs (PWM):
  D3 ── ESC 1 (Front Left)  [Motor spins CCW]
  D5 ── ESC 2 (Front Right) [Motor spins CW]
  D6 ── ESC 3 (Rear Right)  [Motor spins CCW]
  D9 ── ESC 4 (Rear Left)   [Motor spins CW]

nRF24L01+ Radio (SPI):
  D4  ── CE (Chip Enable)
  D10 ── CSN (Chip Select)
  D11 ── MOSI
  D12 ── MISO
  D13 ── SCK
  3.3V ── VCC (IMPORTANT: Use 3.3V!)
  GND ── GND
  
  ⚠️ CRITICAL: Add 10µF capacitor between VCC and GND!
             (Soldered directly to nRF24 pins)
             This prevents "Transmission failed" errors!

Status Indicators:
  D7 ── Buzzer (+)
  D8 ── LED (+)

Power:
  VIN ── 5V from BEC/ESC
  GND ── Common Ground
```

### Remote Controller Connections

```
Arduino Nano Remote Controller
================================

Joysticks (Analog):
  A0 ── Left Joystick Y-axis (Throttle)
  A1 ── Left Joystick X-axis (Yaw)
  A2 ── Right Joystick Y-axis (Pitch)
  A3 ── Right Joystick X-axis (Roll)

Control Buttons (Digital with pull-up):
  D4 ── Button 1 (Calibrate Sensors)
  D5 ── Button 2 (Motor Test)
  D6 ── Button 3 (ARM/DISARM)
  D7 ── Button 4 (Soft Landing)

Mode Switches (Digital with pull-up):
  D2 ── Switch 1 (Reserved)
  D3 ── Switch 2 (ANGLE/ACRO Mode)

nRF24L01+ Radio (SPI):
  D10 ── CSN (Chip Select)
  D9  ── CE (Chip Enable)
  D11 ── MOSI
  D12 ── MISO
  D13 ── SCK
  3.3V ── VCC (IMPORTANT: Use 3.3V!)
  GND ── GND
  
  ⚠️ CRITICAL: Add 10µF capacitor between VCC and GND!
             (Soldered directly to nRF24 pins)
             This prevents "Transmission failed" errors!

Power:
  VIN ── 9V Battery (+)
  GND ── Battery (-)
```

### Motor Configuration (X-Frame)

```
        FRONT
         [↑]
    
   M1 ↺     ↻ M2
      \  X  /
       \   /
       /   \
      /     \
   M4 ↻     ↺ M3

    REAR
    
M1 = Front Left  (CCW - Counter-Clockwise)
M2 = Front Right (CW  - Clockwise)
M3 = Rear Right  (CCW - Counter-Clockwise)
M4 = Rear Left   (CW  - Clockwise)
```

---

## 🧠 How Each Component Works

### 1. MPU6050 - Inertial Measurement Unit (IMU)

**What it does:** Acts as the "inner ear" of your drone - detects tilt and rotation.

**Components:**
- **3-axis Gyroscope**: Measures rotation speed (deg/s) on X, Y, Z axes
- **3-axis Accelerometer**: Measures acceleration and gravity direction (g-force)

**How it works:**

1. **Gyroscope** - Detects rotation rate
   - Measures: "How fast am I rotating right now?"
   - Units: Degrees per second (°/s)
   - Problem: Drifts over time (integrating rotation rates accumulates error)

2. **Accelerometer** - Detects gravity direction
   - Measures: "Which way is down (gravity)?"
   - Units: g-force (1g = 9.8 m/s²)
   - Problem: Sensitive to vibrations and movement

3. **Sensor Fusion (Complementary Filter)**
   - Combines both sensors to get accurate, stable angle
   - Formula: `angle = 0.98 × (angle + gyro×dt) + 0.02 × accel_angle`
   - 98% gyro (short-term accuracy) + 2% accel (long-term correction)

**Example Scenario: Nose Down**
```
Physical Event: Wind pushes nose down
├─ Gyroscope: "Rotating forward at 50°/s!"
├─ Accelerometer: "Gravity shifted - we're at -15° pitch!"
├─ Arduino: "Error = -15° (current) vs 0° (desired)"
└─ Motors: Front motors speed UP, rear motors slow DOWN
Result: Nose lifts back to level
```

### 2. MS5611 - Barometric Pressure Sensor

**What it does:** Measures altitude using air pressure.

**How it works:**
- Air pressure decreases as altitude increases
- At sea level: ~1013 mbar
- At 100m higher: ~1001 mbar (12 mbar difference)
- Formula: `altitude = 44330 × (1 - (P/P0)^0.1903)`

**Use cases:**
- Altitude hold mode (maintain height automatically)
- Smooth takeoff and landing
- Return-to-home height reference

### 3. nRF24L01+ - Wireless Radio

**What it does:** 2.4GHz wireless communication between remote and drone.

**Specifications:**
- Frequency: 2.4 GHz (125 channels)
- Range: Up to 100m (line of sight)
- Data rate: 250kbps - 2Mbps
- Power: 3.3V only!

**How it works:**
1. Remote packs joystick/button data into structure
2. Transmits packet every 20ms (50Hz update rate)
3. Flight controller receives and validates data
4. If no signal for 1 second → FAILSAFE mode (emergency landing)

### 4. PID Controller - Stabilization Algorithm

**What it does:** Automatically corrects drone orientation to desired angle.

**Three Components:**

1. **P (Proportional)**: Reacts to current error
   - `P_output = Kp × error`
   - Higher Kp = stronger but potentially oscillating response

2. **I (Integral)**: Corrects accumulated error over time
   - `I_output = Ki × sum(error × dt)`
   - Eliminates steady-state errors (drift)

3. **D (Derivative)**: Predicts future error based on rate of change
   - `D_output = Kd × (error - previous_error) / dt`
   - Reduces overshoot and oscillation

**Example Values:**
```cpp
// Roll/Pitch PID gains
Kp = 1.5  // Strong response to tilt
Ki = 0.05 // Gentle drift correction
Kd = 0.8  // Dampen oscillations

// Yaw PID gains
Kp = 3.0  // Stronger for rotation
Ki = 0.02
Kd = 0.5
```

---

## 🎮 Control Mapping

### Joysticks

| Stick | Axis | Function | Effect |
|-------|------|----------|--------|
| Left | Y (up/down) | **Throttle** | Altitude control |
| Left | X (left/right) | **Yaw** | Rotate left/right |
| Right | Y (up/down) | **Pitch** | Forward/backward tilt |
| Right | X (left/right) | **Roll** | Left/right tilt |

### Buttons

| Button | Function | Description |
|--------|----------|-------------|
| Button 1 (D4) | **Calibrate** | Calibrate gyro and accelerometer on flat surface |
| Button 2 (D5) | **Motor Test** | Test all motors at low speed (no props!) |
| Button 3 (D6) | **ARM** | Enable motors for flight (throttle must be low) |
| Button 4 (D7) | **Soft Landing** | Automatically descend and land smoothly |

### Switches

| Switch | Function | States |
|--------|----------|--------|
| Switch 1 (D2) | Reserved | Future feature |
| Switch 2 (D3) | **Flight Mode** | ANGLE (self-leveling) / ACRO (manual) |

---

## ✈️ Complete Flight Sequence

### 1. Pre-Flight Checklist
```
☐ Battery fully charged (11.1V)
☐ Propellers installed correctly (check rotation direction)
☐ Flight controller mounted level
☐ Remote control powered on
☐ Clear flight area (outdoors, no obstacles)
```

### 2. Startup Sequence

```
Step 1: Power on remote controller
  └─ LED should light up
  └─ Check all joysticks center at ~512

Step 2: Place drone on FLAT surface
  └─ Must be perfectly level for calibration

Step 3: Power on drone (connect battery)
  └─ LED flashes rapidly (initializing)
  └─ Buzzer beeps 3 times (sensors detected)
  └─ LED solid (ready for calibration)

Step 4: Press Button 1 (Calibrate)
  └─ Keep drone perfectly still!
  └─ Buzzer beeps once (calibrating...)
  └─ LED blinks slowly (calculating sensor offsets)
  └─ Buzzer beeps twice (calibration complete)
```

### 3. Arming Motors

```
Step 1: Ensure throttle stick is DOWN (minimum)
Step 2: Press Button 3 (ARM)
  └─ Buzzer: Long beep
  └─ LED: Solid on
  └─ Motors: Armed (ready to spin)

Note: Motors will NOT arm if:
  - Throttle is not at minimum
  - Sensors not calibrated
  - Radio signal lost
```

### 4. Takeoff

```
Step 1: Slowly increase throttle (left stick up)
  └─ ~40-50% throttle to hover
  
Step 2: Drone lifts off
  └─ PID stabilization automatically corrects tilt
  └─ Hold joysticks centered for stable hover

Step 3: Make small adjustments
  └─ Right stick: Pitch/Roll (move around)
  └─ Left stick horizontal: Yaw (rotate)
```

### 5. Flight Modes

**ANGLE Mode (Recommended for beginners)**
```
- Self-leveling enabled
- Release sticks → drone returns to level
- Max tilt angle limited to ±30°
- Easier to control, more stable
```

**ACRO Mode (Advanced)**
```
- No self-leveling
- Sticks control rotation rate, not angle
- Full 360° flips possible
- Requires skilled pilot
```

Switch between modes with Switch 2 (D3)

### 6. Landing

**Manual Landing:**
```
Step 1: Position drone above landing spot
Step 2: Slowly reduce throttle
Step 3: Touch down gently
Step 4: Throttle to minimum (disarms automatically)
```

**Automatic Landing (Button 4):**
```
Step 1: Press Button 4 (Soft Landing)
Step 2: Drone gradually reduces throttle
Step 3: Descends at controlled rate (~0.5 m/s)
Step 4: Motors disarm on touchdown
```

### 7. Emergency Procedures

**Signal Loss (Failsafe):**
```
If radio signal lost for >1 second:
├─ Buzzer: Continuous beeping
├─ Throttle: Reduced to 30%
├─ Action: Descends slowly
└─ Recovery: Regain signal or land manually
```

**Low Battery (if voltage monitoring added):**
```
If battery < 10.5V (3.5V per cell):
├─ Buzzer: Intermittent beeps
├─ LED: Rapid flashing
└─ Action: Land immediately!
```

**Crash/Tilt >45°:**
```
If drone tilts beyond 45°:
├─ Motors: Immediately disarm
├─ Safety: Prevents runaway
└─ Recovery: Place level, recalibrate, rearm
```

---

## 🔧 Setup Instructions

### 1. Install Arduino IDE

Download from: https://www.arduino.cc/en/software

### 2. Install Required Libraries

Open Arduino IDE → Sketch → Include Library → Manage Libraries

Install the following:
- **Wire** (built-in, for I2C)
- **Servo** (built-in, for ESC control)
- **RF24** by TMRh20 (for nRF24L01+)
- **Adafruit MPU6050** by Adafruit (will auto-install dependencies)
- **MS5611** by Rob Tillaart

### 3. Upload Code

**Flight Controller:**
```
1. Open: FlightController/FlightController.ino
2. Select: Tools → Board → Arduino Nano
3. Select: Tools → Processor → ATmega328P (Old Bootloader)
4. Select: Tools → Port → [Your COM port]
5. Click: Upload
```

**Remote Controller:**
```
1. Open: RemoteController/RemoteController.ino
2. Follow same steps as above
3. Click: Upload
```

### 4. First Time Calibration

```
1. Place drone on perfectly flat surface
2. Power on remote, then drone
3. Press Button 1 (Calibrate) on remote
4. Wait for confirmation beeps
5. Ready to fly!
```

---

## 📊 Understanding the Stabilization

### How IMU Detects Motion

| Scenario | IMU Detects | Motor Response | Result |
|----------|-------------|----------------|--------|
| **Nose Down** | Pitch: -20° | Front ↑, Rear ↓ | Nose lifts |
| **Nose Up** | Pitch: +20° | Front ↓, Rear ↑ | Nose drops |
| **Tilt Left** | Roll: -20° | Left ↑, Right ↓ | Levels right |
| **Tilt Right** | Roll: +20° | Left ↓, Right ↑ | Levels left |
| **Spin CW** | Yaw rate: +50°/s | CW motors ↓, CCW ↑ | Stops spin |

### Complementary Filter Explained

**Problem:** 
- Gyro drifts over time ❌
- Accelerometer is noisy ❌

**Solution:**
- Use gyro for short-term (smooth, responsive)
- Use accelerometer for long-term (drift correction)

**Formula:**
```cpp
// 98% trust gyro, 2% trust accelerometer
angle = 0.98 × (previous_angle + gyro_rate × dt) + 0.02 × accel_angle
```

**Why it works:**
- Gyro provides fast updates (no lag)
- Accelerometer prevents long-term drift
- Result: Smooth + accurate angles

---

## 🛠️ Tuning Guide

### PID Tuning (Start with these values)

```cpp
// Roll/Pitch
float Kp_roll = 1.5;
float Ki_roll = 0.05;
float Kd_roll = 0.8;

// Yaw
float Kp_yaw = 3.0;
float Ki_yaw = 0.02;
float Kd_yaw = 0.5;
```

### Tuning Process

**If drone oscillates (shaking):**
- Reduce Kp
- Increase Kd

**If drone drifts slowly:**
- Increase Ki (carefully!)

**If response is sluggish:**
- Increase Kp
- Reduce Kd

### Complementary Filter Tuning

```cpp
// Default: 98% gyro, 2% accelerometer
float alpha = 0.98;

// If angles drift: Increase accelerometer influence
alpha = 0.96; // (96% gyro, 4% accel)

// If angles are jittery: Increase gyro influence
alpha = 0.99; // (99% gyro, 1% accel)
```

---

## ⚠️ Safety Guidelines

### CRITICAL SAFETY RULES

1. **Remove propellers during testing** (motor test, calibration)
2. **Always test indoors first** without props
3. **Never arm motors with props near people/objects**
4. **Use a LiPo bag** for battery charging
5. **Never discharge LiPo below 3.3V per cell**
6. **Wear safety glasses** during test flights
7. **Keep fingers away** from spinning propellers
8. **Fly outdoors only** in open areas
9. **Check propeller tightness** before each flight
10. **Land immediately** if anything feels wrong

---

## 📁 Project Structure

```
quadcopter-drone/
├── README.md (this file)
├── FlightController/
│   └── FlightController.ino (main flight controller code)
├── RemoteController/
│   └── RemoteController.ino (remote controller code)
├── docs/
│   ├── WIRING_DIAGRAM.md (detailed wiring)
│   ├── COMPONENT_GUIDE.md (component explanations)
│   └── TROUBLESHOOTING.md (common issues)
└── libraries/
    └── requirements.txt (Arduino library list)
```

---

## 🐛 Troubleshooting

### Drone won't arm
- ✓ Check throttle is at minimum
- ✓ Verify calibration completed
- ✓ Ensure radio signal is strong

### Drone flips on takeoff
- ✓ Check motor directions (CW/CCW)
- ✓ Verify propeller installation
- ✓ Recalibrate on flat surface

### Drone drifts in one direction
- ✓ Recalibrate IMU
- ✓ Adjust PID gains (increase Ki)
- ✓ Check motor balance

### Radio connection lost
- ✓ Check nRF24 power (must be 3.3V)
- ✓ Add capacitor (10µF) across nRF24 power
- ✓ Reduce distance/obstacles

### Motors won't spin
- ✓ Check ESC connections
- ✓ Verify ESC calibration
- ✓ Test with motor test mode

---

## 📚 Further Learning

### Recommended Reading
1. **PID Control Theory** - Understanding tuning
2. **IMU Sensor Fusion** - Kalman filters, complementary filters
3. **Drone Physics** - Thrust, lift, torque
4. **Radio Communication** - Protocols, failsafe
5. **LiPo Battery Care** - Safety, charging, storage

### Next Steps
- Add GPS for autonomous waypoint navigation
- Implement optical flow for indoor stabilization
- Add FPV camera for first-person view
- Create mobile app for wireless control
- Implement return-to-home feature

---

## 📄 License

This project is open source and available under the MIT License.

## 🤝 Contributing

Contributions welcome! Please test thoroughly before submitting pull requests.

---

## 📞 Support

For issues or questions, please open an issue on the GitHub repository.

**Happy Flying! 🚁**
