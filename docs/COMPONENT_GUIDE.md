# Component Guide & Technical Explanations

## Table of Contents
1. [MPU6050 - Inertial Measurement Unit](#mpu6050---inertial-measurement-unit)
2. [MS5611 - Barometric Pressure Sensor](#ms5611---barometric-pressure-sensor)
3. [nRF24L01+ - Wireless Radio](#nrf24l01---wireless-radio)
4. [ESC - Electronic Speed Controller](#esc---electronic-speed-controller)
5. [Brushless Motors](#brushless-motors)
6. [LiPo Batteries](#lipo-batteries)
7. [Arduino Nano](#arduino-nano)

---

## MPU6050 - Inertial Measurement Unit

### What It Does
The MPU6050 is the "inner ear" of your drone. It senses orientation and motion in 3D space.

### Internal Components

#### 3-Axis Gyroscope
**Measures:** Rate of rotation (angular velocity)
**Units:** Degrees per second (°/s)
**Range:** ±250, ±500, ±1000, ±2000 °/s (configurable)

```
Rotation Axes:
         Z (Yaw)
         │
         │
         └───── Y (Pitch)
        ╱
       ╱
      X (Roll)

Roll  (X): Rotation around front-back axis (tilting left/right)
Pitch (Y): Rotation around left-right axis (nose up/down)
Yaw   (Z): Rotation around vertical axis (spinning)
```

**How It Works:**
- Uses vibrating mechanical structures
- Coriolis effect causes displacement when rotating
- Measures this displacement capacitively
- Very accurate short-term, but drifts over time

#### 3-Axis Accelerometer
**Measures:** Linear acceleration + gravity
**Units:** g-force (1g = 9.8 m/s²)
**Range:** ±2g, ±4g, ±8g, ±16g (configurable)

**How It Works:**
- Detects gravity's pull direction
- When stationary, measures which way is "down"
- During movement, measures acceleration forces
- Noisy but doesn't drift (gravity is constant)

### Sensor Fusion: Complementary Filter

The MPU6050's data needs processing because:
- **Gyro Problem:** Accurate short-term, drifts long-term
- **Accel Problem:** Knows true down, but very noisy

**Solution: Complementary Filter**

```cpp
// Combine best of both sensors
angle = alpha × (angle + gyro_rate × dt) + (1-alpha) × accel_angle

Where:
  alpha = 0.98 (trust gyro 98%, accel 2%)
  dt = time since last update
```

**Why It Works:**

```
Time Scale:          Gyro              Accelerometer
─────────────────────────────────────────────────────
Short term (ms):     ✅ Excellent       ❌ Noisy
Long term (sec):     ❌ Drifts          ✅ Accurate
```

The filter uses gyro for quick response, accel to prevent drift.

### Real-World Example

**Scenario: Drone Nose Drops Due to Wind**

1. **Gyroscope Detects:**
   - "Rotation on Y-axis: -50°/s"
   - Immediately knows direction and speed

2. **Accelerometer Detects:**
   - Gravity vector shifts from Z-axis toward X-axis
   - "New angle: -15° from level"

3. **Complementary Filter Calculates:**
   ```
   Previous angle: 0°
   Gyro prediction: 0° + (-50°/s × 0.004s) = -0.2°
   Accel measurement: -15°
   
   New angle = 0.98 × (-0.2°) + 0.02 × (-15°)
             = -0.196° + (-0.3°)
             = -0.496°
   ```

4. **Arduino Response:**
   - Detects error: -0.496° vs target 0°
   - PID controller calculates correction
   - Increases front motor speed, decreases rear
   - Drone nose lifts back up

### Configuration Used in Project

```cpp
// In Flight Controller code:
mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);   // ±500°/s
mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);   // ±4g
mpu.setDLPFMode(MPU6050_DLPF_BW_42);              // 42Hz filter
```

**Why These Settings?**
- ±500°/s: Good for acrobatic maneuvers without saturation
- ±4g: Captures flight accelerations, less noise than ±16g
- 42Hz DLPF: Filters vibrations from motors

### Common Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| Drift in angle | Accel weight too low | Increase accel influence (reduce alpha) |
| Jittery readings | Accel weight too high | Increase gyro influence (increase alpha) |
| No communication | Wrong I2C address | Check AD0 pin (LOW=0x68, HIGH=0x69) |
| Noisy data | Motor vibrations | Add foam dampening under sensor |

---

## MS5611 - Barometric Pressure Sensor

### What It Does
Measures atmospheric pressure to calculate altitude.

### How It Works

**Physics Principle:**
Air pressure decreases with altitude:
- Sea level: ~1013 mbar
- 1000m altitude: ~899 mbar
- 2000m altitude: ~795 mbar

**Formula Used:**
```
h = 44330 × (1 - (P/P₀)^0.1903)

Where:
  h = altitude in meters
  P = current pressure
  P₀ = reference pressure (at ground level)
```

### Practical Use in Drone

#### 1. Altitude Hold Mode
Maintains constant height automatically:
```
Target altitude: 5 meters
Current altitude: 4.8 meters
Error: +0.2 meters

PID controller increases throttle slightly
Drone rises back to 5 meters
```

#### 2. Smooth Takeoff/Landing
```
Takeoff sequence:
1. Set target altitude: +0.5m increments
2. PID gradually increases throttle
3. Smooth, controlled ascent

Landing sequence:
1. Reduce target altitude: -0.5m increments
2. PID gradually decreases throttle
3. Soft touchdown
```

### Specifications

| Parameter | Value |
|-----------|-------|
| Pressure range | 10-1200 mbar |
| Altitude range | -500m to +9000m |
| Resolution | 0.012 mbar (10cm altitude) |
| Sample rate | Up to 100Hz |
| Interface | I2C (address 0x77) |
| Voltage | 3.3V |

### Limitations

1. **Relative, Not Absolute**
   - Measures changes, not true altitude
   - Requires ground-level calibration

2. **Weather Sensitivity**
   - Pressure changes with weather
   - Need to recalibrate each session

3. **Indoor Drift**
   - HVAC systems change pressure
   - Less reliable indoors

4. **Temperature Compensation**
   - Built-in temperature sensor
   - Automatically corrects pressure readings

### Code Implementation

```cpp
// Calibration (in setup):
baseAltitude = ms5611.getAltitude(ms5611.readPressure());

// During flight:
currentAltitude = ms5611.getAltitude(ms5611.readPressure()) - baseAltitude;
```

---

## nRF24L01+ - Wireless Radio

### What It Does
2.4GHz wireless communication between remote and drone.

### Technical Specifications

| Feature | Specification |
|---------|---------------|
| Frequency | 2.4 GHz (2.400-2.525 GHz) |
| Channels | 125 channels (1 MHz spacing) |
| Data rate | 250 kbps, 1 Mbps, 2 Mbps |
| Range | Up to 100m (line of sight) |
| Power | 3.3V (±0.3V) |
| Current | 13.5mA receive, 115mA transmit |
| Packet size | 1-32 bytes |

### How It Works

#### Protocol
Uses Enhanced ShockBurst™ protocol:
1. Auto acknowledgment
2. Auto retransmission
3. Dynamic payload length
4. Multi-channel operation

#### Communication Flow

```
Remote Controller (TX)          Flight Controller (RX)
─────────────────────────────────────────────────────
1. Read joysticks/buttons
2. Pack into struct
3. Transmit packet ────────────>
                                 4. Receive packet
                                 5. Validate data
                                 6. Send ACK
7. Receive ACK <────────────────
8. Wait 20ms
9. Repeat
```

**Data Structure:**
```cpp
struct RadioData {
  int throttle;   // 0-1023 (2 bytes)
  int yaw;        // 0-1023 (2 bytes)
  int pitch;      // 0-1023 (2 bytes)
  int roll;       // 0-1023 (2 bytes)
  bool armed;     // (1 byte)
  bool calibrate; // (1 byte)
  bool motorTest; // (1 byte)
  bool softLand;  // (1 byte)
  bool angleMode; // (1 byte)
};
// Total: 13 bytes per packet
```

### Configuration in Project

```cpp
// Flight Controller (Receiver)
radio.openReadingPipe(1, address);
radio.setPALevel(RF24_PA_HIGH);      // Max power
radio.setDataRate(RF24_250KBPS);     // Longest range
radio.startListening();

// Remote Controller (Transmitter)
radio.openWritingPipe(address);
radio.setPALevel(RF24_PA_HIGH);
radio.setDataRate(RF24_250KBPS);
radio.stopListening();
```

### Range vs Data Rate

| Data Rate | Range | Use Case |
|-----------|-------|----------|
| 250 kbps | ~100m | Drone control (our choice) |
| 1 Mbps | ~50m | Balanced |
| 2 Mbps | ~30m | High-speed data |

### Power Amplifier Models

1. **Standard nRF24L01+**
   - Small PCB antenna
   - Range: ~30m
   - Cost: $1-2

2. **nRF24L01+ PA+LNA**
   - External antenna
   - Range: ~100m
   - Cost: $3-5
   - **Recommended for outdoor flight**

### Common Problems & Solutions

#### Problem 1: "Radio hardware not responding"
**Causes:**
- Wrong voltage (using 5V instead of 3.3V)
- Poor power supply (voltage drops during transmission)
- Faulty module

**Solutions:**
```
1. Check voltage: Must be 3.3V
2. Add 10µF capacitor between VCC and GND
3. Keep wires short (<10cm)
4. Try different module (they fail often)
```

#### Problem 2: Short range or packet loss
**Causes:**
- Interference (WiFi, other 2.4GHz devices)
- Antenna orientation
- Obstacles

**Solutions:**
```
1. Change channel: radio.setChannel(108); // Higher channels
2. Maximize power: radio.setPALevel(RF24_PA_HIGH);
3. Lower data rate: radio.setDataRate(RF24_250KBPS);
4. Keep antennas perpendicular (not parallel)
5. Use PA+LNA version
```

#### Problem 3: Random disconnections
**Causes:**
- Power supply noise
- Insufficient current

**Solutions:**
```
1. Add 10µF capacitor at module
2. Add 100µF capacitor at Arduino VIN
3. Use separate 3.3V regulator (LD1117V33)
```

### Failsafe Implementation

```cpp
// In flight controller code:
if (millis() - lastRadioTime > RADIO_TIMEOUT) {
  // Signal lost for >1 second
  failsafe();  // Emergency landing procedure
}
```

---

## ESC - Electronic Speed Controller

### What It Does
Converts DC battery voltage to 3-phase AC to drive brushless motors.

### How It Works

```
Input: DC from Battery (11.1V)
         ↓
    ESC Electronics
    ┌─────────────┐
    │ PWM Signal  │←─── Arduino (1000-2000µs pulses)
    │   Input     │
    │             │
    │ 3-Phase     │
    │ Bridge      │──┬──> Motor Wire A
    │ (6 MOSFETs) │  │
    │             │──┼──> Motor Wire B
    │             │  │
    │  + BEC      │──┴──> Motor Wire C
    │  (5V out)   │
    └─────────────┘
         ↓
    5V to Arduino
```

### PWM Signal Control

ESCs use standard servo protocol:
- 1000µs pulse = 0% throttle (stopped)
- 1500µs pulse = 50% throttle
- 2000µs pulse = 100% throttle (full speed)

**Arduino Code:**
```cpp
escMotor.writeMicroseconds(1500);  // 50% throttle
```

### ESC Calibration Procedure

**Why Needed:** Different ESCs may interpret PWM ranges differently.

**Steps:**
```
1. Disconnect all propellers
2. Power on remote, set throttle to MAXIMUM
3. Connect battery to ESC
4. ESC beeps (recognizing max throttle)
5. Lower throttle to MINIMUM
6. ESC beeps differently (calibration complete)
7. ESC is now calibrated to your PWM range
```

**Do this for ALL ESCs before first flight!**

### BEC (Battery Elimination Circuit)

Built-in voltage regulator:
- Input: 11.1V (battery)
- Output: 5V @ 1-3A
- Powers Arduino and servos

**Important:** Only use ONE ESC's BEC!

```
ESC 1: BEC connected to Arduino VIN ✅
ESC 2: BEC red wire cut ✅
ESC 3: BEC red wire cut ✅
ESC 4: BEC red wire cut ✅
```

### Specifications to Look For

| Specification | Recommended | Why |
|---------------|-------------|-----|
| Current rating | 20-30A | Matches motor needs |
| BEC output | 5V @ 2A | Powers Arduino |
| Refresh rate | 400-500Hz | Smooth control |
| Firmware | SimonK or BLHeli | Fast response |

### Motor Direction Calibration

After connecting ESC to motor:

```
Test spin direction:
  - Expected: Clockwise (CW) ↻
  - Actual: Counter-clockwise (CCW) ↺

Solution: Swap ANY 2 of the 3 motor wires
  - Motor wire order: A-B-C
  - New order: B-A-C (swap A and B)
  - Result: Now spins CW ✅
```

---

## Brushless Motors

### How Brushless Motors Work

Unlike brushed motors with physical commutator, brushless motors use electronic commutation.

```
Motor Components:
┌─────────────────┐
│  Stator (Fixed) │  ← 3 electromagnets (windings)
│                 │
│  Rotor (Spins)  │  ← Permanent magnets
└─────────────────┘

ESC energizes windings in sequence:
A+ B- C-  →  A- B+ C-  →  A- B- C+  →  (repeat)

Result: Rotor pulled around continuously
```

### Motor Specifications

#### KV Rating
**Definition:** RPM per volt (no load)

```
Example: 1200 KV motor
  @ 11.1V: 1200 × 11.1 = 13,320 RPM
  @ 14.8V: 1200 × 14.8 = 17,760 RPM
```

**Choosing KV:**
- Low KV (800-1000): More torque, larger props, heavier drones
- Medium KV (1000-1500): Balanced, good for learning (our choice)
- High KV (2000+): Less torque, smaller props, racing

#### Size Format: XXYY

Example: 2212 motor
- XX = stator diameter (22mm)
- YY = stator height (12mm)

Larger = more torque, lower KV

### Motor Configuration

```
    FRONT
   M1 ↺  ↻ M2
      \ X /
      / X \
   M4 ↻  ↺ M3
    REAR

M1, M3: CCW rotation (↺)
M2, M4: CW rotation (↻)
```

**Why?** Cancels reactive torque for stable yaw.

### Propeller Matching

| Motor KV | Battery | Prop Size | Thrust |
|----------|---------|-----------|--------|
| 1000 | 3S 11.1V | 10×4.5" | ~800g |
| 1200 | 3S 11.1V | 9×4.5" | ~700g |
| 1400 | 3S 11.1V | 8×4.5" | ~600g |
| 2300 | 3S 11.1V | 5×4" | ~400g |

**Rule of Thumb:** Total thrust should be 2× drone weight.

Example:
- Drone weight: 800g
- Need thrust: 1600g total (400g per motor)
- Choose: 1400KV with 8" props

---

## LiPo Batteries

### Understanding LiPo Specifications

#### Format: XXXXmAh YS ZZC

Example: **2200mAh 3S 30C**

**2200mAh:** Capacity
- Stores 2200 milliamp-hours
- Can supply 2.2A for 1 hour
- Or 4.4A for 30 minutes

**3S:** Cells in Series
- 1S = 3.7V nominal (1 cell)
- 2S = 7.4V nominal (2 cells)
- 3S = 11.1V nominal (3 cells) ← Our project
- 4S = 14.8V nominal (4 cells)

**30C:** Discharge Rate
- Max current = Capacity × C-rating
- 2200mAh × 30C = 66A max

### Voltage Monitoring

```
Per Cell Voltage Guide:
─────────────────────────
4.20V  ✅ Fully charged (100%)
4.10V  ✅ Almost full (90%)
3.85V  ✅ Nominal (50%)
3.70V  ✅ Getting low (20%)
3.50V  ⚠️  Land now! (10%)
3.30V  ❌ Critical damage risk
3.00V  ❌ Dead cell

For 3S (3 cells):
12.6V  = Full charge
11.1V  = Nominal
10.5V  = Land soon
9.9V   = Critical
```

### LiPo Safety Rules

#### Charging
```
✅ Use LiPo-specific charger
✅ Charge in LiPo safety bag
✅ Never leave charging unattended
✅ Charge at 1C rate (2200mAh = 2.2A charge)
✅ Balance charge every time
```

#### Storage
```
✅ Store at 3.85V per cell (storage voltage)
✅ Store in LiPo bag or ammo can
✅ Cool, dry place
❌ Never store fully charged
❌ Never store fully discharged
```

#### Disposal
```
⚠️ If battery is puffy, damaged, or hot:
1. Discharge to 3.0V per cell
2. Submerge in salt water for 24 hours
3. Dispose according to local regulations
```

### Battery Connector Types

| Connector | Current Rating | Use |
|-----------|----------------|-----|
| XT60 | 60A continuous | Standard (recommended) |
| Deans/T-Plug | 60A continuous | Also good |
| XT30 | 30A continuous | Small drones only |
| JST | 10A continuous | Micro drones |

**Our Project:** XT60 or Deans

---

## Arduino Nano

### Why Arduino Nano?

1. **Compact Size:** 18mm × 45mm
2. **Lightweight:** ~7g
3. **Sufficient I/O:** 20 digital, 8 analog pins
4. **PWM Capable:** 6 pins for ESC control
5. **I2C & SPI:** Built-in hardware support
6. **Affordable:** $3-5 for clones

### Specifications

| Feature | Specification |
|---------|---------------|
| Microcontroller | ATmega328P |
| Clock Speed | 16 MHz |
| Flash Memory | 32 KB |
| SRAM | 2 KB |
| EEPROM | 1 KB |
| Input Voltage | 7-12V (VIN) or 5V (USB) |
| Logic Level | 5V |
| 3.3V Output | Yes (50mA max) |

### Pin Capabilities

```
Arduino Nano Pins:
─────────────────────────────────────
Digital 0-1:    Serial (RX/TX)
Digital 2-13:   General I/O
Digital 2,3:    External interrupts
Digital 3,5,6,9,10,11: PWM output
Digital 10-13:  SPI (SS, MOSI, MISO, SCK)

Analog A0-A7:   Analog input (10-bit)
Analog A4-A5:   I2C (SDA, SCL)
```

### Performance Considerations

#### Loop Rate
```
Target: 250 Hz (4ms per loop)
Achievable: Yes, with optimized code

Breakdown per loop:
- I2C read (MPU6050): ~0.5ms
- I2C read (MS5611): ~0.3ms
- Angle calculation: ~0.2ms
- PID calculation: ~0.3ms
- Motor output: ~0.1ms
- Remaining: ~2.6ms buffer
```

#### Memory Usage
```
Flight controller code:
- Sketch: ~18 KB / 32 KB (56%)
- Global variables: ~800 bytes / 2 KB (40%)
- Sufficient headroom for features
```

### Bootloader Note

Many Arduino Nano clones use "Old Bootloader":
```
Arduino IDE:
Tools → Processor → ATmega328P (Old Bootloader)
```

If upload fails, try this setting!

---

## Component Shopping List

### Flight Controller Parts

| Component | Specification | Qty | Est. Price |
|-----------|---------------|-----|------------|
| Arduino Nano | ATmega328P | 1 | $5 |
| MPU6050 | 6-axis IMU | 1 | $3 |
| MS5611 | Barometer | 1 | $8 |
| nRF24L01+ PA+LNA | With antenna | 1 | $5 |
| Brushless Motor | 1000-1500KV | 4 | $40 |
| ESC | 20-30A with BEC | 4 | $40 |
| Propellers | 8-10" matched set | 2 sets | $10 |
| LiPo Battery | 11.1V 3S 2200mAh | 1 | $20 |
| Frame | 450mm quadcopter | 1 | $15 |
| Buzzer | 5V piezo | 1 | $1 |
| LED | 5mm | 1 | $0.50 |
| Wires & connectors | Various | - | $10 |
| **Total** | | | **~$158** |

### Remote Controller Parts

| Component | Specification | Qty | Est. Price |
|-----------|---------------|-----|------------|
| Arduino Nano | ATmega328P | 1 | $5 |
| nRF24L01+ PA+LNA | With antenna | 1 | $5 |
| Dual-axis Joystick | Analog | 2 | $4 |
| Push Buttons | Momentary | 4 | $2 |
| Toggle Switches | SPDT | 2 | $2 |
| Project Box | Custom/3D printed | 1 | $10 |
| 9V Battery | With connector | 1 | $5 |
| Wires & connectors | Various | - | $5 |
| **Total** | | | **~$38** |

### **Grand Total: ~$196**

---

## Performance Characteristics

### Flight Time
```
Battery: 2200mAh 3S
Current draw: ~15A hover, ~25A aggressive

Hover time: 2200mAh / 15A = ~8-10 minutes
Aggressive: 2200mAh / 25A = ~5 minutes
```

### Response Time
```
Radio latency: ~20ms (50Hz update)
Sensor reading: ~2ms (250Hz)
PID calculation: ~1ms
Total system lag: ~25ms (adequate for stable flight)
```

### Weight Budget
```
Frame: 150g
Electronics: 50g
Motors: 160g (40g × 4)
ESCs: 80g (20g × 4)
Battery: 180g
Props: 20g
Total: ~640g

Thrust required: 640g × 2 = 1280g (320g per motor)
```

---

**This completes the component technical guide!**
