# 🔧 Hardware Specifications & Shopping List

Complete hardware guide with part numbers, specifications, and purchase recommendations.

---

## 📋 Table of Contents

- [Complete Shopping List](#complete-shopping-list)
- [Component Specifications](#component-specifications)
- [Frame & Structure](#frame--structure)
- [Power System](#power-system)
- [Electronics](#electronics)
- [Tools Required](#tools-required)
- [Optional Upgrades](#optional-upgrades)

---

## 🛒 Complete Shopping List

### Budget Build (~$100-150 USD)

| Component | Qty | Est. Price | Notes |
|-----------|-----|------------|-------|
| **Flight Controller Parts** | | | |
| Arduino Nano (clone) | 1 | $3-5 | ATmega328P |
| MPU6050 module | 1 | $2-3 | GY-521 board |
| MS5611 module | 1 | $5-8 | Barometer |
| nRF24L01+ | 1 | $2-3 | Standard version |
| Passive buzzer | 1 | $0.50 | 5V compatible |
| LED (any color) | 1 | $0.10 | + 220Ω resistor |
| **Remote Controller Parts** | | | |
| Arduino Nano (clone) | 1 | $3-5 | ATmega328P |
| nRF24L01+ | 1 | $2-3 | Standard version |
| Analog joystick | 2 | $2-3 | PS2-style |
| Push button | 4 | $1 | Tactile switches |
| Toggle switch | 2 | $1-2 | SPST or SPDT |
| 9V battery holder | 1 | $1 | Or 3x AA holder |
| **Drone Hardware** | | | |
| Quadcopter frame | 1 | $10-20 | 250mm recommended |
| Brushless motors | 4 | $30-40 | 1500-2200Kv |
| ESC (Electronic Speed Controller) | 4 | $20-30 | 20-30A |
| Propellers | 2 sets | $5-10 | 5-6 inch |
| 3S LiPo battery | 1 | $20-30 | 2200mAh, 11.1V |
| LiPo charger | 1 | $15-25 | Balance charger |
| **Wiring & Connectors** | | | |
| Jumper wires | 1 pack | $3-5 | Male-female, various |
| Breadboard | 2 | $3-5 | For RC prototyping |
| Heat shrink tubing | 1 pack | $3-5 | Various sizes |
| 10µF capacitors | 2+ | $0.50 | For nRF24L01+ |
| 220Ω resistors | 2+ | $0.10 | For LED |
| XT60 connectors | 2 | $2 | Battery connectors |
| **TOTAL** | | **$130-200** | Prices vary by region |

### Where to Buy

**Electronics**:
- AliExpress (cheapest, 2-4 week shipping)
- Amazon (faster, slightly more expensive)
- eBay (varies)
- Local electronics stores

**Drone parts**:
- Banggood
- GetFPV
- RaceDayQuads
- Local hobby shops

---

## 🔌 Component Specifications

### Arduino Nano

**Specifications**:
```
Microcontroller: ATmega328P
Operating Voltage: 5V
Input Voltage: 7-12V (VIN pin)
Digital I/O Pins: 14 (6 PWM outputs)
Analog Input Pins: 8
Flash Memory: 32 KB (2 KB used by bootloader)
SRAM: 2 KB
EEPROM: 1 KB
Clock Speed: 16 MHz
Dimensions: 18mm x 45mm
```

**What to buy**:
- **Original Arduino Nano**: $25 (recommended for reliability)
- **Clone (CH340 chip)**: $3-5 (works fine, needs driver)
- Get **2 units** (one for FC, one for RC)

**USB Driver**:
- Original Nano: Uses FTDI or CH340 chip
- Clones: Usually CH340G
- Download driver: [CH340 Driver](http://www.wch.cn/download/CH341SER_ZIP.html)

---

### MPU6050 (Gyroscope + Accelerometer)

**Specifications**:
```
Chip: InvenSense MPU-6050
Interface: I2C (address 0x68 or 0x69)
Gyroscope Range: ±250, ±500, ±1000, ±2000°/s
Accelerometer Range: ±2g, ±4g, ±8g, ±16g
Power: 3-5V
Dimensions: 20mm x 16mm (GY-521 module)
Update Rate: Up to 1000Hz
```

**Module types**:
- **GY-521**: Most common, includes voltage regulator
- **MPU6050 breakout**: Smaller, no regulator

**Features to look for**:
- Built-in voltage regulator (3.3V or 5V)
- Pull-up resistors on I2C lines (usually 4.7kΩ)
- INT pin broken out (optional, for interrupt)

**Note**: Some cheap modules have poor soldering. Check connections!

---

### MS5611 (Barometric Pressure Sensor)

**Specifications**:
```
Chip: Measurement Specialties MS5611
Interface: I2C or SPI (address 0x77)
Pressure Range: 10-1200 mbar
Resolution: 0.012 mbar (10cm altitude)
Power: 1.8-3.6V (3.3V or 5V with regulator)
Temperature Range: -40 to +85°C
Dimensions: 15mm x 15mm (typical module)
```

**Module types**:
- **GY-63**: Common breakout board
- **MS5611 module**: Generic versions

**Why MS5611?**
- High precision (10cm resolution)
- Fast sampling (up to 100Hz)
- Low power consumption
- Used in professional flight controllers

**Alternatives** (not recommended):
- BMP180: Lower resolution, slower
- BMP280: Good alternative, similar price
- DPS310: New, less tested

---

### nRF24L01+ (2.4GHz Radio Transceiver)

**Specifications**:
```
Chip: Nordic nRF24L01+
Frequency: 2.4-2.525 GHz
Data Rate: 250kbps, 1Mbps, 2Mbps
Range: 100m (standard), 1000m (PA+LNA)
Power: 1.9-3.6V (3.3V typical)
Current: 11-13mA (RX), 12-16mA (TX)
Channels: 125 (1MHz spacing)
Interface: SPI
Dimensions: 29mm x 15mm
```

**Module types**:

1. **Standard nRF24L01+**:
   - Small PCB antenna
   - Range: ~100 meters
   - Good for testing
   - Price: $2-3

2. **nRF24L01+ PA+LNA** (Power Amplifier + Low Noise Amplifier):
   - External antenna
   - Range: 500-1000 meters
   - Higher power consumption (115mA!)
   - Requires external power supply
   - Price: $3-5

**Which to buy?**
- **Beginners**: Standard version (easier power)
- **Long range**: PA+LNA version (need separate 3.3V regulator)

**Critical requirement**:
- **MUST add 10µF capacitor** between VCC and GND!
- Place as close to module as possible
- Prevents voltage drops during transmission

**Common issues**:
- Poor quality clones (buy extras!)
- Incorrect power (needs 3.3V, NOT 5V)
- Missing capacitor (causes intermittent connection)

---

### Brushless Motors

**Specifications** (recommended for 250mm frame):
```
Size: 2204-2208 (22mm diameter, 4-8mm height)
KV Rating: 1500-2200Kv
Power: 150-250W per motor
Weight: 25-35g
Shaft: 5mm (M5 threading)
Mounting: 16x19mm holes
Voltage: 3S (11.1V)
```

**KV rating explained**:
- **KV** = RPM per volt (e.g., 2000Kv × 11.1V = 22,200 RPM)
- **Higher KV** (2200+): Fast, responsive, lower flight time
- **Lower KV** (1500-1800): Efficient, stable, longer flight time
- **Recommendation**: 2000Kv for balanced performance

**Motor configuration**:
- Need **4 motors** total
- **2 CW** (clockwise)
- **2 CCW** (counter-clockwise)
- OR: Buy 4 identical, reverse 2 in software/wiring

**Brands** (by quality):
- Budget: Emax, Racerstar, DYS ($8-12 each)
- Mid-range: T-Motor, XING ($15-25 each)
- Premium: T-Motor F series ($30+ each)

---

### ESC (Electronic Speed Controller)

**Specifications**:
```
Rating: 20-30A continuous
Input: 3S-4S LiPo (11.1-14.8V)
Output: 3-phase AC (to motor)
Signal: PWM (1000-2000µs)
BEC: 5V, 2-3A (to power Arduino)
Dimensions: 25mm x 12mm x 6mm
Weight: 5-8g
```

**What to buy**:
- **4 individual ESCs** (easier to replace if one fails)
- OR: **4-in-1 ESC** (cleaner wiring, but less flexible)

**Features to look for**:
- **BEC output**: Powers Arduino + sensors
- **Active braking**: Faster response
- **Oneshot125/Multishot**: Not needed for this project (standard PWM OK)
- **Calibration**: Must support standard PWM calibration

**Brands**:
- Budget: Mystery, Afro, HobbyKing ($5-7 each)
- Quality: BLHeli_S, BLHeli_32 ($8-15 each)

**Calibration**:
All ESCs must be calibrated to same range (1000-2000µs):
```
1. Disconnect battery
2. Set throttle to 2000µs (max)
3. Connect battery → ESCs beep
4. Set throttle to 1000µs (min)
5. ESCs beep confirmation
```

---

### Propellers

**Specifications** (for 250mm frame):
```
Size: 5x4 to 6x4.5 (diameter x pitch, inches)
Material: Plastic (nylon) or carbon fiber
Mounting: 5mm center hole, M5 thread
Weight: 3-5g each
```

**Propeller notation**:
- **5045**: 5 inch diameter, 4.5 inch pitch
- **6030**: 6 inch diameter, 3.0 inch pitch

**CW vs CCW**:
- Need **2 CW** and **2 CCW** propellers
- Usually marked with "R" (reverse) or color

**Material**:
- **Nylon (plastic)**: Cheap, durable, good for learning
- **Carbon fiber**: Stiff, efficient, expensive, breaks easily

**Brands**:
- Gemfan (best value)
- HQProp (premium)
- DAL (durable)

**Buy extras!** You will break propellers while learning.

---

### LiPo Battery

**Specifications** (recommended):
```
Voltage: 3S (11.1V nominal, 12.6V fully charged)
Capacity: 1500-2500mAh
Discharge Rate: 25-45C continuous
Connector: XT60 or T-plug
Dimensions: ~105mm x 35mm x 25mm (2200mAh)
Weight: ~190g (2200mAh)
```

**Specifications explained**:
- **3S**: 3 cells in series (3.7V × 3 = 11.1V)
- **mAh**: Capacity (2200mAh = 2.2Ah)
- **C rating**: Discharge rate (2200mAh × 25C = 55A max)

**What to buy**:
- **Capacity**: 1500-2500mAh (5-10 min flight time)
- **C rating**: 25C minimum (higher is better)
- **Brand**: Turnigy, Zippy, Tattu, Gens Ace

**⚠️ LiPo Safety**:
- Never discharge below 3.0V per cell
- Store at 3.8V per cell (storage charge)
- Use LiPo-safe bag for charging and storage
- Never puncture or short circuit
- Dispose properly if damaged/swollen

---

### Frame

**Specifications** (recommended):
```
Size: 250mm (diagonal motor-to-motor)
Material: Carbon fiber or fiberglass
Weight: 80-150g
Motor mounting: 16x19mm or 12x19mm
Thickness: 2-3mm arms
Configuration: X-configuration (not H or +)
```

**Frame types**:
- **250mm**: Good balance, recommended
- **210mm**: Smaller, more agile, less stable
- **300mm**: Larger, more stable, slower

**Material**:
- **Carbon fiber**: Strong, light, expensive ($20-50)
- **Fiberglass**: Cheap, heavy, breaks easily ($10-20)
- **Plastic**: Very cheap, very weak ($5-10)

**What's included**:
- Top and bottom plates
- Arms (4x)
- Hardware (screws, standoffs)
- Sometimes: PDB (power distribution board)

**Popular frames**:
- ZMR250 (classic, cheap)
- QAV250 (quality)
- Martian II (compact)

---

## 🔋 Power System

### Power Distribution Options

**Option 1: Individual ESC power** (simplest)
```
Battery → Split 4 ways → 4 ESCs
ESC1 BEC → Arduino + sensors
ESC2-4 → No BEC output (cut red wire)
```

**Option 2: Power Distribution Board (PDB)**
```
Battery → PDB → 4 ESCs in parallel
PDB 5V output → Arduino + sensors
```

**Option 3: Separate BEC** (most reliable)
```
Battery → PDB → 4 ESCs
Battery → BEC (5V/3A) → Arduino + sensors
```

### Current Draw Estimation

| Component | Current | Notes |
|-----------|---------|-------|
| Arduino Nano | 50mA | Plus peripherals |
| MPU6050 | 3mA | Low power |
| MS5611 | 1mA | Very low power |
| nRF24L01+ | 15mA | RX/TX |
| Buzzer | 30mA | When sounding |
| LED | 20mA | Depends on brightness |
| **Total (FC)** | **~120mA** | From 5V BEC |
| **Total (RC)** | **~70mA** | From 9V battery |

Motors: 5-20A each at hover, 30A+ at full throttle

---

## 🛠 Tools Required

### Essential Tools

| Tool | Purpose | Est. Price |
|------|---------|------------|
| Soldering iron | Wiring connections | $15-30 |
| Solder | Electronic connections | $5 |
| Wire strippers | Preparing wires | $10 |
| Multimeter | Testing voltage/continuity | $15 |
| Screwdriver set | Assembly | $10 |
| Hex keys (Allen) | Motor mounting | $5 |
| USB cable (Mini-B) | Arduino programming | $5 |
| LiPo battery bag | Safety | $5 |

### Optional but Helpful

| Tool | Purpose | Est. Price |
|------|---------|------------|
| Helping hands | Soldering | $10 |
| Heat gun | Heat shrink tubing | $15 |
| Wire cutters | Trimming wires | $10 |
| Tweezers | Small parts | $5 |
| Hot glue gun | Securing components | $10 |
| Cable ties | Wire management | $3 |

---

## ⚡ Optional Upgrades

### Performance Upgrades

1. **GPS Module** (NEO-6M or NEO-M8N)
   - Price: $10-25
   - Features: Position hold, return to home
   - Requires code modification

2. **FPV Camera + Transmitter**
   - Price: $30-60
   - Features: First-person view flying
   - Requires goggles or monitor

3. **Telemetry Radio** (HC-12 or LoRa)
   - Price: $5-20
   - Features: Real-time data to ground station
   - Longer range than nRF24L01+

4. **Better Battery** (4S LiPo)
   - Price: $30-40
   - Features: More power, longer flight
   - Requires higher KV motors, ESC update

5. **Carbon Fiber Props**
   - Price: $3-5 per set
   - Features: Stiffer, more efficient
   - Break easily on crash

### Convenience Upgrades

1. **XT60 Connectors** (instead of soldering)
   - Easy battery swapping
   - Standardized

2. **Battery Voltage Monitor/Alarm**
   - Prevents over-discharge
   - $3-5

3. **Balance Charger** (if not included)
   - Safe charging
   - $15-30

4. **Field Charging** (car charger or power supply)
   - Charge at flying location
   - $20-40

---

## 📦 Pre-assembled Options

### Flight Controller Boards

Instead of Arduino Nano, consider:

**Budget FC boards** ($20-40):
- Matek F405
- Omnibus F4
- JHEMCU GHF420

**Advantages**:
- Smaller, lighter
- Built-in sensors
- Better PID processing
- USB programming

**Disadvantages**:
- More complex setup
- Requires Betaflight configuration
- Less educational value
- This code won't run on them (different architecture)

---

## 🌍 Regional Considerations

### Voltage (worldwide)

USB power is universal (5V), but:
- **US/Americas**: 110-120V AC outlets
- **EU/Asia/Africa**: 220-240V AC outlets
- LiPo chargers work on both (check specs)

### Radio Regulations

2.4GHz is generally unlicensed worldwide, but:
- **US (FCC)**: 2.4-2.4835 GHz
- **EU (ETSI)**: 2.4-2.4835 GHz
- **Japan**: 2.4-2.497 GHz
- Check local regulations before flying!

### Shipping

**Batteries**:
- LiPo batteries restricted on airlines
- Some countries ban lithium battery imports
- Ship via ground/sea only

**Drones**:
- Most countries allow hobbyist drones
- Some require registration (>250g)
- Check customs regulations

---

## ✅ Quality Check

When components arrive:

### Arduino Nano
- [ ] USB connection recognized
- [ ] Blink sketch uploads successfully
- [ ] All pins output correct voltage

### MPU6050
- [ ] I2C scanner finds device at 0x68
- [ ] Adafruit example sketch runs
- [ ] Gyro values change when rotated

### MS5611
- [ ] I2C scanner finds device at 0x77
- [ ] Pressure reading is reasonable (~1000 mbar)
- [ ] Temperature reading is reasonable

### nRF24L01+
- [ ] Radio.begin() returns true
- [ ] Can send/receive test packets
- [ ] LED blinks during transmission (if equipped)

### Motors
- [ ] Spin freely by hand
- [ ] No grinding or resistance
- [ ] Bearings feel smooth

### ESCs
- [ ] Beep on power-up (3 ascending tones)
- [ ] Respond to PWM signal
- [ ] All 4 calibrate identically

---

**Total Project Cost Summary**:
- **Minimum (clone parts)**: ~$100-120
- **Recommended (quality parts)**: ~$150-180
- **Premium (best components)**: ~$250-300

---

**Happy Building! 🔧**

*Buy from reputable sellers, check reviews, and always order a few spare parts!*
