# Parts List and Shopping Guide

Complete bill of materials for the Professional Arduino Nano Drone System.

---

## 🛒 Required Components

### Flight Controller Components

| Item | Quantity | Specifications | Estimated Cost |
|------|----------|----------------|----------------|
| **Arduino Nano** | 1 | ATmega328P, 5V 16MHz | $3-8 |
| **NRF24L01 PA+LNA** | 1 | 2.4GHz, PA+LNA with antenna | $3-6 |
| **MPU6050** | 1 | 6-axis IMU (GY-521 module) | $2-5 |
| **Buzzer** | 1 | Active buzzer 5V | $0.50-1 |
| **LED** | 1 | 5mm standard LED (any color) | $0.10 |
| **Resistor 220Ω** | 1 | 1/4W for LED current limiting | $0.05 |
| **ESC** | 4 | 30A SimonK/BLHeli (match motor) | $8-15 each |
| **Brushless Motors** | 4 | 1000-1300KV for 10" props | $10-20 each |
| **Propellers** | 2 sets | 10x4.5 (CW/CCW pairs) | $5-10 |

**Flight Controller Subtotal:** ~$80-150

### Remote Controller Components

| Item | Quantity | Specifications | Estimated Cost |
|------|----------|----------------|----------------|
| **Arduino Nano** | 1 | ATmega328P, 5V 16MHz | $3-8 |
| **NRF24L01 PA+LNA** | 1 | 2.4GHz, PA+LNA with antenna | $3-6 |
| **Dual-Axis Joystick** | 2 | 10K potentiometer, self-centering | $2-4 each |
| **Push Button** | 2 | Momentary tactile switch | $0.20 each |
| **Toggle Switch** | 2 | SPDT or SPST | $0.50-1 each |

**Remote Controller Subtotal:** ~$15-30

### Power and Batteries

| Item | Quantity | Specifications | Estimated Cost |
|------|----------|----------------|----------------|
| **LiPo Battery (Drone)** | 1-2 | 3S 11.1V 2200mAh 25C+ | $15-25 |
| **LiPo Battery (Remote)** | 1 | 2S 7.4V 1000mAh or 9V battery | $10-15 |
| **LiPo Charger** | 1 | Balance charger for 2S-3S | $15-30 |
| **Battery Straps** | 2 | Velcro or rubber straps | $2-5 |

**Power Subtotal:** ~$40-75

### Frame and Hardware

| Item | Quantity | Specifications | Estimated Cost |
|------|----------|----------------|----------------|
| **Quadcopter Frame** | 1 | 250-450mm carbon fiber or plastic | $15-40 |
| **Mounting Screws** | 1 set | M3 screws, nuts, standoffs | $3-5 |
| **Vibration Dampers** | 4 | Soft rubber/silicone for FC | $2-4 |
| **Zip Ties** | 1 pack | Cable management | $2-3 |
| **Velcro Tape** | 1 roll | Battery mounting | $3-5 |

**Frame/Hardware Subtotal:** ~$25-60

### Electronics Accessories

| Item | Quantity | Specifications | Estimated Cost |
|------|----------|----------------|----------------|
| **Capacitors 10μF** | 2-4 | Electrolytic for NRF24 (critical!) | $0.20 each |
| **Capacitors 100μF** | 2-4 | Optional for power filtering | $0.30 each |
| **Breadboard** | 1-2 | For prototyping (optional) | $3-5 |
| **Jumper Wires** | 1 set | Male-Male, Male-Female | $3-5 |
| **22AWG Wire** | 1 spool | Silicone stranded wire (red/black) | $5-10 |
| **Heat Shrink Tubing** | 1 set | Assorted sizes | $5-8 |
| **XT60 Connectors** | 2-4 | Battery connectors | $1 each |

**Electronics Subtotal:** ~$20-40

### Tools (if not owned)

| Item | Purpose | Estimated Cost |
|------|---------|----------------|
| **Soldering Iron** | Assembly | $15-40 |
| **Solder Wire** | Connections | $5-10 |
| **Wire Stripper** | Wire prep | $5-15 |
| **Multimeter** | Testing/debugging | $10-30 |
| **Screwdriver Set** | Assembly | $10-20 |
| **Hex Key Set** | Frame assembly | $5-15 |

**Tools Subtotal:** ~$50-130

---

## 💰 Total Cost Summary

| Category | Budget Option | Quality Option |
|----------|--------------|----------------|
| Flight Controller Parts | $80 | $150 |
| Remote Controller Parts | $15 | $30 |
| Power/Batteries | $40 | $75 |
| Frame/Hardware | $25 | $60 |
| Electronics Accessories | $20 | $40 |
| **Total (without tools)** | **$180** | **$355** |
| Tools (if needed) | +$50 | +$130 |
| **Grand Total** | **$230** | **$485** |

---

## 📦 Recommended Kits

### Option 1: Buy Individual Parts
- More expensive initially
- Better quality components
- Can customize each part
- Best for learning

### Option 2: Starter Drone Kit + Custom Electronics
- Buy ready-made frame+motor+ESC kit (~$80-120)
- Add Arduino Nano + sensors + NRF modules
- Faster assembly
- Good balance of cost and quality

---

## 🛍️ Where to Buy

### Online Retailers:
- **AliExpress**: Cheapest, slow shipping (2-4 weeks)
- **Banggood**: Good drone parts selection
- **Amazon**: Fast shipping, higher prices
- **HobbyKing**: RC/drone specialty store
- **eBay**: Mix of new and used parts
- **Local Electronics Store**: Immediate, but limited selection

### Arduino/Electronics:
- **Arduino.cc**: Official boards
- **Adafruit**: Quality sensors and modules
- **SparkFun**: Educational electronics
- **DigiKey/Mouser**: Professional components

---

## 🔍 Buying Guide and Tips

### Arduino Nano
- **Genuine vs Clone**: Clones work fine, 1/3 the price
- **Check**: CH340 USB chip (may need driver)
- **Tip**: Buy 2-3 extras for development

### NRF24L01 PA+LNA
- **CRITICAL**: Must be PA+LNA version (with antenna)
- **Check**: Includes external antenna
- **Warning**: Basic NRF24L01 (without PA+LNA) has very short range (<10m)
- **Tip**: Buy extras, failure rate ~10-20%

### MPU6050
- **Module**: Get GY-521 breakout board
- **Check**: Has I2C pull-up resistors onboard
- **Alternative**: MPU9250 (9-axis, costs more)
- **Tip**: Pre-test with I2C scanner before soldering

### ESCs
- **Firmware**: SimonK or BLHeli preferred
- **Rating**: 30A for 1000-1300KV motors
- **Features**: Need BEC (5V output) on at least one
- **Check**: Comes with pre-soldered wires
- **Tip**: Buy matched set for consistency

### Motors
- **KV Rating**: 1000-1300 KV for 10" props
- **Size**: 22xx series (e.g., 2212, 2216)
- **Mount**: Standard 16x19mm or 19x25mm holes
- **Check**: Includes motor mount screws
- **Tip**: Spare motors recommended

### Propellers
- **Size**: 10x4.5 or 9x4.5 for 1000KV motors
- **Type**: Need CW and CCW (2 of each)
- **Material**: Plastic for learning (cheap to replace)
- **Check**: Prop adapters match motor shaft
- **Tip**: Buy 3-4 sets (will break some!)

### LiPo Batteries
- **Chemistry**: LiPo only (not NiMH or LiFePO4)
- **Voltage**: 3S (11.1V) recommended
- **Capacity**: 1500-2200mAh for 5-10min flight
- **C-Rating**: Minimum 25C discharge
- **Check**: XT60 or compatible connector
- **Safety**: MUST have LiPo charger and safety bag

### Frame
- **Size**: 250-450mm (diagonal motor-to-motor)
- **Material**: Carbon fiber (strong) or plastic (cheap)
- **Style**: "X" configuration quadcopter
- **Check**: Includes PDB (power distribution board)
- **Tip**: Start with cheaper plastic frame

---

## ⚠️ Critical Components (Don't Skip!)

### Must Have:
1. **10μF Capacitors for NRF24L01** - System won't work reliably without these!
2. **Vibration Dampers for FC** - Essential for stable flight
3. **LiPo Charger** - Never charge LiPo without proper charger
4. **LiPo Safety Bag** - Fire safety essential

### Nice to Have:
- Battery voltage alarm/monitor
- Spare props and motors
- Field repair kit
- Battery strap with voltage display
- FPV camera (future upgrade)

---

## 🔧 Recommended Brands

### Budget-Friendly:
- **Motors**: Emax, Readtosky
- **ESC**: Emax, Hobbywing
- **Frame**: ZMR250, QAV-X clones
- **Battery**: Turnigy, ZOP Power

### Premium Quality:
- **Motors**: T-Motor, KDE Direct
- **ESC**: Castle Creations, Hobbywing Platinum
- **Frame**: iFlight, TBS Vendetta
- **Battery**: Tattu, Gens Ace

---

## 📋 Shopping Checklist

Print this and check off as you acquire parts:

### Flight Controller Board:
- [ ] Arduino Nano
- [ ] NRF24L01 PA+LNA with antenna
- [ ] 10μF capacitor for NRF
- [ ] MPU6050 (GY-521)
- [ ] Active buzzer 5V
- [ ] LED + 220Ω resistor
- [ ] 4x ESC (30A)
- [ ] 4x Brushless motors (1000-1300KV)
- [ ] 2 sets propellers (CW/CCW)
- [ ] Frame with mounting hardware
- [ ] 3S LiPo battery
- [ ] Wire, connectors, zip ties

### Remote Controller Board:
- [ ] Arduino Nano
- [ ] NRF24L01 PA+LNA with antenna
- [ ] 10μF capacitor for NRF
- [ ] 2x Dual-axis joysticks
- [ ] 2x Push buttons
- [ ] 2x Toggle switches
- [ ] 2S LiPo or 9V battery
- [ ] Wire, connectors
- [ ] Enclosure/case (optional)

### Supporting Equipment:
- [ ] LiPo charger (balance charger)
- [ ] LiPo safety bag
- [ ] USB cable for Arduino (Mini-B)
- [ ] Soldering iron + solder
- [ ] Multimeter
- [ ] Screwdriver/hex key set
- [ ] Wire stripper
- [ ] Heat shrink tubing

---

## 💡 Money-Saving Tips

1. **Buy in Bulk**: Get multiple Arduinos, NRF modules (failure spares)
2. **AliExpress**: Cheapest for non-critical parts (long wait)
3. **Local Hobby Shops**: Sometimes have sales on motors/ESCs
4. **Used Parts**: Frame and motors can be bought used
5. **Avoid**: Cheapest LiPo batteries (fire risk!), no-name ESCs (failure risk)

---

## 🎓 First-Time Builder Recommendations

### Start Simple:
- **Frame**: $20 plastic frame (you'll crash it)
- **Motors**: Mid-range 1000KV
- **Arduino**: Clone Nano ($3)
- **Props**: Buy 4 sets (will break)

### Upgrade Later:
- Carbon fiber frame after learning to fly
- Higher KV motors for speed (after mastering hover)
- Telemetry displays
- FPV system
- GPS/altitude hold

---

## 📞 Support Resources

- **Arduino Forum**: forum.arduino.cc
- **RC Groups**: rcgroups.com
- **Reddit**: r/Multicopter, r/arduino
- **YouTube**: Tutorials on drone building

---

**Total Estimated Project Time:**
- Shopping/Ordering: 1-2 weeks
- Assembly: 4-8 hours
- Testing/Calibration: 2-4 hours
- Learning to Fly: Weeks to months!

**Good luck with your build! 🚁**

