# Complete Parts List & Purchasing Guide

## 🛒 Shopping List

### Flight Controller Electronics

| Item | Quantity | Specs | Est. Price | Links/Notes |
|------|----------|-------|------------|-------------|
| **Arduino Nano** | 1 | ATmega328P, 16MHz | $3-5 | Clone boards work fine |
| **MPU6050** | 1 | 6-axis gyro/accel, GY-521 module | $2-4 | Comes with pins |
| **MS5611** | 1 | Barometric pressure sensor | $5-8 | I2C version |
| **NRF24L01+** | 2 | 2.4GHz wireless module | $2-4 each | Get with adapter board |
| **NRF24L01 Adapter** | 2 | 3.3V regulator + capacitor | $1-2 each | **Essential for stability** |
| **Active Buzzer** | 1 | 5V, 12mm diameter | $0.50 | Must be "active" type |
| **LED** | 2 | 5mm, any color | $0.20 each | Red for FC, green for RC |
| **220Ω Resistors** | 2 | 1/4W | $0.10 | For LED current limiting |
| **10µF Capacitors** | 2 | Electrolytic, 16V+ | $0.20 each | For NRF24L01 (if no adapter) |
| **Jumper Wires** | 20+ | Male-to-male & female-to-female | $5 | Dupont connectors |
| **Prototype Board** | 1 | Perforated PCB | $2-3 | For mounting components |

**Subtotal Electronics:** ~$30-45

---

### Motors & Propulsion

| Item | Quantity | Specs | Est. Price | Notes |
|------|----------|-------|------------|-------|
| **Brushless Motors** | 4 | 1000-1500 KV, 2204-2212 size | $8-15 each | Match to frame size |
| **ESCs** | 4 | 20-30A with BEC (5V 2A+) | $6-10 each | Must support 1000-2000µs PWM |
| **Propellers** | 2 sets | 10x4.5 or 11x4.7 (CW+CCW) | $2-4/set | Buy extras! |
| **Motor Mounts** | 4 | M3 screw compatible | $1 each | May come with frame |
| **Propeller Adapters** | 4 | Match motor shaft diameter | $0.50 each | Usually included with motors |

**Recommended Combos:**
- **450mm frame:** 1000KV motors + 10" props + 20A ESCs
- **550mm frame:** 1200KV motors + 11" props + 25A ESCs

**Subtotal Propulsion:** ~$60-100

---

### Frame & Structure

| Item | Quantity | Specs | Est. Price | Notes |
|------|----------|-------|------------|-------|
| **Quadcopter Frame** | 1 | 450mm or 550mm, X-config | $15-30 | Carbon fiber recommended |
| **FC Mounting Plate** | 1 | Vibration dampers/standoffs | $3-5 | Reduces vibration noise |
| **M3 Screws** | 20+ | Various lengths (8mm, 12mm, 16mm) | $3 | Stainless steel |
| **Nylon Locknuts** | 20 | M3 | $2 | Prevents loosening |
| **Zip Ties** | 10+ | 100mm-200mm | $2 | Cable management |
| **Velcro Straps** | 2 | 20mm x 200mm | $2 | Battery mounting |
| **Double-Sided Foam Tape** | 1 roll | 3M VHB or similar | $3 | Component mounting |

**Subtotal Frame:** ~$30-47

---

### Power System

| Item | Quantity | Specs | Est. Price | Notes |
|------|----------|-------|------------|-------|
| **LiPo Battery (Main)** | 1+ | 3S 11.1V, 2200-5000mAh, 25C+ | $15-35 | Higher mAh = longer flight |
| **LiPo Battery (RC)** | 1 | 2S 7.4V 500-1000mAh or 9V | $8-12 | Or use USB power bank |
| **XT60 Connector** | 2 | Male + Female | $1 | Standard LiPo connector |
| **LiPo Balance Charger** | 1 | 2S-4S compatible, 2-5A | $15-30 | **Don't skimp on this!** |
| **LiPo Safe Bag** | 1 | Fireproof charging bag | $5-8 | **Safety essential** |
| **Power Distribution Board** | 1 | XT60 input, 4x ESC outputs | $3-5 | Optional but cleaner |

**Battery Flight Time Estimates:**
- 2200mAh → 5-7 minutes
- 3300mAh → 8-10 minutes
- 5000mAh → 12-15 minutes

**Subtotal Power:** ~$47-90

---

### RC Controller Components

| Item | Quantity | Specs | Est. Price | Notes |
|------|----------|-------|------------|-------|
| **Arduino Nano** | 1 | ATmega328P, 16MHz | $3-5 | Same as FC |
| **Analog Joysticks** | 2 | 2-axis, 10K pots, with button | $2-3 each | PS2-style modules |
| **Push Buttons** | 2 | 12mm momentary, normally open | $0.30 each | Panel-mount type |
| **Toggle Switches** | 2 | SPDT, 6mm | $0.50 each | 2-position |
| **Project Enclosure** | 1 | 150x100x50mm plastic case | $5-8 | Handheld size |
| **LED** | 1 | 5mm, green or blue | $0.20 | Status indicator |
| **220Ω Resistor** | 1 | 1/4W | $0.10 | For LED |

**Subtotal RC Controller:** ~$15-25

---

### Tools & Accessories

| Item | Quantity | Est. Price | Notes |
|------|----------|------------|-------|
| **Soldering Iron** | 1 | $15-30 | 60W recommended |
| **Solder** | 1 roll | $5-8 | 60/40 or lead-free |
| **Wire Strippers** | 1 | $5-10 | Multi-gauge |
| **Multimeter** | 1 | $10-20 | For testing voltages |
| **Hex Key Set** | 1 | $5-8 | Metric (1.5mm, 2mm, 2.5mm) |
| **Propeller Balancer** | 1 | $8-12 | Reduces vibration |
| **USB Cable** | 1 | $2-3 | Mini-USB for Arduino Nano |
| **Heat Shrink Tubing** | 1 set | $5-8 | Assorted sizes |
| **Helping Hands** | 1 | $5-10 | Soldering aid |
| **Spirit Level** | 1 | $3-5 | For calibration |
| **Screwdriver Set** | 1 | $8-12 | Phillips + flathead |

**Subtotal Tools:** ~$74-129 (one-time purchase)

---

## 💰 Total Cost Breakdown

### Minimum Configuration (Budget Build)
| Category | Cost |
|----------|------|
| Electronics | $30 |
| Motors & Props | $60 |
| Frame | $30 |
| Power System | $47 |
| RC Controller | $15 |
| Tools (basic) | $50 |
| **TOTAL** | **~$232** |

### Recommended Configuration
| Category | Cost |
|----------|------|
| Electronics | $38 |
| Motors & Props | $80 |
| Frame | $40 |
| Power System | $70 |
| RC Controller | $20 |
| Tools (complete) | $100 |
| **TOTAL** | **~$348** |

### High-End Configuration
| Category | Cost |
|----------|------|
| Electronics | $45 |
| Motors & Props | $100 |
| Frame (carbon fiber) | $47 |
| Power System | $90 |
| RC Controller | $25 |
| Tools (professional) | $129 |
| **TOTAL** | **~$436** |

---

## 🛍️ Where to Buy

### Online Stores

**Budget Options:**
- **AliExpress** - Cheapest, 2-6 week shipping
- **Banggood** - Good prices, 1-4 week shipping
- **eBay** - Mixed prices, faster shipping options

**Faster/Reliable:**
- **Amazon** - Prime shipping, easy returns
- **HobbyKing** - Drone-specific, good quality
- **GetFPV** - Enthusiast-focused, expert support

**Local:**
- Electronics stores (Fry's, Micro Center)
- Hobby shops (RC planes section)
- Maker spaces (sometimes sell components)

### Recommended Kits

Some sellers offer "quadcopter component kits" that include:
- 4x motors
- 4x ESCs
- 1x power distribution board
- 4x pairs of propellers

**Price:** $50-80 (saves ~20% vs buying separately)

---

## 🔍 Component Selection Guide

### Motors

**KV Rating Selection:**
```
Frame Size    Motor KV    Prop Size    Battery
─────────────────────────────────────────────
450mm         1000KV      10"          3S
450mm         1200KV      9"           3S
550mm         900KV       11"          4S
550mm         1100KV      10"          3S-4S
```

**Rule of thumb:** Lower KV = more torque, bigger props, better efficiency

### ESCs

**Current Rating:**
```
Motor Size    ESC Rating    Safety Margin
────────────────────────────────────────
2204/2205     20A           30A
2212          25A           30A
2216          30A           40A
```

**Must-have features:**
- ✅ 1000-2000µs PWM support (SimonK/BLHeli firmware)
- ✅ BEC output (5V 2A minimum)
- ✅ No brake mode
- ⚠️ Avoid oneshot125/multishot (not compatible with Arduino Servo library)

### Battery

**Capacity Selection:**
```
Use Case              mAh        Flight Time
─────────────────────────────────────────────
Testing/Training      2200       5-7 min
Normal Flying         3300       8-10 min
Extended Flying       5000       12-15 min
```

**Discharge Rate:**
- Minimum: 25C
- Recommended: 35C+
- Higher C = less voltage sag under load

**Example:** 3300mAh 35C battery can deliver 115A burst current

### Frame

**Size Selection:**
```
Frame Size    Motor-to-Motor    Use Case
───────────────────────────────────────────
350mm         350mm            Micro/indoor
450mm         450mm            Beginner/sport
550mm         550mm            Stable/cinematic
650mm+        650mm+           Heavy-lift
```

**Recommended:** 450mm for this system (good balance)

**Material:**
- **Fiberglass:** Cheap, flexible, absorbs crashes
- **Carbon Fiber:** Stiff, lightweight, expensive
- **Plastic:** Very cheap, heavy, breaks easily

---

## ⚙️ Component Compatibility Matrix

### Tested Combinations

#### Configuration A (Budget)
```
Frame:      450mm fiberglass
Motors:     2212 1000KV
ESCs:       SimonK 30A
Props:      10x4.5
Battery:    3S 2200mAh 25C
Weight:     ~650g
Flight:     6-7 min
Cost:       ~$240
```

#### Configuration B (Recommended)
```
Frame:      450mm carbon
Motors:     2204 1400KV  
ESCs:       BLHeli 20A
Props:      9x4.5
Battery:    3S 3300mAh 35C
Weight:     ~580g
Flight:     9-11 min
Cost:       ~$350
```

#### Configuration C (High Performance)
```
Frame:      550mm carbon
Motors:     2212 900KV
ESCs:       BLHeli_S 30A
Props:      11x4.7
Battery:    4S 3300mAh 45C
Weight:     ~700g
Flight:     10-13 min
Cost:       ~$450
```

---

## 📦 Pre-Purchase Checklist

Before ordering, verify:

### Electronics
- [ ] Arduino Nano is ATmega328P (not ATmega168)
- [ ] MPU6050 module includes pull-up resistors
- [ ] MS5611 is I2C version (not SPI)
- [ ] NRF24L01 comes with adapter board OR buy 10µF caps separately
- [ ] ESCs support 1000-2000µs PWM (check datasheet)
- [ ] Buzzer is "active" type (has built-in oscillator)

### Propulsion
- [ ] Motors have compatible mounting pattern (12mm or 16mm holes)
- [ ] Props match motor shaft diameter (usually 5mm)
- [ ] ESCs have BEC output (5V 2A minimum)
- [ ] Battery connector matches ESCs (usually XT60)

### Frame
- [ ] Frame is X-configuration (not +)
- [ ] Includes motor mounts or compatible ones
- [ ] Has space for 35x35mm FC mounting
- [ ] Can fit your chosen battery

### Power
- [ ] Battery voltage matches ESC rating (3S or 4S)
- [ ] Charger supports your battery cell count
- [ ] Connector types match (XT60/XT90/Deans)

---

## 🎁 Optional Upgrades

### Nice to Have

| Item | Purpose | Price |
|------|---------|-------|
| **FPV Camera** | First-person view | $15-30 |
| **VTX (Video TX)** | Transmit FPV video | $15-25 |
| **GPS Module** | Position hold | $20-40 |
| **Telemetry Module** | Real-time data | $10-20 |
| **OSD Module** | On-screen display | $8-15 |
| **Voltage Alarm** | Low battery warning | $3-5 |
| **Landing Gear** | Protect underside | $5-10 |
| **Prop Guards** | Safety | $8-12 |

### Future Proofing

| Item | Purpose | Price |
|------|---------|-------|
| **Spare Motors** | Crash replacement | $8-15 each |
| **Spare Props** | Break often! | $2-4/set |
| **Spare ESCs** | Burn out sometimes | $6-10 each |
| **Extra Battery** | No downtime | $15-35 |
| **XT60 Pigtails** | Quick connectors | $2-3/pair |

---

## 🔧 Assembly Priority Order

### Phase 1: Core Testing (~$50)
Buy first:
1. 2x Arduino Nano
2. 2x NRF24L01 + adapters
3. MPU6050
4. Jumper wires
5. LEDs, buzzer, buttons

**Goal:** Test communication & sensors on breadboard

### Phase 2: Power System (~$70)
Buy next:
1. Battery + charger
2. ESCs
3. XT60 connectors
4. Multimeter

**Goal:** Test ESC response & power delivery

### Phase 3: Propulsion (~$80)
Buy next:
1. Motors
2. Propellers
3. Motor mounts

**Goal:** Verify motor/ESC compatibility (without props!)

### Phase 4: Integration (~$150)
Buy finally:
1. Frame
2. MS5611
3. All mounting hardware
4. RC controller enclosure

**Goal:** Complete build & first flight

---

## 🔬 Quality Indicators

### Motors
✅ Smooth bearing rotation (no grinding)  
✅ No visible shaft wobble  
✅ Wires firmly attached  
✅ Includes mounting screws  
⚠️ Avoid: Scratched magnets, rust, missing parts

### ESCs
✅ Includes BEC wiring diagram  
✅ Firmware clearly labeled (SimonK/BLHeli)  
✅ Has calibration instructions  
⚠️ Avoid: No-name brands, <15A rating

### Frame
✅ All screw holes align  
✅ No cracks in arms  
✅ Includes hardware kit  
✅ Protective film on carbon fiber  
⚠️ Avoid: Very thin arms (<3mm), visible defects

### Battery
✅ Branded cells (Turnigy, Zippy, Tattu)  
✅ Date stamp within 1 year  
✅ Discharge rate clearly marked  
✅ Includes balance connector  
⚠️ Avoid: Puffed/swollen, no markings, very cheap (<$10/cell)

---

## 📊 Price Comparison Tips

### Finding Deals
1. **Bundle Discounts:** Buy motor+ESC+prop combos
2. **Seller Coupons:** Check for first-time buyer codes
3. **Holiday Sales:** Black Friday, 11.11, Prime Day
4. **Open Box:** Check returns section (inspect carefully)
5. **Group Buys:** Split shipping with fellow builders

### Red Flags
🚩 Price too good to be true (<50% of average)  
🚩 No reviews or ratings  
🚩 Stock photos only (no actual product)  
🚩 Vague specifications  
🚩 "Compatible with DJI" (often misleading)

---

## 📝 Bill of Materials Template

Use this for your build:

```
╔════════════════════════════════════════════════════════╗
║           MY DRONE BUILD - PARTS LIST                  ║
╠════════════════════════════════════════════════════════╣
║ COMPONENT         QTY   SPECS          PRICE   SOURCE  ║
╠════════════════════════════════════════════════════════╣
║ Arduino Nano      2     ATmega328P     $___    ______  ║
║ MPU6050           1     GY-521         $___    ______  ║
║ MS5611            1     I2C            $___    ______  ║
║ NRF24L01          2     +Adapter       $___    ______  ║
║ Motors            4     ____KV         $___    ______  ║
║ ESCs              4     ____A BEC      $___    ______  ║
║ Props             2set  ____"          $___    ______  ║
║ Frame             1     ____mm         $___    ______  ║
║ Battery           1     __S ____mAh    $___    ______  ║
║ Charger           1     ____________   $___    ______  ║
║ Joysticks         2     2-axis         $___    ______  ║
║ Buttons           2     12mm           $___    ______  ║
║ Switches          2     SPDT           $___    ______  ║
║ [Add more...]                                           ║
╠════════════════════════════════════════════════════════╣
║                           TOTAL:       $___            ║
╚════════════════════════════════════════════════════════╝

Notes:
_____________________________________________________________
_____________________________________________________________
```

---

## 🚀 Ready to Buy?

**Recommendation:** Start with Phase 1 components to test the code and communication before investing in expensive motors/frame.

**Pro Tip:** Join online communities (r/Multicopter, DIYDrones forums) to ask for deals and recommendations based on your location!

**Happy Building! 🛠️**
