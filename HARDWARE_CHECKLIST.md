# 🔧 Hardware Checklist & Shopping List

## 📦 Parts List

### Flight Controller Assembly

#### Core Components
| Item | Quantity | Specification | Notes | Est. Price |
|------|----------|---------------|-------|-----------|
| Arduino Nano | 1 | ATmega328P | Clone OK | $3-5 |
| MPU6050 | 1 | GY-521 module | 6-axis IMU | $2-3 |
| MS5611 | 1 | GY-63 or similar | Barometer | $5-8 |
| NRF24L01+ | 1 | With PA+LNA | Better range | $3-5 |
| NRF Adapter | 1 | 3.3V regulator + cap | Recommended | $1-2 |

#### Motor & ESC
| Item | Quantity | Specification | Notes | Est. Price |
|------|----------|---------------|-------|-----------|
| Brushless Motors | 4 | 1000-2200 KV | Match to frame size | $8-15 ea |
| ESCs | 4 | 30A minimum | BLHeli preferred | $10-15 ea |
| Propellers | 4+ | 9-10" (250mm frame) | Get spares! | $1-2 ea |

#### Power System
| Item | Quantity | Specification | Notes | Est. Price |
|------|----------|---------------|-------|-----------|
| LiPo Battery | 1 | 3S (11.1V) 2200mAh | 30C+ discharge | $20-30 |
| XT60 Connectors | 2 | Male + Female | For battery | $2 |
| Power Distribution | 1 | PDB or wires | Distribute power to ESCs | $3-5 |
| Battery Alarm | 1 | 3S voltage monitor | Safety! | $2-3 |

#### Outputs & Indicators
| Item | Quantity | Specification | Notes | Est. Price |
|------|----------|---------------|-------|-----------|
| Buzzer | 1 | Active 5V | Piezo OK | $1 |
| LED | 1 | 5mm any color | With resistor | $0.50 |
| 220Ω Resistor | 1 | 1/4W | For LED | $0.10 |

#### Frame & Mounting
| Item | Quantity | Specification | Notes | Est. Price |
|------|----------|---------------|-------|-----------|
| Drone Frame | 1 | 250mm recommended | Includes arms | $15-30 |
| Vibration Dampers | 4 | Rubber grommets | For FC board | $2 |
| Zip Ties | 10+ | Small | Cable management | $2 |
| Velcro Straps | 2 | Battery mounting | Reusable | $3 |
| Double-sided Tape | 1 | 3M VHB preferred | Secure components | $3 |

#### Wiring
| Item | Quantity | Specification | Notes | Est. Price |
|------|----------|---------------|-------|-----------|
| Jumper Wires | 20+ | Dupont M-F, M-M | Connections | $3 |
| Silicone Wire | 1m | 20 AWG | Power wiring | $2 |
| Heat Shrink | Assorted | Various sizes | Protect connections | $5 |
| Solder | 1 roll | 60/40 or lead-free | For connections | $5 |

**Flight Controller Total: ~$150-250**

---

### Remote Controller Assembly

#### Core Components
| Item | Quantity | Specification | Notes | Est. Price |
|------|----------|---------------|-------|-----------|
| Arduino Nano | 1 | ATmega328P | Clone OK | $3-5 |
| NRF24L01+ | 1 | With PA+LNA | Match FC module | $3-5 |
| NRF Adapter | 1 | 3.3V regulator | Recommended | $1-2 |
| Dual Joysticks | 2 | 10K analog | PS2-style | $2-3 ea |
| Toggle Switches | 2 | SPDT | Any size | $1 ea |
| Push Buttons | 2 | Momentary | NO type | $0.50 ea |

#### Power & Enclosure
| Item | Quantity | Specification | Notes | Est. Price |
|------|----------|---------------|-------|-----------|
| Battery | 1 | 9V or 2S LiPo | 9V easier | $5-10 |
| Battery Holder | 1 | For 9V | If using 9V | $1 |
| Project Box | 1 | ~15x10x5cm | Plastic enclosure | $5-10 |
| Panel Mount USB | 1 | Micro USB | Optional: charging | $2 |

#### Wiring
| Item | Quantity | Specification | Notes | Est. Price |
|------|----------|---------------|-------|-----------|
| Jumper Wires | 20+ | Dupont M-M | Connections | $2 |
| Perf Board | 1 | Optional | Cleaner build | $2 |

**Remote Controller Total: ~$40-60**

---

## 🛠 Tools Required

### Essential
- [ ] Soldering iron (25-60W)
- [ ] Solder wire
- [ ] Wire cutters/strippers
- [ ] Screwdriver set (hex + Phillips)
- [ ] Multimeter (voltage check)
- [ ] USB cable (Arduino programming)

### Recommended
- [ ] Helping hands/PCB holder
- [ ] Heat gun (heat shrink)
- [ ] Propeller balancer
- [ ] LiPo safe bag
- [ ] Balance charger (for LiPo)

### Safety Equipment
- [ ] Safety goggles
- [ ] Fire extinguisher (nearby)
- [ ] Ventilated workspace
- [ ] ESD wrist strap (optional)

---

## ✅ Assembly Checklist

### Flight Controller Build

#### Phase 1: Prepare Components
- [ ] Test Arduino Nano (upload blink sketch)
- [ ] Test NRF24L01 (simple transmit test)
- [ ] Verify MPU6050 I2C address (0x68)
- [ ] Verify MS5611 I2C address (0x77)
- [ ] Solder headers if needed

#### Phase 2: Wire Sensors
- [ ] Connect MPU6050 (SDA→A4, SCL→A5, VCC, GND)
- [ ] Connect MS5611 (SDA→A4, SCL→A5, VCC, GND)
- [ ] Add 10µF capacitor to NRF VCC/GND
- [ ] Connect NRF24L01 (CE→D4, CSN→D10, SPI pins)
- [ ] Test I2C scanner sketch

#### Phase 3: Wire Outputs
- [ ] Connect LED to D7 (via 220Ω resistor to GND)
- [ ] Connect buzzer to D8 (other pin to GND)
- [ ] Test outputs with simple sketch

#### Phase 4: Wire ESCs
- [ ] Solder XT60 connector to battery
- [ ] Set up power distribution
- [ ] Connect ESC signal wires:
  - [ ] FL ESC → D3
  - [ ] FR ESC → D5
  - [ ] RR ESC → D6
  - [ ] RL ESC → D9
- [ ] Connect ESC grounds to Arduino GND
- [ ] Double-check polarity!

#### Phase 5: Mount to Frame
- [ ] Install vibration dampers
- [ ] Mount Arduino + breadboard
- [ ] Secure ESCs to arms
- [ ] Mount motors (correct orientation!)
- [ ] Install propellers (REMOVE for testing!)
- [ ] Cable management (zip ties)
- [ ] Secure battery mounting

#### Phase 6: Software & Testing
- [ ] Upload Drone_Flight_Control.ino
- [ ] Open Serial Monitor (57600)
- [ ] Verify startup sequence
- [ ] Check sensor readings
- [ ] Test outputs (LED, buzzer)

---

### Remote Controller Build

#### Phase 1: Prepare Enclosure
- [ ] Drill holes for joysticks
- [ ] Drill holes for switches/buttons
- [ ] Sand/debur holes
- [ ] Plan internal layout

#### Phase 2: Install Components
- [ ] Mount joysticks
- [ ] Install switches (D2, D3)
- [ ] Install buttons (D4, D5)
- [ ] Mount Arduino Nano inside
- [ ] Install NRF24L01 (+ adapter)
- [ ] Add power switch

#### Phase 3: Wiring
- [ ] Wire joysticks to analog pins:
  - [ ] Left Y → A0 (Throttle)
  - [ ] Left X → A1 (Yaw)
  - [ ] Right Y → A2 (Pitch)
  - [ ] Right X → A3 (Roll)
- [ ] Wire switches/buttons (D2-D5)
- [ ] Wire NRF24L01
- [ ] Connect power (9V or 2S LiPo)
- [ ] Add power switch

#### Phase 4: Testing
- [ ] Upload Controller.ino
- [ ] Open Serial Monitor (57600)
- [ ] Test all joystick axes
- [ ] Test all buttons/switches
- [ ] Verify NRF transmission

---

## 🔍 Pre-Flight Verification

### Visual Inspection
- [ ] All connections secure
- [ ] No loose wires
- [ ] Propellers tight (correct rotation!)
- [ ] Battery secure
- [ ] No damaged components
- [ ] Frame screws tight

### Electrical Check
- [ ] Battery voltage >11.4V
- [ ] All grounds connected
- [ ] No shorts (multimeter check)
- [ ] ESC power LEDs on
- [ ] Arduino power LED on

### Communication Test
- [ ] RC powers on
- [ ] FC powers on
- [ ] Connection beep heard
- [ ] Serial shows "RC CONNECTED"
- [ ] LED indicates correctly

### Motor Test (NO PROPS!)
- [ ] Remove all propellers
- [ ] Run smooth motor start
- [ ] All 4 motors spin
- [ ] Correct rotation direction:
  - [ ] FL → CCW ⟲
  - [ ] FR → CW ⟳
  - [ ] RL → CW ⟳
  - [ ] RR → CCW ⟲
- [ ] No unusual vibration/noise

### Calibration
- [ ] Full calibration completed
- [ ] Success beeps heard
- [ ] Values saved to EEPROM
- [ ] Serial confirms calibration

### Final Arming Test (NO PROPS!)
- [ ] Arm via Button 2
- [ ] LED blinks when armed
- [ ] Throttle up → motors respond
- [ ] Disarm via Switch 1
- [ ] Motors stop immediately

---

## 📐 Wiring Verification Table

### Flight Controller

| Pin | Connected To | Wire Color | Verified |
|-----|--------------|------------|----------|
| **Sensors** |
| A4 (SDA) | MPU6050 + MS5611 SDA | Yellow | ☐ |
| A5 (SCL) | MPU6050 + MS5611 SCL | White | ☐ |
| **NRF24L01** |
| D4 | CE | Orange | ☐ |
| D10 | CSN | Yellow | ☐ |
| D11 | MOSI | Blue | ☐ |
| D12 | MISO | Green | ☐ |
| D13 | SCK | Purple | ☐ |
| 3.3V | VCC (via adapter) | Red | ☐ |
| GND | GND | Black | ☐ |
| **Motors** |
| D3 | FL ESC Signal | White | ☐ |
| D5 | FR ESC Signal | White | ☐ |
| D6 | RR ESC Signal | White | ☐ |
| D9 | RL ESC Signal | White | ☐ |
| **Outputs** |
| D7 | LED Anode (+) via 220Ω | Red | ☐ |
| D8 | Buzzer (+) | Red | ☐ |
| GND | LED Cathode (-) | Black | ☐ |
| GND | Buzzer (-) | Black | ☐ |
| **Power** |
| 5V | From ESC BEC or USB | Red | ☐ |
| GND | Common ground | Black | ☐ |

### Remote Controller

| Pin | Connected To | Wire Color | Verified |
|-----|--------------|------------|----------|
| **Joysticks** |
| A0 | Left Y (Throttle) | Yellow | ☐ |
| A1 | Left X (Yaw) | Blue | ☐ |
| A2 | Right Y (Pitch) | Green | ☐ |
| A3 | Right X (Roll) | Orange | ☐ |
| **Controls** |
| D2 | Switch 2 (Alt Hold) | White | ☐ |
| D3 | Switch 1 (Arm) | Gray | ☐ |
| D4 | Button 1 (Cal) | Purple | ☐ |
| D5 | Button 2 (Start) | Brown | ☐ |
| **NRF24L01** |
| D9 | CE | Orange | ☐ |
| D10 | CSN | Yellow | ☐ |
| D11 | MOSI | Blue | ☐ |
| D12 | MISO | Green | ☐ |
| D13 | SCK | Purple | ☐ |
| 3.3V | VCC | Red | ☐ |
| GND | GND | Black | ☐ |
| **Power** |
| VIN | Battery + | Red | ☐ |
| GND | Battery - | Black | ☐ |

---

## 🎨 Color Coding Recommendations

| Wire Purpose | Suggested Color | Notes |
|--------------|----------------|-------|
| Power (+) | Red | Always |
| Ground (-) | Black | Always |
| Signal | Yellow/White/Blue | Analog/Digital |
| I2C SDA | Yellow | Standard |
| I2C SCL | White | Standard |
| SPI MOSI | Blue | Standard |
| SPI MISO | Green | Standard |
| SPI SCK | Purple | Standard |

---

## 📊 Component Testing

### MPU6050 Test (I2C Scanner)
```cpp
// Upload I2C scanner sketch
// Expected: Device found at 0x68
```

### MS5611 Test
```cpp
// Upload MS5611 test sketch
// Expected: Pressure ~101325 Pa at sea level
```

### NRF24L01 Test
```cpp
// Upload RF24 "Getting Started" example
// Expected: Successful transmission
```

### ESC Test (Servo Sweep)
```cpp
// Upload Servo sweep example
// Expected: Motor spins smoothly (no props!)
```

---

## 💾 Backup & Documentation

### Before First Flight
- [ ] Photo of wiring (top view)
- [ ] Photo of wiring (bottom view)
- [ ] Note motor positions
- [ ] Note propeller types (CW/CCW)
- [ ] Record calibration values
- [ ] Save Arduino code version
- [ ] Document any modifications

### Keep These Files
- [ ] Arduino IDE code (all .ino files)
- [ ] Calibration values (from Serial)
- [ ] PID settings (if tuned)
- [ ] Wiring diagram (photos)

---

## 🆘 Common Build Issues

| Issue | Likely Cause | Solution |
|-------|--------------|----------|
| NRF won't initialize | Power/capacitor | Add 10µF cap, check 3.3V |
| MPU not found | Wiring/address | Check I2C scanner, verify 0x68 |
| MS5611 not found | Wiring/address | Check I2C scanner, verify 0x77 |
| ESC beeping | Not calibrated | Run ESC calibration |
| No buzzer sound | Polarity/dead | Check polarity, test with 5V |
| LED doesn't light | Resistor/polarity | Check 220Ω resistor, polarity |
| Arduino won't program | Driver/bootloader | Install CH340 driver, check bootloader |

---

## 📞 Final Checks Before Flight

- [ ] All items from Hardware Checklist completed
- [ ] Pre-Flight Verification passed
- [ ] Wiring Verification Table 100% checked
- [ ] Component Testing all passed
- [ ] Backup/photos taken
- [ ] First flight area scouted (open, safe)
- [ ] Safety equipment ready
- [ ] Fire extinguisher nearby
- [ ] Emergency procedures reviewed
- [ ] Assistant available (recommended)

---

**When all boxes are checked, you're ready to fly! 🚁**

*Print this checklist and keep it handy during assembly.*
