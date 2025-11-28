# 🔌 DETAILED CIRCUIT DIAGRAMS

## Complete Wiring Reference for Quadcopter Drone

---

## 📡 FLIGHT CONTROLLER - Complete Wiring

### Overview
```
┌─────────────────────────────────────────────────────────────────┐
│                    FLIGHT CONTROLLER SYSTEM                      │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐│
│  │                    Arduino Nano                             ││
│  │                    ATmega328P                               ││
│  │                                                             ││
│  │  [USB] ←── Programming & Power (5V)                        ││
│  │                                                             ││
│  │  Digital Pins:                    Analog Pins:             ││
│  │  D2  ── Main LED ───────┐        A0  (not used)           ││
│  │  D3  ── ESC1 (Motor FR) │        A1  (not used)           ││
│  │  D4  ── ESC2 (Motor RR) │        A2  (not used)           ││
│  │  D5  ── ESC3 (Motor RL) │        A3  (not used)           ││
│  │  D6  ── ESC4 (Motor FL) │        A4  ── SDA (I2C)         ││
│  │  D7  ── Buzzer ─────────┤        A5  ── SCL (I2C)         ││
│  │  D8  ── Status LED ─────┘                                  ││
│  │  D9  ── NRF CE                                             ││
│  │  D10 ── NRF CSN          Power:                            ││
│  │  D11 ── NRF MOSI         VIN ── BEC 5V (from ESC)         ││
│  │  D12 ── NRF MISO         5V  ── +5V output                ││
│  │  D13 ── NRF SCK          3.3V── +3.3V output (NRF!)       ││
│  │                          GND ── Ground (common)            ││
│  └────────────────────────────────────────────────────────────┘│
│                                                                  │
│  Sensors & Modules:                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │   MPU6050    │  │   MS5611     │  │  NRF24L01    │         │
│  │   (IMU)      │  │  (Barometer) │  │  (Radio RX)  │         │
│  │              │  │              │  │              │         │
│  │ VCC ── 5V    │  │ VCC ── 5V    │  │ VCC ── 3.3V  │ ⚠️     │
│  │ GND ── GND   │  │ GND ── GND   │  │ GND ── GND   │         │
│  │ SDA ── A4    │  │ SDA ── A4    │  │ CE  ── D9    │         │
│  │ SCL ── A5    │  │ SCL ── A5    │  │ CSN ── D10   │         │
│  │ (AD0── GND)  │  │              │  │ MOSI── D11   │         │
│  │              │  │              │  │ MISO── D12   │         │
│  │              │  │              │  │ SCK ── D13   │         │
│  └──────────────┘  └──────────────┘  └──────────────┘         │
│                                                                  │
│  Add capacitor here: ──┬── VCC (3.3V)                          │
│                      10µF                                       │
│                        ┴── GND                                  │
│                                                                  │
│  Motors & ESCs:                                                 │
│  ┌──────────────┐  ┌──────────────┐                            │
│  │   ESC 1      │  │   ESC 2      │  (Repeat for ESC 3 & 4)   │
│  │              │  │              │                             │
│  │ Signal─ D3   │  │ Signal─ D4   │                             │
│  │ +5V BEC──VIN │  │ +5V BEC─(opt)│                             │
│  │ GND ──── GND │  │ GND ──── GND │                             │
│  │              │  │              │                             │
│  │ Motor+ ────┐ │  │ Motor+ ────┐ │                             │
│  │ Motor- ────┤ │  │ Motor- ────┤ │                             │
│  │ Motor3 ────┘ │  │ Motor3 ────┘ │                             │
│  │              │  │              │                             │
│  │ Battery+ ────┬─── LiPo 11.1V 3S ────┐                       │
│  │ Battery- ────┴────────────────────────┴─ GND (power)        │
│  └──────────────┘  └──────────────┘                            │
│                                                                  │
│  Indicators:                                                    │
│  Main LED:                    Status LED:                       │
│  D2 ──┬── [LED] ──┐          D8 ──┬── [LED] ──┐               │
│       330Ω        │                330Ω        │               │
│                GND                          GND                │
│                                                                  │
│  Buzzer (Active):                                               │
│  D7 ──── [BUZZER+]                                              │
│  GND ─── [BUZZER-]                                              │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🎮 RC TRANSMITTER - Complete Wiring

### Overview
```
┌─────────────────────────────────────────────────────────────────┐
│                    RC TRANSMITTER SYSTEM                         │
│                                                                  │
│  ┌────────────────────────────────────────────────────────────┐│
│  │                    Arduino Nano                             ││
│  │                    ATmega328P                               ││
│  │                                                             ││
│  │  [USB] ←── Programming & Power (5V)                        ││
│  │                                                             ││
│  │  Digital Pins:                    Analog Pins:             ││
│  │  D2  ── Toggle Switch            A0 ── Joy1 X (Throttle)  ││
│  │  D3  ── Button 1 ────┐           A1 ── Joy1 Y (Yaw)       ││
│  │  D4  ── Button 2 ────┤           A2 ── Joy2 X (Pitch)     ││
│  │  D5  (not used)      │           A3 ── Joy2 Y (Roll)      ││
│  │  D6  (not used)      │           A4 ── (not used)         ││
│  │  D7  (not used)      │           A5 ── (not used)         ││
│  │  D8  ── Status LED ──┘                                     ││
│  │  D9  ── NRF CE                                             ││
│  │  D10 ── NRF CSN          Power:                            ││
│  │  D11 ── NRF MOSI         VIN ── Battery or USB             ││
│  │  D12 ── NRF MISO         5V  ── +5V output (Joysticks)    ││
│  │  D13 ── NRF SCK          3.3V── +3.3V output (NRF!)       ││
│  │                          GND ── Ground (common)            ││
│  └────────────────────────────────────────────────────────────┘│
│                                                                  │
│  Radio Module:                                                  │
│  ┌──────────────┐                                               │
│  │  NRF24L01    │                                               │
│  │  (Radio TX)  │                                               │
│  │              │                                               │
│  │ VCC ── 3.3V  │ ⚠️ NOT 5V!                                   │
│  │ GND ── GND   │                                               │
│  │ CE  ── D9    │   Add capacitor: ──┬── VCC                   │
│  │ CSN ── D10   │                   10µF                       │
│  │ MOSI── D11   │                     ┴── GND                  │
│  │ MISO── D12   │                                               │
│  │ SCK ── D13   │                                               │
│  └──────────────┘                                               │
│                                                                  │
│  Joysticks:                                                     │
│  ┌──────────────────┐      ┌──────────────────┐               │
│  │  Joystick 1      │      │  Joystick 2      │               │
│  │  (Left Stick)    │      │  (Right Stick)   │               │
│  │                  │      │                  │               │
│  │  GND ─── GND     │      │  GND ─── GND     │               │
│  │  +5V ─── 5V      │      │  +5V ─── 5V      │               │
│  │  VRx ─── A0      │      │  VRx ─── A2      │               │
│  │  VRy ─── A1      │      │  VRy ─── A3      │               │
│  │  SW  ─── (n/c)   │      │  SW  ─── (n/c)   │               │
│  │                  │      │                  │               │
│  │    ▲ Throttle   │      │    ▲ Pitch      │               │
│  │    │             │      │    │             │               │
│  │  ◄─┼─► Yaw      │      │  ◄─┼─► Roll     │               │
│  │    │             │      │    │             │               │
│  │    ▼             │      │    ▼             │               │
│  └──────────────────┘      └──────────────────┘               │
│                                                                  │
│  Controls:                                                      │
│  ┌──────────────────────────────────┐                          │
│  │  Toggle Switch (SPDT)            │                          │
│  │                                   │                          │
│  │  Common ─── D2                    │                          │
│  │  NO ──────── GND                  │                          │
│  │  NC ──────── (open)               │                          │
│  │                                   │  Internal pullup ON      │
│  │  When switch UP: D2 reads HIGH (KILL)                       │
│  │  When switch DOWN: D2 reads LOW (ARM)                       │
│  └──────────────────────────────────┘                          │
│                                                                  │
│  ┌─────────────────┐      ┌─────────────────┐                 │
│  │  Button 1       │      │  Button 2       │                 │
│  │  (Calibrate)    │      │  (Motor Test)   │                 │
│  │                 │      │                 │                 │
│  │  Pin 1 ── D3    │      │  Pin 1 ── D4    │                 │
│  │  Pin 2 ── GND   │      │  Pin 2 ── GND   │                 │
│  │                 │      │                 │                 │
│  │  Internal       │      │  Internal       │                 │
│  │  pullup: ON     │      │  pullup: ON     │                 │
│  └─────────────────┘      └─────────────────┘                 │
│                                                                  │
│  Indicator:                                                     │
│  Status LED:                                                    │
│  D8 ──┬── [LED] ──┐                                            │
│       330Ω        │                                            │
│                GND                                             │
│                                                                  │
│  Power Options:                                                 │
│  Option 1: USB Power (5V) ───► [USB Port]                     │
│  Option 2: Battery (6-12V) ──► [VIN] (regulated to 5V)        │
│  Option 3: Direct 5V ────────► [5V Pin] (bypass regulator)    │
└─────────────────────────────────────────────────────────────────┘
```

---

## ⚡ POWER DISTRIBUTION

### Flight Controller Power
```
┌────────────────────────────────────────────┐
│         POWER DISTRIBUTION                  │
│                                             │
│  LiPo Battery 11.1V (3S)                   │
│      │                                      │
│      ├──► ESC 1 ──► Motor 1 (11.1V)        │
│      ├──► ESC 2 ──► Motor 2 (11.1V)        │
│      ├──► ESC 3 ──► Motor 3 (11.1V)        │
│      └──► ESC 4 ──► Motor 4 (11.1V)        │
│                                             │
│  ESC BEC Output (5V @ 1-3A)                │
│      │                                      │
│      └──► Arduino VIN ──┬──► Arduino 5V    │
│                         │                   │
│                         ├──► MPU6050 (5V)  │
│                         ├──► MS5611 (5V)   │
│                         ├──► Buzzer (5V)   │
│                         └──► LEDs (via res)│
│                                             │
│  Arduino 3.3V Regulator                    │
│      │                                      │
│      └──► NRF24L01 (3.3V) ⚠️              │
│             │                               │
│             └──[10µF capacitor]── GND      │
│                                             │
│  Ground (Common)                            │
│      │                                      │
│      └──► All GND pins connected together  │
└────────────────────────────────────────────┘
```

### RC Transmitter Power
```
┌────────────────────────────────────────────┐
│         POWER DISTRIBUTION                  │
│                                             │
│  Power Source (choose one):                │
│    • USB (5V)                              │
│    • 2S LiPo (7.4V) via VIN               │
│    • 4x AA batteries (6V) via VIN         │
│    • 5V regulated source via 5V pin        │
│      │                                      │
│      ├──► Arduino Nano                     │
│      │                                      │
│      ├──► Joysticks (5V)                   │
│      │                                      │
│      └──► Arduino 3.3V regulator           │
│             │                               │
│             └──► NRF24L01 (3.3V) ⚠️       │
│                    │                        │
│                    └──[10µF cap]── GND     │
│                                             │
│  Ground (Common):                           │
│      All GND pins connected                │
└────────────────────────────────────────────┘
```

---

## 🔧 I2C BUS CONFIGURATION

### I2C Device Addresses
```
┌────────────────────────────────────────────┐
│         I2C BUS (SCL=A5, SDA=A4)           │
│                                             │
│  Arduino Nano                               │
│      │                                      │
│  A4 (SDA) ──┬────────┬─────────┐          │
│             │        │         │          │
│         MPU6050  MS5611   (future)        │
│         0x68     0x77    devices          │
│             │        │         │          │
│  A5 (SCL) ──┴────────┴─────────┘          │
│                                             │
│  Pull-up Resistors (if needed):            │
│    SDA ──[4.7kΩ]── 5V                     │
│    SCL ──[4.7kΩ]── 5V                     │
│                                             │
│  Note: Most modules have built-in pullups  │
│        Add external only if detection fails│
└────────────────────────────────────────────┘

Device Addresses:
  MPU6050: 0x68 (default) or 0x69 (if AD0=HIGH)
  MS5611:  0x77 (default) or 0x76 (some modules)
```

---

## 📡 SPI BUS CONFIGURATION

### NRF24L01 SPI Connection
```
┌────────────────────────────────────────────┐
│         SPI BUS (NRF24L01)                 │
│                                             │
│  Arduino Nano           NRF24L01            │
│                                             │
│  D9  (CE)  ────────────► CE                │
│  D10 (CSN) ────────────► CSN               │
│  D11 (MOSI)────────────► MOSI              │
│  D12 (MISO)◄────────────┤ MISO             │
│  D13 (SCK) ────────────► SCK               │
│                                             │
│  3.3V ──┬──────────────► VCC               │
│         │               ⚠️ NOT 5V!         │
│       [10µF]                                │
│         │                                   │
│  GND ───┴──────────────► GND               │
│                                             │
│  SPI Settings:                              │
│    Mode: 0                                  │
│    Bit Order: MSB First                     │
│    Clock: 10 MHz max                        │
│    Data Rate: 250 kbps / 1 Mbps / 2 Mbps  │
└────────────────────────────────────────────┘
```

---

## 🎛️ PWM OUTPUTS (ESC Control)

### ESC Signal Timing
```
┌────────────────────────────────────────────┐
│         PWM SIGNALS TO ESC                  │
│                                             │
│  Arduino Nano     PWM Signal                │
│                                             │
│  D3 ────► ESC1    ┌─┐       1000-2000µs   │
│                   │ │_______               │
│                   └─ 20ms ──┘              │
│                                             │
│  D4 ────► ESC2    Same timing              │
│  D5 ────► ESC3    Same timing              │
│  D6 ────► ESC4    Same timing              │
│                                             │
│  Signal Levels:                             │
│    1000µs = Motor stopped                  │
│    1100µs = Motor idle/armed               │
│    1500µs = 50% throttle                   │
│    2000µs = Full throttle                  │
│                                             │
│  Update Rate: 250Hz (4ms)                  │
│  Resolution: 1µs steps                     │
└────────────────────────────────────────────┘
```

---

## 🔌 CONNECTOR PINOUTS

### Arduino Nano Pinout Reference
```
                      ┌────────────────┐
                      │    Arduino     │
                      │      Nano      │
                      │                │
           D1/TX  ────┤ 1          30 ├──── VIN (7-12V input)
           D0/RX  ────┤ 2          29 ├──── GND
           RESET  ────┤ 3          28 ├──── RESET
           GND    ────┤ 4          27 ├──── 5V (5V output)
           D2     ────┤ 5          26 ├──── A7
           D3~    ────┤ 6          25 ├──── A6
           D4     ────┤ 7          24 ├──── A5/SCL
           D5~    ────┤ 8          23 ├──── A4/SDA
           D6~    ────┤ 9          22 ├──── A3
           D7     ────┤10          21 ├──── A2
           D8     ────┤11          20 ├──── A1
           D9~    ────┤12          19 ├──── A0
           D10~   ────┤13          18 ├──── AREF
           D11~   ────┤14          17 ├──── 3.3V (3.3V output)
           D12    ────┤15          16 ├──── D13/SCK
                      └────────────────┘
                           [USB]

  ~ = PWM capable
```

---

## 📊 VOLTAGE LEVELS

### Voltage Reference Table
```
┌─────────────────────────────────────────────────────┐
│  Component         │ Operating Voltage │ Note       │
├─────────────────────────────────────────────────────┤
│  Arduino Nano      │ 5V (regulated)    │ Via USB/VIN│
│  MPU6050           │ 3.3V or 5V        │ Check board│
│  MS5611            │ 3.3V or 5V        │ Check board│
│  NRF24L01          │ 3.3V ONLY! ⚠️    │ Damage at 5V│
│  ESC (signal)      │ 3.3V-5V           │ 5V safe    │
│  ESC (motor power) │ 7.4V-11.1V (2-3S) │ LiPo       │
│  Motors            │ 7.4V-11.1V        │ Match ESC  │
│  Buzzer            │ 5V (active type)  │ 3-12V range│
│  LEDs              │ 2V + 330Ω resistor│ Via Arduino│
│  Joysticks         │ 5V                │ Analog out │
└─────────────────────────────────────────────────────┘
```

---

## ⚠️ CRITICAL WARNINGS

### DO NOT:
1. ❌ **Connect NRF24L01 to 5V** - Use 3.3V only!
2. ❌ **Power Arduino from LiPo directly** - Use BEC or regulator
3. ❌ **Skip common ground** - All grounds must connect
4. ❌ **Reverse battery polarity** - LiPo can explode!
5. ❌ **Connect motors backwards** - Check rotation!

### DO:
1. ✅ **Add capacitor to NRF24** - 10µF on VCC/GND
2. ✅ **Use short wires for NRF** - <10cm for SPI
3. ✅ **Double-check all connections** - Before powering on
4. ✅ **Test with multimeter** - Verify voltages
5. ✅ **Insulate exposed connections** - Prevent shorts

---

## 🧪 CONNECTION TESTING

### Continuity Check (Multimeter)
```
Test before powering on:
  1. Arduino GND to Battery GND: CONTINUOUS
  2. ESC GND to Arduino GND: CONTINUOUS
  3. NRF GND to Arduino GND: CONTINUOUS
  4. MPU6050 GND to Arduino GND: CONTINUOUS
  5. All sensor GND together: CONTINUOUS
  6. VCC to GND: OPEN (no continuity!)
```

### Voltage Check (Multimeter)
```
Test after powering on (no props!):
  1. LiPo battery: 11.1V-12.6V
  2. Arduino VIN: 5V
  3. Arduino 5V pin: 4.9V-5.2V
  4. Arduino 3.3V pin: 3.2V-3.4V
  5. MPU6050 VCC: 5V (or 3.3V if board requires)
  6. NRF24 VCC: 3.3V (CRITICAL!)
  7. ESC signal pins: 0V-5V (should pulse when armed)
```

---

## 🎯 TROUBLESHOOTING CONNECTIONS

### If NRF Not Working:
1. Check voltage: Must be 3.3V
2. Add 10µF capacitor on VCC/GND
3. Verify SPI connections (MOSI, MISO, SCK, CE, CSN)
4. Try different NRF module (many are defective)

### If Sensors Not Detected:
1. Run I2C scanner sketch
2. Check SDA/SCL connections
3. Verify sensor power (5V or 3.3V depending on module)
4. Add pull-up resistors if needed (4.7kΩ)

### If Motors Not Spinning:
1. Check ESC signal wire connections (D3-D6)
2. Verify ESC ground connected to Arduino ground
3. Ensure ESC powered from battery (11.1V)
4. Check PWM signal with oscilloscope or logic analyzer

---

**Always double-check connections before powering on!**

**Document Version**: 1.0.0
**Last Updated**: November 2025
