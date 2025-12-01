# 🔌 Visual Wiring Guide - Quadcopter Drone

**Step-by-step wiring with ASCII diagrams**

---

## 📋 Table of Contents

- [Tools & Materials](#tools--materials)
- [Remote Controller Wiring](#remote-controller-wiring)
- [Flight Controller Wiring](#flight-controller-wiring)
- [Motor Configuration](#motor-configuration)
- [Power Wiring](#power-wiring)
- [Final Assembly](#final-assembly)
- [Testing Checklist](#testing-checklist)

---

## 🛠 Tools & Materials

### Required Tools
- [ ] Soldering iron (30-60W)
- [ ] Solder (60/40 or lead-free)
- [ ] Wire strippers
- [ ] Multimeter
- [ ] Small screwdriver set
- [ ] Hot glue gun (optional)

### Materials
- [ ] Jumper wires (male-female, various lengths)
- [ ] Heat shrink tubing
- [ ] 10µF electrolytic capacitors (2x)
- [ ] 220Ω resistors (2x, for LED)
- [ ] Zip ties
- [ ] Double-sided tape

---

## 🎮 Remote Controller Wiring

### Step 1: Arduino Nano Base

```
┌────────────────────────────────────────┐
│         ARDUINO NANO (TOP VIEW)        │
│                                        │
│  D13 ┤●●●●●●●●●●●●●●●●┤ D12           │
│  3V3 ┤               ┤ D11           │
│  REF ┤               ┤ D10           │
│  A0  ┤               ┤ D9            │
│  A1  ┤               ┤ D8            │
│  A2  ┤               ┤ D7            │
│  A3  ┤               ┤ D6            │
│  A4  ┤               ┤ D5            │
│  A5  ┤               ┤ D4            │
│  A6  ┤               ┤ D3            │
│  A7  ┤               ┤ D2            │
│  5V  ┤               ┤ GND           │
│  RST ┤               ┤ RST           │
│  GND ┤               ┤ RX0           │
│  VIN ┤●●●●●●●●●●●●●●●●┤ TX1           │
│                                        │
│         ┌──────────┐                   │
│         │   USB    │                   │
│         └──────────┘                   │
└────────────────────────────────────────┘
```

### Step 2: Left Joystick (Throttle + Yaw)

```
LEFT JOYSTICK                    ARDUINO NANO
┌─────────────┐                  
│     VRx     │ ─────────────→  A1 (Yaw)
│     VRy     │ ─────────────→  A0 (Throttle)
│     SW      │ ─────────────→  (Not used)
│     +5V     │ ─────────────→  5V
│     GND     │ ─────────────→  GND
└─────────────┘

Wiring:
  Red wire   (VCC) → 5V
  Black wire (GND) → GND
  Green wire (VRx) → A1
  Yellow wire (VRy) → A0
```

### Step 3: Right Joystick (Pitch + Roll)

```
RIGHT JOYSTICK                   ARDUINO NANO
┌─────────────┐                  
│     VRx     │ ─────────────→  A3 (Roll)
│     VRy     │ ─────────────→  A2 (Pitch)
│     SW      │ ─────────────→  (Not used)
│     +5V     │ ─────────────→  5V
│     GND     │ ─────────────→  GND
└─────────────┘

Wiring:
  Red wire   (VCC) → 5V
  Black wire (GND) → GND
  Green wire (VRx) → A3
  Yellow wire (VRy) → A2
```

### Step 4: Buttons

```
BUTTON LAYOUT:

   [BTN1]  [BTN2]        Function:
   [BTN3]  [BTN4]        BTN1 = Calibrate
                         BTN2 = Motor Test
                         BTN3 = Landing
                         BTN4 = Takeoff

WIRING (all identical):
                         
Button → Arduino
┌──┐                     
│  ├────────→ D4, D5, D6, or D7
│  ├────────→ GND
└──┘

Internal pullup enabled in code!
Active LOW (pressed = 0)
```

### Step 5: Switches

```
SWITCH LAYOUT:

  [SW1]                  SW1 = Altitude Hold
  [SW2]                  SW2 = ANGLE/ACRO mode

WIRING (both identical):

Toggle Switch
┌─────┐
│  1  ├────────→ D2 or D3
│  2  ├────────→ (Not connected)
│  3  ├────────→ GND
└─────┘

Internal pullup enabled!
ON = LOW (connected to GND)
OFF = HIGH (floating, pulled up)
```

### Step 6: nRF24L01+ Radio

```
NRF24L01+ MODULE (BOTTOM VIEW)
┌─────────────────┐
│  ┌───────────┐  │
│  │  ANTENNA  │  │
│  │   CHIP    │  │
│  └───────────┘  │
│                 │
│ ● ● ● ●         │  Pin numbers
│ 1 2 3 4         │  (left to right)
│ ● ● ● ●         │
│ 5 6 7 8         │
└─────────────────┘

PIN MAPPING:
1. GND  → GND
2. VCC  → 3.3V (NOT 5V!)
3. CE   → D9
4. CSN  → D10
5. SCK  → D13
6. MOSI → D11
7. MISO → D12
8. IRQ  → (Not connected)

⚠️  CRITICAL: Add 10µF capacitor!
    
    VCC ─┬─ 10µF ─┬─ GND
         │        │
       (Place capacitor as close
        to nRF24 module as possible)
```

### Complete Remote Controller Schematic

```
                    ╔════════════════════════════════╗
                    ║      ARDUINO NANO (RC)         ║
                    ║                                ║
  LEFT JOYSTICK     ║  A0  ←───── Throttle (VRy)     ║
   ┌────────┐       ║  A1  ←───── Yaw (VRx)          ║
   │ VRy→A0 │       ║  A2  ←───── Pitch (VRy)        ║   RIGHT JOYSTICK
   │ VRx→A1 │       ║  A3  ←───── Roll (VRx)         ║    ┌────────┐
   │ +5V→5V │       ║                                ║    │ VRy→A2 │
   │ GND→G  │       ║  D2  ←───── SW1 (Alt Hold)     ║    │ VRx→A3 │
   └────────┘       ║  D3  ←───── SW2 (Angle/Acro)   ║    │ +5V→5V │
                    ║  D4  ←───── Button 1 (Calib)   ║    │ GND→G  │
  SWITCHES          ║  D5  ←───── Button 2 (Test)    ║    └────────┘
   SW1 ──→ D2       ║  D6  ←───── Button 3 (Land)    ║
   SW2 ──→ D3       ║  D7  ←───── Button 4 (Takeoff) ║   BUTTONS
   (both to GND)    ║                                ║    BTN1→D4
                    ║  D9  ────→ nRF24 CE            ║    BTN2→D5
  NRF24L01+         ║  D10 ────→ nRF24 CSN           ║    BTN3→D6
   ┌─────────┐      ║  D11 ────→ nRF24 MOSI          ║    BTN4→D7
   │ CE ←─D9 │      ║  D12 ←──── nRF24 MISO          ║    (all to GND)
   │ CSN←─D10│      ║  D13 ────→ nRF24 SCK           ║
   │ VCC→3.3V│◄─┬─  ║                                ║
   │ GND→GND │◄─┴─  ║  5V  ────→ All +5V connections ║   POWER
   │    [10µF]│     ║  GND ────→ All GND connections ║   9V Battery
   └─────────┘      ║  VIN ←──── 9V+ (or 3xAA)       ║    ┌──┬──┐
                    ║            GND← 9V-             ║    │+ │- │
                    ╚════════════════════════════════╝    └──┴──┘
```

---

## 🚁 Flight Controller Wiring

### Step 1: Arduino Nano Base (Same as RC)

### Step 2: MPU6050 (Gyro + Accelerometer)

```
MPU6050 MODULE                   ARDUINO NANO
┌──────────────┐
│              │
│   MPU6050    │
│              │
│  VCC ────────┼─────────→ 5V (or 3.3V)
│  GND ────────┼─────────→ GND
│  SCL ────────┼─────────→ A5 (SCL)
│  SDA ────────┼─────────→ A4 (SDA)
│  XDA ────────┼─────────→ (Not used)
│  XCL ────────┼─────────→ (Not used)
│  AD0 ────────┼─────────→ (Leave floating = 0x68)
│  INT ────────┼─────────→ D2 (optional)
└──────────────┘

Color coding (typical):
  Red    = VCC → 5V
  Black  = GND → GND
  Yellow = SCL → A5
  Green  = SDA → A4
```

### Step 3: MS5611 (Barometer)

```
MS5611 MODULE                    ARDUINO NANO
┌──────────────┐
│              │
│   MS5611     │
│              │
│  VCC ────────┼─────────→ 5V (or 3.3V)
│  GND ────────┼─────────→ GND
│  SCL ────────┼─────────→ A5 (SCL) ◄─┐
│  SDA ────────┼─────────→ A4 (SDA) ◄─┤ Shared I2C bus
│  CSB ────────┼─────────→ (Leave floating)   │ with MPU6050
│  SDO ────────┼─────────→ GND (= 0x76)       │
└──────────────┘                               │
                                               │
⚠️  Both MPU6050 and MS5611 share I2C! ────────┘
    Different addresses: MPU=0x68, MS5611=0x77
```

### Step 4: nRF24L01+ Radio (Same pinout as RC except CE pin)

```
NRF24L01+ (Flight Controller)

PIN MAPPING:
1. GND  → GND
2. VCC  → 3.3V
3. CE   → D4  ⚠️  DIFFERENT FROM RC! (RC uses D9)
4. CSN  → D10
5. SCK  → D13
6. MOSI → D11
7. MISO → D12
8. IRQ  → (Not connected)

⚠️  Don't forget 10µF capacitor!
```

### Step 5: Buzzer

```
PASSIVE BUZZER                   ARDUINO NANO
                                 
   ┌─────────┐
   │    +    ├───────────→ D8
   │         │
   │    -    ├───────────→ GND
   └─────────┘

Passive buzzer (can play tones)
NOT active buzzer (single beep only)
```

### Step 6: Status LED

```
STATUS LED                       ARDUINO NANO

   ┌─────┐
   │ LED │  Longer leg = +
   └──┬──┘
      │
     [220Ω]  ← Resistor
      │
      ├────────────────→ D7
      
   Shorter leg ──────→ GND

Any color LED works!
Resistor prevents burnout.
```

### Step 7: Motor ESCs

```
MOTOR/ESC CONNECTIONS

ESC Signal Wires:
  Front-Left Motor  ESC → D3
  Front-Right Motor ESC → D5
  Rear-Right Motor  ESC → D6
  Rear-Left Motor   ESC → D9

Each ESC has 3 wires:
  ┌──────────────┐
  │     ESC      │
  │              │
  │  Signal  ────┼───→ D3/D5/D6/D9 (to Arduino)
  │  +5V BEC ────┼───→ (Not connected on ESC 2-4)
  │  GND     ────┼───→ GND (all ESCs)
  └──────────────┘

⚠️  Only ONE ESC should have +5V (BEC) connected!
    Cut red wire on ESC 2, 3, 4 to prevent conflicts.
    
    ESC 1: All 3 wires connected (powers Arduino via VIN)
    ESC 2-4: Only signal + GND (red wire cut)
```

### Complete Flight Controller Schematic

```
                    ╔════════════════════════════════════╗
                    ║     ARDUINO NANO (FLIGHT CTRL)    ║
                    ║                                    ║
  MPU6050           ║  A4 (SDA) ←→ MPU6050 SDA          ║
   ┌──────┐         ║           ←→ MS5611 SDA           ║    MS5611
   │ SDA→A4│        ║  A5 (SCL) ←→ MPU6050 SCL          ║     ┌──────┐
   │ SCL→A5│        ║           ←→ MS5611 SCL           ║     │ SDA→A4│
   │ VCC→5V│        ║  D2       ←─ MPU6050 INT (opt)    ║     │ SCL→A5│
   │ GND→G │        ║                                    ║     │ VCC→5V│
   └──────┘         ║  D3  ─────→ Front-Left Motor ESC  ║     │ GND→G │
                    ║  D5  ─────→ Front-Right Motor ESC ║     └──────┘
  MOTORS/ESCs       ║  D6  ─────→ Rear-Right Motor ESC  ║
   FL ← D3          ║  D9  ─────→ Rear-Left Motor ESC   ║    NRF24L01+
   FR ← D5          ║                                    ║     ┌──────┐
   RR ← D6          ║  D4  ─────→ nRF24 CE              ║     │ CE→D4│
   RL ← D9          ║  D10 ─────→ nRF24 CSN             ║     │CSN→D10│
                    ║  D11 ─────→ nRF24 MOSI            ║     │VCC→3.3V│
  BUZZER            ║  D12 ←───── nRF24 MISO            ║     │GND→G │
   + → D8           ║  D13 ─────→ nRF24 SCK             ║     │ [10µF]│
   - → GND          ║                                    ║     └──────┘
                    ║  D7  ─────→ LED + [220Ω] → GND    ║
  LED               ║  D8  ─────→ Buzzer +              ║
   + → [220Ω] → D7  ║                                    ║    BATTERY
   - → GND          ║  VIN ←──── +11.1V (3S LiPo/BEC)   ║     3S LiPo
                    ║  5V  ─────→ Sensors VCC           ║    ┌───────┐
                    ║  GND ←──── Battery/ESC Ground     ║    │ 11.1V │
                    ╚════════════════════════════════════╝    └───────┘
```

---

## 🔄 Motor Configuration

### X-Configuration Layout

```
                    FRONT OF DRONE
                         ↑
                         
           FL                    FR
          ●────────────────────●
          │ ↺                ↻ │     ↺ = Counter-Clockwise
          │                    │     ↻ = Clockwise
          │                    │
          │         X          │     FL = Front-Left (D3)
          │                    │     FR = Front-Right (D5)
          │                    │     RR = Rear-Right (D6)
          │                    │     RL = Rear-Left (D9)
          │ ↻                ↺ │
          ●────────────────────●
          RL                   RR
          
                    ← REAR
```

### Motor Rotation Directions

**How to check motor rotation**:

1. **Visual inspection** (power OFF):
   - Look at propeller threads
   - CW threads = CW motor
   - CCW threads = CCW motor

2. **Spin test** (power ON, props OFF):
   - Apply low throttle
   - Watch motor spin direction from top
   - Compare to diagram above

3. **Reverse if needed**:
   - Swap ANY 2 of the 3 motor wires to ESC
   - Example: Swap wire 1 ↔ wire 2

### Propeller Installation

```
PROPELLER ORIENTATION:

Front-Left (FL):                Front-Right (FR):
   ↺ CCW                           ↻ CW
┌─────────┐                    ┌─────────┐
│    A    │  ← "A" marking     │    B    │  ← "B" or "R" marking
│  ┌───┐  │     on prop        │  ┌───┐  │     on prop
│  │ ● │  │  (or unmarked)     │  │ ● │  │  (or "R" for Reverse)
│  └───┘  │                    │  └───┘  │
└─────────┘                    └─────────┘

Rear-Left (RL):                 Rear-Right (RR):
   ↻ CW                            ↺ CCW
┌─────────┐                    ┌─────────┐
│    B    │                    │    A    │
│  ┌───┐  │                    │  ┌───┐  │
│  │ ● │  │                    │  │ ● │  │
│  └───┘  │                    │  └───┘  │
└─────────┘                    └─────────┘

⚠️  CRITICAL: Propellers MUST match motor rotation!
    Wrong props = instant flip on takeoff!
```

---

## 🔌 Power Wiring

### Option 1: Single BEC (Simplest)

```
        3S LiPo Battery
        ┌─────────────┐
        │   11.1V     │
        │  2200mAh    │
        └──┬────┬─────┘
           │    │
        ───┴────┴───  (Split 4 ways)
         │  │  │  │
         │  │  │  └──→ ESC 4 (Rear-Left)
         │  │  └─────→ ESC 3 (Rear-Right)
         │  └────────→ ESC 2 (Front-Right)
         └───────────→ ESC 1 (Front-Left)
                       │
                       ├─ Red (+5V BEC) ──→ Arduino VIN
                       ├─ Black (GND) ────→ Arduino GND
                       └─ Signal ─────────→ Arduino D3

ESC 2, 3, 4: Cut red wire! Only signal + GND.
```

### Option 2: Power Distribution Board (Better)

```
        3S LiPo Battery
        ┌─────────────┐
        │   11.1V     │
        └──┬────┬─────┘
           │    │
           │    └───→ PDB 5V Regulator (3A)
           │              │
           │              └────→ Arduino VIN (5V)
           │
           └────────→ PDB Main Pads
                      │  │  │  │
                      │  │  │  └──→ ESC 4 power
                      │  │  └─────→ ESC 3 power
                      │  └────────→ ESC 2 power
                      └───────────→ ESC 1 power
                      
All ESC signal wires → Arduino (D3, D5, D6, D9)
All ESC BEC wires → Disconnected (red wires cut)
Common ground → PDB → Arduino GND
```

### Voltage Regulator for Sensors

```
If using high-power nRF24L01+ PA+LNA:

        5V from Arduino
              │
              ├───→ MPU6050 VCC
              ├───→ MS5611 VCC
              │
              └───→ [AMS1117-3.3] Voltage Regulator
                          │
                          └───→ nRF24L01+ VCC (3.3V)
                          
Standard nRF24L01+ can use 3.3V output from Nano directly.
```

---

## 🔨 Final Assembly

### Assembly Order

1. **Mount Arduino to frame**
   - Use double-sided tape or standoffs
   - Keep away from propellers
   - Access to USB port

2. **Mount sensors**
   - MPU6050: Center of frame, flat orientation
   - MS5611: Away from propeller wash
   - Both secured with foam tape

3. **Install ESCs**
   - Along frame arms
   - Secure with zip ties
   - Keep wires tidy

4. **Connect motors**
   - Test rotation direction first
   - Use bullet connectors for easy swap

5. **Wire everything**
   - Follow schematics above
   - Use colored wires (red=power, black=ground)
   - Keep wires short and tidy

6. **Add capacitors**
   - 10µF on nRF24L01+ VCC/GND
   - As close to module as possible

7. **Secure components**
   - Hot glue, zip ties, or tape
   - Ensure nothing can vibrate loose

8. **Install propellers**
   - LAST STEP (safety!)
   - Verify orientation
   - Tighten securely

---

## ✅ Testing Checklist

### Power-On Test (No Props!)

- [ ] **Step 1**: Power RC, check Serial Monitor
  - Should see: "REMOTE CONTROLLER READY!"
  - Move joysticks, values change
  - Press buttons, shows button names

- [ ] **Step 2**: Power FC, check Serial Monitor
  - Should see: "SYSTEM READY!"
  - 2 beeps from buzzer
  - LED solid on

- [ ] **Step 3**: Verify radio connection
  - FC Serial shows: "RC:OK"
  - RC Serial shows: "TX: ✅ OK"

- [ ] **Step 4**: Test motor output
  - Press Button 2 (Motor Test)
  - All 4 motors spin briefly
  - No beeping/errors

- [ ] **Step 5**: Verify sensor readings
  - Tilt FC, Roll/Pitch values change
  - Lift FC, Altitude increases
  - Rotate FC, Gyro values change

### Pre-Flight Check (Props ON)

- [ ] **Battery**: Fully charged (>11V)
- [ ] **Propellers**: Tight, correct orientation
- [ ] **Wiring**: Secure, no loose connections
- [ ] **Area**: Clear of people/obstacles
- [ ] **Weather**: Calm, no wind
- [ ] **Calibration**: Gyro + altitude done
- [ ] **Radio**: RC connected, "RC:OK"
- [ ] **Emergency**: Know how to disarm (Button 3)

### First Flight Test

- [ ] **Auto Takeoff**: Press Button 4
  - Rises smoothly to 1.5m
  - Hovers stably
  
- [ ] **Manual Control**: Test each axis
  - Roll left/right
  - Pitch forward/back
  - Yaw left/right
  - Throttle up/down
  
- [ ] **Auto Landing**: Press Button 3
  - Descends gently
  - Disarms at ground
  - 3 beeps

### Troubleshooting During Test

**Flips on takeoff**:
→ Check motor directions (see diagram above)

**Drifts in one direction**:
→ Recalibrate gyro (Button 1)

**Won't ARM**:
→ Check radio connection, lower throttle

**Oscillates/vibrates**:
→ Reduce PID P gain (see PID_TUNING.md)

---

## 📐 Wire Management Tips

### Color Coding

Use consistent colors:
- **Red**: +5V, +3.3V, VCC
- **Black**: GND
- **Yellow**: Signal (ESC, buttons)
- **Green**: I2C SDA
- **Blue**: I2C SCL
- **White**: Other signals

### Wire Lengths

Keep wires short but not too tight:
- **Sensors**: 10-15cm
- **ESCs**: Length of arm + 5cm
- **Motors**: 15-20cm from ESC

### Securing Wires

- **Zip ties**: Along frame arms
- **Hot glue**: Connector strain relief
- **Heat shrink**: Exposed solder joints
- **Foam tape**: Sensor mounting

---

## ⚠️ Safety Warnings

### Before Every Flight

1. **Remove props** during all testing/debugging
2. **Check connections** - one loose wire = crash
3. **Verify battery voltage** - low voltage = unstable flight
4. **Clear the area** - 5+ meter radius
5. **Emergency plan** - know how to disarm immediately

### During Wiring

1. **Disconnect battery** when making changes
2. **Check polarity** before connecting power
3. **No short circuits** - keep wires separated
4. **Test with multimeter** before powering on

### Power Safety

1. **LiPo batteries** can catch fire if damaged
2. **Never** over-discharge (below 3.0V per cell)
3. **Always** use LiPo-safe charging bag
4. **Store** at 3.8V per cell (storage charge)

---

**Wiring Complete! Ready to Fly! 🚁**

*Double-check everything, be patient, and prioritize safety!*
