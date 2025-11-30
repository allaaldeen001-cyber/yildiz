# Detailed Wiring Guide

Complete step-by-step wiring instructions for building the Professional Quadcopter Drone System.

---

## ⚠️ Safety First!

- **Disconnect all power** before making connections
- **Remove propellers** during all testing phases
- **Double-check polarity** before applying power
- **Use proper gauge wires** for high-current connections
- **Wear safety glasses** during motor tests

---

## Required Components

### Flight Controller Board
- [ ] 1× Arduino Nano (ATmega328P)
- [ ] 1× NRF24L01+ PA+LNA module with antenna
- [ ] 1× MPU6050 6-axis IMU module
- [ ] 1× Active buzzer (5V)
- [ ] 1× LED (any color) + 220Ω resistor
- [ ] 4× ESC (Electronic Speed Controllers) 20-30A
- [ ] 4× Brushless motors (1000-2300KV)
- [ ] 1× AMS1117-3.3V regulator (for NRF power)
- [ ] 1× 10µF capacitor (for NRF stability)
- [ ] 1× Breadboard or custom PCB
- [ ] Wire (22-26 AWG for signals, 18-20 AWG for power)

### Remote Controller Board
- [ ] 1× Arduino Nano (ATmega328P)
- [ ] 1× NRF24L01+ PA+LNA module with antenna
- [ ] 2× Dual-axis joysticks (10kΩ potentiometers)
- [ ] 3× Push buttons (momentary, normally open)
- [ ] 2× Toggle switches (SPDT or SPST)
- [ ] 5× 10kΩ resistors (pull-ups for buttons/switches)
- [ ] 1× AMS1117-3.3V regulator (for NRF power)
- [ ] 1× 10µF capacitor (for NRF stability)
- [ ] 1× Breadboard or custom PCB
- [ ] 9V battery or 7-12V power source

### Power & Misc
- [ ] 3S LiPo battery (11.1V, 2200-5000mAh)
- [ ] LiPo battery charger
- [ ] XT60 connectors
- [ ] Heat shrink tubing
- [ ] Cable ties
- [ ] Drone frame (450mm recommended)
- [ ] Propellers (matched to motors)

---

## Part 1: Flight Controller Wiring

### Step 1: Prepare the Arduino Nano

1. Mount Arduino Nano on breadboard or PCB
2. Connect GND and 5V rails to power bus

### Step 2: NRF24L01 Module with 3.3V Regulator

**⚠️ CRITICAL**: NRF24L01 requires stable 3.3V power!

#### Option A: Using AMS1117-3.3V Regulator (Recommended)

```
LiPo Battery (+) → VIN on Arduino Nano
                ↓
              5V Pin → AMS1117 Input
                       AMS1117 GND → GND
                       AMS1117 Output (3.3V) → NRF VCC
                       10µF capacitor across NRF VCC and GND
```

#### Wiring Table

| NRF24L01 Pin | Arduino Nano Pin | Notes |
|--------------|------------------|-------|
| VCC | 3.3V from AMS1117 | Add 10µF cap |
| GND | GND | Common ground |
| CE | D4 | Chip Enable |
| CSN | D10 | Chip Select |
| SCK | D13 | SPI Clock |
| MOSI | D11 | Master Out |
| MISO | D12 | Master In |
| IRQ | (not connected) | Optional |

**Wiring Steps:**
1. Solder AMS1117 regulator to small PCB
2. Connect 5V from Arduino → AMS1117 IN
3. Connect GND → AMS1117 GND
4. Solder 10µF capacitor: (+) to 3.3V OUT, (-) to GND
5. Connect AMS1117 OUT (3.3V) → NRF VCC
6. Solder NRF to adapter board if using breakout
7. Connect all signal wires using jumper wires

### Step 3: MPU6050 IMU Sensor

| MPU6050 Pin | Arduino Nano Pin | Notes |
|-------------|------------------|-------|
| VCC | 5V | 5V power |
| GND | GND | Ground |
| SCL | A5 | I2C Clock |
| SDA | A4 | I2C Data |
| INT | D2 | Interrupt (optional) |
| XDA | (not connected) | Auxiliary |
| XCL | (not connected) | Auxiliary |
| AD0 | GND | I2C address select |

**Wiring Steps:**
1. Connect VCC to 5V rail
2. Connect GND to ground rail
3. Connect SCL to A5 (yellow wire recommended)
4. Connect SDA to A4 (green wire recommended)
5. Connect INT to D2 (optional but recommended)
6. Ensure AD0 is grounded (default I2C address 0x68)

### Step 4: Buzzer

| Buzzer Pin | Arduino Nano Pin | Notes |
|------------|------------------|-------|
| Positive (+) | D8 | Signal pin |
| Negative (-) | GND | Ground |

**Wiring Steps:**
1. Identify buzzer polarity (+ is longer pin or marked)
2. Connect (+) to D8
3. Connect (-) to GND
4. If using passive buzzer, add 100Ω resistor in series

### Step 5: Status LED

| LED Pin | Connection | Notes |
|---------|------------|-------|
| Anode (+) | D7 → 220Ω → LED | Through resistor |
| Cathode (-) | GND | Ground |

**Wiring Steps:**
1. Connect 220Ω resistor to D7
2. Connect other end of resistor to LED anode (+, longer leg)
3. Connect LED cathode (-, shorter leg) to GND

### Step 6: ESC and Motor Connections

#### Motor Layout on Frame

```
         FRONT
    [FL]       [FR]
      ↻         ↺
       \   ↑   /
        \ FWD /
       X  |  X
        /   \
       /     \
      ↺       ↻
    [RL]     [RR]
         REAR
```

#### ESC Signal Wire Connections

| ESC | Arduino Pin | Motor Position |
|-----|-------------|----------------|
| FL ESC Signal | D3 | Front Left |
| FR ESC Signal | D5 | Front Right |
| RR ESC Signal | D6 | Rear Right |
| RL ESC Signal | D9 | Rear Left |

#### ESC Power Wiring

```
Battery (+) → Power Distribution Board (PDB)
              ↓
            ESC Red Wires (×4)
              
Battery (-) → PDB Ground
              ↓
            ESC Black Wires (×4)
            
ESC BEC (5V) → Arduino VIN (use only ONE ESC BEC)
ESC Ground   → Arduino GND
```

**Wiring Steps:**
1. **Motor to ESC**: Connect 3 motor wires to ESC (any order initially)
2. **ESC Power**: Connect ESC red wire to PDB positive (+)
3. **ESC Ground**: Connect ESC black wire to PDB negative (-)
4. **ESC Signal**: Connect ESC signal wire to Arduino pins (D3, D5, D6, D9)
5. **ESC Ground**: Connect ESC signal ground to Arduino GND
6. **BEC Power**: Use ONE ESC's BEC (red wire) to Arduino VIN
7. **Common Ground**: Connect ONE ESC's BEC ground to Arduino GND

⚠️ **Motor Direction Check**:
- After first test, if motor spins wrong direction, swap any 2 of the 3 motor wires

#### Correct Motor Rotation

- **FL (Front Left)**: Counter-clockwise ↻
- **FR (Front Right)**: Clockwise ↺
- **RR (Rear Right)**: Counter-clockwise ↻
- **RL (Rear Left)**: Clockwise ↺

### Step 7: Final Flight Controller Assembly

1. Mount all components on drone frame
2. Keep wires organized with cable ties
3. Place Arduino in center of frame (near center of gravity)
4. Mount MPU6050 with arrow pointing forward (align with flight direction)
5. Secure ESCs to motor arms
6. Ensure no wires touch propellers
7. Double-check all connections

---

## Part 2: Remote Controller Wiring

### Step 1: Arduino Nano Setup

1. Mount Arduino Nano on breadboard
2. Connect GND and 5V rails

### Step 2: NRF24L01 with 3.3V Regulator

Use same wiring as Flight Controller, but different pins:

| NRF24L01 Pin | Arduino Nano Pin | Notes |
|--------------|------------------|-------|
| VCC | 3.3V from AMS1117 | Add 10µF cap |
| GND | GND | Common ground |
| CE | D9 | Chip Enable |
| CSN | D10 | Chip Select |
| SCK | D13 | SPI Clock |
| MOSI | D11 | Master Out |
| MISO | D12 | Master In |

### Step 3: Left Joystick (Throttle & Yaw)

| Joystick Pin | Arduino Pin | Function |
|--------------|-------------|----------|
| VCC | 5V | Power |
| GND | GND | Ground |
| VRx (Horizontal) | A1 | Yaw control |
| VRy (Vertical) | A0 | Throttle control |
| SW (Button) | (not used) | Optional |

**Wiring Steps:**
1. Connect VCC to 5V rail
2. Connect GND to ground rail
3. Connect VRx to A1 (Yaw)
4. Connect VRy to A0 (Throttle)

### Step 4: Right Joystick (Pitch & Roll)

| Joystick Pin | Arduino Pin | Function |
|--------------|-------------|----------|
| VCC | 5V | Power |
| GND | GND | Ground |
| VRx (Horizontal) | A3 | Roll control |
| VRy (Vertical) | A2 | Pitch control |
| SW (Button) | (not used) | Optional |

**Wiring Steps:**
1. Connect VCC to 5V rail
2. Connect GND to ground rail
3. Connect VRx to A3 (Roll)
4. Connect VRy to A2 (Pitch)

### Step 5: Push Buttons (with Pull-up Resistors)

#### Button 1 (Calibration) - D4

```
       5V
        |
       10kΩ
        |
D4 -----+----(Button)---- GND
```

Repeat for Button 2 (D5) and Button 3 (D6).

**Simplified: Use Internal Pull-ups (No External Resistors Needed)**

| Button | Arduino Pin | Wiring |
|--------|-------------|--------|
| Button 1 | D4 | One terminal to D4, other to GND |
| Button 2 | D5 | One terminal to D5, other to GND |
| Button 3 | D6 | One terminal to D6, other to GND |

The code enables internal pull-ups, so external resistors are optional.

### Step 6: Toggle Switches

#### Switch 1 (Altitude Hold) - D2

```
       D2 -----(Switch)---- GND
       (internal pull-up enabled)
```

#### Switch 2 (Arming) - D3

```
       D3 -----(Switch)---- GND
       (internal pull-up enabled)
```

**Wiring Steps:**
1. Connect switch pin 1 to D2 (Switch 1) or D3 (Switch 2)
2. Connect switch pins 2 & 3 (common) to GND
3. Internal pull-ups are enabled in code

### Step 7: Power Supply

**Option A: 9V Battery**
- Connect 9V (+) to Arduino VIN
- Connect 9V (-) to Arduino GND

**Option B: 7-12V DC Adapter**
- Use barrel jack connector
- Connect to VIN and GND

### Step 8: Final Remote Controller Assembly

1. Mount all components in enclosure or on board
2. Label buttons and switches clearly
3. Position joysticks ergonomically
4. Secure all connections
5. Add power switch if desired
6. Test continuity with multimeter

---

## Part 3: Wiring Verification Checklist

### Flight Controller

- [ ] NRF24L01 has stable 3.3V power with capacitor
- [ ] MPU6050 connected to I2C (A4, A5)
- [ ] All 4 ESC signal wires connected to correct pins
- [ ] ESC ground wires connected to Arduino GND
- [ ] ONE ESC BEC powering Arduino (VIN)
- [ ] Buzzer connected to D8
- [ ] LED connected to D7 with resistor
- [ ] No short circuits between power and ground
- [ ] All solder joints secure and insulated
- [ ] Battery polarity correct

### Remote Controller

- [ ] NRF24L01 has stable 3.3V power with capacitor
- [ ] Left joystick connected to A0 (Throttle) and A1 (Yaw)
- [ ] Right joystick connected to A2 (Pitch) and A3 (Roll)
- [ ] Button 1 connected to D4
- [ ] Button 2 connected to D5
- [ ] Button 3 connected to D6
- [ ] Switch 1 connected to D2
- [ ] Switch 2 connected to D3
- [ ] Power supply connected correctly (7-12V)
- [ ] All connections secure

---

## Part 4: Visual Wiring Diagram

### Flight Controller Simplified Diagram

```
                   ┌─────────────────────┐
                   │   ARDUINO NANO FC   │
                   │                     │
    NRF24         │  D4 ─────────────── CE
    CE/CSN        │  D10 ────────────── CSN
                   │                     │
    MPU6050       │  A4 ─────────────── SDA
    SDA/SCL       │  A5 ─────────────── SCL
    INT           │  D2 ─────────────── INT
                   │                     │
    Motors        │  D3 ─────────────── FL ESC
                   │  D5 ─────────────── FR ESC
                   │  D6 ─────────────── RR ESC
                   │  D9 ─────────────── RL ESC
                   │                     │
    Buzzer        │  D8 ─────────────── +
                   │                     │
    LED           │  D7 ──┬─[220Ω]─┬── LED (+)
                   │       └─────────┴── LED (-)→GND
                   │                     │
    Power         │  VIN ────────────── ESC BEC 5V
                   │  GND ────────────── GND
                   │  3.3V ───────────── NRF VCC (via reg.)
                   └─────────────────────┘
```

### Remote Controller Simplified Diagram

```
                   ┌─────────────────────┐
                   │   ARDUINO NANO RC   │
                   │                     │
    NRF24         │  D9 ─────────────── CE
    CE/CSN        │  D10 ────────────── CSN
                   │                     │
    Left          │  A0 ─────────────── Throttle (VRy)
    Joystick      │  A1 ─────────────── Yaw (VRx)
                   │                     │
    Right         │  A2 ─────────────── Pitch (VRy)
    Joystick      │  A3 ─────────────── Roll (VRx)
                   │                     │
    Buttons       │  D4 ───┐
                   │        ├───────────── Buttons → GND
                   │  D5 ───┤
                   │  D6 ───┘
                   │                     │
    Switches      │  D2 ───┬───────────── Switches → GND
                   │  D3 ───┘
                   │                     │
    Power         │  VIN ────────────── 9V+ or 7-12V
                   │  GND ────────────── GND
                   │  5V ─────────────── Joysticks VCC
                   └─────────────────────┘
```

---

## Part 5: Testing Before Flight

### Flight Controller Tests (Without Propellers!)

1. **Power Test**
   - Connect battery
   - LED should light up
   - Buzzer should play startup tone

2. **NRF Communication Test**
   - Turn on Remote Controller
   - LED should blink when linked

3. **MPU6050 Test**
   - Open Serial Monitor (115200 baud)
   - Look for "MPU6050 Initialized"

4. **ESC Test**
   - Remove propellers!
   - Perform ESC calibration (Button 2)
   - Listen for motor beeps

5. **Motor Direction Test**
   - Use motor test (Button 3)
   - Check rotation direction
   - Swap motor wires if needed

### Remote Controller Tests

1. **Power Test**
   - Power on
   - Open Serial Monitor (115200 baud)

2. **Joystick Test**
   - Move joysticks
   - Verify values change (1000-2000)

3. **Button Test**
   - Press each button
   - Verify detection in Serial Monitor

4. **Switch Test**
   - Toggle switches
   - Verify status changes

---

## Troubleshooting

### NRF Not Connecting
- Check 3.3V voltage with multimeter (should be 3.2-3.4V)
- Add 10µF capacitor if not present
- Reduce wire length
- Check SPI connections (MOSI, MISO, SCK)

### MPU6050 Not Found
- Verify I2C connections (A4=SDA, A5=SCL)
- Check 5V power
- Ensure AD0 pin is grounded

### Motors Not Spinning
- Check ESC power connections
- Verify signal wire connections
- Perform ESC calibration
- Check motor phase wires

### Joystick Readings Unstable
- Add 0.1µF capacitor across VCC and GND of each joystick
- Check 5V power supply stability

---

**Last Updated**: 2025-11-30
