# 🔌 Wiring Diagram & Connections

## 📐 System Overview

```
┌─────────────┐         ┌──────────────┐
│ RC Transmit │  Radio  │   Flight     │
│  (Arduino)  │◄───────►│  Controller  │
│             │ nRF24L01│  (Arduino)   │
└─────────────┘         └──────────────┘
      │                        │
      │                        │
   Joysticks              Motors + Sensors
```

---

## 🚁 Flight Controller Connections

### Arduino Nano Pinout

```
                    ┌─────────┐
              RESET─┤1      30├─VIN (7-12V)
                   ─┤2      29├─GND
                   ─┤3  A   28├─RESET
                   ─┤4  R   27├─+5V
                   ─┤5  D   26├─A7
                   ─┤6  U   25├─A6
                   ─┤7  I   24├─A5 (SCL) ──┐
                   ─┤8  N   23├─A4 (SDA) ──┼─ MPU6050 + MS5611
                   ─┤9  O   22├─A3        │
                   ─┤10     21├─A2 (Echo) ─── Ultrasonic
        nRF24(CE)───┤11     20├─A1 (Trig) ─── Ultrasonic
                   ─┤12     19├─A0 ─────────── Battery Monitor
      ESC(FL)───────┤13  N  18├─+5V
                   ─┤14  A  17├─D13
      ESC(FR)───────┤15  N  16├─D12
      ESC(RR)───────┤16  O  15├─D11(MOSI) ─┐
            LED─────┤17     14├─D10(CSN)  ─┼─ nRF24L01
         BUZZER─────┤18     13├─D9(ESC-RL) │
      ESC(RL)───────┤19      12├─D8        │
                GND─┤20      11├─D7(MISO)──┘
                    └─────────┘
```

---

## 📡 Flight Controller Detailed Connections

### 1. nRF24L01+ Radio Module

```
nRF24L01+          Arduino Nano
─────────          ────────────
  VCC    ─────────  3.3V (or 5V with regulator)
  GND    ─────────  GND
  CE     ─────────  D4
  CSN    ─────────  D10
  SCK    ─────────  D13
  MOSI   ─────────  D11
  MISO   ─────────  D12
  IRQ    ─────────  (not connected)
```

**IMPORTANT**: 
- Add 10µF capacitor between VCC and GND
- Use 3.3V or add voltage regulator
- Keep wires short (<10cm)

---

### 2. MPU6050 Gyroscope/Accelerometer

```
MPU6050            Arduino Nano
────────           ────────────
  VCC    ─────────  5V
  GND    ─────────  GND
  SCL    ─────────  A5 (SCL)
  SDA    ─────────  A4 (SDA)
  XDA    ─────────  (not connected)
  XCL    ─────────  (not connected)
  AD0    ─────────  GND (for 0x68 address)
  INT    ─────────  (not connected)
```

**MOUNTING**:
- Must be mounted FLAT on board
- Arrows should point FORWARD
- Use foam tape to reduce vibrations

---

### 3. MS5611 Barometer (Optional - for altitude hold)

```
MS5611             Arduino Nano
───────            ────────────
  VCC    ─────────  3.3V or 5V
  GND    ─────────  GND
  SCL    ─────────  A5 (SCL) [shared with MPU6050]
  SDA    ─────────  A4 (SDA) [shared with MPU6050]
```

**NOTE**: 
- Shares I2C bus with MPU6050
- Address: 0x77
- Optional but enables altitude hold

---

### 4. ESCs (Electronic Speed Controllers)

```
ESC Signal Wires:
─────────────────
Front Left  (FL)  ──────  D3
Front Right (FR)  ──────  D5
Rear Right  (RR)  ──────  D6
Rear Left   (RL)  ──────  D9

ESC Power:
──────────
All ESC GND  ────────  Arduino GND (common ground)
ESC +5V/BEC  ────────  Arduino VIN (power from one ESC)
```

**Motor Layout (X Configuration)**:
```
     FRONT
  FL     FR
    \ ⚡ /
     \│/
   ───┼───
     /│\
    / ⚡ \
  RL     RR
     REAR

FL (D3): CCW rotation  FR (D5): CW rotation
RL (D9): CW rotation   RR (D6): CCW rotation
```

**ESC Connections per Motor**:
```
Battery ───┬─── ESC1 ───┬─── Motor 1 (3 wires)
           │            └─── Signal to D3
           │            └─── GND to Arduino GND
           │
           ├─── ESC2 ───┬─── Motor 2 (3 wires)
           │            └─── Signal to D5
           │            └─── GND to Arduino GND
           │
           ├─── ESC3 ───┬─── Motor 3 (3 wires)
           │            └─── Signal to D6
           │            └─── GND to Arduino GND
           │
           └─── ESC4 ───┬─── Motor 4 (3 wires)
                        └─── Signal to D9
                        └─── GND to Arduino GND
                        └─── +5V (BEC) to Arduino VIN
```

---

### 5. HC-SR04 Ultrasonic Sensor

```
HC-SR04            Arduino Nano
────────           ────────────
  VCC    ─────────  5V
  GND    ─────────  GND
  TRIG   ─────────  A1
  ECHO   ─────────  A2
```

**PURPOSE**: Ground proximity warning (<40cm)

---

### 6. Buzzer (Active or Passive)

```
Buzzer             Arduino Nano
───────            ────────────
  +      ─────────  D8
  -      ─────────  GND
```

**NOTE**: If using passive buzzer, code uses `tone()` function

---

### 7. LED Status Indicator

```
LED (with resistor) Arduino Nano
───────────────────────────────
  Anode (+)  ──┬── 220Ω Resistor ──┐
               │                    │
               └────────────────────┴── D7
  Cathode (-) ───────────────────── GND
```

---

### 8. Battery Voltage Monitor

```
Battery +  ───┬─── 1.5kΩ ───┬─── A0 (Arduino)
              │              │
              │              ├─── 1.0kΩ ─── GND
              │              │
              └──────────────┘
            (Voltage Divider)

Formula: Vin = Vout × (R1 + R2) / R2
         Vin = Vout × (1500 + 1000) / 1000
         Vin = Vout × 2.5
```

**WARNING**: 
- Max input to A0 is 5V
- Adjust resistors for your battery voltage
- 3S LiPo (12.6V max) → 5.04V after divider ✓

---

## 🎮 RC Transmitter Connections

### Arduino Nano Pinout

```
                    ┌─────────┐
              RESET─┤1      30├─VIN (USB 5V)
                   ─┤2      29├─GND
                   ─┤3      28├─RESET
                   ─┤4      27├─+5V
          Button1───┤5      26├─A7
          Button2───┤6      25├─A6
                   ─┤7      24├─A5
                   ─┤8      23├─A4
        nRF24(CE)───┤9      22├─A3 (Roll-X)
     nRF24(CSN)─────┤10     21├─A2 (Pitch-Y)
                   ─┤11     20├─A1 (Yaw-X)
                   ─┤12     19├─A0 (Throttle-Y)
                   ─┤13  N  18├─+5V
         Switch2────┤14  A  17├─D13(SCK)   ─┐
         Switch1────┤15  N  16├─D12(MISO)  ─┼─ nRF24L01
                   ─┤16  O  15├─D11(MOSI)  ─┘
                   ─┤17     14├─D10
                   ─┤18     13├─D9
                   ─┤19     12├─D8
                GND─┤20     11├─D7
                    └─────────┘
```

---

### 1. nRF24L01+ Radio Module (Transmitter)

```
nRF24L01+          Arduino Nano
─────────          ────────────
  VCC    ─────────  3.3V
  GND    ─────────  GND
  CE     ─────────  D9
  CSN    ─────────  D10
  SCK    ─────────  D13
  MOSI   ─────────  D11
  MISO   ─────────  D12
  IRQ    ─────────  (not connected)
```

**IMPORTANT**: 
- Add 10µF capacitor!
- Power: LOW setting (to prevent brownouts)

---

### 2. Joysticks (2x Analog)

```
Left Joystick:              Right Joystick:
──────────────              ───────────────
VCC  ────── 5V              VCC  ────── 5V
GND  ────── GND             GND  ────── GND
VRx  ────── A1 (Yaw)        VRx  ────── A3 (Roll)
VRy  ────── A0 (Throttle)   VRy  ────── A2 (Pitch)
SW   ────── (optional)      SW   ────── (optional)
```

**Joystick Orientation**:
```
        Y-axis (Up/Down)
            ↑
            │
    ────────┼────────  X-axis (Left/Right)
            │
            ↓
```

---

### 3. Buttons (with pull-up resistors)

```
Button 1:                   Button 2:
─────────                   ─────────
One terminal ──── D4        One terminal ──── D5
Other terminal ── GND       Other terminal ── GND

(Internal pull-up enabled in code)
```

**Function**:
- Button 1: Calibrate level (hold 2s)
- Button 2: Arm/Disarm (hold 2s)

---

### 4. Switches (SPDT or SPST)

```
Switch 1:                   Switch 2:
─────────                   ─────────
Common ────── D3            Common ────── D2
NO/NC ─────── GND           NO/NC ─────── GND
(other NC/NO - not used)    (other NC/NO - not used)

(Internal pull-up enabled in code)
```

**Function**:
- Switch 1: Emergency disarm
- Switch 2: Altitude hold mode

---

## ⚡ Power Distribution

### Flight Controller Power:

```
                    ┌─────────────┐
LiPo Battery ───────┤ ESC (any)   │
(3S or 4S)          │ with BEC    │
                    └─────┬───────┘
                          │ 5V BEC output
                          │
                          ├────── Arduino VIN
                          │
                          ├────── MPU6050 VCC
                          │
                          ├────── Buzzer VCC
                          │
                          ├────── LED (via resistor)
                          │
                          └────── Ultrasonic VCC

                    Common GND ──── All GND pins connected
```

**NOTE**: 
- ESC BEC provides 5V to Arduino
- All components share common ground
- nRF24 uses 3.3V (from Arduino 3.3V pin or regulator)

---

### RC Transmitter Power:

```
USB or Battery ────── Arduino VIN (5V)
                      │
                      ├────── Joysticks VCC
                      │
                      └────── nRF24 VCC (3.3V output)

              Common GND ──── All GND pins connected
```

---

## 🔧 Component Specifications

### Arduino Nano:
- **Voltage**: 5V (via USB or VIN 7-12V)
- **Digital I/O**: 14 pins
- **Analog In**: 8 pins (A0-A7)
- **Flash**: 32KB
- **SRAM**: 2KB

### nRF24L01+:
- **Voltage**: 1.9-3.6V (use 3.3V)
- **Current**: 12mA RX, 11mA TX
- **Range**: 100m (with PA+LNA: 1km)
- **Frequency**: 2.4GHz

### MPU6050:
- **Voltage**: 3-5V
- **Interface**: I2C (0x68 or 0x69)
- **Gyro**: ±250-2000°/s
- **Accel**: ±2-16g

### MS5611:
- **Voltage**: 1.8-3.6V (some modules 5V tolerant)
- **Interface**: I2C (0x77) or SPI
- **Range**: 10-1200 mbar
- **Resolution**: ±10cm altitude

### HC-SR04:
- **Voltage**: 5V
- **Range**: 2-400cm
- **Accuracy**: ±3mm

### ESCs:
- **Input**: 2-4S LiPo (depends on ESC rating)
- **Output**: 3-phase AC to motors
- **Signal**: 1000-2000µs PWM
- **BEC**: 5V/1-3A (for Arduino power)

---

## 🛠️ Assembly Tips

### 1. Soldering nRF24L01+
```
Pro Tip: Solder all 8 pins, add 10µF capacitor
         between VCC and GND close to module.

  ┌────────────┐
  │  nRF24L01+ │
  └─┬─┬─┬─┬─┬──┘
    │ │ │ │ │
   [====]  ← 10µF capacitor here
    │ │ 
   GND VCC
```

### 2. MPU6050 Vibration Damping
```
Frame ───────  Double-sided foam tape (1-2mm)
             ───────  MPU6050 module
```

### 3. ESC Signal Wires
```
Use servo extension cables for clean connections:
ESC ────── 20cm extension ────── Arduino pin
```

### 4. Battery Connector
```
XT60 Connector ────── Power Distribution Board (PDB)
                      │
                      ├─── ESC1 (+ and -)
                      ├─── ESC2 (+ and -)
                      ├─── ESC3 (+ and -)
                      └─── ESC4 (+ and -)
```

---

## 🔍 Testing Connections

### 1. Continuity Test
Use multimeter to verify:
- All GND connections
- No shorts between VCC and GND
- Signal wire continuity

### 2. Voltage Test
Before connecting Arduino:
- ESC BEC output: Should be ~5V
- Battery voltage: 11.1V (3S) or 14.8V (4S) when charged
- No voltage on signal wires (yet)

### 3. Component Test
Upload test sketches:
```cpp
// Test nRF24:    rf24/examples/GettingStarted
// Test MPU6050:  Wire/examples/I2CScanner
// Test servos:   Servo/examples/Sweep
```

---

## ⚠️ Common Wiring Mistakes

### ❌ Wrong:
1. nRF24 VCC to 5V (causes instability)
2. No capacitor on nRF24 (brownouts)
3. MPU6050 SDA/SCL reversed
4. ESC signal and ground swapped
5. Motor rotation directions wrong
6. Shared ground missing

### ✅ Correct:
1. nRF24 VCC to 3.3V
2. 10µF capacitor added
3. SDA to A4, SCL to A5
4. Signal to correct pins, GND to GND
5. Alternate CW/CCW in X pattern
6. All GND connected together

---

## 📊 Wire Gauge Recommendations

| Connection | Wire Gauge | Notes |
|------------|------------|-------|
| Battery to ESC | 14-16 AWG | High current |
| ESC to Motors | 16-18 AWG | High current |
| ESC Signal | 22-26 AWG | Low current, servo wire |
| I2C (SDA/SCL) | 24-28 AWG | Short (<30cm) |
| nRF24 Power | 24-26 AWG | Low current |
| Sensors | 24-28 AWG | Low current |

---

## 🎨 Color Coding

Standard wire colors:
```
RED    ──── Positive (+) / VCC / Power
BLACK  ──── Negative (-) / GND / Ground
YELLOW ──── Signal / PWM / Data
WHITE  ──── Signal / PWM / Data (alt)
BROWN  ──── Ground (servo standard)
ORANGE ──── Signal (servo standard)
```

---

## ✅ Final Wiring Checklist

- [ ] All connections soldered or firmly plugged
- [ ] 10µF capacitor on nRF24 modules
- [ ] MPU6050 mounted flat with foam tape
- [ ] All ESC signals to correct pins
- [ ] Motor rotation verified (X pattern: FL&RR=CCW, FR&RL=CW)
- [ ] Common ground connected everywhere
- [ ] No shorts between power and ground
- [ ] Battery connector secure
- [ ] Strain relief on all wires
- [ ] Heat shrink over exposed connections
- [ ] Labels on all ESC wires
- [ ] Tested each connection with multimeter

---

**Wiring complete! Proceed to uploading code and calibration.**

*For code upload instructions, see README.md*
*For calibration steps, see CALIBRATION_GUIDE.md*
