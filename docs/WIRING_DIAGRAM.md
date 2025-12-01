# Complete Wiring Diagrams

## Table of Contents
1. [Flight Controller Wiring](#flight-controller-wiring)
2. [Remote Controller Wiring](#remote-controller-wiring)
3. [Power Distribution](#power-distribution)
4. [Component Connection Details](#component-connection-details)
5. [Common Mistakes](#common-mistakes)

---

## Flight Controller Wiring

### Arduino Nano Pinout Reference

```
                    Arduino Nano
                    ┌─────────┐
              D13 ──┤●        ├── D12
              3.3V ──┤         ├── D11
              Aref ──┤         ├── D10
               A0  ──┤         ├── D9
               A1  ──┤         ├── D8
               A2  ──┤         ├── D7
               A3  ──┤         ├── D6
               A4  ──┤  NANO   ├── D5
               A5  ──┤         ├── D4
               A6  ──┤         ├── D3
               A7  ──┤         ├── D2
              +5V  ──┤         ├── GND
              RST  ──┤         ├── RST
              GND  ──┤         ├── RX0
              VIN  ──┤         ├── TX1
                    └──USB───┘
```

### Complete Flight Controller Connections

#### I2C Sensors (MPU6050 + MS5611)

Both sensors share the I2C bus:

```
Arduino Nano          MPU6050          MS5611
─────────────────────────────────────────────
A4 (SDA) ────────┬─── SDA ─────────── SDA
                 │
A5 (SCL) ────────┼─── SCL ─────────── SCL
                 │
3.3V ────────────┼─── VCC ─────────── VCC
                 │
GND ─────────────┴─── GND ─────────── GND
```

**Important Notes:**
- MPU6050 and MS5611 both work at 3.3V (Arduino Nano has 3.3V output)
- Use short wires (<10cm) for stable I2C communication
- Add 4.7kΩ pull-up resistors on SDA and SCL if you experience communication issues
- Default I2C addresses: MPU6050 = 0x68, MS5611 = 0x77

#### Motor Connections (ESCs)

```
Arduino Nano         ESC          Motor        Position
──────────────────────────────────────────────────────────
D3 (PWM) ────────── Signal ──── Motor 1 ──── Front Left (CCW ↺)
                    ├─ VCC (BEC 5V) ──> Arduino VIN
                    └─ GND ──────────> Arduino GND

D5 (PWM) ────────── Signal ──── Motor 2 ──── Front Right (CW ↻)

D6 (PWM) ────────── Signal ──── Motor 3 ──── Rear Right (CCW ↺)

D9 (PWM) ────────── Signal ──── Motor 4 ──── Rear Left (CW ↻)
```

**ESC Connections (Each ESC):**
```
ESC Side:
┌─────────┐
│  ESC    │
│         │
├─ Red ───┼─── BEC 5V Out (Connect ONE ESC's BEC to Arduino VIN)
├─ Black ─┼─── Ground (Connect ALL ESCs' GND to Arduino GND)
├─ White ─┼─── Signal (PWM from Arduino)
└─────────┘

Motor Side:
3 wires to brushless motor (any order, swap 2 to reverse direction)

Battery Side:
Red (+) and Black (-) to power distribution board
```

**Critical Notes:**
- Only connect ONE ESC's BEC (5V) to Arduino VIN
- Connect ALL ESC grounds to Arduino GND
- Use 20-30A ESCs for 1000-1500KV motors
- Calibrate ESCs before first flight

#### nRF24L01+ Radio Module

```
Arduino Nano         nRF24L01+
───────────────────────────────────
D4  ────────────── CE (Chip Enable)
D10 ────────────── CSN (Chip Select)
D11 ────────────── MOSI
D12 ────────────── MISO
D13 ────────────── SCK
3.3V ───────────── VCC (IMPORTANT: 3.3V ONLY!)
GND ────────────── GND
```

**CRITICAL: Radio Power Issues**

nRF24L01+ is very sensitive to power supply. Common problems:
- Needs stable 3.3V (not 5V!)
- Current spikes during transmission can cause resets
- Solution: Add 10µF capacitor between VCC and GND, very close to module

```
Power Supply Enhancement:
                    10µF Capacitor
3.3V ────┬──────────||──────┬──── nRF24 VCC
         │                  │
GND ─────┴──────────────────┴──── nRF24 GND
```

For better range, use nRF24L01+ with external antenna and PA+LNA.

#### Status Indicators

```
Arduino Nano         Component
────────────────────────────────
D7 ──────┬──── Buzzer (+)
         │
        GND ─── Buzzer (-)

D8 ──────┬──── LED (+) Anode
         │
        GND ─┬─ 220Ω Resistor ─── LED (-) Cathode
```

### Complete Flight Controller Wiring Diagram

```
                        FLIGHT CONTROLLER WIRING
                        ========================

                            ┌──────────┐
                   ┌────────┤ MPU6050  ├────────┐
                   │        └──────────┘        │
                   │        ┌──────────┐        │
                   │   ┌────┤ MS5611   ├────┐   │
                   │   │    └──────────┘    │   │
                 (SDA)(SCL)              (SDA)(SCL)
                   │   │                    │   │
                   A4  A5                   │   │
                   │   │                    │   │
              ┌────┴───┴────────────────────┴───┴────┐
              │                                       │
              │          Arduino Nano                 │
              │                                       │
              │  D3  D5  D6  D9        D4  D10       │
              └───┬───┬───┬───┬─────────┬───┬────────┘
                  │   │   │   │         │   │
                 PWM PWM PWM PWM       CE  CSN
                  │   │   │   │         │   │
              ┌───┴┐ ┌┴───┐ ┌┴──┐ ┌────┴┐  │
              │ESC1│ │ESC2│ │ESC3│ │ESC4│  │
              └┬──┬┘ └┬──┬┘ └┬──┬┘ └┬──┬┘  │
               │  │   │  │   │  │   │  │   │
              M1  │  M2  │  M3  │  M4  │   │
              FL  │  FR  │  RR  │  RL  │   │
              ↺   │  ↻   │  ↺   │  ↻   │   │
                  │      │      │      │   │
              ┌───┴──────┴──────┴──────┴───┘
              │   Power Distribution Board   │
              │        (+ / -)               │
              └───────┬─────────────┬────────┘
                      │             │
                   ┌──┴──┐      ┌───┴───┐        ┌────────────┐
                   │LiPo │      │ BEC   │───5V───┤ Arduino    │
                   │11.1V│      │(from  │        │ VIN        │
                   │3S   │      │ ESC)  │        └────────────┘
                   └─────┘      └───────┘
                   
                                          nRF24L01+
                                          ┌────────┐
                            D4  CE ───────┤        │
                            D10 CSN ──────┤        │
                            D11 MOSI ─────┤ RADIO  │
                            D12 MISO ─────┤        │
                            D13 SCK ──────┤        │
                            3.3V ─────────┤ VCC    │
                            GND ──────────┤ GND    │
                                          └────────┘

          D7 ──── Buzzer ──── GND
          D8 ──── LED (220Ω) ─── GND
```

---

## Remote Controller Wiring

### Complete Remote Controller Connections

#### Joysticks

Standard dual-axis joysticks have 5 pins each:
- VCC (5V)
- GND
- VRx (X-axis analog output)
- VRy (Y-axis analog output)
- SW (button, optional)

```
Arduino Nano      Left Joystick       Right Joystick
───────────────────────────────────────────────────
5V ────────────── VCC ────────────── VCC
GND ───────────── GND ────────────── GND

A0 ───────────── VRy (Throttle)
A1 ───────────── VRx (Yaw)
                                    
A2 ─────────────────────────────── VRy (Pitch)
A3 ─────────────────────────────── VRx (Roll)
```

#### Push Buttons

Using internal pull-up resistors (no external resistors needed):

```
Arduino Nano         Button          Function
────────────────────────────────────────────────
D4 ──────────┬──── Button 1 ────── Calibrate
             │
D5 ──────────┼──── Button 2 ────── Motor Test
             │
D6 ──────────┼──── Button 3 ────── ARM/DISARM
             │
D7 ──────────┼──── Button 4 ────── Soft Landing
             │
GND ─────────┴──── Common Ground

(Internal pull-up resistors enabled in code)
```

**Button Wiring (Each Button):**
```
        Arduino Pin
             │
         ┌───┴───┐
         │       │
     10kΩ (internal pull-up)
         │       │
         │    [Button]
         │       │
         └───────┴──── GND
```

#### Toggle Switches

```
Arduino Nano         Switch          Function
────────────────────────────────────────────────
D2 ──────────┬──── Switch 1 ─────── Reserved
             │
D3 ──────────┼──── Switch 2 ─────── ANGLE/ACRO Mode
             │
GND ─────────┴──── Common Ground

(Internal pull-up resistors enabled in code)
```

#### nRF24L01+ Radio Module

Same as flight controller:

```
Arduino Nano         nRF24L01+
───────────────────────────────────
D9  ────────────── CE (Chip Enable)
D10 ────────────── CSN (Chip Select)
D11 ────────────── MOSI
D12 ────────────── MISO
D13 ────────────── SCK
3.3V ───────────── VCC (with 10µF capacitor!)
GND ────────────── GND
```

#### Power Supply

```
9V Battery or 3x AA Batteries
      │
      ├──── VIN (Arduino Nano)
      └──── GND
      
Arduino provides:
- 5V output for joysticks
- 3.3V output for radio
```

### Complete Remote Controller Wiring Diagram

```
                     REMOTE CONTROLLER WIRING
                     ========================

        Left Joystick              Right Joystick
        ┌──────────┐              ┌──────────┐
    ┌───┤ VRx VRy  ├───┐      ┌───┤ VRx VRy  ├───┐
    │   └──────────┘   │      │   └──────────┘   │
   A1        A0        │      │        A3    A2
    │         │        │      │         │     │
    │         │       5V     GND       5V    GND
    │         │        │      │         │     │
  ┌─┴─────────┴────────┴──────┴─────────┴─────┴─┐
  │                                               │
  │             Arduino Nano                      │
  │                                               │
  │  D4  D5  D6  D7     D2  D3      D9  D10      │
  └───┬───┬───┬───┬─────┬───┬────────┬───┬───────┘
      │   │   │   │     │   │        │   │
     B1  B2  B3  B4    SW1 SW2      CE  CSN
      │   │   │   │     │   │        │   │
      └───┴───┴───┴─────┴───┘        │   │
              │                 ┌────┴───┴────┐
             GND                │  nRF24L01+  │
                                │   (Radio)   │
                                │             │
                                │  D11: MOSI  │
          9V Battery            │  D12: MISO  │
          ┌────┐                │  D13: SCK   │
       +  │    │ -              │  3.3V: VCC  │
      ────┤    ├────            │  GND:  GND  │
          └────┘                └─────────────┘
           │  │
          VIN GND
           │  │
      Arduino Nano

  B1-B4 = Push Buttons (Calibrate, Motor Test, ARM, Landing)
  SW1-SW2 = Toggle Switches (Reserved, Mode)
  
  All buttons/switches connected between pin and GND
  (using internal pull-up resistors)
```

---

## Power Distribution

### Flight Controller Power System

```
                    POWER DISTRIBUTION
                    ==================

    ┌──────────────────────────────────────────┐
    │     LiPo Battery 11.1V 3S 2200mAh       │
    │              (XT60 Connector)            │
    └────┬─────────────────────────────┬───────┘
         │(+)                          │(-)
         │                             │
    ┌────┴─────────────────────────────┴───────┐
    │     Power Distribution Board (PDB)       │
    │            or ESC with BEC               │
    │                                           │
    │  + + + +                    - - - -      │
    │  │ │ │ │                    │ │ │ │      │
    └──┼─┼─┼─┼────────────────────┼─┼─┼─┼──────┘
       │ │ │ │                    │ │ │ │
       │ │ │ │                    │ │ │ │
    ┌──┴─┴─┴─┴┐                ┌──┴─┴─┴─┴─┐
    │         │                │          │
    │  ESC 1  │  ESC 2  ESC 3  │  ESC 4   │
    │         │                │          │
    │  BEC 5V ├────────────────┤ Disable  │
    └────┬────┘                └──────────┘
         │ 5V (BEC Output)
         │
    ┌────┴─────────────┐
    │  Arduino Nano    │
    │  VIN  ←── 5V     │
    │  GND  ←── GND    │
    │                  │
    │  Powers:         │
    │  - MCU           │
    │  - MPU6050 3.3V  │
    │  - MS5611 3.3V   │
    │  - nRF24 3.3V    │
    └──────────────────┘

    Motors: 11.1V direct from battery
    Arduino: 5V from ESC BEC
    Sensors: 3.3V from Arduino regulator
```

**Important Power Notes:**
1. Use only ONE ESC's BEC to power Arduino (disable others by cutting red wire)
2. Connect ALL ESC grounds to common ground
3. Add 470µF-1000µF capacitor on PDB to smooth voltage spikes
4. Use 20-30A ESCs with BEC for 5V output
5. LiPo voltage: Full = 12.6V, Nominal = 11.1V, Low = 10.5V, Critical = 9.9V

---

## Component Connection Details

### MPU6050 Module Pinout

```
    MPU6050 Module
    ┌────────────┐
    │  VCC  3.3V │──── Arduino 3.3V
    │  GND  GND  │──── Arduino GND
    │  SCL  A5   │──── Arduino A5
    │  SDA  A4   │──── Arduino A4
    │  XDA       │──── (not used)
    │  XCL       │──── (not used)
    │  AD0       │──── GND (sets I2C address to 0x68)
    │  INT       │──── (not used, optional for interrupts)
    └────────────┘
```

### MS5611 Module Pinout

```
    MS5611 Module
    ┌────────────┐
    │  VCC  3.3V │──── Arduino 3.3V
    │  GND  GND  │──── Arduino GND
    │  SCL  A5   │──── Arduino A5
    │  SDA  A4   │──── Arduino A4
    └────────────┘
```

### nRF24L01+ Module Pinout

```
Top View of nRF24L01+:
    ┌────────────┐
    │  ┌──────┐  │  Pin    Arduino
    │  │ Chip │  │  ─────  ───────
    │  │      │  │  GND  → GND
    │  └──────┘  │  VCC  → 3.3V
    │            │  CE   → D4 (Flight) / D9 (Remote)
    │            │  CSN  → D10
    │  Antenna   │  SCK  → D13
    └────────────┘  MOSI → D11
                    MISO → D12
                    IRQ  → (not used)

Pin Layout (looking at front):
┌─────────────┐
│  1  2  3  4 │
│  •  •  •  • │
│             │
│  5  6  7  8 │
│  •  •  •  • │
└─────────────┘

1: GND    5: CE
2: VCC    6: CSN
3: CE     7: SCK
4: CSN    8: MISO
          (varies by module)
```

### ESC Pinout

```
    Standard ESC
    ┌────────────┐
    │   ESC      │
    │   20-30A   │
    └─┬──┬───┬───┘
      │  │   │
     Red│Black│White
     5V │GND  │Signal (PWM)
        │     │
        │     └──── Arduino D3/D5/D6/D9
        │
        └─────────→ Arduino VIN (from one ESC only)

    Motor Side: 3 thick wires (any order)
    Battery Side: 2 thick wires (+ red, - black)
```

### Joystick Module Pinout

```
    Analog Joystick
    ┌──────────┐
    │    ⊕     │  Pin    Connection
    │   /│\    │  ─────  ──────────
    │  ╱ │ ╲   │  GND  → Arduino GND
    │ │  │  │  │  +5V  → Arduino 5V
    │  ╲ │ ╱   │  VRx  → Arduino A0-A3
    │   \│/    │  VRy  → Arduino A0-A3
    │    ⊗     │  SW   → Optional button
    └──────────┘
    
    Bottom pins (left to right):
    GND  +5V  VRx  VRy  SW
```

---

## Common Mistakes

### ❌ Mistake 1: Wrong Voltage for nRF24L01+
**Problem:** Connecting nRF24 to 5V instead of 3.3V
**Result:** Module gets damaged or behaves erratically
**Solution:** Always use 3.3V pin, add 10µF capacitor

### ❌ Mistake 2: Missing Common Ground
**Problem:** Not connecting all device grounds together
**Result:** Communication failures, random resets
**Solution:** Connect all GND pins: Arduino, ESCs, sensors, radio

### ❌ Mistake 3: Wrong Motor Directions
**Problem:** Motors spinning in wrong direction
**Result:** Drone flips on takeoff
**Solution:** 
- Check motor configuration (2 CW, 2 CCW)
- Swap any 2 of the 3 motor wires to reverse direction

### ❌ Mistake 4: Propeller Installation
**Problem:** Installing propellers backward or in wrong positions
**Result:** No lift or unstable flight
**Solution:**
- Match propeller rotation to motor rotation
- Check markings: R (right/CW), L (left/CCW)
- Propeller blade angle should push air downward

### ❌ Mistake 5: BEC from Multiple ESCs
**Problem:** Connecting BEC (5V) from all ESCs to Arduino
**Result:** Voltage conflict, possible damage
**Solution:** Use only ONE ESC's BEC, cut red wire on others

### ❌ Mistake 6: Long I2C Wires
**Problem:** Using long wires (>20cm) for MPU6050/MS5611
**Result:** Communication errors, failed reads
**Solution:** Keep I2C wires short (<10cm), use pull-up resistors if needed

### ❌ Mistake 7: No ESC Calibration
**Problem:** ESCs not calibrated to Arduino PWM range
**Result:** Motors don't spin or behave incorrectly
**Solution:** Calibrate ESCs using standard procedure (see troubleshooting guide)

---

## Testing Checklist

### Before First Power-On

- ✅ All connections soldered or firmly secured
- ✅ No short circuits (use multimeter to verify)
- ✅ nRF24 connected to 3.3V (not 5V!)
- ✅ Capacitor on nRF24 power
- ✅ Only one ESC BEC connected to Arduino VIN
- ✅ All grounds connected together
- ✅ I2C pull-up resistors installed (if needed)
- ✅ Propellers REMOVED for initial testing

### After Power-On

- ✅ Arduino LED lights up
- ✅ No smoke or burning smell
- ✅ Serial monitor shows sensor initialization
- ✅ MPU6050 detected (address 0x68)
- ✅ MS5611 detected (address 0x77)
- ✅ Radio module initializes successfully

---

## Wire Gauge Recommendations

| Connection | Wire Gauge | Notes |
|------------|------------|-------|
| Battery to PDB | 14-16 AWG | High current |
| ESC to Motor | 16-18 AWG | High current |
| ESC to PDB | 16-18 AWG | High current |
| Signal wires | 22-26 AWG | Low current, flexible |
| Sensor wires | 24-28 AWG | Short, neat |
| BEC to Arduino | 20-22 AWG | Medium current |

---

## Connector Types

| Component | Recommended Connector |
|-----------|-----------------------|
| Battery | XT60 or Deans |
| ESC Signal | Servo connector (3-pin) |
| I2C Sensors | Dupont connectors or soldered |
| Radio | Dupont connectors |
| Joysticks | Header pins or Dupont |

---

**Always double-check connections before powering on!**
**When in doubt, measure with a multimeter!**
