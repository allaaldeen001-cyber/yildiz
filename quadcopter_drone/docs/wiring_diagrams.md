# Detailed Wiring Diagrams

## Flight Controller Wiring

### Arduino Nano Pin Connections

| Arduino Pin | Function | Component | Wire Color (suggested) |
|-------------|----------|-----------|------------------------|
| D2 | Status LED | LED (with 220Ω resistor) | Green |
| D3 | Motor 1 PWM | ESC 1 Signal (Front-Right) | White |
| D4 | NRF CE | NRF24L01 CE Pin | Orange |
| D5 | ARM LED | LED (with 220Ω resistor) | Red |
| D6 | Motor 4 PWM | ESC 4 Signal (Front-Left) | White |
| D7 | NRF CSN | NRF24L01 CSN Pin | Yellow |
| D8 | Buzzer | Active Buzzer + | Brown |
| D9 | Motor 2 PWM | ESC 2 Signal (Rear-Right) | White |
| D10 | Motor 3 PWM | ESC 3 Signal (Rear-Left) | White |
| D11 | NRF MOSI | NRF24L01 MOSI | Blue |
| D12 | NRF MISO | NRF24L01 MISO | Green |
| D13 | NRF SCK | NRF24L01 SCK | Purple |
| A4 | I2C SDA | MPU6050 SDA, MS5611 SDA | Yellow |
| A5 | I2C SCL | MPU6050 SCL, MS5611 SCL | White |
| 3.3V | NRF Power | NRF24L01 VCC | Red |
| 5V | Sensors | MPU6050 VCC, MS5611 VCC | Red |
| GND | Ground | All grounds | Black |

### Detailed NRF24L01 Wiring

```
NRF24L01 Module Pinout (Top View):
┌────────────────────────────────┐
│                                │
│    ┌──┐  ┌──┐  ┌──┐  ┌──┐     │
│    │1 │  │2 │  │3 │  │4 │     │
│    │GND│  │VCC│  │CE│  │CSN│   │
│    └──┘  └──┘  └──┘  └──┘     │
│                                │
│    ┌──┐  ┌──┐  ┌──┐  ┌──┐     │
│    │5 │  │6 │  │7 │  │8 │     │
│    │SCK│ │MOSI│ │MISO│ │IRQ│   │
│    └──┘  └──┘  └──┘  └──┘     │
│                                │
└────────────────────────────────┘

Connections:
Pin 1 (GND)  → Arduino GND
Pin 2 (VCC)  → Arduino 3.3V ⚠️ NOT 5V!
Pin 3 (CE)   → Arduino D4
Pin 4 (CSN)  → Arduino D7
Pin 5 (SCK)  → Arduino D13
Pin 6 (MOSI) → Arduino D11
Pin 7 (MISO) → Arduino D12
Pin 8 (IRQ)  → Not connected
```

### MPU6050 Wiring

```
MPU6050 Module Pinout:
┌─────────────────┐
│     MPU6050     │
│                 │
│  VCC  GND  SCL  │
│  SDA  XDA  XCL  │
│  AD0  INT       │
└─────────────────┘

Connections:
VCC → Arduino 5V
GND → Arduino GND
SCL → Arduino A5
SDA → Arduino A4
AD0 → Not connected (or GND for address 0x68)
INT → Not connected
XDA → Not connected
XCL → Not connected
```

### MS5611 Wiring

```
MS5611 Module Pinout:
┌─────────────────┐
│     MS5611      │
│                 │
│  VCC  GND  SCL  │
│  SDA  CSB  PS   │
└─────────────────┘

Connections:
VCC → Arduino 5V
GND → Arduino GND
SCL → Arduino A5
SDA → Arduino A4
CSB → VCC or not connected (I2C mode)
PS  → VCC or not connected (I2C mode)
```

### ESC Connections

```
ESC Typical Pinout:
┌────────────────────────────────────────┐
│                 ESC                    │
│                                        │
│  Battery+  Battery-   Motor Wires (3)  │
│    (Red)    (Black)   (Any 3 colors)   │
│                                        │
│  Signal    GND       [Optional VCC]    │
│  (White)  (Black)    (Red/Not used)    │
└────────────────────────────────────────┘

Signal Connections:
ESC 1 Signal (White) → Arduino D3 (Front-Right)
ESC 2 Signal (White) → Arduino D9 (Rear-Right)
ESC 3 Signal (White) → Arduino D10 (Rear-Left)
ESC 4 Signal (White) → Arduino D6 (Front-Left)

All ESC GND (Black) → Common Ground with Arduino

⚠️ DO NOT connect ESC VCC (BEC output) to Arduino if using USB power!
   Only one power source should power the Arduino.
```

### Complete Flight Controller Schematic

```
                                    BATTERY (3S LiPo 11.1V)
                                           │
                    ┌──────────────────────┼──────────────────────┐
                    │                      │                      │
                ┌───┴───┐              ┌───┴───┐              ┌───┴───┐
                │ ESC 1 │              │ ESC 2 │              │ ESC 3 │
                │  FR   │              │  RR   │              │  RL   │
                └───┬───┘              └───┬───┘              └───┬───┘
                    │                      │                      │
                    │  ┌───────────────────┼──────────────────────┘
                    │  │                   │
                    │  │               ┌───┴───┐
                    │  │               │ ESC 4 │
                    │  │               │  FL   │
                    │  │               └───┬───┘
                    │  │                   │
        ┌───────────┴──┴───────────────────┴───────────────────┐
        │                                                       │
        │                    ARDUINO NANO                       │
        │                                                       │
        │  D2 ──────── Status LED ──── GND                      │
        │  D3 ──────── ESC1 Signal                              │
        │  D4 ──────── NRF24 CE                                 │
        │  D5 ──────── ARM LED ──── GND                         │
        │  D6 ──────── ESC4 Signal                              │
        │  D7 ──────── NRF24 CSN                                │
        │  D8 ──────── Buzzer+ ──── GND                         │
        │  D9 ──────── ESC2 Signal                              │
        │  D10 ─────── ESC3 Signal                              │
        │  D11 ─────── NRF24 MOSI                               │
        │  D12 ─────── NRF24 MISO                               │
        │  D13 ─────── NRF24 SCK                                │
        │                                                       │
        │  A4 ──────┬─ MPU6050 SDA                              │
        │          └─ MS5611 SDA                                │
        │  A5 ──────┬─ MPU6050 SCL                              │
        │          └─ MS5611 SCL                                │
        │                                                       │
        │  3.3V ───── NRF24 VCC                                 │
        │  5V ──────┬─ MPU6050 VCC                              │
        │          └─ MS5611 VCC                                │
        │  GND ─────── Common Ground                            │
        │                                                       │
        └───────────────────────────────────────────────────────┘

NOTES:
1. Use 10µF capacitor between NRF24 VCC and GND (close to module)
2. Use 100nF capacitors on MPU6050 and MS5611 power pins
3. LEDs need 220Ω-330Ω current limiting resistors
4. Active buzzer needs direct connection (no resistor)
```

---

## RC Transmitter Wiring

### Arduino Nano Pin Connections

| Arduino Pin | Function | Component | Wire Color (suggested) |
|-------------|----------|-----------|------------------------|
| D2 | Status LED | LED (with 220Ω resistor) | Green |
| D3 | Button 1 | Push button (to GND) | Yellow |
| D4 | Button 2 | Push button (to GND) | Yellow |
| D5 | ARM Switch | Toggle switch (to GND) | Red |
| D9 | NRF CE | NRF24L01 CE Pin | Orange |
| D10 | NRF CSN | NRF24L01 CSN Pin | Yellow |
| D11 | NRF MOSI | NRF24L01 MOSI | Blue |
| D12 | NRF MISO | NRF24L01 MISO | Green |
| D13 | NRF SCK | NRF24L01 SCK | Purple |
| A0 | Left Y | Left Joystick VRy (Throttle) | White |
| A1 | Left X | Left Joystick VRx (Yaw) | White |
| A2 | Right Y | Right Joystick VRy (Pitch) | White |
| A3 | Right X | Right Joystick VRx (Roll) | White |
| 3.3V | NRF Power | NRF24L01 VCC | Red |
| 5V | Joysticks | Both joystick VCC | Red |
| GND | Ground | All grounds | Black |

### Joystick Module Wiring

```
KY-023 Joystick Module Pinout:
┌─────────────────────┐
│                     │
│    [Joystick Cap]   │
│         │           │
│    ┌────┴────┐      │
│    │         │      │
│ GND│  +5V│VRx│VRy│SW│
└────┴─────┴───┴───┴──┘

Left Joystick (Throttle/Yaw):
GND → Arduino GND
+5V → Arduino 5V
VRx → Arduino A1 (Yaw)
VRy → Arduino A0 (Throttle)
SW  → Not connected

Right Joystick (Pitch/Roll):
GND → Arduino GND
+5V → Arduino 5V
VRx → Arduino A3 (Roll)
VRy → Arduino A2 (Pitch)
SW  → Not connected
```

### Button and Switch Wiring

```
Push Button Wiring (using internal pullup):
        Arduino Pin ────┬──── Button ──── GND
                        │
                    Internal
                    Pull-up
                    (Enabled in code)

Toggle Switch Wiring:
        Arduino D5 ────┬──── Switch ──── GND
                       │
                   Internal
                   Pull-up

Switch Positions:
- Switch OPEN (not connected to GND) = DISARMED (safe)
- Switch CLOSED (connected to GND) = ARMED
```

### Complete RC Transmitter Schematic

```
        ┌───────────────────────────────────────────────────────┐
        │                                                       │
        │                    ARDUINO NANO                       │
        │                                                       │
        │  D2 ──────── Status LED ──── [220Ω] ──── GND          │
        │  D3 ──────── Button 1 (Calibration) ──── GND          │
        │  D4 ──────── Button 2 (Motor Test) ──── GND           │
        │  D5 ──────── ARM Toggle Switch ──── GND               │
        │                                                       │
        │  D9 ──────── NRF24 CE                                 │
        │  D10 ─────── NRF24 CSN                                │
        │  D11 ─────── NRF24 MOSI                               │
        │  D12 ─────── NRF24 MISO                               │
        │  D13 ─────── NRF24 SCK                                │
        │                                                       │
        │  A0 ──────── Left Joystick VRy (Throttle)             │
        │  A1 ──────── Left Joystick VRx (Yaw)                  │
        │  A2 ──────── Right Joystick VRy (Pitch)               │
        │  A3 ──────── Right Joystick VRx (Roll)                │
        │                                                       │
        │  3.3V ───── NRF24 VCC ──── [10µF Cap] ──── GND        │
        │  5V ──────┬─ Left Joystick +5V                        │
        │          └─ Right Joystick +5V                        │
        │  GND ─────── Common Ground                            │
        │                                                       │
        └───────────────────────────────────────────────────────┘
```

---

## Important Notes

### Power Considerations

1. **NRF24L01 Power**:
   - MUST use 3.3V (5V will damage the module!)
   - Add 10µF capacitor between VCC and GND
   - Keep wires short to reduce noise

2. **Flight Controller Power**:
   - Can be powered via USB (for testing) or from ESC BEC
   - Use only ONE power source at a time
   - Typical ESC BEC provides 5V

3. **RC Transmitter Power**:
   - 9V battery or USB power
   - Consider 2S LiPo (7.4V) for portability

### I2C Bus Notes

- Both MPU6050 and MS5611 share the I2C bus (A4/A5)
- Keep I2C wires short (<10cm) for reliability
- Add 4.7kΩ pull-up resistors if using long wires
- Default addresses: MPU6050 = 0x68, MS5611 = 0x77

### Motor Direction Setup

After wiring, verify motor directions:
1. Motors 1 and 3 should spin CCW (counter-clockwise)
2. Motors 2 and 4 should spin CW (clockwise)
3. Swap any two ESC-to-motor wires to reverse direction

### ESC Calibration

Most ESCs need calibration for throttle range:
1. With props OFF, run ESC calibration from the code
2. Or manually: Send max throttle, power ESC, wait for beeps, send min throttle

### Safety Checklist

Before first flight:
- [ ] All connections secure
- [ ] Props tight but NOT installed for testing
- [ ] NRF24L01 antenna not touching metal
- [ ] ESCs calibrated
- [ ] IMU calibrated on level surface
- [ ] Joysticks calibrated
- [ ] ARM switch works (disarms motors)
- [ ] All motors spin correct direction
- [ ] Motor/prop combos matched for rotation direction
