# Wiring Diagrams

## Complete Connection Guide for Flight Controller and Remote Controller

---

## 🎛️ Flight Controller Wiring

### Main Schematic

```
                         FLIGHT CONTROLLER WIRING DIAGRAM
═══════════════════════════════════════════════════════════════════════════════

                                     ARDUINO NANO
                              ┌────────────────────────┐
                              │         USB            │
                              │      ┌───────┐         │
                              │      │       │         │
                          D13 │●     └───────┘       ●│ D12
                (SPI SCK) ────┼──────────┐  ┌────────┼──── (SPI MISO)
                          D11 │●         │  │        ●│ D10
                (SPI MOSI) ────┼─────┐   │  │   ┌────┼──── NRF24 CSN
                           D9 │●    │   │  │   │    ●│ D8
                  Motor RL ────┼────┼───┼──┼───┼────┼──── Buzzer (+)
                           D7 │●    │   │  │   │    ●│ D6
              Status LED (+) ─┼────┼───┼──┼───┼────┼──── Motor RR
                           D5 │●    │   │  │   │    ●│ D4
                  Motor FR ────┼────┼───┼──┼───┼────┼──── NRF24 CE
                           D3 │●    │   │  │   │    ●│ D2
                  Motor FL ────┼────┼───┼──┼───┼────┼──── MPU6050 INT
                          GND │●    │   │  │   │    ●│ GND
                              │     │   │  │   │     │
                          RST │●    │   │  │   │    ●│ RST
                              │     │   │  │   │     │
                           A0 │●    │   │  │   │    ●│ A7
                              │     │   │  │   │     │
                           A1 │●    │   │  │   │    ●│ A6
                              │     │   │  │   │     │
                           A2 │●    │   │  │   │    ●│ A5
                              │     │   │  │   └────┼──── I2C SCL (MPU6050, MS5611)
                           A3 │●    │   │          ●│ A4
                              │     │   └──────────┼──── I2C SDA (MPU6050, MS5611)
                          VIN │●    │              ●│ AREF
                              │     │               │
                          GND │●    │              ●│ 3.3V
                              │     └──────────────┼──── NRF24 VCC (3.3V ONLY!)
                           5V │●                   ●│ 5V
                              │                     │
                              └─────────────────────┘


          NRF24L01 PA+LNA                           MPU6050
         ┌────────────────┐                    ┌────────────────┐
         │  ●  ●  ●  ●    │                    │ VCC   GND  SCL │
         │ GND VCC CE CSN │                    │  ●     ●    ●  │
         │  ●  ●  ●  ●    │                    │ XDA   XCL  SDA │
         │ SCK MOSI MISO IRQ                   │  ●     ●    ●  │
         └───┬──┬───┬───┬─┘                    │ INT   AD0      │
             │  │   │   │                      │  ●     ●       │
         ┌───┘  │   │   └── (NC)               └──┬─────┬───────┘
         │      │   │                             │     │
         │   ┌──┘   └──┐                          │     └── GND (AD0 low = 0x68)
         │   │         │                          │
    ┌────┴───┴────┬────┴───┐                      │
    │   D13  D11  │  D12   │                      │
    │   SCK MOSI  │  MISO  │                      D2 (Interrupt)
    └─────────────┴────────┘

         MS5611 (CSB High)
         ┌────────────────┐
         │ VCC GND SDA SCL│
         │  ●   ●   ●   ● │
         │ CSB  PS        │
         │  ●   ●         │
         └──┬───┬─────────┘
            │   │
        VCC GND (CSB=VCC for addr 0x77)


         ESCs (to Motors)                      Buzzer & LED
         ┌────────────┐                        ┌──────────┐
         │ FL  FR     │                        │ Buzzer   │
         │ D3  D5     │                        │  +   -   │
         │            │                        │  ●   ●   │
         │ RL  RR     │                        │ D8  GND  │
         │ D9  D6     │                        └──────────┘
         └────────────┘
                                               ┌──────────┐
                                               │ LED      │
    Signal wires from ESC                      │  +   -   │
    go to Arduino pins.                        │  ●   ●   │
    Power (VCC/GND) from ESC                   │ D7  GND  │
    NOT connected to Arduino                   │ (220Ω)   │
    (ESC powered from battery).                └──────────┘
```

### NRF24L01 PA+LNA Connection Detail

```
NRF24L01 PA+LNA MODULE
══════════════════════

        Top View (Antenna side up)
        ┌───────────────────────────┐
        │                           │
        │     ┌─────────────┐       │
        │     │  PA+LNA     │       │
        │     │   Chip      │       │
        │     └─────────────┘       │
        │                           │
        │ ┌─┐ ┌─┐ ┌─┐ ┌─┐          │
        │ │●│ │●│ │●│ │●│  Row 1   │
        │ └─┘ └─┘ └─┘ └─┘          │
        │ ┌─┐ ┌─┐ ┌─┐ ┌─┐          │
        │ │●│ │●│ │●│ │●│  Row 2   │
        │ └─┘ └─┘ └─┘ └─┘          │
        └───────────────────────────┘
              │   │   │   │
          Row 1: GND VCC CE  CSN
          Row 2: SCK MOSI MISO IRQ


CONNECTION TABLE:
═════════════════

NRF24 Pin    Arduino Pin    Notes
─────────    ───────────    ─────
GND          GND            Common ground
VCC          3.3V           ⚠️ 3.3V ONLY! 5V will damage module
CE           D4             Chip Enable
CSN          D10            SPI Chip Select
SCK          D13            SPI Clock
MOSI         D11            SPI Data In
MISO         D12            SPI Data Out
IRQ          (NC)           Not used

⚠️ IMPORTANT: Add 10µF capacitor across VCC-GND
   near the NRF24 module for stability!
```

### MPU6050 Connection Detail

```
MPU6050 BREAKOUT BOARD
══════════════════════

        ┌────────────────────────┐
        │      ┌──────────┐      │
        │      │ MPU6050  │      │
        │      │   Chip   │      │
        │      └──────────┘      │
        │                        │
        │  VCC GND SCL SDA XDA XCL INT AD0
        │   ●   ●   ●   ●   ●   ●   ●   ●
        └───┬───┬───┬───┬───────────┬───┬─┘
            │   │   │   │           │   │
           5V  GND  A5  A4         D2  GND


CONNECTION TABLE:
═════════════════

MPU6050 Pin  Arduino Pin    Notes
───────────  ───────────    ─────
VCC          5V             Module has regulator
GND          GND            Common ground
SCL          A5             I2C Clock (with 4.7kΩ pull-up)
SDA          A4             I2C Data (with 4.7kΩ pull-up)
XDA          (NC)           Aux I2C (not used)
XCL          (NC)           Aux I2C (not used)
INT          D2             Data ready interrupt
AD0          GND            I2C address = 0x68

Note: Most breakout boards have pull-up resistors built-in.
      If using bare chip, add 4.7kΩ pull-ups to SCL and SDA.
```

### MS5611 Connection Detail

```
MS5611 BREAKOUT BOARD
═════════════════════

        ┌────────────────────────┐
        │      ┌──────────┐      │
        │      │  MS5611  │      │
        │      │  Sensor  │      │
        │      └──────────┘      │
        │                        │
        │  VIN  GND  SCL  SDA    │
        │   ●    ●    ●    ●     │
        └───┬────┬────┬────┬─────┘
            │    │    │    │
           5V  GND   A5   A4


CONNECTION TABLE:
═════════════════

MS5611 Pin   Arduino Pin    Notes
──────────   ───────────    ─────
VIN/VCC      5V             Module has regulator
GND          GND            Common ground
SCL          A5             Shared I2C bus with MPU6050
SDA          A4             Shared I2C bus with MPU6050

I2C Address: 0x77 (CSB pin high, default on most boards)

Note: MS5611 shares I2C bus with MPU6050.
      Both can be connected in parallel.
```

### ESC Connection Detail

```
ESC CONNECTIONS
═══════════════

    ESC Layout:
    ┌──────────────────────────────────┐
    │                                   │
    │  ┌───────┐        ┌───────────┐  │
    │  │ ESC   │        │           │  │
    │  │ Board │        │  Battery  │  │
    │  │       │        │   Wires   │  │
    │  └───────┘        │  + (Red)  │  │
    │                   │  - (Black)│  │
    │  ┌───────────┐    └───────────┘  │
    │  │  Motor    │                   │
    │  │  Wires    │    ┌───────────┐  │
    │  │ A  B  C   │    │ Signal    │  │
    │  └───────────┘    │ + - S     │  │
    │                   └───────────┘  │
    └──────────────────────────────────┘


    Signal Wire Connection:
    ───────────────────────

    ESC Signal   Arduino Pin   Motor Position
    ──────────   ───────────   ──────────────
    ESC 1 (S)    D3            Front-Left (FL)
    ESC 2 (S)    D5            Front-Right (FR)
    ESC 3 (S)    D6            Rear-Right (RR)
    ESC 4 (S)    D9            Rear-Left (RL)

    ⚠️ ESC Signal Ground:
       - Can connect ESC signal GND to Arduino GND
       - OR rely on common battery ground
       - Do NOT connect ESC +5V (BEC) to Arduino 5V
         unless you need it for power


    Motor Direction (X-Config):
    ───────────────────────────

            FRONT
         FL     FR
         CCW    CW
          ╲    ╱
           ╲  ╱
            ╲╱
            ╱╲
           ╱  ╲
          ╱    ╲
         CW    CCW
         RL     RR
            REAR

    To reverse direction: Swap any 2 motor wires
```

---

## 📻 Remote Controller Wiring

### Main Schematic

```
                        REMOTE CONTROLLER WIRING DIAGRAM
═══════════════════════════════════════════════════════════════════════════════

                                     ARDUINO NANO
                              ┌────────────────────────┐
                              │         USB            │
                              │      ┌───────┐         │
                              │      │       │         │
                          D13 │●     └───────┘       ●│ D12
                  (LED/SCK) ──┼──────────┐  ┌────────┼──── (SPI MISO)
                          D11 │●         │  │        ●│ D10
                (SPI MOSI) ───┼─────┐    │  │   ┌────┼──── NRF24 CSN
                           D9 │●    │    │  │   │   ●│ D8
                  NRF24 CE ───┼────┼────┼──┼───┼────┼──── (NC)
                           D7 │●    │    │  │   │   ●│ D6
                       (NC) ──┼────┼────┼──┼───┼────┼──── (NC)
                           D5 │●    │    │  │   │   ●│ D4
                  Button 2 ───┼────┼────┼──┼───┼────┼──── Button 1
                           D3 │●    │    │  │   │   ●│ D2
            SW2 (ARM/Kill) ───┼────┼────┼──┼───┼────┼──── SW1 (Alt Hold)
                          GND │●    │    │  │   │   ●│ GND
                              │     │    │  │   │    │
                          RST │●    │    │  │   │   ●│ RST
                              │     │    │  │   │    │
                           A0 │●    │    │  │   │   ●│ A7
             Left V (Thr) ────┼────┼────┼──┼───┼────┼──── (NC)
                           A1 │●    │    │  │   │   ●│ A6
             Left H (Yaw) ────┼────┼────┼──┼───┼────┼──── (NC)
                           A2 │●    │    │  │   │   ●│ A5
           Right V (Pitch) ───┼────┼────┼──┼───┼────┼──── (NC)
                           A3 │●    │    │  │   │   ●│ A4
            Right H (Roll) ───┼────┼────┴──┴───┴────┼──── (NC)
                          VIN │●                    ●│ AREF
                              │                      │
                          GND │●                    ●│ 3.3V ── NRF24 VCC
                              │                      │
                           5V │●                    ●│ 5V ──┬── Joystick VCC
                              │                      │      └── Button Pull-ups
                              └──────────────────────┘


          NRF24L01 PA+LNA                    JOYSTICKS
         ┌────────────────┐            ┌──────────────────────┐
         │  ●  ●  ●  ●    │            │   LEFT      RIGHT    │
         │ GND VCC CE CSN │            │  ┌─────┐   ┌─────┐   │
         │  ●  ●  ●  ●    │            │  │     │   │     │   │
         │ SCK MOSI MISO IRQ           │  │  ●  │   │  ●  │   │
         └───┬──┬───┬───┬─┘            │  │     │   │     │   │
             │  │   │   │              │  └──┬──┘   └──┬──┘   │
             │  │   │   └── (NC)       │     │         │      │
         ┌───┘  │   │                  │   V H GND   V H GND  │
         │   ┌──┘   └──┐               │   │ │  │    │ │  │   │
    ┌────┴───┴────┬────┴───┐           │  A0 A1 G   A2 A3 G   │
    │   D13  D11  │  D12   │           │   └──┴──┘   └──┴──┘  │
    │   SCK MOSI  │  MISO  │           │                      │
    └─────────────┴────────┘           └──────────────────────┘
                                        (Each joystick also needs VCC)


          BUTTONS                           SWITCHES
    ┌──────────────────┐               ┌──────────────────┐
    │  BTN1     BTN2   │               │  SW1      SW2    │
    │  ┌─┐      ┌─┐    │               │  ┌─┐      ┌─┐    │
    │  │ │      │ │    │               │  │ ├──    │ ├──  │
    │  └┬┘      └┬┘    │               │  └┬┘      └┬┘    │
    │   │        │     │               │   │        │     │
    │  D4       D5     │               │  D2       D3     │
    │   └──GND   └─GND │               │   └──GND   └─GND │
    └──────────────────┘               └──────────────────┘

    Using INPUT_PULLUP:                 Using INPUT_PULLUP:
    - Button pressed = LOW             - Switch ON = LOW (to GND)
    - Button released = HIGH           - Switch OFF = HIGH (floating)
```

### Joystick Wiring Detail

```
DUAL-AXIS JOYSTICK MODULE
═════════════════════════

        Typical Joystick Module:
        ┌────────────────────────────┐
        │      ┌──────────────┐      │
        │      │   Joystick   │      │
        │      │    Gimbal    │      │
        │      │      ●       │      │
        │      └──────────────┘      │
        │                            │
        │   GND  +5V  VRx  VRy  SW   │
        │    ●    ●    ●    ●    ●   │
        └────┬────┬────┬────┬────┬───┘
             │    │    │    │    │
            GND  5V   V    H   (NC)


LEFT JOYSTICK (Throttle/Yaw):
═════════════════════════════

Pin      Arduino    Function
───      ───────    ────────
GND      GND        Ground
+5V      5V         Power
VRx      A0         Throttle (Vertical)
VRy      A1         Yaw (Horizontal)
SW       (NC)       Button (not used)


RIGHT JOYSTICK (Pitch/Roll):
════════════════════════════

Pin      Arduino    Function
───      ───────    ────────
GND      GND        Ground
+5V      5V         Power
VRx      A2         Pitch (Vertical)
VRy      A3         Roll (Horizontal)
SW       (NC)       Button (not used)


AXIS ORIENTATION:
═════════════════

        UP (+)
          ▲
          │
   ◄──────┼──────►
  LEFT(-)  │  RIGHT(+)
          │
          ▼
        DOWN (-)

Note: If an axis is inverted, reverse it in software:
      value = 1023 - analogRead(pin);
```

### Button and Switch Wiring Detail

```
BUTTON WIRING (Using Internal Pull-up)
══════════════════════════════════════

        Arduino Internal
        Pull-up Resistor
              │
         ┌────┴────┐
         │  ~20kΩ  │
         └────┬────┘
              │
    Pin ──────●──────┬
              │      │
              │    ┌─┴─┐
              │    │   │  Momentary
              │    │ ● │  Push Button
              │    │   │
              │    └─┬─┘
              │      │
    GND ─────────────┘

    Button RELEASED: Pin reads HIGH (pulled up)
    Button PRESSED:  Pin reads LOW (shorted to GND)

    Code: pinMode(PIN, INPUT_PULLUP);
          if (digitalRead(PIN) == LOW) { /* pressed */ }


TOGGLE SWITCH WIRING (SPDT)
═══════════════════════════

    SPDT (Single Pole, Double Throw) Switch:

         Common
            │
       ┌────●────┐
       │         │
      NC   │    NO
       ●   │    ●
           │
         Wiper

    For our application (Simple ON/OFF):

    Only connect Common and one throw:

        Arduino Pin
             │
             │
        ┌────●────┐
        │   COM   │
        │         │  SPDT Switch
        │    ●────┼─── GND
        │   NO    │
        └─────────┘

    Switch OFF: Pin floating (HIGH via pull-up)
    Switch ON:  Pin connected to GND (LOW)

    Alternative: Use SPST (Single Pole, Single Throw)

        Arduino Pin
             │
        ┌────┴────┐
        │    ●    │  SPST Switch
        │    │    │
        └────┬────┘
             │
            GND
```

---

## 🔌 Power Distribution

### Flight Controller Power

```
POWER DISTRIBUTION - FLIGHT CONTROLLER
══════════════════════════════════════

                    LiPo Battery
                    (3S or 4S)
                         │
                         │
            ┌────────────┴───────────┐
            │                        │
            │    Power Distribution  │
            │         Board          │
            │                        │
            └─┬────┬────┬────┬────┬──┘
              │    │    │    │    │
              │    │    │    │    │
            ESC1 ESC2 ESC3 ESC4   │
            (FL) (FR) (RR) (RL)   │
              │    │    │    │    │
              │    │    │    │    │
              └────┴────┴────┴────┘
                       │
                  (One ESC's BEC)
                       │
                    5V to Arduino
                       │
              ┌────────┴────────┐
              │                 │
            Nano VIN        3.3V Reg
              │                 │
              │             NRF24 VCC
              │
         ┌────┴────┐
         │ Arduino │
         │  Nano   │
         └─────────┘


OPTION A: Power from ESC BEC
────────────────────────────
• Use 5V output from one ESC
• Connect to Arduino VIN or 5V pin
• Simple, no extra components

OPTION B: Dedicated 5V BEC
──────────────────────────
• Separate 5V BEC/regulator
• More reliable, isolated from ESC noise
• Recommended for production

⚠️ IMPORTANT:
• Never connect battery voltage directly to Arduino!
• NRF24 MUST be powered from 3.3V
• Add capacitors for stability (100µF on 5V, 10µF on 3.3V)
```

### Remote Controller Power

```
POWER DISTRIBUTION - REMOTE CONTROLLER
══════════════════════════════════════

         9V Battery
         or 2S LiPo
             │
             │
             ▼
    ┌─────────────────┐
    │ Arduino Nano    │
    │ VIN (7-12V)     │
    │                 │
    │    Internal     │
    │    Regulator    │
    │       │         │
    │    ┌──┴──┐      │
    │    5V  3.3V     │
    │    │    │       │
    └────┼────┼───────┘
         │    │
         │    └─── NRF24 VCC
         │
    ┌────┴────┐
    │ 5V Rail │
    └────┬────┘
         │
    ┌────┼────┬────────┐
    │    │    │        │
  Joy1 Joy2  LEDs  Pull-ups
    │    │    │        │
   GND  GND  GND     (internal)


USB POWER OPTION:
─────────────────
• Can power from USB during development
• Provides 5V directly to 5V rail
• NRF24 still needs 3.3V from Nano regulator
```

---

## ⚠️ Important Notes

### Common Mistakes to Avoid

1. **NRF24 on 5V**: Module will be damaged - MUST use 3.3V
2. **Missing capacitors**: Add 10-100µF near NRF24 for stability
3. **I2C without pull-ups**: Most modules have them, verify yours do
4. **ESC BEC conflicts**: Only connect one ESC's 5V output
5. **Reversed motor direction**: Swap any 2 of 3 motor wires to fix

### Signal Integrity Tips

- Keep NRF24 antenna away from motors and ESCs
- Use short wires for I2C connections
- Twist ESC signal wires with ground
- Add ferrite beads on power lines if experiencing interference

---

*Double-check all connections before applying power!*
