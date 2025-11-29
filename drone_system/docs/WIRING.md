# Drone System Wiring Guide

## Flight Controller Board Wiring

### Arduino Nano Pinout (Flight Controller)

```
                    ┌─────────────────┐
                    │   Arduino Nano  │
                    │                 │
            D13/SCK │█               █│ D12/MISO
           3V3/AREF │█               █│ D11/MOSI
              A0/NC │█               █│ D10/CSN (NRF24)
              A1/NC │█               █│ D9/RL Motor
              A2/NC │█               █│ D8/Buzzer
              A3/NC │█               █│ D7/Status LED
         A4/SDA/IMU │█               █│ D6/RR Motor
         A5/SCL/IMU │█               █│ D5/FR Motor
              A6/NC │█               █│ D4/CE (NRF24)
              A7/NC │█               █│ D3/FL Motor
                 5V │█               █│ D2/MPU INT
               RST │█               █│ GND
               GND │█               █│ RST
               VIN │█     USB       █│ RX0
                    │    ┌───┐       │
                    └────┘   └───────┘
```

### NRF24L01 PA+LNA Module Connection

```
         NRF24L01 PA+LNA                Arduino Nano
        ┌───────────────┐
        │  ┌─────────┐  │
        │  │ Antenna │  │
        │  └─────────┘  │
        │               │
        │  █ █ █ █      │
        │  1 2 3 4      │
        │  █ █ █ █      │
        │  5 6 7 8      │
        └───────────────┘
        
Pin 1 (GND)  ────────────> GND
Pin 2 (VCC)  ────────────> 3.3V (Use capacitor 10-100µF!)
Pin 3 (CE)   ────────────> D4
Pin 4 (CSN)  ────────────> D10
Pin 5 (SCK)  ────────────> D13
Pin 6 (MOSI) ────────────> D11
Pin 7 (MISO) ────────────> D12
Pin 8 (IRQ)  ────────────> Not Connected

⚠️ IMPORTANT: Add 10-100µF capacitor between VCC and GND on NRF24L01!
   The PA+LNA version draws high current spikes.
```

### MPU6050 IMU Connection

```
         MPU6050                    Arduino Nano
        ┌─────────┐
        │         │
        │ VCC ────────────────────> 5V (or 3.3V)
        │ GND ────────────────────> GND
        │ SCL ────────────────────> A5 (SCL)
        │ SDA ────────────────────> A4 (SDA)
        │ XDA ────────────────────> Not Connected
        │ XCL ────────────────────> Not Connected
        │ AD0 ────────────────────> GND (Address = 0x68)
        │ INT ────────────────────> D2
        │         │
        └─────────┘
```

### ESC Motor Connections

```
                    FRONT
                      │
         FL(CW)───────┼───────FR(CCW)
            │         │         │
            │    ┌────┴────┐    │
            │    │  DRONE  │    │
            │    │  BODY   │    │
            │    └────┬────┘    │
            │         │         │
         RL(CCW)──────┼──────RR(CW)
                      │
                    REAR


Motor to ESC to Arduino Connections:

┌────────────────────────────────────────────────────────────┐
│ Position │ Direction │ Arduino Pin │ ESC Signal │ Motor   │
├──────────┼───────────┼─────────────┼────────────┼─────────┤
│ FL       │ CW        │ D3 (PWM)    │ White/Sig  │ 3-wire  │
│ FR       │ CCW       │ D5 (PWM)    │ White/Sig  │ 3-wire  │
│ RR       │ CW        │ D6 (PWM)    │ White/Sig  │ 3-wire  │
│ RL       │ CCW       │ D9 (PWM)    │ White/Sig  │ 3-wire  │
└────────────────────────────────────────────────────────────┘

ESC Power Connections:
- ESC Red wire → Battery +
- ESC Black wire → Battery -
- ESC BEC (if present) → Can power Arduino via VIN

ESC Signal Connections:
- ESC Signal (White/Yellow) → Arduino PWM pin
- ESC Signal GND (Black/Brown) → Arduino GND
```

### Buzzer & LED Connection

```
        Buzzer (Active or Passive)          Status LED
        ┌─────────────────┐                 ┌─────────┐
        │                 │                 │   LED   │
        │    (+) ─────────────> D8          │ (+)─────────> D7
        │    (-) ─────────────> GND         │ (-)──┬──────> GND
        │                 │                 │      │
        └─────────────────┘                 │    [330Ω]
                                            └─────────┘
```

---

## Remote Controller Board Wiring

### Arduino Nano Pinout (Remote Controller)

```
                    ┌─────────────────┐
                    │   Arduino Nano  │
                    │                 │
            D13/SCK │█               █│ D12/MISO
           3V3/AREF │█               █│ D11/MOSI
     Left Joy V/A0  │█               █│ D10/CSN (NRF24)
     Left Joy H/A1  │█               █│ D9/CE (NRF24)
    Right Joy V/A2  │█               █│ D8/NC
    Right Joy H/A3  │█               █│ D7/NC
              A4/NC │█               █│ D6/NC
              A5/NC │█               █│ D5/Button 2
              A6/NC │█               █│ D4/Button 1
              A7/NC │█               █│ D3/Switch 2
                 5V │█               █│ D2/Switch 1
               RST │█               █│ GND
               GND │█               █│ RST
               VIN │█     USB       █│ RX0
                    │    ┌───┐       │
                    └────┘   └───────┘
```

### NRF24L01 PA+LNA Module Connection (Remote)

```
         NRF24L01 PA+LNA                Arduino Nano
        ┌───────────────┐
        │  ┌─────────┐  │
        │  │ Antenna │  │
        │  └─────────┘  │
        │               │
        │  █ █ █ █      │
        │  1 2 3 4      │
        │  █ █ █ █      │
        │  5 6 7 8      │
        └───────────────┘
        
Pin 1 (GND)  ────────────> GND
Pin 2 (VCC)  ────────────> 3.3V (Use capacitor 10-100µF!)
Pin 3 (CE)   ────────────> D9
Pin 4 (CSN)  ────────────> D10
Pin 5 (SCK)  ────────────> D13
Pin 6 (MOSI) ────────────> D11
Pin 7 (MISO) ────────────> D12
Pin 8 (IRQ)  ────────────> Not Connected
```

### Joystick Connections

```
          Left Joystick                      Right Joystick
         (Throttle/Yaw)                      (Pitch/Roll)
        ┌─────────────┐                     ┌─────────────┐
        │    ┌───┐    │                     │    ┌───┐    │
        │    │ O │    │                     │    │ O │    │
        │    └───┘    │                     │    └───┘    │
        │             │                     │             │
        │ GND ────────────> GND             │ GND ────────────> GND
        │ +5V ────────────> 5V              │ +5V ────────────> 5V
        │ VRx ────────────> A1 (Yaw)        │ VRx ────────────> A3 (Roll)
        │ VRy ────────────> A0 (Throttle)   │ VRy ────────────> A2 (Pitch)
        │ SW  ────────────> NC              │ SW  ────────────> NC
        │             │                     │             │
        └─────────────┘                     └─────────────┘


Joystick Orientation (View from Top):

     LEFT JOYSTICK              RIGHT JOYSTICK
    ┌─────────────┐            ┌─────────────┐
    │      ↑      │            │      ↑      │
    │   Throttle  │            │    Pitch    │
    │   Increase  │            │   Forward   │
    │      │      │            │      │      │
    │ ←────┼────→ │            │ ←────┼────→ │
    │ Yaw  │  Yaw │            │ Roll │ Roll │
    │ Left │ Right│            │ Left │ Right│
    │      │      │            │      │      │
    │   Throttle  │            │    Pitch    │
    │   Decrease  │            │   Backward  │
    │      ↓      │            │      ↓      │
    └─────────────┘            └─────────────┘
```

### Push Buttons Connection

```
        Button 1 (Calibration)              Button 2 (Motor On/Arm)
        ┌───────────────┐                   ┌───────────────┐
        │               │                   │               │
        │    ┌───┐      │                   │    ┌───┐      │
        │    │BTN│      │                   │    │BTN│      │
        │    └─┬─┘      │                   │    └─┬─┘      │
        │      │        │                   │      │        │
        │   Pin 1 ──────────> D4            │   Pin 1 ──────────> D5
        │   Pin 2 ──────────> GND           │   Pin 2 ──────────> GND
        │               │                   │               │
        └───────────────┘                   └───────────────┘

Note: Internal pull-up resistors are enabled. No external resistors needed.
      Button pressed = LOW, Button released = HIGH
```

### Toggle Switches Connection

```
        Switch 1 (Altitude Hold)            Switch 2 (Arm/Kill Switch)
        ┌───────────────┐                   ┌───────────────┐
        │     ───       │                   │     ───       │
        │    /   \      │                   │    /   \      │
        │   │     │     │                   │   │     │     │
        │   1  2  3     │                   │   1  2  3     │
        │   │  │  │     │                   │   │  │  │     │
        │   │  │  │     │                   │   │  │  │     │
        │   │  │  └─────────> NC            │   │  │  └─────────> NC
        │   │  └────────────> GND           │   │  └────────────> GND
        │   └───────────────> D2            │   └───────────────> D3
        │               │                   │               │
        └───────────────┘                   └───────────────┘

Note: Internal pull-up resistors are enabled.
      Switch ON (connected to GND) = State active
      Switch OFF (floating) = State inactive
```

---

## Power Supply Requirements

### Flight Controller Power

```
┌────────────────────────────────────────────────────────────────┐
│ Component          │ Voltage │ Current   │ Notes               │
├────────────────────┼─────────┼───────────┼─────────────────────┤
│ Arduino Nano       │ 5V/VIN  │ 50mA      │ Via USB or BEC      │
│ NRF24L01 PA+LNA    │ 3.3V    │ 150mA max │ Use capacitor!      │
│ MPU6050            │ 3.3-5V  │ 5mA       │ Has onboard reg     │
│ LED                │ 5V      │ 20mA      │ With 330Ω resistor  │
│ Buzzer             │ 5V      │ 30mA      │ Active buzzer       │
│ ESCs (x4)          │ LiPo    │ Varies    │ Match to motors     │
└────────────────────────────────────────────────────────────────┘

Recommended: 3S or 4S LiPo battery (11.1V or 14.8V)
ESC BEC output can power Arduino (check BEC current rating)
```

### Remote Controller Power

```
┌────────────────────────────────────────────────────────────────┐
│ Component          │ Voltage │ Current   │ Notes               │
├────────────────────┼─────────┼───────────┼─────────────────────┤
│ Arduino Nano       │ 5V/VIN  │ 50mA      │ Via USB or battery  │
│ NRF24L01 PA+LNA    │ 3.3V    │ 150mA max │ Use capacitor!      │
│ Joysticks (x2)     │ 5V      │ 10mA      │ Potentiometers      │
│ Buttons/Switches   │ -       │ <1mA      │ Internal pullups    │
└────────────────────────────────────────────────────────────────┘

Recommended: 2S LiPo (7.4V) or 6xAA batteries via VIN
Can also use USB power bank for development
```

---

## Component List

### Flight Controller
- 1x Arduino Nano (ATmega328P)
- 1x NRF24L01 PA+LNA module
- 1x MPU6050 6-axis IMU
- 1x Active buzzer (5V)
- 1x LED (any color) + 330Ω resistor
- 4x ESC (matching your motors)
- 4x Brushless motors
- 1x 10-100µF capacitor (for NRF24L01)
- Connectors and wires

### Remote Controller
- 1x Arduino Nano (ATmega328P)
- 1x NRF24L01 PA+LNA module
- 2x Analog joystick modules
- 2x Momentary push buttons
- 2x SPDT toggle switches
- 1x 10-100µF capacitor (for NRF24L01)
- Enclosure
- Connectors and wires
