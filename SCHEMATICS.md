# Hardware Schematics and Pin Connections

## Flight Controller Schematic

### Power Distribution

```
Battery (7.4V - 14.8V)
    │
    ├─── ESC Power Distribution Board
    │       ├─── ESC FL (D3 signal)
    │       ├─── ESC FR (D5 signal)
    │       ├─── ESC RR (D6 signal)
    │       └─── ESC RL (D9 signal)
    │
    └─── Voltage Regulator (5V)
            ├─── Arduino Nano VIN
            ├─── NRF24L01 VCC
            ├─── MPU6050 VCC
            └─── MS5611 VCC
```

### Arduino Nano Pin Mapping

```
Arduino Nano Pinout:
─────────────────────────────────
Digital Pins:
  D2  → MPU6050 INT (interrupt, optional)
  D3  → ESC FL PWM output
  D4  → NRF24L01 CE
  D5  → ESC FR PWM output
  D6  → ESC RR PWM output
  D7  → Status LED (with 220Ω resistor)
  D8  → Buzzer
  D9  → ESC RL PWM output
  D10 → NRF24L01 CSN
  D11 → NRF24L01 MOSI (SPI)
  D12 → NRF24L01 MISO (SPI)
  D13 → NRF24L01 SCK (SPI)

Analog Pins:
  A4  → I2C SDA (MPU6050, MS5611)
  A5  → I2C SCL (MPU6050, MS5611)

Power:
  5V  → Sensor power
  GND → Common ground
  VIN → 7-12V input (via regulator)
```

### NRF24L01 PA+LNA Module

```
NRF24L01 PA+LNA Pinout:
─────────────────────────────────
VCC  → 3.3V (or 5V with level shifter)
GND  → GND
CE   → D4
CSN  → D10
SCK  → D13 (SPI)
MOSI → D11 (SPI)
MISO → D12 (SPI)
IRQ  → Not used (optional)
ANT  → Antenna (2.4GHz)
```

**Important:** NRF24L01 requires stable 3.3V. Use:
- 3.3V regulator from 5V
- OR level shifter if using 5V
- OR use 3.3V module variant

### MPU6050 Module

```
MPU6050 Pinout:
─────────────────────────────────
VCC → 5V (or 3.3V depending on module)
GND → GND
SCL → A5 (I2C)
SDA → A4 (I2C)
INT → D2 (optional interrupt)
```

**I2C Address:** 0x68 (default) or 0x69 (if AD0 pulled high)

**Pull-up Resistors:** Usually included on module (4.7kΩ)

### MS5611 Module

```
MS5611 Pinout:
─────────────────────────────────
VCC → 5V (or 3.3V depending on module)
GND → GND
SCL → A5 (I2C)
SDA → A4 (I2C)
```

**I2C Address:** 0x77 (default) or 0x76 (if CSB pulled low)

**Pull-up Resistors:** Usually included on module (4.7kΩ)

### ESC Connections

```
ESC (Electronic Speed Controller):
─────────────────────────────────
Signal → Arduino PWM pin (D3, D5, D6, D9)
GND    → Common ground
Power+ → Battery positive (via PDB)
Power- → Battery negative (via PDB)
Motor+ → Motor wire 1
Motor- → Motor wire 2
```

**ESC Calibration:**
- ESCs expect 1000-2000 μs PWM pulses
- 1000 μs = minimum throttle
- 1500 μs = neutral/idle
- 2000 μs = maximum throttle

### Status Indicators

```
Status LED:
─────────────────────────────────
LED Anode → D7 (via 220Ω resistor)
LED Cathode → GND

Buzzer:
─────────────────────────────────
Buzzer+ → D8
Buzzer- → GND
```

---

## Remote Controller Schematic

### Arduino Nano Pin Mapping

```
Arduino Nano Pinout:
─────────────────────────────────
Digital Pins:
  D2  → Switch 1 (Altitude Hold)
  D3  → Switch 2 (ARM/DISARM)
  D4  → Button 1 (Calibration)
  D5  → Button 2 (Motor ON/ESC Cal)
  D9  → NRF24L01 CE
  D10 → NRF24L01 CSN
  D11 → NRF24L01 MOSI (SPI)
  D12 → NRF24L01 MISO (SPI)
  D13 → NRF24L01 SCK (SPI)

Analog Pins:
  A0  → Left Joystick Vertical (Throttle)
  A1  → Left Joystick Horizontal (Yaw)
  A2  → Right Joystick Vertical (Pitch)
  A3  → Right Joystick Horizontal (Roll)

Power:
  5V  → Joystick power, switch power
  GND → Common ground
```

### Joystick Connections

```
Joystick Module (2-axis):
─────────────────────────────────
VCC → 5V
GND → GND
VRx → Arduino analog pin (horizontal)
VRy → Arduino analog pin (vertical)
SW  → Not used (optional button)

Typical Values:
  Center: ~512 (0-1023 range)
  Min: ~0
  Max: ~1023
```

**Calibration:** Performed automatically on startup

### Button Connections

```
Button Wiring:
─────────────────────────────────
Button Terminal 1 → Digital pin
Button Terminal 2 → GND

Arduino internal pull-up enabled:
  HIGH when not pressed
  LOW when pressed
```

### Switch Connections

```
Toggle Switch Wiring:
─────────────────────────────────
Switch Pin 1 → Digital pin (signal)
Switch Pin 2 → GND
Switch Pin 3 → GND (or not connected)

When ON:  Pin reads HIGH (1)
When OFF: Pin reads LOW (0)
```

---

## Complete Wiring Diagrams

### Flight Controller Block Diagram

```
                    ┌─────────────┐
                    │   Battery   │
                    │  (7.4-14.8V)│
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │  Power Dist │
                    │    Board    │
                    └──────┬──────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   ┌────▼────┐        ┌────▼────┐       ┌────▼────┐
   │ ESC FL  │        │ ESC FR  │       │ ESC RR  │
   │   D3    │        │   D5    │       │   D6    │
   └────┬────┘        └────┬────┘       └────┬────┘
        │                  │                  │
   ┌────▼────┐        ┌────▼────┐       ┌────▼────┐
   │ Motor FL│        │ Motor FR│       │ Motor RR│
   └─────────┘        └─────────┘       └─────────┘
                           │
                    ┌──────▼──────┐
                    │ Voltage Reg │
                    │   (5V/3.3V) │
                    └──────┬──────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   ┌────▼────┐        ┌────▼────┐       ┌────▼────┐
   │Arduino  │        │ NRF24L01│       │ MPU6050 │
   │  Nano   │        │ PA+LNA  │       │         │
   └────┬────┘        └────┬────┘       └────┬────┘
        │                  │                  │
   ┌────▼────┐        ┌────▼────┐       ┌────▼────┐
   │   LED   │        │ MS5611  │       │ Buzzer  │
   │   D7    │        │         │       │   D8    │
   └─────────┘        └─────────┘       └─────────┘
```

### Remote Controller Block Diagram

```
                    ┌─────────────┐
                    │   Battery   │
                    │   (9V/12V)  │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │Arduino Nano │
                    └──────┬──────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   ┌────▼────┐        ┌────▼────┐       ┌────▼────┐
   │Left     │        │Right    │       │NRF24L01 │
   │Joystick │        │Joystick │       │ PA+LNA  │
   │A0, A1   │        │A2, A3   │       │         │
   └─────────┘        └─────────┘       └─────────┘
        │                  │
   ┌────▼────┐        ┌────▼────┐
   │Button 1 │        │Button 2 │
   │   D4    │        │   D5    │
   └─────────┘        └─────────┘
        │                  │
   ┌────▼────┐        ┌────▼────┐
   │Switch 1 │        │Switch 2 │
   │   D2    │        │   D3    │
   └─────────┘        └─────────┘
```

---

## PCB Layout Recommendations

### Flight Controller PCB

**Component Placement:**
1. Arduino Nano: Center of board
2. NRF24L01: Edge with antenna clearance
3. MPU6050: Center, away from motors (vibration)
4. MS5611: Away from motors, exposed to air
5. ESCs: Near motor connections
6. Power distribution: Central, with large traces

**Ground Plane:**
- Use large ground plane
- Connect all GND together
- Minimize ground loops

**Power Traces:**
- Use thick traces for battery power (20-30 mil)
- Decouple power with capacitors (100μF + 0.1μF)
- Separate analog and digital power if possible

**Signal Traces:**
- Keep I2C traces short
- Add pull-up resistors if needed
- Shield sensitive signals

### Remote Controller PCB

**Component Placement:**
1. Arduino Nano: Center
2. NRF24L01: Edge with antenna
3. Joysticks: Ergonomic positions
4. Buttons/Switches: Accessible positions

**Layout Considerations:**
- Ergonomic button/switch placement
- Antenna clearance for NRF24L01
- Battery compartment access
- Comfortable grip design

---

## Power Requirements

### Flight Controller

**Current Draw:**
- Arduino Nano: ~50 mA
- NRF24L01: ~15 mA (TX), ~13 mA (RX)
- MPU6050: ~3.5 mA
- MS5611: ~1 mA
- LED: ~10 mA
- Buzzer: ~30 mA (when active)
- **Total:** ~110-120 mA

**Voltage:**
- Input: 7-12V (via regulator)
- 5V rail: Arduino, sensors
- 3.3V rail: NRF24L01 (if needed)

### Remote Controller

**Current Draw:**
- Arduino Nano: ~50 mA
- NRF24L01: ~15 mA
- Joysticks: ~5 mA each
- **Total:** ~75-80 mA

**Voltage:**
- 9V battery recommended
- Or 2x AA batteries (3V) with boost converter

---

## Safety Considerations

### Electrical Safety

1. **Power Isolation:**
   - Separate motor power from logic power
   - Use proper voltage regulators
   - Add fuses for protection

2. **Grounding:**
   - Common ground for all components
   - Avoid ground loops
   - Proper ESD protection

3. **EMI Protection:**
   - Decouple power supplies
   - Shield sensitive signals
   - Keep motor wires away from signal wires

### Mechanical Safety

1. **Vibration Isolation:**
   - Mount IMU with vibration dampening
   - Use soft mounting for sensors
   - Isolate from motor vibration

2. **Physical Protection:**
   - Enclose electronics
   - Protect from moisture
   - Secure all connections

---

## Testing Procedures

### Continuity Testing

1. **Power Rails:**
   - Verify 5V on all VCC pins
   - Verify GND continuity
   - Check for shorts

2. **Signal Lines:**
   - Verify I2C connections
   - Verify SPI connections
   - Check PWM outputs

### Functional Testing

1. **Power-On Test:**
   - Verify all components power up
   - Check LED indicators
   - Listen for beeps

2. **Communication Test:**
   - Verify NRF24L01 link
   - Check serial communication
   - Test sensor readings

3. **Motor Test:**
   - Test each ESC individually
   - Verify motor direction
   - Check PWM range

---

## Troubleshooting Wiring Issues

### No Power

- Check battery voltage
- Verify regulator output
- Check for shorts
- Verify connections

### Sensor Not Detected

- Check I2C wiring (SDA, SCL)
- Verify pull-up resistors
- Check sensor address
- Test with I2C scanner

### Communication Failure

- Verify NRF24L01 wiring
- Check power supply stability
- Verify antenna connection
- Check channel settings

### Motor Issues

- Verify ESC connections
- Check PWM signal
- Verify motor wiring
- Test ESC calibration

---

## Component Specifications

### Recommended Components

**Flight Controller:**
- Arduino Nano (ATmega328P)
- NRF24L01 PA+LNA module
- MPU6050 6-axis IMU
- MS5611 barometric sensor
- 5V buzzer
- LED with 220Ω resistor
- 4x ESCs (compatible with motors)
- Power distribution board

**Remote Controller:**
- Arduino Nano (ATmega328P)
- NRF24L01 PA+LNA module
- 2x 2-axis joysticks
- 2x push buttons
- 2x toggle switches
- 9V battery or battery pack

**Frame and Motors:**
- Quadcopter frame (250-500mm)
- 4x brushless motors
- 4x propellers (balanced)
- Battery (7.4V-14.8V LiPo)

---

This schematic guide provides all necessary information for building the flight control system. Always double-check connections before powering on!
