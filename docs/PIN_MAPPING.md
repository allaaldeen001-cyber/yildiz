# Pin Mapping Reference

## 📌 Quick Reference Guide

---

## Flight Controller - Arduino Nano Pin Assignments

### Digital Pins

| Pin | Component | Function | Direction | Notes |
|-----|-----------|----------|-----------|-------|
| D0 | - | RX (Serial) | Input | Reserved for USB/Serial |
| D1 | - | TX (Serial) | Output | Reserved for USB/Serial |
| D2 | MPU6050 | Interrupt | Input | Optional, currently unused |
| D3 | Motor FL | PWM Signal | Output | Front Left motor (ESC) |
| D4 | NRF24L01 | CE (Chip Enable) | Output | **Critical for NRF** |
| D5 | Motor FR | PWM Signal | Output | Front Right motor (ESC) |
| D6 | Motor RR | PWM Signal | Output | Rear Right motor (ESC) |
| D7 | LED | Status Indicator | Output | Visual feedback |
| D8 | Buzzer | Audio Output | Output | Beep patterns |
| D9 | Motor RL | PWM Signal | Output | Rear Left motor (ESC) |
| D10 | NRF24L01 | CSN (Chip Select) | Output | **SPI, critical** |
| D11 | NRF24L01 | MOSI (SPI) | Output | **SPI, critical** |
| D12 | NRF24L01 | MISO (SPI) | Input | **SPI, critical** |
| D13 | NRF24L01 | SCK (SPI Clock) | Output | **SPI, critical** |

---

### Analog Pins

| Pin | Component | Function | Direction | Notes |
|-----|-----------|----------|-----------|-------|
| A0 | - | Reserved | - | Future use (voltage monitor) |
| A1 | - | Reserved | - | Future use |
| A2 | - | Reserved | - | Future use |
| A3 | - | Reserved | - | Future use |
| A4 | MPU6050 + MS5611 | SDA (I2C Data) | Bidirectional | **Shared I2C bus** |
| A5 | MPU6050 + MS5611 | SCL (I2C Clock) | Output | **Shared I2C bus** |
| A6 | - | Reserved | Input | Voltage divider (future) |
| A7 | - | Reserved | Input | Future use |

---

### Power Pins

| Pin | Source | Voltage | Notes |
|-----|--------|---------|-------|
| VIN | ESC BEC | 5V | Main power input from ESC |
| 5V | Regulated | 5V | Power output for sensors |
| 3.3V | Regulated | 3.3V | **Use for NRF24L01!** |
| GND | Common | 0V | Common ground (multiple pins) |

---

## Remote Controller - Arduino Nano Pin Assignments

### Digital Pins

| Pin | Component | Function | Direction | Notes |
|-----|-----------|----------|-----------|-------|
| D0 | - | RX (Serial) | Input | Reserved for USB/Serial |
| D1 | - | TX (Serial) | Output | Reserved for USB/Serial |
| D2 | Switch 1 | Altitude Hold | Input | INPUT_PULLUP, active LOW |
| D3 | Switch 2 | Arm/Disarm | Input | INPUT_PULLUP, active LOW |
| D4 | Button 1 | Calibration | Input | INPUT_PULLUP, active LOW |
| D5 | Button 2 | ESC Calibration | Input | INPUT_PULLUP, active LOW |
| D6 | - | Reserved | - | Future expansion |
| D7 | - | Reserved | - | Future expansion |
| D8 | - | Reserved | - | Future expansion |
| D9 | NRF24L01 | CE (Chip Enable) | Output | **Critical for NRF** |
| D10 | NRF24L01 | CSN (Chip Select) | Output | **SPI, critical** |
| D11 | NRF24L01 | MOSI (SPI) | Output | **SPI, critical** |
| D12 | NRF24L01 | MISO (SPI) | Input | **SPI, critical** |
| D13 | NRF24L01 | SCK (SPI Clock) | Output | **SPI, critical** |

---

### Analog Pins

| Pin | Component | Function | Direction | Notes |
|-----|-----------|----------|-----------|-------|
| A0 | Left Joystick | Throttle (Vertical) | Input | 0-1023 ADC |
| A1 | Left Joystick | Yaw (Horizontal) | Input | 0-1023 ADC |
| A2 | Right Joystick | Pitch (Vertical) | Input | 0-1023 ADC |
| A3 | Right Joystick | Roll (Horizontal) | Input | 0-1023 ADC |
| A4 | - | Reserved | - | Could add I2C display |
| A5 | - | Reserved | - | Could add I2C display |
| A6 | - | Reserved | Input | Battery monitor (future) |
| A7 | - | Reserved | Input | Future expansion |

---

### Power Pins

| Pin | Source | Voltage | Notes |
|-----|--------|---------|-------|
| VIN | 9V Battery or 2S LiPo | 7-12V | Main power input |
| 5V | Regulated | 5V | Power for joysticks |
| 3.3V | Regulated | 3.3V | **Use for NRF24L01!** |
| GND | Common | 0V | Common ground (multiple pins) |

---

## 🔌 Detailed Connection Diagrams

### Flight Controller - NRF24L01 PA+LNA

```
Arduino Nano          NRF24L01 PA+LNA
┌────────────┐        ┌──────────────┐
│         D4 ├────────┤ CE           │
│        D10 ├────────┤ CSN          │
│        D11 ├────────┤ MOSI         │
│        D12 ├────────┤ MISO         │
│        D13 ├────────┤ SCK          │
│       3.3V ├───┬────┤ VCC          │ ⚠️ 3.3V ONLY!
│            │   │    │              │
│            │  ═╪═   └──────────────┘
│            │ 100µF   Capacitor
│        GND ├───┴────┤ GND          │
└────────────┘        └──────────────┘
```

---

### Flight Controller - MPU6050

```
Arduino Nano          MPU6050
┌────────────┐        ┌──────────────┐
│         A4 ├────────┤ SDA          │
│         A5 ├────────┤ SCL          │
│         D2 ├────────┤ INT          │ (optional)
│         5V ├────────┤ VCC          │
│        GND ├────────┤ GND          │
│            │        │ XDA          │ (not used)
│            │        │ XCL          │ (not used)
│        GND ├────────┤ AD0          │ (sets address 0x68)
└────────────┘        └──────────────┘
```

---

### Flight Controller - MS5611

```
Arduino Nano          MS5611
┌────────────┐        ┌──────────────┐
│         A4 ├────────┤ SDA          │ Shared with MPU6050
│         A5 ├────────┤ SCL          │ Shared with MPU6050
│       3.3V ├────────┤ VCC          │ ⚠️ 3.3V recommended
│        GND ├────────┤ GND          │
└────────────┘        └──────────────┘

Note: MS5611 shares I2C bus with MPU6050
```

---

### Flight Controller - Buzzer & LED

```
Arduino Nano          Buzzer              LED
┌────────────┐        ┌────────┐          
│         D8 ├────────┤ +      │          ┌─────┐
│        GND ├────────┤ -      │      D7 ─┤ 220Ω├───┤>├─── GND
└────────────┘        └────────┘          └─────┘   LED
                                          Resistor
```

---

### Flight Controller - Motors (ESCs)

```
Arduino Nano          ESC                Motor
┌────────────┐        ┌────────────┐     ┌────────┐
│         D3 ├────────┤ Signal (FL)├─────┤ FL ↺   │
│         D5 ├────────┤ Signal (FR)├─────┤ FR ↻   │
│         D6 ├────────┤ Signal (RR)├─────┤ RR ↺   │
│         D9 ├────────┤ Signal (RL)├─────┤ RL ↻   │
│            │        │            │     └────────┘
│        GND ├────┬───┤ GND        │
│            │    │   │            │
│        VIN ├────┼───┤ 5V BEC     │ (from one ESC only)
└────────────┘    │   └────────────┘
                  │
                  │   Battery
                  │   ┌────────────┐
                  ├───┤ + (11.1V)  │
                  └───┤ -          │
                      └────────────┘
```

---

### Remote Controller - NRF24L01 PA+LNA

```
Arduino Nano          NRF24L01 PA+LNA
┌────────────┐        ┌──────────────┐
│         D9 ├────────┤ CE           │
│        D10 ├────────┤ CSN          │
│        D11 ├────────┤ MOSI         │
│        D12 ├────────┤ MISO         │
│        D13 ├────────┤ SCK          │
│       3.3V ├───┬────┤ VCC          │ ⚠️ 3.3V ONLY!
│            │   │    │              │
│            │  ═╪═   └──────────────┘
│            │ 100µF   Capacitor
│        GND ├───┴────┤ GND          │
└────────────┘        └──────────────┘
```

---

### Remote Controller - Joysticks

```
Arduino Nano          Left Joystick       Right Joystick
┌────────────┐        ┌────────────┐      ┌────────────┐
│         A0 ├────────┤ VRy (Vert) │      │            │
│         A1 ├────────┤ VRx (Horiz)│      │            │
│         A2 ├────────┼────────────┤──────┤ VRy (Vert) │
│         A3 ├────────┼────────────┤──────┤ VRx (Horiz)│
│            │        │            │      │            │
│         5V ├────┬───┤ VCC        ├───┬──┤ VCC        │
│        GND ├────┼───┤ GND        ├───┴──┤ GND        │
└────────────┘    │   │ SW         │      │ SW         │
                  │   └────────────┘      └────────────┘
                  │   (not used)          (not used)
```

---

### Remote Controller - Switches & Buttons

```
Arduino Nano          Switches              Buttons
┌────────────┐        
│         D2 ├────┬── Switch 1 ──┬── GND    D4 ─┬── Button 1 ──┬── GND
│         D3 ├────┼── Switch 2 ──┼── GND    D5 ─┴── Button 2 ──┴── GND
└────────────┘    │               │
                  │               │
            (when ON: connects    (when pressed: connects
             pin to GND)           pin to GND)
             
Note: Uses INPUT_PULLUP mode
      Open = HIGH (1)
      Closed = LOW (0)
```

---

## ⚠️ Important Pin Notes

### Critical - Do Not Change:
- **SPI Pins** (D10, D11, D12, D13): Hardware SPI for NRF24L01
- **I2C Pins** (A4, A5): Hardware I2C for sensors
- **Serial Pins** (D0, D1): USB communication

### PWM-Capable Pins (for motors):
- D3, D5, D6, D9, D10, D11
- Current code uses: D3, D5, D6, D9

### Interrupt-Capable Pins:
- D2, D3
- D2 reserved for MPU6050 interrupt (optional feature)

---

## 🔄 Pin Modifications (Advanced)

### If You Need to Change Motor Pins:

**Must use PWM-capable pins!**

Valid alternatives:
- D3, D5, D6, D9 (current)
- Could also use: D10, D11 (but conflicts with SPI!)

**Example:**
```cpp
// Change in FlightController.ino:
#define MOTOR_FL_PIN  3  // OK (PWM)
#define MOTOR_FR_PIN  5  // OK (PWM)
#define MOTOR_RR_PIN  6  // OK (PWM)
#define MOTOR_RL_PIN  9  // OK (PWM)

// WRONG - Not PWM capable:
#define MOTOR_FL_PIN  4  // ERROR - No PWM!
```

---

### If You Need to Change NRF CE/CSN Pins:

**Can use any digital pin**, but avoid:
- D0, D1 (Serial)
- D10, D11, D12, D13 (keep for SPI)

**Flight Controller Example:**
```cpp
// Original:
#define NRF_CE_PIN   4
#define NRF_CSN_PIN  10

// Alternative (if needed):
#define NRF_CE_PIN   7  // OK
#define NRF_CSN_PIN  8  // OK
// But then buzzer/LED need new pins!
```

---

## 📊 Pin Usage Summary

### Flight Controller Pin Usage

| Function | Pins Used | Total |
|----------|-----------|-------|
| Motors | D3, D5, D6, D9 | 4 |
| NRF24L01 | D4, D10-D13 | 5 |
| Sensors (I2C) | A4, A5 | 2 |
| Buzzer | D8 | 1 |
| LED | D7 | 1 |
| Reserved (Serial) | D0, D1 | 2 |
| **Total Used** | | **15/20** |
| **Available** | D2, A0-A3, A6-A7 | **5** |

---

### Remote Controller Pin Usage

| Function | Pins Used | Total |
|----------|-----------|-------|
| NRF24L01 | D9, D10-D13 | 5 |
| Joysticks | A0-A3 | 4 |
| Switches | D2, D3 | 2 |
| Buttons | D4, D5 | 2 |
| Reserved (Serial) | D0, D1 | 2 |
| **Total Used** | | **15/20** |
| **Available** | D6-D8, A4-A7 | **5** |

---

## 🔌 Voltage Reference Table

| Component | Operating Voltage | Notes |
|-----------|-------------------|-------|
| Arduino Nano | 5V (regulated) | VIN accepts 7-12V |
| NRF24L01 | **3.3V ONLY** | Will damage at 5V! |
| MPU6050 | 3.3V or 5V | Logic level tolerant |
| MS5611 | 3.3V preferred | May work at 5V |
| Buzzer | 5V | Passive or active |
| LED | 2-3V | Use 220Ω resistor |
| ESC Signal | 3.3V-5V | Standard PWM |
| Joysticks | 5V | Analog potentiometers |
| Switches/Buttons | Pull-up to 5V | Connect to GND |

---

## 🛠️ Troubleshooting Pin Issues

### Problem: NRF24L01 not working
**Check:**
- [ ] VCC connected to 3.3V (NOT 5V!)
- [ ] All 7 pins connected (VCC, GND, CE, CSN, SCK, MOSI, MISO)
- [ ] 100µF capacitor on VCC/GND
- [ ] Wires < 10cm

### Problem: Motors not spinning
**Check:**
- [ ] Motors on PWM pins (D3, D5, D6, D9)
- [ ] ESC signal wires connected
- [ ] Common ground between Arduino and ESC
- [ ] ESC powered from battery

### Problem: Sensors not detected
**Check:**
- [ ] SDA = A4, SCL = A5
- [ ] I2C pull-up resistors (if long wires)
- [ ] Correct voltage (3.3V for MS5611)
- [ ] Run I2C scanner to detect addresses

### Problem: Joysticks not working
**Check:**
- [ ] Connected to analog pins (A0-A3)
- [ ] VCC = 5V, GND connected
- [ ] Read values with `analogRead(A0)`
- [ ] Should show ~512 at center

---

## 📷 Visual Pin Diagrams

```
Arduino Nano Pinout (Top View)
        ┌─────────┐
    D13 │●       ●│ D12
    3V3 │●       ●│ D11
    REF │●       ●│ D10
     A0 │●       ●│ D9
     A1 │●       ●│ D8
     A2 │●       ●│ D7
     A3 │●       ●│ D6
     A4 │●       ●│ D5
     A5 │●       ●│ D4
     A6 │●       ●│ D3
     A7 │●       ●│ D2
    5V  │●       ●│ GND
    RST │●       ●│ RST
    GND │●       ●│ D0 (RX)
    VIN │●       ●│ D1 (TX)
        └─────────┘
         USB Port
```

---

**Need more details?** See WIRING_GUIDE.md for complete wiring instructions.
