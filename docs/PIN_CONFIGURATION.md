# Pin Configuration Reference

Complete pin mapping for both Flight Controller and Remote Controller boards.

---

## Flight Controller Board Pin Configuration

### Arduino Nano Pinout

```
                    Arduino Nano
                   ┌─────────────┐
                   │   USB Port  │
                   ├─────────────┤
    3.3V (NRF) ────┤ 3.3V    VIN ├──── 12V Input
           GND ────┤ GND     GND ├──── GND
   Motor RL (D9)───┤ D13     RST ├──── Reset
   Motor RR (D6)───┤ D12      5V ├──── 5V Out
   Motor FR (D5)───┤ D11      A7 ├──── (unused)
    NRF CSN (D10)──┤ D10      A6 ├──── (unused)
   Motor FL (D3)───┤ D9       A5 ├──── MPU SCL
       Buzzer (D8)─┤ D8       A4 ├──── MPU SDA
     Status LED ───┤ D7       A3 ├──── (unused)
       (unused) ───┤ D6       A2 ├──── (unused)
       (unused) ───┤ D5       A1 ├──── (unused)
      NRF CE (D4)──┤ D4       A0 ├──── (unused)
       (unused) ───┤ D3      REF ├──── Reference
     MPU INT (D2)──┤ D2      GND ├──── GND
                   └─────────────┘
```

### Pin Assignments Table

| Component | Pin | Type | Description |
|-----------|-----|------|-------------|
| **NRF24L01** |
| CE | D4 | Digital Out | Chip Enable |
| CSN | D10 | Digital Out | Chip Select Not |
| SCK | D13 | SPI | SPI Clock |
| MOSI | D11 | SPI | Master Out Slave In |
| MISO | D12 | SPI | Master In Slave Out |
| VCC | 3.3V | Power | **Must use 3.3V!** |
| GND | GND | Ground | Ground |
| **MPU6050 IMU** |
| INT | D2 | Digital In | Interrupt pin |
| SDA | A4 | I2C | I2C Data |
| SCL | A5 | I2C | I2C Clock |
| VCC | 5V | Power | 5V Power |
| GND | GND | Ground | Ground |
| **ESC/Motors** |
| Front Left (FL) | D3 | PWM Out | PWM Signal (1000-2000us) |
| Front Right (FR) | D5 | PWM Out | PWM Signal (1000-2000us) |
| Rear Right (RR) | D6 | PWM Out | PWM Signal (1000-2000us) |
| Rear Left (RL) | D9 | PWM Out | PWM Signal (1000-2000us) |
| **Peripherals** |
| Buzzer | D8 | Digital Out | Audio feedback |
| Status LED | D7 | Digital Out | Visual status indicator |

---

## Remote Controller Board Pin Configuration

### Arduino Nano Pinout

```
                    Arduino Nano
                   ┌─────────────┐
                   │   USB Port  │
                   ├─────────────┤
    3.3V (NRF) ────┤ 3.3V    VIN ├──── 9-12V Input
           GND ────┤ GND     GND ├──── GND
       (unused) ───┤ D13     RST ├──── Reset
       (unused) ───┤ D12      5V ├──── 5V Out (Joysticks)
       (unused) ───┤ D11      A7 ├──── (unused)
    NRF CSN (D10)──┤ D10      A6 ├──── (unused)
      NRF CE (D9)──┤ D9       A5 ├──── (unused)
       (unused) ───┤ D8       A4 ├──── (unused)
       (unused) ───┤ D7       A3 ├──── Right Joystick H (Roll)
   Button_3 (D6)───┤ D6       A2 ├──── Right Joystick V (Pitch)
   Button_2 (D5)───┤ D5       A1 ├──── Left Joystick H (Yaw)
   Button_1 (D4)───┤ D4       A0 ├──── Left Joystick V (Throttle)
   Switch_2 (D3)───┤ D3      REF ├──── Reference
   Switch_1 (D2)───┤ D2      GND ├──── GND
                   └─────────────┘
```

### Pin Assignments Table

| Component | Pin | Type | Description |
|-----------|-----|------|-------------|
| **NRF24L01** |
| CE | D9 | Digital Out | Chip Enable |
| CSN | D10 | Digital Out | Chip Select Not |
| SCK | D13 | SPI | SPI Clock |
| MOSI | D11 | SPI | Master Out Slave In |
| MISO | D12 | SPI | Master In Slave Out |
| VCC | 3.3V | Power | **Must use 3.3V!** |
| GND | GND | Ground | Ground |
| **Left Joystick** |
| Vertical (Throttle) | A0 | Analog In | 0-1023, controls altitude |
| Horizontal (Yaw) | A1 | Analog In | 0-1023, controls rotation |
| VCC | 5V | Power | 5V Power |
| GND | GND | Ground | Ground |
| **Right Joystick** |
| Vertical (Pitch) | A2 | Analog In | 0-1023, forward/backward |
| Horizontal (Roll) | A3 | Analog In | 0-1023, left/right |
| VCC | 5V | Power | 5V Power |
| GND | GND | Ground | Ground |
| **Buttons (Active LOW)** |
| Button 1 (Calibrate) | D4 | Digital In | Gyro calibration trigger |
| Button 2 (ESC Cal) | D5 | Digital In | ESC calibration trigger |
| Button 3 (Motor Test) | D6 | Digital In | Motor test trigger |
| **Switches (Active LOW)** |
| Switch 1 (Alt Hold) | D2 | Digital In | Altitude hold toggle |
| Switch 2 (Arming) | D3 | Digital In | Arm/Disarm kill switch |

---

## Power Requirements

### Flight Controller
- **Input Voltage**: 7-12V (via VIN)
- **Arduino 5V Output**: Powers MPU6050
- **Arduino 3.3V Output**: Powers NRF24L01 (use external regulator recommended)
- **ESC Power**: Separate BEC or battery (5V for logic, battery for motors)

**⚠️ WARNING**: NRF24L01 PA+LNA can draw up to 115mA. Arduino Nano's 3.3V regulator may not provide sufficient current. **Use external 3.3V regulator (AMS1117-3.3) for reliable operation.**

### Remote Controller
- **Input Voltage**: 7-12V (via VIN) or 9V battery
- **Arduino 5V Output**: Powers joysticks
- **Arduino 3.3V Output**: Powers NRF24L01 (use external regulator recommended)

---

## PWM Pin Capabilities

Arduino Nano PWM-capable pins: **D3, D5, D6, D9, D10, D11**

Our motor assignments use: **D3, D5, D6, D9** ✓

---

## I2C Pin Mapping

Arduino Nano I2C pins are fixed:
- **SDA**: A4
- **SCL**: A5

---

## SPI Pin Mapping

Arduino Nano SPI pins are fixed:
- **MOSI**: D11
- **MISO**: D12
- **SCK**: D13
- **SS**: D10 (used as CSN for NRF)

---

## Interrupt Pins

Arduino Nano has 2 external interrupt pins:
- **INT0**: D2 (used for MPU6050 on FC)
- **INT1**: D3

---

## Connection Notes

### NRF24L01 Power Stability
1. Add 10µF capacitor across VCC and GND (close to module)
2. Keep wires short (< 10cm)
3. Use external 3.3V regulator if experiencing connection issues

### MPU6050 Pull-up Resistors
- MPU6050 has internal pull-ups for I2C
- External 4.7kΩ pull-ups recommended for long wires (> 15cm)

### ESC Signal Wires
- Keep signal wires away from power wires to reduce noise
- Use twisted pair or shielded cable if possible

---

**Last Updated**: 2025-11-30
