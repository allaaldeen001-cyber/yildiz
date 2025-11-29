# Pin Reference Card

## Flight Controller Pinout

```
Arduino Nano Pin    Component          Function
─────────────────────────────────────────────────────
D2                  MPU6050 INT        Interrupt pin
D3                  ESC FL             Front Left Motor
D4                  NRF24L01 CE        Chip Enable
D5                  ESC FR             Front Right Motor
D6                  ESC RR             Rear Right Motor
D7                  LED                 Status LED (via 220Ω)
D8                  Buzzer             Audio feedback
D9                  ESC RL             Rear Left Motor
D10                 NRF24L01 CSN        Chip Select
D11                 NRF24L01 MOSI       SPI Data Out
D12                 NRF24L01 MISO       SPI Data In
D13                 NRF24L01 SCK        SPI Clock
A4 (SDA)            MPU6050 SDA        I2C Data
A4 (SDA)            MS5611 SDA         I2C Data
A5 (SCL)            MPU6050 SCL        I2C Clock
A5 (SCL)            MS5611 SCL         I2C Clock
3.3V                NRF24L01 VCC       3.3V Power
5V                  MPU6050 VCC        5V Power
5V                  MS5611 VCC         5V Power
GND                 All GND pins       Common Ground
```

## Remote Controller Pinout

```
Arduino Nano Pin    Component          Function
─────────────────────────────────────────────────────
D2                  Switch 1           Position Hold
D3                  Switch 2           Arming/Kill Switch
D4                  Button 1           Calibration
D5                  Button 2          ESC Calibration/Motor On
D9                  NRF24L01 CE        Chip Enable
D10                 NRF24L01 CSN        Chip Select
D11                 NRF24L01 MOSI       SPI Data Out
D12                 NRF24L01 MISO       SPI Data In
D13                 NRF24L01 SCK        SPI Clock
A0                  Left Joystick V    Throttle (Up/Down)
A1                  Left Joystick H    Yaw (Left/Right)
A2                  Right Joystick V   Pitch (Forward/Backward)
A3                  Right Joystick H   Roll (Left/Right)
3.3V                NRF24L01 VCC       3.3V Power
5V                  Joysticks VCC      5V Power
GND                 All GND pins       Common Ground
```

## Component Pin Details

### NRF24L01 PA+LNA Module
```
Pin     Flight Controller    Remote Controller
──────────────────────────────────────────────
VCC     3.3V                 3.3V
GND     GND                  GND
CE      D4                   D9
CSN     D10                  D10
SCK     D13                  D13
MOSI    D11                  D11
MISO    D12                  D12
```

### MPU6050
```
Pin     Flight Controller
─────────────────────────
VCC     5V
GND     GND
SDA     A4 (SDA)
SCL     A5 (SCL)
INT     D2
```

### MS5611
```
Pin     Flight Controller
─────────────────────────
VCC     5V
GND     GND
SDA     A4 (SDA)
SCL     A5 (SCL)
```

### ESCs (4x)
```
Signal Pin    Flight Controller    Motor Position
──────────────────────────────────────────────────
FL Signal     D3                   Front Left
FR Signal     D5                   Front Right
RR Signal     D6                   Rear Right
RL Signal     D9                   Rear Left

All ESCs:
- Power: Connect to Battery (with BEC for Arduino)
- Ground: Connect to GND
```

### Joysticks (2x)
```
Left Joystick:
- V (Vertical)    → A0 (Throttle)
- H (Horizontal)  → A1 (Yaw)
- VCC             → 5V
- GND             → GND

Right Joystick:
- V (Vertical)    → A2 (Pitch)
- H (Horizontal)  → A3 (Roll)
- VCC             → 5V
- GND             → GND
```

### Buttons (2x)
```
Button 1 (D4):
- One side → D4
- Other side → GND
(Uses internal pull-up)

Button 2 (D5):
- One side → D5
- Other side → GND
(Uses internal pull-up)
```

### Toggle Switches (2x)
```
Switch 1 (D2) - Position Hold:
- Pin 1 → D2
- Pin 2 → GND
- Pin 3 → GND
(Uses internal pull-up)

Switch 2 (D3) - Arming/Kill Switch:
- Pin 1 → D3
- Pin 2 → GND
- Pin 3 → GND
(Uses internal pull-up)
```

### Status LED
```
LED:
- Anode → D7 (via 220Ω resistor)
- Cathode → GND
```

### Buzzer
```
Buzzer:
- Positive → D8
- Negative → GND
```

## Power Requirements

### Flight Controller
- **USB Power**: 5V via USB (for testing)
- **Battery Power**: Via ESC BEC (5V) or separate regulator
- **Current**: ~200-500mA (depending on sensors)

### Remote Controller
- **USB Power**: 5V via USB
- **Battery Power**: 5V via battery pack or regulator
- **Current**: ~100-200mA

## Important Notes

1. **I2C Bus**: MPU6050 and MS5611 share A4 (SDA) and A5 (SCL)
2. **SPI Bus**: NRF24L01 uses hardware SPI (D11-D13)
3. **PWM Pins**: Motors use Servo library (D3, D5, D6, D9)
4. **Analog Pins**: Joysticks use A0-A3
5. **Pull-up Resistors**: Buttons and switches use internal pull-ups
6. **Power**: NRF24L01 requires 3.3V, others use 5V

## Wiring Tips

- Use separate power supply for ESCs (high current)
- Use BEC from one ESC to power Arduino (5V)
- Keep NRF24L01 power supply clean (use capacitor if needed)
- Use twisted pairs for I2C wires (SDA/SCL)
- Keep motor wires away from sensor wires
- Use proper gauge wire for ESC power connections

---

**Print this page for easy reference during assembly!**
