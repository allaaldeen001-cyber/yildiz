# Drone Flight Control System - Wiring Diagram

## Flight Controller (FC) - Arduino Nano

### NRF24L01 Wireless Module
```
NRF24L01 Pin    →    Arduino Nano Pin
─────────────────────────────────────
VCC             →    3.3V
GND             →    GND
CE              →    D4
CSN             →    D10
SCK             →    D13
MOSI            →    D11
MISO            →    D12
```
**Important:** Add a 10µF capacitor between VCC and GND of NRF24L01 for stable operation.

### MPU6050 Gyroscope/Accelerometer (I2C)
```
MPU6050 Pin     →    Arduino Nano Pin
─────────────────────────────────────
VCC             →    5V
GND             →    GND
SCL             →    A5
SDA             →    A4
INT             →    (not used)
```

### MS5611 Barometer (I2C)
```
MS5611 Pin      →    Arduino Nano Pin
─────────────────────────────────────
VCC             →    3.3V or 5V
GND             →    GND
SCL             →    A5 (shared with MPU6050)
SDA             →    A4 (shared with MPU6050)
```

### ESC Connections (Electronic Speed Controllers)
```
Motor Position     ESC Signal Wire    →    Arduino Nano Pin
──────────────────────────────────────────────────────────
Front Left (FL)    Signal             →    D3
Front Right (FR)   Signal             →    D5
Rear Right (RR)    Signal             →    D6
Rear Left (RL)     Signal             →    D9

All ESCs:
- RED wire (5V)    → Not connected to Arduino (power from BEC)
- BLACK wire (GND) → GND (shared ground with Arduino)
```

### Audio/Visual Feedback
```
Component       →    Arduino Nano Pin
─────────────────────────────────────
Buzzer (+)      →    D8
Buzzer (-)      →    GND
LED Anode (+)   →    D7 (through 220Ω resistor)
LED Cathode (-) →    GND
```

### Power Supply
```
- ESCs powered by main LiPo battery (3S or 4S)
- Arduino Nano powered via USB or 5V BEC from one ESC
- Ensure common ground between all components
```

---

## RC Controller - Arduino Nano

### NRF24L01 Wireless Module
```
NRF24L01 Pin    →    Arduino Nano Pin
─────────────────────────────────────
VCC             →    3.3V
GND             →    GND
CE              →    D9
CSN             →    D10
SCK             →    D13
MOSI            →    D11
MISO            →    D12
```
**Important:** Add a 10µF capacitor between VCC and GND of NRF24L01.

### Left Joystick (Throttle/Yaw)
```
Joystick Pin    →    Arduino Nano Pin    →    Function
───────────────────────────────────────────────────────
VCC             →    5V
GND             →    GND
VRx             →    A0                  →    Throttle (Up/Down)
VRy             →    A1                  →    Yaw (Left/Right)
SW              →    (not used)
```

### Right Joystick (Pitch/Roll)
```
Joystick Pin    →    Arduino Nano Pin    →    Function
───────────────────────────────────────────────────────
VCC             →    5V
GND             →    GND
VRx             →    A3                  →    Roll (Left/Right)
VRy             →    A2                  →    Pitch (Up/Down)
SW              →    (not used)
```

### Buttons
```
Button          Arduino Pin    Function
─────────────────────────────────────────
Button 1        D4             Calibration (active LOW)
Button 2        D5             Smooth Motor Start (active LOW)

Wiring: One side to Arduino pin, other side to GND
(INPUT_PULLUP mode used in code)
```

### Switches
```
Switch          Arduino Pin    Function                      States
─────────────────────────────────────────────────────────────────────
Switch 1        D3             Arm/Disarm                    1=Disarmed, 0=Armed
Switch 2        D2             Altitude Hold                 1=Off, 0=On

Wiring: Common terminal to GND, NO (Normally Open) to Arduino pin
```

### Status LED
```
Component       →    Arduino Nano Pin
─────────────────────────────────────
LED Anode (+)   →    D6 (through 220Ω resistor)
LED Cathode (-) →    GND
```

### Power Supply
```
- Power from 9V battery or USB
- Keep powered on at all times during flight
```

---

## Complete System Wiring Notes

### Ground Rules
1. **Common Ground:** All components must share a common ground
2. **Power Distribution:** 
   - Flight battery powers ESCs
   - ESCs provide 5V BEC for Arduino Nano (FC)
   - RC controller uses separate battery/USB
   - NRF24L01 modules must use 3.3V (not 5V!)

### I2C Bus (FC only)
- Both MPU6050 and MS5611 share the I2C bus (A4/A5)
- No pull-up resistors needed (already on modules)
- Default addresses: MPU6050 = 0x68, MS5611 = 0x77

### Signal Wiring Best Practices
1. Keep NRF24L01 wires as short as possible
2. Twist ESC signal wires to reduce EMI
3. Keep sensor wires away from motor wires
4. Use shielded cable for long runs

### Capacitor Placement
- 10µF on NRF24L01 VCC/GND (both FC and RC)
- 1000µF on main battery input (optional but recommended)
- 100nF ceramic caps near each IC VCC pin (optional)

---

## Motor Layout (Quadcopter X Configuration)

```
        FRONT
         ↑
    
    [FL]   [FR]
      ⟲     ⟳
       \   /
        \ /
         X
        / \
       /   \
      ⟳     ⟲
    [RL]   [RR]

Legend:
⟳ = Clockwise rotation
⟲ = Counter-clockwise rotation
```

**Motor Rotation Direction:**
- Front Left (FL) → CCW (Counter-Clockwise)
- Front Right (FR) → CW (Clockwise)
- Rear Left (RL) → CW (Clockwise)
- Rear Right (RR) → CCW (Counter-Clockwise)

**Verify motor rotation before first flight!**
