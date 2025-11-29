# Wiring Guide - Professional Drone System

## 📌 Flight Controller Wiring

### Arduino Nano Pin Connections

#### Power
- **VIN**: 5V from BEC/ESC
- **GND**: Common ground for all components
- **5V**: Power for sensors and NRF24L01

#### NRF24L01 PA+LNA Module
| NRF24L01 Pin | Arduino Nano Pin | Description |
|--------------|------------------|-------------|
| VCC | 3.3V | **IMPORTANT: Use 3.3V!** |
| GND | GND | Ground |
| CE | D4 | Chip Enable |
| CSN | D10 | Chip Select (SPI) |
| SCK | D13 | SPI Clock |
| MOSI | D11 | SPI MOSI |
| MISO | D12 | SPI MISO |

**⚠️ WARNING**: NRF24L01 requires 3.3V! Using 5V will damage the module. Use a 3.3V regulator or the Arduino's 3.3V pin (max 50mA).

#### MPU6050 (6-Axis IMU)
| MPU6050 Pin | Arduino Nano Pin | Description |
|-------------|------------------|-------------|
| VCC | 5V | Power (can use 3.3V or 5V) |
| GND | GND | Ground |
| SDA | A4 | I2C Data |
| SCL | A5 | I2C Clock |
| INT | D2 | Interrupt (optional) |
| XDA | - | Not used |
| XCL | - | Not used |
| AD0 | GND | I2C Address select (0x68) |

#### MS5611 (Barometric Pressure Sensor)
| MS5611 Pin | Arduino Nano Pin | Description |
|------------|------------------|-------------|
| VCC | 3.3V | Power |
| GND | GND | Ground |
| SDA | A4 | I2C Data (shared with MPU6050) |
| SCL | A5 | I2C Clock (shared with MPU6050) |

**Note**: MPU6050 and MS5611 share the same I2C bus (A4/A5).

#### Buzzer (Active or Passive)
| Buzzer Pin | Arduino Nano Pin | Description |
|------------|------------------|-------------|
| + (Signal) | D8 | PWM control |
| - (GND) | GND | Ground |

**Note**: For passive buzzer, use PWM. For active buzzer, simple HIGH/LOW works.

#### Status LED
| LED Pin | Arduino Nano Pin | Description |
|---------|------------------|-------------|
| Anode (+) | D7 | Through 220Ω resistor |
| Cathode (-) | GND | Ground |

#### ESC/Motor Connections
| Motor Position | ESC Signal Pin | Arduino Nano Pin | Motor Direction |
|----------------|----------------|------------------|-----------------|
| Front Left (FL) | Signal (white/yellow) | D3 | CCW ↺ |
| Front Right (FR) | Signal (white/yellow) | D5 | CW ↻ |
| Rear Right (RR) | Signal (white/yellow) | D6 | CCW ↺ |
| Rear Left (RL) | Signal (white/yellow) | D9 | CW ↻ |

**ESC Power Distribution:**
- ESC Red (+): Connect to battery via PDB (Power Distribution Board)
- ESC Black (-): Connect to battery ground via PDB
- ESC BEC (5V): One ESC's BEC to Arduino VIN (remove others to avoid conflict)

---

## 📌 Remote Controller Wiring

### Arduino Nano Pin Connections

#### Power
- **VIN**: 7-12V from battery (9V recommended)
- **GND**: Common ground
- **5V**: Power for joysticks and NRF24L01

#### NRF24L01 PA+LNA Module
| NRF24L01 Pin | Arduino Nano Pin | Description |
|--------------|------------------|-------------|
| VCC | 3.3V | **IMPORTANT: Use 3.3V!** |
| GND | GND | Ground |
| CE | D9 | Chip Enable |
| CSN | D10 | Chip Select (SPI) |
| SCK | D13 | SPI Clock |
| MOSI | D11 | SPI MOSI |
| MISO | D12 | SPI MISO |

#### Left Joystick (Throttle + Yaw)
| Joystick Pin | Arduino Nano Pin | Function |
|--------------|------------------|----------|
| VCC | 5V | Power |
| GND | GND | Ground |
| VRx (Vertical) | A0 | Throttle (Up/Down) |
| VRy (Horizontal) | A1 | Yaw (Left/Right) |
| SW | - | Not used (or add for extra function) |

#### Right Joystick (Pitch + Roll)
| Joystick Pin | Arduino Nano Pin | Function |
|--------------|------------------|----------|
| VCC | 5V | Power |
| GND | GND | Ground |
| VRx (Vertical) | A2 | Pitch (Forward/Back) |
| VRy (Horizontal) | A3 | Roll (Left/Right) |
| SW | - | Not used (or add for extra function) |

#### Push Buttons
| Button | Pin 1 | Pin 2 | Function |
|--------|-------|-------|----------|
| Button 1 | D4 | GND | Gyro Calibration |
| Button 2 | D5 | GND | ESC Calibration |

**Note**: Buttons use internal pull-up resistors. Press = LOW, Release = HIGH.

#### Toggle Switches (SPDT or SPST)
| Switch | Pin 1 | Pin 2 | Pin 3 | Function |
|--------|-------|-------|-------|----------|
| Switch 1 | D2 | GND | - | Altitude Hold |
| Switch 2 | D3 | GND | - | Arm/Disarm Kill Switch |

**Wiring for SPDT Toggle Switch:**
- Common (middle pin) → Arduino Pin (D2 or D3)
- Position 1 → GND
- Position 2 → No connection (floating = HIGH via pull-up)

**Wiring for SPST Toggle Switch:**
- Pin 1 → Arduino Pin (D2 or D3)
- Pin 2 → GND
- When ON: Pin reads LOW
- When OFF: Pin reads HIGH (pull-up resistor)

---

## 🔌 Power Supply Recommendations

### Flight Controller
- **Main Battery**: 3S LiPo (11.1V) or 4S LiPo (14.8V)
- **Arduino Power**: 5V from ESC BEC (most ESCs have built-in 5V BEC)
- **Sensors**: 3.3V-5V (check datasheets)
- **NRF24L01**: 3.3V (max 50mA) - use separate 3.3V regulator if possible

**Recommended Setup:**
1. Use a dedicated 5V BEC (3A+) for Arduino and sensors
2. Use a separate 3.3V regulator (AMS1117-3.3) for NRF24L01
3. Add 10µF and 100nF capacitors near NRF24L01 VCC for stability

### Remote Controller
- **Battery**: 9V alkaline or 2S LiPo (7.4V) with voltage regulator
- **NRF24L01**: 3.3V from Arduino (add 100µF capacitor)
- **Joysticks**: 5V from Arduino
- **Total Current**: ~150mA average

---

## 🛠️ Assembly Tips

### Flight Controller
1. **Mount Arduino Nano** on the drone frame (center of gravity)
2. **Mount MPU6050** on vibration-dampening foam (reduce noise)
3. **Mount NRF24L01** away from ESCs and motors (RF interference)
4. **Use short wires** for I2C connections (max 20cm recommended)
5. **Add capacitors** (100nF ceramic + 10µF electrolytic) near each IC's VCC
6. **Twist I2C wires** together to reduce electromagnetic interference
7. **Calibrate MPU6050** on a perfectly level surface

### Remote Controller
1. **Secure joysticks** firmly to prevent movement during use
2. **Label all switches** clearly for easy identification
3. **Add capacitor** (100µF) across NRF24L01 power pins
4. **Use a case** to protect components
5. **Add external antenna** to NRF24L01 PA+LNA for better range

---

## 📐 Motor Configuration (X-Frame)

```
     FRONT
      ↑
  FL     FR
   ↺     ↻
    \   /
     \ /
      X
     / \
    /   \
   ↻     ↺
  RL     RR

↺ = Counter-Clockwise (CCW)
↻ = Clockwise (CW)
```

**Important**: 
- FL and RR motors rotate CCW (↺)
- FR and RL motors rotate CW (↻)
- Use correct propellers for each motor direction!
- CW motors use CW propellers (standard thread)
- CCW motors use CCW propellers (reverse thread)

---

## ⚡ Current Consumption Estimates

### Flight Controller (per component)
| Component | Current Draw | Notes |
|-----------|--------------|-------|
| Arduino Nano | 15-20mA | Idle |
| NRF24L01 PA+LNA | 115mA | During TX |
| MPU6050 | 3-5mA | Active mode |
| MS5611 | 1-2mA | Normal operation |
| Buzzer | 20-30mA | When active |
| LED | 15-20mA | When lit |
| **Total (idle)** | ~40mA | Without NRF TX |
| **Total (active)** | ~175mA | With NRF TX |

### Remote Controller
| Component | Current Draw | Notes |
|-----------|--------------|-------|
| Arduino Nano | 15-20mA | Idle |
| NRF24L01 PA+LNA | 115mA | During TX |
| Joysticks (x2) | 2mA | Per joystick |
| **Total** | ~135mA | Average |

---

## 🔧 Troubleshooting Common Wiring Issues

### NRF24L01 Not Working
- ✓ Check 3.3V power supply (NOT 5V!)
- ✓ Add 100µF capacitor across VCC and GND
- ✓ Use short wires (< 10cm for SPI connections)
- ✓ Check SPI connections (MOSI, MISO, SCK, CE, CSN)
- ✓ Try different NRF24L01 module (they can be faulty)

### MPU6050 Not Responding
- ✓ Check I2C connections (SDA = A4, SCL = A5)
- ✓ Verify I2C address (0x68 or 0x69 depending on AD0 pin)
- ✓ Add 4.7kΩ pull-up resistors on SDA and SCL (if long wires)
- ✓ Test with I2C scanner sketch

### Motors Not Spinning
- ✓ Check ESC signal wire connections
- ✓ Verify ESC calibration completed
- ✓ Ensure battery is connected and charged
- ✓ Confirm arming sequence completed
- ✓ Check throttle is above minimum (1000µs)

### Erratic Motor Behavior
- ✓ Add capacitors near NRF24L01
- ✓ Separate BEC for Arduino (not shared with ESCs)
- ✓ Shield/twist motor wires to reduce EMI
- ✓ Move NRF24L01 away from ESCs

---

## 📷 Visual Wiring Diagrams

See `WIRING_DIAGRAM_FC.png` and `WIRING_DIAGRAM_RC.png` in the `/docs` folder for detailed visual guides.

---

**Safety First**: Always disconnect the battery before making wiring changes!
