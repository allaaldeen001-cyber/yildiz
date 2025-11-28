# Pinout Reference Guide

## Flight Controller (FC) - Arduino Nano

### Digital Pins
| Pin | Function | Notes |
|-----|----------|-------|
| D0 | RX | Serial communication (avoid if using Serial) |
| D1 | TX | Serial communication (avoid if using Serial) |
| D2 | Altitude Hold Switch | INPUT_PULLUP |
| D3 | Motor FL (Front Left ESC) | PWM output |
| D4 | NRF24L01 CE | Radio chip enable |
| D5 | Motor FR (Front Right ESC) | PWM output |
| D6 | Motor RR (Rear Right ESC) | PWM output |
| D7 | Status LED | OUTPUT |
| D8 | Buzzer | OUTPUT |
| D9 | Motor RL (Rear Left ESC) | PWM output |
| D10 | NRF24L01 CSN | Radio chip select |
| D11 | NRF24L01 MOSI | SPI communication |
| D12 | NRF24L01 MISO | SPI communication |
| D13 | NRF24L01 SCK | SPI communication |

### Analog Pins (Used as Digital)
| Pin | Function | Notes |
|-----|----------|-------|
| A0 | Battery Voltage Monitor | Analog input (optional) |
| A1 | Calibration Button | INPUT_PULLUP (digital read) |
| A2 | Smooth Start Button | INPUT_PULLUP (digital read) |
| A3 | Arm/Disarm Switch | INPUT_PULLUP (digital read) |
| A4 | I2C SDA | MPU6050 & MS5611 |
| A5 | I2C SCL | MPU6050 & MS5611 |

### Power
| Pin | Function |
|-----|----------|
| 5V | Power for sensors, LED |
| 3.3V | Power for NRF24L01 (critical!) |
| GND | Common ground |

## Remote Controller (RC) - Arduino Nano

### Digital Pins
| Pin | Function | Notes |
|-----|----------|-------|
| D2 | Switch 2 | INPUT_PULLUP (optional) |
| D3 | Switch 1 | INPUT_PULLUP (optional) |
| D4 | Button 1 | INPUT_PULLUP (optional) |
| D5 | Button 2 | INPUT_PULLUP (optional) |
| D9 | NRF24L01 CE | Radio chip enable |
| D10 | NRF24L01 CSN | Radio chip select |
| D11 | NRF24L01 MOSI | SPI communication |
| D12 | NRF24L01 MISO | SPI communication |
| D13 | NRF24L01 SCK | SPI communication |

### Analog Pins (Joysticks)
| Pin | Function | Notes |
|-----|----------|-------|
| A0 | Throttle | Left stick Y-axis (Up/Down) |
| A1 | Yaw | Left stick X-axis (Left/Right) |
| A2 | Pitch | Right stick Y-axis (Up/Down) |
| A3 | Roll | Right stick X-axis (Left/Right) |

### Power
| Pin | Function |
|-----|----------|
| 5V | Power for joysticks |
| GND | Common ground |

## Pin Conflict Resolution

**Original Requirements vs. Actual Implementation:**

The original requirements specified:
- D4 = Calibration Button
- D5 = Smooth Motor-Start Button  
- D3 = Arm/Disarm Switch

However, these pins conflict with:
- D4 = NRF24L01 CE (required for radio)
- D5 = Motor FR (required for motor control)
- D3 = Motor FL (required for motor control)

**Solution:** Use analog pins as digital inputs:
- A1 = Calibration Button (instead of D4)
- A2 = Smooth Start Button (instead of D5)
- A3 = Arm/Disarm Switch (instead of D3)

Analog pins A1-A3 can be used as digital inputs with `digitalRead()` and `pinMode(..., INPUT_PULLUP)`.

## Wiring Notes

### NRF24L01 Power Warning
⚠️ **CRITICAL**: NRF24L01 modules MUST be powered with **3.3V**, not 5V! Using 5V will damage the module.

### I2C Pull-up Resistors
MPU6050 and MS5611 modules typically include pull-up resistors. If you're using bare sensors, add 4.7kΩ pull-up resistors on SDA and SCL lines.

### ESC Power
ESCs should be powered from the main battery, not from Arduino. The Arduino only provides the control signal (PWM).

### Button/Switch Wiring
All buttons and switches use INPUT_PULLUP mode, so:
- Connect one terminal to the pin
- Connect other terminal to GND
- When button/switch is pressed/closed, pin reads LOW
- When button/switch is open, pin reads HIGH (due to internal pull-up)

## Pin Usage Summary

### FC - Fully Used Pins
- D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13
- A1, A2, A3, A4, A5

### FC - Available Pins
- D0, D1 (if not using Serial)
- A0 (can be used for battery monitoring)

### RC - Fully Used Pins
- D9, D10, D11, D12, D13
- A0, A1, A2, A3

### RC - Available Pins
- D0-D8 (except D9-D13)
- A4-A7 (if needed)
