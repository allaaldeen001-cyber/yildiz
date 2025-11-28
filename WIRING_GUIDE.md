# Quadcopter Wiring Guide

## Flight Controller Board Wiring

### Arduino Nano Connections

#### Power
- **VIN**: Connect to 5V power supply (via voltage regulator if using battery)
- **GND**: Common ground for all components

#### NRF24L01 Module
- **VCC**: 3.3V (use voltage regulator, NRF24L01 is 3.3V only!)
- **GND**: GND
- **CE**: Digital Pin 7
- **CSN**: Digital Pin 8
- **SCK**: Digital Pin 13 (SPI)
- **MOSI**: Digital Pin 11 (SPI)
- **MISO**: Digital Pin 12 (SPI)

#### MPU6050 (IMU Sensor)
- **VCC**: 5V
- **GND**: GND
- **SDA**: A4 (I2C)
- **SCL**: A5 (I2C)

#### MS5611 (Barometric Pressure Sensor)
- **VCC**: 5V
- **GND**: GND
- **SDA**: A4 (I2C - shared with MPU6050)
- **SCL**: A5 (I2C - shared with MPU6050)

#### ESCs (Electronic Speed Controllers)
- **ESC1 Signal**: Digital Pin 5
- **ESC2 Signal**: Digital Pin 6
- **ESC3 Signal**: Digital Pin 9
- **ESC4 Signal**: Digital Pin 10
- **ESC Power**: Connect to battery (usually 2S-4S LiPo)
- **ESC Ground**: Common ground

#### Buzzer
- **Positive**: Digital Pin 2
- **Negative**: GND

#### Status LED
- **Anode**: Digital Pin 3 (via 220Ω resistor)
- **Cathode**: GND

---

## Remote Control Board Wiring

### Arduino Nano Connections

#### Power
- **VIN**: Connect to 5V power supply (battery pack recommended)
- **GND**: Common ground

#### NRF24L01 Module
- **VCC**: 3.3V (use voltage regulator!)
- **GND**: GND
- **CE**: Digital Pin 7
- **CSN**: Digital Pin 8
- **SCK**: Digital Pin 13 (SPI)
- **MOSI**: Digital Pin 11 (SPI)
- **MISO**: Digital Pin 12 (SPI)

#### Left Joystick
- **VCC**: 5V
- **GND**: GND
- **VRx (X-axis)**: Analog Pin A0
- **VRy (Y-axis)**: Analog Pin A1

#### Right Joystick
- **VCC**: 5V
- **GND**: GND
- **VRx (X-axis)**: Analog Pin A2
- **VRy (Y-axis)**: Analog Pin A3

#### Toggle Switch
- **One terminal**: Digital Pin 3
- **Other terminal**: GND
- **Center terminal**: 5V (via pull-up, or use internal pull-up)

#### Button 1 (Calibration)
- **One terminal**: Digital Pin 4
- **Other terminal**: GND
- (Use internal pull-up resistor)

#### Button 2 (Motor Start)
- **One terminal**: Digital Pin 5
- **Other terminal**: GND
- (Use internal pull-up resistor)

#### Status LED
- **Anode**: Digital Pin 2 (via 220Ω resistor)
- **Cathode**: GND

---

## Motor Configuration (Quadcopter X Layout)

```
        Front
    M1      M2
     \      /
      \    /
       \  /
        \/
       /\
      /  \
     /    \
    M4      M3
        Back
```

- **M1**: Front Left (ESC1)
- **M2**: Front Right (ESC2)
- **M3**: Back Right (ESC3)
- **M4**: Back Left (ESC4)

### Motor Rotation (when viewed from top):
- **M1 & M3**: Clockwise
- **M2 & M4**: Counter-clockwise

---

## Important Notes

1. **Power Supply**: 
   - Flight Controller needs stable 5V for Arduino Nano
   - ESCs typically need 2S-4S LiPo battery (7.4V-14.8V)
   - Use separate BEC (Battery Eliminator Circuit) or voltage regulator for Arduino

2. **NRF24L01 Voltage**: 
   - **CRITICAL**: NRF24L01 modules are 3.3V only!
   - Use a 3.3V voltage regulator (e.g., AMS1117-3.3)
   - Do NOT connect directly to 5V - it will damage the module

3. **I2C Pull-up Resistors**: 
   - MPU6050 and MS5611 may need 4.7kΩ pull-up resistors on SDA and SCL
   - Some modules have these built-in

4. **ESC Calibration**: 
   - ESCs must be calibrated before first use
   - Follow the calibration procedure in the code

5. **Safety**: 
   - Always start with propellers removed
   - Test all systems before attaching propellers
   - Use kill switch during initial testing

---

## Required Arduino Libraries

Install these libraries via Arduino IDE Library Manager:

1. **RF24** by TMRh20
   - Search: "RF24"
   - Install: "RF24" by TMRh20

2. **Adafruit MPU6050**
   - Search: "Adafruit MPU6050"
   - Install: "Adafruit MPU6050" by Adafruit

3. **Adafruit Unified Sensor**
   - Search: "Adafruit Unified Sensor"
   - Install: "Adafruit Unified Sensor" by Adafruit

4. **MS5611**
   - Search: "MS5611"
   - Install: "MS5611" (check compatibility)

5. **Servo** (Built-in Arduino library)

---

## Troubleshooting

### NRF24L01 Not Connecting
- Check 3.3V power supply
- Verify SPI connections
- Ensure both modules use same channel (76)
- Check antenna connection

### MPU6050 Not Working
- Verify I2C connections (SDA/SCL)
- Check for pull-up resistors
- Verify 5V power supply

### ESCs Not Responding
- Check signal wire connections
- Verify ESC power supply
- Ensure ESCs are calibrated
- Check ESC programming (may need special calibration)

### Drone Not Stable
- Recalibrate IMU (keep drone level)
- Adjust PID values in code
- Check motor rotation directions
- Verify propeller installation
