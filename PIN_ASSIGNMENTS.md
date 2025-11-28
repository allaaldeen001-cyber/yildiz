# Pin Assignments Reference

## Flight Controller (FC) - Arduino Nano

### Radio Communication
- **CE**: D4
- **CSN**: D10
- **MOSI**: D11 (SPI)
- **MISO**: D12 (SPI)
- **SCK**: D13 (SPI)

### I2C Bus (MPU6050 & MS5611)
- **SDA**: A4
- **SCL**: A5
- **VCC**: 5V
- **GND**: GND

### Motor Control (ESC)
- **Front Left (FL)**: D3
- **Front Right (FR)**: D5
- **Rear Right (RR)**: D6
- **Rear Left (RL)**: D9

### Inputs
- **Calibration Button**: A1 (local FC button, using analog pin as digital)
- **Smooth Motor Start Button**: A2 (local FC button, using analog pin as digital)
- **Note**: Arm/Disarm and Altitude Hold switches come from RC via radio
- **Note**: D4 is used for radio CE pin, D5 is used for Front Right motor ESC, so analog pins are used for buttons

### Outputs
- **Buzzer**: D8
- **Status LED**: D7

### Analog Inputs
- **Battery Voltage Monitor**: A0 (optional)

---

## Remote Controller (RC) - Arduino Nano

### Radio Communication
- **CE**: D9
- **CSN**: D10
- **MOSI**: D11 (SPI)
- **MISO**: D12 (SPI)
- **SCK**: D13 (SPI)

### Joysticks (Analog)
- **A0**: Throttle (Left joystick Up/Down)
- **A1**: Yaw (Left joystick Left/Right)
- **A2**: Pitch (Right joystick Up/Down)
- **A3**: Roll (Right joystick Left/Right)

### Inputs (Digital with Pull-up)
- **D4**: Button 1
- **D5**: Button 2 (Arming)
- **D3**: Switch 1 (Arm/Disarm: HIGH=Disarmed, LOW=Armed)
- **D2**: Switch 2 (Altitude Hold)

---

## Important Notes

1. **Pin Conflict Resolution**: D3 on FC is used for Front Left motor ESC, not for a switch. Switches are on RC and transmitted via radio.

2. **I2C Addresses**:
   - MPU6050: 0x68 (default)
   - MS5611: 0x77 (as configured in code)

3. **Radio Pipe**: Both FC and RC use the same pipe address: `0xF0F0F0F0E1LL`

4. **Power Requirements**:
   - FC: Requires stable 5V power supply (can use USB or external regulator)
   - RC: Can be powered via USB or battery
   - ESCs: Require separate battery power (typically 3S-4S LiPo)

5. **Servo Library**: Uses standard Servo library with pulse width range 1000-2000 microseconds
