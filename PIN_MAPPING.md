# Pin Mapping Reference

## Flight Controller (FC) Pin Assignments

### Radio Communication
- **NRF24L01 CE** → D4
- **NRF24L01 CSN** → D10
- **NRF24L01 MOSI** → D11 (SPI)
- **NRF24L01 MISO** → D12 (SPI)
- **NRF24L01 SCK** → D13 (SPI)

### Motor Control (ESC)
- **Front Left (FL)** → D3
- **Front Right (FR)** → D5
- **Rear Right (RR)** → D6
- **Rear Left (RL)** → D9

### Sensors (I2C)
- **MPU6050 SDA** → A4
- **MPU6050 SCL** → A5
- **MS5611 SDA** → A4 (shared with MPU6050)
- **MS5611 SCL** → A5 (shared with MPU6050)

### Physical Controls
- **Calibration Button** → A0 (or D11 if you prefer digital pin)
- **Smooth Start Button** → A1 (or D12 if you prefer digital pin)
- **Arm/Disarm Switch** → A2 (or D2 if you prefer digital pin)
- **Altitude Hold Switch** → A3 (or D2 if you prefer digital pin)

**Note**: Analog pins (A0-A7) can be used as digital inputs on Arduino Nano. If you prefer to use digital pins, you can modify the pin assignments in the code. However, avoid D0, D1 (Serial), D4 (NRF24L01 CE), D10 (NRF24L01 CSN), D11-D13 (SPI), and D3, D5, D6, D9 (ESC pins).

### Outputs
- **Buzzer** → D8
- **Status LED** → D7

### Optional
- **Battery Voltage Monitor** → A0 (if not using for calibration button)

## Remote Controller (RC) Pin Assignments

### Radio Communication
- **NRF24L01 CE** → D9
- **NRF24L01 CSN** → D10
- **NRF24L01 MOSI** → D11 (SPI)
- **NRF24L01 MISO** → D12 (SPI)
- **NRF24L01 SCK** → D13 (SPI)

### Joysticks
- **Throttle (Left Up/Down)** → A0
- **Yaw (Left Left/Right)** → A1
- **Pitch (Right Up/Down)** → A2
- **Roll (Right Left/Right)** → A3

## Alternative Pin Assignments (if conflicts occur)

If you need to use different pins due to hardware constraints, here are alternatives:

### FC Physical Controls (Alternative Options):
- Calibration Button: D11, D12, or any unused digital pin
- Smooth Start Button: D11, D12, or any unused digital pin
- Arm/Disarm Switch: D2, D11, D12, or any unused digital pin
- Altitude Hold Switch: D2, D11, D12, or any unused digital pin

**Important**: If you change pin assignments, update the constants in `Drone_Flight_control.ino`:
```cpp
const int CALIBRATION_BUTTON = A0;      // Change this
const int SMOOTH_START_BUTTON = A1;     // Change this
const int ARM_SWITCH = A2;               // Change this
const int ALTITUDE_HOLD_SWITCH = A3;    // Change this
```

## Pin Conflict Resolution

The code uses analog pins (A0-A3) for physical controls to avoid conflicts with:
- D4: NRF24L01 CE (required)
- D3, D5, D6, D9: ESC motor outputs (required)
- D10: NRF24L01 CSN (required)
- D11-D13: SPI communication (required)

Analog pins work perfectly as digital inputs with `INPUT_PULLUP` mode.
