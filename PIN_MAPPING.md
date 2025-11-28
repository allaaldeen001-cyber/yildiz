# 📍 DRONE SYSTEM - CORRECTED PIN MAPPING

## ⚠️ ORIGINAL SPEC HAD CONFLICTS - CORRECTED BELOW

---

## 🛩 FLIGHT CONTROLLER (FC) - Arduino Nano

### Communication
- **NRF24L01**
  - CE → D2 ⚠️ (Changed from D4 due to conflict)
  - CSN → D10
  - MOSI → D11 (SPI)
  - MISO → D12 (SPI)
  - SCK → D13 (SPI)

### Sensors
- **MPU6050** (I2C)
  - SDA → A4
  - SCL → A5

- **MS5611** (I2C)
  - SDA → A4
  - SCL → A5

### Motor ESC Outputs (PWM)
- D3 → Front Left (FL)
- D5 → Front Right (FR)
- D6 → Rear Right (RR)
- D9 → Rear Left (RL)

### Inputs
- **A6** → Calibration Button (MPU + MS5611) ⚠️ (Changed from D4)
- **A7** → Smooth Motor-Start Button ⚠️ (Changed from D5)
- **D4** → Arm/Disarm Switch ⚠️ (Changed from D3)
- **D7** → Altitude-Hold Switch ⚠️ (Changed from D2)

### Outputs
- **D8** → Buzzer
- **A3** → Status LED ⚠️ (Changed from D7 - using analog pin as digital)

### Battery Monitor
- **A0** → Voltage divider input

---

## 🎮 REMOTE CONTROLLER (RC) - Arduino Nano

### Communication
- **NRF24L01**
  - CE → D9
  - CSN → D10
  - MOSI → D11 (SPI)
  - MISO → D12 (SPI)
  - SCK → D13 (SPI)

### Joysticks (Analog Inputs)
- **A0** → Throttle (Left stick - Up/Down)
- **A1** → Yaw (Left stick - Left/Right)
- **A2** → Pitch (Right stick - Up/Down)
- **A3** → Roll (Right stick - Left/Right)

### Status Output
- **D8** → Status LED (optional - shows link status)

---

## 🔄 CHANGES FROM ORIGINAL SPEC

| Function | Original Pin | New Pin | Reason |
|----------|-------------|---------|--------|
| NRF CE (FC) | D4 | D2 | D4 needed for button |
| Calibration Button | D4 | A6 | Avoid conflict with motors |
| Smooth Start Button | D5 | A7 | D5 used by motor FR |
| Arm/Disarm Switch | D3 | D4 | D3 used by motor FL |
| Altitude-Hold Switch | D2 | D7 | D2 needed for NRF CE |
| Status LED | D7 | A3 | D7 reassigned to switch |

---

## ✅ ADVANTAGES OF NEW MAPPING

1. ✅ No pin conflicts
2. ✅ All PWM pins available for motors
3. ✅ SPI pins reserved for NRF
4. ✅ I2C pins reserved for sensors
5. ✅ Analog pins A6/A7 used for buttons (digital input works fine)
6. ✅ Serial pins D0/D1 left free for debugging

---

## 🔌 WIRING CHECKLIST

### FC Board
- [ ] Motors connected to D3, D5, D6, D9
- [ ] NRF24 connected to D2(CE), D10(CSN), D11-13(SPI)
- [ ] MPU6050 + MS5611 on A4(SDA), A5(SCL)
- [ ] Calibration button to A6 → GND
- [ ] Smooth-start button to A7 → GND
- [ ] Arm switch to D4 (LOW=Armed)
- [ ] Alt-hold switch to D7 (LOW=Active)
- [ ] Buzzer to D8
- [ ] LED to A3
- [ ] Battery voltage divider to A0

### RC Board
- [ ] NRF24 connected to D9(CE), D10(CSN), D11-13(SPI)
- [ ] Throttle joystick to A0
- [ ] Yaw joystick to A1
- [ ] Pitch joystick to A2
- [ ] Roll joystick to A3
- [ ] Optional LED to D8
