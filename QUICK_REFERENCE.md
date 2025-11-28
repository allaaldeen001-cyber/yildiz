# Quick Reference Card

## Flight Controller (FC) - Arduino Nano

### Pin Connections
```
Radio (NRF24L01):
  CE → D4, CSN → D10

Motors (ESC):
  FL → D3, FR → D5, RR → D6, RL → D9

I2C (MPU6050 + MS5611):
  SDA → A4, SCL → A5

Buttons:
  Calibration → A1
  Motor Start → A2

Outputs:
  Buzzer → D8
  LED → D7
```

## Remote Controller (RC) - Arduino Nano

### Pin Connections
```
Radio (NRF24L01):
  CE → D9, CSN → D10

Joysticks:
  Throttle → A0 (Left Up/Down)
  Yaw → A1 (Left Left/Right)
  Pitch → A2 (Right Up/Down)
  Roll → A3 (Right Left/Right)

Buttons/Switches:
  Button 1 → D4
  Button 2 (Arm) → D5
  Switch 1 (Arm/Disarm) → D3
  Switch 2 (Altitude Hold) → D2
```

## Startup Sequence

1. **Power ON**: RC first, then FC
2. **Wait**: Buzzer beeps twice = link confirmed
3. **Calibrate**: 
   - RC Switch 1 = Disarmed (LED ON)
   - FC Button A1 (hold 2 sec)
   - Wait for beeps
4. **Arm**:
   - RC Switch 1 = Armed
   - RC Button 2 (hold 2 sec)
   - LED blinks = armed
5. **Motor Start**:
   - FC Button A2
   - Motors ramp up smoothly
6. **Fly**: Use joysticks

## LED Status

- **ON solid** = Disarmed
- **Blinking** = Armed + receiving signal
- **OFF** = No signal or error

## Safety Limits

- Max tilt: 30°
- Max thrust: 1700
- No data timeout: 3 seconds

## Emergency Stop

- Set RC Switch 1 to Disarmed
- Or move throttle to minimum
- Motors stop immediately
