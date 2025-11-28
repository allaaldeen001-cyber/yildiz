# Wiring Checklist

Use this checklist to verify all connections before powering on.

## Flight Controller Checklist

### Power Connections
- [ ] Arduino Nano VIN → Battery via ESC BEC or separate 5V regulator
- [ ] Arduino Nano GND → Common ground (battery negative)
- [ ] All ESCs connected to battery (red/black wires)
- [ ] ESC signal wires connected to Arduino pins

### ESC Connections
- [ ] ESC1 signal → Pin 3 (Motor 1 - Front Left)
- [ ] ESC2 signal → Pin 5 (Motor 2 - Front Right)
- [ ] ESC3 signal → Pin 6 (Motor 3 - Back Left)
- [ ] ESC4 signal → Pin 9 (Motor 4 - Back Right)
- [ ] All ESC grounds connected to Arduino GND

### NRF24L01 Module
- [ ] VCC → 3.3V (IMPORTANT: Use voltage regulator, not 5V!)
- [ ] GND → GND
- [ ] CE → Pin 7
- [ ] CSN → Pin 8
- [ ] SCK → Pin 13
- [ ] MOSI → Pin 11
- [ ] MISO → Pin 12
- [ ] Antenna connected

### MPU6050 Sensor
- [ ] VCC → 5V
- [ ] GND → GND
- [ ] SDA → A4 (I2C)
- [ ] SCL → A5 (I2C)

### MS5611 Sensor
- [ ] VCC → 5V
- [ ] GND → GND
- [ ] SDA → A4 (I2C - shared with MPU6050)
- [ ] SCL → A5 (I2C - shared with MPU6050)

### Output Devices
- [ ] Buzzer positive → Pin 10
- [ ] Buzzer negative → GND
- [ ] Status LED anode → Pin 11 (with 220Ω resistor)
- [ ] Status LED cathode → GND
- [ ] LED anode → Pin 12 (with 220Ω resistor)
- [ ] LED cathode → GND

## RC Controller Checklist

### Power
- [ ] Arduino Nano powered via USB or external 5V
- [ ] GND connected

### Joysticks
- [ ] Left Joystick VCC → 5V
- [ ] Left Joystick GND → GND
- [ ] Left Joystick X → A0
- [ ] Left Joystick Y → A1
- [ ] Right Joystick VCC → 5V
- [ ] Right Joystick GND → GND
- [ ] Right Joystick X → A2
- [ ] Right Joystick Y → A3

### Switches & Buttons
- [ ] Toggle Switch terminal 1 → Pin 2
- [ ] Toggle Switch terminal 2 → GND (with pull-up resistor or use INPUT_PULLUP)
- [ ] Button 1 terminal 1 → Pin 4
- [ ] Button 1 terminal 2 → GND (with pull-up resistor or use INPUT_PULLUP)
- [ ] Button 2 terminal 1 → Pin 5
- [ ] Button 2 terminal 2 → GND (with pull-up resistor or use INPUT_PULLUP)

### NRF24L01 Module
- [ ] VCC → 3.3V (IMPORTANT: Use voltage regulator!)
- [ ] GND → GND
- [ ] CE → Pin 7
- [ ] CSN → Pin 8
- [ ] SCK → Pin 13
- [ ] MOSI → Pin 11
- [ ] MISO → Pin 12
- [ ] Antenna connected

### Status LED
- [ ] Status LED anode → Pin 13 (with 220Ω resistor)
- [ ] Status LED cathode → GND

## Motor Wiring Checklist

### Motor Connections
- [ ] Motor 1 (Front Left) → ESC1
- [ ] Motor 2 (Front Right) → ESC2
- [ ] Motor 3 (Back Left) → ESC3
- [ ] Motor 4 (Back Right) → ESC4

### Motor Rotation Verification
- [ ] Motor 1 & 4 rotate clockwise
- [ ] Motor 2 & 3 rotate counter-clockwise
- [ ] If wrong, swap any two wires on that motor

### Propeller Installation
- [ ] Clockwise motors have clockwise propellers
- [ ] Counter-clockwise motors have counter-clockwise propellers
- [ ] All propellers securely fastened

## Power Safety Checklist

- [ ] Battery voltage appropriate for ESCs (check ESC specs)
- [ ] Battery capacity sufficient for flight time
- [ ] Battery connector secure
- [ ] Fuse or protection circuit installed (recommended)
- [ ] Power wires adequate gauge for current
- [ ] No short circuits
- [ ] Battery charged and balanced (if LiPo)

## Pre-Flight Verification

### Software
- [ ] Flight Controller code uploaded successfully
- [ ] RC Controller code uploaded successfully
- [ ] Serial Monitor open (115200 baud)
- [ ] No compilation errors

### Hardware
- [ ] All connections double-checked
- [ ] No loose wires
- [ ] Components secured to frame
- [ ] Antennas properly positioned
- [ ] Frame balanced and rigid

### Safety
- [ ] Test area clear of people and obstacles
- [ ] Kill switch easily accessible
- [ ] Emergency landing plan ready
- [ ] First aid kit nearby (recommended)

## Testing Sequence

1. **Power On Test**
   - [ ] Flight Controller powers on (LEDs light up)
   - [ ] RC Controller powers on (LED blinks)
   - [ ] Serial Monitor shows initialization messages

2. **Radio Test**
   - [ ] Serial Monitor shows "RC CONNECTED"
   - [ ] Buzzer beeps on connection
   - [ ] Status LED confirms connection

3. **Sensor Test**
   - [ ] MPU6050 initializes (check Serial Monitor)
   - [ ] MS5611 initializes (check Serial Monitor)
   - [ ] No sensor errors

4. **Calibration Test**
   - [ ] Button 1 press detected (LED blinks)
   - [ ] Calibration completes successfully
   - [ ] Serial Monitor confirms calibration

5. **Motor Test**
   - [ ] Toggle switch works (LED responds)
   - [ ] Button 2 starts motors smoothly
   - [ ] Motors spin at idle speed
   - [ ] No unusual sounds or vibrations

6. **Control Test**
   - [ ] Joystick movements detected in Serial Monitor
   - [ ] Throttle increases smoothly
   - [ ] All controls respond correctly

## Common Wiring Mistakes

❌ **NRF24L01 to 5V** - Will damage module! Must use 3.3V
❌ **ESC signal wires reversed** - Motors won't work
❌ **I2C sensors swapped** - Wrong readings
❌ **Missing pull-up resistors** - Buttons/switches won't work
❌ **Wrong motor rotation** - Drone will flip on takeoff
❌ **Loose connections** - Intermittent failures

## Voltage Requirements

- **Arduino Nano**: 7-12V (VIN) or 5V (USB)
- **NRF24L01**: 3.3V only (use regulator!)
- **MPU6050**: 3.3V or 5V (check module)
- **MS5611**: 3.3V or 5V (check module)
- **ESCs**: Battery voltage (check ESC specs)
- **Joysticks**: Usually 5V (check specs)

---

**Complete this checklist before first flight!**
