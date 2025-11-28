# Quick Setup Guide

## Step 1: Install Arduino IDE

Download and install the Arduino IDE from [arduino.cc](https://www.arduino.cc/en/software)

## Step 2: Install Required Libraries

Open Arduino IDE, go to **Sketch → Include Library → Manage Libraries**

Search and install:
- **RF24 by TMRh20** (for NRF24L01 communication)

The following libraries are built-in (no installation needed):
- Wire (I2C communication)
- SPI (NRF24L01)
- EEPROM (calibration storage)
- Servo (ESC control)

## Step 3: Upload Flight Controller Code

1. Open `flight_controller/flight_controller.ino` in Arduino IDE
2. Connect Arduino Nano to computer via USB
3. Select **Tools → Board → Arduino Nano**
4. Select **Tools → Processor → ATmega328P** (or "ATmega328P Old Bootloader" if upload fails)
5. Select the correct **Tools → Port**
6. Click **Upload** button

## Step 4: Upload RC Transmitter Code

1. Open `rc_transmitter/rc_transmitter.ino` in Arduino IDE
2. Connect the second Arduino Nano (RC transmitter)
3. Select correct board and port
4. Click **Upload** button

## Step 5: First-Time Setup

### On RC Transmitter:
1. Power on with Serial Monitor open (115200 baud)
2. Hold Button 1 for 3 seconds to enter joystick calibration
3. Follow on-screen instructions to calibrate joysticks
4. Calibration is saved to EEPROM

### On Flight Controller:
1. Power on with Serial Monitor open (115200 baud)
2. Wait for "NRF Connected!" message
3. Set ARM switch to DISARM position
4. Press Button 1 to calibrate IMU (place drone on flat surface!)
5. Calibration is saved to EEPROM

## Step 6: Pre-Flight Checklist

Before flying, verify:

- [ ] Props are SECURE but correctly oriented
- [ ] Battery is fully charged
- [ ] All connections are secure
- [ ] NRF connection established (buzzer beeps twice)
- [ ] ARM switch in DISARM position
- [ ] Throttle stick at minimum
- [ ] IMU calibrated (drone on level surface)

## Step 7: First Flight

1. ARM the drone with toggle switch
2. Press Button 2 for motor test - verify all motors spin
3. Slowly increase throttle to lift off
4. Keep altitude low (~1m) for first flight
5. Use small control inputs
6. If unstable, immediately DISARM with kill switch

## Troubleshooting

### NRF24L01 Not Connecting
- Add 10µF capacitor between VCC and GND
- Check wiring (CE, CSN, MOSI, MISO, SCK)
- Verify using 3.3V (NOT 5V!)
- Keep antenna away from metal

### Motors Not Spinning
- Verify ESC calibration
- Check signal wire connections
- Ensure throttle above minimum (>1050)
- Check ARM switch is ON

### Drone Unstable
- Re-calibrate IMU on perfectly flat surface
- Check propeller balance
- Verify motor directions
- Adjust PID values (reduce P first)

### Drifting
- Re-calibrate accelerometer
- Check propellers for damage
- Ensure balanced weight distribution

## PID Tuning Tips

If the drone oscillates or feels sluggish:

1. **Start with P only** (set I=0, D=0)
2. Increase P until oscillations begin
3. Reduce P by 20-30%
4. Add D to dampen oscillations
5. Add small I to eliminate drift

Default values in `config.h`:
```cpp
// Roll/Pitch
P = 1.2
I = 0.02
D = 18.0

// Yaw
P = 2.0
I = 0.02
D = 0.0
```

## Safety Reminders

⚠️ **ALWAYS**:
- Remove props when testing indoors
- Keep fingers away from spinning props
- Have kill switch ready
- Fly in open areas
- Check local drone regulations
- Never fly over people
- Maintain line of sight
