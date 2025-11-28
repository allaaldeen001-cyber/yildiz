# ✅ COMPREHENSIVE TESTING CHECKLIST

## 🔍 PRE-UPLOAD VERIFICATION

### Code Compilation
- [ ] `Drone_Flight_Control.ino` compiles without errors
- [ ] `Controller.ino` compiles without errors
- [ ] No missing library errors
- [ ] Board set to "Arduino Nano"
- [ ] Processor matches your board (Old Bootloader vs New)

### Library Installation
- [ ] RF24 library installed (by TMRh20)
- [ ] Smoothed library installed (by Matthew Fryer)
- [ ] MS5611 library installed (by Rob Tillaart)
- [ ] Wire library available (built-in)
- [ ] SPI library available (built-in)
- [ ] Servo library available (built-in)

---

## 🔌 HARDWARE ASSEMBLY TESTS

### Flight Controller - Component Tests

#### NRF24L01 Module
- [ ] 10µF capacitor soldered between VCC and GND
- [ ] VCC connected to 3.3V (NOT 5V!)
- [ ] GND connected
- [ ] CE connected to D2
- [ ] CSN connected to D10
- [ ] MOSI connected to D11
- [ ] MISO connected to D12
- [ ] SCK connected to D13
- [ ] Antenna oriented vertically

#### MPU6050 Sensor
- [ ] VCC to 5V
- [ ] GND to GND
- [ ] SDA to A4
- [ ] SCL to A5
- [ ] Sensor mounted level with drone frame
- [ ] Arrow on sensor points forward

#### MS5611 Barometer
- [ ] VCC to 5V or 3.3V (check your module)
- [ ] GND to GND
- [ ] SDA to A4 (shared with MPU6050)
- [ ] SCL to A5 (shared with MPU6050)
- [ ] No foam covering the pressure hole

#### Motor ESCs
- [ ] Front Left ESC signal to D3
- [ ] Front Right ESC signal to D5
- [ ] Rear Right ESC signal to D6
- [ ] Rear Left ESC signal to D9
- [ ] All ESC grounds connected to Arduino GND
- [ ] ESC power wires to battery (through PDB)
- [ ] ESCs calibrated (separate procedure if needed)

#### Buttons
- [ ] Calibration button: One side to A6, other to GND
- [ ] Smooth start button: One side to A7, other to GND
- [ ] Buttons are normally open (NO) type

#### Switches
- [ ] Arm switch: Center to D4, positions to GND and VCC
  - Position 1 (HIGH) = Disarmed
  - Position 2 (LOW) = Armed
- [ ] Altitude hold switch: Center to D7, positions to GND and VCC
  - Position 1 (HIGH) = Off
  - Position 2 (LOW) = Active

#### Outputs
- [ ] Buzzer positive to D8
- [ ] Buzzer negative to GND
- [ ] LED positive to A3
- [ ] 220Ω resistor from LED negative to GND

#### Battery Monitor
- [ ] Battery + to 1.5kΩ resistor
- [ ] Other end of 1.5kΩ to A0
- [ ] A0 to 1kΩ resistor
- [ ] Other end of 1kΩ to GND

#### Power
- [ ] Arduino VIN to 7-12V (from voltage regulator)
- [ ] Arduino GND to common ground
- [ ] Battery voltage appropriate (3S = 11.1V nominal)

---

### Remote Controller - Component Tests

#### NRF24L01 Module
- [ ] 10µF capacitor soldered between VCC and GND
- [ ] VCC connected to 3.3V
- [ ] GND connected
- [ ] CE connected to D9
- [ ] CSN connected to D10
- [ ] MOSI connected to D11
- [ ] MISO connected to D12
- [ ] SCK connected to D13
- [ ] Antenna oriented vertically

#### Joysticks
- [ ] Left stick X (Yaw) to A1
- [ ] Left stick Y (Throttle) to A0
- [ ] Right stick X (Roll) to A3
- [ ] Right stick Y (Pitch) to A2
- [ ] All joystick VCC to 5V
- [ ] All joystick GND to GND
- [ ] Joysticks move smoothly
- [ ] Springs return sticks to center

#### Status LED (Optional)
- [ ] LED positive to D8
- [ ] 220Ω resistor from LED negative to GND

#### Power
- [ ] Battery or USB power connected
- [ ] Voltage regulator if using LiPo
- [ ] Power switch installed (recommended)

---

## 💻 SOFTWARE UPLOAD TESTS

### Flight Controller Upload
- [ ] Code uploaded successfully
- [ ] No upload errors
- [ ] Serial Monitor opens (57600 baud)
- [ ] See "Motors attached" message
- [ ] See startup beep sequence (if buzzer connected)
- [ ] See "Waiting for RC link..." message

### Remote Controller Upload
- [ ] Code uploaded successfully
- [ ] No upload errors
- [ ] Serial Monitor opens (57600 baud)
- [ ] See "RC Controller Ready" message
- [ ] LED blinks 3 times (if connected)

---

## 📡 COMMUNICATION TESTS

### Link Establishment
- [ ] Power RC first
- [ ] Power FC second
- [ ] Within 10 seconds, hear beep-beep from FC
- [ ] FC Serial Monitor shows "Radio LINKED"
- [ ] RC LED turns on (if connected)
- [ ] FC LED blinks periodically

### Signal Quality
- [ ] RC Serial Monitor shows incrementing ID numbers
- [ ] FC LED blinks consistently (when armed)
- [ ] Move RC 5m away - link maintained
- [ ] Move RC 10m away - link maintained
- [ ] Metal obstacles - link degrades gracefully
- [ ] Turn off RC - FC buzzer alerts within 3 seconds

---

## 🎯 CALIBRATION TESTS

### Pre-Calibration
- [ ] Drone on perfectly level surface
- [ ] No vibrations
- [ ] Arm switch in DISARMED position (D4 = HIGH)
- [ ] FC LED stays constantly ON (disarmed indicator)

### Calibration Procedure
- [ ] Press calibration button (A6)
- [ ] Hold for 2 seconds
- [ ] Hear: beep-pause-beep (started)
- [ ] Wait 10-15 seconds
- [ ] Hear: beep-beep (completed)
- [ ] LED blinks twice
- [ ] Serial Monitor shows "Calibration complete!"
- [ ] Serial Monitor shows ground pressure value

### Post-Calibration
- [ ] Cal values saved to EEPROM
- [ ] Power cycle FC - values retained
- [ ] Drone does not drift when armed (test without props)

---

## 🎮 CONTROL INPUT TESTS

### Throttle (A0)
- [ ] Stick down: RC shows ~1000
- [ ] Stick center: RC shows ~1400
- [ ] Stick up: RC shows ~1800
- [ ] Movement is smooth
- [ ] No jitter in Serial Monitor

### Yaw (A1)
- [ ] Stick left: Shows negative value
- [ ] Stick center: Shows ~0
- [ ] Stick right: Shows positive value
- [ ] Movement is smooth

### Pitch (A2)
- [ ] Stick forward: Shows negative value
- [ ] Stick center: Shows ~0
- [ ] Stick back: Shows positive value
- [ ] Movement is smooth

### Roll (A3)
- [ ] Stick left: Shows negative value
- [ ] Stick center: Shows ~0
- [ ] Stick right: Shows positive value
- [ ] Movement is smooth

### Switches
- [ ] Arm switch changes state
- [ ] Altitude hold switch changes state
- [ ] FC responds to switch changes

---

## 🔄 MOTOR TESTS (NO PROPELLERS!)

### ⚠️ CRITICAL: REMOVE ALL PROPELLERS FOR THESE TESTS!

### Arming Test
- [ ] Arm switch to DISARMED (HIGH)
- [ ] LED stays ON continuously
- [ ] Motors do not spin
- [ ] Arm switch to ARMED (LOW)
- [ ] LED starts blinking
- [ ] Motors still do not spin (correct!)

### Smooth Motor Start
- [ ] System armed
- [ ] Press smooth start button (A7)
- [ ] Hear short beep
- [ ] Motors ramp up slowly over ~2 seconds
- [ ] All 4 motors spin
- [ ] Motors reach ~1100 µs speed
- [ ] No sudden jumps

### Individual Motor Check
- [ ] Front Left (D3) spins CCW ↺
- [ ] Front Right (D5) spins CW ↻
- [ ] Rear Right (D6) spins CCW ↺
- [ ] Rear Left (D9) spins CW ↻

### Motor Direction Correction
If any motor spins wrong direction:
- [ ] Swap any 2 of the 3 motor wires
- [ ] Retest
- [ ] Verify all directions correct before proceeding

### Throttle Response Test
- [ ] Motors started (via smooth start)
- [ ] Slowly increase throttle
- [ ] All motors speed up proportionally
- [ ] Decrease throttle
- [ ] All motors slow down together

### Control Response Tests (No Props!)
- [ ] Roll stick right → FL & RL speed up, FR & RR slow down
- [ ] Roll stick left → FR & RR speed up, FL & RL slow down
- [ ] Pitch stick forward → FL & FR speed up, RL & RR slow down
- [ ] Pitch stick back → RL & RR speed up, FL & FR slow down
- [ ] Yaw stick right → FL & RR speed up, FR & RL slow down
- [ ] Yaw stick left → FR & RL speed up, FL & RR slow down

### Disarm Test
- [ ] Motors spinning
- [ ] Flip arm switch to DISARMED
- [ ] Motors stop immediately
- [ ] LED stays ON (disarmed indicator)

---

## 🛡️ SAFETY FEATURE TESTS

### Auto-Disarm on Signal Loss
- [ ] Arm system and start motors
- [ ] Turn off RC transmitter
- [ ] Within 3 seconds: motors stop
- [ ] Buzzer repeats every 2 seconds
- [ ] Turn on RC transmitter
- [ ] Wait for signal restore
- [ ] Hear long beep (signal restored)
- [ ] System still disarmed (must re-arm)

### Manual Kill Switch
- [ ] Arm and start motors
- [ ] Flip arm switch to DISARMED
- [ ] Motors stop within 1 loop cycle (~7ms)
- [ ] LED goes to always-ON

### Angle Limit (Advanced - needs gyro simulation)
- [ ] Serial Monitor shows gyro angles
- [ ] Tilt FC beyond 30° on X axis
- [ ] System should disarm (if killAngle enabled)
- [ ] Tilt beyond 30° on Y axis
- [ ] System should disarm

---

## 🔔 LED & BUZZER TESTS

### Buzzer Patterns
- [ ] Startup: Beep-Beep-Beeeep ✅
- [ ] Link: Beep-Beep ✅
- [ ] Calibration start: Beep-pause-Beep ✅
- [ ] Calibration end: Beep-Beep ✅
- [ ] Smooth start: Single beep ✅
- [ ] Kill switch: Repeating beep every 2s ✅
- [ ] Signal restore: Long beep ✅

### LED Behavior
- [ ] Disarmed: Always ON ✅
- [ ] Armed + signal: Blinking ✅
- [ ] Signal loss: OFF ✅
- [ ] Calibration: 2 blinks ✅

---

## 📊 SENSOR VERIFICATION TESTS

### MPU6050 Gyro/Accelerometer
- [ ] Serial Monitor shows gyro angles
- [ ] Tilt FC forward: Y angle increases
- [ ] Tilt FC backward: Y angle decreases
- [ ] Tilt FC left: X angle decreases
- [ ] Tilt FC right: X angle increases
- [ ] Return to level: Angles return to ~0
- [ ] No excessive drift (< 5° per minute)

### MS5611 Barometer
- [ ] Serial Monitor shows pressure reading
- [ ] Pressure stable when stationary
- [ ] Lift FC 1 meter: Pressure decreases
- [ ] Lower FC: Pressure increases
- [ ] Cover pressure hole: Reading changes
- [ ] Uncover: Returns to normal

---

## ✈️ FIRST FLIGHT TESTS (PROPELLERS ON)

### ⚠️ SAFETY FIRST!
- [ ] Open outdoor area
- [ ] No people within 10 meters
- [ ] No obstacles overhead
- [ ] Wind < 10 mph
- [ ] Battery fully charged
- [ ] Propellers installed correctly:
  - FL: CCW prop
  - FR: CW prop
  - RR: CCW prop
  - RL: CW prop
- [ ] Propellers tight (use prop tool)
- [ ] Fire extinguisher nearby (for LiPo)

### Ground Test with Propellers
- [ ] Arm system
- [ ] Smooth motor start
- [ ] Increase throttle slowly to 50%
- [ ] Drone should feel light
- [ ] Should lift slightly off ground
- [ ] Disarm
- [ ] Check all screws still tight

### Hover Test
- [ ] Place drone in center of area
- [ ] Arm system
- [ ] Smooth motor start
- [ ] Slowly increase throttle
- [ ] Drone lifts to ~0.5m height
- [ ] Drone hovers relatively stable
- [ ] Small oscillations are normal
- [ ] Reduce throttle
- [ ] Drone lands gently
- [ ] Disarm

### Basic Maneuver Tests
- [ ] Takeoff to 1m
- [ ] Hold hover for 10 seconds
- [ ] Small roll input right - drone moves right
- [ ] Roll input left - drone moves left
- [ ] Small pitch input forward - drone moves forward
- [ ] Pitch input back - drone moves back
- [ ] Yaw input right - drone rotates CW
- [ ] Yaw input left - drone rotates CCW
- [ ] Return to center
- [ ] Land safely

### Altitude Hold Test
- [ ] Takeoff to 2m height
- [ ] Hover steady
- [ ] Flip altitude hold switch (D7) to LOW
- [ ] Release throttle to center (1400-1450)
- [ ] Drone maintains altitude (~±0.5m)
- [ ] Roll/pitch/yaw still work
- [ ] Small throttle up - climbs slowly
- [ ] Small throttle down - descends slowly
- [ ] Flip switch back to HIGH - manual mode
- [ ] Land

### Extended Flight Test
- [ ] Fly for 2 minutes
- [ ] Monitor battery voltage
- [ ] Check for excessive drift
- [ ] Check for motor overheating
- [ ] Land
- [ ] Check all screws and connections

---

## 🔋 BATTERY & POWER TESTS

### Voltage Monitoring
- [ ] Serial Monitor shows battery voltage
- [ ] Full battery: Shows ~12.6V (for 3S)
- [ ] Half depleted: Shows ~11.1V
- [ ] Low battery: Shows ~10.5V
- [ ] Values match multimeter (±0.2V)

### Power Consumption
- [ ] Measure current at hover
- [ ] Typical: 5-15A for small quad
- [ ] Check for excessive current (>20A)
- [ ] Motors should not overheat

### Low Battery Behavior
- [ ] Reduce battery to 10.5V (use discharge)
- [ ] Fly conservatively
- [ ] Land before 9.9V!
- [ ] Note: No automatic low battery protection yet

---

## 📈 PERFORMANCE TUNING TESTS

### PID Response Check
- [ ] Hover steady
- [ ] Quick roll input
- [ ] Drone responds quickly
- [ ] Returns to level without overshoot
- [ ] No oscillations

### If Oscillations Present
- [ ] Decrease `kp` by 0.5
- [ ] Re-upload and test
- [ ] Decrease `kd` if still oscillating
- [ ] Increase slowly until optimal

### If Response Too Slow
- [ ] Increase `kp` by 0.5
- [ ] Re-upload and test
- [ ] Increase `kd` if needed
- [ ] Find balance between speed and stability

---

## 🐛 TROUBLESHOOTING VERIFICATION

### Test Each Common Issue Resolution

#### No Link
- [ ] Verify NRF wiring
- [ ] Check 10µF capacitor
- [ ] Verify 3.3V power
- [ ] Re-upload firmware
- [ ] Test with antennas close

#### Drift
- [ ] Recalibrate on level surface
- [ ] Check frame for damage
- [ ] Verify all motors spin freely
- [ ] Check propeller balance

#### One Motor Not Spinning
- [ ] Check ESC signal wire
- [ ] Swap ESC to test
- [ ] Check Arduino pin with LED
- [ ] Verify ESC is powered

#### Altitude Hold Not Working
- [ ] Verify MS5611 I2C connection
- [ ] Check switch wiring (D7)
- [ ] Ensure throttle in 1400-1450 range
- [ ] Monitor pressure readings

---

## 📋 FINAL CHECKLIST BEFORE REGULAR USE

### Documentation Review
- [ ] Read USER_MANUAL.md completely
- [ ] Review PIN_MAPPING.md
- [ ] Understand QUICK_REFERENCE.md
- [ ] Bookmark TROUBLESHOOTING section

### Safety Verification
- [ ] All safety features tested
- [ ] Kill switch works reliably
- [ ] Signal loss protection verified
- [ ] Angle limit appropriate

### Flight Readiness
- [ ] Multiple successful flights
- [ ] Confident in controls
- [ ] Know emergency procedures
- [ ] Have spare parts (props, battery)

### Legal Compliance
- [ ] Check local drone regulations
- [ ] Register drone if required
- [ ] Avoid restricted airspace
- [ ] Maintain visual line of sight

---

## ✅ SIGN-OFF

I certify that I have completed all applicable tests and the system is safe for flight:

**Tester Name:** _______________

**Date:** _______________

**Signature:** _______________

---

## 📊 TEST RESULTS SUMMARY

| Category | Pass/Fail | Notes |
|----------|-----------|-------|
| Hardware Assembly | ☐ | |
| Software Upload | ☐ | |
| Communication | ☐ | |
| Calibration | ☐ | |
| Motor Tests | ☐ | |
| Safety Features | ☐ | |
| Sensor Verification | ☐ | |
| First Flight | ☐ | |
| Altitude Hold | ☐ | |

**Overall Status:** ☐ PASS ☐ FAIL

**Ready for Regular Use:** ☐ YES ☐ NO

---

## 🎯 NEXT STEPS AFTER PASSING ALL TESTS

1. **Practice Flying**
   - Spend several sessions on basic maneuvers
   - Gradually increase flight time
   - Practice emergency procedures

2. **Tune PID Values**
   - Fine-tune for your specific frame
   - Adjust for different payloads
   - Document optimal values

3. **Consider Upgrades**
   - GPS module
   - FPV camera
   - Better battery
   - Improved props

4. **Maintenance Schedule**
   - Inspect before each flight
   - Check prop balance weekly
   - Recalibrate monthly
   - Update firmware as needed

---

**Happy Flying! 🚁**

*Remember: Safety first, always!*
