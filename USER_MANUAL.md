# Drone Flight Control System - User Manual

## Table of Contents
1. [System Overview](#system-overview)
2. [Pre-Flight Checklist](#pre-flight-checklist)
3. [Operating Instructions](#operating-instructions)
4. [Calibration Procedure](#calibration-procedure)
5. [Flight Controls](#flight-controls)
6. [Safety Features](#safety-features)
7. [Troubleshooting](#troubleshooting)
8. [Maintenance](#maintenance)

---

## System Overview

This DIY drone flight control system consists of two main components:

### Flight Controller (FC)
- **Brain:** Arduino Nano
- **Sensors:** MPU6050 (gyro/accel), MS5611 (barometer)
- **Communication:** NRF24L01 (2.4GHz wireless)
- **Control:** PID-based stabilization with altitude hold
- **Safety:** 30° max tilt angle, communication loss detection

### RC Controller
- **Interface:** 2x analog joysticks, 2x buttons, 2x switches
- **Communication:** NRF24L01 with ACK confirmation
- **Range:** ~100m line-of-sight (depends on environment)

---

## Pre-Flight Checklist

### Every Flight
- [ ] Check battery voltage (minimum 11.1V for 3S LiPo)
- [ ] Verify all propellers are secure and undamaged
- [ ] Check motor rotation directions
- [ ] Ensure RC controller has fresh batteries
- [ ] Verify common ground between all components
- [ ] Check for loose wires or connections
- [ ] Clear flight area of obstacles and people
- [ ] Verify GPS/compass if using navigation features

### Weekly Maintenance
- [ ] Clean propellers and motors
- [ ] Check all solder joints
- [ ] Verify ESC calibration
- [ ] Test buzzer and LED functionality
- [ ] Inspect frame for cracks or damage

---

## Operating Instructions

### Step 1: Power Up Sequence

**IMPORTANT: Always power on RC controller FIRST!**

1. **Turn on RC Controller**
   - LED will blink rapidly (searching for drone)
   - Serial monitor should show "Searching for drone..."

2. **Power on Flight Controller**
   - You'll hear startup melody: 3 ascending beeps
   - Wait for sensor initialization (~3 seconds)
   - "Drone Flight Controller Ready" appears in serial monitor

3. **Wait for Link Confirmation**
   - When FC connects to RC, you'll hear 2 high-pitched beeps
   - LED on RC will change to slow blinking
   - Serial shows "RC LINKED!"

### Step 2: Calibration (First Time / After Crashes)

**Required:** Only calibrate when drone is on a level surface and completely still.

1. Ensure **Switch 1 = 1** (DISARMED position - LED stays ON)
2. Press and hold **Button 1** on RC controller for 2 seconds
3. Release when you hear calibration melody
4. Keep drone absolutely still during calibration
5. You'll hear:
   - Initial beep pattern
   - Serial shows "Calibrating MPU6050..."
   - Serial shows "Calibrating MS5611..."
   - Final confirmation beep
6. Calibration values saved to EEPROM automatically

### Step 3: Arming the Drone

1. Verify **Switch 1 = 1** (disarmed - LED ON)
2. Move throttle to minimum (bottom position)
3. **Move Switch 1 = 0** (armed position)
4. Hold for 2 seconds
5. You'll hear confirmation beep
6. Serial shows "ARMED"
7. **LED turns OFF** (now only blinks when receiving RC signals)

**WARNING:** Motors will now respond to throttle stick!

### Step 4: Smooth Motor Test (Recommended Before First Flight)

This feature lets you verify all motors spin correctly without risking loss of control.

1. After arming, press **Button 2** once
2. All motors will slowly ramp up from 1050 to 1200 µs
3. Motors hold at 1200 µs for 2 seconds
4. Motors slowly ramp down to 1050 µs
5. You hear confirmation beep

**What to check:**
- All 4 motors spin
- Correct rotation direction (see wiring diagram)
- No unusual vibrations or sounds
- ESCs are not overheating

### Step 5: Flying

1. Slowly increase throttle until drone lifts off
2. Use right joystick for pitch/roll control
3. Use left joystick for throttle/yaw control
4. Keep movements smooth and gradual

**For beginners:**
- Start with small throttle inputs
- Hover at knee height first
- Practice in open area away from obstacles
- Have an experienced pilot nearby

### Step 6: Altitude Hold Mode (Optional)

1. While hovering at desired height (throttle around 1425 µs)
2. **Move Switch 2 = 0** (altitude hold ON)
3. You'll hear short beep
4. Drone will now maintain current altitude
5. Small throttle adjustments still work:
   - Up = climb slowly
   - Down = descend slowly
   - Center = hold altitude

**To disable:** Move Switch 2 = 1

### Step 7: Disarming / Landing

1. Land drone gently
2. Reduce throttle to minimum
3. **Move Switch 1 = 1** (disarm)
4. Motors stop immediately
5. Serial shows "DISARMED"
6. **LED turns ON** (warning: disarmed state)

### Step 8: Power Down

1. Disconnect flight battery
2. Turn off RC controller

---

## Calibration Procedure

### When to Calibrate

**Required:**
- First time setup
- After any crash or hard landing
- If drone drifts in hover
- After replacing MPU6050 or MS5611

**Not Required:**
- Every flight (calibration is saved)
- After battery changes
- Minor adjustments

### MPU6050 + MS5611 Calibration

This calibrates both sensors simultaneously.

1. **Place drone on perfectly level surface**
   - Use a spirit level if possible
   - Ensure no vibrations

2. **Verify disarmed** (Switch 1 = 1, LED ON)

3. **Press and hold Button 1** for 2 seconds

4. **Wait for calibration sequence:**
   ```
   Phase 1: Calibrating MPU6050 (gyroscope)
   - Duration: ~3 seconds
   - Keep completely still!
   
   Phase 2: Calibrating MS5611 (barometer)
   - Duration: ~1 second
   - Measures ground pressure
   
   Phase 3: Save to EEPROM
   - Permanent storage
   ```

5. **Confirmation:**
   - Final beep at 2200 Hz
   - LED blinks
   - Serial: "Calibration saved to EEPROM"

### Verifying Calibration

After calibration, check serial monitor:
```
Loaded calibration: X=0.45 Y=-0.32
Ground pressure: 1013.25 hPa
```

If values seem extreme (>5.0), recalibrate.

### ESC Calibration (Separate Procedure)

If motors don't respond correctly:

1. **Disconnect all ESC signal wires from Arduino**
2. **Connect ESC calibration jumper** (or use ESC programming card)
3. **Power on with full throttle**
4. **Wait for confirmation beeps**
5. **Reduce throttle to minimum**
6. **Wait for completion beeps**
7. **Power cycle and test**

---

## Flight Controls

### Joystick Layout

```
   LEFT STICK              RIGHT STICK
   
     UP                      UP
   Throttle                Pitch
   (Climb)               (Forward)
      ↑                      ↑
  ←       →              ←       →
  Yaw     Yaw            Roll    Roll
 (CCW)   (CW)           (Left)  (Right)
      ↓                      ↓
   Throttle                Pitch
  (Descend)              (Backward)
```

### Control Ranges

| Input | Range | Description |
|-------|-------|-------------|
| Throttle | 1000-2000 µs | Limited to 1700 µs in code for safety |
| Roll/Pitch | ±30° | Maximum tilt angle (safety limit) |
| Yaw | ±180° | Full rotation, auto-resets at ±180° |

### Button Functions

| Button | Function | Hold Time | State Required |
|--------|----------|-----------|----------------|
| Button 1 | Calibration | 2 seconds | Must be disarmed |
| Button 2 | Smooth Motor Start | Single press | Must be armed |

### Switch Functions

| Switch | Position | Function | LED Indicator |
|--------|----------|----------|---------------|
| Switch 1 | 1 (UP) | **DISARMED** (Safe) | ON continuously |
| Switch 1 | 0 (DOWN) | **ARMED** (Motors active) | Blinks on signal |
| Switch 2 | 1 (UP) | Altitude Hold OFF | - |
| Switch 2 | 0 (DOWN) | Altitude Hold ON | Beep confirmation |

---

## Safety Features

### 1. Maximum Tilt Angle (30°)

**Purpose:** Prevents loss of control from excessive tilting

**Behavior:**
- If drone tilts >30° on X or Y axis → **KILL SWITCH**
- Motors stop immediately
- Buzzer sounds continuously
- Manual recovery required

**Recovery:**
1. Level the drone manually
2. Wait for buzzer pattern change
3. System auto-resets when angle <30°
4. Re-arm and check for damage

### 2. Communication Loss Detection

**Purpose:** Stops motors if RC link is lost

**Behavior:**
- If no valid packets for 3 seconds → **KILL SWITCH**
- Motors stop
- Buzzer sounds every 2 seconds

**Recovery:**
1. Move RC controller closer
2. Check battery levels
3. System auto-recovers when signal restored
4. Re-arm to resume flight

### 3. Disarm Switch Protection

**Purpose:** Quick emergency stop

**Behavior:**
- Move Switch 1 to position 1 at any time
- Motors stop immediately
- LED turns ON (disarmed warning)
- No confirmation delay

### 4. Throttle Limiting

**Purpose:** Prevents excessive speed

**Behavior:**
- Maximum thrust limited to 1700 µs (not 2000 µs)
- Prevents overpowering and loss of control
- Extends battery life

### 5. Altitude Hold Safety

**Purpose:** Prevents altitude hold at dangerous heights

**Behavior:**
- Only activates when throttle between 1400-1450 µs
- Auto-disables if throttle moves outside range
- Prevents activation during takeoff/landing

---

## Troubleshooting

### Motors Don't Respond

**Symptoms:** Armed, but motors don't spin

**Causes & Solutions:**

1. **ESCs not calibrated**
   - Solution: Perform ESC calibration (see above)

2. **Wrong min/max values in code**
   - Check: `pMIN = 1000`, `pMAX = 2000`
   - Verify ESC range (some use 1100-1900)

3. **Loose signal wires**
   - Check all connections from D3, D5, D6, D9

4. **Insufficient battery voltage**
   - Minimum: 11.1V for 3S LiPo
   - Test: Measure battery voltage under load

5. **ESC not armed**
   - Some ESCs require initialization sequence
   - Power cycle drone while armed

### Drone Won't Link to RC

**Symptoms:** LED blinks rapidly, no connection

**Causes & Solutions:**

1. **NRF24L01 not powered correctly**
   - Must use 3.3V (NOT 5V!)
   - Add 10µF capacitor on VCC/GND

2. **Wrong channel or pipe address**
   - Verify: `radio.setChannel(108)` on both FC & RC
   - Verify: `pipe = 0xF0F0F0F0E1LL` matches

3. **NRF24L01 module damaged**
   - Swap modules to test
   - Check for physical damage

4. **Power on sequence wrong**
   - ALWAYS power RC first, then FC

5. **SPI wiring incorrect**
   - Verify: MOSI, MISO, SCK, CE, CSN connections

### Drone Drifts in Hover

**Symptoms:** Drone slowly drifts even with centered sticks

**Causes & Solutions:**

1. **Calibration needed**
   - Perform MPU6050 calibration
   - Must be on perfectly level surface

2. **Joystick not centered**
   - Check RC: Adjust trim or deadzone values
   - Verify: `lowPassX`, `lowPassY` in code

3. **Propellers unbalanced**
   - Balance all propellers
   - Replace damaged props

4. **Motor thrust uneven**
   - Check: All motors same KV rating
   - Verify: ESCs calibrated identically

5. **Frame flexing**
   - Stiffen frame mounting
   - Check for cracks

### Altitude Hold Not Working

**Symptoms:** Switch 2 activated but altitude varies

**Causes & Solutions:**

1. **Barometer not calibrated**
   - Press Button 1 to calibrate MS5611

2. **Activation range wrong**
   - Must activate between 1400-1450 µs throttle
   - Hover first, then activate

3. **Barometer noise**
   - Shield MS5611 from propeller wash
   - Add foam cover around sensor

4. **PID values need tuning**
   - Adjust: `pid_p_gain_altitude`, `pid_i_gain_altitude`, `pid_d_gain_altitude`
   - Start with P term, then add D, then I

5. **Altitude changes too fast**
   - Normal in windy conditions
   - Kalman filter helps but not perfect

### Excessive Vibration

**Symptoms:** Oscillations, instability in flight

**Causes & Solutions:**

1. **Props out of balance**
   - Balance all propellers
   - Use propeller balancer tool

2. **Soft mounting needed**
   - Add vibration dampers under FC
   - Use double-sided foam tape

3. **PID values too high**
   - Reduce P term first
   - Then reduce D term
   - I term should be very small

4. **Loose frame/screws**
   - Tighten all mounting screws
   - Use thread-lock compound

5. **Bent motor shafts**
   - Replace damaged motors
   - Check motor bearings

### Kill Switch Activated (Buzzer Sounds)

**Cause 1: Angle Exceeded (30°)**
```
Buzzer: Continuous beeping every 2s
LED: Blinking
Serial: "KILL: Angle exceeded!"
```
**Recovery:**
- Level drone manually
- System resets when angle <30°

**Cause 2: Communication Loss**
```
Buzzer: Beeping every 2s
LED: Rapid blinking
Serial: "RC LINK LOST!"
```
**Recovery:**
- Move RC closer
- Check RC battery
- Wait for "Communication restored" message

### Serial Monitor Shows Garbage

**Symptoms:** Unreadable characters

**Solution:**
- Set baud rate to **57600**
- Both FC and RC use same rate

### LED Behavior Reference

| LED Pattern | Meaning |
|-------------|---------|
| Solid ON | Disarmed (safe state) |
| Slow blink (500ms) | Armed, receiving RC signals |
| Rapid blink (100ms) | Communication lost |
| Very rapid blink (50ms) | Altitude hold active |
| Off | Not powered or error |

---

## Maintenance

### After Every Flight
1. Check for loose screws
2. Inspect propellers for cracks
3. Clean dust from motors
4. Check battery voltage

### Weekly
1. Test all buttons/switches
2. Verify LED and buzzer function
3. Check solder joints
4. Clean ESCs and wiring

### Monthly
1. Re-calibrate sensors
2. Update firmware if available
3. Test emergency procedures
4. Check frame alignment

### Storage
1. Remove battery if storing >1 week
2. Store LiPo at 3.8V per cell
3. Keep in cool, dry place
4. Avoid direct sunlight

---

## Performance Tuning

### PID Tuning Basics

**Current values (stable starting point):**
```cpp
kp = 2.0      // Proportional gain
ki = 0.0001   // Integral gain
kd = 0.5      // Derivative gain
```

**If drone oscillates:**
- Reduce P term (try 1.5)
- Reduce D term (try 0.3)

**If drone feels sluggish:**
- Increase P term (try 2.5)
- Increase D term (try 0.7)

**If drone drifts over time:**
- Slightly increase I term (try 0.0002)
- But keep very small to avoid windup

### Altitude Hold Tuning

**Current values:**
```cpp
pid_p_gain_altitude = 14.0
pid_i_gain_altitude = 2.0
pid_d_gain_altitude = 7.5
```

**If altitude fluctuates quickly:**
- Increase D term
- Add more smoothing to barometer

**If altitude drifts slowly:**
- Increase I term
- Check for barometer temperature drift

---

## Advanced Features

### Enabling Debug Mode

Uncomment in `Print()` function:
```cpp
if (dBugging) {
  // Serial output enabled
}
```

Then call: `debugging(true);` in setup()

### Adjusting Control Sensitivity

Edit these values in FC code:
```cpp
float sensiX = -0.45;     // Roll sensitivity
float sensiY = 0.45;      // Pitch sensitivity
float sensiZ = -0.01;     // Yaw sensitivity
float sensiThrust = 1.1;  // Throttle scaling
```

Smaller values = less sensitive
Larger values = more sensitive

### Changing Flight Frequency

Default: 140 Hz
```cpp
float hz = 140;
```

Higher = smoother control, more CPU usage
Lower = more jittery, less CPU usage

**Don't exceed 250 Hz on Arduino Nano!**

---

## Emergency Procedures

### Flyaway
1. **Immediately flip Switch 1 to disarm**
2. Let drone crash (better than losing it)
3. Check compass calibration before next flight

### Brownout (Battery Dies)
1. Drone will fall from sky
2. Be prepared to catch (dangerous!)
3. Next time: Land at 3.5V/cell minimum

### Water Landing
1. **Unplug battery immediately**
2. Disassemble completely
3. Dry for 48+ hours
4. Apply contact cleaner to electronics
5. Test components individually before reassembly

### Crash Recovery Checklist
- [ ] Inspect frame for cracks
- [ ] Check all motor mounts
- [ ] Test motors individually
- [ ] Verify prop installation
- [ ] Re-calibrate IMU
- [ ] Test on ground before flight

---

## Specifications

### Flight Controller
- **Processor:** ATmega328P @ 16MHz
- **Flash Memory:** 32KB
- **RAM:** 2KB
- **Loop Rate:** 140 Hz
- **Sensors:** MPU6050 (6-axis), MS5611 (barometer)
- **Communication:** NRF24L01 @ 250kbps
- **Range:** ~100m line-of-sight

### Supported Configurations
- **Frame:** Quadcopter X configuration only
- **Motors:** 4x brushless outrunners
- **ESCs:** 4x with 1000-2000µs PWM input
- **Battery:** 3S or 4S LiPo
- **Weight:** Depends on frame (tested 400-800g AUW)

---

## Support & Resources

### Useful Commands

**View real-time data:**
```
Open Serial Monitor (57600 baud)
Enable debug mode in code
Watch angles, motor values, pressure
```

### Common Values Reference

| Parameter | Typical Value | Range |
|-----------|---------------|-------|
| Ground pressure | 1013 hPa | 950-1050 hPa |
| Hover throttle | 1400-1500 µs | Depends on weight |
| Level calibration | <1.0° | Should be near 0 |
| Loop time | 7.14 ms | For 140 Hz |

---

**For additional support, check:**
- Serial output messages
- LED/buzzer patterns
- Code comments
- Wiring diagram

**Good luck and fly safe! 🚁**
