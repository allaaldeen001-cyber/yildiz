# Troubleshooting Guide

Complete guide to diagnosing and fixing common issues with your quadcopter drone project.

## Table of Contents
1. [Pre-Flight Issues](#pre-flight-issues)
2. [Communication Problems](#communication-problems)
3. [Motor Issues](#motor-issues)
4. [Flight Problems](#flight-problems)
5. [Sensor Issues](#sensor-issues)
6. [Power Problems](#power-problems)
7. [Diagnostic Tools](#diagnostic-tools)

---

## Pre-Flight Issues

### ❌ Arduino Won't Upload Code

#### Problem 1: "Programmer not responding"
**Symptoms:** Upload fails immediately

**Causes & Solutions:**
```
1. Wrong COM port selected
   → Tools → Port → Select correct port
   → Unplug/replug USB to identify

2. Wrong board type
   → Tools → Board → Arduino Nano
   → Tools → Processor → ATmega328P (Old Bootloader)
   
3. USB driver issue
   → Install CH340G driver for clone Nanos
   → Download from manufacturer website

4. Faulty USB cable
   → Try different cable (must support data, not just power)
```

#### Problem 2: "Sketch too big"
**Symptoms:** "Sketch uses XXX bytes (XXX%) of program storage space"

**Solutions:**
```cpp
// Remove debug code:
// Comment out Serial.println() statements
#define DEBUG 0
#if DEBUG
  Serial.println("Debug message");
#endif

// Optimize strings:
Serial.println(F("Use F() macro")); // Stores in flash, not RAM
```

---

### ❌ Sensors Not Detected

#### MPU6050 Not Found

**Symptoms:**
```
Serial output: "MPU6050 connection failed!"
```

**Diagnosis:**
```cpp
// Add I2C scanner code:
void scanI2C() {
  Serial.println("Scanning I2C bus...");
  for (byte i = 0; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
    }
  }
}
```

**Expected:** Should find device at 0x68

**Solutions:**
```
1. Check wiring:
   ✓ SDA to A4
   ✓ SCL to A5
   ✓ VCC to 3.3V (not 5V)
   ✓ GND to GND

2. Check connections:
   → Resolder loose joints
   → Check for shorts with multimeter

3. Add pull-up resistors:
   → 4.7kΩ from SDA to 3.3V
   → 4.7kΩ from SCL to 3.3V

4. Try different I2C address:
   → Connect AD0 pin to 3.3V (address becomes 0x69)
   → Update code: mpu.setAddress(0x69);

5. Module may be faulty:
   → Test with different MPU6050 module
```

#### MS5611 Not Found

**Symptoms:**
```
Serial output: "MS5611 connection failed!"
I2C scanner doesn't show 0x77
```

**Solutions:**
```
1. MS5611 is optional - drone will fly without it
   → Altitude hold won't work
   → Manual altitude control only

2. Check wiring (same as MPU6050)

3. Try different I2C address variant:
   → Some modules use 0x76 instead of 0x77
   
4. Update code:
   ms5611.begin(MS5611_ULTRA_HIGH_RES);
```

---

## Communication Problems

### ❌ Radio Not Working

#### Problem 1: "Radio hardware not responding"

**Symptoms:**
```
Remote: Serial output shows initialization failed
Flight Controller: No data received
```

**Solutions:**

**Step 1: Verify Power**
```
Measure voltage at nRF24 VCC pin:
Expected: 3.3V (±0.3V)

If outside range:
  → Check 3.3V pin voltage on Arduino
  → Add 10µF capacitor right at nRF24 VCC/GND
  → Use separate 3.3V regulator (LD1117V33)
```

**Step 2: Check Wiring**
```
nRF24 Pin → Arduino
─────────────────────
VCC    → 3.3V (NOT 5V!)
GND    → GND
CE     → D4 (flight) / D9 (remote)
CSN    → D10
MOSI   → D11
MISO   → D12
SCK    → D13
```

**Step 3: Test with Simple Code**
```cpp
// Add to setup():
if (!radio.begin()) {
  Serial.println("FAIL: Radio hardware");
  // Check wiring
} else if (!radio.isChipConnected()) {
  Serial.println("FAIL: Radio not detected");
  // Module may be faulty
} else {
  Serial.println("SUCCESS: Radio OK");
}
```

**Step 4: Module Replacement**
```
nRF24L01+ modules fail frequently
Buy 3-5 modules, expect 1-2 to be DOA
Symptoms of bad module:
  - No response
  - Random disconnections
  - Very short range (<5m)
```

#### Problem 2: Transmission Failed

**Symptoms:**
```
Remote: "Transmission failed!" messages
Flight Controller: Intermittent data
```

**Solutions:**

**1. Increase Retries**
```cpp
radio.setRetries(15, 15);  // Max delay, max retries
```

**2. Change Channel**
```cpp
// Avoid WiFi interference
radio.setChannel(108);  // Try channels 100-125
```

**3. Reduce Data Rate**
```cpp
radio.setDataRate(RF24_250KBPS);  // Slowest = most reliable
```

**4. Check Address Match**
```cpp
// Must be IDENTICAL on both controllers
const byte address[6] = "00001";
```

**5. Verify Antenna**
```
For PA+LNA modules:
  → Ensure external antenna is connected
  → Antenna should be perpendicular to board
  → Never power on PA+LNA without antenna (can damage chip)
```

#### Problem 3: Short Range

**Symptoms:** Works at <10m, fails beyond

**Solutions:**
```
1. Maximize transmit power:
   radio.setPALevel(RF24_PA_MAX);

2. Use PA+LNA module with external antenna

3. Keep antennas perpendicular (not parallel):
   
   GOOD:           BAD:
   TX ↑            TX ↑
                   
   RX →            RX ↑

4. Remove obstacles between remote and drone

5. Check for interference:
   → Turn off WiFi devices nearby
   → Change radio channel
   → Use spectrum analyzer app to find quiet channel
```

---

## Motor Issues

### ❌ Motors Won't Spin

#### Problem 1: No Response When Armed

**Diagnosis Steps:**

**1. Check Arming:**
```
Serial monitor should show:
  "ARMED" when Button 3 pressed
  LED should be solid on
  
If not arming:
  → Throttle must be at minimum
  → Sensors must be calibrated
  → Radio signal must be strong
```

**2. Check PWM Output:**
```cpp
// Add debug code:
Serial.print("Motor FL: "); Serial.println(motorFL_speed);
Serial.print("Motor FR: "); Serial.println(motorFR_speed);

Expected: Values between 1000-2000
If all show 1000: Normal when disarmed
If all show 0: PWM not working
```

**3. Test ESC Directly:**
```cpp
// In setup(), before main loop:
void testMotor() {
  escFL.writeMicroseconds(1200); // Low speed
  delay(3000);
  escFL.writeMicroseconds(1000); // Stop
}
```

**4. Check ESC Power:**
```
Measure voltage at ESC power input:
  Expected: 11.1V (3S LiPo)
  
Check ESC signal connection:
  Expected: Signal wire to Arduino D3/D5/D6/D9
  Expected: Ground wire to Arduino GND
  Expected: BEC wire to Arduino VIN (one ESC only)
```

#### Problem 2: ESC Beeping Continuously

**Beep Patterns:**

| Beep Pattern | Meaning | Solution |
|--------------|---------|----------|
| Beep-beep-beep (rapid) | No signal | Check signal wire connection |
| Beep...beep...beep (slow) | Low voltage | Charge battery or check connections |
| Beep-beep (double) | Waiting for calibration | Perform ESC calibration |
| Continuous tone | Initialization | Normal for 2-3 seconds |

**ESC Calibration Procedure:**
```
Prerequisites:
  ✓ Remove ALL propellers
  ✓ Disconnect Arduino from ESC signal wires

Steps:
1. Connect ESC to battery
2. Listen for beeps (confirms power)
3. Connect signal wire
4. Send max throttle (2000µs)
   → ESC beeps to confirm
5. Send min throttle (1000µs)
   → ESC beeps differently
6. Calibration complete
7. Repeat for all 4 ESCs
```

#### Problem 3: Motor Spins Wrong Direction

**Symptoms:** Motor spins but wrong direction (CW instead of CCW or vice versa)

**Solution:**
```
Swap ANY 2 of the 3 motor wires

Example:
  Current: A-B-C
  Options: B-A-C  or  A-C-B  or  C-B-A
  
All three options will reverse direction
```

**Verify Directions:**
```
Looking down at drone:
  M1 (Front Left):  Should spin CCW ↺
  M2 (Front Right): Should spin CW ↻
  M3 (Rear Right):  Should spin CCW ↺
  M4 (Rear Left):   Should spin CW ↻
```

---

## Flight Problems

### ❌ Drone Flips Immediately on Takeoff

**This is the #1 problem for first-time builds!**

#### Cause 1: Motor Direction Wrong

**Test:**
```
Arm drone (without props)
Slowly increase throttle
Observe motor directions

Expected:
  M1: CCW ↺
  M2: CW ↻
  M3: CCW ↺
  M4: CW ↻

If any motor is wrong:
  → Swap 2 motor wires to reverse
```

#### Cause 2: Motors in Wrong Position

**Test:**
```
With props installed (be careful!):
  Check propeller rotation matches motor position
  
Correct configuration:
      FRONT
    M1↺  ↻M2
       X
    M4↻  ↺M3
      REAR
```

#### Cause 3: Propellers Installed Backwards

**Check:**
```
Propellers have a top and bottom
Curved side should face up
Flat side faces down

Label markings:
  CW propellers:  "R" or clockwise arrow →
  CCW propellers: "L" or counter-clockwise arrow ←
  
Install on matching motors!
```

#### Cause 4: IMU Not Calibrated

**Symptoms:**
- Serial monitor shows large angle values when level
- Drone tries to "correct" even when sitting flat

**Solution:**
```
1. Place drone on perfectly flat surface
2. Press Button 1 (Calibrate) on remote
3. Wait for confirmation beeps
4. Verify angles near 0° in serial monitor
```

#### Cause 5: IMU Orientation Wrong

**Problem:** MPU6050 mounted upside-down or sideways

**Solutions:**
```cpp
// If mounted upside-down:
roll = -roll;
pitch = -pitch;

// If mounted 90° rotated:
float temp = roll;
roll = pitch;
pitch = temp;
```

---

### ❌ Drone Oscillates/Shakes

**Symptoms:** Rapid back-and-forth rocking, even when trying to hover

#### Cause: PID Gains Too High

**The Problem:**
```
P gain too high → Overreacts to error → Overcorrects → Oscillates
D gain too low → No damping → Oscillation continues
```

**Solution: Reduce P, Increase D**
```cpp
// Start with conservative values:
float Kp_roll = 0.8;   // Reduce from 1.5
float Kd_roll = 1.2;   // Increase from 0.8

// Test fly
// If still oscillating: reduce P more
// If too sluggish: increase P slightly
```

**Tuning Process:**
```
1. Start with all gains at 0
2. Increase P until drone responds to tilt
3. Increase P until it starts to oscillate
4. Reduce P by 30%
5. Increase D until oscillations stop
6. Add small amount of I (0.01-0.05)
```

#### Cause: Motor Vibrations

**Symptoms:** High-frequency vibration, especially at high throttle

**Solutions:**
```
1. Balance propellers:
   → Use propeller balancer
   → Add tape to light side
   
2. Check motor bearings:
   → Spin motor by hand
   → Should spin smoothly, no grinding
   
3. Tighten all screws:
   → Motor mount screws
   → Frame screws
   → Flight controller mount
   
4. Add vibration dampening:
   → Mount flight controller on foam
   → Use rubber grommets
```

---

### ❌ Drone Drifts in One Direction

**Symptoms:** Constantly drifts forward/back/left/right even with centered sticks

#### Cause 1: Joystick Not Centered

**Test:**
```
Serial Monitor on Remote:
  Throttle: 512, Yaw: 512, Pitch: 512, Roll: 512
  
If values not near 512:
  → Recalibrate joysticks (power on remote, wait 2 sec)
  → Adjust DEADBAND value (currently 20)
```

#### Cause 2: IMU Drift

**Symptoms:** Angle values in serial monitor slowly increase/decrease over time

**Solution:**
```cpp
// Increase accelerometer influence in complementary filter
const float ALPHA = 0.96; // Was 0.98, now 96% gyro / 4% accel

// Or add integral term to compensate:
float Ki_roll = 0.08;  // Increase from 0.05
float Ki_pitch = 0.08;
```

#### Cause 3: CG (Center of Gravity) Off

**Test:**
```
Balance drone on finger
Should balance at center of X (where arms cross)

If front-heavy:
  → Move battery backward
If rear-heavy:
  → Move battery forward
```

#### Cause 4: Motor Imbalance

**Test:**
```
Motor test mode (Button 2)
All motors should sound the same pitch
Listen for differences

If one motor sounds different:
  → ESC may have different timing
  → Motor may have damaged bearing
  → Replace motor or ESC
```

---

### ❌ Uncontrollable / Runs Away

**DANGER: Disarm immediately!**

#### Cause 1: PID in Wrong Direction

**Symptoms:** Tilting left makes it tilt MORE left (positive feedback)

**Solution:**
```cpp
// Check PID signs in motor mixing:
// Should be:
motorFL_speed = baseThrottle - rollPID - pitchPID + yawPID;
motorFR_speed = baseThrottle + rollPID - pitchPID - yawPID;
motorRR_speed = baseThrottle + rollPID + pitchPID + yawPID;
motorRL_speed = baseThrottle - rollPID + pitchPID - yawPID;

// If backwards, invert PID:
rollPID = -rollPID;
```

#### Cause 2: Flight Mode Mismatch

**Problem:** Beginner pilot in ACRO mode

**Solution:**
```
Switch to ANGLE mode:
  → Flip Switch 2 (D3) on remote
  → Serial monitor should show "Mode: ANGLE"
  
ANGLE mode = self-leveling (easier)
ACRO mode = manual (expert only)
```

---

## Sensor Issues

### ❌ Altitude Hold Doesn't Work

**Symptoms:** Drone doesn't maintain height automatically

**Possible Causes:**

#### 1. MS5611 Not Connected
```
Check serial monitor: "MS5611 connected!"
If missing: altitude hold won't work
```

#### 2. Altitude Hold Not Enabled
```cpp
// In code, check this is true:
altitudeHold = true;

// Or implement altitude hold trigger:
if (receivedData.altHoldButton) {
  altitudeHold = !altitudeHold;
  targetAltitude = currentAltitude;
}
```

#### 3. PID Gains Not Tuned
```cpp
// Altitude PID may need adjustment:
float Kp_alt = 2.0;   // Try increasing
float Ki_alt = 0.1;
float Kd_alt = 1.5;

// Test: Does throttle change when altitude changes?
Serial.println(altPID);  // Should show non-zero values
```

---

### ❌ Angles Drift Over Time

**Symptoms:** 
- Start level, after 30 seconds shows 10° tilt
- Drone thinks it's tilted when actually level

**Cause:** Gyro drift not corrected enough

**Solution:**
```cpp
// Reduce ALPHA (trust accelerometer more)
const float ALPHA = 0.96;  // Was 0.98

// If very bad drift:
const float ALPHA = 0.94;  // 94% gyro, 6% accel

// Trade-off:
//   Lower ALPHA = less drift, more noise
//   Higher ALPHA = more drift, less noise
```

---

## Power Problems

### ❌ Battery Drains Too Fast

**Symptoms:** <5 minutes flight time with 2200mAh battery

**Normal Flight Time:** 8-10 minutes hover, 5-7 minutes aggressive

**Causes:**

#### 1. Old/Damaged Battery
```
Test voltage under load:
  → Full charge: 12.6V
  → Under load (motors on): should stay >11.5V
  → If drops to <11V: battery is worn out
```

#### 2. Excessive Current Draw
```
Measure current with amp meter:
  → Hover: 10-15A normal
  → Full throttle: 25-35A normal
  → >40A: something wrong

Causes of high current:
  - Damaged motor (bad bearing)
  - Oversized propellers
  - Props installed backwards
  - Motor spinning wrong direction
```

#### 3. Faulty ESC
```
One ESC may be failing:
  → Test current per ESC
  → All should be similar
  → Replace outlier
```

---

### ❌ Low Voltage Warning / Failsafe Triggered

**Symptoms:** Drone descends automatically, buzzer sounds

**Causes:**

#### 1. Battery Actually Low
```
Measure voltage:
  → <10.5V per 3S battery = LAND NOW
  → <9.9V = Critical, may damage battery
  
Solution: Charge or replace battery
```

#### 2. Voltage Sag Under Load
```
Voltage may be 12V idle, drop to 10V under throttle
Causes:
  - Weak battery
  - High-resistance connections
  - Undersized battery (need higher C rating)
  
Solution: Use 30C or higher rated battery
```

---

## Diagnostic Tools

### Serial Monitor Debug Commands

Add these to your code for better diagnostics:

```cpp
// Print all angles
void printAngles() {
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Yaw: "); Serial.println(yaw);
}

// Print all motor speeds
void printMotors() {
  Serial.print("M1: "); Serial.print(motorFL_speed);
  Serial.print(" M2: "); Serial.print(motorFR_speed);
  Serial.print(" M3: "); Serial.print(motorRR_speed);
  Serial.print(" M4: "); Serial.println(motorRL_speed);
}

// Print PID values
void printPID() {
  Serial.print("Roll PID: "); Serial.print(rollPID);
  Serial.print(" Pitch PID: "); Serial.print(pitchPID);
  Serial.print(" Yaw PID: "); Serial.println(yawPID);
}

// Print radio data
void printRadio() {
  Serial.print("T:"); Serial.print(receivedData.throttle);
  Serial.print(" Y:"); Serial.print(receivedData.yaw);
  Serial.print(" P:"); Serial.print(receivedData.pitch);
  Serial.print(" R:"); Serial.println(receivedData.roll);
}
```

### Multimeter Checks

**Power Rails:**
```
Arduino VIN:  4.5-5.5V (from ESC BEC)
Arduino 5V:   5.0V ±0.1V
Arduino 3.3V: 3.3V ±0.1V
Battery:      11.1V nominal (10.5-12.6V range)
```

**Continuity Checks:**
```
All grounds should be connected:
  - Arduino GND
  - Each ESC GND
  - Battery GND
  - Sensor GND
  - Radio GND
  
Resistance between any two GND points: <1Ω
```

---

## Emergency Procedures

### During Flight Emergency

#### 1. Loss of Control
```
ACTION: Press Button 3 (DISARM) immediately
RESULT: All motors stop
CONSEQUENCE: Drone will fall and may be damaged
BETTER THAN: Uncontrolled flight into people/property
```

#### 2. Radio Signal Lost
```
AUTOMATIC: Failsafe activates after 1 second
BEHAVIOR: Motors reduce to 30%, slow descent
RECOVERY: Move closer, regain signal, or let it land
```

#### 3. Low Battery
```
WARNING: Buzzer beeps (if voltage monitoring enabled)
ACTION: Land immediately
DO NOT: Continue flying or battery will be damaged
```

#### 4. Strange Behavior
```
If drone:
  - Suddenly tilts >30°
  - Spins uncontrollably
  - Accelerates upward uncontrolled
  
ACTION: DISARM (Button 3) immediately
Better to drop than crash into obstacles
```

---

## Testing Checklist

### Pre-Flight Tests (Every Time)

```
☐ Battery voltage >11.1V
☐ All propellers tight
☐ No loose wires
☐ Remote powered on first
☐ Drone powered on second
☐ Calibration confirmed (if moved)
☐ Radio signal strong
☐ Joysticks centered
☐ ARM/DISARM works
☐ Throttle response smooth
☐ Clear flight area
☐ Safety glasses on
```

### Post-Crash Checklist

```
☐ Disconnect battery immediately
☐ Check for damaged propellers (replace if cracked)
☐ Check for bent motor shafts (spin by hand)
☐ Check for loose screws
☐ Check for broken wires/solder joints
☐ Check frame for cracks
☐ Recalibrate IMU
☐ Test motors without props
☐ Gradual test flight
```

---

## Getting Help

### Information to Provide

When asking for help, include:

1. **Exact symptom** (what happens, when it happens)
2. **Serial monitor output** (copy/paste error messages)
3. **Photo of wiring** (clear, well-lit)
4. **Code modifications** (if you changed anything)
5. **Component list** (exact models used)
6. **What you've tried** (previous troubleshooting steps)

### Useful Serial Monitor Output

```
Expected startup sequence:
─────────────────────────────
=== Flight Controller Starting ===
Initializing MPU6050...
MPU6050 connected!
Initializing MS5611...
MS5611 connected!
Initializing nRF24L01+...
Radio initialized!
Initializing ESCs...
=== Ready for Calibration ===
```

If any step fails, that's your starting point for troubleshooting!

---

## Common Error Messages

| Error Message | Meaning | Solution |
|---------------|---------|----------|
| "MPU6050 connection failed!" | IMU not detected | Check I2C wiring, address |
| "Radio hardware not responding!" | nRF24 not detected | Check 3.3V power, wiring |
| "Transmission failed!" | Radio packet lost | Add capacitor, change channel |
| "EMERGENCY DISARM: Extreme tilt!" | Angle >45° | Check motor directions, props |
| "FAILSAFE" | Signal lost >1sec | Move closer, check radio |

---

**Remember: Safety First!**
**When in doubt, DISARM and troubleshoot on the ground!**
