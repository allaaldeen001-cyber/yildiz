# 🎯 Comprehensive Calibration Guide

## Table of Contents
1. [When to Calibrate](#when-to-calibrate)
2. [MPU6050 Calibration](#mpu6050-calibration)
3. [MS5611 Calibration](#ms5611-calibration)
4. [ESC Calibration](#esc-calibration)
5. [RC Joystick Calibration](#rc-joystick-calibration)
6. [Propeller Balancing](#propeller-balancing)
7. [Verification Procedures](#verification-procedures)

---

## When to Calibrate

### ✅ Required Calibration

| Situation | Components to Calibrate |
|-----------|------------------------|
| **First Setup** | All (MPU6050, MS5611, ESCs, Joysticks) |
| **After Crash** | MPU6050, check ESCs |
| **Drifting in Hover** | MPU6050 |
| **Altitude Hold Issues** | MS5611 |
| **Motors Not Responding** | ESCs |
| **RC Control Erratic** | Joysticks |
| **Vibration** | Balance propellers |

### ❌ Not Required

- Normal battery changes
- Software updates (unless EEPROM cleared)
- Moving between locations
- Daily flights

---

## MPU6050 Calibration

### What It Does

Calibrates the gyroscope and accelerometer to:
- Eliminate sensor drift
- Set level reference (0° when flat)
- Remove manufacturing offsets

### Prerequisites

- ✅ Drone on perfectly level surface
- ✅ Completely still (no vibrations)
- ✅ RC and FC powered and linked
- ✅ Disarmed (Switch 1 = 1)

### Step-by-Step Procedure

#### 1. Prepare Surface

```
Use a spirit level to verify surface is perfectly flat:

  [Spirit Level]
  ──────────────
       ○
  ══════════════  ← Check both axes
```

**Tips:**
- Large table or floor
- Away from HVAC vents
- No foot traffic nearby
- Solid foundation (not wobbly desk)

#### 2. Position Drone

```
Place drone flat with:
- Propellers off (recommended)
- Arrow/front clearly marked
- All 4 legs touching surface
- Not tilted in any direction

     [FC Board]
        ↑ Front
   FL [✕] FR
      \│/
      ─┼─
      /│\
   RL [✕] RR
```

#### 3. Initiate Calibration

**Via RC Controller:**
1. Press and hold **Button 1**
2. Hold for 2 seconds
3. Release when you hear beep pattern

**Serial Monitor Shows:**
```
=== CALIBRATION START ===
Calibrating MPU6050...
Gyro calibration... Keep drone still!
[Progress dots appear]
Calibrating level... Keep drone level!
[More progress]
=== CALIBRATION COMPLETE ===
Loaded calibration: X=0.45 Y=-0.32
Calibration saved to EEPROM
```

#### 4. What Happens During Calibration

**Phase 1: Gyro Zero-Point (3 seconds)**
```
Samples: 1500 readings
Purpose: Find gyro offset when not rotating
Requirements: Absolutely still!
```

**Phase 2: Level Reference (5 seconds)**
```
Samples: 1000 readings
Purpose: Calculate angle offset for level flight
Requirements: Perfectly level surface
```

**Phase 3: EEPROM Storage**
```
Saves: X and Y offsets
Location: EEPROM addresses 10 and 15
Persistence: Survives power cycles
```

#### 5. Verify Calibration

**Check Serial Output:**
```
Loaded calibration: X=0.45 Y=-0.32

✅ GOOD: Values between -3.0 and +3.0
⚠️  WARNING: Values between 3.0 and 5.0 (may work but check surface)
❌ BAD: Values > 5.0 (recalibrate on level surface!)
```

**Check Angles in Flight:**
```
Hover drone and check serial monitor:
Angles: X=0±2° Y=0±2°

If drifting heavily (>5°), recalibrate.
```

### Troubleshooting MPU6050

#### Issue: Calibration Values Too High (>5°)

**Causes:**
- Surface not level
- Drone moved during calibration
- Frame bent/damaged

**Solutions:**
1. Verify surface with spirit level
2. Recalibrate - keep absolutely still
3. Check frame for damage
4. Try different surface

#### Issue: Drone Drifts After Calibration

**Causes:**
- Calibrated on uneven surface
- MPU6050 damaged
- Vibration from motors

**Solutions:**
1. Recalibrate on verified level surface
2. Add vibration damping under FC
3. Balance propellers
4. Check MPU6050 orientation (chip facing up)

#### Issue: Calibration Doesn't Save

**Causes:**
- EEPROM write failure
- Arduino connection issue

**Solutions:**
```cpp
// Verify EEPROM in code:
EEPROM.get(10, cal.x);
EEPROM.get(15, cal.y);
Serial.print("Stored X: ");
Serial.println(cal.x);
```

### Manual MPU6050 Calibration (Advanced)

If automatic calibration fails:

#### Method 1: Serial Monitor Method

```cpp
// Add to setup() for one-time calibration:
void setup() {
  Serial.begin(57600);
  gyro.SetupWire(timepi);
  
  Serial.println("Place drone level, then send 'c' via serial");
  while(Serial.read() != 'c'); // Wait for command
  
  cal = gyro.calibrate(2000); // 2000 samples
  EEPROM.put(10, cal.x);
  EEPROM.put(15, cal.y);
  
  Serial.println("Calibration saved!");
}
```

#### Method 2: Manual Offset Entry

```cpp
// If you know good values, hardcode them:
void setup() {
  cal.x = 0.45;  // Your measured offset
  cal.y = -0.32; // Your measured offset
  EEPROM.put(10, cal.x);
  EEPROM.put(15, cal.y);
}
```

---

## MS5611 Calibration

### What It Does

Measures ground-level atmospheric pressure for:
- Altitude hold reference point
- Relative altitude calculation
- Barometer offset compensation

### Prerequisites

- ✅ MPU6050 already calibrated
- ✅ Drone on ground at takeoff location
- ✅ Disarmed (Switch 1 = 1)

### Step-by-Step Procedure

#### 1. Position for Calibration

```
Place drone:
- At intended takeoff location
- Away from prop wash
- Not in direct sunlight
- Shield MS5611 from wind

   [Foam Cover]    ← Optional but helps
      ┌───┐
      │MS │
      │561│
      │1  │
      └───┘
   [FC Board]
```

#### 2. Initiate Calibration

Same as MPU6050 - **Button 1** for 2 seconds

**Serial Shows:**
```
Calibrating MS5611...
[Short delay]
Ground pressure: 1013.25 hPa
```

#### 3. What Happens

```
Samples: 50 readings over 1 second
Calculation: Average pressure = Ground reference
Storage: RAM only (not EEPROM)
```

**Note:** MS5611 calibration resets on power cycle. Always calibrate before altitude hold flights.

#### 4. Verify Calibration

**Check Pressure Reading:**
```
Serial Monitor shows:
actual_pressure= 1013.25
actual_pressure_2= 1013.30  ← Kalman filtered

✅ GOOD: Values between 950-1050 hPa (typical range)
⚠️  CHECK: Values outside this range (sensor issue or extreme altitude)
```

**Test Altitude Hold:**
1. Arm and takeoff to 1m height
2. Set throttle to 1425 (mid-range)
3. Flip Switch 2 (altitude hold ON)
4. Release throttle to center
5. Drone should maintain height ±20cm

### Troubleshooting MS5611

#### Issue: Pressure Readings Unstable

**Symptoms:**
- Values jumping >5 hPa per second
- Altitude hold very jittery

**Solutions:**
1. **Shield from prop wash:**
   ```
   Add foam cover or tube:
   
   ┌────┐
   │    │← Foam
   │ MS │
   │5611│
   └────┘
   ```

2. **Check I2C connection:**
   - Verify SDA/SCL wiring
   - Measure voltage (should be 3.3V or 5V)
   - Try different I2C address if available

3. **Software smoothing:**
   ```cpp
   // Increase smoothing samples:
   smooth.begin(SMOOTHED_AVERAGE, 20); // Default is 10
   ```

#### Issue: Altitude Drifts Over Time

**Symptoms:**
- Altitude hold works initially, then drifts up/down
- Pressure reading slowly changes

**Causes:**
- Temperature change in barometer
- Weather pressure change
- Barometer heating from ESCs/battery

**Solutions:**
1. **Recalibrate more often** (every flight)
2. **Relocate sensor** away from heat sources
3. **Add insulation** around MS5611
4. **Use Kalman filter** (already in code)

#### Issue: Altitude Hold Too Aggressive

**Symptoms:**
- Drone oscillates vertically
- Rapid throttle changes

**Solutions:**
1. **Reduce PID gains:**
   ```cpp
   float pid_p_gain_altitude = 10.0;  // Reduce from 14.0
   float pid_d_gain_altitude = 5.0;   // Reduce from 7.5
   ```

2. **Increase smoothing** (see above)

3. **Limit PID output:**
   ```cpp
   int pid_max_altitude = 300;  // Reduce from 400
   ```

---

## ESC Calibration

### What It Does

Teaches ESCs the PWM range (1000-2000µs) so all motors respond identically.

### When Required

- ✅ First setup with new ESCs
- ✅ ESCs from different manufacturers
- ✅ Motors not responding correctly
- ✅ One motor spins faster/slower than others

### Prerequisites

- ⚠️ **REMOVE PROPELLERS!** (Safety critical)
- ✅ ESCs connected to Arduino
- ✅ Battery disconnected initially

### Method 1: All-At-Once Calibration

#### Step-by-Step

1. **Disconnect Arduino from ESCs:**
   ```
   Remove signal wires from D3, D5, D6, D9
   Leave power (5V, GND) connected
   ```

2. **Manual Throttle Signal:**
   ```
   Option A: Use servo tester
   Option B: Temporary calibration sketch
   ```

3. **Upload Calibration Sketch:**
   ```cpp
   #include <Servo.h>
   
   Servo esc1, esc2, esc3, esc4;
   
   void setup() {
     Serial.begin(57600);
     esc1.attach(3, 1000, 2000);
     esc2.attach(5, 1000, 2000);
     esc3.attach(6, 1000, 2000);
     esc4.attach(9, 1000, 2000);
     
     Serial.println("Disconnect battery!");
     Serial.println("Send 'H' for HIGH, 'L' for LOW calibration");
   }
   
   void loop() {
     if (Serial.available()) {
       char cmd = Serial.read();
       if (cmd == 'H') {
         // Send maximum throttle
         esc1.writeMicroseconds(2000);
         esc2.writeMicroseconds(2000);
         esc3.writeMicroseconds(2000);
         esc4.writeMicroseconds(2000);
         Serial.println("MAX throttle sent. Connect battery now!");
         Serial.println("Wait for beeps, then send 'L'");
       }
       else if (cmd == 'L') {
         // Send minimum throttle
         esc1.writeMicroseconds(1000);
         esc2.writeMicroseconds(1000);
         esc3.writeMicroseconds(1000);
         esc4.writeMicroseconds(1000);
         Serial.println("MIN throttle sent. Wait for confirmation beeps.");
         Serial.println("Calibration complete! Re-upload flight code.");
       }
     }
   }
   ```

4. **Calibration Sequence:**
   ```
   1. Upload calibration sketch
   2. Open Serial Monitor
   3. Send 'H' → ESCs receive 2000µs
   4. Connect battery
   5. ESCs beep (high tones)
   6. Send 'L' → ESCs receive 1000µs
   7. ESCs beep (musical tones = success)
   8. Disconnect battery
   9. Re-upload normal flight code
   ```

### Method 2: Individual ESC Calibration

Some ESCs have programming cards:

1. **Connect ESC to programming card**
2. **Follow card instructions** to set:
   - Min: 1000µs
   - Max: 2000µs
   - Mode: Fixed wing (no brake)
3. **Repeat for all 4 ESCs**

### Verification

After calibration:

```cpp
// Test sketch to verify:
void loop() {
  int test_value = 1200;  // Safe test speed
  
  esc1.writeMicroseconds(test_value);
  esc2.writeMicroseconds(test_value);
  esc3.writeMicroseconds(test_value);
  esc4.writeMicroseconds(test_value);
  
  // All motors should spin at IDENTICAL speed
  // If not, one ESC failed calibration
}
```

### Troubleshooting ESC

#### Issue: Motors Won't Start After Calibration

**Possible causes:**
- Calibration range wrong
- ESC expecting different protocol

**Solutions:**
1. Check ESC manual for correct range (some use 1100-1900)
2. Try re-calibration
3. Update ESC firmware (BLHeli/SimonK)

#### Issue: One Motor Different Speed

**After calibration, one motor still spins faster/slower:**

1. **Verify motor KV rating** - must all match
2. **Check propeller** - damaged props create unequal thrust
3. **Recalibrate that ESC individually**
4. **Replace ESC** if problem persists

---

## RC Joystick Calibration

### What It Does

Maps joystick analog readings (0-1023) to control values (-100 to +100).

### When Required

- ✅ First setup
- ✅ Controls feel "off" or reversed
- ✅ Drone responds even with centered sticks
- ✅ Not reaching full tilt/throttle range

### Step-by-Step

#### 1. Find Center Values

```cpp
// Upload test sketch to RC controller:
void setup() {
  Serial.begin(57600);
}

void loop() {
  Serial.print("Throttle: "); Serial.print(analogRead(A0));
  Serial.print("\t Yaw: "); Serial.print(analogRead(A1));
  Serial.print("\t Pitch: "); Serial.print(analogRead(A2));
  Serial.print("\t Roll: "); Serial.println(analogRead(A3));
  delay(100);
}
```

**Record values with sticks centered:**
```
Throttle: 512  ← Should be ~512
Yaw:      507
Pitch:    510
Roll:      515
```

#### 2. Find Min/Max Values

**Move each stick to extremes and record:**
```
Stick       MIN    CENTER   MAX
──────────────────────────────────
Throttle    0      512      1023
Yaw         5      507      1018
Pitch       2      510      1020
Roll        10     515      1020
```

#### 3. Calculate Calibration Values

**Formula:**
```
calValue = -centerReading
scaleValue = desiredRange / (maxReading - minReading)
```

**Example for Roll:**
```
Center = 515
Min = 10
Max = 1020
Desired output range = ±100

calRoll = -515
scaleRoll = 100 / (1020 - 10) = 0.099
```

#### 4. Update RC Controller Code

```cpp
// In RC_Controller.ino:

float scaleRoll = 0.099;
float calRoll = -515;

float scalePitch = -0.099;  // Note: negative for inversion
float calPitch = -510;

float scaleYaw = -0.1;
float calYaw = -507;

float scaleThrust = 1.5;    // Different scale for throttle
float calThrust = -512;
float offsetThrust = 1000;  // Minimum PWM value
```

### Verification

After calibration:

**Serial Monitor Output Should Show:**
```
With sticks centered:
Roll: 0±2   Pitch: 0±2   Yaw: 0±2   Thrust: 1000-1100

With sticks at max:
Roll: +100   Pitch: +100   Yaw: +100   Thrust: 2000

With sticks at min:
Roll: -100   Pitch: -100   Yaw: -100   Thrust: 1000
```

---

## Propeller Balancing

### What It Does

Ensures propellers are perfectly balanced to reduce vibration.

### Why Important

Vibration causes:
- Jittery flight
- Poor altitude hold
- Faster motor/bearing wear
- IMU noise

### Equipment Needed

```
Propeller Balancer:

    ┌─────────┐
    │  [Prop] │
    │    │    │
    │    ▼    │
    ├─────────┤
    │  ═══════│← Bearing rails
    └─────────┘

Cost: $8-15
Alternative: Precision balance
```

### Step-by-Step

#### 1. Test Each Propeller

```
Place propeller on balancer:
- Heavier side will tilt down
- Perfectly balanced = stays horizontal
```

#### 2. Balance Methods

**Method A: Add Weight**
```
Use tiny bits of tape on light side:
  
   Light side
      ↓
  [═══│tape══]
      ↑
  Heavy side

Gradually add until balanced
```

**Method B: Remove Weight**
```
Sand heavy side with fine sandpaper:
- Very light pressure
- Check frequently
- Don't sand leading/trailing edge
```

#### 3. Verify All Props

Test each prop:
- Rotate prop on balancer
- Should stay horizontal in any position
- If tilts, repeat balancing

#### 4. Test Flight

After balancing:
- Less vibration in hover
- Smoother video (if FPV)
- More stable flight

---

## Verification Procedures

### Complete System Check

After all calibrations:

#### 1. Static Tests (Props OFF)

```
✅ MPU6050:
   - Place drone level
   - Serial shows: X=0±2° Y=0±2°
   
✅ MS5611:
   - Serial shows pressure reading
   - Value stable (±1 hPa)
   
✅ ESCs:
   - Motor test shows all spin
   - Equal speeds at same throttle
   
✅ RC Link:
   - All controls responsive
   - No deadbands or sticking
```

#### 2. Dynamic Tests (Props ON, Careful!)

```
✅ Hover Test:
   - Takeoff to 30cm
   - Minimal stick input needed to stay level
   - No drifting >1m/min
   
✅ Response Test:
   - Small roll input → quick but smooth response
   - Release stick → returns to level
   - No oscillation
   
✅ Altitude Hold Test:
   - Hover at 1m
   - Enable altitude hold
   - Maintains altitude ±20cm
```

#### 3. Data Validation

Open Serial Monitor during flight:

```
GOOD readings:
┌──────────────────────────────────────┐
│ Pressure: 1013 hPa                   │
│ Angles: X=1° Y=-2° Z=45°             │
│ Motors: FL=1420 FR=1425 RL=1418 ...  │
│ Loop: 7.1ms                          │
└──────────────────────────────────────┘

Motor values should be:
- Similar (within 50µs)
- Changing smoothly
- Not hitting limits constantly
```

---

## Calibration Schedule

### Recommended Calibration Frequency

| Component | First Setup | After Crash | Weekly | Notes |
|-----------|-------------|-------------|--------|-------|
| **MPU6050** | ✅ Required | ✅ Required | ⚪ Optional | Or when drifting |
| **MS5611** | ✅ Required | ⚪ Check | ✅ Recommended | Each flight location |
| **ESCs** | ✅ Required | ⚪ Check | ❌ Not needed | Only if motors act weird |
| **Joysticks** | ✅ Required | ❌ Not needed | ❌ Not needed | Very stable |
| **Props** | ✅ Required | ✅ Required | ⚪ Check | After each prop change |

---

## Quick Reference Card

```
╔═══════════════════════════════════════════════════════╗
║            CALIBRATION QUICK REFERENCE                ║
╠═══════════════════════════════════════════════════════╣
║ Component      Button/Method         Time     Props  ║
╠═══════════════════════════════════════════════════════╣
║ MPU6050        Hold Button 1 (2s)    5s       OFF    ║
║ MS5611         (Same as MPU6050)     1s       OFF    ║
║ ESCs           Calibration sketch    1min     OFF!   ║
║ Joysticks      Edit code values      N/A      N/A    ║
║ Props          Balance each one      5min     N/A    ║
╚═══════════════════════════════════════════════════════╝

Remember:
- Always on level surface for IMU
- Always props OFF for motor tests
- Always recalibrate after crashes
```

---

**Proper calibration = Stable flight!** ✈️
