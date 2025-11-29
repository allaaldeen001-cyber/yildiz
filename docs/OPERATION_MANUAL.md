# Operation Manual - Professional Drone System

## 🎯 Pre-Flight Checklist

### Hardware Inspection
- [ ] All propellers securely attached and undamaged
- [ ] Motor screws tight
- [ ] Battery fully charged (>11.4V for 3S)
- [ ] All wiring secure and insulated
- [ ] Arduino Nano firmly mounted
- [ ] MPU6050 and MS5611 sensors clean and secure
- [ ] NRF24L01 antenna not damaged
- [ ] No loose components on frame
- [ ] Landing gear/legs stable

### Software Checks
- [ ] Flight controller code uploaded successfully
- [ ] Remote controller code uploaded successfully
- [ ] NRF channel set to 103 on both boards
- [ ] Serial monitor working on RC (115200 baud)
- [ ] No compilation errors

---

## 🚀 Startup Procedure

### Step 1: Power On Remote Controller
1. Connect 9V battery to remote controller
2. Open Serial Monitor (115200 baud)
3. Wait for "NRF24L01 Initialized" message
4. Verify all joysticks at neutral position
5. Ensure **SW_2 (Arm Switch)** is **OFF** (position 0)

**Expected Serial Output:**
```
╔════════════════════════════════════════╗
║   DRONE REMOTE CONTROLLER v1.0        ║
║   Professional UAV Control System     ║
╚════════════════════════════════════════╝

Initializing NRF24L01... OK
RF Channel: 103
Waiting for Flight Controller...
```

---

### Step 2: Power On Flight Controller
1. Connect main battery (3S/4S LiPo) to PDB/ESCs
2. Arduino Nano powers on via ESC BEC
3. Listen for **startup beeps**: 
   - 2 short beeps = system ready
4. Watch LED (D7):
   - LED off = no RC connection
   - LED blinking = RC connected

**Flight Controller Startup Sequence:**
```
=== DRONE FLIGHT CONTROLLER ===
Initializing systems...
✓ NRF24L01 initialized
✓ MPU6050 initialized
✓ MS5611 initialized
✓ Motors initialized
System ready. Waiting for RC connection...
```

---

### Step 3: Verify Connection
1. Check RC serial monitor
2. Look for "CONNECTED TO FLIGHT CONTROLLER"
3. LED on flight controller should **blink continuously** (1Hz)
4. RC display shows telemetry data

**RC Display When Connected:**
```
  CONNECTION: ✓ LINKED

┌─── FLIGHT CONTROLLER STATUS ───────────┐
│  Armed:      NO
│  Calibrated: NO ✗
│  Battery:    11.2 V
│  Altitude:   0.0 m
└────────────────────────────────────────┘
```

**⚠️ Troubleshooting**: If no connection after 10 seconds:
- Power cycle both boards
- Check NRF24L01 wiring and power (3.3V)
- Verify both using channel 103
- Add capacitor to NRF24L01 power pins

---

### Step 4: Gyro Calibration
**CRITICAL**: Place drone on **perfectly level surface**. Do not move during calibration.

1. Verify **SW_2 (Arm Switch)** is at position **1** (ON)
2. Press and hold **Button_1 (Calibration)** for 1 second
3. Release button
4. Buzzer beeps once (200ms) - calibration started
5. LED blinks rapidly during calibration (~6 seconds)
6. Wait for completion:
   - **Success**: 2 short beeps (100ms each)
   - **Failure**: 1 long continuous beep (7 seconds)

**RC Serial Monitor During Calibration:**
```
>>> CALIBRATION REQUESTED <<<

=== GYRO CALIBRATION ===
Keep drone on level surface...
✓ Calibration SUCCESS!
Gyro offsets: X=0.23 Y=-0.15 Z=0.08
```

**Success Criteria:**
- Gyro offsets must be < ±10°/s
- Drone completely stationary
- Surface perfectly level

**If Calibration Fails:**
- Ensure surface is truly level
- Remove any vibrations (turn off fans, etc.)
- Check MPU6050 wiring
- Wait 30 seconds and retry

---

### Step 5: ESC Calibration (First Time Setup Only)
**Only needed when:**
- First time using ESCs
- Changing ESCs
- Motors not responding correctly

**Procedure:**
1. Set **SW_1 (Alt Hold)** to position **0** (OFF)
2. Press and hold **Button_2 (ESC Cal)** for 1 second
3. Release button
4. Listen for **3 medium beeps** (150ms each) - ESC cal started
5. Each motor spins up **one by one smoothly**:
   - Motor FL (Front Left)
   - Motor FR (Front Right)
   - Motor RR (Rear Right)
   - Motor RL (Rear Left)
6. After each motor spins, buzzer beeps once
7. Final sequence: **beep - pause - beep - pause - long beep** = complete

**RC Serial Monitor:**
```
>>> ESC CALIBRATION REQUESTED <<<

=== ESC CALIBRATION ===
Motor FL...
Motor FR...
Motor RR...
Motor RL...
✓ ESC Calibration complete!
```

**⚠️ Safety**: 
- Remove propellers during ESC calibration!
- Keep hands away from motors
- Ensure drone is secured

---

### Step 6: Arming the Drone
**Prerequisites:**
- [ ] Gyro calibration successful
- [ ] RC connected (LED blinking)
- [ ] Battery voltage >11.1V (3S) or >14.4V (4S)
- [ ] **Throttle at MINIMUM** (left stick down)

**Arming Procedure:**
1. Verify **left joystick (throttle)** is at bottom position
2. RC shows: `Throttle: 1000` or close to it
3. Flip **SW_2 (Arm Switch)** to position **1** (ON)
4. Listen for **2 quick beeps** (50ms each) - ARMED
5. LED changes pattern (may blink faster)
6. RC display shows: `Armed: YES ⚠`

**RC Display When Armed:**
```
┌─── FLIGHT CONTROLLER STATUS ───────────┐
│  Armed:      YES ⚠
│  Calibrated: YES ✓
│  Battery:    11.2 V
│  Altitude:   0.5 m
└────────────────────────────────────────┘

  ✓ ARMED - Fly safe!
```

**Cannot Arm - Troubleshooting:**

If you hear **1 long beep (1 second)** instead:
- **Problem**: Throttle too high
- **Solution**: Lower left joystick to minimum
- **Verify**: RC shows `Throttle: 1000-1100`

If nothing happens:
- Check calibration completed successfully
- Verify SW_2 switch is working (check RC display)
- Ensure battery voltage sufficient
- Power cycle and retry

---

### Step 7: Flight Test
**First Flight Safety:**
- Clear area of obstacles and people
- Fly in open space (>10m radius)
- Start with gentle movements
- Keep drone within visual line of sight
- Have emergency landing zone identified

**Basic Flight Controls:**

#### Left Joystick (Throttle + Yaw)
```
        UP = Climb
         |
    LEFT-+-RIGHT
    CCW  |  CW
         |
       DOWN = Descend
```

| Movement | Action | Drone Response |
|----------|--------|----------------|
| Push UP | Increase throttle | Climbs vertically |
| Push DOWN | Decrease throttle | Descends vertically |
| Push LEFT | Yaw counter-clockwise | Rotates left (nose turns left) |
| Push RIGHT | Yaw clockwise | Rotates right (nose turns right) |

#### Right Joystick (Pitch + Roll)
```
      UP = Forward
         |
    LEFT-+-RIGHT
  Slide L| Slide R
         |
     DOWN = Backward
```

| Movement | Action | Drone Response |
|----------|--------|----------------|
| Push UP | Pitch forward | Tilts and moves forward |
| Push DOWN | Pitch backward | Tilts and moves backward |
| Push LEFT | Roll left | Tilts and slides left |
| Push RIGHT | Roll right | Tilts and slides right |

---

## 🎮 Advanced Features

### Altitude Hold Mode
**Function**: Maintains current altitude automatically using barometric sensor.

**How to Use:**
1. Fly drone to desired altitude
2. Flip **SW_1 (Alt Hold)** to position **1** (ON)
3. Buzzer beeps once to confirm
4. RC shows: `SW_1 (Alt Hold): ON ✓`
5. Release throttle stick - drone maintains altitude
6. Use pitch/roll/yaw normally
7. To exit: Flip SW_1 to position **0** (OFF)

**RC Display:**
```
│  SW_1 (Alt Hold):  ON  ✓
│  Altitude:   2.5 m    (locked at 2.5m)
```

**Altitude Hold Behavior:**
- Throttle stick **overrides** altitude hold
- Push up = climbs from setpoint
- Push down = descends from setpoint
- Release stick = returns to setpoint
- Maximum correction: ±2m before needing throttle input

**Limitations:**
- Barometer has ~0.5m accuracy
- Indoor altitude hold less reliable (air pressure changes)
- Wind can affect performance
- Battery voltage drop affects performance

---

### Emergency Stop (Kill Switch)
**Function**: Immediately disarms and stops all motors.

**How to Use:**
1. During flight, flip **SW_2 (Arm Switch)** to position **0** (OFF)
2. Motors stop **immediately**
3. Drone **will fall** - use only in emergency!
4. Buzzer sounds long beep

**When to Use:**
- ⚠️ Loss of control
- ⚠️ Drone flying toward obstacles/people
- ⚠️ Unexpected behavior
- ⚠️ Hardware failure detected

**DO NOT USE** for normal landings - gradually reduce throttle instead.

---

### Signal Loss Failsafe
**Automatic Protection**: If RC signal lost for >1 second:
1. Flight controller **automatically disarms**
2. Motors stop immediately
3. LED turns off
4. Buzzer sounds (if still powered)

**Recovery:**
1. Restore RC connection
2. Check for damage
3. Perform full pre-flight checklist again
4. Re-calibrate if drone was impacted

---

## 📊 Reading the RC Display

### Connection Status
```
  CONNECTION: ✓ LINKED      → Good connection
  CONNECTION: ✗ SEARCHING... → No FC connection
```

### Flight Controller Status Section
```
│  Armed:      YES ⚠   → Motors can spin!
│  Armed:      NO      → Safe, motors off
│  Calibrated: YES ✓   → Ready to fly
│  Calibrated: NO ✗    → Must calibrate first
│  Battery:    11.2 V  → Monitor this closely!
│  Altitude:   2.5 m   → Height above start point
│  Attitude:   Roll=5° Pitch=-3° Yaw=45° → Tilt angles
```

### Control Inputs Section
```
│  Throttle:   1523 [████████████░░░░░░░░]
│  Yaw:         125 [░░░░░░░░░░|█░░░░░░░░]
│  Pitch:      -230 [░░░░░█████|░░░░░░░░░]
│  Roll:         0  [░░░░░░░░░░|░░░░░░░░░]
```
- Bars show stick position visually
- Numbers show exact PWM/control values
- Center line `|` shows neutral position

### Switches & Buttons Section
```
│  SW_1 (Alt Hold):  ON  ✓   → Altitude hold active
│  SW_2 (Arm/Kill):  ON  ⚠   → Armed (dangerous!)
│  BTN_1 (Calib):    PRESSED → Currently pressed
│  BTN_2 (ESC Cal):  Released → Not pressed
```

---

## 🔋 Battery Management

### Voltage Monitoring
**3S LiPo (11.1V nominal):**
| Voltage | Status | Action |
|---------|--------|--------|
| >12.0V | Full charge | Safe to fly |
| 11.4-12.0V | Good | Normal operation |
| 11.1-11.4V | Medium | Monitor closely |
| 10.8-11.1V | Low | Land soon |
| <10.8V | Critical | **Land immediately!** |

**4S LiPo (14.8V nominal):**
| Voltage | Status | Action |
|---------|--------|--------|
| >16.0V | Full charge | Safe to fly |
| 15.2-16.0V | Good | Normal operation |
| 14.8-15.2V | Medium | Monitor closely |
| 14.4-14.8V | Low | Land soon |
| <14.4V | Critical | **Land immediately!** |

**⚠️ Warning**: Never discharge LiPo below 3.0V per cell (9.0V for 3S, 12.0V for 4S) or battery will be permanently damaged!

### Flight Time Estimation
**Typical 3S 2200mAh LiPo:**
- Hover: ~8-12 minutes
- Aggressive flying: ~5-7 minutes
- With altitude hold: ~7-10 minutes

**Battery Safety:**
- Always use LiPo-safe charging bag
- Never leave charging unattended
- Store at 3.8V per cell for long-term storage
- Dispose of damaged/puffed batteries properly

---

## 🛠️ In-Flight Adjustments

### PID Tuning (Advanced Users)
If drone is unstable, oscillates, or sluggish, you may need to tune PID values in `FlightController.ino`:

**Current Default Values:**
```cpp
#define KP_ROLL   1.5
#define KI_ROLL   0.05
#define KD_ROLL   15.0

#define KP_PITCH  1.5
#define KI_PITCH  0.05
#define KD_PITCH  15.0

#define KP_YAW    3.0
#define KI_YAW    0.02
#define KD_YAW    0.0
```

**Tuning Guide:**
- **Oscillating/Vibrating**: Decrease KP, increase KD
- **Sluggish Response**: Increase KP
- **Drifting Over Time**: Increase KI (carefully!)
- **Overshooting**: Increase KD

**Safe Tuning Steps:**
1. Change one value at a time
2. Change by 10-20% increments
3. Test in safe environment
4. Document changes

---

## 🚨 Emergency Procedures

### Loss of Control
1. **Immediately** flip SW_2 to OFF (kill switch)
2. Motors stop, drone falls
3. Check for damage before next flight

### Flyaway (Drone Flying Away)
1. Flip SW_2 to OFF immediately
2. Note direction of flyaway
3. Check NRF24L01 connection and antenna

### Motor Not Spinning
1. Disarm (SW_2 to OFF)
2. Check motor/ESC connections
3. Verify ESC calibration
4. Test motors individually (propellers off!)

### Erratic Behavior
1. Land immediately (reduce throttle)
2. Disarm
3. Check sensor calibration
4. Inspect for loose wiring

### Smoke or Burning Smell
1. **IMMEDIATELY** disconnect battery
2. Move to safe area
3. Do not reconnect power
4. Inspect for shorts or damaged components

---

## 📝 Post-Flight Checklist

- [ ] Disarm drone (SW_2 to OFF)
- [ ] Disconnect main battery
- [ ] Check frame for cracks
- [ ] Inspect propellers for damage
- [ ] Check motor screws
- [ ] Verify all wiring intact
- [ ] Clean dust/debris from sensors
- [ ] Check battery voltage and condition
- [ ] Charge battery to storage voltage (3.8V/cell)
- [ ] Log flight time in notebook

---

## 📈 Flight Logging (Recommended)

Keep a log for each flight:
```
Date: 2025-11-29
Flight #: 12
Duration: 8:32
Battery: 3S 2200mAh (11.8V start, 11.2V end)
Weather: Clear, light wind
Notes: Tested altitude hold, worked well. 
       Slight drift to left, may need roll trim.
Issues: None
```

This helps track performance and identify patterns.

---

## 🎓 Pilot Skill Progression

### Beginner (Flights 1-10)
- Focus on: Hover, gentle climbs/descents
- Master: Throttle control, basic orientation
- Avoid: Fast movements, high altitude

### Intermediate (Flights 11-30)
- Focus on: Forward/backward flight, controlled turns
- Master: All joystick inputs, smooth transitions
- Practice: Figure-8 patterns, altitude hold

### Advanced (Flights 31+)
- Focus on: Precision flying, quick maneuvers
- Master: Emergency procedures, PID tuning
- Explore: FPV integration, GPS modules

---

**Happy Flying! 🚁**

*Always fly responsibly and follow local UAV regulations.*
