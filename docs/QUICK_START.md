# Quick Start Guide

Get your quadcopter drone flying in under 30 minutes!

---

## Prerequisites

Before you begin:
- ✅ All hardware components assembled
- ✅ Wiring completed (see [WIRING_GUIDE.md](WIRING_GUIDE.md))
- ✅ Arduino IDE installed
- ✅ Required libraries installed (see [LIBRARIES.md](LIBRARIES.md))
- ✅ Battery fully charged
- ✅ **PROPELLERS REMOVED** (safety first!)

---

## Step 1: Upload Code (10 minutes)

### Remote Controller

1. Open Arduino IDE
2. Open `RemoteController/RemoteController.ino`
3. Select **Tools → Board → Arduino Nano**
4. Select **Tools → Processor → ATmega328P (Old Bootloader)**
   - Try this first; if upload fails, try "ATmega328P"
5. Select **Tools → Port** → Your Arduino's COM port
6. Click **Upload** (→)
7. Wait for "Done uploading"

### Flight Controller

1. Open `FlightController/FlightController.ino`
2. Select same board settings as above
3. Select correct COM port (different from RC)
4. Click **Upload** (→)
5. Wait for "Done uploading"

✅ **Verification**: Both should compile without errors.

---

## Step 2: Power On & Test (5 minutes)

### Remote Controller

1. Connect power (9V battery or USB)
2. You should hear nothing (no buzzer on RC)
3. Open **Serial Monitor** (Ctrl+Shift+M)
4. Set baud rate to **115200**
5. You should see:
   ```
   ================================
     DRONE REMOTE CONTROLLER v1.0
   ================================
   ✓ Remote Controller Ready
   ```

6. Test joysticks - move them and watch values change in Serial Monitor

### Flight Controller

1. **ENSURE PROPELLERS ARE REMOVED**
2. Connect battery to ESCs (or power via USB for testing)
3. You should hear:
   - **Beep-beep-beeeep** (startup tone)
4. Status LED should light up solid
5. Open Serial Monitor (115200 baud)
6. You should see:
   ```
   ================================
     FLIGHT CONTROLLER v1.0
   ================================
   ✓ MPU6050 Initialized
   ✓ NRF24L01 Initialized
   ```

✅ **Verification**: Both boards powered on and responding.

---

## Step 3: Link Remote & Flight Controller (2 minutes)

1. **Both boards powered on**
2. Watch Flight Controller LED:
   - Should **blink slowly** (once per second)
   - This indicates NRF link is active
3. Check RC Serial Monitor:
   - Should show `NRF: LINKED ✓`

✅ **Verification**: NRF communication established.

---

## Step 4: Gyro Calibration (2 minutes)

**CRITICAL**: Must be done before first flight!

1. Place drone on **flat, level surface**
2. **DO NOT MOVE** the drone
3. On Remote Controller:
   - Set **SW2 (Arming Switch)** to position "1"
   - Press **Button 1** (Calibration)
4. Watch Serial Monitor:
   - Should show "Calibrating..."
5. Wait 8 seconds (LED will blink rapidly)
6. Listen for buzzer:
   - **2 beeps** = ✅ Success
   - **1 long beep** = ❌ Failed (retry)

✅ **Verification**: Buzzer beeps twice, Serial shows "Calibration SUCCESS".

---

## Step 5: ESC Calibration (3 minutes)

**CRITICAL**: First-time setup or after changing ESCs.

1. **PROPELLERS STILL REMOVED**
2. On Remote Controller:
   - Set **SW1** to position "0"
   - Press **Button 2** (ESC Calibration)
3. You'll hear unique buzzer tone pattern
4. Motors will spin up **one by one** slowly:
   - Front Left (FL)
   - Front Right (FR)
   - Rear Right (RR)
   - Rear Left (RL)
5. Wait for all motors to complete
6. Buzzer confirms with tone

✅ **Verification**: All 4 motors spun individually.

---

## Step 6: Motor Test (1 minute)

Test all motors spin correctly.

1. **PROPELLERS STILL REMOVED**
2. Press **Button 3** (Motor Test)
3. All motors should spin at low speed for 3 seconds
4. Verify correct rotation:
   - **FL**: Counter-clockwise ↻
   - **FR**: Clockwise ↺
   - **RR**: Counter-clockwise ↻
   - **RL**: Clockwise ↺
5. If wrong direction, swap any 2 motor wires to ESC

✅ **Verification**: All motors spin in correct direction.

---

## Step 7: Pre-Flight Safety Check (2 minutes)

Before attaching propellers:

- [ ] Gyro calibrated successfully
- [ ] ESC calibration complete
- [ ] All motors tested and rotate correctly
- [ ] Remote link active (LED blinking)
- [ ] Battery voltage > 11.1V (3S LiPo)
- [ ] All wires secured with cable ties
- [ ] No loose components
- [ ] Arming switch (SW2) at position "0" (disarmed)
- [ ] Open area for flight (>10m radius)
- [ ] Safety glasses on

---

## Step 8: First Flight (5 minutes)

### Attach Propellers

**⚠️ CAUTION**: Propellers are dangerous!

1. Match propeller type to motor position:
   - **CW (Clockwise)**: FR and RL
   - **CCW (Counter-clockwise)**: FL and RR
2. Propeller markings should face **UP**
3. Tighten securely
4. Check propellers don't touch anything

### Arm the Drone

1. Place drone on flat ground
2. Move to safe distance (3-5 meters)
3. Remote Controller throttle to **minimum** (bottom position)
4. Flip **SW2 (Arming Switch)** to position "1"
5. Flight Controller buzzer should **beep once** (armed)
6. Motors will NOT spin yet (throttle still at minimum)

### Liftoff!

1. **Slowly** push throttle up (left joystick vertical)
2. At ~40% throttle, drone should lift off
3. Hover at 1-2 meters height
4. Practice:
   - **Throttle** (left vertical): Up/down
   - **Yaw** (left horizontal): Rotate left/right
   - **Pitch** (right vertical): Forward/backward
   - **Roll** (right horizontal): Strafe left/right

### Landing

1. Reduce throttle slowly
2. Let drone settle gently on ground
3. Throttle to minimum
4. Flip **SW2** to "0" (disarm)
5. Motors stop immediately

✅ **Verification**: Successful first flight!

---

## Emergency Procedures

### Kill Switch (Emergency Stop)

- **Flip SW2 to "0"** immediately disarms motors
- Use if drone becomes uncontrollable
- Drone will drop - use only if necessary

### Lost Control

1. Release all joysticks (return to center)
2. Reduce throttle to minimum
3. If drone doesn't stabilize, use kill switch

### Low Battery

- Plan to land when voltage drops below 11.1V
- Monitor flight time (typically 8-12 minutes)
- Never drain LiPo below 3.3V per cell

---

## Troubleshooting Quick Fixes

| Problem | Solution |
|---------|----------|
| Motors don't arm | Check SW2 position, throttle at minimum |
| Drone drifts | Recalibrate gyro on flat surface |
| Oscillations | Reduce P gain (see PID_TUNING.md) |
| NRF not linking | Check 3.3V power, add capacitor |
| One motor not spinning | Check ESC connections, redo ESC calibration |

---

## Next Steps

- 🎓 Read [PID_TUNING.md](PID_TUNING.md) to optimize flight characteristics
- 📐 Review [PIN_CONFIGURATION.md](PIN_CONFIGURATION.md) for detailed pinout
- 🔧 Study [WIRING_GUIDE.md](WIRING_GUIDE.md) for troubleshooting

---

## Flight Practice Exercises

### Beginner
1. Hover in place for 30 seconds
2. Forward and backward flight
3. Side-to-side flight
4. Gentle turns using yaw

### Intermediate
1. Figure-8 patterns
2. Circle around a point
3. Square flight pattern
4. Controlled descents

### Advanced
1. Fast forward flight
2. Quick direction changes
3. Low-altitude flight
4. Precision landing

---

## Safety Reminders

- ⚠️ Always fly in open areas away from people
- ⚠️ Respect local drone regulations
- ⚠️ Monitor battery voltage
- ⚠️ Never fly indoors initially
- ⚠️ Keep spare propellers handy
- ⚠️ Have fun and fly safe!

---

**Happy Flying! 🚁**

---

**Last Updated**: 2025-11-30
