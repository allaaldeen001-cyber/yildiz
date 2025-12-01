# Quadcopter Stabilization Testing Guide

## ⚠️ IMPORTANT: Testing Stabilization Response

This guide will help you verify that the MPU6050 is working and the PID stabilization is active.

---

## 🔍 Problem: Motors Don't Respond When Tilting Drone

If you tilt the drone by hand and the motors don't change speed to stabilize, follow these steps:

---

## 📋 Step-by-Step Testing Procedure

### ⚠️ SAFETY FIRST: REMOVE PROPELLERS!

---

### Step 1: Upload Updated Code

1. Upload the updated `FlightController.ino` to your drone
2. The new version includes:
   - Real-time debugging output
   - Minimum throttle for stabilization testing
   - Detailed motor speed display

---

### Step 2: Check MPU6050 Connection

1. **Open Serial Monitor** (115200 baud)
2. **Power on flight controller**
3. **Look for these messages:**

```
[OK] Motors initialized
[OK] Radio initialized
[OK] MPU6050 initialized  ← Must see this!
Calibrating sensors...
[OK] Calibration complete!
System Ready!
```

**If you see "[ERROR] MPU6050 not found!":**
- ❌ MPU6050 is not connected properly
- Check I2C wiring: SDA → A4, SCL → A5
- Check 3.3V power to MPU6050
- Check ground connection

---

### Step 3: Verify IMU Data is Reading

Once system is ready, you should see continuous output like:

```
ARM:0 | Angles R:0.5 P:-0.3 Y:0.0 | Gyro R:0.2 P:-0.1 Y:0.0 | PID R:0 P:0 Y:0 | Motors FL:1000 FR:1000 RR:1000 RL:1000 | RX T:1000
```

**TEST: Tilt the drone (while disarmed)**

Watch the Serial Monitor:
- **Tilt RIGHT** → Roll angle should increase (positive)
- **Tilt LEFT** → Roll angle should decrease (negative)
- **Tilt FORWARD** → Pitch angle should increase (positive)
- **Tilt BACKWARD** → Pitch angle should decrease (negative)
- **Rotate** → Gyro values should change

**If angles DON'T change:**
- ❌ MPU6050 is not reading properly
- Check orientation (chip facing up)
- Check calibration was successful
- Try power cycling

---

### Step 4: Test Stabilization Response

Now let's test if the motors respond to tilting:

#### A. Power On System

1. Flight controller powered and calibrated
2. Remote controller powered
3. Serial Monitor open (115200 baud)
4. **PROPELLERS REMOVED!**

#### B. ARM the Drone

1. **Throttle stick to minimum** (fully down)
2. **Flip SW1 to ARM position**
3. Flight controller beeps once
4. LED stays solid
5. Remote shows "[ARMED]"

#### C. Observe Serial Output

You should now see:
```
ARM:1 | Angles R:0.0 P:0.0 Y:0.0 | Gyro R:0.0 P:0.0 Y:0.0 | PID R:0 P:0 Y:0 | Motors FL:1100 FR:1100 RR:1100 RL:1100 | RX T:1000
```

**Key points:**
- ARM: should show **1** (armed)
- Motors: should show **1100** (minimum for stabilization)
- All motors spinning slowly

#### D. Tilt the Drone (By Hand)

**⚠️ KEEP THROTTLE AT MINIMUM! ⚠️**

**TEST 1: Tilt RIGHT (Roll Right)**

Expected behavior:
```
ARM:1 | Angles R:15.0 P:0.0 | PID R:-200 | Motors FL:900 FR:1300 RR:1300 RL:900
                     ↑                ↑              ↑     ↑     ↑     ↑
                   Tilted          PID tries      LEFT    RIGHT  RIGHT LEFT
                   right           to correct     slower  faster faster slower
```

- **Angle Roll** increases (positive = tilted right)
- **PID Roll** becomes negative (trying to roll left to correct)
- **Left motors (FL, RL)** speed decreases
- **Right motors (FR, RR)** speed increases
- This would make drone roll LEFT to level itself

**TEST 2: Tilt LEFT (Roll Left)**

Expected:
- Roll angle becomes negative
- PID Roll becomes positive
- Right motors slow down
- Left motors speed up

**TEST 3: Tilt FORWARD (Pitch Forward)**

Expected behavior:
```
ARM:1 | Angles R:0.0 P:15.0 | PID P:-200 | Motors FL:900 FR:900 RR:1300 RL:1300
                     ↑                ↑            ↑     ↑     ↑     ↑
                  Pitched          PID tries    FRONT FRONT REAR  REAR
                  forward          to correct   slower slower faster faster
```

- **Angle Pitch** increases (positive = tilted forward)
- **PID Pitch** becomes negative (trying to pitch back)
- **Front motors (FL, FR)** slow down
- **Rear motors (RR, RL)** speed up
- This would pitch drone BACKWARD to level itself

**TEST 4: Tilt BACKWARD (Pitch Backward)**

Expected:
- Pitch angle becomes negative
- PID Pitch becomes positive
- Rear motors slow down
- Front motors speed up

---

### Step 5: Interpret Results

#### ✅ GOOD - Stabilization is Working

If you see:
- Angles change when you tilt
- PID values change (opposite sign to angle)
- Motor speeds change accordingly
- Opposite motors speed up/slow down

**→ Your stabilization is working! Ready to fly!**

#### ❌ BAD - Stabilization NOT Working

**Problem A: Angles don't change**
- MPU6050 not reading
- Check wiring: SDA, SCL, VCC, GND
- Check calibration completed

**Problem B: Angles change but PID stays at 0**
- Not armed properly
- Check ARM:1 in output
- Check remote connection

**Problem C: PID changes but motors don't respond**
- Check motor connections
- Verify ESC calibration
- Check throttle is being received (RX T: value)

**Problem D: Motors respond backwards**
- Motor mixing might be wrong
- Check motor rotation directions
- Verify propeller configuration

---

## 🔧 Troubleshooting Common Issues

### Issue 1: "Motors all stay at 1000"

**When Armed:**
- Motors should be at 1100 minimum
- If they stay at 1000, not actually armed
- Check ARM status in serial output

**Solution:**
- Ensure throttle at minimum before arming
- Check SW1 switch working (test with multimeter)
- Check remote connection (shows CONN)

### Issue 2: "Angles are way off (like 90 degrees when level)"

**Calibration failed:**
- Surface not level during calibration
- Drone moved during calibration
- MPU6050 orientation wrong

**Solution:**
- Power cycle and recalibrate
- Ensure perfectly level surface
- Don't touch during calibration
- Check MPU6050 chip faces UP

### Issue 3: "PID values are huge (like ±400)"

**Too much tilt:**
- PID output maxes at ±400
- Normal for large angles

**Solution:**
- This is normal protection
- Try smaller tilt angles (5-10 degrees)
- Verify constrain() working

### Issue 4: "Motors respond but wrong direction"

**Motor mixing incorrect:**
- Motors speeding up when should slow down
- Motors in wrong configuration

**Solution:**
- Check motor positions match code:
  ```
      FL(D3)  FR(D5)
         \      /
          \    /
           \  /
            \/
            /\
           /  \
          /    \
         /      \
      RL(D9)  RR(D6)
  ```
- Verify motor directions (CCW/CW pattern)

### Issue 5: "Everything looks good but drone still flips"

**With real flight:**
- Stabilization working in test but fails in air
- Likely motor direction or prop issue

**Solution:**
- Verify motor rotation directions:
  - FL: Counter-clockwise
  - FR: Clockwise
  - RR: Counter-clockwise
  - RL: Clockwise
- Check propeller directions match motor rotation
- Check all props are same size and balanced

---

## 📊 Understanding the Debug Output

Let me explain each field:

```
ARM:1 | Angles R:15.3 P:-5.2 Y:0.5 | Gyro R:25.3 P:-10.2 Y:1.0 | PID R:-214 P:73 Y:-5 | Motors FL:886 FR:1314 RR:1173 RL:927 | RX T:1000
  ↑         ↑      ↑      ↑            ↑      ↑      ↑             ↑     ↑    ↑           ↑    ↑    ↑    ↑              ↑
Armed?   Roll  Pitch  Yaw         Roll   Pitch  Yaw          Roll Pitch Yaw    Front  Front Rear  Rear         Throttle
        angle  angle  angle        rate   rate   rate         PID  PID   PID    Left   Right Right Left         from RX
```

**ARM:** 0 = disarmed, 1 = armed
**Angles:** Current tilt angles in degrees (from complementary filter)
**Gyro:** Current rotation rates in degrees/second
**PID:** PID controller outputs (corrections to apply)
**Motors:** Individual motor speeds (1000-2000 microseconds)
**RX T:** Throttle value received from remote

---

## 🎯 Expected Values for Different Tests

### Level Position (No Tilt)
```
Angles R:0.0 P:0.0 Y:0.0
Gyro R:0.0 P:0.0 Y:0.0
PID R:0 P:0 Y:0
Motors FL:1100 FR:1100 RR:1100 RL:1100
```

### Tilted Right 20°
```
Angles R:20.0 P:0.0 Y:0.0
Gyro R:0.0 P:0.0 Y:0.0  (if held steady)
PID R:-280 P:0 Y:0  (trying to roll left)
Motors FL:820 FR:1380 RR:1380 RL:820
       ↑ LEFT SLOW  ↑ RIGHT FAST ↑
```

### Tilted Forward 20°
```
Angles R:0.0 P:20.0 Y:0.0
Gyro R:0.0 P:0.0 Y:0.0
PID R:0 P:-280 Y:0  (trying to pitch back)
Motors FL:820 FR:820 RR:1380 RL:1380
       ↑ FRONT SLOW  ↑  REAR FAST ↑
```

---

## ✅ Stabilization Working Checklist

Test each one:

- [ ] Serial Monitor shows MPU6050 initialized
- [ ] Calibration completes successfully (2 beeps)
- [ ] Angles change when drone tilted (while disarmed)
- [ ] Can ARM successfully (LED solid, motors at 1100)
- [ ] When tilted RIGHT: Left motors slow, right motors speed up
- [ ] When tilted LEFT: Right motors slow, left motors speed up
- [ ] When tilted FORWARD: Front motors slow, rear motors speed up
- [ ] When tilted BACKWARD: Rear motors slow, front motors speed up
- [ ] PID values are opposite sign to angle (negative angle → positive PID)
- [ ] Motor speeds change by 100-400 for moderate tilts

**If ALL boxes checked: Stabilization is working! 🎉**

---

## 🚀 Next Steps After Successful Test

Once stabilization is confirmed working:

1. **Disarm the drone** (SW1 off)
2. **Install propellers** (correct directions!)
3. **Clear 5-meter radius**
4. **Follow OPERATION_GUIDE.md** for first flight
5. **Start with gentle hover at 50% throttle**

---

## 🆘 Still Not Working?

If after all tests, stabilization still doesn't work:

1. **Post your Serial Monitor output**
2. **Include what happens when you tilt**
3. **Check these critical connections:**
   ```
   MPU6050:
   - SDA → Arduino A4
   - SCL → Arduino A5
   - VCC → 3.3V (NOT 5V!)
   - GND → GND
   - INT → D2 (optional)
   ```

4. **Test MPU6050 with simple sketch:**
   ```cpp
   #include <Wire.h>
   
   void setup() {
     Serial.begin(115200);
     Wire.begin();
     Wire.beginTransmission(0x68);
     byte error = Wire.endTransmission();
     if (error == 0) {
       Serial.println("MPU6050 found!");
     } else {
       Serial.println("MPU6050 NOT found!");
     }
   }
   
   void loop() {}
   ```

5. **Common fixes:**
   - Replace MPU6050 (might be damaged)
   - Try different I2C pins
   - Add pull-up resistors (4.7kΩ) on SDA/SCL
   - Check for solder bridges

---

## 💡 Understanding PID Stabilization

**How it works:**

1. **Sensor reads angle** (e.g., tilted 10° right)
2. **PID calculates correction** (need to roll -14 units left)
3. **Motor mixer applies correction:**
   - Left motors slow down (-140 µs)
   - Right motors speed up (+140 µs)
4. **Physical result:** Drone rolls left to level itself
5. **Loop repeats 250 times per second!**

**Key point:** The PID always tries to bring angles back to setpoint (usually 0°)

---

## 📈 PID Tuning (If Needed)

Default values work for most builds, but if needed:

**Too aggressive (oscillates):**
- Reduce P gain: `#define PID_ROLL_KP 1.2` (was 1.4)
- Reduce D gain: `#define PID_ROLL_KD 15.0` (was 18.0)

**Too sluggish:**
- Increase P gain: `#define PID_ROLL_KP 1.6` (was 1.4)

**Drifts over time:**
- Increase I gain: `#define PID_ROLL_KI 0.08` (was 0.05)

Make changes in `FlightController.ino` lines 60-71.

---

**Good luck with testing! Your stabilization should work now! 🚁**
