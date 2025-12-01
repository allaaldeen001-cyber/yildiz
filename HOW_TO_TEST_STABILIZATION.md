# 🚁 Quick Guide: Test Your Stabilization NOW

## ⚠️ SAFETY: REMOVE PROPELLERS!

---

## 🚀 5-Minute Test Procedure

### Step 1: Upload New Code (2 min)

1. Open Arduino IDE
2. Open `FlightController/FlightController.ino`
3. Click **Upload** (wait for "Done uploading")

### Step 2: Open Serial Monitor (30 sec)

1. Click **Tools → Serial Monitor** (or Ctrl+Shift+M)
2. Set baud rate to **115200**
3. You should see:
   ```
   [OK] Motors initialized
   [OK] Radio initialized
   [OK] MPU6050 initialized  ← MUST SEE THIS!
   ```

### Step 3: Test IMU Reading (1 min)

**While disarmed**, tilt the drone and watch Serial Monitor:

```
ARM:0 | Angles R:15.3 P:-5.2 Y:0.5 | ...
              ↑ Changes when you tilt!
```

**Tilt RIGHT** → Roll angle increases (+)
**Tilt LEFT** → Roll angle decreases (-)
**Tilt FORWARD** → Pitch angle increases (+)
**Tilt BACKWARD** → Pitch angle decreases (-)

✅ If angles change → MPU6050 is working!
❌ If angles stay at 0 → Check MPU6050 wiring

---

### Step 4: ARM the Drone (30 sec)

1. Make sure **PROPELLERS ARE REMOVED**
2. **Throttle stick to MINIMUM** (fully down)
3. **Flip SW1 to ARM position**
4. Listen for **1 beep**
5. LED stays **solid**

Serial Monitor should show:
```
ARM:1 | ... | Motors FL:1100 FR:1100 RR:1100 RL:1100
  ↑                    ↑ All motors at 1100!
Armed!
```

---

### Step 5: Test Stabilization (1 min)

**Keep throttle at minimum!**

#### Test A: Tilt RIGHT
**What to do:** Tilt drone right by hand (roll right)

**What you should see:**
```
ARM:1 | Angles R:15.0 | PID R:-210 | Motors FL:890 FR:1310 RR:1310 RL:890
              ↑              ↑               ↑ LEFT  ↑ RIGHT ↑ RIGHT ↑ LEFT
          Tilted        Correcting            SLOW    FAST    FAST   SLOW
           right         to left
```

- **Roll angle** goes positive (15°)
- **PID Roll** goes negative (-210)
- **Left motors (FL, RL)** slow down (~890)
- **Right motors (FR, RR)** speed up (~1310)

**This is CORRECT!** Drone is trying to roll LEFT to level itself.

#### Test B: Tilt LEFT
- Roll angle goes negative
- PID Roll goes positive
- Right motors slow down
- Left motors speed up

#### Test C: Tilt FORWARD
- Pitch angle goes positive
- PID Pitch goes negative
- Front motors (FL, FR) slow down
- Rear motors (RR, RL) speed up

#### Test D: Tilt BACKWARD
- Pitch angle goes negative
- PID Pitch goes positive
- Rear motors slow down
- Front motors speed up

---

## ✅ Success Criteria

Your stabilization is working if:

- [x] MPU6050 initializes (shows "[OK]" message)
- [x] Angles change when you tilt (disarmed)
- [x] Can ARM successfully (ARM:1, motors at 1100)
- [x] Motors change speed when tilted
- [x] **Motor changes are OPPOSITE to tilt:**
  - Tilt right → Left side speeds up (to correct)
  - Tilt left → Right side speeds up
  - Tilt forward → Rear speeds up
  - Tilt backward → Front speeds up

**ALL boxes checked?** → **STABILIZATION WORKING! 🎉**

---

## ❌ Common Problems

### Problem 1: "No debug output in Serial Monitor"

**Cause:** Wrong baud rate

**Fix:**
- Set to 115200 (bottom right of Serial Monitor)

---

### Problem 2: "MPU6050 not found!"

**Cause:** MPU6050 wiring issue

**Fix:**
```
Check these connections:
MPU6050 → Arduino Nano
  SDA → A4
  SCL → A5
  VCC → 3.3V (NOT 5V!)
  GND → GND
```

---

### Problem 3: "Angles don't change when I tilt"

**Cause:** MPU6050 not reading or bad calibration

**Fix:**
- Power cycle (restart)
- Keep drone LEVEL during calibration
- Don't touch during 6-second calibration
- Check MPU6050 chip facing UP

---

### Problem 4: "Can't ARM (ARM stays at 0)"

**Cause:** Throttle not at minimum or remote not connected

**Fix:**
- Throttle stick fully DOWN
- Check remote shows "CONN"
- Check SW1 switch working
- Try flipping SW1 off and on again

---

### Problem 5: "Motors stay at 1000 (don't change)"

**Cause:** Not actually armed or ESC issue

**Fix:**
- Verify ARM:1 in Serial Monitor
- Check ESC calibration done
- Check motor signal wires connected
- Check ESC powered

---

### Problem 6: "Motors change but WRONG direction"

**Cause:** Motor position or rotation direction wrong

**Fix:**

**Check motor positions:**
```
        FRONT
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
       REAR
```

**Check motor directions:**
- FL: Counter-clockwise (CCW)
- FR: Clockwise (CW)
- RR: Counter-clockwise (CCW)
- RL: Clockwise (CW)

To reverse motor: swap any two of the three motor wires

---

## 📊 Example of Working Stabilization

```
Initial (level):
ARM:1 | Angles R:0.0 P:0.0 | PID R:0 P:0 | Motors FL:1100 FR:1100 RR:1100 RL:1100

Tilt RIGHT 20°:
ARM:1 | Angles R:20.0 P:0.0 | PID R:-280 P:0 | Motors FL:820 FR:1380 RR:1380 RL:820
                                                        ↓ LEFT  ↑ RIGHT ↑ RIGHT ↓ LEFT

Tilt FORWARD 20°:
ARM:1 | Angles R:0.0 P:20.0 | PID R:0 P:-280 | Motors FL:820 FR:820 RR:1380 RL:1380
                                                        ↓ FRONT ↓ FRONT ↑ REAR ↑ REAR

Return to level:
ARM:1 | Angles R:0.0 P:0.0 | PID R:0 P:0 | Motors FL:1100 FR:1100 RR:1100 RL:1100
```

---

## 🎯 What Each Motor Does

```
       FRONT
    FL     FR
    ↓       ↑
     \     /
      \   /
       \ /
        X
       / \
      /   \
     /     \
    ↑       ↓
    RL     RR
      REAR

Tilt Right:
- FL & RL: Slow down (left side)
- FR & RR: Speed up (right side)
- Effect: Drone rolls LEFT to correct

Tilt Forward:
- FL & FR: Slow down (front)
- RR & RL: Speed up (rear)
- Effect: Drone pitches BACK to correct
```

---

## 🔧 Advanced: Understanding Serial Output

```
ARM:1 | Angles R:15.3 P:-5.2 Y:0.5 | Gyro R:25.3 P:-10.2 Y:1.0 | PID R:-214 P:73 Y:-5 | Motors FL:886 FR:1314 RR:1173 RL:927 | RX T:1000
  │         │      │      │            │      │      │             │     │    │           │    │    │    │              │
  │         │      │      │            │      │      │             │     │    │           │    │    │    │              └─ Throttle from remote
  │         │      │      │            │      │      │             │     │    │           │    │    │    └─ Rear Left speed
  │         │      │      │            │      │      │             │     │    │           │    │    └─ Rear Right speed
  │         │      │      │            │      │      │             │     │    │           │    └─ Front Right speed
  │         │      │      │            │      │      │             │     │    │           └─ Front Left speed
  │         │      │      │            │      │      │             │     │    └─ Yaw PID output
  │         │      │      │            │      │      │             │     └─ Pitch PID output
  │         │      │      │            │      │      │             └─ Roll PID output
  │         │      │      │            │      │      └─ Yaw rotation rate (°/s)
  │         │      │      │            │      └─ Pitch rotation rate (°/s)
  │         │      │      │            └─ Roll rotation rate (°/s)
  │         │      │      └─ Yaw angle (°)
  │         │      └─ Pitch angle (°)
  │         └─ Roll angle (°)
  └─ Armed status (0=disarmed, 1=armed)
```

---

## 📱 Quick Video Test

**Can't figure it out? Record this:**

1. Power on drone (show Serial Monitor on screen)
2. Show "[OK] MPU6050 initialized" message
3. Tilt drone while showing Serial Monitor
4. ARM the drone
5. Tilt drone again while armed
6. Show motor speed values changing

This shows exactly what's happening!

---

## 🆘 Still Not Working?

If stabilization still doesn't work after following this guide:

### Things to Check:

1. **MPU6050 connection:**
   ```bash
   # Simple test sketch:
   #include <Wire.h>
   void setup() {
     Serial.begin(115200);
     Wire.begin();
     Wire.beginTransmission(0x68);
     Serial.println(Wire.endTransmission() == 0 ? "MPU OK" : "MPU FAIL");
   }
   void loop() {}
   ```

2. **Motor connections:**
   - Use servo tester on each ESC
   - Verify ESCs work independently
   - Check signal wires on D3, D5, D6, D9

3. **Power issues:**
   - ESC BEC providing 5V to Arduino?
   - 3.3V available for nRF24 and MPU?
   - Battery charged (>11V)?

4. **Code issues:**
   - Re-download FlightController.ino
   - Verify upload successful
   - Check no compilation errors

---

## ✨ Next Steps After Success

Once stabilization is confirmed working:

1. **DISARM** (SW1 off)
2. **Install propellers** (check directions!)
3. **Go outside** (5m clear area)
4. **Follow OPERATION_GUIDE.md**
5. **First hover at 50% throttle**
6. **Keep flights low and slow initially**

---

## 📚 Related Documents

- **Full test guide:** `STABILIZATION_TEST_GUIDE.md`
- **What changed:** `STABILIZATION_FIX_SUMMARY.md`
- **Wiring help:** `WIRING_DIAGRAMS.md`
- **Flying guide:** `OPERATION_GUIDE.md`

---

## 💡 Key Points to Remember

1. ⚠️ **ALWAYS test without propellers first**
2. 📺 **Serial Monitor is your best friend**
3. ✅ **Angles must change when you tilt**
4. ✅ **Motors must respond opposite to tilt**
5. ✅ **Minimum speed is 1100 when armed**

---

**Your drone should stabilize now! If you see the motors responding when you tilt it, you're ready to fly! 🚁✨**

**Need help?** Check the Serial Monitor output and compare with examples above.

**Working?** Awesome! Now go to `OPERATION_GUIDE.md` for first flight procedures!
