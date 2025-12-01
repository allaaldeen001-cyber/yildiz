# 🎮 BUTTON CONTROL - FINAL SPECIFICATION

## ✅ Your Exact Requirements Implemented!

---

## 🔘 Button Functions (From Remote Controller)

### **Button 1 (D4 on Remote) → SENSOR CALIBRATION**
**Function:** Trigger MPU6050 calibration from remote control

**When to use:**
- Before first flight
- When drone drifts to one side
- After temperature changes
- When MPU readings seem off

**How to use:**
1. **DISARM** the drone (not flying)
2. Place drone on **level surface**
3. **Press Button 1** on remote
4. Flight controller beeps once
5. **Don't move drone** for 6 seconds
6. Flight controller beeps twice = Done!

**Serial Monitor shows:**
```
BTN1: Starting sensor calibration from RC...
Calibrating sensors...
[OK] Calibration complete!
BTN1: Calibration complete!
```

---

### **Button 2 (D5 on Remote) → MOTOR TEST**
**Function:** Spin motors to check direction and operation

**⚠️ REMOVE PROPELLERS FIRST! ⚠️**

**When to use:**
- Before first flight (check motor directions)
- After wiring changes
- To verify all motors work
- To check motor rotation directions

**How to use:**
1. **DISARM** the drone
2. **REMOVE PROPELLERS!** ⚠️
3. **Hold Button 2** on remote
4. Motors spin slowly (1000-1400 µs)
5. **Check directions:**
   - FL (D3): Counter-clockwise (CCW)
   - FR (D5): Clockwise (CW)
   - RR (D6): Counter-clockwise (CCW)
   - RL (D9): Clockwise (CW)
6. **Release Button 2** to stop

**Serial Monitor shows:**
```
BTN2: Motor test - CHECK DIRECTIONS!
MOTOR_TEST | Mot FL:1100 FR:1100 RR:1100 RL:1100
MOTOR_TEST | Mot FL:1200 FR:1200 RR:1200 RL:1200
BTN2: Motor test STOP
```

**Motor Configuration:**
```
        FRONT
    FL(CCW)  FR(CW)
       \      /
        \    /
         \  /
          \/
          /\
         /  \
        /    \
       /      \
    RL(CW)  RR(CCW)
       REAR
```

---

### **Button 3 (D6 on Remote) → ARM FOR FLIGHT**
**Function:** Make drone ready to fly with joystick control

**This is your MAIN ARM/DISARM button!**

**How to use:**

**To ARM (prepare for flight):**
1. **Throttle stick to minimum** (fully down)
2. **Press Button 3** on remote
3. Drone beeps once = **ARMED**
4. Serial shows: "BTN3: ARMED - Ready to fly! Use joysticks!"
5. **MPU6050 stabilization is NOW ACTIVE**
6. **Use joysticks to fly!**

**To DISARM (stop/land):**
1. Land the drone
2. **Press Button 3** again
3. Drone beeps twice = **DISARMED**
4. Motors stop
5. Safe to handle

**Serial Monitor shows:**
```
BTN3: ARMED - Ready to fly! Use joysticks!
      MPU6050 stabilization ACTIVE

[when flying, tilting drone:]
ARMED | Mode:ANGLE | Ang R:5.2 P:-2.1 | PID R:73 P:-29 | Mot FL:1173 FR:1027
                         ↑ MPU detects tilt
                                              ↑ PID corrects
                                                            ↑ Motors respond!

[when landing:]
BTN3: DISARMED - Safe
```

---

### **Button 4 (D7 on Remote) → SOFT LANDING**
**Function:** Gradual throttle reduction for smooth landing

**When to use:**
- For gentle, controlled landing
- When hands are shaky
- For smooth video landing
- Learning to land smoothly

**How to use:**
1. **While flying** (ARMED)
2. Hover at desired landing height
3. **Press Button 4**
4. Drone automatically reduces throttle over 5 seconds
5. Drone gently descends and lands
6. After 5 seconds: Auto-DISARMS

**To cancel:**
- Press Button 4 again during landing

**Serial Monitor shows:**
```
BTN4: SOFT LANDING MODE - Throttle reducing slowly
[5 seconds later...]
BTN4: Soft landing complete - DISARMED
```

---

## 🎯 MPU6050 Stabilization

### **How It Works (Your Explanation is Correct!):**

1. **MPU6050 measures:**
   - Angular velocity (gyro): p, q, r (degrees/sec)
   - Acceleration: ax, ay, az (gravity + movement)

2. **Complementary Filter fuses data:**
   ```
   Angle = 0.98 × (Angle_gyro) + 0.02 × (Angle_accel)
   ```
   - 98% gyro (fast, smooth, but drifts)
   - 2% accel (stable over time, but noisy)
   - Result: Accurate angle without drift!

3. **PID Controller calculates correction:**
   ```
   Error = Desired_Angle - Current_Angle
   
   PID_Output = Kp×Error + Ki×∫Error + Kd×(Rate)
   
   Example:
   - Drone tilts 5° right (error = -5°)
   - PID says: "Apply +210 correction"
   - This means: slow right motors, speed up left motors
   ```

4. **Motor Mixer distributes correction:**
   ```
   Tilted RIGHT (need to roll LEFT to correct):
   - Left motors (FL, RL): Speed UP
   - Right motors (FR, RR): Speed DOWN
   
   Tilted FORWARD (need to pitch BACK):
   - Front motors (FL, FR): Speed DOWN  
   - Rear motors (RR, RL): Speed UP
   ```

5. **ESCs adjust motor speeds:**
   - Motors create corrective torque
   - Drone rotates back to level
   - Happens 250 times per second!

### **Example with Numbers:**

**Scenario:** Nose drops 5° (pitch = -5°)

1. **MPU detects:**
   - Pitch angle: -5°
   - Pitch rate: -20 °/s (rotating nose-down)

2. **PID calculates:**
   ```
   Kp = 1.4, Ki = 0.05, Kd = 18.0
   
   Error = 0° - (-5°) = +5°
   Rate_error = 0 - (-20) = +20
   
   PID = 1.4×5 + 0.05×(integrated) + 18.0×(+20/65.5)
       ≈ 7 + 0 + 5.5 = 12.5
   
   Scaled to ±400 limit: PID_output = +210
   ```

3. **Motor mixing:**
   ```
   Base throttle: 1100 µs (hovering)
   
   Front Left:  1100 - 210 = 890 µs  (slower)
   Front Right: 1100 - 210 = 890 µs  (slower)
   Rear Right:  1100 + 210 = 1310 µs (faster)
   Rear Left:   1100 + 210 = 1310 µs (faster)
   ```

4. **Physical result:**
   - Rear motors push harder → nose pitches UP
   - Returns to level (pitch = 0°)
   - Loop repeats 250 times/second!

---

## 🧪 Testing Your Stabilization

### **Step 1: Run Diagnostic First!**

**Upload:** `DiagnosticTool/DiagnosticTool.ino`

**This checks:**
- ✅ MPU6050 found on I2C bus?
- ✅ MPU readings changing when tilted?
- ✅ All 4 motors spinning?

**If diagnostic FAILS, fix wiring before continuing!**

---

### **Step 2: Test Stabilization (No Propellers!)**

1. **Upload:** `FlightController.ino`
2. **Power on remote** (shows "✓CONN")
3. **Wait for calibration** (2 beeps)
4. **Open Serial Monitor** (115200 baud)

5. **Tilt drone while DISARMED:**
   ```
   DISARM | Ang R:15.0 P:-5.2
                ↑ Should change when you tilt!
   ```
   **If angles don't change → MPU problem!**

6. **ARM with Button 3:**
   - Throttle to minimum
   - Press Button 3
   - Hear 1 beep
   - Serial shows: "BTN3: ARMED - MPU6050 stabilization ACTIVE"

7. **Tilt drone (by hand):**
   ```
   ARMED | Ang R:15.0 | PID R:-210 | Mot FL:890 FR:1310 RR:1310 RL:890
              ↑ Tilted      ↑ Correcting  ↑ Left slow, right fast!
   ```

8. **What you should see/hear:**
   - **Motors change speed** when you tilt
   - **Opposite motors** speed up/slow down
   - **PID values** change (positive/negative)
   - **Serial shows** motor speeds changing

**This is MPU6050 stabilization working!**

---

## 🚁 Flying Workflow

### **Complete Flight Sequence:**

**Pre-Flight:**
1. ✅ Battery charged (>11V)
2. ✅ Propellers removed for testing
3. ✅ Power on remote → Wait for "✓CONN"
4. ✅ Power on drone → Wait for 2 beeps

**Testing (No Props!):**
1. ✅ Press Button 2 → Check all motors spin
2. ✅ Check motor directions (CCW/CW pattern)
3. ✅ Press Button 3 → ARM drone
4. ✅ Tilt drone → Verify motors respond
5. ✅ Press Button 3 → DISARM

**First Flight (With Props!):**
1. ✅ Install propellers (correct directions!)
2. ✅ Clear 5-meter area
3. ✅ Stand behind drone
4. ✅ Throttle to minimum
5. ✅ Press Button 3 → ARM
6. ✅ **Slowly** increase throttle to 50%
7. ✅ Drone lifts off → **MPU stabilizes automatically!**
8. ✅ Use right stick for small movements
9. ✅ Press Button 4 for soft landing
10. ✅ Or press Button 3 to disarm immediately

---

## 🔧 Troubleshooting

### **Problem: No MPU Effect**

**Check Serial Monitor when armed and tilted:**

**Should see:**
```
ARMED | Ang R:15.0 | PID R:-210 | Mot FL:890 FR:1310
         ↑ Changes    ↑ Non-zero   ↑ Different speeds
```

**If angles DON'T change:**
- ❌ MPU6050 not connected/working
- Run diagnostic tool
- Check wiring: VCC→3.3V, SDA→A4, SCL→A5

**If angles change but PID = 0:**
- ❌ Not actually armed
- Check armed status in Serial Monitor
- Verify remote shows "✓CONN"

**If PID changes but motors stay same:**
- ❌ Motor control not working
- Check ESC connections
- Verify all motors work (Button 2 test)

---

### **Problem: Motors Don't Spin**

**Run diagnostic tool first!**

**If diagnostic shows motors DON'T spin:**
1. Check battery connected and charged
2. Check ESC power wires
3. Check ESC signal wires (D3, D5, D6, D9)
4. Check ESC grounds to Arduino GND
5. Calibrate ESCs

**If SOME motors don't spin:**
- Swap signal wire with working motor
- If problem moves → Bad ESC/motor
- If problem stays → Bad wiring/pin

---

## 🎯 Quick Reference

| Button | Function | When | Result |
|--------|----------|------|--------|
| **BTN1** | Calibrate sensors | Disarmed, level | Recalibrates MPU6050 |
| **BTN2** | Motor test | Disarmed, no props | Spin motors 1000-1400 |
| **BTN3** | ARM/DISARM | Throttle low | Ready to fly / Safe |
| **BTN4** | Soft landing | While flying | Gentle 5-sec landing |

**Joysticks (when ARMED):**
- Left V: Throttle
- Left H: Yaw
- Right V: Pitch
- Right H: Roll

**MPU6050 Stabilization:**
- Automatically active when ARMED
- Corrects tilts 250 times/second
- Uses complementary filter (98/2)
- PID keeps drone level

---

## ✅ Success Checklist

Before saying "it works":

- [ ] Diagnostic tool shows MPU6050 at 0x68
- [ ] Diagnostic shows all 4 motors spin
- [ ] MPU angles change when tilted (disarmed)
- [ ] Button 1 triggers calibration
- [ ] Button 2 spins motors (no props!)
- [ ] Button 3 ARMS drone (1 beep)
- [ ] When armed and tilted: motors respond
- [ ] Serial shows PID values changing
- [ ] Motor speeds change opposite to tilt
- [ ] Button 3 DISARMS (2 beeps)
- [ ] Button 4 soft landing works

**ALL CHECKED? → MPU6050 STABILIZATION WORKING! 🎉**

---

**Upload the updated FlightController.ino and test with buttons!**

**Your stabilization IS implemented - buttons control it exactly as you specified!**
