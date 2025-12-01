# 🔧 FIX IMU Motor Response

## ✅ Good News:
- RC works well ✅
- Communication is good ✅
- You understand the theory perfectly ✅

## ❌ Problem:
Motors don't respond correctly when you tilt the drone (IMU issue)

---

## 🧪 DIAGNOSTIC TEST - Upload This First!

### **Upload:** `TestStabilization/TestStabilization.ino`

This will show you EXACTLY:
- What angle the IMU detects
- What PID correction is calculated
- Which motors speed up/down
- If response is correct or inverted

### **How to use:**

1. **Upload** `TestStabilization.ino`
2. **Open Serial Monitor** (115200 baud)
3. **Wait for calibration** (2 beeps)
4. **Type 'A'** to ARM
5. **Tilt drone** and watch output!

---

## 📊 What You Should See:

### **When tilting NOSE DOWN (pitch forward):**

**Correct Response:**
```
TILT: NOSE DOWN | Ang P:-15.0° | PID P:+210 | Motors: FL:890↓ FR:890↓ RR:1310↑ RL:1310↑
  → Nose DOWN detected. Rear should be FASTER (↑), Front SLOWER (↓)
```

**What this means:**
- IMU detects nose pointing down (-15°)
- PID says: "Push rear down to lift nose" (+210)
- Rear motors (RR, RL) speed UP (1310)
- Front motors (FL, FR) slow DOWN (890)
- Result: Nose lifts back to level ✅

**If you see OPPOSITE (wrong!):**
```
Motors: FL:1310↑ FR:1310↑ RR:890↓ RL:890↓
```
→ **Motor mixing is inverted!** Need to fix signs.

---

### **When tilting NOSE UP (pitch back):**

**Correct Response:**
```
TILT: NOSE UP | Ang P:+15.0° | PID P:-210 | Motors: FL:1310↑ FR:1310↑ RR:890↓ RL:890↓
  → Nose UP detected. Front should be FASTER (↑), Rear SLOWER (↓)
```

---

### **When tilting RIGHT:**

**Correct Response:**
```
TILT: RIGHT | Ang R:+15.0° | PID R:-210 | Motors: FL:1310↑ FR:890↓ RR:890↓ RL:1310↑
  → Tilted RIGHT. Left side should be FASTER (↑), Right SLOWER (↓)
```

**What this means:**
- Tilted right (+15°)
- Left motors (FL, RL) speed UP
- Right motors (FR, RR) slow DOWN  
- Result: Left side lifts, returns to level ✅

---

### **When tilting LEFT:**

**Correct Response:**
```
TILT: LEFT | Ang R:-15.0° | PID R:+210 | Motors: FL:890↓ FR:1310↑ RR:1310↑ RL:890↓
  → Tilted LEFT. Right side should be FASTER (↑), Left SLOWER (↓)
```

---

## 🔧 FIXES Based on Test Results:

### **Case 1: Angles Don't Change**

**Symptom:**
```
Ang P:0.0° R:0.0°  (stays at zero when you tilt)
```

**Problem:** MPU6050 not reading

**Fix:**
- Check wiring: VCC→3.3V, SDA→A4, SCL→A5, GND→GND
- Run DiagnosticTool to verify MPU found at 0x68
- Try power cycling

---

### **Case 2: PID Values Stay at Zero**

**Symptom:**
```
Ang P:15.0° but PID P:0 R:0
```

**Problem:** PID not calculating

**Fix:**
- Not actually armed
- Code issue (but test code should work)

---

### **Case 3: Motors All Same Speed**

**Symptom:**
```
Motors: FL:1100 FR:1100 RR:1100 RL:1100 (never change)
```

**Problem:** Motor mixing not being applied

**Fix:**
- Check motors D5 and D9 still working
- Verify ESC connections
- Re-calibrate ESCs

---

### **Case 4: Motors Respond BACKWARDS**

**Symptom:**
```
Nose DOWN → Front motors speed UP (should be down!)
Nose DOWN → Rear motors slow DOWN (should be up!)
```

**Problem:** Motor mixing signs are inverted

**Fix:** Need to invert the pitch axis in motor mixing

**Change in FlightController.ino:**

**From:**
```cpp
motorFLSpeed = throttle - pidPitch + pidRoll - pidYaw;
motorFRSpeed = throttle - pidPitch - pidRoll + pidYaw;
motorRRSpeed = throttle + pidPitch - pidRoll - pidYaw;
motorRLSpeed = throttle + pidPitch + pidRoll + pidYaw;
```

**To:**
```cpp
motorFLSpeed = throttle + pidPitch + pidRoll - pidYaw;  // Inverted pitch
motorFRSpeed = throttle + pidPitch - pidRoll + pidYaw;  // Inverted pitch
motorRRSpeed = throttle - pidPitch - pidRoll - pidYaw;  // Inverted pitch
motorRLSpeed = throttle - pidPitch + pidRoll + pidYaw;  // Inverted pitch
```

---

### **Case 5: Roll Response Backwards**

**Symptom:**
```
Tilt RIGHT → Right motors speed UP (should be down!)
```

**Fix:** Invert roll axis

**Change roll signs in motor mixing**

---

## 📋 Expected Behavior Summary:

| Tilt Direction | Angle Sign | PID Sign | Which Motors UP | Which Motors DOWN |
|----------------|------------|----------|-----------------|-------------------|
| **Nose DOWN** | Pitch < 0 | PID > 0 | Rear (RR, RL) ↑ | Front (FL, FR) ↓ |
| **Nose UP** | Pitch > 0 | PID < 0 | Front (FL, FR) ↑ | Rear (RR, RL) ↓ |
| **Tilt RIGHT** | Roll > 0 | PID < 0 | Left (FL, RL) ↑ | Right (FR, RR) ↓ |
| **Tilt LEFT** | Roll < 0 | PID > 0 | Right (FR, RR) ↑ | Left (FL, RL) ↓ |

---

## 🎯 Step-by-Step Testing:

### **Test 1: Verify IMU Reads Tilts**

```
1. Upload TestStabilization.ino
2. Type 'A' to arm
3. Tilt NOSE DOWN
4. Check: Ang P shows NEGATIVE value (e.g., -15°)
5. Tilt NOSE UP
6. Check: Ang P shows POSITIVE value (e.g., +15°)
7. Tilt RIGHT
8. Check: Ang R shows POSITIVE value
9. Tilt LEFT
10. Check: Ang R shows NEGATIVE value
```

**If angles don't change → MPU6050 problem**
**If angles change correctly → Continue to Test 2**

---

### **Test 2: Verify PID Responds**

```
1. While armed and tilted
2. Check PID values are NON-ZERO
3. PID should be OPPOSITE sign to angle
   - Angle negative → PID positive
   - Angle positive → PID negative
```

**If PID = 0 → PID not calculating**
**If PID responds → Continue to Test 3**

---

### **Test 3: Verify Motor Response**

```
1. Tilt NOSE DOWN
2. Check Serial output:
   "→ Nose DOWN detected. Rear should be FASTER (↑), Front SLOWER (↓)"
3. Verify motors show:
   FL:↓ FR:↓ RR:↑ RL:↑
```

**If motors respond correctly → WORKING!**
**If motors respond backwards → Invert pitch signs**
**If motors don't respond → Motor/ESC problem**

---

## 🔧 Common Fixes:

### **Fix A: Invert Pitch Response**

If nose down makes front motors speed up (wrong!):

**In `FlightController.ino` line ~450:**

**Change:**
```cpp
motorFLSpeed = throttle - pidPitch + pidRoll - pidYaw;
motorFRSpeed = throttle - pidPitch - pidRoll + pidYaw;
motorRRSpeed = throttle + pidPitch - pidRoll - pidYaw;
motorRLSpeed = throttle + pidPitch + pidRoll + pidYaw;
```

**To:**
```cpp
motorFLSpeed = throttle + pidPitch + pidRoll - pidYaw;
motorFRSpeed = throttle + pidPitch - pidRoll + pidYaw;
motorRRSpeed = throttle - pidPitch - pidRoll - pidYaw;
motorRLSpeed = throttle - pidPitch + pidRoll + pidYaw;
```

---

### **Fix B: Invert Roll Response**

If tilt right makes right motors speed up (wrong!):

**Swap roll signs:**
```cpp
motorFLSpeed = throttle - pidPitch - pidRoll - pidYaw;  // Inverted roll
motorFRSpeed = throttle - pidPitch + pidRoll + pidYaw;  // Inverted roll
motorRRSpeed = throttle + pidPitch + pidRoll - pidYaw;  // Inverted roll
motorRLSpeed = throttle + pidPitch - pidRoll + pidYaw;  // Inverted roll
```

---

### **Fix C: Motors D5, D9 Still Not Working**

If only some motors respond:

1. Re-run ESC calibration tool
2. Check signal wire connections
3. Swap test to identify bad ESC/motor

---

## ✅ Success Checklist:

**Run TestStabilization.ino and verify:**

- [ ] Angles change when you tilt
- [ ] PID values are non-zero and opposite sign to angle
- [ ] Nose DOWN → Rear motors speed UP
- [ ] Nose DOWN → Front motors slow DOWN
- [ ] Nose UP → Front motors speed UP
- [ ] Nose UP → Rear motors slow DOWN
- [ ] Tilt RIGHT → Left motors speed UP
- [ ] Tilt RIGHT → Right motors slow DOWN
- [ ] Tilt LEFT → Right motors speed UP
- [ ] Tilt LEFT → Left motors slow DOWN

**ALL CHECKED? → Stabilization working! Upload FlightController.ino and FLY!**

---

## 🚀 After Test Shows It Works:

1. ✅ Note any fixes needed (pitch/roll inversion)
2. ✅ Apply same fixes to FlightController.ino
3. ✅ Upload FlightController.ino
4. ✅ Upload RemoteController.ino
5. ✅ Test again with Button 3 ARM
6. ✅ Install propellers
7. ✅ FLY!

---

**Upload TestStabilization.ino NOW and tell me what you see!**

**Copy/paste the Serial Monitor output when you tilt it!**
