# 🔧 FIX: Motors D5 and D9 Not Spinning Well

## ✅ GOOD NEWS: MPU6050 Found at 0x68!

**This means:**
- ✅ MPU6050 is connected correctly
- ✅ Stabilization WILL work once motors are fixed
- ✅ Only motor issue to solve!

---

## ❌ Problem: Motors D5 and D9 Don't Spin Well

**Motor mapping:**
- **D5** = Front Right (FR)
- **D9** = Rear Left (RL)

**Motors D3 and D6 work?** → Good! This tells us:
- ✅ Arduino is working
- ✅ Code is working
- ✅ Battery has power
- ❌ Problem is specific to D5 and D9

---

## 🔍 Diagnostic Questions:

**For motors D5 and D9, do they:**

1. **Not spin at all?** → Signal or power issue
2. **Spin very slowly/weak?** → ESC calibration needed
3. **Make beeping sounds?** → ESC not calibrated
4. **Spin then stop?** → Bad connection

---

## 🔧 FIX #1: Check Wiring (5 minutes)

### **Step-by-step check:**

**For Front Right (D5):**
```
Arduino D5 → ESC signal wire (yellow/white)
ESC ground (black/brown) → Arduino GND
ESC power (red) → Battery
Motor 3 wires → ESC motor outputs
```

**For Rear Left (D9):**
```
Arduino D9 → ESC signal wire (yellow/white)
ESC ground (black/brown) → Arduino GND
ESC power (red) → Battery
Motor 3 wires → ESC motor outputs
```

### **Common problems:**

❌ **Signal wire loose on D5 or D9**
✅ **Fix:** Push firmly into Arduino pin, or re-solder

❌ **ESC ground not connected**
✅ **Fix:** All ESC black wires must go to Arduino GND

❌ **Weak battery connection to these ESCs**
✅ **Fix:** Check battery wires to FR and RL ESCs

---

## 🔧 FIX #2: Swap Test (Find the Problem)

**This identifies if problem is Arduino pin, ESC, or motor:**

### **Test 1: Swap D5 with D3**

1. **Disconnect propellers** ⚠️
2. **Swap signal wires:**
   - D3 wire → D5 pin
   - D5 wire → D3 pin
3. **Upload this test code:**

```cpp
#include <Servo.h>

Servo motor3, motor5;

void setup() {
  Serial.begin(115200);
  motor3.attach(3);
  motor5.attach(5);
  
  Serial.println("Testing swapped motors...");
  Serial.println("D3 pin (now has D5's ESC)");
  Serial.println("D5 pin (now has D3's ESC)");
}

void loop() {
  Serial.println("Spinning D3 pin (1200us)");
  motor3.writeMicroseconds(1200);
  motor5.writeMicroseconds(1000);
  delay(2000);
  
  Serial.println("Spinning D5 pin (1200us)");
  motor3.writeMicroseconds(1000);
  motor5.writeMicroseconds(1200);
  delay(2000);
}
```

4. **Observe results:**

**If D5's ESC now works on D3 pin:**
- ❌ Arduino D5 pin is bad
- ✅ Use different pin (like D8) and update code

**If D5's ESC still doesn't work:**
- ❌ D5's ESC or motor is bad
- ✅ Check ESC calibration or replace ESC

---

## 🔧 FIX #3: ESC Calibration (Most Common Fix!)

**If motors spin weak or make beeping sounds, they need calibration.**

### **Calibrate D5 and D9 ESCs:**

1. **Disconnect battery**
2. **Upload this code:**

```cpp
#include <Servo.h>

Servo esc5, esc9;

void setup() {
  Serial.begin(115200);
  esc5.attach(5);
  esc9.attach(9);
  
  Serial.println("ESC CALIBRATION for D5 and D9");
  Serial.println("=================================");
  Serial.println("1. Type 'H' and press Enter");
  Serial.println("2. Connect battery (ESCs will beep)");
  Serial.println("3. Type 'L' and press Enter");
  Serial.println("4. ESCs beep = calibrated!");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == 'H' || c == 'h') {
      Serial.println(">>> SENDING HIGH (2000us)");
      Serial.println(">>> CONNECT BATTERY NOW!");
      esc5.writeMicroseconds(2000);
      esc9.writeMicroseconds(2000);
    }
    
    if (c == 'L' || c == 'l') {
      Serial.println(">>> SENDING LOW (1000us)");
      Serial.println(">>> ESCs should beep!");
      esc5.writeMicroseconds(1000);
      esc9.writeMicroseconds(1000);
      Serial.println(">>> CALIBRATION DONE!");
      Serial.println(">>> Test with DiagnosticTool");
    }
  }
}
```

3. **Follow the steps:**
   - Upload code
   - Open Serial Monitor
   - Type **'H'** and press Enter
   - **Connect battery** (ESCs beep rapidly)
   - Type **'L'** and press Enter
   - ESCs beep confirmation = calibrated!

4. **Re-run DiagnosticTool** to verify fixed!

---

## 🔧 FIX #4: Check ESC Power

**If D5 and D9 ESCs have weak power:**

### **Check battery connections:**

```
Battery → Split to 4 ESCs:
  ├─ ESC for D3 (FL) → ✅ Works
  ├─ ESC for D5 (FR) → ❌ Weak?
  ├─ ESC for D6 (RR) → ✅ Works
  └─ ESC for D9 (RL) → ❌ Weak?
```

**Check:**
- Are D5 and D9 ESC power wires firmly connected to battery?
- Is battery connector loose?
- Try different battery connector

---

## 🔧 FIX #5: Verify Motor Connections

**For each motor that doesn't spin well:**

1. **Check 3 motor wires** connected to ESC
2. **Try swapping any 2 motor wires** (reverses direction)
3. **If motor still doesn't spin → motor is bad**

---

## 🎯 Quick Decision Tree

**Motors D5 and D9 don't spin well:**

### **Do they make beeping sounds?**
**YES** → ESC not calibrated
- **Fix:** Calibrate ESCs (Fix #3)

**NO** → Continue...

### **Do they spin very slowly?**
**YES** → Weak power or bad calibration
- **Fix:** Check battery connections (Fix #4)
- **Fix:** Calibrate ESCs (Fix #3)

### **Do they not spin at all?**
**YES** → Signal or power issue
- **Fix:** Check wiring (Fix #1)
- **Fix:** Swap test (Fix #2)

---

## 🧪 After Fixing - Verify:

**Re-run DiagnosticTool:**

```
Testing Front Left (D3)... ✅ SPINNING
Testing Front Right (D5)... ✅ SHOULD WORK NOW!
Testing Rear Right (D6)... ✅ SPINNING
Testing Rear Left (D9)... ✅ SHOULD WORK NOW!
```

**If all 4 motors work:**
- ✅ Upload FlightController.ino
- ✅ Press Button 3 to ARM
- ✅ Test stabilization!

---

## 💡 Most Likely Solution:

**Based on "don't spin well" (not "don't spin at all"):**

**→ ESCs need calibration!**

**Try FIX #3 first:**
1. Upload calibration code
2. Send 'H', connect battery, send 'L'
3. ESCs beep = calibrated
4. Re-test with DiagnosticTool

**This fixes 80% of "motors spin weak" problems!**

---

## 📋 Quick Checklist:

For motors D5 and D9:

- [ ] Signal wires firmly in D5 and D9 pins
- [ ] ESC grounds connected to Arduino GND
- [ ] ESC power wires connected to battery
- [ ] Motor 3 wires connected to ESC
- [ ] ESCs calibrated (beep confirmation)
- [ ] Battery charged (>11V)
- [ ] Tested with DiagnosticTool

---

## 🚀 Once Fixed:

**Your drone will be ready because:**
- ✅ MPU6050 already working (found at 0x68)
- ✅ Motors D3 and D6 already working
- ✅ Just need D5 and D9 fixed
- ✅ Then stabilization will work perfectly!

---

**Try ESC calibration first (Fix #3) - it's the most common fix!**

**Then tell me: Do D5 and D9 work after calibration?**
