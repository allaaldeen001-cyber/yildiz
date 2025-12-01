# 🔧 TROUBLESHOOTING GUIDE - Fix Your Issues NOW!

## 🚨 REPORTED PROBLEMS:

### Problem 1: No MPU Effect (Motors don't respond to tilting)
### Problem 2: Not all motors spin

Let's fix these step by step!

---

## 🔍 STEP 1: Run Diagnostic Tool (5 minutes)

### Upload the diagnostic sketch:

1. **Open Arduino IDE**
2. **Open:** `DiagnosticTool/DiagnosticTool.ino`
3. **Upload** to flight controller
4. **Open Serial Monitor** (115200 baud)
5. **Read all results carefully**

The diagnostic tool will test:
- ✅ Buzzer and LED
- ✅ I2C bus (finds MPU6050)
- ✅ MPU6050 sensor readings
- ✅ All 4 motors individually

### What to look for:

**I2C Scan should show:**
```
✓ FOUND device at 0x68  ← MPU6050!
```

**If you see:**
```
✗ ERROR: No I2C devices found!
```
→ **MPU6050 is NOT connected!** See Fix #1 below

**Motor test should show each motor spinning:**
```
Testing Front Left (D3)... Should spin for 2 seconds
Testing Front Right (D5)... Should spin for 2 seconds
Testing Rear Right (D6)... Should spin for 2 seconds
Testing Rear Left (D9)... Should spin for 2 seconds
```

**Which motors DON'T spin?** Write them down!

---

## 🔧 FIX #1: MPU6050 Not Found / No I2C Device

### Symptoms:
- Diagnostic shows "No I2C devices found"
- Or shows device at wrong address
- No stabilization effect

### Solution - Check Wiring:

```
MPU6050 Module → Arduino Nano
═══════════════════════════════
VCC  →  3.3V  (NOT 5V!!!)
GND  →  GND
SCL  →  A5
SDA  →  A4
INT  →  D2 (optional)
```

### Common mistakes:

❌ **WRONG: VCC → 5V**
✅ **CORRECT: VCC → 3.3V**

❌ **WRONG: SDA/SCL swapped**
✅ **CORRECT: SDA→A4, SCL→A5**

❌ **WRONG: Loose connections**
✅ **CORRECT: Solder or secure firmly**

### Test after fixing:
1. Re-run diagnostic tool
2. Should see: `✓ FOUND device at 0x68`
3. Should see changing values when you tilt MPU

---

## 🔧 FIX #2: Motors Not Spinning

### Symptoms:
- Some motors don't spin at all
- Or all motors don't spin
- Or motors spin weakly

### Check which motors DON'T work:

Run diagnostic and note results:
- [ ] Front Left (FL, D3) works?
- [ ] Front Right (FR, D5) works?
- [ ] Rear Right (RR, D6) works?
- [ ] Rear Left (RL, D9) works?

---

### Case A: NO motors spin at all

**Possible causes:**

1. **ESCs not powered**
   - Check battery connected to all ESCs
   - Check battery charged (>11V)
   - Check ESC power wires

2. **ESCs not calibrated**
   - See ESC calibration procedure below

3. **Wrong signal wires**
   - Check signal wires on correct pins

4. **ESCs not grounded**
   - All ESC grounds must connect to Arduino GND

### Solution:

**Check power:**
```
Battery → ESC power wires (thick red/black)
ESC BEC → Arduino VIN (one ESC only!)
All ESC GND → Arduino GND
```

**Check signals:**
```
ESC1 signal → Arduino D3 (FL)
ESC2 signal → Arduino D5 (FR)
ESC3 signal → Arduino D6 (RR)
ESC4 signal → Arduino D9 (RL)
```

---

### Case B: Some motors spin, some don't

**Example: FL and FR work, but RR and RL don't**

This means:
- ✅ ESCs are powered (at least 2 work)
- ✅ Code is working (at least 2 respond)
- ❌ Problem with specific ESC or wiring

**For EACH motor that doesn't work:**

1. **Check signal wire connection:**
   - Is wire firmly connected to Arduino pin?
   - Is it on the correct pin?
   - FL=D3, FR=D5, RR=D6, RL=D9

2. **Check ESC power:**
   - Is battery wire connected to this ESC?
   - Is ESC getting power? (LED on ESC?)

3. **Check motor connection:**
   - Are 3 motor wires connected to ESC?
   - Are connections secure?

4. **Swap test:**
   - Swap signal wire with working motor
   - If problem moves → Bad ESC or motor
   - If problem stays → Bad Arduino pin or wiring

---

### Case C: Motors spin but very weak

**Symptoms:**
- Motors barely turn
- Sound is wrong
- Very slow

**Causes:**
1. ESC not calibrated
2. Low battery
3. Wrong motor type for ESC
4. ESC settings wrong

**Fix:** Calibrate ESCs (see below)

---

## 🔧 ESC CALIBRATION PROCEDURE

⚠️ **DO THIS IF MOTORS DON'T SPIN PROPERLY!**

### Method 1: All at once

1. **Disconnect** battery from ESCs
2. Upload `DiagnosticTool.ino` to Arduino
3. **Power Arduino** from USB
4. **Connect battery** to ESCs
5. Wait for beep sequence from ESCs
6. ESCs should beep and be calibrated

### Method 2: Manual calibration

1. **Create this simple sketch:**

```cpp
#include <Servo.h>

Servo esc1, esc2, esc3, esc4;

void setup() {
  Serial.begin(115200);
  
  esc1.attach(3);
  esc2.attach(5);
  esc3.attach(6);
  esc4.attach(9);
  
  Serial.println("ESC CALIBRATION");
  Serial.println("Disconnect battery NOW!");
  Serial.println("Send 'H' for HIGH point");
  Serial.println("Send 'L' for LOW point");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == 'H' || c == 'h') {
      Serial.println("Setting HIGH (2000us)");
      Serial.println("Connect battery NOW!");
      esc1.writeMicroseconds(2000);
      esc2.writeMicroseconds(2000);
      esc3.writeMicroseconds(2000);
      esc4.writeMicroseconds(2000);
    }
    
    if (c == 'L' || c == 'l') {
      Serial.println("Setting LOW (1000us)");
      esc1.writeMicroseconds(1000);
      esc2.writeMicroseconds(1000);
      esc3.writeMicroseconds(1000);
      esc4.writeMicroseconds(1000);
      Serial.println("Calibration done!");
    }
  }
}
```

2. **Upload this sketch**
3. **Open Serial Monitor**
4. **Send 'H'** (for high point)
5. **Connect battery** (ESCs beep)
6. **Send 'L'** (for low point)
7. ESCs beep = calibrated!

---

## 🔧 FIX #3: MPU Works But No Stabilization Effect

### Symptoms:
- MPU6050 found on I2C (diagnostic shows it)
- MPU values change when tilted
- BUT motors don't respond when you tilt drone

### Diagnostic Test:

1. **Upload** `FlightController.ino` (not diagnostic)
2. **Open Serial Monitor** (115200 baud)
3. **Wait for calibration** (2 beeps)
4. **Tilt drone** (while DISARMED)

**You should see:**
```
DISARM | Mode:--- | Ang R:15.0 P:-5.2 | Gyro R:25.3 P:-10.2
              ↑ Changes when you tilt!
```

**If angles DON'T change:** MPU not reading correctly
**If angles DO change:** Good! Continue...

5. **ARM the drone:**
   - Throttle to minimum
   - Flip SW1 to ARM (or arm via remote)
   - Should see: `ARMED | Mode:ANGLE`

6. **Tilt drone** (motors should be at 1100)

**You should see:**
```
ARMED | Mode:ANGLE | Ang R:15.0 | PID R:-210 | Mot FL:890 FR:1310 RR:1310 RL:890
                         ↑              ↑              ↑ Motors respond!
```

### If motors DON'T respond when tilted:

**Check #1: Are you ARMED?**
```
ARM:1  ← Must show 1
```

**Check #2: Are motors spinning at all?**
```
Mot FL:1100 FR:1100 RR:1100 RL:1100  ← Should show 1100 when armed
```
If shows 1000 = not actually armed or motor issue

**Check #3: Do PID values change?**
```
PID R:-210 P:73  ← Should change when you tilt
```
If PID stays at 0 = not calculating stabilization

### Most likely cause: Not actually armed!

**To ARM properly:**
1. Flight controller must be connected to remote
2. Remote must show "✓CONN"
3. Throttle must be at minimum (<1050)
4. Flip SW1 to ARM on remote
5. Should hear 1 beep
6. Serial shows "ARMED"

---

## 🎯 QUICK FIXES SUMMARY

### Problem: No MPU effect
**Fix priority order:**

1. ✅ Check MPU6050 wiring (VCC=3.3V, SDA=A4, SCL=A5)
2. ✅ Run diagnostic - verify MPU found at 0x68
3. ✅ Verify angles change when tilted (Serial Monitor)
4. ✅ Verify actually ARMED (not just motors spinning)
5. ✅ Check remote connection (must show CONN)
6. ✅ Check PID values change when tilted
7. ✅ Check motor values change when tilted

### Problem: Motors don't spin
**Fix priority order:**

1. ✅ Check battery connected and charged
2. ✅ Check ESC power wires to battery
3. ✅ Check ESC signal wires to Arduino pins
4. ✅ Check all ESC grounds to Arduino GND
5. ✅ Run diagnostic - test each motor individually
6. ✅ Calibrate ESCs if needed
7. ✅ Swap test - identify bad ESC/motor

---

## 📋 SYSTEMATIC CHECKLIST

Work through this checklist:

### Power
- [ ] Battery voltage >11V
- [ ] Battery connected to all 4 ESCs
- [ ] One ESC BEC connected to Arduino VIN
- [ ] All ESC grounds connected to Arduino GND

### MPU6050 Wiring
- [ ] VCC → 3.3V (NOT 5V!)
- [ ] GND → GND
- [ ] SDA → A4
- [ ] SCL → A5
- [ ] Connections are solid (solder or crimped)

### Motor/ESC Wiring
- [ ] ESC1 signal → D3 (Front Left)
- [ ] ESC2 signal → D5 (Front Right)
- [ ] ESC3 signal → D6 (Rear Right)
- [ ] ESC4 signal → D9 (Rear Left)
- [ ] All 4 motors connected to ESCs
- [ ] ESCs calibrated

### Code
- [ ] Correct code uploaded (FlightController.ino)
- [ ] Libraries installed (RF24, Wire, Servo)
- [ ] No compilation errors
- [ ] Board = Arduino Nano
- [ ] Processor = ATmega328P (Old Bootloader)

### Testing
- [ ] Diagnostic tool ran successfully
- [ ] MPU6050 found at 0x68
- [ ] MPU angles change when tilted
- [ ] All 4 motors spin in diagnostic test
- [ ] Remote shows "CONN"
- [ ] Can ARM successfully

---

## 🆘 STILL NOT WORKING?

### Post your diagnostic results:

Run the diagnostic tool and copy/paste the output:

1. Did I2C scan find MPU6050?
2. Which motors spun in the test?
3. Do MPU values change when you tilt?
4. What does Serial Monitor show when armed?

---

## 💡 COMMON MISTAKES

### ❌ Mistake #1: MPU6050 on 5V
**Fix:** Use 3.3V! 5V can damage it!

### ❌ Mistake #2: SDA/SCL swapped
**Fix:** SDA=A4, SCL=A5 (not reversed!)

### ❌ Mistake #3: ESC not grounded
**Fix:** All ESC black wires → Arduino GND

### ❌ Mistake #4: Wrong signal pins
**Fix:** FL=D3, FR=D5, RR=D6, RL=D9

### ❌ Mistake #5: Not armed
**Fix:** Throttle low, flip SW1, hear beep

### ❌ Mistake #6: No propellers during test
**This is CORRECT!** Always remove props for testing!

---

## 🚀 ONCE FIXED:

After diagnostic shows all OK:

1. ✅ Upload `FlightController.ino`
2. ✅ Upload `RemoteController.ino`
3. ✅ Test stabilization (no props!)
4. ✅ Follow `HOW_TO_TEST_STABILIZATION.md`
5. ✅ Install props and fly!

---

**Run the diagnostic tool first, then come back here with results!**

**DiagnosticTool.ino is in `/workspace/DiagnosticTool/` folder**

**Good luck! 🔧✨**
