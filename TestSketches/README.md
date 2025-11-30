# NRF24L01 Test Sketches

Quick diagnostic sketches to troubleshoot communication issues.

---

## 📁 Files

1. **NRF_Test_Single.ino** - Test if module is detected
2. **NRF_Test_Transmitter.ino** - Upload to Remote Controller
3. **NRF_Test_Receiver.ino** - Upload to Flight Controller

---

## 🔧 How to Use

### Step 1: Test Detection

**Upload `NRF_Test_Single.ino` to BOTH Arduinos (one at a time)**

**For Flight Controller:**
```cpp
#define CE_PIN 4    // ← Use 4 for FC
```

**For Remote Controller:**
```cpp
#define CE_PIN 9    // ← Use 9 for RC
```

**Expected Output:**
```
================================
   NRF24L01 MODULE TEST
================================

Testing with CE=D4, CSN=D10

Initializing NRF24L01...

✅✅✅ SUCCESS! ✅✅✅
NRF24L01 DETECTED!

Module Details:
-------------------
STATUS: 0x0e
...

✅ Hardware test PASSED!
Wiring is correct.
```

**If you see "FAILED":**
- Check wiring (see FIX_NO_COMMUNICATION.md)
- Add 10μF capacitor
- Verify 3.3V power
- Try different NRF module

---

### Step 2: Test Communication

Once both modules detect successfully:

**A. Upload to Flight Controller:**
- Upload `NRF_Test_Receiver.ino`
- Open Serial Monitor (115200 baud)
- Should show "Listening for messages..."

**B. Upload to Remote Controller:**
- Upload `NRF_Test_Transmitter.ino`
- Open Serial Monitor (115200 baud)
- Should show "Ready to transmit!"

**Expected Output:**

*Transmitter (RC):*
```
Ready to transmit!
Sending test messages...

✅ TX #1 - SUCCESS | Success Rate: 100.0% (1/1)
✅ TX #2 - SUCCESS | Success Rate: 100.0% (2/2)
✅ TX #3 - SUCCESS | Success Rate: 100.0% (3/3)
```

*Receiver (FC):*
```
Listening for messages...

✅ RX #1 - Received: 'Test #0'
✅ RX #2 - Received: 'Test #1'
✅ RX #3 - Received: 'Test #2'
```

---

## ✅ Success Criteria

**Both tests MUST pass before using main firmware!**

### Test 1: Detection
- [ ] FC shows "SUCCESS! NRF24L01 DETECTED!"
- [ ] RC shows "SUCCESS! NRF24L01 DETECTED!"

### Test 2: Communication
- [ ] Transmitter shows "TX SUCCESS"
- [ ] Receiver shows "RX Received"
- [ ] Success rate >95%

---

## ❌ Troubleshooting

### Module not detected
→ See `FIX_NO_COMMUNICATION.md` Section 2

### Transmitter fails
→ Check capacitor, power supply

### Receiver gets nothing
→ Check both modules on same channel (103)
→ Move closer together (<1 meter)
→ Check antennas

### Low success rate (<90%)
→ Add bigger capacitor (100μF)
→ Use external 3.3V regulator
→ Check for interference

---

## 🔄 After Tests Pass

Once both test sketches work:

1. ✅ Re-upload main firmware:
   - `FlightController.ino` to FC
   - `RemoteController.ino` to RC

2. ✅ Test system:
   - Power both on
   - Should hear double beep
   - RC serial shows "CONNECTED"

3. ✅ Ready to calibrate and fly!

---

## 📊 Interpreting Results

| Test Result | Meaning | Next Step |
|-------------|---------|-----------|
| Both modules detect | ✅ Hardware OK | Test communication |
| Only one detects | ⚠️ Bad module or wiring | Check failed module |
| Neither detects | ❌ Power or wiring issue | Check power, capacitors |
| TX works, RX nothing | ⚠️ One-way only | Check RX antenna, power |
| Low success rate | ⚠️ Weak signal | Add capacitor, check power |
| Both tests pass | ✅ Ready! | Upload main firmware |

---

**Use these test sketches BEFORE troubleshooting the main firmware!**

