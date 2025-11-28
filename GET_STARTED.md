# 🚀 GET STARTED - Drone Flight Control System

## 👋 Welcome!

You now have a **complete DIY drone flight control system**. This guide will get you flying in the fastest way possible.

---

## ⚡ 5-MINUTE OVERVIEW

### What You Have

✅ **Flight Controller firmware** - Stabilization + altitude hold  
✅ **Remote Controller firmware** - 4-channel wireless control  
✅ **Complete documentation** - 1500+ lines of guides  
✅ **Pin mappings** - All conflicts resolved  
✅ **Testing procedures** - Systematic verification  
✅ **Safety features** - Auto-disarm, tilt limits, signal loss protection  

---

## 📋 BEFORE YOU START

### ⚠️ CRITICAL: You Need These

**Hardware:**
- 2× Arduino Nano
- 2× NRF24L01 modules (+ 10µF capacitors!)
- 1× MPU6050 (gyro/accelerometer)
- 1× MS5611 (barometer)
- 4× Brushless motors + ESCs
- 1× Buzzer
- 1× LED
- 2× Push buttons
- 2× SPDT switches
- 2× 2-axis joysticks
- LiPo battery (3S recommended)
- Quadcopter frame
- Propellers

**Software (Arduino Libraries):**
- RF24
- Smoothed
- MS5611
- Wire, SPI, Servo, EEPROM (built-in)

---

## 🎯 YOUR 10-STEP PATH TO FLIGHT

### Step 1: Read the Overview (5 min)
📖 **Open: README.md**
- Understand what the system does
- Check feature list
- Review requirements

### Step 2: Understand Pin Connections (10 min)
📖 **Open: PIN_MAPPING.md**
- Learn which pins connect where
- Understand why pins were changed from original spec
- Note the corrected mappings

### Step 3: Wire the Hardware (2 hours)
📖 **Open: WIRING_DIAGRAM.txt**
- Follow ASCII diagrams
- Wire Flight Controller
- Wire Remote Controller
- ⚠️ **CRITICAL:** Add 10µF capacitor to each NRF24L01!
- ⚠️ **CRITICAL:** NRF uses 3.3V, NOT 5V!

### Step 4: Install Libraries (5 min)
In Arduino IDE:
1. Tools → Manage Libraries
2. Install: RF24
3. Install: Smoothed
4. Install: MS5611

### Step 5: Upload FC Firmware (5 min)
1. Open `Drone_Flight_Control.ino`
2. Ensure `Barometer.ino`, `Kalman_Filter.ino`, `Gyro.h`, `Gyro.cpp` are in same folder
3. Board: Arduino Nano
4. Port: Select your FC board
5. Upload!

### Step 6: Upload RC Firmware (5 min)
1. Open `Controller.ino`
2. Board: Arduino Nano
3. Port: Select your RC board
4. Upload!

### Step 7: Test Communication (5 min)
📖 **Open: USER_MANUAL.md → Communication Tests**
1. Power RC first
2. Power FC second
3. Listen for beep-beep (link established)
4. LED on FC blinks = good!

### Step 8: Calibrate Sensors (5 min)
📖 **Open: USER_MANUAL.md → Calibration Procedure**
1. Place drone on level surface
2. Ensure arm switch DISARMED (LED stays ON)
3. Press calibration button (A6) for 2 seconds
4. Wait for completion beeps
5. Done!

### Step 9: Test Motors (30 min)
📖 **Open: TESTING_CHECKLIST.md → Motor Tests**
⚠️ **REMOVE PROPELLERS!**
1. Arm the system
2. Press smooth start button (A7)
3. Verify all 4 motors spin
4. Check rotation directions:
   - FL: CCW ↺
   - FR: CW ↻
   - RR: CCW ↺
   - RL: CW ↻
5. Fix any wrong directions (swap 2 motor wires)

### Step 10: First Flight! (15 min)
📖 **Open: USER_MANUAL.md → Flight Operations**
1. Install propellers (correct types!)
2. Clear open area
3. Arm system
4. Smooth motor start (optional, you've tested already)
5. Slowly increase throttle
6. Hover at 1m height
7. Test basic controls
8. Land
9. Celebrate! 🎉

---

## 📚 DOCUMENT QUICK ACCESS

| Purpose | File | Time |
|---------|------|------|
| **Overview** | README.md | 5 min |
| **Complete Guide** | USER_MANUAL.md | 1 hour |
| **Wiring** | WIRING_DIAGRAM.txt | 30 min |
| **Pins** | PIN_MAPPING.md | 10 min |
| **Quick Lookup** | QUICK_REFERENCE.md | 2 min |
| **Testing** | TESTING_CHECKLIST.md | As needed |
| **All Files** | INDEX.md | 5 min |

---

## 🎮 CONTROLS REMINDER

```
LEFT STICK (Throttle & Yaw):
     Throttle
        ↑
Yaw ←   ● → Yaw
        ↓
     Throttle

RIGHT STICK (Pitch & Roll):
      Pitch
        ↑
Roll ←  ● → Roll
        ↓
      Pitch
```

**Switches:**
- Arm Switch (D4): HIGH = Disarmed (safe), LOW = Armed
- Alt Hold Switch (D7): LOW = Active, HIGH = Off

**Buttons:**
- Calibration (A6): Hold 2s when disarmed
- Smooth Start (A7): Press when armed to test motors

---

## 🔔 STATUS INDICATORS

**LED:**
- Always ON → Disarmed (safe mode)
- Blinking → Armed & linked
- OFF → Signal loss!

**Buzzer:**
- Beep-Beep-Beeeep → Startup
- Beep-Beep → Link OK
- Repeating beep → Kill switch active

---

## 🐛 COMMON FIRST-TIME ISSUES

### Issue: No radio link
**Fix:**
- Check NRF wiring (especially CE pin)
- Verify 10µF capacitor on NRF
- Ensure 3.3V power (NOT 5V!)

### Issue: One motor doesn't spin
**Fix:**
- Check ESC signal wire to D3/D5/D6/D9
- Verify ESC has power
- Test with another motor

### Issue: Drone drifts
**Fix:**
- Recalibrate on perfectly level surface
- Check propeller balance
- Verify frame is straight

### Issue: Motors spin wrong direction
**Fix:**
- Swap any 2 of the 3 motor wires
- FL & RR should spin CCW ↺
- FR & RL should spin CW ↻

**For more:** USER_MANUAL.md → Troubleshooting

---

## ⚠️ SAFETY CHECKLIST

Before every flight:
- [ ] Battery fully charged
- [ ] All wiring secure
- [ ] Propellers undamaged
- [ ] Calibration done
- [ ] Clear flying area
- [ ] Link established (beep-beep)
- [ ] Motors tested without props
- [ ] Motor directions verified

**NEVER:**
- ❌ Fly near people
- ❌ Skip calibration
- ❌ Use damaged props
- ❌ Arm while holding drone

---

## 🎓 LEARNING PROGRESSION

**Day 1:** Build & wire  
**Day 2:** Upload firmware & calibrate  
**Day 3:** Test motors (no props)  
**Day 4:** First hover (1m height)  
**Day 5:** Basic maneuvers  
**Day 6:** Altitude hold practice  
**Day 7:** Confident flying!  

Take your time. Safety first!

---

## 📞 NEED HELP?

1. **Check:** USER_MANUAL.md → Troubleshooting section
2. **Verify:** TESTING_CHECKLIST.md → Relevant test
3. **Review:** PIN_MAPPING.md → Wiring
4. **Debug:** Serial Monitor (57600 baud) on FC

---

## 🎯 YOUR NEXT HOUR

**Right now:**

1. ☐ Read this document completely (you're almost done!)
2. ☐ Read README.md (5 minutes)
3. ☐ Review PIN_MAPPING.md (10 minutes)
4. ☐ Gather all hardware components
5. ☐ Install Arduino libraries

**Then:**

6. ☐ Start wiring using WIRING_DIAGRAM.txt
7. ☐ Take your time, double-check everything
8. ☐ Follow USER_MANUAL.md step by step

---

## ✅ YOU'RE READY!

Everything you need is in these files:

📂 **15 Files Total:**
- 6 code files (ready to upload)
- 9 documentation files (guides + references)

All problems solved:
✅ Pin conflicts fixed  
✅ NRF communication reliable (ACK enabled)  
✅ Smooth motor start implemented  
✅ MS5611 calibration integrated  
✅ Safety features enhanced  
✅ Complete documentation provided  

---

## 🚁 START YOUR BUILD!

**Your first action:** Open README.md

**Your second action:** Follow the 10-step path above

**Your final action:** Fly your DIY drone!

---

## 💡 HELPFUL TIPS

1. **Don't rush** - Proper wiring takes time
2. **Test components individually** - Easier to debug
3. **Keep QUICK_REFERENCE.md handy** - You'll use it often
4. **Join the community** - Share your build!
5. **Document your changes** - If you modify anything
6. **Have fun!** - This is a learning experience

---

## 📈 ESTIMATED TIME

| Task | Time |
|------|------|
| Reading documentation | 2 hours |
| Wiring hardware | 3 hours |
| Software setup | 30 min |
| Calibration & testing | 1 hour |
| First flight practice | 2 hours |
| **TOTAL** | **~9 hours** |

Spread over a weekend for best results.

---

## 🎊 AFTER YOUR FIRST SUCCESSFUL FLIGHT

1. ✅ Mark it in your calendar!
2. ✅ Document any issues you found
3. ✅ Fine-tune PID values if needed
4. ✅ Practice basic maneuvers
5. ✅ Try altitude hold mode
6. ✅ Consider upgrades (GPS, FPV, etc.)

---

## 🌟 GOOD LUCK!

You have everything you need. Take it step by step, follow the guides, and you'll be flying soon!

**Questions?** Check USER_MANUAL.md

**Ready?** Start with README.md

**Let's build! 🚁**

---

**Version 2.0 | 2025-11-28 | Production Ready**
