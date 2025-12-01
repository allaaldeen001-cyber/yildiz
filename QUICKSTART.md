# Quick Start Guide

Get your quadcopter drone flying in 30 minutes! ⚡

## Prerequisites
- Arduino IDE installed
- All components ready (see main README.md)
- Basic soldering skills

---

## 1️⃣ Install Software (5 minutes)

```bash
# Install Arduino IDE libraries:
1. Open Arduino IDE
2. Sketch → Include Library → Manage Libraries
3. Install: RF24, Adafruit MPU6050, MS5611 (by Rob Tillaart)
   Note: Adafruit MPU6050 will auto-install its dependencies
```

---

## 2️⃣ Wire Components (10 minutes)

### Flight Controller Quick Wiring:
```
MPU6050:  SDA→A4, SCL→A5, VCC→3.3V, GND→GND
MS5611:   SDA→A4, SCL→A5, VCC→3.3V, GND→GND (share with MPU)
nRF24:    CE→D4, CSN→D10, MOSI→D11, MISO→D12, SCK→D13, VCC→3.3V
          ⚠️ CRITICAL: 10µF capacitor on VCC/GND (prevents transmission fails!)
Motors:   ESC signals → D3, D5, D6, D9
          ESC grounds → Arduino GND (all)
          ONE ESC BEC → Arduino VIN
Buzzer:   + → D7, - → GND
LED:      + → D8, - → 220Ω → GND
```

### Remote Controller Quick Wiring:
```
Joysticks: VCC→5V, GND→GND
           Left Y→A0, Left X→A1, Right Y→A2, Right X→A3
Buttons:   D4-D7 to GND (4 buttons)
Switches:  D2-D3 to GND (2 switches)
nRF24:     CE→D9, CSN→D10, MOSI→D11, MISO→D12, SCK→D13, VCC→3.3V
           ⚠️ CRITICAL: 10µF capacitor on VCC/GND (prevents transmission fails!)
```

---

## 3️⃣ Upload Code (5 minutes)

### Flight Controller:
```
1. Open: FlightController/FlightController.ino
2. Tools → Board → Arduino Nano
3. Tools → Processor → ATmega328P (Old Bootloader)
4. Select correct COM port
5. Click Upload ✅
```

### Remote Controller:
```
1. Open: RemoteController/RemoteController.ino
2. Same settings as above
3. Click Upload ✅
```

---

## 4️⃣ Test Without Props (5 minutes)

**⚠️ REMOVE ALL PROPELLERS!**

```
1. Power on remote first
2. Power on drone second
3. Press Button 1 → Calibrate (place on flat surface)
4. Press Button 3 → ARM
5. Increase throttle slowly
6. All 4 motors should spin ✅
7. Check directions:
   M1 (FL): CCW ↺
   M2 (FR): CW ↻
   M3 (RR): CCW ↺
   M4 (RL): CW ↻

If wrong: Swap ANY 2 motor wires
```

---

## 5️⃣ First Flight (5 minutes)

```
1. Install propellers (match CW/CCW to motors)
2. Go to open outdoor area
3. Battery charged? Check! ✅
4. Power on remote → Power on drone
5. Calibrate (Button 1) on flat ground
6. ARM (Button 3)
7. Slowly increase throttle
8. Hover at ~50% throttle
9. Make SMALL corrections
10. Land gently (reduce throttle)

🎉 SUCCESS! You're flying!
```

---

## 🆘 Quick Troubleshooting

| Problem | Quick Fix |
|---------|-----------|
| Motors won't spin | Check ESC calibration, battery voltage |
| Flips on takeoff | Check motor directions, propeller installation |
| Won't arm | Throttle must be at minimum, calibrate first |
| "Transmission failed" | ADD 10µF CAPACITOR on nRF24! Check 3.3V power |
| Radio not working | Check nRF24 power (3.3V), wiring, capacitor |
| Buzzer too loud | Already fixed! Uses tone() - adjust frequency if needed |
| Sensor errors | Check I2C wiring (A4, A5), use short wires |
| Oscillates | Reduce PID P gains (Kp_roll = 1.0) |

---

## 📋 Safety Checklist

```
☐ Remove props for all testing
☐ Battery fully charged (12.6V)
☐ All screws tight
☐ Fly outdoors only
☐ Clear area (no people/obstacles)
☐ Safety glasses on
☐ Know how to disarm (Button 3)
```

---

## 🎯 Control Reference

```
Left Stick:
  ↑↓  Throttle (altitude)
  ←→  Yaw (rotate)

Right Stick:
  ↑↓  Pitch (forward/back)
  ←→  Roll (left/right)

Buttons:
  1: Calibrate sensors
  2: Motor test
  3: ARM/DISARM
  4: Auto landing

Switch 2: ANGLE (easy) / ACRO (hard)
```

---

## 📚 Full Documentation

For detailed guides, see:
- `README.md` - Complete project overview
- `docs/SETUP_GUIDE.md` - Step-by-step assembly
- `docs/WIRING_DIAGRAM.md` - Detailed wiring
- `docs/COMPONENT_GUIDE.md` - How components work
- `docs/TROUBLESHOOTING.md` - Fix common issues

---

## 🚀 Ready to Fly?

**Quick Start Success Path:**
```
✅ Software installed
✅ Hardware wired
✅ Code uploaded
✅ Motors tested (no props)
✅ First hover successful
✅ Ready for advanced flights!
```

**Fly safe and have fun! 🚁**

---

## 💡 Pro Tips

1. **First flight:** Use ANGLE mode (Switch 2 UP)
2. **Calibrate:** Do it every time you move the drone
3. **Throttle:** Hover is usually around 50%
4. **Practice:** Start with just hovering in place
5. **Battery:** Land when it reaches 11.0V (don't over-discharge)
6. **PID Tuning:** Stick with defaults until you can hover steadily
7. **Wind:** Don't fly if wind >10 mph
8. **Backup parts:** Keep extra props and motors

---

## 🎓 Learning Path

**Day 1:** Hover practice (stay in one spot)
**Day 2-3:** Basic movements (forward, back, left, right)
**Day 4-5:** Smooth landings and takeoffs
**Week 2:** Figure-8 patterns, rotating while hovering
**Week 3:** Try ACRO mode (experts only)
**Week 4+:** Add features (GPS, FPV camera, etc.)

---

## 📞 Need Help?

Check these resources:
1. `docs/TROUBLESHOOTING.md` - Most common issues
2. Serial Monitor (115200 baud) - Debug messages
3. Multimeter - Test voltages and connections
4. Community forums - Ask questions

---

**You're now ready to fly! 🎉**

Start with hovering, be patient, and always prioritize safety. Every expert pilot started exactly where you are now. Have fun building and flying your Arduino quadcopter!
