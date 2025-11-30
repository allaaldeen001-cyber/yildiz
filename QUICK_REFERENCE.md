# Quadcopter Quick Reference Guide

## 🚁 One-Page Cheat Sheet

### Emergency Contacts
- **Disarm:** Flip SW1 to OFF position
- **Failsafe:** Signal lost = Auto-disarm after 1 second

---

## 📋 Pre-Flight Checklist (30 seconds)
```
□ Battery charged (>11.1V for 3S)
□ Propellers secure & correct direction
□ Remote on, shows "CONN"
□ Flight controller calibrated (2 beeps)
□ Area clear (5m radius minimum)
□ Weather OK (no rain/strong wind)
```

---

## 🔌 Pin Quick Reference

### Flight Controller
| Pin | Function | Pin | Function |
|-----|----------|-----|----------|
| D2  | MPU INT  | D7  | LED |
| D3  | Motor FL | D8  | Buzzer |
| D4  | nRF CE   | D9  | Motor RL |
| D5  | Motor FR | D10 | nRF CSN |
| D6  | Motor RR | A4  | MPU SDA |
| A5  | MPU SCL  | 3.3V| Radio+MPU |

### Remote Controller
| Pin | Function | Pin | Function |
|-----|----------|-----|----------|
| A0  | Throttle | D4  | Button 1 |
| A1  | Yaw      | D5  | Button 2 |
| A2  | Pitch    | D6  | Button 3 |
| A3  | Roll     | D7  | Button 4 |
| D2  | ARM SW   | D9  | nRF CE |
| D3  | MODE SW  | D10 | nRF CSN |

---

## 🎮 Controls

### Stick Layout
```
    LEFT STICK          RIGHT STICK
    
       UP                   UP
    Throttle             Pitch Fwd
       |                     |
   L --+-- R           L --+-- R
    Yaw     Yaw       Roll   Roll
       |                     |
      DOWN                  DOWN
   Throttle             Pitch Back
```

### Switch Functions
- **SW1 (D2):** ARM/DISARM (must be <5% throttle to arm)
- **SW2 (D3):** Flight Mode (Stabilize/Acro)

---

## 🔧 Startup Sequence (1 minute)

1. **Power Remote First**
   - Wait for "System Ready!" (2 sec)

2. **Place Drone Level**
   - Must be perfectly flat

3. **Power Flight Controller**
   - 1 long beep = starting cal
   - Wait 6 seconds
   - 2 short beeps = ready

4. **Verify Connection**
   - Remote shows "CONN"

5. **ARM (throttle down!)**
   - Flip SW1 to ARM
   - 1 beep confirms

---

## 🚨 Status Indicators

### LED Patterns
| Pattern | Meaning |
|---------|---------|
| Blinking | Disarmed (safe) |
| Solid | Armed (ready) |
| Fast blink | Calibrating |

### Beep Codes
| Beeps | Meaning |
|-------|---------|
| 1 long | Starting calibration |
| 2 short | System ready / Disarmed |
| 1 short | Armed |
| 3 short | FAILSAFE! |

---

## ⚙️ Motor Configuration

```
        FRONT
    
    FL     FR
   CCW     CW
     \   /
      \ /
       X
      / \
     /   \
   CW    CCW
   RL     RR
   
     REAR
```

**Propeller Check:**
- FL & RR: Counter-clockwise (CCW)
- FR & RL: Clockwise (CW)

---

## 🔋 Battery Voltages (3S LiPo)

| Status | Voltage | Action |
|--------|---------|--------|
| Full | 12.6V | Ready to fly |
| Good | 11.4V+ | Safe flight |
| Warning | 10.5V | Land soon |
| Critical | 10.0V | **LAND NOW!** |

---

## 🐛 Quick Troubleshooting

### "NO SIGNAL" on Remote
1. Check nRF24L01+ has 10µF capacitor
2. Verify 3.3V power (NOT 5V!)
3. Restart both devices

### Motors Don't Spin
1. Check throttle >10%
2. Verify armed (LED solid)
3. Check ESC calibration done

### Drone Flips on Takeoff
1. Check motor directions (see diagram)
2. Swap any two motor wires to reverse
3. Recalibrate on level surface

### Oscillates/Shakes
1. Reduce P gain by 10%
2. Reduce D gain by 10%
3. Check propellers balanced

---

## 🎯 First Flight Steps (5 minutes)

**⚠️ REMOVE PROPELLERS FOR FIRST TEST! ⚠️**

1. **Throttle at minimum**
2. **Flip SW1 to ARM** (1 beep)
3. **Slowly increase throttle** to 50%
4. **Check all motors spin**
5. **Check motor directions** (see diagram)
6. **Lower throttle, DISARM**

**WITH PROPELLERS:**

7. **Install propellers** (tighten securely)
8. **Place in open area** (5m clearance)
9. **Stand behind drone**
10. **ARM, throttle slowly** to 50%
11. **Hover at 1-2m height**
12. **Small stick movements** only
13. **Land gently, DISARM**

---

## 📊 Default PID Values

```cpp
Roll/Pitch:
  P = 1.4
  I = 0.05
  D = 18.0

Yaw:
  P = 3.0
  I = 0.02
  D = 0.0
```

**Tuning Tips:**
- Oscillating? → Reduce P & D
- Sluggish? → Increase P
- Drift? → Increase I
- Change by 10% max each time

---

## 🔧 Serial Monitor Commands

**Baud Rate:** 115200

### Flight Controller Output
```
ARMED         = Ready to fly
DISARMED      = Safe mode
FAILSAFE!     = Signal lost
```

### Remote Controller Output
```
T:1000 R:0 P:0 Y:0 | CONN [SAFE]
 ↑     ↑  ↑  ↑      ↑     ↑
Thr   Roll  Yaw   Status Armed
      Pitch
```

---

## 🛡️ Safety Rules

1. ⚠️ **NEVER** fly near people
2. ⚠️ **NEVER** fly above 400ft
3. ⚠️ **ALWAYS** remove props for testing
4. ⚠️ **ALWAYS** check battery before flight
5. ⚠️ **KEEP** fingers from spinning props

---

## 🔌 Critical Connections

**DON'T FORGET:**
- ✅ 10µF capacitor on nRF24L01+
- ✅ nRF24L01+ uses 3.3V (NOT 5V!)
- ✅ MPU6050 uses 3.3V (NOT 5V!)
- ✅ Only ONE ESC BEC to Arduino VIN

---

## 📏 Specification Summary

| Parameter | Value |
|-----------|-------|
| Control Loop | 250Hz (4ms) |
| Radio Rate | 50Hz (20ms) |
| Max Range | ~1000m |
| Latency | ~25ms |
| Max Tilt | ±50° |
| Battery | 3S/4S LiPo |
| Flight Time | 5-10 min |

---

## 📱 File Locations

```
/workspace/
├── FlightController/FlightController.ino
├── RemoteController/RemoteController.ino  
├── README.md (full documentation)
├── OPERATION_GUIDE.md (detailed flying)
├── WIRING_DIAGRAMS.md (connections)
└── LIBRARIES_INSTALLATION.md (setup)
```

---

## 🆘 Emergency Procedures

### Loss of Control
1. **Release all sticks** to center
2. **If spinning:** DISARM immediately (SW1 off)
3. **If flying away:** DISARM (triggers failsafe)

### Flyaway
1. DISARM with SW1
2. Or power off remote (failsafe)
3. Note direction for recovery

### Crash Landing
1. DISARM immediately
2. Disconnect battery
3. Check for damage before next flight

---

## ✅ Daily Maintenance

**Before Each Flight:**
- [ ] Check propeller tightness
- [ ] Check battery voltage
- [ ] Check all connections secure
- [ ] Test ARM/DISARM function

**After Each Flight:**
- [ ] Inspect propellers for cracks
- [ ] Check motor temps (should be warm, not hot)
- [ ] Check screws for looseness

---

## 📞 Getting Help

1. Check Serial Monitor (115200 baud)
2. Review OPERATION_GUIDE.md
3. Check WIRING_DIAGRAMS.md
4. Verify LIBRARIES_INSTALLATION.md

---

## 🎓 Recommended Learning Path

**Week 1:** Ground testing (no props)
**Week 2:** Hover practice
**Week 3:** Forward flight
**Week 4:** Advanced maneuvers

---

**FLY SAFE! Have fun! 🚁✨**

---

*Keep this guide handy during flights!*
*Print it out or keep on phone/tablet*
