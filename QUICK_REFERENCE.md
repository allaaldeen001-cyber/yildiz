# 🚀 Quick Reference Card

## 🎮 RC Controls

| Control | Function |
|---------|----------|
| Left Stick ↑↓ | **Throttle** (UP = More Power) ✓ |
| Left Stick ←→ | **Yaw** (Rotate) |
| Right Stick ↑↓ | **Pitch** (Forward/Back) |
| Right Stick ←→ | **Roll** (Strafe Left/Right) |
| Button 1 (Hold 2s) | **Calibrate Level** |
| Button 2 (Hold 2s) | **Arm/Disarm** |
| Switch 1 | **Emergency Stop** |
| Switch 2 | **Altitude Hold** |

---

## 📊 Serial Monitor Commands

### RC Transmitter (57600 baud):
```
T:1500 | Y:0.0 | R:0.0 | P:0.0 | Link:✓OK | Rate:99% [EXCELLENT]
```
- **T**: Throttle (1000-2000)
- **Y**: Yaw (-50 to +50)
- **R**: Roll (-50 to +50)
- **P**: Pitch (-50 to +50)
- **Link**: Connection status
- **Rate**: Success rate %

### Flight Controller (57600 baud):
```
Status: ARMED ✓
Angles -> R:2.3° P:-1.5° Y:45.2°
Throttle: 1450 | Motors: FL:1455 FR:1448 RL:1445 RR:1452
Radio: ✓ OK (142 pkts/s)
```

---

## 🔧 Pin Connections

### Flight Controller:
| Component | Pin |
|-----------|-----|
| ESC Front Left | D3 |
| ESC Front Right | D5 |
| ESC Rear Right | D6 |
| ESC Rear Left | D9 |
| nRF24 CE | D4 |
| nRF24 CSN | D10 |
| MPU6050 SDA | A4 |
| MPU6050 SCL | A5 |
| MS5611 SDA | A4 |
| MS5611 SCL | A5 |
| Ultrasonic Trig | A1 |
| Ultrasonic Echo | A2 |
| Buzzer | D8 |
| LED | D7 |
| Battery Monitor | A0 |

### RC Transmitter:
| Component | Pin |
|-----------|-----|
| nRF24 CE | D9 |
| nRF24 CSN | D10 |
| Throttle (Left Y) | A0 |
| Yaw (Left X) | A1 |
| Pitch (Right Y) | A2 |
| Roll (Right X) | A3 |
| Button 1 | D4 |
| Button 2 | D5 |
| Switch 1 | D3 |
| Switch 2 | D2 |

---

## ⚡ Quick Start

1. **Calibrate ESCs** (one-time)
2. **Power on RC transmitter**
3. **Power on drone** (auto gyro calibration)
4. **Calibrate level** (Button 1, 2s) on flat surface
5. **Arm** (Button 2, 2s)
6. **Throttle up slowly**
7. **Fly!**

**Emergency**: Switch 1 = Instant Stop

---

## 🎯 Pre-Flight Checklist

- [ ] Battery charged
- [ ] Propellers secure
- [ ] RC link >95%
- [ ] Gyro calibrated (auto)
- [ ] Level calibrated (this week)
- [ ] Clear flight area
- [ ] Weather suitable
- [ ] Emergency switch tested

---

## 📈 Key Parameters

| Setting | Value |
|---------|-------|
| Max Angle | 30° (safe) |
| Throttle Range | 1000-2000 µs |
| Loop Rate | 140 Hz |
| Radio Channel | 108 |
| Failsafe Timeout | 3 seconds |
| Ground Warning | <40 cm |

---

## 🛠️ PID Tuning (Advanced)

```cpp
const float kp = 1.8;    // ↑ = More responsive
const float ki = 0.0002; // ↑ = Less drift
const float kd = 0.45;   // ↓ = Less oscillation
const float kpZ = 1.5;   // Yaw response
```

**Tuning Tips**:
- Oscillates → Reduce P or D
- Sluggish → Increase P
- Drifts → Increase I
- Make small changes (±0.2)

---

## ⚠️ Safety Limits

| Feature | Trigger | Action |
|---------|---------|--------|
| **Max Angle** | >30° tilt | Motor stop |
| **Failsafe** | 3s no signal | Motor stop |
| **Ground Warn** | <40cm | LED on |
| **Switch 1** | Manual | Instant disarm |

---

## 🔍 Troubleshooting Fast

| Problem | Quick Fix |
|---------|-----------|
| Won't arm | Check Switch 1, throttle low |
| Drifts | Re-calibrate level (Button 1) |
| Oscillates | Reduce P to 1.5 |
| No RC link | Check power, channel 108 |
| Motors unequal | Recalibrate ESCs |

---

## 📊 Link Quality

| Rate | Status |
|------|--------|
| >98% | ✅ EXCELLENT |
| 90-98% | ✓ GOOD |
| 80-90% | ⚠ WEAK |
| <80% | ❌ POOR - Do not fly |

---

## 🎓 Flight Tips

1. **Start low throttle** (~30%)
2. **Hover first** before moving
3. **Small inputs** = smooth flight
4. **Watch battery** (don't over-discharge)
5. **Practice often** = skill improvement
6. **Land early** if link quality drops

---

## 🆘 Emergency Procedures

### Link Lost:
- Failsafe activates (3s)
- Motors stop automatically
- Drone will drop
- Stay clear!

### Unstable Flight:
1. Reduce throttle slowly
2. Land immediately
3. Disarm (Button 2)
4. Re-calibrate level
5. Check propellers

### Crash:
1. Disarm immediately
2. Disconnect battery
3. Inspect damage
4. Check motor/ESC function
5. Re-calibrate before flying

---

## 📝 Maintenance Schedule

### Every Flight:
- Visual inspection
- Battery check
- Propeller check

### Weekly:
- Level calibration
- Tighten screws
- Clean sensors

### Monthly:
- Deep cleaning
- Connection check
- Performance review

### Yearly:
- ESC recalibration
- Replace worn parts
- Upgrade check

---

## 🎯 Altitude Hold Mode

**Activation**:
1. Enable Switch 2
2. Set throttle 1400-1450
3. Drone holds altitude

**Manual Adjust**:
- Stick >1450: Climb
- Stick <1400: Descend
- 1400-1450: Hold

**Disable**: Switch 2 OFF

---

## 📱 Status Indicators

### LED:
- **Solid ON**: Armed
- **Blinking**: Arming sequence
- **ON while flying**: Ground warning (<40cm)

### Buzzer:
- **1 beep**: Startup
- **2 beeps**: Calibration done
- **Continuous**: Failsafe/Error
- **Short beep**: Arm/Disarm

---

## 🔋 Battery Voltage

| Voltage (3S) | Status | Action |
|--------------|--------|--------|
| >11.7V | ✅ Full | Fly normally |
| 11.1-11.7V | ✓ Good | Monitor |
| 10.5-11.1V | ⚠ Low | Land soon |
| <10.5V | ❌ Critical | Land NOW! |

*Adjust for 4S batteries accordingly*

---

## 🎮 Sensitivity Tuning

In code:
```cpp
float sensiX = -0.3;   // Roll sensitivity
float sensiY = 0.3;    // Pitch sensitivity
float sensiZ = -0.008; // Yaw sensitivity
```

- **Lower value**: Gentler response (beginners)
- **Higher value**: Aggressive response (advanced)
- **Recommended**: Start low, increase slowly

---

## 📊 Calibration Quick Check

**Good Calibration**:
```
Level offsets -> X:0.123 Y:-0.087
(Both within ±2.0°)
```

**Bad Calibration**:
```
Level offsets -> X:5.234 Y:-8.432
(One or both >±5.0°)
→ Recalibrate on flat surface!
```

---

## 🏁 First Flight Protocol

1. **Arm with props OFF**
2. **Test motor directions**
3. **Test tilt response**
4. **Disarm and install props**
5. **Arm again**
6. **Throttle to 30%**
7. **Verify all motors spinning**
8. **Slowly increase to 60%**
9. **Lift off gently**
10. **Hover at 50cm for 30s**
11. **If stable, continue flying**
12. **If unstable, land and recalibrate**

---

## 🎯 This Week's Checklist

- [ ] Monday: Visual inspection
- [ ] Wednesday: Level calibration
- [ ] Friday: Full system test
- [ ] Sunday: Flight practice

---

**Keep this card handy during flights!**

*For detailed info, see README.md*
