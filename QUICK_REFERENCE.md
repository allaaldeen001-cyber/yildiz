# 🚁 QUICK REFERENCE CARD

## 📌 PIN MAPPINGS

### Flight Controller
```
MOTORS:        D3=FL | D5=FR | D6=RR | D9=RL
NRF24:         D2=CE | D10=CSN | D11-13=SPI
SENSORS:       A4=SDA | A5=SCL (MPU6050 + MS5611)
BUTTONS:       A6=Calibrate | A7=Motor Start
SWITCHES:      D4=Arm | D7=AltHold
OUTPUTS:       D8=Buzzer | A3=LED
BATTERY:       A0=VoltDivider
```

### Remote Controller
```
NRF24:         D9=CE | D10=CSN | D11-13=SPI
JOYSTICKS:     A0=Throttle | A1=Yaw | A2=Pitch | A3=Roll
LED:           D8=Status
```

---

## 🎮 CONTROLS

```
LEFT STICK:           RIGHT STICK:
  Throttle               Pitch
     ↑                     ↑
Yaw ← ● → Yaw    Roll ←    ● → Roll
     ↓                     ↓
  Throttle               Pitch
```

**Switches:**
- **D4**: LOW=Armed | HIGH=Disarmed
- **D7**: LOW=AltHold | HIGH=Manual

**Buttons:**
- **A6**: Calibrate (hold 2s, disarmed only)
- **A7**: Smooth motor start (armed only)

---

## 🔔 STATUS CODES

### LED
- **Always ON** → Disarmed (safe)
- **Blinking** → Armed & linked
- **OFF** → No signal

### Buzzer
- **Beep-Beep-Beeeep** → Startup
- **Beep-Beep** → Link OK
- **Repeating beep** → Kill switch

---

## ⚡ QUICK START

1. **Power**: RC first → FC second
2. **Link**: Wait for beep-beep
3. **Calibrate**: Hold A6 button 2s (disarmed)
4. **Test**: Arm → Press A7 → Check motors (no props!)
5. **Fly**: Disarm → Install props → Arm → Fly

---

## 🔧 DEFAULT VALUES

```cpp
PID:      kp=2 | ki=0.0001 | kd=0.5
Limits:   maxAngle=30° | maxThrust=1700
Freq:     140 Hz
Motors:   1000-2000 µs | Armed=1050 µs
```

---

## ⚠️ SAFETY

✅ **Always**: Calibrate | Test without props | Open area | Monitor battery
❌ **Never**: Skip calibration | Fly near people | Touch spinning props

---

## 🔋 BATTERY (3S LiPo)

- Full: 12.6V
- Normal: 11.1V
- Low: 10.5V ⚠️
- Critical: 9.9V 🚨 LAND NOW!

---

## 🐛 QUICK FIXES

| Problem | Solution |
|---------|----------|
| No link | Check NRF wiring, add 10µF cap |
| Drift | Recalibrate on level surface |
| No motors | Check arm switch, ESC wiring |
| One motor dead | Check signal wire to D3/5/6/9 |

---

## 🛠 MOTOR LAYOUT

```
    FL(D3)  FR(D5)
  CCW ↺      ↻ CW
      \  X  /
      /  X  \
  CW ↻      ↺ CCW
    RL(D9)  RR(D6)
```

---

## 📡 NRF24L01 WIRING

```
⚠️ MUST USE 3.3V (NOT 5V!)
⚠️ ADD 10µF CAPACITOR!

VCC → 3.3V
GND → GND
CE  → D2 (FC) or D9 (RC)
CSN → D10
MOSI→ D11
MISO→ D12
SCK → D13
```

---

## 🔄 FLIGHT SEQUENCE

```
1. Power RC
2. Power FC
3. Wait link ✅
4. Disarmed (D4=HIGH)
5. Calibrate if needed
6. Remove props
7. Arm (D4=LOW)
8. Motor start (A7)
9. Verify motors ✅
10. Disarm
11. Install props
12. Arm
13. Takeoff 🚁
14. Land
15. Disarm
16. Power off
```

---

## 📊 SERIAL DEBUG

**Baud**: 57600

**Send 'd'** for detailed output:
- Armed status
- Thrust value
- Altitude hold status
- Pressure reading
- Gyro angles

---

## 🎯 CALIBRATION

1. Disarm (D4=HIGH)
2. Level surface
3. Hold A6 button 2 seconds
4. Wait 10-15 seconds
5. Hear beep-beep ✅

**Calibrates:**
- MPU6050 X/Y offsets
- MS5611 ground pressure

---

## ⚙️ TUNING TIPS

**Oscillates** → Decrease kp, kd
**Slow response** → Increase kp, kd
**Drifts** → Recalibrate first, then increase ki

---

## 📚 FULL DOCS

- **USER_MANUAL.md** → Complete guide
- **PIN_MAPPING.md** → Wiring details
- **README.md** → Project overview

---

**Print this card and keep it handy! ✈️**
