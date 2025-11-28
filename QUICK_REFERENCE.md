# 🚁 Drone Quick Reference Card

## ⚡ Quick Start (30 seconds)

```
1. Power ON Remote Controller first
2. Power ON Flight Controller → Listen for connection beep
3. Press Button 1 (hold 2s) → Calibrate
4. Long-press Button 2 (3s) → ARM
5. Slowly raise throttle → FLY!
```

## 🎮 Controls At-A-Glance

```
┌─────────────────────────────────────────────────────────────┐
│                   REMOTE CONTROLLER                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│      LEFT STICK              RIGHT STICK                    │
│         ↑                        ↑                          │
│    Throttle UP              Pitch FWD                       │
│         │                        │                          │
│    ←────┼────→              ←────┼────→                     │
│    Yaw  │  Yaw              Roll │ Roll                     │
│    CCW  │  CW               Left │ Right                    │
│         │                        │                          │
│    Throttle DOWN            Pitch BACK                      │
│         ↓                        ↓                          │
│                                                             │
├─────────────────────────────────────────────────────────────┤
│  Switch 1 (D3): ARM/DISARM     Switch 2 (D2): ALT HOLD     │
│  Button 1 (D4): CALIBRATE      Button 2 (D5): MOTOR START  │
└─────────────────────────────────────────────────────────────┘
```

## 🔴 Emergency Stop

**IMMEDIATE DISARM:** Flip Switch 1 UP

## 📊 LED Status Indicators

| LED State | Meaning |
|-----------|---------|
| **Solid ON** | Disarmed (SAFE) |
| **Blinking** | Armed + Receiving Data |
| **Fast Blink** | Error / Kill Switch |
| **Off** | No power / Dead battery |

## 🔊 Buzzer Sounds

| Sound | Meaning |
|-------|---------|
| 3 rising tones | Startup OK |
| Single 2000 Hz beep | RC Connected |
| Repeating 1000 Hz | KILL SWITCH ACTIVE |
| 3 quick beeps | Calibration Complete |
| 1500 Hz long | Armed/Disarmed |

## 🛠 Pre-Flight Checklist

- [ ] Battery charged (>11.4V)
- [ ] Propellers secure (correct rotation!)
- [ ] Calibration done recently
- [ ] Clear flight area
- [ ] Switch 1 = DISARM
- [ ] Throttle = MIN

## 🎯 Motor Layout

```
        FRONT
    FL ●━━━━━● FR
       │     │
       │  +  │         FL & RR → CCW ⟲
       │     │         FR & RL → CW  ⟳
    RL ●━━━━━● RR
        REAR
```

## ⚙️ Functions

### Button 1 (D4) - Calibration
- **When:** Disarmed only
- **Hold:** 2 seconds
- **Does:** Calibrate MPU6050 + MS5611
- **Keep drone still!**

### Button 2 (D5) - Dual Purpose
- **When Disarmed:** Long-press 3s → ARM
- **When Armed:** Quick press → Motor test

### Switch 1 (D3) - Arm/Disarm
- **UP (1):** DISARMED ← Safe!
- **DOWN (0):** ARMED ← Ready to fly!

### Switch 2 (D2) - Altitude Hold
- **UP (1):** Normal flight
- **DOWN (0):** Hold altitude (throttle 1400-1450)

## 🚨 Kill Switch Triggers

1. **Tilt > 30°** → Emergency stop
2. **No RC signal 3s** → Auto-disarm
3. **Manual** → Switch 1 UP

**Recovery:** Right drone, wait for beeps to stop, fix issue, re-arm

## 🎚 Key Parameters

| Setting | Value |
|---------|-------|
| Loop Rate | 140 Hz |
| Max Tilt | 30° |
| Max Thrust | 1700 µs |
| Armed Idle | 1050 µs |
| Alt Hold Range | 1400-1450 µs |

## 🔧 Quick Troubleshooting

| Problem | Fix |
|---------|-----|
| **No connection** | Check NRF wiring + capacitor |
| **Drift** | Recalibrate on flat surface |
| **Flips on takeoff** | CHECK MOTOR DIRECTIONS! |
| **Won't arm** | Calibrate first, ensure Switch 1 UP |
| **Alt hold drops** | Fly above 2m, throttle mid-range |

## 📍 Pin Connections (Flight Controller)

```
MOTORS:           SENSORS:
D3 → FL           A4 → SDA (MPU + MS5611)
D5 → FR           A5 → SCL (MPU + MS5611)
D6 → RR           
D9 → RL           NRF24L01:
                  D4  → CE
OUTPUTS:          D10 → CSN
D7 → LED          D11 → MOSI
D8 → BUZZER       D12 → MISO
                  D13 → SCK
```

## 📍 Pin Connections (Remote)

```
JOYSTICKS:        CONTROLS:
A0 → Throttle     D2 → Switch 2
A1 → Yaw          D3 → Switch 1
A2 → Pitch        D4 → Button 1
A3 → Roll         D5 → Button 2

NRF24L01:
D9  → CE
D10 → CSN
D11 → MOSI
D12 → MISO
D13 → SCK
```

## 🔋 Battery Safety

- **Minimum:** 10.5V (land immediately!)
- **Storage:** 11.4V (3.8V per cell)
- **Full:** 12.6V (4.2V per cell)
- **Never** over-discharge or charge unattended

## 📏 Flight Recommendations

| Condition | Action |
|-----------|--------|
| **First flight** | Calm day, open area |
| **Min height** | 0.5m (ground effect) |
| **Best alt hold** | >2m altitude |
| **Max wind** | <5 mph for beginners |
| **Distance** | Keep visual contact |

## 🔄 Calibration Frequency

- **Before first flight** → REQUIRED
- **After crash** → Recommended
- **Location change** → If significant altitude diff
- **Drift noticed** → Immediately

## 🆘 Emergency Procedures

### Flyaway
1. Switch 1 → DISARM (immediate)
2. If no response → Cut RC power
3. Drone auto-stops after 3s

### Crash
1. DISARM immediately
2. Disconnect battery
3. Check damage before next flight

### Lost Orientation
1. Release all sticks
2. Let drone stabilize
3. Slowly regain control

## 📞 Debug Info

**Serial Monitor:** 57600 baud

**Enable Debug:**
```cpp
debugging(true);  // In setup()
```

**Key Messages:**
- `*** RC CONNECTED ***`
- `*** ARMED ***`
- `KILL SWITCH: reason`

---

## 📖 Full Documentation

See **SETUP_AND_OPERATION_GUIDE.md** for:
- Complete wiring diagrams
- Detailed procedures
- Advanced tuning
- Full troubleshooting

---

**Stay Safe, Fly Smart! 🚁**

*Keep this card handy during flights*
