# Quick Start Guide

## 5-Minute Setup

### 1. Install Libraries (2 minutes)
Open Arduino IDE → Sketch → Include Library → Manage Libraries → Install:
- RF24 by TMRh20
- Adafruit MPU6050
- Adafruit Sensor (dependency)
- Adafruit MS5611

### 2. Upload Code (2 minutes)
1. Connect Flight Controller Arduino → Upload `FlightController.ino`
2. Connect RC Controller Arduino → Upload `RCController.ino`

### 3. First Flight (1 minute)
1. Power on both systems
2. Open Serial Monitor (115200 baud) for Flight Controller
3. Wait for "RC CONNECTED" message
4. Set toggle to KILL
5. Press Button 1 (calibration)
6. Set toggle to ARM
7. Press Button 2 (motor start)
8. Increase throttle slowly to fly!

## Control Reference

```
LEFT STICK:
  ↑ = Throttle Up
  ↓ = Throttle Down (center = safe)
  ← = Yaw Left
  → = Yaw Right

RIGHT STICK:
  ↑ = Pitch Forward
  ↓ = Pitch Backward
  ← = Roll Left
  → = Roll Right

BUTTONS:
  Button 1 = Calibrate
  Button 2 = Start Motors

TOGGLE:
  OFF = Kill (safe)
  ON = Arm (ready)
```

## Safety Checklist

- [ ] Open area, away from people
- [ ] Kill switch accessible
- [ ] Propellers secure
- [ ] Battery properly connected
- [ ] Radio connection confirmed
- [ ] Calibration completed
- [ ] Low throttle for first test

## Troubleshooting Quick Fixes

**No Radio Connection:**
- Check NRF24L01 power (3.3V)
- Verify antennas connected
- Check distance (< 100m)

**Motors Don't Spin:**
- Verify ESC calibration
- Check toggle switch (must be ARM)
- Press Button 2 after arming

**Drone Unstable:**
- Recalibrate IMU (Button 1)
- Check motor wiring
- Verify propellers correct

## Serial Monitor States

```
STATE_INIT → Initializing
STATE_WAITING_RADIO → Waiting for RC
STATE_WAITING_KILL_SWITCH → Set toggle OFF
STATE_WAITING_CALIBRATION → Press Button 1
STATE_WAITING_ARM → Set toggle ON
STATE_WAITING_MOTOR_START → Press Button 2
STATE_READY_TO_FLY → Ready!
STATE_FLYING → In flight
STATE_KILLED → Emergency stop
```

## Emergency Procedures

**Immediate Stop:**
- Toggle switch to KILL position
- Motors stop instantly

**Radio Lost:**
- Motors auto-stop after 500ms
- Land immediately if in flight

**Unstable Flight:**
- Reduce throttle immediately
- Toggle to KILL
- Check calibration

---

**Remember:** Center joystick = Safe (1000 throttle, no motors)
