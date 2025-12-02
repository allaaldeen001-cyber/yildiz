# Quick Start Guide

Get your Yildiz drone flying in under 30 minutes! (after assembly)

## What You Need

### Hardware
- ✅ 2x Arduino Nano/Uno (one for TX, one for RX)
- ✅ 2x NRF24L01+ modules
- ✅ 1x MPU6050 IMU
- ✅ 2x Analog joysticks
- ✅ 4x ESCs (20A+)
- ✅ 4x Brushless motors
- ✅ 4x Props
- ✅ LiPo battery (3S 11.1V)
- ✅ 2x Switches
- ✅ Drone frame

### Software
- ✅ Arduino IDE
- ✅ RF24 library (TMRh20)
- ✅ This code repository

---

## Step 1: Install Arduino IDE & Libraries (5 minutes)

### A. Download Arduino IDE
1. Go to: https://www.arduino.cc/en/software
2. Download for your OS
3. Install

### B. Install RF24 Library
1. Open Arduino IDE
2. Go to: **Sketch → Include Library → Manage Libraries**
3. Search: **"RF24"**
4. Install: **RF24 by TMRh20** (version 1.4.8+)
5. Click "Install All" if prompted for dependencies

### C. Test Installation
```cpp
// Open: File → Examples → RF24 → GettingStarted
// If you see the example, library is installed correctly!
```

---

## Step 2: Wire Transmitter (10 minutes)

### Minimal Transmitter Wiring

```
Joystick 1 (Left):
  VRx → A1 (Yaw)
  VRy → A0 (Throttle)
  +5V → 5V
  GND → GND

Joystick 2 (Right):
  VRx → A3 (Roll)
  VRy → A2 (Pitch)
  +5V → 5V
  GND → GND

NRF24L01:
  CE  → D9
  CSN → D10
  MOSI → D11
  MISO → D12
  SCK  → D13
  VCC  → 3.3V (via external regulator!)
  GND  → GND

Switches:
  Arm → D7 (other pin to GND)
  Mode → D8 (other pin to GND)

Power:
  Battery + → Arduino VIN (7-12V)
  Battery - → Arduino GND
```

⚠️ **CRITICAL: NRF24L01 NEEDS EXTERNAL 3.3V REGULATOR!**
- Add 100uF + 0.1uF capacitors
- Arduino's 3.3V pin is NOT enough

---

## Step 3: Upload Transmitter Code (2 minutes)

1. Open **transmitter/transmitter.ino**
2. Select Board: **Tools → Board → Arduino Nano/Uno**
3. Select Port: **Tools → Port → (your Arduino)**
4. Click **Upload** (→ button)
5. Wait for "Done uploading"
6. Open **Serial Monitor** (Ctrl+Shift+M)
7. Set baud rate to **115200**
8. Should see: "=== DRONE TRANSMITTER ==="

✅ Success: You see initialization messages
❌ Error: Check wiring, especially NRF24

---

## Step 4: Wire Receiver (10 minutes)

### Minimal Receiver Wiring

```
NRF24L01:
  CE  → D9
  CSN → D10
  MOSI → D11
  MISO → D12
  SCK  → D13
  VCC  → 3.3V (via external regulator!)
  GND  → GND

MPU6050:
  VCC → 5V
  GND → GND
  SCL → A5
  SDA → A4

Motors (via ESCs):
  ESC1 Signal → D3  (Front-Left)
  ESC2 Signal → D5  (Front-Right)
  ESC3 Signal → D6  (Back-Right)
  ESC4 Signal → D11 (Back-Left)
  All ESC GND → Arduino GND
  One ESC BEC → Arduino VIN (5V)

Battery:
  Battery + → All ESCs
  Battery - → All ESCs + Arduino GND

IMPORTANT: Remove props for testing!
```

---

## Step 5: Upload Receiver Code (2 minutes)

1. Open **receiver/receiver.ino**
2. Select Board: **Tools → Board → Arduino Nano/Uno**
3. Select Port: **Tools → Port → (your Arduino)**
4. Click **Upload**
5. Wait for "Done uploading"
6. Open **Serial Monitor** (115200 baud)
7. Keep drone STILL on level surface
8. Wait for "Gyro calibrated"

✅ Success: "READY FOR FLIGHT!" message
❌ Error: Check wiring

---

## Step 6: Test Connection (2 minutes)

### A. Check Radio Link
1. Power ON transmitter
2. Power ON receiver
3. Watch transmitter serial monitor
4. Should see: Battery voltage, telemetry

✅ **Working:** Telemetry data updating
❌ **Not working:** "NO CONNECTION TO DRONE"

### B. Fix Connection Issues
If no connection:
1. Check NRF24 power (measure 3.2-3.4V)
2. Verify both use same channel (108)
3. Check address matches: "DRON1"
4. Add capacitors to NRF24
5. Try different NRF24 modules

---

## Step 7: Test Motors (5 minutes)

### ⚠️ **PROPS OFF! PROPS OFF! PROPS OFF!** ⚠️

### A. Check Motor Response
1. **Arm switch OFF**
2. **Throttle stick LOW**
3. Turn **Arm switch ON**
4. **Slowly** increase throttle
5. All 4 motors should spin

✅ **Working:** All motors spin smoothly
❌ **Problem:** See troubleshooting below

### B. Verify Motor Directions (IMPORTANT!)

**With hand near motor (careful!):**

| Motor | Location | Should Spin | Test Method |
|-------|----------|-------------|-------------|
| M1 | Front-Left | ⟳ Clockwise | Feel with finger |
| M2 | Front-Right | ⟲ CCW | Feel with finger |
| M3 | Back-Right | ⟳ Clockwise | Feel with finger |
| M4 | Back-Left | ⟲ CCW | Feel with finger |

**Wrong direction?** Swap any 2 motor wires on ESC

### C. Test Flight Controls (Props Still Off!)

Move sticks and watch motors:
- **Throttle up:** All motors speed up equally
- **Roll right:** M1,M4 speed up, M2,M3 slow down
- **Pitch forward:** M1,M2 speed up, M3,M4 slow down
- **Yaw right:** M1,M3 speed up, M2,M4 slow down

---

## Step 8: First Hover Test

### Safety First!
- ✅ Open area (grass field recommended)
- ✅ No people or animals nearby
- ✅ Props installed correctly
- ✅ Battery charged
- ✅ Weather calm (no wind for first test)
- ✅ Failsafe tested (turn off TX → motors stop)

### Install Propellers

```
Front of Drone (marked with colored arm)
  
  M1 (CW)         M2 (CCW)
    ⟳               ⟲
  Normal          CCW Prop
   Prop          (marked R)
   
   
  M4 (CCW)        M3 (CW)
    ⟲               ⟳
  CCW Prop        Normal
  (marked R)       Prop
  
Back of Drone
```

**Check prop installation:**
- Prop tight on motor shaft
- Correct rotation direction
- No damage (cracks, chips)

### The Hover

1. **Place drone on flat ground**
2. **Stand 3-5 meters away**
3. **Arm switch OFF**
4. **Power on transmitter**
5. **Power on drone**
6. **Wait for connection** (LED solid)
7. **Throttle to minimum**
8. **Arm switch ON** (should hear motors spin up slightly)
9. **SLOWLY increase throttle**
10. **Around 1550-1600, it should lift off**
11. **Hover at 1-2 feet for 10 seconds**
12. **Slowly reduce throttle to land**
13. **Arm switch OFF**

### What to Expect

✅ **Good Flight:**
- Lifts off smoothly
- Hovers stable
- Small corrections needed
- Responds to stick inputs
- Lands gently

⚠️ **Needs Tuning:**
- Oscillates (bounces)
- Drifts in one direction
- Slow to respond
- → See TUNING_GUIDE.md

❌ **Crashes Immediately:**
- Check motor directions!
- Check prop directions!
- Reduce PID gains (start with P=0.5, I=0, D=0)
- Verify gyro calibrated

---

## Quick Troubleshooting

### No Connection
→ Check NRF24 power (3.3V with caps)
→ Verify address: "DRON1"
→ Try different channel

### Motors Don't Spin
→ Throttle must be LOW to arm
→ Check failsafe not active
→ Verify ESC connections

### Drone Flips on Takeoff
→ **MOTOR DIRECTION!** Check all 4
→ Check prop directions
→ Verify motor order (M1=D3, M2=D5, M3=D6, M4=D11)

### Oscillations
→ Reduce P gain by 30%
→ Increase D gain by 20%
→ See TUNING_GUIDE.md

### Drifts
→ Recalibrate gyro (on level surface)
→ Check motor balance
→ Add small I gain (0.02)

---

## Next Steps

After successful first hover:

1. **Tune PIDs** (TUNING_GUIDE.md)
   - Reduce oscillations
   - Improve responsiveness
   - Eliminate drift

2. **Practice Flying**
   - Hover in place (hardest!)
   - Small movements
   - Figure-8 patterns
   - Landing practice

3. **Add Features**
   - Battery monitoring
   - Telemetry display
   - Additional flight modes
   - LED indicators

4. **Safety Practice**
   - Emergency landing
   - Failsafe testing
   - Low battery landing
   - Lost orientation recovery

---

## Flight Modes

### Stabilize Mode (Mode Switch OFF)
- **Best for beginners**
- Self-leveling
- Returns to horizontal when sticks centered
- Easier to fly

### Acro Mode (Mode Switch ON)
- **For experienced pilots**
- No self-leveling
- Manual control
- Allows flips and tricks

**Start with Stabilize mode!**

---

## Important Safety Reminders

1. **Never** fly near people or property
2. **Always** test failsafe before flight
3. **Never** catch a falling drone
4. **Always** have spare props
5. **Never** fly with low battery
6. **Always** follow local laws
7. **Never** fly beyond line of sight
8. **Always** have fire extinguisher (LiPo safety)

---

## Pre-Flight Checklist

Print this and check before EVERY flight:

- [ ] Battery fully charged (>11.1V for 3S)
- [ ] Props tight and undamaged
- [ ] All screws tight
- [ ] No loose wires
- [ ] Transmitter powered on
- [ ] Good connection (LED solid)
- [ ] Gyro calibrated
- [ ] Failsafe tested (TX off → motors stop)
- [ ] Clear flight area
- [ ] Weather acceptable (wind <10mph)
- [ ] Spare battery ready
- [ ] First aid kit available

---

## Common First-Flight Issues

### "It wants to flip!"
→ Motor direction wrong - check all 4

### "It spins in circles!"
→ One motor not working or prop missing

### "It shakes like crazy!"
→ PID too high - reduce to P=0.5, I=0, D=0

### "It just sits there!"
→ Not armed, or throttle too low

### "It flew away!"
→ Wind too strong, or gyro not calibrated

---

## Getting Help

If stuck, provide:
1. **Exact problem** (video helps!)
2. **Serial monitor output**
3. **Wiring photos**
4. **PID values**
5. **What you tried**

Check:
- TROUBLESHOOTING.md
- TUNING_GUIDE.md
- README.md

---

## Success Criteria

You're ready for advanced flying when:
- ✅ Can hover steady for 30+ seconds
- ✅ Can land within 1 meter of takeoff
- ✅ Can handle mild wind
- ✅ No unexpected oscillations
- ✅ Battery lasts 5+ minutes
- ✅ Confident with controls

---

**Congratulations on your first flight! 🚁**

**Remember: Safety first, practice often, have fun!**

---

## Time Breakdown

| Step | Time | Total |
|------|------|-------|
| Install software | 5 min | 5 min |
| Wire transmitter | 10 min | 15 min |
| Upload TX code | 2 min | 17 min |
| Wire receiver | 10 min | 27 min |
| Upload RX code | 2 min | 29 min |
| Test connection | 2 min | 31 min |
| Test motors | 5 min | 36 min |
| First hover | 5 min | 41 min |

**Total: ~40 minutes** (assuming assembly complete)

Good luck! 🎉
