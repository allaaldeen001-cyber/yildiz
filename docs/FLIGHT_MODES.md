# ✈️ Flight Modes Guide - Quadcopter Drone

Complete guide to all 5 flight modes and how to use them effectively.

---

## 📋 Table of Contents

- [Mode Overview](#mode-overview)
- [ANGLE Mode](#angle-mode-beginner)
- [ACRO Mode](#acro-mode-advanced)
- [ALTITUDE HOLD Mode](#altitude-hold-mode)
- [TAKEOFF Mode](#takeoff-mode)
- [LANDING Mode](#landing-mode)
- [Mode Switching](#mode-switching)
- [Flight Scenarios](#flight-scenarios)

---

## 🎯 Mode Overview

Your quadcopter has **5 flight modes** designed for different skill levels and use cases:

| Mode | Difficulty | Auto-Level | Auto-Altitude | Best For |
|------|------------|------------|---------------|----------|
| **ANGLE** | 🟢 Easy | ✅ Yes | ❌ No | Learning, photography |
| **ACRO** | 🔴 Expert | ❌ No | ❌ No | Aerobatics, racing |
| **ALT HOLD** | 🔵 Easy | ✅ Yes | ✅ Yes | Easy flight, video |
| **TAKEOFF** | 🟢 Auto | ✅ Yes | ✅ Yes | Automatic takeoff |
| **LANDING** | 🟢 Auto | ✅ Yes | ✅ Yes | Automatic landing |

---

## 🟢 ANGLE Mode (Beginner)

### What is ANGLE Mode?

**Auto-leveling mode** - When you release the sticks, the drone automatically returns to level flight. Perfect for beginners!

### How It Works

Uses **cascaded PID control** (Betaflight algorithm):

```
YOUR STICK INPUT
    ↓
Converts to desired angle (±25°)
    ↓
OUTER LOOP: Angle PID calculates desired rotation rate
    ↓
INNER LOOP: Rate PID adjusts motors to achieve rotation
    ↓
RESULT: Smooth, controlled movement
```

### Activation

**Switch Settings**: SW2 = ON, SW1 = OFF

### Characteristics

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Max Tilt** | ±25° | Safe tilt limit |
| **Auto-Level** | ✅ Yes | Returns to level when sticks centered |
| **Throttle** | Manual | You control altitude |
| **Response** | Medium | Smooth and predictable |

### Stick Behavior

**Right Stick (Roll & Pitch)**:
- Push right → Tilts right 25° → Moves right
- Release → Returns to level → Stops moving
- Push forward → Tilts forward 25° → Moves forward

**Left Stick (Throttle & Yaw)**:
- Up → Climbs
- Down → Descends
- Left/Right → Rotates

### When to Use

✅ **Perfect for:**
- First flights and learning
- Calm, stable flight
- Aerial photography
- Flying near obstacles
- When precision is needed

❌ **Not ideal for:**
- Aerobatics (can't flip)
- Racing (limited tilt angle)
- Advanced maneuvers

### Example Flight Scenario

```
SCENARIO: Taking Photos of a Building
──────────────────────────────────────

1. Enable ANGLE mode (SW2 = ON)
2. Takeoff manually to 3 meters
3. Push right stick forward → Drone moves toward building
4. Release stick → Drone stops and hovers (auto-level)
5. Adjust position with small stick movements
6. Take photo
7. Push stick backward → Return to launch point
```

---

## 🔴 ACRO Mode (Advanced)

### What is ACRO Mode?

**Rate control mode** - Sticks directly control rotation rate. No auto-leveling. For experienced pilots only!

### How It Works

Uses **rate PID only** (no angle stabilization):

```
YOUR STICK INPUT
    ↓
Converts to desired rotation rate (±300°/s)
    ↓
RATE PID: Adjusts motors to achieve rotation rate
    ↓
RESULT: Direct, agile control (can flip!)
```

### Activation

**Switch Settings**: SW2 = OFF, SW1 = OFF

### Characteristics

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Max Rate** | ±300°/s | Can flip in 0.6 seconds |
| **Auto-Level** | ❌ No | You must manually level |
| **Throttle** | Manual | You control altitude |
| **Response** | Very Fast | Instant, direct control |

### Stick Behavior

**Right Stick (Roll & Pitch)**:
- Push right → Rotates right at 300°/s
- Hold → Keeps rotating (can flip!)
- Release → Stops rotation BUT stays tilted
- You must manually return to level

**Left Stick (Throttle & Yaw)**:
- Same as ANGLE mode

### When to Use

✅ **Perfect for:**
- Aerobatics (flips, rolls, loops)
- Racing (maximum agility)
- Advanced maneuvers
- Freestyle flying

❌ **Not ideal for:**
- Beginners (very difficult!)
- Photography (unstable)
- Flying near obstacles

⚠️ **WARNING**: ACRO mode requires significant practice. Start in simulator or with experienced supervision!

### Example Flight Scenario

```
SCENARIO: Performing a Flip
───────────────────────────────

1. Enable ACRO mode (SW2 = OFF)
2. Climb to 10 meters (safety altitude)
3. Full throttle + full forward stick
4. Drone flips forward
5. Reduce throttle during flip
6. Manually level after flip
7. Return to normal flight
```

---

## 🔵 ALTITUDE HOLD Mode

### What is ALTITUDE HOLD Mode?

**Auto-leveling + auto-altitude** - Drone maintains height automatically. Easiest mode to fly!

### How It Works

Combines **angle stabilization** with **altitude PID**:

```
BAROMETER (MS5611)
    ↓
Measures current altitude
    ↓
ALTITUDE PID: Compares to target altitude
    ↓
Adjusts throttle automatically
    ↓
ANGLE PID: Keeps drone level
    ↓
RESULT: Hands-free hovering!
```

### Activation

**Switch Settings**: SW1 = ON (overrides SW2)

### Characteristics

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Max Tilt** | ±25° | Same as ANGLE |
| **Auto-Level** | ✅ Yes | Returns to level |
| **Throttle** | Auto | Maintains altitude |
| **Altitude Adjust** | ±10cm/s | Throttle stick adjusts target |

### Stick Behavior

**Right Stick (Roll & Pitch)**:
- Same as ANGLE mode
- Push right → Moves right
- Release → Stops and hovers

**Left Stick (Throttle & Yaw)**:
- **Throttle up** → Target altitude increases slowly (10cm/s)
- **Throttle center** → Holds current altitude perfectly
- **Throttle down** → Target altitude decreases slowly
- **Yaw** → Rotates (same as other modes)

### When to Use

✅ **Perfect for:**
- Easy flying (least workload)
- Aerial video (stable footage)
- Learning advanced maneuvers
- Windy conditions (auto-compensates)
- Long flights (less fatigue)

❌ **Not ideal for:**
- Indoor flight (barometer affected by HVAC)
- Near ground (<50cm, ground effect)
- Rapid altitude changes

### Example Flight Scenario

```
SCENARIO: Recording Smooth Video
────────────────────────────────────

1. Enable ALT HOLD (SW1 = ON)
2. Takeoff to 2 meters
3. Drone locks altitude at 2m automatically
4. Wind gust pushes down → Drone compensates instantly
5. Focus on flying smoothly (no throttle management!)
6. Throttle up slightly → Climbs to 3m
7. Center throttle → Locks at 3m
8. Fly around without touching throttle
9. Video is perfectly smooth!
```

### Technical Details

**Altitude Locking**:
- When you enable SW1, current altitude becomes target
- Barometer reads altitude 50 times/second
- PID correction applied to throttle automatically

**Throttle Stick Behavior**:
```
Stick Position → Target Altitude Change
────────────────────────────────────────
Full Up (1000)   → +10 cm/s (climbing)
Center (500)     → 0 cm/s (locked)
Full Down (0)    → -10 cm/s (descending)
```

**Example**:
```
Current altitude: 150cm
Throttle stick: Full up for 5 seconds
Target altitude: 150 + (10 cm/s × 5s) = 200cm
Drone climbs smoothly to 200cm
Center stick → Locks at 200cm
```

---

## 🚀 TAKEOFF Mode

### What is TAKEOFF Mode?

**Fully automatic takeoff** - Press one button, drone arms and rises to 1.5 meters, then hovers hands-free!

### How It Works

```
BUTTON 4 PRESSED
    ↓
1. Auto-ARM motors
2. Spin up smoothly
3. Rise at 80 cm/s
4. Reach 150cm (1.5m)
5. Switch to ALTITUDE HOLD
6. Hover hands-free
```

### Activation

**Press Button 4** (D7 on remote controller)

### Sequence Timeline

| Time | Action | Altitude |
|------|--------|----------|
| 0.0s | Button pressed, motors ARM | 0 cm |
| 0.5s | Motors spin up, lifting off | 40 cm |
| 1.0s | Rising at 80 cm/s | 80 cm |
| 1.5s | Approaching target | 120 cm |
| 2.0s | Reached target altitude | 150 cm |
| 2.0s+ | Automatic ALTITUDE HOLD | 150 cm |

### Safety Features

✅ **Automatic checks**:
- Only works when disarmed
- Throttle must be below 100
- Radio must be connected
- Smooth motor spinup (no sudden jumps)

### When to Use

✅ **Perfect for:**
- Easy takeoffs (no skill needed)
- Consistent takeoff height
- Beginners learning to fly
- Quick deployment

❌ **Don't use when:**
- In tight spaces (needs 2m clearance)
- Uneven ground (may drift)
- Indoor (altitude hold less reliable)

### Example Flight Scenario

```
SCENARIO: Quick Photo Mission
──────────────────────────────────

1. Place drone on flat ground
2. Center all sticks
3. Press Button 4
4. Watch drone rise automatically
5. At 1.5m, it stops and hovers
6. You take over with sticks
7. Fly mission
8. Return to start point
9. Press Button 3 for auto-landing
```

---

## 🛬 LANDING Mode

### What is LANDING Mode?

**Fully automatic landing** - Press one button, drone descends gently and disarms safely.

### How It Works

```
BUTTON 3 PRESSED
    ↓
1. Start descent at 50 cm/s
2. Reduce altitude target smoothly
3. Reach 10cm above ground
4. Auto-DISARM motors
5. Beep 3 times (safe landing)
```

### Activation

**Press Button 3** (D6 on remote controller)

### Sequence Timeline

| Time | Action | Altitude |
|------|--------|----------|
| 0.0s | Button pressed, descent starts | 150 cm |
| 1.0s | Descending at 50 cm/s | 100 cm |
| 2.0s | Continuing descent | 50 cm |
| 3.0s | Approaching ground | 10 cm |
| 3.0s | Auto-DISARM | 0 cm |
| 3.1s | 3 beeps (safe landing) | 0 cm |

### Safety Features

✅ **Automatic checks**:
- Only works when armed
- Gentle descent speed (50 cm/s)
- Auto-disarm at 10cm (prevents ground contact with spinning motors)
- Audio confirmation (3 beeps)

### When to Use

✅ **Perfect for:**
- Safe landings every time
- Consistent landing quality
- Low battery situations
- Beginners

❌ **Don't use when:**
- Landing on uneven ground (may tip over)
- Strong winds near ground
- Obstacles below drone

### Example Flight Scenario

```
SCENARIO: Safe Return After Long Flight
────────────────────────────────────────

1. Flying in ALTITUDE HOLD mode
2. Battery getting low
3. Return to launch point
4. Hover at 1.5m above landing spot
5. Press Button 3 (LANDING)
6. Drone descends smoothly at 50cm/s
7. At 10cm, motors disarm automatically
8. 3 beeps confirm safe landing
9. Done! No manual landing needed
```

---

## 🔄 Mode Switching

### Switch Configuration

| SW1 (D2) | SW2 (D3) | Mode |
|----------|----------|------|
| OFF | ON | **ANGLE** |
| OFF | OFF | **ACRO** |
| ON | (any) | **ALT HOLD** |

**Note**: Button 3 and Button 4 override switches temporarily for TAKEOFF/LANDING.

### Switching Rules

✅ **Safe to switch**:
- ANGLE ↔ ACRO (instant switch)
- ANGLE → ALT HOLD (locks current altitude)
- ACRO → ALT HOLD (locks current altitude)

⚠️ **Be careful**:
- ALT HOLD → ANGLE/ACRO (throttle control returns to manual)
- ANGLE → ACRO (auto-leveling stops, drone may tilt suddenly)

### Recommended Switching Procedure

**ANGLE → ACRO**:
1. Level the drone in ANGLE mode
2. Ensure stable hover
3. Flip SW2 to OFF
4. Be ready to correct tilt manually

**ACRO → ANGLE**:
1. Manually level the drone first
2. Flip SW2 to ON
3. Angle PID takes over smoothly

**ANY → ALT HOLD**:
1. Flip SW1 to ON
2. Drone locks current altitude immediately
3. Throttle stick now adjusts target (not direct thrust)

**ALT HOLD → ANY**:
1. Note current throttle position needed to hover
2. Set throttle stick to hover position (~50%)
3. Flip SW1 to OFF
4. You now have manual throttle control

---

## 🎬 Flight Scenarios

### Scenario 1: Beginner's First Flight

**Goal**: Learn to fly safely

**Mode progression**:
1. Start with **ANGLE mode** (SW2 = ON, SW1 = OFF)
2. Practice basic movements:
   - Right stick left/right (roll)
   - Right stick forward/back (pitch)
   - Left stick up/down (throttle)
   - Left stick left/right (yaw)
3. After 5 flights, try **ALT HOLD** (SW1 = ON)
4. Focus on horizontal movement only
5. Master hovering in place

**Timeline**: 1-2 hours of practice

---

### Scenario 2: Aerial Photography

**Goal**: Get smooth, stable footage

**Recommended mode**: **ALTITUDE HOLD**

**Procedure**:
1. Enable ALT HOLD (SW1 = ON)
2. Use **Button 4** for automatic takeoff
3. Rise to desired altitude (add throttle to climb)
4. Center throttle → Altitude locks
5. Fly smoothly with right stick only
6. No need to touch throttle!
7. Use **Button 3** for automatic landing

**Tips**:
- Fly slowly (gentle stick movements)
- Let altitude hold compensate for wind
- Focus on camera framing, not flying

---

### Scenario 3: FPV Racing

**Goal**: Maximum speed and agility

**Recommended mode**: **ACRO**

**Procedure**:
1. Switch to ACRO (SW2 = OFF, SW1 = OFF)
2. Manual takeoff (full throttle + slight forward)
3. Fly through course using rate control
4. Full stick throws for fast turns
5. Manual landing (reduce throttle gradually)

**Tips**:
- Practice in simulator first!
- Keep throttle high during flips
- Always know drone orientation

---

### Scenario 4: Windy Day Flight

**Goal**: Stable flight in wind

**Recommended mode**: **ALTITUDE HOLD**

**Why**:
- Altitude PID compensates for updrafts/downdrafts
- Angle PID with I-term corrects for steady wind drift
- Less pilot workload

**Procedure**:
1. Enable ALT HOLD (SW1 = ON)
2. Increase altitude PID Ki if needed (more drift correction)
3. Fly normally, let drone compensate automatically

---

### Scenario 5: Emergency Situations

**Goal**: Safely recover from problems

**Scenarios**:

**Low Battery**:
1. Press **Button 3** immediately (auto-landing)
2. Land wherever you are
3. Don't risk flying home

**Lost Orientation**:
1. Switch to **ANGLE mode** (auto-levels)
2. Enable **ALT HOLD** (locks altitude)
3. Yaw slowly until you see which way is forward
4. Fly back to launch point

**Signal Loss** (Failsafe):
1. Drone auto-disarms after 1 second
2. Falls safely (better than uncontrolled flight)
3. Retrieve and check radio

**Strong Wind**:
1. Enable **ALT HOLD** if not already
2. Fly into wind (faster return)
3. Land as soon as possible

---

## 📊 Mode Comparison Table

| Feature | ANGLE | ACRO | ALT HOLD | TAKEOFF | LANDING |
|---------|-------|------|----------|---------|---------|
| **Auto-Level** | ✅ | ❌ | ✅ | ✅ | ✅ |
| **Auto-Altitude** | ❌ | ❌ | ✅ | ✅ | ✅ |
| **Max Tilt** | 25° | 360° | 25° | 25° | 25° |
| **Difficulty** | Easy | Hard | Easy | Auto | Auto |
| **Can Flip** | ❌ | ✅ | ❌ | ❌ | ❌ |
| **Hands-Free Hover** | ❌ | ❌ | ✅ | ✅ | N/A |
| **Best For** | Learning | Racing | Video | Takeoff | Landing |

---

## ✅ Flight Mode Checklist

### Before Each Flight

- [ ] Select appropriate mode for task
- [ ] Verify switch positions
- [ ] Test mode switching on ground (motors off)
- [ ] Know how to switch to ANGLE mode quickly (emergency)
- [ ] Understand failsafe behavior

### During Flight

- [ ] Monitor battery voltage
- [ ] Be aware of current mode (check Serial Monitor)
- [ ] Switch modes smoothly (level first)
- [ ] Keep throttle centered when enabling ALT HOLD

### After Flight

- [ ] Review flight performance
- [ ] Adjust PID values if needed
- [ ] Log any issues or improvements

---

**Happy Flying! ✈️**

*Start with ANGLE mode, progress to ALTITUDE HOLD, master basics before trying ACRO!*
