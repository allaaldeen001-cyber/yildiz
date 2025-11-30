# Quadcopter Operation Guide

## Quick Start Checklist

### Pre-Flight Checklist
- [ ] Battery fully charged
- [ ] All connections secure
- [ ] Propellers installed correctly (direction!)
- [ ] Flight area clear
- [ ] Weather conditions suitable
- [ ] Failsafe tested
- [ ] Range test completed

## Detailed Operating Procedures

### 1. System Initialization

#### A. Remote Controller Startup
1. Connect Arduino Nano to computer or battery
2. Open Serial Monitor (115200 baud)
3. Verify output shows:
   ```
   [OK] Radio initialized
   [OK] Calibration complete!
   System Ready!
   ```
4. Check that joystick values respond when moved
5. Center both joysticks and verify readings near 0

#### B. Flight Controller Startup
1. Place drone on **perfectly level surface**
2. Connect battery to flight controller
3. **DO NOT MOVE** drone during calibration
4. Wait for beep sequence:
   - Single long beep: Starting calibration
   - Wait 6 seconds (LED blinking)
   - Two short beeps: Calibration complete
5. LED should be blinking (disarmed state)
6. Check Serial Monitor for calibration values

### 2. Connection Verification

1. Both devices powered on
2. Remote Serial Monitor should show:
   ```
   T:1000 R:0 P:0 Y:0 | SW:0 BTN:0 | CONN [SAFE]
   ```
3. If shows "NO SIGNAL":
   - Power cycle both devices
   - Check nRF24L01+ connections
   - Ensure both on same channel (108)
4. Move joysticks and verify values change
5. Toggle switches and verify SW bits change

### 3. First Flight (With Propellers Removed!)

#### Initial Motor Test

**⚠️ WARNING: PROPELLERS MUST BE REMOVED ⚠️**

1. **Verify throttle at minimum** (stick down)
2. Flip SW1 to ARM position
3. Controller emits single beep
4. LED stays solid on
5. Remote shows "[ARMED]"
6. **Slowly** increase throttle
7. All motors should spin up together
8. Verify motor directions:
   ```
        FRONT
      FL     FR
       \ CCW CW /
         \ / \  /
          X   X
         / \ / \
       / CW CCW \
      RL     RR
        REAR
   ```
9. If any motor wrong direction: swap two ESC wires
10. Lower throttle to minimum
11. Flip SW1 to DISARM
12. Two beeps confirm disarm

### 4. Range Test (Before Flight)

1. Arm drone (propellers removed)
2. Walk away from drone with remote
3. Watch for "NO SIGNAL" on remote
4. Should maintain connection to at least 30 meters
5. If signal lost early:
   - Check antenna orientation
   - Verify nRF24L01+ powered properly
   - Add capacitor to radio module

### 5. First Hover (With Propellers!)

**⚠️ PROPELLERS NOW INSTALLED - EXTREME CAUTION ⚠️**

#### Propeller Installation
```
     FRONT
   FL     FR
  CCW     CW
   \       /
    \     /
     \   /
      \ /
      / \
     /   \
    /     \
   /       \
  CW      CCW
  RL      RR
    REAR
```

1. Install propellers in correct direction
2. Tighten securely (use thread lock)
3. Verify no loose wires near propellers
4. Place drone in open area (5m clearance)

#### Hover Procedure
1. Stand 3-4 meters behind drone
2. Ensure SW2 in STABILIZE mode
3. Throttle to minimum
4. Arm with SW1
5. **Slowly** increase throttle to ~50%
6. Drone should lift off ground
7. Use right stick for small corrections
8. Practice hovering at 1-2 meters height
9. If unstable, immediately disarm
10. Land gently and disarm

### 6. Flight Maneuvers

#### Basic Flight
- **Throttle** (Left Stick Up/Down): Altitude
- **Yaw** (Left Stick Left/Right): Rotation
- **Pitch** (Right Stick Up/Down): Forward/Backward
- **Roll** (Right Stick Left/Right): Left/Right

#### Stabilize Mode (Beginner)
- Self-levels when sticks centered
- Easier to control
- Recommended for learning

#### Acro Mode (Advanced)
- No self-leveling
- Full manual control
- Toggle with SW2 switch

### 7. Emergency Procedures

#### Loss of Signal
- **Automatic**: Failsafe disarms after 1 second
- Drone will fall - clear area important!
- Do not fly beyond reliable range

#### Unstable Flight
1. Immediately reduce throttle
2. Disarm if out of control
3. Do not attempt to catch spinning propellers!
4. Review PID tuning section

#### Flyaway
1. Flip SW1 to disarm
2. If no response, power off remote (failsafe triggers)
3. Note direction and search area

### 8. Post-Flight Procedures

1. Disarm drone (SW1)
2. Disconnect battery immediately
3. Check for:
   - Loose screws
   - Wire damage
   - Propeller cracks
   - Hot motors or ESCs
4. Download flight logs if implemented
5. Charge battery (storage voltage if not flying soon)

## Control Sensitivity

### Throttle Response
- 0-25%: Minimal thrust
- 25-50%: Hovering range
- 50-75%: Climbing power
- 75-100%: Maximum power

### Stick Sensitivity
- **Center ±10**: Deadband (no input)
- **±10 to ±100**: Gradual response
- **±100 to ±500**: Full deflection

## Flight Modes Explained

### Mode 1: Stabilize (Beginner)
```
✓ Self-leveling enabled
✓ Limited tilt angles (±50°)
✓ Returns to level when sticks centered
✓ Altitude hold: NO (manual throttle)
✓ Heading hold: Yaw rate control
```

### Mode 2: Acro (Expert)
```
✗ No self-leveling
✓ Full rate control
✓ Unlimited rotation
✗ Can flip/roll
⚠️ Requires experience!
```

## Tuning for Different Conditions

### Windy Conditions
- Increase I gain slightly (helps hold position)
- Reduce maximum tilt angle
- Fly in Stabilize mode only

### Indoor Flying
- Reduce PID gains by 20%
- Lower maximum tilt angle
- Use gentle stick movements
- Ensure adequate clearance

### Heavy Payload
- Increase P gain
- May need more throttle for hover
- Check propeller efficiency
- Monitor motor temperatures

## Status Indicators

### LED Patterns
| Pattern | Meaning |
|---------|---------|
| Blinking slow (1Hz) | Disarmed, normal |
| Solid ON | Armed, ready to fly |
| Fast blinking | Error during calibration |
| Off | No power or failed initialization |

### Buzzer Codes
| Beeps | Meaning |
|-------|---------|
| 1 long | Starting calibration |
| 2 short | System ready / Disarmed |
| 1 short | Armed |
| 3 short | Failsafe triggered |
| Error code (n beeps, repeat) | Error #n occurred |

### Serial Monitor Messages

**Flight Controller:**
```
ARMED           - Drone is armed
DISARMED        - Drone is disarmed
FAILSAFE!       - Signal lost, auto-disarm
```

**Remote Controller:**
```
CONN [ARMED]    - Connected and armed
CONN [SAFE]     - Connected but disarmed
NO SIGNAL       - No connection to drone
```

## Advanced Operations

### PID Tuning Flight Test

1. Set all PID values to defaults
2. Arm and hover at 2m height
3. Give quick roll input
4. Observe response:
   - **Oscillates**: Reduce P or D
   - **Overshoots**: Increase D
   - **Slow**: Increase P
   - **Drifts**: Increase I
5. Make 10% adjustments only
6. Test each axis separately

### Calibration Validation

After calibration, check Serial Monitor:
```
Gyro Cal: X, Y, Z
```
- Values should be < 5.0 for all axes
- If higher, surface not level or vibrations present
- Recalibrate on different surface

### Battery Monitoring

Monitor voltage during flight:
- **Full**: 12.6V (3S) / 16.8V (4S)
- **Storage**: 11.4V (3S) / 15.2V (4S)
- **Warning**: 10.5V (3S) / 14.0V (4S)
- **Critical**: 9.9V (3S) / 13.2V (4S)

Land immediately if battery voltage drops below warning level.

## Common Flight Problems

### Problem: Drone flips on takeoff
**Solutions:**
- Check motor directions
- Verify propeller directions
- Recalibrate on level surface
- Check PID gains not too high

### Problem: Drifts in one direction
**Solutions:**
- Recalibrate IMU
- Increase I gain slightly
- Check propellers for damage
- Verify motor thrust equal

### Problem: Oscillates/bounces
**Solutions:**
- Reduce P gain
- Reduce D gain
- Check for loose connections
- Verify propellers balanced

### Problem: Sluggish response
**Solutions:**
- Increase P gain
- Check battery voltage
- Verify motor/ESC performance
- Reduce weight if possible

### Problem: Spins uncontrollably
**Solutions:**
- Check yaw PID gains
- Verify motor directions correct
- Test motors individually
- Check propeller condition

## Training Progression

### Week 1: Ground School
- [ ] Understand all controls
- [ ] Complete all calibrations
- [ ] Motor direction test
- [ ] Range test
- [ ] Failsafe test

### Week 2: Basic Flying
- [ ] Stable hover 30 seconds
- [ ] Controlled landing
- [ ] Forward/backward flight
- [ ] Left/right flight
- [ ] 360° rotation hover

### Week 3: Maneuvers
- [ ] Figure-8 pattern
- [ ] Controlled ascent/descent
- [ ] Circle pattern
- [ ] Return to home position
- [ ] Emergency stop practice

### Week 4: Advanced
- [ ] Switch to Acro mode
- [ ] Fast forward flight
- [ ] Quick direction changes
- [ ] Low altitude flight
- [ ] Long range flight

## Maintenance Schedule

### After Every Flight
- Visual inspection
- Propeller check
- Connection check
- Battery voltage check

### Weekly (10+ flights)
- Tighten all screws
- Clean propellers
- Check solder joints
- Verify calibration

### Monthly
- Deep clean all components
- Check for wire wear
- Test backup failsafe
- Update firmware if available

## Safety Rules - READ CAREFULLY

1. ⚠️ **NEVER** fly near airports
2. ⚠️ **NEVER** fly above 400 feet
3. ⚠️ **NEVER** fly over people
4. ⚠️ **ALWAYS** maintain line of sight
5. ⚠️ **ALWAYS** check local regulations
6. ⚠️ **ALWAYS** have fire extinguisher for LiPo
7. ⚠️ **NEVER** fly in rain
8. ⚠️ **ALWAYS** disconnect battery when working on drone

## Legal Considerations

- Check your country's drone regulations
- Register drone if required
- Obtain permits for certain areas
- Respect privacy laws
- Have liability insurance
- Follow local flying restrictions

---

**Remember: Safety first, fly responsibly! 🚁**

For technical details, see main README.md
For wiring, see WIRING_DIAGRAMS.md
