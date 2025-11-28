# 📝 CHANGELOG

## Version 2.0 - Major Overhaul (2025-11-28)

### 🔧 Hardware Changes
- **CRITICAL FIX**: Resolved pin conflicts between motors, NRF, and buttons/switches
- Moved NRF CE from D4 to D2 (D4 conflicted with motor)
- Moved calibration button from D4 to A6
- Moved smooth start button from D5 to A7
- Moved arm switch from D3 to D4
- Moved altitude hold switch from D2 to D7
- Moved LED from D7 to A3
- Motors remain on D3, D5, D6, D9 (PWM capable pins)

### ✨ New Features
- **NRF24L01 ACK enabled**: Reliable communication with automatic acknowledgment
- **Smooth motor start button (A7)**: Ramp motors up slowly for testing
- **MS5611 calibration**: Ground pressure saved to EEPROM during calibration
- **Enhanced LED behavior**: 
  - Always ON when disarmed (safety indicator)
  - Blinks when armed and receiving signals
- **Link confirmation**: Buzzer beeps when RC connects at startup
- **Better safety limits**: Max tilt angle reduced to 30° (from 180°)
- **Improved altitude hold**: Uses switch D7 instead of package.switch2

### 🐛 Bug Fixes
- Fixed race condition in motor start sequence
- Fixed altitude hold activation logic
- Corrected joystick mapping to match spec:
  - A0 = Throttle (was YL)
  - A1 = Yaw (was XL)
  - A2 = Pitch (was YR, now inverted)
  - A3 = Roll (was XR)
- Fixed motor output when not armed (now properly stops)
- Added input validation for EEPROM reads (NaN check)

### 📚 Documentation
- **NEW**: PIN_MAPPING.md - Complete pin assignment guide with conflict resolution
- **NEW**: USER_MANUAL.md - Comprehensive 400+ line user guide
- **NEW**: QUICK_REFERENCE.md - One-page cheat sheet
- **UPDATED**: README.md - Complete project overview

### 🔄 Code Improvements
- Added function prototypes
- Improved code comments
- Better variable naming
- Separated altitude hold logic
- Enhanced error handling
- Added debug output option
- Cleaner motor control flow

### ⚙️ Configuration Changes
```cpp
// OLD
RF24 radio(4, 10);           // Pin conflict!
radio.setAutoAck(false);     // No ACK
int maxAngle = 180;          // Too permissive

// NEW
RF24 radio(2, 10);           // No conflict
radio.setAutoAck(true);      // Reliable communication
int maxAngle = 30;           // Safer limit
```

### 🔒 Safety Enhancements
- Disarm indicator (LED always on) prevents accidental arming
- Link establishment check at startup
- NRF retry mechanism (5 retries, 15×250µs delay)
- Smooth motor start prevents sudden thrust
- Clearer buzzer feedback patterns

### 📊 System Architecture
```
Before:
- Mixed control logic
- Unclear pin assignments
- No motor start verification

After:
- Separated concerns (calibration, motor start, flight)
- Documented pin mappings
- Safe motor testing procedure
```

---

## Version 1.0 - Initial Release

### Features
- Basic PID stabilization
- MPU6050 integration
- MS5611 altitude sensing
- NRF24L01 communication
- Simple arming mechanism
- Altitude hold mode
- Kalman filter for altitude

### Known Issues
- Pin conflicts between motors and buttons
- No ACK on NRF (unreliable)
- No motor testing procedure
- Unclear calibration process
- Missing documentation

---

## Migration Guide (v1.0 → v2.0)

### Hardware Changes Required
1. Move NRF CE wire from D4 to D2
2. Move calibration button from D4 to A6
3. Move smooth start button from D5 to A7
4. Move arm switch from D3 to D4
5. Move altitude hold switch from D2 to D7
6. Move LED from D7 to A3
7. **Add 10µF capacitor** across NRF power pins (if not present)

### Software Changes
1. Re-upload firmware to both FC and RC
2. Recalibrate on level surface (new EEPROM layout)
3. Test with smooth motor start feature (no props!)
4. Verify motor directions match documentation

### New Workflow
```
OLD: Arm → Motors spin immediately
NEW: Arm → Press smooth start → Motors ramp up slowly
```

### Benefits of Upgrading
- ✅ Reliable NRF communication (ACK)
- ✅ Safe motor testing procedure
- ✅ Better visual feedback (LED)
- ✅ Comprehensive documentation
- ✅ Resolved all pin conflicts
- ✅ MS5611 calibration included

---

## Future Roadmap

### Planned Features
- [ ] GPS module integration
- [ ] Return-to-home functionality
- [ ] Waypoint navigation
- [ ] Battery voltage telemetry to RC
- [ ] LCD display on RC
- [ ] Blackbox logging (SD card)
- [ ] PID auto-tuning
- [ ] Mobile app control (Bluetooth)
- [ ] Multiple flight modes (Acro, Angle, Horizon)

### Under Consideration
- [ ] Optical flow sensor for indoor flight
- [ ] Ultrasonic altitude sensing (ground proximity)
- [ ] Gesture control
- [ ] Voice commands
- [ ] FPV camera integration
- [ ] Multi-drone swarm capability

---

## Known Issues

### Current Limitations
1. **No failsafe altitude**: Drone falls if signal lost mid-flight
   - Workaround: Fly in open areas only
2. **No GPS**: Cannot return to launch point
   - Workaround: Keep visual contact
3. **No battery telemetry**: Must monitor manually
   - Workaround: Use battery alarm or timer
4. **Fixed PID values**: Not adaptive to different loads
   - Workaround: Retune for different payloads

### Won't Fix
- Optical flow (requires additional hardware)
- Advanced failsafe (needs GPS)
- In-flight PID tuning (complexity vs. benefit)

---

## Contributing

If you improve this project:
1. Document hardware changes in PIN_MAPPING.md
2. Update USER_MANUAL.md for new features
3. Test thoroughly before flight
4. Add entry to this CHANGELOG.md

---

## Version History Summary

| Version | Date | Key Changes |
|---------|------|-------------|
| 2.0 | 2025-11-28 | Pin fix, ACK, smooth start, docs |
| 1.0 | Original | Initial working version |

---

**Current Version: 2.0**
**Last Updated: 2025-11-28**
