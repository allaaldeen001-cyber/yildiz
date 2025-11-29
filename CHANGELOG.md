# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.0.0] - 2025-11-29

### 🎉 Initial Release

**Professional Arduino Nano Drone System - First stable version**

### Added - Flight Controller
- ✅ Complete PID-based flight stabilization
- ✅ MPU6050 6-axis IMU integration with complementary filter
- ✅ NRF24L01 PA+LNA wireless communication (2.4GHz)
- ✅ 4-motor quadcopter control (X configuration)
- ✅ Automatic gyroscope calibration routine
- ✅ ESC calibration sequence
- ✅ Safety features:
  - Maximum 30° tilt angle limit
  - 65% throttle cap
  - Failsafe mode on signal loss
  - Kill switch support
  - Low throttle disarm
- ✅ Buzzer audio feedback:
  - Startup confirmation
  - Calibration success/failure
  - Arming status
  - System alerts
- ✅ Status LED indicators
- ✅ Bidirectional telemetry with ACK packets
- ✅ 250Hz control loop frequency
- ✅ Professional code structure with extensive comments

### Added - Remote Controller
- ✅ Dual joystick control (4-axis):
  - Left stick: Throttle and Yaw
  - Right stick: Pitch and Roll
- ✅ NRF24L01 PA+LNA wireless transmission
- ✅ Button controls:
  - Calibration trigger (Button 1)
  - Motor arming (Button 2)
- ✅ Toggle switches:
  - Altitude hold mode (Switch 1) - framework ready
  - Kill switch / Arming (Switch 2)
- ✅ Real-time serial monitor dashboard:
  - Communication status with success rate
  - Live control input values with progress bars
  - Button and switch states
  - Drone telemetry display
  - Attitude angles
  - System warnings and notifications
- ✅ Joystick deadband and calibration
- ✅ Checksum verification for data integrity
- ✅ 50Hz transmission rate

### Added - Documentation
- ✅ Comprehensive README with project overview
- ✅ QUICK_START.md - Get flying in 30 minutes
- ✅ WIRING_DIAGRAM.md - Complete connection schematics
- ✅ CALIBRATION_GUIDE.md - Step-by-step procedures
- ✅ TROUBLESHOOTING.md - Common issues and solutions
- ✅ PARTS_LIST.md - Complete BOM and shopping guide
- ✅ CONTRIBUTING.md - Contribution guidelines
- ✅ LIBRARIES.txt - Required library information
- ✅ LICENSE - MIT license with safety disclaimer

### Technical Specifications
- **Control Loop**: 250Hz for stable flight
- **Communication**: 50Hz bidirectional with ACK
- **Radio Channel**: 103 (2.503 GHz)
- **Range**: ~500m (open area with PA+LNA modules)
- **PID Defaults**: Roll/Pitch Kp=1.3, Ki=0.04, Kd=18.0
- **Safety Limits**: 30° max angle, 65% max throttle

### Hardware Support
- **MCU**: Arduino Nano (ATmega328P)
- **IMU**: MPU6050 (GY-521)
- **Radio**: NRF24L01 PA+LNA
- **Motors**: Brushless outrunner (1000-1300KV tested)
- **ESC**: Standard PWM ESCs (tested with 30A)
- **Frame**: Standard X-configuration quadcopter

### Known Limitations
- Altitude hold requires barometer (not yet implemented)
- GPS features not implemented (planned for future)
- Battery voltage monitoring shows placeholder value
- ESC control uses analogWrite (Servo library recommended for precision)

### Dependencies
- RF24 library v1.4.2+ by TMRh20
- Wire library (built-in)
- Arduino IDE 1.8.13+ or 2.0+

---

## [Unreleased]

### Planned Features
- [ ] Altitude hold using BMP280/BME280 barometer
- [ ] Real battery voltage monitoring
- [ ] GPS position hold
- [ ] Return to home (RTH)
- [ ] Automatic PID tuning
- [ ] Flight data logging
- [ ] Waypoint navigation
- [ ] Acro mode (rate mode)
- [ ] Angle mode optimization
- [ ] OLED display for remote
- [ ] Configuration via serial commands

### Planned Improvements
- [ ] Migrate to Servo library for ESC control
- [ ] Optimize I2C speed for MPU6050
- [ ] Add moving average filter for joysticks
- [ ] Implement exponential control curves
- [ ] Add flight mode switching
- [ ] Implement smooth arming/disarming
- [ ] Add pre-flight check sequence
- [ ] Optimize code size and RAM usage

### Planned Hardware Support
- [ ] ESP32 port (WiFi telemetry)
- [ ] STM32 port (faster processing)
- [ ] MPU9250 support (9-axis IMU)
- [ ] Alternative radios (HC-12, LoRa)
- [ ] Support for different frame sizes

---

## Version History Summary

### v1.0.0 (2025-11-29)
**First stable release** - Complete working drone system with:
- Full PID stabilization
- Wireless control
- Safety features
- Comprehensive documentation

---

## How to Read This Changelog

**Version Format**: MAJOR.MINOR.PATCH

- **MAJOR**: Incompatible API changes
- **MINOR**: New features (backwards compatible)
- **PATCH**: Bug fixes (backwards compatible)

**Categories:**
- `Added`: New features
- `Changed`: Changes to existing features
- `Deprecated`: Soon-to-be removed features
- `Removed`: Removed features
- `Fixed`: Bug fixes
- `Security`: Security fixes

---

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on how to contribute.

---

## Support

- **Issues**: Report bugs via GitHub Issues
- **Discussions**: Ask questions in GitHub Discussions
- **Documentation**: See docs/ folder

---

*Keep your drone firmware up to date for the latest features and fixes!*
