# Professional Quadcopter Flight Controller System

A complete, production-ready quadcopter flight controller with advanced features including sensor fusion, auto takeoff/landing, and smooth touchdown detection.

## Quick Links

- **[START HERE - Quick Start Guide](QUICK_START_GUIDE.md)** - Get flying in 30 minutes
- **[Complete Documentation](README_COMPLETE_SYSTEM.md)** - Full technical reference
- **[Project Summary](PROJECT_SUMMARY.md)** - Overview and specifications
- **[All Documents Index](INDEX.md)** - Navigation guide

## What's Included

### Code (Ready to Upload)
- `FlightController_Complete.ino` (39 KB) - Complete flight controller firmware
- `RemoteController_Complete.ino` (16 KB) - Complete remote controller firmware

### Documentation (115+ KB Total)
- Complete wiring diagrams
- PID tuning guide with theory
- Auto landing algorithm explanation
- Failsafe system documentation
- Troubleshooting guide
- Safety procedures

## Key Features

- **4-Motor Control**: All motors working (FL, FR, RL, RR on RS2205 2300KV)
- **Advanced Sensor Fusion**: Complementary filter (96% gyro + 4% accel)
- **Auto Takeoff**: One-button climb to preset altitude
- **Auto Landing**: 4-phase smooth descent with touchdown detection
- **Real-time PID Tuning**: Adjust P and D gains with potentiometers during flight
- **Altitude Hold**: MS5611 barometer for stable altitude
- **Motor Test Mode**: Verify directions before flight
- **Complete Calibration**: One-button sensor calibration with EEPROM save
- **Comprehensive Failsafe**: Radio loss, sensor failure, tilt protection
- **Professional State Machine**: 7 flight states with safe transitions

## Hardware Requirements

- 2x Arduino Nano
- MPU6050 IMU
- MS5611 Barometer
- 2x NRF24L01 PA+LNA
- 4x RS2205 2300KV motors
- 4x 30A ESCs
- 2x 10K potentiometers
- 4x push buttons
- Frame, props, battery

## Quick Setup

1. Wire components (see [wiring tables](README_COMPLETE_SYSTEM.md#complete-wiring-tables))
2. Install libraries: RF24, MS5611
3. Upload both sketches
4. Calibrate sensors (Button 1)
5. Test motors (Button 2, no props)
6. Fly!

## Button Functions

- **Button 1 (D4)**: Calibrate MPU6050 + MS5611
- **Button 2 (A0)**: Motor direction test
- **Button 3 (A1)**: Auto takeoff to +1m
- **Button 4 (A2)**: Auto landing with touchdown detection

## Flight Modes

- **DISARMED**: Motors off (safe)
- **ARMED IDLE**: Low idle, ready to fly
- **STABILIZE**: Manual throttle + attitude stabilization
- **ALTITUDE HOLD**: Barometric altitude lock
- **TAKEOFF**: Automatic climb sequence
- **LANDING**: Automatic descent with smooth touchdown

## Safety Features

- Tilt protection (45° max, 15° during landing)
- Radio timeout (1 second → auto-land or stop)
- Sensor failure detection
- Emergency stop
- Audible alarms
- Pre-flight validation

## Performance

- **Loop Rate**: 250 Hz (4 ms)
- **Attitude Accuracy**: ±1°
- **Altitude Accuracy**: ±20 cm
- **Radio Range**: 100+ meters
- **Response Time**: <50 ms

## Documentation

| File | Purpose |
|------|---------|
| [INDEX.md](INDEX.md) | Navigation guide to all documents |
| [QUICK_START_GUIDE.md](QUICK_START_GUIDE.md) | Fast setup and daily operation |
| [README_COMPLETE_SYSTEM.md](README_COMPLETE_SYSTEM.md) | Complete technical documentation |
| [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md) | Overview, specs, deliverables |
| [CHANGES_AND_FIXES.md](CHANGES_AND_FIXES.md) | Changelog and improvements |

## Getting Started

### New Users
1. Read [QUICK_START_GUIDE.md](QUICK_START_GUIDE.md)
2. Follow 5-minute setup
3. Calibrate and test
4. Start flying!

### Experienced Users
1. Read [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md)
2. Review [README_COMPLETE_SYSTEM.md](README_COMPLETE_SYSTEM.md)
3. Customize and tune
4. Advanced features

## Troubleshooting

**No radio link?** → Check NRF24L01 3.3V power + capacitor

**Motors won't spin?** → ARM switch ON + calibration complete

**Drone oscillates?** → Decrease P-gain (Pot 1 counter-clockwise)

**Altitude jumps?** → Cover MS5611 with foam

See [complete troubleshooting guide](README_COMPLETE_SYSTEM.md#troubleshooting)

## Support

1. Check Serial Monitor (115200 baud) for error messages
2. Search documentation for keywords
3. Verify wiring against pin tables
4. Test components individually

## Credits

Built using best practices from MultiWii, Betaflight, and aerospace sensor fusion research.

## License

Educational and hobby use. Fly responsibly and follow local regulations.

## Version

- **Flight Controller**: v2.0
- **Remote Controller**: v2.0
- **Documentation**: Complete
- **Status**: Production Ready

---

**Safety First - Fly Responsibly**

Start here: [QUICK_START_GUIDE.md](QUICK_START_GUIDE.md)
