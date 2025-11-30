# Project File Structure

```
/workspace/
│
├── README.md                           # Main project documentation
│
├── RemoteController/
│   └── RemoteController.ino            # Remote Controller Arduino sketch
│
├── FlightController/
│   └── FlightController.ino            # Flight Controller Arduino sketch
│
└── docs/
    ├── LIBRARIES.md                    # Library installation guide
    ├── PIN_CONFIGURATION.md            # Complete pin mapping reference
    ├── WIRING_GUIDE.md                 # Step-by-step wiring instructions
    ├── QUICK_START.md                  # Quick start guide for first flight
    ├── PID_TUNING.md                   # PID tuning guide
    ├── TROUBLESHOOTING.md              # Common issues and solutions
    ├── PARTS_LIST.md                   # Complete bill of materials
    └── PROJECT_STRUCTURE.md            # This file
```

## File Descriptions

### Root Level

- **README.md**: Main entry point with project overview, features, hardware requirements, and operation procedures

### RemoteController/

- **RemoteController.ino**: Complete Arduino code for the remote controller board including:
  - NRF24L01 communication (TX mode)
  - Joystick reading and calibration
  - Button and switch handling
  - Serial monitor status display
  - Data packet transmission with ACK

### FlightController/

- **FlightController.ino**: Complete Arduino code for the flight controller board including:
  - NRF24L01 communication (RX mode)
  - MPU6050 IMU sensor reading
  - Complementary filter for angle estimation
  - PID control algorithm
  - Motor mixing for quadcopter X configuration
  - Safety features (failsafe, angle limits, throttle caps)
  - Calibration routines (gyro, ESC)
  - Audio/visual feedback

### docs/

Comprehensive documentation covering all aspects:

1. **LIBRARIES.md**
   - Required Arduino libraries
   - Installation instructions
   - Version compatibility
   - Troubleshooting library issues

2. **PIN_CONFIGURATION.md**
   - Complete pinout for both boards
   - Arduino Nano pin diagrams
   - Pin assignment tables
   - Hardware interface details

3. **WIRING_GUIDE.md**
   - Step-by-step wiring instructions
   - Visual wiring diagrams
   - Power system setup
   - Component mounting
   - Wiring verification checklist

4. **QUICK_START.md**
   - Fast track to first flight
   - Upload instructions
   - Calibration procedures
   - Pre-flight checklist
   - Safety procedures

5. **PID_TUNING.md**
   - PID theory explanation
   - Default values
   - Step-by-step tuning process
   - Drone size recommendations
   - Common issues and solutions

6. **TROUBLESHOOTING.md**
   - Hardware issues (NRF, MPU6050, motors, etc.)
   - Software issues (compilation, upload, etc.)
   - Flight issues (oscillations, drift, etc.)
   - Diagnostic checklists

7. **PARTS_LIST.md**
   - Complete bill of materials
   - Price estimates
   - Where to buy recommendations
   - Optional components
   - Tool requirements

8. **PROJECT_STRUCTURE.md**
   - This file
   - Project organization
   - File descriptions

## Code Organization

### RemoteController.ino Structure

```
- Header comments and documentation
- Libraries inclusion
- Configuration defines
- Data structures (RC_Data, FC_Ack)
- Global variables
- setup() function
  ├─ Pin initialization
  ├─ Radio initialization
  ├─ Joystick calibration
  └─ Data initialization
- loop() function
  ├─ Input reading (joysticks, buttons, switches)
  ├─ Data transmission
  ├─ ACK reception
  └─ Status display
- Helper functions
  ├─ initializePins()
  ├─ initializeRadio()
  ├─ calibrateJoysticks()
  ├─ readJoysticks()
  ├─ readButtons()
  ├─ readSwitches()
  ├─ sendData()
  ├─ processAcknowledgment()
  ├─ calculateChecksum()
  └─ displayStatus()
```

### FlightController.ino Structure

```
- Header comments and documentation
- Libraries inclusion
- Configuration defines
- Data structures (RC_Data, FC_Ack, PID, IMU_Data, Motors)
- Enums (FC_State, Cal_Result)
- Global variables
- setup() function
  ├─ Pin initialization
  ├─ MPU6050 initialization
  ├─ Radio initialization
  ├─ PID initialization
  └─ Motor initialization
- loop() function (250Hz)
  ├─ IMU reading
  ├─ RC data reception
  ├─ State processing
  ├─ PID calculation
  ├─ Motor mixing
  └─ Motor update
- Helper functions
  ├─ IMU Functions
  │   ├─ readIMU()
  │   └─ calibrateGyro()
  ├─ Communication Functions
  │   ├─ receiveRC()
  │   ├─ sendAcknowledgment()
  │   └─ calculateChecksum()
  ├─ State Machine
  │   ├─ processState()
  │   └─ activateFailsafe()
  ├─ PID Control
  │   ├─ computePID()
  │   ├─ calculatePID()
  │   └─ resetPID()
  ├─ Motor Control
  │   ├─ mixMotors()
  │   ├─ stopMotors()
  │   └─ updateMotors()
  ├─ Calibration & Tests
  │   ├─ performESCCalibration()
  │   └─ performMotorTest()
  ├─ Feedback Functions
  │   ├─ playStartupTone()
  │   ├─ playCalibrationSuccessTone()
  │   ├─ playErrorTone()
  │   ├─ playArmTone()
  │   ├─ playDisarmTone()
  │   ├─ playESCCalTone()
  │   ├─ blinkLED()
  │   └─ updateStatusLED()
  └─ Utility Functions
      └─ map_float()
```

## Key Features by File

### RemoteController.ino
- ✓ 50Hz control loop
- ✓ Joystick calibration and deadband
- ✓ Button debouncing
- ✓ Reliable NRF24L01 communication with ACK
- ✓ Checksum verification
- ✓ Comprehensive Serial Monitor display
- ✓ Telemetry reception from FC

### FlightController.ino
- ✓ 250Hz flight control loop
- ✓ IMU sensor fusion (complementary filter)
- ✓ PID control for roll, pitch, yaw
- ✓ Motor mixing (X configuration)
- ✓ Safety features (failsafe, angle limits, throttle cap)
- ✓ Gyro calibration routine
- ✓ ESC calibration routine
- ✓ Motor test function
- ✓ Audio/visual feedback
- ✓ State machine architecture

## Configuration Constants

Both sketches use `#define` for easy configuration:

### Communication
- NRF channel
- Data rate
- Power level
- Pipe addresses

### Safety
- Max angle limits
- Throttle caps
- Failsafe timeout
- Min armed throttle

### Control
- PID gains (Kp, Ki, Kd)
- Max rates
- Loop timing

### Hardware
- Pin assignments
- ESC PWM range
- Sensor ranges

## Data Flow

```
Remote Controller              Flight Controller
      │                               │
      ├─ Read Joysticks              │
      ├─ Read Buttons                │
      ├─ Read Switches               │
      │                               │
      ├─ Package Data ───────────────>├─ Receive RC Data
      │   (RC_Data struct)            │   (with checksum verify)
      │                               │
      │                               ├─ Read IMU
      │                               ├─ Calculate PID
      │                               ├─ Mix Motors
      │                               ├─ Update ESCs
      │                               │
      │<──────── Send ACK ────────────┤
      │   (FC_Ack struct)             │   (telemetry)
      │                               │
      ├─ Display Status               ├─ Update LED/Buzzer
```

## Memory Usage

### RemoteController.ino
- Flash: ~20KB / 30KB (66%)
- SRAM: ~1.2KB / 2KB (60%)

### FlightController.ino
- Flash: ~25KB / 30KB (83%)
- SRAM: ~1.5KB / 2KB (75%)

**Note**: Arduino Nano has 30KB flash and 2KB SRAM

## Future Enhancements

Possible additions (not implemented):
- [ ] Altitude hold using barometer
- [ ] GPS position hold
- [ ] Return-to-home function
- [ ] FPV camera integration
- [ ] Telemetry logging to SD card
- [ ] Wireless PID tuning
- [ ] Multiple flight modes
- [ ] Battery voltage monitoring
- [ ] Blackbox flight recorder

## Version History

**v1.0** (2025-11-30)
- Initial release
- Full quadcopter control system
- PID stabilization
- Safety features
- Comprehensive documentation

---

**Last Updated**: 2025-11-30
