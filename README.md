# Professional Quadcopter System v2.0

A complete quadcopter flight control system with stabilization, PID control, and wireless communication.

## 🔥 LATEST UPDATE: Stabilization Fix

**Issue Fixed:** Motors now properly respond to MPU6050 tilting for stabilization!

**What's New:**
- ✅ Real-time debugging output showing angles, PID, and motor speeds
- ✅ Minimum throttle (1100µs) ensures stabilization is visible when testing
- ✅ Clear motor response when tilting drone by hand
- ✅ Easy verification that MPU6050 and PID are working

**Quick Test:** Upload code, ARM drone (no props!), tilt it by hand, watch motors respond!

👉 **See:** `HOW_TO_TEST_STABILIZATION.md` for 5-minute test procedure

## Hardware Components

### Flight Controller Board
- **Microcontroller**: Arduino Nano
- **Radio**: nRF24L01+ PA (CE:D4, CSN:D10)
- **IMU**: MPU6050 (I2C: SDA/SCL, INT:D2)
- **Buzzer**: D8
- **Status LED**: D7
- **ESC/Motors** (X Configuration):
  - Front Left (FL): D3
  - Front Right (FR): D5
  - Rear Right (RR): D6
  - Rear Left (RL): D9

### Remote Controller Board
- **Microcontroller**: Arduino Nano
- **Radio**: nRF24L01+ PA+LNA (CE:D9, CSN:D10)
- **Left Joystick**:
  - Vertical (Throttle): A0
  - Horizontal (Yaw): A1
- **Right Joystick**:
  - Vertical (Pitch): A2
  - Horizontal (Roll): A3
- **Push Buttons**:
  - Button 1: D4
  - Button 2: D5
  - Button 3: D6
  - Button 4: D7
- **Toggle Switches**:
  - SW1 (Arm/Disarm): D2
  - SW2 (Flight Mode): D3

## Features

### Flight Controller
- ✅ **250Hz Control Loop** - Fast and responsive
- ✅ **Complementary Filter** - Combines gyro and accelerometer data
- ✅ **3-Axis PID Control** - Roll, Pitch, and Yaw stabilization
- ✅ **Motor Mixing** - Quadcopter X configuration
- ✅ **Automatic Calibration** - Sensor offset calibration on startup
- ✅ **Failsafe Protection** - Auto-disarm on signal loss
- ✅ **Status Indicators** - LED and buzzer feedback
- ✅ **Flight Modes** - Stabilize and Acro modes

### Remote Controller
- ✅ **50Hz Transmission** - Smooth control updates
- ✅ **Joystick Calibration** - Auto-center calibration
- ✅ **Deadband Filtering** - Eliminates stick drift
- ✅ **Connection Monitoring** - Real-time link status
- ✅ **Serial Debugging** - Live data monitoring

## Installation

### Required Libraries

Install these libraries via Arduino IDE Library Manager:

1. **RF24** by TMRh20
   - Go to: Sketch → Include Library → Manage Libraries
   - Search for "RF24"
   - Install "RF24 by TMRh20"

2. **Wire** (Built-in with Arduino IDE)
   - No installation needed

3. **Servo** (Built-in with Arduino IDE)
   - No installation needed

4. **SPI** (Built-in with Arduino IDE)
   - No installation needed

### Board Setup

1. **Select Board**: Arduino Nano
2. **Select Processor**: ATmega328P (Old Bootloader) - try this first if upload fails
3. **Select Port**: Choose the correct COM port

## Wiring Diagrams

### Flight Controller Connections

```
Arduino Nano → nRF24L01+
  D4  → CE
  D10 → CSN
  D11 → MOSI
  D12 → MISO
  D13 → SCK
  3.3V → VCC
  GND  → GND

Arduino Nano → MPU6050
  A4 (SDA) → SDA
  A5 (SCL) → SCL
  D2       → INT
  3.3V     → VCC
  GND      → GND

Arduino Nano → ESCs
  D3 → Front Left ESC Signal
  D5 → Front Right ESC Signal
  D6 → Rear Right ESC Signal
  D9 → Rear Left ESC Signal

Arduino Nano → Peripherals
  D7 → Status LED (+) → 220Ω → GND
  D8 → Buzzer (+) → GND
```

### Remote Controller Connections

```
Arduino Nano → nRF24L01+
  D9  → CE
  D10 → CSN
  D11 → MOSI
  D12 → MISO
  D13 → SCK
  3.3V → VCC
  GND  → GND

Arduino Nano → Left Joystick
  A0 → VRy (Vertical)
  A1 → VRx (Horizontal)
  5V → +5V
  GND → GND

Arduino Nano → Right Joystick
  A2 → VRy (Vertical)
  A3 → VRx (Horizontal)
  5V → +5V
  GND → GND

Arduino Nano → Buttons (with internal pullup)
  D4 → Button 1 → GND
  D5 → Button 2 → GND
  D6 → Button 3 → GND
  D7 → Button 4 → GND

Arduino Nano → Toggle Switches
  D2 → SW1 (Arm) → GND
  D3 → SW2 (Mode) → GND
```

## First Time Setup

### 1. Upload Code

**Flight Controller:**
1. Open `FlightController/FlightController.ino`
2. Select: Tools → Board → Arduino Nano
3. Select: Tools → Processor → ATmega328P (Old Bootloader)
4. Select correct COM port
5. Click Upload

**Remote Controller:**
1. Open `RemoteController/RemoteController.ino`
2. Follow same board settings
3. Click Upload

### 2. Calibration

**Flight Controller Calibration:**
1. Place drone on flat, level surface
2. Power on the flight controller
3. **Keep drone completely still** during calibration
4. Wait for 2 beeps (about 6 seconds)
5. LED will stay solid when ready

**Remote Controller Calibration:**
1. Power on remote controller
2. **Center both joysticks**
3. Wait 2 seconds for auto-calibration
4. Check Serial Monitor for calibration values

### 3. Pre-Flight Checks

Before first flight:
1. ✅ Remove propellers for testing
2. ✅ Verify all connections are secure
3. ✅ Ensure battery is fully charged
4. ✅ Test range is clear of obstacles
5. ✅ ESCs are calibrated (see below)

## ESC Calibration (Important!)

Perform this once before first flight:

1. **Disconnect** flight controller from battery
2. Upload the flight controller code
3. Turn on remote controller
4. Move throttle stick to **MAXIMUM**
5. Connect flight controller to battery
6. Wait for ESC beep sequence
7. Move throttle stick to **MINIMUM**
8. Wait for confirmation beeps
9. ESCs are now calibrated!

## Operation Guide

### Starting the System

1. **Power on Remote Controller first**
   - Wait for "System Ready!" message
   - Check Serial Monitor shows calibration values

2. **Power on Flight Controller**
   - Place on level surface
   - Wait for calibration (6 seconds)
   - Wait for 2 beeps and solid LED

3. **Check Connection**
   - Remote Serial Monitor should show "CONN"
   - If "NO SIGNAL", check radio connections

### Arming the Drone

**⚠️ REMOVE PROPELLERS FOR FIRST TEST! ⚠️**

1. Ensure throttle stick is at minimum
2. Flip SW1 switch to ARM position
3. Flight controller will beep once
4. LED stays solid
5. Remote shows "[ARMED]"

**To Disarm:**
- Flip SW1 switch to DISARM position
- Controller beeps twice
- LED starts blinking

### Flight Controls

**Left Stick:**
- Up/Down: **Throttle** (altitude control)
- Left/Right: **Yaw** (rotation)

**Right Stick:**
- Up/Down: **Pitch** (forward/backward)
- Left/Right: **Roll** (left/right)

**Switches:**
- SW1: Arm/Disarm
- SW2: Flight Mode (Stabilize/Acro)

### Flight Modes

**Stabilize Mode (Recommended for beginners):**
- Drone self-levels when sticks centered
- Maximum tilt angles limited to ±50°
- Easier to control

**Acro Mode (Advanced):**
- No self-leveling
- Full manual control
- For experienced pilots

### Safety Features

**Failsafe:**
- If signal lost for >1 second:
  - Motors stop immediately
  - Drone disarms automatically
  - 3 beeps sound
  - Prevents flyaway

**Low Throttle Arming:**
- Can only arm when throttle <5%
- Prevents accidental spin-up

**Status LED:**
- Blinking: Disarmed/Safe
- Solid: Armed/Ready
- Error codes: See troubleshooting

## Monitoring and Debugging

### Serial Monitor Setup

**Flight Controller (115200 baud):**
```
=================================
Quadcopter Flight Controller v2.0
=================================
[OK] Motors initialized
[OK] Radio initialized
[OK] MPU6050 initialized
Calibrating sensors...
Keep drone level and still!
[OK] Calibration complete!
Gyro Cal: 0.12, -0.34, 0.08
System Ready!
```

**Remote Controller (115200 baud):**
```
=================================
Quadcopter Remote Controller v2.0
=================================
[OK] Radio initialized
Calibrating joysticks...
Centers: LV=512 LH=508 RV=515 RH=510
System Ready!

T:1000 R:0 P:0 Y:0 | SW:0 BTN:0 | CONN [SAFE]
```

## Troubleshooting

### Radio Issues

**"NO SIGNAL" on Remote:**
- Check nRF24L01+ connections
- Ensure both use 3.3V (NOT 5V!)
- Add 10µF capacitor across VCC/GND
- Try different USB power source
- Check CE and CSN pins match code

**Intermittent Connection:**
- Add 10µF capacitor to nRF24L01+ VCC
- Use shorter wires (<10cm)
- Keep nRF24L01+ away from motors
- Use shielded wires if possible

### Flight Controller Issues

**"MPU6050 not found!":**
- Check I2C connections (A4=SDA, A5=SCL)
- Verify 3.3V power to MPU6050
- Try different I2C address (0x69)
- Check for solder bridges

**Motors Don't Spin:**
- Verify ESC calibration completed
- Check ESC signal wire connections
- Ensure ESC BEC can power Arduino
- Test motors individually with servo tester

**Drone Drifts/Unstable:**
- Recalibrate on level surface
- Adjust PID gains (see tuning section)
- Check propeller direction
- Verify motor rotation order
- Check for loose connections

### Calibration Issues

**Calibration Fails:**
- Ensure surface is perfectly level
- Keep drone completely still
- No vibrations during calibration
- Wait full 6 seconds

**Joysticks Not Centered:**
- Re-run joystick calibration
- Clean joystick contacts
- Check for mechanical binding
- Replace faulty joysticks

## PID Tuning (Advanced)

Default PID values (in FlightController.ino):

```cpp
// Roll/Pitch
#define PID_ROLL_KP 1.4
#define PID_ROLL_KI 0.05
#define PID_ROLL_KD 18.0

// Yaw
#define PID_YAW_KP 3.0
#define PID_YAW_KI 0.02
#define PID_YAW_KD 0.0
```

### Tuning Guide:

1. **Start with defaults** - Fly and observe behavior
2. **If oscillating** - Reduce P, reduce D
3. **If sluggish** - Increase P slightly
4. **If drift over time** - Increase I slightly
5. **Make small changes** - 10% adjustments

## Motor Configuration

```
     FRONT
   FL     FR
      \ /
       X
      / \
   RL     RR
     REAR
```

**Motor Rotation (X Configuration):**
- FL: Counter-clockwise (CCW)
- FR: Clockwise (CW)
- RR: Counter-clockwise (CCW)
- RL: Clockwise (CW)

## Power Requirements

- **Flight Controller**: 5V via USB or ESC BEC
- **nRF24L01+**: 3.3V (100mA peak)
- **MPU6050**: 3.3V (3.9mA)
- **Total**: ~150mA @ 3.3V, ~50mA @ 5V

**Battery Recommendations:**
- LiPo 3S or 4S (11.1V - 14.8V)
- Minimum 1500mAh for 5-10 min flight
- 25C discharge rating or higher

## Performance Specifications

- **Control Loop**: 250Hz (4ms)
- **Radio Update**: 50Hz (20ms)
- **Radio Range**: ~1000m (line of sight)
- **Latency**: ~25ms total
- **Max Tilt**: ±50° (Stabilize mode)
- **PID Output**: ±400 (constrained)

## Safety Warnings

⚠️ **CRITICAL SAFETY:**
1. **ALWAYS** remove propellers for testing
2. **NEVER** arm indoors with propellers
3. **ALWAYS** check battery voltage before flight
4. **NEVER** fly near people or animals
5. **ALWAYS** have failsafe enabled
6. **KEEP** fingers away from propellers
7. **WEAR** safety glasses during testing

## File Structure

```
/workspace/
├── FlightController/
│   └── FlightController.ino    # Flight controller firmware
├── RemoteController/
│   └── RemoteController.ino    # Remote controller firmware
├── README.md                    # This file
├── OPERATION_GUIDE.md          # Detailed operating procedures
└── WIRING_DIAGRAMS.md          # Visual wiring guides
```

## Technical Details

### Communication Protocol

**Data Packet (Flight Controller ← Remote):**
```cpp
struct ControlData {
  int16_t throttle;  // 1000-2000 µs
  int16_t roll;      // -500 to +500
  int16_t pitch;     // -500 to +500
  int16_t yaw;       // -500 to +500
  uint8_t switches;  // Bit flags
  uint8_t buttons;   // Bit flags
  uint8_t checksum;  // Integrity check
};
```

### Control Algorithm

1. **Read IMU** (MPU6050 @ 250Hz)
2. **Calculate angles** (Complementary filter)
3. **Calculate PID** (3-axis stabilization)
4. **Mix motors** (X configuration)
5. **Update outputs** (PWM to ESCs)

## License

This project is open-source for educational and personal use.

## Support

For issues or questions:
1. Check troubleshooting section
2. Verify all connections
3. Check Serial Monitor output
4. Review calibration procedure

## Version History

- **v2.0** - Complete rewrite with professional features
  - 250Hz control loop
  - Complementary filter
  - Tuned PID controllers
  - Failsafe protection
  - Improved calibration

---

**Built with ❤️ for the maker community**

**Fly safe! 🚁**
