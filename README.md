# Arduino Nano Quadcopter Flight Controller v2.0

A professional flight controller system for Arduino Nano with MPU6050 DMP, MS5611 barometer, and reliable NRF24L01 communication.

## Features

- **MPU6050 with DMP** - Hardware-accelerated sensor fusion for stable attitude
- **MS5611 Barometer** - Altitude hold and smooth landing capability
- **NRF24L01 with ACK** - Reliable communication with packet confirmation
- **Serial Debug** - Real-time monitoring on both devices
- **Auto-Disarm** - Safety failsafe on connection loss
- **Self-Leveling** - Angle mode for easy flying

---

## Required Libraries

### For Flight Controller:

| Library | Author | Install Method | Purpose |
|---------|--------|----------------|---------|
| **I2Cdev** | Jeff Rowberg | Manual | I2C device communication |
| **MPU6050** | Jeff Rowberg | Manual | IMU with DMP support |
| **MS5611** | Rob Tillaart | Library Manager | Barometer for altitude |
| **RF24** | TMRh20 | Library Manager | NRF24L01 communication |

### For Remote Controller:

| Library | Author | Install Method | Purpose |
|---------|--------|----------------|---------|
| **RF24** | TMRh20 | Library Manager | NRF24L01 communication |

---

## Library Installation

### 1. RF24 Library (Arduino Library Manager)

```
Arduino IDE → Sketch → Include Library → Manage Libraries
Search: "RF24"
Install: "RF24 by TMRh20"
```

### 2. MS5611 Library (Arduino Library Manager)

```
Arduino IDE → Sketch → Include Library → Manage Libraries
Search: "MS5611"
Install: "MS5611 by Rob Tillaart"
```

### 3. I2Cdev + MPU6050 with DMP (Manual Installation)

**Download from GitHub:**
```
https://github.com/jrowberg/i2cdevlib
```

**Installation Steps:**
1. Download or clone the repository
2. Navigate to: `i2cdevlib-master/Arduino/`
3. Copy these folders to your Arduino libraries folder:
   - `I2Cdev/`
   - `MPU6050/`

**Arduino Libraries Folder Location:**
- Windows: `Documents/Arduino/libraries/`
- Mac: `~/Documents/Arduino/libraries/`
- Linux: `~/Arduino/libraries/`

**Final structure should be:**
```
Arduino/libraries/
├── I2Cdev/
│   ├── I2Cdev.cpp
│   ├── I2Cdev.h
│   └── ...
├── MPU6050/
│   ├── MPU6050.cpp
│   ├── MPU6050.h
│   ├── MPU6050_6Axis_MotionApps20.h
│   └── ...
├── RF24/
└── MS5611/
```

**Restart Arduino IDE after installation!**

---

## Hardware Wiring

### Flight Controller (Drone)

```
ARDUINO NANO
     ┌─────────────────────┐
     │ D13  SCK ──────────────┐
     │ D12  MISO ─────────────┼──┐
     │ D11  MOSI ─────────────┼──┼──┐
     │ D10  CSN ──────────────┼──┼──┼──────── NRF24L01 CSN
     │ D9   MOTOR_RR ─────────┼──┼──┼──┐
     │ D8   BUZZER ───────────┼──┼──┼──┼───── Buzzer +
     │ D7   LED ──────────────┼──┼──┼──┼───── LED +
     │ D6   MOTOR_RL ─────────┼──┼──┼──┼──┐
     │ D5   MOTOR_FR ─────────┼──┼──┼──┼──┼── ESC FR Signal
     │ D4   RF_CE ────────────┼──┼──┼──┼──┼── NRF24L01 CE
     │ D3   MOTOR_FL ─────────┼──┼──┼──┼──┼── ESC FL Signal
     │ D2   MPU_INT ──────────┼──┼──┼──┼──┼── MPU6050 INT
     │                        │  │  │  │  │
     │ A5   SCL ──────────────┼──┼──┼──┼──┼── MPU6050 & MS5611 SCL
     │ A4   SDA ──────────────┼──┼──┼──┼──┼── MPU6050 & MS5611 SDA
     │                        │  │  │  │  │
     │ 5V  ───────────────────┼──┼──┼──┼──┼── MPU6050 VCC, MS5611 VCC, ESCs VCC
     │ 3.3V ──────────────────┼──┼──┼──┼──┼── NRF24L01 VCC (3.3V ONLY!)
     │ GND ───────────────────┴──┴──┴──┴──┴── All GND
     └─────────────────────────────────────┘
```

### NRF24L01 Module (Both Devices)

```
NRF24L01 Pinout:
┌─────────────┐
│ GND     VCC │ ← 3.3V ONLY! (NOT 5V!)
│ CE      CSN │
│ SCK    MOSI │
│ MISO    IRQ │ ← Not used
└─────────────┘

IMPORTANT: Add 10-100µF capacitor between VCC and GND!
```

### Remote Controller

```
ARDUINO NANO
     ┌─────────────────────┐
     │ D13  SCK ──────────────── NRF24L01 SCK
     │ D12  MISO ─────────────── NRF24L01 MISO
     │ D11  MOSI ─────────────── NRF24L01 MOSI
     │ D10  CSN ──────────────── NRF24L01 CSN
     │ D9   CE ───────────────── NRF24L01 CE
     │ D7   LED ──────────────── LED + (220Ω to GND)
     │ D6   BUZZER ───────────── Buzzer + (100Ω to GND)
     │ D5   BTN_MOTOR ────────── Motor Test Button → GND
     │ D4   BTN_CALIB ────────── Calibrate Button → GND
     │ D3   SW_ALTHOLD ───────── Alt Hold Toggle → GND
     │ D2   SW_ARM ───────────── Arm Toggle → GND
     │                        
     │ A3   ROLL ─────────────── Joystick Roll
     │ A2   PITCH ────────────── Joystick Pitch
     │ A1   YAW ──────────────── Joystick Yaw
     │ A0   THROTTLE ─────────── Joystick Throttle
     │                        
     │ 5V  ───────────────────── Joysticks VCC
     │ 3.3V ──────────────────── NRF24L01 VCC
     │ GND ───────────────────── All GND
     └─────────────────────────────────────┘
```

### Motor Layout (Quad-X)

```
        FRONT
    FL (CCW)    FR (CW)
       D3          D5
         \        /
          \      /
           \    /
            \  /
             \/
             /\
            /  \
           /    \
          /      \
         /        \
       D6          D9
    RL (CW)     RR (CCW)
        REAR

Motor Rotation:
  FL = Counter-Clockwise (CCW)
  FR = Clockwise (CW)
  RL = Clockwise (CW)
  RR = Counter-Clockwise (CCW)
```

---

## Configuration

### RF Channel

**CRITICAL: Both devices MUST use the same RF channel!**

Edit this line in BOTH `.ino` files:

```cpp
#define RF_CHANNEL          108    // Use same value on both!
```

Valid range: 0-125 (use 100+ to avoid WiFi interference)

### Serial Debug

Both devices output debug info at 115200 baud:

```cpp
#define DEBUG_SERIAL        true
#define SERIAL_BAUD         115200
```

---

## How to Use

### Step 1: Upload Firmware

1. Open `quadcopter_remote/quadcopter_remote.ino` in Arduino IDE
2. Select Board: "Arduino Nano"
3. Select Port
4. Upload to REMOTE Arduino

5. Open `quadcopter_fc/quadcopter_fc.ino` in Arduino IDE
6. Upload to DRONE Arduino

### Step 2: Open Serial Monitors

Open two serial monitors (115200 baud):
- One for Remote Controller
- One for Flight Controller

### Step 3: Power On Sequence

```
1. Power ON the REMOTE first
   → Wait for startup beeps (ascending tones)
   → LED blinks fast (searching for drone)

2. Ensure ARM switch is OFF

3. Power ON the DRONE
   → Wait for startup beeps
   → Watch for "MPU6050 connected" in serial
   → Watch for "MS5611 connected" in serial
   → Watch for "Waiting for RC connection..."

4. Wait for pairing:
   → Remote: 3 beeps when connected
   → Drone: 5 beeps when paired
   → Both serial monitors show "CONNECTED"
```

### Step 4: Pre-Flight Checks

```
1. Verify in serial monitor:
   - RF: CONNECTED
   - IMU angles stable (Roll≈0, Pitch≈0)
   - ALT reading valid

2. Move joysticks and verify:
   - CMD values change in drone serial
   - Throttle: 0-1000
   - Roll/Pitch/Yaw: -45 to +45 degrees

3. Ensure throttle is at MINIMUM (0)

4. Flip ARM switch ON
   → Drone beeps (ascending tone)
   → Drone LED goes SOLID
   → Serial shows "*** ARMED ***"
```

### Step 5: Flying

```
1. Slowly increase throttle
2. Use Roll/Pitch to control position
3. Use Yaw to rotate
4. Flip Alt Hold switch to maintain altitude
```

### Step 6: Landing

```
1. Reduce throttle slowly
2. OR flip Alt Hold OFF and reduce throttle
3. When landed, flip ARM switch OFF
   → Motors stop immediately
   → Drone beeps (descending tone)
```

---

## Serial Monitor Output

### Remote Controller

```
--- RC STATUS ---
RF: CONNECTED | Sent: 1523 | ACK: 1520 | Fail: 3 | Rate: 99.8%
RAW: T=12 Y=508 P=515 R=510
OUT: T=0 Y=0 P=0 R=0
SW: ARM=OFF ALT=OFF CAL=OFF MTR=OFF
---
```

### Flight Controller

```
--- STATUS ---
State: DISARMED
RF: CONNECTED | Pkts: 1520 | Lost: 3 | Last: 15ms ago
IMU: Roll=0.5° Pitch=-0.3° Yaw=45.2°
ALT: 0.12m | Vvel: 0.01m/s
CMD: Thr=0 R=0.0 P=0.0 Y=0.0 AltHold=OFF
MTR: FL=1000 FR=1000 RL=1000 RR=1000
---
```

---

## Safety Features

### Auto-Disarm on Connection Loss

If RF connection is lost for more than 500ms:
- Motors immediately stop
- State changes to FAILSAFE
- Buzzer sounds alarm
- Must reconnect and disarm/rearm to fly again

### Arming Requirements

All conditions must be met to arm:
1. ✅ ARM switch ON (transition from OFF)
2. ✅ Throttle at minimum (< 5%)
3. ✅ RF connected
4. ✅ IMU (DMP) ready
5. ✅ Not in error state

### Disarm Conditions

Drone disarms when:
- ARM switch turned OFF
- RF connection lost > 500ms
- Any critical error detected

---

## Troubleshooting

### Problem: "NRF24L01 not found!"

**Causes:**
1. Wiring incorrect
2. Using 5V instead of 3.3V
3. No capacitor on NRF24L01 VCC

**Solutions:**
- Check all SPI connections (SCK, MISO, MOSI, CSN, CE)
- Verify 3.3V power (NOT 5V!)
- Add 10-100µF capacitor between VCC and GND
- Try a different NRF24L01 module

### Problem: "MPU6050 connection failed!"

**Causes:**
1. I2C wiring wrong
2. Wrong I2C address
3. Damaged sensor

**Solutions:**
- Check SDA (A4) and SCL (A5) connections
- Run I2C scanner to find address
- Try 0x68 or 0x69 address

### Problem: No communication (0 packets)

**Causes:**
1. Different RF channels
2. Wrong pipe address
3. Interference

**Solutions:**
- Verify `RF_CHANNEL` matches on both devices (check carefully!)
- Verify `PIPE_ADDRESS` matches ("QUAD1")
- Try different channel (100-120)
- Add capacitor to NRF24L01
- Move away from WiFi routers

### Problem: High packet loss (>5%)

**Causes:**
1. Poor power supply
2. Interference
3. Antenna issues

**Solutions:**
- Add larger capacitor (100µF) to NRF24L01
- Change RF channel
- Improve antenna orientation (vertical)
- Reduce distance for testing

### Problem: DMP initialization failed

**Causes:**
1. MPU6050 not level during calibration
2. I2C speed issue
3. Library issue

**Solutions:**
- Keep drone perfectly level and still during startup
- Check I2C at 400kHz
- Reinstall I2Cdev and MPU6050 libraries

### Problem: Motors don't spin when armed

**Causes:**
1. ESC not calibrated
2. ESC not receiving signal
3. Throttle not at zero

**Solutions:**
- Calibrate ESCs (all-high then all-low procedure)
- Check ESC signal wire connections
- Verify throttle shows 0 in serial monitor
- Check motor output values (should be 1000 when armed, low throttle)

---

## ACK Mode Explanation

This firmware uses **ACK (Acknowledgment) mode** for RF communication:

### Why ACK Mode?

| Aspect | ACK Mode | NO_ACK Mode |
|--------|----------|-------------|
| Reliability | ✅ Confirmed delivery | ❌ Best-effort |
| Latency | ~1-4ms | ~1ms |
| Debugging | ✅ Know if packets arrive | ❌ Guessing |
| Safety | ✅ Know connection state | ❌ Uncertain |

### How it works:

1. Remote sends packet
2. Drone receives and sends ACK back
3. If no ACK in 1.5ms, Remote retries (up to 3 times)
4. If all retries fail, packet marked as failed
5. Success rate tracked and displayed

### Latency Analysis:

- Packet transmission: ~0.5ms
- ACK response: ~0.5ms  
- Retry (if needed): +1.5ms each
- Worst case (3 retries): ~5ms
- Typical case: ~1ms

At 50Hz transmission (20ms period), even 5ms latency is acceptable.

---

## Sensor Roles

### MPU6050 (IMU with DMP)

**Purpose:** Stability and auto-level

**What it measures:**
- Gyroscope: Angular velocity (how fast rotating)
- Accelerometer: Linear acceleration (which way is down)

**DMP (Digital Motion Processor):**
- Onboard sensor fusion
- Outputs quaternions
- ~100Hz update rate
- Provides: Roll, Pitch, Yaw angles

**Why DMP?**
- Offloads computation from Arduino
- Built-in sensor fusion algorithm
- Lower latency than software fusion
- More stable attitude estimate

### MS5611 (Barometer)

**Purpose:** Altitude hold and smooth landing

**What it measures:**
- Air pressure (mbar)
- Converted to altitude using barometric formula

**Features:**
- ~40Hz update rate
- Resolution: ~10cm
- Complementary filter for smooth readings

**Enables:**
- Altitude hold mode
- Smooth vertical control
- Controlled descent for landing

---

## PID Tuning

### Default Values

```cpp
// Roll/Pitch (angle mode)
pidRoll.kp = 4.0f;
pidRoll.ki = 0.02f;
pidRoll.kd = 1.5f;

// Yaw (rate mode)
pidYaw.kp = 3.0f;
pidYaw.ki = 0.01f;
pidYaw.kd = 0.0f;

// Altitude hold
pidAlt.kp = 50.0f;
pidAlt.ki = 0.5f;
pidAlt.kd = 30.0f;
```

### Tuning Process

1. Start with Kp only (Ki=0, Kd=0)
2. Increase Kp until oscillation
3. Reduce Kp by 30%
4. Add Kd to dampen oscillation
5. Add small Ki to eliminate drift

### Symptoms and Fixes

| Symptom | Fix |
|---------|-----|
| Oscillation | Reduce Kp or increase Kd |
| Sluggish response | Increase Kp |
| Drifts over time | Increase Ki |
| Overshoots | Increase Kd or reduce Kp |
| Noisy motors | Reduce Kd |

---

## Version History

- **v2.0.0**: Complete rewrite
  - Added MPU6050 DMP support
  - Added MS5611 barometer
  - Changed to ACK mode
  - Added serial debugging
  - Added auto-disarm failsafe
  - Added altitude hold

- **v1.0.0**: Initial release

---

## Safety Warning

⚠️ **IMPORTANT: Quadcopters are dangerous!**

- Always remove propellers when testing
- Test in open areas away from people
- Keep a safe distance during flight
- Have a spotter when flying
- Never fly over people
- Check all connections before each flight
- Ensure batteries are secure
- Know your local drone regulations

---

## License

Educational use. Fly at your own risk.
