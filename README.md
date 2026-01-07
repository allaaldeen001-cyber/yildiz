# Arduino Nano Quadcopter Flight Controller v3.0

A complete, production-ready flight controller system for Arduino Nano quadcopters.

## Features

| Feature | Description |
|---------|-------------|
| **MPU6050 + DMP** | Hardware sensor fusion for stable attitude |
| **MS5611 Barometer** | Altitude hold capability |
| **NRF24L01 + ACK** | Reliable bidirectional communication |
| **Auto-Level** | Self-leveling angle mode |
| **Failsafe** | Auto-disarm on signal loss |
| **PID Control** | Tunable Roll/Pitch/Yaw/Altitude |
| **Motor Smoothing** | Rate-limited output for stability |

---

## Required Libraries

### Flight Controller

| Library | Author | Install |
|---------|--------|---------|
| I2Cdev | Jeff Rowberg | [Manual](https://github.com/jrowberg/i2cdevlib) |
| MPU6050 | Jeff Rowberg | [Manual](https://github.com/jrowberg/i2cdevlib) |
| RF24 | TMRh20 | Arduino Library Manager |
| MS5611 | Rob Tillaart | Arduino Library Manager |

### Remote Controller

| Library | Author | Install |
|---------|--------|---------|
| RF24 | TMRh20 | Arduino Library Manager |

---

## Hardware Wiring

### Flight Controller

```
Arduino Nano          Peripherals
────────────          ───────────
D2  ──────────────── MPU6050 INT (optional)
D3  ──────────────── ESC Front-Left (CCW)
D4  ──────────────── NRF24 CE
D5  ──────────────── ESC Front-Right (CW)
D6  ──────────────── ESC Rear-Left (CW)
D7  ──────────────── LED (+)
D8  ──────────────── Buzzer (+)
D9  ──────────────── ESC Rear-Right (CCW)
D10 ──────────────── NRF24 CSN
D11 ──────────────── NRF24 MOSI
D12 ──────────────── NRF24 MISO
D13 ──────────────── NRF24 SCK
A4  ──────────────── MPU6050 SDA + MS5611 SDA
A5  ──────────────── MPU6050 SCL + MS5611 SCL
5V  ──────────────── MPU6050 VCC + MS5611 VCC
3.3V ─────────────── NRF24 VCC (⚠️ NOT 5V!)
GND ──────────────── All GND
```

### Remote Controller

```
Arduino Nano          Peripherals
────────────          ───────────
D2  ──────────────── ARM Switch (to GND)
D3  ──────────────── Alt Hold Switch (to GND)
D4  ──────────────── Calibrate Button (to GND)
D5  ──────────────── Motor Test Button (to GND)
D6  ──────────────── Buzzer (+)
D7  ──────────────── LED (+)
D9  ──────────────── NRF24 CE
D10 ──────────────── NRF24 CSN
D11 ──────────────── NRF24 MOSI
D12 ──────────────── NRF24 MISO
D13 ──────────────── NRF24 SCK
A0  ──────────────── Throttle Joystick
A1  ──────────────── Yaw Joystick
A2  ──────────────── Pitch Joystick
A3  ──────────────── Roll Joystick
5V  ──────────────── Joysticks VCC
3.3V ─────────────── NRF24 VCC (⚠️ NOT 5V!)
GND ──────────────── All GND
```

### Motor Layout (Quad-X)

```
        FRONT
   FL (CCW)   FR (CW)
      D3        D5
        \      /
         \    /
          \  /
           \/
           /\
          /  \
         /    \
        /      \
      D6        D9
   RL (CW)   RR (CCW)
        REAR
```

---

## Setup Instructions

### Step 1: Install Libraries

**I2Cdev + MPU6050 (Manual):**
1. Download from: https://github.com/jrowberg/i2cdevlib
2. Copy `Arduino/I2Cdev` folder to `Documents/Arduino/libraries/`
3. Copy `Arduino/MPU6050` folder to `Documents/Arduino/libraries/`

**RF24 + MS5611 (Library Manager):**
1. Arduino IDE → Sketch → Include Library → Manage Libraries
2. Search "RF24" → Install "RF24 by TMRh20"
3. Search "MS5611" → Install "MS5611 by Rob Tillaart"

### Step 2: Calibrate MPU6050

1. Open Arduino IDE → File → Examples → MPU6050 → IMU_Zero
2. Upload to flight controller Arduino
3. Open Serial Monitor (115200 baud)
4. Keep sensor **perfectly level and still**
5. Wait for calibration to complete (~1 minute)
6. Copy the offset values

### Step 3: Configure Flight Controller

Edit `quadcopter_fc.ino`:

```cpp
// Set YOUR calibration values (from Step 2)
#define ACCEL_OFFSET_X      -2366   // Your XAccel
#define ACCEL_OFFSET_Y      755     // Your YAccel
#define ACCEL_OFFSET_Z      -2006   // Your ZAccel
#define GYRO_OFFSET_X       10      // Your XGyro
#define GYRO_OFFSET_Y       21      // Your YGyro
#define GYRO_OFFSET_Z       -19     // Your ZGyro

// Set RF channel (must match remote!)
#define RF_CHANNEL          108
```

### Step 4: Configure Remote Controller

Edit `quadcopter_remote.ino`:

```cpp
// Set same RF channel as flight controller!
#define RF_CHANNEL          108
```

### Step 5: Upload Code

1. Upload `quadcopter_remote.ino` to remote Arduino
2. Upload `quadcopter_fc.ino` to flight controller Arduino

---

## Usage

### Power On Sequence

1. **Power ON remote first** (wait for startup beeps)
2. Ensure **ARM switch is OFF**
3. **Power ON drone** (wait for initialization)
4. Wait for **5 beeps** (pairing successful)

### Arming

Requirements:
- ✅ ARM switch OFF → ON (transition)
- ✅ Throttle at minimum (<5%)
- ✅ RF connected
- ✅ DMP ready

Steps:
1. Move throttle to **minimum**
2. Flip ARM switch **ON**
3. Listen for **armed beep**
4. LED turns **solid**

### Flying

| Control | Action |
|---------|--------|
| Throttle Up | Climb |
| Throttle Down | Descend |
| Roll Left/Right | Tilt left/right |
| Pitch Forward/Back | Move forward/back |
| Yaw Left/Right | Rotate |

### Altitude Hold

1. Flip **Alt Hold switch ON**
2. Set throttle to ~50% (hover)
3. Drone maintains altitude
4. Small throttle changes adjust target altitude

### Disarming

- Flip ARM switch **OFF**
- OR: Signal loss >500ms triggers **failsafe**

---

## Serial Monitor Output

### Flight Controller

```
=== QuadFC v3.0 ===
RF Channel: 108
MPU6050...calibrating...OK
MS5611...OK (1013.2 mbar)
NRF24L01...OK
ESCs...OK

*** READY ***
Waiting for RC connection...

ARM  RF:1523 R:0.5 P:-0.3 PID:3,-2 Alt:0.12* M:1485,1492,1488,1495
ARM  RF:1530 R:1.2 P:0.8 PID:-7,5 Alt:0.15* M:1490,1480,1475,1495
```

### Remote Controller

```
=== QuadRC v3.0 ===
RF Channel: 108
NRF24L01...OK
Calibrating joystick centers...
Centers: Y=512 P=508 R=515

*** READY ***
Searching for drone...

CONN TX:1523 OK:1520 FAIL:3 (99.8%) | T:450 Y:0 P:12 R:-5 | ARM:ON ALT:ON
```

---

## PID Tuning

### Default Values

```cpp
// Roll/Pitch
Kp = 6.0    Ki = 0.03   Kd = 2.5

// Yaw
Kp = 4.0    Ki = 0.02   Kd = 0.0

// Altitude
Kp = 15.0   Ki = 0.1    Kd = 8.0
```

### Tuning Guide

| Problem | Fix |
|---------|-----|
| Fast oscillation | Reduce Kp or Kd |
| Slow wobble | Increase Kd or reduce Kp |
| Sluggish response | Increase Kp |
| Drifting | Increase Ki |
| Overshoots | Increase Kd |
| Motor noise/vibration | Reduce Kd |

### Tuning Process

1. Set Ki=0, Kd=0
2. Increase Kp until slight oscillation
3. Reduce Kp by 20%
4. Increase Kd until oscillation stops
5. Add small Ki for drift correction

---

## Safety Features

| Feature | Trigger | Action |
|---------|---------|--------|
| Arm Safety | Throttle >5% | Cannot arm |
| Auto-Disarm | Signal loss >500ms | Motors off |
| Failsafe | RF timeout | State → FAILSAFE |
| Ground Lock | Alt <0.3m | Altitude hold disabled |

---

## LED Indicators

### Flight Controller

| State | LED Pattern |
|-------|-------------|
| Disarmed | Slow blink (500ms) |
| Armed | Solid ON |
| Failsafe | Fast blink (100ms) |

### Remote Controller

| State | LED Pattern |
|-------|-------------|
| Searching | Fast blink (100ms) |
| Connected | Slow blink (500ms) |
| Armed | Solid ON |

---

## Buzzer Sounds

| Event | Sound |
|-------|-------|
| Startup | Rising 4-tone |
| RF Paired | 5 quick beeps |
| Armed | 2-tone rising |
| Disarmed | 1-tone falling |
| Failsafe | Long low tone |
| Button Press | Short beep |

---

## Troubleshooting

### "MPU6050 FAIL"

- Check I2C wiring (SDA→A4, SCL→A5)
- Verify 5V power to MPU6050
- Run I2C scanner to find address

### "NRF24L01 FAIL"

- Check SPI wiring
- Verify **3.3V** power (NOT 5V!)
- Add 10-100µF capacitor on NRF24 VCC

### No Communication

- Verify **same RF channel** on both devices
- Check NRF24 antenna orientation
- Reduce distance for testing

### Drone Flips on Takeoff

- Check motor rotation directions
- Verify motor pin assignments
- Check prop orientation (CW vs CCW)

### Oscillation During Flight

- Reduce PID Kp value
- Increase PID Kd value
- Check for loose parts/vibration

---

## Flash Size

If "Sketch too big" error:

```cpp
#define ENABLE_DEBUG    0   // Disable debug to save ~2KB
```

---

## Version History

- **v3.0** - Full release with DMP + MS5611 + ACK mode
- **v2.0** - Added altitude hold and filtering
- **v1.0** - Initial release

---

## License

MIT License - Use at your own risk.

⚠️ **Always remove propellers when testing!**
