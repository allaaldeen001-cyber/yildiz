# 🔧 TROUBLESHOOTING GUIDE

## Complete Problem-Solution Reference

This document covers common issues and their solutions for the quadcopter drone project.

---

## 🚫 COMMON ISSUES INDEX

### Communication Issues
1. [NRF24L01 Not Connecting](#nrf24l01-not-connecting)
2. [Signal Drops/Intermittent Connection](#signal-drops)
3. [High Latency](#high-latency)

### Sensor Issues
4. [MPU6050 Not Detected](#mpu6050-not-detected)
5. [MS5611 Not Detected](#ms5611-not-detected)
6. [Incorrect IMU Readings](#incorrect-imu-readings)
7. [Altitude Drift](#altitude-drift)

### Motor Issues
8. [Motors Not Spinning](#motors-not-spinning)
9. [One Motor Not Working](#one-motor-not-working)
10. [Motors Spin at Different Speeds](#motors-uneven-speed)
11. [ESC Beeping Continuously](#esc-continuous-beeping)

### Flight Issues
12. [Drone Flips on Takeoff](#drone-flips-on-takeoff)
13. [Drone Oscillates/Vibrates](#drone-oscillates)
14. [Drone Drifts](#drone-drifts)
15. [Poor Throttle Response](#poor-throttle-response)
16. [Uncontrollable Yaw](#uncontrollable-yaw)

### Control Issues
17. [Joystick Not Responding](#joystick-not-responding)
18. [Wrong Control Mapping](#wrong-control-mapping)
19. [Kill Switch Not Working](#kill-switch-not-working)
20. [Calibration Fails](#calibration-fails)

### Power Issues
21. [Battery Drains Too Fast](#battery-drains-fast)
22. [Brown-Out Resets](#brownout-resets)
23. [NRF Keeps Resetting](#nrf-keeps-resetting)

---

## 📡 COMMUNICATION ISSUES

### <a name="nrf24l01-not-connecting"></a>1. NRF24L01 Not Connecting

#### Symptoms
- No connection beeps on FC
- Serial shows "NRF24L01 init failed"
- LED blinks fast on RC
- No data transmission

#### Diagnosis Steps
1. Check serial monitor on both Arduino:
   ```
   FC: [ERROR] NRF24L01 init failed!
   RC: [OK] NRF24L01 initialized (but no connection)
   ```

2. Measure NRF voltage:
   ```
   VCC pin should read 3.2V - 3.4V
   NOT 5V! (will damage module)
   ```

3. Check SPI pins with multimeter (continuity test)

#### Solutions

**Solution 1: Power Supply Issue**
```
Problem: NRF24L01 underpowered
Fix: Add 10µF capacitor across VCC and GND
     Place capacitor as close to NRF module as possible
     
Wiring:
  NRF VCC ──┬── Arduino 3.3V
            │
          ─┴─  10µF capacitor
          ─┬─
            │
  NRF GND ──┴── Arduino GND
```

**Solution 2: Wrong Voltage**
```
Check: NRF VCC must be 3.3V (NOT 5V!)
Fix:   Connect to Arduino 3.3V pin
       If using external regulator, verify output voltage
```

**Solution 3: Wiring Errors**
```
Verify connections:
  NRF24     Arduino Nano
  ────────────────────
  VCC   →   3.3V (NOT 5V!)
  GND   →   GND
  CE    →   D9
  CSN   →   D10
  SCK   →   D13
  MOSI  →   D11
  MISO  →   D12
```

**Solution 4: Faulty NRF Module**
```
Test: Swap with known working NRF24L01
      Many cheap modules are defective
      Try NRF24L01+PA+LNA for better reliability
```

**Solution 5: Software Issue**
```cpp
// Try different channel:
radio.setChannel(108);  // Default
radio.setChannel(76);   // Alternative

// Try different power level:
radio.setPALevel(RF24_PA_MAX);
radio.setPALevel(RF24_PA_HIGH);  // Try this

// Try different data rate:
radio.setDataRate(RF24_250KBPS);  // Default
radio.setDataRate(RF24_1MBPS);    // Faster
```

---

### <a name="signal-drops"></a>2. Signal Drops/Intermittent Connection

#### Symptoms
- Connection established but drops frequently
- Failsafe activates randomly
- "Signal lost" messages in serial

#### Solutions

**Solution 1: Improve Antenna**
```
Problem: Weak signal
Fix: 
  - Extend NRF antenna (add wire if using PCB antenna)
  - Keep antennas vertical
  - Don't cover antennas with carbon fiber
```

**Solution 2: Reduce Interference**
```
Sources of interference:
  - Motors (EMI)
  - ESCs (switching noise)
  - Power wires
  
Fix:
  - Add ferrite beads to motor wires
  - Twist ESC wires together
  - Keep NRF away from power wires (5cm minimum)
  - Use shielded cable for NRF if needed
```

**Solution 3: Increase Transmission Rate**
```cpp
// In RC transmitter, reduce transmit interval:
#define TRANSMIT_RATE  25  // Faster (40Hz instead of 20Hz)
```

---

### <a name="high-latency"></a>3. High Latency

#### Symptoms
- Delayed response to stick inputs
- Drone reacts 0.5-1 second late

#### Solutions

**Solution 1: Optimize Radio Settings**
```cpp
radio.setRetries(1, 3);  // Reduce retries (faster, less reliable)
radio.setDataRate(RF24_1MBPS);  // Faster data rate
```

**Solution 2: Reduce Serial Printing**
```cpp
// Print less frequently in flight controller:
if (millis() - lastTelemetryTime > 200) {  // 5Hz instead of 10Hz
  printTelemetry();
}
```

---

## 🎛️ SENSOR ISSUES

### <a name="mpu6050-not-detected"></a>4. MPU6050 Not Detected

#### Symptoms
- Serial shows "[ERROR] MPU6050 init failed!"
- Drone won't calibrate
- No IMU data

#### Diagnosis
```cpp
// Upload I2C scanner sketch:
// File → Examples → Wire → i2c_scanner
// Should detect device at 0x68 or 0x69
```

#### Solutions

**Solution 1: Wrong I2C Address**
```cpp
// Some MPU6050 modules use 0x69 instead of 0x68
#define MPU6050_ADDR  0x69  // Try alternative address
```

**Solution 2: I2C Pull-up Resistors**
```
Problem: Missing pull-up resistors on SDA/SCL
Fix: Add 4.7kΩ resistors:
     SDA ──[4.7kΩ]── 5V
     SCL ──[4.7kΩ]── 5V
```

**Solution 3: Wiring Check**
```
Verify:
  MPU6050 VCC → Arduino 5V (or 3.3V, check module specs)
  MPU6050 GND → Arduino GND
  MPU6050 SDA → Arduino A4
  MPU6050 SCL → Arduino A5
  
If module has AD0 pin:
  AD0 → GND (for address 0x68)
  AD0 → 5V (for address 0x69)
```

---

### <a name="ms5611-not-detected"></a>5. MS5611 Not Detected

#### Symptoms
- Serial shows "[ERROR] MS5611 init failed!"
- No altitude data
- I2C scanner doesn't find it

#### Solutions

**Solution 1: Check I2C Address**
```cpp
// MS5611 can be 0x76 or 0x77
#define MS5611_ADDR  0x76  // Try alternative

// Or use I2C scanner to detect actual address
```

**Solution 2: Module Compatibility**
```
Some MS5611 modules are actually GY-63 (MS5611 + other sensors)
These may have different addresses or initialization sequences
Check module datasheet
```

---

### <a name="incorrect-imu-readings"></a>6. Incorrect IMU Readings

#### Symptoms
- Roll/pitch angles always wrong
- Drone thinks it's tilted when level
- Angles drift over time

#### Solutions

**Solution 1: Recalibrate**
```
1. Place drone on perfectly level surface
2. Don't move drone during calibration
3. Press Button 1 for full calibration
4. Wait for completion beeps
```

**Solution 2: Check IMU Orientation**
```
MPU6050 must be mounted:
  - Flat (parallel to drone frame)
  - X-axis pointing forward
  - Y-axis pointing right
  - Z-axis pointing up
  
If mounted differently, update code:
// Swap axes as needed
float accelForward = accelX;  // Or Y, or -X depending on mounting
```

**Solution 3: Complementary Filter Tuning**
```cpp
// Adjust filter ratio in calculateAttitude():
roll = 0.98f * roll + 0.02f * accelRoll;  // Default
roll = 0.95f * roll + 0.05f * accelRoll;  // More accel influence
roll = 0.99f * roll + 0.01f * accelRoll;  // More gyro influence
```

---

### <a name="altitude-drift"></a>7. Altitude Drift

#### Symptoms
- Altitude reading increases/decreases while stationary
- Inconsistent altitude hold
- Altitude jumps randomly

#### Solutions

**Solution 1: Recalibrate Barometer**
```
MS5611 is temperature sensitive
Recalibrate:
  - After moving to new location
  - When temperature changes significantly
  - Before each flight session
```

**Solution 2: Protect from Prop Wash**
```
Barometer must be isolated from:
  - Propeller downwash
  - Airflow from motors
  - Vibration
  
Fix:
  - Cover sensor with foam
  - Seal in case with single pressure port
  - Use vibration dampening
```

**Solution 3: Lower Altitude PID Gains**
```cpp
#define PID_ALT_KP  1.0f  // Reduce from 2.0
#define PID_ALT_KI  0.05f  // Reduce from 0.1
```

---

## ⚙️ MOTOR ISSUES

### <a name="motors-not-spinning"></a>8. Motors Not Spinning

#### Symptoms
- Drone armed but no motors spin
- Motor test does nothing
- No beeps from ESCs

#### Diagnosis
1. Check serial monitor for motor values (should be >1000)
2. Listen for ESC initialization beeps on power-up
3. Test motor manually (disconnect ESC, connect to receiver)

#### Solutions

**Solution 1: ESC Not Armed**
```
ESCs need throttle signal to arm:
  1. Connect battery
  2. Wait for ESC beep sequence
  3. If continuous beeping, ESCs not calibrated
  4. Run ESC calibration (Button 1)
```

**Solution 2: Wrong PWM Range**
```cpp
// Some ESCs use different ranges
motor1.attach(MOTOR1_PIN, 1000, 2000);  // Default
motor1.attach(MOTOR1_PIN, 900, 2100);   // Try wider range
```

**Solution 3: Wiring**
```
Check:
  - ESC signal wire connected to correct pin (D3-D6)
  - ESC ground connected to Arduino ground
  - ESC powered from battery (NOT Arduino!)
  - Motor wires connected to ESC
```

**Solution 4: Throttle Too Low**
```cpp
// Ensure minimum throttle for armed state:
#define MOTOR_IDLE  1100  // Increase if motors don't start
#define MOTOR_IDLE  1150  // Try higher value
```

---

### <a name="one-motor-not-working"></a>9. One Motor Not Working

#### Symptoms
- 3 motors spin, 1 doesn't
- Or one motor spins wrong speed

#### Solutions

**Solution 1: Check Motor/ESC**
```
Test sequence:
  1. Swap motor with working motor
     - If problem follows motor: bad motor
     - If problem stays: bad ESC or wiring
  
  2. Swap ESC with working ESC
     - If problem follows ESC: bad ESC
     - If problem stays: bad wiring or code
```

**Solution 2: Pin Problem**
```
Test: Upload simple servo test sketch to verify pin works
Connect working ESC to problem pin
If works: original ESC bad
If doesn't work: Arduino pin damaged
```

---

### <a name="motors-uneven-speed"></a>10. Motors Spin at Different Speeds

#### Symptoms
- One motor spins faster/slower than others
- Drone tilts even with sticks centered
- Unbalanced thrust

#### Solutions

**Solution 1: ESC Calibration**
```
All ESCs must be calibrated together:
  1. Connect all ESCs
  2. Run calibration (Button 1)
  3. All ESCs should beep in sync
```

**Solution 2: Check Motor Specs**
```
All 4 motors must be:
  - Same model
  - Same KV rating
  - Same condition (not damaged)
  - Same propeller type/size
```

**Solution 3: Check Throttle Curve**
```cpp
// Verify motors get equal throttle when armed:
Serial.print("M1:"); Serial.print(motor1Speed);
Serial.print(" M2:"); Serial.print(motor2Speed);
Serial.print(" M3:"); Serial.print(motor3Speed);
Serial.print(" M4:"); Serial.println(motor4Speed);

// Should all be same when hovering with no stick input
```

---

### <a name="esc-continuous-beeping"></a>11. ESC Beeping Continuously

#### Symptoms
- ESC beeps repeatedly on power-up
- Never stops beeping
- Motor won't spin

#### Meaning
```
ESC beep codes:
  1 beep  : Low voltage warning
  2 beeps : Throttle signal error
  3 beeps : Motor connection error
  Continuous: No signal or wrong signal range
```

#### Solutions

**Solution 1: No Signal**
```
Check:
  - Signal wire connected
  - Arduino powered on
  - Correct pin in code matches wiring
```

**Solution 2: Wrong Signal Range**
```
ESC expects 1000-2000µs, check:
  - Arduino sends minimum 1000µs when disarmed
  - Servo library initialized correctly
  - PWM frequency correct (50Hz)
```

**Solution 3: ESC Configuration**
```
Some ESCs need configuration:
  - Brake on/off
  - Battery type (LiPo 3S)
  - Cut-off voltage
  
Check ESC manual for programming procedure
```

---

## 🚁 FLIGHT ISSUES

### <a name="drone-flips-on-takeoff"></a>12. Drone Flips on Takeoff

#### Symptoms
- Drone immediately flips over when throttle applied
- Cannot take off
- One side rises, other drops

#### This is a CRITICAL issue - DO NOT FLY until fixed!

#### Solutions

**Solution 1: Wrong Motor Direction**
```
Check motor spin directions:
  M1 (FR): Clockwise ↻
  M2 (RR): Counter-Clockwise ↺
  M3 (RL): Clockwise ↻
  M4 (FL): Counter-Clockwise ↺

Fix wrong direction:
  - Swap any 2 motor wires (not all 3!)
  Example: Swap blue and yellow wires
```

**Solution 2: Wrong Propeller Placement**
```
Check propeller types:
  M1 (FR): CW propeller (↻ label or no label)
  M2 (RR): CCW propeller (↺ label or "R" or "P")
  M3 (RL): CW propeller
  M4 (FL): CCW propeller

Each propeller has:
  - Leading edge (thicker)
  - Trailing edge (thinner)
  Leading edge must be at front of rotation
```

**Solution 3: Inverted Motor Mixing**
```cpp
// Verify motor mixing formula:
motor1Speed = throttle + pidPitch.output - pidRoll.output - pidYaw.output;
motor2Speed = throttle - pidPitch.output - pidRoll.output + pidYaw.output;
motor3Speed = throttle - pidPitch.output + pidRoll.output - pidYaw.output;
motor4Speed = throttle + pidPitch.output + pidRoll.output + pidYaw.output;

// If still flips, try inverting signs:
// Example: Change + to - or vice versa
```

**Solution 4: Wrong Motor Position**
```
Verify physical motor layout matches code:
      FRONT
   M4     M1
     \   /
      \ /
      / \
     /   \
   M3     M2

If motors in wrong positions:
  - Swap ESC connections
  OR
  - Update motor pin assignments in code
```

---

### <a name="drone-oscillates"></a>13. Drone Oscillates/Vibrates

#### Symptoms
- Drone shakes or vibrates in hover
- Fast oscillations (buzzing)
- Slow oscillations (bouncing)

#### Solutions

**Solution 1: PID Tuning (Fast Oscillations)**
```cpp
// P gain too high or D gain too low
// Reduce P by 20%:
#define PID_ROLL_KP   1.2f  // Was 1.5
#define PID_PITCH_KP  1.2f  // Was 1.5

// Or increase D by 20%:
#define PID_ROLL_KD   21.6f  // Was 18.0
#define PID_PITCH_KD  21.6f  // Was 18.0
```

**Solution 2: Mechanical Issues**
```
Check:
  - Propellers balanced
  - Motor bearings smooth
  - Frame rigid (no flex)
  - All screws tight
  - No cracks in frame/arms
```

**Solution 3: Slow Oscillations**
```cpp
// I gain too high
#define PID_ROLL_KI   0.025f  // Reduce from 0.05
#define PID_PITCH_KI  0.025f
```

See [PID_TUNING_GUIDE.md](PID_TUNING_GUIDE.md) for detailed tuning.

---

### <a name="drone-drifts"></a>14. Drone Drifts

#### Symptoms
- Drone slowly moves in one direction
- Cannot hold position
- Constant correction needed

#### Solutions

**Solution 1: IMU Calibration**
```
Drift usually means IMU not level:
  1. Place drone on PERFECTLY level surface
  2. Use spirit level to verify
  3. Run calibration (Button 1)
  4. Don't move drone during calibration
```

**Solution 2: Increase I Gain**
```cpp
// I gain corrects persistent error:
#define PID_ROLL_KI   0.08f  // Increase from 0.05
#define PID_PITCH_KI  0.08f
```

**Solution 3: Center of Gravity**
```
Check weight distribution:
  - Battery centered
  - Components balanced
  - CG should be in center of frame
  
Test: Hang drone from center, should be level
```

---

### <a name="poor-throttle-response"></a>15. Poor Throttle Response

#### Symptoms
- Throttle stick doesn't affect altitude much
- Needs lots of throttle to hover
- Very sensitive at high throttle

#### Solutions

**Solution 1: Check Throttle Range**
```cpp
// Verify throttle mapping:
Serial.println(receivedData.throttle);
// Should be 1000 at bottom, 2000 at top
// Center around 1400-1600 for hover
```

**Solution 2: Motor Power**
```
Check:
  - Battery voltage >11.1V
  - Motors appropriate for frame size
  - Propellers correct size
  - Total weight not too heavy
```

**Solution 3: ESC Throttle Curve**
```
Some ESCs have throttle curves:
  - Linear (best for manual control)
  - Exponential (more bottom-end power)
  
Configure ESC for linear throttle
```

---

### <a name="uncontrollable-yaw"></a>16. Uncontrollable Yaw

#### Symptoms
- Drone spins uncontrollably
- Cannot stop rotation
- Yaw stick doesn't work properly

#### Solutions

**Solution 1: Motor Direction**
```
Verify CW/CCW pattern correct:
  Opposite corners must spin same direction
  Adjacent motors must spin opposite directions
```

**Solution 2: Yaw PID Too High**
```cpp
#define PID_YAW_KP  1.5f  // Reduce from 3.0
#define PID_YAW_KI  0.01f  // Reduce from 0.02
```

**Solution 3: Propeller Damage**
```
Even small chips in propellers cause yaw issues
Replace all propellers with new matched set
```

---

## 🎮 CONTROL ISSUES

### <a name="joystick-not-responding"></a>17. Joystick Not Responding

#### Symptoms
- Moving stick doesn't change values
- Stuck at one value
- Random values

#### Solutions

**Solution 1: Wiring Check**
```
Verify joystick connections:
  Joystick VCC → 5V
  Joystick GND → GND
  VRx → Correct analog pin (A0-A3)
  VRy → Correct analog pin (A0-A3)
```

**Solution 2: Recalibrate**
```
Run joystick calibration:
  1. Power on RC
  2. On first boot, auto-calibration runs
  3. To force recalibrate: Clear EEPROM first
  
To clear EEPROM:
  EEPROM.write(EEPROM_CALIBRATED, 0);
```

**Solution 3: Bad Joystick**
```
Test: Measure voltage on VRx/VRy pins
  Should vary from 0V to 5V as stick moves
  If stuck at 0V or 5V: joystick broken
```

---

### <a name="wrong-control-mapping"></a>18. Wrong Control Mapping

#### Symptoms
- Roll stick affects pitch
- Throttle inverted
- Yaw backwards

#### Solutions

**Solution 1: Swap Pins**
```cpp
// If axes are swapped, change pin assignments:
#define JOYSTICK1_X_PIN   A0    // Throttle
#define JOYSTICK1_Y_PIN   A1    // Yaw
#define JOYSTICK2_X_PIN   A2    // Pitch
#define JOYSTICK2_Y_PIN   A3    // Roll

// Or swap in code:
throttleRaw = analogRead(JOYSTICK1_Y_PIN);  // Swap X and Y
yawRaw = analogRead(JOYSTICK1_X_PIN);
```

**Solution 2: Invert Axis**
```cpp
// If axis is backwards:
txData.pitch = map(pitchRaw, 0, 1023, CONTROL_MAX, CONTROL_MIN);
//                                    ↑ Swapped min and max ↑
```

---

### <a name="kill-switch-not-working"></a>19. Kill Switch Not Working

#### Symptoms
- Cannot disarm drone
- Toggle has no effect
- Always armed or always disarmed

#### This is CRITICAL SAFETY ISSUE!

#### Solutions

**Solution 1: Test Switch**
```cpp
// Add debug code:
void loop() {
  bool switchState = digitalRead(TOGGLE_SWITCH_PIN);
  Serial.println(switchState);  // Should toggle between 0 and 1
}
```

**Solution 2: Pullup Resistor**
```cpp
// Ensure pullup enabled:
pinMode(TOGGLE_SWITCH_PIN, INPUT_PULLUP);
```

**Solution 3: Inverted Logic**
```cpp
// Try inverting:
bool armSwitch = digitalRead(TOGGLE_SWITCH_PIN);  // Original
bool armSwitch = !digitalRead(TOGGLE_SWITCH_PIN);  // Inverted
```

**DO NOT FLY** until kill switch is proven working!

---

### <a name="calibration-fails"></a>20. Calibration Fails

#### Symptoms
- Calibration never completes
- Error beeps during calibration
- Calibration doesn't save

#### Solutions

**Solution 1: Keep Drone Still**
```
IMU calibration requires:
  - Perfectly level surface
  - No movement
  - No vibration
  - No touching drone
  
Wait for completion beeps before moving
```

**Solution 2: EEPROM Issue**
```cpp
// Test EEPROM:
EEPROM.write(0, 123);
byte test = EEPROM.read(0);
if (test != 123) {
  Serial.println("EEPROM not working!");
}
```

**Solution 3: Sensor Timeout**
```
If sensors not responding:
  - Check I2C connections
  - Verify sensor power
  - Run I2C scanner
```

---

## ⚡ POWER ISSUES

### <a name="battery-drains-fast"></a>21. Battery Drains Too Fast

#### Symptoms
- Flight time <5 minutes
- Battery hot after flight
- Voltage drops quickly

#### Solutions

**Solution 1: Check Current Draw**
```
Measure current with multimeter:
  Hovering: 10-20A typical
  >30A: Problem (too much power draw)
  
High current causes:
  - Motors too high KV for propeller size
  - Propellers too large
  - Damaged motors (friction)
  - ESC problems
```

**Solution 2: Battery Health**
```
Check battery:
  - Voltage per cell: 3.7V nominal, 4.2V full
  - Capacity remaining: Should be >80% of rated
  - Internal resistance: Should be low
  - Physical condition: No swelling
  
Old or damaged batteries drain fast
```

**Solution 3: Reduce Power**
```
  - Use smaller/lighter propellers
  - Reduce flight weight
  - Fly less aggressively
  - Lower throttle limit
```

---

### <a name="brownout-resets"></a>22. Brown-Out Resets

#### Symptoms
- Arduino resets randomly
- Flight controller reboots mid-flight
- Loses connection then reconnects

#### Solutions

**Solution 1: Separate Power**
```
Don't power Arduino from ESC BEC when also powering servos/others:
  - Use separate 5V regulator for Arduino
  - Add 470µF capacitor on Arduino VIN
  - Use LC filter (inductor + capacitor)
```

**Solution 2: Better BEC**
```
ESC BECs are often weak (1-2A)
Use external BEC:
  - 3A+ rated
  - Switching regulator (more efficient)
  - Connect directly to battery
```

---

### <a name="nrf-keeps-resetting"></a>23. NRF Keeps Resetting

#### Symptoms
- Connection drops when motors spin
- Works on bench, fails in flight
- NRF stops responding

#### Solutions

**Solution 1: Power Supply**
```
NRF needs clean, stable 3.3V:
  - Add 10µF capacitor at NRF (already mentioned)
  - Add 100µF capacitor at Arduino 3.3V pin
  - Use LC filter between Arduino and NRF
```

**Solution 2: EM Interference**
```
Motors create huge EMI:
  - Move NRF away from motors (10cm+)
  - Shield NRF with aluminum foil (ground foil)
  - Add ferrite beads to motor wires
  - Twist motor wires together
```

**Solution 3: Better Module**
```
Original NRF24L01 is sensitive
Upgrade to:
  - NRF24L01+ (plus version, more robust)
  - NRF24L01+PA+LNA (with power amp, more range)
```

---

## 🔍 DIAGNOSIS TOOLS

### I2C Scanner
```cpp
// Upload this to find I2C devices:
#include <Wire.h>
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Scanning I2C bus...");
  for(byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if(Wire.endTransmission() == 0) {
      Serial.print("Device at 0x");
      Serial.println(i, HEX);
    }
  }
}
void loop() {}
```

### Servo Test
```cpp
// Test motor pins:
#include <Servo.h>
Servo testMotor;
void setup() {
  testMotor.attach(3);  // Change pin to test
}
void loop() {
  testMotor.writeMicroseconds(1000);  // Stopped
  delay(2000);
  testMotor.writeMicroseconds(1200);  // Slow
  delay(2000);
}
```

### Joystick Test
```cpp
// Test joystick analog pins:
void setup() {
  Serial.begin(115200);
}
void loop() {
  Serial.print("A0: "); Serial.print(analogRead(A0));
  Serial.print(" A1: "); Serial.print(analogRead(A1));
  Serial.print(" A2: "); Serial.print(analogRead(A2));
  Serial.print(" A3: "); Serial.println(analogRead(A3));
  delay(100);
}
```

---

## 📞 Getting Help

### Before Asking for Help, Collect:
1. Exact error messages from serial monitor
2. Photos of wiring
3. Video of problem behavior
4. What you've already tried
5. Component specifications (motor KV, frame size, etc.)

### Where to Ask:
- Arduino Forums
- RC Groups (multirotor section)
- Reddit: r/Multicopter, r/fpv
- DIY Drones community

---

## ✅ Preventive Maintenance

### Before Each Flight:
- [ ] Check battery voltage
- [ ] Inspect propellers for damage
- [ ] Verify all screws tight
- [ ] Test kill switch
- [ ] Check for loose wires

### Weekly:
- [ ] Clean dust from electronics
- [ ] Check motor bearings
- [ ] Inspect solder joints
- [ ] Test battery capacity

### Monthly:
- [ ] Replace propellers (even if look OK)
- [ ] Re-calibrate IMU
- [ ] Check ESC settings
- [ ] Update firmware if needed

---

**Remember: When in doubt, DON'T FLY!**

**Document Version**: 1.0.0
**Last Updated**: November 2025
