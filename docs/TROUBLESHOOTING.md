# Troubleshooting Guide

## 🔧 Common Issues and Solutions

---

## 📡 NRF24L01 Communication Issues

### ❌ Problem: No connection between RC and FC

**Symptoms:**
- RC shows "SEARCHING..." continuously
- LED on FC doesn't blink
- No telemetry data on RC

**Solutions:**

#### 1. Check Power Supply
```
✓ NRF24L01 requires 3.3V (NOT 5V!)
✓ Add 100µF capacitor across VCC and GND
✓ Use dedicated 3.3V regulator if possible
✓ Check voltage with multimeter: should be 3.0-3.6V
```

**Test:**
```cpp
// Add to setup():
Serial.println(analogRead(A7) * 5.0 / 1023.0); // Check voltage
```

#### 2. Verify Wiring
**Flight Controller:**
| NRF Pin | Arduino Pin |
|---------|-------------|
| CE | D4 |
| CSN | D10 |
| SCK | D13 |
| MOSI | D11 |
| MISO | D12 |
| VCC | 3.3V |
| GND | GND |

**Remote Controller:**
| NRF Pin | Arduino Pin |
|---------|-------------|
| CE | D9 |
| CSN | D10 |
| SCK | D13 |
| MOSI | D11 |
| MISO | D12 |
| VCC | 3.3V |
| GND | GND |

#### 3. Check RF Channel
```cpp
// Both boards must use same channel:
#define NRF_CHANNEL 103
```

#### 4. Verify Addresses Match
```cpp
// Flight Controller:
const byte rxAddress[6] = "DRONE";
const byte txAddress[6] = "REMOT";

// Remote Controller:
const byte txAddress[6] = "DRONE";
const byte rxAddress[6] = "REMOT";
```

#### 5. Test NRF24L01 Module
Replace with known-good module - NRF24L01 modules are commonly faulty.

#### 6. Reduce Distance
Start with modules 1 meter apart for initial testing.

#### 7. Check for Interference
- Move away from WiFi routers
- Turn off other 2.4GHz devices
- Avoid metal objects between modules

---

### ❌ Problem: Connection drops frequently

**Symptoms:**
- LED blinks then stops
- "Signal lost - DISARMED" messages
- Intermittent connection

**Solutions:**

#### 1. Add Capacitor
```
100µF electrolytic capacitor across NRF24L01 VCC/GND
+ 100nF ceramic capacitor for high-frequency filtering
```

#### 2. Shorten SPI Wires
- Keep SPI wires < 10cm
- Use shielded cable if longer

#### 3. Improve Power Supply
```
Use dedicated 5V BEC → 3.3V regulator for NRF24L01
Minimum 250mA capacity
```

#### 4. Check ESC Noise
- ESCs generate electrical noise
- Move NRF24L01 away from ESCs/motors
- Add ferrite beads on ESC wires

#### 5. Reduce Data Rate (if needed)
```cpp
// In initNRF():
radio.setDataRate(RF24_250KBPS); // Slowest but most reliable
```

---

## 🎯 MPU6050 Sensor Issues

### ❌ Problem: MPU6050 not detected / initialization failed

**Symptoms:**
- "ERROR: MPU6050 init failed!"
- Continuous error beeps on startup

**Solutions:**

#### 1. Check I2C Connections
```
SDA → A4
SCL → A5
VCC → 5V (or 3.3V)
GND → GND
```

#### 2. Verify I2C Address
Run I2C scanner:
```cpp
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found: 0x");
      Serial.println(i, HEX);
    }
  }
}

void loop() {}
```

**Expected:** `Found: 0x68` (MPU6050)

#### 3. Check Pull-up Resistors
If using long wires (>20cm), add 4.7kΩ pull-up resistors:
- SDA to 5V
- SCL to 5V

#### 4. Test Different Address
```cpp
#define MPU6050_ADDR 0x69 // Try if 0x68 doesn't work
```

---

### ❌ Problem: Gyro calibration fails

**Symptoms:**
- 7-second continuous beep after calibration
- "Calibration FAILED" message
- Gyro offsets > ±10

**Solutions:**

#### 1. Ensure Level Surface
- Use bubble level to verify surface is flat
- Place on stable table (not carpet)
- No vibrations during calibration

#### 2. Remove Vibration Sources
- Turn off fans
- No touching drone during calibration
- Wait 10 seconds after powering on before calibrating

#### 3. Check Sensor Mounting
- MPU6050 must be firmly attached
- Use vibration dampening foam
- Arrows on MPU6050 should align with drone axes

#### 4. Verify Sensor Health
```cpp
// Add to readMPU6050():
Serial.print("Gyro X: "); Serial.println(gyroX);
Serial.print("Gyro Y: "); Serial.println(gyroY);
Serial.print("Gyro Z: "); Serial.println(gyroZ);
```

Stationary drone should show gyro values near 0 (±5).

---

### ❌ Problem: Drone drifts or oscillates

**Symptoms:**
- Drone tilts to one side
- Rapid oscillations (vibrations)
- Cannot maintain stable hover

**Solutions:**

#### 1. Re-calibrate on Level Surface
Ensure calibration is perfect.

#### 2. Reduce Vibrations
- Use soft-mount foam for Arduino/MPU6050
- Balance propellers
- Check for loose screws
- Tighten motor mounts

#### 3. Adjust Complementary Filter
```cpp
// In calculateAngles(), adjust ratio:
angleRoll = 0.98 * (angleRoll + gyroX * dt) + 0.02 * accelAngleRoll;
// Try 0.96/0.04 for more accel influence
// Try 0.99/0.01 for more gyro influence
```

#### 4. Tune PID Values
See PID Tuning section below.

---

## 🔋 MS5611 Barometer Issues

### ❌ Problem: MS5611 not detected

**Symptoms:**
- "ERROR: MS5611 init failed!" (but continues)
- Altitude always 0.0m

**Solutions:**

#### 1. Check I2C Connections
Same as MPU6050 (shared I2C bus):
```
SDA → A4
SCL → A5
VCC → 3.3V (MS5611 is 3.3V only!)
GND → GND
```

#### 2. Verify I2C Address
I2C scanner should show: `Found: 0x77`

#### 3. Check Voltage
MS5611 requires 3.3V (not 5V!):
```
Measure VCC pin: should be 3.0-3.6V
```

#### 4. Test Without MS5611
The drone can fly without barometer (no altitude hold).

---

### ❌ Problem: Altitude hold not working

**Symptoms:**
- Drone doesn't maintain altitude
- Altitude readings erratic
- Switch ON but no effect

**Solutions:**

#### 1. Verify Switch Works
Check RC display: `SW_1 (Alt Hold): ON ✓`

#### 2. Allow Barometer Warmup
Wait 30 seconds after powering on before using altitude hold.

#### 3. Calibrate at Ground Level
Power on drone at takeoff location for baseline reading.

#### 4. Check for Air Leaks
- Ensure MS5611 sensor hole is not blocked
- Don't cover sensor with tape
- Keep away from propeller wash

#### 5. Tune Altitude PID
```cpp
#define KP_ALT 2.0  // Increase for faster response
#define KI_ALT 0.1  // Increase if drifts over time
#define KD_ALT 1.5  // Increase if oscillates
```

---

## 🚁 Motor & ESC Issues

### ❌ Problem: Motors don't spin when armed

**Symptoms:**
- Drone armed, but motors silent
- Throttle up has no effect

**Solutions:**

#### 1. Check Arming Sequence
```
✓ SW_2 (Arm Switch) ON
✓ Calibration completed
✓ Throttle at minimum (1000-1100)
✓ Battery connected
```

#### 2. Verify ESC Connections
```
Motor FL → D3
Motor FR → D5
Motor RR → D6
Motor RL → D9
```

#### 3. Check ESC Power
- Battery connected to ESCs
- ESC BEC providing 5V to Arduino
- Check with multimeter: VIN should show ~5V

#### 4. Perform ESC Calibration
See Operation Manual → ESC Calibration section.

#### 5. Test Individual Motors
```cpp
// Add to setup() for testing:
motorFL.attach(MOTOR_FL_PIN);
motorFL.writeMicroseconds(1100); // Should spin slowly
delay(3000);
motorFL.writeMicroseconds(1000); // Should stop
```

---

### ❌ Problem: Motors spin at different speeds (unbalanced)

**Symptoms:**
- One motor spins faster/slower
- Drone tilts immediately when armed

**Solutions:**

#### 1. Check Propeller Direction
```
Front Left (FL):  CCW ↺
Front Right (FR): CW ↻
Rear Right (RR):  CCW ↺
Rear Left (RL):   CW ↻
```

#### 2. Verify Correct Propellers
- CW motors need CW propellers
- CCW motors need CCW propellers
- Wrong propellers = no/reverse thrust

#### 3. Calibrate ESCs
All ESCs must be calibrated together for consistent throttle response.

#### 4. Check for Motor/ESC Faults
Test each motor individually with known-good ESC.

#### 5. Verify PID Not Saturating
```cpp
// Add to loop():
Serial.print("Roll PID: "); Serial.println(rollPID);
Serial.print("Pitch PID: "); Serial.println(pitchPID);
```

If values constantly at ±400, PID is saturated - reduce gains.

---

### ❌ Problem: One motor doesn't spin

**Symptoms:**
- Three motors spin, one is silent
- Or one motor spins erratically

**Solutions:**

#### 1. Swap ESC
Replace suspected bad ESC with known-good ESC.

#### 2. Check Motor
Manually spin motor - should turn smoothly with slight resistance.

#### 3. Verify Pin Connection
```cpp
// Test specific motor:
motorFL.writeMicroseconds(1200); // Replace FL with problematic motor
```

#### 4. Check Solder Joints
- ESC to motor wires
- ESC signal wire to Arduino

---

## 🎮 Remote Controller Issues

### ❌ Problem: Joysticks not responding correctly

**Symptoms:**
- Throttle/controls inverted
- Stick movement shows no change
- Values stuck at min/max

**Solutions:**

#### 1. Check Analog Pin Connections
```
Throttle (Left V):  A0
Yaw (Left H):       A1
Pitch (Right V):    A2
Roll (Right H):     A3
```

#### 2. Test Raw Values
```cpp
// Add to loop():
Serial.print("A0: "); Serial.println(analogRead(A0));
Serial.print("A1: "); Serial.println(analogRead(A1));
// etc.
```

Should show:
- Stick neutral: ~512
- Stick min: ~0
- Stick max: ~1023

#### 3. Adjust Deadzone
```cpp
#define DEADZONE 30 // Increase if jittery, decrease if unresponsive
```

#### 4. Reverse Channel if Inverted
```cpp
// In prepareRCData():
rcData.throttle = map(throttleRaw, STICK_MAX, STICK_MIN, THROTTLE_MIN, THROTTLE_MAX);
// Swap STICK_MAX and STICK_MIN to reverse
```

---

### ❌ Problem: Switches not working

**Symptoms:**
- RC display shows switch always OFF or always ON
- No change when toggling

**Solutions:**

#### 1. Verify Wiring
```
SW_1: D2 to GND (when ON)
SW_2: D3 to GND (when ON)
```

#### 2. Check Pull-up Resistors
Switches use internal pull-ups (pinMode INPUT_PULLUP).
- Switch ON = GND = reads LOW = true
- Switch OFF = Floating = reads HIGH (pulled up) = false

#### 3. Test Switch
```cpp
// Add to loop():
Serial.print("SW1 pin: "); Serial.println(digitalRead(SW_ALTHOLD_PIN));
Serial.print("SW2 pin: "); Serial.println(digitalRead(SW_ARM_PIN));
```

Should show:
- Switch ON: 0 (LOW)
- Switch OFF: 1 (HIGH)

#### 4. Check for Short Circuit
Use multimeter continuity mode to verify switch operation.

---

## 🐛 Software / Compilation Issues

### ❌ Problem: Code won't compile

**Symptoms:**
- Red error messages in Arduino IDE
- "error: ..." messages

**Solutions:**

#### 1. Check Libraries Installed
See LIBRARIES_INSTALLATION.md

#### 2. Verify Board Selection
```
Tools → Board → Arduino Nano
Tools → Processor → ATmega328P (Old Bootloader)
```

Try "ATmega328P" if "Old Bootloader" doesn't work.

#### 3. Update Arduino IDE
Use version 1.8.13 or newer.

#### 4. Common Error Fixes

**Error: "RF24 does not name a type"**
```cpp
// Add at top:
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
```

**Error: "Servo.h: No such file"**
- Servo library missing (should be built-in)
- Reinstall Arduino IDE

**Error: "stray '\' in program"**
- Check for special characters in comments
- Re-type code (don't copy-paste from PDF)

---

### ❌ Problem: Upload fails

**Symptoms:**
- "avrdude: stk500_recv(): programmer is not responding"
- "Problem uploading to board"

**Solutions:**

#### 1. Check USB Cable
- Use data cable (not charge-only cable)
- Try different cable
- Try different USB port

#### 2. Select Correct Port
```
Tools → Port → COM# (Windows) or /dev/ttyUSB# (Linux)
```

#### 3. Try Old Bootloader
```
Tools → Processor → ATmega328P (Old Bootloader)
```

Many cheap Nano clones use old bootloader.

#### 4. Check Drivers (Windows)
Install CH340 driver if using clone Arduino Nano:
https://sparks.gogo.co.nz/ch340.html

#### 5. Reset During Upload
Press reset button on Arduino when "Uploading..." appears.

---

## ⚡ Power Issues

### ❌ Problem: Arduino reboots randomly

**Symptoms:**
- System restarts mid-flight
- Startup beeps repeat
- "watchdog reset" messages

**Solutions:**

#### 1. Check Power Supply
```
✓ Use dedicated 5V BEC (3A minimum)
✓ Don't share power with servos/other loads
✓ Add 1000µF capacitor on Arduino VIN
```

#### 2. Reduce Current Draw
- Use separate 3.3V regulator for NRF24L01
- Disable unused peripherals

#### 3. Check for Shorts
Inspect for solder bridges or exposed wires touching.

---

### ❌ Problem: Low battery voltage reading

**Symptoms:**
- Battery shows lower than actual voltage
- Voltage drops under load

**Solutions:**

#### 1. Use Voltage Divider
Current code shows placeholder (11.1V). Implement actual reading:

```cpp
float readBatteryVoltage() {
  // Use voltage divider: R1=10kΩ, R2=2.2kΩ
  // Battery → R1 → A7 → R2 → GND
  float raw = analogRead(A7);
  return (raw / 1023.0) * 5.0 * (10 + 2.2) / 2.2;
}
```

#### 2. Calibrate Reading
Measure actual voltage with multimeter and adjust formula.

---

## 🎛️ PID Tuning Issues

### ❌ Problem: Drone oscillates / shakes

**Cause:** PID gains too high

**Solution:**
```cpp
// Reduce P and increase D:
#define KP_ROLL 1.0  // Was 1.5
#define KD_ROLL 20.0 // Was 15.0
```

---

### ❌ Problem: Drone sluggish / slow to respond

**Cause:** PID gains too low

**Solution:**
```cpp
// Increase P:
#define KP_ROLL 2.0  // Was 1.5
#define KP_PITCH 2.0 // Was 1.5
```

---

### ❌ Problem: Drone drifts over time

**Cause:** I gain too low

**Solution:**
```cpp
// Increase I carefully:
#define KI_ROLL 0.1   // Was 0.05
#define KI_PITCH 0.1  // Was 0.05
```

**Warning:** Too much I causes instability!

---

## 📊 Diagnostic Tools

### Serial Monitor Debugging

Add to FlightController loop():
```cpp
Serial.print("Roll:"); Serial.print(angleRoll);
Serial.print(" Pitch:"); Serial.print(anglePitch);
Serial.print(" Motors: FL="); Serial.print(motorFLSpeed);
Serial.print(" FR="); Serial.print(motorFRSpeed);
Serial.print(" RR="); Serial.print(motorRRSpeed);
Serial.print(" RL="); Serial.println(motorRLSpeed);
```

---

### LED Blink Codes

| Pattern | Meaning |
|---------|---------|
| Solid OFF | No power or no RC connection |
| Slow blink (1Hz) | Connected, disarmed |
| Fast blink (5Hz) | Calibrating |
| Solid ON | Armed (dangerous!) |

---

## 🆘 Emergency Recovery

### Complete System Reset

1. Disconnect all power (batteries)
2. Wait 30 seconds
3. Check all connections
4. Re-upload both sketches
5. Power on RC first, then FC
6. Perform full calibration

---

### Factory Reset PID Values

Replace with conservative safe values:
```cpp
#define KP_ROLL   1.0
#define KI_ROLL   0.0
#define KD_ROLL   10.0

#define KP_PITCH  1.0
#define KI_PITCH  0.0
#define KD_PITCH  10.0

#define KP_YAW    2.0
#define KI_YAW    0.0
#define KD_YAW    0.0
```

---

## 📞 Getting More Help

If problems persist:

1. **Check wiring** - 90% of issues are wiring errors
2. **Test components individually** - isolate the problem
3. **Search Arduino forums** - others likely had same issue
4. **Use I2C/SPI scanners** - verify hardware detection
5. **Simplify setup** - remove one component at a time

---

**Remember**: Most problems are hardware-related. Double-check wiring before suspecting code issues!
