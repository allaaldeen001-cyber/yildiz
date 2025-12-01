# 🔧 Troubleshooting Guide - Quadcopter Drone

Complete guide to diagnosing and fixing common issues.

---

## 📋 Table of Contents

- [Quick Diagnostics](#quick-diagnostics)
- [Power & Electrical Issues](#power--electrical-issues)
- [Motor Issues](#motor-issues)
- [Radio Communication Issues](#radio-communication-issues)
- [Sensor Issues](#sensor-issues)
- [Flight Behavior Issues](#flight-behavior-issues)
- [Software Issues](#software-issues)

---

## 🔍 Quick Diagnostics

### Diagnostic Checklist

Run through this list before investigating specific issues:

```
┌─────────────────────────────────────────────────────────┐
│ POWER                                                   │
├─────────────────────────────────────────────────────────┤
│ [ ] Battery charged (>11V for 3S LiPo)                 │
│ [ ] Power LED on Arduino Nano lights up                │
│ [ ] nRF24L01+ LED blinks (if equipped)                 │
│ [ ] ESCs beep on power-up                              │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│ CONNECTIONS                                             │
├─────────────────────────────────────────────────────────┤
│ [ ] All wires firmly connected                         │
│ [ ] No loose propellers                                │
│ [ ] ESC signal wires in correct pins (D3,D5,D6,D9)     │
│ [ ] I2C devices on SDA/SCL (A4/A5)                     │
│ [ ] nRF24L01+ properly seated                          │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│ SOFTWARE                                                │
├─────────────────────────────────────────────────────────┤
│ [ ] Correct sketch uploaded (FC vs RC)                 │
│ [ ] Serial Monitor shows startup messages              │
│ [ ] All libraries installed                            │
│ [ ] Baud rate set to 115200                            │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│ CALIBRATION                                             │
├─────────────────────────────────────────────────────────┤
│ [ ] Gyro calibrated on flat surface                    │
│ [ ] Altitude calibrated                                │
│ [ ] Joysticks centered during RC startup               │
│ [ ] ESCs calibrated (1000-2000µs range)                │
└─────────────────────────────────────────────────────────┘
```

---

## ⚡ Power & Electrical Issues

### Issue: Arduino doesn't power on

**Symptoms**: No lights, no response

**Possible causes & solutions**:

1. **Dead/disconnected battery**
   - Check battery voltage (should be >10.5V for 3S)
   - Check all power connections
   - Try different battery

2. **Damaged voltage regulator**
   - Arduino Nano VIN accepts 7-12V
   - If you applied >12V, regulator may be damaged
   - Try powering via USB (5V)

3. **Short circuit**
   - Disconnect all peripherals
   - Power Arduino alone
   - Add peripherals one-by-one to find culprit

---

### Issue: nRF24L01+ won't work

**Symptoms**: Radio initialization failed, "Radio initialization FAILED!" message

**Solutions**:

1. **Add decoupling capacitor** (MOST COMMON FIX)
   ```
   ┌──────────┐
   │ NRF24L01 │
   │          │
   │ VCC ─────┼──┬── 3.3V
   │          │  │
   │          │ [10µF]
   │          │  │
   │ GND ─────┼──┴── GND
   └──────────┘
   ```
   - Add 10µF electrolytic capacitor between VCC and GND
   - Place as close to nRF24 module as possible
   - nRF24 draws current spikes that cause voltage drops

2. **Check power supply**
   - nRF24L01+ requires **3.3V**, not 5V!
   - Most Arduino Nano clones have 3.3V output
   - Some PA+LNA versions need more current than Nano can supply
   - Solution: External 3.3V regulator (AMS1117-3.3)

3. **Check wiring**
   ```
   Flight Controller:
   CE  → D4
   CSN → D10
   MOSI → D11
   MISO → D12
   SCK → D13
   
   Remote Controller:
   CE  → D9
   CSN → D10
   MOSI → D11
   MISO → D12
   SCK → D13
   ```

4. **Try different nRF24 module**
   - Clones are often defective
   - Try swapping modules between FC and RC

---

### Issue: ESCs keep beeping

**Symptoms**: Continuous beeping, motors don't arm

**Beep patterns & meanings**:

| Beeps | Meaning | Solution |
|-------|---------|----------|
| Continuous | No signal | Check signal wire connection |
| 1-2-3 ascending | Normal startup | This is good! |
| Rapid beeping | Low voltage | Charge battery |
| 3 beeps repeating | Throttle not at minimum | Lower throttle |

**Solutions**:

1. **Calibrate ESCs**
   ```
   Procedure:
   1. Disconnect battery
   2. Set throttle to maximum (2000µs)
   3. Connect battery (ESCs beep)
   4. Set throttle to minimum (1000µs)
   5. ESCs beep confirmation
   6. Done!
   ```

2. **Check PWM signal**
   - Upload simple test code:
   ```cpp
   #include <Servo.h>
   Servo motor;
   
   void setup() {
     motor.attach(3);
     motor.writeMicroseconds(1000); // Min throttle
   }
   
   void loop() {
     delay(2000);
     motor.writeMicroseconds(1100); // Slight spin
     delay(2000);
     motor.writeMicroseconds(1000); // Stop
   }
   ```

---

## 🔄 Motor Issues

### Issue: Motors don't spin

**Symptoms**: Armed, throttle up, but no motor movement

**Diagnostic steps**:

1. **Check if motors are receiving signal**
   - Open Serial Monitor
   - Check motor speed values (should be 1000-2000)
   - If values are correct but motors don't spin → hardware issue

2. **Test individual motors**
   - Press Button 2 (Motor Test) on RC
   - Each motor should spin briefly
   - If not → check connections

3. **Check ESC calibration**
   - See ESC calibration procedure above
   - All 4 ESCs must be calibrated identically

4. **Check motor direction**
   - Each motor should spin when tested
   - Direction matters for flight, not for testing

---

### Issue: Motors spin but drone flips on takeoff

**Symptoms**: Immediate flip when armed

**This is a CONFIGURATION issue, not hardware failure!**

**Solution 1: Check motor rotation direction**

```
          FRONT
         FL   FR
          \ X /
           X
          / X \
         RL   RR

Required rotation:
  FL (D3) = Counter-Clockwise (CCW)
  FR (D5) = Clockwise (CW)
  RR (D6) = Counter-Clockwise (CCW)
  RL (D9) = Clockwise (CW)
```

To reverse motor direction:
- Swap any 2 of the 3 motor wires to ESC

**Solution 2: Check motor positions**

```
Correct wiring:
  Pin D3 → Front-Left motor
  Pin D5 → Front-Right motor
  Pin D6 → Rear-Right motor
  Pin D9 → Rear-Left motor
```

**Solution 3: Recalibrate gyro**
- Gyro must be calibrated on flat, level surface
- Press Button 1 on RC to recalibrate

---

### Issue: One motor spins faster/slower

**Symptoms**: Drone drifts in one direction, unstable hover

**Solutions**:

1. **Check PID balance**
   - All motors should have similar speeds when hovering
   - Check Serial Monitor: `Motors:1245,1255,1240,1250`
   - If one is consistently different → investigate

2. **Check motor/propeller**
   - Damaged propeller → uneven thrust
   - Bent motor shaft → vibrations
   - Debris in motor → reduced efficiency

3. **Check ESC programming**
   - All ESCs should have identical settings
   - Re-calibrate all ESCs together

---

## 📡 Radio Communication Issues

### Issue: "Radio signal lost" / Failsafe triggered

**Symptoms**: Serial shows "FAILSAFE: Radio signal lost!"

**Solutions**:

1. **Check RC is powered on**
   - Obvious but often forgotten!
   - RC Serial Monitor should show "TX: ✅ OK"

2. **Check antenna orientation**
   - nRF24L01+ is directional
   - Antennas should be parallel
   - Keep modules within range (start with 10m)

3. **Check for interference**
   - WiFi routers on 2.4GHz interfere
   - Other drones/RC toys nearby
   - Metal obstacles block signal
   - Try different channel (currently 108):
   ```cpp
   radio.setChannel(108); // Try 76, 100, 120
   ```

4. **Check radio addresses match**
   - Both FC and RC must use same address:
   ```cpp
   const uint64_t radioAddress = 0xF0F0F0F0E1LL;
   ```

---

### Issue: RC transmits but FC doesn't receive

**Symptoms**: RC shows "TX: ✅ OK", FC shows no data

**Diagnostic**:

Check FC Serial Monitor for radio values:
```
RC:OK | Throttle:0 | Yaw:0 | Pitch:0 | Roll:0
```

If all zeros → not receiving

**Solutions**:

1. **Swap nRF24 modules**
   - Test if one module is faulty

2. **Check FC radio initialization**
   - Should see: "✅ Radio initialized (2.4GHz, 250kbps)"
   - If not → wiring or power issue

3. **Test with simple code**
   ```cpp
   // On FC, add in loop:
   if (radio.available()) {
     Serial.println("PACKET RECEIVED!");
   }
   ```

---

## 🎯 Sensor Issues

### Issue: MPU6050 initialization failed

**Symptoms**: "❌ MPU6050 initialization FAILED!"

**Solutions**:

1. **Check I2C wiring**
   ```
   MPU6050 → Arduino Nano
   VCC → 5V
   GND → GND
   SDA → A4
   SCL → A5
   ```

2. **Check I2C address**
   - MPU6050 default address: 0x68
   - If AD0 pin high: 0x69
   - Scan I2C bus:
   ```cpp
   #include <Wire.h>
   
   void setup() {
     Serial.begin(115200);
     Wire.begin();
     
     for (byte addr = 1; addr < 127; addr++) {
       Wire.beginTransmission(addr);
       if (Wire.endTransmission() == 0) {
         Serial.print("Found device at 0x");
         Serial.println(addr, HEX);
       }
     }
   }
   ```

3. **Check power supply**
   - MPU6050 needs stable 3.3V or 5V
   - Try powering from external 5V source

---

### Issue: MS5611 initialization failed

**Symptoms**: "❌ MS5611 initialization FAILED!"

**Solutions**:

1. **Check I2C wiring** (same as MPU6050)

2. **Check I2C address**
   - MS5611 default address: 0x77
   - Run I2C scanner (code above)

3. **Check for I2C conflicts**
   - Both MPU6050 and MS5611 share I2C bus
   - Addresses must be different
   - MPU6050: 0x68, MS5611: 0x77 ✅ OK

---

### Issue: Altitude readings incorrect

**Symptoms**: Serial shows altitude fluctuating wildly

**Solutions**:

1. **Recalibrate altitude**
   - Press Button 1 on RC
   - Must be done on flat surface

2. **Check mounting location**
   - MS5611 sensitive to airflow
   - Mount away from propellers
   - Add foam cover to shield from wind

3. **Check for air leaks**
   - MS5611 measures air pressure
   - Case must be sealed
   - Small hole OK for pressure equalization

4. **Environmental factors**
   - Indoor: HVAC causes pressure changes
   - Outdoor: Wind causes fluctuations
   - This is normal, PID compensates

---

### Issue: Gyro drifts over time

**Symptoms**: Drone tilts slowly even when level

**Solutions**:

1. **Recalibrate gyro**
   - Must be done on flat surface
   - Drone must be completely still
   - Wait for "Gyro calibration complete"

2. **Check temperature**
   - MPU6050 drifts with temperature changes
   - Let sensors warm up for 1-2 minutes
   - Recalibrate after warmup

3. **Increase complementary filter accel weight**
   ```cpp
   // In updateAttitude():
   // Current: 98% gyro, 2% accel
   angleRoll = 0.98 * (angleRoll + gyroX * deltaTime) + 0.02 * accelRoll;
   
   // Try: 96% gyro, 4% accel (more drift correction)
   angleRoll = 0.96 * (angleRoll + gyroX * deltaTime) + 0.04 * accelRoll;
   ```

---

## ✈️ Flight Behavior Issues

### Issue: Drone oscillates/vibrates when hovering

**Symptoms**: Rapid back-and-forth movement

**This is a PID TUNING issue!**

**Solutions**:

1. **Reduce P gain** (Rate PID)
   ```cpp
   // Before
   pidRateRoll.Kp = 1.0; // Too high!
   
   // After
   pidRateRoll.Kp = 0.7; // Reduced by 30%
   ```

2. **Increase D gain** (Rate PID)
   ```cpp
   // Before
   pidRateRoll.Kd = 0.010; // Too low!
   
   // After
   pidRateRoll.Kd = 0.020; // Increased
   ```

3. **Check for mechanical issues**
   - Loose motor mounts → vibrations
   - Bent propellers → imbalance
   - Damaged frame → flexing
   - Tighten all screws!

4. **Check loop timing**
   - Should run at 250Hz (4ms)
   - Check Serial Monitor for loop time
   - If >4ms → code optimization needed

---

### Issue: Drone drifts in one direction

**Symptoms**: Moves left/right/forward/back without input

**Solutions**:

1. **Recalibrate gyro and accelerometer**
   - Press Button 1 on RC

2. **Check frame is level**
   - Use spirit level
   - Frame must be perfectly flat during calibration

3. **Increase I gain** (Rate PID)
   ```cpp
   // Before
   pidRateRoll.Ki = 0.2; // Too low for drift correction
   
   // After
   pidRateRoll.Ki = 0.5; // Increased
   ```

4. **Check wind compensation**
   - Some drift is normal in wind
   - ALTITUDE HOLD mode compensates better

---

### Issue: Altitude won't hold steady

**Symptoms**: Bounces up and down in ALT HOLD mode

**Solutions**:

1. **Reduce altitude P gain**
   ```cpp
   // Before
   pidAltitude.Kp = 8.0; // Too aggressive!
   
   // After
   pidAltitude.Kp = 5.0; // More stable
   ```

2. **Increase altitude D gain** (velocity damping)
   ```cpp
   // Before
   pidAltitude.Kd = 1.0; // Not enough damping
   
   // After
   pidAltitude.Kd = 3.0; // Smoother
   ```

3. **Check MS5611 mounting**
   - Should not be in direct airflow from props
   - Add foam shield

4. **Fly higher**
   - Ground effect causes instability below 50cm
   - Try hovering at 1.5m+

---

### Issue: Drone won't ARM

**Symptoms**: Button 4 doesn't start takeoff, motor test fails

**Diagnostic**:

Check Serial Monitor for error messages

**Solutions**:

1. **Check throttle position**
   - Throttle must be <100 to ARM
   - Center left stick

2. **Check radio connection**
   - Serial should show "RC:OK"
   - If "RC:LOST" → fix radio first

3. **Check for previous ARM**
   - Already armed? Can't ARM twice
   - Land and disarm first

---

## 💻 Software Issues

### Issue: Sketch won't compile

**Symptoms**: "Compilation error" in Arduino IDE

**Solutions**:

1. **Check all libraries installed**
   - See LIBRARIES.txt
   - Ensure correct versions

2. **Check board selection**
   - Tools → Board → Arduino Nano
   - Tools → Processor → ATmega328P (Old Bootloader)

3. **Check for typos**
   - If you modified code, check syntax

4. **Try example sketch**
   - File → Examples → Basics → Blink
   - If Blink compiles → libraries issue
   - If Blink fails → Arduino IDE issue

---

### Issue: Sketch won't upload

**Symptoms**: "Upload error" or timeout

**Solutions**:

1. **Check USB cable**
   - Try different cable (some are charge-only)
   - Try different USB port

2. **Check COM port**
   - Tools → Port → Select correct port
   - Windows: COM1, COM3, etc.
   - Linux: /dev/ttyUSB0, /dev/ttyACM0
   - Mac: /dev/cu.usbserial-*

3. **Try different processor**
   - If "Old Bootloader" fails, try without

4. **Check drivers**
   - CH340 driver for most Nano clones
   - Download from manufacturer website

---

### Issue: Serial Monitor shows gibberish

**Symptoms**: Random characters, not readable text

**Solutions**:

1. **Check baud rate**
   - Must be set to **115200**
   - Bottom-right corner of Serial Monitor

2. **Check board is running**
   - Upload "Blink" example first
   - LED should blink → board OK

---

### Issue: Loop rate not 250Hz

**Symptoms**: Serial shows loop time >4ms

**Solutions**:

1. **Reduce debug output**
   - Comment out some Serial.print() statements
   - Each print takes time

2. **Optimize sensor reading**
   - MPU6050 should be in DLPF mode
   - MS5611 should update at 50Hz, not 250Hz

3. **Check I2C speed**
   ```cpp
   Wire.setClock(400000); // 400kHz fast mode
   ```

---

## 🆘 Emergency Procedures

### Situation: Drone is flying away (flyaway)

**Immediate actions**:
1. Try to regain control with sticks
2. Press Button 3 (emergency landing)
3. If no response, cut power remotely (not possible with current design)
4. Let failsafe trigger (1 second)

**Prevention**:
- Always fly within visual range
- Test failsafe before each flight
- Check compass calibration (if added)

---

### Situation: Drone crashed

**After crash checklist**:
1. **Safety first**: Disarm immediately, disconnect battery
2. **Inspect damage**: Check frame, motors, props, ESCs
3. **Check connections**: Wires may have pulled loose
4. **Test motors**: Use Button 2 (motor test), props off
5. **Recalibrate sensors**: Crash may have shifted calibration
6. **Test flight**: Hover at low altitude first

---

## 📞 Getting Help

### Information to provide when asking for help:

1. **Hardware**:
   - Arduino model (Nano, clone, etc.)
   - Sensor models (MPU6050 module type)
   - Frame size, motor KV, propeller size

2. **Software**:
   - Arduino IDE version
   - Library versions (check Library Manager)
   - Any code modifications

3. **Behavior**:
   - Exact symptoms
   - When it started
   - What changed recently

4. **Serial Monitor output**:
   - Copy/paste relevant messages
   - Include startup sequence

5. **Video** (if possible):
   - Flight behavior
   - Serial Monitor during issue

---

**Good luck with your build! 🚁**

*Most issues are simple fixes - methodically work through diagnostics and you'll find the solution!*
