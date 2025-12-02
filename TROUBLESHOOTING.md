# Troubleshooting Guide

## Radio Communication Issues

### ❌ "ERROR: NRF24L01 not found!"

**Symptoms:**
- Transmitter/receiver won't initialize
- Stuck at startup with error message

**Causes & Solutions:**

1. **Wiring Error**
   - Double-check all connections
   - CE pin -> D9
   - CSN pin -> D10
   - MOSI -> D11, MISO -> D12, SCK -> D13
   - Make sure using SPI pins (not I2C!)

2. **Bad Power Supply**
   - NRF24L01 REQUIRES 3.3V (NOT 5V!)
   - Use external regulator (AMS1117-3.3 or similar)
   - Add capacitors: 100uF electrolytic + 0.1uF ceramic
   - Measure voltage with multimeter: should be 3.2-3.4V
   - Arduino's 3.3V pin often insufficient

3. **Defective Module**
   - NRF24L01 modules can be faulty (common issue)
   - Try different module
   - Prefer modules with external antenna

4. **Loose Connections**
   - Solder connections (breadboard unreliable)
   - Check for cold solder joints
   - Wiggle wires to test

**Quick Test:**
```cpp
if (!radio.begin()) {
  Serial.println("Module not detected");
  // Check wiring and power
}
```

---

### ❌ "WARNING: NO CONNECTION TO DRONE!"

**Symptoms:**
- Transmitter powers on but can't reach receiver
- "No connection" messages
- High packet loss

**Causes & Solutions:**

1. **Address Mismatch**
   - Verify address matches in both codes
   - Default: `"DRON1"` (exactly 5 characters)
   ```cpp
   // Must be IDENTICAL in TX and RX
   const byte address[6] = "DRON1";
   ```

2. **Channel Mismatch**
   - Both must use same channel (default: 108)
   ```cpp
   radio.setChannel(108);  // Same in both!
   ```

3. **Power Supply Issues**
   - Most common cause!
   - Add larger capacitors (470uF + 10uF)
   - Use separate regulator for each NRF24L01
   - Keep wires short (<10cm)

4. **Distance Too Far**
   - Start with modules 1 meter apart
   - Increase distance gradually
   - Max range: 100-300m (varies with environment)
   - Walls/obstacles significantly reduce range

5. **Interference**
   - Move away from WiFi routers
   - Try different channel (try 76, 108, 124)
   - Avoid operating near microwaves
   - Metal objects block signal

6. **Module Orientation**
   - Antennas should be parallel
   - Avoid metal near antenna
   - Keep antennas vertical

**Testing Steps:**
1. Place modules 50cm apart
2. Upload code to both
3. Check serial monitor for connection
4. If working, increase distance slowly
5. Note where connection fails

---

### ❌ High Packet Loss (>5%)

**Symptoms:**
- Connection established but unreliable
- Frequent "lost" messages
- Intermittent control

**Solutions:**

1. **Improve Power Supply** (most important!)
   ```
   Battery + -> Regulator IN
   Regulator OUT -> 100uF cap -> NRF24 VCC
   NRF24 VCC -> 0.1uF cap -> GND
   ```

2. **Reduce Interference**
   - Change channel: `radio.setChannel(76);` or `108` or `124`
   - Move away from WiFi
   - Shield NRF24 with aluminum foil (leave antenna exposed)

3. **Adjust Retry Settings**
   ```cpp
   // More aggressive retries
   radio.setRetries(5, 15);  // 5x250us delay, 15 retries
   ```

4. **Check for Loose Wires**
   - Solder all connections
   - Use shorter wires
   - Add hot glue for strain relief

5. **Verify Data Rate**
   - 250kbps most reliable (default)
   ```cpp
   radio.setDataRate(RF24_250KBPS);  // Best range
   ```

---

## Flight Control Issues

### ❌ Drone Won't Arm

**Symptoms:**
- Arm switch on but motors don't spin
- LED not indicating armed state

**Causes & Solutions:**

1. **Throttle Not Low**
   - Throttle stick must be at minimum
   - Must be < 1050 to arm
   - Check serial monitor for throttle value

2. **No Radio Connection**
   - Must have active connection to arm
   - Check for "NO CONNECTION" messages
   - Fix radio issues first

3. **Failsafe Active**
   - Failsafe prevents arming
   - Ensure good signal before arming

4. **Code Logic Check**
   ```cpp
   // Arming conditions:
   if (controlData.arm && !failsafe && controlData.throttle < 1050) {
     armed = true;
   }
   ```

---

### ❌ Rapid Oscillations (Shaking)

**Symptoms:**
- Drone shakes rapidly when hovering
- High-frequency vibration
- Can't maintain stable hover

**Causes & Solutions:**

1. **P Gain Too High** (most common)
   - Reduce P by 30%
   ```cpp
   pidRollP = 1.3;   // Try 0.9
   pidPitchP = 1.3;  // Try 0.9
   ```
   - Retest and adjust incrementally

2. **D Gain Too Low**
   - Increase D by 20%
   ```cpp
   pidRollD = 18.0;  // Try 22.0
   ```

3. **Mechanical Issues**
   - Check propeller balance (spin test)
   - Verify motor mounting tight
   - Check frame for cracks/flex
   - Replace damaged props

4. **Vibration in Sensors**
   - Add foam under flight controller
   - Isolate MPU6050 from vibration
   - Use soft mounting

---

### ❌ Slow Wobbles

**Symptoms:**
- Drone wobbles slowly (1-2 Hz)
- Overshoots when correcting
- Waves back and forth

**Causes & Solutions:**

1. **I Gain Too High**
   - Reduce I by 50%
   ```cpp
   pidRollI = 0.04;   // Try 0.02
   pidPitchI = 0.04;  // Try 0.02
   ```

2. **P Gain Too High**
   - Reduce P by 20%
   ```cpp
   pidRollP = 1.3;   // Try 1.0
   ```

3. **Battery Low**
   - Low voltage affects motor response
   - Use fresh battery for tuning

---

### ❌ Drone Drifts in One Direction

**Symptoms:**
- Constantly drifts left/right/forward/back
- Requires constant stick input to stay in place

**Causes & Solutions:**

1. **I Gain Too Low**
   - Increase I slightly
   ```cpp
   pidRollI = 0.04;   // Try 0.05
   pidPitchI = 0.04;  // Try 0.05
   ```

2. **Gyro Not Calibrated**
   - Recalibrate gyro on LEVEL surface
   - Keep drone COMPLETELY STILL during calibration
   - Watch for "Calibrating..." message

3. **Motor Imbalance**
   - Check motor thrust (swap motors to isolate)
   - Verify ESC calibration
   - Check prop condition (replace damaged)
   - One motor may be weaker

4. **Frame Not Level**
   - Check if frame bent/damaged
   - Verify motors mounted at same angles

5. **CG (Center of Gravity) Off**
   - Balance battery placement
   - Distribute weight evenly

---

### ❌ Sluggish Response

**Symptoms:**
- Drone slow to respond to inputs
- Takes long time to level out
- Feels "floaty"

**Causes & Solutions:**

1. **P Gain Too Low**
   - Increase P by 20%
   ```cpp
   pidRollP = 1.3;   // Try 1.6
   pidPitchP = 1.3;  // Try 1.6
   ```

2. **Battery Voltage Low**
   - Check battery (should be >11.1V for 3S)
   - Use fresh battery

3. **Motors/Props Wrong Size**
   - Verify motor KV matches drone weight
   - Check prop size appropriate
   - Too small = insufficient thrust

4. **Too Much Weight**
   - Remove unnecessary components
   - Optimize wire routing
   - Use lighter battery if possible

---

### ❌ Drone Flips on Takeoff

**Symptoms:**
- Immediately flips when armed
- Crashes before lifting off

**Causes & Solutions:**

1. **Motor Direction Wrong** ⚠️ MOST COMMON!
   - Check motor rotation directions
   ```
   M1 (FL): Clockwise (CW)
   M2 (FR): Counter-clockwise (CCW)
   M3 (BR): Clockwise (CW)
   M4 (BL): Counter-clockwise (CCW)
   ```
   - Swap any 2 motor wires on ESC to reverse

2. **Motor Order Wrong**
   - Verify motor plugged into correct pins
   ```
   M1 -> Pin 3  (Front-Left)
   M2 -> Pin 5  (Front-Right)
   M3 -> Pin 6  (Back-Right)
   M4 -> Pin 11 (Back-Left)
   ```

3. **Prop Direction Wrong**
   - Props have CW and CCW versions
   - Match prop to motor rotation
   - Check for "R" marking on CCW props

4. **Gyro Orientation Wrong**
   - MPU6050 must be level
   - X-axis forward, Y-axis left
   - If wrong, code needs adjustment

5. **P Gain WAY Too High**
   - Try starting values (P=0.5, I=0, D=0)
   - Tune up slowly

---

### ❌ Motors Spin Unevenly

**Symptoms:**
- One or more motors spin faster/slower
- Even at low throttle, imbalance visible

**Causes & Solutions:**

1. **ESC Calibration Needed**
   - Calibrate all ESCs together
   - Upload this to receiver:
   ```cpp
   void setup() {
     motor1.attach(3); motor2.attach(5);
     motor3.attach(6); motor4.attach(11);
     
     // Max throttle
     motor1.writeMicroseconds(2000);
     motor2.writeMicroseconds(2000);
     motor3.writeMicroseconds(2000);
     motor4.writeMicroseconds(2000);
     delay(2000);  // Wait for ESC beep
     
     // Min throttle
     motor1.writeMicroseconds(1000);
     motor2.writeMicroseconds(1000);
     motor3.writeMicroseconds(1000);
     motor4.writeMicroseconds(1000);
     delay(2000);  // Wait for ESC beep
   }
   void loop() {}
   ```
   - Power on with battery, wait for beeps
   - All ESCs should sound together

2. **Bad ESC**
   - Swap ESCs to isolate
   - Replace if defective

3. **Motor Damage**
   - Check motor bearings (spin freely?)
   - Look for physical damage
   - Test motor current (should be similar)

---

## Sensor Issues

### ❌ Gyro Drift

**Symptoms:**
- Drone tilts more over time
- Angle drifts even when level

**Solutions:**

1. **Recalibrate on Level Surface**
   - Place on perfectly flat surface
   - Don't touch during calibration
   - Wait for "Calibrated" message

2. **Temperature Affecting Gyro**
   - MPU6050 drifts when temperature changes
   - Let warm up before flying
   - Add complementary filter weight to accel:
   ```cpp
   angleX = angleX * 0.96 + accelAngleX * 0.04;  // More accel
   ```

3. **Vibration**
   - Isolate MPU6050 from vibration
   - Use foam mounting
   - Check prop balance

---

### ❌ Accelerometer Noisy

**Symptoms:**
- Jittery angle readings
- Unstable in hover despite tuning

**Solutions:**

1. **Reduce Accel Weight in Filter**
   ```cpp
   angleX = angleX * 0.99 + accelAngleX * 0.01;  // Less accel
   ```

2. **Add Vibration Damping**
   - Foam under flight controller
   - Balance propellers
   - Tighten motor screws

3. **Check MPU6050 Settings**
   - Verify low-pass filter enabled
   ```cpp
   Wire.write(0x02);  // DLPF_CFG = 2 (98Hz)
   ```

---

## Power Issues

### ❌ Battery Drains Too Fast

**Solutions:**
- Use higher capacity battery
- Check for motor/ESC inefficiency
- Verify proper prop size
- Reduce flight time
- Check for short circuits

---

### ❌ Brownout / Resets During Flight

**Symptoms:**
- Flight controller resets mid-flight
- Lost connection then reconnects
- Motors stop then restart

**Solutions:**

1. **Voltage Regulator Insufficient**
   - Use 5V BEC from ESC or separate regulator
   - Ensure 2A+ capacity
   - Add capacitors (1000uF) on 5V rail

2. **Battery Connector Loose**
   - Check all power connections
   - Solder if using connectors
   - Use XT60 or similar quality connectors

3. **Battery Too Weak**
   - Check C-rating (should be 25C+)
   - Test battery voltage under load
   - Replace old/damaged battery

---

## ESC Issues

### ❌ ESC Beeping Continuously

**Meanings:**
- **Continuous tone**: No signal / not calibrated
- **Beep-beep-beep**: Low voltage
- **Different tones**: Check ESC manual

**Solutions:**
1. Check PWM signal connected
2. Calibrate ESC (see Motor Spin Unevenly)
3. Check battery voltage
4. Verify correct PWM range (1000-2000us)

---

### ❌ Motor Stuttering

**Symptoms:**
- Motor jerks/stutters instead of smooth
- Cogging at low throttle

**Solutions:**
1. Update ESC firmware (BLHeli, SimonK)
2. Change ESC timing settings
3. Check motor magnet alignment
4. Verify PWM frequency (50-490Hz)

---

## Calibration Procedures

### Gyro Calibration
1. Place drone on level surface
2. Don't touch for 6 seconds
3. Watch for "Calibrated" message
4. LED will blink when done

### ESC Calibration
1. Disconnect battery
2. Upload calibration sketch (see above)
3. Connect battery
4. Wait for beeps (high-low)
5. Disconnect battery
6. Upload normal firmware

### Radio Range Test
1. Arm drone (props off!)
2. Walk away slowly
3. Watch serial for packet loss
4. Mark distance where loss starts
5. Safe flight range = 50% of that

---

## Safety Procedures

### Before Every Flight
- [ ] Check battery voltage (>11.1V for 3S)
- [ ] Verify propellers tight and undamaged
- [ ] Test failsafe (turn off TX, motors stop?)
- [ ] Check all screws tight
- [ ] Clear flight area
- [ ] Test arm/disarm

### After a Crash
- [ ] Disconnect battery IMMEDIATELY
- [ ] Check for broken props
- [ ] Inspect motors (spin freely?)
- [ ] Check frame for cracks
- [ ] Test gyro calibration
- [ ] Verify motor directions
- [ ] Test on bench before flying

---

## Getting Help

### Useful Debug Output

Add to receiver code for diagnosis:
```cpp
void debugPrint() {
  Serial.print("ANG:"); Serial.print(angleX); Serial.print(","); Serial.print(angleY);
  Serial.print(" GYRO:"); Serial.print(gyroX); Serial.print(","); Serial.print(gyroY);
  Serial.print(" PID:"); Serial.print(pidRollOutput); Serial.print(","); Serial.print(pidPitchOutput);
  Serial.print(" MOT:"); Serial.print(motor1Speed); Serial.print(",");
  Serial.print(motor2Speed); Serial.print(",");
  Serial.print(motor3Speed); Serial.print(",");
  Serial.println(motor4Speed);
}
```

### What to Include When Asking for Help
1. Exact symptoms
2. Serial monitor output
3. PID values being used
4. Drone specifications (weight, motors, props)
5. Video if possible
6. What you've already tried

---

## Quick Reference: LED Status

- **Solid ON**: System ready, good connection
- **Slow Blink** (1Hz): Waiting for connection
- **Fast Blink** (5Hz): Error / failsafe
- **Rapid Flash**: Radio initialization failed

---

**Still stuck? Double-check:**
1. Power supply (most common issue!)
2. Wiring matches code
3. Address and channel match
4. Gyro calibrated on level surface
5. Props on correct motors
6. Motor directions correct

Good luck! 🚁
