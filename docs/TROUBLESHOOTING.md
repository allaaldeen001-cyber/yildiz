# Troubleshooting Guide

Comprehensive solutions to common issues with your quadcopter drone system.

---

## Hardware Issues

### NRF24L01 Not Connecting

**Symptoms:**
- Remote shows "NRF: NO LINK"
- Flight Controller LED doesn't blink
- No communication between boards

**Solutions:**

1. **Check Power Supply**
   ```
   - Measure voltage at NRF VCC pin
   - Should be 3.2-3.4V (NOT 5V!)
   - Use multimeter to verify
   ```

2. **Add Decoupling Capacitor**
   ```
   - Solder 10µF capacitor between VCC and GND
   - Place as close to NRF module as possible
   ```

3. **Use External 3.3V Regulator**
   ```
   Arduino 5V → AMS1117-3.3V → NRF VCC
                     ↓
                    10µF capacitor
   ```

4. **Check Wiring**
   ```
   Flight Controller:        Remote Controller:
   CE  → D4                  CE  → D9
   CSN → D10                 CSN → D10
   SCK → D13                 SCK → D13
   MOSI→ D11                 MOSI→ D11
   MISO→ D12                 MISO→ D12
   ```

5. **Reduce Wire Length**
   - Keep wires < 10cm
   - Use shorter jumper wires
   - Consider PCB adapter board

6. **Test with Basic NRF Example**
   ```cpp
   File → Examples → RF24 → GettingStarted
   Upload to both boards and test
   ```

---

### MPU6050 Not Detected

**Symptoms:**
- Serial Monitor shows "MPU6050 FAILED!"
- Error tone on startup
- Cannot calibrate gyro

**Solutions:**

1. **Check I2C Connections**
   ```
   MPU6050 → Arduino Nano
   SDA → A4
   SCL → A5
   VCC → 5V
   GND → GND
   ```

2. **Run I2C Scanner**
   ```cpp
   // Upload I2C scanner sketch
   // Should detect device at address 0x68
   ```

3. **Check AD0 Pin**
   ```
   - AD0 pin should be connected to GND
   - This sets I2C address to 0x68
   ```

4. **Add Pull-up Resistors (if needed)**
   ```
   - 4.7kΩ resistor: SDA to 5V
   - 4.7kΩ resistor: SCL to 5V
   - Only if wire length > 15cm
   ```

5. **Test with Basic MPU Example**
   ```cpp
   File → Examples → MPU6050 → Examples → MPU6050_DMP6
   ```

---

### Motors Not Spinning

**Symptoms:**
- Motors don't respond during tests
- No beeps from ESCs
- Can arm but motors stay off

**Solutions:**

1. **Check ESC Power**
   ```
   - Verify battery connected to PDB
   - Check voltage at ESC power wires (11.1V for 3S)
   - Ensure battery not depleted
   ```

2. **Check Signal Wires**
   ```
   ESC Signal → Arduino Pin
   FL → D3
   FR → D5
   RR → D6
   RL → D9
   
   ESC Ground → Arduino GND (IMPORTANT!)
   ```

3. **Perform ESC Calibration**
   ```
   - SW1 to "0"
   - Press Button 2
   - Wait for beep sequence
   - Motors should spin one by one
   ```

4. **Test Individual ESC**
   ```
   - Connect only one ESC
   - Upload Servo sweep example
   - Verify ESC responds
   ```

5. **Check ESC Settings**
   ```
   - Most ESCs need calibration first
   - Some ESCs have brake mode (disable it)
   - Check for low voltage cutoff (set > 10.5V for 3S)
   ```

---

### One Motor Wrong Direction

**Symptoms:**
- Motor spins but in wrong direction
- Drone flips on takeoff
- Unstable flight

**Solutions:**

1. **Swap Any 2 Motor Wires**
   ```
   ESC has 3 wires to motor:
   Black, Red, Yellow (example)
   
   Swap: Red ↔ Yellow
   or:   Black ↔ Red
   or:   Black ↔ Yellow
   ```

2. **Verify Correct Rotation**
   ```
          FRONT
       FL ↻   FR ↺
          \ X /
          / X \
       RL ↺   RR ↻
          REAR
   ```

3. **Test with Motor Test Function**
   ```
   - Remove propellers
   - Press Button 3
   - Observe rotation direction
   ```

---

### Buzzer Not Working

**Symptoms:**
- No startup tone
- No calibration beeps
- No arming confirmation

**Solutions:**

1. **Check Buzzer Type**
   ```
   Active Buzzer:  Has oscillator, needs only DC
   Passive Buzzer: Needs PWM signal
   
   Code uses tone() function - both types work
   ```

2. **Check Polarity**
   ```
   Buzzer (+) → D8
   Buzzer (-) → GND
   
   + is longer pin or marked
   ```

3. **Test with Simple Code**
   ```cpp
   void setup() {
     tone(8, 1000, 1000); // 1kHz for 1 second
   }
   ```

4. **Add Resistor (if too loud)**
   ```
   100Ω resistor in series with buzzer
   ```

---

### LED Not Blinking

**Symptoms:**
- Status LED doesn't light up
- No visual indication of link

**Solutions:**

1. **Check Polarity**
   ```
   D7 → 220Ω resistor → LED (+) anode
                        LED (-) cathode → GND
   
   Longer leg = anode (+)
   ```

2. **Test LED**
   ```
   - Connect LED directly to 5V through resistor
   - Should light up
   ```

3. **Check Code**
   ```cpp
   pinMode(LED_PIN, OUTPUT);
   digitalWrite(LED_PIN, HIGH);
   ```

---

## Software Issues

### Code Won't Compile

**Error: "RF24.h: No such file or directory"**

**Solution:**
```
1. Install RF24 library:
   Sketch → Include Library → Manage Libraries
   Search: "RF24"
   Install: "RF24 by TMRh20"
```

**Error: "MPU6050.h: No such file or directory"**

**Solution:**
```
1. Install MPU6050 library:
   Sketch → Include Library → Manage Libraries
   Search: "MPU6050"
   Install: "MPU6050 by Electronic Cats"
```

**Error: "Sketch too big"**

**Solution:**
```
1. Check board selection:
   Tools → Processor → ATmega328P (not ATmega168)
   
2. Remove debug Serial.print() statements
```

---

### Upload Fails

**Error: "avrdude: stk500_recv(): programmer is not responding"**

**Solutions:**

1. **Try Different Bootloader**
   ```
   Tools → Processor → ATmega328P (Old Bootloader)
   ```

2. **Check COM Port**
   ```
   Tools → Port → Select correct port
   
   Windows: COM3, COM4, etc.
   Mac: /dev/cu.usbserial-...
   Linux: /dev/ttyUSB0
   ```

3. **Reset During Upload**
   ```
   - Press reset button right after upload starts
   - Timing is critical
   ```

4. **Check USB Cable**
   ```
   - Use data cable (not charge-only cable)
   - Try different USB port
   - Try different cable
   ```

---

## Flight Issues

### Drone Oscillates/Shakes

**Symptoms:**
- Rapid shaking when hovering
- Vibrations visible on drone
- High-frequency oscillations

**Solutions:**

1. **Reduce P Gain**
   ```cpp
   #define KP_ANGLE  1.8  // was 2.0
   ```

2. **Increase D Gain**
   ```cpp
   #define KD_ANGLE  18.0  // was 15.0
   ```

3. **Check for Mechanical Issues**
   ```
   - Tighten all screws
   - Check for bent propellers
   - Balance propellers
   - Secure MPU6050 (no vibrations)
   ```

4. **Add Vibration Damping**
   ```
   - Mount FC on foam tape
   - Use soft grommets
   - Isolate MPU6050 from vibrations
   ```

See [PID_TUNING.md](PID_TUNING.md) for detailed instructions.

---

### Drone Drifts

**Symptoms:**
- Slowly moves in one direction
- Won't hold position
- Constant correction needed

**Solutions:**

1. **Recalibrate Gyro**
   ```
   - Place on perfectly flat surface
   - Press Button 1
   - Wait 8 seconds without moving
   - Listen for 2 beeps
   ```

2. **Increase I Gain**
   ```cpp
   #define KI_ANGLE  0.025  // was 0.02
   ```

3. **Check Center of Gravity**
   ```
   - Battery should be centered
   - All components balanced
   - Drone level when suspended
   ```

4. **Check Propellers**
   ```
   - All same pitch
   - All same condition (no damage)
   - Properly tightened
   ```

5. **Check Motor Thrust**
   ```
   - All motors same KV
   - All ESCs calibrated same
   - No damaged motors
   ```

---

### Drone Flips on Takeoff

**Symptoms:**
- Immediate flip when armed
- Can't lift off
- One side drops suddenly

**Solutions:**

1. **Check Motor Directions**
   ```
   Verify rotation:
   FL: ↻  FR: ↺
   RL: ↺  RR: ↻
   
   If wrong, swap any 2 motor wires
   ```

2. **Check Propeller Orientation**
   ```
   CW propellers on CW motors (FR, RL)
   CCW propellers on CCW motors (FL, RR)
   
   Markings face UP
   ```

3. **Check Motor Layout**
   ```
   Verify pins:
   FL → D3
   FR → D5
   RR → D6
   RL → D9
   ```

4. **Test Individual Motors**
   ```
   - Use motor test (Button 3)
   - Verify each motor location
   - Swap connections if wrong
   ```

---

### Sluggish Response

**Symptoms:**
- Slow to respond to sticks
- Feels "mushy"
- Won't tilt much

**Solutions:**

1. **Increase P Gain**
   ```cpp
   #define KP_ANGLE  2.3  // was 2.0
   ```

2. **Increase Max Rates**
   ```cpp
   #define MAX_ROLL_RATE   240.0  // was 180.0
   #define MAX_PITCH_RATE  240.0
   ```

3. **Check Battery Voltage**
   ```
   - Fully charged: 12.6V (3S)
   - Low battery = low power
   ```

4. **Check Throttle Cap**
   ```cpp
   #define THROTTLE_CAP  0.80  // was 0.65
   ```

---

### Gyro Calibration Fails

**Symptoms:**
- Buzzer beeps once for 7 seconds
- Calibration never succeeds
- Drone not stable

**Solutions:**

1. **Ensure Perfectly Flat Surface**
   ```
   - Use level surface (table, floor)
   - Check with spirit level
   - Don't touch during calibration
   ```

2. **Remove Vibrations**
   ```
   - No fans nearby
   - No washing machines running
   - No foot traffic near table
   ```

3. **Check MPU6050**
   ```
   - Securely mounted
   - Not touching any wires
   - Arrow pointing forward
   ```

4. **Try Multiple Times**
   ```
   - Can take 2-3 attempts
   - Let drone sit still for 1 minute first
   ```

---

### Link Keeps Dropping

**Symptoms:**
- Intermittent connection
- "NO LINK" on Serial Monitor
- Drone disarms randomly

**Solutions:**

1. **Improve NRF Power**
   ```
   - Add 10µF capacitor
   - Use external 3.3V regulator
   - Check voltage under load
   ```

2. **Reduce Interference**
   ```
   - Keep away from WiFi routers
   - Change NRF channel (103 → 108)
   - Use shielded wires
   ```

3. **Improve Antennas**
   ```
   - Use NRF24L01+ PA+LNA (long range)
   - Keep antennas vertical
   - Don't block with metal
   ```

4. **Reduce Distance**
   ```
   - Start with 5-10m distance
   - Test range incrementally
   ```

---

### Joystick Values Jittery

**Symptoms:**
- Values jump around in Serial Monitor
- Unstable hover
- Erratic movements

**Solutions:**

1. **Add Capacitors to Joysticks**
   ```
   0.1µF capacitor across each pot:
   VCC to GND
   ```

2. **Increase Deadband**
   ```cpp
   #define STICK_DEADBAND  15  // was 10
   ```

3. **Check Power Supply**
   ```
   - Stable 5V to joysticks
   - No voltage drops
   - Good connections
   ```

4. **Average Readings**
   ```cpp
   // Take multiple samples
   int sum = 0;
   for(int i=0; i<10; i++) {
     sum += analogRead(A0);
   }
   value = sum / 10;
   ```

---

## Emergency Recovery

### Code Upload Failed - Nano Unresponsive

**Recovery:**

1. **Burn Bootloader**
   ```
   - Requires USBasp or Arduino as ISP
   - Tools → Burn Bootloader
   ```

2. **Try Different Arduino IDE Version**
   ```
   - Download Arduino IDE 1.8.19
   - Some bootloaders work better with older versions
   ```

---

### Lost Configuration

**Recovery:**

1. **Re-upload Default Code**
   ```
   - Upload fresh copy from repository
   - Recalibrate gyro
   - Re-tune PID if needed
   ```

---

## Getting Help

If issue persists:

1. Check documentation in `/docs` folder
2. Review code comments
3. Test with minimal configuration (one component at a time)
4. Use multimeter to verify voltages
5. Check for damaged components

---

## Diagnostic Checklist

Use this to systematically troubleshoot:

### Power System
- [ ] Battery voltage > 11.1V
- [ ] 5V rail measures 4.8-5.2V
- [ ] 3.3V rail measures 3.2-3.4V
- [ ] No voltage drops under load
- [ ] All grounds connected

### Communication
- [ ] NRF modules powered correctly
- [ ] SPI connections correct
- [ ] I2C connections correct
- [ ] Capacitors in place
- [ ] No loose wires

### Calibration
- [ ] Gyro calibrated successfully
- [ ] ESCs calibrated
- [ ] Motors spin correct direction
- [ ] Propellers correct orientation

### Software
- [ ] Correct board selected
- [ ] Correct COM port
- [ ] All libraries installed
- [ ] Code compiles without errors
- [ ] Upload successful

---

**Last Updated**: 2025-11-30
