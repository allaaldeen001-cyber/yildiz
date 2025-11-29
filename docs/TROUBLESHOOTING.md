# Troubleshooting Guide

Comprehensive troubleshooting for the Professional Arduino Nano Drone System.

---

## 🔍 Diagnostic Tools

### Serial Monitor
- **Remote Controller**: Real-time status display at 115200 baud
- **Flight Controller**: Debug messages at 115200 baud

### Visual Indicators
- **Status LED (D7)**: Communication and state indication
  - Fast blink (200ms): Not calibrated
  - Slow blink (500ms): Calibrated, receiving data
  - Solid ON: Armed
  - OFF: No power or no communication

### Audio Indicators
- **1 short beep**: Power on
- **2 short beeps**: System ready / Calibration success
- **3 short beeps**: System armed
- **1 long beep (7s)**: Calibration failed

---

## ❌ Common Problems and Solutions

### 1. No Communication Between FC and RC

**Symptoms:**
- Serial monitor shows "✗ NO SIGNAL"
- Status LED not blinking or off
- No telemetry data received

**Possible Causes and Solutions:**

| Cause | Check | Solution |
|-------|-------|----------|
| **NRF24L01 not powered** | Measure 3.3V at NRF VCC pin | - Verify 3.3V connection<br>- Check Arduino 3.3V regulator<br>- Use external 3.3V regulator if needed |
| **Missing capacitor** | Check for 10μF cap on NRF | **Add 10μF capacitor between VCC and GND on BOTH NRF modules** |
| **Wrong pin connections** | Verify CE and CSN pins | FC: CE=D4, CSN=D10<br>RC: CE=D9, CSN=D10 |
| **SPI pins incorrect** | Check MOSI/MISO/SCK | MOSI=D11, MISO=D12, SCK=D13 (both) |
| **Different channels** | Verify NRF_CHANNEL in code | Both must be 103 (line 37 in both files) |
| **Bad NRF module** | Swap modules between boards | If problem follows module, replace it |
| **Antenna damaged** | Inspect PA+LNA antenna | Ensure antenna not broken, properly connected |

**Quick Test:**
```cpp
// Add to setup() in both sketches to verify NRF detection
if (!radio.begin()) {
  Serial.println("NRF24L01 FAILED!");
  while(1);  // Halt
}
Serial.println("NRF24L01 OK");
```

---

### 2. Calibration Fails (1 Long Beep)

**Symptoms:**
- Press Button_1
- Drone beeps once for 7 seconds
- Serial shows "Calibration FAILED"

**Possible Causes:**

| Cause | Solution |
|-------|----------|
| **Drone not on level surface** | Place on flat, stable table (not carpet) |
| **Vibration during calibration** | Ensure no fans, AC, or movement nearby<br>Don't touch table during calibration |
| **MPU6050 loose connection** | Check I2C wiring (SDA=A4, SCL=A5)<br>Verify INT pin connection (D2) |
| **MPU6050 faulty** | Test with I2C scanner sketch<br>Should show address 0x68 |
| **Excessive drift** | MPU6050 may be damaged<br>Try cooling (may be overheating) |

**Verification:**
```cpp
// In Flight Controller setup(), add diagnostic:
Wire.beginTransmission(MPU_ADDRESS);
byte error = Wire.endTransmission();
if (error == 0) {
  Serial.println("MPU6050 detected OK");
} else {
  Serial.println("MPU6050 NOT FOUND!");
}
```

---

### 3. Motors Don't Spin

**Symptoms:**
- System arms (3 beeps)
- Increase throttle
- Motors silent, no movement

**Troubleshooting Steps:**

#### Step 1: Check Arming
- [ ] Kill Switch (SW_2) in "ARMED" position?
- [ ] Button_2 pressed to arm?
- [ ] Serial shows "Status: ⚠ ARMED & FLYING"?
- [ ] Status LED solid ON?

#### Step 2: Check ESC Power
- [ ] Battery connected to ESCs?
- [ ] Battery voltage >10.5V?
- [ ] ESC beeping (indicates no signal)?

#### Step 3: Check Signal Wires
- [ ] FL signal wire to D3?
- [ ] FR signal wire to D5?
- [ ] RR signal wire to D6?
- [ ] RL signal wire to D9?
- [ ] ESC ground connected to Arduino GND?

#### Step 4: Test Individual Motor
```cpp
// In setup(), add manual test (REMOVE PROPS FIRST!):
void setup() {
  // ... existing code ...
  
  // Manual motor test
  pinMode(MOTOR_FL_PIN, OUTPUT);
  Serial.println("Testing FL motor in 3 seconds...");
  delay(3000);
  
  for(int i = 0; i < 100; i++) {
    analogWrite(MOTOR_FL_PIN, 100);  // Low speed
    delay(20);
  }
  analogWrite(MOTOR_FL_PIN, 0);
  Serial.println("Test complete");
}
```

#### Step 5: ESC Calibration
- Perform ESC calibration sequence (see CALIBRATION_GUIDE.md)
- Each ESC must learn the throttle range

---

### 4. One Motor Doesn't Work

**Symptoms:**
- Three motors spin normally
- One motor silent or erratic

**Diagnosis:**

| Test | Procedure | If Fails |
|------|-----------|----------|
| **Swap signal wire** | Swap working motor signal with dead motor | If problem follows wire → Bad Arduino pin<br>If problem stays → Bad ESC/motor |
| **Swap ESC** | Replace ESC with known working one | If works → Bad ESC<br>If fails → Bad motor |
| **Check motor** | Spin motor by hand | Should spin freely<br>If stiff → Damaged motor |
| **ESC beeps** | Power ESC without signal wire | Should beep error tone<br>No beep → ESC dead |

**Common Fixes:**
- Resolder signal wire connection
- Replace damaged ESC
- Replace motor if bearing seized

---

### 5. Drone Oscillates/Vibrates in Flight

**Symptoms:**
- Drone shakes rapidly when trying to hover
- Wobbles back and forth
- Cannot maintain stable hover

**Causes and Solutions:**

| Type | Description | Solution |
|------|-------------|----------|
| **High-frequency oscillation** | Rapid shaking, buzzing sound | **P gain too high**<br>Reduce PID_ROLL_KP and PID_PITCH_KP by 0.2<br>Try: 1.1 instead of 1.3 |
| **Low-frequency wobble** | Slow rocking motion | **D gain too low**<br>Increase PID_ROLL_KD and PID_PITCH_KD by 2<br>Try: 20.0 instead of 18.0 |
| **Toilet bowling** | Spiraling when hovering | **Gyro not calibrated**<br>Recalibrate on level surface<br>Check for vibration isolation |
| **Drifts in one direction** | Continuously moves forward/side | **IMU not level**<br>Recalibrate<br>Check frame is straight |

**PID Tuning Process:**
1. Start with Roll/Pitch Kp = 0.8, Ki = 0, Kd = 0
2. Increase Kp until oscillation starts
3. Reduce Kp by 20%
4. Add Kd to dampen (start at 15)
5. Add small Ki for drift correction (0.04)

---

### 6. Wrong Motor Spins or Wrong Direction

**Symptoms:**
- Pitch/roll inputs cause unexpected movement
- Drone flips immediately on takeoff
- Yaw rotates opposite direction

**Motor Configuration Check:**

```
Correct Layout (X configuration):
        FRONT
    FL ↻    ↺ FR
      \    /
       \  /
        \/
        /\
       /  \
      /    \
    RL ↺    ↻ RR
        REAR

↻ = CCW (Counter-Clockwise)
↺ = CW (Clockwise)
```

**Fix Wrong Direction:**
- Swap any TWO of the three motor wires
- Or change ESC direction setting (if BLHeli)

**Fix Wrong Motor:**
- Verify signal wire connections:
  - Front-Left → D3
  - Front-Right → D5
  - Rear-Right → D6
  - Rear-Left → D9

---

### 7. Drone Flips on Takeoff

**Symptoms:**
- Increase throttle
- Drone immediately flips over
- Cannot get off ground

**Causes:**

| Cause | Check | Fix |
|-------|-------|-----|
| **Props backwards** | Check leading edge faces forward | Flip propellers or rotate 180° |
| **Props on wrong motor** | CCW props on CW motors or vice versa | Swap propellers to correct motors |
| **Motor direction wrong** | One or more motors spinning wrong way | Swap two motor wires on each wrong motor |
| **Severe PID issues** | Too aggressive P gain | Reduce P gains to 0.5, test gently |
| **IMU backwards** | MPU6050 mounted upside down | Correct orientation or invert in code |

**Quick Test (NO PROPS!):**
1. Arm system
2. Tilt drone forward (pitch)
3. Rear motors should speed up (to correct)
4. Front motors should slow down
5. If opposite → motor mixer is wrong

---

### 8. Random Disconnections

**Symptoms:**
- Connection drops intermittently
- "NO SIGNAL" appears randomly
- Drone disarms unexpectedly

**Causes:**

| Cause | Solution |
|-------|----------|
| **Interference** | - Move away from WiFi routers<br>- Avoid 2.4GHz devices<br>- Try different NRF channel (e.g., 108, 115) |
| **Low power to NRF** | - Verify 3.3V stable under load<br>- Add larger capacitor (100μF)<br>- Use external 3.3V regulator |
| **Antenna issues** | - Ensure antennas not touching metal<br>- Keep antennas perpendicular<br>- Check for damage |
| **Range too far** | - PA+LNA range is ~500m open area<br>- Obstacles reduce range<br>- Stay closer during testing |
| **Vibration** | - NRF module vibrating loose<br>- Secure with hot glue or mount on foam |

---

### 9. Serial Monitor Shows Garbage

**Symptoms:**
- Random characters instead of text
- Unreadable output
- Symbols and boxes

**Solutions:**
- **Check baud rate**: Must be 115200
- **Check board selection**: Should be "Arduino Nano"
- **Check processor**: ATmega328P (Old Bootloader) vs new
- **Try different USB cable**: Some are power-only
- **Reset Arduino**: Press reset button, reopen serial monitor

---

### 10. Joystick Values Incorrect

**Symptoms:**
- Throttle not at minimum when stick down
- Centered sticks show values far from 1500
- Maximum values don't reach 2000

**Diagnosis:**

Open RC serial monitor, check CONTROL INPUTS:

| Stick Position | Expected Value | If Wrong |
|----------------|----------------|----------|
| Throttle DOWN | 1000-1050 | Check A0 connection |
| Throttle UP | 1950-2000 | Check A0 connection |
| Yaw CENTER | 1480-1520 | Adjust STICK_DEADBAND |
| Yaw LEFT | 1000-1050 | Check A1, may need swap |
| Yaw RIGHT | 1950-2000 | Check A1 |
| Pitch/Roll similar | 1000/1500/2000 | Check A2, A3 |

**Fixes:**
- Clean joystick potentiometers (contact cleaner)
- Replace faulty joystick
- Adjust deadband in code (line 45, RemoteController.ino)
- Verify 5V power to joystick
- Check for loose GND connection

---

### 11. ESC Keeps Beeping

**Different Beep Patterns:**

| Beep Pattern | Meaning | Solution |
|--------------|---------|----------|
| Continuous tone | No signal detected | Check signal wire connection to Arduino |
| 3 beeps repeating | Low voltage | Charge/replace battery |
| Musical tones | Startup sequence | Normal, wait for completion |
| Single beep every 2s | Signal OK, waiting for throttle | Normal when armed at low throttle |

---

### 12. Battery Drains Quickly

**Expected Flight Time:**
- 1500mAh: ~5-7 minutes
- 2200mAh: ~8-12 minutes

**If Much Shorter:**

| Cause | Solution |
|-------|----------|
| **Battery aged** | Replace with fresh LiPo |
| **One motor working harder** | Check frame is symmetrical, props balanced |
| **ESC damaged** | Test current draw, replace if excessive |
| **Full throttle flying** | Fly gently for longer flight time |
| **Cold weather** | LiPo performs poorly in cold, warm before use |

---

## 🔧 Advanced Diagnostics

### I2C Scanner (for MPU6050)

Upload this to Flight Controller to verify MPU6050:

```cpp
#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("I2C Scanner");
}

void loop() {
  byte error, address;
  int devices = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Device at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devices++;
    }
  }
  
  if (devices == 0)
    Serial.println("No I2C devices found");
  
  delay(5000);
}
```

Expected output: `Device at 0x68` (MPU6050)

### NRF24L01 Test

Basic ping-pong test to verify NRF modules work:

```cpp
// Transmitter test code
#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);  // CE, CSN
const byte address[6] = "00001";

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.setChannel(103);
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
}

void loop() {
  const char text[] = "Hello";
  bool ok = radio.write(&text, sizeof(text));
  Serial.println(ok ? "Sent OK" : "Send FAILED");
  delay(1000);
}
```

---

## 📊 Debugging Checklist

When something goes wrong, work through this checklist:

### Hardware:
- [ ] All connections tight and soldered
- [ ] No short circuits (multimeter continuity test)
- [ ] Battery voltage >10.5V (3S) or >14V (4S)
- [ ] 10μF capacitor on BOTH NRF modules
- [ ] MPU6050 responds to I2C scanner
- [ ] Each ESC beeps on power-up
- [ ] No damaged propellers

### Software:
- [ ] Correct board selected (Arduino Nano)
- [ ] Correct processor (ATmega328P)
- [ ] Upload successful (no errors)
- [ ] Serial monitor at 115200 baud
- [ ] Both sketches use same NRF_CHANNEL (103)
- [ ] Library versions compatible (RF24 by TMRh20)

### Calibration:
- [ ] Gyro calibration successful (2 beeps)
- [ ] ESC calibration done (if new ESCs)
- [ ] Level surface during calibration
- [ ] No vibration during calibration

### Pre-Flight:
- [ ] Communication link active
- [ ] All joystick axes responding
- [ ] Switches changing states in serial monitor
- [ ] Motor test passed (NO PROPS!)
- [ ] Propellers correct orientation (WITH PROPS)

---

## 🆘 Last Resort: Factory Reset

If all else fails, start from scratch:

1. **Re-upload firmware** to both Arduinos
2. **Clear EEPROM** (if used)
3. **Redo all wiring** from scratch (check against diagrams)
4. **Test components individually** (motors, ESCs, sensors)
5. **Replace suspected bad components** one at a time
6. **Start with basic hover test** before attempting maneuvers

---

## 📞 Getting Help

When asking for help, provide:

1. **Detailed description** of the problem
2. **Serial monitor output** (copy/paste)
3. **When it occurs** (startup, arming, flight, etc.)
4. **What changed** since last working state
5. **Photos of wiring** (clear, well-lit)
6. **Component list** (ESC type, motor KV, battery specs)
7. **Any modifications** to code or hardware

---

## ⚡ Safety Reminders

- **ALWAYS** remove propellers when testing/debugging
- **NEVER** arm with propellers installed unless ready to fly
- Keep fingers away from spinning motors/props
- Disconnect battery when working on drone
- Use LiPo safety bag for charging/storage
- Have fire extinguisher nearby when charging

---

**Remember**: Most problems are simple wiring errors or missing capacitors. Double-check everything before assuming a component is bad!

