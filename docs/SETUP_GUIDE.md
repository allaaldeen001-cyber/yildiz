# Complete Setup Guide

Step-by-step instructions to build, program, and fly your Arduino quadcopter drone.

## Table of Contents
1. [Software Setup](#software-setup)
2. [Hardware Assembly](#hardware-assembly)
3. [Wiring](#wiring)
4. [Programming](#programming)
5. [Initial Testing](#initial-testing)
6. [First Flight](#first-flight)

---

## Software Setup

### Step 1: Install Arduino IDE

**Windows:**
```
1. Download from: https://www.arduino.cc/en/software
2. Run installer: arduino-ide_x.x.x_Windows.exe
3. Follow installation wizard
4. Launch Arduino IDE
```

**Mac:**
```
1. Download from: https://www.arduino.cc/en/software
2. Open DMG file
3. Drag Arduino to Applications folder
4. Launch Arduino IDE
```

**Linux:**
```
1. Download from: https://www.arduino.cc/en/software
2. Extract: tar -xvf arduino-ide_x.x.x_Linux_64bit.tar.gz
3. Run: ./arduino-ide
4. Optional: Create desktop shortcut
```

### Step 2: Install CH340G Driver (For Arduino Nano Clones)

Many Arduino Nano clones use CH340G USB chip and need driver.

**Windows:**
```
1. Download from: http://www.wch-ic.com/downloads/CH341SER_EXE.html
2. Run installer: CH341SER.EXE
3. Restart computer
4. Verify: Device Manager → Ports → Should show "USB-SERIAL CH340"
```

**Mac:**
```
1. Download from: https://github.com/adrianmihalko/ch340g-ch34g-ch34x-mac-os-x-driver
2. Install driver package
3. Restart computer
4. System may require security approval in System Preferences
```

**Linux:**
```
Driver usually included in kernel
If not working:
  sudo apt-get install linux-headers-$(uname -r)
  sudo apt-get install dkms
```

### Step 3: Install Arduino Libraries

Open Arduino IDE:

```
Tools → Manage Libraries... (or Ctrl+Shift+I)

Search and install each:

1. RF24 by TMRh20
   → Click "Install"
   → Wait for completion

2. Adafruit MPU6050 by Adafruit
   → Click "Install"
   → Will automatically install dependencies:
     - Adafruit Unified Sensor
     - Adafruit BusIO
   
3. MS5611 by Rob Tillaart
   → Click "Install"
```

**Verify Installation:**
```
Sketch → Include Library

You should see under "Contributed libraries":
✓ RF24
✓ Adafruit MPU6050
✓ Adafruit Unified Sensor
✓ Adafruit BusIO
✓ MS5611
```

### Step 4: Download Project Files

**Option 1: Git Clone**
```bash
git clone https://github.com/[your-repo]/quadcopter-drone.git
cd quadcopter-drone
```

**Option 2: Download ZIP**
```
1. Download ZIP from GitHub
2. Extract to: Documents/Arduino/quadcopter-drone/
```

### Step 5: Configure Arduino IDE

```
Tools → Board → Arduino AVR Boards → Arduino Nano

Tools → Processor → ATmega328P (Old Bootloader)
  Note: If upload fails, try "ATmega328P" (without Old Bootloader)

Tools → Port → [Select your Arduino's COM port]
  Windows: COM3, COM4, etc.
  Mac: /dev/cu.usbserial-xxxxx
  Linux: /dev/ttyUSB0
```

---

## Hardware Assembly

### Part 1: Frame Assembly

```
1. Lay out all frame parts
2. Attach arms to center plate:
   ✓ Front left arm
   ✓ Front right arm  
   ✓ Rear left arm
   ✓ Rear right arm
3. Tighten all screws (don't overtighten!)
4. Frame should be rigid, no wobble
```

### Part 2: Motor Installation

```
For Each Motor:

1. Identify motor position:
   M1 = Front Left  (CCW rotation)
   M2 = Front Right (CW rotation)
   M3 = Rear Right  (CCW rotation)
   M4 = Rear Left   (CW rotation)

2. Mount motor to arm:
   ✓ Use 4 screws per motor
   ✓ Motor wires face inward (toward center)
   ✓ Check motor is flat against arm

3. Label motors with tape:
   Write: "M1 CCW", "M2 CW", etc.
```

### Part 3: ESC Installation

```
For Each ESC:

1. Connect ESC to motor (3 wires, any order initially)

2. Mount ESC to arm:
   ✓ Use zip ties or velcro
   ✓ Keep ESC cool (airflow)
   ✓ Route wires neatly

3. Connect ESC to power distribution:
   ✓ Red wire (+) to PDB positive
   ✓ Black wire (-) to PDB negative
   ✓ Use XT60 or Deans connectors

4. BEC wire management:
   ✓ ESC 1: Keep red wire (BEC)
   ✓ ESC 2-4: CUT red wire (disable BEC)
   ✓ All: Keep black (GND) and white (signal) wires
```

### Part 4: Flight Controller Mounting

```
1. Choose location:
   ✓ Center of frame (balance point)
   ✓ As level as possible
   ✓ Accessible for programming

2. Vibration dampening:
   ✓ Cut foam to size (5mm thick)
   ✓ Stick foam under Arduino Nano
   ✓ Or use rubber standoffs

3. Mount Arduino Nano:
   ✓ USB port accessible
   ✓ Pin headers accessible for wiring
   ✓ Secure with screws or double-sided tape

4. Mark orientation:
   ✓ Arrow pointing forward
   ✓ Ensures correct IMU orientation
```

### Part 5: Sensor Mounting

**MPU6050:**
```
1. Mount near Arduino (short I2C wires)
2. Keep level with flight controller
3. Minimize vibration (use foam)
4. Ensure no flex in mount
```

**MS5611:**
```
1. Mount away from motor airflow
2. No direct propeller wash
3. Protected from debris
4. I2C wires to Arduino
```

**nRF24L01+:**
```
1. Mount away from power wires (RF interference)
2. Antenna pointing up (if external antenna)
3. Accessible for troubleshooting
4. Add 10µF capacitor between VCC and GND pins
```

### Part 6: Power Distribution

```
1. Connect PDB to battery connector:
   ✓ Solder XT60 or Deans connector
   ✓ Red to positive pad
   ✓ Black to negative pad
   ✓ Heat shrink over connections

2. Add main power capacitor:
   ✓ 470µF-1000µF electrolytic capacitor
   ✓ Positive to PDB +
   ✓ Negative to PDB -
   ✓ Reduces voltage spikes

3. Test continuity:
   ✓ Check + rail continuous
   ✓ Check - rail continuous
   ✓ Verify NO short between + and -
```

### Part 7: Battery Mounting

```
1. Choose position (usually bottom of frame):
   ✓ Center of gravity
   ✓ Adjustable forward/back for balance

2. Secure battery:
   ✓ Velcro straps (removable)
   ✓ Double-check: battery won't shift in flight!

3. Balance test:
   ✓ Hold drone by center
   ✓ Should be level (not nose/tail heavy)
   ✓ Adjust battery position if needed
```

---

## Wiring

### Flight Controller I2C Bus

```
Connect MPU6050:
  MPU6050 VCC  → Arduino 3.3V
  MPU6050 GND  → Arduino GND
  MPU6050 SDA  → Arduino A4
  MPU6050 SCL  → Arduino A5

Connect MS5611:
  MS5611 VCC   → Arduino 3.3V (share with MPU6050)
  MS5611 GND   → Arduino GND (share with MPU6050)
  MS5611 SDA   → Arduino A4 (share with MPU6050)
  MS5611 SCL   → Arduino A5 (share with MPU6050)

Wire lengths: <10cm for reliability
Wire gauge: 24-28 AWG
```

### Flight Controller Motor Outputs

```
Connect ESC signals:
  ESC 1 Signal → Arduino D3 (Front Left)
  ESC 2 Signal → Arduino D5 (Front Right)
  ESC 3 Signal → Arduino D6 (Rear Right)
  ESC 4 Signal → Arduino D9 (Rear Left)

Connect ESC grounds:
  All ESC GND  → Arduino GND (common ground!)

Connect ONE ESC BEC:
  ESC 1 BEC +5V → Arduino VIN
  ESC 2-4 BEC: Red wire cut/removed

Wire gauge: 22-24 AWG for signal/ground
```

### Flight Controller Radio

```
Connect nRF24L01+:
  nRF24 CE     → Arduino D4
  nRF24 CSN    → Arduino D10
  nRF24 MOSI   → Arduino D11
  nRF24 MISO   → Arduino D12
  nRF24 SCK    → Arduino D13
  nRF24 VCC    → Arduino 3.3V
  nRF24 GND    → Arduino GND

CRITICAL: Add 10µF capacitor:
  Capacitor +  → nRF24 VCC
  Capacitor -  → nRF24 GND
  Solder directly to nRF24 pins!
```

### Flight Controller Status

```
Connect Buzzer:
  Buzzer +     → Arduino D7
  Buzzer -     → Arduino GND

Connect LED:
  LED Anode    → Arduino D8
  LED Cathode  → 220Ω Resistor → Arduino GND
```

### Remote Controller Joysticks

```
Connect Left Joystick:
  VCC  → Arduino 5V
  GND  → Arduino GND
  VRx  → Arduino A1 (Yaw)
  VRy  → Arduino A0 (Throttle)

Connect Right Joystick:
  VCC  → Arduino 5V
  GND  → Arduino GND
  VRx  → Arduino A3 (Roll)
  VRy  → Arduino A2 (Pitch)
```

### Remote Controller Buttons/Switches

```
Connect Buttons (using internal pull-ups):
  Button 1 → Between D4 and GND
  Button 2 → Between D5 and GND
  Button 3 → Between D6 and GND
  Button 4 → Between D7 and GND

Connect Switches:
  Switch 1 → Between D2 and GND
  Switch 2 → Between D3 and GND

No external resistors needed!
```

### Remote Controller Radio

```
Connect nRF24L01+:
  nRF24 CE     → Arduino D9
  nRF24 CSN    → Arduino D10
  nRF24 MOSI   → Arduino D11
  nRF24 MISO   → Arduino D12
  nRF24 SCK    → Arduino D13
  nRF24 VCC    → Arduino 3.3V
  nRF24 GND    → Arduino GND

Add 10µF capacitor (same as flight controller)
```

### Remote Controller Power

```
9V Battery:
  Red wire    → Arduino VIN
  Black wire  → Arduino GND

Or 3× AA batteries (4.5V):
  Red wire    → Arduino VIN
  Black wire  → Arduino GND

Optional LED:
  LED Anode   → Arduino D13
  LED Cathode → 220Ω → GND
```

---

## Programming

### Upload Flight Controller Code

```
1. Connect Arduino Nano via USB

2. Open Arduino IDE

3. File → Open → FlightController/FlightController.ino

4. Verify board settings:
   Tools → Board → Arduino Nano
   Tools → Processor → ATmega328P (Old Bootloader)
   Tools → Port → [Your COM port]

5. Click "Verify" (checkmark icon)
   Wait for: "Done compiling"
   Check for errors

6. Click "Upload" (right arrow icon)
   Wait for: "Done uploading"

7. Open Serial Monitor:
   Tools → Serial Monitor (or Ctrl+Shift+M)
   Set baud rate: 115200
   
8. Expected output:
   === Flight Controller Starting ===
   Initializing MPU6050...
   MPU6050 connected!
   Initializing MS5611...
   MS5611 connected!
   Initializing nRF24L01+...
   Radio initialized!
   === Ready for Calibration ===

9. If any "failed" messages, check wiring!
```

### Upload Remote Controller Code

```
1. Disconnect flight controller Arduino

2. Connect remote controller Arduino via USB

3. File → Open → RemoteController/RemoteController.ino

4. Verify board settings (same as above)

5. Click "Upload"

6. Open Serial Monitor (115200 baud)

7. Expected output:
   === Remote Controller Starting ===
   Initializing nRF24L01+...
   Radio initialized!
   Calibrating joysticks...
   (Center all sticks now!)
   Calibration complete!
   === Remote Controller Ready ===

8. Move joysticks, press buttons:
   Serial monitor should show value changes
```

---

## Initial Testing

### Test 1: Sensor Communication

**Remove all propellers!**

```
1. Power on flight controller
2. Open Serial Monitor (115200 baud)
3. Check output:
   ✓ MPU6050 connected
   ✓ MS5611 connected
   ✓ Radio initialized

4. Tilt drone:
   ✓ Roll value changes when tilting left/right
   ✓ Pitch value changes when tilting forward/back
   ✓ Values should be near 0° when level

If values wrong:
   → Check MPU6050 orientation
   → Recalibrate on flat surface
```

### Test 2: Radio Communication

```
1. Power on remote controller first
2. Power on flight controller second
3. Check flight controller Serial Monitor:
   Should show received data changing

4. Move joysticks:
   ✓ Throttle: 0-1023
   ✓ Yaw: 0-1023
   ✓ Pitch: 0-1023
   ✓ Roll: 0-1023

5. Press Button 3 (ARM):
   ✓ Serial should show: "ARMED"
   ✓ LED should light up
   ✓ Press again: "DISARMED"

If no data received:
   → Check radio wiring
   → Check 10µF capacitor on nRF24
   → Move remote closer
   → Try different radio channel in code
```

### Test 3: Calibration

```
1. Place drone on perfectly flat surface
2. Keep completely still
3. Press Button 1 (Calibrate) on remote
4. Wait 2-3 seconds (LED blinks during calibration)
5. Buzzer beeps twice: Calibration complete

6. Check Serial Monitor:
   Roll: ~0°
   Pitch: ~0°
   (Should be within ±1°)

If angles not near 0:
   → Surface not flat
   → IMU not mounted level
   → Recalibrate
```

### Test 4: Motor Test (No Props!)

**CRITICAL: Remove ALL propellers first!**

```
1. Disconnect Arduino from all ESCs
2. Connect ONLY ONE ESC to D3
3. Press Button 2 (Motor Test) on remote
4. Motor should spin at low speed
5. Verify rotation direction:
   Motor 1 (D3): Should spin CCW ↺

6. If wrong direction:
   → Swap ANY 2 of the 3 motor wires

7. Repeat for all 4 motors:
   M1 (D3): CCW ↺
   M2 (D5): CW ↻
   M3 (D6): CCW ↺
   M4 (D9): CW ↻
```

### Test 5: ESC Calibration

**For each ESC separately:**

```
1. Disconnect Arduino signal wire from ESC
2. Connect Arduino to computer (USB power only)
3. Open Serial Monitor
4. Type "2000" and press Enter
   (Sends max throttle to ESC)
5. Connect battery to ESC
6. ESC beeps to confirm max throttle
7. Type "1000" and press Enter
   (Sends min throttle)
8. ESC beeps differently: Calibration done
9. Disconnect battery
10. Reconnect Arduino signal wire
11. Repeat for all 4 ESCs
```

### Test 6: Armed Motor Response

**Still without propellers!**

```
1. Power on remote, then drone
2. Press Button 1 (Calibrate)
3. Ensure throttle is down
4. Press Button 3 (ARM)
5. Slowly increase throttle:
   ✓ All 4 motors should spin up together
   ✓ Should respond smoothly to throttle
6. Lower throttle to minimum
7. Press Button 3 (DISARM)

If motors don't spin:
   → Check ESC connections
   → Verify calibration
   → Check battery voltage (>11V)
```

---

## First Flight

### Pre-Flight Checklist

```
☐ Battery fully charged (12.6V)
☐ All propellers installed CORRECTLY:
  → M1 (FL): CCW propeller ↺
  → M2 (FR): CW propeller ↻
  → M3 (RR): CCW propeller ↺
  → M4 (RL): CW propeller ↻
☐ All screws tight (motors, frame, flight controller)
☐ No loose wires
☐ Flight area clear (outdoor, no obstacles)
☐ Wind < 10 mph
☐ Safety glasses on
☐ Battery strap secure
☐ Remote controller battery good
```

### Flight Procedure

**Step 1: Power-Up Sequence**
```
1. Power on remote controller first
2. Check joysticks centered (Serial Monitor)
3. Place drone on flat ground
4. Connect battery to drone
5. Wait for 3 beeps (initialization)
6. LED should blink slowly (ready)
```

**Step 2: Calibration**
```
1. Drone must be perfectly level
2. Press Button 1 (Calibrate)
3. Don't move drone!
4. Wait for 2 beeps (done)
5. LED should be solid or slow blink
```

**Step 3: Arming**
```
1. Ensure throttle stick fully down
2. Stand back 3 meters
3. Press Button 3 (ARM)
4. Long beep confirms
5. LED solid on
6. Motors are now armed (props will spin!)
```

**Step 4: Takeoff**
```
1. Slowly increase throttle (left stick up)
2. At ~40-50%, drone should lift off
3. Hold throttle steady for hover
4. Keep movements SMALL at first

Tips for first hover:
  → Relax! Small corrections only
  → Release sticks → drone self-levels (ANGLE mode)
  → If drifting, trim with right stick
  → Stay calm, be ready to disarm if needed
```

**Step 5: Basic Maneuvers**
```
Once hovering stable:

1. Forward/Back:
   Right stick up/down (pitch)

2. Left/Right:
   Right stick left/right (roll)

3. Rotate:
   Left stick left/right (yaw)

4. Altitude:
   Left stick up/down (throttle)

Start with TINY movements!
```

**Step 6: Landing**
```
Manual Landing:
1. Position drone above landing spot
2. Slowly reduce throttle
3. Gentle touchdown
4. Throttle to minimum
5. Motors auto-disarm

Emergency Landing:
1. Press Button 4 (Soft Landing)
2. Drone descends automatically
3. Hands off sticks!
4. Auto-disarms on ground
```

### Emergency Procedures

**If losing control:**
```
Press Button 3 (DISARM) immediately
Drone will fall and may break
Better than uncontrolled crash!
```

**If radio signal lost:**
```
Automatic failsafe activates:
→ Buzzer beeps continuously
→ Throttle reduces to 30%
→ Slow descent
→ Move closer to regain signal
```

**If drone tilts >45°:**
```
Automatic safety disarm:
→ Motors cut immediately
→ Prevents runaway
→ Check motor directions/props
```

---

## Post-Flight

### After Each Flight

```
☐ Disconnect battery immediately
☐ Check propellers for damage
☐ Check for loose screws
☐ Check motor temperatures (should be warm, not hot)
☐ Note battery voltage (should be >10.5V)
☐ Log flight time and issues
```

### Maintenance Schedule

**After Every 10 Flights:**
```
☐ Tighten all screws
☐ Check for frame cracks
☐ Clean propellers
☐ Check motor bearings (spin by hand)
☐ Inspect wiring for wear
```

**Monthly:**
```
☐ Balance propellers
☐ Recalibrate ESCs
☐ Clean sensors (compressed air)
☐ Check solder joints
☐ Update PID gains if needed
```

---

## Tuning for Better Performance

### PID Tuning (After successful first flights)

**If drone oscillates (shakes):**
```cpp
// Reduce P gain
float Kp_roll = 1.0;  // Was 1.5
float Kp_pitch = 1.0; // Was 1.5

// OR increase D gain
float Kd_roll = 1.2;  // Was 0.8
float Kd_pitch = 1.2; // Was 0.8
```

**If drone drifts:**
```cpp
// Increase I gain
float Ki_roll = 0.08;  // Was 0.05
float Ki_pitch = 0.08; // Was 0.05
```

**If response sluggish:**
```cpp
// Increase P gain
float Kp_roll = 2.0;  // Was 1.5
float Kp_pitch = 2.0; // Was 1.5
```

### Complementary Filter Tuning

**If angles drift over time:**
```cpp
// Trust accelerometer more
const float ALPHA = 0.96; // Was 0.98
```

**If angles jittery:**
```cpp
// Trust gyro more
const float ALPHA = 0.99; // Was 0.98
```

---

## Upgrading Features

### Add GPS (Future Enhancement)
```
1. Get GPS module (Neo-6M or similar)
2. Connect to Serial pins (RX/TX)
3. Add GPS library
4. Implement position hold
5. Return-to-home feature
```

### Add FPV Camera
```
1. Get 5.8GHz camera + transmitter
2. Mount on front of drone
3. Get FPV goggles or screen
4. Adjust camera angle (30-45°)
5. First-person view flying!
```

### Add Altitude Hold (Using MS5611)
```cpp
// In code, enable altitude hold:
if (button_pressed) {
  altitudeHold = true;
  targetAltitude = currentAltitude;
}

// PID will maintain height automatically
```

---

## Congratulations!

You've successfully built a quadcopter drone from scratch! 🚁

**Next Steps:**
- Practice flying in open areas
- Experiment with PID tuning
- Try ACRO mode (advanced)
- Add features (GPS, FPV, etc.)
- Join drone communities online
- Share your build!

**Fly safe and have fun!**
