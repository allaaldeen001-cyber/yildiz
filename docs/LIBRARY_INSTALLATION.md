# Library Installation Guide

Complete guide to installing Arduino libraries for the quadcopter drone project.

## Required Libraries

This project uses the following libraries:

### 1. RF24 by TMRh20
- **Purpose:** Wireless communication via nRF24L01+
- **Version:** 1.4.7 or later
- **Install from:** Arduino Library Manager

### 2. Adafruit MPU6050 by Adafruit
- **Purpose:** Read gyroscope and accelerometer data
- **Version:** 2.2.4 or later
- **Install from:** Arduino Library Manager
- **Dependencies:** Automatically installs:
  - Adafruit Unified Sensor
  - Adafruit BusIO

### 3. MS5611 by Rob Tillaart
- **Purpose:** Barometric pressure sensor for altitude
- **Version:** 0.4.2 or later
- **Install from:** Arduino Library Manager

---

## Installation Methods

### Method 1: Library Manager (Recommended) ⭐

**Step-by-Step:**

```
1. Open Arduino IDE

2. Go to: Sketch → Include Library → Manage Libraries
   (Or press Ctrl+Shift+I / Cmd+Shift+I)

3. Library Manager window opens

4. Install RF24:
   ├─ Type "RF24" in search box
   ├─ Find "RF24 by TMRh20"
   ├─ Click "Install" button
   └─ Wait for "Installed" confirmation

5. Install Adafruit MPU6050:
   ├─ Type "Adafruit MPU6050" in search box
   ├─ Find "Adafruit MPU6050 by Adafruit"
   ├─ Click "Install" button
   ├─ Popup: "Install library dependencies?"
   ├─ Click "Install All" (installs Adafruit Sensor + BusIO)
   └─ Wait for completion

6. Install MS5611:
   ├─ Type "MS5611" in search box
   ├─ Find "MS5611 by Rob Tillaart"
   ├─ Click "Install" button
   └─ Wait for "Installed" confirmation

7. Close Library Manager

8. Verify installation:
   └─ Sketch → Include Library → See list of installed libraries
```

---

### Method 2: Manual ZIP Installation

If Library Manager doesn't work, install from ZIP files:

**Download Links:**
- RF24: https://github.com/nRF24/RF24/archive/refs/heads/master.zip
- Adafruit MPU6050: https://github.com/adafruit/Adafruit_MPU6050/archive/refs/heads/master.zip
- Adafruit Sensor: https://github.com/adafruit/Adafruit_Sensor/archive/refs/heads/master.zip
- Adafruit BusIO: https://github.com/adafruit/Adafruit_BusIO/archive/refs/heads/master.zip
- MS5611: https://github.com/RobTillaart/MS5611/archive/refs/heads/master.zip

**Installation Steps:**

```
1. Download all ZIP files above

2. In Arduino IDE:
   Sketch → Include Library → Add .ZIP Library...

3. Select each ZIP file one at a time

4. Arduino will extract and install to libraries folder

5. Restart Arduino IDE after installing all

6. Verify: Sketch → Include Library → See installed libraries
```

---

### Method 3: Manual Folder Installation

For advanced users:

**Libraries Folder Location:**
- **Windows:** `C:\Users\[YourName]\Documents\Arduino\libraries\`
- **Mac:** `~/Documents/Arduino/libraries/`
- **Linux:** `~/Arduino/libraries/`

**Steps:**

```
1. Download libraries from GitHub (as ZIP)

2. Extract each ZIP file

3. Rename folders (remove "-master" suffix):
   ├─ RF24-master → RF24
   ├─ Adafruit_MPU6050-master → Adafruit_MPU6050
   ├─ Adafruit_Sensor-master → Adafruit_Sensor
   ├─ Adafruit_BusIO-master → Adafruit_BusIO
   └─ MS5611-master → MS5611

4. Copy folders to Arduino libraries folder

5. Restart Arduino IDE

6. Verify installation
```

---

## Verifying Installation

### Check Installed Libraries

```
Arduino IDE → Sketch → Include Library

Look for these under "Contributed libraries":
✅ RF24
✅ Adafruit MPU6050
✅ Adafruit Unified Sensor
✅ Adafruit BusIO
✅ MS5611
```

### Test Compilation

```
1. Open FlightController/FlightController.ino

2. Click "Verify" (checkmark icon)

3. Wait for compilation

4. Success message: "Done compiling"
   ├─ Sketch uses XXX bytes of program storage
   └─ Global variables use XXX bytes of dynamic memory

5. If errors → See troubleshooting below
```

---

## Troubleshooting

### Error: "Library not found"

**Symptoms:**
```
fatal error: Adafruit_MPU6050.h: No such file or directory
```

**Solutions:**
1. Check library is installed: Sketch → Include Library
2. Library name must match exactly (case-sensitive)
3. Restart Arduino IDE
4. Reinstall library using Library Manager

---

### Error: "Multiple libraries found"

**Symptoms:**
```
WARNING: library RF24 claims to run on avr architecture(s)...
```

**Solutions:**
1. Check libraries folder for duplicates
2. Remove old/duplicate versions
3. Keep only one version of each library
4. Restart Arduino IDE

---

### Error: "Dependencies not installed"

**Symptoms:**
```
fatal error: Adafruit_Sensor.h: No such file or directory
```

**Solutions:**
1. Install Adafruit MPU6050 again via Library Manager
2. When prompted "Install dependencies?" → Click "Install All"
3. Manually install missing dependencies:
   - Adafruit Unified Sensor
   - Adafruit BusIO

---

### Error: "Not enough memory"

**Symptoms:**
```
Sketch too big; see http://www.arduino.cc/en/Guide/Troubleshooting#size
```

**Solutions:**
1. This is normal for Arduino Nano (limited 32KB flash)
2. Remove debug Serial.print() statements
3. Use F() macro for strings:
   ```cpp
   Serial.println(F("Text here"));  // Stores in flash, not RAM
   ```
4. Current project uses ~18KB (should fit fine)

---

### Error: "Library compilation errors"

**Symptoms:**
```
error: 'class Adafruit_MPU6050' has no member named 'begin'
```

**Solutions:**
1. Update to latest library version
2. Check Arduino IDE version (need 1.8.x or 2.x)
3. Try alternative library (see below)

---

## Library Versions Tested

These versions are confirmed working:

| Library | Version | Date Tested |
|---------|---------|-------------|
| RF24 | 1.4.7+ | Dec 2025 |
| Adafruit MPU6050 | 2.2.4+ | Dec 2025 |
| Adafruit Sensor | 1.1.14+ | Dec 2025 |
| Adafruit BusIO | 1.14.5+ | Dec 2025 |
| MS5611 | 0.4.2+ | Dec 2025 |

**Note:** Newer versions should also work. Update libraries if needed.

---

## Alternative Libraries

If you have issues with the recommended libraries:

### Alternative MPU6050 Library

**MPU6050_tockn by tockn**
- Simpler API
- Smaller code size
- Less features

**To use:**
```cpp
// Replace in code:
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// With:
#include <MPU6050_tockn.h>

// Update initialization code accordingly
```

---

## Library File Structure

After installation, your libraries folder should look like:

```
Arduino/libraries/
├── Adafruit_BusIO/
│   ├── Adafruit_BusIO_Register.h
│   ├── Adafruit_I2CDevice.h
│   └── ...
├── Adafruit_MPU6050/
│   ├── Adafruit_MPU6050.h
│   ├── Adafruit_MPU6050.cpp
│   └── examples/
├── Adafruit_Sensor/
│   ├── Adafruit_Sensor.h
│   └── ...
├── MS5611/
│   ├── MS5611.h
│   ├── MS5611.cpp
│   └── examples/
└── RF24/
    ├── RF24.h
    ├── RF24.cpp
    └── ...
```

---

## Understanding Library Dependencies

### Dependency Tree

```
FlightController.ino
├─ Wire (built-in)
├─ Servo (built-in)
├─ SPI (built-in)
├─ RF24
│  └─ SPI
├─ Adafruit_MPU6050
│  ├─ Adafruit_Sensor
│  └─ Adafruit_BusIO
│     ├─ Wire
│     └─ SPI
└─ MS5611
   └─ Wire
```

**Key Points:**
- Built-in libraries (Wire, Servo, SPI) need no installation
- Adafruit MPU6050 requires 2 additional libraries
- All libraries use I2C (Wire) or SPI for communication

---

## Library Usage in Code

### How Libraries Are Included

```cpp
// In FlightController.ino:

// Built-in libraries
#include <Wire.h>           // I2C communication
#include <Servo.h>          // ESC control
#include <SPI.h>            // SPI for radio

// External libraries (must install)
#include <RF24.h>                // nRF24L01+ radio
#include <Adafruit_MPU6050.h>    // MPU6050 IMU
#include <Adafruit_Sensor.h>     // Sensor framework
#include <MS5611.h>              // MS5611 barometer
```

### Creating Library Objects

```cpp
// Create hardware objects:
Adafruit_MPU6050 mpu;       // IMU sensor
MS5611 ms5611;              // Barometer
RF24 radio(CE_PIN, CSN_PIN);// Radio module
```

---

## Updating Libraries

### Check for Updates

```
1. Sketch → Include Library → Manage Libraries

2. Filter: "Updatable" (top dropdown)

3. Update available libraries:
   └─ Click "Update" button for each

4. Restart Arduino IDE
```

### When to Update

✅ **Update when:**
- Bug fixes available
- New features needed
- Security patches released
- Compatibility issues

❌ **Don't update if:**
- Current version works perfectly
- Breaking changes in new version
- Project deadline approaching (test first!)

---

## Library Examples

Each library includes example sketches:

### Viewing Examples

```
File → Examples → [Library Name]

Useful examples:
├─ RF24 → GettingStarted
├─ Adafruit MPU6050 → basic_readings
└─ MS5611 → MS5611_test
```

### Testing Individual Libraries

**Test MPU6050:**
```
File → Examples → Adafruit MPU6050 → basic_readings
Upload and open Serial Monitor (115200 baud)
Should show gyro and accel readings
```

**Test MS5611:**
```
File → Examples → MS5611 → MS5611_test
Upload and open Serial Monitor (115200 baud)
Should show pressure and temperature
```

**Test RF24:**
```
File → Examples → RF24 → GettingStarted
Need 2 Arduinos (transmitter + receiver)
Follow example instructions
```

---

## Common Issues Summary

| Issue | Cause | Solution |
|-------|-------|----------|
| Library not found | Not installed | Use Library Manager to install |
| Compilation errors | Wrong version | Update library or use alternative |
| Out of memory | Too many libraries | Remove unused, optimize code |
| Duplicate libraries | Multiple versions | Remove old versions |
| Missing dependencies | Incomplete install | Reinstall with "Install All" |

---

## Getting Help

### If Installation Fails

1. **Check Arduino IDE version:**
   - Help → About Arduino
   - Need version 1.8.x or 2.x

2. **Check internet connection:**
   - Library Manager needs internet to download

3. **Clear library cache:**
   ```
   Close Arduino IDE
   Delete: [Arduino]/libraries/.tmp/
   Restart Arduino IDE
   ```

4. **Reinstall Arduino IDE:**
   - Last resort if persistent issues
   - Backup your sketches first!

### Support Resources

- **Adafruit Forums:** https://forums.adafruit.com/
- **Arduino Forums:** https://forum.arduino.cc/
- **RF24 GitHub:** https://github.com/nRF24/RF24/issues
- **MS5611 GitHub:** https://github.com/RobTillaart/MS5611/issues

---

## Quick Reference

### Installation Checklist

```
☐ Arduino IDE installed (1.8.x or 2.x)
☐ Opened Library Manager
☐ Installed RF24 by TMRh20
☐ Installed Adafruit MPU6050 (with dependencies)
☐ Installed MS5611 by Rob Tillaart
☐ Verified libraries appear in menu
☐ Test compiled FlightController.ino
☐ No errors → Ready to upload!
```

### Library Manager Shortcuts

| Action | Windows/Linux | Mac |
|--------|---------------|-----|
| Open Library Manager | Ctrl+Shift+I | Cmd+Shift+I |
| Search | Type in box | Type in box |
| Install | Click Install | Click Install |
| Update | Click Update | Click Update |

---

**Installation complete! You're ready to build the quadcopter! 🚁**
