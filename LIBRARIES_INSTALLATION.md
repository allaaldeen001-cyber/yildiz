# Arduino Libraries Installation Guide

## Required Libraries

This quadcopter project requires the following Arduino libraries:

### 1. RF24 Library (Required)
**For:** nRF24L01+ radio communication
**Version:** Latest stable version
**Author:** TMRh20

### 2. Wire Library (Built-in)
**For:** I2C communication with MPU6050
**Version:** Built into Arduino IDE

### 3. SPI Library (Built-in)
**For:** SPI communication with nRF24L01+
**Version:** Built into Arduino IDE

### 4. Servo Library (Built-in)
**For:** ESC control
**Version:** Built into Arduino IDE

## Installation Methods

### Method 1: Arduino IDE Library Manager (Recommended)

1. **Open Arduino IDE**

2. **Go to Library Manager:**
   - Click: `Sketch → Include Library → Manage Libraries...`
   - Or press: `Ctrl+Shift+I` (Windows/Linux) or `Cmd+Shift+I` (Mac)

3. **Install RF24 Library:**
   - In the search box, type: `RF24`
   - Find "RF24 by TMRh20"
   - Click the `Install` button
   - Wait for installation to complete

4. **Verify Installation:**
   - Go to: `Sketch → Include Library`
   - You should see "RF24" in the list

### Method 2: Manual Installation (Advanced)

If Library Manager doesn't work:

1. **Download RF24 Library:**
   - Visit: https://github.com/nRF24/RF24
   - Click: `Code → Download ZIP`

2. **Install ZIP Library:**
   - Arduino IDE: `Sketch → Include Library → Add .ZIP Library...`
   - Navigate to downloaded ZIP file
   - Click `Open`

3. **Restart Arduino IDE**

### Method 3: Git Clone (For Developers)

```bash
cd ~/Arduino/libraries/
git clone https://github.com/nRF24/RF24.git
```

Restart Arduino IDE after cloning.

## Verifying Installation

### Test 1: Include Statements

Create a new sketch and try to compile:

```cpp
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

void setup() {
}

void loop() {
}
```

If it compiles without errors, all libraries are installed correctly.

### Test 2: Check Examples

1. Go to: `File → Examples → RF24`
2. If you see example sketches, RF24 is installed correctly

### Test 3: Library Version

```cpp
#include <RF24.h>

void setup() {
  Serial.begin(115200);
  Serial.println(RF24_VERSION);
}

void loop() {
}
```

Upload this to check the RF24 version.

## Common Installation Issues

### Issue 1: Library Not Found

**Error:**
```
fatal error: RF24.h: No such file or directory
```

**Solution:**
- Restart Arduino IDE after installation
- Check library installed in correct folder: `~/Arduino/libraries/`
- Reinstall using Library Manager

### Issue 2: Multiple Library Versions

**Error:**
```
Multiple libraries were found for "RF24.h"
```

**Solution:**
- Go to Arduino libraries folder
- Delete old RF24 folders
- Keep only the latest version
- Restart Arduino IDE

### Issue 3: Compilation Errors After Update

**Error:**
```
error: 'class RF24' has no member named 'xxx'
```

**Solution:**
- Update to latest RF24 version
- Check compatibility with Arduino IDE version
- Clear Arduino cache: Delete folder at `C:\Users\[User]\AppData\Local\Temp\arduino_*`

### Issue 4: Examples Don't Appear

**Solution:**
- Reinstall RF24 library
- Restart Arduino IDE completely
- Check File → Examples → RF24

## Library Folder Locations

### Windows:
```
C:\Users\[Username]\Documents\Arduino\libraries\
```

### Mac:
```
~/Documents/Arduino/libraries/
```

### Linux:
```
~/Arduino/libraries/
```

## Arduino IDE Setup

### Board Configuration

1. **Select Board:**
   - Tools → Board → Arduino AVR Boards → Arduino Nano

2. **Select Processor:**
   - Tools → Processor → ATmega328P (Old Bootloader)
   - Try "ATmega328P" if upload fails

3. **Select Port:**
   - Tools → Port → COM# (Windows) or /dev/ttyUSB# (Linux) or /dev/cu.usbserial (Mac)

### Upload Settings

- Upload Speed: 57600 (default)
- If upload fails, try 115200 or 9600

## Additional Tools (Optional)

### 1. Serial Monitor
- Built into Arduino IDE
- Tools → Serial Monitor (Ctrl+Shift+M)
- Set baud rate to 115200

### 2. Serial Plotter
- Tools → Serial Plotter (Ctrl+Shift+L)
- Useful for visualizing IMU data
- Set baud rate to 115200

### 3. Board Manager (for other boards)
- Tools → Board → Boards Manager
- Search for specific board packages if needed

## Testing Hardware Connection

After library installation, test hardware:

### Test nRF24L01+ Connection

```cpp
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(4, 10);  // CE, CSN for Flight Controller

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("nRF24L01+ Test");
  
  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    while (1);
  }
  
  Serial.println("Radio initialized successfully!");
}

void loop() {
}
```

### Test MPU6050 Connection

```cpp
#include <Wire.h>

#define MPU6050_ADDR 0x68

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Serial.println("MPU6050 Test");
  
  Wire.beginTransmission(MPU6050_ADDR);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("MPU6050 found!");
  } else {
    Serial.println("MPU6050 not found!");
  }
}

void loop() {
}
```

## Troubleshooting Upload Issues

### "avrdude: stk500_recv(): programmer is not responding"

**Solutions:**
1. Try "ATmega328P (Old Bootloader)"
2. Press reset button during upload
3. Try different upload speed (57600, 115200)
4. Check USB cable (use data cable, not charge-only)
5. Update CH340 drivers (if using clone Nano)

### "Port is not available"

**Solutions:**
1. Close Serial Monitor
2. Close Serial Plotter
3. Unplug and replug USB
4. Check Device Manager (Windows) for COM port
5. Install USB drivers for Arduino

### CH340 Driver Installation (Clone Boards)

**Windows:**
1. Download CH340 drivers: http://www.wch.cn/downloads/CH341SER_ZIP.html
2. Extract and run SETUP.exe
3. Restart computer
4. Check Device Manager for COM port

**Mac:**
1. Download from: https://github.com/adrianmihalko/ch340g-ch34g-ch34x-mac-os-x-driver
2. Install package
3. Restart Mac

**Linux:**
- Usually works without drivers
- If not, install: `sudo apt-get install ch341-dkms`

## Arduino IDE Version Compatibility

| Arduino IDE Version | RF24 Compatibility | Notes |
|---------------------|-------------------|-------|
| 2.x.x (Latest) | ✅ Yes | Recommended |
| 1.8.19 | ✅ Yes | Stable |
| 1.8.13+ | ✅ Yes | Good |
| 1.6.x - 1.8.12 | ⚠️ Maybe | Update recommended |
| <1.6.x | ❌ No | Too old |

### Recommended: Arduino IDE 2.x

Download from: https://www.arduino.cc/en/software

**Advantages:**
- Modern interface
- Better error messages
- Faster compilation
- Auto-complete
- Improved Library Manager

## Quick Reference: All Include Statements

```cpp
// Flight Controller includes:
#include <Wire.h>        // I2C for MPU6050
#include <SPI.h>         // SPI for nRF24L01+
#include <nRF24L01.h>    // nRF24L01+ definitions
#include <RF24.h>        // nRF24L01+ library
#include <Servo.h>       // ESC control

// Remote Controller includes:
#include <SPI.h>         // SPI for nRF24L01+
#include <nRF24L01.h>    // nRF24L01+ definitions
#include <RF24.h>        // nRF24L01+ library
```

## Getting Help

### Documentation Links

- **RF24 Library:** https://nrf24.github.io/RF24/
- **Arduino Reference:** https://www.arduino.cc/reference/en/
- **Wire Library:** https://www.arduino.cc/en/reference/wire
- **Servo Library:** https://www.arduino.cc/en/reference/servo

### Common Questions

**Q: Do I need to install anything for Wire/SPI/Servo?**
A: No, these are built into Arduino IDE.

**Q: Which RF24 library should I use?**
A: Use "RF24 by TMRh20" from Library Manager.

**Q: Can I use Arduino Uno instead of Nano?**
A: Yes, code is compatible. Just check pin mapping.

**Q: What if Library Manager is slow?**
A: Be patient, or use manual ZIP installation method.

**Q: Do I need admin rights to install libraries?**
A: Usually no, but may help if installation fails.

## Verification Checklist

Before uploading flight controller code:

- [ ] RF24 library installed
- [ ] Wire library available (built-in)
- [ ] SPI library available (built-in)
- [ ] Servo library available (built-in)
- [ ] Board set to Arduino Nano
- [ ] Processor set to ATmega328P
- [ ] Correct COM port selected
- [ ] Code compiles without errors

Before uploading remote controller code:

- [ ] RF24 library installed
- [ ] SPI library available (built-in)
- [ ] Board set to Arduino Nano
- [ ] Processor set to ATmega328P
- [ ] Correct COM port selected
- [ ] Code compiles without errors

## Post-Installation Testing

After successful library installation and code upload:

1. **Open Serial Monitor** (Ctrl+Shift+M)
2. **Set baud rate to 115200**
3. **Power on device**
4. **Check for startup messages:**

**Flight Controller should show:**
```
=================================
Quadcopter Flight Controller v2.0
=================================
[OK] Motors initialized
[OK] Radio initialized
[OK] MPU6050 initialized
Calibrating sensors...
[OK] Calibration complete!
System Ready!
```

**Remote Controller should show:**
```
=================================
Quadcopter Remote Controller v2.0
=================================
[OK] Radio initialized
[OK] Calibration complete!
System Ready!
```

If you see these messages, installation is successful! ✅

---

**Next Steps:** See WIRING_DIAGRAMS.md for hardware connections
**Then:** See OPERATION_GUIDE.md for flying instructions
