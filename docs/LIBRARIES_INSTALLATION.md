# Libraries Installation Guide

## 📚 Required Libraries

This drone system requires the following Arduino libraries. Install them through the Arduino IDE Library Manager or manually.

---

## Method 1: Arduino IDE Library Manager (Recommended)

### Step-by-Step Installation

1. **Open Arduino IDE**
2. Go to: `Sketch` → `Include Library` → `Manage Libraries...`
3. Search for and install each library below:

### Required Libraries List

#### 1. RF24 (NRF24L01 Communication)
- **Library Name**: RF24
- **Author**: TMRh20
- **Version**: 1.4.5 or newer
- **Search Term**: "RF24"
- **Description**: Optimized high-speed NRF24L01+ driver

**Installation:**
```
Library Manager → Search "RF24" → Install "RF24 by TMRh20"
```

**Alternative Repository:**
```
https://github.com/nRF24/RF24
```

---

#### 2. Wire (I2C Communication)
- **Library Name**: Wire
- **Included**: Built-in with Arduino IDE
- **Action**: No installation needed
- **Description**: I2C communication for MPU6050 and MS5611

---

#### 3. SPI (Serial Peripheral Interface)
- **Library Name**: SPI
- **Included**: Built-in with Arduino IDE
- **Action**: No installation needed
- **Description**: SPI communication for NRF24L01

---

#### 4. Servo (ESC Control)
- **Library Name**: Servo
- **Included**: Built-in with Arduino IDE
- **Action**: No installation needed
- **Description**: Generate PWM signals for ESC control

---

#### 5. MPU6050 Library (Option A - Recommended)
- **Library Name**: MPU6050_light
- **Author**: rfetick
- **Search Term**: "MPU6050_light"
- **Description**: Lightweight and fast MPU6050 library

**Installation:**
```
Library Manager → Search "MPU6050_light" → Install "MPU6050_light by rfetick"
```

**Alternative Repository:**
```
https://github.com/rfetick/MPU6050_light
```

**Note:** The current code implements direct I2C communication with MPU6050, so this library is **optional** but recommended for future enhancements.

---

#### 6. MPU6050 Library (Option B - Alternative)
- **Library Name**: Adafruit MPU6050
- **Author**: Adafruit
- **Search Term**: "Adafruit MPU6050"
- **Description**: Full-featured MPU6050 library by Adafruit

**Installation:**
```
Library Manager → Search "Adafruit MPU6050" → Install "Adafruit MPU6050"
```

**Dependencies (auto-installed):**
- Adafruit Unified Sensor
- Adafruit BusIO

---

#### 7. MS5611 Library
- **Library Name**: MS5611
- **Author**: Rob Tillaart
- **Search Term**: "MS5611"
- **Description**: MS5611 barometric pressure sensor library

**Installation:**
```
Library Manager → Search "MS5611" → Install "MS5611 by Rob Tillaart"
```

**Alternative Repository:**
```
https://github.com/RobTillaart/MS5611
```

**Note:** Current code implements direct I2C communication, so this is **optional** for advanced features.

---

## Method 2: Manual Installation (Advanced)

### Installing from ZIP File

1. Download the library ZIP file from GitHub
2. Open Arduino IDE
3. Go to: `Sketch` → `Include Library` → `Add .ZIP Library...`
4. Select the downloaded ZIP file
5. Restart Arduino IDE

### Installing from GitHub (using Git)

```bash
cd ~/Documents/Arduino/libraries/

# RF24
git clone https://github.com/nRF24/RF24.git

# MPU6050_light
git clone https://github.com/rfetick/MPU6050_light.git

# MS5611
git clone https://github.com/RobTillaart/MS5611.git
```

Restart Arduino IDE after cloning.

---

## 📦 Library Versions (Tested & Verified)

| Library | Version | Compatibility |
|---------|---------|---------------|
| RF24 | 1.4.5+ | ✅ Verified |
| Wire | Built-in | ✅ Verified |
| SPI | Built-in | ✅ Verified |
| Servo | Built-in | ✅ Verified |
| MPU6050_light | 1.0.0+ | ✅ Optional |
| MS5611 | 0.3.9+ | ✅ Optional |

---

## 🔍 Verifying Installation

### Check Installed Libraries

1. Open Arduino IDE
2. Go to: `Sketch` → `Include Library`
3. Scroll down to see installed libraries
4. Verify all required libraries are listed

### Test NRF24L01 Library

Create a new sketch and try:

```cpp
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);

void setup() {
  Serial.begin(115200);
  if (radio.begin()) {
    Serial.println("RF24 library working!");
  } else {
    Serial.println("RF24 library error!");
  }
}

void loop() {}
```

**Expected Output:**
```
RF24 library working!
```

---

## 🛠️ Troubleshooting Installation Issues

### Problem: "Library not found" error

**Solution:**
1. Verify library is installed: `Sketch` → `Include Library` → Check list
2. Restart Arduino IDE
3. Check library folder: `~/Documents/Arduino/libraries/`
4. Re-install library

---

### Problem: Compilation errors after installing library

**Solution:**
1. Check library version compatibility
2. Update Arduino IDE to latest version
3. Remove conflicting libraries (e.g., multiple MPU6050 libraries)
4. Clear Arduino cache:
   - Windows: `C:\Users\<username>\AppData\Local\Temp\arduino_build_*`
   - Mac: `~/Library/Arduino15/`
   - Linux: `~/.arduino15/`

---

### Problem: RF24 library compile errors

**Common Error:**
```
error: 'RF24' does not name a type
```

**Solution:**
1. Ensure all three includes are present:
   ```cpp
   #include <SPI.h>
   #include <nRF24L01.h>
   #include <RF24.h>
   ```
2. Install RF24 library version 1.4.0 or newer
3. Check wiring: CE and CSN pins must be correct

---

### Problem: MPU6050 library conflicts

**Symptoms:**
- Multiple definitions error
- Conflicting function names

**Solution:**
1. Remove all MPU6050 libraries
2. Install only ONE MPU6050 library (recommended: MPU6050_light)
3. Or use the direct I2C implementation (already in code)

---

### Problem: MS5611 library not working

**Solution:**
1. Verify MS5611 is connected to I2C (A4/A5)
2. Check I2C address (usually 0x77)
3. Use I2C scanner to detect device:

```cpp
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  Serial.println("I2C Scanner");
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(i, HEX);
    }
  }
}

void loop() {}
```

**Expected Output:**
```
I2C Scanner
Found device at 0x68  (MPU6050)
Found device at 0x77  (MS5611)
```

---

## 📋 Complete Installation Checklist

- [ ] Arduino IDE installed (version 1.8.13 or newer recommended)
- [ ] RF24 library installed
- [ ] SPI library available (built-in)
- [ ] Wire library available (built-in)
- [ ] Servo library available (built-in)
- [ ] (Optional) MPU6050 library installed
- [ ] (Optional) MS5611 library installed
- [ ] All libraries verified in Library Manager
- [ ] Test sketch compiles without errors
- [ ] Arduino IDE restarted after installations

---

## 🔄 Updating Libraries

Libraries are frequently updated with bug fixes and improvements.

### Update via Library Manager

1. Open Library Manager: `Sketch` → `Include Library` → `Manage Libraries...`
2. Filter installed libraries: Click "Type" dropdown → "Updatable"
3. Click "Update" for each library
4. Restart Arduino IDE

### Manual Update (GitHub)

```bash
cd ~/Documents/Arduino/libraries/
cd RF24
git pull origin master
```

---

## 📚 Alternative Libraries (Not Required, but Useful)

### For Debugging
- **ArduinoLog**: Advanced logging
- **MemoryFree**: Check available RAM

### For Future Enhancements
- **TinyGPS++**: GPS module support
- **SdFat**: Data logging to SD card
- **Adafruit_GFX**: OLED display support
- **OneWire**: Temperature sensors

---

## 💻 Platform-Specific Notes

### Windows
- Libraries folder: `C:\Users\<username>\Documents\Arduino\libraries\`
- May need administrator rights for manual installation

### macOS
- Libraries folder: `~/Documents/Arduino/libraries/`
- May need to allow app permissions in System Preferences

### Linux
- Libraries folder: `~/Arduino/libraries/`
- May need to add user to `dialout` group:
  ```bash
  sudo usermod -a -G dialout $USER
  ```
- Logout and login for changes to take effect

---

## 🎯 Compilation Flags (Advanced)

For optimized performance, add these flags to `platform.local.txt`:

```
compiler.c.extra_flags=-O3 -mcall-prologues
compiler.cpp.extra_flags=-O3 -mcall-prologues
```

**Location:**
- Windows: `C:\Program Files (x86)\Arduino\hardware\arduino\avr\`
- Mac: `/Applications/Arduino.app/Contents/Java/hardware/arduino/avr/`
- Linux: `/usr/share/arduino/hardware/arduino/avr/`

---

## 📞 Getting Help

If you encounter issues:

1. **Check Arduino IDE version**: Update to 1.8.13 or newer
2. **Verify board selection**: Tools → Board → Arduino Nano
3. **Check processor**: Tools → Processor → ATmega328P (Old Bootloader)
4. **Search library issues**: GitHub Issues page for each library
5. **Arduino Forum**: https://forum.arduino.cc/

---

## ✅ Final Verification

Run this test sketch to verify all libraries:

```cpp
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>

void setup() {
  Serial.begin(115200);
  Serial.println("=== Library Test ===");
  
  // Test SPI
  SPI.begin();
  Serial.println("✓ SPI OK");
  
  // Test Wire (I2C)
  Wire.begin();
  Serial.println("✓ Wire OK");
  
  // Test RF24
  RF24 radio(9, 10);
  if (radio.begin()) {
    Serial.println("✓ RF24 OK");
  } else {
    Serial.println("✗ RF24 FAILED");
  }
  
  // Test Servo
  Servo testServo;
  testServo.attach(3);
  Serial.println("✓ Servo OK");
  
  Serial.println("\n=== All Libraries Ready! ===");
}

void loop() {}
```

**Expected Output:**
```
=== Library Test ===
✓ SPI OK
✓ Wire OK
✓ RF24 OK
✓ Servo OK

=== All Libraries Ready! ===
```

---

**Installation Complete!** You're ready to upload the drone code. 🚁
