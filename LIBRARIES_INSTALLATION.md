# 📚 Arduino Libraries Installation Guide

## Required Libraries for Quadcopter Project

This document provides detailed instructions for installing all required Arduino libraries for both the Flight Controller and RC Transmitter.

---

## 🔧 Installation Methods

### Method 1: Arduino IDE Library Manager (Recommended)

1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries...**
3. Search for each library name
4. Click **Install**
5. Wait for installation to complete

### Method 2: Manual Installation

1. Download library ZIP file
2. Go to **Sketch → Include Library → Add .ZIP Library...**
3. Select downloaded ZIP file
4. Restart Arduino IDE

---

## 📦 Required Libraries List

### 1. RF24 (NRF24L01 Wireless Module)

**Library Name**: RF24
**Author**: TMRh20
**Version**: 1.4.0 or later
**Purpose**: Wireless communication between RC and flight controller

**Installation via Library Manager:**
1. Search: "RF24"
2. Install: "RF24 by TMRh20"

**GitHub**: https://github.com/nRF24/RF24

**Verification:**
```cpp
#include <RF24.h>
// If no errors, library is installed correctly
```

---

### 2. MPU6050 (IMU Sensor)

**Library Name**: MPU6050
**Author**: Electronic Cats
**Version**: Latest
**Purpose**: Read gyroscope and accelerometer data

**Installation via Library Manager:**
1. Search: "MPU6050"
2. Install: "MPU6050 by Electronic Cats"

**Alternative**: You can also use "MPU6050 by Jeff Rowberg"

**GitHub**: https://github.com/ElectronicCats/mpu6050

**Verification:**
```cpp
#include <MPU6050.h>
// If no errors, library is installed correctly
```

---

### 3. MS5611 (Barometric Pressure Sensor)

**Library Name**: MS5611
**Author**: Rob Tillaart
**Version**: 0.3.6 or later
**Purpose**: Measure altitude and atmospheric pressure

**Installation via Library Manager:**
1. Search: "MS5611"
2. Install: "MS5611 by Rob Tillaart"

**GitHub**: https://github.com/RobTillaart/MS5611

**Verification:**
```cpp
#include <MS5611.h>
// If no errors, library is installed correctly
```

---

### 4. Servo (Built-in)

**Library Name**: Servo
**Author**: Arduino
**Version**: Built-in
**Purpose**: Control ESC PWM signals

**Installation**: Pre-installed with Arduino IDE

**Verification:**
```cpp
#include <Servo.h>
// Should work without installation
```

---

### 5. Wire (Built-in)

**Library Name**: Wire
**Author**: Arduino
**Version**: Built-in
**Purpose**: I2C communication (MPU6050, MS5611)

**Installation**: Pre-installed with Arduino IDE

**Verification:**
```cpp
#include <Wire.h>
// Should work without installation
```

---

### 6. SPI (Built-in)

**Library Name**: SPI
**Author**: Arduino
**Version**: Built-in
**Purpose**: SPI communication (NRF24L01)

**Installation**: Pre-installed with Arduino IDE

**Verification:**
```cpp
#include <SPI.h>
// Should work without installation
```

---

### 7. EEPROM (Built-in)

**Library Name**: EEPROM
**Author**: Arduino
**Version**: Built-in
**Purpose**: Store calibration data permanently

**Installation**: Pre-installed with Arduino IDE

**Verification:**
```cpp
#include <EEPROM.h>
// Should work without installation
```

---

## ✅ Complete Installation Checklist

### For Flight Controller:
- [ ] RF24 library installed
- [ ] MPU6050 library installed
- [ ] MS5611 library installed
- [ ] Servo library (built-in)
- [ ] Wire library (built-in)
- [ ] SPI library (built-in)
- [ ] EEPROM library (built-in)

### For RC Transmitter:
- [ ] RF24 library installed
- [ ] SPI library (built-in)
- [ ] EEPROM library (built-in)

---

## 🧪 Testing Library Installation

### Test Sketch for Flight Controller

Create a new sketch and paste:

```cpp
// Library Test Sketch - Flight Controller
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Servo.h>
#include <RF24.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Testing libraries...");
  
  // Test Wire (I2C)
  Wire.begin();
  Serial.println("[OK] Wire library loaded");
  
  // Test SPI
  SPI.begin();
  Serial.println("[OK] SPI library loaded");
  
  // Test RF24
  RF24 radio(9, 10);
  Serial.println("[OK] RF24 library loaded");
  
  // Test Servo
  Servo testServo;
  Serial.println("[OK] Servo library loaded");
  
  // Test EEPROM
  byte test = EEPROM.read(0);
  Serial.println("[OK] EEPROM library loaded");
  
  Serial.println("\n[✓] ALL LIBRARIES LOADED SUCCESSFULLY!");
}

void loop() {
  // Nothing here
}
```

**Expected Output:**
```
Testing libraries...
[OK] Wire library loaded
[OK] SPI library loaded
[OK] RF24 library loaded
[OK] Servo library loaded
[OK] EEPROM library loaded

[✓] ALL LIBRARIES LOADED SUCCESSFULLY!
```

---

### Test Sketch for RC Transmitter

Create a new sketch and paste:

```cpp
// Library Test Sketch - RC Transmitter
#include <SPI.h>
#include <EEPROM.h>
#include <RF24.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Testing libraries...");
  
  // Test SPI
  SPI.begin();
  Serial.println("[OK] SPI library loaded");
  
  // Test RF24
  RF24 radio(9, 10);
  Serial.println("[OK] RF24 library loaded");
  
  // Test EEPROM
  byte test = EEPROM.read(0);
  Serial.println("[OK] EEPROM library loaded");
  
  Serial.println("\n[✓] ALL LIBRARIES LOADED SUCCESSFULLY!");
}

void loop() {
  // Nothing here
}
```

---

## 🐛 Troubleshooting

### Problem: "Library not found" error

**Solution 1**: Reinstall library via Library Manager
1. Go to Sketch → Include Library → Manage Libraries
2. Search for library
3. Uninstall if already installed
4. Install again

**Solution 2**: Manual installation
1. Download library ZIP from GitHub
2. Extract to Arduino libraries folder:
   - Windows: `Documents\Arduino\libraries\`
   - Mac: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`
3. Restart Arduino IDE

---

### Problem: "Multiple libraries found" warning

**Solution**: Remove duplicate libraries
1. Check Arduino libraries folder
2. Delete duplicate library folders
3. Keep only one version (latest)
4. Restart Arduino IDE

---

### Problem: Compilation errors after library installation

**Solution 1**: Update Arduino IDE
- Download latest version from arduino.cc
- Install and restart

**Solution 2**: Check library compatibility
- Some libraries require specific Arduino IDE versions
- Check library documentation for requirements

**Solution 3**: Clean build
1. Close Arduino IDE
2. Delete contents of:
   - Windows: `%TEMP%\arduino_build_*`
   - Mac/Linux: `/tmp/arduino_build_*`
3. Restart Arduino IDE and recompile

---

### Problem: NRF24L01 library conflicts

**Solution**: Use correct RF24 library
- Install "RF24 by TMRh20" (NOT "RF24Network" or others)
- Remove other NRF24 libraries
- Use example code to test

---

### Problem: MPU6050 library doesn't work

**Solution**: Try alternative library
- If "MPU6050 by Electronic Cats" doesn't work
- Try "Adafruit MPU6050" or "Jeff Rowberg I2Cdev"
- Update code includes accordingly

---

### Problem: MS5611 not detected

**Solution**: Check I2C address
- Some MS5611 modules use address 0x77
- Others use 0x76
- Modify code if needed:
```cpp
#define MS5611_ADDR 0x76  // Change if needed
```

---

## 🔍 Library Locations

### Windows
```
C:\Users\[YourName]\Documents\Arduino\libraries\
```

### Mac
```
/Users/[YourName]/Documents/Arduino/libraries/
```

### Linux
```
/home/[YourName]/Arduino/libraries/
```

---

## 📖 Library Documentation

### RF24
- Documentation: https://nrf24.github.io/RF24/
- Examples: File → Examples → RF24
- Common issues: Power supply (use 3.3V, add capacitor)

### MPU6050
- Documentation: https://github.com/ElectronicCats/mpu6050
- Examples: File → Examples → MPU6050
- Common issues: I2C address (usually 0x68)

### MS5611
- Documentation: https://github.com/RobTillaart/MS5611
- Examples: File → Examples → MS5611
- Common issues: I2C address (0x77 or 0x76)

---

## 🎯 Version Compatibility

### Arduino IDE Versions
- **Minimum**: 1.8.0
- **Recommended**: 1.8.19 or Arduino IDE 2.x
- **Tested**: 1.8.19, 2.2.1

### Arduino Board Support
- **Board**: Arduino Nano
- **Processor**: ATmega328P
- **Bootloader**: Old Bootloader (for clone boards)

**Installation**:
1. Tools → Board → Boards Manager
2. Search "Arduino AVR Boards"
3. Install if not already installed

---

## 🚀 Quick Installation Script

**Copy-paste into Arduino IDE Serial Monitor** (115200 baud) after uploading test sketch:

```
Libraries needed:
1. RF24 by TMRh20
2. MS5611 by Rob Tillaart

Built-in (no install needed):
- Wire
- SPI
- Servo
- EEPROM

Note: MPU6050 library direct register access used in code (no external library needed for our implementation)
```

---

## ✅ Final Verification

After installing all libraries:

1. **Open Flight Controller sketch**
2. **Click Verify/Compile** (checkmark icon)
3. **Should compile without errors**

If errors occur:
- Check library versions
- Update Arduino IDE
- Review troubleshooting section

---

## 📞 Getting Help

### If libraries still don't work:

1. **Check Arduino IDE output window** for specific error
2. **Google the exact error message**
3. **Check library GitHub issues** page
4. **Try library examples** first before main code
5. **Ask in Arduino forums** with full error log

---

## 🎓 Learning Resources

### Understanding Arduino Libraries
- Arduino Library Tutorial: https://www.arduino.cc/en/Hacking/Libraries
- Creating Custom Libraries: https://www.arduino.cc/en/Hacking/LibraryTutorial

### I2C Communication
- Wire Library Guide: https://www.arduino.cc/en/Reference/Wire
- I2C Scanner Sketch: File → Examples → Wire → i2c_scanner

### SPI Communication
- SPI Library Guide: https://www.arduino.cc/en/Reference/SPI
- Understanding SPI: https://learn.sparkfun.com/tutorials/serial-peripheral-interface-spi

---

**Ready to fly once all libraries are installed! 🚁**

---

**Document Version**: 1.0.0
**Last Updated**: November 2025
