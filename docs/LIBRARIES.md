# Arduino Libraries Required

This document lists all the required libraries for the Professional Quadcopter Drone System.

## Installation Instructions

### Method 1: Arduino IDE Library Manager (Recommended)

1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search for each library below and click **Install**

### Method 2: Manual Installation

Download the libraries from the links provided and install them manually.

---

## Required Libraries

### 1. RF24 Library
**Purpose**: NRF24L01 wireless communication module  
**Author**: TMRh20  
**Version**: 1.4.8 or later  
**Repository**: https://github.com/nRF24/RF24

**Installation via Library Manager**:
```
Search: "RF24"
Install: "RF24 by TMRh20"
```

---

### 2. MPU6050 Library
**Purpose**: MPU6050 IMU (Inertial Measurement Unit) sensor  
**Author**: Electronic Cats (based on jrowberg's work)  
**Version**: 1.5.0 or later  
**Repository**: https://github.com/ElectronicCats/mpu6050

**Installation via Library Manager**:
```
Search: "MPU6050"
Install: "MPU6050 by Electronic Cats"
```

**Note**: This library includes I2Cdev dependency automatically.

---

### 3. Wire Library
**Purpose**: I2C communication (for MPU6050)  
**Status**: Built-in with Arduino IDE  
**Action**: No installation required

---

### 4. SPI Library
**Purpose**: SPI communication (for NRF24L01)  
**Status**: Built-in with Arduino IDE  
**Action**: No installation required

---

## Verification

After installing all libraries, verify by compiling the code:

1. Open `FlightController/FlightController.ino`
2. Select **Board**: Arduino Nano
3. Select **Processor**: ATmega328P or ATmega328P (Old Bootloader)
4. Click **Verify** (checkmark icon)

If compilation succeeds, all libraries are installed correctly!

---

## Troubleshooting

### Issue: "RF24.h: No such file or directory"
**Solution**: Install RF24 library via Library Manager

### Issue: "MPU6050.h: No such file or directory"
**Solution**: Install MPU6050 library by Electronic Cats

### Issue: "I2Cdev.h: No such file or directory"
**Solution**: The MPU6050 library should include I2Cdev. Try reinstalling MPU6050 library.

### Issue: Compilation errors with MPU6050
**Solution**: Make sure you have the "MPU6050 by Electronic Cats" library, not other variants. Alternatively, use the library from: https://github.com/ElectronicCats/mpu6050

---

## Alternative: I2Cdev and MPU6050 by Jeff Rowberg

If the Electronic Cats version doesn't work, you can use the original:

1. Download from: https://github.com/jrowberg/i2cdevlib
2. Navigate to `Arduino/MPU6050` and `Arduino/I2Cdev`
3. Copy both folders to your Arduino `libraries` folder
4. Restart Arduino IDE

---

## Library Versions Tested

| Library | Version | Status |
|---------|---------|--------|
| RF24 | 1.4.8 | ✓ Tested |
| MPU6050 (Electronic Cats) | 1.5.0 | ✓ Tested |
| Wire | Built-in | ✓ Tested |
| SPI | Built-in | ✓ Tested |

---

## Additional Resources

- **RF24 Documentation**: https://nrf24.github.io/RF24/
- **MPU6050 Datasheet**: https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/
- **Arduino Nano Pinout**: https://docs.arduino.cc/hardware/nano

---

**Last Updated**: 2025-11-30
