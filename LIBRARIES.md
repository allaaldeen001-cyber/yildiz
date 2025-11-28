# Library Installation Guide

## Required Libraries

### 1. RF24 Library (NRF24L01 Communication)
**Installation:**
- Arduino IDE → Sketch → Include Library → Manage Libraries
- Search: `RF24`
- Install: **RF24 by TMRh20** (version 1.4.9 or newer)

**Alternative:** If TMRh20 version doesn't work, try:
- `RF24` by maniacbug (older version)

### 2. Adafruit MPU6050 Library
**Installation:**
- Arduino IDE → Sketch → Include Library → Manage Libraries
- Search: `Adafruit MPU6050`
- Install: **Adafruit MPU6050** by Adafruit
- Also install: **Adafruit Unified Sensor** (dependency)

**Alternative Libraries:**
If Adafruit library doesn't work, you can use:
- `MPU6050` by Electronic Cats
- `MPU6050_light` by rfetick (simpler, lighter)

### 3. Adafruit MS5611 Library
**Installation:**
- Arduino IDE → Sketch → Include Library → Manage Libraries
- Search: `Adafruit MS5611`
- Install: **Adafruit MS5611** by Adafruit

**Alternative Libraries:**
If Adafruit library doesn't work, try:
- `MS5611` by SparkFun
- `MS5611-01BA` by SodaqMoja

### 4. Servo Library
**Built-in Library** - No installation needed
- Already included with Arduino IDE

### 5. Built-in Libraries (No Installation Needed)
- `Wire.h` - I2C communication
- `SPI.h` - SPI communication  
- `EEPROM.h` - Non-volatile memory
- `nRF24L01.h` - Included with RF24 library

## Library Compatibility Notes

### If Adafruit Libraries Don't Work:

**For MPU6050**, you can modify the code to use a simpler library:

```cpp
// Replace:
#include <Adafruit_MPU6050.h>
Adafruit_MPU6050 mpu;

// With:
#include <MPU6050.h>
MPU6050 mpu(Wire);

// And change initialization:
mpu.begin();
mpu.setAccelRange(MPU6050_RANGE_2_G);
mpu.setGyroRange(MPU6050_RANGE_250_DEG);
```

**For MS5611**, if Adafruit library doesn't work, you may need to use direct I2C communication or find an alternative library that matches your sensor module.

## Verifying Installation

After installing libraries, verify they're available:
1. Arduino IDE → Sketch → Include Library
2. Check if libraries appear in the list
3. Try compiling the code (Ctrl+R or Cmd+R)
4. Check for compilation errors related to missing libraries

## Common Issues

### "No such file or directory" Error
- Library not installed correctly
- Try reinstalling the library
- Check library folder: `Arduino/libraries/`

### Compilation Errors
- Ensure all dependencies are installed
- Check library versions (some may be incompatible)
- Try updating Arduino IDE to latest version

### Sensor Not Detected
- Check wiring (SDA/SCL for I2C)
- Verify sensor power (3.3V or 5V depending on module)
- Some sensors need pull-up resistors (usually built into modules)

## Library Versions Tested

- Arduino IDE: 1.8.x or 2.x
- RF24: 1.4.9+
- Adafruit MPU6050: 2.2.3+
- Adafruit MS5611: 1.1.0+
- Servo: Built-in (1.2.0+)

## Manual Library Installation

If Library Manager doesn't work:

1. Download library ZIP file from GitHub
2. Arduino IDE → Sketch → Include Library → Add .ZIP Library
3. Select downloaded ZIP file
4. Restart Arduino IDE

## GitHub Links (for manual download)

- RF24: https://github.com/nRF24/RF24
- Adafruit MPU6050: https://github.com/adafruit/Adafruit_MPU6050
- Adafruit MS5611: https://github.com/adafruit/Adafruit_MS5611
