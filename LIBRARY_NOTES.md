# Library Installation and Compatibility Notes

## Required Libraries

### 1. RF24 (NRF24L01 Radio Module)
**Library**: RF24 by TMRh20
- **Installation**: Arduino IDE → Sketch → Include Library → Manage Libraries → Search "RF24"
- **Version**: Latest stable version
- **GitHub**: https://github.com/nRF24/RF24

### 2. Adafruit MPU6050
**Library**: Adafruit MPU6050 by Adafruit
- **Installation**: Arduino IDE → Sketch → Include Library → Manage Libraries → Search "Adafruit MPU6050"
- **Dependencies**: Adafruit Unified Sensor (will install automatically)
- **GitHub**: https://github.com/adafruit/Adafruit_MPU6050

### 3. Adafruit Unified Sensor
**Library**: Adafruit Unified Sensor by Adafruit
- **Installation**: Usually installed automatically with MPU6050 library
- **GitHub**: https://github.com/adafruit/Adafruit_Sensor

### 4. MS5611 (Barometric Pressure Sensor)
**Note**: There are multiple MS5611 libraries available. The code uses a generic API.

**Option 1: SparkFun MS5611 Library** (Recommended)
- **Library**: SparkFun MS5611 Arduino Library
- **Installation**: Arduino IDE → Sketch → Include Library → Manage Libraries → Search "SparkFun MS5611"
- **GitHub**: https://github.com/sparkfun/SparkFun_MS5611_Arduino_Library
- **API**: Uses `baro.read()` and `baro.getPressure()`

**Option 2: MS5611 by SodaqMoja**
- **Library**: MS5611 by SodaqMoja
- **Installation**: Arduino IDE → Sketch → Include Library → Manage Libraries → Search "MS5611"
- **GitHub**: https://github.com/SodaqMoja/MS5611

**If your library has different API**, you may need to modify the `readMS5611()` function in `FC_Quadcopter.ino`:

```cpp
void readMS5611() {
  // For SparkFun library:
  ms5611.read();
  pressure = ms5611.getPressure();
  temperature = ms5611.getTemperature();
  
  // OR for other libraries, adjust accordingly:
  // pressure = ms5611.readPressure();
  // temperature = ms5611.readTemperature();
}
```

### 5. Servo Library
**Library**: Built-in Arduino library
- No installation needed, comes with Arduino IDE

## Alternative MPU6050 Libraries

If Adafruit MPU6050 doesn't work, you can use:

**Option: MPU6050 by Electronic Cats**
- Search "MPU6050" in Library Manager
- May require code modifications

**Option: I2Cdevlib MPU6050**
- More complex but very reliable
- GitHub: https://github.com/jrowberg/i2cdevlib

## Troubleshooting Library Issues

### Compilation Errors

1. **"No such file or directory"**
   - Make sure all libraries are installed
   - Check library names match exactly
   - Try restarting Arduino IDE

2. **"Multiple definition" errors**
   - Close and reopen Arduino IDE
   - Delete `libraries` folder in Arduino sketchbook and reinstall

3. **MPU6050 not found**
   - Install Adafruit Unified Sensor first
   - Check I2C connections

4. **MS5611 API mismatch**
   - Check your library's documentation
   - Modify `readMS5611()` function to match your library's API

### Library Version Compatibility

- **Arduino IDE**: Version 1.8.x or 2.x recommended
- **Arduino AVR Boards**: Latest version
- If using older IDE, you may need older library versions

## Manual Library Installation

If Library Manager doesn't work:

1. Download library ZIP from GitHub
2. Arduino IDE → Sketch → Include Library → Add .ZIP Library
3. Select downloaded ZIP file

## Testing Libraries

After installation, test each library with example sketches:

1. **RF24**: File → Examples → RF24 → GettingStarted
2. **MPU6050**: File → Examples → Adafruit MPU6050 → basic_readings
3. **MS5611**: Check library examples folder

## Code Modifications for Different Libraries

If you need to use different libraries, modify these sections in `FC_Quadcopter.ino`:

### For Different MPU6050 Library:
```cpp
// Replace the include
#include <YourMPU6050Library.h>

// Replace initialization
YourMPU6050 mpu;

// Replace readMPU6050() function with your library's API
```

### For Different MS5611 Library:
```cpp
// Replace the include
#include <YourMS5611Library.h>

// Replace initialization
YourMS5611 ms5611;

// Modify readMS5611() to match your library's API
```
