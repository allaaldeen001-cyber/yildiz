# Required Libraries Installation Guide

## Quick Install via Arduino IDE

1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search for each library and click **Install**

## Required Libraries

### 1. RF24 Library
- **Name**: RF24
- **Author**: TMRh20
- **Version**: Latest
- **Purpose**: NRF24L01 wireless communication
- **Installation**: 
  ```
  Library Manager → Search "RF24" → Install "RF24" by TMRh20
  ```

### 2. PID Library
- **Name**: PID
- **Author**: Brett Beauregard
- **Version**: Latest
- **Purpose**: PID control for flight stability
- **Installation**:
  ```
  Library Manager → Search "PID" → Install "PID Library" by Brett Beauregard
  ```

### 3. Servo Library
- **Name**: Servo
- **Status**: Built-in (usually pre-installed)
- **Purpose**: ESC control via PWM
- **Installation**: Usually included with Arduino IDE

### 4. Wire Library
- **Name**: Wire
- **Status**: Built-in (usually pre-installed)
- **Purpose**: I2C communication for sensors
- **Installation**: Usually included with Arduino IDE

### 5. SPI Library
- **Name**: SPI
- **Status**: Built-in (usually pre-installed)
- **Purpose**: SPI communication for NRF24L01
- **Installation**: Usually included with Arduino IDE

## Optional Libraries (for Enhanced Functionality)

### MPU6050 Libraries (Optional)
The code uses direct I2C communication, but you can use libraries for easier integration:

**Option 1: I2Cdev Library**
- **Name**: I2Cdev
- **Author**: Jeff Rowberg
- **Includes**: MPU6050 support
- **Installation**: Library Manager → Search "I2Cdev"

**Option 2: Adafruit MPU6050**
- **Name**: Adafruit_MPU6050
- **Author**: Adafruit
- **Installation**: Library Manager → Search "Adafruit MPU6050"

### MS5611 Libraries (Optional)
**Option 1: SparkFun MS5611**
- **Name**: SparkFun_MS5611_Arduino_Library
- **Author**: SparkFun Electronics
- **Installation**: Library Manager → Search "SparkFun MS5611"

## Manual Installation (if Library Manager fails)

1. Download library ZIP file from GitHub
2. In Arduino IDE: **Sketch → Include Library → Add .ZIP Library**
3. Select the downloaded ZIP file
4. Restart Arduino IDE

## Library Links

- **RF24**: https://github.com/nRF24/RF24
- **PID**: https://github.com/br3ttb/Arduino-PID-Library
- **I2Cdev**: https://github.com/jrowberg/i2cdevlib
- **Adafruit MPU6050**: https://github.com/adafruit/Adafruit_MPU6050
- **SparkFun MS5611**: https://github.com/sparkfun/SparkFun_MS5611_Arduino_Library

## Verification

After installation, verify libraries are available:
1. Open Arduino IDE
2. Go to **Sketch → Include Library**
3. Check that RF24 and PID appear in the list
4. If not, restart Arduino IDE

## Troubleshooting

### Library Not Found
- Restart Arduino IDE after installation
- Check library is in correct folder (usually `Documents/Arduino/libraries/`)
- Verify library folder name matches include statement

### Compilation Errors
- Ensure all required libraries are installed
- Check Arduino IDE version (1.8.x or later recommended)
- Verify board selection matches your hardware

### Version Conflicts
- Use latest stable versions
- If issues occur, try specific versions mentioned in library documentation
