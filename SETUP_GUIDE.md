# Setup and Installation Guide

## Required Arduino Libraries

### 1. RF24 Library (NRF24L01)

**Installation:**
1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search for "RF24"
4. Install "RF24" by TMRh20

**Alternative Manual Installation:**
```bash
cd ~/Arduino/libraries
git clone https://github.com/nRF24/RF24.git
```

### 2. MPU6050 Library

**Option A: I2Cdev Library (Recommended)**
1. Install "I2Cdevlib-MPU6050" from Library Manager
2. Also install "Wire" library (usually included with Arduino IDE)

**Option B: Simple MPU6050 Library**
1. Install "MPU6050" by Electronic Cats from Library Manager

**Manual Installation:**
```bash
cd ~/Arduino/libraries
git clone https://github.com/jrowberg/i2cdevlib.git
# Use the Arduino/MPU6050 folder
```

### 3. MS5611 Library

**Installation:**
1. Install "MS5611" by SparkFun from Library Manager

**Manual Installation:**
```bash
cd ~/Arduino/libraries
git clone https://github.com/sparkfun/SparkFun_MS5611_Arduino_Library.git
```

### 4. Standard Libraries (Included)
- **SPI** - Usually included
- **Wire** - Usually included

---

## Hardware Connections

### Flight Controller Wiring

```
Arduino Nano          Component
─────────────────────────────────
5V                   → NRF24L01 VCC
GND                  → NRF24L01 GND
D4                   → NRF24L01 CE
D10                  → NRF24L01 CSN
D11 (MOSI)           → NRF24L01 MOSI
D12 (MISO)           → NRF24L01 MISO
D13 (SCK)            → NRF24L01 SCK

5V                   → MPU6050 VCC
GND                  → MPU6050 GND
A4 (SDA)             → MPU6050 SDA
A5 (SCL)             → MPU6050 SCL
D2                   → MPU6050 INT (optional)

5V                   → MS5611 VCC
GND                  → MS5611 GND
A4 (SDA)             → MS5611 SDA
A5 (SCL)             → MS5611 SCL

D8                   → Buzzer (+)
GND                  → Buzzer (-)

D7                   → LED (+)
220Ω Resistor        → LED (-) → GND

D3                   → ESC FL Signal
D5                   → ESC FR Signal
D6                   → ESC RR Signal
D9                   → ESC RL Signal
GND                  → ESC GND (common)
```

### Remote Controller Wiring

```
Arduino Nano          Component
─────────────────────────────────
5V                   → NRF24L01 VCC
GND                  → NRF24L01 GND
D9                   → NRF24L01 CE
D10                  → NRF24L01 CSN
D11 (MOSI)           → NRF24L01 MOSI
D12 (MISO)           → NRF24L01 MISO
D13 (SCK)            → NRF24L01 SCK

A0                   → Left Joystick V (Throttle)
A1                   → Left Joystick H (Yaw)
A2                   → Right Joystick V (Pitch)
A3                   → Right Joystick H (Roll)
5V                   → Joystick VCC
GND                  → Joystick GND

D4                   → Button 1 (with pull-up)
D5                   → Button 2 (with pull-up)
D2                   → Switch 1 (with pull-up)
D3                   → Switch 2 (with pull-up)
```

---

## Software Setup

### Step 1: Install Arduino IDE

1. Download Arduino IDE 1.8.x or 2.x from [arduino.cc](https://www.arduino.cc)
2. Install Arduino IDE
3. Install Arduino AVR Boards (Tools → Board → Boards Manager)

### Step 2: Install Libraries

Follow the library installation instructions above.

### Step 3: Upload Firmware

**Flight Controller:**
1. Open `FlightController.ino` in Arduino IDE
2. Select **Tools → Board → Arduino Nano**
3. Select **Tools → Processor → ATmega328P (Old Bootloader)** or **ATmega328P**
4. Select correct COM port
5. Click **Upload**

**Remote Controller:**
1. Open `RemoteController.ino` in Arduino IDE
2. Select **Tools → Board → Arduino Nano**
3. Select **Tools → Processor → ATmega328P (Old Bootloader)** or **ATmega328P**
4. Select correct COM port
5. Click **Upload**

### Step 4: Verify Installation

**Flight Controller:**
- Open Serial Monitor (115200 baud)
- Should see "Flight Controller Initialized"
- Should hear 2 beeps
- LED should blink when RC is connected

**Remote Controller:**
- Open Serial Monitor (115200 baud)
- Should see calibration messages
- Should see status updates every 200ms

---

## Troubleshooting

### Library Not Found Errors

**Problem:** `MPU6050.h: No such file or directory`

**Solution:**
1. Verify library is installed in `~/Arduino/libraries/`
2. Restart Arduino IDE
3. Check library folder name matches `#include` statement

### NRF24L01 Not Communicating

**Problem:** No link between RC and FC

**Solutions:**
1. Check wiring (CE, CSN, MOSI, MISO, SCK)
2. Verify power supply (3.3V stable)
3. Check antenna connection
4. Verify channel matches (103)
5. Try different channel if interference

### MPU6050 Not Detected

**Problem:** MPU6050 connection failed

**Solutions:**
1. Check I2C wiring (SDA, SCL)
2. Verify MPU6050 address (usually 0x68)
3. Check power supply (3.3V or 5V depending on module)
4. Add pull-up resistors (4.7kΩ) if needed

### MS5611 Not Reading

**Problem:** Altitude readings incorrect

**Solutions:**
1. Check I2C wiring
2. Verify MS5611 address (usually 0x77)
3. Check power supply
4. Verify sensor initialization

### Compilation Errors

**Problem:** Code won't compile

**Solutions:**
1. Verify all libraries installed
2. Check Arduino IDE version (1.8+)
3. Verify board selection (Arduino Nano)
4. Check for syntax errors
5. Ensure `DroneProtocol.h` is in same folder

---

## Alternative: Minimal Library Implementation

If you have trouble with libraries, you can use basic I2C communication. However, the provided code uses standard libraries for simplicity and reliability.

### Basic MPU6050 I2C Functions

```cpp
void initMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Wake up MPU6050
  Wire.endTransmission();
}

void readMPU6050(int16_t* ax, int16_t* ay, int16_t* az, 
                 int16_t* gx, int16_t* gy, int16_t* gz) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // ACCEL_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  
  *ax = Wire.read() << 8 | Wire.read();
  *ay = Wire.read() << 8 | Wire.read();
  *az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Temperature
  *gx = Wire.read() << 8 | Wire.read();
  *gy = Wire.read() << 8 | Wire.read();
  *gz = Wire.read() << 8 | Wire.read();
}
```

### Basic MS5611 I2C Functions

```cpp
void initMS5611() {
  Wire.beginTransmission(0x77);
  Wire.write(0x1E); // Reset command
  Wire.endTransmission();
  delay(10);
}

float readMS5611Pressure() {
  // Read pressure from MS5611
  // Implementation depends on specific library
  // See MS5611 datasheet for details
}
```

**Note:** Using libraries is recommended for reliability and ease of use.

---

## Board Configuration

### Arduino Nano Settings

**Important Settings:**
- **Board:** Arduino Nano
- **Processor:** ATmega328P (Old Bootloader) OR ATmega328P
  - Try "Old Bootloader" first if upload fails
- **Clock:** 16 MHz
- **Programmer:** Arduino as ISP

### Memory Usage

**Flight Controller:**
- Flash: ~28 KB (of 32 KB)
- SRAM: ~1.5 KB (of 2 KB)

**Remote Controller:**
- Flash: ~18 KB (of 32 KB)
- SRAM: ~0.8 KB (of 2 KB)

**Optimization Tips:**
- Remove Serial.print() statements if running out of memory
- Reduce string usage
- Use PROGMEM for constants

---

## First-Time Setup Checklist

- [ ] Arduino IDE installed
- [ ] All libraries installed
- [ ] Hardware wired correctly
- [ ] Flight Controller firmware uploaded
- [ ] Remote Controller firmware uploaded
- [ ] Serial monitors open (both)
- [ ] Link established (LED blinking)
- [ ] IMU calibrated (Button 1)
- [ ] ESC calibrated (Button 2)
- [ ] Motors tested (no props!)
- [ ] Ready for flight testing

---

## Next Steps

After setup is complete:
1. Read **README.md** for system overview
2. Read **TUNING_GUIDE.md** for PID tuning
3. Follow **Testing Procedures** in README
4. Begin with bench testing (no props!)
5. Proceed to hover testing
6. Tune PIDs as needed

---

## Support

For issues:
1. Check troubleshooting section above
2. Verify all connections
3. Check serial monitor for error messages
4. Review code comments
5. Consult sensor datasheets

**Remember:** Always test without props first!
