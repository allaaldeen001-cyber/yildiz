# Drone Control System - Installation Guide

## Library Installation

### Required Library
Install the **TMRh20 RF24** library (not the standard RF24 library):

1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search for **"RF24"** by **TMRh20**
4. Install **"RF24"** by **TMRh20** (version 1.4.9 or later)

Or install manually:
- Download from: https://github.com/nRF24/RF24
- Extract to Arduino libraries folder

## Hardware Connections

### Transmitter (Controller)
```
nRF24L01    Arduino
--------    -------
VCC    →    3.3V (IMPORTANT: Use 3.3V, NOT 5V!)
GND    →    GND
CE     →    Pin 9
CSN    →    Pin 10
SCK    →    Pin 13
MOSI   →    Pin 11
MISO   →    Pin 12
```

### Receiver (Drone)
```
nRF24L01    Arduino
--------    -------
VCC    →    3.3V (IMPORTANT: Use 3.3V, NOT 5V!)
GND    →    GND
CE     →    Pin 9
CSN    →    Pin 10
SCK    →    Pin 13
MOSI   →    Pin 11
MISO   →    Pin 12

Motors/ESCs:
MOTOR_FL → Pin 3 (Front Left)
MOTOR_FR → Pin 5 (Front Right)
MOTOR_BL → Pin 6 (Back Left)
MOTOR_BR → Pin 9 (Back Right)
```

## Important Notes

1. **Power Supply**: nRF24L01 modules require 3.3V. Using 5V will damage them!
   - Use a voltage regulator or 3.3V power supply
   - Add a 10µF capacitor between VCC and GND for stability

2. **Antenna**: Ensure antennas are properly connected and not touching

3. **Distance**: Keep modules at least 1 meter apart during testing

4. **Channel**: Both transmitter and receiver must use the same channel (currently set to 76)

## Configuration

### Why ACK is Enabled (Perfect Communication)

**ACK (Acknowledgment) is ENABLED** for the following reasons:

✅ **Reliability**: Ensures every command packet is received
✅ **Automatic Retransmission**: Failed packets are automatically resent
✅ **Error Detection**: Built-in CRC checking
✅ **Critical for Drones**: Lost commands could cause crashes
✅ **Low Latency**: With optimized settings, ACK adds only ~1-2ms

### Radio Settings Explained

- **Data Rate**: 250KBPS (lower = better range and reliability)
- **Power Level**: MAX (maximum range)
- **Channel**: 76 (2.476 GHz, less interference)
- **CRC**: 16-bit (strong error detection)
- **Retries**: 3 attempts with 5*250µs delay

## Testing

1. Upload `transmitter.ino` to your controller Arduino
2. Upload `receiver_drone.ino` to your drone Arduino
3. Open Serial Monitor (115200 baud) on both devices
4. You should see initialization messages
5. Transmitter will send commands every 20ms (50Hz)
6. Receiver will apply motor control and print status

## Troubleshooting

### No Connection
- Check wiring (especially VCC = 3.3V)
- Verify both devices are powered
- Check Serial Monitor for error messages
- Ensure antennas are connected

### Poor Range
- Use external power supply (not USB)
- Add capacitors (10µF + 100nF) near nRF24L01 VCC
- Check antenna connections
- Try different channel (change `setChannel()`)

### Motors Not Responding
- Verify motor pins match your setup
- Check ESC calibration
- Ensure throttle is above minimum threshold
- Check Serial Monitor for "Signal lost" messages
