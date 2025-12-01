# Wiring Diagrams and Connection Guide

## Overview

This document provides detailed wiring information for both the Flight Controller and Remote Controller boards.

## Important Notes

⚠️ **CRITICAL:**
- nRF24L01+ modules require **3.3V** (NOT 5V!)
- MPU6050 requires **3.3V** (NOT 5V!)
- Always add 10µF capacitor across nRF24L01+ VCC/GND
- Use short wires for nRF24L01+ (<10cm recommended)
- Keep nRF24L01+ away from motors and ESCs
- Double-check polarity before powering on

## Flight Controller Wiring

### Complete Connection Table

| Arduino Pin | Component | Component Pin | Notes |
|-------------|-----------|---------------|-------|
| **SPI Bus** |
| D10 | nRF24L01+ | CSN | Chip Select |
| D4 | nRF24L01+ | CE | Chip Enable |
| D11 | nRF24L01+ | MOSI | SPI Data Out |
| D12 | nRF24L01+ | MISO | SPI Data In |
| D13 | nRF24L01+ | SCK | SPI Clock |
| 3.3V | nRF24L01+ | VCC | **Add 10µF cap!** |
| GND | nRF24L01+ | GND | Common ground |
| **I2C Bus** |
| A4 (SDA) | MPU6050 | SDA | I2C Data |
| A5 (SCL) | MPU6050 | SCL | I2C Clock |
| D2 | MPU6050 | INT | Interrupt (optional) |
| 3.3V | MPU6050 | VCC | 3.3V power |
| GND | MPU6050 | GND | Common ground |
| **Motor Outputs** |
| D3 | ESC Front Left | Signal | PWM output |
| D5 | ESC Front Right | Signal | PWM output |
| D6 | ESC Rear Right | Signal | PWM output |
| D9 | ESC Rear Left | Signal | PWM output |
| **Indicators** |
| D7 | Status LED | Anode (+) | Through 220Ω resistor |
| GND | Status LED | Cathode (-) | Common ground |
| D8 | Buzzer | Positive (+) | Active buzzer |
| GND | Buzzer | Negative (-) | Common ground |
| **Power** |
| VIN | Battery/BEC | +5V | From ESC BEC or USB |
| GND | Battery/BEC | GND | Common ground |

### Visual Layout - Flight Controller

```
                    Arduino Nano (Flight Controller)
                    
                         USB Port
                    +----------------+
          [MPU6050]--| A5 (SCL)   VIN|--[5V from ESC BEC]
                    | A4 (SDA)   GND|--[Common Ground]
                    | A3         RST|
                    | A2         +5V|
                    | A1          A7|
                    | A0          A6|
                    |                |
    [MPU INT] ------| D2         D13|------[nRF SCK]
   [ESC FL] --------| D3         D12|------[nRF MISO]
     [nRF CE] ------| D4         D11|------[nRF MOSI]
   [ESC FR] --------| D5         D10|------[nRF CSN]
   [ESC RR] --------| D6          D9|------[ESC RL]
   [LED] -----------| D7          D8|------[Buzzer]
                    +----------------+

Motor Configuration (Top View):
          FRONT
        FL     FR
     (D3)     (D5)
       CCW     CW
         \    /
          \  /
           \/
           /\
          /  \
         /    \
       CW     CCW
     (D9)     (D6)
        RL     RR
          REAR
```

### nRF24L01+ Module Detail

```
nRF24L01+ PA Module (Top View)
    Antenna
      |
   +-----+
   |  *  |  * = Looking at component side
   |     |
   +-----+
   |||||||
   |||||||
   VGCCCM
   CSMESI
   CNEKOS
         I
Pin 1: GND   → Arduino GND
Pin 2: VCC   → Arduino 3.3V + 10µF capacitor to GND
Pin 3: CE    → Arduino D4
Pin 4: CSN   → Arduino D10
Pin 5: SCK   → Arduino D13
Pin 6: MOSI  → Arduino D11
Pin 7: MISO  → Arduino D12
Pin 8: IRQ   → Not connected

⚠️ ADD 10µF CAPACITOR between VCC and GND pins!
```

### MPU6050 Module Detail

```
MPU6050 Module
   +-------+
   | MPU   |
   | 6050  |
   +-------+
   ||||||||
   VGSSII
   CNDCDN
   CAL2T
       A
Pin 1: VCC  → Arduino 3.3V
Pin 2: GND  → Arduino GND
Pin 3: SCL  → Arduino A5
Pin 4: SDA  → Arduino A4
Pin 5: XDA  → Not connected
Pin 6: XCL  → Not connected
Pin 7: AD0  → Not connected (or GND for 0x68)
Pin 8: INT  → Arduino D2
```

### ESC Connections

```
ESC (Electronic Speed Controller)

Each ESC has 3 sets of wires:

1. Motor Wires (3 wires, thick):
   - Connect to motor in any order
   - Swap any two to reverse direction

2. Power Wires (2 wires, thick):
   - Red (+) → Battery positive
   - Black (-) → Battery negative
   - All ESCs connect to same battery

3. Signal Wires (3 wires, thin):
   - Brown/Black → GND (common ground)
   - Red → +5V (BEC output, connect ONE ESC only)
   - Yellow/White → Signal (PWM from Arduino)
   
ESC BEC Connection:
   ESC1 BEC Red → Arduino VIN (power Arduino)
   ESC1 BEC Black → Arduino GND
   ESC2,3,4 → Disconnect red wire or connect to GND only
```

### LED Connection

```
Status LED Circuit

Arduino D7 → 220Ω Resistor → LED Anode (+)
                              LED Cathode (-) → GND

Or use LED with built-in resistor:
Arduino D7 → LED (+) → LED (-) → GND
```

### Buzzer Connection

```
Active Buzzer (has internal oscillator)

Arduino D8 → Buzzer (+) → GND
         
Note: Use active buzzer (beeps when powered)
      Not passive buzzer (requires PWM signal)
```

## Remote Controller Wiring

### Complete Connection Table

| Arduino Pin | Component | Component Pin | Notes |
|-------------|-----------|---------------|-------|
| **SPI Bus** |
| D10 | nRF24L01+ | CSN | Chip Select |
| D9 | nRF24L01+ | CE | Chip Enable |
| D11 | nRF24L01+ | MOSI | SPI Data Out |
| D12 | nRF24L01+ | MISO | SPI Data In |
| D13 | nRF24L01+ | SCK | SPI Clock |
| 3.3V | nRF24L01+ | VCC | **Add 10µF cap!** |
| GND | nRF24L01+ | GND | Common ground |
| **Left Joystick** |
| A0 | Joystick | VRy | Vertical axis (Throttle) |
| A1 | Joystick | VRx | Horizontal axis (Yaw) |
| 5V | Joystick | +5V | Power |
| GND | Joystick | GND | Ground |
| **Right Joystick** |
| A2 | Joystick | VRy | Vertical axis (Pitch) |
| A3 | Joystick | VRx | Horizontal axis (Roll) |
| 5V | Joystick | +5V | Power |
| GND | Joystick | GND | Ground |
| **Buttons** (with internal pullup) |
| D4 | Button 1 | Pin 1 | Uses INPUT_PULLUP |
| D5 | Button 2 | Pin 1 | Uses INPUT_PULLUP |
| D6 | Button 3 | Pin 1 | Uses INPUT_PULLUP |
| D7 | Button 4 | Pin 1 | Uses INPUT_PULLUP |
| GND | All Buttons | Pin 2 | Common ground |
| **Toggle Switches** |
| D2 | SW1 (Arm) | Common | Uses INPUT_PULLUP |
| D3 | SW2 (Mode) | Common | Uses INPUT_PULLUP |
| GND | Both Switches | Off Position | Common ground |
| **Power** |
| VIN | Battery | +7-12V | Or USB 5V |
| GND | Battery | GND | Common ground |

### Visual Layout - Remote Controller

```
                    Arduino Nano (Remote Controller)
                    
                         USB Port
                    +----------------+
  [Joy L V/Throttle]--| A0         VIN|--[Battery +]
     [Joy L H/Yaw]----| A1         GND|--[Battery -]
  [Joy R V/Pitch] ----| A2         RST|
    [Joy R H/Roll]----| A3         +5V|--[Joystick Power]
                      | A4          A7|
                      | A5          A6|
                      |                |
    [SW1/Arm] --------| D2         D13|------[nRF SCK]
   [SW2/Mode] --------| D3         D12|------[nRF MISO]
   [Button 1] --------| D4         D11|------[nRF MOSI]
   [Button 2] --------| D5         D10|------[nRF CSN]
   [Button 3] --------| D6          D9|------[nRF CE]
   [Button 4] --------| D7          D8|
                      +----------------+

All buttons and switches connect to GND when pressed/toggled.
```

### Joystick Module Detail

```
Analog Joystick Module (Typical)

   +-------+
   | JOY-  |
   |  STICK|
   +-------+
   |||||
   GVVVS
   N+RR W
   D5XY

Pin 1: GND → Arduino GND
Pin 2: +5V → Arduino 5V
Pin 3: VRx → Arduino A1 (Left) or A3 (Right)
Pin 4: VRy → Arduino A0 (Left) or A2 (Right)
Pin 5: SW  → Not used (optional button)

Left Joystick:
  VRy (A0) = Throttle (up/down)
  VRx (A1) = Yaw (left/right rotation)

Right Joystick:
  VRy (A2) = Pitch (forward/back)
  VRx (A3) = Roll (left/right strafe)
```

### Button Wiring

```
Push Button (Momentary Contact)

Simple button connection using internal pullup:

Arduino D4/D5/D6/D7 ---|
                      [ ] Button
GND -------------------|

When button pressed, pin reads LOW
When button released, pin reads HIGH (internal pullup)

No external resistor needed!
```

### Toggle Switch Wiring

```
Toggle Switch (SPDT - Single Pole Double Throw)

    Arduino D2/D3 ---o  o--- Not Connected
                      \ 
                       o--- GND

Switch positions:
  - Center/Up: Pin reads HIGH (internal pullup)
  - Down: Pin reads LOW (connected to GND)

Or use SPST (Single Pole Single Throw):

    Arduino D2/D3 ---o o--- GND
                     \___/
                    Switch

Simpler, works the same way.
```

## Power Distribution

### Flight Controller Power

```
LiPo Battery (11.1V - 14.8V)
       |
       |
    +----+----+----+----+
    |    |    |    |    |
  ESC1 ESC2 ESC3 ESC4  (All ESCs in parallel)
    |    |    |    |
  Motor Motor Motor Motor
    |
  BEC (5V output)
    |
Arduino VIN (Powers Nano + 3.3V regulator)
    |
    +---> nRF24L01+ (3.3V from Nano)
    +---> MPU6050 (3.3V from Nano)
    +---> LED + Buzzer (5V)

⚠️ Only connect ONE ESC BEC to Arduino!
```

### Remote Controller Power

```
Option 1: USB Power
USB Cable → Arduino → 3.3V Regulator → nRF24L01+
                  → 5V → Joysticks

Option 2: Battery Power  
9V Battery → Arduino VIN → Same as above

Option 3: LiPo with voltage regulator
2S/3S LiPo → Buck Converter (7.4V→5V) → Arduino VIN
```

## Common Wiring Mistakes

### ❌ DON'T DO THIS:

1. **5V to nRF24L01+**
   ```
   Arduino 5V → nRF24L01+ VCC ❌ WRONG!
   (Will damage nRF24L01+!)
   ```

2. **No Capacitor on nRF24L01+**
   ```
   nRF24L01+ VCC → 3.3V (no capacitor) ❌
   (Causes brown-outs and connection issues)
   ```

3. **Wrong Motor Directions**
   ```
   All motors same direction ❌
   (Drone will flip on takeoff)
   ```

4. **Multiple BECs to VIN**
   ```
   ESC1 BEC → VIN
   ESC2 BEC → VIN  ❌ WRONG!
   (Can cause voltage conflicts)
   ```

### ✅ DO THIS:

1. **Correct nRF24L01+ Power**
   ```
   Arduino 3.3V → [10µF Cap] → nRF24L01+ VCC ✓
                     |
                    GND
   ```

2. **Single BEC Connection**
   ```
   ESC1 BEC Red → Arduino VIN ✓
   ESC2 BEC Red → Cut or tape off ✓
   All ESC Black → Arduino GND ✓
   ```

3. **Correct Motor Pattern**
   ```
      FL(CCW)  FR(CW)
         \      /
          \    /
           \  /
            \/
            /\
           /  \
          /    \
         /      \
      RL(CW)  RR(CCW)  ✓
   ```

## Wire Gauge Recommendations

| Connection | Wire Gauge | Max Length |
|------------|-----------|------------|
| Battery → ESC | 12-14 AWG | Short as possible |
| ESC → Motor | 16-18 AWG | 5-10cm |
| ESC Signal | 22-24 AWG | 10-15cm |
| nRF24L01+ | 24-26 AWG | <10cm |
| I2C (MPU6050) | 24-26 AWG | <15cm |
| Joysticks | 24-26 AWG | <30cm |
| Buttons/Switches | 24-26 AWG | <30cm |

## Soldering Tips

1. **Clean Tips:**
   - Use flux on all joints
   - Clean soldering iron tip regularly
   - Use appropriate temperature (350-400°C)

2. **Good Solder Joints:**
   - Shiny, smooth surface
   - Cone-shaped
   - Good wire-to-pad contact

3. **Heat Shrink:**
   - Use on all exposed connections
   - Cover before soldering
   - Shrink with heat gun

4. **Strain Relief:**
   - Use hot glue on wire exits
   - Tie down heavy wires
   - Don't stress solder joints

## Testing Checklist

Before first power-on:

- [ ] Visual inspection of all connections
- [ ] Continuity test: All grounds connected
- [ ] No shorts between VCC and GND
- [ ] nRF24L01+ has 10µF capacitor
- [ ] Only one ESC BEC connected to VIN
- [ ] All motor directions correct
- [ ] All solder joints solid
- [ ] No loose wires near propellers
- [ ] Battery polarity correct
- [ ] Voltage check: 3.3V at nRF24L01+ and MPU6050

## Troubleshooting Wiring Issues

### No Radio Communication
- Check nRF24L01+ VCC is 3.3V (NOT 5V!)
- Add/replace 10µF capacitor
- Verify SPI connections (MOSI, MISO, SCK)
- Check CE and CSN pins match code

### MPU6050 Not Detected
- Verify I2C connections (SDA=A4, SCL=A5)
- Check 3.3V power to MPU6050
- Try different I2C address (0x68 or 0x69)
- Check for solder bridges

### Motors Don't Spin
- Verify ESC signal wire connections
- Check ESC calibration completed
- Ensure ESC BEC powering Arduino
- Test with servo tester

### Erratic Behavior
- Check all ground connections
- Verify power supply stable
- Keep nRF24L01+ away from motors
- Add capacitors to power rails

---

**Double-check everything before powering on!**

For operation instructions, see OPERATION_GUIDE.md
For technical details, see README.md
