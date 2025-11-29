# Wiring Diagrams

Complete wiring guide for the Professional Arduino Nano Drone System.

## 📌 Important Notes

- **Power**: Ensure proper power distribution. NRF24L01 PA+LNA modules require 3.3V with adequate current (use capacitors)
- **Decoupling**: Add 10μF capacitor across NRF24L01 power pins (very close to module)
- **Ground**: Common ground between all components is critical
- **ESCs**: Calibrate ESCs before first flight
- **Props**: REMOVE propellers during testing!

---

## Flight Controller Wiring

### Arduino Nano Pinout Reference
```
         ┌─────────────┐
    D13 ─┤ SCK    VIN  ├─ 7-12V (from BEC/ESC)
    D12 ─┤ MISO   GND  ├─ GND
    D11 ─┤ MOSI   RST  ├─ 
    D10 ─┤ CSN    5V   ├─ 5V
     D9 ─┤ RL     A7   ├─
     D8 ─┤ BUZ    A6   ├─
     D7 ─┤ LED    A5   ├─ SCL (MPU6050)
     D6 ─┤ RR     A4   ├─ SDA (MPU6050)
     D5 ─┤ FR     A3   ├─
     D4 ─┤ CE     A2   ├─
     D3 ─┤ FL     A1   ├─
     D2 ─┤ INT    A0   ├─
         └─────────────┘
```

### NRF24L01 PA+LNA Module

**CRITICAL**: Add 10μF capacitor between VCC and GND at module!

```
NRF24L01           Arduino Nano
───────────────────────────────
GND         ────►   GND
VCC (3.3V)  ────►   3.3V (NOT 5V!)
CE          ────►   D4
CSN         ────►   D10
SCK         ────►   D13
MOSI        ────►   D11
MISO        ────►   D12
IRQ         ────►   Not Connected

Capacitor: 10μF between VCC-GND (physically close to module)
```

**Recommended Power Solution**:
Since Arduino Nano's 3.3V regulator may not provide enough current for PA+LNA:
- Option 1: Use external 3.3V regulator (AMS1117-3.3) from 5V rail
- Option 2: Use LC filter from 3.3V pin with 100μF capacitor

### MPU6050 IMU

```
MPU6050            Arduino Nano
───────────────────────────────
VCC         ────►   5V (or 3.3V)
GND         ────►   GND
SCL         ────►   A5
SDA         ────►   A4
INT         ────►   D2 (optional)
AD0         ────►   GND (I2C address 0x68)
```

### ESC and Motor Connections

```
Motor Layout (X Configuration):
        FRONT
    FL      FR
      \    /
       \  /
        \/
        /\
       /  \
      /    \
    RL      RR
        REAR

ESC Signal Wires:
─────────────────
FL ESC Signal  ────►  D3 (Arduino Nano)
FR ESC Signal  ────►  D5 (Arduino Nano)
RR ESC Signal  ────►  D6 (Arduino Nano)
RL ESC Signal  ────►  D9 (Arduino Nano)

ESC Ground     ────►  GND (Arduino Nano)

Power Distribution:
──────────────────
Battery (+)    ────►  All ESC power inputs (red wires)
Battery (-)    ────►  All ESC ground (black wires)

BEC/5V Output from one ESC:
──────────────────────────
ESC 5V (red)   ────►  VIN or 5V (Arduino Nano)
ESC GND        ────►  GND (Arduino Nano)

IMPORTANT: Remove 5V wire from 3 ESCs, keep only one connected!
```

### Motor Rotation Directions

```
      FRONT
   FL ↻    ↺ FR
      \    /
       \  /
        \/
        /\
       /  \
      /    \
   RL ↺    ↻ RR
      REAR

↻ = Counter-Clockwise
↺ = Clockwise

If drone rotates opposite when yawing, swap two adjacent motor wires.
```

### Buzzer

```
Buzzer             Arduino Nano
───────────────────────────────
+ (Positive)  ────►  D8
- (Negative)  ────►  GND

Use active buzzer (has internal oscillator)
Or passive buzzer with series resistor (100-220Ω)
```

### Status LED

```
LED                Arduino Nano
───────────────────────────────
Anode (+)     ────►  D7
Cathode (-)   ────►  GND (via 220Ω resistor)

        D7 ─────┤>├─────[220Ω]───── GND
               LED
```

### Complete Flight Controller Schematic

```
┌─────────────────────────────────────────────────────────────────┐
│                    FLIGHT CONTROLLER WIRING                      │
└─────────────────────────────────────────────────────────────────┘

                   ┌────────────┐
                   │ LiPo 3S/4S │
                   │  Battery   │
                   └─┬────────┬─┘
                     │        │
                  [+]│        │[-]
                     │        │
        ┌────────────┴────────┴────────────┐
        │    Power Distribution Board       │
        │  (or solder connections)          │
        └┬───┬───┬───┬──────────────────┬──┘
         │   │   │   │                  │
    ┌────┴┐ ┌┴───┴┐ ┌┴───┐ ┌───┴──┐   │
    │ESC │ │ESC  │ │ESC │ │ESC   │   │
    │FL  │ │FR   │ │RR  │ │RL    │   │
    └┬─┬─┘ └┬──┬─┘ └┬─┬─┘ └┬──┬──┘   │
     │ │    │  │    │ │    │  │       │
     │ │    │  │    │ │    │  │       │
   [M][5V] [M][x] [M][x]  [M] [x]     │
     │  │   │      │       │           │
     │  └───┼──────┼───────┘           │
     │      │      │                   │
    D3      D5     D6     D9           │
     │      │      │      │            │
  ┌──┴──────┴──────┴──────┴────────────┴─────┐
  │         Arduino Nano                      │
  │  D2: MPU-INT   A4: MPU-SDA               │
  │  D4: NRF-CE    A5: MPU-SCL               │
  │  D7: LED       VIN: 5V from ESC BEC      │
  │  D8: BUZZER    GND: Common Ground        │
  │  D10: NRF-CSN  3.3V: NRF-VCC+Cap         │
  │  D11: NRF-MOSI                           │
  │  D12: NRF-MISO                           │
  │  D13: NRF-SCK                            │
  └──┬────┬────┬──────┬──────────────────────┘
     │    │    │      │
   [LED][BUZ][MPU][NRF24L01+Cap]
```

---

## Remote Controller Wiring

### Arduino Nano Pinout Reference
```
         ┌─────────────┐
    D13 ─┤ SCK    VIN  ├─ 7-9V (Battery)
    D12 ─┤ MISO   GND  ├─ GND
    D11 ─┤ MOSI   RST  ├─ 
    D10 ─┤ CSN    5V   ├─ 5V
     D9 ─┤ CE     A7   ├─
     D8 ─┤        A6   ├─
     D7 ─┤        A5   ├─
     D6 ─┤        A4   ├─
     D5 ─┤ BTN2   A3   ├─ R-Stick H (Roll)
     D4 ─┤ BTN1   A2   ├─ R-Stick V (Pitch)
     D3 ─┤ SW2    A1   ├─ L-Stick H (Yaw)
     D2 ─┤ SW1    A0   ├─ L-Stick V (Throttle)
         └─────────────┘
```

### NRF24L01 PA+LNA Module

Same as Flight Controller - add 10μF capacitor!

```
NRF24L01           Arduino Nano
───────────────────────────────
GND         ────►   GND
VCC (3.3V)  ────►   3.3V
CE          ────►   D9
CSN         ────►   D10
SCK         ────►   D13
MOSI        ────►   D11
MISO        ────►   D12

Capacitor: 10μF between VCC-GND
```

### Joysticks (Analog)

Standard dual-axis joysticks with potentiometers:

```
LEFT JOYSTICK (Throttle/Yaw):
──────────────────────────────
GND         ────►   GND
+5V         ────►   5V
VRx (Horiz) ────►   A1 (Yaw)
VRy (Vert)  ────►   A0 (Throttle)
SW (Button) ────►   Not connected

RIGHT JOYSTICK (Pitch/Roll):
──────────────────────────────
GND         ────►   GND
+5V         ────►   5V
VRx (Horiz) ────►   A3 (Roll)
VRy (Vert)  ────►   A2 (Pitch)
SW (Button) ────►   Not connected
```

### Buttons

Momentary push buttons (normally open):

```
BUTTON 1 (Calibration):
────────────────────────
Terminal 1  ────►   D4
Terminal 2  ────►   GND
(Internal pullup enabled in code)

BUTTON 2 (Motor Arm):
─────────────────────
Terminal 1  ────►   D5
Terminal 2  ────►   GND
(Internal pullup enabled in code)
```

### Toggle Switches

SPDT (Single Pole Double Throw) or simple switches:

```
SWITCH 1 (Altitude Hold):
─────────────────────────
Common      ────►   D2
Position 1  ────►   GND
Position 2  ────►   No connection
(Internal pullup: ON=open, OFF=to GND)

SWITCH 2 (Kill Switch / Arming):
────────────────────────────────
Common      ────►   D3
Position 1  ────►   GND
Position 2  ────►   No connection
(Internal pullup: ARMED=open, DISARMED=to GND)
```

### Power Supply

```
Battery (2S LiPo 7.4V or 9V):
─────────────────────────────
+ Terminal  ────►   VIN (Arduino Nano)
- Terminal  ────►   GND (Arduino Nano)

Recommended: 2S LiPo with JST connector
Alternative: 9V battery (shorter runtime)
```

### Complete Remote Controller Schematic

```
┌─────────────────────────────────────────────────────────────────┐
│                   REMOTE CONTROLLER WIRING                       │
└─────────────────────────────────────────────────────────────────┘

        ┌──────────┐
        │ 2S LiPo  │
        │  7.4V    │
        └─┬──────┬─┘
      [+] │      │ [-]
          │      │
       VIN│      │GND
    ┌─────┴──────┴─────────────────────────────┐
    │         Arduino Nano                      │
    │                                           │
    │  D2: SW1 (Alt Hold)                      │
    │  D3: SW2 (Kill Switch)                   │
    │  D4: BTN1 (Calibrate)                    │
    │  D5: BTN2 (Motor Arm)                    │
    │  D9: NRF-CE                              │
    │  D10: NRF-CSN                            │
    │  D11: NRF-MOSI                           │
    │  D12: NRF-MISO                           │
    │  D13: NRF-SCK                            │
    │  A0: L-Joy Vert (Throttle)               │
    │  A1: L-Joy Horiz (Yaw)                   │
    │  A2: R-Joy Vert (Pitch)                  │
    │  A3: R-Joy Horiz (Roll)                  │
    │  5V: Joystick power                      │
    │  GND: Common ground                      │
    │  3.3V: NRF power                         │
    └┬─┬─┬─┬──┬──┬──┬──┬──────────────────────┘
     │ │ │ │  │  │  │  │
    [SW1][SW2][BTN1][BTN2][L-Joy][R-Joy][NRF+Cap]
     │   │   │    │    │     │      │
    GND GND GND  GND  5V/G  5V/G   3.3V/G
```

---

## Assembly Tips

### 1. **Test Components Individually**
   - Test NRF24L01 modules with simple ping-pong sketch first
   - Verify MPU6050 reads correctly before mounting
   - Test each joystick axis separately

### 2. **Soldering**
   - Use quality solder (60/40 or lead-free)
   - Ensure all joints are shiny, not cold/dull
   - Test continuity with multimeter
   - No solder bridges between pins!

### 3. **Wire Management**
   - Use different colors: Red (+), Black (GND), other colors for signals
   - Keep wires short but not too tight
   - Secure wires with zip ties or heat shrink
   - Label wires with tape/marker

### 4. **Power Considerations**
   - NRF24L01 PA+LNA draws significant current - ALWAYS use capacitor
   - Separate analog and digital grounds if possible (star ground)
   - Use thick wires for battery/ESC power (at least 18-20 AWG)

### 5. **Vibration Damping (for Flight Controller)**
   - Mount MPU6050 on soft foam or gel pads
   - Keep IMU away from motors/ESCs
   - Ensure Arduino Nano is secure but isolated from vibrations

### 6. **Antenna Orientation**
   - Keep NRF24L01 antennas perpendicular to each other
   - Don't let metal frame/components block antenna
   - External antenna is better for PA+LNA modules

---

## Testing Procedure

### Stage 1: Power Test
1. Connect power (WITHOUT PROPELLERS!)
2. Check voltages: 5V rail should be 4.8-5.2V, 3.3V rail should be 3.2-3.4V
3. Verify no components get hot

### Stage 2: Communication Test
1. Upload code to both controllers
2. Power both on
3. Check serial monitor on RC - should show "CONNECTED"
4. Watch status LED on FC - should blink when linked

### Stage 3: Sensor Test
1. Move FC around, check angle readings on RC serial monitor
2. Press calibration button, verify 2 beeps on success
3. Move joysticks, verify values change on serial monitor

### Stage 4: ESC Test
1. **PROPELLERS REMOVED!**
2. Perform ESC calibration sequence
3. Verify each motor spins smoothly
4. Check motor directions match configuration

### Stage 5: Flight Test (with props)
1. Install propellers (correct rotation!)
2. Clear area of obstacles
3. Arm drone
4. Gentle throttle test at low altitude
5. Test pitch/roll/yaw at low altitude
6. Gradually increase confidence

---

## Troubleshooting

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| NRF not connecting | No capacitor, bad power | Add 10μF cap, check 3.3V supply |
| Motors don't spin | Not armed, bad ESC | Check arming sequence, test ESC |
| Drone unstable | Bad PID, wrong props | Reduce PID gains, check prop direction |
| One motor doesn't work | Bad connection | Check ESC signal wire, resolder |
| Calibration fails | Vibration, bad IMU | Place on stable surface, check MPU |
| RC loses signal | Antenna, interference | Reposition antenna, check frequency |

---

## Safety Checklist

- [ ] All solder joints verified
- [ ] No short circuits (multimeter continuity test)
- [ ] NRF24L01 has 10μF capacitor
- [ ] Correct motor rotation directions
- [ ] Propellers correct orientation (leading edge forward)
- [ ] Battery secure and properly connected
- [ ] Calibration completed successfully
- [ ] Kill switch tested and working
- [ ] Clear flight area with no people/obstacles
- [ ] First flight at low altitude

---

**⚠️ ALWAYS REMOVE PROPELLERS DURING TESTING AND DEBUGGING!**

