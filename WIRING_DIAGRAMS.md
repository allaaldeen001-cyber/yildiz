# 🔌 Detailed Wiring Diagrams

## Flight Controller Wiring

### Complete System Overview

```
                              FLIGHT CONTROLLER
                    ┌─────────────────────────────────┐
                    │                                 │
    ┌───────────────┤        ARDUINO NANO            ├────────────────┐
    │               │                                 │                │
    │               └─────────────────────────────────┘                │
    │                                                                  │
    │                                                                  │
┌───▼─────┐     ┌──────────┐      ┌──────────┐      ┌──────────┐     │
│ MPU6050 │     │ MS5611   │      │ NRF24L01 │      │  OUTPUTS │     │
│         │     │          │      │          │      │          │     │
│ VCC ────┼─────┼─── VCC   │      │ VCC ─────┼──────┤ LED  ─── D7   │
│ GND ────┼─────┼─── GND   │      │ GND ─────┼──────┤ BUZZ ─── D8   │
│ SDA ────┼─────┼─── SDA ──┼──────┤ CE ────────────── D4             │
│ SCL ────┼─────┼─── SCL ──┼──────┤ CSN ───────────── D10            │
│         │     │          │      │ SCK ───────────── D13            │
└─────────┘     └──────────┘      │ MOSI ──────────── D11            │
                                  │ MISO ──────────── D12            │
                                  └──────────┘                        │
                                                                      │
    ┌─────────────────────────────────────────────────────────────────┘
    │
    │  MOTOR ESCs (Signal wires only)
    │
    ├──── ESC FL ───── Motor Front Left  ⟲ CCW
    │     (D3)
    │
    ├──── ESC FR ───── Motor Front Right ⟳ CW
    │     (D5)
    │
    ├──── ESC RR ───── Motor Rear Right  ⟲ CCW
    │     (D6)
    │
    └──── ESC RL ───── Motor Rear Left   ⟳ CW
          (D9)

                    POWER DISTRIBUTION
                    
    [3S LiPo] ──┬──[ESC FL]──[Motor FL]
    11.1V       │
                ├──[ESC FR]──[Motor FR]
                │
                ├──[ESC RR]──[Motor RR]
                │
                ├──[ESC RL]──[Motor RL]
                │
                └──[5V BEC]──[Arduino VIN]
                   (from any ESC)

    Note: All grounds must be connected together!
```

---

### NRF24L01 Connection Detail

```
        NRF24L01+ MODULE              ARDUINO NANO
    ┌─────────────────────┐
    │  ┌───┐   ┌───┐     │
    │  │   │   │ o │     │           3.3V ────┐
    │  │ A │   │ o │ GND ├────────── GND      │
    │  │ N │   │ o │ CE  ├────────── D4       │
    │  │ T │   │ o │ CSN ├────────── D10      │
    │  │   │   │ o │ SCK ├────────── D13      │
    │  └───┘   │ o │MOSI ├────────── D11      │
    │          │ o │MISO ├────────── D12      │
    │          │ o │ IRQ │ (not used)         │
    │          │ o │ VCC ├────────── 3.3V ◄───┘
    │          └───┘     │
    └─────────────────────┘
    
    ⚠️ CRITICAL: Add 10µF capacitor between VCC and GND
                 on the NRF module!
    
    Recommended: Use NRF adapter board with voltage
                 regulator and built-in capacitor
```

---

### I2C Sensors Connection (MPU6050 + MS5611)

```
    ARDUINO NANO          MPU6050 (GY-521)      MS5611 (GY-63)
    
    5V ──────────────┬──── VCC                    (not connected)
                     │
    3.3V ────────────┼──────────────────────────── VCC
                     │
    GND ─────────────┼──── GND ────────────────── GND
                     │
    A4 (SDA) ────────┼──── SDA ────────────────── SDA
                     │
    A5 (SCL) ────────┴──── SCL ────────────────── SCL
    
    Note: Both sensors share the same I2C bus
    MPU6050: I2C Address 0x68
    MS5611:  I2C Address 0x77
    
    ⚠️ Check your MS5611 module voltage rating!
       Some require 5V, others 3.3V
```

---

### LED & Buzzer Connection

```
    LED Connection:
    
    Arduino D7 ──── [220Ω Resistor] ──── LED Anode (+)
                                          │
                                          LED
                                          │
    Arduino GND ──────────────────────── LED Cathode (-)
    
    (Flat side of LED = cathode = -)
    (Longer leg of LED = anode = +)
    
    
    Buzzer Connection:
    
    Arduino D8 ──── Buzzer + (Red wire)
    
    Arduino GND ─── Buzzer - (Black wire)
    
    Note: Active buzzers work best (no external oscillator needed)
```

---

### ESC Signal Wire Connections

```
                    ARDUINO NANO
                    
                    ┌─────────┐
                    │         │
         D3 ────────┤ o     o │──── D9
                    │         │
         D4         │ o     o │──── D8 (Buzzer)
                    │         │
         D5 ────────┤ o     o │──── D7 (LED)
                    │         │
         D6 ────────┤ o     o │
                    │         │
                    └─────────┘
    
    ESC FL ── White (signal) ──► D3
    ESC FR ── White (signal) ──► D5
    ESC RR ── White (signal) ──► D6
    ESC RL ── White (signal) ──► D9
    
    Each ESC:
    - Red wire → Power distribution (not to Arduino!)
    - Black wire → Common ground
    - White/Yellow wire → Arduino signal pin
    
    Power comes from one ESC's BEC (5V regulator)
    Connect one ESC's red wire to Arduino VIN
```

---

### Complete Power Distribution

```
    ┌──────────────┐
    │  3S LiPo     │
    │  11.1V       │
    │  2200mAh+    │
    └──┬───────┬───┘
       │       │
       +       -
       │       │
    ┌──▼───────▼───────────────────┐
    │  Power Distribution Board    │
    │  (or wire junction)          │
    └┬────┬────┬────┬──────────────┘
     │    │    │    │
     │    │    │    └───► ESC RL → Motor RL
     │    │    │
     │    │    └────────► ESC RR → Motor RR
     │    │
     │    └─────────────► ESC FR → Motor FR
     │
     └──────────────────► ESC FL → Motor FL
                          │
                          └─► 5V BEC ─► Arduino VIN
                          
    ⚠️ CRITICAL: Connect ALL grounds together:
       - Battery ground
       - All 4 ESC grounds
       - Arduino ground
       - All sensor grounds
```

---

## Remote Controller Wiring

### Complete System Overview

```
                         REMOTE CONTROLLER
                    ┌─────────────────────────┐
                    │    ARDUINO NANO         │
                    └────────┬────────────────┘
                             │
        ┌────────────────────┼──────────────────────┐
        │                    │                      │
        │                    │                      │
    ┌───▼────┐       ┌───────▼──────┐      ┌───────▼────────┐
    │JOYSTICK│       │   NRF24L01   │      │   CONTROLS     │
    │  LEFT  │       │              │      │                │
    │        │       │ CE  ─────── D9      │ Switch 1 ─── D3│
    │ X ─ A1 │       │ CSN ──────── D10    │ Switch 2 ─── D2│
    │ Y ─ A0 │       │ SCK ──────── D13    │ Button 1 ─── D4│
    │        │       │ MOSI ─────── D11    │ Button 2 ─── D5│
    └────────┘       │ MISO ─────── D12    │                │
                     │ VCC ──────── 3.3V   └────────────────┘
    ┌────────┐       │ GND ──────── GND    
    │JOYSTICK│       └──────────────┘      
    │  RIGHT │       
    │        │       
    │ X ─ A3 │       
    │ Y ─ A2 │       
    │        │       
    └────────┘       
    
    Power: 9V Battery or 2S LiPo → Arduino VIN
```

---

### Joystick Connections

```
    LEFT JOYSTICK              ARDUINO NANO
    
    ┌─────────────┐
    │   ╔═══╗     │           A0 ◄── Y-axis (Throttle)
    │   ║   ║     │           A1 ◄── X-axis (Yaw)
    │   ║ • ║     │           5V ◄── VCC
    │   ║   ║     │          GND ◄── GND
    │   ╚═══╝     │          (SW not used)
    │             │
    │  VCC GND SW │
    │   X   Y     │
    └─────────────┘
    
    
    RIGHT JOYSTICK             ARDUINO NANO
    
    ┌─────────────┐
    │   ╔═══╗     │           A2 ◄── Y-axis (Pitch)
    │   ║   ║     │           A3 ◄── X-axis (Roll)
    │   ║ • ║     │           5V ◄── VCC
    │   ║   ║     │          GND ◄── GND
    │   ╚═══╝     │          (SW not used)
    │             │
    │  VCC GND SW │
    │   X   Y     │
    └─────────────┘
    
    Typical Joystick Pinout:
    - VCC: 5V power
    - GND: Ground
    - X:   X-axis output (analog 0-1023)
    - Y:   Y-axis output (analog 0-1023)
    - SW:  Push button (not used in this project)
```

---

### Switches & Buttons

```
    SWITCHES (SPDT Toggle)
    
    Switch 1 (Arm/Disarm):
    
        ┌──o──┐
        │  │  │
    C ──┤  │  ├── NC (not used)
        │  │  │
        │  └──┼── NO → Arduino D3
        │     │
        └─────┴─────► Arduino GND
    
    Common (C) to GND
    Normally Open (NO) to D3
    
    Switch 2 (Altitude Hold) - same wiring to D2
    
    
    BUTTONS (Momentary Push)
    
    Button 1 (Calibration):
    
        ┌───┐
        │   │
        │ • │──── Arduino D4
        │   │
        └───┘──── Arduino GND
    
    One pin to D4, other pin to GND
    
    Button 2 (Motor Start) - same wiring to D5
    
    Note: Arduino internal pull-up resistors are used,
          so no external resistors needed!
```

---

### Remote Controller Power

```
    Option 1: 9V Battery (Easiest)
    
    ┌──────────┐
    │ 9V       │
    │ Battery  │
    └─┬──────┬─┘
      +      -
      │      │
      │      │
    ┌─▼──────▼─┐
    │  Arduino │
    │  VIN GND │
    └──────────┘
    
    Pros: Simple, easy to replace
    Cons: Shorter runtime (~2-3 hours)
    
    
    Option 2: 2S LiPo (Better)
    
    ┌──────────┐
    │ 2S LiPo  │
    │ 7.4V     │
    └─┬──────┬─┘
      +      -
      │      │
    ┌─▼──────▼──┐
    │ ON/OFF    │ ← Add power switch!
    │ Switch    │
    └─┬──────┬──┘
      │      │
    ┌─▼──────▼─┐
    │  Arduino │
    │  VIN GND │
    └──────────┘
    
    Pros: Longer runtime (6-8 hours), rechargeable
    Cons: Need charger, more expensive
```

---

## Wire Color Guide

### Recommended Color Coding

```
    POWER & GROUND:
    Red ────────────► Positive (+) / VCC
    Black ──────────► Negative (-) / GND
    
    I2C BUS:
    Yellow ─────────► SDA (Serial Data)
    White ──────────► SCL (Serial Clock)
    
    SPI BUS:
    Orange ─────────► CE (Chip Enable)
    Yellow ─────────► CSN (Chip Select Not)
    Blue ───────────► MOSI (Master Out Slave In)
    Green ──────────► MISO (Master In Slave Out)
    Purple ─────────► SCK (Serial Clock)
    
    SIGNALS:
    White/Yellow ───► PWM signals (ESCs)
    Various ────────► Analog inputs (joysticks)
```

---

## Common Wiring Mistakes ⚠️

### DON'T DO THIS:

```
❌ WRONG: Connecting 5V to NRF24L01 VCC
    Arduino 5V ──X──► NRF VCC
    (Will damage NRF! Use 3.3V)

❌ WRONG: No capacitor on NRF
    (Causes unstable connection)

❌ WRONG: ESC power to Arduino VIN
    3S LiPo (11.1V) ──X──► Arduino VIN
    (Too high! Use ESC BEC 5V output)

❌ WRONG: Separate grounds
    (All grounds MUST be connected)

❌ WRONG: LED without resistor
    Arduino pin ──X──► LED ──► GND
    (Will burn out LED!)
```

### DO THIS INSTEAD:

```
✅ CORRECT: NRF with 3.3V
    Arduino 3.3V ──►[10µF cap]──► NRF VCC
                          └────► NRF GND

✅ CORRECT: ESC BEC to Arduino
    ESC 5V BEC ──────────► Arduino VIN
    ESC GND ─────────────► Arduino GND

✅ CORRECT: All grounds connected
    Battery GND ┐
    ESC GND     ├─── Common Ground
    Arduino GND │
    Sensor GND  ┘

✅ CORRECT: LED with resistor
    Arduino pin ──►[220Ω]──► LED (+) ──► LED (-) ──► GND
```

---

## Testing Connections

### Continuity Test (Multimeter)

```
    Before powering on:
    
    1. Check for shorts:
       [Multimeter in continuity mode]
       
       Test VCC to GND: Should be OPEN (no beep)
       ❌ If beeps = SHORT CIRCUIT! Find and fix!
    
    2. Check connections:
       Test signal wires: Should have continuity
       ✅ Beeps = good connection
       ❌ No beep = broken wire
    
    3. Check power rails:
       Test all GND pins: Should all beep (connected)
       ✅ All grounds must be common
```

### Voltage Test

```
    After powering on:
    
    1. Check Arduino:
       VIN pin: Should read battery voltage
       5V pin:  Should read ~5.0V
       3.3V pin: Should read ~3.3V
       GND pin: Should read 0V
    
    2. Check NRF:
       VCC: Should read 3.3V (NOT 5V!)
       If reading 5V → DANGER! Turn off immediately
    
    3. Check sensors:
       MPU6050 VCC: 5V
       MS5611 VCC: 3.3V or 5V (check your module)
```

---

## Cable Management Tips

```
    FLIGHT CONTROLLER:
    
    1. Keep wires SHORT
       - Reduces weight
       - Reduces interference
       - Cleaner build
    
    2. Route wires UNDER arms
       - Protects from props
       - Looks professional
    
    3. Use zip ties at stress points
       - ESC connections
       - Motor wires
       - Battery lead
    
    4. Label everything!
       - Use tape + marker
       - Note FL, FR, RL, RR
       - Makes repairs easier
    
    
    REMOTE CONTROLLER:
    
    1. Keep wires organized
       - Bundle with zip ties
       - Use perf board if possible
       - Hot glue to secure
    
    2. Strain relief
       - Don't pull on solder joints
       - Glue wires near connections
    
    3. Test before closing enclosure!
       - Verify all functions work
       - Easy to fix while open
```

---

## Soldering Tips

```
    GOOD SOLDER JOINT:
    
        ┌────┐
        │Wire│
        └──┬─┘
           │
        ┌──▼──┐ ← Smooth cone shape
        │ /\ │   Shiny surface
        │/  \│   No cold joints
        └────┘
          Pad
    
    
    BAD SOLDER JOINT:
    
        Blob         ← Too much solder
        Cold joint   ← Dull, grainy appearance  
        Bridge       ← Solder touching adjacent pins
        Dry joint    ← Not enough solder
```

---

## Final Assembly Checklist

### Before First Power-On:

- [ ] Visual inspection of all connections
- [ ] Continuity test (no shorts!)
- [ ] Double-check polarity (especially power!)
- [ ] Verify NRF gets 3.3V, not 5V
- [ ] Capacitor on NRF module
- [ ] All grounds connected together
- [ ] No loose wires touching
- [ ] ESC signal wires to correct pins
- [ ] LED resistor installed
- [ ] Battery secured (will not move)
- [ ] Propellers REMOVED for testing

### After First Power-On:

- [ ] Voltage test all power rails
- [ ] Check for hot components
- [ ] Verify LED lights up
- [ ] Test buzzer beeps
- [ ] Open Serial Monitor
- [ ] Verify sensor detection
- [ ] Test RC connection
- [ ] Test motor response (no props!)

---

**Print this guide and keep it handy during assembly! 📋**

*Double-check everything before connecting power!*
