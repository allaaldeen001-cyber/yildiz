# Detailed Wiring Diagrams

## Overview

This guide provides detailed wiring instructions for building the Yildiz drone system.

---

## Transmitter Wiring

### Arduino Nano/Uno Pinout

```
                    Arduino Nano/Uno
                   ┌─────────────┐
                   │             │
        (Reset) RST│1          30│A7
               3V3│2          29│A6
               Ref│3          28│A5 ← MPU6050 SCL
          Throttle│A0 4       27│A4 ← MPU6050 SDA
               Yaw│A1 5       26│A3 ← Roll Stick
             Pitch│A2 6       25│A2 ← Pitch Stick
               N/C│A3 7       24│A1 ← Yaw Stick
               GND│8          23│A0 ← Throttle Stick
               N/C│9          22│REF
               N/C│10         21│GND
        Arm Switch│D7 11      20│VIN
       Mode Switch│D8 12      19│D13 → NRF24 SCK
      NRF24 CE Pin│D9 13      18│D12 ← NRF24 MISO
     NRF24 CSN Pin│D10 14     17│D11 → NRF24 MOSI
               N/C│D11 15     16│D10 → NRF24 CSN
               N/C│D12 15     16│D9  → NRF24 CE
               N/C│D13 15     15│D8  ← Mode SW
               GND│GND        14│D7  ← Arm SW
                   └─────────────┘
```

### Transmitter Complete Wiring Table

| Component | Pin | Arduino Pin | Notes |
|-----------|-----|-------------|-------|
| **NRF24L01** |
| CE | 1 | D9 | Chip Enable |
| CSN | 2 | D10 | SPI Chip Select |
| SCK | 3 | D13 | SPI Clock |
| MOSI | 4 | D11 | SPI Master Out |
| MISO | 5 | D12 | SPI Master In |
| VCC | 6 | 3.3V | **Use external regulator!** |
| GND | 7 | GND | Common ground |
| **Left Joystick** |
| VRx (H) | - | A1 | Yaw control |
| VRy (V) | - | A0 | Throttle control |
| +5V | - | 5V | Power |
| GND | - | GND | Ground |
| **Right Joystick** |
| VRx (H) | - | A3 | Roll control |
| VRy (V) | - | A2 | Pitch control |
| +5V | - | 5V | Power |
| GND | - | GND | Ground |
| **Switches** |
| Arm Switch | - | D7 | With internal pullup |
| Mode Switch | - | D8 | With internal pullup |
| Common | - | GND | Ground |
| **Status LED** |
| Anode (+) | - | D13 | Built-in LED |
| Cathode (-) | - | GND | Via 220Ω resistor if external |
| **Power** |
| Battery + | - | VIN | 7-12V recommended |
| Battery - | - | GND | Common ground |

### NRF24L01 Power Circuit (CRITICAL!)

```
                     External 3.3V Regulator
                     (AMS1117-3.3 or similar)

Battery 7-12V ──┬──────┐
                │      │
                │      │ AMS1117-3.3
                │      │  IN     OUT
                │      ├───┬───────┬──────┐
                │          │       │      │
                │          │     [100uF]  │
                │          │     (Elec)   │
                │          │       │      │
                │          │      GND   [0.1uF]
                │          │            (Cer)
                │          │              │
                │      Arduino VIN      NRF24 VCC
                │          │              │
               GND ────────┴──────────────┴── GND
                
Legend:
  [100uF] = 100uF Electrolytic capacitor (>10V)
  [0.1uF] = 0.1uF Ceramic capacitor
  
IMPORTANT: Arduino's onboard 3.3V regulator is NOT sufficient!
           Use separate regulator with proper capacitors!
```

### Joystick Connection Detail

```
     Left Joystick               Right Joystick
     (Throttle/Yaw)              (Pitch/Roll)
     
    ┌─────────┐                  ┌─────────┐
    │  ┌───┐  │                  │  ┌───┐  │
    │  │ O │  │ Stick            │  │ O │  │ Stick
    │  └───┘  │                  │  └───┘  │
    │         │                  │         │
    └─────────┘                  └─────────┘
    │ │ │ │ │                    │ │ │ │ │
    G 5 X Y S                    G 5 X Y S
    │ │ │ │ │                    │ │ │ │ │
    │ │ │ │ └─── SW (not used)  │ │ │ │ └─── SW (not used)
    │ │ │ └───── VRy -> A0       │ │ │ └───── VRy -> A2
    │ │ └─────── VRx -> A1       │ │ └─────── VRx -> A3
    │ └───────── +5V             │ └───────── +5V
    └─────────── GND             └─────────── GND
```

---

## Receiver (Drone) Wiring

### Arduino Nano/Uno Pinout for Receiver

```
                    Arduino Nano/Uno
                   ┌─────────────┐
                   │             │
        (Reset) RST│1          30│A7
               3V3│2          29│A6
               Ref│3          28│A5 ← MPU6050 SCL
       Battery Mon│A0 4       27│A4 ← MPU6050 SDA
               N/C│A1 5       26│A3
               N/C│A2 6       25│A2
               N/C│A3 7       24│A1
               GND│8          23│A0 ← Battery Monitor
               N/C│9          22│REF
               N/C│10         21│GND
               N/C│D7 11      20│VIN
               N/C│D8 12      19│D13 → NRF24 SCK & LED
      NRF24 CE Pin│D9 13      18│D12 ← NRF24 MISO
     NRF24 CSN Pin│D10 14     17│D11 → NRF24 MOSI & M4
          Motor 4│D11 15     16│D10 → NRF24 CSN
          Motor 3│D6  15     15│D9  → NRF24 CE
          Motor 2│D5  15     14│D6  → Motor 3
          Motor 1│D3  15     13│D5  → Motor 2
               GND│GND        12│D3  → Motor 1
                   └─────────────┘
```

### Receiver Complete Wiring Table

| Component | Pin | Arduino Pin | Notes |
|-----------|-----|-------------|-------|
| **NRF24L01** |
| CE | 1 | D9 | Chip Enable |
| CSN | 2 | D10 | SPI Chip Select |
| SCK | 3 | D13 | SPI Clock |
| MOSI | 4 | D11 | SPI Master Out |
| MISO | 5 | D12 | SPI Master In |
| VCC | 6 | 3.3V | **Use external regulator!** |
| GND | 7 | GND | Common ground |
| **MPU6050 IMU** |
| VCC | 1 | 5V | Power |
| GND | 2 | GND | Ground |
| SCL | 3 | A5 | I2C Clock |
| SDA | 4 | A4 | I2C Data |
| XDA | 5 | N/C | Not used |
| XCL | 6 | N/C | Not used |
| AD0 | 7 | GND | I2C Address select |
| INT | 8 | N/C | Not used (optional) |
| **ESC 1 (Front-Left)** |
| Signal | White | D3 | PWM control |
| +5V | Red | N/C | BEC (not needed) |
| GND | Black | GND | Common ground |
| **ESC 2 (Front-Right)** |
| Signal | White | D5 | PWM control |
| +5V | Red | N/C | BEC (use only one!) |
| GND | Black | GND | Common ground |
| **ESC 3 (Back-Right)** |
| Signal | White | D6 | PWM control |
| +5V | Red | N/C | BEC (not needed) |
| GND | Black | GND | Common ground |
| **ESC 4 (Back-Left)** |
| Signal | White | D11 | PWM control |
| +5V | Red | Arduino VIN | Use ONE ESC BEC for Arduino |
| GND | Black | GND | Common ground |
| **Battery Monitor** |
| Sense | - | A0 | Via voltage divider |
| GND | - | GND | Ground |
| **Status LED** |
| Anode (+) | - | D13 | Built-in LED |
| Cathode (-) | - | GND | - |

### Battery Voltage Monitor Circuit

```
                Voltage Divider for Battery Monitoring
                
    Battery + (11.1V - 16.8V)
         │
         │
        [R1]  10kΩ  (adjust for your battery)
         │
         ├──────────→ Arduino A0 (0-5V)
         │
        [R2]  2.2kΩ
         │
        GND
        
    Calculation:
    Vout = Vin × (R2 / (R1 + R2))
    
    For 3S LiPo (12.6V max):
    Vout = 12.6V × (2.2k / (10k + 2.2k)) = 2.27V ✓
    
    For 4S LiPo (16.8V max):
    Use R1=18kΩ, R2=2.2kΩ
    Vout = 16.8V × (2.2k / (18k + 2.2k)) = 1.83V ✓
    
    Adjust in code:
    batteryVoltage = analogRead(A0) × (R1+R2) / R2 × 5.0 / 1023.0
```

### Complete Drone Power Distribution

```
                    Power Distribution Diagram
                    
                          LiPo Battery
                           (3S/4S)
                              │
                              ├─────────────┬──────────────┬──────────────┬─────────┐
                              │             │              │              │         │
                           ESC 1         ESC 2          ESC 3          ESC 4    Voltage
                              │             │              │              │      Divider
                           Motor 1       Motor 2        Motor 3       Motor 4      │
                          (FL-CW)      (FR-CCW)       (BR-CW)       (BL-CCW)      │
                              │             │              │              │         │
                           [PWM D3]     [PWM D5]       [PWM D6]      [PWM D11]    A0
                              │             │              │              │
                              └─────────────┴──────────────┴──────────────┘
                                             │
                                          GND Common
                                             │
                                    ┌────────┴────────┐
                                    │                 │
                                 Arduino           NRF24
                                   GND             GND
                                    │
                                 Arduino VIN ← ESC 4 BEC (5V)
                                 Arduino 5V  → MPU6050 VCC
                                 3.3V Reg    → NRF24 VCC
                                 
Important:
  - Use ONE ESC BEC to power Arduino (ESC 4 recommended)
  - ALL grounds must be connected together
  - NRF24 needs separate 3.3V regulator with capacitors
  - Keep motor wires short (<15cm from ESC)
```

### ESC to Motor Connection

```
        ESC                    Brushless Motor
        
     ┌──────┐               ┌───────────┐
     │      │───Yellow──────│  Phase A  │
     │ ESC  │───Blue────────│  Phase B  │
     │      │───Red─────────│  Phase C  │
     └──────┘               └───────────┘
        │ │ │
        │ │ └── GND (Black)
        │ └──── +5V (Red) - BEC output
        └────── Signal (White/Yellow) - from Arduino
        
    Motor Direction:
      - To reverse: Swap ANY 2 phase wires
      - Test with props OFF first!
      
    Correct Directions (X Configuration):
      M1 (Front-Left):  Clockwise (CW)
      M2 (Front-Right): Counter-Clockwise (CCW)
      M3 (Back-Right):  Clockwise (CW)
      M4 (Back-Left):   Counter-Clockwise (CCW)
```

---

## Physical Layout

### Motor and Prop Configuration

```
                        FRONT
                          ↑
                          
         M1 (CW)                    M2 (CCW)
         ╔═══╗                      ╔═══╗
         ║ ↻ ║────────────────────→║ ↺ ║
         ╚═══╝         ARM          ╚═══╝
           │                          │
    Prop: │ Normal CW         CCW Prop│
          │                            │
          │         DRONE              │
          │      CENTERPLATE           │
          │    (Flight Controller)     │
          │                            │
          │                            │
         ╔═══╗                      ╔═══╗
         ║ ↺ ║←───────────────────→║ ↻ ║
         ╚═══╝         ARM          ╚═══╝
         M4 (CCW)                   M3 (CW)
         
                        BACK
                          ↓

Legend:
  ↻ = Clockwise rotation
  ↺ = Counter-clockwise rotation
  
Front = Direction of flight (usually marked with colored arms)
```

### Component Placement Recommendations

```
                    TOP VIEW OF DRONE
                    
                      Front (↑)
                          
         Motor 1 ●                    ● Motor 2
                  \                  /
                   \                /
                    \   ┌────┐    /
                     \  │FC +│   /
                      \ │MPU │  /
                       \└────┘ /
                        \    /
                         \  /
                          \/
                          
                         Battery
                       (underneath)
                       
                          /\
                         /  \
                        /    \
                       /      \
                      /        \
                     /          \
         Motor 4 ●                    ● Motor 3
         
                      Back (↓)
                      
Key Points:
  - Flight controller in center (near CG)
  - MPU6050 on FC board, aligned with frame
  - Battery underneath, centered
  - ESCs on arms near motors (cooling)
  - NRF24 antenna away from motors (interference)
  - Keep wiring neat and secure
```

---

## Cable Management Tips

1. **Keep Motor Wires Short**
   - ESC should be close to motor (<15cm)
   - Reduces resistance and interference

2. **Separate Signal from Power**
   - Route signal wires away from motor wires
   - Prevents electrical noise interference

3. **Secure Everything**
   - Use zip ties or hot glue
   - No loose wires to catch in props!

4. **Label Wires**
   - Mark M1, M2, M3, M4 on ESC wires
   - Easier troubleshooting

5. **Use Proper Connectors**
   - XT60 for battery
   - Bullet connectors for motors (3.5mm or 4mm)
   - JST for small signals

---

## Testing Without Props

### Bench Test Setup

```
    Power Supply (or Battery)
            │
            ├──→ ESC 1 ──→ Motor 1 (no prop!)
            ├──→ ESC 2 ──→ Motor 2 (no prop!)
            ├──→ ESC 3 ──→ Motor 3 (no prop!)
            ├──→ ESC 4 ──→ Motor 4 (no prop!)
            └──→ Arduino VIN
            
    Transmitter nearby (powered separately)
    
    Safety:
      - Props OFF!
      - Secure drone (can't fly)
      - Low throttle only
      - Fingers away from motors
```

---

## Recommended Wire Gauges

| Connection | Wire Gauge | Notes |
|------------|------------|-------|
| Battery to ESC | 16-18 AWG | High current |
| ESC to Motor | 18-20 AWG | Per ESC spec |
| ESC BEC to Arduino | 22 AWG | 5V signal |
| Signal wires | 24-26 AWG | Low current |
| I2C (MPU6050) | 28 AWG | Short runs |
| Battery monitor | 28 AWG | Low current |

---

## Common Wiring Mistakes

❌ **Wrong:**
- Using Arduino's 3.3V for NRF24 (insufficient current)
- No capacitors on NRF24 power
- Motors directly to Arduino (need ESCs!)
- Multiple ESC BECs connected (voltage conflict)
- Props on during testing

✅ **Correct:**
- External 3.3V regulator for NRF24
- 100uF + 0.1uF caps on NRF24
- ESCs between Arduino and motors
- Only ONE ESC BEC powers Arduino
- Props off until verified safe

---

## Soldering Tips

1. **Use Flux** - Makes everything easier
2. **Proper Heat** - 350-380°C for electronics
3. **Tin Both Sides** - Wire and pad
4. **Quick Contact** - 1-2 seconds max
5. **Inspect Joints** - Shiny, smooth, no bridges
6. **Strain Relief** - Hot glue after soldering

---

## Final Checklist Before First Power-On

- [ ] All connections double-checked
- [ ] No shorts (use multimeter continuity test)
- [ ] Correct voltage to each component
- [ ] NRF24 has capacitors
- [ ] Battery voltage correct
- [ ] Props OFF
- [ ] Fire extinguisher ready (LiPo safety)
- [ ] Code uploaded successfully
- [ ] Serial monitor ready

---

**Take your time with wiring - it's the foundation of a stable drone!** 🔌
