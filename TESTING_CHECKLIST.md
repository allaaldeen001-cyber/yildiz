# ✅ COMPREHENSIVE TESTING CHECKLIST

## Pre-Flight Testing & Verification Guide

This document provides a systematic approach to testing all drone systems and safety features before first flight.

---

## 🧰 Test Equipment Needed

- [ ] USB cable for Arduino programming
- [ ] Serial monitor (115200 baud)
- [ ] Multimeter (for voltage testing)
- [ ] Fully charged LiPo battery (11.1V 3S)
- [ ] LiPo voltage checker
- [ ] Propeller balancer (recommended)
- [ ] Small screwdriver set
- [ ] Zip ties or tape for securing wires

---

## 📋 PHASE 1: HARDWARE VERIFICATION (30 minutes)

### 1.1 Power System Tests

#### Battery Test
- [ ] Battery voltage: 12.0V - 12.6V (full charge)
- [ ] No physical damage or swelling
- [ ] Connectors secure and clean
- [ ] Balance plug intact

#### Arduino Power Test (RC Transmitter)
- [ ] Powers on via USB (5V)
- [ ] LED indicators work
- [ ] Can upload sketches successfully
- [ ] No overheating after 5 minutes

#### Arduino Power Test (Flight Controller)
- [ ] Powers on via USB (5V)
- [ ] Powers on via BEC/battery (5V regulated)
- [ ] LED indicators work
- [ ] Can upload sketches successfully
- [ ] No overheating after 5 minutes

#### Voltage Regulator Test
- [ ] 5V rail measures 4.9V - 5.2V
- [ ] 3.3V rail measures 3.2V - 3.4V
- [ ] No voltage drop under load

**CRITICAL**: NRF24L01 must be powered by **3.3V ONLY!**

---

### 1.2 Wiring Verification

#### Flight Controller Wiring
- [ ] NRF24 CE → D9
- [ ] NRF24 CSN → D10
- [ ] NRF24 MOSI → D11
- [ ] NRF24 MISO → D12
- [ ] NRF24 SCK → D13
- [ ] NRF24 VCC → 3.3V (NOT 5V!)
- [ ] NRF24 GND → GND
- [ ] MPU6050 SDA → A4
- [ ] MPU6050 SCL → A5
- [ ] MPU6050 VCC → 5V
- [ ] MPU6050 GND → GND
- [ ] MS5611 SDA → A4
- [ ] MS5611 SCL → A5
- [ ] MS5611 VCC → 5V
- [ ] MS5611 GND → GND
- [ ] ESC1 signal → D3
- [ ] ESC2 signal → D4
- [ ] ESC3 signal → D5
- [ ] ESC4 signal → D6
- [ ] Buzzer + → D7
- [ ] Buzzer - → GND
- [ ] Status LED + → D8
- [ ] LED + → D2
- [ ] All GND connected together

#### RC Transmitter Wiring
- [ ] NRF24 CE → D9
- [ ] NRF24 CSN → D10
- [ ] NRF24 MOSI → D11
- [ ] NRF24 MISO → D12
- [ ] NRF24 SCK → D13
- [ ] NRF24 VCC → 3.3V (NOT 5V!)
- [ ] NRF24 GND → GND
- [ ] Joystick1 VRx → A0
- [ ] Joystick1 VRy → A1
- [ ] Joystick2 VRx → A2
- [ ] Joystick2 VRy → A3
- [ ] Joysticks VCC → 5V
- [ ] Joysticks GND → GND
- [ ] Toggle switch → D2
- [ ] Button 1 → D3 (with pullup)
- [ ] Button 2 → D4 (with pullup)
- [ ] Status LED + → D8
- [ ] All GND connected together

---

### 1.3 Motor & Propeller Check

#### Motor Inspection (BATTERY DISCONNECTED!)
- [ ] All 4 motors spin freely by hand
- [ ] No grinding or resistance
- [ ] Bearings smooth
- [ ] No loose wires
- [ ] Motor mounts secure
- [ ] Motor screws tight

#### ESC Check
- [ ] All ESCs have power (5V BEC and battery power)
- [ ] Signal wires connected correctly
- [ ] ESC calibration beeps heard on power-up
- [ ] No damage to ESC

#### Propeller Check (NOT INSTALLED YET!)
- [ ] Correct CW/CCW propellers identified
- [ ] No cracks or chips
- [ ] Balanced (use propeller balancer)
- [ ] Propeller nuts/bolts available
- [ ] Propellers NOT installed for testing

---

## 📋 PHASE 2: SOFTWARE VERIFICATION (20 minutes)

### 2.1 Library Installation
- [ ] All required libraries installed (see LIBRARIES_INSTALLATION.md)
- [ ] No compilation errors
- [ ] Test sketches compile successfully

### 2.2 Firmware Upload

#### RC Transmitter Firmware
- [ ] RCTransmitter.ino opens without errors
- [ ] Board set to "Arduino Nano"
- [ ] Processor set to "ATmega328P (Old Bootloader)"
- [ ] Correct COM port selected
- [ ] Upload successful (100%)
- [ ] Serial monitor shows startup message
- [ ] No error messages

**Expected Serial Output:**
```
========================================
  RC TRANSMITTER v1.0
========================================
[OK] NRF24L01 initialized
========================================
  RC TRANSMITTER READY
========================================
```

#### Flight Controller Firmware
- [ ] FlightController.ino opens without errors
- [ ] Board set to "Arduino Nano"
- [ ] Processor set to "ATmega328P (Old Bootloader)"
- [ ] Correct COM port selected
- [ ] Upload successful (100%)
- [ ] Serial monitor shows startup message
- [ ] No error messages

**Expected Serial Output:**
```
========================================
  QUADCOPTER FLIGHT CONTROLLER v1.0
========================================
[INIT] I2C initialized
[OK] MPU6050 initialized
[OK] MS5611 initialized
[OK] Motors initialized
[OK] NRF24L01 initialized
```

---

### 2.3 Sensor Tests

#### MPU6050 IMU Test
- [ ] Sensor detected on I2C bus (address 0x68)
- [ ] Gyro data updating
- [ ] Accelerometer data updating
- [ ] No constant zero readings
- [ ] Tilt drone → values change appropriately

**Test procedure:**
1. Open serial monitor (Flight Controller)
2. Tilt drone forward → accel X changes
3. Tilt drone right → accel Y changes
4. Rotate drone → gyro values change

#### MS5611 Barometer Test
- [ ] Sensor detected on I2C bus (address 0x77)
- [ ] Pressure reading reasonable (90000-110000 Pa)
- [ ] Temperature reading reasonable (15-35°C)
- [ ] Altitude calculation working
- [ ] Values update regularly

**Test procedure:**
1. Note initial altitude reading
2. Lift drone 1 meter → altitude increases ~1m
3. Lower drone → altitude decreases

---

## 📋 PHASE 3: COMMUNICATION TESTS (15 minutes)

### 3.1 NRF24L01 Radio Test

#### Power-On Test
- [ ] Power on RC transmitter first
- [ ] Serial monitor shows NRF initialized
- [ ] Power on flight controller
- [ ] Connection established within 5 seconds
- [ ] Buzzer beeps twice on FC (connection confirmed)
- [ ] Serial monitor shows "NRF CONNECTION ESTABLISHED"

#### Range Test (No Propellers!)
- [ ] Connection stable at 1 meter
- [ ] Connection stable at 5 meters
- [ ] Connection stable at 10 meters
- [ ] No "signal lost" messages
- [ ] LED solid on FC when connected

#### Data Transmission Test
- [ ] Move throttle stick → throttle value changes (1000-2000)
- [ ] Move yaw stick → yaw value changes (1000-2000)
- [ ] Move pitch stick → pitch value changes (1000-2000)
- [ ] Move roll stick → roll value changes (1000-2000)
- [ ] Toggle arm switch → armed status changes (true/false)
- [ ] Press button 1 → command = 1 sent
- [ ] Press button 2 → command = 2 sent

**Verify on serial monitor:**
```
T:1500 Y:1500 P:1500 R:1500 | ARM:NO | CMD:0
```

---

### 3.2 Joystick Calibration Test

#### Auto-Calibration (First Boot)
- [ ] Joystick calibration starts automatically
- [ ] Instructions displayed on serial monitor
- [ ] Center positions recorded
- [ ] Extreme positions recorded (move sticks in full circles)
- [ ] Calibration saved to EEPROM
- [ ] 5 LED blinks confirm completion

#### Calibration Verification
- [ ] All sticks centered → values approximately 1500
- [ ] Throttle full down → 1000
- [ ] Throttle full up → 2000
- [ ] Yaw full left → 1000
- [ ] Yaw full right → 2000
- [ ] Pitch full down → 1000
- [ ] Pitch full up → 2000
- [ ] Roll full left → 1000
- [ ] Roll full right → 2000
- [ ] Deadband working (center = 1500 ±20)

**CRITICAL**: Center position MUST be ~1500, NOT ~1000!

---

## 📋 PHASE 4: SAFETY SYSTEM TESTS (20 minutes)

### 4.1 Kill Switch Test (CRITICAL!)

#### Test 1: Disarm from Armed
1. [ ] Toggle switch to ARM position
2. [ ] Status LED turns solid
3. [ ] Serial shows "DRONE ARMED"
4. [ ] Toggle switch to KILL position
5. [ ] Status LED starts slow blinking
6. [ ] Serial shows "DRONE DISARMED"
7. [ ] All motor values return to 1000

#### Test 2: Cannot Arm During Calibration
1. [ ] Start calibration (Button 1)
2. [ ] Try to toggle ARM switch
3. [ ] Drone should remain disarmed
4. [ ] Error beep sounds
5. [ ] Serial shows error message

#### Test 3: Emergency Stop
1. [ ] Arm drone
2. [ ] Simulate throttle up (move stick)
3. [ ] Quickly toggle KILL switch
4. [ ] Motor values immediately drop to 1000
5. [ ] Response time < 100ms

**Result**: KILL switch must ALWAYS work instantly!

---

### 4.2 Failsafe Test

#### Test 1: Power Off RC Transmitter
1. [ ] Arm drone
2. [ ] Set throttle to 50%
3. [ ] Turn off RC transmitter
4. [ ] Wait 2 seconds
5. [ ] Flight controller enters FAILSAFE mode
6. [ ] Serial shows "FAILSAFE ACTIVATED"
7. [ ] Throttle gradually decreases
8. [ ] Status LED blinks fast
9. [ ] Error beeps sound

#### Test 2: Out of Range Simulation
1. [ ] Arm drone
2. [ ] Walk away with RC >20 meters
3. [ ] Connection lost triggers failsafe
4. [ ] Throttle reduces
5. [ ] Drone prepares for emergency landing

#### Test 3: Failsafe Recovery
1. [ ] Trigger failsafe (power off RC)
2. [ ] Power RC back on within 5 seconds
3. [ ] Connection re-establishes
4. [ ] Normal operation resumes
5. [ ] Serial shows "CONNECTION RESTORED"

---

### 4.3 Calibration System Test

#### ESC Calibration (NO PROPELLERS!)
1. [ ] Toggle switch to KILL
2. [ ] Press Button 1 on RC
3. [ ] Calibration starts on FC
4. [ ] Serial shows "Calibrating ESCs..."
5. [ ] High throttle signal sent (2000)
6. [ ] Low throttle signal sent (1000)
7. [ ] ESC beeps heard (if equipped)
8. [ ] Calibration completes in ~30 seconds

#### IMU Calibration
1. [ ] Place drone on LEVEL surface
2. [ ] Don't move drone!
3. [ ] IMU calibration runs automatically
4. [ ] Progress dots appear (1000 samples)
5. [ ] Offsets calculated
6. [ ] Serial shows gyro and accel offsets
7. [ ] Completion beep

#### Barometer Calibration
1. [ ] Barometer calibration runs automatically
2. [ ] Baseline altitude set
3. [ ] Serial shows base altitude value
4. [ ] Completion beep

#### EEPROM Save/Load
1. [ ] Calibration saves to EEPROM
2. [ ] Serial shows "Calibration saved"
3. [ ] Power cycle drone
4. [ ] Calibration loads from EEPROM
5. [ ] Serial shows "Calibration loaded"
6. [ ] No need to recalibrate

---

## 📋 PHASE 5: MOTOR TESTS (15 minutes)

### ⚠️ WARNING: REMOVE PROPELLERS FOR ALL MOTOR TESTS!

### 5.1 Individual Motor Test

#### Motor 1 (Front-Right) - Pin D3
1. [ ] Connect battery (drone DISARMED)
2. [ ] Arm drone
3. [ ] Manually set motor1Speed = 1200 in code (temporary test)
4. [ ] Motor 1 spins, others stopped
5. [ ] Motor spins in CLOCKWISE direction
6. [ ] Smooth operation, no stuttering

Repeat for Motor 2 (D4), Motor 3 (D5), Motor 4 (D6)

#### Motor 2 (Rear-Right) - Pin D4
- [ ] Spins independently
- [ ] Spins COUNTER-CLOCKWISE

#### Motor 3 (Rear-Left) - Pin D5
- [ ] Spins independently
- [ ] Spins CLOCKWISE

#### Motor 4 (Front-Left) - Pin D6
- [ ] Spins independently
- [ ] Spins COUNTER-CLOCKWISE

---

### 5.2 Motor Test Button (Button 2)

#### Test Sequence
1. [ ] Arm drone
2. [ ] Press Button 2 on RC
3. [ ] LED blinks twice (command sent)
4. [ ] Serial shows "Running motor test"
5. [ ] All 4 motors spin up gradually
6. [ ] Motors run for 3 seconds at low speed
7. [ ] Motors spin down gradually
8. [ ] Serial shows "Motor test complete"
9. [ ] Completion beeps

#### Motor Sync Test
- [ ] All motors start at same time
- [ ] All motors run at same speed
- [ ] All motors stop at same time
- [ ] No vibration or oscillation

---

### 5.3 Throttle Response Test

#### Throttle Stick Test (NO PROPELLERS!)
1. [ ] Arm drone
2. [ ] Slowly increase throttle from 0% to 25%
3. [ ] Motors spin up smoothly
4. [ ] All motors increase equally
5. [ ] Serial shows motor speeds (1000-1250)
6. [ ] Reduce throttle to 0%
7. [ ] Motors spin down smoothly

#### Throttle Mapping Test
- [ ] Throttle 0% → Motor values ~1100 (idle)
- [ ] Throttle 25% → Motor values ~1250
- [ ] Throttle 50% → Motor values ~1500
- [ ] Throttle 75% → Motor values ~1750
- [ ] Throttle 100% → Motor values ~2000

---

### 5.4 PID Response Test (NO PROPELLERS!)

#### Roll Test
1. [ ] Arm drone
2. [ ] Set throttle to 40%
3. [ ] Move roll stick right (full)
4. [ ] Motor 2 & 3 increase (right side down)
5. [ ] Motor 1 & 4 decrease (left side up)
6. [ ] Center stick → motors equalize
7. [ ] No oscillation or hunting

#### Pitch Test
1. [ ] Arm drone
2. [ ] Set throttle to 40%
3. [ ] Move pitch stick forward (full)
4. [ ] Motor 2 & 3 increase (rear down)
5. [ ] Motor 1 & 4 decrease (front up)
6. [ ] Center stick → motors equalize

#### Yaw Test
1. [ ] Arm drone
2. [ ] Set throttle to 40%
3. [ ] Move yaw stick right
4. [ ] Motor 1 & 3 increase (CW motors)
5. [ ] Motor 2 & 4 decrease (CCW motors)
6. [ ] Center stick → motors equalize

**Verify motor mixing:**
```
M1 = Throttle + Pitch - Roll - Yaw
M2 = Throttle - Pitch - Roll + Yaw
M3 = Throttle - Pitch + Roll - Yaw
M4 = Throttle + Pitch + Roll + Yaw
```

---

## 📋 PHASE 6: SYSTEM INTEGRATION (10 minutes)

### 6.1 Complete Startup Sequence

#### Power-On Procedure
1. [ ] Power on RC transmitter
2. [ ] Serial shows "RC TRANSMITTER READY"
3. [ ] Toggle switch to KILL
4. [ ] Power on flight controller
5. [ ] Serial shows startup messages
6. [ ] 2 beeps on power-up
7. [ ] Wait 5 seconds
8. [ ] NRF connection established
9. [ ] 2 more beeps (connection confirmed)
10. [ ] Serial shows "NRF CONNECTION ESTABLISHED"

---

### 6.2 Full Calibration Sequence

1. [ ] Drone on level surface
2. [ ] Press Button 1
3. [ ] ESC calibration runs (~30s)
4. [ ] IMU calibration runs (~15s) - DON'T MOVE!
5. [ ] Barometer calibration runs (~5s)
6. [ ] All values saved to EEPROM
7. [ ] Serial shows "CALIBRATION COMPLETE"
8. [ ] 3 short beeps + 1 long beep

---

### 6.3 Arming Procedure

1. [ ] Calibration complete
2. [ ] Toggle switch to ARM
3. [ ] Status LED turns solid
4. [ ] Serial shows "DRONE ARMED"
5. [ ] 2 quick beeps
6. [ ] Motors at idle (1100)

---

### 6.4 Motor Test Procedure

1. [ ] Drone armed
2. [ ] Press Button 2
3. [ ] Motor test runs (3 seconds)
4. [ ] All motors spin smoothly
5. [ ] Serial shows "Motor test complete"
6. [ ] Serial shows "DRONE READY TO FLY"

---

## 📋 PHASE 7: PRE-FLIGHT FINAL CHECK (5 minutes)

### 7.1 Visual Inspection
- [ ] All wires secured away from propellers
- [ ] No loose screws or components
- [ ] Frame solid and rigid
- [ ] Battery secure and connected
- [ ] Arduino secure and mounted
- [ ] No visible damage

### 7.2 Propeller Installation
- [ ] Motor 1: CW propeller installed
- [ ] Motor 2: CCW propeller installed
- [ ] Motor 3: CW propeller installed
- [ ] Motor 4: CCW propeller installed
- [ ] All propellers tight and secure
- [ ] No cracks or damage
- [ ] Proper orientation (leading edge correct)

### 7.3 Final Systems Check
- [ ] Battery voltage >11.1V
- [ ] All LEDs functioning
- [ ] Buzzer working
- [ ] NRF connection solid
- [ ] Serial monitor shows good data
- [ ] Throttle response correct
- [ ] Kill switch tested and working
- [ ] No error messages

---

## ✅ FINAL APPROVAL

### All Tests Passed?
- [ ] Phase 1: Hardware Verification ✓
- [ ] Phase 2: Software Verification ✓
- [ ] Phase 3: Communication Tests ✓
- [ ] Phase 4: Safety Systems ✓
- [ ] Phase 5: Motor Tests ✓
- [ ] Phase 6: System Integration ✓
- [ ] Phase 7: Pre-Flight Check ✓

### Sign-Off
**Date**: _______________
**Pilot Name**: _______________
**Signature**: _______________

**Weather Conditions**:
- Temperature: ______°C
- Wind: ______km/h
- Visibility: Good / Fair / Poor

**Flight Location**: _______________

---

## 🚁 READY FOR FIRST FLIGHT!

**Remember**:
1. Start with LOW throttle
2. Small stick movements
3. Keep kill switch ready
4. Stay calm
5. Practice hovering first

**Good luck and fly safe! 🎉**

---

**Document Version**: 1.0.0
**Last Updated**: November 2025
