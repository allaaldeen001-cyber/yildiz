# 🚀 UPLOAD INSTRUCTIONS - IMPORTANT!

## ⚠️ ONLY Upload Flight Controller!

The recent fixes are **ONLY** in the Flight Controller. The Remote Controller **does not need any changes**.

---

## Step-by-Step Upload

### 1. Upload Flight Controller ✅

**File**: `FlightController/FlightController.ino`

**Connect**: Flight Controller Arduino Nano (the one on the drone)

**Steps**:
1. Open Arduino IDE
2. File → Open → `FlightController/FlightController.ino`
3. Tools → Board → "Arduino Nano"
4. Tools → Processor → "ATmega328P (Old Bootloader)"
5. Tools → Port → Select your FC port
6. Click "Upload" button
7. Wait for "Done uploading"

---

### 2. Remote Controller - NO UPLOAD NEEDED ❌

**File**: `RemoteController/RemoteController.ino`

**Status**: Already working perfectly with communication!

**Do NOT upload** - it doesn't need any changes!

---

## What Changed?

### Flight Controller Changes:
✅ Button 2 - Motor test now tests each motor individually  
✅ Motor minimum throttle - Prevents motors from stopping  
✅ Smooth takeoff - Uses MS5611 altitude control  
✅ Smooth landing - Uses MS5611 altitude control  

### Remote Controller Changes:
❌ NONE - Remote Controller is already perfect!

---

## If You See "stopMotors" Error

This means you're trying to upload the **Remote Controller** by mistake!

**Error**:
```
RC_Transmitter:602:9: error: 'stopMotors' was not declared in this scope
```

**Solution**:
- Close the RemoteController file
- Open the FlightController file instead
- Upload FlightController only

---

## Quick Check

### Files in Your Project:

```
Your Project Folder/
├── FlightController/
│   └── FlightController.ino  ← UPLOAD THIS! ✅
└── RemoteController/
    └── RemoteController.ino  ← DON'T TOUCH! ❌
```

---

## After Upload

1. **Connect Flight Controller** to computer
2. **Open Serial Monitor** (115200 baud)
3. You should see:
   ```
   ✅ Radio initialized
   ✅ MPU6050 initialized
   ✅ MS5611 initialized
   Listening...
   ```

4. **Power on Remote Controller**
5. Flight Controller should show:
   ```
   ✅ Radio connected
   Throttle: 500 | Roll: 0 | Pitch: 0 | Yaw: 0
   ```

6. **Test Button 2** (no propellers!)
   - Each motor spins for 2 seconds
   - Check rotation directions

7. **Test Button 4** (with propellers, outdoors)
   - Smooth takeoff to 150cm
   - Auto-hover

8. **Test Button 3**
   - Smooth landing from 150cm
   - Auto-disarm

---

## ✅ Ready!

**Upload only FlightController.ino and test!**

The Remote Controller is already working - don't upload it again!
