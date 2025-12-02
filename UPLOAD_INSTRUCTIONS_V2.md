# 🚀 UPLOAD INSTRUCTIONS - READ THIS!

## ⚠️ IMPORTANT: You Only Need to Upload ONE File!

Your **RemoteController is already working** (you confirmed communication works).

The new landing system only changes the **FlightController**.

---

## What to Upload

### ✅ Upload This:
**File**: `FlightController/FlightController.ino`  
**Arduino**: Flight Controller (the one on the drone)

### ❌ DON'T Upload This:
**File**: `RemoteController/RemoteController.ino`  
**Reason**: Already working, no changes needed!

---

## Step-by-Step

### 1. Open FlightController
```
File → Open → FlightController/FlightController.ino
```

### 2. Select Board & Port
```
Tools → Board → Arduino Nano
Tools → Processor → ATmega328P (Old Bootloader)
Tools → Port → [Your Flight Controller Port]
```

### 3. Upload
```
Click "Upload" button (→)
Wait for "Done uploading"
```

### 4. Test
```
Open Serial Monitor (115200 baud)
Should see:
  ✅ MPU6050 initialized
  ✅ MS5611 initialized
  ✅ Radio initialized
  📡 Waiting for radio connection...
```

### 5. Power On Remote
```
Should see:
  ✅ Radio connected
  Throttle: 500 | Roll: 0 | Pitch: 0 | Yaw: 0
```

---

## Why You Got the Error

You tried to compile `RC_Transmitter.ino` which has this error:

```cpp
// Line 131: Flight Controller code (shouldn't be in RC!)
#define LANDING_SAFE_IDLE  1050  // ❌ Wrong file!

// Line 219: Enum using it
enum LandingState {
  ...
  LANDING_SAFE_IDLE,  // ❌ This causes the error!
};
```

**Solution**: Don't compile RC_Transmitter! It's already working.

---

## If You Really Need to Fix RC_Transmitter

If your RC file is corrupted, use the clean version from `/workspace/RemoteController/RemoteController.ino`.

It has:
- ✅ No LANDING defines
- ✅ No LandingState enum
- ✅ Just joystick, button, and radio code
- ✅ Works perfectly with the new FlightController

---

## Summary

| File | Action | Why |
|------|--------|-----|
| FlightController.ino | ✅ Upload | New landing system |
| RemoteController.ino | ❌ Don't touch | Already working |

---

## Test After Upload

1. Remove propellers
2. Press Button 2 (Motor Test)
3. Each motor spins for 2s
4. Install propellers
5. Press Button 4 (Takeoff)
6. Press Button 3 (Landing)
7. Watch smooth descent! 🎉

---

**Just upload FlightController.ino and you're done!** ✅
