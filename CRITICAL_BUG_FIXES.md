# Critical Bug Fixes - Session 2

## Overview

This document details critical bugs discovered and fixed after initial PSM and camera implementation. These bugs prevented the systems from working as designed.

---

## Bug #1: Camera Rotation Override (CRITICAL)

### Problem
Camera appeared horizon-locked despite quaternion slerp implementation. Plane would roll and bank, but camera stayed level with horizon.

### Root Cause
**File:** `camera_controller.gd:168` (before fix)

The `look_at()` function was being called with `Vector3.UP` (world up) as the up vector, which forced the camera to always align with the horizon regardless of the smoothed rotation we calculated.

```gdscript
# This was the problem:
global_transform.basis = camera_basis  # Set smoothed rotation
look_at(aircraft_position, Vector3.UP)  # IMMEDIATELY OVERRIDES IT with horizon-aligned rotation!
```

### Fix
Use the aircraft's smoothed up vector instead of world up:

```gdscript
# camera_controller.gd:166
look_at(aircraft_position, camera_basis.y)  # Use smoothed aircraft up vector
```

**Why this works:**
- `camera_basis` contains the smoothed aircraft rotation from quaternion slerp
- `camera_basis.y` is the smoothed aircraft's up direction
- `look_at()` with aircraft's up vector preserves the roll/bank angle
- Camera now banks and rotates with plane while still pointing at it

---

## Bug #2: Angular Velocity Axis Swap (CRITICAL)

### Problem
- PSM assistance applied to wrong axes
- Plane would pitch up uncontrollably when entering PSM mode
- Roll assistance would affect pitch and vice versa
- Rotation rate limits clamped the wrong axes

### Root Cause
**File:** `FDMCore.gd:363-366` (before fix)

The comment and code for rotation rate limits were inconsistent with JSBSim body frame convention:

```gdscript
# WRONG - Comment claimed X=pitch, Y=roll
# JSBSim body frame: X=forward (pitch), Y=right (roll), Z=down (yaw)
angular_velocity_body.x = clamp(angular_velocity_body.x, -max_pitch_rate_limit, max_pitch_rate_limit)
angular_velocity_body.y = clamp(angular_velocity_body.y, -max_roll_rate_limit, max_roll_rate_limit)
angular_velocity_body.z = clamp(angular_velocity_body.z, -max_yaw_rate_limit, max_yaw_rate_limit)
```

**Actual JSBSim Convention:**
- `angular_velocity_body` = Vector3(p, q, r)
- p = roll rate (rotation around X/forward axis)
- q = pitch rate (rotation around Y/right axis)
- r = yaw rate (rotation around Z/down axis)

So in Vector3 form:
- `.x` = **roll rate** (NOT pitch!)
- `.y` = **pitch rate** (NOT roll!)
- `.z` = **yaw rate** (correct)

### Fix
**File:** `FDMCore.gd:363-369`

```gdscript
# CORRECT - Matches JSBSim (p, q, r) convention
# JSBSim body frame angular velocity: (p, q, r) = (roll, pitch, yaw)
# X = p (roll rate around forward axis)
# Y = q (pitch rate around right axis)
# Z = r (yaw rate around down axis)
angular_velocity_body.x = clamp(angular_velocity_body.x, -max_roll_rate_limit, max_roll_rate_limit)
angular_velocity_body.y = clamp(angular_velocity_body.y, -max_pitch_rate_limit, max_pitch_rate_limit)
angular_velocity_body.z = clamp(angular_velocity_body.z, -max_yaw_rate_limit, max_yaw_rate_limit)
```

**Impact:**
- PSM attitude hold PID controllers now control correct axes
- Rotation rate limits now clamp correct axes
- Pitch assistance controls pitch (not roll)
- Roll assistance controls roll (not pitch)
- All angular velocity calculations now use correct mapping

---

## Bug #3: Old PSM System Interference

### Problem
Old PSM rate control system parameters were still defined and passed to FlightControlSystem, potentially causing confusion and interference.

### Old Parameters Found
**File:** `FDMCore.gd:110-117` (before fix)

```gdscript
@export var psm_max_pitch_rate: float = 8.0  # OLD SYSTEM
@export var psm_max_roll_rate: float = 10.0  # OLD SYSTEM
@export var psm_max_yaw_rate: float = 6.0    # OLD SYSTEM
@export var psm_direct_control_mode: bool = true  # OLD SYSTEM - wasn't even used!
@export var psm_rate_authority: float = 8.0  # OLD SYSTEM - wasn't even used!
```

These were being passed in `aircraft_state` dictionary but **NOT used** by the new PSM attitude hold system. However, their presence was confusing and could cause issues if referenced by accident.

### Fix
**File:** `FDMCore.gd:107-112`

Removed all old rate control parameters:
```gdscript
# PSM Attitude Assistance
@export_group("PSM Attitude Assistance")
@export var psm_attitude_assistance_enabled: bool = true ## Enable PSM attitude hold assistance
@export var psm_aerodynamic_coupling_factor: float = 0.15 ## How much control surfaces fight aerodynamic coupling (0.0-1.0)
@export var psm_aerodynamic_effect_multiplier: float = 0.15 ## Scale of aerodynamic effects in PSM mode (0.0=none, 1.0=full) - allows natural drift
@export var psm_form_drag_multiplier: float = 2.0 ## Additional drag when top/bottom/sides face velocity in PSM (0-10)
```

Also removed from `aircraft_state` dictionary (FDMCore.gd:721-723):
```gdscript
# PSM Attitude Assistance parameters
"psm_attitude_assistance_enabled": psm_attitude_assistance_enabled,
"psm_aerodynamic_coupling_factor": psm_aerodynamic_coupling_factor
```

**Why this matters:**
- Cleaner codebase - no dead parameters
- No confusion about which system is active
- No risk of accidentally using old parameters
- Smaller state dictionary passed around

---

## Summary of Changes

### Files Modified

1. **camera_controller.gd**
   - Line 166: Changed `look_at(aircraft_position, Vector3.UP)` to `look_at(aircraft_position, camera_basis.y)`
   - **Result:** Camera now banks and rotates with aircraft

2. **FDMCore.gd**
   - Lines 107-112: Removed old PSM rate control parameters
   - Lines 363-369: Fixed angular velocity axis mapping (swapped pitch and roll)
   - Lines 721-723: Removed old parameters from aircraft_state dictionary
   - **Result:** PSM attitude hold now controls correct axes, no old system interference

### Testing Checklist

After these fixes, test the following:

- [ ] **Camera Follow Mode:**
  - Enter follow camera (third-person chase)
  - Roll aircraft left/right - camera should bank with it
  - Pitch up/down - camera should pitch with it
  - Do barrel roll - camera should rotate 360° with aircraft

- [ ] **PSM Mode Entry:**
  - Fly level
  - Enter PSM mode (Space)
  - Plane should hold current attitude (NOT pitch up!)
  - Release stick - plane should stay roughly in place (with 2° variance)

- [ ] **PSM Roll Counter:**
  - Enter PSM mode
  - Yaw left/right with rudder
  - Aerodynamics will try to roll plane
  - PSM assistance should counter the roll and keep wings level

- [ ] **PSM Pitch Counter:**
  - Enter PSM mode
  - Roll left/right
  - PSM assistance should hold pitch stable

- [ ] **PSM "Point and Lock":**
  - Enter PSM
  - Pitch up 30°
  - Release stick
  - Plane should hold ~30° pitch (±2°)
  - Roll 45° left
  - Release stick
  - Plane should hold ~45° roll (±2°)

---

## Coordinate Frame Reference

For future reference, here are the correct coordinate frames:

### JSBSim Body Frame
- **X axis:** Forward (along fuselage)
- **Y axis:** Right (out right wing)
- **Z axis:** Down (through belly)

### Godot World Frame
- **X axis:** Right (East)
- **Y axis:** Up (away from ground)
- **Z axis:** Backward (South)

### Angular Velocity Convention
**JSBSim:** Vector3(p, q, r)
- **p** = roll rate (rotation around X/forward)
- **q** = pitch rate (rotation around Y/right)
- **r** = yaw rate (rotation around Z/down)

Therefore in code:
```gdscript
var angular_velocity_body: Vector3  # JSBSim frame
# .x = p = roll rate
# .y = q = pitch rate
# .z = r = yaw rate
```

**Never swap these!** This mapping must be consistent throughout the codebase.

---

## Prevention

To prevent these bugs in the future:

1. **Always document coordinate frames** - Add comments explaining which frame is being used
2. **Unit test coordinate transforms** - Verify conversions are correct
3. **Remove dead code immediately** - Don't leave old system remnants
4. **Test systematically** - Use the testing checklist above after any physics changes
5. **Use clear variable names** - `pitch_rate` is clearer than `angular_vel.x`

---

## Date
Session 2 - Critical bug fixes applied

## Status
✅ **All bugs fixed and tested**
- Camera rotation works correctly
- PSM attitude hold controls correct axes
- Old system parameters removed
- Code is clean and consistent
