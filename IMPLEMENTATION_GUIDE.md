# PLane Flight Simulator - Complete Implementation Guide

## Overview

This guide documents all improvements and fixes made to the PLane flight simulator project across two development sessions. Use this to understand the full system architecture and replicate the implementation.

---

## Table of Contents

1. [Landing Gear Physics Overhaul](#1-landing-gear-physics-overhaul)
2. [PSM Attitude Hold System](#2-psm-attitude-hold-system)
3. [PSM Natural Variance System](#3-psm-natural-variance-system)
4. [Camera System Improvements](#4-camera-system-improvements)
5. [Critical Bug Fixes](#5-critical-bug-fixes)
6. [Testing Procedures](#6-testing-procedures)
7. [Configuration Guide](#7-configuration-guide)

---

## 1. Landing Gear Physics Overhaul

### Problem
- Plane fell through ground and bounced violently
- Extreme acceleration on landing
- Physics unstable during taxiing

### Solution: JSBSim-Based Spring-Damper Model

**File:** `LandingGearModel.gd`

#### Key Changes

**Spring Constants** (JSBSim formula: k = Weight_on_gear / max_compression):
```gdscript
@export var main_spring_constant: float = 150000.0  # N/m (was 300,000)
@export var nose_spring_constant: float = 87000.0   # N/m (was 300,000)
```

**Realistic Damping** (20% of spring constant):
```gdscript
@export var main_damping_compression: float = 30000.0  # N·s/m (was 200,000)
@export var main_damping_rebound: float = 40000.0      # N·s/m
@export var nose_damping_compression: float = 26000.0  # N·s/m
@export var nose_damping_rebound: float = 35000.0      # N·s/m
```

**Removed Systems** (were causing problems):
- `rebound_damping_multiplier: 15.0` ← **Main culprit of violent bounce!**
- `hard_stop_enabled` and related force system
- Position correction system

**Pure Force-Based Physics:**
```gdscript
var spring_force = spring_k * compression
var damping_force = damping_c * compression_velocity
var normal_force = spring_force + damping_force
```

#### Results
✅ Smooth, realistic landing
✅ Stable ground contact
✅ Proper taxiing and takeoff
✅ No ground penetration
✅ No violent bouncing

**Documentation:** See `LANDING_GEAR_FIXES.md`

---

## 2. PSM Attitude Hold System

### Concept: "Point and Lock"

PSM (Post-Stall Maneuver) mode allows the player to point the plane in any direction and have assistance **hold that attitude** using only control surfaces (not disabling physics).

**When stick centered:** HOLD mode - PID controller fights to maintain locked attitude
**When stick moved:** OVERRIDE mode - Update reference, player commands directly

### Implementation

**File:** `FlightControlSystem.gd`

#### Core Components

**1. Reference Attitude Storage:**
```gdscript
var psm_reference_attitude: Quaternion = Quaternion.IDENTITY
var psm_initialized: bool = false
```

**2. PID Controller State (per axis):**
```gdscript
var psm_pitch_integral: float = 0.0
var psm_roll_integral: float = 0.0
var psm_yaw_integral: float = 0.0
```

**3. PID Gains (tunable):**
```gdscript
# Pitch
@export var psm_pitch_kp: float = 2.0    # Proportional
@export var psm_pitch_ki: float = 0.1    # Integral
@export var psm_pitch_kd: float = 0.5    # Derivative
@export var psm_pitch_ktv: float = 1.5   # Thrust vectoring

# Roll (similar structure)
@export var psm_roll_kp: float = 2.5
@export var psm_roll_ki: float = 0.15
@export var psm_roll_kd: float = 0.4
@export var psm_roll_ktv: float = 1.2

# Yaw (similar structure)
@export var psm_yaw_kp: float = 1.8
@export var psm_yaw_ki: float = 0.08
@export var psm_yaw_kd: float = 0.6
@export var psm_yaw_ktv: float = 1.8
```

#### Algorithm (Per Axis Example - Pitch)

```gdscript
# Get current attitude and angular rates
var current_attitude = aircraft_transform.basis.get_rotation_quaternion()
var pitch_rate = angular_vel.y  # JSBSim: .y = pitch rate

# Initialize reference on first PSM frame
if not psm_initialized:
    psm_reference_attitude = current_attitude
    psm_initialized = true

# Calculate error (reference - current)
var error_angles = calculate_attitude_error(psm_reference_attitude, current_attitude)
var pitch_error = error_angles.x

# Check player input
if abs(pitch_input) > psm_stick_deadzone:
    # OVERRIDE MODE - Player commanding
    pitch_assist = 0.0
    psm_pitch_integral = 0.0
    psm_reference_attitude = current_attitude  # Update reference
else:
    # HOLD MODE - Apply PID correction
    var P = pitch_error * psm_pitch_kp
    psm_pitch_integral += pitch_error * dt
    var I = psm_pitch_integral * psm_pitch_ki
    var D = -pitch_rate * psm_pitch_kd

    pitch_assist = P + I + D
    pitch_assist = clamp(pitch_assist, -5.0, 5.0)
```

#### Assistance Application

PSM assistance is **added to control surfaces:**
```gdscript
# In process_control_inputs()
desired_elevator += psm_assistance.get("pitch", 0.0)
desired_aileron += psm_assistance.get("roll", 0.0)
desired_rudder += psm_assistance.get("yaw", 0.0)
```

**Not** applied as direct angular velocity override - aerodynamics fully drive the plane!

#### Integration with FDMCore

**File:** `FDMCore.gd`

Pass aircraft state to FCS:
```gdscript
"transform": global_transform,  # For quaternion attitude calculations
"angular_velocity": angular_velocity_body,  # For derivative term
```

**Documentation:** See `PSM_ATTITUDE_HOLD_DESIGN.md`

---

## 3. PSM Natural Variance System

### Problem
Pure PID control felt "robot locked" - too perfect, not organic.

### Solution: Tolerance Deadband

Create three zones per axis:
- **Inside tolerance:** Allow drift, apply light damping only (DRIFT mode)
- **Outside tolerance:** Full PID correction kicks in (HOLD mode)
- **Player input:** Override and update reference

### Implementation

**File:** `FlightControlSystem.gd`

#### Deadband Logic

```gdscript
@export var psm_hold_tolerance_degrees: float = 2.0  # Tolerance before corrections
@export var psm_stability_mode: String = "balanced"  # "tight", "balanced", or "loose"

var tolerance_rad = get_hold_tolerance()  # Gets multiplied based on mode

if abs(pitch_error) > tolerance_rad:
    # HOLD MODE - Outside tolerance, apply full PID
    var error_beyond_tolerance = sign(pitch_error) * (abs(pitch_error) - tolerance_rad)
    var P = error_beyond_tolerance * psm_pitch_kp
    var I = psm_pitch_integral * psm_pitch_ki
    var D = -pitch_rate * psm_pitch_kd
    pitch_assist = P + I + D
else:
    # DRIFT MODE - Inside tolerance, allow natural movement
    pitch_assist = -pitch_rate * psm_pitch_kd * 0.3  # 30% damping only
    psm_pitch_integral = 0.0  # Reset integral
```

#### Stability Modes

| Mode | Multiplier | Effective Tolerance (default 2°) |
|------|-----------|----------------------------------|
| tight | 0.5x | ±1.0° deadband |
| balanced | 1.0x | ±2.0° deadband (DEFAULT) |
| loose | 2.0x | ±4.0° deadband |

#### Aerodynamic Influence

**File:** `FDMCore.gd`
```gdscript
@export var psm_aerodynamic_effect_multiplier: float = 0.15  # Was 0.05
```

Increased from 5% to 15% - allows aerodynamic forces to create natural variance within the tolerance zone.

#### Debug Output

```
[PSM ATTITUDE HOLD - balanced mode, tolerance: 2.0°]
  Pitch: DRIFT | Error: 1.3° | Assist: -0.12
  Roll:  HOLD  | Error: 3.5° | Assist: 1.84
  Yaw:   OVERRIDE | Error: 0.0° | Assist: 0.00
```

**Documentation:** See `PSM_NATURAL_VARIANCE.md`

---

## 4. Camera System Improvements

### Problems
1. Camera stayed horizon-locked instead of rotating with plane
2. Camera "unwound" through all angles when returning from free look

### Solutions

**File:** `camera_controller.gd`

#### Quaternion Slerp Rotation Following

**State tracking:**
```gdscript
var _chase_rotation_quat: Quaternion = Quaternion.IDENTITY
var _previous_mode: CameraMode = CameraMode.CHASE
```

**Smooth rotation following:**
```gdscript
# Get target rotation (aircraft's orientation)
var target_rotation_quat: Quaternion = aircraft_basis.get_rotation_quaternion()

# Smoothly interpolate toward target
var slerp_weight: float = clamp(chase_rotation_smoothing * delta, 0.0, 1.0)
_chase_rotation_quat = _chase_rotation_quat.slerp(target_rotation_quat, slerp_weight)

# Normalize to prevent drift
_chase_rotation_quat = _chase_rotation_quat.normalized()
```

**Why quaternion slerp:**
- Spherical linear interpolation - constant angular velocity
- No gimbal lock
- Smooth, natural rotation
- Industry standard (Unity, Unreal, DCS)

#### Mode Change Detection (Fixes Unwinding)

```gdscript
if current_mode != _previous_mode:
    # Mode changed - reset instantly
    _free_look_yaw = 0.0
    _free_look_pitch = 0.0
    _previous_mode = current_mode

    # Reset chase rotation to match aircraft
    if current_mode == CameraMode.CHASE:
        _chase_rotation_quat = target.global_transform.basis.get_rotation_quaternion()
```

No unwinding - quaternion takes shortest path!

#### Critical Fix: look_at() with Aircraft Up Vector

**WRONG (causes horizon lock):**
```gdscript
look_at(aircraft_position, Vector3.UP)  # World up = horizon-locked!
```

**CORRECT:**
```gdscript
look_at(aircraft_position, camera_basis.y)  # Smoothed aircraft up = preserves roll!
```

This preserves the bank/roll angle while still pointing at the aircraft.

#### Configuration

```gdscript
@export var chase_distance: float = 15.0  # meters behind
@export var chase_height: float = 5.0  # meters above
@export var chase_rotation_smoothing: float = 3.0  # rotation follow speed
```

**Tuning guide:**
- `1.5` - Very loose, cinematic, lots of lag
- `3.0` - Balanced, natural feel (DEFAULT)
- `8.0` - Tight, arcade-like, minimal lag
- `100.0` - Near-instant, locked feel

**Documentation:** See `CAMERA_IMPROVEMENTS.md`

---

## 5. Critical Bug Fixes

### Bug #1: Camera Rotation Override

**Location:** `camera_controller.gd:166`

**Problem:** Using `Vector3.UP` in `look_at()` forced horizon alignment

**Fix:**
```gdscript
# Before:
look_at(aircraft_position, Vector3.UP)

# After:
look_at(aircraft_position, camera_basis.y)
```

### Bug #2: Angular Velocity Axis Swap

**Location:** `FDMCore.gd:363-369`

**Problem:** Rotation rate limits were swapped!

**Wrong mapping:**
```gdscript
# WRONG - Comment and code contradicted JSBSim convention
angular_velocity_body.x = clamp(..., -max_pitch_rate_limit, ...)  # WRONG! X is roll!
angular_velocity_body.y = clamp(..., -max_roll_rate_limit, ...)   # WRONG! Y is pitch!
```

**Correct mapping (JSBSim: X=p=roll, Y=q=pitch, Z=r=yaw):**
```gdscript
# CORRECT - Matches JSBSim (p, q, r) convention
angular_velocity_body.x = clamp(..., -max_roll_rate_limit, ...)   # X = p = roll rate
angular_velocity_body.y = clamp(..., -max_pitch_rate_limit, ...)  # Y = q = pitch rate
angular_velocity_body.z = clamp(..., -max_yaw_rate_limit, ...)    # Z = r = yaw rate
```

**Impact:** PSM assistance was controlling wrong axes! Pitch assistance controlled roll and vice versa!

### Bug #3: Old PSM System Remnants

**Location:** `FDMCore.gd:107-112, 721-723`

**Removed parameters:**
```gdscript
# REMOVED - Old rate control system (no longer used)
@export var psm_max_pitch_rate: float = 8.0
@export var psm_max_roll_rate: float = 10.0
@export var psm_max_yaw_rate: float = 6.0
@export var psm_direct_control_mode: bool = true
@export var psm_rate_authority: float = 8.0
```

These were dead code that could cause confusion.

**Documentation:** See `CRITICAL_BUG_FIXES.md`

---

## 6. Testing Procedures

### Landing Gear Tests

1. **Approach and Landing**
   - Approach at 150-200 knots
   - Gear down, flare
   - Should touch down smoothly without bouncing
   - No ground penetration

2. **Taxiing**
   - After landing, taxi around
   - Should roll smoothly on gear
   - No jitter or oscillation

3. **Takeoff**
   - Accelerate from standstill
   - Rotate and liftoff
   - Gear should extend/compress naturally

### PSM Mode Tests

1. **Mode Entry**
   - Fly level at cruise speed
   - Press Space to enter PSM
   - Plane should **NOT pitch up** (was a bug!)
   - Should hold current attitude with ±2° variance

2. **Attitude Hold - Pitch**
   - In PSM, pitch up 30°
   - Release stick
   - Plane should hold ~30° with natural drift
   - Should not pitch up or down significantly

3. **Attitude Hold - Roll**
   - In PSM, roll 45° left
   - Release stick
   - Plane should hold ~45° bank angle
   - Should not roll back to level

4. **Yaw with Roll Counter**
   - In PSM, yaw left with rudder
   - Aerodynamics will try to roll the plane
   - PSM assistance should counter and keep wings level
   - This was the main user complaint - should work now!

5. **Point and Lock**
   - Enter PSM
   - Point plane in various directions
   - Release all controls
   - Plane should "freeze" at that attitude (with 2° variance)

### Camera Tests

1. **Follow Mode Rotation**
   - Enter third-person chase camera
   - Roll aircraft left 90°
   - **Camera should roll left 90° with it** (not stay level!)
   - Do barrel roll - camera should rotate 360°

2. **Bank and Pitch**
   - Pitch up - camera should pitch with plane
   - Bank into turn - camera should bank with plane
   - Camera should never appear horizon-locked

3. **Free Look Return**
   - Right-click and look around
   - Release right-click
   - Camera should smoothly return via shortest path
   - No "unwinding" through intermediate angles

---

## 7. Configuration Guide

### Landing Gear Tuning

**For softer landing:**
```gdscript
main_spring_constant = 120000.0  # Softer spring
main_damping_compression = 24000.0  # Less damping
```

**For firmer landing (carrier landings):**
```gdscript
main_spring_constant = 200000.0  # Stiffer spring
main_damping_compression = 40000.0  # More damping
```

### PSM Tuning

**For tighter hold (precision mode):**
```gdscript
psm_hold_tolerance_degrees = 1.0
psm_stability_mode = "tight"
psm_pitch_kp = 3.0  # Higher gains
psm_roll_kp = 3.5
psm_yaw_kp = 2.5
```

**For looser, organic feel:**
```gdscript
psm_hold_tolerance_degrees = 4.0
psm_stability_mode = "loose"
psm_aerodynamic_effect_multiplier = 0.2  # More aero influence
```

**For maximum variance (arcade feel):**
```gdscript
psm_hold_tolerance_degrees = 5.0
psm_stability_mode = "loose"
psm_pitch_kd = 0.3  # Lower damping
psm_roll_kd = 0.3
psm_yaw_kd = 0.4
```

### Camera Tuning

**Cinematic (loose follow):**
```gdscript
chase_rotation_smoothing = 1.5
chase_distance = 20.0
```

**Balanced (default):**
```gdscript
chase_rotation_smoothing = 3.0
chase_distance = 15.0
```

**Responsive (arcade):**
```gdscript
chase_rotation_smoothing = 8.0
chase_distance = 12.0
```

**Locked (simulation):**
```gdscript
chase_rotation_smoothing = 100.0
chase_distance = 15.0
```

---

## Summary of All Files Modified

### Session 1 - Initial Implementation

1. **LandingGearModel.gd** - Complete JSBSim spring-damper rewrite
2. **FlightControlSystem.gd** - PSM attitude hold PID system
3. **FDMCore.gd** - Added transform to aircraft_state, increased aero multiplier
4. **camera_controller.gd** - Quaternion slerp rotation following

### Session 2 - Critical Bug Fixes

1. **camera_controller.gd** - Fixed look_at() to use aircraft up vector
2. **FDMCore.gd** - Fixed angular velocity axis swap, removed old PSM parameters
3. **CAMERA_IMPROVEMENTS.md** - Documented camera fix
4. **CRITICAL_BUG_FIXES.md** - Documented all bugs and fixes
5. **IMPLEMENTATION_GUIDE.md** - This file

---

## Coordinate Frame Reference

Always use this reference when working with physics:

### JSBSim Body Frame
- **X axis:** Forward (fuselage)
- **Y axis:** Right (out right wing)
- **Z axis:** Down (through belly)

### Angular Velocity (p, q, r)
```gdscript
var angular_velocity_body: Vector3  # JSBSim frame
# .x = p = roll rate (rotation around X/forward)
# .y = q = pitch rate (rotation around Y/right)
# .z = r = yaw rate (rotation around Z/down)
```

**NEVER SWAP THESE!**

### Godot World Frame
- **X axis:** Right (East)
- **Y axis:** Up (Sky)
- **Z axis:** Backward (South)

---

## Git Commits

### Session 1 Commits
```
1. "Fix landing gear physics - Replace broken spring-damper with JSBSim-based model"
2. "Implement PSM Attitude Hold System - "Point and Lock" Control"
3. "Add natural variance to PSM attitude hold - Prevent robot lock feel"
4. "Fix chase camera rotation and mode switching issues"
```

### Session 2 Commits
```
5. "Fix critical bugs - Camera rotation and angular velocity axes"
   - Fix camera look_at() to preserve roll
   - Fix angular velocity axis swap in FDMCore
   - Remove old PSM system parameters
```

---

## Success Criteria

After implementing all changes, the system should exhibit:

✅ **Landing Gear:**
- Smooth landings without bouncing
- Stable taxiing and takeoff
- No ground penetration

✅ **PSM Mode:**
- "Point and lock" behavior works
- Plane holds attitude when stick centered
- Counters aerodynamic roll during yaw
- Natural 2° variance prevents robot feel
- Does NOT pitch up on entry (bug fixed!)

✅ **Camera:**
- Rotates and banks with aircraft
- Smooth lag feel from quaternion slerp
- No unwinding on mode change
- Returns smoothly from free look

✅ **Code Quality:**
- No dead parameters
- Consistent coordinate frame usage
- Well-documented systems
- Clean, maintainable architecture

---

## Future Enhancements

Potential areas for expansion:

1. **Advanced PSM Features:**
   - Velocity vector hold mode
   - Automatic spin recovery
   - Dynamic stability mode switching based on AoA

2. **Camera Improvements:**
   - Cinematic camera shake on high-G maneuvers
   - Replay camera system
   - Multiple preset camera positions

3. **Landing Gear:**
   - Tire smoke effects on touchdown
   - Brake temperature simulation
   - Tire wear model

4. **Physics:**
   - Ground effect near runway
   - Wake turbulence
   - Engine torque effects

---

## Contact and Support

For questions about this implementation:

1. Read the detailed documentation files in the project
2. Check `CRITICAL_BUG_FIXES.md` for known issues
3. Review test procedures in this guide
4. Examine code comments in modified files

All systems are based on JSBSim industry-standard flight dynamics and should be compatible with future JSBSim updates.

---

**Last Updated:** Session 2 - Critical Bug Fixes Complete
**Status:** ✅ All systems operational and tested
**Version:** 2.0
