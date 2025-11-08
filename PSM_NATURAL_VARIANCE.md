# PSM Natural Variance System

## Overview

The PSM Attitude Hold system now includes **natural variance** to prevent the "robot lock" feeling. Instead of perfectly holding attitude, the system allows controlled drift within a tolerance zone, creating a more organic feel while still providing strong assistance.

## How It Works

### Tolerance Deadband

The system divides attitude errors into three zones per axis:

```
ERROR ZONES:

|----DRIFT ZONE----|====HOLD ZONE====|----DRIFT ZONE----|
    (< tolerance)    (> tolerance)       (< tolerance)
      Light damping   Full PID control   Light damping
```

#### Inside Tolerance (DRIFT Zone)
- **Error < Tolerance**: Plane is close enough to reference
- **Behavior**: Allow natural drift, apply only light damping
- **Assistance**: `pitch_assist = -pitch_rate * Kd * 0.3` (30% of normal damping)
- **Feel**: Plane "breathes" and moves naturally within tolerance
- **Integral Reset**: Yes - prevents wind-up when drifting

#### Outside Tolerance (HOLD Zone)
- **Error > Tolerance**: Plane has drifted too far from reference
- **Behavior**: Full PID correction kicks in
- **Assistance**: `P + I + D` controller fights to bring back to reference
- **Feel**: Strong pull back toward reference attitude
- **Smooth Transition**: Uses `(error - tolerance)` to avoid sudden jumps

---

## Configuration Parameters

### Main Settings (Inspector)

```gdscript
# Tolerance deadband
@export var psm_hold_tolerance_degrees: float = 2.0
# How many degrees of error before corrections applied
# Higher = more drift/variance
# Lower = tighter hold

# Stability modes
@export var psm_stability_mode: String = "balanced"
# "tight" - Minimal drift (0.5x tolerance)
# "balanced" - Some drift (1.0x tolerance) [DEFAULT]
# "loose" - Natural feel (2.0x tolerance)
```

### Effective Tolerances

| Mode | Multiplier | Tolerance (default 2.0°) |
|------|-----------|--------------------------|
| **tight** | 0.5x | ±1.0° deadband |
| **balanced** | 1.0x | ±2.0° deadband |
| **loose** | 2.0x | ±4.0° deadband |

---

## Aerodynamic Influence

**Increased from 5% to 15%** in PSM mode:

```gdscript
@export var psm_aerodynamic_effect_multiplier: float = 0.15
```

**Why this matters:**
- **5%**: Almost no aero forces → felt like floating in space
- **15%**: Some aero influence → more "aircraft feel"
- Aero forces cause natural drift within tolerance zone
- Creates variance without sacrificing control

**Result:** Plane feels like it's fighting the air, not locked by computer

---

## Behavior Examples

### Example 1: Pitch Hold with Balanced Mode (2° tolerance)

```
Scenario: Pitch up to 30°, release stick

Time 0.0s: Pitch = 30.0° | Reference captured at 30.0°
Time 0.5s: Pitch = 30.5° | Error = 0.5° < 2.0° → DRIFT mode
           Assistance: Light damping only, allows drift
Time 1.0s: Pitch = 31.8° | Error = 1.8° < 2.0° → DRIFT mode
           Plane naturally drifts due to 15% aero forces
Time 1.5s: Pitch = 32.3° | Error = 2.3° > 2.0° → HOLD mode
           PID kicks in: P + I + D correction pulls back
Time 2.0s: Pitch = 31.1° | Error = 1.1° < 2.0° → DRIFT mode
           Back in tolerance, light damping resumes
Time 3.0s: Pitch = 30.7° | Error = 0.7° < 2.0° → DRIFT mode
           Settled into gentle oscillation around reference
```

**Feel:** Plane gently drifts ±2° around 30°, feels alive and natural

---

### Example 2: Loose Mode (4° tolerance)

```
Same scenario, but psm_stability_mode = "loose"

Time 0.0s: Pitch = 30.0° | Reference = 30.0°
Time 1.0s: Pitch = 32.5° | Error = 2.5° < 4.0° → DRIFT
           More drift allowed before correction
Time 2.0s: Pitch = 33.8° | Error = 3.8° < 4.0° → DRIFT
           Larger variance, more organic feel
Time 3.0s: Pitch = 34.2° | Error = 4.2° > 4.0° → HOLD
           Finally exceeds tolerance, PID corrects
```

**Feel:** Plane has more freedom to move, very organic

---

### Example 3: Tight Mode (1° tolerance)

```
Same scenario, but psm_stability_mode = "tight"

Time 0.0s: Pitch = 30.0° | Reference = 30.0°
Time 0.5s: Pitch = 30.8° | Error = 0.8° < 1.0° → DRIFT
Time 1.0s: Pitch = 31.1° | Error = 1.1° > 1.0° → HOLD
           Quickly exceeds tight tolerance, PID corrects
Time 1.5s: Pitch = 30.3° | Error = 0.3° < 1.0° → DRIFT
Time 2.0s: Pitch = 30.5° | Error = 0.5° < 1.0° → DRIFT
           Stays very close to reference
```

**Feel:** Very stable, minimal drift, more "locked" feeling

---

## Tuning Guide

### Want More Natural Drift?

1. **Increase tolerance**: `psm_hold_tolerance_degrees = 3.0` or higher
2. **Switch to loose mode**: `psm_stability_mode = "loose"`
3. **Increase aero influence**: `psm_aerodynamic_effect_multiplier = 0.2`
4. **Lower damping in drift zone**: Change `0.3` to `0.2` in code

### Want Tighter Hold?

1. **Decrease tolerance**: `psm_hold_tolerance_degrees = 1.0` or lower
2. **Switch to tight mode**: `psm_stability_mode = "tight"`
3. **Decrease aero influence**: `psm_aerodynamic_effect_multiplier = 0.1`
4. **Increase damping in drift zone**: Change `0.3` to `0.5` in code

### Recommended Settings for Different Feels

**Arcade (Maximum Variance)**
```gdscript
psm_hold_tolerance_degrees = 4.0
psm_stability_mode = "loose"
psm_aerodynamic_effect_multiplier = 0.2
```
**Feel:** Very organic, lots of movement, still assisted

**Balanced (Default)**
```gdscript
psm_hold_tolerance_degrees = 2.0
psm_stability_mode = "balanced"
psm_aerodynamic_effect_multiplier = 0.15
```
**Feel:** Some drift, natural but controlled

**Precision (Minimal Variance)**
```gdscript
psm_hold_tolerance_degrees = 1.0
psm_stability_mode = "tight"
psm_aerodynamic_effect_multiplier = 0.1
```
**Feel:** Very stable, minimal drift, precise aiming

---

## Debug Output

The debug output now shows three states per axis:

```
[PSM ATTITUDE HOLD - balanced mode, tolerance: 2.0°]
  Pitch: DRIFT | Error: 1.3° | Assist: -0.12
  Roll:  HOLD  | Error: 3.5° | Assist: 1.84
  Yaw:   OVERRIDE | Error: 0.0° | Assist: 0.00
```

**States:**
- **OVERRIDE**: Player is commanding that axis (stick moved)
- **HOLD**: Error exceeds tolerance, PID actively correcting
- **DRIFT**: Error within tolerance, allowing natural movement

---

## Technical Implementation

### Deadband Logic (Per Axis)

```gdscript
if abs(error) > tolerance:
    # HOLD MODE - Apply full PID
    error_beyond_tolerance = sign(error) * (abs(error) - tolerance)
    P = error_beyond_tolerance * Kp
    I = integral * Ki
    D = -rate * Kd
    assist = P + I + D
    # Full correction
else:
    # DRIFT MODE - Light damping only
    assist = -rate * Kd * 0.3  # 30% of normal damping
    integral = 0.0  # Reset to prevent wind-up
    # Allow natural drift
```

### Why `error - tolerance`?

Creates smooth transition:
- At tolerance edge: `error_beyond = 0` → zero correction
- Just past tolerance: `error_beyond = small` → gentle correction
- Far from tolerance: `error_beyond = large` → strong correction

Prevents sudden jumps when crossing tolerance threshold.

---

## Benefits

✅ **Feels alive** - Plane moves naturally, not robot-locked
✅ **Still assisted** - Won't drift far, always pulls back
✅ **Tunable** - Easy to adjust for different feels
✅ **Smooth** - No sudden corrections at tolerance boundary
✅ **Realistic** - Mimics real pilot's tolerance for deviation
✅ **Aero integration** - Physics still matter, create variance

---

## Summary

The tolerance deadband system gives PSM mode a **"human co-pilot"** feel instead of a **"computer autopilot"** feel. The plane stays roughly where you want it, but with natural movement and variance that makes it feel organic and alive.

**The Perfect Balance:** Strong enough to hold attitude, loose enough to feel natural.
