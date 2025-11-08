# Landing Gear Physics Fixes - JSBSim-Based Implementation

## Problem Description
The original landing gear had severe physics issues:
- **Plane falling through ground** then springing back violently
- **Crazy acceleration** on landing
- **Unrealistic bouncing** behavior

## Root Causes Identified

1. **Spring constants too high** (300,000 N/m)
   - Caused violent "spring back" effect
   - Should be calculated using: k = Weight_on_gear / max_compression

2. **Damping too high** (200,000 N·s/m)
   - Prevented proper compression
   - JSBSim uses 20-30% of spring rate

3. **Rebound damping multiplier (15x)**
   - **This was the main issue** causing violent bounce-back
   - JSBSim uses similar or slightly higher damping for rebound, not 15x

4. **Hard stop system**
   - Added extra violent forces on top of spring forces
   - Not present in JSBSim model

5. **Position correction**
   - Caused jumping and penetration issues
   - JSBSim uses only forces, no position correction

## Changes Implemented

### 1. Realistic Spring Constants (JSBSim Formula)
```gdscript
# Formula: k = Weight_on_gear / max_compression
# For F-16 (14,000 kg total):
#   Mains: 78% of weight = ~5,400 kg each
#   Nose: 22% of weight = ~3,100 kg

main_spring_constant: 150,000 N/m  # Was 300,000
nose_spring_constant: 87,000 N/m   # Was 300,000
```

### 2. Realistic Damping Constants (JSBSim Standard)
```gdscript
# JSBSim: 20% of spring for mains, 30% for nose

main_damping_compression: 30,000 N·s/m  # Was 200,000
main_damping_rebound: 40,000 N·s/m      # Was 200,000 * 15 = 3,000,000!
nose_damping_compression: 26,000 N·s/m  # Was 200,000
nose_damping_rebound: 35,000 N·s/m      # Was 200,000 * 15 = 3,000,000!
```

### 3. Removed Problematic Systems
- ❌ **Removed** 15x rebound damping multiplier
- ❌ **Removed** hard stop system (hard_stop_enabled, hard_stop_stiffness)
- ❌ **Removed** position correction (was causing jumping)
- ❌ **Removed** extra touchdown damping
- ❌ **Removed** emergency forces

### 4. Pure JSBSim Spring-Damper Model
```gdscript
# Simple and stable formula:
spring_force = spring_k * compression
damping_force = damping_c * compression_velocity
normal_force = spring_force + damping_force
```

### 5. Improved Friction Model (JSBSim-Based)
- **Rolling resistance**: 0.02 (realistic for aircraft tires)
- **Static friction**: 0.8 (for braking)
- **Brake multiplier**: 3.5 (realistic max braking coefficient ~2.8)
- **Speed-dependent friction** to prevent jitter at low speeds
- **No brakes on nose gear** (realistic for F-16)

### 6. Reduced max_compression
- Changed from 0.5m to 0.35m (more realistic strut travel)

## Expected Behavior Now

### Landing
- ✅ Smooth touchdown with realistic compression
- ✅ No violent bouncing or "spring back"
- ✅ No falling through ground
- ✅ No crazy acceleration
- ✅ Gradual settling as dampers absorb energy

### Taxiing
- ✅ Smooth rolling with low friction (rolling_friction_coeff = 0.02)
- ✅ Effective braking when requested
- ✅ Realistic deceleration rates

### Takeoff
- ✅ Smooth acceleration along runway
- ✅ Natural gear compression under load
- ✅ Clean liftoff transition

## Technical Details

### JSBSim Reference
Based on JSBSim's FGLGear.cpp implementation:
- Spring-damper model with separate compression/rebound damping
- No position correction - forces only
- Realistic friction coefficients
- 120Hz physics timestep (already implemented in FDMCore)

### Spring Constant Formula
```
k = Weight_on_gear / max_compression

Main gear example:
k = (5400 kg × 9.81 m/s²) / 0.35 m
k = 52,974 N / 0.35 m
k ≈ 151,000 N/m
```

### Damping Coefficient (JSBSim Standard)
```
Main gear: c = 0.20 × k = 0.20 × 150,000 = 30,000 N·s/m
Nose gear: c = 0.30 × k = 0.30 × 87,000 = 26,000 N·s/m
```

## Files Modified

1. **LandingGearModel.gd**
   - Replaced spring-damper parameters with JSBSim-based values
   - Rewrote calculate_single_gear_force() with pure JSBSim model
   - Improved friction calculation
   - Removed all problematic systems
   - Added compression_velocity tracking

2. **FDMCore.gd**
   - Removed position correction code
   - Fixed fuselage collision to use proper spring-damper
   - Improved force conversion to body frame

## Testing Recommendations

1. **Test landing at various speeds**
   - Slow approach (~70 m/s)
   - Normal approach (~100 m/s)
   - Fast approach (~130 m/s)

2. **Test taxiing**
   - Acceleration from standstill
   - Rolling at constant speed
   - Braking to full stop

3. **Test takeoff**
   - Full throttle acceleration
   - Rotation and liftoff

4. **Enable debug output**
   ```gdscript
   landing_gear.debug_landing_gear = true
   ```

## References

- JSBSim FGLGear source code: https://github.com/JSBSim-Team/jsbsim/blob/master/src/models/FGLGear.cpp
- JSBSim Ground Reactions: https://wiki.flightgear.org/JSBSim_GroundReactions
- F-16 specifications for weight distribution and gear parameters
