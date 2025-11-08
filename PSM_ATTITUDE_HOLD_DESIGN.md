# PSM Attitude Hold System Design

## Core Concept: "Point and Lock"

The PSM system should act like a **3-axis attitude hold autopilot** that the player can override at will.

### How It Works

#### 1. **Reference Capture**
When PSM mode is entered or when player releases a control:
- **Pitch Reference**: Current pitch angle (nose up/down)
- **Roll Reference**: Current roll angle (wings level)
- **Yaw Reference**: Current heading (compass direction)

#### 2. **Player Override Detection**
For each axis (pitch, roll, yaw):
- **Deadzone** (e.g., 0.05): Inside = player NOT commanding that axis
- **Active Control** (e.g., >0.05): Outside = player IS commanding that axis

#### 3. **Assistance Logic Per Axis**

```
IF player_input[axis] is inside deadzone:
    # HOLD MODE - Lock current attitude
    error = reference[axis] - current[axis]
    control_output[axis] = PID_controller(error)
    # Fight aerodynamics to maintain reference!

ELSE:
    # OVERRIDE MODE - Player is commanding
    control_output[axis] = player_input[axis] * sensitivity
    reference[axis] = current[axis]  # Update reference for when they let go
    # Let player rotate freely, update lock point
```

#### 4. **Thrust Vectoring Integration**
When aerodynamic controls aren't enough (high AoA, low speed):
- PSM assistance can command thrust vectoring
- Adds moment authority when control surfaces stall

---

## Example Scenarios

### Scenario 1: Level Flight → Yaw Right → Lock
```
1. Flying straight and level
2. Enter PSM
   - Capture: pitch=0°, roll=0°, yaw=0° (north)
3. Player yaws right (stick right)
   - Yaw: Player override active, rotate right
   - Pitch: HOLD at 0° (level) ← assistance fights drift
   - Roll: HOLD at 0° (wings level) ← assistance fights drift
4. Player reaches yaw=90° (pointing east), centers yaw stick
   - Capture new yaw reference: 90°
   - Yaw: HOLD at 90° ← lock pointing east!
   - Pitch: Still HOLD at 0°
   - Roll: Still HOLD at 0°
5. Result: Plane frozen pointing east, level, stable
```

### Scenario 2: Pitch Up Vertical → Lock
```
1. Flying level (pitch=0°)
2. Enter PSM, pull full back stick
   - Pitch: Player override, nose up to 90°
   - Roll: HOLD at current (assistance keeps wings level during pitch)
   - Yaw: HOLD at current (assistance prevents yaw drift during pitch)
3. Player reaches pitch=90° (vertical), centers stick
   - Capture: pitch=90°
   - Pitch: HOLD at 90° ← lock vertical!
   - Roll: HOLD (keeps wings oriented)
   - Yaw: HOLD (prevents slow rotation/spin)
4. Result: Plane frozen pointing straight up, not drifting
```

### Scenario 3: Cobra Maneuver
```
1. Flying fast, pitch=0°
2. Enter PSM, full back stick
   - Pitch up to 120° (past vertical, nose high)
   - Roll/Yaw: HOLD to prevent tumble
3. Airspeed drops, plane nearly stops in air
4. Player centers pitch stick
   - Lock at pitch=120°
   - Assistance fights gravity and aero forces
   - Plane hangs nose-high, stable
5. Player can now yaw to aim at target behind them
6. Player pitches forward to recover
```

---

## Control Algorithm

### Attitude Hold PID Controller

For each axis when in HOLD mode:

```gdscript
# Proportional: Error magnitude
P = (reference_angle - current_angle) * Kp

# Integral: Accumulated error (fights persistent forces)
integral_error += (reference_angle - current_angle) * dt
I = integral_error * Ki

# Derivative: Rate damping (prevents oscillation)
D = -current_angular_velocity * Kd

# Total control output
control_surface = P + I + D

# If not enough authority, add thrust vectoring
if abs(control_surface) > 0.8 and thrust_vectoring_available:
    thrust_vector = P * Ktv  # Proportional TV assist
```

### Gain Values (Starting Point)

```gdscript
# Pitch Hold
pitch_Kp = 2.0   # Proportional gain
pitch_Ki = 0.1   # Integral gain (fight gravity, aero forces)
pitch_Kd = 0.5   # Derivative gain (damping)
pitch_Ktv = 1.5  # Thrust vectoring multiplier

# Roll Hold
roll_Kp = 2.5    # Stronger for roll (faster response needed)
roll_Ki = 0.15
roll_Kd = 0.4
roll_Ktv = 1.2

# Yaw Hold
yaw_Kp = 1.8     # Weaker (yaw is slower naturally)
yaw_Ki = 0.08
yaw_Kd = 0.6
yaw_Ktv = 1.8    # Stronger TV for yaw (rudder less effective)
```

---

## Implementation Details

### Reference Angles

**Euler Angles vs Quaternions:**
- **Problem with Euler**: Gimbal lock at pitch=±90°
- **Solution**: Use quaternions for reference storage, convert to errors

```gdscript
# Store reference as quaternion
var reference_attitude_quat: Quaternion

# Calculate attitude errors in body frame
var error_angles = calculate_attitude_error(reference_quat, current_quat)
# Returns Vector3(pitch_error, roll_error, yaw_error) in radians
```

### Deadzone Logic

```gdscript
var STICK_DEADZONE = 0.05  # 5% deadzone

func is_player_commanding(axis_input: float) -> bool:
    return abs(axis_input) > STICK_DEADZONE
```

### Reference Update

```gdscript
# Each frame for each axis:
if is_player_commanding(pitch_input):
    # Player is pitching - update reference continuously
    reference_pitch = current_pitch
    pitch_integral_error = 0.0  # Reset integral
else:
    # Player not pitching - HOLD reference
    # (reference_pitch stays constant)
```

---

## Advantages Over Current System

| Feature | Current (Rate Control) | New (Attitude Hold) |
|---------|----------------------|---------------------|
| Stick centered | Stops rotation | **Freezes attitude** |
| Aero disturbances | Plane drifts | **Actively countered** |
| Pointing precision | Difficult | **Point and lock** |
| Ease of aiming | Must constantly correct | **Set and forget** |
| Feel | "Swimming" | **Rock solid** |

---

## Tuning for Feel

### More Responsive (Twitchy)
- Increase Kp (faster response to errors)
- Decrease Kd (less damping)

### More Stable (Smooth)
- Decrease Kp (slower response)
- Increase Kd (more damping)
- Increase Ki if plane drifts off reference (fights persistent forces)

### More Thrust Vectoring
- Increase Ktv (thrust vector authority)
- Lower threshold for TV activation

---

## Visual Feedback Ideas

To help player understand the system:

1. **HUD Indicators**
   - Green border when axis is LOCKED
   - Yellow border when player OVERRIDING
   - Show reference attitude (ghost horizon?)

2. **Audio Cues**
   - Beep when entering PSM (attitude captured)
   - Tone change when locking axis

3. **Subtle Visual**
   - Control surface positions visible
   - Thrust vectoring nozzle glow when active

---

## Summary

**The Goal:**
Make PSM mode feel like **flying a spaceship with thrusters** - you point where you want, let go, and the plane STAYS there. The assistance system is your invisible co-pilot constantly fighting the physics to hold your chosen attitude.

**Player Experience:**
"I pull back, nose goes up, I let go, and it STAYS UP. I can now yaw around to find my target. Perfect!"

This is exactly what you described - it makes sense and will feel amazing!
