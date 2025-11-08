class_name FlightControlSystem
extends Node

## Flight Control System (FCS)
## Manages control surfaces, alpha limiter, stability augmentation

# Control surface limits (radians)
@export var elevator_limit: float = deg_to_rad(25.0)
@export var aileron_limit: float = deg_to_rad(20.0)
@export var rudder_limit: float = deg_to_rad(30.0)

# Control surface positions
var elevator_position: float = 0.0
var aileron_position: float = 0.0
var rudder_position: float = 0.0

# FCS modes
@export var fcs_enabled: bool = false  # Disable SAS - natural stability is better
@export var alpha_limiter_enabled: bool = true
@export var max_alpha: float = deg_to_rad(28.0)  # Normal limit
var cobra_override: bool = false

# High-speed handling
@export var high_speed_damping_enabled: bool = true  # Reduce control sensitivity at high speeds
@export var high_speed_threshold: float = 150.0  # m/s (~290 knots) - start scaling above this (lowered from 180)
@export var max_pitch_scaling: float = 0.15  # At very high speeds, reduce pitch input to 15% (was 30%)
@export var max_roll_scaling: float = 0.4  # At very high speeds, reduce roll input to 40% (was 50%)

# Pitch rate limiting (prevents rapid oscillations)
@export var pitch_rate_limiter_enabled: bool = true
@export var max_pitch_rate: float = 2.0  # rad/s - maximum pitch rate at high speeds
@export var pitch_rate_limit_threshold: float = 200.0  # m/s - engage rate limiter above this

# Stability augmentation gains (ONLY used if fcs_enabled = true)
# These should be SMALL - the aerodynamic damping is already strong
@export var pitch_rate_gain: float = 0.05
@export var roll_rate_gain: float = 0.03
@export var yaw_damper_gain: float = 0.04

# Airbrake
var airbrake_extended: bool = false

# === PSM Attitude Hold System ===
# "Point and Lock" - Player points plane, assistance locks it there

# Reference attitude (captured when PSM entered or when player releases stick)
var psm_reference_attitude: Quaternion = Quaternion.IDENTITY
var psm_initialized: bool = false

# PID controller state (one per axis: pitch, roll, yaw)
var psm_pitch_integral: float = 0.0
var psm_roll_integral: float = 0.0
var psm_yaw_integral: float = 0.0

# Deadzone for detecting player override
@export var psm_stick_deadzone: float = 0.05  # 5% deadzone - inside = hold, outside = override

# Attitude hold tolerance (natural variance)
@export_group("PSM Stability vs Responsiveness")
@export var psm_hold_tolerance_degrees: float = 2.0  ## Tolerance before corrections applied (degrees) - higher = more drift/variance
@export var psm_stability_mode: String = "balanced"  ## "tight" (minimal drift), "balanced" (some drift), "loose" (natural feel)

# PID Gains for Attitude Hold
@export_group("PSM Attitude Hold - Pitch")
@export var psm_pitch_kp: float = 2.0    # Proportional gain
@export var psm_pitch_ki: float = 0.1    # Integral gain (fights persistent forces)
@export var psm_pitch_kd: float = 0.5    # Derivative gain (damping)
@export var psm_pitch_ktv: float = 1.5   # Thrust vectoring multiplier

@export_group("PSM Attitude Hold - Roll")
@export var psm_roll_kp: float = 2.5     # Stronger for quick roll response
@export var psm_roll_ki: float = 0.15
@export var psm_roll_kd: float = 0.4
@export var psm_roll_ktv: float = 1.2

@export_group("PSM Attitude Hold - Yaw")
@export var psm_yaw_kp: float = 1.8      # Weaker (yaw naturally slower)
@export var psm_yaw_ki: float = 0.08
@export var psm_yaw_kd: float = 0.6
@export var psm_yaw_ktv: float = 1.8     # Stronger TV for yaw

# Track previous PSM state to detect mode transitions
var previous_psm_mode: bool = false

func process_control_inputs(raw_inputs: Dictionary, aircraft_state: Dictionary) -> Dictionary:
	var processed_inputs = {}

	# Get airspeed for high-speed handling
	var airspeed = aircraft_state.get("airspeed", 0.0)

	# Get PSM mode early (needed for multiple checks)
	var psm_mode = raw_inputs.get("psm_mode", false)

	# Detect PSM mode transition (entering or exiting)
	if psm_mode != previous_psm_mode:
		if psm_mode:
			print("[PSM] Entering PSM mode - Attitude Hold Active")
		else:
			print("[PSM] Exiting PSM mode - Attitude Hold Deactivated")
			reset_psm_state()
		previous_psm_mode = psm_mode

	# Calculate speed-based control scaling (fly-by-wire gain scheduling)
	var pitch_scale = 1.0
	var roll_scale = 1.0

	# HIGH-SPEED PITCH SCALING (DISABLED)
	# This was reducing pitch control to 15% at high speeds, making the aircraft uncontrollable
	# Full input authority at all speeds - rotation rate limits provide safety
	# if psm_mode:
	#     pitch_scale = 1.0  # Full authority in PSM mode
	#     roll_scale = 1.0
	# elif high_speed_damping_enabled and airspeed > high_speed_threshold:
	#     # Disabled - was reducing control input at high speeds

	# Pre-calculate PSM Attitude Assistance for all axes
	var psm_assistance = {}
	if psm_mode and aircraft_state.get("psm_attitude_assistance_enabled", true):
		# Don't apply assistance if stall recovery is active (it uses thrust vectoring)
		var stall_recovery_active = aircraft_state.get("stall_recovery_enabled", true)
		var auto_psm_from_stall = aircraft_state.get("auto_psm_engaged", false) if "auto_psm_engaged" in aircraft_state else false
		var should_apply_psm_assist = not (stall_recovery_active and auto_psm_from_stall)

		if should_apply_psm_assist:
			psm_assistance = calculate_psm_attitude_assistance(raw_inputs, aircraft_state)

	# Auto-engage PSM for deep stalls (DISABLED - manual PSM control only)
	# Reason: Auto-PSM engagement interferes with intentional high-AoA maneuvers
	# User has full control - if they want PSM, they press Space
	var auto_psm_engaged = false
	# if not psm_mode:  # Don't override manual PSM activation
	#     auto_psm_engaged = should_auto_engage_psm(aircraft_state)
	#     if auto_psm_engaged:
	#         psm_mode = true

	# Process elevator with AoA limiting (disabled in PSM mode)
	var desired_elevator = raw_inputs.get("pitch", 0.0) * pitch_scale

	# Apply PSM pitch assistance (always apply, assists with coupling resistance)
	if psm_assistance:
		desired_elevator += psm_assistance.get("pitch", 0.0)

	# Apply AoA limiter (only in normal flight, not PSM)
	if not psm_mode:
		var aoa_limiter_enabled = aircraft_state.get("aoa_limiter_enabled", true)
		var max_aoa_limit = aircraft_state.get("max_aoa_limit", deg_to_rad(15.0))
		var limiter_strength = aircraft_state.get("limiter_strength", 2.0)

		if aoa_limiter_enabled:
			desired_elevator = apply_aoa_limiter(desired_elevator, aircraft_state.get("alpha", 0.0),
												 max_aoa_limit, limiter_strength)

	# Apply stall recovery assistance (DISABLED - full manual control)
	# Reason: Stall recovery interferes with intentional high-AoA aerobatic maneuvers
	# User can manually control the aircraft's attitude without automatic assistance
	# var stall_recovery_enabled = aircraft_state.get("stall_recovery_enabled", true)
	# var manual_psm_active = psm_mode and not auto_psm_engaged
	# if stall_recovery_enabled and not manual_psm_active:
	#     var stall_recovery_input = calculate_stall_recovery(aircraft_state)
	#     desired_elevator += stall_recovery_input

	# Apply stability augmentation
	if fcs_enabled:
		desired_elevator += calculate_pitch_sas(aircraft_state)

	# HIGH-SPEED PITCH RATE LIMITER (DISABLED)
	# Removed: This system was making the aircraft feel sluggish at high speeds
	# Instead, rely on:
	# 1. Hard rotation rate limits in FDMCore (max_pitch_rate_limit, etc.)
	# 2. Natural aerodynamic damping (pitch damping, alpha damping)
	# 3. Passive roll damping to prevent oscillations
	# This provides smooth, responsive control without artifical input limiting

	elevator_position = clamp(desired_elevator, -elevator_limit, elevator_limit)

	# Process ailerons with roll SAS
	var desired_aileron = raw_inputs.get("roll", 0.0) * roll_scale

	# Apply PSM roll assistance (always apply, assists with coupling resistance)
	if psm_assistance:
		desired_aileron += psm_assistance.get("roll", 0.0)

	# Stall recovery roll assistance (DISABLED - full manual control)
	# var roll_recovery_input = calculate_roll_recovery(aircraft_state)
	# desired_aileron += roll_recovery_input

	if fcs_enabled:
		desired_aileron += calculate_roll_sas(aircraft_state)
	aileron_position = clamp(desired_aileron, -aileron_limit, aileron_limit)

	# Process rudder with yaw damper
	var desired_rudder = raw_inputs.get("yaw", 0.0)

	# Apply PSM yaw assistance (always apply, assists with coupling resistance)
	if psm_assistance:
		desired_rudder += psm_assistance.get("yaw", 0.0)

	# Stall recovery yaw assistance (DISABLED - full manual control)
	# var yaw_recovery_input = calculate_yaw_recovery(aircraft_state)
	# desired_rudder += yaw_recovery_input

	if fcs_enabled:
		desired_rudder += calculate_yaw_damper(aircraft_state)
	rudder_position = clamp(desired_rudder, -rudder_limit, rudder_limit)

	# Airbrake
	airbrake_extended = raw_inputs.get("airbrake", false)

	# Prepare output
	processed_inputs["elevator"] = elevator_position
	processed_inputs["aileron"] = aileron_position
	processed_inputs["rudder"] = rudder_position
	processed_inputs["airbrake"] = airbrake_extended
	processed_inputs["throttle"] = raw_inputs.get("throttle", 0.0)
	processed_inputs["brake"] = raw_inputs.get("brake", 0.0)
	processed_inputs["flaps"] = raw_inputs.get("flaps", 0.0)

	# Pass through PSM mode (including auto-engaged PSM) and afterburner
	processed_inputs["psm_mode"] = psm_mode
	processed_inputs["auto_psm_engaged"] = auto_psm_engaged
	processed_inputs["afterburner"] = raw_inputs.get("afterburner", false)

	# PSM assistance is applied through control surface inputs
	# No direct angular velocity override - aerodynamics fully drive the plane

	# Thrust vectoring commands (coupled to flight controls + PSM assistance)
	var tv_pitch = 0.0
	var tv_yaw = 0.0
	var tv_active = false

	# Manual thrust vectoring from player
	if raw_inputs.get("thrust_vector_active", false):
		tv_pitch = calculate_thrust_vector_pitch(raw_inputs, aircraft_state)
		tv_yaw = calculate_thrust_vector_yaw(raw_inputs, aircraft_state)
		tv_active = true

	# PSM attitude hold thrust vectoring (adds to manual if both active)
	if psm_assistance and psm_assistance.has("thrust_vector_pitch"):
		tv_pitch += psm_assistance.get("thrust_vector_pitch", 0.0)
		tv_yaw += psm_assistance.get("thrust_vector_yaw", 0.0)
		if abs(tv_pitch) > 0.01 or abs(tv_yaw) > 0.01:
			tv_active = true  # PSM can activate TV even without manual command

	# Clamp thrust vectoring to reasonable limits
	tv_pitch = clamp(tv_pitch, -5.0, 5.0)
	tv_yaw = clamp(tv_yaw, -5.0, 5.0)

	processed_inputs["vector_pitch"] = tv_pitch
	processed_inputs["vector_yaw"] = tv_yaw
	processed_inputs["thrust_vector_active"] = tv_active

	return processed_inputs

func should_auto_engage_psm(aircraft_state: Dictionary) -> bool:
	# Automatically engage PSM mode during deep stall for better recovery
	var stall_recovery_enabled = aircraft_state.get("stall_recovery_enabled", true)
	if not stall_recovery_enabled:
		return false

	var current_alpha = aircraft_state.get("alpha", 0.0)
	var full_stall_aoa = aircraft_state.get("full_stall_aoa", deg_to_rad(18.0))

	# Engage PSM if in deep stall (past full stall AoA)
	return abs(current_alpha) >= full_stall_aoa

func apply_aoa_limiter(elevator_command: float, current_alpha: float, max_alpha_rad: float, strength: float) -> float:
	# Modern fly-by-wire AoA limiter - gradually reduces pitch authority as you approach max AoA
	# This prevents stalling while still allowing aggressive maneuvering

	# Only limit nose-up (negative elevator in our convention increases AoA)
	if elevator_command < 0:  # Trying to pitch up
		# Calculate how close we are to the limit (0 = safe, 1 = at limit)
		var alpha_fraction = current_alpha / max_alpha_rad

		if alpha_fraction > 0.7:  # Start limiting at 70% of max AoA
			# Reduce pitch-up authority as we approach the limit
			var reduction_factor = 1.0 - ((alpha_fraction - 0.7) / 0.3) * strength
			reduction_factor = clamp(reduction_factor, 0.0, 1.0)
			return elevator_command * reduction_factor

	# Nose-down is always allowed (helps recovery)
	return elevator_command

func calculate_stall_recovery(aircraft_state: Dictionary) -> float:
	# Automatic stall recovery - aligns aircraft with velocity vector (direction of travel)
	# Step 1: Dampen pitch rotation
	# Step 2: Align nose with velocity vector (reduce AoA to zero)

	var current_alpha = aircraft_state.get("alpha", 0.0)
	var stall_warning_aoa = aircraft_state.get("stall_warning_aoa", deg_to_rad(70.0))
	var full_stall_aoa = aircraft_state.get("full_stall_aoa", deg_to_rad(80.0))
	var recovery_pitch_strength = aircraft_state.get("recovery_pitch_strength", 0.8)
	var angular_velocity = aircraft_state.get("angular_velocity", Vector3.ZERO)

	# Work with absolute AoA (handles both positive and negative)
	var abs_alpha = abs(current_alpha)

	# No recovery needed if below warning threshold
	if abs_alpha < stall_warning_aoa:
		return 0.0

	# Calculate recovery input strength based on how deep in the stall we are
	var recovery_factor = 0.0

	if abs_alpha >= full_stall_aoa:
		# Full stall - maximum recovery input
		recovery_factor = 1.0
	else:
		# Between warning and full stall - gradually increase recovery input as stall deepens
		# But once we've engaged recovery, keep it at full strength until below warning
		# Maps from stall_warning_aoa (0%) to full_stall_aoa (100%)
		recovery_factor = (abs_alpha - stall_warning_aoa) / (full_stall_aoa - stall_warning_aoa)
		# Clamp to ensure we have at least some recovery assist above warning threshold
		recovery_factor = max(0.3, recovery_factor)  # Minimum 30% assist above warning

	# STEP 1: Active rate damping - oppose pitch rotation to stabilize
	var pitch_rate = angular_velocity.y  # Pitch rate in JSBSim frame
	var damping_input = -pitch_rate * 0.5 * recovery_factor  # Dampen rotation proportional to recovery strength

	# STEP 2: Align with velocity vector - reduce AoA toward zero
	# Positive AoA means nose is too high, need to pitch down (positive elevator)
	# Negative AoA means nose is too low, need to pitch up (negative elevator)
	var alignment_input = recovery_pitch_strength * recovery_factor * sign(current_alpha)

	# Combine damping and alignment (damping prevents oscillation during recovery)
	var recovery_input = damping_input + alignment_input

	return recovery_input

func calculate_roll_recovery(aircraft_state: Dictionary) -> float:
	# Roll recovery - levels wings and corrects sideslip to align with velocity vector
	# Step 1: Dampen roll rotation
	# Step 2: Level wings and correct sideslip

	var current_alpha = aircraft_state.get("alpha", 0.0)
	var current_beta = aircraft_state.get("beta", 0.0)  # Sideslip angle
	var bank_angle = aircraft_state.get("bank_angle", 0.0)  # Roll angle relative to horizon
	var stall_warning_aoa = aircraft_state.get("stall_warning_aoa", deg_to_rad(70.0))
	var full_stall_aoa = aircraft_state.get("full_stall_aoa", deg_to_rad(80.0))
	var recovery_roll_strength = aircraft_state.get("recovery_roll_strength", 0.8)
	var wing_leveling_weight = aircraft_state.get("wing_leveling_weight", 0.7)
	var sideslip_correction_weight = aircraft_state.get("sideslip_correction_weight", 0.3)
	var angular_velocity = aircraft_state.get("angular_velocity", Vector3.ZERO)

	# Work with absolute AoA
	var abs_alpha = abs(current_alpha)

	# No recovery needed if below warning threshold
	if abs_alpha < stall_warning_aoa:
		return 0.0

	# Calculate recovery input strength based on how deep in the stall we are
	var recovery_factor = 0.0

	if abs_alpha >= full_stall_aoa:
		recovery_factor = 1.0
	else:
		recovery_factor = (abs_alpha - stall_warning_aoa) / (full_stall_aoa - stall_warning_aoa)
		# Clamp to ensure we have at least some recovery assist above warning threshold
		recovery_factor = max(0.3, recovery_factor)  # Minimum 30% assist above warning

	# STEP 1: Active rate damping - oppose roll rotation to stabilize
	var roll_rate = angular_velocity.x  # Roll rate in JSBSim frame
	var damping_input = -roll_rate * 0.4 * recovery_factor  # Dampen roll rotation

	# STEP 2: Alignment - level wings and correct sideslip
	var alignment_input = 0.0

	# 2a. Level the wings
	# Positive bank (right wing down) = roll left (negative aileron)
	# Negative bank (left wing down) = roll right (positive aileron)
	var wing_leveling_input = -sign(bank_angle) * abs(bank_angle) / deg_to_rad(45.0)  # Normalized to 45 degrees
	wing_leveling_input = clamp(wing_leveling_input, -1.0, 1.0)
	alignment_input += wing_leveling_input * wing_leveling_weight

	# 2b. Correct sideslip
	# Positive beta (velocity right of nose) = roll left (negative aileron)
	# Negative beta (velocity left of nose) = roll right (positive aileron)
	var sideslip_correction = -sign(current_beta) * abs(current_beta) / deg_to_rad(10.0)
	sideslip_correction = clamp(sideslip_correction, -1.0, 1.0)
	alignment_input += sideslip_correction * sideslip_correction_weight

	# Apply recovery strength to alignment
	alignment_input = alignment_input * recovery_roll_strength * recovery_factor

	# Combine damping and alignment
	var roll_input = damping_input + alignment_input
	roll_input = clamp(roll_input, -1.0, 1.0)

	return roll_input

func calculate_yaw_recovery(aircraft_state: Dictionary) -> float:
	# Yaw recovery - uses rudder to coordinate with roll and prevent spin entry
	# Step 1: Dampen yaw rotation
	# Step 2: Eliminate sideslip to align with velocity vector

	var current_alpha = aircraft_state.get("alpha", 0.0)
	var current_beta = aircraft_state.get("beta", 0.0)  # Sideslip angle
	var stall_warning_aoa = aircraft_state.get("stall_warning_aoa", deg_to_rad(70.0))
	var full_stall_aoa = aircraft_state.get("full_stall_aoa", deg_to_rad(80.0))
	var recovery_yaw_strength = aircraft_state.get("recovery_yaw_strength", 0.4)
	var angular_velocity = aircraft_state.get("angular_velocity", Vector3.ZERO)

	# Work with absolute AoA
	var abs_alpha = abs(current_alpha)

	# No recovery needed if below warning threshold
	if abs_alpha < stall_warning_aoa:
		return 0.0

	# Calculate recovery input strength
	var recovery_factor = 0.0

	if abs_alpha >= full_stall_aoa:
		recovery_factor = 1.0
	else:
		recovery_factor = (abs_alpha - stall_warning_aoa) / (full_stall_aoa - stall_warning_aoa)
		# Clamp to ensure we have at least some recovery assist above warning threshold
		recovery_factor = max(0.3, recovery_factor)  # Minimum 30% assist above warning

	# STEP 1: Active rate damping - oppose yaw rotation to prevent spin
	var yaw_rate = angular_velocity.z  # Yaw rate in JSBSim frame
	var damping_input = -yaw_rate * 0.3 * recovery_factor  # Dampen yaw rotation (lighter than pitch/roll)

	# STEP 2: Eliminate sideslip - align with velocity vector
	# This prevents spin entry and helps coordinate with roll recovery
	# Positive beta (velocity to right) = yaw right (positive rudder)
	# Negative beta (velocity to left) = yaw left (negative rudder)
	var alignment_input = sign(current_beta) * abs(current_beta) / deg_to_rad(10.0)
	alignment_input = clamp(alignment_input, -1.0, 1.0)
	alignment_input = alignment_input * recovery_yaw_strength * recovery_factor

	# Combine damping and alignment
	var yaw_input = damping_input + alignment_input

	return yaw_input

func calculate_pitch_sas(state: Dictionary) -> float:
	# Stability augmentation - dampen pitch rate
	var pitch_rate = state.get("angular_velocity", Vector3.ZERO).x
	return -pitch_rate * pitch_rate_gain

func calculate_roll_sas(state: Dictionary) -> float:
	# Roll damping
	var roll_rate = state.get("angular_velocity", Vector3.ZERO).y
	return -roll_rate * roll_rate_gain

func calculate_yaw_damper(state: Dictionary) -> float:
	# Yaw damping
	var yaw_rate = state.get("angular_velocity", Vector3.ZERO).z
	return -yaw_rate * yaw_damper_gain

func calculate_thrust_vector_pitch(inputs: Dictionary, state: Dictionary) -> float:
	# Enhanced pitch control through thrust vectoring
	var base_command = inputs.get("pitch", 0.0)

	# Add assistance for high-alpha maneuvers
	var alpha = state.get("alpha", 0.0)
	if alpha > deg_to_rad(25.0):
		return base_command * 1.5  # Increased authority at high alpha

	return base_command

func calculate_thrust_vector_yaw(inputs: Dictionary, _state: Dictionary) -> float:
	# Yaw thrust vectoring
	return inputs.get("yaw", 0.0) * 0.8

func calculate_psm_attitude_assistance(raw_inputs: Dictionary, aircraft_state: Dictionary) -> Dictionary:
	## PSM Attitude Hold System - "Point and Lock"
	## Player points plane where they want, assistance locks it there
	## When stick is centered = HOLD attitude. When stick moved = OVERRIDE and update reference.

	# Get aircraft transform for attitude calculations
	var aircraft_transform = aircraft_state.get("transform", Transform3D.IDENTITY)
	if aircraft_transform == Transform3D.IDENTITY and aircraft_state.has("position"):
		# Build transform from position and basis if available
		var fdm = get_parent()  # Assume FCS is child of FDMCore
		if fdm and fdm is RigidBody3D:
			aircraft_transform = fdm.global_transform

	var current_attitude = aircraft_transform.basis.get_rotation_quaternion()

	# Initialize reference on first PSM frame
	if not psm_initialized:
		psm_reference_attitude = current_attitude
		psm_pitch_integral = 0.0
		psm_roll_integral = 0.0
		psm_yaw_integral = 0.0
		psm_initialized = true
		print("[PSM] Attitude hold initialized - reference captured")

	# Get player inputs
	var pitch_input = raw_inputs.get("pitch", 0.0)
	var roll_input = raw_inputs.get("roll", 0.0)
	var yaw_input = raw_inputs.get("yaw", 0.0)

	# Get current angular velocity (for derivative term)
	var angular_vel = aircraft_state.get("angular_velocity", Vector3.ZERO)
	var pitch_rate = angular_vel.y  # JSBSim frame: Y = pitch
	var roll_rate = angular_vel.x   # JSBSim frame: X = roll
	var yaw_rate = angular_vel.z    # JSBSim frame: Z = yaw

	# Calculate attitude errors (reference - current) in Euler angles
	var error_angles = calculate_attitude_error(psm_reference_attitude, current_attitude)
	var pitch_error = error_angles.x
	var roll_error = error_angles.y
	var yaw_error = error_angles.z

	# Delta time (assume 120Hz physics)
	var dt = 1.0 / 120.0

	# Get tolerance based on stability mode
	var tolerance_rad = get_hold_tolerance()

	# === PITCH AXIS ===
	var pitch_assist = 0.0
	var pitch_tv_request = 0.0

	if abs(pitch_input) > psm_stick_deadzone:
		# OVERRIDE MODE - Player is pitching
		pitch_assist = 0.0  # Let player command directly (no assistance interference)
		psm_pitch_integral = 0.0  # Reset integral
		# Update reference continuously while player commands
		psm_reference_attitude = current_attitude
	else:
		# HOLD MODE - Lock pitch attitude (with tolerance deadband)
		if abs(pitch_error) > tolerance_rad:
			# Outside tolerance - apply PID correction
			# Use (error - tolerance) to create smooth transition
			var error_beyond_tolerance = sign(pitch_error) * (abs(pitch_error) - tolerance_rad)
			var P = error_beyond_tolerance * psm_pitch_kp
			psm_pitch_integral += error_beyond_tolerance * dt
			psm_pitch_integral = clamp(psm_pitch_integral, -10.0, 10.0)  # Anti-windup
			var I = psm_pitch_integral * psm_pitch_ki
			var D = -pitch_rate * psm_pitch_kd  # Damping (opposes rotation)

			pitch_assist = P + I + D
			pitch_assist = clamp(pitch_assist, -5.0, 5.0)

			# Request thrust vectoring if needed (large error or saturated control)
			if abs(pitch_error) > deg_to_rad(10.0) or abs(pitch_assist) > 4.0:
				pitch_tv_request = P * psm_pitch_ktv
		else:
			# Inside tolerance - allow natural drift, only dampen rotation
			pitch_assist = -pitch_rate * psm_pitch_kd * 0.3  # Light damping only
			psm_pitch_integral = 0.0  # Reset integral when in deadband

	# === ROLL AXIS ===
	var roll_assist = 0.0
	var roll_tv_request = 0.0

	if abs(roll_input) > psm_stick_deadzone:
		# OVERRIDE MODE - Player is rolling
		roll_assist = 0.0
		psm_roll_integral = 0.0
		psm_reference_attitude = current_attitude
	else:
		# HOLD MODE - Lock roll attitude (with tolerance deadband)
		if abs(roll_error) > tolerance_rad:
			# Outside tolerance - apply PID correction
			var error_beyond_tolerance = sign(roll_error) * (abs(roll_error) - tolerance_rad)
			var P = error_beyond_tolerance * psm_roll_kp
			psm_roll_integral += error_beyond_tolerance * dt
			psm_roll_integral = clamp(psm_roll_integral, -10.0, 10.0)
			var I = psm_roll_integral * psm_roll_ki
			var D = -roll_rate * psm_roll_kd

			roll_assist = P + I + D
			roll_assist = clamp(roll_assist, -5.0, 5.0)

			if abs(roll_error) > deg_to_rad(10.0) or abs(roll_assist) > 4.0:
				roll_tv_request = P * psm_roll_ktv
		else:
			# Inside tolerance - allow natural drift, only dampen rotation
			roll_assist = -roll_rate * psm_roll_kd * 0.3
			psm_roll_integral = 0.0

	# === YAW AXIS ===
	var yaw_assist = 0.0
	var yaw_tv_request = 0.0

	if abs(yaw_input) > psm_stick_deadzone:
		# OVERRIDE MODE - Player is yawing
		yaw_assist = 0.0
		psm_yaw_integral = 0.0
		psm_reference_attitude = current_attitude
	else:
		# HOLD MODE - Lock yaw attitude (with tolerance deadband)
		if abs(yaw_error) > tolerance_rad:
			# Outside tolerance - apply PID correction
			var error_beyond_tolerance = sign(yaw_error) * (abs(yaw_error) - tolerance_rad)
			var P = error_beyond_tolerance * psm_yaw_kp
			psm_yaw_integral += error_beyond_tolerance * dt
			psm_yaw_integral = clamp(psm_yaw_integral, -10.0, 10.0)
			var I = psm_yaw_integral * psm_yaw_ki
			var D = -yaw_rate * psm_yaw_kd

			yaw_assist = P + I + D
			yaw_assist = clamp(yaw_assist, -5.0, 5.0)

			if abs(yaw_error) > deg_to_rad(10.0) or abs(yaw_assist) > 4.0:
				yaw_tv_request = P * psm_yaw_ktv
		else:
			# Inside tolerance - allow natural drift, only dampen rotation
			yaw_assist = -yaw_rate * psm_yaw_kd * 0.3
			psm_yaw_integral = 0.0

	# Debug output
	if Engine.get_physics_frames() % 60 == 0:
		var pitch_mode = "OVERRIDE" if abs(pitch_input) > psm_stick_deadzone else ("DRIFT" if abs(pitch_error) <= tolerance_rad else "HOLD")
		var roll_mode = "OVERRIDE" if abs(roll_input) > psm_stick_deadzone else ("DRIFT" if abs(roll_error) <= tolerance_rad else "HOLD")
		var yaw_mode = "OVERRIDE" if abs(yaw_input) > psm_stick_deadzone else ("DRIFT" if abs(yaw_error) <= tolerance_rad else "HOLD")
		print("[PSM ATTITUDE HOLD - %s mode, tolerance: %.1f°]" % [psm_stability_mode, rad_to_deg(tolerance_rad)])
		print("  Pitch: %s | Error: %.1f° | Assist: %.2f" % [pitch_mode, rad_to_deg(pitch_error), pitch_assist])
		print("  Roll:  %s | Error: %.1f° | Assist: %.2f" % [roll_mode, rad_to_deg(roll_error), roll_assist])
		print("  Yaw:   %s | Error: %.1f° | Assist: %.2f" % [yaw_mode, rad_to_deg(yaw_error), yaw_assist])

	# Return control surface deflections + thrust vectoring requests
	return {
		"pitch": pitch_assist,
		"roll": roll_assist,
		"yaw": yaw_assist,
		"thrust_vector_pitch": pitch_tv_request,
		"thrust_vector_yaw": yaw_tv_request
	}

func calculate_attitude_error(reference_quat: Quaternion, current_quat: Quaternion) -> Vector3:
	## Calculate attitude error in Euler angles (pitch, roll, yaw)
	## Returns Vector3(pitch_error, roll_error, yaw_error) in radians

	# Calculate error quaternion: q_error = q_reference * q_current^-1
	var error_quat = reference_quat * current_quat.inverse()

	# Convert error quaternion to Euler angles (using Godot's convention)
	var error_euler = error_quat.get_euler()

	# Wrap angles to [-PI, PI] for shortest path
	error_euler.x = wrap_angle(error_euler.x)
	error_euler.y = wrap_angle(error_euler.y)
	error_euler.z = wrap_angle(error_euler.z)

	# Return as (pitch, roll, yaw) in JSBSim convention
	# Godot Euler: (X=pitch, Y=yaw, Z=roll) → JSBSim: (pitch, roll, yaw)
	return Vector3(error_euler.x, error_euler.z, error_euler.y)

func wrap_angle(angle: float) -> float:
	## Wrap angle to [-PI, PI] range for shortest path
	while angle > PI:
		angle -= TAU
	while angle < -PI:
		angle += TAU
	return angle

func get_hold_tolerance() -> float:
	## Get tolerance based on stability mode
	## Returns tolerance in radians

	# Apply mode-based multiplier
	var tolerance_multiplier = 1.0
	match psm_stability_mode:
		"tight":
			tolerance_multiplier = 0.5  # Half the configured tolerance (tighter hold)
		"balanced":
			tolerance_multiplier = 1.0  # Use configured tolerance as-is
		"loose":
			tolerance_multiplier = 2.0  # Double the tolerance (more drift)
		_:
			tolerance_multiplier = 1.0  # Default to balanced

	return deg_to_rad(psm_hold_tolerance_degrees) * tolerance_multiplier

func reset_psm_state():
	## Call when exiting PSM mode to reset state
	psm_initialized = false
	psm_pitch_integral = 0.0
	psm_roll_integral = 0.0
	psm_yaw_integral = 0.0

func get_control_state() -> Dictionary:
	return {
		"elevator": elevator_position,
		"aileron": aileron_position,
		"rudder": rudder_position,
		"airbrake": airbrake_extended,
		"alpha_limiter": alpha_limiter_enabled,
		"cobra_override": cobra_override
	}
