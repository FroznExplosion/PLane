class_name PropulsionModel
extends Node

## Propulsion System with Thrust Vectoring
## Based on F-16 engine with enhanced thrust vectoring capability

# Engine parameters
@export_group("Engine Thrust")
@export_range(10000.0, 300000.0, 1000.0, "suffix:N") var max_thrust: float = 129000.0  ## Maximum thrust with afterburner (F-16: 129 kN)
@export_range(5000.0, 200000.0, 1000.0, "suffix:N") var mil_power_thrust: float = 76000.0  ## Military power without afterburner (F-16: 76 kN)
@export_range(1000.0, 20000.0, 100.0, "suffix:N") var idle_thrust: float = 5000.0  ## Idle thrust

# Current state
var current_throttle: float = 0.0
var afterburner_active: bool = false
var engine_rpm: float = 0.0  # 0 to 1

# Thrust vectoring
@export_group("Thrust Vectoring")
@export var thrust_vectoring_enabled: bool = true
var max_vector_angle: float = deg_to_rad(20.0)  ## Maximum nozzle deflection angle (set by FDMCore from thrust_vector_max)
@export var vector_rate: float = deg_to_rad(300.0)  ## Nozzle deflection rate (deg/s)

@export_group("PSM Thrust Vectoring Control Authority")
@export var psm_thrust_effectiveness: float = 1.0  ## Overall thrust vectoring effectiveness in PSM mode (0-2)
@export var psm_pitch_authority: float = 1.0  ## How much pitch input affects thrust vectoring in PSM (0-2)
@export var psm_yaw_authority: float = 0.8  ## How much yaw input affects thrust vectoring in PSM (0-2)
@export var psm_roll_to_yaw: float = 0.3  ## How much roll input affects yaw vectoring in PSM (0-1)
@export var psm_moment_multiplier: float = 1.0  ## Multiplier for rotational power of thrust vectoring in PSM (0-3)
@export var psm_thrust_magnitude_boost: float = 1.0  ## Thrust output multiplier in PSM mode (0.5-1.5)

var vector_pitch: float = 0.0  # Current pitch deflection
var vector_yaw: float = 0.0  # Current yaw deflection

# Combat thrust vectoring (set from FDMCore when combat mode is active)
var combat_moment_multiplier: float = 1.2

# Engine position relative to CG (JSBSim frame: X=forward, Y=right, Z=down)
@export var engine_position: Vector3 = Vector3(-5.0, 0, 0)  # 5m behind CG along thrust line

# Fuel consumption
@export var fuel_flow_rate: float = 2.0  # kg/s at full thrust
@export var afterburner_fuel_multiplier: float = 3.0

func calculate_thrust_force_and_moment(control_inputs: Dictionary, dt: float,
									   airspeed: float, altitude: float) -> Dictionary:
	# Update throttle
	current_throttle = control_inputs.get("throttle", 0.0)

	# Afterburner active when R key held
	afterburner_active = control_inputs.get("afterburner", false)

	# Update engine RPM (simple model)
	var target_rpm = current_throttle
	engine_rpm = move_toward(engine_rpm, target_rpm, dt * 2.0)

	# Calculate thrust magnitude
	var thrust_magnitude = calculate_thrust_magnitude(airspeed, altitude)

	# Update thrust vectoring (auto-enabled in PSM mode or combat mode)
	var psm_mode = control_inputs.get("psm_mode", false)
	var combat_tv_mode = control_inputs.get("combat_tv_mode", false)
	update_thrust_vectoring(control_inputs, dt, psm_mode, combat_tv_mode)

	# Apply PSM thrust magnitude boost
	if psm_mode:
		thrust_magnitude *= psm_thrust_magnitude_boost

	# Calculate thrust vector direction
	var thrust_direction = calculate_thrust_vector_direction()
	var thrust_force = thrust_direction * thrust_magnitude

	# Calculate moment from thrust offset
	var thrust_moment = engine_position.cross(thrust_force)

	# Add thrust vectoring moment (additional control authority)
	if thrust_vectoring_enabled:
		var tv_moment = calculate_vectoring_moment(thrust_force, psm_mode, combat_tv_mode)
		thrust_moment += tv_moment

	return {
		"force": thrust_force,
		"moment": thrust_moment,
		"fuel_flow": calculate_fuel_flow()
	}

func calculate_thrust_magnitude(airspeed: float, altitude: float) -> float:
	# Base thrust
	var base_thrust: float
	if afterburner_active:
		base_thrust = max_thrust * current_throttle
	else:
		base_thrust = idle_thrust + (mil_power_thrust - idle_thrust) * current_throttle

	# Altitude effects (thrust decreases with altitude)
	var density_ratio = max(0.3, exp(-altitude / 10000.0))  # Simple model
	base_thrust *= density_ratio

	# Speed effects (ram air)
	var mach = airspeed / 340.0
	if mach < 0.5:
		base_thrust *= (1.0 + 0.05 * mach)  # Slight increase at low speed

	# Thrust vectoring penalty (3% loss when deflected)
	if thrust_vectoring_enabled:
		var total_deflection = sqrt(vector_pitch * vector_pitch + vector_yaw * vector_yaw)
		var thrust_loss = 0.03 * (total_deflection / max_vector_angle)
		base_thrust *= (1.0 - thrust_loss)

	return base_thrust

func update_thrust_vectoring(control_inputs: Dictionary, dt: float, psm_mode: bool = false, combat_tv_mode: bool = false) -> void:
	# Thrust vectoring enabled in PSM mode, Combat mode, OR if manually enabled
	var tv_active = thrust_vectoring_enabled or psm_mode or combat_tv_mode


	if not tv_active:
		vector_pitch = 0.0
		vector_yaw = 0.0
		return

	# In PSM or Combat mode, use pitch/roll/yaw inputs directly for thrust vectoring
	var desired_pitch: float
	var desired_yaw: float

	if psm_mode:
		# PSM mode: use flight control inputs directly for thrust vectoring
		# NOTE: No active attitude corrections applied - PSM assistance is NOW PASSIVE ONLY
		# Thrust vectoring responds only to player control inputs
		var pitch_input = control_inputs.get("pitch", 0.0)
		var roll_input = control_inputs.get("roll", 0.0)
		var yaw_input = control_inputs.get("yaw", 0.0)

		# Thrust vectoring responds directly to player inputs
		# IMPORTANT: Negate pitch because positive pitch input (nose up) should deflect nozzles DOWN (positive Z in JSBSim)
		# Authority controls how much input translates to nozzle angle (0-2), clamped by max_vector_angle
		desired_pitch = -pitch_input * psm_pitch_authority * max_vector_angle
		desired_yaw = (yaw_input * psm_yaw_authority + roll_input * psm_roll_to_yaw) * max_vector_angle

		# Debug PSM
		if Engine.get_physics_frames() % 30 == 0:
			print("PSM_VECTORING: pitch_input=%.2f, psm_pitch_authority=%.1f, max_vector_angle=%.4f rad (%.1f°) | desired_pitch=%.4f rad (%.1f°)" % [
				pitch_input, psm_pitch_authority, max_vector_angle, rad_to_deg(max_vector_angle), desired_pitch, rad_to_deg(desired_pitch)
			])

	elif combat_tv_mode:
		# Combat mode: Use PSM thrust vectoring authority values (same animation behavior)
		# This ensures combat TV nozzle moves identically to PSM mode
		var pitch_input = control_inputs.get("pitch", 0.0)
		var roll_input = control_inputs.get("roll", 0.0)
		var yaw_input = control_inputs.get("yaw", 0.0)

		# Use PSM authority values for consistent nozzle movement
		# Combat TV shares the same animation path as PSM
		desired_pitch = -pitch_input * psm_pitch_authority * max_vector_angle
		desired_yaw = (yaw_input * psm_yaw_authority + roll_input * psm_roll_to_yaw) * max_vector_angle

		# Debug Combat TV with all intermediate values
		if Engine.get_physics_frames() % 30 == 0:
			var intermediate_calc = pitch_input * psm_pitch_authority * max_vector_angle
			print("COMBAT_TV_CALC_DETAIL: pitch_input=%.4f, psm_pitch_auth=%.4f, max_vector_angle=%.4f rad (%.1f°) | pitch*auth*angle=%.4f rad (%.1f°) | desired_pitch=%.4f rad (%.1f°)" % [
				pitch_input, psm_pitch_authority, max_vector_angle, rad_to_deg(max_vector_angle),
				intermediate_calc, rad_to_deg(intermediate_calc), desired_pitch, rad_to_deg(desired_pitch)
			])
			print("COMBAT_TV_VECTORING: pitch_input=%.2f, psm_pitch_authority=%.1f, max_vector_angle=%.4f rad (%.1f°) | desired_pitch=%.4f rad (%.1f°)" % [
				pitch_input, psm_pitch_authority, max_vector_angle, rad_to_deg(max_vector_angle), desired_pitch, rad_to_deg(desired_pitch)
			])

	else:
		# Normal mode: use dedicated thrust vector commands
		desired_pitch = control_inputs.get("vector_pitch", 0.0) * max_vector_angle
		desired_yaw = control_inputs.get("vector_yaw", 0.0) * max_vector_angle

	# Clamp desired values to max deflection before rate limiting
	desired_pitch = clamp(desired_pitch, -max_vector_angle, max_vector_angle)
	desired_yaw = clamp(desired_yaw, -max_vector_angle, max_vector_angle)

	# For PSM and Combat TV, directly set to desired angle (no rate limiting)
	# The visual animation layer will smooth the movement via lerp_angle
	if psm_mode or combat_tv_mode:
		# Direct assignment for control-based thrust vectoring
		# Visual smoothing happens in AircraftVisualController.animate_thrust_vectoring()
		vector_pitch = desired_pitch
		vector_yaw = desired_yaw

		# Debug output
		if Engine.get_physics_frames() % 30 == 0:
			if psm_mode:
				print("PSM_DIRECT: desired_pitch=%.4f rad (%.1f°), vector_pitch=%.4f rad (%.1f°)" % [
					desired_pitch, rad_to_deg(desired_pitch), vector_pitch, rad_to_deg(vector_pitch)
				])
			elif combat_tv_mode:
				print("COMBAT_TV_DIRECT: desired_pitch=%.4f rad (%.1f°), vector_pitch=%.4f rad (%.1f°)" % [
					desired_pitch, rad_to_deg(desired_pitch), vector_pitch, rad_to_deg(vector_pitch)
				])
	else:
		# Apply rate limiting for manual thrust vector commands
		var old_pitch = vector_pitch
		var old_yaw = vector_yaw
		vector_pitch = move_toward(vector_pitch, desired_pitch, vector_rate * dt)
		vector_yaw = move_toward(vector_yaw, desired_yaw, vector_rate * dt)


func calculate_thrust_vector_direction() -> Vector3:
	# Thrust points forward in JSBSim body frame (+X direction)
	var thrust_dir = Vector3(1, 0, 0)  # Forward in JSBSim frame

	if thrust_vectoring_enabled and (abs(vector_pitch) > 0.001 or abs(vector_yaw) > 0.001):
		# Apply thrust vectoring using spherical coordinates
		# The thrust vector is rotated from the forward direction by pitch and yaw angles
		# JSBSim frame: X=forward, Y=right, Z=down
		# - Pitch rotation around Y axis: positive = nose down (but we want intuitive control)
		# - Yaw rotation around Z axis: positive = nose right

		# Create the base forward vector and rotate it
		# Use proper 3D rotation matrix approach for stability
		var cos_pitch = cos(vector_pitch)
		var sin_pitch = sin(vector_pitch)
		var cos_yaw = cos(vector_yaw)
		var sin_yaw = sin(vector_yaw)

		# Rotation matrix for pitch around Y axis, then yaw around Z axis
		# Result: thrust components in JSBSim frame
		# IMPORTANT: Negate yaw component because nozzle is behind CG (X=-5m)
		# Physics: Force at +Y applied at X=-5 creates NEGATIVE Z moment (nose left)
		# To get positive Z moment (nose right), we need force at -Y instead
		var x_component = cos_pitch * cos_yaw
		var y_component = -sin_yaw  # NEGATED: correct moment direction for rear-mounted nozzle
		var z_component = sin_pitch * cos_yaw

		thrust_dir = Vector3(x_component, y_component, z_component)

	return thrust_dir.normalized()

func calculate_vectoring_moment(thrust_force: Vector3, psm_mode: bool = false, combat_tv_mode: bool = false) -> Vector3:
	# Additional moment from redirected thrust
	# Nozzle is behind CG, so deflection creates strong pitch/yaw moments
	var moment_arm_length = 7.0  # Effective moment arm for nozzle

	# Apply mode-specific moment multiplier
	var effective_moment_arm = moment_arm_length
	if psm_mode:
		effective_moment_arm *= psm_moment_multiplier
		effective_moment_arm *= psm_thrust_effectiveness  # Effectiveness scales moment power
	elif combat_tv_mode:
		# Combat mode uses a different moment multiplier (stored from FDMCore)
		effective_moment_arm *= combat_moment_multiplier
		# Note: combat_thrust_effectiveness is available via control_inputs if needed

	# Scale thrust vectoring effectiveness by current throttle
	# More throttle = more thrust = more powerful vectoring moments
	# Minimum of 0.1 to ensure some control even at idle
	var throttle_scaling = max(0.1, current_throttle)
	effective_moment_arm *= throttle_scaling

	# Calculate moments (JSBSim frame: X=roll, Y=pitch, Z=yaw)
	var pitch_moment = Vector3(0, thrust_force.length() * sin(vector_pitch) * effective_moment_arm, 0)
	var yaw_moment = Vector3(0, 0, thrust_force.length() * sin(vector_yaw) * effective_moment_arm)

	return pitch_moment + yaw_moment

func calculate_fuel_flow() -> float:
	# Fuel flow in kg/s
	var flow = fuel_flow_rate * current_throttle

	if afterburner_active:
		flow *= afterburner_fuel_multiplier

	return flow

func get_engine_state() -> Dictionary:
	return {
		"throttle": current_throttle,
		"rpm": engine_rpm,
		"afterburner": afterburner_active,
		"vector_pitch": vector_pitch,
		"vector_yaw": vector_yaw,
		"thrust_vectoring_active": thrust_vectoring_enabled
	}
