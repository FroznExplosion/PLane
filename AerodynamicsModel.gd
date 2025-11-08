class_name AerodynamicsModel
extends Node

## Aerodynamics Model - Force and Moment Calculations
## Uses coefficient buildup method like JSBSim
## Simplified surface element model (64 elements instead of 640 for performance)

# Reference geometry
@export var wing_area: float = 27.87  # m² (F-16)
@export var wing_span: float = 9.96   # m
@export var mean_aerodynamic_chord: float = 3.45  # m
@export var aspect_ratio: float = 3.2  # b²/S

# Aerodynamic data
var aero_tables: AerodynamicDataTables

# Surface elements (simplified - 64 instead of 640)
var surface_elements: Array = []
var num_elements: int = 64

func _ready():
	aero_tables = AerodynamicDataTables.new()
	initialize_surface_elements()

func initialize_surface_elements():
	# Create simplified surface element distribution
	# Wings: 40 elements, Tail: 16 elements, Fuselage: 8 elements
	surface_elements.clear()

	# Wing elements (40)
	for i in range(40):
		var element = {
			"type": "wing",
			"position": Vector3(randf_range(-2.0, 2.0), randf_range(-5.0, 5.0), 0),
			"area": wing_area / 40.0,
			"normal": Vector3.UP
		}
		surface_elements.append(element)

	# Tail elements (16)
	for i in range(16):
		var element = {
			"type": "tail",
			"position": Vector3(randf_range(-1.0, 1.0), randf_range(-2.0, 2.0), 6.0),
			"area": (wing_area * 0.25) / 16.0,
			"normal": Vector3.UP
		}
		surface_elements.append(element)

	# Fuselage elements (8)
	for i in range(8):
		var element = {
			"type": "fuselage",
			"position": Vector3(0, 0, randf_range(0.0, 8.0)),
			"area": wing_area * 0.1 / 8.0,
			"normal": Vector3.FORWARD
		}
		surface_elements.append(element)

func calculate_forces_and_moments(_velocity_body: Vector3, angular_velocity: Vector3,
								  alpha: float, beta: float, dynamic_pressure: float,
								  control_inputs: Dictionary, altitude: float, control_surface_powers: Dictionary,
								  damping_params: Dictionary = {}) -> Dictionary:

	# Get base aerodynamic coefficients from tables
	var coeffs = aero_tables.get_all_coefficients(alpha, beta)

	# Calculate forces in stability frame
	var lift = dynamic_pressure * wing_area * coeffs["CL"]
	var drag = dynamic_pressure * wing_area * coeffs["CD"]
	var side_force = dynamic_pressure * wing_area * coeffs["CY"]

	# Add control surface contributions
	var elevator_deflection = control_inputs.get("elevator", 0.0)
	var _aileron_deflection = control_inputs.get("aileron", 0.0)
	var _rudder_deflection = control_inputs.get("rudder", 0.0)
	var flaps_position = control_inputs.get("flaps", 0.0)

	# Flaps increase lift and drag when deployed
	var flap_lift_increase = flaps_position * 0.8  # Significant lift increase
	var flap_drag_increase = flaps_position * 0.4  # Moderate drag increase
	lift += dynamic_pressure * wing_area * flap_lift_increase
	drag += dynamic_pressure * wing_area * flap_drag_increase

	# Elevator affects lift and pitch moment
	lift += dynamic_pressure * wing_area * elevator_deflection * 0.5 * coeffs["elevator_eff"]

	# Transform to body frame
	var force_body = MathUtils.stability_to_body(lift, drag, side_force, alpha, beta)

	# Calculate moments
	var moments = calculate_moments(alpha, beta, angular_velocity, dynamic_pressure,
									control_inputs, coeffs, control_surface_powers, damping_params)

	# Ground effect (increases lift, reduces drag near ground)
	if altitude < wing_span:
		var ground_effect = calculate_ground_effect(altitude)
		force_body.y *= ground_effect["lift_multiplier"]
		force_body.x *= ground_effect["drag_multiplier"]

	return {
		"force": force_body,
		"moment": moments
	}

func calculate_moments(_alpha: float, _beta: float, angular_rates: Vector3,
					   q: float, controls: Dictionary, coeffs: Dictionary, control_surface_powers: Dictionary,
					   damping_params: Dictionary = {}) -> Vector3:
	var moments = Vector3.ZERO

	# Static moments from coefficients
	var Cm = coeffs["Cm"]
	var Cl = coeffs["Cl"]
	var Cn = coeffs["Cn"]

	# Roll moment
	moments.x = q * wing_area * wing_span * Cl

	# Pitch moment (includes static stability Cm_alpha)
	moments.y = q * wing_area * mean_aerodynamic_chord * Cm

	# Yaw moment
	moments.z = q * wing_area * wing_span * Cn

	# Damping moments (oppose angular rates) - CRITICAL for stability
	var p = angular_rates.x  # roll rate
	var q_rate = angular_rates.y  # pitch rate
	var r = angular_rates.z  # yaw rate

	# Proper JSBSim damping: stronger at higher speeds
	var V = max(1.0, sqrt(q / 600.0))  # Approximate airspeed from dynamic pressure

	# Standard aerodynamic damping with speed-dependent scaling
	# Scale the natural damping by airspeed to prevent discontinuities
	# This ensures smooth transitions at all speeds

	# Get ramp info for smooth scaling
	var ramp_end_speed = damping_params.get("ramp_end_speed", 1000.0)
	var ramp_ratio = clamp(V / ramp_end_speed, 0.0, 1.0)
	var ramp_factor = ramp_ratio * ramp_ratio  # Quadratic scaling (0 to 1)

	# Apply scaled aerodynamic damping (increases smoothly with speed)
	var damping_scale = (0.3 + 0.7 * ramp_factor)  # Scales from 0.3 to 1.0
	moments.x += q * wing_area * wing_span * coeffs["Clp"] * p * (wing_span / (2.0 * V)) * damping_scale
	moments.y += q * wing_area * mean_aerodynamic_chord * coeffs["Cmq"] * q_rate * (mean_aerodynamic_chord / (2.0 * V)) * damping_scale
	moments.z += q * wing_area * wing_span * coeffs["Cnr"] * r * (wing_span / (2.0 * V)) * damping_scale

	# NOTE: Removed "passive roll damping" - was triple-damping the aircraft
	# We already have Clp + high-speed damping, no need for additional passive damping

	# HIGH-SPEED DAMPING AUGMENTATION - SMOOTH QUADRATIC RAMP
	# Damping increases smoothly from 0 m/s upward with NO discontinuities
	# This prevents the jittering that occurs at sharp threshold crossings

	var pitch_str = damping_params.get("pitch_strength", 0.05)
	var alpha_str = damping_params.get("alpha_strength", 0.03)
	var roll_str = damping_params.get("roll_strength", 0.015)
	var yaw_str = damping_params.get("yaw_strength", 0.01)
	var max_mult = damping_params.get("max_multiplier", 10.0)

	# Reuse ramp_factor calculated above for consistent damping scaling
	# ramp_factor is already quadratic (V/ramp_end_speed)²
	var high_speed_factor = ramp_factor * max_mult  # Scale by max_mult

	# Only apply extra damping if factor > 0 (optimization for low speeds)
	if high_speed_factor > 0.001:
		# Pitch damping (critical for stability)
		var extra_pitch_damping = -q_rate * q * wing_area * mean_aerodynamic_chord * pitch_str * high_speed_factor
		moments.y += extra_pitch_damping

		# Alpha-based damping (directly opposes AoA changes)
		var alpha_damping = -q_rate * q * wing_area * mean_aerodynamic_chord * alpha_str * high_speed_factor
		moments.y += alpha_damping

		# Roll and yaw damping
		var extra_roll_damping = -p * q * wing_area * wing_span * roll_str * high_speed_factor
		var extra_yaw_damping = -r * q * wing_area * wing_span * yaw_str * high_speed_factor
		moments.x += extra_roll_damping
		moments.z += extra_yaw_damping

	# Control surface moments (power values from inspector)
	var elevator = controls.get("elevator", 0.0)
	var aileron = controls.get("aileron", 0.0)
	var rudder = controls.get("rudder", 0.0)

	var elevator_power = control_surface_powers.get("elevator", 0.5)
	var aileron_power = control_surface_powers.get("aileron", 0.08)
	var rudder_power = control_surface_powers.get("rudder", 0.06)

	moments.x += q * wing_area * wing_span * aileron * 1.0 * coeffs["aileron_eff"] * aileron_power
	moments.y += q * wing_area * mean_aerodynamic_chord * elevator * -1.0 * coeffs["elevator_eff"] * elevator_power
	moments.z += q * wing_area * wing_span * rudder * 1.0 * coeffs["rudder_eff"] * rudder_power

	return moments

func calculate_ground_effect(altitude_agl: float) -> Dictionary:
	# Ground effect model (Wieselsberger's formula)
	var h_over_b = altitude_agl / wing_span

	if h_over_b > 1.0:
		return {"lift_multiplier": 1.0, "drag_multiplier": 1.0}

	# Ground effect factor
	var ge_factor = 1.0 - exp(-2.48 * h_over_b)

	# Lift increases (reduced downwash)
	var lift_mult = 1.0 + 0.1 * (1.0 - ge_factor)

	# Induced drag reduces
	var drag_mult = 0.6 + 0.4 * ge_factor

	return {"lift_multiplier": lift_mult, "drag_multiplier": drag_mult}

func get_stall_state(alpha: float) -> Dictionary:
	var stall_angle = deg_to_rad(16.0)
	var deep_stall_angle = deg_to_rad(30.0)

	return {
		"is_stalled": abs(alpha) > stall_angle,
		"is_deep_stall": abs(alpha) > deep_stall_angle,
		"stall_severity": clamp((abs(alpha) - stall_angle) / (deep_stall_angle - stall_angle), 0.0, 1.0)
	}
