class_name FDMCore
extends RigidBody3D

## Flight Dynamics Model Core - 6-DOF Physics
## Based on JSBSim architecture with 120Hz fixed timestep
##
## This is the main flight dynamics integrator that runs at 120Hz.
## It handles 6 degrees of freedom (3 translational + 3 rotational).
##
## Coordinate System (JSBSim Convention):
## - X-axis: Forward (nose direction)
## - Y-axis: Right wing
## - Z-axis: Down
##
## Note: Internally converted from Godot's coordinate system
## (Godot: X=right, Y=up, Z=backward)

# Physics timestep
const PHYSICS_TIMESTEP: float = PhysicsConstants.PHYSICS_TIMESTEP
var accumulated_time: float = 0.0

# State vectors (JSBSim body frame: X=forward, Y=right, Z=down)
var velocity_body: Vector3 = Vector3.ZERO  # m/s - Components: u (forward), v (right), w (down)
var angular_velocity_body: Vector3 = Vector3.ZERO  # rad/s - Components: p (roll), q (pitch), r (yaw)
var orientation_quaternion: Quaternion = Quaternion.IDENTITY

# Force and moment accumulators (in body frame)
var total_force_body: Vector3 = Vector3.ZERO  # Newtons
var total_moment_body: Vector3 = Vector3.ZERO  # Newton-meters

# Aircraft Mass Properties
@export_group("Mass Properties")
@export var aircraft_mass: float = 11340.0 ## Empty aircraft mass (kg) - F-16: 11,340 kg
@export var fuel_mass: float = 3175.0 ## Current fuel mass (kg) - F-16 max internal: 3,175 kg
var inertia_tensor: Basis = Basis()  # Calculated from Ixx, Iyy, Izz, Ixz

# Moment of Inertia Values
@export_group("Inertia Tensor")
@export var Ixx: float = 12874.0 ## Roll inertia (kg·m²)
@export var Iyy: float = 75673.0 ## Pitch inertia (kg·m²)
@export var Izz: float = 85552.0 ## Yaw inertia (kg·m²)
@export var Ixz: float = 1331.0 ## Product of inertia (kg·m²)

# Control Effectiveness - Normal Flight
@export_group("Control Surfaces - Normal Flight")
@export var elevator_power: float = 0.5 ## Elevator (pitch) control effectiveness
@export var aileron_power: float = 0.08 ## Aileron (roll) control effectiveness
@export var rudder_power: float = 0.06 ## Rudder (yaw) control effectiveness

# Control Effectiveness - PSM Mode (Post-Stall Maneuver)
@export_group("Control Surfaces - PSM Mode")
@export var psm_elevator_power: float = 5.0 ## Elevator power in PSM mode (10x normal)
@export var psm_aileron_power: float = 2.0 ## Aileron power in PSM mode (25x normal)
@export var psm_rudder_power: float = 1.5 ## Rudder power in PSM mode (25x normal)

# Maneuverability Settings
@export_group("Maneuverability Tuning")
@export var wing_loading_multiplier: float = 1.0 ## Effective wing size multiplier (higher = more lift, tighter turns) (0.5-2.0)
@export var maneuver_mass_multiplier: float = 1.0 ## Effective mass for maneuvers (lower = more agile) (0.5-1.5)
@export var instantaneous_turn_rate_boost: float = 1.0 ## Turn rate multiplier (higher = faster direction change) (0.5-3.0)

# Angle of Attack Limiter (Fly-by-Wire)
@export_group("Flight Envelope Protection")
@export var aoa_limiter_enabled: bool = true ## Enable AoA limiter (prevents stalling in normal flight)
@export var max_aoa_limit: float = 15.0 ## Maximum AoA before limiter engages (degrees)
@export var limiter_strength: float = 2.0 ## How aggressively to limit pitch input near max AoA

# Stall Recovery System
@export_group("Stall Recovery Assistance")
@export var stall_recovery_enabled: bool = true ## Automatic stall recovery assistance
@export var stall_warning_aoa: float = 70.0 ## AoA where recovery assistance starts (degrees)
@export var full_stall_aoa: float = 80.0 ## AoA where full recovery kicks in (degrees)
@export var recovery_pitch_strength: float = 0.8 ## Pitch recovery strength (elevator input)
@export var recovery_roll_strength: float = 0.8 ## Roll recovery strength (aileron input)
@export var recovery_yaw_strength: float = 0.4 ## Yaw recovery strength (rudder input) - gentler than pitch/roll
@export var wing_leveling_weight: float = 0.7 ## How much roll recovery prioritizes leveling wings (0-1)
@export var sideslip_correction_weight: float = 0.3 ## How much roll recovery prioritizes sideslip (0-1)

# Automatic Trim
@export_group("Automatic Trim")
@export var auto_trim_enabled: bool = true ## Automatically reduce AoA to zero when no pitch input
@export var auto_trim_strength: float = 0.3 ## How aggressively trim adjusts (higher = faster)
@export var auto_trim_deadzone: float = 0.05 ## Pitch input threshold for trim activation

# Airbrake
@export_group("Airbrake (Hold Throttle Down)")
@export var airbrake_drag_multiplier: float = 3.0 ## Drag multiplier when airbrake deployed (1.0-10.0)

# High-Speed Damping System (Smooth Quadratic Ramp - No Discontinuities)
@export_group("High-Speed Damping (Smooth Ramp)")
@export var damping_ramp_end_speed: float = 1000.0 ## Speed where damping reaches full strength (m/s) - damping ramps smoothly from 0 m/s
@export var pitch_damping_strength: float = 0.05 ## Pitch damping coefficient (0.01-0.1, higher = more damping)
@export var alpha_damping_strength: float = 0.03 ## AoA rate damping coefficient (0.01-0.08)
@export var roll_damping_strength: float = 0.015 ## Roll damping coefficient (0.005-0.03)
@export var yaw_damping_strength: float = 0.01 ## Yaw damping coefficient (0.005-0.02)
@export var max_damping_multiplier: float = 10.0 ## Maximum damping multiplier at full ramp speed (3.0-15.0)

# Visual Animation Limits
@export_group("Visual Animation Limits (Degrees)")
@export var aileron_deflection_max: float = 20.0 ## Maximum aileron deflection angle
@export var elevator_deflection_max: float = 25.0 ## Maximum elevator deflection angle
@export var rudder_deflection_max: float = 30.0 ## Maximum rudder deflection angle
@export var flap_deflection_max: float = 40.0 ## Maximum flap deployment angle
@export var airbrake_deflection_max: float = 60.0 ## Maximum airbrake opening angle
@export var thrust_vector_max: float = 20.0 ## Maximum thrust vectoring angle

# PSM Attitude Assistance (Coupling Resistance)
@export_group("PSM Attitude Assistance")
@export var psm_attitude_assistance_enabled: bool = true ## Enable coupling resistance assistance
@export var psm_max_pitch_rate: float = 8.0 ## Maximum desired pitch rate in PSM mode (rad/s)
@export var psm_max_roll_rate: float = 10.0 ## Maximum desired roll rate in PSM mode (rad/s)
@export var psm_max_yaw_rate: float = 6.0 ## Maximum desired yaw rate in PSM mode (rad/s)
@export var psm_aerodynamic_coupling_factor: float = 0.15 ## How much control surfaces fight aerodynamic coupling (0.0-1.0)
@export var psm_aerodynamic_effect_multiplier: float = 0.05 ## Scale of aerodynamic effects in PSM mode (0.0=none, 1.0=full) - reduces natural yaw/pitch coupling
@export var psm_form_drag_multiplier: float = 2.0 ## Additional drag when top/bottom/sides face velocity in PSM (0-10)
@export var psm_direct_control_mode: bool = true ## PSM uses direct rate control (spaceship-like) instead of attitude hold
@export var psm_rate_authority: float = 8.0 ## How strongly PSM controls rotation rates (0-20)

# Rotation Rate Limits
@export_group("Rotation Rate Limits")
@export var max_pitch_rate_limit: float = 12.0 ## Hard limit for pitch rotation rate (rad/s, ~690°/s)
@export var max_roll_rate_limit: float = 12.0 ## Hard limit for roll rotation rate (rad/s, ~690°/s)
@export var max_yaw_rate_limit: float = 8.0 ## Hard limit for yaw rotation rate (rad/s, ~460°/s)

# PSM Thrust Vectoring Settings
@export_group("PSM Thrust Vectoring (Space)")
@export var psm_thrust_effectiveness: float = 1.0 ## Overall thrust vectoring effectiveness in PSM (0-2)
@export var psm_pitch_authority: float = 1.0 ## Pitch input affects thrust vectoring (0-2)
@export var psm_yaw_authority: float = 0.8 ## Yaw input affects thrust vectoring (0-2)
@export var psm_roll_to_yaw: float = 0.3 ## Roll input contributes to yaw vectoring (0-1)
@export var psm_moment_multiplier: float = 1.0 ## Rotational power multiplier (0-3)

# Combat Thrust Vectoring Settings (Afterburner + Heavy Control)
@export_group("Combat Thrust Vectoring (Afterburner + 80% Controls)")
@export var combat_tv_enabled: bool = true ## Enable automatic combat thrust vectoring when afterburner + heavy controls
@export var combat_control_threshold: float = 0.8 ## Control surface deflection threshold to activate (0.0-1.0)
@export var combat_thrust_effectiveness: float = 1.5 ## Overall thrust vectoring effectiveness in combat mode (0-2)
@export var combat_pitch_authority: float = 1.2 ## Pitch input affects thrust vectoring in combat (0-2)
@export var combat_yaw_authority: float = 1.0 ## Yaw input affects thrust vectoring in combat (0-2)
@export var combat_roll_to_yaw: float = 0.4 ## Roll input contributes to yaw vectoring in combat (0-1)
@export var combat_moment_multiplier: float = 1.2 ## Rotational power multiplier in combat mode (0-3)

# Speed Limits
@export_group("Speed Limits")
@export var max_speed_limit: float = 1700.0 ## Hard speed limit (m/s) - Mach 5 at sea level = ~1700 m/s
@export var speed_damping_threshold: float = 1500.0 ## Speed above which damping applies (m/s) - slightly below max_speed_limit
@export var speed_damping_rate: float = 0.95 ## Speed damping per frame (0.9-0.99, lower = faster deceleration)

# Engine Thrust
@export_group("Engine Thrust")
@export var max_thrust: float = 129000.0 ## Maximum thrust with afterburner (Newtons) - F-16: 129,000 N
@export var mil_power_thrust: float = 76000.0 ## Military power without afterburner (Newtons) - F-16: 76,000 N
@export var idle_thrust: float = 5000.0 ## Idle thrust (Newtons)

# Flight State (calculated each frame)
@export_group("Flight State (Read-Only)")
var altitude_msl: float = 1000.0  # Mean Sea Level altitude in meters
var altitude_agl: float = 1000.0  # Above Ground Level altitude in meters
var airspeed: float = 0.0  # Total airspeed magnitude in m/s
var angle_of_attack: float = 0.0  # Alpha (α) in radians - angle between body X-axis and velocity
var sideslip_angle: float = 0.0  # Beta (β) in radians - lateral angle of velocity
var mach_number: float = 0.0  # Airspeed / speed of sound
var dynamic_pressure: float = 0.0  # q = 0.5 * ρ * V² in Pascals
var linear_acceleration: Vector3 = Vector3.ZERO  # Linear acceleration in world frame (m/s²)
var g_force: float = 1.0  # G-force magnitude (1.0 = 1G)

# Subsystems (created automatically in _ready)
var aerodynamics: AerodynamicsModel  # Calculates lift, drag, and moments
var propulsion: PropulsionModel  # Engine thrust and thrust vectoring
var landing_gear: LandingGearModel  # Ground contact forces
var fcs: FlightControlSystem  # Flight Control System with SAS and limiters
var atmosphere: AtmosphereModel  # ISA atmosphere model
var terrain: TerrainGenerator  # Terrain height lookup

# Debug visualization
var gear_debug_visualizer: GearDebugVisualizer  # Visual gear contact indicators

# Control inputs (set by FlightInputController)
var control_inputs: Dictionary = {}  # Contains: pitch, roll, yaw, throttle, brake, etc.
var processed_controls: Dictionary = {}  # Contains: elevator, aileron, rudder (processed by FCS)

func _ready():
	# Initialize inertia tensor
	inertia_tensor = Basis(
		Vector3(Ixx, 0, -Ixz),
		Vector3(0, Iyy, 0),
		Vector3(-Ixz, 0, Izz)
	)

	# Disable Godot's built-in physics
	gravity_scale = 0.0
	linear_damp = 0.0
	angular_damp = 0.0
	can_sleep = false
	lock_rotation = false
	freeze = false
	freeze_mode = FREEZE_MODE_KINEMATIC

	# Use custom integrator
	custom_integrator = true

	# Set mass
	mass = aircraft_mass + fuel_mass

	# Initialize position and orientation
	global_position = Vector3(0, altitude_msl, 0)
	global_rotation = Vector3.ZERO  # Level flight

	# Initialize velocity (flying forward at cruise speed)
	var initial_speed = 200.0  # m/s (~400 knots)
	linear_velocity = -global_basis.z * initial_speed
	angular_velocity = Vector3.ZERO

	# Initialize subsystems
	initialize_subsystems()

	print("FDMCore initialized - 120Hz physics active")

func initialize_subsystems():
	# Create subsystems
	atmosphere = AtmosphereModel.new()
	add_child(atmosphere)

	aerodynamics = AerodynamicsModel.new()
	add_child(aerodynamics)

	propulsion = PropulsionModel.new()
	# Pass thrust values from FDMCore to PropulsionModel
	propulsion.max_thrust = max_thrust
	propulsion.mil_power_thrust = mil_power_thrust
	propulsion.idle_thrust = idle_thrust
	# Pass thrust vectoring max angle from visual limits
	propulsion.max_vector_angle = deg_to_rad(thrust_vector_max)
	# Pass PSM thrust vectoring settings
	propulsion.psm_thrust_effectiveness = psm_thrust_effectiveness
	propulsion.psm_pitch_authority = psm_pitch_authority
	propulsion.psm_yaw_authority = psm_yaw_authority
	propulsion.psm_roll_to_yaw = psm_roll_to_yaw
	propulsion.psm_moment_multiplier = psm_moment_multiplier
	add_child(propulsion)

	landing_gear = LandingGearModel.new()
	add_child(landing_gear)

	fcs = FlightControlSystem.new()
	add_child(fcs)

	# Create debug visualizer (always create, can be toggled)
	gear_debug_visualizer = GearDebugVisualizer.new()
	add_child(gear_debug_visualizer)
	# Initially hidden - will show when landing gear debug is enabled
	gear_debug_visualizer.set_visibility(false)

	# Find terrain in scene
	terrain = get_node_or_null("/root/Main/TerrainGenerator")
	if not terrain:
		push_warning("TerrainGenerator not found - landing gear will use Y=0 as ground")

func _integrate_forces(state: PhysicsDirectBodyState3D):
	var delta = state.step
	accumulated_time += delta

	# Run physics at 120Hz fixed timestep
	while accumulated_time >= PHYSICS_TIMESTEP:
		integrate_equations_of_motion_with_state(state, PHYSICS_TIMESTEP)
		accumulated_time -= PHYSICS_TIMESTEP

func integrate_equations_of_motion_with_state(state: PhysicsDirectBodyState3D, dt: float):
	# Clear accumulators
	total_force_body = Vector3.ZERO
	total_moment_body = Vector3.ZERO

	# Update atmospheric conditions
	altitude_msl = state.transform.origin.y
	altitude_agl = max(0.0, altitude_msl)  # Simplified (no terrain lookup)
	atmosphere.update_atmosphere(altitude_msl)

	# Get current velocity in Godot body frame
	var velocity_godot_body = state.transform.basis.inverse() * state.linear_velocity
	var angular_velocity_godot_body = state.transform.basis.inverse() * state.angular_velocity

	# Convert to JSBSim body frame (X=forward, Y=right, Z=down)
	velocity_body = godot_to_jsbsim_vector(velocity_godot_body)
	angular_velocity_body = godot_to_jsbsim_vector(angular_velocity_godot_body)

	# Calculate flight state
	update_flight_state()

	# Process control inputs through FCS
	processed_controls = fcs.process_control_inputs(control_inputs, get_aircraft_state())

	# Detect combat thrust vectoring and add to processed_controls for HUD access
	# Pass BOTH processed_controls (for elevator/aileron/rudder positions) AND control_inputs (for raw pitch/roll/yaw)
	var combat_tv_mode = detect_combat_thrust_vectoring(processed_controls, control_inputs)
	processed_controls["combat_tv_mode"] = combat_tv_mode

	# Accumulate forces and moments from all sources
	accumulate_gravitational_forces()
	accumulate_aerodynamic_forces(processed_controls)
	accumulate_propulsion_forces(processed_controls, control_inputs, dt)  # Pass both for thrust vectoring to access raw inputs
	accumulate_landing_gear_forces(processed_controls, dt)

	# Debug: print every 60 frames (0.5 seconds)
	if Engine.get_physics_frames() % 60 == 0:
		var psm_status = "PSM ON" if processed_controls.get("psm_mode", false) else "Normal"
		var high_speed_status = ""
		if fcs and fcs.high_speed_damping_enabled and airspeed > fcs.high_speed_threshold:
			var speed_ratio = (airspeed - fcs.high_speed_threshold) / fcs.high_speed_threshold
			speed_ratio = clamp(speed_ratio, 0.0, 1.0)
			var pitch_scale = 1.0 - (1.0 - fcs.max_pitch_scaling) * speed_ratio
			high_speed_status = " | HIGH-SPEED (pitch: %.0f%%)" % (pitch_scale * 100.0)
		print("%s%s | AoA: %.1f° | Speed: %.0f m/s | G-Force: %.1f" % [psm_status, high_speed_status, rad_to_deg(angle_of_attack), airspeed, g_force])

	# Integrate translational dynamics (F = ma) in JSBSim frame
	# Apply maneuver mass multiplier (lower = more responsive to forces)
	var effective_mass = mass * maneuver_mass_multiplier
	var linear_acceleration_body = total_force_body / effective_mass

	# Clamp acceleration to prevent explosions and oscillations
	# Reduced from 500 (50G) to 300 (30G) to prevent extreme forces at high speed
	var max_accel = 300.0  # ~30G max
	if linear_acceleration_body.length() > max_accel:
		linear_acceleration_body = linear_acceleration_body.normalized() * max_accel

	# Convert acceleration to world frame for G-force calculation
	var acceleration_godot_body = jsbsim_to_godot_vector(linear_acceleration_body)
	linear_acceleration = state.transform.basis * acceleration_godot_body

	# Calculate G-force (total acceleration / gravity, accounting for gravity itself)
	# G-force is felt acceleration: positive = pushing into seat, negative = floating
	# We need the vertical component relative to the aircraft
	var gravity_vector = Vector3(0, -PhysicsConstants.GRAVITY, 0)  # World down
	var total_acceleration = linear_acceleration - gravity_vector  # Subtract gravity

	# G-force along aircraft's vertical axis (up through cockpit)
	var aircraft_up = state.transform.basis.y  # Aircraft's up direction in world space
	g_force = aircraft_up.dot(total_acceleration) / PhysicsConstants.GRAVITY

	velocity_body += linear_acceleration_body * dt

	# Apply speed limiting with damping for over-speed
	var airspeed_magnitude = velocity_body.length()

	# Hard limit: clamp to max_speed_limit if exceeded
	if airspeed_magnitude > max_speed_limit:
		var speed_ratio = max_speed_limit / airspeed_magnitude
		velocity_body *= speed_ratio

	# Soft damping: if above threshold but below limit, apply damping
	elif airspeed_magnitude > speed_damping_threshold:
		velocity_body *= speed_damping_rate

	# Integrate rotational dynamics (Euler's equations) in JSBSim frame
	var angular_acceleration = calculate_angular_acceleration()

	# NOTE: Angular acceleration clamping removed - rely on rotation rate limits instead
	# The hard rotation rate limits per axis (max_pitch_rate_limit, etc.) provide sufficient control
	# without making the aircraft feel sluggish

	angular_velocity_body += angular_acceleration * dt

	# Apply hard rotation rate limits per axis
	# JSBSim body frame: X=forward (pitch), Y=right (roll), Z=down (yaw)
	angular_velocity_body.x = clamp(angular_velocity_body.x, -max_pitch_rate_limit, max_pitch_rate_limit)
	angular_velocity_body.y = clamp(angular_velocity_body.y, -max_roll_rate_limit, max_roll_rate_limit)
	angular_velocity_body.z = clamp(angular_velocity_body.z, -max_yaw_rate_limit, max_yaw_rate_limit)

	# PSM MODE: Full aerodynamic effects with coupling resistance
	# In PSM mode, aerodynamics fully drive the plane, but control surfaces can fight coupling
	# FlightControlSystem generates control surface inputs using desired rotation rates to resist coupling

	# Convert updated velocities back to Godot body frame
	var velocity_godot_body_new = jsbsim_to_godot_vector(velocity_body)
	var angular_velocity_godot_body_new = jsbsim_to_godot_vector(angular_velocity_body)

	# Transform to world frame
	var new_linear_velocity = state.transform.basis * velocity_godot_body_new
	var new_angular_velocity = state.transform.basis * angular_velocity_godot_body_new

	# Clamp velocities for safety
	if new_linear_velocity.length() > PhysicsConstants.MAX_SAFE_VELOCITY:
		new_linear_velocity = new_linear_velocity.normalized() * PhysicsConstants.MAX_SAFE_VELOCITY
	if new_angular_velocity.length() > PhysicsConstants.MAX_SAFE_ANGULAR_VEL:
		new_angular_velocity = new_angular_velocity.normalized() * PhysicsConstants.MAX_SAFE_ANGULAR_VEL

	# Apply velocities to state
	state.linear_velocity = new_linear_velocity
	state.angular_velocity = new_angular_velocity

func calculate_angular_acceleration() -> Vector3:
	# Euler's equations: I·ω̇ + ω × (I·ω) = M
	# Apply maneuver mass multiplier to inertia (lower = faster rotation)
	var effective_inertia = inertia_tensor * maneuver_mass_multiplier
	var I_omega = effective_inertia * angular_velocity_body
	var omega_cross_I_omega = angular_velocity_body.cross(I_omega)
	var net_moment = total_moment_body - omega_cross_I_omega
	return effective_inertia.inverse() * net_moment

## Coordinate System Conversion Functions
## Godot: X=right, Y=up, Z=backward (aircraft faces -Z)
## JSBSim: X=forward, Y=right, Z=down

func godot_to_jsbsim_vector(godot_vec: Vector3) -> Vector3:
	## Convert vector from Godot body frame to JSBSim body frame
	## Godot (X, Y, Z) → JSBSim (u, v, w)
	return Vector3(
		-godot_vec.z,  # JSBSim X (forward) = -Godot Z
		godot_vec.x,   # JSBSim Y (right) = Godot X
		-godot_vec.y   # JSBSim Z (down) = -Godot Y
	)

func jsbsim_to_godot_vector(jsbsim_vec: Vector3) -> Vector3:
	## Convert vector from JSBSim body frame to Godot body frame
	## JSBSim (u, v, w) → Godot (X, Y, Z)
	return Vector3(
		jsbsim_vec.y,   # Godot X (right) = JSBSim Y
		-jsbsim_vec.z,  # Godot Y (up) = -JSBSim Z
		-jsbsim_vec.x   # Godot Z (backward) = -JSBSim X
	)

func update_flight_state():
	# Calculate airspeed (total velocity magnitude)
	airspeed = velocity_body.length()

	# Calculate angle of attack (AoA) in JSBSim frame
	# AoA = atan2(w, u) where:
	# - u = velocity_body.x (forward velocity)
	# - w = velocity_body.z (downward velocity)
	# Positive AoA means nose is above the velocity vector
	if abs(velocity_body.x) > 0.1:
		angle_of_attack = atan2(velocity_body.z, velocity_body.x)
	else:
		angle_of_attack = 0.0

	# Calculate sideslip angle (beta) in JSBSim frame
	# Beta = asin(v / |V|) where:
	# - v = velocity_body.y (right velocity)
	# Positive beta means velocity vector is to the right of nose
	if airspeed > 0.1:
		sideslip_angle = asin(clamp(velocity_body.y / airspeed, -1.0, 1.0))
	else:
		sideslip_angle = 0.0

	# Calculate Mach number
	mach_number = atmosphere.calculate_mach_number(airspeed)

	# Calculate dynamic pressure
	dynamic_pressure = atmosphere.calculate_dynamic_pressure(airspeed)

func accumulate_gravitational_forces():
	# Gravity always acts downward in world frame (Godot: -Y direction)
	var gravity_force_world = Vector3(0, -mass * PhysicsConstants.GRAVITY, 0)

	# Transform to Godot body frame
	var gravity_force_godot_body = global_basis.inverse() * gravity_force_world

	# Convert to JSBSim body frame for physics calculations
	var gravity_force_jsbsim = godot_to_jsbsim_vector(gravity_force_godot_body)

	total_force_body += gravity_force_jsbsim

func accumulate_aerodynamic_forces(controls: Dictionary):
	if airspeed < 1.0:
		return

	# Select control surface powers based on mode
	var psm_mode = controls.get("psm_mode", false)
	var control_surface_powers = {
		"elevator": psm_elevator_power if psm_mode else elevator_power,
		"aileron": psm_aileron_power if psm_mode else aileron_power,
		"rudder": psm_rudder_power if psm_mode else rudder_power
	}

	# Pass high-speed damping parameters to aerodynamics
	var damping_params = {
		"ramp_end_speed": damping_ramp_end_speed,
		"pitch_strength": pitch_damping_strength,
		"alpha_strength": alpha_damping_strength,
		"roll_strength": roll_damping_strength,
		"yaw_strength": yaw_damping_strength,
		"max_multiplier": max_damping_multiplier
	}

	var aero_result = aerodynamics.calculate_forces_and_moments(
		velocity_body, angular_velocity_body,
		angle_of_attack, sideslip_angle, dynamic_pressure,
		controls, altitude_agl, control_surface_powers, damping_params
	)

	# Apply maneuverability multipliers
	var force = aero_result["force"]
	var moment = aero_result["moment"]

	# Wing loading multiplier: increases lift (more wing area = more lift)
	force.z *= wing_loading_multiplier  # Z is vertical in JSBSim (down is positive, so lift is negative)

	# Turn rate boost: increases control moments (faster rotation = sharper turns)
	moment *= instantaneous_turn_rate_boost

	# PSM aerodynamic reduction: reduce natural yaw/pitch coupling and other aerodynamic effects
	if psm_mode:
		force *= psm_aerodynamic_effect_multiplier
		moment *= psm_aerodynamic_effect_multiplier

		# PSM form drag: additional drag based on which surface faces velocity
		# This simulates increased drag when top/bottom/sides face the airflow
		# (e.g., cobra maneuver with bottom facing forward)
		var velocity_dir_body = velocity_body.normalized()

		# Calculate how much each surface faces the velocity
		# JSBSim frame: X=forward, Y=right, Z=down
		var forward_component = abs(velocity_dir_body.x)  # Forward/backward facing (nose/tail)
		var side_component = abs(velocity_dir_body.y)  # Side facing (wings)
		var vertical_component = abs(velocity_dir_body.z)  # Top/bottom facing

		# Form drag is strongest when NOT facing forward (forward_component close to 0)
		# and when top/bottom or sides are facing the flow
		var form_drag_factor = (side_component + vertical_component) * (1.0 - forward_component * 0.5)

		# Apply form drag opposite to velocity direction
		var form_drag = -velocity_dir_body * dynamic_pressure * psm_form_drag_multiplier * form_drag_factor * 5.0
		force += form_drag

	aero_result["force"] = force
	aero_result["moment"] = moment

	# Accumulate aerodynamic forces and moments
	total_force_body += aero_result["force"]
	total_moment_body += aero_result["moment"]

	# Airbrake drag (separate from PSM mode) - only applies extra drag
	var airbrake_active = controls.get("airbrake", false)
	if airbrake_active and velocity_body.length() > 1.0:
		# Airbrake creates drag opposite to velocity direction
		var base_airbrake_drag = -velocity_body.normalized() * dynamic_pressure * 3.0
		var airbrake_drag = base_airbrake_drag * airbrake_drag_multiplier
		total_force_body += airbrake_drag

func accumulate_propulsion_forces(processed_controls: Dictionary, raw_control_inputs: Dictionary, dt: float):
	# Combat thrust vectoring mode is already detected and stored in processed_controls["combat_tv_mode"]
	var combat_tv_mode = processed_controls.get("combat_tv_mode", false)

	# Merge processed controls with raw inputs for thrust vectoring
	# Thrust vectoring needs raw pitch/roll/yaw inputs, not processed elevator/aileron/rudder
	var control_inputs_for_propulsion = processed_controls.duplicate()

	# Add raw pitch/roll/yaw for thrust vectoring
	control_inputs_for_propulsion["pitch"] = raw_control_inputs.get("pitch", 0.0)
	control_inputs_for_propulsion["roll"] = raw_control_inputs.get("roll", 0.0)
	control_inputs_for_propulsion["yaw"] = raw_control_inputs.get("yaw", 0.0)

	# Pass combat TV mode flag to propulsion (CRITICAL - was missing!)
	control_inputs_for_propulsion["combat_tv_mode"] = combat_tv_mode

	# Add combat TV parameters and set them in propulsion model
	if combat_tv_mode:
		control_inputs_for_propulsion["combat_thrust_effectiveness"] = combat_thrust_effectiveness
		control_inputs_for_propulsion["combat_pitch_authority"] = combat_pitch_authority
		control_inputs_for_propulsion["combat_yaw_authority"] = combat_yaw_authority
		control_inputs_for_propulsion["combat_roll_to_yaw"] = combat_roll_to_yaw
		control_inputs_for_propulsion["combat_moment_multiplier"] = combat_moment_multiplier
		# Set the moment multiplier in propulsion model
		propulsion.combat_moment_multiplier = combat_moment_multiplier

	var thrust_result = propulsion.calculate_thrust_force_and_moment(
		control_inputs_for_propulsion, dt, airspeed, altitude_msl
	)

	total_force_body += thrust_result["force"]
	total_moment_body += thrust_result["moment"]

func detect_combat_thrust_vectoring(processed_controls: Dictionary, raw_inputs: Dictionary) -> bool:
	# Combat thrust vectoring activates when:
	# 1. Combat TV is enabled
	# 2. Afterburner is active
	# 3. Any control input is >80% deflected (pitch, roll, or yaw at >0.8 normalized)
	# 4. NOT in PSM mode (PSM has its own thrust vectoring)
	if not combat_tv_enabled:
		return false

	var psm_mode = processed_controls.get("psm_mode", false)
	if psm_mode:
		return false  # PSM mode has its own thrust vectoring, don't use combat TV

	var afterburner = processed_controls.get("afterburner", false)
	if not afterburner:
		return false

	# Check raw control inputs (these are normalized 0-1 values from FlightInputController)
	var pitch_input = abs(raw_inputs.get("pitch", 0.0))
	var roll_input = abs(raw_inputs.get("roll", 0.0))
	var yaw_input = abs(raw_inputs.get("yaw", 0.0))

	# Use absolute values and take max of all three inputs
	var max_input = max(pitch_input, roll_input, yaw_input)

	var is_active = max_input >= combat_control_threshold

	# Debug output every 30 frames
	if Engine.get_physics_frames() % 30 == 0:
		print("COMBAT TV: enabled=%s, afterburner=%s, pitch=%.2f, roll=%.2f, yaw=%.2f, max_input=%.2f, threshold=%.2f, ACTIVE=%s" % [
			combat_tv_enabled, afterburner, pitch_input, roll_input, yaw_input, max_input, combat_control_threshold, is_active
		])

	return is_active

func accumulate_landing_gear_forces(controls: Dictionary, dt: float):
	# Pass terrain generator to landing gear so each gear can query its own position
	var gear_result = landing_gear.calculate_gear_forces(
		global_position, linear_velocity, angular_velocity,
		global_basis, terrain, controls, dt
	)

	var gear_force_body = global_basis.inverse() * gear_result["force"]
	total_force_body += gear_force_body
	total_moment_body += gear_result["moment"]

	# Update debug visualizer
	if landing_gear.debug_landing_gear and gear_debug_visualizer:
		gear_debug_visualizer.set_visibility(true)
		gear_debug_visualizer.update_gear_visualization(
			landing_gear.gear_compression,
			landing_gear.max_compression
		)
	elif gear_debug_visualizer:
		gear_debug_visualizer.set_visibility(false)

	# Check if fuselage/body is hitting terrain (belly landing or mountain collision)
	var fuselage_bottom_y = global_position.y - 1.0  # Aircraft center minus approximate fuselage bottom

	# Use raycast to detect ground (terrain OR landing strips)
	var space_state = get_world_3d().direct_space_state
	var query = PhysicsRayQueryParameters3D.create(
		global_position + Vector3(0, 10, 0),  # Start 10m above center
		global_position - Vector3(0, 100, 0)  # Cast 100m down
	)
	query.collide_with_areas = false
	query.collide_with_bodies = true
	query.exclude = [self]  # Don't hit the aircraft itself

	var result = space_state.intersect_ray(query)
	var terrain_height_at_center = 0.0

	if result:
		# Hit something! Use the collision point
		terrain_height_at_center = result.position.y
	elif terrain:
		# No raycast hit, fall back to terrain height query
		terrain_height_at_center = terrain.get_terrain_height(global_position.x, global_position.z)

	if fuselage_bottom_y < terrain_height_at_center:
		# Fuselage is scraping ground or hitting mountain!
		var penetration = terrain_height_at_center - fuselage_bottom_y

		# Much weaker spring force with damping to prevent bounce
		var spring_force = penetration * 100000.0  # Reduced from 500000
		var damping_force = -linear_velocity.y * 30000.0  # Add damping based on vertical velocity
		var total_normal_force = spring_force + damping_force
		var fuselage_collision_force = Vector3(0, total_normal_force, 0)
		total_force_body += global_basis.inverse() * fuselage_collision_force

		# Apply heavy drag when scraping
		var horizontal_vel = Vector3(linear_velocity.x, 0, linear_velocity.z)
		var scrape_drag = -horizontal_vel * 10000.0  # Increased friction to slow down
		total_force_body += global_basis.inverse() * scrape_drag

		# Debug output - ALWAYS show when fuselage hits
		if landing_gear and landing_gear.debug_landing_gear:
			var hit_object_name = "Unknown"
			if result and result.collider:
				hit_object_name = result.collider.name
			print("\n[FUSELAGE COLLISION]")
			print("  Hit object: %s" % hit_object_name)
			print("  Fuselage bottom Y: %.2f m | Ground Y: %.2f m" % [fuselage_bottom_y, terrain_height_at_center])
			print("  Penetration: %.2f m" % penetration)
			print("  Spring force: %.0f N | Damping: %.0f N | Total: %.0f N" % [spring_force, damping_force, total_normal_force])
			print("  Vertical velocity: %.2f m/s" % linear_velocity.y)
			print("  Horizontal velocity: %.2f m/s" % horizontal_vel.length())

	# Apply position correction to prevent penetration
	var pos_correction = gear_result.get("position_correction", Vector3.ZERO)
	if pos_correction.length() > 0.001:
		if landing_gear.debug_landing_gear:
			print("FDM: Penetrating ground! Correction: %s, Velocity: %.2f m/s" % [pos_correction, linear_velocity.y])

		# Only apply minimal position correction - let spring forces handle the rest
		global_position += pos_correction * 0.1  # Only 10% correction

		# Clamp downward velocity to prevent extreme penetration
		if linear_velocity.y < -5.0:
			linear_velocity.y = -5.0  # Limit downward speed when penetrating

func get_aircraft_state() -> Dictionary:
	# Calculate bank angle (roll) relative to horizon
	var up_vector = global_basis.y  # Aircraft's up direction
	var world_up = Vector3.UP
	var bank_angle = acos(clamp(up_vector.dot(world_up), -1.0, 1.0))
	# Determine sign (left or right bank)
	var right_vector = global_basis.x
	if right_vector.dot(world_up) < 0:
		bank_angle = -bank_angle

	return {
		"position": global_position,
		"velocity": linear_velocity,
		"angular_velocity": angular_velocity_body,
		"altitude": altitude_msl,
		"airspeed": airspeed,
		"alpha": angle_of_attack,
		"beta": sideslip_angle,
		"mach": mach_number,
		"q": dynamic_pressure,
		"bank_angle": bank_angle,
		"on_ground": landing_gear.is_on_ground if landing_gear else false,
		"aoa_limiter_enabled": aoa_limiter_enabled,
		"max_aoa_limit": deg_to_rad(max_aoa_limit),
		"limiter_strength": limiter_strength,
		"stall_recovery_enabled": stall_recovery_enabled,
		"stall_warning_aoa": deg_to_rad(stall_warning_aoa),
		"full_stall_aoa": deg_to_rad(full_stall_aoa),
		"recovery_pitch_strength": recovery_pitch_strength,
		"recovery_roll_strength": recovery_roll_strength,
		"recovery_yaw_strength": recovery_yaw_strength,
		"wing_leveling_weight": wing_leveling_weight,
		"sideslip_correction_weight": sideslip_correction_weight,
		# PSM Attitude Assistance parameters
		"psm_attitude_assistance_enabled": psm_attitude_assistance_enabled,
		"psm_max_pitch_rate": psm_max_pitch_rate,
		"psm_max_roll_rate": psm_max_roll_rate,
		"psm_max_yaw_rate": psm_max_yaw_rate,
		"psm_aerodynamic_coupling_factor": psm_aerodynamic_coupling_factor,
		"psm_direct_control_mode": psm_direct_control_mode,
		"psm_rate_authority": psm_rate_authority
	}

func set_control_input(input_name: String, value: float):
	control_inputs[input_name] = value

func set_control_inputs(inputs: Dictionary):
	control_inputs = inputs

func reset_aircraft():
	# Reset position and orientation
	global_position = Vector3(0, altitude_msl, 0)
	global_rotation = Vector3.ZERO

	# Reset velocities
	var initial_speed = 200.0  # m/s
	linear_velocity = -global_basis.z * initial_speed
	angular_velocity = Vector3.ZERO

	# Reset body frame velocities
	velocity_body = Vector3(initial_speed, 0, 0)  # JSBSim: forward
	angular_velocity_body = Vector3.ZERO

	# Reset control inputs
	control_inputs["pitch"] = 0.0
	control_inputs["roll"] = 0.0
	control_inputs["yaw"] = 0.0
	control_inputs["throttle"] = 0.5

	print("Aircraft reset to initial state")
