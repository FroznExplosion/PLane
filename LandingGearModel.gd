class_name LandingGearModel
extends Node

## Landing Gear Physics - JSBSim-Based Spring-Damper Model
## Three gear: nose, left main, right main
## Based on JSBSim FGLGear implementation with realistic spring/damper coefficients

# Gear configuration
@export var gear_extended: bool = true

# Gear positions relative to CG (body frame)
var gear_positions: Array[Vector3] = [
	Vector3(0, -1.5, -4.0),   # Nose gear (forward, below)
	Vector3(-1.5, -1.5, 1.5), # Left main
	Vector3(1.5, -1.5, 1.5)   # Right main
]

# Gear compression state
var gear_compression: Array[float] = [0.0, 0.0, 0.0]
var gear_compression_velocity: Array[float] = [0.0, 0.0, 0.0]  # For damping calculation
var max_compression: float = 0.35  # meters - realistic strut travel

# Spring-damper parameters (JSBSim-based)
# Formula: k = Weight_on_gear / max_compression
# For F-16 (14,000 kg total): Mains carry ~78%, Nose carries ~22%
# Main gear (each): 5,400 kg * 9.81 / 0.35 = ~151,000 N/m
# Nose gear: 3,100 kg * 9.81 / 0.35 = ~87,000 N/m

# Main gear spring/damper (left and right)
@export var main_spring_constant: float = 150000.0  # N/m - JSBSim formula based
@export var main_damping_compression: float = 30000.0  # N·s/m - 20% of spring (JSBSim standard)
@export var main_damping_rebound: float = 40000.0  # N·s/m - Slightly higher for rebound (JSBSim)

# Nose gear spring/damper
@export var nose_spring_constant: float = 87000.0  # N/m - JSBSim formula based
@export var nose_damping_compression: float = 26000.0  # N·s/m - 30% of spring (JSBSim standard)
@export var nose_damping_rebound: float = 35000.0  # N·s/m - Slightly higher for rebound

@export var max_load: float = 800000.0  # N per gear - Safety limit

# Friction parameters (JSBSim-based)
# These values are realistic for aircraft tires on concrete/asphalt
@export var static_friction_coeff: float = 0.8  # Static friction (stationary)
@export var dynamic_friction_coeff: float = 0.65  # Dynamic friction (sliding)
@export var rolling_friction_coeff: float = 0.02  # Rolling resistance
@export var brake_friction_multiplier: float = 3.5  # Multiplier when brakes applied (max μ = 2.8)

# Tire parameters
@export var tire_radius: float = 0.4  # meters - wheel radius
@export var tire_lateral_stiffness: float = 1.2  # Lateral (side) friction multiplier

# Ground contact state
var is_on_ground: bool = false
var num_gears_compressed: int = 0

# Debug
@export var debug_landing_gear: bool = false
var debug_frame_counter: int = 0

func calculate_gear_forces(aircraft_position: Vector3, aircraft_velocity: Vector3,
						   angular_velocity: Vector3, aircraft_basis: Basis,
						   terrain_generator: Node, control_inputs: Dictionary,
						   dt: float) -> Dictionary:
	## Calculate forces and moments from all landing gear
	## Returns: Dictionary with "force" (Vector3) and "moment" (Vector3)

	if not gear_extended:
		is_on_ground = false
		num_gears_compressed = 0
		return {"force": Vector3.ZERO, "moment": Vector3.ZERO}

	var total_force = Vector3.ZERO
	var total_moment = Vector3.ZERO
	num_gears_compressed = 0

	# Calculate forces for each gear
	for i in range(3):
		var result = calculate_single_gear_force(i, aircraft_position, aircraft_velocity,
												  angular_velocity, aircraft_basis,
												  terrain_generator, control_inputs, dt)
		total_force += result["force"]
		total_moment += result["moment"]

		if result["compressed"]:
			num_gears_compressed += 1

	is_on_ground = num_gears_compressed > 0

	# Debug output
	if debug_landing_gear:
		debug_frame_counter += 1
		if debug_frame_counter >= 60:  # Print every 60 frames (0.5 seconds at 120fps)
			debug_frame_counter = 0
			print("\n========== LANDING GEAR DEBUG (JSBSim Model) ==========")
			print("On Ground: %s | Gears compressed: %d/3" % [is_on_ground, num_gears_compressed])
			print("Total force: %s (magnitude: %.1f N)" % [total_force, total_force.length()])
			print("Aircraft velocity: %s (Y velocity: %.2f m/s)" % [aircraft_velocity, aircraft_velocity.y])
			print("Aircraft altitude: %.2f m" % aircraft_position.y)
			for i in range(3):
				if gear_compression[i] > 0.001:
					var gear_names = ["Nose", "Left Main", "Right Main"]
					print("  %s: compression=%.3fm (%.1f%%) | vel=%.2f m/s" % [
						gear_names[i],
						gear_compression[i],
						(gear_compression[i]/max_compression)*100,
						gear_compression_velocity[i]
					])
			print("=======================================================\n")

	return {
		"force": total_force,
		"moment": total_moment
	}

func calculate_single_gear_force(gear_index: int, aircraft_pos: Vector3,
								 aircraft_vel: Vector3, angular_vel: Vector3,
								 basis: Basis, terrain_generator: Node,
								 controls: Dictionary, dt: float) -> Dictionary:

	# Calculate gear world position
	var gear_pos_local = gear_positions[gear_index]
	var gear_pos_world = aircraft_pos + basis * gear_pos_local

	# Use raycast to detect ground (terrain OR landing strips)
	var terrain_height = 0.0
	var hit_surface = false

	# Get world from parent aircraft (RigidBody3D)
	var aircraft = get_parent()
	if aircraft and aircraft.has_method("get_world_3d"):
		var space_state = aircraft.get_world_3d().direct_space_state
		var query = PhysicsRayQueryParameters3D.create(
			gear_pos_world + Vector3(0, 10, 0),  # Start 10m above gear
			gear_pos_world - Vector3(0, 100, 0)  # Cast 100m down
		)
		query.collide_with_areas = false
		query.collide_with_bodies = true
		query.exclude = [aircraft]  # Don't hit the aircraft itself

		var result = space_state.intersect_ray(query)

		if result:
			# Hit something! Use the collision point
			terrain_height = result.position.y
			hit_surface = true

			# Debug: Show what we hit when contact occurs
			if debug_landing_gear and gear_pos_world.y - terrain_height - tire_radius <= 0:
				var gear_names = ["Nose", "Left Main", "Right Main"]
				var hit_object = result.collider
				var penetration = tire_radius - (gear_pos_world.y - terrain_height)
				print("[CONTACT] %s gear touching %s | Gear Y=%.2f | Ground Y=%.2f | Penetration=%.3fm" % [
					gear_names[gear_index],
					hit_object.name if hit_object else "Unknown",
					gear_pos_world.y,
					terrain_height,
					penetration
				])
		elif terrain_generator and terrain_generator.has_method("get_terrain_height"):
			# No raycast hit, fall back to terrain height query
			terrain_height = terrain_generator.get_terrain_height(gear_pos_world.x, gear_pos_world.z)
	elif terrain_generator and terrain_generator.has_method("get_terrain_height"):
		# No world access, fall back to terrain height query
		terrain_height = terrain_generator.get_terrain_height(gear_pos_world.x, gear_pos_world.z)

	# Calculate ground distance (gear bottom to terrain)
	var ground_distance = gear_pos_world.y - terrain_height - tire_radius

	if ground_distance > 0:
		# No contact - gear is above ground
		gear_compression[gear_index] = 0.0
		gear_compression_velocity[gear_index] = 0.0
		return {"force": Vector3.ZERO, "moment": Vector3.ZERO, "compressed": false}

	# === GROUND CONTACT - JSBSim Spring-Damper Model ===

	# Calculate compression (how much strut is compressed)
	# Clamp to max_compression to prevent over-compression
	var compression = clamp(-ground_distance, 0.0, max_compression)

	# Calculate compression velocity (rate of change)
	var prev_compression = gear_compression[gear_index]
	var compression_velocity = (compression - prev_compression) / dt if dt > 0 else 0.0

	# Store compression state
	gear_compression[gear_index] = compression
	gear_compression_velocity[gear_index] = compression_velocity

	# Calculate gear velocity (including rotation)
	var gear_velocity = aircraft_vel + angular_vel.cross(basis * gear_pos_local)
	var vertical_velocity = gear_velocity.y

	# === JSBSim Spring-Damper Force Calculation ===
	# Formula: F = -k*x - c*v
	# where: k = spring constant, x = compression, c = damping, v = compression velocity

	# Select spring/damping constants based on gear type
	var spring_k: float
	var damping_c: float

	if gear_index == 0:
		# Nose gear
		spring_k = nose_spring_constant
		damping_c = nose_damping_compression if compression_velocity >= 0 else nose_damping_rebound
	else:
		# Main gear (left and right)
		spring_k = main_spring_constant
		damping_c = main_damping_compression if compression_velocity >= 0 else main_damping_rebound

	# Calculate spring force (proportional to compression)
	var spring_force = spring_k * compression

	# Calculate damping force (proportional to compression velocity)
	# Negative sign because damping opposes motion
	var damping_force = damping_c * compression_velocity

	# Total normal force (upward, perpendicular to ground)
	var normal_force = spring_force + damping_force

	# Clamp to positive values and max load
	normal_force = clamp(normal_force, 0.0, max_load)

	# === Friction Force Calculation ===
	var friction_force = calculate_friction(gear_velocity, normal_force, gear_index, controls)

	# === Debug Output ===
	if debug_landing_gear and compression > 0.01:
		var gear_names = ["Nose", "Left Main", "Right Main"]
		print("[GEAR %s]" % gear_names[gear_index])
		print("  Position: World Y=%.2f | Ground Y=%.2f | Distance=%.3fm" % [gear_pos_world.y, terrain_height, ground_distance])
		print("  Compression: %.3fm / %.2fm (%.1f%%)" % [compression, max_compression, (compression/max_compression)*100])
		print("  Compression velocity: %.2f m/s (%s)" % [compression_velocity, "COMPRESSING" if compression_velocity > 0 else "REBOUNDING"])
		print("  Spring force: %.0f N (k=%.0f)" % [spring_force, spring_k])
		print("  Damping force: %.0f N (c=%.0f)" % [damping_force, damping_c])
		print("  Total normal force: %.0f N" % normal_force)
		print("  Friction force: %.0f N" % friction_force.length())
		print("  Vertical velocity: %.2f m/s" % vertical_velocity)

	# === Total Force and Moment ===
	# Total force in world frame (friction horizontal + normal vertical)
	var total_force = Vector3(friction_force.x, normal_force, friction_force.z)

	# Calculate moment in body frame
	# Moment = position × force
	var force_body = basis.inverse() * total_force
	var moment_body = gear_pos_local.cross(force_body)

	return {
		"force": total_force,
		"moment": moment_body,
		"compressed": true
	}

func calculate_friction(velocity: Vector3, normal_force: float,
						gear_index: int, controls: Dictionary) -> Vector3:
	## JSBSim-based friction model
	## Combines rolling resistance with braking friction

	# Get horizontal velocity (ground plane)
	var horizontal_velocity = Vector3(velocity.x, 0, velocity.z)
	var speed = horizontal_velocity.length()

	# No friction if no horizontal movement
	if speed < 0.001:
		return Vector3.ZERO

	# Get control inputs
	var braking = controls.get("brake", 0.0)  # 0.0 to 1.0
	var airbrake_active = controls.get("airbrake", false)

	# Airbrake acts as full brake when on ground
	if airbrake_active:
		braking = 1.0

	# === JSBSim Friction Model ===
	# Formula: F_friction = μ * F_normal
	# where μ depends on: rolling resistance, braking, and slip state

	var friction_coeff: float

	if gear_index == 0:
		# === NOSE GEAR ===
		# Nose gear typically doesn't have brakes (some aircraft do, but F-16 doesn't)
		# Only rolling resistance + some passive friction
		friction_coeff = rolling_friction_coeff + 0.15  # Base friction for nose

	else:
		# === MAIN GEAR (Left and Right) ===
		# Base rolling resistance
		var base_friction = rolling_friction_coeff

		# Add braking friction when brakes applied
		# JSBSim: F_brake = brake_input * static_friction * normal_force
		if braking > 0.01:
			# Interpolate between rolling and max brake friction
			var brake_friction = static_friction_coeff * brake_friction_multiplier
			friction_coeff = base_friction + braking * brake_friction
		else:
			# Just rolling resistance
			friction_coeff = base_friction + dynamic_friction_coeff * 0.1

	# === Speed-dependent friction transition ===
	# At very low speeds, reduce friction to prevent jitter and allow smooth stop
	if speed < 1.0:
		# Smoothly reduce friction as speed approaches zero
		var speed_factor = speed / 1.0
		friction_coeff *= speed_factor

	# Calculate friction force magnitude
	var max_friction_force = normal_force * friction_coeff

	# Friction opposes motion
	var friction_direction = -horizontal_velocity.normalized()

	# === Static vs Dynamic Friction ===
	# At low speeds with high friction, might enter static friction regime
	# This prevents sliding and allows the tire to "stick" to the ground

	var friction_force_magnitude: float

	if speed < 0.5 and braking > 0.5:
		# Strong braking at low speed - use static friction (can fully stop)
		# Apply force proportional to velocity to smoothly bring to rest
		friction_force_magnitude = min(speed * normal_force * static_friction_coeff * 2.0, max_friction_force)
	else:
		# Normal dynamic friction
		friction_force_magnitude = max_friction_force

	return friction_direction * friction_force_magnitude

func get_gear_state() -> Dictionary:
	return {
		"extended": gear_extended,
		"on_ground": is_on_ground,
		"num_compressed": num_gears_compressed,
		"compressions": gear_compression.duplicate()
	}
