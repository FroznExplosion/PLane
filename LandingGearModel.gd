class_name LandingGearModel
extends Node

## Landing Gear Physics - Spring-Damper Model
## Three gear: nose, left main, right main

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
var max_compression: float = 0.5  # meters

# Spring-damper parameters
@export var spring_constant: float = 300000.0  # N/m - Increased for stiffer response
@export var damping_constant: float = 200000.0  # N·s/m - Very high damping for minimal bounce
@export var max_load: float = 800000.0  # N per gear - Increased cap to allow hard stops
@export var rebound_damping_multiplier: float = 15.0  # Extra damping when gear extends (prevents bounce)

# Hard stop parameters (for runway/solid surface contact)
@export var hard_stop_enabled: bool = true  # Enable hard stop on solid surfaces
@export var hard_stop_stiffness: float = 1000000.0  # N/m - Very stiff when hitting hard surface
@export var hard_stop_threshold: float = 0.4  # Start hard stop at 80% compression (0.4/0.5)

# Friction parameters
@export var static_friction: float = 0.8
@export var dynamic_friction: float = 0.6
@export var rolling_friction: float = 0.02
@export var brake_friction: float = 1.2

# Tire parameters
@export var tire_radius: float = 0.4  # meters
@export var tire_spring: float = 500000.0  # N/m

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

	if not gear_extended:
		is_on_ground = false
		num_gears_compressed = 0
		return {"force": Vector3.ZERO, "moment": Vector3.ZERO}

	var total_force = Vector3.ZERO
	var total_moment = Vector3.ZERO
	var position_correction = Vector3.ZERO
	num_gears_compressed = 0

	for i in range(3):
		var result = calculate_single_gear_force(i, aircraft_position, aircraft_velocity,
												  angular_velocity, aircraft_basis,
												  terrain_generator, control_inputs, dt)
		total_force += result["force"]
		total_moment += result["moment"]
		position_correction += result.get("position_correction", Vector3.ZERO)

		if result["compressed"]:
			num_gears_compressed += 1

	is_on_ground = num_gears_compressed > 0

	# Debug output
	if debug_landing_gear:
		debug_frame_counter += 1
		if debug_frame_counter >= 30:  # Print every 30 frames (0.5 seconds at 60fps)
			debug_frame_counter = 0
			print("\n========== LANDING GEAR DEBUG ==========")
			print("On Ground: %s | Gears compressed: %d/3" % [is_on_ground, num_gears_compressed])
			print("Total force: %s (magnitude: %.1f N)" % [total_force, total_force.length()])
			print("Position correction: %s" % position_correction)
			print("Aircraft velocity: %s (Y velocity: %.2f m/s)" % [aircraft_velocity, aircraft_velocity.y])
			print("Aircraft altitude: %.2f m" % aircraft_position.y)
			for i in range(3):
				if gear_compression[i] > 0.01:
					var gear_names = ["Nose", "Left Main", "Right Main"]
					print("  %s: compression=%.3fm" % [gear_names[i], gear_compression[i]])
			print("========================================\n")

	return {
		"force": total_force,
		"moment": total_moment,
		"position_correction": position_correction
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

			# Debug: Show what we hit EVERY TIME for contact tracking
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

	# Check ground contact
	var ground_distance = gear_pos_world.y - terrain_height - tire_radius

	if ground_distance > 0:
		# No contact
		gear_compression[gear_index] = 0.0
		return {"force": Vector3.ZERO, "moment": Vector3.ZERO, "compressed": false}

	# Calculate compression
	var compression = min(-ground_distance, max_compression)
	var compression_rate = (compression - gear_compression[gear_index]) / dt if dt > 0 else 0.0
	gear_compression[gear_index] = compression

	# Calculate gear velocity (including rotation) - BEFORE we use it!
	var gear_velocity = aircraft_vel + angular_vel.cross(basis * gear_pos_local)

	# Spring-damper force (normal force)
	var spring_force = spring_constant * compression
	var damping_force = damping_constant * compression_rate

	# Apply asymmetric damping - MUCH stronger when extending (rebounding)
	if compression_rate < 0:  # Gear is extending (rebounding)
		damping_force *= rebound_damping_multiplier  # 15x damping on rebound!

	var normal_force = spring_force + damping_force

	# Add extra damping on initial touchdown (absorb landing energy)
	var vertical_velocity = gear_velocity.y
	if vertical_velocity < -1.0:  # Moving downward faster than 1 m/s
		# Add touchdown damping force proportional to vertical velocity
		var touchdown_damping = abs(vertical_velocity) * damping_constant * 0.3
		normal_force += touchdown_damping

	# HARD STOP SYSTEM - Prevents sinking through runways
	# Progressive stiffening as compression increases
	var hard_stop_active = false
	var hard_stop_force = 0.0

	if hard_stop_enabled and compression > hard_stop_threshold:
		# Gear is significantly compressed - engage hard stop
		var excess_compression = compression - hard_stop_threshold
		hard_stop_force = excess_compression * hard_stop_stiffness
		# Add velocity-based damping to hard stop
		if vertical_velocity < 0:  # Moving downward
			hard_stop_force += abs(vertical_velocity) * hard_stop_stiffness * 0.5
		normal_force += hard_stop_force
		hard_stop_active = true

	# Emergency stop: If penetrating beyond max compression (should rarely happen now)
	if ground_distance < -max_compression:
		# Penetrating beyond max compression - apply VERY strong resistance
		var penetration_depth = abs(ground_distance + max_compression)
		var emergency_force = penetration_depth * hard_stop_stiffness * 2.0
		normal_force = max(normal_force, emergency_force)
		hard_stop_active = true
		if debug_landing_gear:
			print("  !!! EMERGENCY STOP - Penetration beyond max: %.3fm !!!" % penetration_depth)

	normal_force = clamp(normal_force, 0.0, max_load)  # Cap to max load

	# Calculate friction force
	var friction_force = calculate_friction(gear_velocity, normal_force, gear_index, controls)

	# Debug friction and forces - DETAILED OUTPUT EVERY FRAME when debug enabled
	if debug_landing_gear and compression > 0.01:
		var gear_names = ["Nose", "Left Main", "Right Main"]
		var spring_f = spring_constant * compression
		var damp_f = damping_constant * compression_rate
		var touch_damp_f = abs(vertical_velocity) * damping_constant * 0.3 if vertical_velocity < -1.0 else 0.0

		print("[FORCE] %s gear:" % gear_names[gear_index])
		print("  Compression: %.3fm (rate: %.2f m/s) | Ground dist: %.3fm | Max: %.2fm" % [compression, compression_rate, ground_distance, max_compression])
		print("  Spring force: %.0f N" % spring_f)
		print("  Damping force: %.0f N (rebound multiplier: %s)" % [damp_f, "ACTIVE x%.0f" % rebound_damping_multiplier if compression_rate < 0 else "OFF"])
		print("  Touchdown damping: %.0f N" % touch_damp_f)
		if hard_stop_active:
			print("  >>> HARD STOP ACTIVE <<< Force: %.0f N | Compression over threshold: %.3fm" % [hard_stop_force, compression - hard_stop_threshold])
		print("  Total normal force: %.0f N (capped at %.0f N)" % [normal_force, max_load])
		print("  Friction force: %.0f N" % friction_force.length())
		print("  Gear velocity: (%.1f, %.1f, %.1f) m/s | Vertical: %.2f m/s" % [
			gear_velocity.x, gear_velocity.y, gear_velocity.z, vertical_velocity
		])

	# Total force in world frame
	var total_force = Vector3(friction_force.x, normal_force, friction_force.z)

	# Moment = position × force (in body frame)
	var force_body = basis.inverse() * total_force
	var moment_body = gear_pos_local.cross(force_body)

	# Position correction: If penetrating beyond max compression, push aircraft up
	var pos_correction = Vector3.ZERO
	if ground_distance < -max_compression:
		var penetration = abs(ground_distance + max_compression)
		# Push aircraft upward to prevent pass-through
		pos_correction = Vector3(0, penetration, 0)

		if debug_landing_gear:
			var gear_names = ["Nose", "Left Main", "Right Main"]
			print("WARNING: %s gear penetrating! Depth: %.3fm, Normal Force: %.1f N" % [gear_names[gear_index], penetration, normal_force])

	return {
		"force": total_force,
		"moment": moment_body,
		"compressed": true,
		"position_correction": pos_correction
	}

func calculate_friction(velocity: Vector3, normal_force: float,
						gear_index: int, controls: Dictionary) -> Vector3:

	var horizontal_velocity = Vector3(velocity.x, 0, velocity.z)
	var speed = horizontal_velocity.length()

	if speed < 0.01:
		return Vector3.ZERO

	# Determine friction coefficient
	var friction_coef: float
	var braking = controls.get("brake", 0.0)
	var airbrake_active = controls.get("airbrake", false)

	# Airbrake also acts as brake when on ground
	if airbrake_active:
		braking = max(braking, 1.0)  # Full braking when airbrake deployed

	if gear_index == 0:
		# Nose gear has good friction
		friction_coef = 0.5  # Increased from 0.4
	else:
		# Main gears - use high base friction + braking
		var base_friction = 0.4  # Increased from 0.3
		friction_coef = base_friction + braking * brake_friction

	# Calculate friction force
	var friction_direction = -horizontal_velocity.normalized()
	var max_friction = normal_force * friction_coef

	# Use dynamic friction model that works at all speeds
	var friction_magnitude: float
	if speed < 2.0:
		# At low speeds, use static friction (proportional to velocity to avoid jitter)
		friction_magnitude = min(speed * normal_force * static_friction * 0.5, max_friction)
	else:
		# At higher speeds, use full friction
		friction_magnitude = max_friction

	return friction_direction * friction_magnitude

func get_gear_state() -> Dictionary:
	return {
		"extended": gear_extended,
		"on_ground": is_on_ground,
		"num_compressed": num_gears_compressed,
		"compressions": gear_compression.duplicate()
	}
