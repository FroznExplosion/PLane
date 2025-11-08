class_name MinigunWeapon
extends Node3D

## Minigun Weapon System
## Features: Spin-up mechanic, ramping fire rate, predictive aiming

signal gun_fired(hit_position: Vector3, hit_normal: Vector3, hit_object: Node)

# Gun parameters
@export_group("Weapon Stats")
@export var damage: float = 25.0  ## Damage per bullet
@export var max_fire_rate: float = 100.0  ## Maximum rounds per minute (6000 RPM = 100 RPS)
@export var spin_up_time: float = 0.5  ## Time to reach max fire rate (seconds)
@export var spin_down_time: float = 0.3  ## Time to spin down when released (seconds)
@export var max_range: float = 2000.0  ## Maximum bullet range (meters)
@export var muzzle_velocity: float = 1000.0  ## Bullet velocity (m/s)

@export_group("Ammo")
@export var ammo_capacity: int = 2000  ## Total ammo
@export var infinite_ammo: bool = true  ## Infinite ammo (for testing)

# Barrel configuration
@export_group("Visual/Audio")
@export var num_barrels: int = 6  ## Number of rotating barrels
@export var barrel_rotation_speed: float = 360.0  ## Degrees per second at full spin
@export var bullet_tracer_speed: float = 600.0  ## Visible bullet speed (m/s)
@export var bullet_tracer_lifetime: float = 3.0  ## How long bullets are visible (seconds)

# Internal state
var current_ammo: int = 2000
var is_firing: bool = false
var spin_level: float = 0.0  # 0.0 = stopped, 1.0 = full speed
var time_since_last_shot: float = 0.0
var barrel_rotation: float = 0.0  # Current barrel rotation angle

# Bullet trajectory
var bullet_drop_gravity: float = 9.81  # m/s² downward

# References
var aircraft: FDMCore
var muzzle_point: Node3D  # Position where bullets spawn

func _ready():
	aircraft = get_node("../../") as FDMCore  # Assuming gun is child of visual -> aircraft
	if not aircraft:
		push_warning("MinigunWeapon: Could not find parent FDMCore")

	# Create muzzle point if it doesn't exist
	if not has_node("MuzzlePoint"):
		muzzle_point = Node3D.new()
		muzzle_point.name = "MuzzlePoint"
		muzzle_point.position = Vector3(0, 0, -2.0)  # 2m forward
		add_child(muzzle_point)
	else:
		muzzle_point = get_node("MuzzlePoint")

	current_ammo = ammo_capacity

func _process(delta):
	# Handle firing input (left mouse button only)
	var fire_button_held = Input.is_mouse_button_pressed(MOUSE_BUTTON_LEFT)

	# Update spin level
	if fire_button_held and (infinite_ammo or current_ammo > 0):
		# Spin up
		is_firing = true
		spin_level = min(1.0, spin_level + delta / spin_up_time)
	else:
		# Spin down
		is_firing = false
		spin_level = max(0.0, spin_level - delta / spin_down_time)

	# Rotate barrels based on spin level
	if spin_level > 0.0:
		barrel_rotation += barrel_rotation_speed * spin_level * delta
		if barrel_rotation >= 360.0:
			barrel_rotation -= 360.0

	# Fire bullets if spun up enough
	if is_firing and spin_level > 0.2:  # Start firing at 20% spin
		var current_fire_rate = max_fire_rate * spin_level  # Ramping fire rate
		var time_between_shots = 1.0 / current_fire_rate
		time_since_last_shot += delta

		while time_since_last_shot >= time_between_shots:
			fire_bullet()
			time_since_last_shot -= time_between_shots
	else:
		time_since_last_shot = 0.0

func fire_bullet():
	if not infinite_ammo:
		if current_ammo <= 0:
			return
		current_ammo -= 1

	if not aircraft or not muzzle_point:
		return

	# Get muzzle world position and direction
	var muzzle_world_pos = muzzle_point.global_position
	var muzzle_world_dir = -muzzle_point.global_basis.z  # Forward direction

	# Add aircraft velocity to bullet velocity (relative velocity)
	var aircraft_velocity = aircraft.linear_velocity
	var bullet_velocity_vec = muzzle_world_dir * muzzle_velocity + aircraft_velocity

	# Spawn visible bullet tracer (no aircraft velocity - can shoot yourself!)
	var tracer_velocity_vec = muzzle_world_dir * bullet_tracer_speed
	spawn_bullet_tracer(muzzle_world_pos, tracer_velocity_vec)

	# Raycast to detect hit
	var space_state = get_world_3d().direct_space_state
	var query = PhysicsRayQueryParameters3D.create(
		muzzle_world_pos,
		muzzle_world_pos + bullet_velocity_vec.normalized() * max_range
	)
	query.collide_with_areas = false
	query.collide_with_bodies = true
	query.exclude = [aircraft]  # Don't hit own aircraft

	var result = space_state.intersect_ray(query)

	if result:
		# Hit something
		gun_fired.emit(result.position, result.normal, result.collider)

		# Apply damage if target has health
		if result.collider.has_method("take_damage"):
			result.collider.take_damage(damage)

		# Visual feedback (spawn impact effect)
		create_impact_effect(result.position)
	else:
		# Miss - no hit within range
		pass

func spawn_bullet_tracer(start_pos: Vector3, velocity: Vector3):
	# Create bullet tracer instance using BulletTracer class
	var tracer = BulletTracer.new()

	# Set tracer properties BEFORE adding to tree
	tracer.velocity = velocity
	tracer.max_distance = max_range
	tracer.lifetime = bullet_tracer_lifetime

	# Add to scene first
	get_tree().root.add_child(tracer)

	# Now set position after it's in the tree
	tracer.global_position = start_pos

func create_impact_effect(impact_pos: Vector3):
	# TODO: Create visual impact effect (particles, decal, etc.)
	# For now, just a debug print
	if Engine.get_physics_frames() % 30 == 0:  # Print every 30 frames
		print("Hit at: %s" % impact_pos)

func get_aiming_solution(target_distance: float) -> Vector3:
	## Calculate where to aim to hit a target at given distance
	## Returns offset in local space (pitch/yaw adjustments)

	if not aircraft:
		return Vector3.ZERO

	# Get aircraft velocity
	var aircraft_vel = aircraft.linear_velocity
	var aircraft_speed = aircraft_vel.length()

	# Time to target (assuming bullet travels at muzzle_velocity + aircraft_speed)
	var effective_bullet_speed = muzzle_velocity + aircraft_speed
	var time_to_target = target_distance / effective_bullet_speed

	# Calculate bullet drop due to gravity
	var drop = 0.5 * bullet_drop_gravity * time_to_target * time_to_target

	# Calculate lead (target moving relative to us)
	# This is simplified - assumes target moving perpendicular
	# For more accuracy, would need target velocity

	# Return pitch adjustment needed to compensate for drop
	var pitch_adjustment = atan2(drop, target_distance)

	return Vector3(0, pitch_adjustment, 0)  # Pitch up to compensate

func get_gun_state() -> Dictionary:
	return {
		"ammo": current_ammo,
		"max_ammo": ammo_capacity,
		"spin_level": spin_level,
		"is_firing": is_firing,
		"fire_rate": max_fire_rate * spin_level
	}
