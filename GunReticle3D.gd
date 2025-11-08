class_name GunReticle3D
extends MeshInstance3D

## 3D Gun Reticle - Circle in world space showing bullet impact point
## Positioned where bullets will hit, accounts for bullet drop and velocity

@export var aircraft: FDMCore
@export var gun: MinigunWeapon
@export var reticle_distance: float = 200.0  ## Distance ahead to show reticle (meters)
@export var reticle_size: float = 3.0  ## Reticle radius (meters)
@export var reticle_color: Color = Color(0, 1, 0, 0.8)  ## Green, semi-transparent

var circle_segments: int = 32

func _ready():
	create_reticle_mesh()

	# Find aircraft and gun if not set
	if not aircraft:
		aircraft = get_node_or_null("../../..") as FDMCore
	if not gun:
		gun = get_node_or_null("..")

func create_reticle_mesh():
	# Create circle mesh using ImmediateMesh
	var immediate_mesh = ImmediateMesh.new()
	mesh = immediate_mesh

	immediate_mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)

	# Draw circle
	for i in range(circle_segments + 1):
		var angle = (i / float(circle_segments)) * TAU
		var x = cos(angle) * reticle_size
		var y = sin(angle) * reticle_size
		immediate_mesh.surface_add_vertex(Vector3(x, y, 0))

	immediate_mesh.surface_end()

	# Add center dot
	immediate_mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	immediate_mesh.surface_add_vertex(Vector3(-0.5, 0, 0))
	immediate_mesh.surface_add_vertex(Vector3(0.5, 0, 0))
	immediate_mesh.surface_add_vertex(Vector3(0, -0.5, 0))
	immediate_mesh.surface_add_vertex(Vector3(0, 0.5, 0))
	immediate_mesh.surface_end()

	# Create unshaded glowing material
	var material = StandardMaterial3D.new()
	material.albedo_color = reticle_color
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material.no_depth_test = true  # Always visible through objects
	material.disable_receive_shadows = true
	material_override = material

func _process(_delta):
	if not aircraft or not gun:
		visible = false
		return

	visible = true

	# Calculate where bullets will hit
	var bullet_impact_point = calculate_bullet_impact()

	# Position reticle at impact point
	global_position = bullet_impact_point

	# Orient reticle to face camera
	var camera = get_viewport().get_camera_3d()
	if camera:
		look_at(camera.global_position, Vector3.UP)

	# Change color based on gun state
	var gun_state = gun.get_gun_state()
	var new_color = reticle_color

	if gun_state["is_firing"] and gun_state["spin_level"] > 0.2:
		new_color = Color(1, 0, 0, 0.8)  # Red when firing
	elif gun_state["spin_level"] > 0.0:
		new_color = Color(1, 1, 0, 0.8)  # Yellow when spinning up
	else:
		new_color = Color(0, 1, 0, 0.8)  # Green when idle

	if material_override:
		var mat = material_override as StandardMaterial3D
		if mat:
			mat.albedo_color = new_color

func calculate_bullet_impact() -> Vector3:
	## Calculate where bullets will hit accounting for drop and velocity

	if not aircraft or not gun:
		return global_position

	# Get gun muzzle position and direction
	var muzzle_pos = gun.global_position
	if gun.has_node("MuzzlePoint"):
		muzzle_pos = gun.get_node("MuzzlePoint").global_position

	var gun_forward = -gun.global_basis.z

	# Aircraft velocity
	var aircraft_velocity = aircraft.linear_velocity

	# Bullet velocity (muzzle velocity + aircraft velocity)
	var bullet_velocity = gun_forward * gun.muzzle_velocity + aircraft_velocity
	var bullet_speed = bullet_velocity.length()

	# Raycast to find actual hit point or use default distance
	var space_state = get_world_3d().direct_space_state
	var query = PhysicsRayQueryParameters3D.create(
		muzzle_pos,
		muzzle_pos + gun_forward * gun.max_range
	)
	query.collide_with_areas = false
	query.collide_with_bodies = true
	query.exclude = [aircraft]

	var result = space_state.intersect_ray(query)
	var target_distance = reticle_distance

	if result:
		target_distance = muzzle_pos.distance_to(result.position)

	# Time to reach target
	var time_to_target = target_distance / bullet_speed if bullet_speed > 0 else 0

	# Calculate bullet drop
	var drop = 0.5 * gun.bullet_drop_gravity * time_to_target * time_to_target

	# Impact point
	var impact_point = muzzle_pos + gun_forward * target_distance
	impact_point.y -= drop  # Apply drop

	return impact_point
