extends MeshInstance3D

## Velocity Vector Indicator - Prograde Arrow
## Shows direction of actual travel (velocity vector)

var aircraft: FDMCore

@export var arrow_distance: float = 10.0  ## Distance ahead of aircraft to place arrow (meters)
@export var arrow_scale: float = 0.5  ## Size of the arrow
@export var show_indicator: bool = true  ## Toggle visibility

func _ready() -> void:
	# Find the aircraft
	aircraft = get_tree().get_first_node_in_group("aircraft")
	if not aircraft:
		push_warning("VelocityVectorIndicator: No aircraft found!")
		return

	# Create arrow mesh
	create_arrow_mesh()

func create_arrow_mesh():
	# Create a simple arrow pointing forward (using cone + cylinder)
	var arrow_material = StandardMaterial3D.new()
	arrow_material.albedo_color = Color(0.0, 1.0, 0.0, 0.8)  # Green with transparency
	arrow_material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	arrow_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	arrow_material.disable_receive_shadows = true
	arrow_material.no_depth_test = true  # Always visible through objects

	# Create cone for arrow head
	var cone = CylinderMesh.new()
	cone.top_radius = 0.0
	cone.bottom_radius = 0.3
	cone.height = 0.8

	mesh = cone
	material_override = arrow_material

	# Scale it
	scale = Vector3(arrow_scale, arrow_scale, arrow_scale)

func _process(_delta: float) -> void:
	if not aircraft or not show_indicator:
		visible = false
		return

	visible = true

	# Get velocity in world space
	var velocity = aircraft.linear_velocity

	# Only show if moving
	if velocity.length() < 1.0:
		visible = false
		return

	# Position arrow ahead of aircraft in direction of velocity
	var velocity_direction = velocity.normalized()
	global_position = aircraft.global_position + velocity_direction * arrow_distance

	# Point arrow in direction of velocity
	# Arrow mesh points up (Y-axis), so align Y-axis with velocity
	var up_dir = velocity_direction
	var right_dir = up_dir.cross(Vector3.UP)
	if right_dir.length() < 0.1:  # Handle edge case when velocity is straight up/down
		right_dir = up_dir.cross(Vector3.FORWARD)
	right_dir = right_dir.normalized()
	var forward_dir = right_dir.cross(up_dir)

	# Create basis from directions
	basis = Basis(right_dir, up_dir, forward_dir)
