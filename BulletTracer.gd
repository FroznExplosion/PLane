class_name BulletTracer
extends MeshInstance3D

## Visible bullet tracer that travels through the air
## Shows bullet path with glowing trail effect

var velocity: Vector3 = Vector3.ZERO
var lifetime: float = 3.0  # seconds before auto-delete
var current_life: float = 0.0
var start_position: Vector3 = Vector3.ZERO
var max_distance: float = 2000.0
var distance_traveled: float = 0.0

# Visual parameters
var tracer_length: float = 5.0  # meters
var tracer_thickness: float = 0.05  # meters
var tracer_color: Color = Color(1.0, 0.8, 0.2, 1.0)  # Bright yellow-orange

func _ready():
	# Create tracer mesh
	var cylinder = CylinderMesh.new()
	cylinder.top_radius = tracer_thickness
	cylinder.bottom_radius = tracer_thickness
	cylinder.height = tracer_length
	mesh = cylinder

	# Create glowing material
	var material = StandardMaterial3D.new()
	material.albedo_color = tracer_color
	material.emission_enabled = true
	material.emission = tracer_color
	material.emission_energy_multiplier = 2.0
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material_override = material

	# Orient along velocity direction
	if velocity.length() > 0.0:
		look_at(global_position + velocity.normalized(), Vector3.UP)
		rotate_object_local(Vector3.RIGHT, PI / 2)  # Align with forward

func _process(delta):
	current_life += delta

	# Move bullet
	var movement = velocity * delta
	global_position += movement
	distance_traveled += movement.length()

	# Fade out over time
	if material_override:
		var fade = 1.0 - (current_life / lifetime)
		var mat = material_override as StandardMaterial3D
		if mat:
			mat.albedo_color.a = fade

	# Destroy if too old or traveled too far
	if current_life >= lifetime or distance_traveled >= max_distance:
		queue_free()

func hit_target():
	# Called when bullet hits something - destroy immediately
	queue_free()
