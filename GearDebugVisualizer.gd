class_name GearDebugVisualizer
extends Node3D

## Visual debug overlay for landing gear contact points
## Shows spheres at gear positions, colored by contact state

var gear_markers: Array[MeshInstance3D] = []
var gear_names = ["Nose", "Left Main", "Right Main"]

# Gear positions relative to aircraft (should match LandingGearModel)
var gear_positions: Array[Vector3] = [
	Vector3(0, -1.5, -4.0),   # Nose gear (forward, below)
	Vector3(-1.5, -1.5, 1.5), # Left main
	Vector3(1.5, -1.5, 1.5)   # Right main
]

# Materials for different states
var material_no_contact: StandardMaterial3D
var material_contact: StandardMaterial3D
var material_compressed: StandardMaterial3D

func _ready():
	# Create materials
	material_no_contact = StandardMaterial3D.new()
	material_no_contact.albedo_color = Color.GRAY
	material_no_contact.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material_no_contact.albedo_color.a = 0.5

	material_contact = StandardMaterial3D.new()
	material_contact.albedo_color = Color.YELLOW
	material_contact.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED

	material_compressed = StandardMaterial3D.new()
	material_compressed.albedo_color = Color.RED
	material_compressed.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED

	# Create gear markers
	for i in range(3):
		var marker = MeshInstance3D.new()
		var sphere = SphereMesh.new()
		sphere.radius = 0.3
		sphere.height = 0.6
		marker.mesh = sphere
		marker.material_override = material_no_contact
		add_child(marker)
		gear_markers.append(marker)

func update_gear_visualization(gear_compressions: Array[float], max_compression: float):
	## Update visual markers based on gear compression state
	for i in range(min(3, gear_compressions.size())):
		var compression = gear_compressions[i]
		var marker = gear_markers[i]

		# Position marker at gear location
		marker.position = gear_positions[i]

		# Color based on compression state
		if compression < 0.01:
			# No contact
			marker.material_override = material_no_contact
			marker.scale = Vector3.ONE * 0.5
		elif compression < max_compression * 0.5:
			# Light contact
			marker.material_override = material_contact
			marker.scale = Vector3.ONE * (0.5 + compression * 2.0)
		else:
			# Heavy compression
			marker.material_override = material_compressed
			marker.scale = Vector3.ONE * (1.0 + compression * 3.0)

func set_visibility(visible: bool):
	## Toggle visibility of all markers
	for marker in gear_markers:
		marker.visible = visible
