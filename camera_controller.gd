extends Camera3D
class_name FlightCamera

## Multi-view camera system for aircraft
## Supports third-person chase, cockpit, and other views

enum CameraMode {
	CHASE,          # Third-person chase camera
	COCKPIT         # First-person cockpit view
}

@export var target: FDMCore  # The aircraft to follow
@export var current_mode: CameraMode = CameraMode.CHASE

## Chase camera settings
@export_group("Chase Camera")
@export var chase_distance: float = 15.0  # meters behind aircraft
@export var chase_height: float = 5.0  # meters above aircraft
@export var chase_smoothing: float = 12.0  # how smooth the camera follows (higher = less lag)
@export var look_ahead: float = 10.0  # how far ahead to look
@export var loose_rotation_influence: float = 0.3  # how much plane rotation affects camera (0.0-1.0)

## Free look settings (right-click in any mode)
@export_group("Free Look")
@export var free_look_sensitivity: float = 0.3  # mouse look sensitivity
@export var free_look_pitch_limit: float = 85.0  # max pitch angle (degrees)
@export var free_look_auto_center_speed: float = 2.0  # how fast camera returns to center when released

## Cockpit camera settings
@export_group("Cockpit Camera")
@export var cockpit_position: Vector3 = Vector3(0, 0.5, -1.5)  # relative to aircraft
@export var cockpit_fov: float = 90.0

# Free look state (right-click)
var _free_look_yaw: float = 0.0  # horizontal rotation
var _free_look_pitch: float = 0.0  # vertical rotation
var _free_look_active: bool = false

func _ready() -> void:
	if not target:
		# Try to find aircraft in scene
		target = get_tree().get_first_node_in_group("aircraft")
		if not target:
			push_warning("FlightCamera: No aircraft target assigned!")
			return

func _input(event: InputEvent) -> void:
	# Handle right-click free look in any camera mode
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_RIGHT:
			_free_look_active = event.pressed

	# Handle mouse motion during free look
	if event is InputEventMouseMotion and _free_look_active:
		_free_look_yaw -= event.relative.x * free_look_sensitivity * 0.01
		_free_look_pitch -= event.relative.y * free_look_sensitivity * 0.01
		_free_look_pitch = clamp(_free_look_pitch, -deg_to_rad(free_look_pitch_limit), deg_to_rad(free_look_pitch_limit))

func _process(delta: float) -> void:
	if not target:
		return

	# Handle camera mode switching
	if Input.is_action_just_pressed("camera_change"):
		cycle_camera_mode()

	# Auto-center free look when not active
	if not _free_look_active:
		_free_look_yaw = lerp(_free_look_yaw, 0.0, free_look_auto_center_speed * delta)
		_free_look_pitch = lerp(_free_look_pitch, 0.0, free_look_auto_center_speed * delta)

	# Update camera based on current mode
	match current_mode:
		CameraMode.CHASE:
			update_chase_camera(delta)
		CameraMode.COCKPIT:
			update_cockpit_camera(delta)

func cycle_camera_mode() -> void:
	var next_mode_int := (current_mode + 1) % CameraMode.size()
	current_mode = next_mode_int as CameraMode
	print("Camera mode: ", CameraMode.keys()[current_mode])

func update_chase_camera(_delta: float) -> void:
	## Third-person chase camera with orbiting free look and loose plane-relative rotation
	## - Rigid follow with fixed distance (no jitter)
	## - Free look orbits independently (not affected by plane orientation)
	## - Normal mode: Camera loosely rotates with plane

	var aircraft_position: Vector3 = target.global_position

	# Determine camera position: either fixed behind or orbiting with free look
	if _free_look_active or abs(_free_look_yaw) > 0.01 or abs(_free_look_pitch) > 0.01:
		# FREE LOOK ORBITING: Camera orbits in world space (independent of plane orientation)
		# Offset is calculated from world center, not aircraft local space

		# Default back position in world space
		var base_offset: Vector3 = Vector3(0, 0, chase_distance) + Vector3(0, chase_height, 0)

		# Apply orbit rotations in world space (not affected by plane orientation)
		# Yaw rotation (horizontal orbit around world up axis)
		var yaw_quat: Quaternion = Quaternion(Vector3.UP, _free_look_yaw)
		# Pitch rotation (vertical orbit around world right axis)
		var pitch_quat: Quaternion = Quaternion(Vector3.RIGHT, _free_look_pitch)

		# Combine rotations and apply to offset
		var orbit_offset: Vector3 = (pitch_quat * yaw_quat) * base_offset
		global_position = aircraft_position + orbit_offset

		# Always look at aircraft
		look_at(aircraft_position, Vector3.UP)
	else:
		# NORMAL MODE: Follow plane's rear in all 6DOF with smooth rotation only
		# Camera position is FIXED relative to aircraft (no position smoothing to avoid jitter)

		# Desired position: behind and above aircraft in aircraft's local space
		var target_pos: Vector3 = aircraft_position
		target_pos += target.global_transform.basis.z * chase_distance  # behind
		target_pos += target.global_transform.basis.y * chase_height  # above

		# FIXED position follow (NO smoothing to maintain rigid distance)
		global_position = target_pos

		# Desired rotation: match plane's rotation exactly
		var target_basis: Basis = target.global_transform.basis

		# Direct rotation follow (no smoothing - camera rotates with plane)
		global_transform.basis = target_basis

		# Always look at aircraft
		look_at(aircraft_position, Vector3.UP)

func update_cockpit_camera(_delta: float) -> void:
	## First-person cockpit view with mouse look control
	## In first-person, free look always applies when right-click is held
	## Provides natural head-look camera control

	# Position camera inside the cockpit
	global_position = target.global_transform * cockpit_position

	# In first-person cockpit, free look IS the head look (mouse rotation)
	# Apply yaw and pitch rotations to look around independently
	var yaw_rotation: Basis = Basis(Vector3.UP, _free_look_yaw)
	var pitch_rotation: Basis = Basis(Vector3.RIGHT, _free_look_pitch)
	var look_rotation: Basis = yaw_rotation * pitch_rotation

	# Apply rotations on top of aircraft orientation
	global_transform.basis = target.global_transform.basis * look_rotation

	# Set cockpit FOV
	fov = cockpit_fov
