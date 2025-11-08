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
@export var chase_position_smoothing: float = 12.0  # how smooth position follows (higher = less lag)
@export var chase_rotation_smoothing: float = 3.0  # how smooth rotation follows (lower = looser, higher = tighter)
@export var look_ahead: float = 10.0  # how far ahead to look (currently unused)

## Free look settings (right-click in any mode)
@export_group("Free Look")
@export var free_look_sensitivity: float = 0.3  # mouse look sensitivity
@export var free_look_pitch_limit: float = 85.0  # max pitch angle (degrees)
@export var free_look_smooth_return_speed: float = 5.0  # how fast camera returns when released (quaternion slerp)

## Cockpit camera settings
@export_group("Cockpit Camera")
@export var cockpit_position: Vector3 = Vector3(0, 0.5, -1.5)  # relative to aircraft
@export var cockpit_fov: float = 90.0

# Free look state (right-click)
var _free_look_yaw: float = 0.0  # horizontal rotation
var _free_look_pitch: float = 0.0  # vertical rotation
var _free_look_active: bool = false

# Smooth rotation tracking for chase camera
var _chase_rotation_quat: Quaternion = Quaternion.IDENTITY  # Current camera rotation
var _previous_mode: CameraMode = CameraMode.CHASE  # Track mode changes

func _ready() -> void:
	if not target:
		# Try to find aircraft in scene
		target = get_tree().get_first_node_in_group("aircraft")
		if not target:
			push_warning("FlightCamera: No aircraft target assigned!")
			return

	# Initialize chase rotation to current aircraft rotation
	if target:
		_chase_rotation_quat = target.global_transform.basis.get_rotation_quaternion()

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

	# Detect mode changes and reset free look smoothly
	if current_mode != _previous_mode:
		# Mode changed - reset free look angles instantly (no unwinding)
		_free_look_yaw = 0.0
		_free_look_pitch = 0.0
		_previous_mode = current_mode

		# Reset chase rotation to match aircraft
		if current_mode == CameraMode.CHASE:
			_chase_rotation_quat = target.global_transform.basis.get_rotation_quaternion()

	# Auto-center free look when not active (smooth quaternion return)
	if not _free_look_active and (abs(_free_look_yaw) > 0.01 or abs(_free_look_pitch) > 0.01):
		# Smooth return to center using lerp
		_free_look_yaw = lerp(_free_look_yaw, 0.0, free_look_smooth_return_speed * delta)
		_free_look_pitch = lerp(_free_look_pitch, 0.0, free_look_smooth_return_speed * delta)

		# Snap to zero when very close to avoid endless tiny movements
		if abs(_free_look_yaw) < 0.001:
			_free_look_yaw = 0.0
		if abs(_free_look_pitch) < 0.001:
			_free_look_pitch = 0.0

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

func update_chase_camera(delta: float) -> void:
	## Third-person chase camera with loose rotation following and free look
	## - Position follows aircraft rigidly (no jitter)
	## - Rotation smoothly follows aircraft using quaternion slerp
	## - Free look adds offset rotation when active

	var aircraft_position: Vector3 = target.global_position
	var aircraft_basis: Basis = target.global_transform.basis

	# Determine camera position: either fixed behind or orbiting with free look
	if _free_look_active or abs(_free_look_yaw) > 0.01 or abs(_free_look_pitch) > 0.01:
		# === FREE LOOK MODE ===
		# Camera orbits in world space, independent of plane orientation

		# Base offset in world space (behind and above)
		var base_offset: Vector3 = Vector3(0, 0, chase_distance) + Vector3(0, chase_height, 0)

		# Apply orbit rotations in world space
		var yaw_quat: Quaternion = Quaternion(Vector3.UP, _free_look_yaw)
		var pitch_quat: Quaternion = Quaternion(Vector3.RIGHT, _free_look_pitch)

		# Combine rotations and apply to offset
		var orbit_offset: Vector3 = (pitch_quat * yaw_quat) * base_offset
		global_position = aircraft_position + orbit_offset

		# Look at aircraft
		look_at(aircraft_position, Vector3.UP)
	else:
		# === NORMAL CHASE MODE ===
		# Loosely follow plane's rotation using quaternion slerp

		# Get target rotation (aircraft's current rotation)
		var target_rotation_quat: Quaternion = aircraft_basis.get_rotation_quaternion()

		# Smoothly interpolate camera rotation toward aircraft rotation
		# Lower chase_rotation_smoothing = looser follow (more lag)
		# Higher chase_rotation_smoothing = tighter follow (less lag)
		var slerp_weight: float = clamp(chase_rotation_smoothing * delta, 0.0, 1.0)
		_chase_rotation_quat = _chase_rotation_quat.slerp(target_rotation_quat, slerp_weight)

		# Normalize to prevent drift
		_chase_rotation_quat = _chase_rotation_quat.normalized()

		# Create camera basis from smoothed rotation
		var camera_basis: Basis = Basis(_chase_rotation_quat)

		# Calculate position: behind and above aircraft in camera's smoothed local space
		var target_pos: Vector3 = aircraft_position
		target_pos += camera_basis.z * chase_distance  # behind (in Godot, Z is backward)
		target_pos += camera_basis.y * chase_height    # above

		# Set position (fixed, no smoothing to avoid jitter)
		global_position = target_pos

		# Look at aircraft using the smoothed up vector (preserves roll)
		# Using camera_basis.y instead of Vector3.UP prevents horizon-lock
		look_at(aircraft_position, camera_basis.y)

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
