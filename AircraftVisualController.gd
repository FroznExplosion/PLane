extends Node3D

## Aircraft Visual Controller
## Animates control surfaces, thrust vectoring nozzle, flaps, and airbrake

var aircraft: FDMCore

# Control surface references
var left_aileron: Node3D
var right_aileron: Node3D
var left_elevator: Node3D
var right_elevator: Node3D
var rudder: Node3D
var left_flap: Node3D
var right_flap: Node3D
var airbrake: Node3D
var thrust_nozzle: Node3D
var nose_gear: Node3D
var left_main_gear: Node3D
var right_main_gear: Node3D
var nose_wheel: Node3D

# Animation settings
@export var gear_retract_angle: float = 90.0  ## Angle gear rotates when retracting (degrees)
@export var nose_wheel_max_angle: float = 45.0  ## Maximum nose wheel steering angle (degrees)
@export var animation_smoothing: float = 10.0  ## How fast surfaces move to target position
@export var debug_animation: bool = false  ## Print animation debug info

var debug_counter: int = 0

func _ready() -> void:
	# Find aircraft
	aircraft = get_parent() as FDMCore
	if not aircraft:
		push_error("AircraftVisualController must be child of FDMCore!")
		return

	# Get all control surface nodes
	left_aileron = get_node_or_null("../VisualModel/Wings/LeftWing/LeftAileron")
	right_aileron = get_node_or_null("../VisualModel/Wings/RightWing/RightAileron")
	left_elevator = get_node_or_null("../VisualModel/Tail/HorizontalStabilizers/LeftElevator")
	right_elevator = get_node_or_null("../VisualModel/Tail/HorizontalStabilizers/RightElevator")
	rudder = get_node_or_null("../VisualModel/Tail/VerticalStabilizer/Rudder")
	left_flap = get_node_or_null("../VisualModel/Wings/LeftWing/LeftFlap")
	right_flap = get_node_or_null("../VisualModel/Wings/RightWing/RightFlap")
	airbrake = get_node_or_null("../VisualModel/Airbrake")
	thrust_nozzle = get_node_or_null("../VisualModel/Engine/ThrustVectoringNozzle")
	nose_gear = get_node_or_null("../VisualModel/LandingGear/NoseGear")
	left_main_gear = get_node_or_null("../VisualModel/LandingGear/LeftMainGear")
	right_main_gear = get_node_or_null("../VisualModel/LandingGear/RightMainGear")
	nose_wheel = get_node_or_null("../VisualModel/LandingGear/NoseGear/NoseWheel")

	# Debug: Print what was found
	print("AircraftVisualController initialized:")
	print("  Left Aileron: ", left_aileron != null)
	print("  Right Aileron: ", right_aileron != null)
	print("  Left Elevator: ", left_elevator != null)
	print("  Right Elevator: ", right_elevator != null)
	print("  Rudder: ", rudder != null)
	print("  Left Flap: ", left_flap != null)
	print("  Right Flap: ", right_flap != null)
	print("  Airbrake: ", airbrake != null)
	print("  Thrust Nozzle: ", thrust_nozzle != null)
	print("  Nose Gear: ", nose_gear != null)
	print("  Left Main Gear: ", left_main_gear != null)
	print("  Right Main Gear: ", right_main_gear != null)
	print("  Nose Wheel: ", nose_wheel != null)

func _process(delta: float) -> void:
	if not aircraft:
		return

	animate_control_surfaces(delta)
	animate_flaps(delta)
	animate_airbrake(delta)
	animate_thrust_vectoring(delta)
	animate_landing_gear(delta)
	animate_nose_wheel_steering(delta)

	# Debug output every 60 frames
	if debug_animation:
		debug_counter += 1
		if debug_counter >= 60:
			debug_counter = 0
			print_debug_info()

func animate_control_surfaces(delta: float) -> void:
	# Get processed control inputs (elevator, aileron, rudder from FlightControlSystem)
	var controls = aircraft.processed_controls
	var aileron_input = controls.get("aileron", 0.0)
	var elevator_input = controls.get("elevator", 0.0)
	var rudder_input = controls.get("rudder", 0.0)

	# Ailerons (differential - left and right move opposite)
	# Positive aileron = roll right = left aileron down (positive X rotation), right aileron up (negative X rotation)
	if left_aileron:
		var target_angle = deg_to_rad(aileron_input * aircraft.aileron_deflection_max)
		left_aileron.rotation.x = lerp_angle(left_aileron.rotation.x, target_angle, animation_smoothing * delta)

	if right_aileron:
		var target_angle = deg_to_rad(-aileron_input * aircraft.aileron_deflection_max)
		right_aileron.rotation.x = lerp_angle(right_aileron.rotation.x, target_angle, animation_smoothing * delta)

	# Elevators (both move together)
	# Positive elevator = pitch down = elevators down (positive X rotation)
	var elevator_angle = deg_to_rad(elevator_input * aircraft.elevator_deflection_max)
	if left_elevator:
		left_elevator.rotation.x = lerp_angle(left_elevator.rotation.x, elevator_angle, animation_smoothing * delta)

	if right_elevator:
		right_elevator.rotation.x = lerp_angle(right_elevator.rotation.x, elevator_angle, animation_smoothing * delta)

	# Rudder
	# Positive rudder = yaw right = rudder deflects right (positive Y rotation)
	if rudder:
		var target_angle = deg_to_rad(rudder_input * aircraft.rudder_deflection_max)
		rudder.rotation.y = lerp_angle(rudder.rotation.y, target_angle, animation_smoothing * delta)

func animate_flaps(delta: float) -> void:
	# Get flap position (0.0 = retracted, 1.0 = fully deployed)
	var flap_position = aircraft.control_inputs.get("flaps", 0.0)
	# Flaps deploy downward (positive X rotation)
	var target_angle = deg_to_rad(flap_position * aircraft.flap_deflection_max)

	if left_flap:
		left_flap.rotation.x = lerp_angle(left_flap.rotation.x, target_angle, animation_smoothing * delta)

	if right_flap:
		right_flap.rotation.x = lerp_angle(right_flap.rotation.x, target_angle, animation_smoothing * delta)

func animate_airbrake(delta: float) -> void:
	# F-16 style airbrake on top of fuselage - opens upward
	# Deploys when throttle_down OR PSM mode active
	var airbrake_active = aircraft.control_inputs.get("airbrake", false)
	var psm_mode = aircraft.processed_controls.get("psm_mode", false)
	var should_deploy = airbrake_active or psm_mode

	# Airbrake opens upward (negative X rotation to open up)
	var target_angle = deg_to_rad(-aircraft.airbrake_deflection_max if should_deploy else 0.0)

	if airbrake:
		airbrake.rotation.x = lerp_angle(airbrake.rotation.x, target_angle, animation_smoothing * delta)

func animate_thrust_vectoring(delta: float) -> void:
	if not thrust_nozzle or not aircraft.propulsion:
		return

	# Get thrust vectoring angles from propulsion system
	var engine_state = aircraft.propulsion.get_engine_state()
	var vector_pitch = engine_state.get("vector_pitch", 0.0)  # Radians
	var vector_yaw = engine_state.get("vector_yaw", 0.0)  # Radians

	# Clamp to maximum deflection (convert thrust_vector_max to radians)
	var max_deflection = deg_to_rad(aircraft.thrust_vector_max)
	vector_pitch = clamp(vector_pitch, -max_deflection, max_deflection)
	vector_yaw = clamp(vector_yaw, -max_deflection, max_deflection)

	# Debug: Show nozzle animation status
	if Engine.get_physics_frames() % 30 == 0:
		var psm_mode = aircraft.control_inputs.get("psm_mode", false)
		var combat_tv = aircraft.processed_controls.get("combat_tv_mode", false) if "combat_tv_mode" in aircraft.processed_controls else false
		print("NOZZLE_ANIM: target=%.4f rad (%.1f°) | current=%.4f rad (%.1f°) | mode=PSM:%s CTV:%s" % [
			vector_pitch, rad_to_deg(vector_pitch),
			thrust_nozzle.rotation.x, rad_to_deg(thrust_nozzle.rotation.x),
			psm_mode, combat_tv
		])

	# Apply pitch deflection (rotation around X axis for pitch in Godot coordinates)
	# Negate the pitch to match the nozzle's geometry (positive rotation should point nozzle down)
	# Use exact same animation smoothing as other control surfaces for consistency
	thrust_nozzle.rotation.x = lerp_angle(thrust_nozzle.rotation.x, -vector_pitch, animation_smoothing * delta)

	# Apply yaw deflection (rotation around Y axis for yaw in Godot coordinates)
	# Use exact same animation smoothing as other control surfaces for consistency
	thrust_nozzle.rotation.y = lerp_angle(thrust_nozzle.rotation.y, vector_yaw, animation_smoothing * delta)

func print_debug_info() -> void:
	var raw_inputs = aircraft.control_inputs
	var processed = aircraft.processed_controls
	print("=== ANIMATION DEBUG ===")
	print("Raw inputs - Pitch: %.2f | Roll: %.2f | Yaw: %.2f" % [raw_inputs.get("pitch", 0.0), raw_inputs.get("roll", 0.0), raw_inputs.get("yaw", 0.0)])
	print("Processed - Elevator: %.2f | Aileron: %.2f | Rudder: %.2f" % [processed.get("elevator", 0.0), processed.get("aileron", 0.0), processed.get("rudder", 0.0)])
	print("Flaps: %.2f | Airbrake: %s" % [raw_inputs.get("flaps", 0.0), raw_inputs.get("airbrake", false)])

	if left_aileron:
		print("Left aileron rotation: %.2f°" % rad_to_deg(left_aileron.rotation.x))
	if left_elevator:
		print("Left elevator rotation: %.2f°" % rad_to_deg(left_elevator.rotation.x))
	if rudder:
		print("Rudder rotation: %.2f°" % rad_to_deg(rudder.rotation.y))

	if aircraft.propulsion:
		var engine_state = aircraft.propulsion.get_engine_state()
		print("Thrust vector pitch: %.2f°" % rad_to_deg(engine_state.get("vector_pitch", 0.0)))
		print("Thrust vector yaw: %.2f°" % rad_to_deg(engine_state.get("vector_yaw", 0.0)))

func animate_landing_gear(delta: float) -> void:
	# Get gear state from landing gear system
	if not aircraft.landing_gear:
		return

	var gear_extended = aircraft.landing_gear.gear_extended
	# Gear retracts upward (negative X rotation)
	var target_angle = deg_to_rad(0.0 if gear_extended else -gear_retract_angle)

	# Animate all gear legs
	if nose_gear:
		nose_gear.rotation.x = lerp_angle(nose_gear.rotation.x, target_angle, animation_smoothing * delta)

	if left_main_gear:
		left_main_gear.rotation.x = lerp_angle(left_main_gear.rotation.x, target_angle, animation_smoothing * delta)

	if right_main_gear:
		right_main_gear.rotation.x = lerp_angle(right_main_gear.rotation.x, target_angle, animation_smoothing * delta)

	# Hide gear when retracted (for cleaner look)
	if nose_gear:
		nose_gear.visible = (nose_gear.rotation.x > deg_to_rad(-gear_retract_angle * 0.9))
	if left_main_gear:
		left_main_gear.visible = (left_main_gear.rotation.x > deg_to_rad(-gear_retract_angle * 0.9))
	if right_main_gear:
		right_main_gear.visible = (right_main_gear.rotation.x > deg_to_rad(-gear_retract_angle * 0.9))

func animate_nose_wheel_steering(delta: float) -> void:
	# Nose wheel steers with rudder input (only when on ground)
	if not nose_wheel or not aircraft.landing_gear:
		return

	# Only steer when gear is down and on ground
	if not aircraft.landing_gear.gear_extended:
		nose_wheel.rotation.y = lerp_angle(nose_wheel.rotation.y, 0.0, animation_smoothing * delta)
		return

	# Get rudder input
	var rudder_input = aircraft.processed_controls.get("rudder", 0.0)

	# Nose wheel steering (reduced angle compared to rudder)
	var steering_angle = deg_to_rad(rudder_input * nose_wheel_max_angle)
	nose_wheel.rotation.y = lerp_angle(nose_wheel.rotation.y, steering_angle, animation_smoothing * delta)
