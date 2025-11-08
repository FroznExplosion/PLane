extends Node

## Flight Input Controller
## Handles user input and sends to FDM

var aircraft: FDMCore
var control_inputs: Dictionary = {
	"pitch": 0.0,
	"roll": 0.0,
	"yaw": 0.0,
	"throttle": 0.5,
	"brake": 0.0,
	"psm_mode": false,  # Post-Stall Maneuver mode (Space)
	"afterburner": false,
	"flaps": 0.0  # Flaps position (0.0 = retracted, 1.0 = full down)
}

# PSM throttle management
var saved_throttle: float = 0.5  # Throttle value before entering PSM
var psm_was_active: bool = false

# Flaps
var flaps_position: float = 0.0  # Current flap position (0.0 to 1.0)

# Automatic trim
var trim_adjustment: float = 0.0  # Trim value that gets added to pitch input

# Mouse control
var free_look_active: bool = false
var mouse_delta: Vector2 = Vector2.ZERO

# Mouse control settings
@export_group("Mouse Control")
@export var mouse_control_enabled: bool = true  ## Use mouse for pitch/yaw
@export var mouse_sensitivity: float = 0.015  ## Mouse sensitivity (0.001-0.05)
@export var mouse_pitch_invert: bool = false  ## Invert pitch axis
@export var mouse_yaw_invert: bool = false  ## Invert yaw axis

# Input rates (time to reach 100% from neutral)
@export_group("Control Input Rates")
@export var pitch_ramp_time: float = 1.0  ## Time to reach full pitch deflection (seconds)
@export var roll_ramp_time: float = 1.0  ## Time to reach full roll deflection (seconds)
@export var yaw_ramp_time: float = 1.0  ## Time to reach full yaw deflection (seconds)
@export var control_return_time: float = 0.2  ## Time for controls to return to neutral when released (seconds)
@export var throttle_ramp_time: float = 2.0  ## Time to move throttle from 0 to 100% (seconds)
@export var psm_throttle_decay_time: float = 0.3  ## How fast throttle decays to 0 in PSM when released (seconds)
@export var flap_deployment_time: float = 5.0  ## Time for flaps to fully deploy/retract (seconds)

func _ready():
	# Find aircraft
	aircraft = get_parent() as FDMCore
	if not aircraft:
		push_error("FlightInputController must be child of FDMCore!")

	# Capture mouse for flight control
	if mouse_control_enabled:
		Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)

func _input(event):
	# Handle mouse motion for flight control
	if event is InputEventMouseMotion and mouse_control_enabled:
		if not free_look_active:
			mouse_delta = event.relative

	# Right-click for free look
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_RIGHT:
			free_look_active = event.pressed
			# TODO: Signal camera controller for free look

func _process(delta):
	if not aircraft:
		return

	handle_input(delta)
	aircraft.set_control_inputs(control_inputs)

	# Reset mouse delta after processing
	mouse_delta = Vector2.ZERO

func handle_input(delta: float):
	# Calculate rates (1.0 per ramp_time)
	var pitch_rate = 1.0 / pitch_ramp_time
	var roll_rate = 1.0 / roll_ramp_time
	var yaw_rate = 1.0 / yaw_ramp_time
	var return_rate = 1.0 / control_return_time  # Faster return to neutral
	var throttle_rate = 1.0 / throttle_ramp_time

	# Pitch (W/S + Mouse) - ramps up/down over time
	var target_pitch = 0.0

	# Keyboard input
	if Input.is_action_pressed("pitch_up"):
		target_pitch = 1.0
	elif Input.is_action_pressed("pitch_down"):
		target_pitch = -1.0

	# Add mouse input (if not in free look)
	if mouse_control_enabled and not free_look_active:
		var mouse_pitch_input = -mouse_delta.y * mouse_sensitivity
		if mouse_pitch_invert:
			mouse_pitch_input = -mouse_pitch_input
		target_pitch += mouse_pitch_input
		target_pitch = clamp(target_pitch, -1.0, 1.0)

	# Use faster rate when returning to neutral (target = 0)
	var pitch_rate_to_use = return_rate if target_pitch == 0.0 else pitch_rate
	control_inputs["pitch"] = move_toward(control_inputs["pitch"], target_pitch, pitch_rate_to_use * delta)

	# Automatic trim adjuster - reduces AoA to zero when no manual pitch input
	var manual_pitch_input_active = abs(target_pitch) > 0.01
	if aircraft and aircraft.auto_trim_enabled and not manual_pitch_input_active and abs(control_inputs["pitch"]) < aircraft.auto_trim_deadzone:
		# Get current AoA from aircraft
		var current_aoa = aircraft.angle_of_attack

		# Adjust trim to reduce AoA toward zero
		# Positive AoA = nose too high, need nose-down trim (positive pitch input in our system)
		# The trim adjustment is gradual and proportional to AoA
		var trim_rate = aircraft.auto_trim_strength * delta
		var desired_trim_adjustment = current_aoa * 2.0  # Scale AoA to trim range
		trim_adjustment = move_toward(trim_adjustment, desired_trim_adjustment, trim_rate)

		# Apply trim to pitch input
		control_inputs["pitch"] += trim_adjustment
		control_inputs["pitch"] = clamp(control_inputs["pitch"], -1.0, 1.0)
	else:
		# Reset trim when manual input is active
		trim_adjustment = move_toward(trim_adjustment, 0.0, delta * 2.0)

	# Roll (A/D) - ramps up/down over time
	var target_roll = 0.0
	if Input.is_action_pressed("roll_left"):
		target_roll = -1.0
	elif Input.is_action_pressed("roll_right"):
		target_roll = 1.0
	# Use faster rate when returning to neutral
	var roll_rate_to_use = return_rate if target_roll == 0.0 else roll_rate
	control_inputs["roll"] = move_toward(control_inputs["roll"], target_roll, roll_rate_to_use * delta)

	# Yaw (Q/E + Mouse) - ramps up/down over time
	var target_yaw = 0.0

	# Keyboard input
	if Input.is_action_pressed("yaw_left"):
		target_yaw = -1.0
	elif Input.is_action_pressed("yaw_right"):
		target_yaw = 1.0

	# Add mouse input (if not in free look)
	if mouse_control_enabled and not free_look_active:
		var mouse_yaw_input = mouse_delta.x * mouse_sensitivity
		if mouse_yaw_invert:
			mouse_yaw_input = -mouse_yaw_input
		target_yaw += mouse_yaw_input
		target_yaw = clamp(target_yaw, -1.0, 1.0)

	# Use faster rate when returning to neutral
	var yaw_rate_to_use = return_rate if target_yaw == 0.0 else yaw_rate
	control_inputs["yaw"] = move_toward(control_inputs["yaw"], target_yaw, yaw_rate_to_use * delta)

	# Throttle (Shift/Ctrl) - handle before airbrake/PSM
	if Input.is_action_pressed("throttle_up"):
		control_inputs["throttle"] = min(1.0, control_inputs["throttle"] + throttle_rate * delta)
	elif Input.is_action_pressed("throttle_down"):
		control_inputs["throttle"] = max(0.0, control_inputs["throttle"] - throttle_rate * delta)

	# Airbrake (holding throttle_down) - only adds drag, no PSM effects
	var airbrake_active = Input.is_action_pressed("throttle_down")
	control_inputs["airbrake"] = airbrake_active

	# Post-Stall Maneuver Mode (Space) - thrust vectoring active, reduced aero
	var psm_active = Input.is_action_pressed("airbrake")
	control_inputs["psm_mode"] = psm_active

	# PSM mode state change detection (only for manual PSM activation)
	# Auto PSM from stall recovery is handled separately
	if psm_active and not psm_was_active:
		# Just entered manual PSM - save current throttle
		saved_throttle = control_inputs["throttle"]
		control_inputs["throttle"] = 0.0
	elif not psm_active and psm_was_active:
		# Just exited manual PSM - restore saved throttle
		control_inputs["throttle"] = saved_throttle

	psm_was_active = psm_active

	# In PSM mode, throttle decays rapidly when not actively increased
	# More throttle = more effective thrust vectoring
	if psm_active:
		var psm_decay_rate = 1.0 / psm_throttle_decay_time
		if not Input.is_action_pressed("throttle_up"):
			# Rapid decay to 0 when no throttle up input in PSM
			control_inputs["throttle"] = max(0.0, control_inputs["throttle"] - psm_decay_rate * delta)

	# Afterburner - Manual (R key) OR Automatic when throttle up + 100% throttle OR in PSM mode with throttle_up
	var manual_afterburner = Input.is_action_pressed("afterburner")
	var auto_afterburner = Input.is_action_pressed("throttle_up") and control_inputs["throttle"] >= 0.99
	var psm_afterburner = psm_active and Input.is_action_pressed("throttle_up")
	control_inputs["afterburner"] = manual_afterburner or auto_afterburner or psm_afterburner

	# Flaps (F/V keys) - 5 second deployment/retraction
	var flap_rate = 1.0 / flap_deployment_time  # 0.2 per second for 5 seconds
	if Input.is_action_pressed("flaps_down"):
		flaps_position = min(1.0, flaps_position + flap_rate * delta)
	elif Input.is_action_pressed("flaps_up"):
		flaps_position = max(0.0, flaps_position - flap_rate * delta)
	control_inputs["flaps"] = flaps_position

	# Camera change (C) - moved from cobra
	# Note: Camera controller handles this separately via flight_camera_change action

	# Landing gear toggle (G)
	if Input.is_action_just_pressed("toggle_gear"):
		if aircraft.landing_gear:
			aircraft.landing_gear.gear_extended = not aircraft.landing_gear.gear_extended
			print("Landing Gear: ", "DOWN" if aircraft.landing_gear.gear_extended else "UP")

	# Reset aircraft (Tab)
	if Input.is_action_just_pressed("reset_aircraft"):
		aircraft.reset_aircraft()
