class_name GunReticle
extends Control

## Predictive Gun Reticle
## Shows where bullets will impact based on speed, distance, and bullet drop

@export var gun: MinigunWeapon  ## Reference to gun system
@export var aircraft: FDMCore  ## Reference to aircraft
@export var reticle_color: Color = Color.GREEN  ## Reticle color
@export var reticle_size: float = 20.0  ## Reticle size in pixels
@export var lead_indicator_size: float = 10.0  ## Lead indicator size

# Reticle components
var center_dot: ColorRect
var crosshair_lines: Array[ColorRect] = []
var lead_indicator: ColorRect
var ammo_label: Label
var fire_rate_label: Label
var spin_indicator: ProgressBar

func _ready():
	# Create reticle UI elements
	create_center_dot()
	create_crosshair()
	create_lead_indicator()
	create_hud_elements()

	# Find gun and aircraft if not set
	if not gun:
		gun = get_node_or_null("/root/Main/Aircraft/VisualModel/MinigunWeapon")
	if not aircraft:
		aircraft = get_node_or_null("/root/Main/Aircraft")

func create_center_dot():
	center_dot = ColorRect.new()
	center_dot.size = Vector2(4, 4)
	center_dot.color = reticle_color
	center_dot.position = Vector2(-2, -2)  # Center it
	add_child(center_dot)

func create_crosshair():
	# Top line
	var top = ColorRect.new()
	top.size = Vector2(2, reticle_size)
	top.color = reticle_color
	top.position = Vector2(-1, -reticle_size - 5)
	add_child(top)
	crosshair_lines.append(top)

	# Bottom line
	var bottom = ColorRect.new()
	bottom.size = Vector2(2, reticle_size)
	bottom.color = reticle_color
	bottom.position = Vector2(-1, 5)
	add_child(bottom)
	crosshair_lines.append(bottom)

	# Left line
	var left = ColorRect.new()
	left.size = Vector2(reticle_size, 2)
	left.color = reticle_color
	left.position = Vector2(-reticle_size - 5, -1)
	add_child(left)
	crosshair_lines.append(left)

	# Right line
	var right = ColorRect.new()
	right.size = Vector2(reticle_size, 2)
	right.color = reticle_color
	right.position = Vector2(5, -1)
	add_child(right)
	crosshair_lines.append(right)

func create_lead_indicator():
	# Lead indicator (shows where bullets will go with velocity/drop compensation)
	lead_indicator = ColorRect.new()
	lead_indicator.size = Vector2(lead_indicator_size, lead_indicator_size)
	lead_indicator.color = Color(reticle_color.r, reticle_color.g, reticle_color.b, 0.5)  # Semi-transparent
	lead_indicator.position = Vector2(-lead_indicator_size / 2, -lead_indicator_size / 2)
	add_child(lead_indicator)

func create_hud_elements():
	# Ammo counter
	ammo_label = Label.new()
	ammo_label.add_theme_font_size_override("font_size", 24)
	ammo_label.position = Vector2(20, 20)
	ammo_label.text = "AMMO: 2000"
	add_child(ammo_label)

	# Fire rate indicator
	fire_rate_label = Label.new()
	fire_rate_label.add_theme_font_size_override("font_size", 18)
	fire_rate_label.position = Vector2(20, 50)
	fire_rate_label.text = "RPM: 0"
	add_child(fire_rate_label)

	# Spin-up indicator
	spin_indicator = ProgressBar.new()
	spin_indicator.size = Vector2(200, 20)
	spin_indicator.position = Vector2(20, 80)
	spin_indicator.max_value = 1.0
	spin_indicator.value = 0.0
	spin_indicator.show_percentage = false
	add_child(spin_indicator)

	var spin_label = Label.new()
	spin_label.text = "SPIN"
	spin_label.position = Vector2(20, 60)
	add_child(spin_label)

func _process(_delta):
	if not gun or not aircraft:
		return

	# Update position to screen center
	position = get_viewport_rect().size / 2

	# Get gun state
	var gun_state = gun.get_gun_state()

	# Update HUD
	if gun.infinite_ammo:
		ammo_label.text = "AMMO: ∞"
	else:
		ammo_label.text = "AMMO: %d" % gun_state["ammo"]

	fire_rate_label.text = "RPM: %d" % (gun_state["fire_rate"] * 60)
	spin_indicator.value = gun_state["spin_level"]

	# Change color based on firing state
	if gun_state["is_firing"] and gun_state["spin_level"] > 0.2:
		reticle_color = Color.RED
	elif gun_state["spin_level"] > 0.0:
		reticle_color = Color.YELLOW
	else:
		reticle_color = Color.GREEN

	# Update all reticle colors
	center_dot.color = reticle_color
	for line in crosshair_lines:
		line.color = reticle_color

	# Calculate predictive lead indicator position
	update_lead_indicator()

func update_lead_indicator():
	## Calculate where bullets will hit and position lead indicator

	if not aircraft or not gun:
		lead_indicator.visible = false
		return

	# Cast ray forward to find target distance
	var camera = get_viewport().get_camera_3d()
	if not camera:
		lead_indicator.visible = false
		return

	var ray_origin = camera.global_position
	var ray_direction = -camera.global_basis.z
	var ray_length = gun.max_range

	# Get 3D world from the viewport, not from Control node
	var world_3d = get_viewport().world_3d
	if not world_3d:
		lead_indicator.visible = false
		return

	var space_state = world_3d.direct_space_state
	var query = PhysicsRayQueryParameters3D.create(
		ray_origin,
		ray_origin + ray_direction * ray_length
	)
	query.collide_with_areas = false
	query.collide_with_bodies = true
	query.exclude = [aircraft]

	var result = space_state.intersect_ray(query)

	if result:
		var target_distance = ray_origin.distance_to(result.position)

		# Get aircraft velocity
		var aircraft_velocity = aircraft.linear_velocity
		var aircraft_speed = aircraft_velocity.length()

		# Calculate time to target
		var effective_bullet_speed = gun.muzzle_velocity + aircraft_speed
		var time_to_target = target_distance / effective_bullet_speed

		# Calculate bullet drop
		var drop = 0.5 * gun.bullet_drop_gravity * time_to_target * time_to_target

		# Convert drop to screen space
		var drop_angle = atan2(drop, target_distance)

		# Project to screen (simplified - just offset downward proportional to distance)
		var screen_offset_y = drop_angle * 1000.0  # Scale factor for visibility

		lead_indicator.position = Vector2(-lead_indicator_size / 2, screen_offset_y - lead_indicator_size / 2)
		lead_indicator.visible = true
		lead_indicator.color = Color(reticle_color.r, reticle_color.g, reticle_color.b, 0.7)
	else:
		# No target in range
		lead_indicator.visible = false
