class_name AerodynamicDataTables
extends Resource

## Aerodynamic Coefficient Lookup Tables
## Based on typical F-16 characteristics with realistic stall behavior
## Uses efficient PackedFloat32Array for fast interpolation

# Alpha (angle of attack) breakpoints in radians
# Covers -20° to 90° (full envelope including cobra)
var alpha_breakpoints: PackedFloat32Array = PackedFloat32Array([
	deg_to_rad(-20), deg_to_rad(-15), deg_to_rad(-10), deg_to_rad(-5),
	deg_to_rad(0), deg_to_rad(5), deg_to_rad(10), deg_to_rad(15),
	deg_to_rad(16), deg_to_rad(18), deg_to_rad(20), deg_to_rad(25),
	deg_to_rad(30), deg_to_rad(40), deg_to_rad(50), deg_to_rad(60),
	deg_to_rad(70), deg_to_rad(80), deg_to_rad(90)
])

# Lift Coefficient (CL) vs Alpha
# Pre-stall: linear ~5.5/rad
# Stall at 16°: CL_max ~1.4
# Post-stall: gradual decrease, maintains 30-50% at high alpha
var CL_values: PackedFloat32Array = PackedFloat32Array([
	-0.9,   # -20° (negative lift)
	-0.7,   # -15°
	-0.48,  # -10°
	-0.24,  # -5°
	0.2,    # 0° (CL0 - some lift at zero AoA)
	0.64,   # 5°
	1.07,   # 10°
	1.45,   # 15° (approaching stall)
	1.50,   # 16° (CL_max - stall angle)
	1.48,   # 18° (just past stall)
	1.38,   # 20° (post-stall, still high lift)
	1.10,   # 25°
	0.85,   # 30° (60% of max)
	0.65,   # 40°
	0.50,   # 50°
	0.40,   # 60°
	0.32,   # 70°
	0.25,   # 80°
	0.20    # 90° (minimal but non-zero for recovery)
])

# Drag Coefficient (CD) vs Alpha
# Low at cruise, MASSIVE increase in stall (JSBSim key insight!)
# Base CD0 = 0.018, induced drag K*CL², plus stall drag
var CD_values: PackedFloat32Array = PackedFloat32Array([
	0.15,   # -20° (high drag inverted)
	0.10,   # -15°
	0.06,   # -10°
	0.03,   # -5°
	0.018,  # 0° (CD0 - minimum drag)
	0.025,  # 5°
	0.045,  # 10°
	0.08,   # 15° (drag rising)
	0.25,   # 16° (STALL - drag increase)
	0.35,   # 18°
	0.45,   # 20°
	0.60,   # 25°
	0.75,   # 30°
	0.90,   # 40°
	1.00,   # 50°
	1.05,   # 60°
	1.08,   # 70°
	1.10,   # 80°
	1.10    # 90° (maximum drag)
])

# Side Force Coefficient (CY) vs Beta (sideslip)
# Beta range: -30° to +30°
var beta_breakpoints: PackedFloat32Array = PackedFloat32Array([
	deg_to_rad(-30), deg_to_rad(-20), deg_to_rad(-10), deg_to_rad(-5),
	deg_to_rad(0), deg_to_rad(5), deg_to_rad(10), deg_to_rad(20), deg_to_rad(30)
])

var CY_values: PackedFloat32Array = PackedFloat32Array([
	1.2,   # -30° (strong side force)
	0.8,   # -20°
	0.4,   # -10°
	0.2,   # -5°
	0.0,   # 0° (no sideslip)
	-0.2,  # 5°
	-0.4,  # 10°
	-0.8,  # 20°
	-1.2   # 30°
])

# Pitching Moment Coefficient (Cm) vs Alpha
# Negative Cm_alpha = stable (nose drops with increasing AoA)
# Cm = Cm0 + Cm_alpha * alpha
var Cm_values: PackedFloat32Array = PackedFloat32Array([
	0.15,   # -20° (nose-up moment)
	0.10,   # -15°
	0.06,   # -10°
	0.03,   # -5°
	0.05,   # 0° (Cm0 - slight nose-up at level)
	0.00,   # 5°
	-0.10,  # 10° (nose-down moment develops)
	-0.25,  # 15° (strong nose-down for stability)
	-0.30,  # 16° (stall - nose drops)
	-0.35,  # 18°
	-0.38,  # 20°
	-0.40,  # 25°
	-0.38,  # 30° (recovery moment)
	-0.30,  # 40°
	-0.20,  # 50°
	-0.10,  # 60°
	0.00,   # 70°
	0.05,   # 80°
	0.10    # 90° (helps recovery from cobra)
])

# Rolling Moment Coefficient (Cl) vs Beta
# Dihedral effect: positive beta creates negative roll moment (levels wings)
var Cl_beta_values: PackedFloat32Array = PackedFloat32Array([
	0.30,   # -30° (roll right)
	0.20,   # -20°
	0.10,   # -10°
	0.05,   # -5°
	0.0,    # 0°
	-0.05,  # 5°
	-0.10,  # 10° (dihedral effect)
	-0.20,  # 20°
	-0.30   # 30° (roll left)
])

# Yawing Moment Coefficient (Cn) vs Beta
# Weathercock stability: positive beta creates negative yaw moment (points into wind)
var Cn_beta_values: PackedFloat32Array = PackedFloat32Array([
	-0.25,  # -30° (yaw left)
	-0.17,  # -20°
	-0.08,  # -10°
	-0.04,  # -5°
	0.0,    # 0°
	0.04,   # 5°
	0.08,   # 10° (weathercock stability)
	0.17,   # 20°
	0.25    # 30° (yaw right)
])

# Damping Derivatives (oppose angular rates)
# These are constant multipliers in this simplified model
# In reality they'd vary with alpha/beta

# Roll damping (Clp) - opposes roll rate
var Clp: float = -0.6  # per rad/s

# Pitch damping (Cmq) - opposes pitch rate (CRITICAL for stability)
var Cmq: float = -25.0  # per rad/s (very strong for longitudinal stability)

# Yaw damping (Cnr) - opposes yaw rate
var Cnr: float = -0.25  # per rad/s

# Control Effectiveness

# Elevator effectiveness vs alpha (reduces in stall)
var elevator_effectiveness: PackedFloat32Array = PackedFloat32Array([
	1.0,   # -20°
	1.0,   # -15°
	1.0,   # -10°
	1.0,   # -5°
	1.0,   # 0°
	1.0,   # 5°
	1.0,   # 10°
	1.0,   # 15°
	0.9,   # 16° (stall starts)
	0.75,  # 18°
	0.6,   # 20°
	0.45,  # 25°
	0.35,  # 30°
	0.25,  # 40°
	0.20,  # 50°
	0.15,  # 60°
	0.12,  # 70°
	0.10,  # 80°
	0.08   # 90°
])

# Aileron effectiveness vs alpha
var aileron_effectiveness: PackedFloat32Array = PackedFloat32Array([
	1.0,   # -20°
	1.0,   # -15°
	1.0,   # -10°
	1.0,   # -5°
	1.0,   # 0°
	1.0,   # 5°
	1.0,   # 10°
	1.0,   # 15°
	0.85,  # 16° (stall)
	0.65,  # 18°
	0.50,  # 20°
	0.35,  # 25°
	0.25,  # 30°
	0.15,  # 40°
	0.10,  # 50°
	0.08,  # 60°
	0.05,  # 70°
	0.03,  # 80°
	0.02   # 90°
])

# Rudder effectiveness vs alpha
var rudder_effectiveness: PackedFloat32Array = PackedFloat32Array([
	1.0,   # -20°
	1.0,   # -15°
	1.0,   # -10°
	1.0,   # -5°
	1.0,   # 0°
	1.0,   # 5°
	1.0,   # 10°
	1.0,   # 15°
	0.95,  # 16° (stall - rudder less affected)
	0.85,  # 18°
	0.75,  # 20°
	0.65,  # 25°
	0.55,  # 30°
	0.45,  # 40°
	0.35,  # 50°
	0.28,  # 60°
	0.22,  # 70°
	0.18,  # 80°
	0.15   # 90°
])

func _init():
	# Validate table sizes match
	assert(alpha_breakpoints.size() == CL_values.size(), "CL table size mismatch")
	assert(alpha_breakpoints.size() == CD_values.size(), "CD table size mismatch")
	assert(beta_breakpoints.size() == CY_values.size(), "CY table size mismatch")

func get_CL(alpha: float) -> float:
	return MathUtils.linear_interpolate(alpha, alpha_breakpoints, CL_values)

func get_CD(alpha: float) -> float:
	return MathUtils.linear_interpolate(alpha, alpha_breakpoints, CD_values)

func get_Cm(alpha: float) -> float:
	return MathUtils.linear_interpolate(alpha, alpha_breakpoints, Cm_values)

func get_CY(beta: float) -> float:
	return MathUtils.linear_interpolate(beta, beta_breakpoints, CY_values)

func get_Cl_beta(beta: float) -> float:
	return MathUtils.linear_interpolate(beta, beta_breakpoints, Cl_beta_values)

func get_Cn_beta(beta: float) -> float:
	return MathUtils.linear_interpolate(beta, beta_breakpoints, Cn_beta_values)

func get_elevator_effectiveness(alpha: float) -> float:
	return MathUtils.linear_interpolate(alpha, alpha_breakpoints, elevator_effectiveness)

func get_aileron_effectiveness(alpha: float) -> float:
	return MathUtils.linear_interpolate(alpha, alpha_breakpoints, aileron_effectiveness)

func get_rudder_effectiveness(alpha: float) -> float:
	return MathUtils.linear_interpolate(alpha, alpha_breakpoints, rudder_effectiveness)

func get_all_coefficients(alpha: float, beta: float) -> Dictionary:
	return {
		"CL": get_CL(alpha),
		"CD": get_CD(alpha),
		"CY": get_CY(beta),
		"Cm": get_Cm(alpha),
		"Cl": get_Cl_beta(beta),
		"Cn": get_Cn_beta(beta),
		"Clp": Clp,
		"Cmq": Cmq,
		"Cnr": Cnr,
		"elevator_eff": get_elevator_effectiveness(alpha),
		"aileron_eff": get_aileron_effectiveness(alpha),
		"rudder_eff": get_rudder_effectiveness(alpha)
	}
