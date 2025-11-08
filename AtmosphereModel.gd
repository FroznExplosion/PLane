class_name AtmosphereModel
extends Node

## Atmospheric Model - ISA (International Standard Atmosphere)
## Calculates air density, pressure, temperature, viscosity vs altitude
## Based on standard atmosphere model used in aviation

# Current atmospheric state
var current_altitude: float = 0.0
var current_density: float = PhysicsConstants.SEA_LEVEL_DENSITY
var current_pressure: float = PhysicsConstants.SEA_LEVEL_PRESSURE
var current_temperature: float = PhysicsConstants.SEA_LEVEL_TEMP
var current_speed_of_sound: float = PhysicsConstants.SPEED_OF_SOUND_SL
var current_dynamic_viscosity: float = 1.789e-5

# Wind model (future expansion)
var wind_velocity: Vector3 = Vector3.ZERO
var turbulence_intensity: float = 0.0

func _ready():
	update_atmosphere(0.0)

func update_atmosphere(altitude: float) -> void:
	current_altitude = clamp(altitude, -100.0, 30000.0)

	var properties = get_atmospheric_properties(current_altitude)
	current_density = properties["density"]
	current_pressure = properties["pressure"]
	current_temperature = properties["temperature"]
	current_speed_of_sound = properties["speed_of_sound"]
	current_dynamic_viscosity = properties["dynamic_viscosity"]

func get_atmospheric_properties(altitude: float) -> Dictionary:
	var properties = {}

	if altitude < 0:
		altitude = 0  # Clamp to sea level minimum

	if altitude < PhysicsConstants.TROPOPAUSE_ALT:
		# TROPOSPHERE (0 to 11,000m)
		# Temperature decreases linearly with altitude
		var temp_ratio = 1.0 + (PhysicsConstants.LAPSE_RATE * altitude / PhysicsConstants.SEA_LEVEL_TEMP)
		properties["temperature"] = PhysicsConstants.SEA_LEVEL_TEMP * temp_ratio

		# Pressure using barometric formula
		var exponent = -PhysicsConstants.GRAVITY / (PhysicsConstants.LAPSE_RATE * PhysicsConstants.GAS_CONSTANT)
		properties["pressure"] = PhysicsConstants.SEA_LEVEL_PRESSURE * pow(temp_ratio, exponent)

		# Density from ideal gas law: ρ = P / (R * T)
		properties["density"] = properties["pressure"] / (PhysicsConstants.GAS_CONSTANT * properties["temperature"])
	else:
		# STRATOSPHERE (11,000m+)
		# Temperature is constant in lower stratosphere
		var trop_temp = PhysicsConstants.SEA_LEVEL_TEMP + PhysicsConstants.LAPSE_RATE * PhysicsConstants.TROPOPAUSE_ALT
		var trop_temp_ratio = 1.0 + (PhysicsConstants.LAPSE_RATE * PhysicsConstants.TROPOPAUSE_ALT / PhysicsConstants.SEA_LEVEL_TEMP)
		var trop_exponent = -PhysicsConstants.GRAVITY / (PhysicsConstants.LAPSE_RATE * PhysicsConstants.GAS_CONSTANT)
		var trop_pressure = PhysicsConstants.SEA_LEVEL_PRESSURE * pow(trop_temp_ratio, trop_exponent)

		properties["temperature"] = trop_temp

		# Exponential pressure decrease in stratosphere
		var height_above_trop = altitude - PhysicsConstants.TROPOPAUSE_ALT
		properties["pressure"] = trop_pressure * exp(-PhysicsConstants.GRAVITY * height_above_trop / (PhysicsConstants.GAS_CONSTANT * trop_temp))

		# Density from ideal gas law
		properties["density"] = properties["pressure"] / (PhysicsConstants.GAS_CONSTANT * properties["temperature"])

	# Calculate speed of sound: a = √(γ * R * T)
	properties["speed_of_sound"] = sqrt(PhysicsConstants.GAMMA * PhysicsConstants.GAS_CONSTANT * properties["temperature"])

	# Calculate dynamic viscosity using Sutherland's formula
	properties["dynamic_viscosity"] = calculate_viscosity(properties["temperature"])

	# Kinematic viscosity: ν = μ / ρ
	properties["kinematic_viscosity"] = properties["dynamic_viscosity"] / properties["density"]

	return properties

func calculate_viscosity(temperature: float) -> float:
	## Sutherland's formula for air viscosity
	## μ = μ₀ * (T/T₀)^(3/2) * (T₀ + S) / (T + S)
	var T0 = PhysicsConstants.SUTHERLAND_REF_TEMP
	var mu0 = PhysicsConstants.SUTHERLAND_REF_VISC
	var S = PhysicsConstants.SUTHERLAND_TEMP

	return mu0 * pow(temperature / T0, 1.5) * (T0 + S) / (temperature + S)

func calculate_reynolds_number(velocity: float, characteristic_length: float) -> float:
	## Reynolds Number: Re = ρ * V * L / μ
	if current_dynamic_viscosity < 1e-10:
		return 0.0
	return current_density * velocity * characteristic_length / current_dynamic_viscosity

func calculate_mach_number(velocity: float) -> float:
	## Mach Number: M = V / a
	if current_speed_of_sound < 1.0:
		return 0.0
	return velocity / current_speed_of_sound

func calculate_dynamic_pressure(velocity: float) -> float:
	## Dynamic Pressure: q = 0.5 * ρ * V²
	return 0.5 * current_density * velocity * velocity

func get_density() -> float:
	return current_density

func get_pressure() -> float:
	return current_pressure

func get_temperature() -> float:
	return current_temperature

func get_speed_of_sound() -> float:
	return current_speed_of_sound

func get_wind_velocity() -> Vector3:
	return wind_velocity

func set_wind(velocity: Vector3) -> void:
	wind_velocity = velocity

func get_atmospheric_state() -> Dictionary:
	return {
		"altitude": current_altitude,
		"density": current_density,
		"pressure": current_pressure,
		"temperature": current_temperature,
		"speed_of_sound": current_speed_of_sound,
		"dynamic_viscosity": current_dynamic_viscosity,
		"wind_velocity": wind_velocity
	}
