class_name PhysicsConstants
extends Object

## Physical Constants for Flight Simulation
## All values in SI units unless noted

# Gravitational Constants
const GRAVITY: float = 9.80665  # m/s² (standard Earth gravity)
const EARTH_RADIUS: float = 6371000.0  # meters

# Atmospheric Constants (ISA - International Standard Atmosphere)
const SEA_LEVEL_PRESSURE: float = 101325.0  # Pa
const SEA_LEVEL_DENSITY: float = 1.225  # kg/m³
const SEA_LEVEL_TEMP: float = 288.15  # K (15°C)
const GAS_CONSTANT: float = 287.05  # J/(kg·K) for air
const LAPSE_RATE: float = -0.0065  # K/m (troposphere)
const TROPOPAUSE_ALT: float = 11000.0  # meters
const SPEED_OF_SOUND_SL: float = 340.29  # m/s at sea level
const GAMMA: float = 1.4  # Ratio of specific heats for air

# Unit Conversions
const FEET_TO_METERS: float = 0.3048
const METERS_TO_FEET: float = 3.28084
const KMH_TO_MS: float = 0.277778
const MS_TO_KMH: float = 3.6
const KNOTS_TO_MS: float = 0.514444
const MS_TO_KNOTS: float = 1.94384
const LBS_TO_KG: float = 0.453592
const KG_TO_LBS: float = 2.20462
const LBF_TO_N: float = 4.44822
const N_TO_LBF: float = 0.224809

# Mathematical Constants
const TWO_PI: float = 6.28318530718
const HALF_PI: float = 1.57079632679
const DEG_TO_RAD: float = 0.0174532925
const RAD_TO_DEG: float = 57.2957795131

# Physics Simulation
const PHYSICS_TIMESTEP: float = 1.0 / 120.0  # 120Hz standard
const MAX_PHYSICS_ITERATIONS: int = 10
const INTEGRATION_TOLERANCE: float = 1e-6

# Atmospheric Properties
const SUTHERLAND_TEMP: float = 110.4  # K (for viscosity calculation)
const SUTHERLAND_REF_TEMP: float = 273.15  # K
const SUTHERLAND_REF_VISC: float = 1.716e-5  # Pa·s

# Aerodynamic Reference Values (typical fighter)
const REFERENCE_DYNAMIC_PRESSURE: float = 10000.0  # Pa (typical cruise)
const REFERENCE_MACH: float = 0.8
const REFERENCE_REYNOLDS: float = 1e7

# Safety Limits
const MAX_SAFE_VELOCITY: float = 1000.0  # m/s (~Mach 3)
const MAX_SAFE_ANGULAR_VEL: float = 10.0  # rad/s
const MIN_SAFE_ALTITUDE: float = -100.0  # meters (below sea level)
const MAX_SAFE_ALTITUDE: float = 30000.0  # meters
