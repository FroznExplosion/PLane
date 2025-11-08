class_name MathUtils
extends Object

## Mathematical Utilities for Flight Dynamics
## Vector operations, quaternions, coordinate transforms, interpolation

# Vector Operations
static func vector_is_valid(v: Vector3) -> bool:
	return v.is_finite() and v.length() < 1e10

static func safe_normalize(v: Vector3) -> Vector3:
	var vector_length = v.length()
	if vector_length < 1e-10:
		return Vector3.ZERO
	return v / vector_length

static func clamp_vector(v: Vector3, min_val: float, max_val: float) -> Vector3:
	return Vector3(
		clamp(v.x, min_val, max_val),
		clamp(v.y, min_val, max_val),
		clamp(v.z, min_val, max_val)
	)

static func vector_abs(v: Vector3) -> Vector3:
	return Vector3(abs(v.x), abs(v.y), abs(v.z))

# Quaternion Operations
static func quaternion_is_valid(q: Quaternion) -> bool:
	return q.is_finite() and abs(q.length() - 1.0) < 0.01

static func safe_normalize_quaternion(q: Quaternion) -> Quaternion:
	var quat_length = q.length()
	if quat_length < 0.001:
		return Quaternion.IDENTITY
	return q.normalized()

static func quaternion_from_angular_velocity(angular_vel: Vector3, dt: float) -> Quaternion:
	# Convert angular velocity to quaternion rotation
	var angle = angular_vel.length() * dt
	if angle < 1e-10:
		return Quaternion.IDENTITY
	var axis = angular_vel.normalized()
	return Quaternion(axis, angle)

# Coordinate System Transforms
static func body_to_wind(vector_body: Vector3, alpha: float, beta: float) -> Vector3:
	# Transform from body frame to wind frame
	# Wind frame: X forward (velocity), Y right, Z down
	var ca = cos(alpha)
	var sa = sin(alpha)
	var cb = cos(beta)
	var sb = sin(beta)

	return Vector3(
		ca * cb * vector_body.x + sb * vector_body.y + sa * cb * vector_body.z,
		-ca * sb * vector_body.x + cb * vector_body.y - sa * sb * vector_body.z,
		-sa * vector_body.x + ca * vector_body.z
	)

static func wind_to_body(vector_wind: Vector3, alpha: float, beta: float) -> Vector3:
	# Transform from wind frame to body frame
	var ca = cos(alpha)
	var sa = sin(alpha)
	var cb = cos(beta)
	var sb = sin(beta)

	return Vector3(
		ca * cb * vector_wind.x - ca * sb * vector_wind.y - sa * vector_wind.z,
		sb * vector_wind.x + cb * vector_wind.y,
		sa * cb * vector_wind.x - sa * sb * vector_wind.y + ca * vector_wind.z
	)

static func stability_to_body(lift: float, drag: float, side_force: float, alpha: float, _beta: float) -> Vector3:
	# Transform from stability axes to JSBSim body frame
	# JSBSim Body: X=forward, Y=right, Z=down
	# Drag always opposes velocity, Lift perpendicular to velocity
	var ca = cos(alpha)
	var sa = sin(alpha)

	# Simple 2D rotation in X-Z plane (ignoring sideslip for now)
	# X_body = -drag*cos(alpha) + lift*sin(alpha)
	# Z_body = -lift*cos(alpha) - drag*sin(alpha)
	return Vector3(
		-drag * ca + lift * sa,  # Forward: drag opposes, lift helps when nose up
		side_force,              # Side force (lateral)
		-lift * ca - drag * sa   # Down: lift opposes gravity, drag pulls down when nose up
	)

# Interpolation Functions
static func linear_interpolate(x: float, x_array: PackedFloat32Array, y_array: PackedFloat32Array) -> float:
	var n = x_array.size()
	if n == 0:
		return 0.0
	if n == 1 or x <= x_array[0]:
		return y_array[0]
	if x >= x_array[n-1]:
		return y_array[n-1]

	# Binary search
	var low = 0
	var high = n - 1
	while high - low > 1:
		var mid = int((low + high) / 2.0)  # Integer division
		if x_array[mid] > x:
			high = mid
		else:
			low = mid

	# Linear interpolation
	var t = (x - x_array[low]) / (x_array[high] - x_array[low])
	return y_array[low] + t * (y_array[high] - y_array[low])

static func bilinear_interpolate(x: float, y: float,
								 x_array: PackedFloat32Array,
								 y_array: PackedFloat32Array,
								 z_matrix: Array) -> float:
	# 2D interpolation for tables like CL(alpha, mach)
	var nx = x_array.size()
	var ny = y_array.size()

	if nx == 0 or ny == 0:
		return 0.0

	# Find x brackets
	var x_low = 0
	var x_high = nx - 1
	if x > x_array[0] and x < x_array[nx-1]:
		for i in range(nx - 1):
			if x >= x_array[i] and x <= x_array[i+1]:
				x_low = i
				x_high = i + 1
				break

	# Find y brackets
	var y_low = 0
	var y_high = ny - 1
	if y > y_array[0] and y < y_array[ny-1]:
		for j in range(ny - 1):
			if y >= y_array[j] and y <= y_array[j+1]:
				y_low = j
				y_high = j + 1
				break

	# Interpolation weights
	var tx = 0.0 if x_high == x_low else (x - x_array[x_low]) / (x_array[x_high] - x_array[x_low])
	var ty = 0.0 if y_high == y_low else (y - y_array[y_low]) / (y_array[y_high] - y_array[y_low])

	# Bilinear interpolation
	var z00 = z_matrix[x_low][y_low]
	var z10 = z_matrix[x_high][y_low]
	var z01 = z_matrix[x_low][y_high]
	var z11 = z_matrix[x_high][y_high]

	var z0 = z00 + tx * (z10 - z00)
	var z1 = z01 + tx * (z11 - z01)

	return z0 + ty * (z1 - z0)

# Angle Utilities
static func wrap_angle(angle: float) -> float:
	# Wrap angle to [-PI, PI]
	while angle > PI:
		angle -= TAU
	while angle < -PI:
		angle += TAU
	return angle

static func angle_difference(angle1: float, angle2: float) -> float:
	# Shortest angular distance from angle1 to angle2
	var diff = angle2 - angle1
	return wrap_angle(diff)

# Numerical Derivatives
static func numerical_derivative(func_values: PackedFloat32Array, dt: float) -> PackedFloat32Array:
	var n = func_values.size()
	var derivatives = PackedFloat32Array()
	derivatives.resize(n)

	if n < 2:
		return derivatives

	# Forward difference for first point
	derivatives[0] = (func_values[1] - func_values[0]) / dt

	# Central difference for middle points
	for i in range(1, n-1):
		derivatives[i] = (func_values[i+1] - func_values[i-1]) / (2.0 * dt)

	# Backward difference for last point
	derivatives[n-1] = (func_values[n-1] - func_values[n-2]) / dt

	return derivatives

# Runge-Kutta 4th Order Integration
static func rk4_step(state: Vector3, derivative_func: Callable, dt: float, params: Dictionary) -> Vector3:
	# RK4 integration step for a 3D state vector
	var k1 = derivative_func.call(state, params) * dt
	var k2 = derivative_func.call(state + k1 * 0.5, params) * dt
	var k3 = derivative_func.call(state + k2 * 0.5, params) * dt
	var k4 = derivative_func.call(state + k3, params) * dt

	return state + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0

# Matrix Operations (3x3)
static func mat3_multiply_vec3(mat: Basis, vec: Vector3) -> Vector3:
	return mat * vec

static func mat3_transpose(mat: Basis) -> Basis:
	return mat.transposed()

static func mat3_inverse(mat: Basis) -> Basis:
	return mat.inverse()

# Saturation Function (smooth limiting)
static func smooth_saturate(x: float, limit: float, smoothness: float = 0.1) -> float:
	# Smooth saturation function using tanh
	if limit <= 0:
		return x
	var normalized = x / limit
	return limit * tanh(normalized / smoothness) * smoothness

# Sign function that returns 0 for 0
static func sign_or_zero(x: float) -> float:
	if abs(x) < 1e-10:
		return 0.0
	return sign(x)
