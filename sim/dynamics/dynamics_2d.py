import math

class Dynamics:
	def __init__(
		self, mass, gravity, drag_coefficient, drone_length, angle, angular_velocity
	):
		self.mass = mass
		self.gravity = gravity
		self.drag_coefficient = drag_coefficient
		self.drone_length = drone_length
		self.angle = angle
		self.angular_velocity = angular_velocity
		self.x_pos = 0.0
		self.y_pos = 0.0
		self.x_velocity = 0.0
		self.y_velocity = 0.0
		self.moment_of_inertia = (1.0 / 12.0) * self.mass * self.drone_length ** 2

	def update(self, left_thrust, right_thrust, dt):
		total_thrust = left_thrust + right_thrust
		x_drag_force = self.drag_coefficient * self.x_velocity
		y_drag_force = self.drag_coefficient * self.y_velocity
		x_thrust = total_thrust * math.sin(-self.angle)
		y_thrust = total_thrust * math.cos(self.angle)
		x_acceleration = (x_thrust - x_drag_force) / self.mass
		y_acceleration = (y_thrust - self.mass * self.gravity - y_drag_force) / self.mass
		half_length = self.drone_length / 2
		torque = (right_thrust - left_thrust) * half_length
		angular_acceleration = torque / self.moment_of_inertia

		self.angular_velocity += angular_acceleration * dt
		self.x_velocity += x_acceleration * dt
		self.y_velocity += y_acceleration * dt
		self.angle += self.angular_velocity * dt
		self.x_pos += self.x_velocity * dt
		self.y_pos += self.y_velocity * dt

		if self.y_pos <= 0.0:
			self.y_pos = 0.0
			self.y_velocity = 0.0
			self.angular_velocity = 0.0  # optional: stop spin on ground contact

		left_motor_x_pos = self.x_pos - half_length * math.cos(self.angle)
		left_motor_y_pos = self.y_pos - half_length * math.sin(self.angle)
		right_motor_x_pos = self.x_pos + half_length * math.cos(self.angle)
		right_motor_y_pos = self.y_pos + half_length * math.sin(self.angle)

		return (
			self.x_pos,
			self.y_pos,
			self.x_velocity,
			self.y_velocity,
			self.angle,
			self.angular_velocity,
			x_acceleration,
			y_acceleration,
			angular_acceleration,
			left_motor_x_pos,
			left_motor_y_pos,
			right_motor_x_pos,
			right_motor_y_pos,
		)