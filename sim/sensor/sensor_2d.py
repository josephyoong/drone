import numpy as np

class Sensor:
	def __init__(self, x_pos_stddev, y_pos_stddev, x_velocity_stddev, y_velocity_stddev, angle_stddev, angular_velocity_stddev):
		self.x_pos_stddev = x_pos_stddev
		self.y_pos_stddev = y_pos_stddev
		self.x_velocity_stddev = x_velocity_stddev
		self.y_velocity_stddev = y_velocity_stddev
		self.angle_stddev = angle_stddev
		self.angular_velocity_stddev = angular_velocity_stddev

	def measure(self, x_pos, y_pos, x_velocity, y_velocity, angle, angular_velocity):
		x_pos += np.random.normal(0, self.x_pos_stddev)
		y_pos += np.random.normal(0, self.y_pos_stddev)
		x_velocity += np.random.normal(0, self.x_velocity_stddev)
		y_velocity += np.random.normal(0, self.y_velocity_stddev)
		angle += np.random.normal(0, self.angle_stddev)
		angular_velocity += np.random.normal(0, self.angular_velocity_stddev)
		return x_pos, y_pos, x_velocity, y_velocity, angle, angular_velocity