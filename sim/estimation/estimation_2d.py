class Estimation:
	def __init__(self, x_pos_stddev, y_pos_stddev, x_velocity_stddev, y_velocity_stddev, angle_stddev, angular_velocity_stddev):
		self.x_pos_stddev = x_pos_stddev
		self.y_pos_stddev = y_pos_stddev
		self.x_velocity_stddev = x_velocity_stddev
		self.y_velocity_stddev = y_velocity_stddev
		self.angle_stddev = angle_stddev
		self.angular_velocity_stddev = angular_velocity_stddev

	def estimate(self, x_pos, y_pos, x_velocity, y_velocity, angle, angular_velocity):
		return x_velocity, y_velocity, angle
	