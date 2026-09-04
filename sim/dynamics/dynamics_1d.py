class Dynamics:
	def __init__(self, mass, gravity, drag_coefficient):
		self.mass = mass
		self.gravity = gravity
		self.drag_coefficient = drag_coefficient
		self.height = 0.0
		self.velocity = 0.0

	def update(self, thrust, dt):
		drag_force = self.drag_coefficient * self.velocity
		acceleration = (
			thrust - self.mass * self.gravity - drag_force
		) / self.mass
		self.velocity += acceleration * dt
		self.height += self.velocity * dt

		if self.height <= 0.0:
			self.height = 0.0
			self.velocity = 0.0

		return self.height, self.velocity, acceleration
