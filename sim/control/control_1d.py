class Control:
    def __init__(
        self,
        mass,
        gravity,
        max_thrust,
        control_dt,
        height_kp,
        height_ki,
        height_kd,
        velocity_kp,
        velocity_ki,
        velocity_kd,
    ):
        self.mass = mass
        self.gravity = gravity
        self.max_thrust = max_thrust
        self.control_dt = control_dt
        self.height_gains = (height_kp, height_ki, height_kd)
        self.velocity_gains = (velocity_kp, velocity_ki, velocity_kd)
        self.reset()

    def reset(self):
        self.integral_error = 0.0
        self.previous_error = 0.0

    def control(self, height, target_height, velocity, target_velocity):
        if target_velocity == 0:
            k_p, k_i, k_d = self.height_gains
            error = target_height - height
        else:
            k_p, k_i, k_d = self.velocity_gains
            error = target_velocity - velocity

        self.integral_error += error * self.control_dt
        derivative_error = (error - self.previous_error) / self.control_dt
        self.previous_error = error

        thrust = (
            self.mass * self.gravity
            + k_p * error
            + k_i * self.integral_error
            + k_d * derivative_error
        )
        return max(0, min(self.max_thrust, thrust))