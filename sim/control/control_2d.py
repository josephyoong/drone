import math

class Control:
    def __init__(
        self,
        mass,
        gravity,
        max_thrust,
        control_dt,
        y_velocity_kp,
        y_velocity_ki,
        y_velocity_kd,
        x_velocity_kp,
        x_velocity_ki,
        x_velocity_kd,
        angular_velocity_kp,
        angular_velocity_ki,
        angular_velocity_kd,
        drone_length,
    ):
        self.mass = mass
        self.gravity = gravity
        self.max_thrust = max_thrust
        self.control_dt = control_dt
        self.y_velocity_gains = (y_velocity_kp, y_velocity_ki, y_velocity_kd)
        self.x_velocity_gains = (x_velocity_kp, x_velocity_ki, x_velocity_kd)
        self.angular_velocity_gains = (
            angular_velocity_kp,
            angular_velocity_ki,
            angular_velocity_kd,
        )
        self.drone_length = drone_length
        self.reset()

    def reset(self):
        self.y_integral_error = 0.0
        self.previous_y_error = 0.0
        self.x_integral_error = 0.0
        self.previous_x_error = 0.0
        self.angle_integral_error = 0.0
        self.previous_angle_error = 0.0

    def control(
        self,
        x_velocity,
        y_velocity,
        target_x_velocity,
        target_y_velocity,
        angle,
    ):
        # target velocities and estimated velocities -> target x and y thrusts
        k_p, k_i, k_d = self.y_velocity_gains
        y_error = target_y_velocity - y_velocity
        self.y_integral_error += y_error * self.control_dt
        y_derivative_error = (y_error - self.previous_y_error) / self.control_dt
        self.previous_y_error = y_error
        target_y_thrust = (self.mass * self.gravity + k_p * y_error + k_i * self.y_integral_error + k_d * y_derivative_error)

        k_p, k_i, k_d = self.x_velocity_gains
        x_error = target_x_velocity - x_velocity
        self.x_integral_error += x_error * self.control_dt
        x_derivative_error = (x_error - self.previous_x_error) / self.control_dt
        self.previous_x_error = x_error
        target_x_thrust = k_p * x_error + k_i * self.x_integral_error + k_d * x_derivative_error

        # target x and y thrusts -> target angle and total thrust (left + right)
        target_angle = -math.atan2(target_x_thrust, target_y_thrust)
        target_thrust = math.sqrt(target_x_thrust ** 2 + target_y_thrust ** 2)

        # angle controller: target angle and total thrust -> target torque
        angle_error = target_angle - angle
        k_p, k_i, k_d = self.angular_velocity_gains
        self.angle_integral_error += angle_error * self.control_dt
        angle_derivative_error = (angle_error - self.previous_angle_error) / self.control_dt
        self.previous_angle_error = angle_error
        target_torque = (k_p * angle_error + k_i * self.angle_integral_error + k_d * angle_derivative_error)

        # motor mixer: target torque and total thrust -> left and right thrusts
        left_target_thrust = (target_thrust / 2) - (target_torque / (2 * self.drone_length))
        right_target_thrust = (target_thrust / 2) + (target_torque / (2 * self.drone_length))

        return (
            max(0, min(self.max_thrust / 2, left_target_thrust)),
            max(0, min(self.max_thrust / 2, right_target_thrust)),
        )