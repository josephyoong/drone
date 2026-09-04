import matplotlib.pyplot as plt
from control.control_2d import Control
from dynamics.dynamics_2d import Dynamics
from estimation.estimation_2d import Estimation
from motor.motor_2d import Motor
from receiver.receiver_2d import Receiver
from sensor.sensor_2d import Sensor

# ==============================================================================
# SETUP
# ==============================================================================

# ==============================================================================
# constants and variables
# ==============================================================================
x_pos = 0.0
y_pos = 0.0
x_velocity = 0.0
y_velocity = 0.0
x_acceleration = 0.0
y_acceleration = 0.0
angular_velocity = 0.0
target_x_velocity = 0.0
drone_length = 1.0
angle = 0.0
m = 1
g = 9.81
drag_coefficient = 0.2
target_thrust = 0
thrust = 0
max_thrust = 1.5*g
y_velocity_kp = 1.0
y_velocity_ki = 0.05
y_velocity_kd = 0.5
x_velocity_kp = 1.0
x_velocity_ki = 0.05
x_velocity_kd = 0.5
angular_velocity_kp = 1.0
angular_velocity_ki = 0.05
angular_velocity_kd = 0.5
target_y_velocity = 1
x_pos_stddev = 0.1
y_pos_stddev = 0.1
x_velocity_stddev = 0.1
y_velocity_stddev = 0.1
angle_stddev = 0.01
angular_velocity_stddev = 0.01
dt = 0.1
control_dt = 0.2
time = 0.0
next_control_time = 0.0
x_positions = [x_pos]
y_positions = [y_pos]
dynamics = Dynamics(
    m, g, drag_coefficient, drone_length, angle, angular_velocity
)
sensor = Sensor(
    x_pos_stddev,
    y_pos_stddev,
    x_velocity_stddev,
    y_velocity_stddev,
    angle_stddev,
    angular_velocity_stddev,
)
estimation = Estimation(
    x_pos_stddev,
    y_pos_stddev,
    x_velocity_stddev,
    y_velocity_stddev,
    angle_stddev,
    angular_velocity_stddev,
)
motor = Motor()
left_motor = Motor()
right_motor = Motor()
receiver = Receiver()
controller = Control(
    m,
    g,
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
)
# ==============================================================================





# ==============================================================================
# setup graph
# ==============================================================================
figure, (graph, info) = plt.subplots(
    1, 2, figsize=(10, 5), gridspec_kw={"width_ratios": [4, 1]}
)
figure.canvas.manager.set_window_title("Drone Simulation")
point, = graph.plot(
    [x_positions[-1]],
    [y_positions[-1]],
    marker="o",
    linestyle="None",
    color="blue",
)
motors, = graph.plot(
    [x_pos - drone_length / 2, x_pos + drone_length / 2],
    [y_pos, y_pos],
    marker="x",
    linestyle="-",
    color="black",
)

graph.set_title("Drone Position")
graph.set_xlabel("X position (meters)")
graph.set_ylabel("Y position (meters)")
graph.grid(True)
info.axis("off")
info_text = info.text(0.0, 0.95, "", fontsize=12, va="top")
plt.show(block=False)
plt.pause(0.001)
# ==============================================================================





# ==============================================================================
# LOOP
# ==============================================================================
while plt.fignum_exists(figure.number):
    # ==============================================================================
    # sensor: true state -> sensor measurement
    # ==============================================================================
    measured_x_pos, measured_y_pos, measured_x_velocity, measured_y_velocity, measured_angle, measured_angular_velocity = sensor.measure(
        dynamics.x_pos,
        dynamics.y_pos,
        dynamics.x_velocity,
        dynamics.y_velocity,
        dynamics.angle,
        dynamics.angular_velocity,
    )
    # ==============================================================================





    # ==============================================================================
    # state estimation: sensor measurement -> state estimation
    # ==============================================================================
    (
        estimated_x_velocity,
        estimated_y_velocity,
        estimated_angle,
    ) = estimation.estimate(
        measured_x_pos,
        measured_y_pos,
        measured_x_velocity,
        measured_y_velocity,
        measured_angle,
        measured_angular_velocity,
    )
    # ==============================================================================





    # ==============================================================================
    # reciever: user input commands -> target height and velocity
    # ==============================================================================
    target_y_velocity, target_x_velocity, target_changed = receiver.receive(
        target_y_velocity,
        target_x_velocity,
    )
    if target_changed:
        controller.reset()
    # ==============================================================================





    # ==============================================================================
    # control: state estimation and target velocities -> target thrust
    # ==============================================================================
    if time >= next_control_time:
        left_target_thrust, right_target_thrust = controller.control(
            estimated_x_velocity,
            estimated_y_velocity,
            target_x_velocity,
            target_y_velocity,
            estimated_angle,
        )
        next_control_time += control_dt
    # ==============================================================================





    # ==============================================================================
    # motor: target thrust -> actual thrust
    # ==============================================================================
    left_thrust = left_motor.update(left_target_thrust)
    right_thrust = right_motor.update(right_target_thrust)
    thrust = left_thrust + right_thrust
    # ==============================================================================





    # ==============================================================================
    # dynamics: actual thrust -> new state
    # ==============================================================================
    (
        x_pos,
        y_pos,
        x_velocity,
        y_velocity,
        angle,
        angular_velocity,
        x_acceleration,
        y_acceleration,
        angular_acceleration,
        left_motor_x_pos,
        left_motor_y_pos,
        right_motor_x_pos,
        right_motor_y_pos,
    ) = dynamics.update(left_thrust, right_thrust, dt)
    # ==============================================================================





    # ==============================================================================
    # time
    # ==============================================================================
    time += dt

    # delay
    plt.pause(dt)
    # ==============================================================================





    # ==============================================================================
    # graph state
    # ==============================================================================
    x_positions.append(x_pos)
    y_positions.append(y_pos)
    point.set_data([x_pos], [y_pos])
    motors.set_data(
        [left_motor_x_pos, x_pos, right_motor_x_pos],
        [left_motor_y_pos, y_pos, right_motor_y_pos],
    )
    graph.relim()
    graph.autoscale_view(scaley=False)
    graph.set_xlim(-5.0, 5.0)
    graph.set_ylim(0.0, 20.0)
    info_text.set_text(
        f"Target x velocity = {target_x_velocity:.2f} m/s\n"
        f"Target y velocity = {target_y_velocity:.2f} m/s\n"
        f"\n"
        f"Time = {time:.1f} s\n"
        f"X position = {x_pos:.2f} m\n"
        f"Y position = {y_pos:.2f} m\n"
        f"X velocity = {x_velocity:.2f} m/s\n"
        f"Y velocity = {y_velocity:.2f} m/s\n"
        f"X acceleration = {x_acceleration:.2f} m/s^2\n"
        f"Y acceleration = {y_acceleration:.2f} m/s^2\n"
        f"Angle = {dynamics.angle:.2f} rad\n"
        f"Angular velocity = {dynamics.angular_velocity:.2f} rad/s\n"
        f"Thrust = {thrust:.2f} N"
    )
    figure.canvas.draw()
    figure.canvas.flush_events()
    # ==============================================================================
# ==============================================================================