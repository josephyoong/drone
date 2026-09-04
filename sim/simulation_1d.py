import matplotlib.pyplot as plt
from control.control_1d import Control
from dynamics.dynamics_1d import Dynamics
from estimation.estimation_1d import Estimation
from motor.motor_1d import Motor
from receiver.receiver_1d import Receiver
from sensor.sensor_1d import Sensor

# ==============================================================================
# SETUP
# ==============================================================================

# ==============================================================================
# constants and variables
# ==============================================================================
height = 0.0
velocity = 0.0
acceleration = 0.0
m = 1
g = 9.81
drag_coefficient = 0.2
target_thrust = 0
thrust = 0
max_thrust = 1.5*g
height_kp = 2.0
height_ki = 0.1
height_kd = 1.0
velocity_kp = 1.0
velocity_ki = 0.05
velocity_kd = 0.5
target_height = 10
target_velocity = 1
dt = 0.1
control_dt = 0.2
time = 0.0
next_control_time = 0.0
times = [time]
heights = [height]
dynamics = Dynamics(m, g, drag_coefficient)
sensor = Sensor()
estimation = Estimation()
motor = Motor()
receiver = Receiver()
controller = Control(
    m,
    g,
    max_thrust,
    control_dt,
    height_kp,
    height_ki,
    height_kd,
    velocity_kp,
    velocity_ki,
    velocity_kd,
)
# ==============================================================================





# ==============================================================================
# setup graph
# ==============================================================================
figure, (graph, info) = plt.subplots(
    1, 2, figsize=(10, 5), gridspec_kw={"width_ratios": [4, 1]}
)
figure.canvas.manager.set_window_title("Drone Simulation")
line, = graph.plot(times, heights, color="blue")
graph.set_title("Drone Height")
graph.set_xlabel("Time (seconds)")
graph.set_ylabel("Height (meters)")
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
    measured_height, measured_velocity = sensor.measure(
        dynamics.height, dynamics.velocity
    )
    # ==============================================================================





    # ==============================================================================
    # state estimation: sensor measurement -> state estimation
    # ==============================================================================
    estimated_height, estimated_velocity = estimation.estimate(
        measured_height, measured_velocity
    )
    # ==============================================================================





    # ==============================================================================
    # reciever: user input commands -> target height and velocity
    # ==============================================================================
    target_height, target_velocity, target_changed = receiver.receive(
        estimated_height, target_height, target_velocity
    )
    if target_changed:
        controller.reset()
    # ==============================================================================





    # ==============================================================================
    # control: state estimation and target -> target thrust
    # ==============================================================================
    if time >= next_control_time:
        target_thrust = controller.control(
            estimated_height, target_height, estimated_velocity, target_velocity
        )
        next_control_time += control_dt

    control_mode = "Height control" if target_velocity == 0 else "Velocity control"
    # ==============================================================================





    # ==============================================================================
    # motor: target thrust -> actual thrust
    # ==============================================================================
    thrust = motor.update(target_thrust)
    # ==============================================================================





    # ==============================================================================
    # dynamics: actual thrust -> new state
    # ==============================================================================
    height, velocity, acceleration = dynamics.update(thrust, dt)
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
    times.append(time)
    heights.append(height)
    line.set_data(times, heights)
    graph.set_xlim(max(0.0, time - 20.0), max(20.0, time))
    graph.set_ylim(0.0, max(10.0, max(heights) * 1.2))
    info_text.set_text(
        f"Mode = {control_mode}\n"
        f"Target height = {target_height:.2f} m\n"
        f"Target velocity = {target_velocity:.2f} m/s\n"
        f"\n"
        f"Time = {time:.1f} s\n"
        f"Height = {height:.2f} m\n"
        f"Velocity = {velocity:.2f} m/s\n"
        f"Acceleration = {acceleration:.2f} m/s^2\n"
        f"Thrust = {thrust:.2f} N"
    )
    figure.canvas.draw()
    figure.canvas.flush_events()
    # ==============================================================================
# ==============================================================================