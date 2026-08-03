# AIR26 — AprilTag following with a PID controller (THIS COMMANDS THE MOTORS).
#
# Mechanism ported from yenode/raju_bot_ws (src/bot_detection/apriltag_node.py):
# angular PID on the tag's pixel offset from image centre, linear PID on
# (distance - safe_distance) with a deadband, integrals reset when the tag is lost.
#
# The tuning-free alternative is still there: follow.launch.py (no PID, bang-bang + arcs).
#
#   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888   # T1: agent
#   ros2 launch microbot_description hardware.launch.py         # T2: camera feed + RViz
#   ros2 launch microbot_description follow_pid.launch.py       # T3: PID following
#
# Tune live, e.g.:
#   ros2 launch microbot_description follow_pid.launch.py angular_kp:=0.005
#   ros2 launch microbot_description follow_pid.launch.py safe_distance:=0.30
#   ros2 launch microbot_description follow_pid.launch.py angular_max:=0.0   # straight only
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

FLOAT_ARGS = [
    # --- tag / camera ---
    ('tag_size', '0.08'),        # OUR printed tag: outer black border, metres
    ('fx', '820.0'),             # MEASURED on this camera (~22 deg FOV), not the ~250 guess
    ('fy', '820.0'),
    # --- setpoint ---
    ('safe_distance', '0.45'),   # STOP distance; an 80 mm tag fills the frame below ~0.35 m
    ('dist_deadband', '0.05'),   # reference value: stops micro-oscillation at the setpoint
    ('angular_deadband_px', '12.0'),  # centred-enough band; stops endless forward creep
    # --- linear PID (error in metres) ---
    ('linear_kp', '0.6'),
    ('linear_ki', '0.05'),
    ('linear_kd', '0.1'),
    ('linear_max', '0.04'),      # our cruise speed cap (m/s)
    ('linear_min', '-0.03'),     # our reverse cap
    # --- angular PID (error in PIXELS from image centre) ---
    ('angular_kp', '0.003'),
    ('angular_ki', '0.0005'),
    ('angular_kd', '0.001'),
    ('angular_max', '0.15'),     # rad/s cap; 0.0 = straight only
    # --- real-drivetrain limits (no equivalent in the sim reference) ---
    ('turn_cruise', '0.11'),     # keep rolling while steering (keeps the tag in frame)
    ('wheel_min', '0.0'),        # 0 = firmware velocity PID removes the PWM deadband
    ('wheel_sep', '0.251'),
    ('fw_max_lin', '0.25'),
    ('straight_bias', '0.0'),    # 0 = firmware velocity PID already corrects the drift
]


def generate_launch_description():
    ld = LaunchDescription()
    for name, default in FLOAT_ARGS:
        ld.add_action(DeclareLaunchArgument(name, default_value=default))
    ld.add_action(DeclareLaunchArgument('tag_id', default_value='12'))
    ld.add_action(DeclareLaunchArgument('arc_only', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_pnp', default_value='true'))

    params = {n: ParameterValue(LaunchConfiguration(n), value_type=float)
              for n, _ in FLOAT_ARGS}
    params['tag_id'] = ParameterValue(LaunchConfiguration('tag_id'), value_type=int)
    params['arc_only'] = ParameterValue(LaunchConfiguration('arc_only'), value_type=bool)
    params['use_pnp'] = ParameterValue(LaunchConfiguration('use_pnp'), value_type=bool)

    ld.add_action(Node(
        package='microbot_description', executable='apriltag_follower_pid',
        output='screen', parameters=[params]))
    return ld
