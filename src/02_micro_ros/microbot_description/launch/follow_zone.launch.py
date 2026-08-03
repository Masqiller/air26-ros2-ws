# AIR26 — ZONE steering test (NO PID). Deliberately simple, for debugging.
#
# The frame is split into LEFT | CENTRE | RIGHT. A dot is drawn on the tag's centre; if the
# dot lands in the left/right zone the vehicle rotates that way at a FIXED rate until the
# dot is back in the centre. No gains, nothing to tune.
#
#   mode:=align   (default) rotate ONLY — never drives forward/back. Start here.
#   mode:=follow  once alignment looks right, add the forward/hold/back distance rule.
#
#   ros2 launch microbot_description follow_zone.launch.py
#   ros2 launch microbot_description follow_zone.launch.py mode:=follow
#   ros2 launch microbot_description follow_zone.launch.py center_frac:=0.5   # wider centre
#   ros2 launch microbot_description follow_zone.launch.py turn_speed:=0.25   # gentler
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

FLOATS = [
    ('center_frac', '0.34'),     # centre zone = middle third of the frame width
    ('exit_ratio', '0.6'),       # hysteresis: stop turning inside 60% of the centre zone
    ('turn_speed', '0.15'),      # rad/s, fixed; slow pivots lean on the velocity PID
    ('turn_creep', '0.0'),       # forward speed while turning (0 = pivot in place)
    ('tag_size', '0.08'),        # metres, outer black border of the PRINTED tag
    ('fx', '820.0'),             # MEASURED on this camera (~22 deg FOV), not the ~250 guess
    ('safe_distance', '0.45'),   # STOP distance (mode=follow only)
    ('dist_band', '0.05'),       # tolerance: stop between 0.40 and 0.50 m
    ('forward_speed', '0.04'),   # m/s
    ('reverse_speed', '0.03'),
]


def generate_launch_description():
    ld = LaunchDescription()
    for n, d in FLOATS:
        ld.add_action(DeclareLaunchArgument(n, default_value=d))
    ld.add_action(DeclareLaunchArgument('mode', default_value='align',
                                        choices=['align', 'follow']))
    ld.add_action(DeclareLaunchArgument('tag_id', default_value='12'))

    params = {n: ParameterValue(LaunchConfiguration(n), value_type=float) for n, _ in FLOATS}
    params['mode'] = ParameterValue(LaunchConfiguration('mode'), value_type=str)
    params['tag_id'] = ParameterValue(LaunchConfiguration('tag_id'), value_type=int)

    ld.add_action(Node(
        package='microbot_description', executable='apriltag_follower_zone',
        output='screen', parameters=[params]))
    return ld
