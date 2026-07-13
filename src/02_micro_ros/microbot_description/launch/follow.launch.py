# AIR26 — AprilTag following (this COMMANDS THE MOTORS). Kept separate from the viz launch
# on purpose. Run it after the agent + camera feed are already up:
#
#   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888        # T1: agent
#   ros2 launch microbot_description hardware.launch.py              # T2: camera feed + RViz
#   ros2 launch microbot_description follow.launch.py                # T3: start following
#
# The follower reads /camera/image_raw and publishes /cmd_vel; it stops the vehicle in place
# whenever the tag leaves the frame. Tune on the fly, e.g.:
#   ros2 launch microbot_description follow.launch.py desired_distance:=0.6 max_linear:=0.2
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('desired_distance', default_value='0.2'))  # follow distance (smaller = closer)
    ld.add_action(DeclareLaunchArgument('forward_speed', default_value='0.12'))  # cruise (m/s)
    ld.add_action(DeclareLaunchArgument('max_turn', default_value='0.0'))        # 0 = straight only (no turning); set 0.6 to re-enable arcs
    ld.add_action(DeclareLaunchArgument('straight_bias', default_value='0.03'))  # >0 nudges left to cancel a right drift on straight commands
    ld.add_action(DeclareLaunchArgument('wheel_min', default_value='0.28'))      # motor deadband (0..1)
    ld.add_action(DeclareLaunchArgument('center_deadband', default_value='0.35'))  # bigger = stay straight longer, turn only near the edges
    ld.add_action(DeclareLaunchArgument('tag_id', default_value='12'))

    ld.add_action(Node(
        package='microbot_description', executable='apriltag_follower', output='screen',
        parameters=[{
            'desired_distance': ParameterValue(LaunchConfiguration('desired_distance'),
                                               value_type=float),
            'forward_speed': ParameterValue(LaunchConfiguration('forward_speed'), value_type=float),
            'max_turn': ParameterValue(LaunchConfiguration('max_turn'), value_type=float),
            'straight_bias': ParameterValue(LaunchConfiguration('straight_bias'),
                                            value_type=float),
            'wheel_min': ParameterValue(LaunchConfiguration('wheel_min'), value_type=float),
            'center_deadband': ParameterValue(LaunchConfiguration('center_deadband'),
                                              value_type=float),
            'tag_id': ParameterValue(LaunchConfiguration('tag_id'), value_type=int),
        }]))
    return ld
