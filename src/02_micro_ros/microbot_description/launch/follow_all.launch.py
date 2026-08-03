# AIR26 — one-command AprilTag following (THIS DRIVES THE MOTORS).
#
# Brings up the camera bridge + TF/odometry/RViz (hardware.launch.py) AND the PID
# AprilTag follower in a single launch, so following is one command instead of two.
#
# The micro-ROS Agent still runs separately, because it lives in a different workspace:
#     cd ~/microros_ws && source install/setup.bash
#     ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
#
# then:
#     ros2 launch microbot_description follow_all.launch.py
#
# Useful overrides:
#     follower:=none            just the camera + RViz, no motion at all
#     follower:=nopid           use the tuning-free bang-bang follower instead
#     use_rviz:=false           headless
#     safe_distance:=0.30       hold closer
#     angular_max:=0.0          straight-only (no steering)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('microbot_description')
    launch_dir = os.path.join(share, 'launch')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true',
                                        choices=['true', 'false']))
    ld.add_action(DeclareLaunchArgument('cam_url',
                                        default_value='http://192.168.0.117/stream'))
    # which follower drives the motors: the PID port, the tuning-free one, or nothing
    ld.add_action(DeclareLaunchArgument('follower', default_value='pid',
                                        choices=['pid', 'nopid', 'none']))
    # passed through to the PID follower (the args it does not declare are ignored)
    ld.add_action(DeclareLaunchArgument('safe_distance', default_value='0.45'))
    ld.add_action(DeclareLaunchArgument('angular_max', default_value='0.15'))
    ld.add_action(DeclareLaunchArgument('tag_size', default_value='0.08'))

    # camera bridge + AprilTag overlay + TF + wheel odometry + RViz
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'hardware.launch.py')),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz'),
                          'cam_url': LaunchConfiguration('cam_url')}.items()))

    # PID follower (default)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'follow_pid.launch.py')),
        launch_arguments={'safe_distance': LaunchConfiguration('safe_distance'),
                          'angular_max': LaunchConfiguration('angular_max'),
                          'tag_size': LaunchConfiguration('tag_size')}.items(),
        condition=LaunchConfigurationEquals('follower', 'pid')))

    # tuning-free bang-bang follower
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'follow.launch.py')),
        launch_arguments={'tag_size': LaunchConfiguration('tag_size')}.items(),
        condition=LaunchConfigurationEquals('follower', 'nopid')))

    return ld
