# AIR26 Workshop 02 — the obstacle-avoider behaviours.
# Run this alongside a TF/robot source:
#   - sim:  ros2 launch microbot_sim mujoco.launch.py        (publishes odom + TF)
#   - real: ros2 launch microbot_description hardware.launch.py  (URDF TF via robot_state_publisher)
#
#   ros2 launch microbot_behaviors behaviors.launch.py
#   ros2 service call /set_behavior microbot_interfaces/srv/SetBehavior "{behavior: 3}"
#
# NOTE: the ultrasonic/camera frames (base_link -> us_front/us_left/us_right, camera_link)
# come from the URDF via robot_state_publisher (hardware.launch.py). Don't also publish them
# with static_transform_publisher here, or you'll get two conflicting /tf_static sources.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # the brain: random walk + B1/B2 (pub-sub) + B3 client + /set_behavior
        Node(package='microbot_behaviors', executable='behavior_manager', output='screen'),
        # the B3 service + action server
        Node(package='microbot_behaviors', executable='obstacle_services', output='screen'),
    ])
