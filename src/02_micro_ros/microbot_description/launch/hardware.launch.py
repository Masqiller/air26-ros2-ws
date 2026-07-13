# AIR26 Workshop 02 — real-hardware view (no sim).
#
# The ESP32 bot (via the micro-ROS Agent) publishes /ultrasonic/{front,left,right} and
# /camera/{mean_color,mean_intensity}. This launch brings up ONLY the ROS-side
# visualization for the real robot:
#   - robot_state_publisher : URDF -> TF (sensor + camera frames on /tf_static)
#   - joint_state_publisher : zero wheel angles so the RobotModel renders whole
#   - static odom->base_link: the real bot has no odometry, so pin it to the origin
#   - camera_viz            : paint /camera/* aggregates as an RViz marker
#   - camera_stream         : ESP32-CAM MJPEG (HTTP/WiFi) -> /camera/image_raw for RViz
#   - rviz2                 : the range cones + robot + camera swatch
#
# The camera VIDEO is plain HTTP over WiFi, independent of micro-ROS, so camera_stream
# connects straight to the board (cam_url) and needs NO micro-ROS agent of its own. Only
# the vehicle needs the agent:
#   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
#   ros2 launch microbot_description hardware.launch.py
#
# If the cam's IP changes, pass it in:  cam_url:=http://<cam-ip>/stream
# (or set cam_url:='' to auto-discover via /camera/ip — that route needs the cam's own
#  micro-ROS agent running on port 8889).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc = get_package_share_directory('microbot_description')
    xacro_file = os.path.join(desc, 'urdf', 'microbot.urdf.xacro')
    rviz_cfg = os.path.join(desc, 'rviz', 'microbot.rviz')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true',
                                        choices=['true', 'false']))
    # ESP32-CAM MJPEG stream URL. Direct HTTP -> no dependency on the cam's micro-ROS/agent.
    # Set to '' to instead auto-discover the IP from /camera/ip (needs the 8889 cam agent).
    ld.add_action(DeclareLaunchArgument('cam_url',
                                        default_value='http://192.168.0.117/stream'))

    # URDF -> TF. Fixed joints (the 3 ultrasonics + camera) go on /tf_static, so the
    # zero-stamped Range msgs from micro-ROS still transform cleanly.
    ld.add_action(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]))

    # wheels are 'continuous' joints -> publish zero angles so the RobotModel shows them
    # (self-contained stand-in for joint_state_publisher).
    ld.add_action(Node(
        package='microbot_description', executable='joint_states', output='screen'))

    # wheel-encoder odometry: /joint_states -> /odom + the odom->base_link TF, so the robot
    # drives around in RViz. Replaces the old static odom->base_link now that we have encoders.
    # (Publishes identity at startup, so base_link is placed even before the bot moves.)
    ld.add_action(Node(
        package='microbot_description', executable='wheel_odometry', output='screen'))

    # turn the camera's mean colour / brightness into a visible RViz marker.
    ld.add_action(Node(
        package='microbot_description', executable='camera_viz', output='screen'))

    # bridge the ESP32-CAM MJPEG HTTP stream -> /camera/image_raw for the RViz Image display.
    # url set -> connect straight to the board (no /camera/ip, no cam agent needed).
    ld.add_action(Node(
        package='microbot_description', executable='camera_stream', output='screen',
        parameters=[{'url': LaunchConfiguration('cam_url')}]))

    # detect AprilTags in the stream -> /apriltag/image (annotated) for the RViz Image display.
    # reads the ROS topic (not the board HTTP), so it coexists with the camera feed.
    ld.add_action(Node(
        package='microbot_description', executable='apriltag_detector', output='screen'))

    ld.add_action(Node(
        package='rviz2', executable='rviz2', output='log', arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('use_rviz'))))

    return ld
