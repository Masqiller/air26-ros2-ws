#!/usr/bin/env python3
"""wheel_odometry — differential-drive odometry from the ESP32-S3 wheel encoders.

Subscribes: /joint_states  (sensor_msgs/JointState) — wheel angles in radians from the
            encoders (base_back_left_wheel_joint / base_back_right_wheel_joint).
Publishes:  /odom          (nav_msgs/Odometry)
            odom -> base_link TF   (so the robot moves in RViz as you drive it)

Tailored from the reference EnhancedWheelOdometryNode: kept the differential-drive
kinematics, trimmed the Kalman filter + slip detection for a clean, easy-to-debug
baseline. Tune wheel_radius / track_width to YOUR robot for accurate distance + heading.

Params (defaults match the Platoon vehicle):
  wheel_radius 0.0325 m (65 mm wheel) | track_width 0.251 m
  left_wheel_joint  base_back_left_wheel_joint
  right_wheel_joint base_back_right_wheel_joint
  odom_frame odom | base_frame base_footprint
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.0325).value
        self.track_width = self.declare_parameter('track_width', 0.251).value
        # The firmware now applies ENC_L_SIGN/ENC_R_SIGN itself, so /joint_states angles
        # already INCREASE when the robot drives forward -> no flip needed here (+1).
        # (This used to be -1 to correct raw backward-counting encoders. If you ever flash
        # firmware that publishes raw ticks again, set this back to -1.)
        self.encoder_direction = float(self.declare_parameter('encoder_direction', 1.0).value)
        self.left_joint = self.declare_parameter('left_wheel_joint',
                                                 'base_back_left_wheel_joint').value
        self.right_joint = self.declare_parameter('right_wheel_joint',
                                                  'base_back_right_wheel_joint').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        # drive base_footprint, NOT base_link: the URDF authors base_link rotated +90 deg
        # yaw (CAD convention), so integrating into base_link would make the robot appear
        # to drive sideways in RViz. base_footprint's +X is the real forward direction.
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value

        # pose + velocity state
        self.x = self.y = self.theta = 0.0
        self.v = self.w = 0.0
        self.last_left = None
        self.last_right = None
        self.last_time = None

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_bc = TransformBroadcaster(self)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_timer(1.0 / 30.0, self.publish)     # steady TF/odom even between msgs
        self.get_logger().info(
            'wheel_odometry up: r=%.3f track=%.3f joints=[%s, %s] -> /odom + %s->%s'
            % (self.wheel_radius, self.track_width, self.left_joint, self.right_joint,
               self.odom_frame, self.base_frame))

    def on_joints(self, msg):
        # only our encoder messages carry these joint names; the URDF zero-publisher's
        # messages don't -> they raise ValueError here and are skipped.
        try:
            li = msg.name.index(self.left_joint)
            ri = msg.name.index(self.right_joint)
        except ValueError:
            return
        left = msg.position[li]
        right = msg.position[ri]
        now = self.get_clock().now()

        if self.last_left is None:
            self.last_left, self.last_right, self.last_time = left, right, now
            return
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        # wheel angle deltas (rad) -> ground distance (m); encoder_direction (-1) corrects
        # the sign so driving forward advances the robot forward in odom/RViz
        dl = (left - self.last_left) * self.wheel_radius * self.encoder_direction
        dr = (right - self.last_right) * self.wheel_radius * self.encoder_direction
        dc = (dl + dr) / 2.0                        # centre travel
        dth = (dr - dl) / self.track_width          # heading change

        # integrate pose (midpoint heading)
        self.x += dc * math.cos(self.theta + dth / 2.0)
        self.y += dc * math.sin(self.theta + dth / 2.0)
        self.theta = math.atan2(math.sin(self.theta + dth), math.cos(self.theta + dth))
        self.v = dc / dt
        self.w = dth / dt

        self.last_left, self.last_right, self.last_time = left, right, now

    def publish(self):
        now = self.get_clock().now().to_msg()
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_bc.sendTransform(t)


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
