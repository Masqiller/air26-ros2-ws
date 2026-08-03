#!/usr/bin/env python3
"""joint_states — FALLBACK wheel angles so the RobotModel always renders whole.

robot_state_publisher only emits a link's TF once it knows that joint's angle, so the
two continuous wheel joints need /joint_states or the wheels vanish in RViz.

On the real robot the ESP32-S3 already publishes those exact joint names from the wheel
encoders. Publishing zeros at the same time would fight that data (the wheels would
snap between the encoder angle and 0), so this node only fills in while the firmware is
NOT publishing: it watches /joint_states and stays quiet for `hold` seconds after any
message that isn't one of its own. Bot offline -> zeros so the model renders; bot online
-> silent, and the wheels spin from the encoders.

Our own messages are tagged with header.frame_id='fallback' so they are easy to ignore
(the firmware sends frame_id='base_link').
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

TAG = 'fallback'


class WheelJointStates(Node):
    def __init__(self):
        super().__init__('wheel_joint_states')
        # must match the URDF's continuous joints (and the firmware's encoder names)
        self.names = self.declare_parameter(
            'joints', ['base_back_left_wheel_joint',
                       'base_back_right_wheel_joint']).value
        self.hold = float(self.declare_parameter('hold', 1.0).value)

        self.last_external = None
        self.quiet = False
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            'joint_states fallback up: %s (publishes only while nothing else does)'
            % ', '.join(self.names))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_joints(self, msg):
        if msg.header.frame_id == TAG:
            return                              # our own message, ignore
        if not any(n in msg.name for n in self.names):
            return                              # unrelated joints, ignore
        self.last_external = self._now()
        if not self.quiet:
            self.quiet = True
            self.get_logger().info('real wheel encoders detected -> fallback silent')

    def tick(self):
        if self.last_external is not None and (self._now() - self.last_external) < self.hold:
            return                              # firmware is publishing; stay out of the way
        if self.quiet:
            self.quiet = False
            self.get_logger().info('no encoder data -> publishing zero wheel angles')

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = TAG
        msg.name = list(self.names)
        msg.position = [0.0] * len(self.names)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = WheelJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
