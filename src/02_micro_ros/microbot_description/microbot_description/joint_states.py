#!/usr/bin/env python3
"""joint_states — constant zero angles for the 4 continuous wheel joints.

robot_state_publisher only emits a link's TF once it knows that joint's angle. The real
bot has no wheel encoders and we don't need the wheels to spin just to look at the robot,
so this publishes a steady 0 for each wheel joint -> the RobotModel renders whole in RViz.
It's a 15-line stand-in for joint_state_publisher (which isn't installed on this box).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WheelJointStates(Node):
    def __init__(self):
        super().__init__('wheel_joint_states')
        self.names = ['wheel_fl_joint', 'wheel_fr_joint',
                      'wheel_rl_joint', 'wheel_rr_joint']
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.1, self.tick)

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
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
