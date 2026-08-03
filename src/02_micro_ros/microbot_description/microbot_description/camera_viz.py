#!/usr/bin/env python3
"""camera_viz — make the ESP32-CAM's data visible in RViz.

The ESP32-CAM can't push full frames through micro-ROS/UDP, so instead of an image it
publishes two small aggregates of what it sees:
    /camera/mean_color     (std_msgs/ColorRGBA)  average R,G,B of the frame, 0..1
    /camera/mean_intensity (std_msgs/Float32)    average brightness, 0..1

This node paints those onto a coloured swatch + a brightness label at the camera_link
frame, so "what the camera sees" shows up right on the robot in RViz (add a Marker
display on /camera/viz). The full live image is a plain MJPEG stream at
http://<board-ip>/stream (the IP is on /camera/ip) — too big for RViz/micro-ROS.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Float32
from visualization_msgs.msg import Marker


class CameraViz(Node):
    def __init__(self):
        super().__init__('camera_viz')
        self.frame = self.declare_parameter('frame_id', 'camera_link').value
        self.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)
        self.intensity = 0.0
        self.create_subscription(ColorRGBA, '/camera/mean_color', self.on_color, 10)
        self.create_subscription(Float32, '/camera/mean_intensity', self.on_intensity, 10)
        self.pub = self.create_publisher(Marker, '/camera/viz', 10)
        self.create_timer(0.2, self.publish)

    def on_color(self, msg):
        self.color = msg
        if self.color.a == 0.0:      # firmware may leave alpha unset
            self.color.a = 1.0

    def on_intensity(self, msg):
        self.intensity = float(msg.data)

    def publish(self):
        now = self.get_clock().now().to_msg()

        # coloured swatch just in front of the camera, painted with the mean colour
        cube = Marker()
        cube.header.frame_id = self.frame
        cube.header.stamp = now
        cube.ns = 'camera'
        cube.id = 0
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        # small panel just above the lens; keep it bot-sized so it doesn't swamp
        # the model in RViz (camera_link +X is the viewing direction)
        cube.pose.position.x = 0.012
        cube.pose.position.z = 0.045
        cube.pose.orientation.w = 1.0
        cube.scale.x = 0.006
        cube.scale.y = 0.045
        cube.scale.z = 0.030
        cube.color = self.color
        self.pub.publish(cube)

        # brightness read-out above the swatch
        txt = Marker()
        txt.header.frame_id = self.frame
        txt.header.stamp = now
        txt.ns = 'camera'
        txt.id = 1
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = 0.012
        txt.pose.position.z = 0.085
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.02
        txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        txt.text = 'intensity {:.2f}'.format(self.intensity)
        self.pub.publish(txt)


def main():
    rclpy.init()
    node = CameraViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
