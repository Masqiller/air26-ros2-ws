#!/usr/bin/env python3
"""apriltag_detector — detect AprilTags in the ESP32-CAM stream, draw them on the video.

Subscribes:  /camera/image_raw    (sensor_msgs/Image)  the MJPEG bridge output (bgr8)
Publishes:   /apriltag/image      (sensor_msgs/Image, bgr8)  same frame, tags outlined + ID
             /apriltag/ids        (std_msgs/Int32MultiArray) detected tag IDs this frame

Point RViz's Image display at /apriltag/image to see detections overlaid on the live feed.
Because it reads the ROS topic (not the board's HTTP directly), it coexists with the camera
feed — the ESP32-CAM's single HTTP client stays camera_stream.

Uses OpenCV's built-in ArUco/AprilTag detector. Detects tag25h9 AND tag36h11 by default
(the reference follower uses tag25h9 / id 12), so whichever family your tag is, it shows.
Override with the 'families' param, e.g. -p families:="['25h9']".
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

APRILTAG_FAMILIES = {
    '25h9': cv2.aruco.DICT_APRILTAG_25h9,
    '36h11': cv2.aruco.DICT_APRILTAG_36h11,
    '16h5': cv2.aruco.DICT_APRILTAG_16h5,
}


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        families = self.declare_parameter('families', ['25h9', '36h11']).value
        self.in_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        self.out_topic = self.declare_parameter('annotated_topic', '/apriltag/image').value

        params = cv2.aruco.DetectorParameters()
        self.detectors = []
        for fam in families:
            key = APRILTAG_FAMILIES.get(fam)
            if key is None:
                self.get_logger().warn('unknown family "%s" (skipped)' % fam)
                continue
            self.detectors.append((fam, cv2.aruco.ArucoDetector(
                cv2.aruco.getPredefinedDictionary(key), params)))

        self.pub_img = self.create_publisher(Image, self.out_topic, 10)
        self.pub_ids = self.create_publisher(Int32MultiArray, '/apriltag/ids', 10)
        self.create_subscription(Image, self.in_topic, self.on_image, 10)
        self._last = None
        self.get_logger().info('AprilTag detector up (families=%s): %s -> %s'
                               % ([f for f, _ in self.detectors], self.in_topic, self.out_topic))

    def on_image(self, msg):
        # decode incoming Image -> numpy by hand (no cv_bridge: avoids its NumPy 2 ABI break)
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == 'mono8':
            gray = buf.reshape(msg.height, msg.width)
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        elif msg.encoding in ('bgr8', 'rgb8'):
            img = buf.reshape(msg.height, msg.width, 3)
            bgr = img.copy() if msg.encoding == 'bgr8' else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        else:
            self.get_logger().warn('unsupported encoding: %s' % msg.encoding, once=True)
            return

        found = []   # (family, id, corners)
        for fam, det in self.detectors:
            corners, ids, _ = det.detectMarkers(gray)
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
                for c, tid in zip(corners, ids.flatten()):
                    found.append((fam, int(tid), c))

        for fam, tid, c in found:
            cx, cy = c[0].mean(axis=0)
            cv2.putText(bgr, 'id %d (%s)' % (tid, fam), (int(cx) - 30, int(cy) - 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        if found:
            self.pub_ids.publish(Int32MultiArray(data=[tid for _, tid, _ in found]))
        report = sorted({(fam, tid) for fam, tid, _ in found})
        if report != self._last:
            self.get_logger().info('tags: %s' % (report if report else 'none'))
            self._last = report

        out = Image()
        out.header = msg.header
        out.height, out.width = bgr.shape[0], bgr.shape[1]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = bgr.tobytes()
        self.pub_img.publish(out)


def main():
    rclpy.init()
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
