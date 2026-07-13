#!/usr/bin/env python3
"""camera_stream — bring the ESP32-CAM's live video into RViz.

The ESP32-CAM serves its full image as a plain MJPEG stream over HTTP
(http://<board-ip>/stream) — it's too big to push through micro-ROS/UDP. RViz can't
open an HTTP URL, only ROS image topics, so this node bridges the two: it pulls the
MJPEG stream, splits out each JPEG frame, and republishes it as sensor_msgs/Image on
/camera/image_raw. Add an "Image" display in RViz on that topic to watch the feed.

Getting the board IP:
  - the ESP32-CAM firmware publishes its IP on /camera/ip (std_msgs/String); by default
    this node listens there and builds http://<ip>/stream automatically, OR
  - set the 'url' parameter directly, e.g.
      ros2 run microbot_description camera_stream --ros-args -p url:=http://192.168.0.42/stream
"""
import threading
import time

import cv2
import numpy as np
import rclpy
import requests
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraStream(Node):
    def __init__(self):
        super().__init__('camera_stream')
        self.url = self.declare_parameter('url', '').value
        self.frame_id = self.declare_parameter('frame_id', 'camera_link').value
        self.topic = self.declare_parameter('topic', '/camera/image_raw').value

        self.pub = self.create_publisher(Image, self.topic, 10)
        self._url_lock = threading.Lock()

        if not self.url:
            # no explicit URL -> discover the board IP from /camera/ip
            self.create_subscription(String, '/camera/ip', self.on_ip, 10)
            self.get_logger().info('waiting for board IP on /camera/ip '
                                    '(or pass -p url:=http://<ip>/stream)')
        else:
            self.get_logger().info('streaming from %s' % self.url)

        self._stop = False
        self._worker = threading.Thread(target=self.run, daemon=True)
        self._worker.start()

    def on_ip(self, msg):
        ip = msg.data.strip()
        if not ip:
            return
        url = 'http://%s/stream' % ip
        with self._url_lock:
            if url != self.url:
                self.url = url
                self.get_logger().info('board IP -> streaming from %s' % url)

    def current_url(self):
        with self._url_lock:
            return self.url

    def run(self):
        while rclpy.ok() and not self._stop:
            url = self.current_url()
            if not url:
                self._sleep(0.5)
                continue
            try:
                self.pump(url)
            except Exception as e:                     # noqa: BLE001 - keep the bridge alive
                self.get_logger().warn('stream error (%s); retrying in 2s' % e)
                self._sleep(2.0)

    def pump(self, url):
        """Read the multipart MJPEG stream and publish each JPEG frame as an Image."""
        with requests.get(url, stream=True, timeout=5) as resp:
            resp.raise_for_status()
            buf = b''
            for chunk in resp.iter_content(chunk_size=4096):
                if self._stop or not rclpy.ok():
                    return
                if self.current_url() != url:          # IP changed -> reconnect
                    return
                buf += chunk
                start = buf.find(b'\xff\xd8')          # JPEG SOI
                end = buf.find(b'\xff\xd9')            # JPEG EOI
                if start != -1 and end != -1 and end > start:
                    jpg = buf[start:end + 2]
                    buf = buf[end + 2:]
                    self.publish_jpeg(jpg)

    def publish_jpeg(self, jpg):
        img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        # build sensor_msgs/Image by hand (no cv_bridge -> avoids its NumPy ABI break)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = img.shape[0], img.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        self.pub.publish(msg)

    def _sleep(self, secs):
        # interruptible sleep so shutdown stays prompt
        end = time.time() + secs
        while rclpy.ok() and not self._stop and time.time() < end:
            time.sleep(0.02)

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
