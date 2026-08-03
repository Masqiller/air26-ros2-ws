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

        # --- steering-zone overlay, drawn straight onto /apriltag/image so it shows up in
        #     the RViz "AprilTag Detections" display without adding another display.
        #     LEFT | CENTRE | RIGHT, plus a dot on the target tag's centre: this is the
        #     same geometry apriltag_follower_zone acts on, so KEEP center_frac IN SYNC
        #     with that node (both default to 0.34 = middle third of the frame width). ---
        self.draw_zones = bool(self.declare_parameter('draw_zones', True).value)
        self.center_frac = float(self.declare_parameter('center_frac', 0.34).value)
        self.target_id = int(self.declare_parameter('target_id', 12).value)

        # --- distance estimate shown next to each tag id ---
        # Pinhole:  distance = tag_size * fx / tag_side_pixels
        # Same formula apriltag_follower_zone uses, so the number on screen is exactly what
        # the follower is acting on. It is only as good as these two numbers:
        #   tag_size = side of the PRINTED tag's outer BLACK border, in metres
        #   fx       = focal length in pixels
        # Get either wrong and every distance scales by the same ratio.
        #
        # tag_size 0.080 = the generator's "TAG SIZE" (black square). Its "TOTAL SIZE"
        # 102.9 mm includes the white quiet zone (80 x 9/7) and is NOT what to use here,
        # because the detector's corners sit on the black border.
        #
        # fx MEASURED on this camera, not assumed: a tag at a tape-measured 0.56 m gave
        # side_px ~118, so fx = 118 * 0.56 / 0.08 ~ 820. That implies only a ~22 deg
        # horizontal FOV, i.e. the ESP32-CAM is WINDOWING the sensor for grayscale QVGA
        # rather than downscaling the full field. The textbook "~250 for QVGA" guess was
        # wrong by 3.3x and made every distance read 3.3x too small.
        # Re-check any time: hold the tag at a measured distance and read the on-screen
        # label; if it reads X at a true D, set fx := 820 * D / X.
        self.tag_size = float(self.declare_parameter('tag_size', 0.08).value)
        self.fx = float(self.declare_parameter('fx', 820.0).value)

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

    def tag_distance(self, corners):
        """Pinhole distance in metres from the tag's mean side length in pixels."""
        side_px = float(np.mean([np.linalg.norm(corners[k] - corners[(k + 1) % 4])
                                for k in range(4)]))
        if side_px <= 1.0:
            return None
        return self.tag_size * self.fx / side_px

    def paint_zones(self, bgr):
        """Shade LEFT / CENTRE / RIGHT steering zones. Returns the two boundary columns."""
        h, w = bgr.shape[0], bgr.shape[1]
        half = self.center_frac * w / 2.0
        le, re = int(w / 2.0 - half), int(w / 2.0 + half)

        ov = bgr.copy()
        cv2.rectangle(ov, (0, 0), (le, h), (200, 80, 0), -1)      # left  = blue
        cv2.rectangle(ov, (re, 0), (w, h), (0, 80, 200), -1)      # right = red
        cv2.rectangle(ov, (le, 0), (re, h), (0, 140, 0), -1)      # centre = green
        cv2.addWeighted(ov, 0.18, bgr, 0.82, 0, bgr)

        cv2.line(bgr, (le, 0), (le, h), (255, 200, 0), 1)
        cv2.line(bgr, (re, 0), (re, h), (0, 200, 255), 1)
        cv2.line(bgr, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)   # frame centre
        cv2.putText(bgr, 'LEFT', (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 0), 1)
        cv2.putText(bgr, 'CENTRE', (le + 4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 230, 0), 1)
        cv2.putText(bgr, 'RIGHT', (re + 4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
        return le, re

    def paint_pointer(self, bgr, corners, zone_edges):
        """Red dot on the target tag's centre + which zone it sits in."""
        h, w = bgr.shape[0], bgr.shape[1]
        le, re = zone_edges
        if corners is None:
            cv2.putText(bgr, 'target id %d: not visible' % self.target_id,
                        (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            return
        dx = float(np.mean(corners[:, 0]))
        dy = float(np.mean(corners[:, 1]))

        # error bar from frame centre to the dot, then the dot itself
        cv2.line(bgr, (w // 2, int(dy)), (int(dx), int(dy)), (0, 0, 255), 1)
        cv2.circle(bgr, (int(dx), int(dy)), 6, (0, 0, 255), -1)
        cv2.circle(bgr, (int(dx), int(dy)), 8, (255, 255, 255), 1)

        if dx < le:
            zone = 'LEFT -> turn LEFT'
        elif dx > re:
            zone = 'RIGHT -> turn RIGHT'
        else:
            zone = 'CENTRE - aligned'
        dist = self.tag_distance(corners)
        dtxt = '%.2f m' % dist if dist is not None else '--'
        line = 'id %d  d=%s  off=%+.0f px  %s' % (self.target_id, dtxt,
                                                  dx - w / 2.0, zone)
        cv2.putText(bgr, line, (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
        cv2.putText(bgr, line, (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

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

        # zones go down first so the tag outlines and the dot draw on top of the shading
        zone_edges = self.paint_zones(bgr) if self.draw_zones else None

        found = []   # (family, id, corners)
        for fam, det in self.detectors:
            corners, ids, _ = det.detectMarkers(gray)
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
                for c, tid in zip(corners, ids.flatten()):
                    found.append((fam, int(tid), c))

        for fam, tid, c in found:
            corners = c[0]
            cx, cy = corners.mean(axis=0)
            dist = self.tag_distance(corners)
            label = ('id %d (%s)  %.2f m' % (tid, fam, dist) if dist is not None
                     else 'id %d (%s)' % (tid, fam))
            # keep the label on screen on a narrow QVGA frame
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
            tx = int(min(max(cx - tw / 2.0, 2), bgr.shape[1] - tw - 2))
            ty = int(cy) - 12
            if ty < th + 2:                      # tag near the top -> put it underneath
                ty = int(cy) + th + 14
            cv2.putText(bgr, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (0, 0, 0), 3)            # dark outline for contrast
            cv2.putText(bgr, label, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (0, 255, 0), 1)

        # centre pointer + zone read-out for the tag we actually follow
        if zone_edges is not None:
            target = next((c[0] for _f, tid, c in found if tid == self.target_id), None)
            self.paint_pointer(bgr, target, zone_edges)

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
