#!/usr/bin/env python3
"""apriltag_follower_zone — dead-simple ZONE steering (no PID). Built for debugging.

Subscribes: /camera/image_raw  (sensor_msgs/Image)
Publishes:  /cmd_vel          (geometry_msgs/Twist)
            /apriltag/follow  (sensor_msgs/Image)  annotated view (same topic as the other
                                                   followers, so RViz needs no change)

The frame is split into three vertical zones and a DOT is drawn on the tag's centre:

        |   LEFT    |  CENTRE  |   RIGHT   |
        |  turn L   |  aligned |  turn R   |

  * dot in the LEFT  zone -> turn LEFT  (angular.z > 0)
  * dot in the RIGHT zone -> turn RIGHT (angular.z < 0)
  * dot in the CENTRE zone -> aligned; then (mode=follow) the distance rule drives
    forward / holds / backs off. In mode=align the vehicle NEVER drives, it only rotates,
    which isolates steering from distance while you debug.

There is no PID here at all: turns are a FIXED rate, so behaviour is completely
predictable. Hysteresis stops the classic bang-bang chatter at a zone boundary: once
turning, it keeps turning until the dot is well inside the centre zone.

Params worth knowing:
  mode          align | follow      (default align: rotate only, safest first test)
  center_frac   0.34  -> centre zone is the middle third of the frame width
  turn_speed    rad/s, fixed
  exit_ratio    0.6   -> stop turning once inside 60% of the centre zone (hysteresis)
"""
from collections import deque

import numpy as np

import cv2
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

APRILTAG_FAMILIES = {
    '25h9': cv2.aruco.DICT_APRILTAG_25h9,
    '36h11': cv2.aruco.DICT_APRILTAG_36h11,
    '16h5': cv2.aruco.DICT_APRILTAG_16h5,
}


class ZoneFollower(Node):
    def __init__(self):
        super().__init__('apriltag_follower_zone')
        self.tag_id = int(self.declare_parameter('tag_id', 12).value)
        self.tag_size = float(self.declare_parameter('tag_size', 0.08).value)
        # fx MEASURED on this camera (~22 deg FOV: it windows the sensor for grayscale
        # QVGA). The old "~250 for QVGA" guess made distances read 3.3x too small.
        self.fx = float(self.declare_parameter('fx', 820.0).value)
        families = self.declare_parameter('families', ['25h9', '36h11']).value

        # 'align' = rotate only (no forward/back at all) -> isolates steering for debugging
        self.mode = str(self.declare_parameter('mode', 'align').value).lower()
        # centre zone as a fraction of frame WIDTH (0.34 = middle third)
        self.center_frac = float(self.declare_parameter('center_frac', 0.34).value)
        # hysteresis: stop turning only once inside this fraction of the centre zone
        self.exit_ratio = float(self.declare_parameter('exit_ratio', 0.6).value)
        # 0.4 -> 0.3 -> 0.15 rad/s. A 0.15 rad/s pivot spins each wheel ~0.019 m/s, whose
        # feedforward is only ~19 PWM, so the closed-loop integrator supplies most of the
        # torque. Expect a short hesitation before the wheels break away from rest; that is
        # stiction, not a control fault. If it hesitates badly, use turn_creep so the robot
        # arcs while turning (a rolling wheel needs far less torque than a stationary one).
        self.turn_speed = float(self.declare_parameter('turn_speed', 0.15).value)
        # optional forward creep while turning (0 = pivot in place; the firmware velocity
        # PID can now hold a slow pivot, so 0 is fine)
        self.turn_creep = float(self.declare_parameter('turn_creep', 0.0).value)

        # distance rule, only used in mode=follow
        #
        # STOP DISTANCE = safe_distance. The band is the tolerance that stops the vehicle
        # hunting forward/back around the setpoint:
        #     d > safe + band   -> drive forward (following)
        #     within the band    -> STOP and hold
        #     d < safe - band   -> ease back (too close)
        # At 0.45 m an 80 mm tag spans 146 px (61% of the 240 px frame height), and at the
        # closest approach 0.40 m it spans 164 px (68%) — still comfortably inside frame,
        # so the tag will not be lost by filling the view.
        self.safe_distance = float(self.declare_parameter('safe_distance', 0.45).value)
        self.dist_band = float(self.declare_parameter('dist_band', 0.05).value)
        # 0.12 -> 0.07 -> 0.04 m/s. NOTE these are only true m/s if TICKS_PER_REV (330) and
        # the wheel radius (0.0325 m) in the firmware are right; neither has been verified by
        # a physical measurement yet, so the absolute scale may be off even though the
        # relative reduction is real. See the one-turn tick test if absolute speed matters.
        self.forward_speed = float(self.declare_parameter('forward_speed', 0.04).value)
        self.reverse_speed = float(self.declare_parameter('reverse_speed', 0.03).value)
        self.frame_timeout = float(self.declare_parameter('frame_timeout', 0.5).value)

        params = cv2.aruco.DetectorParameters()
        self.detectors = []
        for fam in families:
            key = APRILTAG_FAMILIES.get(fam)
            if key is not None:
                self.detectors.append(cv2.aruco.ArucoDetector(
                    cv2.aruco.getPredefinedDictionary(key), params))

        self.turning = 0            # -1 right, 0 none, +1 left (latched for hysteresis)
        self.have_tag = False
        self.last_image_t = None
        self.last_zone = None
        self.dist_hist = deque(maxlen=5)   # moving average on the distance estimate

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg_pub = self.create_publisher(Image, '/apriltag/follow', 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            'zone follower up (NO PID): mode=%s, centre zone=middle %.0f%% of frame, '
            'turn=%.2f rad/s, tag=%.0f cm'
            % (self.mode, self.center_frac * 100.0, self.turn_speed, self.tag_size * 100.0))
        if self.mode == 'align':
            self.get_logger().info(
                'mode=align -> rotation ONLY, the vehicle will not drive forward or back')
        else:
            self.get_logger().info(
                'distance rule: FOLLOW while d > %.2f m | STOP between %.2f and %.2f m | '
                'ease back if d < %.2f m'
                % (self.safe_distance + self.dist_band,
                   self.safe_distance - self.dist_band,
                   self.safe_distance + self.dist_band,
                   self.safe_distance - self.dist_band))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def watchdog(self):
        if self.last_image_t is not None and (self._now() - self.last_image_t) > self.frame_timeout:
            self.cmd_pub.publish(Twist())

    def on_image(self, msg):
        self.last_image_t = self._now()
        buf = np.frombuffer(msg.data, np.uint8)
        if msg.encoding == 'mono8':
            gray = buf.reshape(msg.height, msg.width)
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        elif msg.encoding in ('bgr8', 'rgb8'):
            img = buf.reshape(msg.height, msg.width, 3)
            bgr = img.copy() if msg.encoding == 'bgr8' else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        else:
            return
        h, w = msg.height, msg.width

        target = None
        for det in self.detectors:
            cs, ids, _ = det.detectMarkers(gray)
            if ids is not None:
                for c, i in zip(cs, ids.flatten()):
                    if int(i) == self.tag_id:
                        target = c[0]
                        break
            if target is not None:
                break

        twist = Twist()
        if target is None:
            if self.have_tag:
                self.get_logger().info('tag lost -> STOP')
                self.have_tag = False
            self.turning = 0
            self.dist_hist.clear()      # don't average across a gap
            self.cmd_pub.publish(twist)
            self.publish_debug(bgr, None, twist, 'NO TAG')
            return
        if not self.have_tag:
            self.get_logger().info('tag %d acquired' % self.tag_id)
            self.have_tag = True

        # ---- the dot: centre of the tag ----
        dot_x = float(np.mean(target[:, 0]))
        dot_y = float(np.mean(target[:, 1]))
        side_px = float(np.mean([np.linalg.norm(target[k] - target[(k + 1) % 4])
                                 for k in range(4)]))
        raw_dist = (self.tag_size * self.fx / side_px) if side_px > 1.0 else 999.0
        # Smooth before comparing against the thresholds. Per-frame distance jitters a few
        # percent, which is enough to flip the decision back and forth right at a band edge
        # and make the vehicle stutter as it settles.
        self.dist_hist.append(raw_dist)
        dist = sum(self.dist_hist) / len(self.dist_hist)

        # ---- zone boundaries in pixels ----
        half = self.center_frac * w / 2.0
        left_edge = w / 2.0 - half           # dot left of this  -> LEFT zone
        right_edge = w / 2.0 + half          # dot right of this -> RIGHT zone
        inner = half * self.exit_ratio       # hysteresis band

        off = dot_x - w / 2.0                # + = tag right of centre

        # ---- latched zone decision (hysteresis) ----
        if self.turning == 0:
            if dot_x < left_edge:
                self.turning = +1            # tag on the LEFT -> rotate left
            elif dot_x > right_edge:
                self.turning = -1            # tag on the RIGHT -> rotate right
        else:
            if abs(off) <= inner:            # well inside the centre -> stop turning
                self.turning = 0

        if self.turning > 0:
            zone = 'LEFT zone -> turn LEFT'
        elif self.turning < 0:
            zone = 'RIGHT zone -> turn RIGHT'
        else:
            zone = 'CENTRE (aligned)'

        # ---- commands ----
        if self.turning != 0:
            twist.angular.z = self.turning * self.turn_speed
            twist.linear.x = self.turn_creep          # 0 = pivot in place
        elif self.mode == 'follow':
            if dist > self.safe_distance + self.dist_band:
                twist.linear.x = self.forward_speed
                zone += ' | FAR -> forward'
            elif dist < self.safe_distance - self.dist_band:
                twist.linear.x = -self.reverse_speed
                zone += ' | CLOSE -> back'
            else:
                zone += ' | IN BAND -> hold'
        else:
            zone += ' | mode=align (no drive)'

        self.cmd_pub.publish(twist)
        if zone != self.last_zone:
            self.get_logger().info('%s  (dot x=%.0f, d=%.2f m)' % (zone, dot_x, dist))
            self.last_zone = zone
        self.publish_debug(bgr, (target, dot_x, dot_y, dist), twist, zone,
                           left_edge, right_edge)

    def publish_debug(self, bgr, tag, twist, zone, left_edge=None, right_edge=None):
        h, w = bgr.shape[0], bgr.shape[1]
        if left_edge is None:
            half = self.center_frac * w / 2.0
            left_edge, right_edge = w / 2.0 - half, w / 2.0 + half
        le, re = int(left_edge), int(right_edge)

        # translucent zone shading: left = blue, right = red, centre = green
        ov = bgr.copy()
        cv2.rectangle(ov, (0, 0), (le, h), (200, 80, 0), -1)
        cv2.rectangle(ov, (re, 0), (w, h), (0, 80, 200), -1)
        cv2.rectangle(ov, (le, 0), (re, h), (0, 140, 0), -1)
        cv2.addWeighted(ov, 0.18, bgr, 0.82, 0, bgr)

        # zone borders + frame centre line
        cv2.line(bgr, (le, 0), (le, h), (255, 200, 0), 1)
        cv2.line(bgr, (re, 0), (re, h), (0, 200, 255), 1)
        cv2.line(bgr, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        cv2.putText(bgr, 'LEFT', (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200, 0), 1)
        cv2.putText(bgr, 'CENTRE', (le + 6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 230, 0), 1)
        cv2.putText(bgr, 'RIGHT', (re + 6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)

        if tag is not None:
            corners, dot_x, dot_y, dist = tag
            cv2.polylines(bgr, [corners.astype(np.int32)], True, (0, 255, 0), 1)
            # THE DOT: tag centre, what the vehicle aligns to
            cv2.circle(bgr, (int(dot_x), int(dot_y)), 6, (0, 0, 255), -1)
            cv2.circle(bgr, (int(dot_x), int(dot_y)), 8, (255, 255, 255), 1)
            # horizontal error bar from frame centre to the dot
            cv2.line(bgr, (w // 2, int(dot_y)), (int(dot_x), int(dot_y)), (0, 0, 255), 1)
            txt = '%s  d=%.2fm  v=%+.2f w=%+.2f' % (zone, dist, twist.linear.x,
                                                    twist.angular.z)
        else:
            txt = 'NO TAG -> STOP'
        cv2.putText(bgr, txt, (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1)

        out = Image()
        out.height, out.width = h, w
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = w * 3
        out.data = bgr.tobytes()
        self.dbg_pub.publish(out)


def main():
    rclpy.init()
    node = ZoneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
