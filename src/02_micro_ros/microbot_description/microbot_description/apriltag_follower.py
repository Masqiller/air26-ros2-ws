#!/usr/bin/env python3
"""apriltag_follower — follow an AprilTag with the 2WD differential vehicle (NO PID / no tuning).

Subscribes: /camera/image_raw  (sensor_msgs/Image)  ESP32-CAM feed via camera_stream
Publishes:  /cmd_vel          (geometry_msgs/Twist) drive commands (micro-ROS -> motors)
            /apriltag/follow  (sensor_msgs/Image)   debug view: tag box + command overlay

Control (deliberately tuning-free):
  * steering: proportional on the tag's NORMALISED horizontal offset (-1..1) times the max
    turn speed -> angular.z. Tag at frame edge = full (slow) turn, near centre = gentle turn,
    centred = straight. The only "gain" is max_angular (your speed cap) — nothing to tune.
  * distance: 3-zone BANG-BANG -> too far = creep forward, in the band = hold/stop, too close
    = ease back. No gains, just a hold band around desired_distance.

Tag lost -> the vehicle STOPS in place and waits until the tag returns. A watchdog also
stops it if camera frames stop arriving.

Detection: OpenCV ArUco/AprilTag (tag25h9 + tag36h11, tracking id 12). Distance is a pinhole
estimate  dist = tag_size * fx / tag_pixels  (QVGA fx ~250) — approximate; set desired_distance
to taste. Measurements are lightly smoothed (moving average) so bang-bang doesn't chatter.
"""
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

APRILTAG_FAMILIES = {
    '25h9': cv2.aruco.DICT_APRILTAG_25h9,
    '36h11': cv2.aruco.DICT_APRILTAG_36h11,
    '16h5': cv2.aruco.DICT_APRILTAG_16h5,
}


class AprilTagFollower(Node):
    def __init__(self):
        super().__init__('apriltag_follower')
        self.tag_id = int(self.declare_parameter('tag_id', 12).value)
        self.tag_size = float(self.declare_parameter('tag_size', 0.15).value)      # metres
        self.fx = float(self.declare_parameter('fx', 250.0).value)                 # QVGA estimate
        families = self.declare_parameter('families', ['25h9', '36h11']).value

        self.desired_distance = float(self.declare_parameter('desired_distance', 0.2).value)  # hold this close to the tag (smaller = follows in closer)
        # --- speeds (m/s, rad/s): quicker again, still moderate ---
        self.forward_speed = float(self.declare_parameter('forward_speed', 0.12).value)  # straight cruise
        self.turn_cruise = float(self.declare_parameter('turn_cruise', 0.13).value)      # forward while arcing
        self.reverse_speed = float(self.declare_parameter('reverse_speed', 0.08).value)  # ease back if too close
        self.max_turn = float(self.declare_parameter('max_turn', 0.0).value)             # rad/s; 0 = turning OFF (drive straight only), set >0 (e.g. 0.6) to re-enable arcs
        # mechanical trim: the drivetrain veers right on a "straight" command; a small
        # positive bias speeds the right wheel / slows the left -> nudges left to cancel it
        self.straight_bias = float(self.declare_parameter('straight_bias', 0.03).value)  # rad/s added to w while driving forward (bigger = more left)
        # --- arc geometry: keep BOTH wheels forward + above the motor deadband (no spin, no stall) ---
        # these must match the vehicle firmware's mixing; wheel_min is the motor's PWM deadband (0..1).
        self.wheel_min = float(self.declare_parameter('wheel_min', 0.28).value)          # raise if a wheel still stalls
        self.wheel_sep = float(self.declare_parameter('wheel_sep', 0.251).value)         # firmware WHEEL_SEP
        self.fw_max_lin = float(self.declare_parameter('fw_max_lin', 0.25).value)        # firmware MAX_LIN
        # hold bands (not gains — just how close counts as "good enough")
        self.center_deadband = float(self.declare_parameter('center_deadband', 0.35).value)  # wide -> stay straight unless the tag is well out to a side
        self.dist_band = float(self.declare_parameter('dist_band', 0.08).value)              # metres (smaller -> reacts sooner)
        self.frame_timeout = float(self.declare_parameter('frame_timeout', 0.5).value)

        aruco_params = cv2.aruco.DetectorParameters()
        self.detectors = []
        for fam in families:
            key = APRILTAG_FAMILIES.get(fam)
            if key is not None:
                self.detectors.append(cv2.aruco.ArucoDetector(
                    cv2.aruco.getPredefinedDictionary(key), aruco_params))

        # light smoothing of the measurements (filtering, not tuning)
        self.offset_hist = deque(maxlen=5)
        self.dist_hist = deque(maxlen=5)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg_pub = self.create_publisher(Image, '/apriltag/follow', 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.create_timer(0.1, self.watchdog)
        self.last_image_t = None
        self.have_tag = False
        self.get_logger().info(
            'apriltag_follower up: follow id=%d, desired=%.2f m, fwd=%.2f m/s, turning=%s'
            % (self.tag_id, self.desired_distance, self.forward_speed,
               'ON' if self.max_turn > 0.0 else 'OFF (straight only)'))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def watchdog(self):
        # camera frames stopped -> stop the vehicle (safety)
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
        w = msg.width

        # find the target tag id in any configured family
        target = None
        for det in self.detectors:
            corners, ids, _ = det.detectMarkers(gray)
            if ids is not None:
                for c, i in zip(corners, ids.flatten()):
                    if int(i) == self.tag_id:
                        target = c[0]        # (4,2) corner pixels
                        break
            if target is not None:
                break

        twist = Twist()
        if target is None:
            # ---- tag lost: STOP in place, wait for it to return ----
            if self.have_tag:
                self.get_logger().info('tag lost -> STOP in place')
                self.have_tag = False
                self.offset_hist.clear()
                self.dist_hist.clear()
            self.cmd_pub.publish(twist)
            self.publish_debug(bgr, None, twist)
            return

        if not self.have_tag:
            self.get_logger().info('tag %d acquired -> following' % self.tag_id)
            self.have_tag = True

        cx = float(np.mean(target[:, 0]))
        raw_offset = (cx - w / 2.0) / (w / 2.0)            # -1..1, + = tag right of centre
        side_px = float(np.mean([np.linalg.norm(target[k] - target[(k + 1) % 4])
                                 for k in range(4)]))
        raw_dist = (self.tag_size * self.fx / side_px) if side_px > 1.0 else 999.0

        # smooth (moving average) so the arc control doesn't jitter on noisy measurements
        self.offset_hist.append(raw_offset)
        self.dist_hist.append(raw_dist)
        offset = max(-1.0, min(1.0, sum(self.offset_hist) / len(self.offset_hist)))
        dist = sum(self.dist_hist) / len(self.dist_hist)

        # ---- ARC-ONLY motion: never spin in place, never stall a wheel ----
        v = 0.0
        w = 0.0
        if dist < self.desired_distance - self.dist_band:
            # too close -> ease straight back (no turn, so the tag stays in view)
            v = -self.reverse_speed
        elif self.max_turn > 0.0 and abs(offset) > self.center_deadband:
            # off-centre -> ARC: roll forward while turning so BOTH wheels drive forward
            # (outer faster, inner slower). Forward speed depends on distance.
            v = self.forward_speed if dist > self.desired_distance + self.dist_band \
                else self.turn_cruise
            # steer by how far the tag is PAST the straight zone: 0 right at the edge of
            # the deadband, ramping up to max_turn only near the frame edge -> mostly
            # straight, with a gentle correction only when the tag is well left/right
            mag = (abs(offset) - self.center_deadband) / max(1e-3, 1.0 - self.center_deadband)
            mag = max(0.0, min(1.0, mag))
            w = -np.sign(offset) * mag * self.max_turn         # tag right -> w<0 -> turn right
            # cap the turn so the INNER wheel stays forward AND above the motor deadband:
            #   inner_norm = (v - |w|*sep/2) / MAX_LIN  >=  wheel_min
            w_cap = max(0.0, 2.0 * (v - self.wheel_min * self.fw_max_lin) / self.wheel_sep)
            w = max(-w_cap, min(w_cap, w))
        elif dist > self.desired_distance + self.dist_band:
            # centred but far -> straight forward
            v = self.forward_speed
        # else: centred and within the band -> stop (v = w = 0)

        # mechanical trim: cancel the drivetrain's right drift with a small LEFT bias,
        # but only while actually driving forward (don't creep when stopped/backing up)
        if v > 0.0:
            w += self.straight_bias

        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)
        self.publish_debug(bgr, target, twist, dist, offset)

    def publish_debug(self, bgr, target, twist, dist=None, offset=None):
        if target is not None:
            cv2.polylines(bgr, [target.astype(np.int32)], True, (0, 255, 0), 2)
            txt = 'd=%.2fm off=%+.2f  v=%.2f w=%.2f' % (dist, offset,
                                                        twist.linear.x, twist.angular.z)
            color = (0, 255, 0)
        else:
            txt = 'NO TAG -> STOP'
            color = (0, 0, 255)
        cv2.putText(bgr, txt, (8, bgr.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        out = Image()
        out.height, out.width = bgr.shape[0], bgr.shape[1]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = bgr.tobytes()
        self.dbg_pub.publish(out)


def main():
    rclpy.init()
    node = AprilTagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())     # stop on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
