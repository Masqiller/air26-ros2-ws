#!/usr/bin/env python3
"""apriltag_follower_pid — PID AprilTag follower, ported from yenode/raju_bot_ws.

Subscribes: /camera/image_raw  (sensor_msgs/Image)   ESP32-CAM feed via camera_stream
Publishes:  /cmd_vel          (geometry_msgs/Twist)  drive commands
            /apriltag/follow  (sensor_msgs/Image)    debug overlay (same topic as the
                                                     no-PID follower, so RViz needs no change)

MECHANISM KEPT FROM THE REFERENCE (src/bot_detection/apriltag_node.py):
  * PID class: P + I + D, output clamped, integral clamped to out_max/ki (anti-windup)
  * angular PID on the tag's horizontal PIXEL error from the image centre
        err_x = (width / 2) - center_x        (tag left of centre -> +err -> turn left)
  * linear PID on (distance - safe_distance), with a small deadband around zero that
    also zeroes the linear integral to stop micro-oscillation at the hold point
  * both integrals reset when the tag is lost, and a zero Twist is published -> stop in place

ADAPTED FOR THIS ROBOT (the reference runs in Gazebo; we drive a real ESP32 bot):
  * cv2.aruco instead of pupil_apriltags — guaranteed present in our ROS env, and it is
    what the rest of our stack already uses.
  * pose via solvePnP (IPPE_SQUARE) so we get a real translation vector like the
    reference's pose_t; falls back to the pinhole estimate tag_size*fx/side_px.
  * cx/cy are taken from the ACTUAL frame size, not hardcoded. The reference hardcodes
    cx=160, cy=120 (a QVGA centre) while its sim camera is 640x480, so its distances are
    biased by ~1.5x and its "1.0 m" safe distance is not really 1.0 m. Do not copy that.
  * OUR 8 cm tag: an 8 cm tag stops decoding past ~0.74 m on QVGA, so safe_distance must
    stay well under that. Default 0.35 m. (The reference uses 1.0 m because its sim tag is
    ~0.32 m and its camera is 640 px wide.)
  * ARC CAP: every turn is capped so the inner wheel stays FORWARD and above the motor's
    PWM deadband, and the bot keeps rolling while steering. Without this the PID output
    pivots the bot in place, the inner wheel stalls below the deadband, and the tag leaves
    the frame — exactly the failure we already fixed once on this vehicle.
  * straight_bias trims the drivetrain's mechanical right-drift (the reference has no
    equivalent because a simulated drivetrain is perfectly symmetric).
  * manual Image decode (no cv_bridge — it has a NumPy 2 ABI break on this box).
  * a camera watchdog stops the vehicle if frames stop arriving.
"""
import math
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


class PID:
    """Straight port of the reference PID (kept deliberately simple and inspectable)."""

    def __init__(self, kp, ki, kd, out_max, out_min):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self.out_min = out_min
        self.prev_error = 0.0
        self.integral = 0.0

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0
        self.integral += error * dt

        # anti-windup: clamp the integral so ki*integral alone cannot exceed the output range
        if self.ki != 0.0:
            int_max = self.out_max / self.ki
            int_min = self.out_min / self.ki
            self.integral = max(min(self.integral, int_max), int_min)

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(min(output, self.out_max), self.out_min)


class AprilTagFollowerPID(Node):
    def __init__(self):
        super().__init__('apriltag_follower_pid')

        # ---- tag / camera ----
        self.tag_id = int(self.declare_parameter('tag_id', 12).value)
        # OUR printed tag: side of the outer black border, in metres.
        self.tag_size = float(self.declare_parameter('tag_size', 0.08).value)
        # MEASURED, not assumed: tag at a tape-measured 0.56 m gave side_px ~118, so
        # fx = 118 * 0.56 / 0.08 ~ 820 (a ~22 deg FOV -> the camera windows the sensor for
        # grayscale QVGA). The old "~250 for QVGA" guess read every distance 3.3x too small.
        self.fx = float(self.declare_parameter('fx', 820.0).value)
        self.fy = float(self.declare_parameter('fy', 820.0).value)
        families = self.declare_parameter('families', ['25h9', '36h11']).value

        # ---- setpoint ----
        # Keep this comfortably inside the tag's decode range (~9 x tag_size on QVGA):
        # an 8 cm tag dies past ~0.74 m, so 0.35 m leaves plenty of approach room.
        self.safe_distance = float(self.declare_parameter('safe_distance', 0.45).value)
        self.dist_deadband = float(self.declare_parameter('dist_deadband', 0.05).value)
        # Steering deadband in pixels (the reference has none). Needed because arc steering
        # implies forward motion: without it the vehicle creeps forward for ever at the
        # setpoint chasing a 1-pixel error. ~12 px of a 320 px frame.
        self.ang_deadband_px = float(self.declare_parameter('angular_deadband_px', 12.0).value)

        # ---- PID gains (reference values, re-ranged for this vehicle) ----
        # Angular error is in PIXELS. Our frame is 320 px wide so |err| <= 160, and the
        # reference's kp=0.003 therefore yields <= ~0.48 rad/s, which suits us as-is.
        # slowed to match the zone follower
        self.ang_max = float(self.declare_parameter('angular_max', 0.15).value)
        self.lin_max = float(self.declare_parameter('linear_max', 0.04).value)   # our cruise
        self.lin_min = float(self.declare_parameter('linear_min', -0.03).value)  # our reverse
        self.linear_pid = PID(
            kp=float(self.declare_parameter('linear_kp', 0.6).value),
            ki=float(self.declare_parameter('linear_ki', 0.05).value),
            kd=float(self.declare_parameter('linear_kd', 0.1).value),
            out_max=self.lin_max, out_min=self.lin_min)
        self.angular_pid = PID(
            kp=float(self.declare_parameter('angular_kp', 0.003).value),
            ki=float(self.declare_parameter('angular_ki', 0.0005).value),
            kd=float(self.declare_parameter('angular_kd', 0.001).value),
            out_max=self.ang_max, out_min=-self.ang_max)

        # ---- vehicle limits: keep both wheels forward and above the PWM deadband ----
        self.arc_only = bool(self.declare_parameter('arc_only', True).value)
        self.turn_cruise = float(self.declare_parameter('turn_cruise', 0.04).value)
        # wheel_min WAS 0.28 to keep the inner wheel above the motor's PWM deadband. The
        # firmware now runs a closed-loop wheel-velocity PID, whose integrator raises PWM
        # until the wheel actually turns, so there is no deadband to dodge -> 0.0. (Set it
        # back to 0.28 if you ever reflash with VELOCITY_PID 0.)
        self.wheel_min = float(self.declare_parameter('wheel_min', 0.0).value)
        self.wheel_sep = float(self.declare_parameter('wheel_sep', 0.251).value)
        self.fw_max_lin = float(self.declare_parameter('fw_max_lin', 0.25).value)
        # straight_bias WAS 0.03 to cancel a mechanical right-drift. Each wheel is now
        # regulated to its own speed target, so that drift is corrected in closed loop and
        # this trim would ADD an unwanted left curve -> 0.0.
        self.straight_bias = float(self.declare_parameter('straight_bias', 0.0).value)
        self.frame_timeout = float(self.declare_parameter('frame_timeout', 0.5).value)
        self.use_pnp = bool(self.declare_parameter('use_pnp', True).value)

        aruco_params = cv2.aruco.DetectorParameters()
        self.detectors = []
        for fam in families:
            key = APRILTAG_FAMILIES.get(fam)
            if key is not None:
                self.detectors.append(cv2.aruco.ArucoDetector(
                    cv2.aruco.getPredefinedDictionary(key), aruco_params))

        # light measurement smoothing so the D term doesn't amplify pixel jitter
        self.dist_hist = deque(maxlen=5)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dbg_pub = self.create_publisher(Image, '/apriltag/follow', 10)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.create_timer(0.1, self.watchdog)

        self.last_image_t = None
        self.last_time = self.get_clock().now()
        self.have_tag = False

        self.get_logger().info(
            'apriltag_follower_pid up: id=%d, tag=%.0f cm, safe_distance=%.2f m, '
            'lin PID(%.3f,%.3f,%.3f) cap %.2f, ang PID(%.4f,%.4f,%.4f) cap %.2f, arc_only=%s'
            % (self.tag_id, self.tag_size * 100.0, self.safe_distance,
               self.linear_pid.kp, self.linear_pid.ki, self.linear_pid.kd, self.lin_max,
               self.angular_pid.kp, self.angular_pid.ki, self.angular_pid.kd, self.ang_max,
               self.arc_only))
        decode_limit = self.tag_size * self.fx / 27.0
        if self.safe_distance > 0.8 * decode_limit:
            self.get_logger().warn(
                'safe_distance %.2f m is close to this tag\'s decode limit %.2f m — the tag '
                'will drop out before the bot settles. Use a bigger tag or a smaller '
                'safe_distance.' % (self.safe_distance, decode_limit))

    # ---------------------------------------------------------------- helpers
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def watchdog(self):
        # camera frames stopped -> stop the vehicle (safety)
        if self.last_image_t is not None and (self._now() - self.last_image_t) > self.frame_timeout:
            self.cmd_pub.publish(Twist())

    def solve_distance(self, corners, w, h):
        """Tag distance in metres. solvePnP (like the reference's pose_t) with a pinhole
        fallback. cx/cy come from the real frame size, not a hardcoded guess."""
        side_px = float(np.mean([np.linalg.norm(corners[k] - corners[(k + 1) % 4])
                                for k in range(4)]))
        pinhole = (self.tag_size * self.fx / side_px) if side_px > 1.0 else None

        if self.use_pnp and side_px > 1.0:
            half = self.tag_size / 2.0
            objp = np.array([[-half, half, 0.0], [half, half, 0.0],
                             [half, -half, 0.0], [-half, -half, 0.0]], dtype=np.float64)
            K = np.array([[self.fx, 0.0, w / 2.0],
                          [0.0, self.fy, h / 2.0],
                          [0.0, 0.0, 1.0]], dtype=np.float64)
            flag = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)
            try:
                ok, _rvec, tvec = cv2.solvePnP(objp, corners.astype(np.float64), K,
                                               np.zeros(5), flags=flag)
                if ok:
                    return float(np.linalg.norm(tvec)), side_px
            except cv2.error:
                pass
        return pinhole, side_px

    # ---------------------------------------------------------------- main
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

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        twist = Twist()

        if target is None:
            # tag lost -> reset both integrals (reference behaviour) and stop in place
            if self.have_tag:
                self.get_logger().info('tag lost -> STOP in place (PID integrals reset)')
                self.have_tag = False
            self.linear_pid.reset()
            self.angular_pid.reset()
            self.dist_hist.clear()
            self.cmd_pub.publish(twist)
            self.publish_debug(bgr, None, twist)
            return

        if not self.have_tag:
            self.get_logger().info('tag %d acquired -> PID following' % self.tag_id)
            self.have_tag = True

        center_x = float(np.mean(target[:, 0]))
        dist, side_px = self.solve_distance(target, w, h)
        if dist is None:
            self.cmd_pub.publish(twist)
            self.publish_debug(bgr, target, twist)
            return
        self.dist_hist.append(dist)
        dist = sum(self.dist_hist) / len(self.dist_hist)

        # a stale/absurd dt (first frame, stream hiccup) would spike the D term
        if dt <= 0.0 or dt > 1.0:
            self.publish_debug(bgr, target, twist, dist, 0.0)
            return

        # ---- angular: PID on the pixel error from image centre (reference formulation) ----
        err_x = (w / 2.0) - center_x                 # tag left of centre -> positive -> turn left
        if abs(err_x) < self.ang_deadband_px:
            # Centred enough. Zeroing here matters on a real bot: arc steering needs forward
            # motion, so a permanently non-zero ang would make the vehicle creep forward for
            # ever instead of holding at the setpoint.
            ang = 0.0
            self.angular_pid.integral = 0.0
        else:
            ang = self.angular_pid.compute(err_x, dt)

        # ---- linear: PID on distance error, with the reference's deadband ----
        err_dist = dist - self.safe_distance
        if abs(err_dist) < self.dist_deadband:
            lin = 0.0
            self.linear_pid.integral = 0.0           # reference: kill windup at the setpoint
        else:
            lin = self.linear_pid.compute(err_dist, dt)

        # ---- vehicle limits (NOT in the reference; required by our real drivetrain) ----
        if self.arc_only:
            if err_dist < -self.dist_deadband:
                # TOO CLOSE: reversing wins. Back straight out and do not steer — a reverse
                # pivot stalls the inner wheel and swings the tag out of frame. (Never let
                # the "keep rolling" rule below override a reverse command.)
                ang = 0.0
            elif abs(ang) > 1e-3:
                # keep rolling forward while steering so neither wheel drops below the PWM
                # deadband and the tag stays in frame
                lin = max(lin, self.turn_cruise)
                # cap the turn so the INNER wheel stays forward AND above the deadband:
                #   (lin - |ang|*sep/2) / MAX_LIN >= wheel_min
                cap = max(0.0, 2.0 * (lin - self.wheel_min * self.fw_max_lin) / self.wheel_sep)
                ang = max(-cap, min(cap, ang))

        if lin > 0.0:
            ang += self.straight_bias                # mechanical trim for the right-drift

        twist.linear.x = float(lin)
        twist.angular.z = float(ang)
        self.cmd_pub.publish(twist)
        self.publish_debug(bgr, target, twist, dist, err_x)

    # ---------------------------------------------------------------- debug view
    def publish_debug(self, bgr, target, twist, dist=None, err_x=None):
        h, w = bgr.shape[0], bgr.shape[1]
        # centre zone, as the reference draws
        cv2.rectangle(bgr, (int(w * 0.4), 0), (int(w * 0.6), h), (30, 150, 30), 1)
        if target is not None:
            cv2.polylines(bgr, [target.astype(np.int32)], True, (0, 255, 0), 2)
            cx, cy = int(np.mean(target[:, 0])), int(np.mean(target[:, 1]))
            cv2.circle(bgr, (cx, cy), 4, (255, 0, 0), -1)
            txt = 'd=%.2fm err=%+.0fpx v=%+.3f w=%+.3f' % (
                dist if dist is not None else float('nan'),
                err_x if err_x is not None else 0.0,
                twist.linear.x, twist.angular.z)
            color = (0, 255, 0)
        else:
            txt = 'NO TAG -> STOP'
            color = (0, 0, 255)
        cv2.putText(bgr, txt, (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        out = Image()
        out.height, out.width = h, w
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = w * 3
        out.data = bgr.tobytes()
        self.dbg_pub.publish(out)


def main():
    rclpy.init()
    node = AprilTagFollowerPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())      # stop on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
