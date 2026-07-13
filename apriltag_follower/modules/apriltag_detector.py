"""AprilTag detector for the Platoon ESP32-CAM feed.

Adapted from Masqiller/AprilTag_Follower (src/modules/apriltag_detector.py). Same interface
(detect_tags / find_target_tag / draw_visualization) so it can grow into the follower later.
Two things are tuned for THIS setup:
  * camera intrinsics default to the ESP32-CAM's QVGA (320x240) frame, not the reference's
    640x480 USB cam. fx/fy are a rough estimate — see CALIBRATION note below.
  * detection runs on the grayscale QVGA MJPEG frames the board streams.

Tag being tracked (from the reference): family tag25h9, ID 12, size 0.15 m.
"""
import cv2
import numpy as np
from collections import deque
from pupil_apriltags import Detector

# ---- target tag ----
TARGET_TAG_FAMILY = 'tag25h9'
TARGET_TAG_ID = 12
TARGET_TAG_SIZE = 0.15          # metres (15 cm)

# ---- camera intrinsics (ESP32-CAM, QVGA 320x240) ----
# cx/cy are the image centre for a 320x240 frame.
# CALIBRATION: fx/fy below are a rough guess for a ~65 deg-HFOV lens (fx ~= 160/tan(HFOV/2)).
# Distance/angle are only as good as these numbers — run a proper cv2 checkerboard
# calibration on the board and drop the real fx,fy,cx,cy here for accurate pose. Detection,
# tag ID, centre and pixel size do NOT depend on them.
FX = 250.0
FY = 250.0
CX = 160.0
CY = 120.0

GRID_ROWS = 10
GRID_COLS = 10

# module-level smoothing memory (mirrors the reference)
last_position_x = None
last_position_y = None


class AprilTagDetector:
    def __init__(self, family=TARGET_TAG_FAMILY, tag_id=TARGET_TAG_ID, tag_size=TARGET_TAG_SIZE,
                 camera_params=(FX, FY, CX, CY)):
        self.detector = Detector(
            families=family, nthreads=1, quad_decimate=1.0, quad_sigma=0.0,
            refine_edges=1, decode_sharpening=0.25, debug=0)
        self.target_tag_id = tag_id
        self.target_tag_size = tag_size
        self.camera_params = list(camera_params)

        self.grid_lines = True
        self.smoothing_alpha = 0.5
        self.min_alpha = 0.3
        self.max_alpha = 0.8
        self.position_history = deque(maxlen=10)
        self.velocity_x_filtered = 0
        self.velocity_y_filtered = 0
        self.velocity_filter_alpha = 0.2

    def detect_tags(self, image, rgb_input=True):
        """Detect AprilTags. Returns pupil_apriltags detection objects (with pose)."""
        try:
            if image.ndim == 2:
                gray = image
            elif rgb_input:
                gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            else:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            return self.detector.detect(
                gray, estimate_tag_pose=True,
                camera_params=self.camera_params, tag_size=self.target_tag_size)
        except Exception as e:                      # noqa: BLE001
            print("Error in detect_tags:", e)
            return []

    def find_target_tag(self, tags, frame_width, frame_height):
        """Pick the tag whose id == target_tag_id and summarise it."""
        target_tag = next((t for t in tags if t.tag_id == self.target_tag_id), None)
        if target_tag is None:
            return None
        try:
            center_x = int(sum(c[0] for c in target_tag.corners) / 4)
            center_y = int(sum(c[1] for c in target_tag.corners) / 4)
            area = cv2.contourArea(np.array(target_tag.corners, dtype=np.int32))

            center_x, center_y = self.adaptive_smooth_position(center_x, center_y, area)
            velocity_x, velocity_y = self.calculate_velocity(center_x, center_y)
            grid_x, grid_y = self.map_to_grid(center_x, center_y, frame_width, frame_height)

            xs = [c[0] for c in target_tag.corners]
            ys = [c[1] for c in target_tag.corners]
            x_min, x_max, y_min, y_max = min(xs), max(xs), min(ys), max(ys)
            width, height = x_max - x_min, y_max - y_min
            aspect_ratio = width / height if height > 0 else 0

            t = target_tag.pose_t                    # 3x1 translation (metres)
            distance = float(np.linalg.norm(t))
            t_x, t_y, t_z = float(t[0]), float(t[1]), float(t[2])
            angle_deg = int(np.degrees(np.arccos(t_z / distance))) if distance > 0 else 0
            azimuth = int(np.degrees(np.arctan2(t_x, t_z)))

            return {
                'tag_id': target_tag.tag_id,
                'center_x': center_x, 'center_y': center_y,
                'area': area, 'width': width, 'height': height,
                'aspect_ratio': aspect_ratio,
                'grid_position': (grid_x, grid_y),
                'bounding_box': (x_min, y_min, x_max, y_max),
                'corners': target_tag.corners,
                'velocity_x': velocity_x, 'velocity_y': velocity_y,
                'distance': distance, 'angle': angle_deg, 'azimuth': azimuth,
            }
        except Exception as e:                       # noqa: BLE001
            print("Error in find_target_tag:", e)
            return None

    def adaptive_smooth_position(self, new_x, new_y, area):
        global last_position_x, last_position_y
        if last_position_x is not None and last_position_y is not None:
            distance = np.sqrt((new_x - last_position_x) ** 2 + (new_y - last_position_y) ** 2)
            size_factor = min(1.0, area / 3000)
            if distance > 30:
                self.smoothing_alpha = min(self.max_alpha, self.smoothing_alpha + 0.05)
            elif distance < 10:
                self.smoothing_alpha = max(self.min_alpha, self.smoothing_alpha - 0.02)
            adjusted_alpha = self.smoothing_alpha * (1 - size_factor * 0.5)
        else:
            adjusted_alpha = 1.0
        if last_position_x is None:
            last_position_x, last_position_y = new_x, new_y
        else:
            last_position_x = int(adjusted_alpha * new_x + (1 - adjusted_alpha) * last_position_x)
            last_position_y = int(adjusted_alpha * new_y + (1 - adjusted_alpha) * last_position_y)
        return last_position_x, last_position_y

    def calculate_velocity(self, x, y):
        self.position_history.append((x, y))
        if len(self.position_history) >= 2:
            span = min(5, len(self.position_history) - 1)
            prev_x, prev_y = self.position_history[0]
            curr_x, curr_y = self.position_history[-1]
            raw_vx, raw_vy = (curr_x - prev_x) / span, (curr_y - prev_y) / span
            self.velocity_x_filtered = (self.velocity_filter_alpha * raw_vx +
                                        (1 - self.velocity_filter_alpha) * self.velocity_x_filtered)
            self.velocity_y_filtered = (self.velocity_filter_alpha * raw_vy +
                                        (1 - self.velocity_filter_alpha) * self.velocity_y_filtered)
            return self.velocity_x_filtered, self.velocity_y_filtered
        return 0, 0

    def map_to_grid(self, x, y, frame_width, frame_height):
        grid_x = max(0, min(int((x / frame_width) * GRID_COLS), GRID_COLS - 1))
        grid_y = max(0, min(int((y / frame_height) * GRID_ROWS), GRID_ROWS - 1))
        return grid_x, grid_y

    def draw_visualization(self, image, tag_data):
        if image is None:
            return None
        viz = image.copy()
        if self.grid_lines:
            self.draw_grid(viz)

        h, w = viz.shape[:2]
        left, right = int(w * 0.4), int(w * 0.6)
        cv2.rectangle(viz, (left, 0), (right, h), (30, 150, 30), 2)   # centre zone

        if tag_data is None:
            cv2.putText(viz, "No AprilTag (id %d) detected" % self.target_tag_id, (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return viz

        corners = np.array(tag_data['corners'], dtype=np.int32)
        cv2.polylines(viz, [corners], True, (0, 255, 0), 2)
        cx, cy = tag_data['center_x'], tag_data['center_y']
        cv2.circle(viz, (cx, cy), 5, (255, 0, 0), -1)

        lines = [
            "Tag ID: %d" % tag_data['tag_id'],
            "Pos: (%d, %d)  grid %s" % (cx, cy, tag_data['grid_position']),
            "Area: %.0f  ratio %.2f" % (tag_data['area'], tag_data['aspect_ratio']),
            "Dist: %.2f m  (approx - calibrate fx/fy)" % tag_data['distance'],
            "Angle: %d deg  azimuth %d deg" % (tag_data['angle'], tag_data['azimuth']),
        ]
        for i, txt in enumerate(lines):
            cv2.putText(viz, txt, (10, 30 + i * 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        return viz

    def draw_grid(self, image):
        h, w = image.shape[:2]
        for i in range(1, GRID_COLS):
            x = int((i / GRID_COLS) * w)
            cv2.line(image, (x, 0), (x, h), (100, 100, 100), 1)
        for i in range(1, GRID_ROWS):
            y = int((i / GRID_ROWS) * h)
            cv2.line(image, (0, y), (w, y), (100, 100, 100), 1)
