#!/usr/bin/env python3
"""AprilTag detection test on the Platoon ESP32-CAM feed.

Same flow as Masqiller/AprilTag_Follower's test_apriltag.py, but the camera source is the
ESP32-CAM MJPEG stream instead of a USB webcam. Detects the target tag (family tag25h9,
id 12, 0.08 m), draws the overlay, and shows it live.

  cd ~/air26-ros2-ws/apriltag_follower
  python3 test_apriltag.py                          # uses http://192.168.0.117/stream
  python3 test_apriltag.py --url http://<cam-ip>/stream

Keys:  q = quit,  s = save a snapshot (apriltag_test.jpg)

IMPORTANT: the ESP32-CAM serves ONE HTTP client at a time. If RViz's camera feed
(camera_stream / hardware.launch.py) is running, stop it first or this can't grab the stream.
"""
import argparse

import cv2

from modules.apriltag_detector import AprilTagDetector
from modules.esp32_camera import ESP32Camera

DEFAULT_URL = "http://192.168.0.117/stream"


def test_apriltag_detection(url):
    """Verify the camera can see and identify the AprilTag."""
    print("AprilTag Detection Test - Press 'q' to quit, 's' to save image")

    # Open the ESP32-CAM stream (drop-in for cv2.VideoCapture on the real webcam)
    cap = ESP32Camera(url)
    if not cap.isOpened():
        print("Error: Could not open ESP32-CAM stream at %s" % url)
        print("       (is the cam on WiFi? is RViz's camera_stream holding the connection?)")
        return

    # Create AprilTag detector
    detector = AprilTagDetector()

    while True:
        # Capture frame
        ret, frame = cap.read()
        if not ret:
            continue

        # Convert to RGB (for consistency with the reference program)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Detect AprilTags
        tags = detector.detect_tags(rgb_frame)

        # Get image dimensions
        height, width = frame.shape[:2]

        # Find the target tag (id 12)
        tag_data = detector.find_target_tag(tags, width, height)

        # Create visualization
        viz_image = detector.draw_visualization(rgb_frame, tag_data)

        # Help + count overlays
        cv2.putText(viz_image, "Press 's' to save image, 'q' to quit", (10, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(viz_image, "Detected tags: %d" % len(tags), (10, height - 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Convert back to BGR for display
        display_image = cv2.cvtColor(viz_image, cv2.COLOR_RGB2BGR)

        # Show the image
        cv2.imshow("AprilTag Test", display_image)

        # Keypresses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite("apriltag_test.jpg", display_image)
            print("Image saved as 'apriltag_test.jpg'")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=DEFAULT_URL, help="ESP32-CAM MJPEG stream URL")
    args = ap.parse_args()
    test_apriltag_detection(args.url)
