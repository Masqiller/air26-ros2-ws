"""ESP32Camera — read frames from the ESP32-CAM MJPEG HTTP stream.

Gives a cv2.VideoCapture-like interface (isOpened / read / release) but pulls from the
board's MJPEG stream (http://<ip>/stream). A background thread keeps the latest frame, so
read() is always low-latency and the ESP32-CAM's quirky single-client HTTP is handled
robustly (auto-reconnects if the stream drops).

NOTE: the ESP32-CAM serves only ONE HTTP client at a time. If RViz's camera_stream is
running, stop it before using this, or the stream can't be grabbed here.
"""
import threading
import time

import cv2
import numpy as np
import requests


class ESP32Camera:
    def __init__(self, url):
        self.url = url
        self._frame = None
        self._ok = False
        self._stop = False
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop:
            try:
                with requests.get(self.url, stream=True, timeout=5) as resp:
                    resp.raise_for_status()
                    buf = b""
                    for chunk in resp.iter_content(chunk_size=4096):
                        if self._stop:
                            return
                        buf += chunk
                        start = buf.find(b"\xff\xd8")      # JPEG SOI
                        end = buf.find(b"\xff\xd9")        # JPEG EOI
                        if start != -1 and end != -1 and end > start:
                            jpg = buf[start:end + 2]
                            buf = buf[end + 2:]
                            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                            if img is not None:
                                with self._lock:
                                    self._frame = img
                                    self._ok = True
            except Exception:                              # noqa: BLE001 - keep retrying
                self._ok = False
                time.sleep(1.0)

    def isOpened(self):
        # wait up to ~5 s for the first frame to arrive
        for _ in range(50):
            with self._lock:
                if self._frame is not None:
                    return True
            time.sleep(0.1)
        return False

    def read(self):
        with self._lock:
            if self._frame is None:
                return False, None
            return True, self._frame.copy()

    def release(self):
        self._stop = True
