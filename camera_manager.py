"""
CameraManager — handles multiple OpenCV camera streams in background threads,
provides latest frames for recording and MJPEG streaming.
"""

import base64
import logging
import threading
import time

import numpy as np

log = logging.getLogger("cameras")

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    log.warning("opencv-python not found — cameras in simulation mode")


class CameraStream:
    """Single camera capture thread."""

    def __init__(self, device_id: int, width: int, height: int, fps: int, codec: str,
                 sim_frame_callback=None):
        self.device_id = device_id
        self.width = width
        self.height = height
        self.fps = fps
        self.codec = codec
        self._sim_frame_callback = sim_frame_callback

        self._cap = None
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._running = False
        self._frame_count = 0
        self._t_start = 0.0
        self._sim = not CV2_AVAILABLE

    def start(self):
        self._running = True
        self._t_start = time.time()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        log.info("Camera %s started (%dx%d @ %d fps)", self.device_id, self.width, self.height, self.fps)

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()
        log.info("Camera %s stopped", self.device_id)

    def _capture_loop(self):
        if self._sim:
            self._sim_loop()
            return
        self._cap = cv2.VideoCapture(self.device_id)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS,          self.fps)
        fourcc = cv2.VideoWriter_fourcc(*self.codec)
        self._cap.set(cv2.CAP_PROP_FOURCC, fourcc)

        if not self._cap.isOpened():
            log.error("Cannot open camera %s — falling back to simulation", self.device_id)
            self._sim = True
            self._sim_loop()
            return

        while self._running:
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._frame = frame
                    self._frame_count += 1
            else:
                time.sleep(0.01)

    def _sim_loop(self):
        """Generate synthetic frames — from MuJoCo callback or gradient fallback."""
        while self._running:
            frame = None
            if self._sim_frame_callback:
                frame = self._sim_frame_callback(
                    self.device_id, self.width, self.height
                )
            if frame is None:
                h, w = self.height, self.width
                t = time.time()
                frame = np.zeros((h, w, 3), dtype=np.uint8)
                xs = np.linspace(0, 1, w)
                ys = np.linspace(0, 1, h)
                xg, yg = np.meshgrid(xs, ys)
                r = (np.sin(xg * 4 + t) * 30 + 30).astype(np.uint8)
                g = (np.sin(yg * 4 + t * 0.7 + 1) * 40 + 50).astype(np.uint8)
                b = np.zeros_like(r)
                frame[:, :, 0] = b
                frame[:, :, 1] = g
                frame[:, :, 2] = r
                cx, cy = w // 2, h // 2
                frame[cy-20:cy+20, cx-1:cx+2] = [0, 200, 100]
                frame[cy-1:cy+2, cx-20:cx+20] = [0, 200, 100]
                if CV2_AVAILABLE:
                    cv2.putText(frame, f"SIM CAM {self.device_id}  {t:.1f}s",
                                (10, 30), cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 220, 120), 1)
            with self._lock:
                self._frame = frame
                self._frame_count += 1
            time.sleep(1.0 / self.fps)

    def get_frame(self) -> np.ndarray | None:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def get_jpeg_b64(self, quality: int = 70) -> str | None:
        """Return the latest frame as a base64-encoded JPEG string."""
        frame = self.get_frame()
        if frame is None:
            return None
        if CV2_AVAILABLE:
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if ok:
                return base64.b64encode(buf.tobytes()).decode()
        return None

    @property
    def actual_fps(self) -> float:
        elapsed = time.time() - self._t_start
        return round(self._frame_count / elapsed, 1) if elapsed > 0 else 0.0

    @property
    def frame_count(self) -> int:
        return self._frame_count


class CameraManager:
    """Manages multiple CameraStream instances."""

    def __init__(self, devices: list, config, sim_frame_callback=None):
        self.cfg = config
        self._sim_frame_callback = sim_frame_callback
        self._streams: dict[int, CameraStream] = {}
        for dev in devices:
            self._streams[dev] = CameraStream(
                device_id=dev,
                width=config.CAMERA_WIDTH,
                height=config.CAMERA_HEIGHT,
                fps=config.CAMERA_FPS,
                codec=config.CAMERA_CODEC,
                sim_frame_callback=sim_frame_callback,
            )

    def start(self):
        for s in self._streams.values():
            s.start()

    def stop(self):
        for s in self._streams.values():
            s.stop()

    def get_frame(self, device_id: int) -> np.ndarray | None:
        s = self._streams.get(device_id)
        return s.get_frame() if s else None

    def get_all_frames(self) -> dict[int, np.ndarray | None]:
        return {dev: s.get_frame() for dev, s in self._streams.items()}

    def get_jpeg_b64(self, device_id: int, quality: int = 70) -> str | None:
        s = self._streams.get(device_id)
        return s.get_jpeg_b64(quality) if s else None

    def get_stats(self) -> dict:
        return {
            dev: {"fps": s.actual_fps, "frames": s.frame_count}
            for dev, s in self._streams.items()
        }

    @property
    def devices(self) -> list[int]:
        return list(self._streams.keys())
