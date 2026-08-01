"""
Head tracking for controlling the SO-101 arm via webcam.
Uses MediaPipe FaceLandmarker to track nose position and map to arm joints.
"""
import logging, threading, time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Optional
import numpy as np

log = logging.getLogger(__name__)

try:
    import cv2
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions
    from mediapipe.tasks.python.vision import (
        FaceLandmarker, FaceLandmarkerOptions, RunningMode,
    )
    _MP_AVAILABLE = True
except ImportError:
    _MP_AVAILABLE = False

_MODEL_PATH: str = "/home/melvin/.mediapipe/models/face_landmarker.task"

def set_model_path(path: str):
    global _MODEL_PATH
    _MODEL_PATH = path

@dataclass
class HeadPose:
    x: float = 0.5
    y: float = 0.5
    mouth_open: float = 0.0
    detected: bool = False

class HeadTracker:
    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480):
        self._camera_id = camera_id; self._width = width; self._height = height
        self._running = False; self._thread: Optional[threading.Thread] = None
        self._pose = HeadPose(); self._pose_lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None; self._frame_lock = threading.Lock()
        self._arm_callback: Optional[Callable] = None
        self._smooth = deque(maxlen=4)

    def set_arm_callback(self, cb: Callable): self._arm_callback = cb
    @property
    def active(self) -> bool: return self._running
    @property
    def latest_pose(self) -> HeadPose:
        with self._pose_lock: return self._pose
    def get_frame(self) -> Optional[np.ndarray]:
        with self._frame_lock: return self._frame.copy() if self._frame is not None else None

    def start(self) -> bool:
        if not _MP_AVAILABLE:
            log.error("mediapipe not installed"); return False
        if self._running: return True
        self._running = True; self._smooth.clear()
        self._thread = threading.Thread(target=self._run, daemon=True); self._thread.start()
        log.info("Head tracker started (camera %d)", self._camera_id)
        return True

    def stop(self):
        self._running = False
        if self._thread: self._thread.join(timeout=3); self._thread = None
        log.info("Head tracker stopped")

    def _run(self):
        try:
            opts = FaceLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=_MODEL_PATH),
                running_mode=RunningMode.IMAGE,
                num_faces=1,
                min_face_detection_confidence=0.5,
            )
            landmarker = FaceLandmarker.create_from_options(opts)
        except Exception as e:
            log.error("FaceLandmarker failed: %s", e); self._running = False; return

        cap = cv2.VideoCapture(self._camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        if not cap.isOpened():
            log.error("Cannot open camera %d", self._camera_id); self._running = False; return

        while self._running:
            ret, frame = cap.read()
            if not ret: time.sleep(0.01); continue
            try:
                frame = cv2.flip(frame, 1)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                result = landmarker.detect(mp_img)

                pose = HeadPose(detected=False)
                if result.face_landmarks:
                    lm = result.face_landmarks[0]
                    h, w, _ = frame.shape
                    # Nose tip (1), face center approximation (168)
                    nose = lm[1]
                    face_center = lm[168]
                    # Upper lip (13) and lower lip (14) for mouth open
                    upper = lm[13]; lower = lm[14]
                    mouth_dist = np.sqrt((upper.x-lower.x)**2 + (upper.y-lower.y)**2 + (upper.z-lower.z)**2)

                    # Draw key points
                    for idx in (1, 168, 13, 14):
                        cx, cy = int(lm[idx].x * w), int(lm[idx].y * h)
                        cv2.circle(frame, (cx, cy), 4, (0, 255, 255), -1)

                    # Nose relative to face center
                    dx = (nose.x - face_center.x) * 2
                    dy = (nose.y - face_center.y) * 2
                    pose = HeadPose(
                        x=np.clip(dx, -1, 1),
                        y=np.clip(dy, -1, 1),
                        mouth_open=min(1.0, mouth_dist * 5),
                        detected=True,
                    )
                    cv2.putText(frame, f"Mouth: {pose.mouth_open:.1f}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    cv2.putText(frame, "No face", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                with self._pose_lock: self._pose = pose
                with self._frame_lock: self._frame = frame

                if pose.detected and self._arm_callback:
                    self._arm_callback(self._map_to_joints(pose))
                time.sleep(0.03)
            except Exception as e:
                log.warning("Head frame error: %s", e)
                with self._frame_lock: self._frame = frame
        cap.release()

    def _map_to_joints(self, pose: HeadPose) -> dict:
        self._smooth.append([pose.x, pose.y, pose.mouth_open])
        avg = np.mean(self._smooth, axis=0)
        x, y, mouth = avg
        j0 = x * 60.0
        j1 = y * -50.0
        j2 = 0.0 if abs(mouth) < 0.5 else -60.0  # mouth open → bend elbow
        j3 = 0.0; j4 = 0.0
        gripper = 100.0 if mouth > 0.4 else 0.0  # mouth open → open gripper
        return {"joints": [float(j0), float(j1), float(j2), j3, j4], "gripper": float(gripper)}
