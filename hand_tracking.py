"""
Hand tracking for controlling the SO-101 arm via webcam.
Uses OpenCV + MediaPipe HandLandmarker to detect hand landmarks and map them to joint angles.
"""

import logging
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

log = logging.getLogger(__name__)

try:
    import cv2
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions
    from mediapipe.tasks.python.vision import (
        HandLandmarker,
        HandLandmarkerOptions,
        RunningMode,
    )
    _MEDIAPIPE_AVAILABLE = True
except ImportError:
    _MEDIAPIPE_AVAILABLE = False

_MODEL_PATH: str = ""


def set_model_path(path: str):
    global _MODEL_PATH
    _MODEL_PATH = path


@dataclass
class HandPose:
    x: float = 0.5
    y: float = 0.5
    z: float = 0.0
    pinch: float = 0.0
    detected: bool = False


class HandTracker:
    """Opens webcam, detects hand, produces joint angles for the arm."""

    def __init__(self, camera_id: int = 0, width: int = 640, height: int = 480):
        self._camera_id = camera_id
        self._width = width
        self._height = height

        self._running = False
        self._thread: threading.Thread | None = None
        self._pose = HandPose()
        self._pose_lock = threading.Lock()

        self._frame: np.ndarray | None = None
        self._frame_lock = threading.Lock()

        self._arm_callback: Callable | None = None
        self._smooth = deque(maxlen=4)
        self._z_ref: float | None = None

    def set_arm_callback(self, cb: Callable):
        self._arm_callback = cb

    @property
    def active(self) -> bool:
        return self._running

    @property
    def latest_pose(self) -> HandPose:
        with self._pose_lock:
            return self._pose

    def get_frame(self) -> np.ndarray | None:
        with self._frame_lock:
            return self._frame.copy() if self._frame is not None else None

    def start(self) -> bool:
        if not _MEDIAPIPE_AVAILABLE:
            log.error("mediapipe not installed")
            return False
        if self._running:
            return True
        self._running = True
        self._z_ref = None
        self._smooth.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        log.info("Hand tracker started (camera %d)", self._camera_id)
        return True

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
            self._thread = None
        log.info("Hand tracker stopped")

    def _run(self):
        model_path = _MODEL_PATH or "/home/melvin/.mediapipe/models/hand_landmarker.task"
        try:
            options = HandLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=model_path),
                running_mode=RunningMode.IMAGE,
                num_hands=1,
                min_hand_detection_confidence=0.7,
                min_tracking_confidence=0.5,
            )
            landmarker = HandLandmarker.create_from_options(options)
        except Exception as e:
            log.error("Failed to create HandLandmarker: %s", e)
            self._running = False
            return

        cap = cv2.VideoCapture(self._camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)

        if not cap.isOpened():
            log.error("Cannot open camera %d", self._camera_id)
            self._running = False
            return

        landmark_indices = type("Idx", (), {
            "WRIST": 0, "THUMB_TIP": 4, "INDEX_FINGER_TIP": 8,
            "MIDDLE_FINGER_MCP": 9, "MIDDLE_FINGER_TIP": 12,
        })

        hand_connections = [
            (0, 1), (1, 2), (2, 3), (3, 4),
            (0, 5), (5, 6), (6, 7), (7, 8),
            (0, 9), (9, 10), (10, 11), (11, 12),
            (0, 13), (13, 14), (14, 15), (15, 16),
            (0, 17), (17, 18), (18, 19), (19, 20),
            (5, 9), (9, 13), (13, 17),
        ]

        while self._running:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            try:
                frame = cv2.flip(frame, 1)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                result = landmarker.detect(mp_img)

                pose = HandPose(detected=False)

                if result.hand_landmarks:
                    lm = result.hand_landmarks[0]
                    h, w, _ = frame.shape

                    for conn in hand_connections:
                        pt1 = (int(lm[conn[0]].x * w), int(lm[conn[0]].y * h))
                        pt2 = (int(lm[conn[1]].x * w), int(lm[conn[1]].y * h))
                        cv2.line(frame, pt1, pt2, (100, 200, 100), 2)
                    for i, lm_pt in enumerate(lm):
                        cx, cy = int(lm_pt.x * w), int(lm_pt.y * h)
                        cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)
                        if i in (0, 4, 8):
                            cv2.circle(frame, (cx, cy), 6, (0, 200, 255), -1)

                    wrist = lm[landmark_indices.WRIST]
                    idx_tip = lm[landmark_indices.INDEX_FINGER_TIP]
                    mid_mcp = lm[landmark_indices.MIDDLE_FINGER_MCP]
                    mid_tip = lm[landmark_indices.MIDDLE_FINGER_TIP]

                    cx = (wrist.x + mid_mcp.x) / 2
                    cy = (wrist.y + mid_mcp.y) / 2

                    if self._z_ref is None:
                        self._z_ref = wrist.z
                    z_delta = wrist.z - self._z_ref

                    # Binary gripper: distance between index tip and middle tip
                    # Fingers apart → gripper open, fingers together → gripper closed
                    fd = np.sqrt(
                        (idx_tip.x - mid_tip.x) ** 2 +
                        (idx_tip.y - mid_tip.y) ** 2 +
                        (idx_tip.z - mid_tip.z) ** 2
                    )
                    gripper_target = 1.0 if fd > 0.08 else 0.0

                    pose = HandPose(cx, cy, z_delta, gripper_target, detected=True)

                    cv2.putText(frame, f"Grip: {gripper_target:.0f}", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, f"Pos: ({cx:.2f}, {cy:.2f})", (10, 55),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)
                else:
                    cv2.putText(frame, "No hand detected", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            except Exception as e:
                log.warning("Hand tracking frame error: %s", e)
                pose = HandPose(detected=False)

            with self._pose_lock:
                self._pose = pose
            with self._frame_lock:
                self._frame = frame

            if pose.detected and self._arm_callback:
                joints = self._map_to_joints(pose)
                self._arm_callback(joints)

            time.sleep(0.03)

        cap.release()

    def _map_to_joints(self, pose: HandPose) -> dict:
        self._smooth.append([pose.x, pose.y, pose.z, pose.pinch])
        avg = np.mean(self._smooth, axis=0)
        x, y, z, pinch = avg

        j0 = (x - 0.5) * 120.0
        j1 = (0.5 - y) * 130.0
        j2 = np.clip(z * 60.0, -80.0, 80.0)
        j3 = 0.0
        j4 = 0.0
        gripper = 100.0 if pinch > 0.5 else 0.0

        return {
            "joints": [float(j0), float(j1), float(j2), j3, j4],
            "gripper": float(gripper),
        }
