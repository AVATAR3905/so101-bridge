"""
SO-101 Bridge Configuration
Edit config.json in the project root, or pass env vars prefixed with SO101_.
"""

import json
import os
from pathlib import Path

_DEFAULTS = {
    # WebSocket
    "WS_HOST": "0.0.0.0",
    "WS_PORT": 8765,

    # Serial / Robot
    "DEFAULT_PORT": "/dev/ttyACM0",
    "DEFAULT_BAUD": 1000000,
    "ROBOT_TYPE": "so101",          # so101 | so100 | lekiwi
    "MOTOR_IDS": [1, 2, 3, 4, 5, 6],  # 1-5 arm joints + 6 gripper
    "HOME_ANGLES": [0.0, -34.4, -45.8, -22.9, 0.0],
    "JOINT_LIMITS": [
        [-180, 180],  # shoulder pan
        [-90,   90],  # shoulder lift
        [-135, 135],  # elbow
        [-90,   90],  # wrist pitch
        [-180, 180],  # wrist roll
    ],
    "GRIPPER_OPEN":  0.0,    # PWM / position units (calibrated at runtime)
    "GRIPPER_CLOSED": 100.0,

    # Cameras
    "CAMERA_DEVICES": [0, 2],      # /dev/video0, /dev/video2
    "CAMERA_WIDTH":  1280,
    "CAMERA_HEIGHT":  720,
    "CAMERA_FPS":      30,
    "CAMERA_CODEC":  "MJPG",

    # Recording
    "RECORD_FPS": 50,
    "DATASET_DIR": str(Path.home() / "datasets" / "so101"),
    "DATASET_FORMAT": "lerobot_v2",   # lerobot_v2 | rlds | hdf5

    # Per-arm calibration (same defaults as JOINT_LIMITS / GRIPPER_* / HOME_ANGLES)
    "LEADER_JOINT_LIMITS": [[-180, 180], [-90, 90], [-135, 135], [-90, 90], [-180, 180]],
    "LEADER_GRIPPER_OPEN": 0.0,
    "LEADER_GRIPPER_CLOSED": 100.0,
    "LEADER_HOME_ANGLES": [0.0, -34.4, -45.8, -22.9, 0.0],
    "FOLLOWER_JOINT_LIMITS": [[-180, 180], [-90, 90], [-135, 135], [-90, 90], [-180, 180]],
    "FOLLOWER_GRIPPER_OPEN": 0.0,
    "FOLLOWER_GRIPPER_CLOSED": 100.0,
    "FOLLOWER_HOME_ANGLES": [0.0, -34.4, -45.8, -22.9, 0.0],

    # Calibration
    "CAL_FILE": "calibration.json",

    # Safety
    "MAX_VELOCITY":  30.0,    # deg/s
    "MAX_TORQUE_PCT": 60.0,
    "COLLISION_DETECTION": True,

    # Simulation
    "SIM_MODE": None,          # null = auto, "mujoco" = force MuJoCo sim

    # Teleoperation
    "LEADER_PORT": "/dev/ttyACM0",   # leader arm serial port
    "FOLLOWER_PORT": "/dev/ttyACM1", # follower arm serial port (or null = sim)
    "TELEOP_RATE": 100,              # leader→follower control rate (Hz)
}


class Config:
    _FILE = Path(__file__).parent / "config.json"

    def __init__(self):
        self._data = dict(_DEFAULTS)
        # Load from file
        if self._FILE.exists():
            try:
                with open(self._FILE) as f:
                    self._data.update(json.load(f))
            except Exception as e:
                print(f"[config] Warning: could not load config.json: {e}")
        # Override from environment
        for key in _DEFAULTS:
            env_val = os.environ.get(f"SO101_{key}")
            if env_val is not None:
                try:
                    self._data[key] = json.loads(env_val)
                except json.JSONDecodeError:
                    self._data[key] = env_val

    def __getattr__(self, name):
        if name.startswith("_"):
            raise AttributeError(name)
        try:
            return self._data[name]
        except KeyError:
            raise AttributeError(f"Config has no key '{name}'")

    def role_key(self, key: str, role: str | None = None) -> str:
        """Return '{ROLE}_{KEY}' when role is set and key exists, else 'KEY'."""
        if role:
            prefixed = f"{role}_{key}"
            if prefixed in self._data:
                return prefixed
        return key

    def update(self, patch: dict):
        self._data.update(patch)

    def reload(self):
        with open(self._FILE) as f:
            self._data = json.load(f)

    def save(self):
        with open(self._FILE, "w") as f:
            json.dump(self._data, f, indent=2)

    def to_dict(self) -> dict:
        return dict(self._data)
