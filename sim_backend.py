"""
MuJoCo simulation backend for the SO-101 robotic arm.
SimArmController is a drop-in replacement for ArmController with
real-time physics, 3D viewer, and offscreen camera rendering.
"""

import logging
import math
import threading
import time
from pathlib import Path

import numpy as np

log = logging.getLogger("sim")

_MUJOCO_OK = False
try:
    import mujoco
    _MUJOCO_OK = True
except ImportError:
    pass

MODEL_PATH = str(Path(__file__).parent / "scene.xml")

_GLFW_OK = False
_GLFW_MODULE = None
_GLFW_INITED = False
_GLFW_INIT_LOCK = threading.Lock()
try:
    import mujoco.glfw as _glfw_mod
    _GLFW_MODULE = _glfw_mod.glfw
    _GLFW_OK = True
except ImportError:
    pass


class SimArmController:
    """Drop-in replacement for ArmController using MuJoCo physics + 3D viewer."""

    N_JOINTS = 5  # 5 arm joints; gripper is handled separately via set_gripper → qpos[5]

    def __init__(self, port: str = "/dev/ttyACM0", baud: int = 1000000, config=None, role=None):
        self.cfg = config
        self._role = role
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._renderer: mujoco.Renderer | None = None
        self._viewer = None
        self._viewer_thread: threading.Thread | None = None
        self._running = False
        self._connected = False
        self._sim = True
        self._positions = [0.0] * self.N_JOINTS
        self._gripper = 50.0
        self._cal = {}
        self._lock = threading.RLock()

    def connect(self) -> bool:
        if not _MUJOCO_OK:
            log.error("MuJoCo not installed — run: pip install mujoco")
            return False
        if not Path(MODEL_PATH).exists():
            log.error("MJCF model not found: %s", MODEL_PATH)
            return False

        with self._lock:
            self._model = mujoco.MjModel.from_xml_path(MODEL_PATH)
            self._data = mujoco.MjData(self._model)
            self.go_home()
            self.set_gripper(50)
            mujoco.mj_forward(self._model, self._data)

        self._running = True
        self._connected = True
        self._viewer_thread = threading.Thread(target=self._run_viewer, daemon=True)
        self._viewer_thread.start()

        log.info("[SIM] MuJoCo simulation started (3D viewer should appear)")
        return True

    def disconnect(self):
        self._running = False
        self._connected = False
        with self._lock:
            if self._renderer:
                self._renderer.close()
                self._renderer = None
            self._model = None
            self._data = None
        log.info("[SIM] Simulation stopped")

    def is_connected(self) -> bool:
        return self._connected

    def _run_viewer(self):
        """GLFW-based 3D viewer. Runs in a daemon thread."""
        global _GLFW_INITED
        g = _GLFW_MODULE
        if not _GLFW_OK:
            log.info("[SIM] glfw not available — running headless")
            self._headless_loop()
            return
        with _GLFW_INIT_LOCK:
            if not _GLFW_INITED:
                try:
                    if not g.init():
                        log.info("[SIM] GLFW init failed — running headless")
                        self._headless_loop()
                        return
                    _GLFW_INITED = True
                except Exception:
                    log.info("[SIM] GLFW init failed — running headless")
                    self._headless_loop()
                    return
        window = g.create_window(800, 600, "SO-101 MuJoCo Sim", None, None)
        if not window:
            log.warning("[SIM] GLFW window creation failed — running headless")
            self._headless_loop()
            return
        try:
            g.init()
        except Exception:
            log.info("[SIM] GLFW init failed — running headless")
            self._headless_loop()
            return
        window = g.create_window(800, 600, "SO-101 MuJoCo Sim", None, None)
        if not window:
            log.warning("[SIM] GLFW window creation failed — running headless")
            self._headless_loop()
            return

        g.make_context_current(window)
        with self._lock:
            opt = mujoco.MjvOption()
            mujoco.mjv_defaultOption(opt)
            scn = mujoco.MjvScene(self._model, maxgeom=1000)
            cam = mujoco.MjvCamera()
            mujoco.mjv_defaultFreeCamera(self._model, cam)
            cam.distance = 0.6
            cam.azimuth = 135
            cam.elevation = -25
            cam.lookat[:] = [0.15, 0.0, 0.18]
            ctx = mujoco.MjrContext(self._model, mujoco.mjtFontScale.mjFONTSCALE_150)

        g.set_scroll_callback(window, lambda w, xo, yo: setattr(
            cam, 'distance', max(0.1, cam.distance * (1 - yo * 0.1))
        ))

        while self._running and not g.window_should_close(window):
            buf_w, buf_h = g.get_framebuffer_size(window)
            rect = mujoco.MjrRect(0, 0, buf_w, buf_h)
            with self._lock:
                if self._data is None:
                    break
                mujoco.mj_forward(self._model, self._data)
                mujoco.mjv_updateScene(self._model, self._data, opt, None, cam,
                                        mujoco.mjtCatBit.mjCAT_ALL, scn)
            mujoco.mjr_render(rect, scn, ctx)
            g.swap_buffers(window)
            g.poll_events()

        g.destroy_window(window)
        self._viewer = None

    def _headless_loop(self):
        while self._running:
            with self._lock:
                if self._data is not None:
                    mujoco.mj_forward(self._model, self._data)
            time.sleep(0.01)

    def _role_cfg(self, key: str):
        """Return config value, respecting arm role (leader/follower)."""
        if self.cfg:
            rk = self.cfg.role_key(key, self._role)
            return getattr(self.cfg, rk) if hasattr(self.cfg, rk) else getattr(self.cfg, key)
        return None

    def set_joint(self, joint_id: int, angle: float):
        if not _MUJOCO_OK:
            return
        if not 0 <= joint_id < self.N_JOINTS:
            return
        lo, hi = self._role_cfg("JOINT_LIMITS")[joint_id] if self.cfg else (-180, 180)
        angle = max(float(lo), min(float(hi), float(angle)))
        self._positions[joint_id] = angle
        with self._lock:
            if self._data is not None:
                self._data.qpos[joint_id] = math.radians(angle)

    def set_joints(self, angles):
        for i, a in enumerate(list(angles)[: self.N_JOINTS]):
            self.set_joint(i, float(a))

    def set_gripper(self, pct: float):
        pct = max(0.0, min(100.0, float(pct)))
        self._gripper = pct
        with self._lock:
            if self._data is not None:
                GRIPPER_RANGE = 1.7453292 - (-0.174533)
                gap = -0.174533 + (pct / 100.0) * GRIPPER_RANGE
                self._data.qpos[5] = gap

    def go_home(self):
        angles = self._role_cfg("HOME_ANGLES") if self.cfg else [0.0] * self.N_JOINTS
        self.set_joints(angles)

    def go_zero(self):
        self.set_joints([0.0] * self.N_JOINTS)

    def set_torque(self, enable: bool):
        pass  # no-op for simulation

    def emergency_stop(self):
        log.warning("[SIM] E-STOP (no-op)")

    def get_state(self) -> dict:
        return {
            "joints": list(self._positions),
            "gripper": round(self._gripper, 1),
            "tcp": self._fk(self._positions),
            "current": [0.0] * self.N_JOINTS,
            "temperature": [25.0] * self.N_JOINTS,
            "voltage": 12.0,
            "mode": "simulation (mujoco)",
        }

    def calibrate_step(self, step: int) -> dict:
        return {"ok": True, "msg": "No calibration needed in simulation"}

    def save_calibration(self, path: str):
        pass

    def render_frame(self, camera_name: str, width: int, height: int) -> np.ndarray | None:
        if not _MUJOCO_OK:
            return None
        with self._lock:
            if self._model is None or self._data is None:
                return None
            if self._renderer is None or self._renderer.width != width or self._renderer.height != height:
                if self._renderer:
                    self._renderer.close()
                self._renderer = mujoco.Renderer(self._model, height=height, width=width)
            try:
                mujoco.mj_forward(self._model, self._data)
                self._renderer.update_scene(self._data, camera=camera_name)
                return self._renderer.render()
            except Exception as e:
                log.debug("Render error: %s", e)
                return None

    @staticmethod
    def _fk(joints: list) -> dict:
        L = [117.0, 130.0, 124.0, 60.0]
        cum = 0.0
        x = 0.0
        z = 0.0
        for i, length in enumerate(L):
            cum += math.radians(float(joints[i])) if i < len(joints) else 0.0
            x += length * math.cos(cum)
            z += length * math.sin(cum)
        return {"x": round(x, 1), "y": 0.0, "z": round(z, 1)}
