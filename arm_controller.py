"""
ArmController for lerobot 0.5.1 — handles both old and new API styles.

lerobot 0.5.1 has TWO motor APIs:
  NEW (preferred): lerobot.motors.feetech  — Motor dataclass, MotorNormMode
  OLD (fallback):  lerobot.common.robot_devices.motors.feetech — tuple motors

We try new first, fall back to old, fall back to simulation.
"""

import json
import logging
import time
from pathlib import Path

log = logging.getLogger("arm")

# ── API detection ─────────────────────────────────────────────────────────────
API = None          # "new" | "old" | None
FeetechMotorsBus = None
Motor            = None
MotorNormMode    = None

# Try NEW API (lerobot >= 0.4 / 0.5)
try:
    from lerobot.motors.feetech import FeetechMotorsBus as _FB  # type: ignore
    from lerobot.motors.motors_bus import Motor as _M  # type: ignore
    from lerobot.motors.motors_bus import MotorNormMode as _MNM  # type: ignore
    FeetechMotorsBus = _FB
    Motor            = _M
    MotorNormMode    = _MNM
    API = "new"
    log.info("lerobot motors API: NEW  (lerobot.motors.motors_bus)")
except ImportError:
    pass

# Try OLD API (lerobot <= 0.3)
if API is None:
    try:
        from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus as _FB
        FeetechMotorsBus = _FB
        Motor            = None
        API = "old"
        log.info("lerobot motors API: OLD  (lerobot.common.robot_devices.motors.feetech)")
    except ImportError:
        pass

if API is None:
    log.warning("No lerobot motor driver found — SIMULATION mode only")
    log.warning("Fix: pip install lerobot")

MOTOR_MODEL   = "sts3215"
JOINT_NAMES   = [f"joint_{i+1}" for i in range(5)]
GRIPPER_NAME  = "joint_6"
TICKS_CENTER  = 2048
TICKS_PER_DEG = 4096 / 360.0


def _make_motors_dict() -> dict:
    """Build the motors dict in the format expected by the detected API."""
    if API == "new" and Motor is not None:
        norm = MotorNormMode.RANGE_M100_100 if MotorNormMode else None
        d = {}
        for i, name in enumerate(JOINT_NAMES):
            d[name] = Motor(i + 1, MOTOR_MODEL, norm) if norm else Motor(i + 1, MOTOR_MODEL)
        gripper_id = len(JOINT_NAMES) + 1
        d[GRIPPER_NAME] = Motor(gripper_id, MOTOR_MODEL, norm) if norm else Motor(gripper_id, MOTOR_MODEL)
        return d
    else:
        # OLD API: dict of {name: (id, model_str)}
        d = {name: (i + 1, MOTOR_MODEL) for i, name in enumerate(JOINT_NAMES)}
        d[GRIPPER_NAME] = (len(JOINT_NAMES) + 1, MOTOR_MODEL)
        return d


class ArmController:

    N_JOINTS = 5

    def __init__(self, port: str, baud: int, config, role: str | None = None):
        self.port  = port
        self.baud  = baud
        self.cfg   = config
        self._role = role

        self._bus        = None
        self._connected  = False
        self._sim        = True

        self._positions  = [0.0] * self.N_JOINTS
        self._gripper    = 50.0
        self._cal        = {}

    def _role_cfg(self, key: str):
        """Return config value, respecting arm role (leader/follower)."""
        rk = self.cfg.role_key(key, self._role)
        return getattr(self.cfg, rk) if hasattr(self.cfg, rk) else getattr(self.cfg, key)

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self):
        if API is None:
            self._sim = True
            self._connected = True
            log.info("[SIM] lerobot not available")
            return

        motors = _make_motors_dict()
        log.info("Connecting via %s API on %s @ %d …", API, self.port, self.baud)

        try:
            if API == "new":
                bus = FeetechMotorsBus(port=self.port, motors=motors)
                # Disable built-in normalization — arm_controller handles deg↔tick conversion
                bus.normalized_data = []
            else:
                # OLD API may need a config object
                try:
                    from lerobot.common.robot_devices.motors.configs import FeetechMotorsBusConfig
                    cfg_obj = FeetechMotorsBusConfig(port=self.port, motors=motors)
                    bus = FeetechMotorsBus(cfg_obj)
                except ImportError:
                    bus = FeetechMotorsBus(port=self.port, motors=motors)

            bus.connect()
            self._bus       = bus
            self._sim       = False
            self._connected = True
            log.info("Hardware connected on %s", self.port)

        except Exception as e:
            self._bus       = None
            self._sim       = True
            self._connected = True
            log.error("Bus connect failed: %s", e)
            log.warning("Running in SIMULATION mode — check port and dialout group")
            return

        # Enable torque on all motors
        for name in JOINT_NAMES + [GRIPPER_NAME]:
            try:
                self._write("Torque_Enable", 1, name)
            except Exception as e:
                log.warning("Torque_Enable failed for %s: %s", name, e)

        # Load calibration
        cal_path = Path(self.cfg.CAL_FILE)
        if cal_path.exists():
            try:
                with open(cal_path) as f:
                    self._cal = json.load(f)
                log.info("Calibration loaded")
            except Exception as e:
                log.warning("Calibration load error: %s", e)

        # Seed position cache
        self._positions = self._read_all_joints()

    def disconnect(self):
        if self._bus is not None:
            for name in JOINT_NAMES + [GRIPPER_NAME]:
                try:
                    self._write("Torque_Enable", 0, name)
                except Exception:
                    pass
            try:
                self._bus.disconnect()
            except Exception as e:
                log.warning("Disconnect: %s", e)
        self._bus       = None
        self._connected = False
        self._sim       = True
        log.info("Arm disconnected")

    def is_connected(self) -> bool:
        return self._connected

    # ── Low-level read/write helpers ──────────────────────────────────────────

    def _write(self, register: str, value, motor_name: str, normalize: bool = False):
        """Write a register. Handles both API styles."""
        if self._bus is None:
            return
        if API == "new":
            self._bus.write(register, motor_name, value, normalize=normalize)
        else:
            self._bus.write(register, motor_name, value)

    def _read(self, register: str, motor_name: str, normalize: bool = True):
        """
        Read a register for one motor. Returns a scalar float.
        New API returns numpy arrays; old API returns scalar or array.
        """
        if self._bus is None:
            raise RuntimeError("Bus not connected")

        if API == "new":
            raw = self._bus.read(register, motor_name, normalize=normalize)
        else:
            raw = self._bus.read(register, motor_name)

        # Unwrap numpy arrays / lists
        if hasattr(raw, '__len__'):
            return float(raw[0])
        return float(raw)

    def _safe_read(self, register: str, motor_name: str, default: float = 0.0) -> float:
        try:
            return self._read(register, motor_name)
        except Exception as e:
            log.debug("Read %s[%s] failed: %s", register, motor_name, e)
            return default

    # ── Joint control ─────────────────────────────────────────────────────────

    def set_joint(self, joint_id: int, angle: float):
        if not 0 <= joint_id < self.N_JOINTS:
            return
        lo, hi = self._role_cfg("JOINT_LIMITS")[joint_id]
        angle = max(float(lo), min(float(hi), float(angle)))
        self._positions[joint_id] = angle
        if self._bus is not None:
            try:
                self._write("Goal_Position",
                            self._deg_to_ticks(angle),
                            JOINT_NAMES[joint_id])
            except Exception as e:
                log.warning("set_joint[%d] error: %s", joint_id, e)

    def set_joints(self, angles):
        for i, a in enumerate(list(angles)[:self.N_JOINTS]):
            self.set_joint(i, float(a))

    def go_home(self):
        self.set_joints(self._role_cfg("HOME_ANGLES"))
        log.info("→ HOME")

    def go_zero(self):
        self.set_joints([0.0] * self.N_JOINTS)
        log.info("→ ZERO")

    def emergency_stop(self):
        log.warning("E-STOP")
        if self._bus is not None:
            for name in JOINT_NAMES + [GRIPPER_NAME]:
                try:
                    self._write("Torque_Enable", 0, name)
                except Exception:
                    pass

    # ── Torque control (leader mode) ──────────────────────────────────────────

    def set_torque(self, enable: bool):
        """Enable or disable torque on all motors."""
        val = 1 if enable else 0
        for name in JOINT_NAMES + [GRIPPER_NAME]:
            try:
                self._write("Torque_Enable", val, name)
            except Exception as e:
                log.debug("set_torque(%s, %d): %s", name, val, e)

    # ── Gripper ───────────────────────────────────────────────────────────────

    def set_gripper(self, pct: float):
        pct = max(0.0, min(100.0, float(pct)))
        self._gripper = pct
        if self._bus is not None:
            try:
                gripper_open   = float(self._role_cfg("GRIPPER_OPEN"))
                gripper_closed = float(self._role_cfg("GRIPPER_CLOSED"))
                span  = gripper_closed - gripper_open
                ticks = int(gripper_open + span * pct / 100.0)
                self._write("Goal_Position", ticks, GRIPPER_NAME)
            except Exception as e:
                log.warning("set_gripper error: %s", e)

    # ── State / telemetry ─────────────────────────────────────────────────────

    def get_state(self) -> dict:
        if self._bus is None:
            return {
                "joints":      list(self._positions),
                "gripper":     round(self._gripper, 1),
                "tcp":         self._fk(self._positions),
                "current":     [0.0] * self.N_JOINTS,
                "temperature": [0.0] * self.N_JOINTS,
                "voltage":     0.0,
                "mode":        "simulation",
            }

        joints = self._read_all_joints()
        self._positions = joints

        currents = [self._safe_read("Present_Current",     n) * 0.01 for n in JOINT_NAMES]
        temps    = [self._safe_read("Present_Temperature", n)         for n in JOINT_NAMES]
        voltage  =  self._safe_read("Present_Voltage", JOINT_NAMES[0]) * 0.1

        # Gripper position → percentage
        gripper_raw = self._safe_read("Present_Position", GRIPPER_NAME, None)
        if gripper_raw is not None:
            gripper_open   = float(self._role_cfg("GRIPPER_OPEN"))
            gripper_closed = float(self._role_cfg("GRIPPER_CLOSED"))
            span = gripper_closed - gripper_open
            gripper_pct = round((gripper_raw - gripper_open) / span * 100, 1) if span else 50.0
        else:
            gripper_pct = self._gripper

        return {
            "joints":      joints,
            "gripper":     max(0.0, min(100.0, gripper_pct)),
            "tcp":         self._fk(joints),
            "current":     [round(c, 3) for c in currents],
            "temperature": [round(t, 1) for t in temps],
            "voltage":     round(voltage, 2),
            "mode":        "hardware",
        }

    def _read_all_joints(self) -> list:
        angles = []
        for i, name in enumerate(JOINT_NAMES):
            try:
                raw = self._read("Present_Position", name)
                angles.append(self._ticks_to_deg(int(raw)))
            except Exception as e:
                log.debug("Read joint %s: %s", name, e)
                angles.append(self._positions[i] if i < len(self._positions) else 0.0)
        return angles

    # ── Calibration ───────────────────────────────────────────────────────────

    def calibrate_step(self, step: int) -> dict:
        fns = [self._cal_home, self._cal_zero, self._cal_rom,
               self._cal_stiffness, self._cal_gripper]
        if 0 <= step < len(fns):
            return fns[step]()
        return {"ok": False, "msg": f"Unknown step {step}"}

    def _cal_home(self) -> dict:
        self.go_home(); time.sleep(1.0)
        return {"ok": True, "msg": "Home position set"}

    def _cal_zero(self) -> dict:
        offsets = {}
        if self._bus is not None:
            for name in JOINT_NAMES:
                offsets[name] = self._safe_read("Present_Position", name, TICKS_CENTER)
            self._cal["zero_offsets"] = offsets
        return {"ok": True, "msg": "Zero offsets recorded", "offsets": offsets}

    def _cal_rom(self) -> dict:
        return {"ok": True, "msg": "Manually sweep each joint through full range, then click Next"}

    def _cal_stiffness(self) -> dict:
        if self._bus is not None:
            val = int(float(self.cfg.MAX_TORQUE_PCT) * 10)
            for name in JOINT_NAMES:
                try:
                    self._write("Maximum_Acceleration", val, name)
                except Exception as e:
                    log.warning("Stiffness %s: %s", name, e)
        return {"ok": True, "msg": f"Torque set to {self.cfg.MAX_TORQUE_PCT}%"}

    def _cal_gripper(self) -> dict:
        if self._bus is None:
            return {"ok": False, "msg": "No bus connected"}
        # Write extremes to find mechanical limits
        self._write("Goal_Position", 500, GRIPPER_NAME)
        time.sleep(1.5)
        open_ticks = self._safe_read("Present_Position", GRIPPER_NAME, 1500)
        self._write("Goal_Position", 3500, GRIPPER_NAME)
        time.sleep(1.5)
        close_ticks = self._safe_read("Present_Position", GRIPPER_NAME, 2500)
        # Return to middle
        mid = (open_ticks + close_ticks) // 2
        self._write("Goal_Position", mid, GRIPPER_NAME)
        time.sleep(0.5)
        self._cal["gripper_open"] = int(open_ticks)
        self._cal["gripper_closed"] = int(close_ticks)
        return {"ok": True, "msg": f"Gripper: open={int(open_ticks)} ticks, close={int(close_ticks)} ticks",
                "gripper_open": int(open_ticks), "gripper_closed": int(close_ticks)}

    def save_calibration(self, path: str):
        # Save gripper ticks to config (role-aware)
        if "gripper_open" in self._cal and "gripper_closed" in self._cal:
            gk_open   = self.cfg.role_key("GRIPPER_OPEN", self._role)
            gk_closed = self.cfg.role_key("GRIPPER_CLOSED", self._role)
            self.cfg._data[gk_open]   = float(self._cal["gripper_open"])
            self.cfg._data[gk_closed] = float(self._cal["gripper_closed"])
            self.cfg.save()
        with open(path, "w") as f:
            json.dump(self._cal, f, indent=2)
        log.info("Calibration saved to %s", path)

    # ── Unit helpers (old API only) ───────────────────────────────────────────

    @staticmethod
    def _deg_to_ticks(deg: float) -> int:
        return int(TICKS_CENTER + float(deg) * TICKS_PER_DEG)

    @staticmethod
    def _ticks_to_deg(ticks: int) -> float:
        return round((int(ticks) - TICKS_CENTER) / TICKS_PER_DEG, 2)

    @staticmethod
    def _fk(joints: list) -> dict:
        import math
        L   = [117.0, 130.0, 124.0, 60.0]
        cum = 0.0; x = 0.0; z = 0.0
        for i, length in enumerate(L):
            cum += math.radians(float(joints[i])) if i < len(joints) else 0.0
            x   += length * math.cos(cum)
            z   += length * math.sin(cum)
        return {"x": round(x, 1), "y": 0.0, "z": round(z, 1)}
