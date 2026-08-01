#!/usr/bin/env python3
"""
SO-101 Robotic Arm Control Bridge — main entry point
Starts:
  • WebSocket server  (port 8765)  — GUI control messages
  • MJPEG HTTP server (port 8766)  — live camera streams
"""

import asyncio
import json
import logging
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import websockets
from websockets import serve as ws_serve
# websockets v14+ uses ServerConnection; v12/v13 used WebSocketServerProtocol.
# Support both by aliasing at import time.
try:
    from websockets.asyncio.server import ServerConnection as _WSConn
except ImportError:
    try:
        from websockets.server import WebSocketServerProtocol as _WSConn  # type: ignore
    except ImportError:
        _WSConn = object  # type: ignore

from arm_controller import ArmController
from calibrate_limits import (build_bus, go_home, find_limit, find_gripper_limits,
                               ticks_to_deg, deg_to_ticks, load_config as load_cal_cfg,
                               save_config as save_cal_cfg)
from camera_manager import CameraManager
from episode_recorder import EpisodeRecorder
from config import Config
from hand_tracking import HandTracker, set_model_path as set_hand_model_path
from head_tracking import HeadTracker
from ik_solver import solve_ik
from voice_control import VoiceController
import mjpeg_server as mjpeg

try:
    from sim_backend import SimArmController as _SimAC
    _SIM_BACKEND_AVAILABLE = True
except ImportError:
    _SIM_BACKEND_AVAILABLE = False

try:
    from teleop import TeleopController as _TeleopCtl
    _TELEOP_AVAILABLE = True
except ImportError:
    _TELEOP_AVAILABLE = False

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("logs/bridge.log"),
    ],
)
log = logging.getLogger("bridge")

cfg = Config()
arm: Optional[ArmController] = None
leader_arm: Optional[ArmController] = None
follower_arm: Optional[ArmController] = None
cams: Optional[CameraManager] = None
recorder: Optional[EpisodeRecorder] = None
teleop: Optional["TeleopController"] = None
hand_tracker: Optional[HandTracker] = None
head_tracker: Optional[HeadTracker] = None
voice_controller: Optional[VoiceController] = None
ik_active: bool = False
connected_clients: set = set()
_cal_stop: threading.Event = threading.Event()
_loop: Optional[asyncio.AbstractEventLoop] = None


def _get_sim_camera_callback():
    """Return a CameraManager-compatible frame callback if arm has render_frame."""
    global arm
    if arm is None:
        return None
    render = getattr(arm, 'render_frame', None)
    if render is None:
        return None
    cam_names = getattr(cfg, "SIM_CAMERAS", ["front", "top", "side"])
    try:
        import cv2
    except ImportError:
        return None
    def _frame(dev, w, h):
        cname = cam_names[dev] if isinstance(dev, int) and dev < len(cam_names) else "front"
        rgb = render(cname, w, h)
        if rgb is None:
            return None
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    return _frame


def _hand_arm_cmd(joint_data: dict):
    """Callback from HandTracker thread — send joints to follower arm."""
    global arm
    if arm is None:
        return
    try:
        arm.set_joints(joint_data["joints"])
        arm.set_gripper(joint_data["gripper"])
    except Exception as e:
        log.warning("hand_arm_cmd error: %s", e)


def _head_arm_cmd(joint_data: dict):
    """Callback from HeadTracker thread."""
    global arm
    if arm is None:
        return
    try:
        arm.set_joints(joint_data["joints"])
        arm.set_gripper(joint_data["gripper"])
    except Exception as e:
        log.warning("head_arm_cmd error: %s", e)


async def _do_arm_cmd(cmd: str, value=None):
    """Run a single arm command on the event loop (serial-safe)."""
    global arm, recorder
    try:
        if cmd == "home" and arm:
            arm.go_home()
        elif cmd == "zero" and arm:
            arm.go_zero()
        elif cmd == "gripper" and arm:
            arm.set_gripper(value or 50)
        elif cmd == "estop" and arm:
            arm.emergency_stop()
        elif cmd == "record":
            await _handle_voice_record()
        elif cmd == "stop_record" and recorder:
            recorder.stop(); recorder = None
        elif cmd == "wave":
            await _wave_sequence()
    except Exception as e:
        log.warning("voice arm cmd error: %s", e)


def _voice_cmd_callback(action: dict):
    """Callback from VoiceController thread — schedule arm cmds on event loop."""
    cmd = action.get("cmd")
    label = cmd or "?"
    if cmd in ("home", "zero", "gripper", "estop", "record", "stop_record", "wave"):
        asyncio.run_coroutine_threadsafe(
            _do_arm_cmd(cmd, action.get("value")),
            _loop)
    if cmd == "gripper":
        label = "gripper " + str(action.get("value", 50))
    elif cmd:
        label = cmd
    asyncio.run_coroutine_threadsafe(
        broadcast({"type": "status", "voice_last": label}),
        _loop)


async def _handle_voice_record():
    """Start recording from voice command."""
    global recorder, arm
    if not arm: return
    if recorder and recorder.active: return
    name = f"voice_{int(time.time())}"
    recorder = EpisodeRecorder(arm, name, config=cfg)
    recorder.start()
    await broadcast({"type": "log", "msg": f"Voice: recording '{name}'"})


async def _wave_sequence():
    """Simple wave motion."""
    global arm
    if not arm: return
    import math
    for i in range(30):
        angle = math.sin(i * 0.3) * 20
        arm.set_joint(2, angle)
        await asyncio.sleep(0.05)
    arm.go_home()


async def broadcast(msg: dict):
    if not connected_clients:
        return
    data = json.dumps(msg)
    await asyncio.gather(
        *[ws.send(data) for ws in connected_clients],
        return_exceptions=True,
    )


async def handle_message(ws, raw: str):
    global arm, cams, recorder, teleop, leader_arm, follower_arm, hand_tracker, head_tracker, voice_controller
    try:
        msg = json.loads(raw)
    except json.JSONDecodeError:
        await ws.send(json.dumps({"type": "error", "msg": "Invalid JSON"}))
        return

    cmd = msg.get("cmd")

    if cmd == "connect":
        port = msg.get("port", cfg.DEFAULT_PORT)
        baud = int(msg.get("baud", cfg.DEFAULT_BAUD))
        if port == cfg.LEADER_PORT:
            role = "leader"
        elif port == cfg.FOLLOWER_PORT:
            role = "follower"
        else:
            role = None
        try:
            use_mujoco = cfg.SIM_MODE == "mujoco" and _SIM_BACKEND_AVAILABLE
            if use_mujoco:
                arm = _SimAC(port=port, baud=baud, config=cfg, role=role)
                arm.connect()
                await broadcast({"type": "status", "connected": True,
                                 "leader_connected": role == "leader",
                                 "follower_connected": role == "follower",
                                 "msg": "MuJoCo simulation started"})
            else:
                arm = ArmController(port=port, baud=baud, config=cfg, role=role)
                arm.connect()
                await broadcast({"type": "status", "connected": True,
                                 "leader_connected": role == "leader",
                                 "follower_connected": role == "follower",
                                 "msg": f"Connected to {port} @ {baud}"})

            # Auto-start teleop with sim follower when leader is connected
            # in hardware mode (no follower available).
            if (role == "leader" and cfg.SIM_MODE is None
                    and _SIM_BACKEND_AVAILABLE and _TELEOP_AVAILABLE
                    and not (teleop and teleop.active)):
                try:
                    follower = _SimAC(port="sim", baud=cfg.DEFAULT_BAUD,
                                      config=cfg, role="follower")
                    follower.connect()
                    teleop = _TeleopCtl(arm, follower, cfg)
                    teleop.start()
                    leader_arm = arm
                    follower_arm = follower
                    await broadcast({"type": "status", "teleop": True,
                                     "msg": "Auto-teleop: leader → sim follower"})
                except Exception as e2:
                    log.warning("Auto-teleop failed: %s", e2)
        except Exception as e:
            log.error("Connect failed: %s", e)
            await ws.send(json.dumps({"type": "error", "msg": str(e)}))

    elif cmd == "connect_arms":
        leader_port = msg.get("leader_port", cfg.LEADER_PORT)
        follower_port = msg.get("follower_port", cfg.FOLLOWER_PORT)
        baud = int(msg.get("baud", cfg.DEFAULT_BAUD))
        try:
            if teleop:
                teleop.stop()
                if hasattr(teleop.leader, 'disconnect'):
                    teleop.leader.disconnect()
                teleop = None
            if hand_tracker and hand_tracker.active:
                hand_tracker.stop()
                hand_tracker = None
            if head_tracker and head_tracker.active:
                head_tracker.stop()
                head_tracker = None
            if voice_controller:
                voice_controller.stop()
                voice_controller = None
            if arm:
                arm.disconnect()
                arm = None
            leader_arm = None
            follower_arm = None

            # Connect leader
            use_sim_leader = (cfg.SIM_MODE == "mujoco" or leader_port == "sim") and _SIM_BACKEND_AVAILABLE
            if use_sim_leader:
                leader_arm = _SimAC(port=leader_port, baud=baud, config=cfg, role="leader")
            else:
                leader_arm = ArmController(port=leader_port, baud=baud, config=cfg, role="leader")
            leader_arm.connect()

            # Connect follower
            use_sim_follower = (cfg.SIM_MODE == "mujoco" or follower_port == "sim") and _SIM_BACKEND_AVAILABLE
            if use_sim_follower:
                follower_arm = _SimAC(port=follower_port, baud=baud, config=cfg, role="follower")
            else:
                follower_arm = ArmController(port=follower_port, baud=baud, config=cfg, role="follower")
            follower_arm.connect()

            arm = follower_arm  # primary arm for control commands

            # Auto-start teleop
            if _TELEOP_AVAILABLE:
                teleop = _TeleopCtl(leader_arm, follower_arm, cfg)
                teleop.start()

            both_hw = cfg.SIM_MODE is None and not use_sim_leader and not use_sim_follower
            await broadcast({
                "type": "status", "connected": True,
                "leader_connected": True, "follower_connected": True,
                "teleop": True,
                "msg": f"Both arms connected — teleop active (leader={leader_port}, follower={follower_port})",
            })
        except Exception as e:
            log.error("connect_arms failed: %s", e)
            await ws.send(json.dumps({"type": "error", "msg": f"connect_arms failed: {e}"}))

    elif cmd == "disconnect":
        if teleop:
            teleop.stop()
            if hasattr(teleop.leader, 'disconnect'):
                teleop.leader.disconnect()
            leader_arm = None  # teleop.leader IS leader_arm
            teleop = None
        if hand_tracker and hand_tracker.active:
            hand_tracker.stop()
            hand_tracker = None
        if head_tracker and head_tracker.active:
            head_tracker.stop()
            head_tracker = None
        if voice_controller:
            voice_controller.stop()
            voice_controller = None
        if leader_arm:
            leader_arm.disconnect()
        leader_arm = None
        if arm:
            arm.disconnect()
            arm = None
        follower_arm = None
        await broadcast({"type": "status", "connected": False,
                         "leader_connected": False, "follower_connected": False,
                         "msg": "Disconnected"})

    elif cmd == "set_joint":
        if teleop and teleop.active:
            teleop.leader.set_joint(int(msg["joint"]), float(msg["angle"]))
        elif arm:
            arm.set_joint(int(msg["joint"]), float(msg["angle"]))

    elif cmd == "set_joints":
        if teleop and teleop.active:
            teleop.leader.set_joints(msg["angles"])
        elif arm:
            arm.set_joints(msg["angles"])

    elif cmd == "home":
        if arm:
            arm.go_home()
        await broadcast({"type": "log", "level": "info", "msg": "Moving to HOME"})

    elif cmd == "zero":
        if arm:
            arm.go_zero()
        await broadcast({"type": "log", "level": "info", "msg": "Zeroing joints"})

    elif cmd == "estop":
        if arm:
            arm.emergency_stop()
        if recorder and recorder.active:
            recorder.discard()
            recorder = None
        await broadcast({"type": "estop", "msg": "EMERGENCY STOP TRIGGERED"})

    elif cmd == "start_teleop":
        if teleop and teleop.active:
            await ws.send(json.dumps({"type": "error", "msg": "Teleop already running"}))
            return
        # Stop hand/head tracking if running (teleop takes priority)
        if hand_tracker and hand_tracker.active:
            hand_tracker.stop()
            hand_tracker = None
            await broadcast({"type": "status", "hand_tracking": False,
                             "msg": "Hand tracking stopped for teleop"})
        if head_tracker and head_tracker.active:
            head_tracker.stop()
            head_tracker = None
            await broadcast({"type": "status", "head_tracking": False,
                             "msg": "Head tracking stopped for teleop"})
        if voice_controller:
            voice_controller.stop()
            voice_controller = None
        leader_port = msg.get("leader_port", cfg.LEADER_PORT)
        follower_port = msg.get("follower_port", cfg.FOLLOWER_PORT)
        # Disconnect the global arm if it's using either teleop port
        if arm:
            arm.disconnect()
            arm = None
        try:
            use_sim_leader = (
                leader_port == follower_port
                or leader_port == "sim"
                or (cfg.SIM_MODE == "mujoco" and _SIM_BACKEND_AVAILABLE)
            )
            use_sim_follower = (
                cfg.SIM_MODE == "mujoco" and _SIM_BACKEND_AVAILABLE
            ) or follower_port == "sim"

            if use_sim_leader and _SIM_BACKEND_AVAILABLE:
                leader = _SimAC(port=leader_port, baud=cfg.DEFAULT_BAUD, config=cfg, role="leader")
            else:
                leader = ArmController(port=leader_port, baud=cfg.DEFAULT_BAUD, config=cfg, role="leader")
            if use_sim_follower and _SIM_BACKEND_AVAILABLE:
                follower = _SimAC(port=follower_port, baud=cfg.DEFAULT_BAUD, config=cfg, role="follower")
            else:
                follower = ArmController(port=follower_port, baud=cfg.DEFAULT_BAUD, config=cfg, role="follower")

            leader.connect()
            follower.connect()
            teleop = _TeleopCtl(leader, follower, cfg)
            teleop.start()
            mode_leader = "sim" if use_sim_leader else leader_port
            mode_follower = "sim" if use_sim_follower else follower_port
            await broadcast({"type": "status", "teleop": True,
                             "msg": f"Teleop started — leader={mode_leader} follower={mode_follower}"})
        except Exception as e:
            log.error("Teleop start failed: %s", e)
            await ws.send(json.dumps({"type": "error", "msg": f"Teleop start failed: {e}"}))

    elif cmd == "stop_teleop":
        if teleop:
            teleop.stop()
            if hasattr(teleop.leader, 'disconnect'):
                teleop.leader.disconnect()
            # Keep follower connected so GUI can still control it
            if teleop.follower and hasattr(teleop.follower, 'is_connected') and teleop.follower.is_connected():
                arm = teleop.follower
            teleop = None
        await broadcast({"type": "status", "teleop": False, "msg": "Teleop stopped"})

    elif cmd == "set_gripper":
        if teleop and teleop.active:
            teleop.leader.set_gripper(float(msg["value"]))
        elif arm:
            arm.set_gripper(float(msg["value"]))

    elif cmd == "cal_step":
        if arm:
            result = arm.calibrate_step(int(msg["step"]))
            await broadcast({"type": "cal_result", "step": msg["step"], "result": result})
        else:
            await ws.send(json.dumps({"type": "error", "msg": "Arm not connected"}))

    elif cmd == "save_calibration":
        cfg.save()
        await broadcast({"type": "log", "level": "info",
                         "msg": "Calibration configuration saved"})

    elif cmd == "reset_calibration":
        cfg.reload()
        await broadcast({"type": "config", "data": cfg.to_dict()})
        await broadcast({"type": "log", "level": "info",
                         "msg": "Configuration reloaded from disk"})

    elif cmd == "start_cameras":
        devices = msg.get("devices", cfg.CAMERA_DEVICES)
        sim_cb = _get_sim_camera_callback()
        if cams is None:
            cams = CameraManager(devices=devices, config=cfg,
                                 sim_frame_callback=sim_cb)
            mjpeg.set_camera_manager(cams)
        elif msg.get("devices"):
            cams.stop()
            cams = CameraManager(devices=devices, config=cfg,
                                 sim_frame_callback=sim_cb)
            mjpeg.set_camera_manager(cams)
        cams.start()
        await broadcast({"type": "log", "level": "info",
                         "msg": f"Cameras started: {devices}"})

    elif cmd == "stop_cameras":
        if cams:
            cams.stop()
        await broadcast({"type": "log", "level": "info", "msg": "Cameras stopped"})

    elif cmd == "start_recording":
        if not arm and not (teleop and teleop.active):
            await ws.send(json.dumps({"type": "error", "msg": "Arm or teleop not active"}))
            return
        ep_name = msg.get("name", f"episode_{int(time.time())}")
        task    = msg.get("task", "")
        fps     = int(msg.get("fps", cfg.RECORD_FPS))
        recorder = EpisodeRecorder(
            name=ep_name, task=task, fps=fps,
            output_dir=cfg.DATASET_DIR,
            camera_manager=cams,
            arm=arm,
            teleop_controller=teleop,
        )
        recorder.start()
        await broadcast({"type": "recording_started", "name": ep_name})

    elif cmd == "stop_recording":
        if recorder and recorder.active:
            info = recorder.stop()
            await broadcast({"type": "recording_stopped", "info": info})
            recorder = None
        else:
            await ws.send(json.dumps({"type": "error", "msg": "Not currently recording"}))

    elif cmd == "discard_recording":
        if recorder and recorder.active:
            recorder.discard()
            recorder = None
        await broadcast({"type": "recording_discarded"})

    elif cmd == "list_episodes":
        await ws.send(json.dumps({"type": "episodes", "list": _list_episodes()}))

    elif cmd == "get_episode":
        ep_path = Path(cfg.DATASET_DIR).expanduser() / msg["name"] / "episode.json"
        if not ep_path.exists():
            await ws.send(json.dumps({"type": "error", "msg": f"Episode not found: {msg['name']}"}))
            return
        with open(ep_path) as f:
            ep_data = json.load(f)
        await ws.send(json.dumps({"type": "episode_data", "name": msg["name"], "data": ep_data}))

    elif cmd == "replay_episode":
        if not arm:
            await ws.send(json.dumps({"type": "error", "msg": "Arm not connected"}))
            return
        asyncio.create_task(
            _replay_episode(msg["name"], float(msg.get("speed", 1.0)), ws)
        )

    elif cmd == "save_config":
        cfg.update(msg.get("config", {}))
        cfg.save()
        await broadcast({"type": "log", "level": "info", "msg": "Config saved"})

    elif cmd == "get_config":
        await ws.send(json.dumps({"type": "config", "data": cfg.to_dict()}))

    elif cmd == "push_to_hub":
        asyncio.create_task(_push_to_hub(msg.get("repo", ""), ws))

    elif cmd == "auto_calibrate":
        if arm:
            arm.disconnect()
            arm = None
        _cal_stop.clear()
        cal_port = msg.get("port", cfg.FOLLOWER_PORT)
        cal_role = msg.get("role", "follower")  # "leader", "follower", or "both"
        await broadcast({"type": "log", "msg": f"Starting auto-calibration on {cal_port} ({cal_role})...",
                         "level": "info"})
        loop = asyncio.get_running_loop()
        threading.Thread(target=_run_calibration, args=(loop, cal_port, cal_role), daemon=True).start()

    elif cmd == "cal_estop":
        _cal_stop.set()
        await broadcast({"type": "log", "msg": "Calibration E-stop triggered", "level": "warn"})
        await broadcast({"type": "cal_estop"})
        # Immediately reset GUI — the background thread may not respond in time
        await broadcast({"type": "cal_done", "aborted": True, "error": "E-stop"})

    elif cmd == "diagnose":
        lerobot_ok = False
        lerobot_ver = "unknown"
        lerobot_path = ""
        lerobot_err = None
        try:
            import lerobot
            lerobot_ok = True
            lerobot_ver = getattr(lerobot, "__version__", "unknown")
            lerobot_path = lerobot.__file__
        except ImportError as e:
            lerobot_err = str(e)
        info = {
            "lerobot_available": lerobot_ok,
            "lerobot_version": lerobot_ver,
            "lerobot_path": lerobot_path,
            "arm_exists": arm is not None,
            "arm_sim": getattr(arm, '_sim', True) if arm else True,
            "arm_bus": str(type(arm._bus)) if arm and hasattr(arm, '_bus') and arm._bus else "None",
            "arm_connected": arm.is_connected() if arm else False,
            "python": __import__("sys").version,
        }
        if lerobot_err:
            info["lerobot_import_error"] = lerobot_err
        await ws.send(json.dumps({"type": "log", "level": "info",
                                  "msg": "DIAG: " + __import__("json").dumps(info)}))

    elif cmd == "start_hand_tracking":
        try:
            if hand_tracker and hand_tracker.active:
                await ws.send(json.dumps({"type": "log", "msg": "Hand tracking already running",
                                          "level": "warn"}))
                return
            # Stop teleop if running (hand tracking takes priority)
            if teleop:
                teleop.stop()
                if hasattr(teleop.leader, 'disconnect'):
                    teleop.leader.disconnect()
                teleop = None
                await broadcast({"type": "status", "teleop": False,
                                 "msg": "Teleop stopped for hand tracking"})
            if head_tracker and head_tracker.active:
                head_tracker.stop()
                head_tracker = None
            if voice_controller:
                voice_controller.stop()
                voice_controller = None
            hand_tracker = HandTracker(camera_id=int(msg.get("camera", 0)))
            hand_tracker.set_arm_callback(_hand_arm_cmd)
            if hand_tracker.start():
                await broadcast({"type": "status", "hand_tracking": True,
                                 "msg": "Hand tracking started"})
            else:
                await ws.send(json.dumps({"type": "error", "msg": "Failed to start hand tracking (no camera?)"}))
        except Exception as e:
            log.error("start_hand_tracking failed: %s", e)
            await ws.send(json.dumps({"type": "error", "msg": f"Hand tracking failed: {e}"}))

    elif cmd == "stop_hand_tracking":
        if hand_tracker:
            hand_tracker.stop()
            hand_tracker = None
        await broadcast({"type": "status", "hand_tracking": False,
                         "msg": "Hand tracking stopped"})

    elif cmd == "start_head_tracking":
        try:
            if head_tracker and head_tracker.active:
                await ws.send(json.dumps({"type": "log", "msg": "Head tracking already running",
                                          "level": "warn"}))
                return
            if hand_tracker and hand_tracker.active:
                hand_tracker.stop()
                hand_tracker = None
            if teleop:
                teleop.stop()
                if hasattr(teleop.leader, 'disconnect'):
                    teleop.leader.disconnect()
                teleop = None
                await broadcast({"type": "status", "teleop": False})
            if voice_controller:
                voice_controller.stop()
                voice_controller = None
            head_tracker = HeadTracker(camera_id=int(msg.get("camera", 0)))
            head_tracker.set_arm_callback(_head_arm_cmd)
            if head_tracker.start():
                await broadcast({"type": "status", "head_tracking": True,
                                 "msg": "Head tracking started"})
            else:
                await ws.send(json.dumps({"type": "error", "msg": "Failed to start head tracking"}))
        except Exception as e:
            log.error("start_head_tracking failed: %s", e)
            await ws.send(json.dumps({"type": "error", "msg": f"Head tracking failed: {e}"}))

    elif cmd == "stop_head_tracking":
        if head_tracker:
            head_tracker.stop()
            head_tracker = None
        await broadcast({"type": "status", "head_tracking": False,
                         "msg": "Head tracking stopped"})

    elif cmd == "voice_start_record":
        # Stop conflicting modes
        if teleop:
            teleop.stop()
            if hasattr(teleop.leader, 'disconnect'):
                teleop.leader.disconnect()
            teleop = None
            await broadcast({"type": "status", "teleop": False})
        if hand_tracker and hand_tracker.active:
            hand_tracker.stop()
            hand_tracker = None
        if head_tracker and head_tracker.active:
            head_tracker.stop()
            head_tracker = None
        if not voice_controller:
            voice_controller = VoiceController()
            voice_controller.set_command_callback(_voice_cmd_callback)
            await asyncio.get_event_loop().run_in_executor(
                None, voice_controller.start)
        voice_controller.start_recording()
        await broadcast({"type": "status", "voice_status": "LISTENING",
                         "msg": "Voice: listening..."})

    elif cmd == "voice_stop_record":
        if voice_controller and voice_controller.active:
            text = await asyncio.get_event_loop().run_in_executor(
                None, voice_controller.stop_and_process)
            status = "READY"
            if text:
                await broadcast({"type": "status", "voice_status": status,
                                 "voice_last": text,
                                 "msg": f"Voice: '{text}'"})
            else:
                await broadcast({"type": "status", "voice_status": status,
                                 "msg": "Voice: no speech detected"})
        else:
            await broadcast({"type": "status", "voice_status": "READY",
                             "msg": "Voice: idle"})

    elif cmd == "set_ik_target":
        if not arm:
            await ws.send(json.dumps({"type": "error", "msg": "Arm not connected"}))
            return
        try:
            tx = float(msg.get("x", 0))
            ty = float(msg.get("y", 200))
            # Get current joints for initial guess
            state = arm.get_state()
            initial = state.get("joints", [0, -34.4, -45.8, -22.9, 0])
            ik_joints = solve_ik(tx, ty, initial)
            # Keep J4 unchanged
            full_joints = ik_joints + [initial[4]] if len(initial) > 4 else ik_joints + [0]
            arm.set_joints(full_joints)
            await broadcast({"type": "log", "msg": f"IK move to ({tx}, {ty}) → {ik_joints}",
                             "level": "info"})
        except Exception as e:
            log.warning("IK error: %s", e)
            await ws.send(json.dumps({"type": "error", "msg": f"IK failed: {e}"}))

    else:
        await ws.send(json.dumps({"type": "error", "msg": f"Unknown command: {cmd}"}))


async def _replay_episode(name: str, speed: float, ws):
    ep_path = Path(cfg.DATASET_DIR).expanduser() / name / "episode.json"
    if not ep_path.exists():
        await ws.send(json.dumps({"type": "error", "msg": f"Episode not found: {name}"}))
        return
    with open(ep_path) as f:
        episode = json.load(f)
    frames = episode.get("frames", [])
    fps    = episode.get("fps", cfg.RECORD_FPS)
    dt     = 1.0 / fps / max(speed, 0.01)
    await broadcast({"type": "replay_started", "name": name, "frames": len(frames)})
    for i, frame in enumerate(frames):
        if arm:
            arm.set_joints(frame["joints"])
            if "gripper" in frame:
                arm.set_gripper(frame["gripper"])
        await broadcast({
            "type":    "replay_frame",
            "frame":   i,
            "total":   len(frames),
            "joints":  frame["joints"],
            "gripper": frame.get("gripper", 50),
        })
        await asyncio.sleep(dt)
    await broadcast({"type": "replay_done", "name": name})


async def _push_to_hub(repo: str, ws):
    try:
        proc = await asyncio.create_subprocess_exec(
            sys.executable, "-m", "lerobot.scripts.push_dataset_to_hub",
            "--repo-id", repo,
            "--raw-dir", str(Path(cfg.DATASET_DIR).expanduser()),
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        async for line in proc.stdout:
            await ws.send(json.dumps({"type": "log", "level": "info",
                                      "msg": line.decode().strip()}))
        await proc.wait()
        await ws.send(json.dumps({"type": "hub_done", "repo": repo,
                                  "success": proc.returncode == 0}))
    except Exception as e:
        await ws.send(json.dumps({"type": "error", "msg": str(e)}))


def _calibrate_one_arm(loop: asyncio.AbstractEventLoop, port: str, role: str,
                       home_angles: list[float]) -> bool:
    """Run calibration on a single arm at `port`, save with `role` prefix.
    Returns True on success, False on abort/error."""
    home_ticks = [deg_to_ticks(a) for a in home_angles]
    stop = _cal_stop

    async def _progress(msg: dict):
        await broadcast(msg)

    async def _step(jid: int, phase: str, **kw):
        await broadcast({"type": "cal_progress", "joint": jid, "phase": phase, "role": role, **kw})

    try:
        bus = build_bus(port)
    except Exception as e:
        asyncio.run_coroutine_threadsafe(
            _progress({"type": "error", "msg": f"[{role}] Connect failed: {e}"}), loop)
        asyncio.run_coroutine_threadsafe(
            _progress({"type": "cal_done", "role": role, "aborted": True, "error": str(e)}), loop)
        return False

    try:
        asyncio.run_coroutine_threadsafe(
            _progress({"type": "log", "msg": f"[{role}] Moving to home...", "level": "info"}), loop)
        go_home(bus, home_ticks, stop)
        if stop.is_set():
            asyncio.run_coroutine_threadsafe(
                _progress({"type": "cal_done", "role": role, "aborted": True}), loop)
            bus.disconnect()
            return False

        joint_info = [
            (0, "Shoulder pan"), (1, "Shoulder lift"), (2, "Elbow"),
            (3, "Wrist pitch"), (4, "Wrist roll"),
        ]
        limits = []

        for jid, jname in joint_info:
            if stop.is_set():
                break
            asyncio.run_coroutine_threadsafe(
                _step(jid, "start", name=jname), loop)
            go_home(bus, home_ticks, stop)
            if stop.is_set():
                break
            lo = find_limit(bus, jid, home_ticks[jid], -60, "min", stop)
            hi = find_limit(bus, jid, home_ticks[jid], 60, "max", stop)
            lo_deg = ticks_to_deg(lo)
            hi_deg = ticks_to_deg(hi)
            limits.append([lo_deg, hi_deg])
            asyncio.run_coroutine_threadsafe(
                _step(jid, "done", name=jname, lo=lo_deg, hi=hi_deg), loop)
            if jid == 1:
                bus.write("Goal_Position", "j1", hi, normalize=False)
                time.sleep(1.0)

        if not stop.is_set():
            go_home(bus, home_ticks, stop)

            asyncio.run_coroutine_threadsafe(
                _progress({"type": "log", "msg": f"[{role}] Calibrating gripper...", "level": "info"}), loop)
            gripper_open, gripper_closed = find_gripper_limits(bus, stop)

            # Save with role prefix
            cal_cfg = load_cal_cfg()
            cal_cfg[f"{role}_JOINT_LIMITS".upper()] = limits
            cal_cfg[f"{role}_GRIPPER_OPEN".upper()] = float(gripper_open)
            cal_cfg[f"{role}_GRIPPER_CLOSED".upper()] = float(gripper_closed)
            cal_cfg[f"{role}_HOME_ANGLES".upper()] = home_angles
            save_cal_cfg(cal_cfg)

            # Reload bridge config in-memory
            cfg.update(cal_cfg)

            asyncio.run_coroutine_threadsafe(_progress({"type": "cal_done",
                "role": role, "limits": limits,
                "gripper": {"open": gripper_open, "closed": gripper_closed}}), loop)
            bus.disconnect()
            return True
        else:
            asyncio.run_coroutine_threadsafe(
                _progress({"type": "cal_done", "role": role, "aborted": True}), loop)
            bus.disconnect()
            return False
    except Exception as e:
        asyncio.run_coroutine_threadsafe(
            _progress({"type": "error", "msg": f"Calibration failed: {e}"}), loop)
        asyncio.run_coroutine_threadsafe(
            _progress({"type": "cal_done", "role": role, "aborted": True, "error": str(e)}), loop)
        try:
            bus.disconnect()
        except Exception:
            pass
        return False


def _run_calibration(loop: asyncio.AbstractEventLoop, port: str, role: str):
    """Run calibration on one or both arms.
    For 'both', runs follower then leader in one thread, sending two cal_done messages."""
    cal_cfg = load_cal_cfg()
    home_angles = cal_cfg.get("HOME_ANGLES", [0.0, -34.4, -45.8, -22.9, 0.0])

    try:
        if role == "both":
            ok = _calibrate_one_arm(loop, cfg.FOLLOWER_PORT, "follower", home_angles)
            if not ok or _cal_stop.is_set():
                return
            _calibrate_one_arm(loop, cfg.LEADER_PORT, "leader", home_angles)
        else:
            _calibrate_one_arm(loop, port, role, home_angles)
    except Exception as e:
        asyncio.run_coroutine_threadsafe(
            broadcast({"type": "cal_done", "role": role, "aborted": True, "error": str(e)}), loop)


def _list_episodes() -> list[dict]:
    base = Path(cfg.DATASET_DIR).expanduser()
    eps  = []
    if base.exists():
        for ep_dir in sorted(base.iterdir()):
            meta = ep_dir / "episode.json"
            if meta.exists():
                try:
                    with open(meta) as f:
                        d = json.load(f)
                    eps.append({
                        "name":     ep_dir.name,
                        "task":     d.get("task", ""),
                        "frames":   d.get("frame_count", 0),
                        "fps":      d.get("fps", cfg.RECORD_FPS),
                        "duration": round(d.get("duration_s", 0), 2),
                        "size_mb":  round(d.get("size_bytes", 0) / 1e6, 1),
                    })
                except Exception:
                    pass
    return eps


async def telemetry_loop():
    while True:
        await asyncio.sleep(0.05)
        if not connected_clients:
            continue
        payload = {"type": "telemetry"}
        if arm is not None:
            try:
                state   = arm.get_state()
                payload.update(state)
                if recorder and recorder.active:
                    payload["rec_frames"]  = recorder.frame_count
                    payload["rec_elapsed"] = recorder.elapsed()
                if state.get("mode") == "hardware":
                    j = state.get("joints", [])
                    g = state.get("gripper", 0)
                    c = state.get("current", [])
                    t = state.get("temperature", [])
                    v = state.get("voltage", 0)
                    log.info("SERVO  joints=[%.1f,%.1f,%.1f,%.1f,%.1f] grip=%.0f%%  current=%.2fA  temp=%.0f°C  volt=%.1fV",
                             *j[:5], g, sum(c), max(t) if t else 0, v)
            except Exception as e:
                log.warning("Telemetry error: %s", e)
        if teleop is not None:
            try:
                ts = teleop.get_states()
                payload["teleop"] = {
                    "active":   ts["active"],
                    "leader":   ts["leader"],
                    "follower": ts["follower"],
                }
                # Copy follower state to top-level so the GUI Control tab
                # (joint sliders, SVG, TCP, electrical) stays live during teleop
                if ts.get("follower"):
                    follower = ts["follower"]
                    for key in ("joints", "gripper", "tcp", "current", "temperature", "voltage", "mode"):
                        if key in follower:
                            payload[key] = follower[key]
            except Exception as e:
                log.warning("Teleop telemetry error: %s", e)
        if hand_tracker and hand_tracker.active:
            pose = hand_tracker.latest_pose
            payload["hand"] = {
                "detected": pose.detected,
                "x": pose.x,
                "y": pose.y,
                "z": pose.z,
                "pinch": pose.pinch,
            }
        if head_tracker and head_tracker.active:
            pose = head_tracker.latest_pose
            payload["head"] = {
                "detected": pose.detected,
                "x": pose.x,
                "y": pose.y,
                "mouth": pose.mouth_open,
            }
        if payload.get("joints") is not None or payload.get("teleop") is not None or payload.get("hand") is not None or payload.get("head") is not None:
            await broadcast(payload)


async def handler(ws):
    connected_clients.add(ws)
    log.info("GUI connected from %s", ws.remote_address)
    await ws.send(json.dumps({
        "type":    "hello",
        "version": "1.0.0",
        "msg":     "SO-101 bridge ready",
    }))
    try:
        async for raw in ws:
            await handle_message(ws, raw)
    except websockets.exceptions.ConnectionClosed:
        pass
    except Exception as e:
        log.warning("Handler error: %s", e)
    finally:
        connected_clients.discard(ws)
        log.info("GUI disconnected: %s", ws.remote_address)


async def main():
    global _loop
    _loop = asyncio.get_running_loop()
    Path("logs").mkdir(exist_ok=True)
    Path(cfg.DATASET_DIR).expanduser().mkdir(parents=True, exist_ok=True)

    # MediaPipe hand tracking model
    _hand_model = Path.home() / ".mediapipe" / "models" / "hand_landmarker.task"
    if _hand_model.exists():
        set_hand_model_path(str(_hand_model))
        log.info("Hand model: %s", _hand_model)
    else:
        log.warning("Hand model not found at %s", _hand_model)

    # ── Startup diagnostic ────────────────────────────────────────────────
    try:
        import lerobot as _lr
        log.info("lerobot %s found at %s",
                 getattr(_lr, "__version__", "?"), _lr.__file__)
    except ImportError as e:
        log.warning("lerobot not importable at startup: %s", e)
        log.warning("Run:  source .venv/bin/activate && pip install lerobot")

    ws_host    = cfg.WS_HOST
    ws_port    = cfg.WS_PORT
    mjpeg_port = getattr(cfg, "MJPEG_PORT", 8766)

    log.info("═" * 56)
    log.info("  SO-101 Control Bridge  v1.0")
    log.info("  WebSocket : ws://%s:%d", ws_host, ws_port)
    log.info("  GUI       : http://localhost:%d   ← open this in Chrome", mjpeg_port)
    log.info("  Cameras   : http://localhost:%d/cam/0", mjpeg_port)
    log.info("  Dataset   : %s", cfg.DATASET_DIR)
    log.info("═" * 56)

    def _get_hand_frame():
        if hand_tracker and hand_tracker.active:
            return hand_tracker.get_frame()
        if head_tracker and head_tracker.active:
            return head_tracker.get_frame()
        return None
    mjpeg.set_hand_frame_provider(_get_hand_frame)

    mjpeg_runner = await mjpeg.start_mjpeg_server(ws_host, mjpeg_port, cfg.DATASET_DIR)

    # Suppress noisy "invalid Connection header" warnings from browser preflight probes
    logging.getLogger("websockets.server").setLevel(logging.ERROR)

    async with websockets.serve(
        handler,
        ws_host,
        ws_port,
    ):
        asyncio.create_task(telemetry_loop())
        log.info("Bridge is live. Press Ctrl+C to stop.")
        try:
            while True:
                await asyncio.sleep(3600)
        except asyncio.CancelledError:
            log.info("Main loop cancelled (Ctrl+C)")
        except Exception as e:
            log.error("Main loop error: %s", e)

    log.info("Shutting down gracefully...")
    if arm:
        arm.disconnect()
    if cams:
        cams.stop()
    await mjpeg_runner.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
