# SO-101 Bridge — Architecture & Protocol Reference

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Process Lifecycle](#2-process-lifecycle)
3. [WebSocket Protocol](#3-websocket-protocol)
4. [Motor Control Layer](#4-motor-control-layer)
5. [Camera System](#5-camera-system)
6. [Episode Recording](#6-episode-recording)
7. [Replay System](#7-replay-system)
8. [Calibration System](#8-calibration-system)
9. [Configuration System](#9-configuration-system)
10. [Frontend Architecture](#10-frontend-architecture)
11. [Dataset Format](#11-dataset-format)
12. [Simulation Backend](#12-simulation-backend)
13. [Teleoperation](#13-teleoperation)
14. [Safety Systems](#14-safety-systems)
15. [Troubleshooting](#15-troubleshooting)

---

## 1. Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     Browser (GUI)                            │
│               static/index.html (JS app)                     │
│                                                              │
│  ┌──────────┐   ┌────────────────────────────────────────┐  │
│  │ WebSocket│   │  <img> tags for MJPEG streams           │  │
│  │ client   │   │  http://host:8766/cam/{dev}             │  │
│  └────┬─────┘   └───────────────┬────────────────────────┘  │
└───────┼───────────────────────┼────────────────────────────┘
        │ ws://host:8765        │ http://host:8766
        ▼                       ▼
┌─────────────────────────────────────────────────────────────┐
│                       server.py                              │
│  ┌──────────────────┐  ┌──────────────────────────┐        │
│  │ WebSocket server  │  │ MJPEG HTTP server         │        │
│  │ (websockets)      │  │ (aiohttp)                 │        │
│  │ port 8765         │  │ port 8766                 │        │
│  └───────┬──────────┘  └───────────┬──────────────┘        │
│          │                         │                        │
│  ┌───────┴─────────────────────────┴──────────────────┐    │
│  │              handle_message() / telemetry_loop()      │    │
│  │        dispatches commands → module calls             │    │
│  └───────┬─────────────────────────┬──────────────┬───┘    │
│          │                         │              │        │
│    ┌─────▼──────┐          ┌───────▼──────┐  ┌────▼─────┐ │
│    │ArmController│          │CameraManager │  │Teleop    │ │
│    │or           │          │(OpenCV       │  │Controller│ │
│    │SimArmCtl    │          │ threads)     │  │(thread)  │ │
│    │(lerobot     │          │         │    │  └────┬─────┘ │
│    │Feetech bus │          │         │    │       │        │
│    │or MuJoCo)  │          │         │    │       │        │
│    └──────┬──────┘          └────┬────┘    │       │        │
│           │                      │         │       │        │
│  ┌────────┴──────────────────────┴─────────┴───────┴───┐  │
│  │            EpisodeRecorder (thread)                  │  │
│  │    captures joints + gripper + frames + leader state │  │
│  └───────────────────────────┬─────────────────────────┘  │
└──────────────────────────────┼────────────────────────────┘
                               │
         ┌─────────────────────┼────────────────────┐
         ▼                     ▼                    ▼
   /dev/ttyACM0           /dev/video0          ~/datasets/so101/
   6× STS3215             /dev/video2          episode_{N}/
   servos (ID 1-6)                              ├── episode.json
                                                ├── joints.csv
                                                └── frames/
                                                    ├── cam_0_*.jpg
                                                    └── cam_2_*.jpg

   ┌─────────────────────────────────────────┐
   │     SimArmController (mujoco)           │
   │   ┌──────────────┐  ┌────────────────┐  │
   │   │ 3D viewer     │  │ offscreen      │  │
   │   │ (GLFW thread) │  │ renderer       │  │
   │   └──────────────┘  │ → sim_frame_cb  │  │
   │                     └────────────────┘  │
   └─────────────────────────────────────────┘
```

**Two servers, one process.** `server.py` is the single entry point. It starts:

| Server | Port | Protocol | Purpose |
|--------|------|----------|---------|
| WebSocket | 8765 | `ws://` | Bidirectional control + telemetry |
| HTTP | 8766 | `http://` | Serves GUI, MJPEG camera streams, episode frame images |

The GUI auto-connects on load. No page refresh needed — all state flows over WebSocket at 20 Hz.

**Controller selection** — On `connect`, the server checks `SIM_MODE`. If `"mujoco"` and MuJoCo is installed, a `SimArmController` is created instead of `ArmController`. The sim controller provides physics, a 3D viewer, and offscreen camera renders fed into the MJPEG stream via `sim_frame_callback`.

**Teleoperation** — The `TeleopController` runs a leader/follower loop (default 100 Hz) in a background daemon thread. It reads leader joint positions and writes them to the follower, while both states are reported via telemetry and optionally recorded.

---

## 2. Process Lifecycle

### 2.1 Startup Sequence

```
server.py main()
  │
  ├── 1. Create Config (loads config.json, env vars)
  │     └── Checks SIM_MODE, LEADER_PORT, FOLLOWER_PORT, TELEOP_RATE
  │
  ├── 2. Create logs/ directory
  ├── 3. Create dataset directory (e.g. ~/datasets/so101)
  │
  ├── 4. Import-check lerobot + sim/teleop backends
  │     ├── lerobot → ArmController
  │     ├── sim_backend.SimArmController → MuJoCo sim
  │     └── teleop.TeleopController → leader/follower
  │
  ├── 5. Start MJPEG HTTP server on port 8766
  │     └── sets dataset_dir in aiohttp app for frame serving
  │
  ├── 6. Start WebSocket server on port 8765
  │
  ├── 7. Launch telemetry_loop() as asyncio Task
  │     └── polls arm.get_state() at 20 Hz, broadcasts to all clients
  │     └── if teleop active, appends teleop leader/follower state
  │
  └── 8. Wait for SIGINT/SIGTERM → graceful shutdown
```

### 2.2 Connection Lifecycle

```
GUI opens page
  │
  ├── 1. JS creates WebSocket → ws://hostname:8765
  │
  ├── 2. Server accepts, sends {"type": "hello", "version": "1.0.0"}
  │
  ├── 3. Client requests episode list (list_episodes)
  │
  ├── 4. User clicks "CONNECT ARM"
  │     └── Client sends {"cmd": "connect", "port": "...", "baud": ...}
  │
  ├── 5. Server selects controller:
  │     ├── SIM_MODE == "mujoco" → SimArmController.connect()
  │     │     ├── Loads MJCF model (so101.xml) into MuJoCo
  │     │     ├── Initialises qpos, runs mj_forward
  │     │     ├── Spawns GLFW 3D viewer in daemon thread
  │     │     └── Broadcasts {"type": "status", "connected": true, "msg": "MuJoCo simulation started"}
  │     │
  │     └── default → ArmController.connect()
  │           ├── Detects lerobot API version (new/old)
  │           ├── Opens FeetechMotorsBus on the serial port
  │           ├── Sets torque enable on all 6 motors
  │           ├── Loads calibration.json if it exists
  │           ├── Seeds position cache
  │           └── Broadcasts {"type": "status", "connected": true}
  │
  ├── 6. Telemetry loop starts streaming joint positions, currents, temps
  │     └── In sim mode, currents=0, temp=25°C, voltage=12V, mode="simulation (mujoco)"
  │
  ├── 7. User interacts with sliders → set_joint commands → servos/sim move
  │
  ├── 8. User clicks "DISCONNECT ARM" or closes browser
  │     ├── ArmController: torque disabled on all motors, bus disconnected
  │     └── SimArmController: renderer closed, viewer thread stops
  │
  └── 9. Browser closes WS → onclose handler → 3s auto-reconnect
```

### 2.3 Shutdown Sequence

```
SIGINT/SIGTERM received
  │
  ├── 1. ArmController.disconnect()
  │     └── Torque_Enable=0 on all 6 motors, bus.disconnect()
  │
  ├── 2. CameraManager.stop()
  │     └── Each CameraStream thread joins (2s timeout)
  │
  └── 3. MJPEG HTTP runner.cleanup()
```

---

## 3. WebSocket Protocol

All messages are JSON. The protocol is asymmetric: commands flow client→server, events/telemetry flow server→client.

### 3.1 Client → Server Commands

| Command | Parameters | Description |
|---------|-----------|-------------|
| `connect` | `port` (str), `baud` (int) | Connect to serial port |
| `disconnect` | — | Disconnect arm, disable torque |
| `set_joint` | `joint` (int 0-4), `angle` (float °) | Move one joint |
| `set_joints` | `angles` (float[5]) | Move all 5 joints at once |
| `set_gripper` | `value` (float 0-100) | Set gripper position % |
| `home` | — | Move all joints to HOME_ANGLES |
| `zero` | — | Move all joints to 0° |
| `estop` | — | Cut torque on all motors, discard recording |
| `cal_step` | `step` (int 0-4) | Execute calibration step |
| `save_calibration` | — | Save calibration to `calibration.json` |
| `start_cameras` | `devices` (int[], optional) | Start camera streams |
| `stop_cameras` | — | Stop all camera streams |
| `start_teleop` | `leader_port` (str), `follower_port` (str) | Start leader/follower teleop loop |
| `stop_teleop` | — | Stop teleop, re-enable leader torque |
| `start_recording` | `name`, `task`, `fps`, `devices` | Start episode recording |
| `stop_recording` | — | Stop and save episode |
| `discard_recording` | — | Stop and delete episode |
| `list_episodes` | — | List all saved episodes |
| `get_episode` | `name` | Return full episode JSON data |
| `replay_episode` | `name`, `speed` (float) | Play episode on real arm |
| `save_config` | `config` (dict) | Save config.json keys |
| `get_config` | — | Return full config dict |
| `push_to_hub` | `repo` (str, HF repo ID) | Push dataset to HuggingFace |
| `diagnose` | — | Return system diagnostic info |

### 3.2 Server → Client Messages

| Type | Payload | Trigger |
|------|---------|---------|
| `hello` | `version`, `msg` | On WebSocket open |
| `status` | `connected` (bool), `msg` | After connect/disconnect |
| `telemetry` | `joints`, `gripper`, `tcp`, `current`, `temperature`, `voltage`, `mode` | 20 Hz loop while arm exists |
| `telemetry` (recording) | +`rec_frames`, `rec_elapsed` | Same, when recorder active |
| `log` | `level` (info/warn/err), `msg` | Any noteworthy event |
| `error` | `msg` | Command failure |
| `estop` | `msg` | Emergency stop triggered |
| `episodes` | `list` (array of episode summaries) | Response to `list_episodes` |
| `episode_data` | `name`, `data` (full episode JSON) | Response to `get_episode` |
| `recording_started` | `name` | Confirmation |
| `recording_stopped` | `info` (name, frames, duration, size) | Confirmation |
| `recording_discarded` | — | Confirmation |
| `replay_started` | `name`, `frames` | Replay begins |
| `replay_frame` | `frame`, `total`, `joints`, `gripper` | Each frame during replay |
| `replay_done` | `name` | Replay completes |
| `cal_result` | `step`, `result` | Calibration step result |
| `teleop_started` | `msg` | Teleop loop started |
| `teleop_stopped` | `msg` | Teleop loop stopped |
| `config` | `data` (full config dict) | Response to `get_config` |
| `hub_done` | `repo`, `success` | HuggingFace push result |

### 3.3 Telemetry Data Format

Sent at 20 Hz (every 50 ms) to all connected clients. Example:

```json
{
  "type": "telemetry",
  "joints": [12.3, -45.0, 88.1, -112.2, 5.0],
  "gripper": 50.0,
  "tcp": {"x": 215.4, "y": 0.0, "z": 185.2},
  "current": [0.05, 0.12, 0.08, 0.03, 0.06],
  "temperature": [28.0, 29.0, 28.5, 27.0, 27.5],
  "voltage": 11.2,
  "mode": "hardware"
}
```

When teleop is active, an additional `teleop` key is included:

```json
{
  "type": "telemetry",
  "joints": [...],
  "gripper": 50.0,
  "tcp": {...},
  "current": [...],
  "temperature": [...],
  "voltage": 11.2,
  "mode": "hardware",
  "teleop": {
    "active": true,
    "leader": {
      "joints": [12.3, -45.0, 88.1, -112.2, 5.0],
      "gripper": 50.0,
      "tcp": {...},
      "current": [...],
      "temperature": [...],
      "voltage": 11.2,
      "mode": "hardware"
    },
    "follower": {
      "joints": [12.5, -44.8, 88.3, -112.0, 5.1],
      "gripper": 50.0,
      "tcp": {...},
      "current": [...],
      "temperature": [...],
      "voltage": 11.2,
      "mode": "hardware"
    }
  }
}
```

In simulation mode, all currents/temps/voltage are 0 (or defaults) and `mode` is `"simulation (mujoco)"`.

---

## 4. Motor Control Layer

### 4.1 `arm_controller.py` — ArmController

The `ArmController` class wraps lerobot's `FeetechMotorsBus` with:

- **API detection** — auto-detects lerobot 0.5.x (new) vs ≤0.3 (old) API styles
- **Hardware/Simulation fallback** — if bus connection fails, runs in simulation with in-memory state
- **Degree↔Tick conversion** — STS3215 servos use 12-bit position (0-4096 ticks, center=2048), the controller converts degrees↔ticks manually (bypasses lerobot's normalization layer)
- **Position cache** — keeps last-known position in `self._positions[5]` for simulation and read-error fallback

### 4.2 Motor Configuration

```python
MOTOR_MODEL   = "sts3215"           # Feetech servo model
JOINT_NAMES   = ["joint_1", ..., "joint_5"]  # Arm joints
GRIPPER_NAME  = "joint_6"           # Gripper motor
TICKS_CENTER  = 2048                # 12-bit servo center
TICKS_PER_DEG = 4096 / 360.0        # 11.377 ticks per degree
```

**Motor ID mapping:**

| ID | Name | Function |
|----|------|----------|
| 1 | `joint_1` | Shoulder Pan (base rotation) |
| 2 | `joint_2` | Shoulder Lift |
| 3 | `joint_3` | Elbow Flex |
| 4 | `joint_4` | Wrist Pitch |
| 5 | `joint_5` | Wrist Roll |
| 6 | `joint_6` | Gripper |

### 4.3 Register Read/Write Path

Every register access flows through `_write()` / `_read()`:

```
set_joint(0, 45.0)
  │
  ├── angle clamped to JOINT_LIMITS[0] = [-180, 180]
  ├── _positions[0] = 45.0
  │
  └── _write("Goal_Position", _deg_to_ticks(45.0), "joint_1")
        │
        ├── self._bus.write("Goal_Position", 2600, "joint_1", normalize=False)
        │     │
        │     └── lerobot FeetechMotorsBus.write()
        │           │
        │           ├── scservo_sdk PacketHandler.write()
        │           │     └── serial port write → servo ID 1
        │           │
        │           └── waits for ACK/status packet from servo
        │
        └── or self._bus.write("Goal_Position", 2600, "joint_1")  # old API
```

Key detail: lerobot 0.5.x applies built-in normalization to `Goal_Position` / `Present_Position` (converting between raw ticks and user units). The bridge **disables** this by setting `bus.normalized_data = []` after connect, because `ArmController` handles the conversion itself using `_deg_to_ticks` / `_ticks_to_deg`. This avoids the need for lerobot's calibration system to be set up.

### 4.4 Telemetry Read

`get_state()` reads these registers per joint:

| Register | Scaling | Purpose |
|----------|---------|---------|
| `Present_Position` | _ticks_to_deg → ° | Joint angle |
| `Present_Current` | × 0.01 → A | Motor current draw |
| `Present_Temperature` | 1:1 → °C | Motor temperature |
| `Present_Voltage` | × 0.1 → V | Bus voltage (read from joint_1 only) |

### 4.5 Forward Kinematics

Simple 2D (X-Z plane) kinematic chain with 4 segments:

```python
L = [117.0, 130.0, 124.0, 60.0]  # mm — shoulder, upper arm, forearm, wrist
```

The `_fk()` method sums cos/sin contributions for each joint angle, returning `{x, y, z}` in mm. This is a simplified 2D model and ignores Y-axis (always 0).

### 4.6 Torque Control

`set_torque(enable: bool)` enables or disables torque on all 6 motors by writing `Torque_Enable`:

```python
def set_torque(self, enable: bool):
    val = 1 if enable else 0
    for name in JOINT_NAMES + [GRIPPER_NAME]:
        self._write("Torque_Enable", val, name)
```

This is used by the teleop subsystem to put the leader arm into free-move mode on start and re-enable torque on stop. In simulation mode (`SimArmController`), `set_torque` is a no-op.

### 4.7 Lerobot API Detection

```python
try:
    from lerobot.motors.feetech import FeetechMotorsBus           # lerobot >= 0.4
    from lerobot.motors.motors_bus import Motor, MotorNormMode    # new API
    API = "new"
except ImportError:
    from lerobot.common.robot_devices.motors.feetech import ...   # lerobot <= 0.3
    API = "old"
except ImportError:
    API = None → simulation only
```

The `_make_motors_dict()` function builds the appropriate format:
- **New API**: `{"joint_1": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100), ...}`
- **Old API**: `{"joint_1": (1, "sts3215"), ...}`

---

## 5. Camera System

### 5.1 `camera_manager.py` — CameraManager + CameraStream

Each camera runs in a **dedicated daemon thread** that loops at the configured FPS, capturing frames from OpenCV `VideoCapture` and storing the latest frame under a thread lock.

```
CameraManager
  ├── CameraStream(device_id=0)  → /dev/video0 (wrist)
  │     ├── thread: _capture_loop()
  │     │     ├── cv2.VideoCapture(0)
  │     │     ├── sets width, height, fps, codec
  │     │     └── loop: cap.read() → self._frame (under lock)
  │     │
  │     └── get_jpeg_b64() → base64 JPEG string
  │
  └── CameraStream(device_id=2)  → /dev/video2 (overhead)
        └── (same structure)
```

**Simulation fallback**: If OpenCV is unavailable or the camera can't be opened, `_sim_loop()` generates frames from an optional `sim_frame_callback` (which calls `SimArmController.render_frame()` for MuJoCo offscreen renders), or falls back to animated gradient frames with a crosshair and timestamp label.

The `sim_frame_callback` is injected by `server.py` via `_get_sim_camera_callback()` at camera start. It maps integer device indices to MuJoCo camera names via the `SIM_CAMERAS` config key (default: `["front", "top", "side"]`).

### 5.2 `mjpeg_server.py` — MJPEG HTTP Server

Built on `aiohttp`. Routes:

| Route | Method | Purpose |
|-------|--------|---------|
| `GET /` | → `_index_handler` | Serve `static/index.html` |
| `GET /cam/{dev}` | → `_stream_handler` | MJPEG stream (multipart/x-mixed-replace) |
| `GET /snapshot/{dev}` | → `_snapshot_handler` | Single JPEG frame |
| `GET /stats` | → `_stats_handler` | Per-camera FPS + frame count JSON |
| `GET /episode-frame/{name}/{cam}/{idx}` | → `_episode_frame_handler` | Saved episode frame JPEG |

The MJPEG stream works by sending an infinite sequence of JPEG frames separated by a boundary string:

```
--jpgboundary\r\n
Content-Type: image/jpeg\r\n
Content-Length: 42351\r\n
\r\n
<JPEG bytes>\r\n
--jpgboundary\r\n
Content-Type: image/jpeg\r\n
Content-Length: 42410\r\n
\r\n
<JPEG bytes>\r\n
...
```

The browser `<img>` tag decodes this automatically — no JavaScript needed for live video.

Camera manager is injected into `mjpeg_server` via `set_camera_manager()` at connection time.

---

## 6. Episode Recording

### 6.1 `episode_recorder.py` — EpisodeRecorder

Recording runs in a **separate daemon thread** at `RECORD_FPS` Hz (default 50). Each frame captures:

1. **Arm state** — calls `arm.get_state()` for joint angles + gripper
2. **Teleop leader state** — if `teleop_controller` is active, records `leader_joints` + `leader_gripper` alongside the follower (arm) state
3. **Camera frames** — calls `cams.get_frame(dev)` for each camera, writes JPEG to disk
4. **Timestamp** — elapsed seconds since recording start

Thread safety: all frame data is stored under `self._lock` so the stop/save path can safely copy it.

### 6.2 Recording Flow

```
START → _record_loop()
  │
  ├── while self._running:
  │     ├── arm_state = arm.get_state()
  │     ├── for each camera device:
  │     │     ├── frame = cams.get_frame(dev)
  │     │     ├── cv2.imwrite(f"cam_{dev}_{idx:06d}.jpg", frame)
  │     │     └── record relative path in frame dict
  │     ├── append frame dict to self._frames[]
  │     ├── idx++
  │     └── sleep(1/fps - elapsed) to maintain rate
  │
STOP → _save_metadata()
  │
  ├── Write episode.json (frame_count, duration, all frame data)
  ├── Write joints.csv (flat CSV for quick inspection)
  └── Return {name, frames, duration_s, size_mb, path}
```

### 6.3 Actual vs. Configured FPS

The recording loop respects wall-clock time. If `get_state()` or camera capture takes longer than `1/fps`, the loop skips the sleep and runs at the maximum achievable rate. The actual rate can be observed in the GUI's frame stats (FS-FPS telemetry field).

### 6.4 Discard

If `discard()` is called, `shutil.rmtree()` deletes the entire episode directory. This is triggered by the E-stop or the DISCARD button.

---

## 7. Replay System

### 7.1 Replay Architecture

Replay uses the same `episode.json` files created during recording. The flow is:

```
GUI: loadReplay(i)
  │
  ├── send({cmd: "get_episode", name: "episode_001"})
  │     └── Server reads episode.json, sends back as episode_data
  │
  ├── Frontend stores frames[] array locally
  ├── Renders timeline from frame data (bar heights = joint activity)
  │
  └── User clicks PLAY
        │
        ├── send({cmd: "replay_episode", name, speed})
        │     └── Server creates async task:
        │           for each frame:
        │             arm.set_joints(frame.joints)
        │             arm.set_gripper(frame.gripper)
        │             broadcast replay_frame event
        │             sleep(1/fps/speed)
        │
        └── Frontend receives replay_frame events
              ├── Updates timeline head position
              ├── Updates joint bar graph
              ├── Sets cam-rp-img src to MJPEG episode-frame URL
              └── Frame counter updates
```

### 7.2 Episode Frame Serving

Recorded camera frames are served by the MJPEG server at:

```
GET /episode-frame/{episode_name}/{camera_id}/{frame_index}
```

The path resolves to `~/datasets/so101/{name}/frames/cam_{cam}_{idx:06d}.jpg`.

During replay, the frontend sets `<img id="cam-rp-img">` src to this URL for each frame index.

### 7.3 Timeline Rendering

The timeline bars are computed from actual joint data:

```javascript
const avg = joints.reduce((a,b)=>a+Math.abs(b),0)/joints.length;
const op = 0.3 + Math.min(0.7, avg / 90);  // opacity = activity level
```

Each bar's height encodes J1 activity. The click-to-scrub and ±5s skip buttons use `rpFrameIdx` to jump to the corresponding frame index.

### 7.4 Speed Control

Replay speed is a multiplier applied as: `dt = 1.0 / fps / speed`. The server's replay loop uses `asyncio.sleep(dt)` between frames. Speed 0.25× = 4× slow motion, 2× = double speed.

---

## 8. Calibration System

### 8.1 5-Step Wizard

The GUI presents 5 sequential steps:

| Step | Server Method | What Happens |
|------|--------------|--------------|
| 0 | `_cal_home` | Arm moves to HOME_ANGLES (all 0°) |
| 1 | `_cal_zero` | Reads Present_Position for each joint, stores as zero offsets |
| 2 | `_cal_rom` | Prompts user to manually sweep each joint through full range |
| 3 | `_cal_stiffness` | Writes Maximum_Acceleration from MAX_TORQUE_PCT |
| 4 | `_cal_gripper` | Opens (0%) → closes (100%) → centers (50%) |

The GUI advances on button click. Each step sends `cal_step` and logs the result.

### 8.2 Calibration Storage

Saved to `calibration.json` in the project root via `save_calibration`:

```json
{
  "zero_offsets": {
    "joint_1": 2048,
    "joint_2": 2052,
    "joint_3": 2040,
    "joint_4": 2045,
    "joint_5": 2051
  }
}
```

Auto-loaded on next `connect()` if the file exists.

---

## 9. Configuration System

### 9.1 `config.py` — Config Class

Three-layer priority:

1. Compile-time defaults (`_DEFAULTS` dict in `config.py`)
2. `config.json` file (overrides defaults)
3. Environment variables `SO101_{KEY}` (highest priority)

Environment values are parsed as JSON if possible, otherwise treated as strings:
```bash
SO101_DEFAULT_PORT=/dev/ttyACM0 SO101_CAMERA_DEVICES='[0, 4]' python server.py
```

### 9.2 All Config Keys

| Key | Default | Type | Description |
|-----|---------|------|-------------|
| `WS_HOST` | `"0.0.0.0"` | str | WebSocket bind address |
| `WS_PORT` | `8765` | int | WebSocket port |
| `MJPEG_PORT` | `8766` | int | HTTP/MJPEG port |
| `DEFAULT_PORT` | `"/dev/ttyACM0"` | str | Serial port |
| `DEFAULT_BAUD` | `1000000` | int | Serial baud rate |
| `ROBOT_TYPE` | `"so101"` | str | Robot model name |
| `MOTOR_IDS` | `[1,2,3,4,5,6]` | int[] | Servo bus IDs |
| `HOME_ANGLES` | `[0,0,0,0,0]` | float[] | Home position (degrees) |
| `JOINT_LIMITS` | see config.py | float[][] | Min/max per joint |
| `GRIPPER_OPEN` | `0` | float | Gripper open position (ticks) |
| `GRIPPER_CLOSED` | `100` | float | Gripper closed position (ticks) |
| `CAMERA_DEVICES` | `[0, 2]` | int[] | Video device indices |
| `CAMERA_WIDTH` | `1280` | int | Capture width |
| `CAMERA_HEIGHT` | `720` | int | Capture height |
| `CAMERA_FPS` | `30` | int | Capture framerate |
| `CAMERA_CODEC` | `"MJPG"` | str | FourCC codec |
| `RECORD_FPS` | `50` | int | Episode recording rate |
| `DATASET_DIR` | `"~/datasets/so101"` | str | Episode output directory |
| `DATASET_FORMAT` | `"lerobot_v2"` | str | Dataset schema version |
| `CAL_FILE` | `"calibration.json"` | str | Calibration save path |
| `MAX_TORQUE_PCT` | `60.0` | float | Motor torque limit % |
| `COLLISION_DETECTION` | `true` | bool | Enable collision safety |
| `SIM_MODE` | `null` | str\|null | `"mujoco"` to force MuJoCo sim, `null` = auto |
| `LEADER_PORT` | `"/dev/ttyACM0"` | str | Teleop leader arm serial port |
| `FOLLOWER_PORT` | `"/dev/ttyACM1"` | str | Teleop follower arm serial port |
| `TELEOP_RATE` | `100` | int | Teleop leader→follower control loop rate (Hz) |
| `SIM_CAMERAS` | `["front","top","side"]` | str[] | MuJoCo camera names for offscreen render |

---

## 10. Frontend Architecture

### 10.1 Technology

Single HTML file (`static/index.html`). No build step, no framework, no dependencies. Everything is vanilla JS + CSS.

### 10.2 Key JavaScript Modules (within the script tag)

| Module | Lines | Purpose |
|--------|-------|---------|
| WebSocket client | 470-530 | Connect/reconnect, send/receive |
| Message handler | 536-554 | Route incoming messages by type |
| Joint UI builder | 661-671 | Generates 5 slider rows dynamically |
| Telemetry updater | 584-639 | Updates sliders, metrics, SVG from live data |
| Arm SVG renderer | 688-721 | 2D kinematics (FK side-view) with gripper |
| Recording controls | 748-799 | Start/stop/discard episode capture |
| Episode list/Replay | 801-890 | List episodes, load data, play/scrub |
| Calibration wizard | 892-937 | 5-step UI with progress bar |
| Settings | 940-950 | Save/get config, push to HF |
| Teleop UI | 470-514 | Leader/follower state panels, start/stop buttons |
| Log viewer | 954-965 | Scrollable log with timestamp |
| Tab system | 975-982 | Panel visibility switching |

### 10.3 State Variables (Frontend)

| Variable | Type | Purpose |
|----------|------|---------|
| `ws` | WebSocket | Connection object |
| `wsReady` | bool | Connection state |
| `armConnected` | bool | Arm connection state |
| `recording` | bool | Recording in progress |
| `replayActive` | bool | Replay playing |
| `rpEpisode` | object | Current loaded episode |
| `rpFrames` | array | Full frame data for loaded episode |
| `rpFrameIdx` | int | Current frame during replay/scrub |
| `rpSpeed` | float | Replay speed multiplier |

### 10.4 Arm SVG Visualization

The arm SVG uses a forward-kinematics side-view model matching the hardware geometry:

```javascript
const ARM_L = [117, 130, 124, 60]; // mm: J1→J2, J2→J3, J3→J4, J4→J5
```

The `updateArmSVG(joints)` function computes 2D positions via `fkSideView()` and updates SVG `line`/`circle` element positions dynamically via `setAttr()`. The gripper fingers (`gf1`, `gf2`) open/close proportionally to the current gripper percentage read from the DOM. CSS transitions on `line` and `circle` elements (`transition: all .08s`) provide smooth animation.

### 10.5 Teleop UI

The TELEOP tab (panel-teleop) shows two side-by-side cards for Leader and Follower state. Each card displays:
- Port selector for the serial device
- 5 joint angle readouts (updated from telemetry `teleop.leader.joints` / `teleop.follower.joints`)
- Gripper percentage

A teleop control bar has START/STOP buttons and a status badge. The `startTeleop()` / `stopTeleop()` functions send `start_teleop` / `stop_teleop` commands. The `onTelemetry` handler updates the leader/follower readouts from the `teleop` key in the telemetry message.

### 10.6 WebSocket Auto-Reconnect

On unexpected disconnect, the client retries every 3 seconds indefinitely:

```javascript
ws.onclose = (ev) => {
    if (!_manualDisconnect) {
        _reconnectTimer = setTimeout(_doConnect, 3000);
    }
};
```

Manual disconnect (clicking DISCONNECT BRIDGE) sets `_manualDisconnect = true` to suppress auto-retry.

### 10.7 MJPEG URL Construction

Camera image URLs use the page's hostname (works across LAN without hardcoded localhost):

```javascript
const MJPEG_URL = `http://${location.hostname || 'localhost'}:8766`;
```

Set dynamically when WebSocket opens:
```javascript
document.getElementById('cam1-img').src = MJPEG_URL + '/cam/0';
document.getElementById('cam2-img').src = MJPEG_URL + '/cam/2';
```

---

## 11. Dataset Format

### 11.1 Directory Structure

```
~/datasets/so101/episode_001/
├── episode.json          ← Full metadata + frame data (LeRobot v2 schema)
├── joints.csv            ← Flat CSV for quick inspection
└── frames/
    ├── cam_0_000000.jpg  ← Wrist camera, frame 0
    ├── cam_0_000001.jpg  ← Wrist camera, frame 1
    ├── cam_2_000000.jpg  ← Overhead camera, frame 0
    └── ...               ← One JPEG per camera per frame
```

### 11.2 `episode.json` Schema

```json
{
  "name": "episode_001",
  "task": "Pick and place",
  "fps": 50,
  "frame_count": 842,
  "duration_s": 16.84,
  "size_bytes": 84500000,
  "created_at": "2026-05-09T17:30:00",
  "format": "lerobot_v2",
  "frames": [
    {
      "idx": 0,
      "timestamp": 0.0000,
      "joints": [0.0, 0.0, 0.0, 0.0, 0.0],
      "gripper": 50.0,
      "cameras": {
        "cam_0": "frames/cam_0_000000.jpg",
        "cam_2": "frames/cam_2_000000.jpg"
      }
    },
    {
      "idx": 1,
      "timestamp": 0.0200,
      "joints": [0.5, -1.2, 0.8, -0.3, 0.1],
      "gripper": 50.0,
      "leader_joints": [0.6, -1.1, 0.9, -0.2, 0.0],
      "leader_gripper": 50.0,
      "cameras": {
        "cam_0": "frames/cam_0_000001.jpg",
        "cam_2": "frames/cam_2_000001.jpg"
      }
    }
  ]
}
```

### 11.3 `joints.csv` Schema

```csv
idx,timestamp,j1,j2,j3,j4,j5,gripper,l_j1,l_j2,l_j3,l_j4,l_j5,l_gripper
0,0.0000,0.000,0.000,0.000,0.000,0.000,50.0,0.000,0.000,0.000,0.000,0.000,50.0
1,0.0200,0.500,-1.200,0.800,-0.300,0.100,50.0,0.600,-1.100,0.900,-0.200,0.000,50.0
```

### 11.4 LeRobot Compatibility

The dataset can be pushed to HuggingFace Hub for training:

```bash
python -m lerobot.scripts.push_dataset_to_hub \
  --repo-id username/so101-dataset \
  --raw-dir ~/datasets/so101
```

Or directly from the GUI Settings tab.

---

## 12. Simulation Backend

### 12.1 Overview

The simulation backend (`sim_backend.py`) provides a drop-in replacement for `ArmController` using the MuJoCo physics engine. When `SIM_MODE` is set to `"mujoco"` in `config.json` and MuJoCo is installed, `server.py` instantiates `SimArmController` instead of `ArmController` on `connect`.

### 12.2 SimArmController

`SimArmController` mirrors the `ArmController` public API (`connect`, `disconnect`, `set_joint`, `set_joints`, `set_gripper`, `get_state`, `emergency_stop`, `set_torque`, `go_home`, `go_zero`, `calibrate_step`, `save_calibration`, `render_frame`).

**Constructor:**
```python
class SimArmController:
    N_JOINTS = 5
    def __init__(self, port, baud, config):
```

The `port` and `baud` parameters are accepted for API compatibility but ignored — MuJoCo uses a local MJCF model file.

**Physics model:**
- Loads `so101.xml` (MJCF format) from the project root
- 6 actuated degrees of freedom (5 arm joints + 1 gripper)
- `qpos[0..4]` mapped to arm joint angles (radians), `qpos[5]` to gripper gap
- Default gripper gap: 0.009 (approximately 50% open)
- `mj_forward()` called each frame to advance physics

### 12.3 3D Viewer

A GLFW-based 3D viewer runs in a daemon thread:

```
connect() → _run_viewer() thread
  │
  ├── GLFW init (mutex-protected, single init)
  ├── Create 800×600 window titled "SO-101 MuJoCo Sim"
  ├── Configure camera: distance=0.6, azimuth=90, elevation=-30
  ├── Scroll to zoom (distance clamped ≥ 0.1)
  │
  └── Loop: mj_forward → mjv_updateScene → mjr_render → swap buffers
```

If GLFW is unavailable or window creation fails, the sim falls back to a headless loop (`_headless_loop`) that runs `mj_forward` at ~100 Hz without rendering.

### 12.4 Offscreen Camera Rendering

`render_frame(camera_name, width, height)` renders a MuJoCo camera view to a numpy RGB array:

```python
def render_frame(self, camera_name: str, width: int, height: int) -> Optional[np.ndarray]:
    # Lazy-init renderer at requested resolution
    # mj_forward → update_scene → render()
    # Returns RGB ndarray or None on error
```

This is called by the `sim_frame_callback` injected into `CameraManager` when cameras start in sim mode. The `SIM_CAMERAS` config key (default `["front", "top", "side"]`) maps integer device indices to MuJoCo camera names defined in `so101.xml`.

The callback (in `server.py`):
```python
def _get_sim_camera_callback():
    dev → cam_names[dev] ("front", "top", "side")
    render(cam_name, w, h) → RGB ndarray
    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR) → BGR for MJPEG stream
```

### 12.5 Telemetry in Sim Mode

`get_state()` returns:

| Field | Value |
|-------|-------|
| `joints` | Current in-memory positions |
| `gripper` | 0.0 – 100.0 |
| `tcp` | FK-computed (2D side-view) |
| `current` | `[0.0] * N_JOINTS` |
| `temperature` | `[25.0] * N_JOINTS` |
| `voltage` | 12.0 |
| `mode` | `"simulation (mujoco)"` |

### 12.6 MJCF Model (so101.xml)

The MuJoCo XML model at `so101.xml` defines:
- 5-DOF serial arm with shoulder pan/lift, elbow, wrist pitch/roll
- Gripper with prismatic joint
- Collision geoms for each link
- Named cameras: `"front"`, `"top"`, `"side"` for offscreen rendering

The model is loaded at `SimArmController.connect()` time.

### 12.7 Dependencies

```bash
pip install mujoco       # Physics engine (required)
# Optional:
pip install mujoco-python  # GLFW viewer (if window desired)
```

If MuJoCo is not installed, `SimArmController.connect()` returns `False` and the bridge logs an error.

---

## 13. Teleoperation

### 13.1 Overview

The teleoperation subsystem (`teleop.py`) implements a leader/follower control loop. The leader arm is manually guided (torque disabled = free-move), and its joint positions are continuously read and mirrored on the follower arm at a configurable rate.

### 13.2 TeleopController

```python
class TeleopController:
    def __init__(self, leader, follower, config):
```

- `leader` / `follower` — Either `ArmController` or `SimArmController` instances
- `TELEOP_RATE` (default 100 Hz) — Control loop frequency

### 13.3 Lifecycle

```
start_teleop cmd received
  │
  ├── 1. Create leader + follower controllers (ArmController or SimArmController)
  │     └── SIM_MODE="mujoco" → SimArmController for both
  │
  ├── 2. leader.connect(), follower.connect()
  │
  ├── 3. leader.set_torque(False) → free-move mode
  │
  ├── 4. Start _loop() in daemon thread:
  │     ├── while running:
  │     │     ├── leader_state = leader.get_state()
  │     │     ├── follower_state = follower.get_state()
  │     │     ├── follower.set_joints(leader.joints)
  │     │     ├── follower.set_gripper(leader.gripper)
  │     │     ├── Store both states under lock
  │     │     └── sleep(1/rate - elapsed)
  │
  ├── 5. Broadcast {"type": "status", "teleop": true}
  │
  └── Telemetry loop appends teleop.leader + teleop.follower to 20 Hz broadcast
```

**Stop:**
```
stop_teleop cmd received
  │
  ├── 1. Set _running = False, join thread (3s timeout)
  ├── 2. leader.set_torque(True) → re-enable torque
  ├── 3. leader.disconnect(), follower.disconnect()
  └── 4. Broadcast {"type": "status", "teleop": false}
```

### 13.4 Telemetry Shape

When teleop is active, every telemetry message includes a `teleop` key:

```json
{
  "type": "telemetry",
  "joints": [...],           // Follower (arm) joints
  "gripper": 50.0,
  "teleop": {
    "active": true,
    "leader": { "joints": [...], "gripper": 50.0, "tcp": {...}, ... },
    "follower": { "joints": [...], "gripper": 50.0, "tcp": {...}, ... }
  }
}
```

### 13.5 Recording with Teleop

When the recorder is started while teleop is active, it captures `leader_joints` and `leader_gripper` in each frame alongside the follower (arm) state. The CSV output includes `l_j1`–`l_j5` and `l_gripper` columns.

This enables behavioral cloning training where the policy learns to map leader (human demonstration) states to follower actions.

### 13.6 WebSocket Commands

| Command | Parameters | Description |
|---------|-----------|-------------|
| `start_teleop` | `leader_port`, `follower_port` | Begin leader/follower teleoperation |
| `stop_teleop` | — | Stop teleoperation, re-enable leader torque |

Events broadcast: `{"type": "status", "teleop": true/false}` (also `teleop_started` / `teleop_stopped`).

### 13.7 Configuration

Keys in `config.json`:

| Key | Default | Description |
|-----|---------|-------------|
| `LEADER_PORT` | `"/dev/ttyACM0"` | Leader arm serial port |
| `FOLLOWER_PORT` | `"/dev/ttyACM1"` | Follower arm serial port |
| `TELEOP_RATE` | `100` | Control loop frequency (Hz) |

---

## 14. Safety Systems

### 14.1 Emergency Stop

```python
def emergency_stop(self):
    for name in JOINT_NAMES + [GRIPPER_NAME]:
        self._write("Torque_Enable", 0, name)  # Cuts power to all servos
```

Also discards any active recording. Broadcasts an `estop` event to all GUI clients. The mode badge switches to IDLE.

### 14.2 Joint Limits

Each joint has hard-coded software limits in `JOINT_LIMITS`. The `set_joint` method clamps before writing:

```python
lo, hi = self.cfg.JOINT_LIMITS[joint_id]
angle = max(float(lo), min(float(hi), float(angle)))
```

### 14.3 Torque Limit

`MAX_TORQUE_PCT` (default 60%) limits the `Maximum_Acceleration` register value, reducing maximum torque output during calibration.

### 14.4 Collision Detection

Config option `COLLISION_DETECTION` (default `true`) is a placeholder for future expansion. Not currently enforced in the motor control path.

### 14.5 Disconnect Safety

On `disconnect()` and server shutdown, `Torque_Enable` is set to 0 on all motors before closing the serial bus. This prevents servos from holding position if the software crashes.

---

## 15. Troubleshooting

### 15.1 Serial Port Permission Denied

```
ERROR: Could not connect on port '/dev/ttyACM0'.
```

Fix:
```bash
sudo usermod -aG dialout $USER
# Then LOG OUT and back in (newgrp or sg dialout for temporary fix)
```

Verify group membership:
```bash
groups  # should include "dialout"
ls -la /dev/ttyACM0  # should show crw-rw---- root dialout
```

### 15.2 Wrong Baud Rate

The SO-101's STS3215 servos communicate at **1000000** baud. Using 115200 will fail silently. Check `config.json` or the GUI baud selector.

### 15.3 No lerobot Found

```
WARNING: No lerobot motor driver found — SIMULATION mode only
```

Install:
```bash
source .venv/bin/activate
pip install lerobot
```

### 15.4 Camera Not Opening

```
ERROR: Cannot open camera 0 — falling back to simulation
```

Check camera indices with:
```bash
ls /dev/video*
v4l2-ctl --list-devices
```

Update `CAMERA_DEVICES` in `config.json`.

### 15.5 WebSocket Won't Connect

- Verify server is running: `curl http://localhost:8766/` should return HTTP 200
- Check port 8765 isn't blocked by firewall
- If accessing from another machine, use the machine's IP address, not localhost
- The GUI auto-detects the hostname — no configuration needed

### 15.6 Hardware Connected but No Telemetry

Check the server log for `SERVO` lines. If they appear but the GUI shows zeros, check:
- WebSocket is connected (green dot in top bar)
- CONNECT ARM was clicked (mode badge shows HARDWARE, not SIM)
- All 6 motor IDs are unique on the bus (run `lerobot-setup-motors` to configure)

### 15.7 Joint Values Seem Wrong

The arm's current position in degrees reflects raw encoder ticks relative to the servo's zero position (2048 ticks = center). If the arm was assembled at a different physical offset, the angles will be offset. Run calibration (CALIBRATE tab) to record zero offsets.

### 15.8 MuJoCo Not Installed

```
ERROR: MuJoCo not installed — run: pip install mujoco
```

Install:
```bash
source .venv/bin/activate
pip install mujoco   # Physics engine
pip install mujoco-python  # Optional — 3D viewer
```

### 15.9 Sim Mode Port Conflicts

In sim mode both `LEADER_PORT` and `FOLLOWER_PORT` can be set to the same value since the serial port is ignored by `SimArmController`. If using real hardware for one side, ensure the other side's port is distinct or set to a simulated controller.

### 15.10 Teleop Connection Refused

```
ERROR: Teleop start failed: [Errno 2] No such file or directory: '/dev/ttyACM1'
```

Check that the follower arm's serial port exists. In simulation mode, both arms use MuJoCo so any port value works. For hardware teleop, verify both ports with:
```bash
ls -la /dev/ttyACM*
```

### 15.11 Teleop Timing Mismatch

If the follower visibly lags behind the leader, reduce `TELEOP_RATE` in `config.json` (default 100 Hz). The actual rate is limited by the slower of leader read time, follower write time, and the configured rate.

---

## Appendix: MJPEG Stream Protocol

Each camera stream is `multipart/x-mixed-replace` with boundary `--jpgboundary`:

```http
HTTP/1.1 200 OK
Content-Type: multipart/x-mixed-replace; boundary=--jpgboundary
Cache-Control: no-cache
Connection: keep-alive

--jpgboundary
Content-Type: image/jpeg
Content-Length: 42351

<JPEG bytes>
--jpgboundary
Content-Type: image/jpeg
Content-Length: 42410

<JPEG bytes>
--jpgboundary
...
```

The browser treats this as a never-ending image. Each new JPEG replaces the previous one automatically. Frame rate is capped at 30 fps server-side.

---

## Appendix: Port Reference

| Port | Service | Protocol | Config Key |
|------|---------|----------|-----------|
| 8765 | WebSocket (arm control) | `ws://` | `WS_PORT` |
| 8766 | HTTP (GUI + cameras) | `http://` | `MJPEG_PORT` |

The GUI is accessible at `http://{host}:8766/`.  
WebSocket auto-connects to `ws://{host}:8765`.

---

## Appendix: File Reference

| File | Purpose |
|------|---------|
| `server.py` | Entry point — WebSocket + MJPEG servers |
| `arm_controller.py` | Motor control (lerobot Feetech bus wrapper) |
| `sim_backend.py` | MuJoCo simulation backend (SimArmController) |
| `teleop.py` | Leader/follower teleoperation controller |
| `camera_manager.py` | Multi-camera capture threads |
| `episode_recorder.py` | Episode recording to disk |
| `mjpeg_server.py` | HTTP MJPEG stream server |
| `config.py` | Configuration loader (file + env) |
| `config.json` | User settings |
| `so101.xml` | MuJoCo MJCF model for simulation |
| `static/index.html` | Web GUI (single-file app) |
| `requirements.txt` | Python dependencies |
| `deploy.sh` | One-shot install + launch |
| `calibration.json` | Saved calibration data |
| `logs/bridge.log` | Server log file |
