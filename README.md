# SO-101 Robotic Arm Control Bridge

Full-stack bridge between the web GUI and the SO-101 robot arm via lerobot.

```
so101-bridge/
├── server.py           ← Main WebSocket + MJPEG server (entry point)
├── arm_controller.py   ← lerobot / Feetech motor control wrapper
├── camera_manager.py   ← Multi-camera OpenCV capture threads
├── episode_recorder.py ← Joint + image episode recording
├── mjpeg_server.py     ← HTTP MJPEG stream server (aiohttp)
├── config.py           ← Config loader (file + env vars)
├── config.json         ← Your settings (auto-created on first run)
├── requirements.txt    ← Python dependencies
├── deploy.sh           ← One-shot install + launch script
├── static/
│   └── index.html      ← Self-contained GUI (open in browser)
├── logs/               ← bridge.log
└── datasets/           ← Recorded episodes
```

---

## Quick Start

```bash
# 1. Clone / copy this folder to your Linux machine
cd so101-bridge

# 2. Run the deploy script (installs deps, starts the bridge)
bash deploy.sh

# Simulation mode (no hardware needed):
bash deploy.sh --sim

# 3. Open the GUI
xdg-open static/index.html
# or just open static/index.html in Chrome / Firefox
```

The GUI auto-connects to `ws://localhost:8765`.  
Camera streams are served at `http://localhost:8766/cam/0` and `/cam/2`.

---

## Manual install

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python server.py
```

---

## Hardware wiring

| SO-101 cable | Linux device     |
|-------------|------------------|
| Arm USB-serial | `/dev/ttyUSB0` |
| Wrist camera   | `/dev/video0`  |
| Overhead camera| `/dev/video2`  |

If you see "Permission denied" on the serial port:
```bash
sudo usermod -aG dialout $USER
# then log out and back in
```

---

## Configuration (`config.json`)

| Key | Default | Description |
|-----|---------|-------------|
| `DEFAULT_PORT` | `/dev/ttyUSB0` | Serial port |
| `DEFAULT_BAUD` | `1000000` | Baud rate (Feetech default) |
| `ROBOT_TYPE` | `so101` | `so101` or `so100` |
| `CAMERA_DEVICES` | `[0, 2]` | `/dev/videoN` indices |
| `RECORD_FPS` | `50` | Joint recording rate |
| `DATASET_DIR` | `~/datasets/so101` | Episode output dir |
| `MAX_TORQUE_PCT` | `60` | Motor torque limit % |
| `WS_PORT` | `8765` | WebSocket port |
| `MJPEG_PORT` | `8766` | Camera stream port |

All keys can be overridden with environment variables prefixed `SO101_`:
```bash
SO101_DEFAULT_PORT=/dev/ttyACM0 python server.py
```

---

## GUI features

| Tab | What it does |
|-----|-------------|
| **Control** | Live joint sliders → arm moves in real time. Kinematic SVG updates from hardware telemetry. E-stop. |
| **Calibrate** | 5-step wizard: home → zero offsets → ROM → stiffness → gripper. Saves `calibration.json`. |
| **Record** | Start/stop episode capture. Dual MJPEG camera feeds. Saves joint CSV + JPEG frames. |
| **Replay** | Load a saved episode, scrub the timeline, play it back on the real arm at any speed. |
| **Settings** | Serial port, camera devices, dataset path, HuggingFace Hub push. |

---

## Dataset format

Each episode is saved to `~/datasets/so101/<episode_name>/`:

```
episode_001/
├── episode.json    ← metadata + per-frame joint arrays
├── joints.csv      ← flat CSV (idx, timestamp, j1..j5, gripper)
└── frames/
    ├── cam_0_000000.jpg
    ├── cam_2_000000.jpg
    └── ...
```

`episode.json` is compatible with the **LeRobot v2** dataset schema.

Push to HuggingFace Hub from the Settings tab, or:
```bash
python -m lerobot.scripts.push_dataset_to_hub \
  --repo-id username/so101-dataset \
  --raw-dir ~/datasets/so101
```

---

## systemd auto-start

`deploy.sh` offers to install a systemd service. Manual install:

```bash
sudo cp so101-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now so101-bridge
journalctl -fu so101-bridge
```

---

## Ports used

| Port | Protocol | Purpose |
|------|----------|---------|
| 8765 | WebSocket | GUI ↔ bridge control |
| 8766 | HTTP MJPEG | Camera live streams |

Both ports must be open if accessing the GUI from a different machine on the LAN.
