#!/usr/bin/env bash
# ============================================================
#  SO-101 Bridge — deploy & launch script
#  Usage: bash deploy.sh [--sim]   (--sim = simulation mode)
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

SIM_MODE=0
for arg in "$@"; do
  [[ "$arg" == "--sim" ]] && SIM_MODE=1
done

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║       SO-101 Robotic Arm Control Bridge          ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""

# ── 1. Python check ──────────────────────────────────────────────────────
PYTHON=$(command -v python3 || command -v python || echo "")
if [[ -z "$PYTHON" ]]; then
  echo "❌  Python 3.10+ is required. Install with: sudo apt install python3"
  exit 1
fi
PY_VER=$($PYTHON -c "import sys; print(sys.version_info.minor)")
echo "✅  Python: $($PYTHON --version)"

# ── 2. Virtual environment ───────────────────────────────────────────────
VENV="$SCRIPT_DIR/.venv"
if [[ ! -d "$VENV" ]]; then
  echo "🔧  Creating virtual environment..."
  $PYTHON -m venv "$VENV"
fi
source "$VENV/bin/activate"
echo "✅  Virtual environment: $VENV"

# ── 3. Install dependencies ──────────────────────────────────────────────
echo "📦  Installing Python dependencies..."
pip install --quiet --upgrade pip
pip install --quiet -r requirements.txt

# Check lerobot
if python -c "import lerobot" 2>/dev/null; then
  echo "✅  lerobot found"
else
  echo "⚠️   lerobot not found — installing from PyPI..."
  pip install --quiet lerobot || {
    echo "⚠️   PyPI install failed, trying GitHub HEAD..."
    pip install --quiet "git+https://github.com/huggingface/lerobot.git" || true
  }
fi

# ── 4. System packages (best-effort) ────────────────────────────────────
if command -v apt-get &>/dev/null; then
  echo "📦  Checking system packages..."
  sudo apt-get install -y -qq \
    libusb-1.0-0 \
    v4l-utils \
    ffmpeg \
    2>/dev/null || true
fi

# ── 5. USB serial permissions ────────────────────────────────────────────
if ! groups | grep -q dialout; then
  echo "⚠️   Adding $USER to 'dialout' group for serial access..."
  sudo usermod -aG dialout "$USER"
  echo "   ⚠️  Log out and back in (or run: newgrp dialout) for this to take effect."
fi

# ── 6. Create directories ────────────────────────────────────────────────
mkdir -p logs recordings

# ── 7. Default config ────────────────────────────────────────────────────
if [[ ! -f config.json ]]; then
  echo "📝  Writing default config.json..."
  cat > config.json <<'EOF'
{
  "WS_HOST": "0.0.0.0",
  "WS_PORT": 8765,
  "MJPEG_PORT": 8766,
  "DEFAULT_PORT": "/dev/ttyUSB0",
  "DEFAULT_BAUD": 1000000,
  "ROBOT_TYPE": "so101",
  "CAMERA_DEVICES": [0, 2],
  "CAMERA_WIDTH": 1280,
  "CAMERA_HEIGHT": 720,
  "CAMERA_FPS": 30,
  "RECORD_FPS": 50,
  "DATASET_DIR": "~/datasets/so101",
  "MAX_TORQUE_PCT": 60.0
}
EOF
fi

# ── 8. systemd service (optional) ────────────────────────────────────────
SERVICE_FILE="/etc/systemd/system/so101-bridge.service"
if command -v systemctl &>/dev/null && [[ ! -f "$SERVICE_FILE" ]]; then
  echo ""
  read -r -p "Install systemd service for auto-start? [y/N] " INSTALL_SVC
  if [[ "${INSTALL_SVC,,}" == "y" ]]; then
    sudo bash -c "cat > $SERVICE_FILE" <<EOF
[Unit]
Description=SO-101 Robotic Arm Control Bridge
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$SCRIPT_DIR
ExecStart=$VENV/bin/python server.py
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
    sudo systemctl daemon-reload
    sudo systemctl enable so101-bridge
    echo "✅  systemd service installed. Start with: sudo systemctl start so101-bridge"
  fi
fi

# ── 9. Launch ─────────────────────────────────────────────────────────────
echo ""
echo "══════════════════════════════════════════════════"
echo "  Starting SO-101 bridge..."
echo ""
echo "  ✅  Open this URL in Chrome / Firefox:"
echo "      http://localhost:8766"
echo ""
echo "  WebSocket : ws://0.0.0.0:8765"
echo "  Cameras   : http://0.0.0.0:8766/cam/0"
echo ""
if [[ $SIM_MODE -eq 1 ]]; then
  echo "  ⚠️  SIMULATION MODE — no hardware required"
  echo ""
fi
echo "  Press Ctrl+C to stop."
echo "══════════════════════════════════════════════════"
echo ""

# Open browser after 1.5s (give server time to start)
(sleep 1.5 && xdg-open "http://localhost:8766" 2>/dev/null || true) &

exec python server.py
