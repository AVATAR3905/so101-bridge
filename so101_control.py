#!/usr/bin/env python3
"""
SO-101 Robotic Arm Control GUI.

Interfaces with the SO-101 (6-DOF) robotic arm using the Hugging Face LeRobot
library. Provides live telemetry, calibration shortcuts, and a dark-themed UI.

Requirements:
    pip install lerobot torch

Usage:
    python so101_control.py
"""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, scrolledtext, ttk

# ---------------------------------------------------------------------------
# Library check (runs at import time)
# ---------------------------------------------------------------------------
LEROBOT_AVAILABLE = False
_lerobot_error = ""

try:
    from lerobot.common.robot_devices.robots.manipulator import (  # type: ignore[import-untyped]
        ManipulatorRobot,
    )

    LEROBOT_AVAILABLE = True
except ImportError as exc:
    _lerobot_error = str(exc)

TORCH_AVAILABLE = False
try:
    import torch  # type: ignore[import-untyped]

    TORCH_AVAILABLE = True
except ImportError:
    pass

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
DEFAULT_ROBOT_TYPE = "so100"
TELEMETRY_HZ = 30
JOINT_COUNT = 6
MAX_CONSECUTIVE_ERRORS = 5

# Joint limits in degrees (approximate for SO-101)
JOINT_LIMITS: dict[str, tuple[float, float]] = {
    "J1": (-180.0, 180.0),
    "J2": (-90.0, 90.0),
    "J3": (-90.0, 90.0),
    "J4": (-180.0, 180.0),
    "J5": (-90.0, 90.0),
    "J6": (-180.0, 180.0),
}

# Fraction of the limit range at which we show a warning
WARNING_RATIO = 0.85

# ---------------------------------------------------------------------------
# Color palette (dark theme)
# ---------------------------------------------------------------------------
COLORS = {
    "bg": "#1e1e1e",
    "frame_bg": "#2d2d2d",
    "entry_bg": "#3c3c3c",
    "entry_fg": "#d4d4d4",
    "fg": "#d4d4d4",
    "dim": "#808080",
    "button_bg": "#0e639c",
    "button_fg": "#ffffff",
    "button_disabled_bg": "#3c3c3c",
    "safe": "#4ec9b0",
    "warning": "#ce9178",
    "danger": "#f44747",
    "log_bg": "#1a1a1a",
    "log_fg": "#cccccc",
    "log_timestamp": "#569cd6",
    "log_info": "#d4d4d4",
    "log_error": "#f44747",
    "log_success": "#4ec9b0",
    "log_warning": "#ce9178",
    "progress_bg": "#3c3c3c",
    "status_bar_bg": "#007acc",
    "highlight": "#264f78",
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _format_angle(degrees: float) -> str:
    """Format an angle value with sign and two decimal places."""
    return f"{degrees:+.2f}"


def _status_color(value: float, limits: tuple[float, float]) -> str:
    """Return the status color for a joint given its value and limits."""
    lo, hi = limits
    span = hi - lo
    if span <= 0:
        return COLORS["safe"]
    frac = (abs(value) - abs(lo)) / (span / 2) if span > 0 else 0.0
    if frac >= 1.0:
        return COLORS["danger"]
    if frac >= WARNING_RATIO:
        return COLORS["warning"]
    return COLORS["safe"]


def _status_text(value: float, limits: tuple[float, float]) -> str:
    """Return a human-readable status label for a joint."""
    lo, hi = limits
    span = hi - lo
    if span <= 0:
        return "Safe"
    frac = (abs(value) - abs(lo)) / (span / 2) if span > 0 else 0.0
    if frac >= 1.0:
        return "Limit"
    if frac >= WARNING_RATIO:
        return "Warn"
    return "Safe"


# ---------------------------------------------------------------------------
# Main Application
# ---------------------------------------------------------------------------
class SO101ControlApp:
    """Tkinter GUI for controlling and monitoring an SO-101 robotic arm."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.robot: object | None = None
        self._robot_lock = threading.Lock()
        self._telemetry_stop = threading.Event()
        self._telemetry_thread: threading.Thread | None = None
        self._telemetry_hz = TELEMETRY_HZ
        self._error_count = 0
        self._frame_times: list[float] = []

        # Per-joint tkinter variables
        self._joint_labels: list[tk.StringVar] = []
        self._joint_status_labels: list[tk.StringVar] = []
        self._joint_bars: list[ttk.Progressbar] = []

        self._setup_window()
        self._build_connection_frame()
        self._build_telemetry_frame()
        self._build_calibration_frame()
        self._build_log_frame()
        self._build_status_bar()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        if not LEROBOT_AVAILABLE:
            self._log(
                "LeRobot library not found. Install with:  pip install lerobot",
                "error",
            )
            self._log(f"Import error: {_lerobot_error}", "error")
            self._set_connection_enabled(False)
        else:
            self._log("LeRobot library detected. Ready to connect.", "success")

    # ------------------------------------------------------------------
    # Window setup
    # ------------------------------------------------------------------
    def _setup_window(self) -> None:
        self.root.title("SO-101 Robotic Arm Control")
        self.root.geometry("820x720")
        self.root.minsize(700, 600)
        self.root.configure(bg=COLORS["bg"])

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background=COLORS["frame_bg"])
        style.configure(
            "TLabelframe",
            background=COLORS["frame_bg"],
            foreground=COLORS["fg"],
        )
        style.configure(
            "TLabelframe.Label",
            background=COLORS["frame_bg"],
            foreground=COLORS["fg"],
        )
        style.configure(
            "TLabel", background=COLORS["frame_bg"], foreground=COLORS["fg"]
        )
        style.configure(
            "TButton",
            background=COLORS["button_bg"],
            foreground=COLORS["button_fg"],
            borderwidth=0,
            padding=(10, 5),
            font=("", 10),
        )
        style.map(
            "TButton",
            background=[
                ("disabled", COLORS["button_disabled_bg"]),
                ("active", COLORS["highlight"]),
            ],
        )
        style.configure(
            "TEntry",
            fieldbackground=COLORS["entry_bg"],
            foreground=COLORS["entry_fg"],
            insertcolor=COLORS["fg"],
        )
        style.configure(
            "red.Horizontal.TProgressbar",
            troughcolor=COLORS["progress_bg"],
            background=COLORS["danger"],
        )
        style.configure(
            "yellow.Horizontal.TProgressbar",
            troughcolor=COLORS["progress_bg"],
            background=COLORS["warning"],
        )
        style.configure(
            "green.Horizontal.TProgressbar",
            troughcolor=COLORS["progress_bg"],
            background=COLORS["safe"],
        )

    # ------------------------------------------------------------------
    # Connection frame
    # ------------------------------------------------------------------
    def _build_connection_frame(self) -> None:
        frame = ttk.LabelFrame(
            self.root, text=" Connection Management ", padding=(12, 8)
        )
        frame.pack(fill="x", padx=12, pady=(12, 6))

        ttk.Label(frame, text="Robot Type:").pack(side="left", padx=(0, 6))

        self._robot_type_var = tk.StringVar(value=DEFAULT_ROBOT_TYPE)
        self._robot_type_entry = ttk.Entry(
            frame, textvariable=self._robot_type_var, width=12
        )
        self._robot_type_entry.pack(side="left", padx=(0, 12))

        self._connect_btn = ttk.Button(
            frame, text="Connect", command=self._connect_robot
        )
        self._connect_btn.pack(side="left", padx=(0, 8))

        self._disconnect_btn = ttk.Button(
            frame, text="Disconnect", command=self._disconnect_robot, state="disabled"
        )
        self._disconnect_btn.pack(side="left", padx=(0, 12))

        self._connection_status_var = tk.StringVar(value="Disconnected")
        status_lbl = tk.Label(
            frame,
            textvariable=self._connection_status_var,
            bg=COLORS["frame_bg"],
            fg=COLORS["danger"],
            font=("", 10, "bold"),
        )
        status_lbl.pack(side="left")
        self._connection_status_widget = status_lbl

    def _set_connection_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        self._connect_btn.configure(state=state)
        self._robot_type_entry.configure(state=state)

    # ------------------------------------------------------------------
    # Telemetry frame
    # ------------------------------------------------------------------
    def _build_telemetry_frame(self) -> None:
        frame = ttk.LabelFrame(
            self.root, text=" Live Position Monitor ", padding=(12, 8)
        )
        frame.pack(fill="both", expand=True, padx=12, pady=6)

        # Header row
        hdr_frame = tk.Frame(frame, bg=COLORS["frame_bg"])
        hdr_frame.pack(fill="x", pady=(0, 4))
        tk.Label(
            hdr_frame,
            text="Joint",
            bg=COLORS["frame_bg"],
            fg=COLORS["dim"],
            width=6,
            anchor="w",
            font=("", 9),
        ).pack(side="left", padx=(0, 8))
        tk.Label(
            hdr_frame,
            text="Position",
            bg=COLORS["frame_bg"],
            fg=COLORS["dim"],
            width=12,
            anchor="w",
            font=("", 9),
        ).pack(side="left", padx=(0, 12))
        tk.Label(
            hdr_frame,
            text="Range",
            bg=COLORS["frame_bg"],
            fg=COLORS["dim"],
            font=("", 9),
        ).pack(side="left", fill="x", expand=True, padx=(0, 12))
        tk.Label(
            hdr_frame,
            text="Status",
            bg=COLORS["frame_bg"],
            fg=COLORS["dim"],
            width=8,
            anchor="w",
            font=("", 9),
        ).pack(side="left")

        # Per-joint rows
        for i in range(JOINT_COUNT):
            self._build_joint_row(frame, i)

        # Refresh rate
        rate_frame = tk.Frame(frame, bg=COLORS["frame_bg"])
        rate_frame.pack(fill="x", pady=(8, 0))
        tk.Label(
            rate_frame,
            text="Refresh:",
            bg=COLORS["frame_bg"],
            fg=COLORS["dim"],
            font=("", 8),
        ).pack(side="left")
        self._refresh_var = tk.StringVar(value="-- Hz")
        tk.Label(
            rate_frame,
            textvariable=self._refresh_var,
            bg=COLORS["frame_bg"],
            fg=COLORS["dim"],
            font=("", 8),
        ).pack(side="left", padx=(4, 0))

    def _build_joint_row(self, parent: tk.Frame, index: int) -> None:
        joint_name = f"J{index + 1}"
        limits = JOINT_LIMITS[joint_name]
        row = tk.Frame(parent, bg=COLORS["frame_bg"])
        row.pack(fill="x", pady=2)

        # Joint name
        tk.Label(
            row,
            text=joint_name,
            bg=COLORS["frame_bg"],
            fg=COLORS["fg"],
            width=6,
            anchor="w",
            font=("", 11, "bold"),
        ).pack(side="left", padx=(0, 8))

        # Position value
        pos_var = tk.StringVar(value="---.--°")
        self._joint_labels.append(pos_var)
        tk.Label(
            row,
            textvariable=pos_var,
            bg=COLORS["frame_bg"],
            fg=COLORS["fg"],
            width=12,
            anchor="w",
            font=("Courier", 11),
        ).pack(side="left", padx=(0, 12))

        # Progress bar (visual range indicator)
        bar = ttk.Progressbar(
            row,
            mode="determinate",
            maximum=100,
            value=0,
            style="green.Horizontal.TProgressbar",
        )
        bar.pack(side="left", fill="x", expand=True, padx=(0, 12))
        self._joint_bars.append(bar)

        # Side labels for limits
        tk.Label(
            row,
            text=f"{limits[0]:+.0f}",
            bg=COLORS["frame_bg"],
            fg=COLORS["dim"],
            font=("", 7),
        ).place(x=310, y=2)  # type: ignore[call-arg]

        # Status indicator
        status_var = tk.StringVar(value="Safe")
        self._joint_status_labels.append(status_var)
        status_lbl = tk.Label(
            row,
            textvariable=status_var,
            bg=COLORS["frame_bg"],
            fg=COLORS["safe"],
            width=8,
            anchor="w",
            font=("", 10, "bold"),
        )
        status_lbl.pack(side="left")
        # Store for color updates
        if not hasattr(self, "_joint_status_widgets"):
            self._joint_status_widgets: list[tk.Label] = []
        self._joint_status_widgets.append(status_lbl)

    # ------------------------------------------------------------------
    # Calibration frame
    # ------------------------------------------------------------------
    def _build_calibration_frame(self) -> None:
        frame = ttk.LabelFrame(self.root, text=" Calibration ", padding=(12, 8))
        frame.pack(fill="x", padx=12, pady=6)

        self._home_btn = ttk.Button(
            frame, text="Home (All to 0°)", command=self._home_robot, state="disabled"
        )
        self._home_btn.pack(side="left", padx=(0, 8))

        self._zero_btn = ttk.Button(
            frame, text="Zero Joints", command=self._zero_joints, state="disabled"
        )
        self._zero_btn.pack(side="left")

    def _set_calibration_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        self._home_btn.configure(state=state)
        self._zero_btn.configure(state=state)

    # ------------------------------------------------------------------
    # Log console
    # ------------------------------------------------------------------
    def _build_log_frame(self) -> None:
        frame = ttk.LabelFrame(self.root, text=" Log ", padding=(12, 8))
        frame.pack(fill="both", expand=True, padx=12, pady=(6, 6))

        self._log_widget = scrolledtext.ScrolledText(
            frame,
            height=8,
            bg=COLORS["log_bg"],
            fg=COLORS["log_fg"],
            insertbackground=COLORS["fg"],
            font=("Courier", 9),
            relief="flat",
            borderwidth=0,
            highlightthickness=0,
            wrap="word",
        )
        self._log_widget.pack(fill="both", expand=True)
        self._log_widget.configure(state="disabled")

    def _log(self, msg: str, level: str = "info") -> None:
        """Append a timestamped message to the log console (thread-safe)."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        line = f"[{timestamp}] {msg}\n"

        def _write() -> None:
            self._log_widget.configure(state="normal")
            self._log_widget.insert("end", line)
            self._log_widget.see("end")
            self._log_widget.configure(state="disabled")

        self.root.after(0, _write)

    # ------------------------------------------------------------------
    # Status bar
    # ------------------------------------------------------------------
    def _build_status_bar(self) -> None:
        bar = tk.Frame(self.root, bg=COLORS["status_bar_bg"], height=24)
        bar.pack(fill="x", side="bottom")

        self._bar_conn_var = tk.StringVar(value="Disconnected")
        tk.Label(
            bar,
            textvariable=self._bar_conn_var,
            bg=COLORS["status_bar_bg"],
            fg="#ffffff",
            font=("", 9),
            padx=12,
        ).pack(side="left")

        self._bar_telem_var = tk.StringVar(value="Telemetry: Off")
        tk.Label(
            bar,
            textvariable=self._bar_telem_var,
            bg=COLORS["status_bar_bg"],
            fg="#ffffff",
            font=("", 9),
            padx=12,
        ).pack(side="left")

    # ------------------------------------------------------------------
    # Permissions
    # ------------------------------------------------------------------
    @staticmethod
    def _ensure_permissions() -> None:
        """Attempt to grant rw access to /dev/ttyACM* devices."""
        cmd = "sudo chmod 666 /dev/ttyACM* 2>/dev/null"
        try:
            subprocess.run(
                cmd,
                shell=True,
                timeout=5,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except subprocess.TimeoutExpired:
            pass
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Connect / Disconnect
    # ------------------------------------------------------------------
    def _connect_robot(self) -> None:
        """Initialize and connect to the ManipulatorRobot."""
        robot_type = self._robot_type_var.get().strip() or DEFAULT_ROBOT_TYPE
        self._log(f"Attempting connection to robot type '{robot_type}'...")
        self._log("Requesting device permissions...")

        self._ensure_permissions()

        try:
            with self._robot_lock:
                robot = ManipulatorRobot(robot_type=robot_type)
                robot.connect()
                self.robot = robot
        except Exception as exc:
            self._log(f"Connection failed: {exc}", "error")
            messagebox.showerror(
                "Connection Error",
                f"Failed to connect to robot '{robot_type}'.\n\n{exc}",
            )
            return

        self._log(f"Connected to '{robot_type}' successfully.", "success")
        self._connection_status_var.set("Connected")
        self._connection_status_widget.configure(fg=COLORS["safe"])
        self._bar_conn_var.set("Connected")
        self._connect_btn.configure(state="disabled")
        self._disconnect_btn.configure(state="normal")
        self._set_calibration_enabled(True)
        self._start_telemetry()

    def _disconnect_robot(self) -> None:
        """Disconnect from the robot and stop telemetry."""
        self._log("Disconnecting...")
        self._stop_telemetry()

        with self._robot_lock:
            robot = self.robot
            self.robot = None

        if robot is not None:
            try:
                robot.disconnect()
            except Exception as exc:
                self._log(f"Error during disconnect: {exc}", "warning")

        self._log("Disconnected.", "info")
        self._connection_status_var.set("Disconnected")
        self._connection_status_widget.configure(fg=COLORS["danger"])
        self._bar_conn_var.set("Disconnected")
        self._connect_btn.configure(state="normal")
        self._disconnect_btn.configure(state="disabled")
        self._set_calibration_enabled(False)

        for i in range(JOINT_COUNT):
            self._joint_labels[i].set("---.--°")

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------
    def _start_telemetry(self) -> None:
        if self._telemetry_thread is not None and self._telemetry_thread.is_alive():
            return
        self._telemetry_stop.clear()
        self._error_count = 0
        self._frame_times.clear()
        self._telemetry_thread = threading.Thread(
            target=self._telemetry_loop, daemon=True, name="telemetry"
        )
        self._telemetry_thread.start()
        self._bar_telem_var.set("Telemetry: Active")
        self._log(f"Telemetry started at {TELEMETRY_HZ} Hz.", "info")

    def _stop_telemetry(self) -> None:
        self._telemetry_stop.set()
        if self._telemetry_thread is not None:
            self._telemetry_thread.join(timeout=2.0)
            self._telemetry_thread = None
        self._bar_telem_var.set("Telemetry: Off")
        self._refresh_var.set("-- Hz")

    def _telemetry_loop(self) -> None:
        """Background thread: read robot state and queue UI updates."""
        period = 1.0 / TELEMETRY_HZ

        while not self._telemetry_stop.is_set():
            t_start = time.perf_counter()

            with self._robot_lock:
                robot = self.robot
                if robot is None:
                    break

                try:
                    observation, _ = robot.read_state()
                except Exception as exc:
                    self._error_count += 1
                    if self._error_count == 1:
                        self._log(f"read_state() error: {exc}", "error")
                    if self._error_count >= MAX_CONSECUTIVE_ERRORS:
                        self._log(
                            f"{MAX_CONSECUTIVE_ERRORS} consecutive read errors, "
                            "stopping telemetry.",
                            "error",
                        )
                        self.root.after(0, self._disconnect_robot)
                        return
                    # Sleep before retry
                    self._telemetry_stop.wait(period)
                    continue

            self._error_count = 0

            if isinstance(observation, dict):
                state = observation.get("state", observation)
            elif hasattr(observation, "state"):
                state = observation.state
            else:
                state = observation

            joint_values = self._extract_joint_values(state)
            self.root.after(0, self._update_telemetry_ui, joint_values)

            # Track actual frame rate
            elapsed = time.perf_counter() - t_start
            self._frame_times.append(elapsed)
            if len(self._frame_times) > 20:
                self._frame_times.pop(0)

            avg_hz = (
                len(self._frame_times) / sum(self._frame_times)
                if self._frame_times
                else 0
            )
            self.root.after(0, lambda hz=avg_hz: self._refresh_var.set(f"{hz:.1f} Hz"))

            # Sleep for remainder of period
            remain = period - elapsed
            if remain > 0:
                self._telemetry_stop.wait(remain)

    @staticmethod
    def _extract_joint_values(state: object) -> list[float]:
        """Pull 6 joint values from a state object (dict, list, tensor, or ndarray)."""
        values: list[float] = []

        if isinstance(state, dict):
            for i in range(JOINT_COUNT):
                key = f"J{i + 1}"
                val = state.get(key, state.get(f"joint_{i}", state.get(f"q_{i}", 0.0)))
                values.append(float(val))
        elif hasattr(state, "tolist"):
            raw = state.tolist()
            values = [float(v) for v in raw[:JOINT_COUNT]]
        elif isinstance(state, (list, tuple)):
            values = [float(v) for v in list(state)[:JOINT_COUNT]]
        else:
            values = [0.0] * JOINT_COUNT

        while len(values) < JOINT_COUNT:
            values.append(0.0)
        return values[:JOINT_COUNT]

    def _update_telemetry_ui(self, joint_values: list[float]) -> None:
        """Main-thread callback: update joint displays from telemetry data."""
        # Detect communication error (all zeros may indicate failure)
        all_zero = all(abs(v) < 0.001 for v in joint_values)

        for i, value in enumerate(joint_values):
            joint_name = f"J{i + 1}"
            limits = JOINT_LIMITS.get(joint_name, (-180.0, 180.0))
            lo, hi = limits

            self._joint_labels[i].set(_format_angle(value))

            # Map value to progress bar (0-100)
            span = hi - lo
            if span > 0:
                pct = (value - lo) / span * 100
                pct = max(0.0, min(100.0, pct))
            else:
                pct = 0.0
            self._joint_bars[i]["value"] = pct

            # Color-code bar
            if all_zero and not hasattr(self, "_joint_status_widgets"):
                color = COLORS["warning"]
                status = "NoData"
            else:
                color = _status_color(value, limits)
                status = _status_text(value, limits)

            self._joint_status_labels[i].set(status)
            self._joint_status_widgets[i].configure(fg=color)

            # Update bar style
            if color == COLORS["danger"]:
                style = "red.Horizontal.TProgressbar"
            elif color == COLORS["warning"]:
                style = "yellow.Horizontal.TProgressbar"
            else:
                style = "green.Horizontal.TProgressbar"
            self._joint_bars[i].configure(style=style)

    # ------------------------------------------------------------------
    # Calibration commands
    # ------------------------------------------------------------------
    def _home_robot(self) -> None:
        """Send the robot to the home position (all joints at 0)."""
        if not TORCH_AVAILABLE:
            self._log(
                "PyTorch required for sending actions. Install: pip install torch",
                "error",
            )
            return
        self._log("Sending home command (all joints → 0°)...")
        self._send_zero_action()

    def _zero_joints(self) -> None:
        """Zero / recalibrate the joint encoders."""
        self._log("Sending zero-joints command...")
        try:
            with self._robot_lock:
                if self.robot is not None and hasattr(self.robot, "reset"):
                    self.robot.reset()
                    self._log("Joint calibration (reset) complete.", "success")
                    return
        except Exception as exc:
            self._log(f"reset() failed: {exc}", "warning")

        # Fallback: send zero action
        self._send_zero_action()

    def _send_zero_action(self) -> None:
        """Send a zero-position action to the robot."""
        try:
            action = torch.zeros(JOINT_COUNT, dtype=torch.float32)
            with self._robot_lock:
                if self.robot is not None:
                    self.robot.send_action(action)
                    self._log("Zero action sent.", "success")
        except Exception as exc:
            self._log(f"Failed to send action: {exc}", "error")

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def _on_close(self) -> None:
        """Gracefully shut down telemetry and disconnect on window close."""
        self._log("Shutting down...")
        self._stop_telemetry()
        with self._robot_lock:
            robot = self.robot
            self.robot = None
        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass
        self.root.destroy()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> None:
    root = tk.Tk()
    SO101ControlApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
