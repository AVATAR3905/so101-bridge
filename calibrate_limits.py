#!/usr/bin/env python3
"""
Auto-calibration script for SO-101 arm.
Sweeps each joint to find mechanical limits and gripper range,
then saves results to config.json.

Usage:
  # Stop the bridge first, then:
  python calibrate_limits.py

  # With custom port:
  python calibrate_limits.py --port /dev/ttyACM0
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).parent
CONFIG_PATH = HERE / "config.json"

try:
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorNormMode
except ImportError:
    print("ERROR: lerobot not available. Activate your venv first.")
    sys.exit(1)


# ── Helpers ────────────────────────────────────────────────────────────────

TICKS_CENTER = 2048
TICKS_PER_DEG = 4096 / 360.0


def ticks_to_deg(t: int) -> float:
    return round((t - TICKS_CENTER) / TICKS_PER_DEG, 1)


def deg_to_ticks(d: float) -> int:
    return int(TICKS_CENTER + d * TICKS_PER_DEG)


def load_config() -> dict:
    if CONFIG_PATH.exists():
        return json.loads(CONFIG_PATH.read_text())
    return {}


def save_config(cfg: dict):
    CONFIG_PATH.write_text(json.dumps(cfg, indent=2) + "\n")
    print(f"\n  ✓ Saved to {CONFIG_PATH}")


def build_bus(port: str):
    norm = MotorNormMode.RANGE_M100_100
    motors = {f"j{i}": Motor(i + 1, "sts3215", norm) for i in range(6)}
    bus = FeetechMotorsBus(port=port, motors=motors)
    bus.normalized_data = []
    bus.connect()
    return bus


# ── Limit finding ──────────────────────────────────────────────────────────


def _release_torque(bus, jid: int):
    """Disable torque on a specific joint so it stops pushing."""
    try:
        bus.write("Torque_Enable", f"j{ jid }", 0, normalize=False)
        print(f"      → Torque released on j{ jid } (shoulder lift)")
    except Exception:
        pass


def find_limit(bus, jid: int, home_ticks: int, step: int, label: str,
               stop_event: threading.Event | None = None) -> int:
    """Step `jid` from home in `step`-tick increments until the servo stops moving."""
    prev = None
    stuck = 0
    limit = home_ticks
    progress_window = []
    MIN_PROGRESS = 8  # minimum ticks of movement over 3 steps to not be stalled
    for s in range(40):
        if stop_event and stop_event.is_set():
            print(f"      ← {label} limit ABORTED")
            return limit
        goal = home_ticks + step * (s + 1)
        goal = max(50, min(4040, goal))
        try:
            bus.write("Goal_Position", f"j{ jid }", goal, normalize=False)
        except Exception as e:
            print(f"      write error at step {s+1}: {e} — treating as limit")
            return limit
        time.sleep(1.2)
        if stop_event and stop_event.is_set():
            return limit
        try:
            pos = bus.read("Present_Position", f"j{ jid }")
        except Exception as e:
            print(f"      read error at step {s+1}: {e} — treating as limit")
            return limit
        deg = ticks_to_deg(pos)

        # Track recent movement for stall detection
        progress_window.append(pos)
        if len(progress_window) > 3:
            progress_window.pop(0)

        # Check consecutive micro-moves (near-instant stall)
        if prev is not None and abs(pos - prev) < 5:
            stuck += 1
            if stuck >= 3:
                print(f"    ← {label} limit (stalled {stuck}x): {pos} ticks ({deg}°)")
                if jid == 1:
                    _release_torque(bus, jid)
                return pos
        else:
            stuck = 0

        # Check progressive stall — barely moved over last 3 steps
        if (len(progress_window) == 3 and step != 0
                and abs(progress_window[-1] - progress_window[0]) < MIN_PROGRESS
                and s >= 3):
            total = abs(progress_window[-1] - progress_window[0])
            print(f"    ← {label} limit (progress {total} ticks over 3 steps): {pos} ticks ({deg}°)")
            if jid == 1:
                _release_torque(bus, jid)
            return pos

        prev = pos
        limit = pos
        if s < 3 or s % 10 == 0:
            print(f"      step {s+1:2d}: goal={goal:5d} → {pos:5d} ({deg:6.1f}°)")
    return limit


def find_gripper_limits(bus, stop_event: threading.Event | None = None) -> tuple[int, int]:
    """Sweep gripper to find open and close mechanical stops."""
    print("\n  ── Gripper ──")
    # Open extreme
    try:
        bus.write("Goal_Position", "j5", 500, normalize=False)
    except Exception as e:
        print(f"    gripper open write error: {e}")
        return 0, 0
    time.sleep(1.5)
    if stop_event and stop_event.is_set():
        return 0, 0
    try:
        open_t = bus.read("Present_Position", "j5")
    except Exception as e:
        print(f"    gripper open read error: {e}")
        return 0, 0
    print(f"    Open goal= 500 → {open_t} ticks ({ticks_to_deg(open_t)}°)")
    # Close extreme
    try:
        bus.write("Goal_Position", "j5", 3500, normalize=False)
    except Exception as e:
        print(f"    gripper close write error: {e}")
        return 0, 0
    time.sleep(1.5)
    if stop_event and stop_event.is_set():
        return 0, 0
    try:
        close_t = bus.read("Present_Position", "j5")
    except Exception as e:
        print(f"    gripper close read error: {e}")
        return 0, 0
    print(f"    Close goal=3500 → {close_t} ticks ({ticks_to_deg(close_t)}°)")
    # Re-center
    mid = (open_t + close_t) // 2 if open_t and close_t else 2048
    try:
        bus.write("Goal_Position", "j5", mid, normalize=False)
    except Exception:
        pass
    time.sleep(0.5)
    return int(open_t), int(close_t)


def set_joint_speed(bus, speed: int, joint_names: list[str] | None = None):
    """Set Moving_Speed (addr 24, 2 bytes) for all or specified joints."""
    names = joint_names or [f"j{i}" for i in range(5)]
    for name in names:
        try:
            bus.write("Moving_Speed", name, speed, normalize=False)
        except Exception:
            pass  # some motors may not support this register


def go_home(bus, home_ticks: list[int], stop_event: threading.Event | None = None):
    STEPS = 6
    # Enable torque on all joints first
    for i in range(5):
        try:
            bus.write("Torque_Enable", f"j{i}", 1, normalize=False)
        except Exception:
            pass
    # Read current positions (handle read failures gracefully)
    cur = []
    for i in range(5):
        try:
            cur.append(bus.read("Present_Position", f"j{i}"))
        except Exception:
            cur.append(home_ticks[i])  # fallback to home position
    # Move each joint gradually
    for step in range(1, STEPS + 1):
        if stop_event and stop_event.is_set():
            return
        for i in range(5):
            t = step / STEPS
            pos = int(cur[i] + (home_ticks[i] - cur[i]) * t)
            try:
                bus.write("Goal_Position", f"j{i}", pos, normalize=False)
            except Exception:
                pass  # skip joints that fail to respond
        time.sleep(0.6)
    time.sleep(1.5)


# ── Main ───────────────────────────────────────────────────────────────────


def main():
    ap = argparse.ArgumentParser(description="Auto-calibrate SO-101 arm joint limits")
    ap.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    ap.add_argument("--baud", type=int, default=1000000, help="Baud rate")
    ap.add_argument("-y", "--yes", action="store_true", help="Skip confirmation prompt")
    ap.add_argument("--only-joint", type=int, default=None, choices=range(6),
                    help="Only calibrate this one joint, skip the rest")
    args = ap.parse_args()

    cfg = load_config()
    home_angles = cfg.get("HOME_ANGLES", [0.0, -34.4, -45.8, -22.9, 0.0])
    home_ticks = [deg_to_ticks(a) for a in home_angles]

    print("=" * 56)
    print("  SO-101 Joint Limit Auto-Calibration")
    print("=" * 56)
    print(f"\n  Port:  {args.port} @ {args.baud}")
    print(f"  Home:  {home_angles}")
    print("\n  Make sure the arm has clearance to move in all directions.")
    print("  Press Ctrl+C to abort at any time.\n")

    if not args.yes:
        try:
            input("  Press Enter to start calibration… ")
        except KeyboardInterrupt:
            print("\n  Aborted.")
            return
    else:
        print("  Starting calibration…")

    print("\n  Connecting…")
    try:
        bus = build_bus(args.port)
    except Exception as e:
        print(f"  ERROR: {e}")
        sys.exit(1)

    print("  Moving to home position…")
    go_home(bus, home_ticks)

    joint_info = [
        (0, "Shoulder pan", home_ticks[0]),
        (1, "Shoulder lift", home_ticks[1]),
        (2, "Elbow", home_ticks[2]),
        (3, "Wrist pitch", home_ticks[3]),
        (4, "Wrist roll", home_ticks[4]),
    ]

    results = {}
    limits = list(cfg.get("JOINT_LIMITS", [[0,0]]*5)) if args.only_joint is not None else []

    for jid, jname, home in joint_info:
        if args.only_joint is not None and jid != args.only_joint:
            continue
        print(f"\n  ── {jname} (j{jid}) ──")
        go_home(bus, home_ticks)
        lo = find_limit(bus, jid, home, -60, "min")
        hi = find_limit(bus, jid, home, 60, "max")
        lo_deg = ticks_to_deg(lo)
        hi_deg = ticks_to_deg(hi)
        if args.only_joint is None:
            limits.append([lo_deg, hi_deg])
        else:
            limits[jid] = [lo_deg, hi_deg]
        print(f"    → Range: [{lo_deg}°, {hi_deg}°]  ticks=[{lo}, {hi}]")
        results[jname] = {"min_deg": lo_deg, "max_deg": hi_deg, "min_ticks": lo, "max_ticks": hi}
        if jid == 1:
            print("    → Leaving shoulder lift at max position")
            bus.write("Goal_Position", "j1", hi, normalize=False)
            time.sleep(1.5)
    if args.only_joint is None:
        go_home(bus, home_ticks)

    gripper_open, gripper_closed = find_gripper_limits(bus)
    print(f"    → Gripper: open={gripper_open} ticks, close={gripper_closed} ticks")

    bus.disconnect()

    # Update config
    cfg["JOINT_LIMITS"] = limits
    cfg["GRIPPER_OPEN"] = float(gripper_open)
    cfg["GRIPPER_CLOSED"] = float(gripper_closed)
    cfg["HOME_ANGLES"] = home_angles
    save_config(cfg)

    print("\n" + "=" * 56)
    print("  Calibration complete!")
    print("=" * 56)
    for jname, r in results.items():
        print(f"  {jname:15s}  [{r['min_deg']:4.0f}°, {r['max_deg']:4.0f}°]")
    print(f"  {'Gripper':15s}  open={gripper_open}  close={gripper_closed}")
    print("\n  Restart the bridge, then use the GUI.")


if __name__ == "__main__":
    main()
