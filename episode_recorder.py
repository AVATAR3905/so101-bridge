"""
EpisodeRecorder — records joint states + camera frames into a LeRobot-compatible
dataset on disk. Each episode is saved as:

  datasets/so101/<episode_name>/
    episode.json        — metadata + joint frames
    frames/
      cam_<dev>_<idx>.jpg   — camera images
"""

import json
import logging
import os
import threading
import time
from pathlib import Path

log = logging.getLogger("recorder")

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


class EpisodeRecorder:

    def __init__(self, name: str, task: str, fps: int,
                 output_dir: str, camera_manager, arm,
                 teleop_controller=None):
        self.name = name
        self.task = task
        self.fps = fps
        self.output_dir = Path(output_dir).expanduser()
        self.cams = camera_manager
        self.arm = arm
        self.teleop = teleop_controller

        self._ep_dir = self.output_dir / name
        self._frames_dir = self._ep_dir / "frames"
        self._frames: list[dict] = []
        self._thread: threading.Thread | None = None
        self._running = False
        self._t_start: float = 0.0
        self._lock = threading.Lock()
        self._discard = False

    # ── Public API ────────────────────────────────────────────────────────

    def start(self):
        self._ep_dir.mkdir(parents=True, exist_ok=True)
        self._frames_dir.mkdir(exist_ok=True)
        self._frames = []
        self._running = True
        self._discard = False
        self._t_start = time.time()
        self._thread = threading.Thread(target=self._record_loop, daemon=True)
        self._thread.start()
        log.info("Recording started: %s  (fps=%d)", self.name, self.fps)

    def stop(self) -> dict:
        self._running = False
        if self._thread:
            self._thread.join(timeout=5.0)
        if self._discard:
            self._cleanup()
            return {"discarded": True}
        info = self._save_metadata()
        log.info("Recording saved: %s  (%d frames)", self.name, len(self._frames))
        return info

    def discard(self):
        self._discard = True
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        self._cleanup()
        log.info("Episode discarded: %s", self.name)

    @property
    def active(self) -> bool:
        return self._running

    @property
    def frame_count(self) -> int:
        with self._lock:
            return len(self._frames)

    def elapsed(self) -> float:
        return round(time.time() - self._t_start, 2) if self._t_start else 0.0

    # ── Record loop ────────────────────────────────────────────────────────

    def _record_loop(self):
        dt = 1.0 / self.fps
        idx = 0
        while self._running:
            t0 = time.time()

            # Capture arm state (use teleop follower if active, else the arm object)
            arm_state = self.arm.get_state() if self.arm else {}
            leader_joints = None
            leader_gripper = None
            if self.teleop and self.teleop.active:
                ts = self.teleop.get_states()
                if ts.get("follower"):
                    arm_state = ts["follower"]
                if ts.get("leader"):
                    leader_joints = ts["leader"].get("joints")
                    leader_gripper = ts["leader"].get("gripper")
            joints = arm_state.get("joints", [0.0] * 5)
            gripper = arm_state.get("gripper", 50.0)
            timestamp = self.elapsed()

            # Capture camera frames
            cam_files: dict[str, str] = {}
            if self.cams:
                for dev in self.cams.devices:
                    frame = self.cams.get_frame(dev)
                    if frame is not None and CV2_AVAILABLE:
                        fname = f"cam_{dev}_{idx:06d}.jpg"
                        fpath = self._frames_dir / fname
                        cv2.imwrite(str(fpath), frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        cam_files[f"cam_{dev}"] = str(fpath.relative_to(self._ep_dir))

            with self._lock:
                frame = {
                    "idx": idx,
                    "timestamp": timestamp,
                    "joints": joints,
                    "gripper": gripper,
                    "cameras": cam_files,
                }
                if leader_joints is not None:
                    frame["leader_joints"]  = leader_joints
                    frame["leader_gripper"] = leader_gripper
                self._frames.append(frame)

            idx += 1
            elapsed = time.time() - t0
            sleep = dt - elapsed
            if sleep > 0:
                time.sleep(sleep)

    # ── Persistence ────────────────────────────────────────────────────────

    def _save_metadata(self) -> dict:
        with self._lock:
            frames = list(self._frames)

        duration = frames[-1]["timestamp"] if frames else 0.0
        size_bytes = sum(
            os.path.getsize(self._frames_dir / Path(f).name)
            for frame in frames
            for f in frame["cameras"].values()
            if (self._frames_dir / Path(f).name).exists()
        )

        meta = {
            "name": self.name,
            "task": self.task,
            "fps": self.fps,
            "frame_count": len(frames),
            "duration_s": duration,
            "size_bytes": size_bytes,
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "format": "lerobot_v2",
            "frames": frames,
        }

        with open(self._ep_dir / "episode.json", "w") as f:
            json.dump(meta, f, indent=2)

        # Also export a flat CSV for quick inspection
        self._save_csv(frames)

        return {
            "name": self.name,
            "frames": len(frames),
            "duration_s": duration,
            "size_mb": round(size_bytes / 1e6, 2),
            "path": str(self._ep_dir),
        }

    def _save_csv(self, frames: list[dict]):
        path = self._ep_dir / "joints.csv"
        with open(path, "w") as f:
            has_leader = any("leader_joints" in fr for fr in frames)
            header = "idx,timestamp,j1,j2,j3,j4,j5,gripper"
            if has_leader:
                header += ",l_j1,l_j2,l_j3,l_j4,l_j5,l_gripper"
            header += "\n"
            f.write(header)
            for fr in frames:
                j = fr["joints"] + [0.0] * (5 - len(fr["joints"]))
                row = f"{fr['idx']},{fr['timestamp']:.4f}," \
                      f"{j[0]:.3f},{j[1]:.3f},{j[2]:.3f},{j[3]:.3f},{j[4]:.3f}," \
                      f"{fr['gripper']:.1f}"
                if has_leader:
                    lj = fr.get("leader_joints", [0.0] * 5) + [0.0] * (5 - len(fr.get("leader_joints", [])))
                    lg = fr.get("leader_gripper", 50.0)
                    row += f",{lj[0]:.3f},{lj[1]:.3f},{lj[2]:.3f},{lj[3]:.3f},{lj[4]:.3f},{lg:.1f}"
                row += "\n"
                f.write(row)

    def _cleanup(self):
        import shutil
        try:
            shutil.rmtree(self._ep_dir, ignore_errors=True)
        except Exception as e:
            log.warning("Cleanup failed: %s", e)
