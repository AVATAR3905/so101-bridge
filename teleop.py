"""
TeleopController — leader/follower teleoperation loop.
Reads leader joint states, applies to follower in a background thread,
and reports both states for telemetry and recording.
"""

import logging
import threading
import time

log = logging.getLogger("teleop")


class TeleopController:
    """Controls leader→follower teleoperation loop."""

    def __init__(self, leader, follower, config):
        self.leader = leader
        self.follower = follower
        self.cfg = config
        self._running = False
        self._thread: threading.Thread | None = None
        self._leader_state = None
        self._follower_state = None
        self._lock = threading.Lock()
        self._rate = getattr(config, 'TELEOP_RATE', 100) if config else 100

    def start(self):
        """Disable leader torque, begin control loop."""
        if hasattr(self.leader, 'set_torque'):
            self.leader.set_torque(False)
            log.info("[TELEOP] Leader torque disabled (free-move mode)")
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("[TELEOP] Started")

    def stop(self):
        """Stop the loop, re-enable leader torque."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if hasattr(self.leader, 'set_torque'):
            self.leader.set_torque(True)
            log.info("[TELEOP] Leader torque re-enabled")
        log.info("[TELEOP] Stopped")

    @property
    def active(self) -> bool:
        return self._running

    def get_states(self) -> dict:
        """Return latest leader and follower states."""
        with self._lock:
            return {
                "leader":   self._leader_state,
                "follower": self._follower_state,
                "active":   self._running,
            }

    def _loop(self):
        dt = 1.0 / self._rate
        while self._running:
            t0 = time.time()
            try:
                leader_state   = self.leader.get_state()
                follower_state = self.follower.get_state()

                joints  = leader_state.get("joints", [0.0] * 5)
                gripper = leader_state.get("gripper", 50.0)

                self.follower.set_joints(joints)
                self.follower.set_gripper(gripper)

                with self._lock:
                    self._leader_state   = leader_state
                    self._follower_state = follower_state
            except Exception as e:
                log.warning("Tick error: %s", e)

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
