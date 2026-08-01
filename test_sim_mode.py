"""
Test all features in simulation mode via WebSocket.
Usage: python test_sim_mode.py
"""

import asyncio
import json
import sys
import time
import traceback

import websockets

WS_URL = "ws://localhost:8765"

async def test():
    results = []

    async with websockets.connect(WS_URL) as ws:
        # hello
        raw = await asyncio.wait_for(ws.recv(), timeout=5.0)
        data = json.loads(raw)
        assert data["type"] == "hello"
        print("  [OK] Bridge connected")

        async def send(cmd, **kw):
            await ws.send(json.dumps({"cmd": cmd, **kw}))

        async def drain_until(type_filter, timeout=10.0):
            """Read and discard messages until we see type_filter, return it."""
            deadline = time.time() + timeout
            while time.time() < deadline:
                raw = await asyncio.wait_for(ws.recv(), max(0.5, deadline - time.time()))
                data = json.loads(raw)
                if data.get("type") == type_filter:
                    return data
            raise TimeoutError(f"Did not receive {type_filter} within {timeout}s")

        async def drain_all(count=10):
            """Drain up to `count` pending messages (stops early if buffer empties)."""
            for _ in range(count):
                try:
                    await asyncio.wait_for(ws.recv(), timeout=0.1)
                except (asyncio.TimeoutError, asyncio.CancelledError):
                    break

        async def expect_telemetry(timeout=3.0):
            """Get the next telemetry message (fresh)."""
            deadline = time.time() + timeout
            while time.time() < deadline:
                raw = await asyncio.wait_for(ws.recv(), max(0.5, deadline - time.time()))
                data = json.loads(raw)
                if data.get("type") == "telemetry":
                    return data
            raise TimeoutError("No telemetry within timeout")

        async def expect_type(type_filter, timeout=10.0):
            """Get the next message of a specific type (fresh, discarding others)."""
            deadline = time.time() + timeout
            while time.time() < deadline:
                raw = await asyncio.wait_for(ws.recv(), max(0.5, deadline - time.time()))
                data = json.loads(raw)
                if data.get("type") == type_filter:
                    return data
            raise TimeoutError(f"No {type_filter} within {timeout}s")

        # ── Connect arm ──
        print("\n" + "="*56 + "\nTEST: Connect arm\n" + "="*56)
        try:
            await send("connect", port="/dev/ttyACM1", baud=1000000)
            while True:
                raw = await asyncio.wait_for(ws.recv(), timeout=5.0)
                data = json.loads(raw)
                if data.get("type") == "status" and data.get("connected") is True:
                    print("  [PASS] Arm connected in sim mode")
                    results.append(("Connect arm", "PASS"))
                    break
                if data.get("type") == "status" and data.get("connected") is False:
                    raise AssertionError("Connect failed: " + str(data))
        except Exception as e:
            print(f"  [FAIL] {e}")
            results.append(("Connect arm", "FAIL"))

        # ── Set joints + telemetry ──
        print("\n" + "="*56 + "\nTEST: Set joints + telemetry\n" + "="*56)
        try:
            await drain_all()

            await send("set_joint", joint=0, angle=45.0)
            await asyncio.sleep(0.15)
            await send("set_joint", joint=1, angle=-30.0)
            await asyncio.sleep(0.15)
            await send("set_joint", joint=2, angle=60.0)
            await asyncio.sleep(0.3)

            await drain_all()  # clear stale telemetry queued during commands
            await asyncio.sleep(0.05)

            telem = await expect_telemetry()
            joints = telem.get("joints", [])
            assert len(joints) >= 3, f"Expected >=3 joints, got {len(joints)}"
            assert abs(joints[0] - 45.0) < 1.0, f"J0 expected ~45, got {joints[0]}"
            assert abs(joints[1] - (-30.0)) < 1.0, f"J1 expected ~-30, got {joints[1]}"
            assert abs(joints[2] - 60.0) < 1.0, f"J2 expected ~60, got {joints[2]}"
            print(f"  [PASS] Joint angles: {[f'{j:.1f}' for j in joints[:3]]}")
            results.append(("Set joints + telemetry", "PASS"))
        except Exception as e:
            print(f"  [FAIL] {e}")
            results.append(("Set joints + telemetry", "FAIL"))

        # ── Set gripper ──
        print("\n" + "="*56 + "\nTEST: Set gripper + telemetry\n" + "="*56)
        try:
            await drain_all()
            await send("set_gripper", value=75.0)
            await asyncio.sleep(0.3)
            await drain_all()  # clear stale telemetry queued during command
            await asyncio.sleep(0.05)
            telem = await expect_telemetry()
            grip = telem.get("gripper", 0)
            assert abs(grip - 75.0) < 2.0, f"Gripper expected ~75, got {grip}"
            print(f"  [PASS] Gripper: {grip}%")
            results.append(("Set gripper + telemetry", "PASS"))
        except Exception as e:
            print(f"  [FAIL] {e}")
            results.append(("Set gripper + telemetry", "FAIL"))

        # ── Disconnect arm ──
        print("\n" + "="*56 + "\nTEST: Disconnect arm\n" + "="*56)
        try:
            await drain_all()
            await send("disconnect")
            msg = await expect_type("status")
            assert msg.get("connected") is False
            print("  [PASS] Arm disconnected")
            results.append(("Disconnect arm", "PASS"))
        except Exception as e:
            print(f"  [FAIL] {e}")
            results.append(("Disconnect arm", "FAIL"))

        # ── Calibration graceful failure ──
        print("\n" + "="*56 + "\nTEST: Calibration graceful failure\n" + "="*56)
        try:
            await send("auto_calibrate", port="/dev/ttyACM1", role="follower")
            msg = await expect_type("cal_done", timeout=15.0)
            assert msg.get("aborted") is True
            err = msg.get("error", "E-stop")
            print(f"  [PASS] Calibration gracefully failed: {err}")
            results.append(("Calibration graceful failure", "PASS"))
        except Exception as e:
            print(f"  [FAIL] {e}")
            results.append(("Calibration graceful failure", "FAIL"))

        # ── Teleop start/stop ──
        print("\n" + "="*56 + "\nTEST: Teleop start/stop\n" + "="*56)
        try:
            await send("start_teleop", leader_port="sim", follower_port="sim")
            msg = await expect_type("status")
            assert msg.get("teleop") is True
            print("  [PASS] Teleop started with sim arms")
            await asyncio.sleep(0.5)
            await send("stop_teleop")
            msg = await expect_type("status")
            assert msg.get("teleop") is False
            print("  [PASS] Teleop stopped")
            results.append(("Teleop start/stop", "PASS"))
        except Exception as e:
            print(f"  [FAIL] {e}")
            results.append(("Teleop start/stop", "FAIL"))

        # ── Teleop leader→follower ──
        print("\n" + "="*56 + "\nTEST: Teleop leader->follower\n" + "="*56)
        try:
            await send("start_teleop", leader_port="sim", follower_port="sim")
            await expect_type("status")
            await asyncio.sleep(0.3)

            await drain_all()

            await send("set_joint", joint=0, angle=30.0)
            await asyncio.sleep(0.15)
            await send("set_joint", joint=1, angle=-20.0)
            await asyncio.sleep(0.15)
            await send("set_gripper", value=80.0)
            await asyncio.sleep(0.3)

            await drain_all()  # clear stale telemetry queued during commands
            await asyncio.sleep(0.05)

            telem = await expect_telemetry()
            teleop_data = telem.get("teleop", {})
            assert teleop_data.get("active") is True, "Teleop should be active"

            leader = teleop_data.get("leader", {}) or {}
            follower = teleop_data.get("follower", {}) or {}
            lj = leader.get("joints", [])
            fj = follower.get("joints", [])

            print(f"  Leader joints: {[f'{j:.1f}' for j in lj[:3]]}")
            print(f"  Follower joints: {[f'{j:.1f}' for j in fj[:3]]}")

            if lj and fj:
                diffs = [abs(lj[i] - fj[i]) for i in range(min(3, len(lj), len(fj)))]
                assert max(diffs) < 5.0, f"Follower should track leader, diffs={diffs}"
                print(f"  [PASS] Follower tracks leader (max diff={max(diffs):.1f})")

            # Top-level joints match follower
            top_joints = telem.get("joints", [])
            if top_joints and fj:
                diffs2 = [abs(top_joints[i] - fj[i]) for i in range(min(3, len(top_joints), len(fj)))]
                assert max(diffs2) < 2.0, "Top-level joints should match follower"
                print("  [PASS] Top-level joints match follower")

            # Check gripper propagation
            lg = leader.get("gripper")
            fg = follower.get("gripper")
            print(f"  Leader gripper: {lg}, Follower gripper: {fg}")
            if lg is not None and fg is not None:
                assert abs(lg - fg) < 5.0, f"Gripper should match: leader={lg} follower={fg}"
                assert abs(lg - 80.0) < 5.0, f"Leader gripper should be ~80, got {lg}"

            results.append(("Teleop leader->follower", "PASS"))
            await send("stop_teleop")
            await expect_type("status")
        except Exception as e:
            print(f"  [FAIL] {e}")
            traceback.print_exc()
            results.append(("Teleop leader->follower", "FAIL"))
            try:
                await send("stop_teleop")
                await expect_type("status")
            except Exception:
                pass

        # ── Recording during teleop ──
        print("\n" + "="*56 + "\nTEST: Recording during teleop\n" + "="*56)
        try:
            await send("start_teleop", leader_port="sim", follower_port="sim")
            await expect_type("status")
            await asyncio.sleep(0.2)
            await drain_all()

            await send("start_recording", name="test_episode_001", task="sim_test", fps=10, devices=[])
            msg = await expect_type("recording_started")
            print(f"  Recording started: {msg.get('name')}")

            # Move during recording
            for i in range(3):
                await send("set_joint", joint=0, angle=10.0 * (i + 1))
                await asyncio.sleep(0.2)
                await send("set_joint", joint=1, angle=-5.0 * (i + 1))
                await asyncio.sleep(0.2)

            await asyncio.sleep(1.0)

            await send("stop_recording")
            msg = await expect_type("recording_stopped")
            info = msg.get("info", {})
            frames = info.get("frames", 0)
            print(f"  Recording stopped: {frames} frames, name={info.get('name')}")
            assert frames > 0, f"Expected >0 frames, got {frames}"

            await send("list_episodes")
            ep_msg = await expect_type("episodes")
            names = [e["name"] for e in ep_msg.get("list", [])]
            assert "test_episode_001" in names, f"Episode not in list: {names}"
            print(f"  [PASS] Episode recorded with {frames} frames")

            results.append(("Recording during teleop", "PASS"))
            await send("stop_teleop")
            await expect_type("status")
        except Exception as e:
            print(f"  [FAIL] {e}")
            traceback.print_exc()
            results.append(("Recording during teleop", "FAIL"))
            try:
                await send("stop_teleop")
                await expect_type("status")
            except Exception:
                pass

        # ── Replay episode ──
        print("\n" + "="*56 + "\nTEST: Replay episode\n" + "="*56)
        try:
            await send("connect", port="/dev/ttyACM1", baud=1000000)
            await expect_type("status")

            await send("replay_episode", name="test_episode_001", speed=2.0)
            msg = await expect_type("replay_started")
            print(f"  Replay started: {msg.get('frames')} frames")

            msg = await expect_type("replay_done", timeout=30.0)
            print(f"  [PASS] Replay done: {msg.get('name')}")

            results.append(("Replay episode", "PASS"))
            await send("disconnect")
            await expect_type("status")
        except Exception as e:
            print(f"  [FAIL] {e}")
            results.append(("Replay episode", "FAIL"))
            try:
                await send("disconnect")
                await expect_type("status")
            except Exception:
                pass

        # ── Both calibration graceful failure ──
        print("\n" + "="*56 + "\nTEST: Both calibration graceful failure\n" + "="*56)
        try:
            await send("auto_calibrate", role="both")
            msg = await expect_type("cal_done", timeout=15.0)
            assert msg.get("aborted") is True
            print(f"  [PASS] Both calibration gracefully failed: {msg.get('error', 'E-stop')}")
            results.append(("Both calibration graceful failure", "PASS"))
        except Exception as e:
            print(f"  [FAIL] {e}")
            results.append(("Both calibration graceful failure", "FAIL"))

    # ── Summary ──
    print(f"\n{'='*56}")
    passed = sum(1 for _, s in results if s == "PASS")
    failed = sum(1 for _, s in results if s == "FAIL")
    print(f"RESULTS: {passed} passed, {failed} failed out of {len(results)}")
    for name, status in results:
        print(f"  {status}: {name}")

    return failed == 0


async def main():
    for attempt in range(15):
        try:
            ws = await asyncio.wait_for(websockets.connect(WS_URL), timeout=2.0)
            await ws.close()
            break
        except Exception as e:
            if attempt == 14:
                print(f"ERROR: Bridge not reachable after 15 attempts: {e}")
                sys.exit(1)
            print(f"Waiting for bridge (attempt {attempt+1}/15)...")
            await asyncio.sleep(1)

    success = await test()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    asyncio.run(main())
