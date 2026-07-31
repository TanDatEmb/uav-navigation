#!/usr/bin/env python3

import subprocess
import tempfile
import time
import unittest
from unittest import mock
from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ros_replay import (
    acceptance_failures,
    diagnostics_values,
    drained,
    process_group_exists,
    register_process,
    rviz_command,
    stop,
    track_process,
    parser,
)
import cleanup_replay


class DrainTest(unittest.TestCase):
    def test_default_replay_does_not_enable_rviz(self) -> None:
        args = parser().parse_args([
            "run", "--bag", "/tmp/bag", "--config", "/tmp/config",
            "--output", "/tmp/output", "--imu-topic", "/imu",
            "--lidar-topic", "/lidar",
        ])
        self.assertFalse(args.enable_rviz)
        self.assertIsNone(rviz_command(args))

    def test_enable_rviz_builds_canonical_command(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            config = Path(temporary) / "fast_lio.rviz"
            config.touch()
            args = SimpleNamespace(enable_rviz=True, rviz_config=config)
            with mock.patch("ros_replay.shutil.which", return_value="/usr/bin/rviz2"):
                self.assertEqual(
                    rviz_command(args, {"DISPLAY": ":0"}),
                    ["/usr/bin/rviz2", "-d", str(config)],
                )

    def test_missing_rviz2_is_rejected(self) -> None:
        args = SimpleNamespace(enable_rviz=True, rviz_config=Path("/tmp/unused"))
        with mock.patch("ros_replay.shutil.which", return_value=None):
            with self.assertRaisesRegex(RuntimeError, "rviz2"):
                rviz_command(args, {"DISPLAY": ":0"})

    def test_missing_rviz_config_is_rejected(self) -> None:
        args = SimpleNamespace(enable_rviz=True, rviz_config=Path("/missing/config"))
        with mock.patch("ros_replay.shutil.which", return_value="/usr/bin/rviz2"):
            with self.assertRaisesRegex(RuntimeError, "config does not exist"):
                rviz_command(args, {"DISPLAY": ":0"})

    def test_headless_rviz_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            config = Path(temporary) / "fast_lio.rviz"
            config.touch()
            args = SimpleNamespace(enable_rviz=True, rviz_config=config)
            with mock.patch("ros_replay.shutil.which", return_value="/usr/bin/rviz2"):
                with self.assertRaisesRegex(RuntimeError, "headless"):
                    rviz_command(args, {})

    def test_rviz_is_tracked_for_process_cleanup(self) -> None:
        process = SimpleNamespace()
        processes = []
        with mock.patch("ros_replay.register_process") as register:
            track_process(processes, Path("registry.json"), process, "rviz", ["rviz2"])
        self.assertEqual(processes, [process])
        register.assert_called_once_with(
            Path("registry.json"), process, "rviz", ["rviz2"]
        )

    def test_parses_jazzy_byte_encoded_diagnostic_level(self) -> None:
        message = SimpleNamespace(status=[
            SimpleNamespace(
                name="fast_lio/transport",
                level=b"\x01",
                message="lag",
                values=[SimpleNamespace(key="imu_drop_count", value="0")],
            )
        ])
        self.assertEqual(
            diagnostics_values(message),
            {"level": 1, "message": "lag", "imu_drop_count": 0},
        )

    def test_requires_zero_queues_and_converged_counters(self) -> None:
        state = {
            "current_input_queue_depth": 0,
            "current_imu_queue_depth": 0,
            "current_lidar_queue_depth": 0,
            "received_imu_count": 10,
            "processed_imu_count": 10,
            "received_lidar_count": 2,
            "processed_lidar_count": 2,
        }
        self.assertTrue(drained(state))
        state["processed_lidar_count"] = 1
        self.assertFalse(drained(state))

    def test_overflow_and_drop_fail_acceptance(self) -> None:
        state = {
            "current_input_queue_depth": 0,
            "current_imu_queue_depth": 0,
            "current_lidar_queue_depth": 0,
            "received_imu_count": 10,
            "processed_imu_count": 10,
            "received_lidar_count": 2,
            "processed_lidar_count": 2,
            "overflow_detected": True,
            "imu_drop_count": 1,
            "lidar_drop_count": 0,
        }
        self.assertEqual(
            acceptance_failures(state),
            ["input queue overflow detected", "IMU messages dropped"],
        )

    def test_processing_lag_limit_fails_acceptance(self) -> None:
        state = {
            "current_input_queue_depth": 0,
            "current_imu_queue_depth": 0,
            "current_lidar_queue_depth": 0,
            "received_imu_count": 1,
            "processed_imu_count": 1,
            "received_lidar_count": 1,
            "processed_lidar_count": 1,
            "processing_lag_exceeded": True,
        }
        self.assertEqual(
            acceptance_failures(state), ["maximum processing lag exceeded"]
        )

    def test_stop_kills_group_after_process_leader_has_exited(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            child_pid_path = Path(temporary) / "child.pid"
            leader = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    (
                        "import os, pathlib, sys, time; "
                        "pid=os.fork(); "
                        f"path=pathlib.Path({str(child_pid_path)!r}); "
                        "path.write_text(str(pid)) if pid else None; "
                        "sys.exit(0) if pid else time.sleep(60)"
                    ),
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            leader.wait(timeout=5)
            deadline = time.monotonic() + 5
            while not child_pid_path.is_file() and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(child_pid_path.is_file())
            self.assertTrue(process_group_exists(leader.pid))

            stop(leader, timeout=1)

            self.assertFalse(process_group_exists(leader.pid))

    def test_registered_cleanup_stops_only_manifest_process_group(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            artifact_root = Path(temporary) / "datasets"
            run = artifact_root / "fixture" / "abc-replay-1.0x-test"
            run.mkdir(parents=True)
            registry = run / "process_groups.json"
            process = subprocess.Popen(
                [sys.executable, "-c", "import time; time.sleep(60)"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            try:
                register_process(
                    registry, process, "fixture",
                    [sys.executable, "-c", "import time; time.sleep(60)"],
                )
                previous_artifacts = cleanup_replay.ARTIFACTS
                cleanup_replay.ARTIFACTS = artifact_root
                try:
                    self.assertEqual(cleanup_replay.cleanup(dry_run=True), 0)
                    self.assertTrue(process_group_exists(process.pid))
                    self.assertEqual(cleanup_replay.cleanup(), 0)
                    process.wait(timeout=2)
                finally:
                    cleanup_replay.ARTIFACTS = previous_artifacts
                self.assertFalse(process_group_exists(process.pid))
            finally:
                if process_group_exists(process.pid):
                    stop(process, timeout=1)


if __name__ == "__main__":
    unittest.main()
