import unittest
import json
from pathlib import Path
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sitl_preflight import (
    HostTelemetryRecorder,
    PreflightViolation,
    check_preflight,
    competing_processes,
    parse_process_table,
)


class SitlPreflightTests(unittest.TestCase):
    def test_rejects_competing_build_processes_but_ignores_runner_ancestors(self) -> None:
        text = """
            101 make 0.1 make run
            102 colcon 2.0 colcon build --parallel-workers 2
            103 python3 1.0 python3 tools/runtime/runner.py external-mode-check
        """
        rows = competing_processes(
            text,
            excluded_pids={101, 103},
        )

        violation = check_preflight(
            loadavg_1m=0.2,
            max_loadavg_1m=4.0,
            process_rows=rows,
            excluded_pids={101, 103},
            active_runtime_sessions=[],
        )

        self.assertIsInstance(violation, PreflightViolation)
        self.assertEqual([row.pid for row in violation.process_rows], [102])

    def test_rejects_high_load_and_another_runtime_with_configured_threshold(self) -> None:
        violation = check_preflight(
            loadavg_1m=4.01,
            max_loadavg_1m=4.0,
            process_rows=[],
            excluded_pids=set(),
            active_runtime_sessions=["external-mode-123 (roles: px4_gazebo)"],
        )

        self.assertIsInstance(violation, PreflightViolation)
        self.assertIn("loadavg_1m=4.01", str(violation))
        self.assertIn("external-mode-123", str(violation))

    def test_accepts_idle_host_and_no_runtime(self) -> None:
        self.assertIsNone(
            check_preflight(
                loadavg_1m=0.2,
                max_loadavg_1m=4.0,
                process_rows=[],
                excluded_pids=set(),
                active_runtime_sessions=[],
            )
        )

    def test_recorder_appends_host_and_gazebo_factor_to_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            metadata_path = root / "metadata.json"
            rtf_path = root / "gazebo_native_rtf.json"
            metadata_path.write_text(
                json.dumps({"schema_version": 1}) + "\n", encoding="utf-8"
            )
            rtf_path.write_text(
                json.dumps({"real_time_factor": 0.97}) + "\n", encoding="utf-8"
            )
            recorder = HostTelemetryRecorder(metadata_path, rtf_path, interval_s=10.0)
            recorder.start()
            recorder.stop()
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            samples = metadata["host_telemetry"]["samples"]
            self.assertGreaterEqual(len(samples), 2)
            self.assertEqual(samples[-1]["real_time_factor"], 0.97)
            self.assertEqual(metadata["host_telemetry"]["rtf_gazebo"][-1]["real_time_factor"], 0.97)


if __name__ == "__main__":
    unittest.main()
