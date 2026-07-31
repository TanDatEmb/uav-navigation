import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from report_generator import generate, render
from session_manager import create, prune
from snapshot_collector import collect_snapshot


class LifecycleTest(unittest.TestCase):
    def test_session_isolation_and_pruning(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            root = base/"simulation"; workspace = base/"workspace"; px4 = base/"px4"
            (workspace/"tools/simulation/config").mkdir(parents=True)
            (workspace/"tools/simulation/config/px4_mid360_observer.yaml").write_text("x: 1\n")
            (workspace/".git").mkdir(); (px4/".git").mkdir(parents=True)
            first = create(root, workspace, px4, {"GZ_GUI": "0"})
            second = create(root, workspace, px4, {"GZ_GUI": "0"})
            self.assertNotEqual(first, second)
            self.assertEqual((root/"latest").resolve(), second.resolve())
            removed = prune(root, 1)
            self.assertEqual(removed, [first])

    def test_snapshot_and_report_generation(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            for name in ("metrics", "logs", "latest", "snapshots", "pids"):
                (session/name).mkdir()
            event = {"code": "TEST_FAULT", "severity": "ERROR"}
            target = collect_snapshot(session, event)
            self.assertTrue((target/"event.json").exists())
            fields = ["wall_time", "monotonic_time_s", "name", "message_count",
                      "receive_rate_hz_short_window", "receive_rate_hz_long_window",
                      "wall_gap_max", "wall_gap_p95", "header_stamp_regression_count", "stale_events"]
            with (session/"metrics/streams.csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader()
                writer.writerow(dict.fromkeys(fields, "0") | {"name": "lidar", "message_count": "10"})
            (session/"metrics/events.jsonl").write_text("")
            summary = generate(session)
            render(summary, session/"REPORT.md")
            self.assertEqual(summary["overall_result"], "PASS")
            self.assertTrue((session/"REPORT.md").exists())


if __name__ == "__main__":
    unittest.main()
