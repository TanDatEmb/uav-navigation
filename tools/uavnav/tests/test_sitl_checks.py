"""Tests for tools.uavnav.sitl_checks (the SITL probe verdict, D33 O20/O24, D29). Synthetic records only."""
import json
import pathlib
import shutil
import tempfile
import unittest

from tools.uavnav import sitl_checks

S = 1_000_000_000  # ns per second
MS = 1_000_000     # ns per millisecond

HOVER_START_NS = 30 * S
HOVER_END_NS = 50 * S
GROUND_Z_M = 0.05
TARGET_HEIGHT_M = 2.5

XYZ_FLOAT32 = [
    {"name": "x", "offset": 0, "datatype": 7, "count": 1},
    {"name": "y", "offset": 4, "datatype": 7, "count": 1},
    {"name": "z", "offset": 8, "datatype": 7, "count": 1},
    {"name": "intensity", "offset": 12, "datatype": 7, "count": 1},
]


def write_jsonl(path, records):
    with open(path, "w", encoding="utf-8") as f:
        for record in records:
            f.write(json.dumps(record) + "\n")


def make_run(run_dir, nav_state=2, x_datatype=7):
    """A 10..60 s run: IMU 200 Hz, clouds 10 Hz, PX4 position 50 Hz, status 5 Hz; hover 30..50 s."""
    run_dir = pathlib.Path(run_dir)
    imu = [{"arrival_ns": t + 1 * MS, "stamp_ns": t} for t in range(10 * S, 60 * S + 1, 5 * MS)]
    fields = [dict(f) for f in XYZ_FLOAT32]
    fields[0]["datatype"] = x_datatype
    points = [{"arrival_ns": t + 8 * MS, "stamp_ns": t, "newest_imu_stamp_ns": t - 2 * MS,
               "frame_id": "livox_frame", "point_step": 16, "fields": fields,
               "n_points": 20160, "n_nan": 0, "n_inf": 6000}
              for t in range(11 * S, 60 * S, 100 * MS)]
    clock = [{"clock_ns": t} for t in range(10 * S, 60 * S, 4 * MS)]
    vlp = []
    for t in range(10 * S, 60 * S, 20 * MS):
        z = GROUND_Z_M - (TARGET_HEIGHT_M if HOVER_START_NS - 5 * S <= t <= HOVER_END_NS + 2 * S else 0.0)
        vlp.append({"arrival_ns": t + 3 * MS, "timestamp_us": (t + 1 * MS) // 1000,
                    "timestamp_sample_us": t // 1000, "xy_valid": True, "z_valid": True,
                    "x": 0.01, "y": -0.02, "z": z, "vz": 0.0})
    status = []
    for t in range(10 * S, 60 * S, 200 * MS):
        armed = HOVER_START_NS - 10 * S <= t <= HOVER_END_NS + 5 * S
        status.append({"arrival_ns": t + 2 * MS, "timestamp_us": t // 1000,
                       "nav_state": nav_state if armed else 0, "arming_state": 2 if armed else 1})
    write_jsonl(run_dir / "lidar_imu.jsonl", imu)
    write_jsonl(run_dir / "lidar_points.jsonl", points)
    write_jsonl(run_dir / "clock.jsonl", clock)
    write_jsonl(run_dir / "vehicle_local_position.jsonl", vlp)
    write_jsonl(run_dir / "vehicle_status.jsonl", status)
    meta = {"profile": "gps", "hover_mode": "posctl", "target_height_m": TARGET_HEIGHT_M,
            "ground_z_m": GROUND_Z_M, "hover_window": {"start_ns": HOVER_START_NS, "end_ns": HOVER_END_NS},
            "phases": [{"name": "hover", "ok": True, "detail": "", "start_ns": HOVER_START_NS,
                        "end_ns": HOVER_END_NS}]}
    (run_dir / "probe_meta.json").write_text(json.dumps(meta), encoding="utf-8")
    return run_dir


PROBE_CHECKS = {"posctl_accepted", "cloud_layout", "cloud_non_finite_ratio", "imu_rate",
                "scan_stamp_vs_imu", "px4_time_identity"}


class SitlChecksTest(unittest.TestCase):
    def setUp(self):
        self.run_dir = pathlib.Path(tempfile.mkdtemp(prefix="sitl_probe_test_"))
        self.addCleanup(shutil.rmtree, self.run_dir, True)

    def failing(self, summary):
        return {name for name, check in summary["checks"].items() if not check["pass"]}

    def test_probe_pass_on_synthetic_records(self):
        make_run(self.run_dir)
        summary = sitl_checks.evaluate_probe(self.run_dir)
        self.assertEqual(set(summary["checks"]), PROBE_CHECKS)
        self.assertEqual(self.failing(summary), set())
        self.assertEqual(summary["result"], "PASS")
        on_disk = json.loads((self.run_dir / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(on_disk, summary)
        checks = summary["checks"]
        self.assertAlmostEqual(checks["imu_rate"]["value"]["rate_hz"], 200.0, places=3)
        self.assertAlmostEqual(checks["cloud_non_finite_ratio"]["value"]["ratio"], 6000 / 20160, places=6)
        self.assertEqual(checks["cloud_non_finite_ratio"]["value"]["inf"], 6000 * len(
            range(11 * S, 60 * S, 100 * MS)))
        self.assertAlmostEqual(checks["scan_stamp_vs_imu"]["value"]["p99_ms"], 2.0, places=6)
        self.assertAlmostEqual(checks["px4_time_identity"]["value"]["p99_ms"], 2.0, places=6)

    def test_probe_fails_when_posctl_rejected(self):
        make_run(self.run_dir, nav_state=4)  # PX4 stayed in Hold: Position mode was not accepted
        summary = sitl_checks.evaluate_probe(self.run_dir)
        self.assertEqual(self.failing(summary), {"posctl_accepted"})
        self.assertEqual(summary["result"], "FAIL")

    def test_probe_fails_on_float64_cloud(self):
        make_run(self.run_dir, x_datatype=8)
        summary = sitl_checks.evaluate_probe(self.run_dir)
        self.assertEqual(self.failing(summary), {"cloud_layout"})
        self.assertEqual(summary["result"], "FAIL")

    def test_missing_topic_fails_closed(self):
        make_run(self.run_dir)
        (self.run_dir / "lidar_imu.jsonl").unlink()
        summary = sitl_checks.evaluate_probe(self.run_dir)
        self.assertIn("imu_rate", self.failing(summary))
        self.assertEqual(summary["result"], "FAIL")
        # An empty record file is missing data too, and so is a missing hover window.
        make_run(self.run_dir)
        write_jsonl(self.run_dir / "vehicle_local_position.jsonl", [])
        summary = sitl_checks.evaluate_probe(self.run_dir)
        self.assertTrue({"px4_time_identity", "posctl_accepted"} <= self.failing(summary))
        make_run(self.run_dir)
        meta = json.loads((self.run_dir / "probe_meta.json").read_text(encoding="utf-8"))
        meta["hover_window"] = None
        (self.run_dir / "probe_meta.json").write_text(json.dumps(meta), encoding="utf-8")
        self.assertIn("posctl_accepted", self.failing(sitl_checks.evaluate_probe(self.run_dir)))
        # Clouds that arrived before the first IMU sample are counted, not used; with none left the check fails.
        make_run(self.run_dir)
        records = [json.loads(line) for line in
                   (self.run_dir / "lidar_points.jsonl").read_text(encoding="utf-8").splitlines()]
        records[0]["newest_imu_stamp_ns"] = None
        write_jsonl(self.run_dir / "lidar_points.jsonl", records)
        checks = sitl_checks.evaluate_probe(self.run_dir)["checks"]
        self.assertTrue(checks["scan_stamp_vs_imu"]["pass"])
        self.assertEqual(checks["scan_stamp_vs_imu"]["value"]["clouds_before_first_imu"], 1)
        for record in records:
            record["newest_imu_stamp_ns"] = None
        write_jsonl(self.run_dir / "lidar_points.jsonl", records)
        self.assertIn("scan_stamp_vs_imu", self.failing(sitl_checks.evaluate_probe(self.run_dir)))
        # A malformed cloud record fails every cloud check instead of passing it.
        make_run(self.run_dir)
        (self.run_dir / "lidar_points.jsonl").write_text("{oops\n", encoding="utf-8")
        summary = sitl_checks.evaluate_probe(self.run_dir)
        self.assertTrue({"cloud_layout", "cloud_non_finite_ratio", "scan_stamp_vs_imu"}
                        <= self.failing(summary))


if __name__ == "__main__":
    unittest.main()
