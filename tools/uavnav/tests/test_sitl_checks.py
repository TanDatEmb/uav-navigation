"""Tests for tools.uavnav.sitl_checks (the SITL probe verdict, D33 O20/O24, D29). Synthetic records only."""
import json
import math
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


def make_run(run_dir, nav_state=2, x_datatype=7, hover_mode="posctl", imu_period_ms=5, scan_imu_lag_ms=2,
             px4_offset_ms=1, hover_height_m=TARGET_HEIGHT_M, disarm_in_hover=False, imu_arrival_ms=1,
             scan_arrival_ms=8, phase_ok=True):
    """A 10..60 s run: IMU 200 Hz, clouds 10 Hz, PX4 position 50 Hz, status 5 Hz; hover 30..50 s.

    scan_imu_lag_ms: scan stamp - newest IMU stamp at arrival; *_arrival_ms: /clock at arrival - stamp;
    px4_offset_ms: vehicle_local_position.timestamp - stamp grid (arrival is the grid + 3 ms)."""
    run_dir = pathlib.Path(run_dir)
    imu = [{"arrival_ns": t + imu_arrival_ms * MS, "stamp_ns": t}
           for t in range(10 * S, 60 * S + 1, imu_period_ms * MS)]
    fields = [dict(f) for f in XYZ_FLOAT32]
    fields[0]["datatype"] = x_datatype
    points = [{"arrival_ns": t + scan_arrival_ms * MS, "stamp_ns": t,
               "newest_imu_stamp_ns": t - scan_imu_lag_ms * MS,
               "frame_id": "livox_frame", "point_step": 16, "fields": fields,
               "n_points": 20160, "n_nan": 0, "n_inf": 6000}
              for t in range(11 * S, 60 * S, 100 * MS)]
    clock = [{"clock_ns": t} for t in range(10 * S, 61 * S, 4 * MS)]
    vlp = []
    for t in range(10 * S, 60 * S, 20 * MS):
        z = GROUND_Z_M - (hover_height_m if HOVER_START_NS - 5 * S <= t <= HOVER_END_NS + 2 * S else 0.0)
        vlp.append({"arrival_ns": t + 3 * MS, "timestamp_us": (t + px4_offset_ms * MS) // 1000,
                    "timestamp_sample_us": t // 1000, "xy_valid": True, "z_valid": True,
                    "x": 0.01, "y": -0.02, "z": z, "vz": 0.0})
    status = []
    for t in range(10 * S, 60 * S, 200 * MS):
        armed = HOVER_START_NS - 10 * S <= t <= HOVER_END_NS + 5 * S
        if disarm_in_hover and HOVER_START_NS + 10 * S <= t:
            armed = False
        status.append({"arrival_ns": t + 2 * MS, "timestamp_us": t // 1000,
                       "nav_state": nav_state if armed else 0, "arming_state": 2 if armed else 1})
    write_jsonl(run_dir / "lidar_imu.jsonl", imu)
    write_jsonl(run_dir / "lidar_points.jsonl", points)
    write_jsonl(run_dir / "clock.jsonl", clock)
    write_jsonl(run_dir / "vehicle_local_position.jsonl", vlp)
    write_jsonl(run_dir / "vehicle_status.jsonl", status)
    meta = {"profile": "gps", "hover_mode": hover_mode, "target_height_m": TARGET_HEIGHT_M,
            "ground_z_m": GROUND_Z_M, "hover_window": {"start_ns": HOVER_START_NS, "end_ns": HOVER_END_NS},
            "phases": [{"name": "hover", "ok": True, "detail": "", "start_ns": HOVER_START_NS,
                        "end_ns": HOVER_END_NS},
                       {"name": "land", "ok": phase_ok, "detail": "", "start_ns": HOVER_END_NS,
                        "end_ns": HOVER_END_NS + 5 * S}]}
    (run_dir / "probe_meta.json").write_text(json.dumps(meta), encoding="utf-8")
    return run_dir


PROBE_CHECKS = {"posctl_accepted", "cloud_layout", "cloud_non_finite_ratio", "imu_rate",
                "scan_stamp_vs_imu", "sensor_time_identity", "px4_time_identity"}

# --- the S1 hover run (Task 11) -------------------------------------------------------------------------------
# A 10..120 s run of /clock time: LIO TRACKING from 22 s, alignment VALID from 25 s, EV fused from 26 s, armed
# 35..118 s, the 60 s hover window 50..110 s at 2.5 m. Every stream is nominal; the keyword arguments break one.
H_START_NS, H_END_NS = 50 * S, 110 * S
RUN_START_NS, RUN_END_NS = 10 * S, 120 * S
HOVER_CHECKS = {"posctl_accepted", "ekf2_ev_pos", "ekf2_heading", "lio_tracking", "lio_flap_rate", "alignment_valid",
                "alignment_rate", "alignment_jump_rejections", "drift", "lio_state_rate", "ev_age",
                "px4_time_identity", "events", "f34_values", "ev_variance", "gnss_consistency"}
HOVER_CHECKS_NO_GPS = HOVER_CHECKS - {"gnss_consistency"}   # GNSS consistency is judged in the gps profile only
HEALTH_CODE = {"INITIALIZING": 0, "TRACKING": 1, "DEGRADED": 2, "LOST": 3}
HEADING_FLAGS = ("cs_mag_hdg", "cs_mag_3d", "cs_ev_yaw")     # the heading sources the recorder keeps


def event(t_ns, component, name, before="", after="", reason="", epoch=1, **values):
    return {"t_steady_ns": t_ns + 7 * S, "t_ros_ns": t_ns, "component": component, "event": name,
            "state_before": before, "state_after": after, "reason": reason, "lio_epoch": epoch, "values": values}


def lio_node_summary(t_ns, **overrides):
    values = {"imu_received": 21800.0, "scans_received": 1090.0, "scans_converted": 1090.0,
              "points_in": 1090 * 20160.0, "points_non_finite": 1090 * 14000.0, "rejected_bad_stamp": 0.0,
              "rejected_not_finite": 0.0, "rejected_wrong_frame": 0.0, "rejected_malformed_cloud": 0.0,
              "rejected_no_ros_time": 0.0, "ingest_refused": 0.0, "states_published": 9600.0,
              "health_published": 980.0, "odometry_published": 960.0, "publish_failed": 0.0}
    values.update(overrides)
    return event(t_ns, "lio", "LioNodeSummary", reason="SHUTDOWN", **values)


def bridge_node_summary(t_ns, **overrides):
    values = {"odometry_received": 960.0, "health_received": 980.0, "local_position_received": 5500.0,
              "rejected_odometry": 0.0, "rejected_health": 0.0, "rejected_local_position": 0.0, "no_ros_time": 0.0,
              "ticks": 1100.0, "first_tick": 0.0, "ev_published": 950.0, "alignment_published": 2000.0,
              "publish_failed": 0.0}
    values.update(overrides)
    return event(t_ns, "px4_bridge", "BridgeNodeSummary", reason="SHUTDOWN", **values)


def lio_state_at(t, states):
    """The LIO state at t from a sorted [(t_ns, state_name)] list of transitions."""
    current = "INITIALIZING"
    for when, name in states:
        if t >= when:
            current = name
    return current


def make_hover_run(run_dir, profile="gps", spawn_yaw_rad=0.0, lio_transitions=(), ev_pos_gap=None, heading=None,
                   px4_offset_ms=1, scan_too_old_ages_s=(), lio_events=(), bridge_events=(), lio_summary=None,
                   bridge_summary=None, alignment_states=(), phase_ok=True, px4_north_m=0.0, truth_north_m=0.0,
                   state_period_ms=10, alignment_period_ms=100, ev_age_ms=40, yaw_t_rad=None, heading_resets=(),
                   gnss_gap=None, gnss_outage=None, ev_position_sigma_m=0.03):
    """Write a synthetic hover run.

    lio_transitions: extra [(t_ns, before, after)] LIO lifecycle changes after TRACKING at 22 s (health and the
    StateTransition events follow them; odometry is published only in TRACKING). ev_pos_gap: (start_ns, end_ns)
    with cs_ev_pos false. heading: [(t_ns, source)] the EKF2 heading source from t_ns on, source one of "none",
    "mag_hdg", "mag_3d", "ev_yaw" (default: none, then from 26 s ev_yaw for no_gps and mag_3d for gps);
    cs_yaw_align holds from the first source that is not "none".
    alignment_states: extra [(t_ns, state_code)] alignment state changes after VALID at 25 s.
    px4_north_m, truth_north_m: northward displacement over the hover window of PX4 and of the ground truth.
    yaw_t_rad: the yaw of T in /alignment (default: pi/2 - spawn yaw for gps, 0 for no_gps, as measured in S1b).
    heading_resets: [(t_ns, delta_rad)] EKF2 heading resets (vehicle_local_position heading_reset_counter +1 and
    delta_heading). gnss_gap: (start_ns, end_ns) with cs_gnss_pos false (gps). gnss_outage: the hover_meta.json
    "gnss_outage" record (None: no outage requested). ev_position_sigma_m: the EV position sigma sent, every axis."""
    run_dir = pathlib.Path(run_dir)
    transitions = [(22 * S, "INITIALIZING", "TRACKING")] + sorted(lio_transitions)
    states = [(t, after) for t, _, after in transitions]
    yaw_t = yaw_t_rad if yaw_t_rad is not None else (math.pi / 2 - spawn_yaw_rad if profile == "gps" else 0.0)
    if heading is None:
        heading = [(RUN_START_NS, "none"), (26 * S, "ev_yaw" if profile == "no_gps" else "mag_3d")]
    lio_state, health, odometry, alignment, ev = [], [], [], [], []
    for t in range(20 * S, 118 * S, state_period_ms * MS):
        lio_state.append({"arrival_ns": t + 2 * MS, "wall_ns": t + 5 * S, "stamp_ns": t, "epoch": 1, "lio_state": 1,
                          "x": 0.0, "y": 0.0, "z": 2.5, "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
                          "vx": 0.0, "vy": 0.0, "vz": 0.0})
    align_changes = [(25 * S, 1)] + sorted(alignment_states)
    for t in range(20 * S, 118 * S, 100 * MS):
        name = lio_state_at(t, states)
        health.append({"arrival_ns": t + 3 * MS, "stamp_ns": t, "state": HEALTH_CODE[name], "epoch": 1,
                       "reason_name": "", "correction_age_s": 0.03, "output_error_attitude_rad": 0.001,
                       "output_error_velocity_mps": 0.01, "output_error_position_m": 0.002})
        if t >= 22 * S and name == "TRACKING":
            odometry.append({"arrival_ns": t + 30 * MS, "stamp_ns": t, "epoch": 1, "reset_counter": 0,
                             "quality": 90, "x": 0.0, "y": 0.0, "z": 2.5})
        if t >= 22 * S and (t // MS) % alignment_period_ms == 0:
            code = 0
            for when, value in align_changes:
                if t >= when:
                    code = value
            alignment.append({"arrival_ns": t + 32 * MS, "stamp_ns": t, "state": code, "x_m": 0.0, "y_m": 0.0,
                              "z_m": 0.0, "yaw_rad": yaw_t, "age_s": 0.0, "residual_position_m": 0.01,
                              "residual_yaw_rad": 0.001})
        if t >= 25 * S and name == "TRACKING":
            variance = ev_position_sigma_m ** 2
            ev.append({"arrival_ns": t + ev_age_ms * MS, "timestamp_us": (t + ev_age_ms * MS) // 1000,
                       "timestamp_sample_us": t // 1000, "position_variance": [variance] * 3,
                       "velocity_variance": [0.0004] * 3, "orientation_variance": [1e-6] * 3})
    vlp, truth = [], []
    resets = sorted(heading_resets)
    for t in range(RUN_START_NS, 119 * S, 20 * MS):
        z = -TARGET_HEIGHT_M if 40 * S <= t <= 115 * S else 0.0
        progress = min(1.0, max(0.0, (t - H_START_NS) / (H_END_NS - H_START_NS)))
        done = [delta for when, delta in resets if t >= when]
        vlp.append({"arrival_ns": t + 3 * MS, "timestamp_us": (t + px4_offset_ms * MS) // 1000,
                    "timestamp_sample_us": t // 1000, "xy_valid": True, "z_valid": True,
                    "x": 0.01 + px4_north_m * progress, "y": -0.02, "z": z, "vz": 0.0, "heading": yaw_t,
                    "xy_reset_counter": 0, "z_reset_counter": 0, "heading_reset_counter": len(done),
                    "delta_heading": done[-1] if done else 0.0})
        truth.append({"arrival_ns": t + 2 * MS, "stamp_ns": t, "x": -0.02, "y": 0.01 + truth_north_m * progress,
                      "z": -z})
    flags = []
    schedule = sorted(heading)
    for t in range(RUN_START_NS, 119 * S, 200 * MS):
        fused = t >= 26 * S and not (ev_pos_gap and ev_pos_gap[0] <= t < ev_pos_gap[1])
        source = [s for when, s in schedule if t >= when]
        source = source[-1] if source else "none"
        aligned = any(t >= when and s != "none" for when, s in schedule)
        gnss = profile == "gps" and not (gnss_gap and gnss_gap[0] <= t < gnss_gap[1])
        flags.append({"arrival_ns": t + 2 * MS, "timestamp_us": t // 1000, "cs_ev_pos": fused,
                      "cs_ev_hgt": fused, "cs_ev_vel": fused, "cs_gnss_pos": gnss, "cs_yaw_align": aligned,
                      **{flag: flag == f"cs_{source}" for flag in HEADING_FLAGS}})
    status = []
    for t in range(RUN_START_NS, 119 * S, 200 * MS):
        armed = 35 * S <= t <= 118 * S
        status.append({"arrival_ns": t + 2 * MS, "timestamp_us": t // 1000, "nav_state": 2 if armed else 0,
                       "arming_state": 2 if armed else 1})
    events_lio = [event(t, "lio", "StateTransition", before, after, "SCAN_GOOD", sensor_time_s=t / 1e9)
                  for t, before, after in transitions]
    events_lio += [event(60 * S + i * S, "lio", "ScanResultTooOld", reason="RESULT_TOO_OLD", age_s=age,
                         gap_degraded_s=0.25) for i, age in enumerate(scan_too_old_ages_s)]
    events_lio += list(lio_events)
    events_lio.sort(key=lambda r: r["t_ros_ns"])
    events_lio.append(lio_summary if lio_summary is not None else lio_node_summary(119 * S))
    events_lio.append(event(119 * S, "lio", "IngestSummary", reason="ACCEPTED", scans_pushed=1080.0))
    events_bridge = [event(25 * S, "px4_bridge", "AlignmentTransition", "INIT", "VALID", "CONSISTENT_PAIRS",
                           input=0.0, sensor_time_s=25.0)]
    events_bridge += list(bridge_events)
    events_bridge.sort(key=lambda r: r["t_ros_ns"])
    events_bridge.append(bridge_summary if bridge_summary is not None else bridge_node_summary(119 * S))
    for name, records in (("lio_state", lio_state), ("lio_health", health), ("lio_odometry", odometry),
                          ("alignment", alignment), ("vehicle_visual_odometry", ev),
                          ("vehicle_local_position", vlp), ("ground_truth", truth),
                          ("estimator_status_flags", flags), ("vehicle_status", status),
                          ("events_lio", events_lio), ("events_px4_bridge", events_bridge)):
        write_jsonl(run_dir / f"{name}.jsonl", records)
    phases = [{"name": name, "ok": True, "detail": "", "start_ns": start, "end_ns": end}
              for name, start, end in (("wait_ready", 20 * S, 30 * S), ("posctl", 30 * S, 30 * S),
                                       ("arm", 30 * S, 35 * S), ("climb", 35 * S, H_START_NS),
                                       ("hover", H_START_NS, H_END_NS), ("land", H_END_NS, H_END_NS + 5 * S))]
    phases[-1]["ok"] = phase_ok
    meta = {"profile": profile, "spawn_yaw_rad": spawn_yaw_rad, "pattern": "hover", "hover_mode": "posctl",
            "target_height_m": TARGET_HEIGHT_M, "ground_z_m": 0.0,
            "hover_window": {"start_ns": H_START_NS, "end_ns": H_END_NS}, "phases": phases, "interrupted": False,
            "gnss_outage": gnss_outage}
    (run_dir / "hover_meta.json").write_text(json.dumps(meta), encoding="utf-8")
    return run_dir


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
        self.assertEqual(checks["sensor_time_identity"]["value"]["imu"]["p99_abs_ms"], 1.0)
        self.assertEqual(checks["sensor_time_identity"]["value"]["scan"]["p99_abs_ms"], 8.0)
        self.assertEqual(checks["cloud_non_finite_ratio"]["value"]["min_finite_points"], 20160 - 6000)
        self.assertEqual(checks["cloud_layout"]["value"]["hover_window_clouds"], 200)
        self.assertAlmostEqual(checks["cloud_layout"]["value"]["hover_window_rate_hz"], 10.0, places=6)
        self.assertTrue(summary["phases_ok"])

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

    def assert_only_failing(self, expected, **kwargs):
        make_run(self.run_dir, **kwargs)
        summary = sitl_checks.evaluate_probe(self.run_dir)
        self.assertEqual(self.failing(summary), expected, kwargs)
        self.assertEqual(summary["result"], "FAIL")
        return summary

    def test_imu_rate_out_of_band_fails(self):
        summary = self.assert_only_failing({"imu_rate"}, imu_period_ms=6)  # 166.7 Hz
        self.assertAlmostEqual(summary["checks"]["imu_rate"]["value"]["rate_hz"], 1000 / 6, places=3)

    def test_scan_stamp_vs_imu_is_two_sided(self):
        self.assert_only_failing({"scan_stamp_vs_imu"}, scan_imu_lag_ms=60)    # scan 60 ms ahead of the IMU
        self.assert_only_failing({"scan_stamp_vs_imu"}, scan_imu_lag_ms=-300)  # scan 300 ms behind the IMU

    def test_px4_time_offset_fails(self):
        summary = self.assert_only_failing({"px4_time_identity"}, px4_offset_ms=33)  # 30 ms from /clock
        self.assertAlmostEqual(summary["checks"]["px4_time_identity"]["value"]["p99_ms"], 30.0, places=6)

    def test_hover_height_error_fails(self):
        self.assert_only_failing({"posctl_accepted"}, hover_height_m=TARGET_HEIGHT_M + 0.6)

    def test_disarm_during_hover_fails(self):
        summary = self.assert_only_failing({"posctl_accepted"}, disarm_in_hover=True)
        self.assertFalse(summary["checks"]["posctl_accepted"]["value"]["armed_throughout"])

    def test_hold_mode_uses_hold_accepted(self):
        make_run(self.run_dir, hover_mode="hold", nav_state=4)
        summary = sitl_checks.evaluate_probe(self.run_dir)
        self.assertIn("hold_accepted", summary["checks"])
        self.assertNotIn("posctl_accepted", summary["checks"])
        self.assertEqual(summary["result"], "PASS")
        self.assert_only_failing({"hold_accepted"}, hover_mode="hold", nav_state=2)

    def test_sensor_time_identity(self):
        # IMU stamped 600 ms ahead of /clock: within no p99 bound, and beyond the O20 0.5 s lead.
        summary = self.assert_only_failing({"sensor_time_identity"}, imu_arrival_ms=-600)
        self.assertEqual(summary["checks"]["sensor_time_identity"]["value"]["imu"]["max_ms"], 600.0)
        # Scan arriving 150 ms after its stamp (the newest IMU still within bounds of the scan).
        self.assert_only_failing({"sensor_time_identity"}, scan_arrival_ms=150)
        # /clock going backwards fails too.
        make_run(self.run_dir)
        write_jsonl(self.run_dir / "clock.jsonl", [{"clock_ns": 20 * S}, {"clock_ns": 19 * S}])
        self.assertEqual(self.failing(sitl_checks.evaluate_probe(self.run_dir)), {"sensor_time_identity"})

    def test_cloud_without_finite_points_fails(self):
        make_run(self.run_dir)
        records = [json.loads(line) for line in
                   (self.run_dir / "lidar_points.jsonl").read_text(encoding="utf-8").splitlines()]
        records[5]["n_inf"] = records[5]["n_points"]
        write_jsonl(self.run_dir / "lidar_points.jsonl", records)
        summary = sitl_checks.evaluate_probe(self.run_dir)
        self.assertEqual(self.failing(summary), {"cloud_non_finite_ratio"})
        self.assertEqual(summary["checks"]["cloud_non_finite_ratio"]["value"]["min_finite_points"], 0)

    def test_failed_phase_fails_the_result(self):
        make_run(self.run_dir, phase_ok=False)
        summary = sitl_checks.evaluate_probe(self.run_dir)
        self.assertEqual(self.failing(summary), set())
        self.assertFalse(summary["phases_ok"])
        self.assertEqual(summary["result"], "FAIL")

    # --- hover (Task 11) ----------------------------------------------------------------------------------------
    def hover(self, profile="gps", spawn_yaw_rad=0.0, **kwargs):
        make_hover_run(self.run_dir, profile=profile, spawn_yaw_rad=spawn_yaw_rad, **kwargs)
        return sitl_checks.evaluate_hover(self.run_dir, profile, spawn_yaw_rad)

    def test_hover_pass_on_synthetic_records(self):
        summary = self.hover()
        self.assertEqual(set(summary["checks"]), HOVER_CHECKS)
        self.assertEqual(self.failing(summary), set())
        self.assertEqual(summary["result"], "PASS")
        self.assertEqual(json.loads((self.run_dir / "summary.json").read_text(encoding="utf-8")), summary)
        checks = summary["checks"]
        self.assertEqual(checks["ekf2_ev_pos"]["value"]["fraction"], 1.0)
        self.assertEqual(checks["lio_flap_rate"]["value"]["degraded_to_tracking_in_window"], 0)
        self.assertAlmostEqual(checks["lio_state_rate"]["value"]["rate_hz"], 100.0, delta=0.1)
        self.assertAlmostEqual(checks["lio_state_rate"]["value"]["interarrival_p99_ms"], 10.0, places=6)
        # T11 M2: the inter-arrival is also reported in wall time (the recorder's wall_ns), a cross-check of the
        # /clock-time value whose resolution is one Gazebo step.
        self.assertAlmostEqual(checks["lio_state_rate"]["value"]["interarrival_wall_p99_ms"], 10.0, places=6)
        self.assertAlmostEqual(checks["lio_state_rate"]["value"]["f23_step_max_m"], 0.0, places=9)
        self.assertAlmostEqual(checks["alignment_rate"]["value"]["rate_hz"], 10.0, delta=0.1)
        self.assertAlmostEqual(checks["ev_age"]["value"]["p99_ms"], 40.0, places=6)
        self.assertAlmostEqual(checks["px4_time_identity"]["value"]["p99_ms"], 2.0, places=6)
        self.assertAlmostEqual(checks["drift"]["value"]["max_m"], 0.0, places=9)
        events = checks["events"]["value"]
        self.assertEqual(events["imu_gap"], 0)
        self.assertEqual(events["px4_sample_not_finite"], 0)
        self.assertEqual(events["lio_node_summary"]["rejected_wrong_frame"], 0)
        self.assertAlmostEqual(events["scan_rate_hz_vs_imu"], 10.0, places=6)
        self.assertAlmostEqual(checks["f34_values"]["value"]["yaw_t_spread_rad"], 0.0, places=9)
        self.assertAlmostEqual(checks["f34_values"]["value"]["yaw_t_offset_rad"], 0.0, places=9)
        self.assertTrue(summary["phases_ok"])

    def test_fails_on_lio_lost(self):
        summary = self.hover(lio_transitions=[(80 * S, "TRACKING", "LOST")])
        self.assertIn("lio_tracking", self.failing(summary))
        self.assertEqual(summary["checks"]["lio_tracking"]["value"]["lost_entries"], 1)
        self.assertEqual(summary["result"], "FAIL")
        # LOST after the window still fails (no LOST in the run), and so do too many TRACKING entries.
        summary = self.hover(lio_transitions=[(116 * S, "TRACKING", "LOST")])
        self.assertIn("lio_tracking", self.failing(summary))
        # A LOST health sample after the window, without its transition event, fails too.
        make_hover_run(self.run_dir)
        health = [json.loads(line) for line in
                  (self.run_dir / "lio_health.jsonl").read_text(encoding="utf-8").splitlines()]
        health[-1]["state"] = 3
        write_jsonl(self.run_dir / "lio_health.jsonl", health)
        summary = sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)
        self.assertEqual(self.failing(summary), {"lio_tracking"})
        self.assertEqual(summary["checks"]["lio_tracking"]["value"]["lost_health_samples"], 1)
        lost_after_health = event(119 * S, "lio", "StateTransition", "TRACKING", "LOST", "GAP_LOST")
        summary = self.hover(lio_events=[lost_after_health])  # the transition alone, no LOST health sample
        self.assertEqual(self.failing(summary), {"lio_tracking"})
        self.assertEqual(summary["checks"]["lio_tracking"]["value"]["lost_entries"], 1)
        # A DEGRADED excursion between two 10 Hz health samples is still seen through its transitions.
        summary = self.hover(lio_transitions=[(70 * S + 20 * MS, "TRACKING", "DEGRADED"),
                                              (70 * S + 75 * MS, "DEGRADED", "TRACKING")])
        self.assertEqual(summary["checks"]["lio_tracking"]["value"]["window_health_states"], [1])
        self.assertIn("lio_tracking", self.failing(summary))
        summary = self.hover(lio_transitions=[(30 * S, "TRACKING", "DEGRADED"), (31 * S, "DEGRADED", "TRACKING"),
                                              (40 * S, "TRACKING", "DEGRADED"), (41 * S, "DEGRADED", "TRACKING")])
        self.assertEqual(self.failing(summary), {"lio_tracking"})
        self.assertEqual(summary["checks"]["lio_tracking"]["value"]["tracking_entries"], 3)

    def test_fails_on_ev_fusion_gap(self):
        summary = self.hover(ev_pos_gap=(70 * S, 80 * S))  # 10 s of the 60 s window without EV position fusion
        self.assertEqual(self.failing(summary), {"ekf2_ev_pos"})
        self.assertAlmostEqual(summary["checks"]["ekf2_ev_pos"]["value"]["fraction"], 50 / 60, places=4)
        # D42: PX4 chooses the heading source; EV yaw fused or not fails nothing, in either profile.
        for profile in ("gps", "no_gps"):
            for source in ("ev_yaw", "mag_3d"):
                summary = self.hover(profile=profile, heading=[(RUN_START_NS, "none"), (26 * S, source)])
                self.assertEqual(self.failing(summary), set(), (profile, source))
        # T11 M3: the fraction is weighted by time (each flags sample holds until the next one), not by sample.
        # 3.2 s without EV fusion carried by two samples (70.0 and 71.6 s) is 5.3 % of the window: it fails,
        # where a per-sample count (2 of 286) would pass.
        make_hover_run(self.run_dir, ev_pos_gap=(70 * S, 73200 * MS))
        flags = [r for r in self.read("estimator_status_flags.jsonl")
                 if not (70 * S < r["arrival_ns"] < 73200 * MS) or r["arrival_ns"] in (70002 * MS, 71602 * MS)]
        write_jsonl(self.run_dir / "estimator_status_flags.jsonl", flags)
        summary = sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)
        self.assertEqual(self.failing(summary), {"ekf2_ev_pos"})
        self.assertAlmostEqual(summary["checks"]["ekf2_ev_pos"]["value"]["fraction"], 1 - 3.2 / 60, places=4)

    def flaps(self, count, start_s=60, every_s=15):
        transitions = []
        for i in range(count):
            t = (start_s + i * every_s) * S
            transitions += [(t, "TRACKING", "DEGRADED"), (t + 55 * MS, "DEGRADED", "TRACKING")]
        return transitions

    def test_fails_on_degraded_tracking_flapping(self):
        ages = (0.26, 0.27, 0.30, 0.28)
        summary = self.hover(lio_transitions=self.flaps(3), scan_too_old_ages_s=ages)
        self.assertIn("lio_flap_rate", self.failing(summary))
        value = summary["checks"]["lio_flap_rate"]["value"]
        self.assertEqual(value["degraded_to_tracking_in_window"], 3)
        self.assertAlmostEqual(value["per_60s"], 3.0, places=6)
        self.assertEqual(value["scan_result_too_old"]["count"], 4)
        self.assertAlmostEqual(value["scan_result_too_old"]["age_p50_s"], 0.27, places=9)
        self.assertAlmostEqual(value["scan_result_too_old"]["age_p99_s"], 0.30, places=9)
        self.assertAlmostEqual(value["applied_result_age"]["p50_s"], 0.03, places=6)
        self.assertAlmostEqual(value["applied_result_age"]["p99_s"], 0.03, places=6)
        # Zero transitions pass; one in the 60 s window is the limit (it still fails lio_tracking: 100% TRACKING).
        self.assertNotIn("lio_flap_rate", self.failing(self.hover()))
        summary = self.hover(lio_transitions=self.flaps(1))
        self.assertNotIn("lio_flap_rate", self.failing(summary))
        self.assertIn("lio_tracking", self.failing(summary))
        # Transitions outside the window are reported, not judged.
        summary = self.hover(lio_transitions=self.flaps(2, start_s=30, every_s=2))
        self.assertNotIn("lio_flap_rate", self.failing(summary))
        self.assertEqual(summary["checks"]["lio_flap_rate"]["value"]["degraded_to_tracking_in_run"], 2)

    def test_px4_time_identity_violation_fails(self):
        summary = self.hover(px4_offset_ms=53)  # timestamp 50 ms ahead of /clock at arrival
        self.assertEqual(self.failing(summary), {"px4_time_identity"})
        self.assertAlmostEqual(summary["checks"]["px4_time_identity"]["value"]["p99_ms"], 50.0, places=6)

    def test_hover_missing_topic_fails_closed(self):
        make_hover_run(self.run_dir)
        (self.run_dir / "estimator_status_flags.jsonl").unlink()
        failing = self.failing(sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0))
        self.assertEqual(failing, {"ekf2_ev_pos", "ekf2_heading", "gnss_consistency"})
        for name, expected in (("events_lio.jsonl", {"events", "lio_tracking", "lio_flap_rate"}),
                               ("events_px4_bridge.jsonl", {"events", "alignment_valid",
                                                             "alignment_jump_rejections", "f34_values"}),
                               ("ground_truth.jsonl", {"drift"}),
                               ("vehicle_visual_odometry.jsonl", {"ev_age", "ev_variance"}),
                               ("vehicle_local_position.jsonl", {"ekf2_heading", "drift"}),
                               ("lio_state.jsonl", {"lio_state_rate"}),
                               ("alignment.jsonl", {"alignment_valid", "alignment_rate", "f34_values"})):
            make_hover_run(self.run_dir)
            (self.run_dir / name).unlink()
            summary = sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)
            self.assertTrue(expected <= self.failing(summary), (name, self.failing(summary)))
            self.assertEqual(summary["result"], "FAIL")
        # No node summary (a process that did not shut down cleanly) fails the events check.
        make_hover_run(self.run_dir)
        records = [json.loads(line) for line in
                   (self.run_dir / "events_lio.jsonl").read_text(encoding="utf-8").splitlines()]
        write_jsonl(self.run_dir / "events_lio.jsonl", [r for r in records if r["event"] != "LioNodeSummary"])
        self.assertEqual(self.failing(sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)), {"events"})
        # No hover window: every windowed check fails.
        make_hover_run(self.run_dir)
        meta = json.loads((self.run_dir / "hover_meta.json").read_text(encoding="utf-8"))
        meta["hover_window"] = None
        (self.run_dir / "hover_meta.json").write_text(json.dumps(meta), encoding="utf-8")
        failing = self.failing(sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0))
        self.assertTrue({"ekf2_ev_pos", "lio_tracking", "lio_flap_rate", "drift", "ev_age", "alignment_rate",
                         "f34_values", "posctl_accepted"} <= failing, failing)
        # The profile the run was started with must match the one judged.
        make_hover_run(self.run_dir)
        with self.assertRaises(ValueError):
            sitl_checks.evaluate_hover(self.run_dir, "no_gps", 0.0)

    def test_hover_events_criteria(self):
        # Node summary: a wrong-frame or malformed cloud fails (the Gazebo layout and the IMU frame).
        self.assertEqual(self.failing(self.hover(lio_summary=lio_node_summary(119 * S, rejected_wrong_frame=3.0))),
                         {"events"})
        self.assertEqual(self.failing(self.hover(
            lio_summary=lio_node_summary(119 * S, rejected_malformed_cloud=1.0))), {"events"})
        # Zero-tolerance events, in either file.
        for component, name in (("lio", "LioQueueOverflow"), ("lio", "LioBackendFailed"), ("lio", "ImuRateTooHigh"),
                                ("core", "EventsDropped")):
            summary = self.hover(lio_events=[event(70 * S, component, name, reason="X", count=1.0)])
            self.assertEqual(self.failing(summary), {"events"}, name)
        summary = self.hover(bridge_events=[event(70 * S, "core", "EventsDropped", reason="RING_FULL", count=4.0)])
        self.assertEqual(self.failing(summary), {"events"})
        # Scan drops: 10 of 1090 scans (0.9 %) pass, 12 (1.1 %) fail.
        drops = [event(30 * S + i * S, "lio", "LioScanDropped", reason="SUPERSEDED", waited_s=0.0) for i in range(12)]
        self.assertEqual(self.failing(self.hover(lio_events=drops[:10])), set())
        summary = self.hover(lio_events=drops)
        self.assertEqual(self.failing(summary), {"events"})
        self.assertEqual(summary["checks"]["events"]["value"]["lio_scan_dropped"], 12)
        # ImuGap is reported (rate-limited: the running count of the last record of an epoch), not judged.
        gaps = [event(30 * S, "lio", "ImuGap", reason="IMU_GAP", count=1.0),
                event(90 * S, "lio", "ImuGap", reason="IMU_GAP", count=7.0)]
        summary = self.hover(lio_events=gaps)
        self.assertEqual(self.failing(summary), set())
        self.assertEqual(summary["checks"]["events"]["value"]["imu_gap"], 7)
        # Px4SampleRejected NOT_FINITE is reported (expected 0), not judged.
        summary = self.hover(bridge_summary=bridge_node_summary(119 * S, rejected_local_position=5.0),
                             bridge_events=[event(40 * S, "px4_bridge", "Px4SampleRejected", reason="NOT_FINITE",
                                                  count_total=1.0)])
        self.assertEqual(self.failing(summary), set())
        self.assertEqual(summary["checks"]["events"]["value"]["px4_sample_not_finite"], 5)

    def test_hover_alignment_criteria(self):
        # Jump rejections are counted over rate-limited episodes: the first record (count 1) and the summary
        # (count = the episode total) of each episode; 6 of 600 pairs is 1 %, 7 fails.
        def episode(t_s, total):
            return [event(t_s * S, "px4_bridge", "AlignmentPairRejected", "VALID", "VALID", "PAIR_REJECTED_JUMP",
                          input=0.0, count=1.0),
                    event(t_s * S + 500 * MS, "px4_bridge", "AlignmentPairRejected", "VALID", "VALID",
                          "PAIR_REJECTED_JUMP", input=0.0, count=float(total), suppressed=float(total - 1),
                          summary=1.0)]
        summary = self.hover(bridge_events=episode(60, 4) + episode(80, 2))
        self.assertEqual(self.failing(summary), set())
        self.assertEqual(summary["checks"]["alignment_jump_rejections"]["value"]["jump_rejected"], 6)
        self.assertEqual(summary["checks"]["alignment_jump_rejections"]["value"]["pairs"], 600)
        summary = self.hover(bridge_events=episode(60, 4) + episode(80, 3))
        self.assertEqual(self.failing(summary), {"alignment_jump_rejections"})
        # Single-occurrence episodes have no summary record: each first record (count 1) is one rejection.
        singles = [episode(60 + 5 * i, 1)[0] for i in range(7)]
        self.assertEqual(self.failing(self.hover(bridge_events=singles[:6])), set())
        summary = self.hover(bridge_events=singles)
        self.assertEqual(self.failing(summary), {"alignment_jump_rejections"})
        self.assertEqual(summary["checks"]["alignment_jump_rejections"]["value"]["jump_rejected"], 7)
        # /alignment below 9 Hz fails (5 Hz here).
        summary = self.hover(alignment_period_ms=200)
        self.assertEqual(self.failing(summary), {"alignment_rate"})
        # FROZEN in the window, a STALE anywhere, or a reset that leaves VALID fail alignment_valid.
        self.assertEqual(self.failing(self.hover(alignment_states=[(70 * S, 2), (71 * S, 1)])), {"alignment_valid"})
        stale = event(116 * S, "px4_bridge", "AlignmentTransition", "VALID", "INVALID", "STALE")
        self.assertEqual(self.failing(self.hover(bridge_events=[stale])), {"alignment_valid"})
        reset_ok = event(70 * S, "px4_bridge", "Px4ResetApplied", "VALID", "VALID", "PX4_RESET_APPLIED")
        reset_bad = event(70 * S, "px4_bridge", "Px4ResetApplied", "VALID", "INVALID", "PX4_RESET_APPLIED")
        summary = self.hover(bridge_events=[reset_ok])
        self.assertEqual(self.failing(summary), set())
        self.assertEqual(summary["checks"]["alignment_valid"]["value"]["px4_resets"], 1)
        self.assertEqual(self.failing(self.hover(bridge_events=[reset_bad])), {"alignment_valid", "f34_values"})

    def test_hover_rates_and_ages(self):
        # /lio/state at 1/12 ms = 83 Hz fails; EV older than 150 ms (p99) fails.
        summary = self.hover(state_period_ms=12)
        self.assertEqual(self.failing(summary), {"lio_state_rate"})
        self.assertAlmostEqual(summary["checks"]["lio_state_rate"]["value"]["rate_hz"], 1000 / 12, delta=0.1)
        summary = self.hover(ev_age_ms=160)
        self.assertEqual(self.failing(summary), {"ev_age"})
        self.assertAlmostEqual(summary["checks"]["ev_age"]["value"]["p99_ms"], 160.0, places=6)

    def test_hover_records_outside_the_window_or_torn_fail_closed(self):
        make_hover_run(self.run_dir)
        flags = [json.loads(line) for line in
                 (self.run_dir / "estimator_status_flags.jsonl").read_text(encoding="utf-8").splitlines()]
        write_jsonl(self.run_dir / "estimator_status_flags.jsonl", [r for r in flags if r["arrival_ns"] < H_START_NS])
        self.assertEqual(self.failing(sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)),
                         {"ekf2_ev_pos", "ekf2_heading", "gnss_consistency"})
        # A torn line in an event log (a process killed mid-write) is missing evidence.
        make_hover_run(self.run_dir)
        with open(self.run_dir / "events_lio.jsonl", "a", encoding="utf-8") as f:
            f.write('{"t_steady_ns": 1, "event": "Torn\n')
        summary = sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)
        self.assertEqual(self.failing(summary), {"events", "lio_tracking", "lio_flap_rate"})
        self.assertIn("malformed", summary["checks"]["events"]["detail"])

    def read(self, name):
        return [json.loads(line) for line in (self.run_dir / name).read_text(encoding="utf-8").splitlines()]

    def edit_meta(self, **changes):
        meta = json.loads((self.run_dir / "hover_meta.json").read_text(encoding="utf-8"))
        meta.update(changes)
        (self.run_dir / "hover_meta.json").write_text(json.dumps(meta), encoding="utf-8")
        return meta

    def test_hover_window_shorter_than_60_s_fails(self):
        windowed = {"ekf2_ev_pos", "ekf2_heading", "lio_tracking", "lio_flap_rate",
                    "alignment_valid", "alignment_rate", "alignment_jump_rejections", "drift", "lio_state_rate",
                    "ev_age", "px4_time_identity", "f34_values", "ev_variance", "gnss_consistency"}
        for end_ns in (H_START_NS + 2 * S, H_END_NS - 100 * MS):  # healthy data, but 2 s and 59.9 s windows
            make_hover_run(self.run_dir)
            self.edit_meta(hover_window={"start_ns": H_START_NS, "end_ns": end_ns})
            summary = sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)
            self.assertTrue(windowed <= self.failing(summary), (end_ns, windowed - self.failing(summary)))
            self.assertIn("shorter than", summary["checks"]["ekf2_ev_pos"]["detail"])
            self.assertEqual(summary["result"], "FAIL")

    def test_hover_streams_must_cover_the_window(self):
        # One sample cannot stand for 60 s: a single flags, health or alignment record in the window fails.
        for name, checks in (("estimator_status_flags.jsonl", {"ekf2_ev_pos", "ekf2_heading", "gnss_consistency"}),
                             ("lio_health.jsonl", {"lio_tracking"}),
                             ("alignment.jsonl", {"alignment_valid", "alignment_rate", "f34_values"})):
            make_hover_run(self.run_dir)
            records = self.read(name)
            inside = [r for r in records if H_START_NS <= r["arrival_ns"] <= H_END_NS]
            kept = [r for r in records if not H_START_NS <= r["arrival_ns"] <= H_END_NS] + [inside[len(inside) // 2]]
            write_jsonl(self.run_dir / name, sorted(kept, key=lambda r: r["arrival_ns"]))
            summary = sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)
            self.assertTrue(checks <= self.failing(summary), (name, self.failing(summary)))
            self.assertIn("cover", summary["checks"][sorted(checks)[0]]["detail"])
        # A 3 s hole in the 5 Hz flags fails; a 1.8 s hole passes.
        for hole_s, fails in ((3.0, True), (1.8, False)):
            make_hover_run(self.run_dir)
            records = [r for r in self.read("estimator_status_flags.jsonl")
                       if not 70 * S < r["arrival_ns"] < 70 * S + int(hole_s * S)]
            write_jsonl(self.run_dir / "estimator_status_flags.jsonl", records)
            failing = self.failing(sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0))
            self.assertEqual(failing, {"ekf2_ev_pos", "ekf2_heading", "gnss_consistency"} if fails else set(), hole_s)

    def test_phases_ok_requires_every_named_phase(self):
        make_hover_run(self.run_dir)
        meta = self.edit_meta()
        self.edit_meta(phases=[ph for ph in meta["phases"] if ph["name"] != "land"])  # ends at hover, all ok
        summary = sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)
        self.assertEqual(self.failing(summary), set())
        self.assertFalse(summary["phases_ok"])
        self.assertEqual(summary["result"], "FAIL")
        for missing in ("wait_ready", "posctl", "arm", "climb", "hover"):
            self.edit_meta(phases=[ph for ph in meta["phases"] if ph["name"] != missing])
            self.assertFalse(sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)["phases_ok"], missing)
        # Hold mode needs its "hold" phase instead of "posctl".
        hold = [dict(ph, name="hold") if ph["name"] == "posctl" else ph for ph in meta["phases"]]
        self.edit_meta(hover_mode="hold", phases=hold)
        self.assertTrue(sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)["phases_ok"])
        self.edit_meta(hover_mode="hold", phases=meta["phases"])
        self.assertFalse(sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)["phases_ok"])
        # The probe needs at least its hover and land phases.
        make_run(self.run_dir)
        probe_meta = json.loads((self.run_dir / "probe_meta.json").read_text(encoding="utf-8"))
        probe_meta["phases"] = probe_meta["phases"][:1]
        (self.run_dir / "probe_meta.json").write_text(json.dumps(probe_meta), encoding="utf-8")
        self.assertFalse(sitl_checks.evaluate_probe(self.run_dir)["phases_ok"])

    def test_drift_needs_valid_and_matched_px4_samples(self):
        def edit(name, predicate, change):
            records = self.read(name)
            for r in records:
                if predicate(r["arrival_ns"]):
                    change(r)
            write_jsonl(self.run_dir / name, records)
        # xy_valid false on 2 % of the window's PX4 samples (1.2 s) fails; 0.5 % (0.3 s) passes.
        for span_ms, fails in ((1200, True), (300, False)):
            make_hover_run(self.run_dir)
            edit("vehicle_local_position.jsonl", lambda t: 70 * S < t <= 70 * S + span_ms * MS,
                 lambda r: r.update(xy_valid=False))
            self.assertEqual(self.failing(sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)),
                             {"drift"} if fails else set(), span_ms)
        # Ground truth missing for 10 % of the window (6 s) fails; 3 % (1.8 s) passes.
        for gap_s, fails in ((6.0, True), (1.8, False)):
            make_hover_run(self.run_dir)
            truth = [r for r in self.read("ground_truth.jsonl")
                     if not 70 * S < r["stamp_ns"] <= 70 * S + int(gap_s * S)]
            write_jsonl(self.run_dir / "ground_truth.jsonl", truth)
            summary = sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)
            self.assertEqual(self.failing(summary), {"drift"} if fails else set(), gap_s)

    def test_resets_are_judged_once_alignment_was_valid(self):
        before_valid = event(20 * S, "px4_bridge", "Px4ResetApplied", "INIT", "INIT", "PX4_RESET_APPLIED")
        summary = self.hover(bridge_events=[before_valid])  # alignment VALID only from 25 s: reported, not judged
        self.assertEqual(self.failing(summary), set())
        self.assertEqual(summary["checks"]["alignment_valid"]["value"]["resets_before_first_valid"], 1)
        after_window = event(116 * S, "px4_bridge", "Px4ResetApplied", "FROZEN", "INVALID",
                             "PX4_RESET_WHILE_FROZEN")
        self.assertEqual(self.failing(self.hover(bridge_events=[after_window])), {"alignment_valid", "f34_values"})

    def test_hover_drift_against_ground_truth(self):
        # PX4 and the truth (ENU y = north = NED x) both move 1 m north: no drift.
        summary = self.hover(px4_north_m=1.0, truth_north_m=1.0)
        self.assertEqual(self.failing(summary), set())
        self.assertAlmostEqual(summary["checks"]["drift"]["value"]["max_m"], 0.0, places=9)
        # PX4 walks 0.4 m north while the vehicle stays: drift fails; 0.25 m passes.
        summary = self.hover(px4_north_m=0.4)
        self.assertEqual(self.failing(summary), {"drift"})
        self.assertAlmostEqual(summary["checks"]["drift"]["value"]["max_m"], 0.4, delta=0.01)
        self.assertEqual(self.failing(self.hover(px4_north_m=0.25)), set())

    def test_hover_f34_values_reported(self):
        summary = self.hover(spawn_yaw_rad=1.2)
        self.assertEqual(self.failing(summary), set())
        value = summary["checks"]["f34_values"]["value"]
        self.assertAlmostEqual(value["yaw_t_mean_rad"], math.pi / 2 - 1.2, places=9)
        self.assertAlmostEqual(value["expected_yaw_t_rad"], math.pi / 2 - 1.2, places=9)
        self.assertAlmostEqual(value["yaw_t_offset_rad"], 0.0, places=9)
        # The spawn yaw the run was started with must match the one judged.
        with self.assertRaises(ValueError):
            sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)

    def shift_yaw_t(self, delta_rad, only_after_s=None):
        records = [json.loads(line) for line in
                   (self.run_dir / "alignment.jsonl").read_text(encoding="utf-8").splitlines()]
        for r in records:
            if only_after_s is None or r["stamp_ns"] >= only_after_s * S:
                r["yaw_rad"] += delta_rad
        write_jsonl(self.run_dir / "alignment.jsonl", records)

    def test_f34_passes_on_any_stable_offset(self):
        # D42: T absorbs whatever heading PX4 chose; the absolute yaw_T is reported, never judged. A steady T at
        # 0 or 21 degrees passes in both profiles, whatever the spawn yaw.
        for profile in ("gps", "no_gps"):
            for yaw_t_deg in (0.0, 21.0):
                summary = self.hover(profile=profile, spawn_yaw_rad=1.2, yaw_t_rad=math.radians(yaw_t_deg))
                self.assertEqual(self.failing(summary), set(), (profile, yaw_t_deg))
                value = summary["checks"]["f34_values"]["value"]
                self.assertAlmostEqual(value["yaw_t_mean_rad"], math.radians(yaw_t_deg), places=9)
                self.assertAlmostEqual(value["yaw_t_spread_rad"], 0.0, places=9)
                self.assertAlmostEqual(value["yaw_t_offset_rad"],
                                       math.remainder(math.radians(yaw_t_deg) - (math.pi / 2 - 1.2), 2 * math.pi),
                                       places=9)
        # A PX4 reset applied while VALID keeps T VALID across it: passes, and the reset is listed.
        reset = event(70 * S, "px4_bridge", "Px4ResetApplied", "VALID", "VALID", "PX4_RESET_APPLIED",
                      sensor_time_s=70.0, heading_reset_step=1.0, delta_heading_rad=0.35)
        summary = self.hover(bridge_events=[reset])
        self.assertEqual(self.failing(summary), set())
        resets = summary["checks"]["f34_values"]["value"]["resets"]
        self.assertEqual([(r["t_s"], r["states"], r["delta_heading_rad"]) for r in resets],
                         [(70.0, "VALID->VALID", 0.35)])

    def test_f34_fails_on_unstable_t(self):
        # A 1.5 degree spread of yaw_T inside the window fails in both profiles; 0.9 degree passes.
        for profile in ("gps", "no_gps"):
            for step_deg, fails in ((1.5, True), (0.9, False)):
                make_hover_run(self.run_dir, profile=profile)
                self.shift_yaw_t(math.radians(step_deg), only_after_s=70)
                summary = sitl_checks.evaluate_hover(self.run_dir, profile, 0.0)
                self.assertEqual(self.failing(summary), {"f34_values"} if fails else set(), (profile, step_deg))
                self.assertAlmostEqual(math.degrees(summary["checks"]["f34_values"]["value"]["yaw_t_spread_rad"]),
                                       step_deg, places=6)
        # T not VALID after a PX4 reset (once it has been VALID in the run) fails F34 even with a steady yaw_T.
        reset_bad = event(70 * S, "px4_bridge", "Px4ResetApplied", "FROZEN", "INVALID", "PX4_RESET_WHILE_FROZEN",
                          sensor_time_s=70.0)
        summary = self.hover(bridge_events=[reset_bad])
        self.assertIn("f34_values", self.failing(summary))
        self.assertEqual(summary["checks"]["f34_values"]["value"]["resets_not_valid_after"], 1)
        # A reset before the first VALID is reported, not judged.
        early = event(20 * S, "px4_bridge", "Px4ResetApplied", "INIT", "INIT", "PX4_RESET_APPLIED", sensor_time_s=20.0)
        summary = self.hover(bridge_events=[early])
        self.assertEqual(self.failing(summary), set())
        self.assertFalse(summary["checks"]["f34_values"]["value"]["resets"][0]["judged"])

    def test_f34_fails_closed_on_missing_data(self):
        make_hover_run(self.run_dir, profile="no_gps")
        (self.run_dir / "alignment.jsonl").unlink()
        summary = sitl_checks.evaluate_hover(self.run_dir, "no_gps", 0.0)
        self.assertIn("f34_values", self.failing(summary))
        self.assertIsNone(summary["checks"]["f34_values"]["value"])
        # F34 needs the bridge events (the resets); it does not read the EKF2 flags in any profile (D42).
        make_hover_run(self.run_dir)
        (self.run_dir / "events_px4_bridge.jsonl").unlink()
        self.assertIn("f34_values", self.failing(sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)))
        for profile in ("gps", "no_gps"):
            make_hover_run(self.run_dir, profile=profile)
            (self.run_dir / "estimator_status_flags.jsonl").unlink()
            self.assertNotIn("f34_values", self.failing(sitl_checks.evaluate_hover(self.run_dir, profile, 0.0)))
        # A non-finite yaw is malformed data, not a pass.
        make_hover_run(self.run_dir)
        records = [json.loads(line) for line in
                   (self.run_dir / "alignment.jsonl").read_text(encoding="utf-8").splitlines()]
        next(r for r in records if r["stamp_ns"] >= 70 * S)["yaw_rad"] = "nan"
        write_jsonl(self.run_dir / "alignment.jsonl", records)
        self.assertIn("f34_values", self.failing(sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)))

    def test_heading_sources_are_reported(self):
        # Compass (3-axis) until 80 s, then EV yaw; one heading reset before the first VALID (25 s) and one at
        # 80 s, after it. Reported, not judged (D42): the run passes whatever source PX4 uses.
        summary = self.hover(heading=[(RUN_START_NS, "none"), (12 * S, "mag_3d"), (80 * S, "ev_yaw")],
                             heading_resets=[(15 * S, 0.02), (80 * S, -0.35)])
        self.assertEqual(self.failing(summary), set())
        value = summary["checks"]["ekf2_heading"]["value"]
        self.assertAlmostEqual(value["window_fractions"]["mag_3d"], 0.5, places=4)
        self.assertAlmostEqual(value["window_fractions"]["ev_yaw"], 0.5, places=4)
        self.assertAlmostEqual(value["yaw_align_fraction"], 1.0, places=9)
        self.assertEqual([(round(r["t_s"], 3), r["delta_heading_rad"], r["after_first_valid"])
                          for r in value["heading_resets"]], [(15.003, 0.02, False), (80.003, -0.35, True)])
        self.assertEqual([(round(c["t_s"], 3), c["source"]) for c in value["timeline"]],
                         [(10.002, "none"), (12.002, "mag_3d"), (80.002, "ev_yaw")])
        # No heading source at all in the window is reported as "none", still not judged.
        summary = self.hover(heading=[(RUN_START_NS, "none")])
        self.assertNotIn("ekf2_heading", self.failing(summary))
        self.assertAlmostEqual(summary["checks"]["ekf2_heading"]["value"]["window_fractions"]["none"], 1.0, places=9)
        # Missing data still fails closed: no reset size (delta_heading) in the PX4 records.
        make_hover_run(self.run_dir)
        records = self.read("vehicle_local_position.jsonl")
        for r in records:
            del r["delta_heading"]
        write_jsonl(self.run_dir / "vehicle_local_position.jsonl", records)
        self.assertEqual(self.failing(sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)), {"ekf2_heading"})

    def test_gnss_dropout_outside_the_outage_fails_gps(self):
        # 5 s of the 60 s window without GNSS fusion and no outage requested: 91.7 % < 95 %, fails.
        summary = self.hover(gnss_gap=(70 * S, 75 * S))
        self.assertEqual(self.failing(summary), {"gnss_consistency"})
        self.assertAlmostEqual(summary["checks"]["gnss_consistency"]["value"]["fraction"], 55 / 60, places=4)
        # The same dropout inside the injected outage is excluded from the judged time.
        outage = {"requested_s": 5.0, "off_ns": 70 * S, "ok_ns": 75 * S, "off_ack": 0, "ok_ack": 0}
        summary = self.hover(gnss_gap=(70 * S, 75 * S), gnss_outage=outage)
        self.assertEqual(self.failing(summary), set())
        value = summary["checks"]["gnss_consistency"]["value"]
        self.assertGreater(value["fraction"], 0.999)
        self.assertAlmostEqual(value["judged_s"], 55.0, places=6)
        self.assertLess(value["outage_fraction"], 0.01)
        # GNSS that stays lost 4 s past the outage is 7.3 % of the judged 55 s: fails.
        self.assertEqual(self.failing(self.hover(gnss_gap=(70 * S, 79 * S), gnss_outage=outage)),
                         {"gnss_consistency"})
        # An outage PX4 did not accept (no ack, or not ACCEPTED) is not an outage: fails closed.
        for change in ({"off_ack": None}, {"ok_ack": 4}):
            summary = self.hover(gnss_outage=dict(outage, **change))
            self.assertEqual(self.failing(summary), {"gnss_consistency"}, change)
            self.assertIn("not accepted", summary["checks"]["gnss_consistency"]["detail"])
        # no_gps has no GNSS check (GNSS is telemetry only there).
        summary = self.hover(profile="no_gps")
        self.assertEqual(set(summary["checks"]), HOVER_CHECKS_NO_GPS)
        self.assertEqual(summary["result"], "PASS")

    def test_ev_variance_is_reported(self):
        summary = self.hover(ev_position_sigma_m=0.03)
        self.assertEqual(self.failing(summary), set())
        value = summary["checks"]["ev_variance"]["value"]
        self.assertAlmostEqual(value["position_sigma_p50_m"], 0.03, places=9)
        self.assertAlmostEqual(value["position_sigma_max_m"], 0.03, places=9)
        self.assertEqual(value["evp_noise_m"], 0.1)
        self.assertAlmostEqual(value["fraction_below_evp_noise"], 1.0, places=9)
        # A sigma above the floor is reported, not judged; the max takes the largest axis.
        make_hover_run(self.run_dir)
        records = self.read("vehicle_visual_odometry.jsonl")
        inside = [r for r in records if H_START_NS <= r["arrival_ns"] <= H_END_NS]
        inside[10]["position_variance"] = [0.0009, 0.0009, 0.25]
        write_jsonl(self.run_dir / "vehicle_visual_odometry.jsonl", records)
        summary = sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)
        self.assertEqual(self.failing(summary), set())
        self.assertAlmostEqual(summary["checks"]["ev_variance"]["value"]["position_sigma_max_m"], 0.5, places=9)
        # A record without its variance is missing data.
        del inside[10]["position_variance"]
        write_jsonl(self.run_dir / "vehicle_visual_odometry.jsonl", records)
        self.assertEqual(self.failing(sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)), {"ev_variance"})

    def test_events_fail_on_publish_failures(self):
        for component, name in (("lio", "LioNodeFailed"), ("lio", "LioPublishFailed"), ("lio", "LioIngestReentered"),
                                ("px4_bridge", "BridgeNodeFailed"), ("px4_bridge", "BridgePublishFailed")):
            record = event(70 * S, component, name, reason="X", count=1.0)
            summary = self.hover(**{"lio_events" if component == "lio" else "bridge_events": [record]})
            self.assertEqual(self.failing(summary), {"events"}, name)
            self.assertEqual(summary["checks"]["events"]["value"]["zero_tolerance"][name], 1)
        summary = self.hover(lio_summary=lio_node_summary(119 * S, publish_failed=1.0))
        self.assertEqual(self.failing(summary), {"events"})


# --- the altitude sweep (Task 13) -------------------------------------------------------------------------------
# A 10..180 s run of /clock time: LIO TRACKING from 22 s, armed 30..170 s; eight steps 2.5..20 m, step k climbs
# from 35 + 13k s for 3 s and holds 10 s; land 139..165 s. Every stream is nominal; the keyword arguments break one.
SWEEP_TARGETS_M = tuple(2.5 * (k + 1) for k in range(8))
SWEEP_CHECKS = {"sweep_steps", "calibration_inputs", "events"}


def sweep_hold(k):
    return (38 + 13 * k) * S, (48 + 13 * k) * S


def make_sweep_run(run_dir, lost_at_ns=None, hold_s=10.0, targets=SWEEP_TARGETS_M, height_error_m=0.0,
                   scans_until_ns=None, phase_ok=True):
    """Write a synthetic altitude sweep. lost_at_ns: LIO LOST then (no scan with eigenvalues after it, as in
    S1b where a LOST LIO stays RESTARTING); scans_until_ns: the last LioScan with eigenvalues."""
    run_dir = pathlib.Path(run_dir)
    steps = []
    for k, target in enumerate(targets):
        hold_start, _ = sweep_hold(k)
        steps.append({"target_m": target, "ok": True, "climb_start_ns": hold_start - 3 * S,
                      "hold_start_ns": hold_start, "hold_end_ns": hold_start + int(hold_s * S),
                      "reached_m": target})

    def height(t):
        h = 0.0
        for k, step in enumerate(steps):
            if t >= step["climb_start_ns"]:
                h = step["target_m"] if t >= step["hold_start_ns"] else (steps[k - 1]["target_m"] if k else 0.0)
        return h if t < 140 * S else max(0.0, h - (t - 140 * S) / 1e9)

    end_ns = 175 * S
    lio_state, alignment, vlp, truth, scans = [], [], [], [], []
    for t in range(20 * S, end_ns, 10 * MS):
        lio_state.append({"arrival_ns": t + 2 * MS, "stamp_ns": t, "epoch": 1, "lio_state": 1, "x": 0.0, "y": 0.0,
                          "z": height(t), "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
                          "vx": 0.0, "vy": 0.0, "vz": 0.0})
    for t in range(22 * S, end_ns, 100 * MS):
        alignment.append({"arrival_ns": t + 30 * MS, "stamp_ns": t, "state": 1, "x_m": 0.0, "y_m": 0.0,
                          "z_m": 0.0, "yaw_rad": math.pi / 2, "age_s": 0.0, "residual_position_m": 0.01,
                          "residual_yaw_rad": 0.001})
        last = lost_at_ns if lost_at_ns is not None else scans_until_ns
        if last is None or t < last:
            h = height(t)
            scans.append(event(t + 20 * MS, "lio", "LioScan", reason="SCAN_GOOD", sensor_time_s=t / 1e9,
                               input_points=8000.0, points=2900.0, translation_min_eigenvalue=5e5 / (1 + h),
                               rotation_min_eigenvalue=2e7 / (1 + h), quality=90.0, degenerate=0.0))
    for t in range(RUN_START_NS, end_ns, 20 * MS):
        h = height(t)
        vlp.append({"arrival_ns": t + 3 * MS, "timestamp_us": (t + MS) // 1000, "timestamp_sample_us": t // 1000,
                    "xy_valid": True, "z_valid": True, "x": 0.0, "y": 0.0, "z": -(h + height_error_m), "vz": 0.0,
                    "heading": math.pi / 2})
        truth.append({"arrival_ns": t + 2 * MS, "stamp_ns": t, "x": 0.0, "y": 0.0, "z": h})
    status = [{"arrival_ns": t + 2 * MS, "timestamp_us": t // 1000, "nav_state": 2,
               "arming_state": 2 if 30 * S <= t <= 165 * S else 1} for t in range(RUN_START_NS, end_ns, 200 * MS)]
    events_lio = [event(22 * S, "lio", "StateTransition", "INITIALIZING", "TRACKING", "CONFIRMATION_REACHED",
                        sensor_time_s=22.0)]
    if lost_at_ns is not None:
        events_lio += [event(lost_at_ns - S, "lio", "StateTransition", "TRACKING", "DEGRADED", "DEGENERATE_SCANS",
                             sensor_time_s=(lost_at_ns - S) / 1e9),
                       event(lost_at_ns, "lio", "StateTransition", "DEGRADED", "LOST", "DEGENERACY_LOST",
                             sensor_time_s=lost_at_ns / 1e9)]
    events_lio = sorted(events_lio + scans, key=lambda r: r["t_ros_ns"])
    events_lio.append(lio_node_summary(176 * S))
    events_bridge = [event(25 * S, "px4_bridge", "AlignmentTransition", "INIT", "VALID", "CONSISTENT_PAIRS",
                           input=0.0, sensor_time_s=25.0), bridge_node_summary(176 * S)]
    for name, records in (("lio_state", lio_state), ("alignment", alignment), ("vehicle_local_position", vlp),
                          ("ground_truth", truth), ("vehicle_status", status), ("events_lio", events_lio),
                          ("events_px4_bridge", events_bridge)):
        write_jsonl(run_dir / f"{name}.jsonl", records)
    phases = [{"name": name, "ok": True, "detail": "", "start_ns": start, "end_ns": end}
              for name, start, end in (("wait_ready", 20 * S, 30 * S), ("posctl", 30 * S, 30 * S),
                                       ("arm", 30 * S, 35 * S), ("sweep", 35 * S, 139 * S),
                                       ("land", 139 * S, 165 * S))]
    phases[-1]["ok"] = phase_ok
    meta = {"profile": "gps", "spawn_yaw_rad": 0.0, "pattern": "altitude_sweep", "hover_mode": "posctl",
            "target_height_m": TARGET_HEIGHT_M, "ground_z_m": 0.0, "hover_window": None, "sweep_steps": steps,
            "phases": phases, "interrupted": False}
    (run_dir / "hover_meta.json").write_text(json.dumps(meta), encoding="utf-8")
    return run_dir


class SitlSweepChecksTest(unittest.TestCase):
    def setUp(self):
        self.run_dir = pathlib.Path(tempfile.mkdtemp(prefix="sitl_sweep_test_"))
        self.addCleanup(shutil.rmtree, self.run_dir, True)

    def failing(self, summary):
        return {name for name, check in summary["checks"].items() if not check["pass"]}

    def sweep(self, **kwargs):
        make_sweep_run(self.run_dir, **kwargs)
        return sitl_checks.evaluate_sweep(self.run_dir, "gps", 0.0)

    def edit_meta(self, change):
        meta = json.loads((self.run_dir / "hover_meta.json").read_text(encoding="utf-8"))
        change(meta)
        (self.run_dir / "hover_meta.json").write_text(json.dumps(meta), encoding="utf-8")

    def test_sweep_pass_on_synthetic_records(self):
        summary = self.sweep()
        self.assertEqual(set(summary["checks"]), SWEEP_CHECKS)
        self.assertEqual(self.failing(summary), set())
        self.assertTrue(summary["phases_ok"])
        self.assertEqual(summary["result"], "PASS")
        self.assertEqual(summary["kind"], "sweep")
        self.assertEqual(json.loads((self.run_dir / "summary.json").read_text(encoding="utf-8")), summary)
        steps = summary["checks"]["sweep_steps"]["value"]["steps"]
        self.assertEqual([s["target_m"] for s in steps], list(SWEEP_TARGETS_M))
        self.assertAlmostEqual(steps[0]["hold_s"], 10.0, places=9)
        self.assertAlmostEqual(steps[0]["max_height_error_m"], 0.0, places=9)
        inputs = summary["checks"]["calibration_inputs"]["value"]
        self.assertIsNone(inputs["first_lost_s"])
        self.assertEqual(inputs["steps_before_lost"], 8)
        self.assertEqual(inputs["scans_per_step"][0], 100)  # 10 s at 10 Hz
        self.assertEqual(sitl_checks.main(["--sweep", str(self.run_dir), "--profile", "gps"]), 0)

    def test_sweep_runs_the_data_checks_only(self):
        # LIO LOST in the 5 m hold (no scan after it) and no EV fusion record at all: the hover criteria are not
        # judged, so the sweep still passes; the steps after the LOST are reported.
        hold_start, _ = sweep_hold(1)
        summary = self.sweep(lost_at_ns=hold_start + 5 * S)
        self.assertEqual(self.failing(summary), set())
        inputs = summary["checks"]["calibration_inputs"]["value"]
        self.assertAlmostEqual(inputs["first_lost_s"], (hold_start + 5 * S) / 1e9, places=9)
        self.assertEqual(inputs["steps_before_lost"], 2)
        self.assertEqual(inputs["scans_per_step"][1], 50)
        self.assertEqual(inputs["scans_per_step"][2:], [0] * 6)
        # LOST 1 s into a hold: that hold is reported (10 scans) but not required to be covered.
        summary = self.sweep(lost_at_ns=hold_start + 1 * S)
        self.assertEqual(self.failing(summary), set())
        inputs = summary["checks"]["calibration_inputs"]["value"]
        self.assertEqual((inputs["steps_before_lost"], inputs["scans_per_step"][1]), (1, 10))
        # LOST before the first hold leaves no calibration data: that fails (closed), it is not a pass.
        summary = self.sweep(lost_at_ns=36 * S)
        self.assertEqual(self.failing(summary), {"calibration_inputs"})

    def test_sweep_missing_data_fails_closed(self):
        for name, expected in (("events_lio.jsonl", {"calibration_inputs", "events"}),
                               ("events_px4_bridge.jsonl", {"events"}),
                               ("ground_truth.jsonl", {"calibration_inputs"}),
                               ("lio_state.jsonl", {"calibration_inputs"}),
                               ("alignment.jsonl", {"calibration_inputs"}),
                               ("vehicle_local_position.jsonl", {"sweep_steps"})):
            make_sweep_run(self.run_dir)
            (self.run_dir / name).unlink()
            summary = sitl_checks.evaluate_sweep(self.run_dir, "gps", 0.0)
            self.assertEqual(self.failing(summary), expected, name)
            self.assertEqual(summary["result"], "FAIL")
        # No step list: both step-based checks fail.
        make_sweep_run(self.run_dir)
        self.edit_meta(lambda m: m.pop("sweep_steps"))
        self.assertEqual(self.failing(sitl_checks.evaluate_sweep(self.run_dir, "gps", 0.0)),
                         {"sweep_steps", "calibration_inputs"})
        # No meta at all: every check fails and the result is FAIL.
        make_sweep_run(self.run_dir)
        (self.run_dir / "hover_meta.json").unlink()
        summary = sitl_checks.evaluate_sweep(self.run_dir, "gps", 0.0)
        self.assertEqual(self.failing(summary), SWEEP_CHECKS - {"events"})
        self.assertEqual(summary["result"], "FAIL")
        # A scan stream that stops before the end of a hold while LIO is not LOST is missing data.
        _, hold_end = sweep_hold(3)
        summary = self.sweep(scans_until_ns=hold_end - 4 * S)
        self.assertEqual(self.failing(summary), {"calibration_inputs"})
        # A hole of more than 2 s in /lio/state inside a hold before the LOST is missing data.
        make_sweep_run(self.run_dir)
        hold_start, _ = sweep_hold(0)
        states = [json.loads(line) for line in
                  (self.run_dir / "lio_state.jsonl").read_text(encoding="utf-8").splitlines()]
        write_jsonl(self.run_dir / "lio_state.jsonl",
                    [r for r in states if not hold_start + 2 * S < r["stamp_ns"] < hold_start + 5 * S])
        self.assertEqual(self.failing(sitl_checks.evaluate_sweep(self.run_dir, "gps", 0.0)), {"calibration_inputs"})
        # Ground truth must cover every hold, also after a LOST (the altitude of each step is reported).
        hold_start, _ = sweep_hold(1)
        make_sweep_run(self.run_dir, lost_at_ns=hold_start + 5 * S)
        last_start, _ = sweep_hold(7)
        truth = [json.loads(line) for line in
                 (self.run_dir / "ground_truth.jsonl").read_text(encoding="utf-8").splitlines()]
        write_jsonl(self.run_dir / "ground_truth.jsonl",
                    [r for r in truth if not last_start + 2 * S < r["stamp_ns"] < last_start + 5 * S])
        self.assertEqual(self.failing(sitl_checks.evaluate_sweep(self.run_dir, "gps", 0.0)), {"calibration_inputs"})
        # T must be known: an alignment that never was VALID before the LOST is missing data.
        make_sweep_run(self.run_dir)
        records = [json.loads(line) for line in
                   (self.run_dir / "alignment.jsonl").read_text(encoding="utf-8").splitlines()]
        write_jsonl(self.run_dir / "alignment.jsonl", [dict(r, state=0) for r in records])
        self.assertEqual(self.failing(sitl_checks.evaluate_sweep(self.run_dir, "gps", 0.0)), {"calibration_inputs"})

    def test_sweep_steps_are_judged(self):
        self.assertEqual(self.failing(self.sweep(hold_s=8.0)), {"sweep_steps"})           # held 8 s, not 10 s
        self.assertEqual(self.failing(self.sweep(targets=SWEEP_TARGETS_M[:7])), {"sweep_steps"})  # 17.5 m top
        self.assertEqual(self.failing(self.sweep(height_error_m=0.6)), {"sweep_steps"})   # PX4 0.6 m off
        self.assertEqual(self.failing(self.sweep(height_error_m=0.4)), set())
        make_sweep_run(self.run_dir)
        self.edit_meta(lambda m: m["sweep_steps"][4].update(ok=False))
        self.assertEqual(self.failing(sitl_checks.evaluate_sweep(self.run_dir, "gps", 0.0)), {"sweep_steps"})
        # The driver phases still decide the result: a failed land fails the run, and so does a missing sweep phase.
        summary = self.sweep(phase_ok=False)
        self.assertEqual((self.failing(summary), summary["phases_ok"], summary["result"]), (set(), False, "FAIL"))
        make_sweep_run(self.run_dir)
        self.edit_meta(lambda m: m.update(phases=[p for p in m["phases"] if p["name"] != "sweep"]))
        self.assertEqual(sitl_checks.evaluate_sweep(self.run_dir, "gps", 0.0)["result"], "FAIL")

    def test_hover_and_sweep_are_not_interchangeable(self):
        make_sweep_run(self.run_dir)
        with self.assertRaisesRegex(ValueError, "altitude_sweep"):
            sitl_checks.evaluate_hover(self.run_dir, "gps", 0.0)
        with self.assertRaisesRegex(ValueError, "gps profile only"):
            sitl_checks.evaluate_sweep(self.run_dir, "no_gps", 0.0)
        make_hover_run(self.run_dir)
        with self.assertRaisesRegex(ValueError, "altitude_sweep"):
            sitl_checks.evaluate_sweep(self.run_dir, "gps", 0.0)
        self.assertEqual(sitl_checks.main(["--sweep", str(self.run_dir), "--profile", "gps"]), 2)


if __name__ == "__main__":
    unittest.main()
