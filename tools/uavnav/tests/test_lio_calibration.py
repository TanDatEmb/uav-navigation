"""Tests for tools.uavnav.lio_calibration (S1b Task 13; SYSTEM_DESIGN §3.2, D20, O6). Synthetic data only."""
import json
import math
import pathlib
import shutil
import tempfile
import unittest

from tools.uavnav import lio_calibration as cal

GRID_STEP = 1.0 / cal.GRID_PER_DECADE  # one grid step in log10 units
T_KNEE, R_KNEE = 2e5, 3e6
GOOD_M, BAD_M = 0.01, 0.2


def sweep(n_windows=600, knee_fraction=0.5, t_knee=T_KNEE, r_knee=R_KNEE, drift=None):
    """A synthetic altitude sweep of n_windows 1 s windows, one scan per window at its middle.

    The altitude grows linearly; both smallest eigenvalues fall exponentially with it and reach their knees at the
    same altitude (knee_fraction of the sweep), as in a real sweep where every direction loses its walls together.
    drift(i, lt, lr) -> metres; default: GOOD_M above both knees, BAD_M below either."""
    a_knee = knee_fraction * n_windows
    scans, windows = [], []
    for i in range(n_windows):
        lt = t_knee * math.exp(-math.log(5.0) * (i - a_knee) / a_knee)    # 1e6 at the start, 2e5 at the knee
        lr = r_knee * math.exp(-math.log(10.0) * (i - a_knee) / a_knee)   # 3e7 at the start, 3e6 at the knee
        t_end = 10.0 + i + 1.0
        scans.append((t_end - 0.5, lt, lr))
        if drift is None:
            d = GOOD_M if lt >= t_knee and lr >= r_knee else BAD_M
        else:
            d = drift(i, lt, lr)
        windows.append((t_end, d))
    return scans, windows


def lio_scan(t_s, lt, lr, reason="SCAN_GOOD", t_ros_ns=None):
    values = {"sensor_time_s": t_s, "input_points": 8000.0, "points": 2900.0, "translation_min_eigenvalue": lt,
              "rotation_min_eigenvalue": lr, "quality": 90.0, "degenerate": 0.0}
    return {"t_ros_ns": int(round(t_s * 1e9)) if t_ros_ns is None else t_ros_ns, "component": "lio",
            "event": "LioScan", "state_before": "", "state_after": "", "reason": reason, "lio_epoch": 1,
            "values": values}


def transition(t_s, before, after, reason):
    return {"t_ros_ns": int(round(t_s * 1e9)), "component": "lio", "event": "StateTransition",
            "state_before": before, "state_after": after, "reason": reason, "lio_epoch": 1,
            "values": {"sensor_time_s": t_s}}


def within_one_step(value, expected):
    return abs(math.log10(value) - math.log10(expected)) <= GRID_STEP + 1e-9


class LioCalibrationTest(unittest.TestCase):
    # --- recommend_thresholds -----------------------------------------------------------------------------------
    def test_synthetic_sweep_gives_the_knees(self):
        scans, windows = sweep()
        result = cal.recommend_thresholds(scans, windows)
        self.assertTrue(within_one_step(result["translation"], T_KNEE), result["translation"])
        self.assertTrue(within_one_step(result["rotation"], R_KNEE), result["rotation"])
        support = result["support"]
        self.assertEqual(support["windows"], 600)
        for block in ("translation", "rotation"):
            self.assertGreaterEqual(support[block]["above"], 50)
            self.assertGreaterEqual(support[block]["below"], 50)
            self.assertGreaterEqual(support[block]["good_fraction_above"], 0.95)
            self.assertLess(support[block]["good_fraction_below"], 0.95)
            self.assertTrue(support[block]["monotone"])
        self.assertGreaterEqual(support["joint_good_fraction"], 0.95)
        # The blocks fall together in a sweep: the report must say they cannot be separated.
        self.assertGreater(support["block_rank_correlation"], 0.9)

    def test_fewer_than_50_windows_per_side_raises(self):
        # 40 windows below the knee: the degenerate side is too thin.
        scans, windows = sweep(n_windows=400, knee_fraction=0.9)
        with self.assertRaisesRegex(ValueError, "insufficient support"):
            cal.recommend_thresholds(scans, windows)
        # 40 windows above the knee: the healthy side is too thin.
        scans, windows = sweep(n_windows=400, knee_fraction=0.1)
        with self.assertRaisesRegex(ValueError, "insufficient support"):
            cal.recommend_thresholds(scans, windows)
        # 50 on each side is enough (the knee stays within a grid step).
        scans, windows = sweep(n_windows=110, knee_fraction=0.5)
        result = cal.recommend_thresholds(scans, windows)
        self.assertTrue(within_one_step(result["translation"], T_KNEE), result["translation"])
        self.assertTrue(within_one_step(result["rotation"], R_KNEE), result["rotation"])

    def test_sweep_that_never_degenerates_is_unsupported(self):
        scans, windows = sweep(drift=lambda i, lt, lr: GOOD_M)
        with self.assertRaisesRegex(ValueError, "insufficient support.*no degenerate region"):
            cal.recommend_thresholds(scans, windows)
        # A threshold is never invented from the lowest level seen: the per-block call refuses too.
        with self.assertRaises(cal.InsufficientSupport):
            cal.block_threshold([(lt, GOOD_M) for _, lt, _ in scans])

    def test_quantile_tolerates_five_percent_above_the_knee(self):
        # 3 % of the healthy windows drift too (every 33rd): still the knee.
        scans, windows = sweep(drift=lambda i, lt, lr: BAD_M if lt < T_KNEE or i % 33 == 0 else GOOD_M)
        result = cal.recommend_thresholds(scans, windows)
        self.assertTrue(within_one_step(result["translation"], T_KNEE), result["translation"])
        # 10 % (every 10th, spread over the whole healthy side): no level keeps 95 %, so no threshold.
        scans, windows = sweep(drift=lambda i, lt, lr: BAD_M if lt < T_KNEE or i % 10 == 0 else GOOD_M)
        with self.assertRaisesRegex(ValueError, "insufficient support"):
            cal.recommend_thresholds(scans, windows)

    def test_non_monotone_drift_curve_moves_the_threshold_above_the_bad_band(self):
        # A band of bad windows high in the healthy region (lt in [5e5, 6e5)): the windows above 5e5 keep < 95 %,
        # so the threshold must rise above the band, and the curve is flagged non-monotone.
        def drift(i, lt, lr):
            return BAD_M if lt < T_KNEE or 5e5 <= lt < 6e5 else GOOD_M
        scans, windows = sweep(n_windows=1200, drift=drift)
        result = cal.block_threshold([(lt, d) for (_, lt, _), (_, d) in zip(scans, windows)])
        self.assertGreaterEqual(result["threshold"], 6e5 / 10 ** GRID_STEP)
        self.assertFalse(result["monotone"])
        # The monotone curve is flagged monotone and its knee is unchanged.
        scans, windows = sweep(n_windows=1200)
        result = cal.block_threshold([(lt, d) for (_, lt, _), (_, d) in zip(scans, windows)])
        self.assertTrue(within_one_step(result["threshold"], T_KNEE), result["threshold"])
        self.assertTrue(result["monotone"])

    def test_a_bad_band_at_the_top_is_not_averaged_away(self):
        # 39 bad windows at the very top (lt >= 9.5e5) are only 3 % of the windows above the knee, so the level at
        # the knee alone would keep 95 %; but every level with 50..780 windows above fails, and above those there is
        # no support left: no threshold.
        scans, windows = sweep(n_windows=2400, drift=lambda i, lt, lr: BAD_M if lt < T_KNEE or lt >= 9.5e5 else GOOD_M)
        with self.assertRaisesRegex(cal.InsufficientSupport, "no level keeps"):
            cal.block_threshold([(lt, d) for (_, lt, _), (_, d) in zip(scans, windows)])

    def test_non_finite_windows_and_scans_are_dropped(self):
        scans, windows = sweep()
        base = cal.recommend_thresholds(scans, windows)
        noisy_windows = windows + [(t + 0.25, float("nan")) for t, _ in windows[:100]]
        noisy_scans = scans + [(t + 0.1, float("nan"), 1e7) for t, _, _ in scans[300:]]
        result = cal.recommend_thresholds(noisy_scans, noisy_windows)
        self.assertEqual(result["translation"], base["translation"])
        self.assertEqual(result["rotation"], base["rotation"])
        self.assertEqual(result["support"]["non_finite_windows"], 100)
        self.assertEqual(result["support"]["non_finite_scans"], 300)
        # A window without any scan carries no eigenvalue and is dropped (counted).
        result = cal.recommend_thresholds(scans[:500], windows)
        self.assertEqual(result["support"]["without_scan"], 100)

    def test_windows_take_the_weakest_scan(self):
        # Two scans in a window: the window carries the smaller eigenvalue of each block.
        scans = [(10.2, 5e5, 9e6), (10.7, 3e5, 2e7)]
        self.assertEqual(cal.window_eigenvalues(scans, [(11.0, 0.01)]), [(11.0, 3e5, 9e6, 0.01)])
        # The window is (end - window_s, end]: a scan exactly at the start belongs to the previous window.
        self.assertEqual(cal.window_eigenvalues([(10.0, 5e5, 9e6)], [(11.0, 0.01)]), [])

    def test_windows_after_the_first_lost_are_excluded(self):
        scans, windows = sweep()
        base = cal.recommend_thresholds(scans, windows)
        # After a LOST at 700 s, healthy-looking eigenvalues with a bad drift (the vehicle on IMU only) follow.
        late_scans = [(700.0 + i + 0.5, 1e6, 3e7) for i in range(200)]
        late_windows = [(701.0 + i, BAD_M) for i in range(200)]
        with self.assertRaisesRegex(ValueError, "insufficient support"):
            cal.recommend_thresholds(scans + late_scans, windows + late_windows)  # without the cut it is ruined
        result = cal.recommend_thresholds(scans + late_scans, windows + late_windows, lost_s=700.0)
        self.assertEqual(result["translation"], base["translation"])
        self.assertEqual(result["support"]["after_lost"], 200)
        # A window that straddles the LOST is excluded too.
        result = cal.recommend_thresholds(scans + [(699.9, 1e6, 3e7)], windows + [(700.5, BAD_M)], lost_s=700.0)
        self.assertEqual(result["support"]["after_lost"], 1)

    def test_rejects_bad_arguments(self):
        scans, windows = sweep()
        for kwargs in ({"max_drift_m": 0.0}, {"max_drift_m": float("nan")}, {"quantile": 1.5},
                       {"quantile": 0.0}):
            with self.assertRaises(ValueError):
                cal.recommend_thresholds(scans, windows, **kwargs)

    # --- scan_eigenvalues ---------------------------------------------------------------------------------------
    def test_scan_eigenvalues_reads_lio_scan_events(self):
        events = [
            lio_scan(5.2, 4e5, 9e6),
            lio_scan(5.1, 3e5, 8e6, reason="SCAN_DEGENERATE"),
            {"t_ros_ns": 1, "component": "lio", "event": "LioScan", "state_before": "", "state_after": "",
             "reason": "BEFORE_IMU_INIT", "lio_epoch": 1, "values": {"sensor_time_s": 0.1}},  # no eigenvalues
            lio_scan(5.3, float("nan"), 9e6),                                                   # not finite
            transition(5.0, "INITIALIZING", "TRACKING", "CONFIRMATION_REACHED"),
        ]
        self.assertEqual(cal.scan_eigenvalues(events), [(5.1, 3e5, 8e6), (5.2, 4e5, 9e6)])
        # Empty scans with a zero-information report keep their 0.0 eigenvalues: they are the weakest scans.
        self.assertEqual(cal.scan_eigenvalues([lio_scan(6.0, 0.0, 0.0, reason="SCAN_DEGENERATE")]),
                         [(6.0, 0.0, 0.0)])

    def test_scan_eigenvalues_stop_at_the_first_lost(self):
        events = [lio_scan(5.0, 4e5, 9e6), transition(6.0, "DEGRADED", "LOST", "DEGENERACY_LOST"),
                  lio_scan(6.0, 4e5, 9e6), lio_scan(7.0, 4e5, 9e6),
                  transition(8.0, "LOST", "RESTARTING", "GEOMETRY_RECOVERED")]
        self.assertEqual(cal.first_lost_s(events), 6.0)
        self.assertEqual(cal.scan_eigenvalues(events), [(5.0, 4e5, 9e6)])
        self.assertIsNone(cal.first_lost_s([lio_scan(5.0, 4e5, 9e6)]))

    # --- drift_windows ------------------------------------------------------------------------------------------
    def test_drift_windows(self):
        # Truth climbs at 1 m/s; LIO follows with a constant frame offset (cancels) and, from 20 s, a 0.1 m/s
        # error along x (0.1 m per 1 s window).
        truth = [(10.0 + 0.02 * k, 1.0, 2.0, 0.02 * k) for k in range(1501)]          # 10..40 s, 50 Hz
        lio = [(10.0 + 0.01 * k, 5.0 + max(0.0, 0.01 * k - 10.0) * 0.1, -3.0, 0.01 * k) for k in range(3001)]
        windows = cal.drift_windows(lio, truth)
        self.assertEqual(len(windows), 30)
        self.assertAlmostEqual(windows[0][0], 11.0, places=9)
        for t_end, d in windows:
            self.assertAlmostEqual(d, 0.0 if t_end <= 20.0 + 1e-9 else 0.1, places=6, msg=t_end)
        # A hole in the LIO stream larger than the bracket limit drops the windows whose ends fall in it.
        holed = [s for s in lio if not 24.6 < s[0] < 25.4]
        ends = [round(t, 6) for t, _ in cal.drift_windows(holed, truth)]
        self.assertEqual(len(ends), 28)
        self.assertNotIn(25.0, ends)  # its end falls in the hole
        self.assertNotIn(26.0, ends)  # its start does
        # Non-finite samples are skipped, not interpolated through.
        broken = list(lio)
        broken[1500] = (broken[1500][0], float("nan"), -3.0, 15.0)
        self.assertTrue(all(math.isfinite(d) for _, d in cal.drift_windows(broken, truth)))
        self.assertEqual(cal.drift_windows([], truth), [])

    def test_lio_in_truth_frame(self):
        # FLU lio_odom -> FRD -> PX4 NED (yaw_T) -> ENU: with spawn yaw 0, yaw_T = pi/2 and LIO x is east.
        out = cal.lio_in_truth_frame([(1.0, 1.0, 0.0, 0.0), (2.0, 0.0, 1.0, 0.0), (3.0, 0.0, 0.0, 1.0)],
                                     math.pi / 2)
        for (t, x, y, z), expected in zip(out, ((1.0, 1.0, 0.0, 0.0), (2.0, 0.0, 1.0, 0.0), (3.0, 0.0, 0.0, 1.0))):
            self.assertEqual(t, expected[0])
            for got, want in zip((x, y, z), expected[1:]):
                self.assertAlmostEqual(got, want, places=12)
        # Spawn yaw 1.2 rad (ENU): yaw_T = pi/2 - 1.2, LIO x points along the vehicle heading.
        (_, x, y, _), = cal.lio_in_truth_frame([(0.0, 1.0, 0.0, 0.0)], math.pi / 2 - 1.2)
        self.assertAlmostEqual(x, math.cos(1.2), places=12)
        self.assertAlmostEqual(y, math.sin(1.2), places=12)

    # --- the run directory ----------------------------------------------------------------------------------------
    def test_analyse_run_fails_closed_on_missing_records(self):
        tmp = pathlib.Path(tempfile.mkdtemp(prefix="lio_calibration_test_"))
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        with self.assertRaises(cal.MissingData):
            cal.analyse_run(tmp)
        (tmp / "hover_meta.json").write_text(json.dumps({"pattern": "hover"}), encoding="utf-8")
        with self.assertRaisesRegex(cal.MissingData, "altitude_sweep"):
            cal.analyse_run(tmp)
        self.assertEqual(cal.main([str(tmp)]), 2)

    def test_analyse_run_on_a_synthetic_sweep_directory(self):
        from tools.uavnav.tests.test_sitl_checks import make_sweep_run, sweep_hold
        tmp = pathlib.Path(tempfile.mkdtemp(prefix="lio_calibration_test_"))
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        # LIO follows the truth exactly and goes LOST in the 5 m hold: per-step rows, nothing after the LOST, and
        # a sweep without drift has no degenerate region, so no pair is recommended (exit 3, the beta pair stays).
        hold_start, _ = sweep_hold(1)
        make_sweep_run(tmp, lost_at_ns=hold_start + 5_000_000_000)
        # After the LOST, T is FROZEN and then may move: only the alignment before the LOST gives T's yaw.
        records = [json.loads(line) for line in (tmp / "alignment.jsonl").read_text(encoding="utf-8").splitlines()]
        with open(tmp / "alignment.jsonl", "w", encoding="utf-8") as f:
            for r in records:
                if r["stamp_ns"] >= hold_start + 5_000_000_000:
                    r["yaw_rad"] = 0.3
                f.write(json.dumps(r) + "\n")
        result = cal.analyse_run(tmp)
        self.assertAlmostEqual(result["first_lost_s"], hold_start / 1e9 + 5.0, places=9)
        self.assertEqual([row["target_m"] for row in result["steps"]], [2.5 * (k + 1) for k in range(8)])
        self.assertEqual([row["scans"] for row in result["steps"]][:3], [100, 50, 0])
        self.assertTrue(result["steps"][2]["after_lost"])
        self.assertAlmostEqual(result["steps"][0]["truth_altitude_m"], 2.5, places=9)
        self.assertAlmostEqual(result["steps"][0]["translation"]["p50"], 5e5 / 3.5, places=3)
        self.assertEqual(result["steps"][0]["drift_m"]["n"], 10)  # the 1 s windows inside the 10 s hold
        self.assertAlmostEqual(result["steps"][0]["drift_m"]["max"], 0.0, places=9)
        self.assertAlmostEqual(result["yaw_t_rad"], math.pi / 2, places=12)
        self.assertIsNone(result["recommendation"])
        self.assertIn("no degenerate region", result["unsupported"])
        self.assertIn("no degenerate region", result["blocks"]["translation"]["unsupported"])
        self.assertEqual(result["imu_gap_total"], 0)
        self.assertEqual(cal.main([str(tmp)]), 3)
        self.assertTrue((tmp / "calibration.json").is_file())


if __name__ == "__main__":
    unittest.main()
