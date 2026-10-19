"""Verdicts of SITL runs from the records a run leaves in its directory. Standard library only.

evaluate_probe(run_dir) judges the Task 3 probe of the SITL assumptions (D33 O20/O24, D29) and
writes run_dir/summary.json with seven checks: <mode>_accepted, cloud_layout, cloud_non_finite_ratio,
imu_rate, scan_stamp_vs_imu, sensor_time_identity, px4_time_identity.

evaluate_hover(run_dir, profile, spawn_yaw_rad) judges the S1 hover (S1b Task 11, S1c Task 1; SYSTEM_DESIGN
§0, §4.1, §7.2; D29, D33 O15/O21/O24, D42) over the hover window (60 s of /clock time) and the whole run, and
writes summary.json with these checks: <mode>_accepted, ekf2_ev_pos, ekf2_heading (reported), lio_tracking,
lio_flap_rate, alignment_valid, alignment_rate, alignment_jump_rejections, drift, lio_state_rate, ev_age,
px4_time_identity, events, f34_values (D42: yaw_T steady within 1 degree and T VALID across every PX4 reset;
the absolute yaw_T is reported only), ev_variance (reported) and, in the gps profile only, gnss_consistency.

evaluate_sweep(run_dir, profile, spawn_yaw_rad) judges the data of the altitude sweep (Task 13; gps only) and
writes summary.json with three data checks: sweep_steps, calibration_inputs and events. The hover criteria are
not judged (LIO is expected to degrade); tools/uavnav/lio_calibration.py analyses the data.

Every check fails when its data is missing, empty or malformed (fail closed, SYSTEM_DESIGN §0): a check
never passes on absent evidence. The result is PASS only when every check passes and every phase of the
driver was ok (phases_ok).

Probe records (written by tools/uavnav/sitl_probe.py; one JSON object per line):

    probe_meta.json               profile, hover_mode, target_height_m, ground_z_m,
                                  hover_window {start_ns, end_ns} (ROS /clock ns) or null, phases
    lidar_points.jsonl            arrival_ns, stamp_ns, newest_imu_stamp_ns, frame_id, point_step,
                                  fields [{name, offset, datatype, count}], n_points, n_nan, n_inf
                                  (newest_imu_stamp_ns is null for a cloud that arrived before any IMU)
    lidar_imu.jsonl               arrival_ns, stamp_ns
    clock.jsonl                   clock_ns
    vehicle_local_position.jsonl  arrival_ns, timestamp_us, timestamp_sample_us, xy_valid, z_valid,
                                  x, y, z, vz (the hover adds heading)
    vehicle_status.jsonl          arrival_ns, timestamp_us, nav_state, arming_state

Hover records (written by tools/uavnav/sitl_hover.py, plus the two event logs of the nodes):

    hover_meta.json               as probe_meta.json, plus spawn_yaw_rad, pattern, interrupted
    lio_state.jsonl               arrival_ns, wall_ns (monotonic), stamp_ns, epoch, lio_state, x, y, z,
                                  qw, qx, qy, qz, vx, vy, vz (velocity in base_link, O21)
    lio_health.jsonl              arrival_ns, stamp_ns, state, epoch, reason_name, correction_age_s,
                                  output_error_{attitude_rad, velocity_mps, position_m}
    lio_odometry.jsonl            arrival_ns, stamp_ns, epoch, reset_counter, quality, x, y, z
    alignment.jsonl               arrival_ns, stamp_ns, state, x_m, y_m, z_m, yaw_rad, age_s,
                                  residual_position_m, residual_yaw_rad
    vehicle_visual_odometry.jsonl arrival_ns, timestamp_us, timestamp_sample_us, position_variance,
                                  velocity_variance, orientation_variance (the EV the bridge sent)
    estimator_status_flags.jsonl  arrival_ns, timestamp_us, cs_ev_pos, cs_ev_yaw, cs_ev_hgt, cs_ev_vel,
                                  cs_gnss_pos, cs_mag_hdg, cs_mag_3d, cs_yaw_align
    vehicle_local_position.jsonl  as the probe's, plus heading, xy/z/heading_reset_counter, delta_heading
    ground_truth.jsonl            arrival_ns, stamp_ns, x, y, z (Gazebo world, ENU)
    events_lio.jsonl, events_px4_bridge.jsonl   the JSONL event logs (tools/uavnav/events.py)

arrival_ns is the newest /clock value the recorder had received when the message arrived; event times
are the records' t_ros_ns (ROS time = /clock with use_sim_time, D29).

CLI:

    python3 tools/uavnav/sitl_checks.py --probe RUN_DIR
    python3 tools/uavnav/sitl_checks.py --hover RUN_DIR --profile gps|no_gps --spawn-yaw-rad RAD
    python3 tools/uavnav/sitl_checks.py --sweep RUN_DIR --profile gps --spawn-yaw-rad RAD
        -> prints the checks, writes RUN_DIR/summary.json; exit 0 PASS, 1 FAIL, 2 usage
"""
import argparse
import bisect
import json
import math
import pathlib
import sys

try:
    from tools.uavnav import events as event_log
except ImportError:  # run as a script: tools/uavnav is sys.path[0]
    import events as event_log  # type: ignore[no-redef]

# sensor_msgs/PointField and px4_msgs/VehicleStatus constants.
FLOAT32 = 7
NAV_STATE = {"posctl": 2, "hold": 4}  # NAVIGATION_STATE_POSCTL, NAVIGATION_STATE_AUTO_LOITER
ARMING_STATE_ARMED = 2

CLOUD_FRAME = "livox_frame"             # O20 / model.sdf gz_frame_id
HOVER_HEIGHT_TOL_M = 0.5                # posctl_accepted: hover within 0.5 m of the climb height
IMU_RATE_HZ, IMU_RATE_TOL_HZ = 200.0, 10.0
SCAN_AHEAD_OF_IMU_P99_MAX_MS = 50.0     # p99 of (scan stamp - newest IMU stamp at arrival) <= +50 ms
SCAN_BEHIND_IMU_P1_MIN_MS = -200.0      # and p1 >= -200 ms: a stale, zeroed or old-epoch scan stamp fails
SENSOR_CLOCK_P99_MAX_MS = 100.0         # p99 of |sensor stamp - /clock at arrival|, IMU and scan separately
SENSOR_AHEAD_MAX_MS = 500.0             # O20: a stamp more than 0.5 s ahead of ROS now is refused
PX4_TIME_P99_MAX_MS = 20.0              # p99 of |vehicle_local_position.timestamp - /clock| (D29)


class MissingData(Exception):
    """A record file or field a check needs is missing, empty or malformed."""


def _reject_constant(name):
    raise ValueError(f"non-standard JSON constant {name}")


def _read_jsonl(path: pathlib.Path) -> list:
    if not path.is_file():
        raise MissingData(f"missing {path.name}")
    records = []
    with open(path, "rb") as f:
        for number, raw in enumerate(f, 1):
            if not raw.strip():
                continue
            try:
                record = json.loads(raw.decode("utf-8"), parse_constant=_reject_constant)
            except ValueError as exc:
                raise MissingData(f"{path.name}:{number}: malformed record: {exc}") from None
            if not isinstance(record, dict):
                raise MissingData(f"{path.name}:{number}: expected a JSON object")
            records.append(record)
    if not records:
        raise MissingData(f"no records in {path.name}")
    return records


def _field(record: dict, key: str, kind, source: str):
    value = record.get(key)
    if kind is int:
        ok = isinstance(value, int) and not isinstance(value, bool)
    elif kind is float:
        ok = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    else:
        ok = isinstance(value, kind)
    if not ok:
        raise MissingData(f"{source}: field {key!r} missing or not {kind.__name__}: {value!r}")
    return value


def percentile(values: list, q: float) -> float:
    """Nearest-rank percentile (q in 0..100) of a non-empty list."""
    ordered = sorted(values)
    rank = max(1, math.ceil(q / 100.0 * len(ordered)))
    return ordered[rank - 1]


class _Loader:
    """Reads each record file once; a failed read is remembered and re-raised for every check using it."""

    def __init__(self, run_dir: pathlib.Path, meta_name: str = "probe_meta.json"):
        self.run_dir = run_dir
        self.meta_name = meta_name
        self._cache = {}

    def cached(self, key: str, load):
        """load() once per key; a MissingData it raises is remembered and re-raised for every later use."""
        if key not in self._cache:
            try:
                self._cache[key] = load()
            except MissingData as exc:
                self._cache[key] = exc
        value = self._cache[key]
        if isinstance(value, MissingData):
            raise value
        return value

    def jsonl(self, name: str) -> list:
        return self.cached(name, lambda: _read_jsonl(self.run_dir / name))

    def meta(self) -> dict:
        if "meta" not in self._cache:
            path = self.run_dir / self.meta_name
            try:
                data = json.loads(path.read_text(encoding="utf-8"), parse_constant=_reject_constant)
                self._cache["meta"] = data if isinstance(data, dict) else MissingData(f"{self.meta_name}: not an object")
            except (OSError, ValueError) as exc:
                self._cache["meta"] = MissingData(f"{self.meta_name}: {exc}")
        value = self._cache["meta"]
        if isinstance(value, MissingData):
            raise value
        return value


def _hover_mode(meta: dict) -> str:
    mode = meta.get("hover_mode", "posctl")
    if mode not in NAV_STATE:
        raise MissingData(f"run meta: unknown hover_mode {mode!r}")
    return mode


def _check_mode_accepted(data: _Loader) -> dict:
    meta = data.meta()
    mode = _hover_mode(meta)
    window = meta.get("hover_window")
    if not isinstance(window, dict):
        raise MissingData("no hover window: the probe never reached the hover phase")
    start = _field(window, "start_ns", int, "hover_window")
    end = _field(window, "end_ns", int, "hover_window")
    if end <= start:
        raise MissingData(f"empty hover window {start}..{end}")
    target = _field(meta, "target_height_m", float, data.meta_name)
    ground_z = _field(meta, "ground_z_m", float, data.meta_name)
    status = [r for r in data.jsonl("vehicle_status.jsonl")
              if start <= _field(r, "arrival_ns", int, "vehicle_status.jsonl") <= end]
    position = [r for r in data.jsonl("vehicle_local_position.jsonl")
                if start <= _field(r, "arrival_ns", int, "vehicle_local_position.jsonl") <= end]
    if not status:
        raise MissingData("no vehicle_status sample in the hover window")
    if not position:
        raise MissingData("no vehicle_local_position sample in the hover window")
    nav_states = sorted({_field(r, "nav_state", int, "vehicle_status.jsonl") for r in status})
    armed = all(_field(r, "arming_state", int, "vehicle_status.jsonl") == ARMING_STATE_ARMED for r in status)
    errors = []
    for r in position:
        if _field(r, "z_valid", bool, "vehicle_local_position.jsonl") is not True:
            raise MissingData("vehicle_local_position z invalid in the hover window")
        errors.append(abs((ground_z - _field(r, "z", float, "vehicle_local_position.jsonl")) - target))
    max_error = max(errors)
    expected = NAV_STATE[mode]
    passed = nav_states == [expected] and armed and max_error <= HOVER_HEIGHT_TOL_M
    return {"pass": passed,
            "value": {"hover_mode": mode, "nav_states": nav_states, "expected_nav_state": expected,
                      "armed_throughout": armed, "max_height_error_m": max_error,
                      "window_s": (end - start) / 1e9, "status_samples": len(status),
                      "position_samples": len(position)},
            "detail": f"nav_state {expected} and armed throughout the hover window, height within "
                      f"{HOVER_HEIGHT_TOL_M} m of {target} m"}


def _clouds(data: _Loader) -> list:
    clouds = data.jsonl("lidar_points.jsonl")
    for r in clouds:
        fields = r.get("fields")
        if not isinstance(fields, list):
            raise MissingData("lidar_points.jsonl: field 'fields' missing")
    return clouds


def _check_cloud_layout(data: _Loader) -> dict:
    clouds = _clouds(data)
    bad = []
    frames = set()
    names = None
    for index, r in enumerate(clouds):
        frame = _field(r, "frame_id", str, "lidar_points.jsonl")
        frames.add(frame)
        by_name = {f.get("name"): f for f in r["fields"] if isinstance(f, dict)}
        layout_ok = all(axis in by_name and by_name[axis].get("datatype") == FLOAT32
                        and by_name[axis].get("count") == 1 for axis in ("x", "y", "z"))
        if not layout_ok or frame != CLOUD_FRAME:
            bad.append(index)
        if names is None:
            names = [{k: f.get(k) for k in ("name", "offset", "datatype", "count")}
                     for f in r["fields"] if isinstance(f, dict)]
    per_point_time = sorted({f.get("name") for r in clouds for f in r["fields"] if isinstance(f, dict)}
                            & {"t", "time", "timestamp", "offset_time"})
    hover_clouds = hover_rate = None
    try:
        window = data.meta().get("hover_window")
        if isinstance(window, dict):
            start = _field(window, "start_ns", int, "hover_window")
            end = _field(window, "end_ns", int, "hover_window")
            if end > start:
                hover_clouds = sum(1 for r in clouds if start <= _field(r, "arrival_ns", int, "lidar_points.jsonl") <= end)
                hover_rate = hover_clouds / ((end - start) / 1e9)
    except MissingData:
        pass  # reported as null; the received rate is information, not a layout criterion
    return {"pass": not bad,
            "value": {"clouds": len(clouds), "bad_clouds": len(bad), "first_bad_index": bad[0] if bad else None,
                      "fields": names, "point_step": clouds[0].get("point_step"), "frames": sorted(frames),
                      "per_point_time_fields": per_point_time,
                      "hover_window_clouds": hover_clouds, "hover_window_rate_hz": hover_rate},
            "detail": f"x, y, z FLOAT32 (count 1) and frame {CLOUD_FRAME!r} in every cloud"}


def _check_cloud_non_finite(data: _Loader) -> dict:
    clouds = _clouds(data)
    total = nan = inf = 0
    ratios = []
    finite_per_cloud = []
    for r in clouds:
        n = _field(r, "n_points", int, "lidar_points.jsonl")
        n_nan = _field(r, "n_nan", int, "lidar_points.jsonl")
        n_inf = _field(r, "n_inf", int, "lidar_points.jsonl")
        total, nan, inf = total + n, nan + n_nan, inf + n_inf
        finite_per_cloud.append(n - n_nan - n_inf)
        if n > 0:
            ratios.append((n_nan + n_inf) / n)
    if total == 0:
        raise MissingData("every cloud is empty")
    empty = sum(1 for k in finite_per_cloud if k <= 0)
    return {"pass": empty == 0,
            "value": {"ratio": (nan + inf) / total, "points": total, "nan": nan, "inf": inf,
                      "min_ratio": min(ratios), "max_ratio": max(ratios), "clouds": len(clouds),
                      "min_finite_points": min(finite_per_cloud), "clouds_without_finite_points": empty},
            "detail": "ratio reported (O20: non-finite points are dropped and counted); fails when any cloud "
                      "has no finite point"}


def _check_imu_rate(data: _Loader) -> dict:
    stamps = [_field(r, "stamp_ns", int, "lidar_imu.jsonl") for r in data.jsonl("lidar_imu.jsonl")]
    if len(stamps) < 2:
        raise MissingData("fewer than 2 IMU samples")
    span_s = (max(stamps) - min(stamps)) / 1e9
    if span_s <= 0.0:
        raise MissingData("IMU stamps do not advance")
    rate = (len(stamps) - 1) / span_s
    gaps = [(b - a) / 1e6 for a, b in zip(stamps, stamps[1:])]
    return {"pass": abs(rate - IMU_RATE_HZ) <= IMU_RATE_TOL_HZ,
            "value": {"rate_hz": rate, "samples": len(stamps), "span_s": span_s,
                      "dt_p50_ms": percentile(gaps, 50), "dt_max_ms": max(gaps),
                      "non_increasing": sum(1 for g in gaps if g <= 0)},
            "detail": f"{IMU_RATE_HZ} +- {IMU_RATE_TOL_HZ} Hz over the IMU header stamps"}


def _check_scan_stamp_vs_imu(data: _Loader) -> dict:
    deltas = []
    before_first_imu = 0
    for r in _clouds(data):
        stamp = _field(r, "stamp_ns", int, "lidar_points.jsonl")
        if "newest_imu_stamp_ns" in r and r["newest_imu_stamp_ns"] is None:
            before_first_imu += 1  # arrived before any IMU sample: no reference, counted and reported
            continue
        newest = _field(r, "newest_imu_stamp_ns", int, "lidar_points.jsonl")
        deltas.append((stamp - newest) / 1e6)
    if not deltas:
        raise MissingData("no cloud arrived after an IMU sample")
    p99, p1 = percentile(deltas, 99), percentile(deltas, 1)
    return {"pass": p99 <= SCAN_AHEAD_OF_IMU_P99_MAX_MS and p1 >= SCAN_BEHIND_IMU_P1_MIN_MS,
            "value": {"p1_ms": p1, "p50_ms": percentile(deltas, 50), "p99_ms": p99, "min_ms": min(deltas),
                      "max_ms": max(deltas), "clouds": len(deltas), "clouds_before_first_imu": before_first_imu},
            "detail": f"(scan stamp - newest IMU stamp at arrival): p99 <= {SCAN_AHEAD_OF_IMU_P99_MAX_MS} ms and "
                      f"p1 >= {SCAN_BEHIND_IMU_P1_MIN_MS} ms"}


def _sensor_offsets(records: list, source: str) -> dict:
    """stamp - /clock at arrival [ms] of one sensor stream: p99 of the magnitude and the largest lead."""
    offsets = [(_field(r, "stamp_ns", int, source) - _field(r, "arrival_ns", int, source)) / 1e6 for r in records]
    magnitudes = [abs(o) for o in offsets]
    return {"samples": len(offsets), "p50_abs_ms": percentile(magnitudes, 50), "p99_abs_ms": percentile(magnitudes, 99),
            "min_ms": min(offsets), "max_ms": max(offsets)}


def _check_sensor_time_identity(data: _Loader) -> dict:
    clock = [_field(r, "clock_ns", int, "clock.jsonl") for r in data.jsonl("clock.jsonl")]
    backwards = sum(1 for a, b in zip(clock, clock[1:]) if b < a)
    imu = _sensor_offsets(data.jsonl("lidar_imu.jsonl"), "lidar_imu.jsonl")
    scan = _sensor_offsets(_clouds(data), "lidar_points.jsonl")
    passed = (backwards == 0
              and all(s["p99_abs_ms"] <= SENSOR_CLOCK_P99_MAX_MS for s in (imu, scan))
              and all(s["max_ms"] <= SENSOR_AHEAD_MAX_MS for s in (imu, scan)))
    return {"pass": passed,
            "value": {"imu": imu, "scan": scan, "clock_samples": len(clock), "clock_backwards": backwards,
                      "p99_abs_max_ms": SENSOR_CLOCK_P99_MAX_MS, "ahead_max_ms": SENSOR_AHEAD_MAX_MS},
            "detail": f"D29: sensor stamp vs /clock at arrival, IMU and scan separately: p99 |stamp - clock| <= "
                      f"{SENSOR_CLOCK_P99_MAX_MS} ms, stamp - clock never above +{SENSOR_AHEAD_MAX_MS} ms (O20), "
                      f"/clock never goes backwards"}


def _check_px4_time_identity(data: _Loader) -> dict:
    offsets = []
    for r in data.jsonl("vehicle_local_position.jsonl"):
        timestamp_ns = _field(r, "timestamp_us", int, "vehicle_local_position.jsonl") * 1000
        offsets.append(abs(timestamp_ns - _field(r, "arrival_ns", int, "vehicle_local_position.jsonl")) / 1e6)
    p99 = percentile(offsets, 99)
    return {"pass": p99 <= PX4_TIME_P99_MAX_MS,
            "value": {"p50_ms": percentile(offsets, 50), "p99_ms": p99, "max_ms": max(offsets),
                      "samples": len(offsets)},
            "detail": f"p99 of |vehicle_local_position.timestamp - /clock| <= {PX4_TIME_P99_MAX_MS} ms (D29)"}


def _run(check, data: _Loader) -> dict:
    try:
        return check(data)
    except MissingData as exc:
        return {"pass": False, "value": None, "detail": f"missing data: {exc}"}


def evaluate_probe(run_dir) -> dict:
    """Judge a probe run and write run_dir/summary.json. Returns the summary."""
    run_dir = pathlib.Path(run_dir)
    data = _Loader(run_dir)
    try:
        meta = data.meta()
        mode = _hover_mode(meta)
    except MissingData:
        meta, mode = {}, "posctl"
    checks = {
        f"{mode}_accepted": _run(_check_mode_accepted, data),
        "cloud_layout": _run(_check_cloud_layout, data),
        "cloud_non_finite_ratio": _run(_check_cloud_non_finite, data),
        "imu_rate": _run(_check_imu_rate, data),
        "scan_stamp_vs_imu": _run(_check_scan_stamp_vs_imu, data),
        "sensor_time_identity": _run(_check_sensor_time_identity, data),
        "px4_time_identity": _run(_check_px4_time_identity, data),
    }
    return _write_summary(run_dir, {"kind": "probe", "run_dir": str(run_dir), "profile": meta.get("profile"),
                                    "hover_mode": mode}, meta, checks)


# --- the S1 hover (Task 11) -------------------------------------------------------------------------------------

LIO_TRACKING, LIO_LOST, ALIGNMENT_VALID = 1, 3, 1  # LioHealth.TRACKING, LioHealth.LOST, Alignment.VALID
HOVER_WINDOW_MIN_S = 60.0                        # the evaluation window: 60 s of /clock time (the driver's hover)
CLOCK_STEP_S = 0.004                             # one Gazebo /clock step: the window ends are quantised to it
COVERAGE_EDGE_MAX_S = 2.0                        # a stream's first/last window record within 2 s of the window ends
COVERAGE_MAX_GAP_S = 2.0                         # and no hole above 2 s (flags ~1 Hz; the others 10-100 Hz)
XY_VALID_MIN_FRACTION = 0.99                     # vehicle_local_position xy_valid in >= 99 % of the window
TRUTH_MATCH_MIN_FRACTION = 0.95                  # >= 95 % of the PX4 samples paired with ground truth (drift)
EV_POS_MIN_FRACTION = 0.95                       # cs_ev_pos in >= 95 % of the window's time (T11 M3: held samples)
F34_YAW_T_SPREAD_MAX_RAD = math.radians(1.0)     # D42: yaw_T max - min over the window <= 1 degree
GNSS_POS_MIN_FRACTION = 0.95                     # gps: cs_gnss_pos in >= 95 % of the window's time
HEADING_SOURCES = ("mag_hdg", "mag_3d", "ev_yaw")  # estimator_status_flags cs_<source>: the heading aiding fused
EKF2_EVP_NOISE_M = 0.1                           # PX4 v1.17 defaults (params_external_vision.yaml, params_gnss.yaml):
EKF2_EVV_NOISE_MPS = 0.1                         # the floors EKF2 puts under the EV variance it receives (SYSTEM_DESIGN
EKF2_EVA_NOISE_RAD = 0.1                         # §4.1); EKF2_GPS_P_NOISE while GNSS is fused. Not profile values:
EKF2_GPS_P_NOISE_M = 0.5                         # reported next to the sigma sent, never judged
LIO_MAX_TRACKING_ENTRIES = 2                     # TRACKING entered at most twice in the run
LIO_MAX_FLAPS_PER_60S = 1.0                      # DEGRADED -> TRACKING per 60 s of the window (owner 2026-10-13, O15)
ALIGNMENT_MIN_RATE_HZ = 9.0                      # /alignment >= 9 Hz
ALIGNMENT_MAX_JUMP_FRACTION = 0.01               # PAIR_REJECTED_JUMP <= 1 % of the pairs (LIO odometry)
DRIFT_MAX_M = 0.3                                # PX4 horizontal drift vs ground truth over the window
TRUTH_MATCH_MAX_NS = 50_000_000                  # a PX4 sample pairs with ground truth stamped within 50 ms
STATE_RATE_HZ, STATE_RATE_TOL_HZ = 100.0, 5.0    # /lio/state 100 +- 5 Hz (O21)
STATE_INTERARRIVAL_P99_MAX_MS = 20.0             # P7: the publisher is not starved
EV_AGE_P99_MAX_MS = 150.0                        # ROS now - timestamp_sample < EKF2_DELAY_MAX (200 ms)
MAX_SCAN_DROP_FRACTION = 0.01                    # LioScanDropped <= 1 % of the scans received
IMU_NOMINAL_HZ = 200.0                           # the Gazebo IMU (probe: 200.01 Hz), the reference of the scan rate
ZERO_TOLERANCE_EVENTS = ("LioQueueOverflow", "LioBackendFailed", "ImuRateTooHigh", "EventsDropped", "LioNodeFailed",
                         "BridgeNodeFailed", "LioPublishFailed", "BridgePublishFailed", "LioIngestReentered")


def _wrap(angle: float) -> float:
    return math.remainder(angle, 2.0 * math.pi)


def _rotate(q, v):
    """Rotate v by the unit quaternion q = (w, x, y, z)."""
    w, x, y, z = q
    tx, ty, tz = 2.0 * (y * v[2] - z * v[1]), 2.0 * (z * v[0] - x * v[2]), 2.0 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx))


def _window(data: _Loader):
    window = data.meta().get("hover_window")
    if not isinstance(window, dict):
        raise MissingData("no hover window: the driver never reached the hover phase")
    start = _field(window, "start_ns", int, "hover_window")
    end = _field(window, "end_ns", int, "hover_window")
    if end <= start:
        raise MissingData(f"empty hover window {start}..{end}")
    if (end - start) / 1e9 < HOVER_WINDOW_MIN_S - CLOCK_STEP_S:
        raise MissingData(f"hover window of {(end - start) / 1e9:.3f} s is shorter than {HOVER_WINDOW_MIN_S} s")
    return start, end


def _windowed(data: _Loader, name: str, key: str = "arrival_ns") -> list:
    """The records of name whose key (ROS ns) lies in the hover window. They must cover it: none, a first or last
    record farther than COVERAGE_EDGE_MAX_S from the window ends, or a hole above COVERAGE_MAX_GAP_S is missing
    data (one sample never stands for the window)."""
    start, end = _window(data)
    records = [r for r in data.jsonl(name) if start <= _field(r, key, int, name) <= end]
    _require_coverage([r[key] for r in records], start, end, f"{name} in the hover window")
    return records


def _require_coverage(times: list, start: int, end: int, what: str) -> None:
    """Sorted times (ns) must cover start..end: at least one, the first/last within COVERAGE_EDGE_MAX_S of the ends
    and no hole above COVERAGE_MAX_GAP_S; otherwise MissingData."""
    if not times:
        raise MissingData(f"no record of {what}")
    holes = [start, *times, end]
    worst_edge = max(times[0] - start, end - times[-1]) / 1e9
    worst_gap = max((b - a for a, b in zip(holes[1:-1], holes[2:-1])), default=0) / 1e9
    if worst_edge > COVERAGE_EDGE_MAX_S or worst_gap > COVERAGE_MAX_GAP_S:
        raise MissingData(f"{what}: not covered, {len(times)} record(s), first/last {worst_edge:.3f} s from an end "
                          f"(max {COVERAGE_EDGE_MAX_S}), largest hole {worst_gap:.3f} s (max {COVERAGE_MAX_GAP_S})")


def _events(data: _Loader, name: str) -> list:
    """A node's event log; a missing file, a torn line or a log without records is missing data."""
    def load():
        skipped = []
        try:
            records = event_log.load(data.run_dir / name, skip_malformed=True, skipped=skipped)
        except OSError as exc:
            raise MissingData(f"missing {name}: {exc.strerror}") from None
        if skipped:
            raise MissingData(f"{name}: {len(skipped)} malformed line(s), first {skipped[0]}")
        if not records:
            raise MissingData(f"no records in {name}")
        for r in records:
            _field(r, "t_ros_ns", int, name)
        return records
    return data.cached("events:" + name, load)


def _named(records: list, event: str, reason=None) -> list:
    return [r for r in records if r["event"] == event and (reason is None or r["reason"] == reason)]


def _event_total(records: list) -> int:
    """Occurrences behind rate-limited records. Every emitter carries a running count that only grows within
    its slot until it restarts at 1: lio, per reason and epoch (the 1st, every kEventSummaryEvery-th and the
    end-of-epoch record); px4_bridge, per episode (the first record has count 1, a summary record carries the
    episode total); node-level count_total, never restarted. So the total is the sum, over the runs of growing
    counts of each slot, of the last (largest) count. A record without a count is one occurrence."""
    totals, open_max = 0, {}
    for r in records:
        values = r["values"]
        count = values.get("count", values.get("count_total"))
        if not isinstance(count, (int, float)) or isinstance(count, bool) or not math.isfinite(count):
            totals += 1
            continue
        slot = (r["component"], r["event"], r["reason"], r.get("lio_epoch"), values.get("input"), values.get("stream"))
        current = open_max.get(slot, 0)
        if current and count <= current:  # the count restarted: a new episode
            totals += current
        open_max[slot] = int(count)
    return totals + sum(open_max.values())


def _summary_values(records: list, event: str, source: str) -> dict:
    found = _named(records, event)
    if not found:
        raise MissingData(f"no {event} in {source} (the process did not shut down cleanly)")
    return found[-1]["values"]


def _in_window(data: _Loader, records: list) -> list:
    start, end = _window(data)
    return [r for r in records if start <= r["t_ros_ns"] <= end]


def _held(data: _Loader, name: str) -> list:
    """The window cut into [(t0, t1, record)]: each record of name holds from its arrival until the next one
    (T11 M3: time, not sample count). The window start takes the last record before it, or the first one in it
    (_windowed bounds that edge by COVERAGE_EDGE_MAX_S)."""
    start, end = _window(data)
    inside = sorted(_windowed(data, name), key=lambda r: r["arrival_ns"])
    before = [r for r in data.jsonl(name) if _field(r, "arrival_ns", int, name) < start]
    first = max(before, key=lambda r: r["arrival_ns"]) if before else inside[0]
    points = [(start, first)] + [(r["arrival_ns"], r) for r in inside]
    return [(t0, t1, r) for (t0, r), (t1, _) in zip(points, points[1:] + [(end, None)]) if t1 > t0]


def _held_fraction(segments: list, holds):
    """(fraction of the time holds(record) is true, the time counted in ns) over the segments."""
    true_ns = total_ns = 0
    for t0, t1, r in segments:
        total_ns += t1 - t0
        true_ns += t1 - t0 if holds(r) else 0
    if total_ns <= 0:
        raise MissingData("no time left to judge")
    return true_ns / total_ns, total_ns


def _flag(key: str):
    return lambda r: _field(r, key, bool, "estimator_status_flags.jsonl")


def _check_ekf2_ev_pos(data: _Loader) -> dict:
    segments = _held(data, "estimator_status_flags.jsonl")
    fraction, _ = _held_fraction(segments, _flag("cs_ev_pos"))
    return {"pass": fraction >= EV_POS_MIN_FRACTION,
            "value": {"fraction": fraction, "samples": len(segments),
                      "cs_ev_hgt_fraction": _held_fraction(segments, _flag("cs_ev_hgt"))[0]},
            "detail": f"cs_ev_pos in >= {EV_POS_MIN_FRACTION:.0%} of the window's time (each estimator_status_flags "
                      "sample holds until the next)"}


def _heading_source(record: dict) -> str:
    fused = [s for s in HEADING_SOURCES if _field(record, f"cs_{s}", bool, "estimator_status_flags.jsonl")]
    return "+".join(fused) or "none"


def _check_ekf2_heading(data: _Loader) -> dict:
    """Reported, not judged (D42: PX4 chooses the heading source): the fraction of the window per EKF2 heading
    source, the source over the whole run, and every EKF2 heading reset with its time and size, before or after
    the first VALID of T. Missing data still fails (MissingData)."""
    segments = _held(data, "estimator_status_flags.jsonl")
    fractions = {}
    for source in sorted({_heading_source(r) for _, _, r in segments}):
        fractions[source] = _held_fraction(segments, lambda r, s=source: _heading_source(r) == s)[0]
    timeline, last = [], None
    for r in sorted(data.jsonl("estimator_status_flags.jsonl"), key=lambda r: _field(
            r, "arrival_ns", int, "estimator_status_flags.jsonl")):
        now = (_heading_source(r), _field(r, "cs_yaw_align", bool, "estimator_status_flags.jsonl"))
        if now != last:
            timeline.append({"t_s": r["arrival_ns"] / 1e9, "source": now[0], "yaw_align": now[1]})
            last = now
    source = "vehicle_local_position.jsonl"
    valid = [_field(r, "arrival_ns", int, "alignment.jsonl") for r in data.jsonl("alignment.jsonl")
             if r.get("state") == ALIGNMENT_VALID]
    first_valid = min(valid, default=None)
    resets, counter = [], None
    positions = sorted(data.jsonl(source), key=lambda r: _field(r, "arrival_ns", int, source))
    for r in positions:
        now = _field(r, "heading_reset_counter", int, source)
        delta = _field(r, "delta_heading", float, source)
        if counter is not None and now != counter:
            resets.append({"t_s": r["arrival_ns"] / 1e9, "counter": now, "delta_heading_rad": delta,
                           "delta_heading_deg": math.degrees(delta),
                           "after_first_valid": first_valid is not None and r["arrival_ns"] >= first_valid})
        counter = now
    return {"pass": True,
            "value": {"window_fractions": fractions,
                      "yaw_align_fraction": _held_fraction(segments, _flag("cs_yaw_align"))[0],
                      "timeline": timeline, "heading_resets": resets,
                      "heading_reset_counter_at_first_record": _field(positions[0], "heading_reset_counter", int,
                                                                      source),
                      "first_valid_s": None if first_valid is None else first_valid / 1e9},
            "detail": "reported (D42: PX4 chooses the heading source): window fraction per source (cs_mag_hdg, "
                      "cs_mag_3d, cs_ev_yaw), the source timeline, every heading reset (vehicle_local_position "
                      "heading_reset_counter, delta_heading) before or after the first VALID of T"}


def _transitions(data: _Loader) -> list:
    return _named(_events(data, "events_lio.jsonl"), "StateTransition")


def _check_lio_tracking(data: _Loader) -> dict:
    health = _windowed(data, "lio_health.jsonl")
    states = sorted({_field(r, "state", int, "lio_health.jsonl") for r in health})
    transitions = _transitions(data)
    in_window = _in_window(data, transitions)
    lost = sum(1 for r in transitions if r["state_after"] == "LOST")
    lost_health = sum(1 for r in data.jsonl("lio_health.jsonl") if r.get("state") == LIO_LOST)
    entries = sum(1 for r in transitions if r["state_after"] == "TRACKING")
    passed = (states == [LIO_TRACKING] and not in_window and lost == 0 and lost_health == 0
              and entries <= LIO_MAX_TRACKING_ENTRIES)
    return {"pass": passed,
            "value": {"window_health_states": states, "window_health_samples": len(health),
                      "window_transitions": [f"{r['state_before']}->{r['state_after']} {r['reason']}"
                                             for r in in_window],
                      "lost_entries": lost, "lost_health_samples": lost_health, "tracking_entries": entries},
            "detail": "TRACKING for the whole window (health and transitions); no LOST in the run; at most "
                      f"{LIO_MAX_TRACKING_ENTRIES} TRACKING entries in the run"}


def _check_lio_flap_rate(data: _Loader) -> dict:
    start, end = _window(data)
    transitions = _transitions(data)
    flaps = [r for r in transitions if r["state_before"] == "DEGRADED" and r["state_after"] == "TRACKING"]
    in_window = len(_in_window(data, flaps))
    per_60s = in_window * 60.0 / ((end - start) / 1e9)
    too_old = _named(_events(data, "events_lio.jsonl"), "ScanResultTooOld")
    ages = [_field(r["values"], "age_s", float, "ScanResultTooOld") for r in too_old]
    applied = None
    try:  # the age at application of the results that reached /lio/odometry (TRACKING only)
        odometry_ages = [(_field(r, "arrival_ns", int, "lio_odometry.jsonl")
                          - _field(r, "stamp_ns", int, "lio_odometry.jsonl")) / 1e9
                         for r in _windowed(data, "lio_odometry.jsonl")]
        applied = {"samples": len(odometry_ages), "p50_s": percentile(odometry_ages, 50),
                   "p99_s": percentile(odometry_ages, 99), "max_s": max(odometry_ages)}
    except MissingData:
        pass  # reported as null; the verdict is the transition rate
    return {"pass": per_60s <= LIO_MAX_FLAPS_PER_60S,
            "value": {"degraded_to_tracking_in_window": in_window, "per_60s": per_60s,
                      "degraded_to_tracking_in_run": len(flaps),
                      "degraded_entries_in_run": sum(1 for r in transitions if r["state_after"] == "DEGRADED"),
                      "scan_result_too_old": {"count": len(ages),
                                              "age_p50_s": percentile(ages, 50) if ages else None,
                                              "age_p99_s": percentile(ages, 99) if ages else None},
                      "applied_result_age": applied},
            "detail": f"<= {LIO_MAX_FLAPS_PER_60S:g} DEGRADED->TRACKING per 60 s of the window (O15 band, owner "
                      "2026-10-13); a larger rate stops for the owner. Ages: ScanResultTooOld.age_s (run) and "
                      "/lio/odometry arrival - stamp (window)"}


def _check_alignment_valid(data: _Loader) -> dict:
    records = _windowed(data, "alignment.jsonl")
    states = sorted({_field(r, "state", int, "alignment.jsonl") for r in records})
    bridge = _events(data, "events_px4_bridge.jsonl")
    stale = [r for r in bridge if r["reason"] == "STALE"]
    changes = _in_window(data, _named(bridge, "AlignmentTransition"))
    resets = _named(bridge, "Px4ResetApplied")
    # A reset is judged once the alignment has been VALID in the run; one while still INIT cannot leave it VALID.
    first_valid = min((r["t_ros_ns"] for r in bridge if r["state_after"] == "VALID"), default=None)
    judged = [r for r in resets if first_valid is not None and r["t_ros_ns"] >= first_valid]
    bad_resets = [r for r in judged if r["state_after"] != "VALID"]
    passed = states == [ALIGNMENT_VALID] and not stale and not changes and not bad_resets
    return {"pass": passed,
            "value": {"window_states": states, "samples": len(records), "stale_events": len(stale),
                      "window_transitions": [f"{r['state_before']}->{r['state_after']} {r['reason']}"
                                             for r in changes],
                      "px4_resets": len(resets), "resets_before_first_valid": len(resets) - len(judged),
                      "resets_not_valid_after": len(bad_resets)},
            "detail": "VALID throughout the window; no STALE in the run; once the alignment has been VALID in the "
                      "run, VALID after every Px4ResetApplied (a reset before the first VALID is reported only)"}


def _check_alignment_rate(data: _Loader) -> dict:
    start, end = _window(data)
    records = _windowed(data, "alignment.jsonl")
    rate = len(records) / ((end - start) / 1e9)
    return {"pass": rate >= ALIGNMENT_MIN_RATE_HZ, "value": {"rate_hz": rate, "samples": len(records)},
            "detail": f"/alignment >= {ALIGNMENT_MIN_RATE_HZ} Hz over the window"}


def _check_alignment_jumps(data: _Loader) -> dict:
    pairs = len(_windowed(data, "lio_odometry.jsonl"))
    rejected = _in_window(data, _named(_events(data, "events_px4_bridge.jsonl"), "AlignmentPairRejected",
                                       "PAIR_REJECTED_JUMP"))
    jumps = _event_total(rejected)
    return {"pass": jumps <= ALIGNMENT_MAX_JUMP_FRACTION * pairs,
            "value": {"jump_rejected": jumps, "pairs": pairs, "fraction": jumps / pairs},
            "detail": f"PAIR_REJECTED_JUMP <= {ALIGNMENT_MAX_JUMP_FRACTION:.0%} of the pairs (/lio/odometry in "
                      "the window; rate-limited records summed per episode)"}


def _check_drift(data: _Loader) -> dict:
    px4 = _windowed(data, "vehicle_local_position.jsonl")
    truth = sorted(((_field(r, "stamp_ns", int, "ground_truth.jsonl"),
                     _field(r, "y", float, "ground_truth.jsonl"),    # ENU y = north = NED x
                     _field(r, "x", float, "ground_truth.jsonl"))    # ENU x = east = NED y
                    for r in data.jsonl("ground_truth.jsonl")))
    stamps = [t for t, _, _ in truth]
    valid = [r for r in px4 if _field(r, "xy_valid", bool, "vehicle_local_position.jsonl") is True]
    if len(valid) < XY_VALID_MIN_FRACTION * len(px4):
        raise MissingData(f"xy_valid in {len(valid)} of {len(px4)} PX4 samples of the window "
                          f"(< {XY_VALID_MIN_FRACTION:.0%})")
    pairs = []
    for r in valid:
        t = _field(r, "timestamp_sample_us", int, "vehicle_local_position.jsonl") * 1000
        i = bisect.bisect_left(stamps, t)
        near = min((j for j in (i - 1, i) if 0 <= j < len(stamps)), key=lambda j: abs(stamps[j] - t), default=None)
        if near is None or abs(stamps[near] - t) > TRUTH_MATCH_MAX_NS:
            continue
        pairs.append((_field(r, "x", float, "vehicle_local_position.jsonl"),
                      _field(r, "y", float, "vehicle_local_position.jsonl"), truth[near][1], truth[near][2]))
    if len(pairs) < TRUTH_MATCH_MIN_FRACTION * len(px4):
        raise MissingData(f"only {len(pairs)} of {len(px4)} PX4 samples have ground truth within 50 ms "
                          f"(< {TRUTH_MATCH_MIN_FRACTION:.0%})")
    x0, y0, n0, e0 = pairs[0]
    drift = [math.hypot((x - x0) - (n - n0), (y - y0) - (e - e0)) for x, y, n, e in pairs]
    return {"pass": max(drift) <= DRIFT_MAX_M,
            "value": {"max_m": max(drift), "final_m": drift[-1], "samples": len(pairs), "window_samples": len(px4)},
            "detail": f"|PX4 xy displacement - ground truth displacement (ENU->NED)| <= {DRIFT_MAX_M} m from the "
                      "window start"}


def _check_lio_state_rate(data: _Loader) -> dict:
    start, end = _window(data)
    states = _windowed(data, "lio_state.jsonl")
    source = "lio_state.jsonl"
    arrivals = [_field(r, "arrival_ns", int, source) for r in states]
    gaps = [(b - a) / 1e6 for a, b in zip(arrivals, arrivals[1:])]
    if not gaps:
        raise MissingData("fewer than 2 /lio/state samples in the window")
    rate = len(states) / ((end - start) / 1e9)
    steps = []
    for a, b in zip(states, states[1:]):
        if a.get("epoch") != b.get("epoch"):
            continue
        dt = (_field(b, "stamp_ns", int, source) - _field(a, "stamp_ns", int, source)) / 1e9
        va = _rotate([_field(a, k, float, source) for k in ("qw", "qx", "qy", "qz")],
                     [_field(a, k, float, source) for k in ("vx", "vy", "vz")])
        vb = _rotate([_field(b, k, float, source) for k in ("qw", "qx", "qy", "qz")],
                     [_field(b, k, float, source) for k in ("vx", "vy", "vz")])
        steps.append(math.dist([_field(b, k, float, source) - _field(a, k, float, source) for k in "xyz"],
                               [0.5 * (va[i] + vb[i]) * dt for i in range(3)]))
    p99 = percentile(gaps, 99)
    walls = [r.get("wall_ns") for r in states]  # T11 M2: the same gaps in wall time, reported as a cross-check of
    wall_gaps = None                            # the /clock-time value (resolution: one Gazebo step, 4 ms)
    if all(isinstance(w, int) and not isinstance(w, bool) for w in walls):
        wall_gaps = [(b - a) / 1e6 for a, b in zip(walls, walls[1:])]
    return {"pass": abs(rate - STATE_RATE_HZ) <= STATE_RATE_TOL_HZ and p99 <= STATE_INTERARRIVAL_P99_MAX_MS,
            "value": {"rate_hz": rate, "samples": len(states), "interarrival_p50_ms": percentile(gaps, 50),
                      "interarrival_p99_ms": p99, "interarrival_max_ms": max(gaps),
                      "interarrival_wall_p50_ms": percentile(wall_gaps, 50) if wall_gaps else None,
                      "interarrival_wall_p99_ms": percentile(wall_gaps, 99) if wall_gaps else None,
                      "f23_step_max_m": max(steps) if steps else None,
                      "f23_step_p99_m": percentile(steps, 99) if steps else None},
            "detail": f"{STATE_RATE_HZ} +- {STATE_RATE_TOL_HZ} Hz and inter-arrival p99 <= "
                      f"{STATE_INTERARRIVAL_P99_MAX_MS} ms (P7, /clock time, resolution one 4 ms step; the wall-time "
                      "inter-arrival is reported as a cross-check); F23 step max |dp - v dt| reported"}


def _check_ev_age(data: _Loader) -> dict:
    source = "vehicle_visual_odometry.jsonl"
    ages = [(_field(r, "arrival_ns", int, source) - _field(r, "timestamp_sample_us", int, source) * 1000) / 1e6
            for r in _windowed(data, source)]
    p99 = percentile(ages, 99)
    return {"pass": p99 <= EV_AGE_P99_MAX_MS,
            "value": {"p50_ms": percentile(ages, 50), "p99_ms": p99, "max_ms": max(ages), "samples": len(ages)},
            "detail": f"ROS now - EV timestamp_sample: p99 <= {EV_AGE_P99_MAX_MS} ms (< EKF2_DELAY_MAX)"}


def _check_hover_px4_time(data: _Loader) -> dict:
    source = "vehicle_local_position.jsonl"
    offsets = [abs(_field(r, "timestamp_us", int, source) * 1000 - _field(r, "arrival_ns", int, source)) / 1e6
               for r in _windowed(data, source)]
    p99 = percentile(offsets, 99)
    return {"pass": p99 <= PX4_TIME_P99_MAX_MS,
            "value": {"p50_ms": percentile(offsets, 50), "p99_ms": p99, "max_ms": max(offsets),
                      "samples": len(offsets)},
            "detail": f"p99 of |vehicle_local_position.timestamp - ROS now| <= {PX4_TIME_P99_MAX_MS} ms (D29)"}


def _check_events(data: _Loader) -> dict:
    lio = _events(data, "events_lio.jsonl")
    bridge = _events(data, "events_px4_bridge.jsonl")
    zero = {name: len(_named(lio, name)) + len(_named(bridge, name)) for name in ZERO_TOLERANCE_EVENTS}
    node = _summary_values(lio, "LioNodeSummary", "events_lio.jsonl")
    bridge_node = _summary_values(bridge, "BridgeNodeSummary", "events_px4_bridge.jsonl")
    keys = ("imu_received", "scans_received", "scans_converted", "points_in", "points_non_finite",
            "rejected_wrong_frame", "rejected_malformed_cloud", "rejected_not_finite", "rejected_bad_stamp",
            "rejected_no_ros_time", "ingest_refused", "publish_failed")
    summary = {k: int(_field(node, k, float, "LioNodeSummary")) for k in keys}
    scans = summary["scans_received"]
    if scans <= 0 or summary["imu_received"] <= 0:
        raise MissingData("LioNodeSummary: no scan or no IMU sample received")
    dropped = len(_named(lio, "LioScanDropped"))
    timed = [r["t_ros_ns"] for r in lio if r["t_ros_ns"] > 0]
    span_s = (max(timed) - min(timed)) / 1e9 if len(timed) > 1 else None
    not_finite = int(_field(bridge_node, "rejected_local_position", float, "BridgeNodeSummary"))
    passed = (all(n == 0 for n in zero.values()) and summary["rejected_wrong_frame"] == 0
              and summary["rejected_malformed_cloud"] == 0 and summary["publish_failed"] == 0
              and dropped <= MAX_SCAN_DROP_FRACTION * scans)
    return {"pass": passed,
            "value": {"zero_tolerance": zero, "lio_scan_dropped": dropped, "scan_drop_fraction": dropped / scans,
                      "imu_gap": _event_total(_named(lio, "ImuGap")),
                      "px4_sample_not_finite": not_finite,
                      "px4_sample_not_finite_events": _event_total(
                          _named(bridge, "Px4SampleRejected", "NOT_FINITE")),
                      "scan_rate_hz_vs_imu": IMU_NOMINAL_HZ * scans / summary["imu_received"],
                      "scan_rate_hz_over_log_span": scans / span_s if span_s else None,
                      "points_per_scan": summary["points_in"] / max(1, summary["scans_converted"]),
                      "non_finite_point_ratio": summary["points_non_finite"] / max(1, summary["points_in"]),
                      "lio_node_summary": summary,
                      "bridge_node_summary": {k: v for k, v in bridge_node.items()}},
            "detail": f"whole run: zero {', '.join(ZERO_TOLERANCE_EVENTS)}; LioScanDropped <= "
                      f"{MAX_SCAN_DROP_FRACTION:.0%} of the scans received; LioNodeSummary rejected_wrong_frame, "
                      "rejected_malformed_cloud and publish_failed 0. Reported: ImuGap count, Px4SampleRejected NOT_FINITE "
                      "(expected 0; BridgeNodeSummary rejected_local_position), the scan rate at the lio node "
                      f"(scans x {IMU_NOMINAL_HZ:g} Hz / IMU samples received, and over the log span)"}


def _check_f34_values(data: _Loader, spawn_yaw_rad: float) -> dict:
    """F34 verdict (D42): T absorbs whatever heading PX4 chose, so T is judged on its stability, never on its value.

    The window is split at every Px4ResetApplied and at every non-VALID alignment output (those outputs are not
    yaw_T samples of a VALID segment). PASS when the spread of yaw_T is <= 1 degree WITHIN each VALID segment, so a
    PX4 heading reset that T absorbs (yaw_T steps once) is correct, and T is VALID after every Px4ResetApplied once
    it has been VALID in the run (a reset before the first VALID is listed, not judged). Reported only: the mean
    yaw_T over the VALID samples, its offset from pi/2 - spawn yaw (the NED heading at LIO start), every reset with
    its heading delta, and every segment. T's state over the window is judged by alignment_valid. A window without
    one VALID output, or non-finite data, is missing data (MissingData)."""
    records = _windowed(data, "alignment.jsonl")
    bridge = _events(data, "events_px4_bridge.jsonl")
    first_valid = min((r["t_ros_ns"] for r in bridge if r["state_after"] == "VALID"), default=None)
    resets = []
    for r in _named(bridge, "Px4ResetApplied"):
        delta = r["values"].get("delta_heading_rad")
        resets.append({"t_s": r["t_ros_ns"] / 1e9, "t_ros_ns": r["t_ros_ns"],
                       "states": f"{r['state_before']}->{r['state_after']}", "reason": r["reason"],
                       "judged": first_valid is not None and r["t_ros_ns"] >= first_valid,
                       "delta_heading_rad": delta, "heading_reset_step": r["values"].get("heading_reset_step")})
    not_valid = [r for r in resets if r["judged"] and not r["states"].endswith("->VALID")]
    cuts = sorted(r["t_ros_ns"] for r in resets)
    runs, previous = [], None   # [[reset index, [(arrival_ns, yaw_rad)]]]: consecutive VALID outputs between cuts
    for r in sorted(records, key=lambda r: r["arrival_ns"]):
        yaw = _field(r, "yaw_rad", float, "alignment.jsonl")
        if _field(r, "state", int, "alignment.jsonl") != ALIGNMENT_VALID:
            previous = None
            continue
        cell = bisect.bisect_right(cuts, r["arrival_ns"])
        if previous != cell:
            runs.append([cell, []])
            previous = cell
        runs[-1][1].append((r["arrival_ns"], yaw))
    if not runs:
        raise MissingData("no VALID alignment output in the hover window")
    segments = []
    for _, samples in runs:
        base = samples[0][1]
        deltas = [_wrap(y - base) for _, y in samples]
        segments.append({"t0_s": samples[0][0] / 1e9, "t1_s": samples[-1][0] / 1e9, "samples": len(samples),
                         "spread_rad": max(deltas) - min(deltas), "mean_rad": _wrap(base + sum(deltas) / len(deltas))})
    spread = max(s["spread_rad"] for s in segments)
    total = sum(s["samples"] for s in segments)
    base = segments[0]["mean_rad"]
    mean = _wrap(base + sum(_wrap(s["mean_rad"] - base) * s["samples"] for s in segments) / total)
    expected = _wrap(math.pi / 2.0 - spawn_yaw_rad)
    return {"pass": spread <= F34_YAW_T_SPREAD_MAX_RAD and not not_valid,
            "value": {"yaw_t_spread_rad": spread, "yaw_t_mean_rad": mean, "expected_yaw_t_rad": expected,
                      "yaw_t_offset_rad": _wrap(mean - expected), "spawn_yaw_rad": spawn_yaw_rad,
                      "samples": total, "non_valid_samples": len(records) - total, "segments": segments,
                      "resets": resets, "resets_not_valid_after": len(not_valid)},
            "detail": "D42: window split at every Px4ResetApplied and every non-VALID alignment output; yaw_T max - min "
                      "<= 1 degree within each VALID segment, and T VALID after every Px4ResetApplied once it has "
                      "been VALID in the run; the absolute yaw_T and its offset from pi/2 - spawn yaw are reported "
                      "only (T's state over the window is judged by alignment_valid)"}


def _check_gnss_consistency(data: _Loader) -> dict:
    """gps only: cs_gnss_pos in >= 95 % of the window's time. GNSS loss is not injected in this SITL (S1c Task 1:
    PX4 ignores VEHICLE_CMD_INJECT_FAILURE), so there is no outage window to exclude."""
    fraction, judged_ns = _held_fraction(_held(data, "estimator_status_flags.jsonl"), _flag("cs_gnss_pos"))
    return {"pass": fraction >= GNSS_POS_MIN_FRACTION, "value": {"fraction": fraction, "judged_s": judged_ns / 1e9},
            "detail": f"gps: cs_gnss_pos in >= {GNSS_POS_MIN_FRACTION:.0%} of the window's time"}


def _check_ev_variance(data: _Loader) -> dict:
    """Reported, not judged: the EV sigma the bridge sent in the window against the floors EKF2 puts under it
    (EKF2_EVP_NOISE, EKF2_EVV_NOISE, EKF2_EVA_NOISE; EKF2_GPS_P_NOISE while GNSS is fused). Missing data fails."""
    source = "vehicle_visual_odometry.jsonl"

    def sigmas(record, key):
        values = record.get(key)
        if not isinstance(values, list) or len(values) != 3:
            raise MissingData(f"{source}: field {key!r} missing or not 3 values: {values!r}")
        return [math.sqrt(_field({"v": v}, "v", float, f"{source} {key}")) for v in values]
    records = _windowed(data, source)
    position = [max(sigmas(r, "position_variance")) for r in records]
    per_axis = list(zip(*(sigmas(r, "position_variance") for r in records)))
    velocity = [max(sigmas(r, "velocity_variance")) for r in records]
    yaw = [sigmas(r, "orientation_variance")[2] for r in records]
    return {"pass": True,
            "value": {"position_sigma_p50_m": percentile(position, 50), "position_sigma_max_m": max(position),
                      "position_sigma_p50_per_axis_m": [percentile(list(a), 50) for a in per_axis],
                      "velocity_sigma_p50_mps": percentile(velocity, 50), "yaw_sigma_p50_rad": percentile(yaw, 50),
                      "evp_noise_m": EKF2_EVP_NOISE_M, "evv_noise_mps": EKF2_EVV_NOISE_MPS,
                      "eva_noise_rad": EKF2_EVA_NOISE_RAD, "gps_p_noise_m": EKF2_GPS_P_NOISE_M,
                      "fraction_below_evp_noise": sum(1 for p in position if p < EKF2_EVP_NOISE_M) / len(position),
                      "samples": len(records)},
            "detail": "reported: EV position sigma (largest axis) p50 and max against the PX4 floor EKF2_EVP_NOISE "
                      "(PX4 default; EKF2_GPS_P_NOISE while GNSS is fused), velocity and yaw sigma p50"}


def evaluate_hover(run_dir, profile: str, spawn_yaw_rad: float) -> dict:
    """Judge an S1 hover run and write run_dir/summary.json. Returns the summary.

    Raises ValueError when profile or spawn_yaw_rad differ from what the run recorded in hover_meta.json."""
    if profile not in ("gps", "no_gps") or not math.isfinite(spawn_yaw_rad):
        raise ValueError(f"profile must be gps or no_gps and the spawn yaw finite: {profile!r} {spawn_yaw_rad!r}")
    run_dir = pathlib.Path(run_dir)
    data = _Loader(run_dir, "hover_meta.json")
    try:
        meta = data.meta()
        mode = _hover_mode(meta)
    except MissingData:
        meta, mode = {}, "posctl"
    if meta.get("pattern", "hover") != "hover":
        raise ValueError(f"the run recorded pattern {meta['pattern']!r}, not a hover (judge an altitude_sweep "
                         "with --sweep)")
    _require_same_run(meta, profile, spawn_yaw_rad)
    checks = {
        f"{mode}_accepted": _run(_check_mode_accepted, data),
        "ekf2_ev_pos": _run(_check_ekf2_ev_pos, data),
        "ekf2_heading": _run(_check_ekf2_heading, data),
        "lio_tracking": _run(_check_lio_tracking, data),
        "lio_flap_rate": _run(_check_lio_flap_rate, data),
        "alignment_valid": _run(_check_alignment_valid, data),
        "alignment_rate": _run(_check_alignment_rate, data),
        "alignment_jump_rejections": _run(_check_alignment_jumps, data),
        "drift": _run(_check_drift, data),
        "lio_state_rate": _run(_check_lio_state_rate, data),
        "ev_age": _run(_check_ev_age, data),
        "px4_time_identity": _run(_check_hover_px4_time, data),
        "events": _run(_check_events, data),
        "f34_values": _run(lambda d: _check_f34_values(d, spawn_yaw_rad), data),
        "ev_variance": _run(_check_ev_variance, data),
    }
    if profile == "gps":
        checks["gnss_consistency"] = _run(_check_gnss_consistency, data)
    return _write_summary(run_dir, {
        "kind": "hover", "run_dir": str(run_dir), "profile": profile, "spawn_yaw_rad": spawn_yaw_rad,
        "pattern": meta.get("pattern"), "hover_mode": mode}, meta, checks)


def _require_same_run(meta: dict, profile: str, spawn_yaw_rad: float) -> None:
    if "profile" in meta and meta["profile"] != profile:
        raise ValueError(f"the run recorded profile {meta['profile']!r}, not {profile!r}")
    recorded_yaw = meta.get("spawn_yaw_rad")
    if isinstance(recorded_yaw, (int, float)) and abs(recorded_yaw - spawn_yaw_rad) > 1e-9:
        raise ValueError(f"the run recorded spawn yaw {recorded_yaw!r} rad, not {spawn_yaw_rad!r}")


# --- the altitude sweep (Task 13) -------------------------------------------------------------------------------
# Data checks only: the sweep feeds tools/uavnav/lio_calibration.py, and LIO is expected to degrade and may go
# LOST as the walls leave the view (F36), so the hover criteria (TRACKING, EV fusion, alignment, drift) are not
# judged. Every check still fails when its data is missing (fail closed).

SWEEP_TARGETS_M = tuple(2.5 * (k + 1) for k in range(8))   # climb in 2.5 m steps to 20 m (plan Task 13)
SWEEP_HOLD_MIN_S = 10.0                                    # each step held 10 s of /clock time


def _sweep_steps(data: _Loader) -> list:
    steps = data.meta().get("sweep_steps")
    if not isinstance(steps, list) or not steps:
        raise MissingData(f"no sweep_steps in {data.meta_name}: the driver never flew the sweep")
    out = []
    for step in steps:
        if not isinstance(step, dict):
            raise MissingData(f"sweep step is not an object: {step!r}")
        row = {"target_m": _field(step, "target_m", float, "sweep_steps"),
               "ok": _field(step, "ok", bool, "sweep_steps"),
               "start_ns": _field(step, "hold_start_ns", int, "sweep_steps"),
               "end_ns": _field(step, "hold_end_ns", int, "sweep_steps")}
        if row["end_ns"] <= row["start_ns"]:
            raise MissingData(f"empty hold of the {row['target_m']} m step")
        out.append(row)
    return out


def _check_sweep_steps(data: _Loader) -> dict:
    steps = _sweep_steps(data)
    ground_z = _field(data.meta(), "ground_z_m", float, data.meta_name)
    source = "vehicle_local_position.jsonl"
    positions = data.jsonl(source)
    rows = []
    for step in steps:
        inside = [r for r in positions if step["start_ns"] <= _field(r, "arrival_ns", int, source) <= step["end_ns"]]
        _require_coverage([r["arrival_ns"] for r in inside], step["start_ns"], step["end_ns"],
                          f"{source} in the {step['target_m']} m hold")
        errors = []
        for r in inside:
            if _field(r, "z_valid", bool, source) is not True:
                raise MissingData(f"vehicle_local_position z invalid in the {step['target_m']} m hold")
            errors.append(abs((ground_z - _field(r, "z", float, source)) - step["target_m"]))
        rows.append({"target_m": step["target_m"], "ok": step["ok"],
                     "hold_s": (step["end_ns"] - step["start_ns"]) / 1e9,
                     "max_height_error_m": max(errors), "samples": len(inside)})
    targets_ok = (len(rows) == len(SWEEP_TARGETS_M)
                  and all(abs(r["target_m"] - t) <= 1e-9 for r, t in zip(rows, SWEEP_TARGETS_M)))
    passed = targets_ok and all(r["ok"] and r["hold_s"] >= SWEEP_HOLD_MIN_S - CLOCK_STEP_S
                                and r["max_height_error_m"] <= HOVER_HEIGHT_TOL_M for r in rows)
    return {"pass": passed, "value": {"steps": rows},
            "detail": f"steps {SWEEP_TARGETS_M[0]}..{SWEEP_TARGETS_M[-1]} m every 2.5 m, each ok, held >= "
                      f"{SWEEP_HOLD_MIN_S} s of /clock with the PX4 height within {HOVER_HEIGHT_TOL_M} m"}


def _lio_scan_eigen_times(records: list) -> list:
    """Sensor times (ns) of the LioScan events carrying both eigenvalues as finite numbers (the calibration input)."""
    times = []
    for r in _named(records, "LioScan"):
        values = r["values"]
        if all(isinstance(values.get(k), (int, float)) and not isinstance(values.get(k), bool)
               and math.isfinite(values[k]) for k in ("translation_min_eigenvalue", "rotation_min_eigenvalue")):
            times.append(_event_sensor_ns(r))
    return sorted(times)


def _event_sensor_ns(record: dict) -> int:
    t = record["values"].get("sensor_time_s")
    if isinstance(t, (int, float)) and not isinstance(t, bool) and math.isfinite(t):
        return int(round(t * 1e9))
    return record["t_ros_ns"]  # SITL: sensor time = ROS time (D29)


def _check_calibration_inputs(data: _Loader) -> dict:
    steps = _sweep_steps(data)
    lio = _events(data, "events_lio.jsonl")
    lost = min((_event_sensor_ns(r) for r in _named(lio, "StateTransition") if r["state_after"] == "LOST"),
               default=None)
    scans = _lio_scan_eigen_times(lio)
    states = sorted(_field(r, "stamp_ns", int, "lio_state.jsonl") for r in data.jsonl("lio_state.jsonl"))
    truth = sorted(_field(r, "stamp_ns", int, "ground_truth.jsonl") for r in data.jsonl("ground_truth.jsonl"))
    valid = [r for r in data.jsonl("alignment.jsonl")
             if r.get("state") == ALIGNMENT_VALID and _field(r, "stamp_ns", int, "alignment.jsonl") >= 0
             and (lost is None or r["stamp_ns"] < lost)]
    for r in valid:
        _field(r, "yaw_rad", float, "alignment.jsonl")
    if not valid:
        raise MissingData("no VALID alignment sample before the first LOST: T is unknown")
    per_step, before_lost = [], 0
    for step in steps:
        start, end = step["start_ns"], step["end_ns"]
        _require_coverage([t for t in truth if start <= t <= end], start, end,
                          f"ground_truth.jsonl in the {step['target_m']} m hold")
        cut = end if lost is None else min(end, lost)
        in_hold = [t for t in scans if start <= t < cut]
        per_step.append(len(in_hold))
        if cut - start <= COVERAGE_EDGE_MAX_S * 1e9:
            continue  # LOST before or right after the hold started: reported (0 or a few scans), not required
        before_lost += 1
        _require_coverage(in_hold, start, cut, f"LioScan eigenvalues in the {step['target_m']} m hold")
        _require_coverage([t for t in states if start <= t <= cut], start, cut,
                          f"lio_state.jsonl in the {step['target_m']} m hold")
    if before_lost == 0:
        raise MissingData("LIO went LOST before the first hold: no calibration data")
    return {"pass": True,
            "value": {"first_lost_s": None if lost is None else lost / 1e9, "steps_before_lost": before_lost,
                      "scans_per_step": per_step, "scans_with_eigenvalues": len(scans),
                      "alignment_valid_before_lost": len(valid)},
            "detail": "every hold before the first LOST is covered by LioScan eigenvalues and /lio/state; every hold "
                      "by ground truth; T known (a VALID alignment before the LOST). Steps after the LOST are "
                      "reported only (S1b: a LOST LIO stays RESTARTING)"}


def evaluate_sweep(run_dir, profile: str, spawn_yaw_rad: float) -> dict:
    """Judge the data of an altitude sweep run (Task 13) and write run_dir/summary.json. Returns the summary.

    Raises ValueError for a profile other than gps (the sweep is flown with GNSS, because EV is lost when LIO
    degrades), a run that is not an altitude_sweep, or a profile/spawn yaw different from the recorded one."""
    if profile != "gps" or not math.isfinite(spawn_yaw_rad):
        raise ValueError(f"the altitude sweep is judged in the gps profile only, with a finite spawn yaw: "
                         f"{profile!r} {spawn_yaw_rad!r}")
    run_dir = pathlib.Path(run_dir)
    data = _Loader(run_dir, "hover_meta.json")
    try:
        meta = data.meta()
        mode = _hover_mode(meta)
    except MissingData:
        meta, mode = {}, "posctl"
    if meta and meta.get("pattern") != "altitude_sweep":
        raise ValueError(f"the run recorded pattern {meta.get('pattern')!r}, not altitude_sweep")
    _require_same_run(meta, profile, spawn_yaw_rad)
    checks = {
        "sweep_steps": _run(_check_sweep_steps, data),
        "calibration_inputs": _run(_check_calibration_inputs, data),
        "events": _run(_check_events, data),
    }
    return _write_summary(run_dir, {
        "kind": "sweep", "run_dir": str(run_dir), "profile": profile, "spawn_yaw_rad": spawn_yaw_rad,
        "pattern": "altitude_sweep", "hover_mode": mode}, meta, checks)


def _required_phases(kind: str, mode: str) -> set:
    """The phases a complete run records: the probe at least hover and land; the hover and sweep drivers all of
    their own."""
    if kind == "probe":
        return {"hover", "land"}
    if kind == "sweep":
        return {"wait_ready", mode, "arm", "sweep", "land"}
    return {"wait_ready", mode, "arm", "climb", "hover", "land"}


def _write_summary(run_dir: pathlib.Path, head: dict, meta: dict, checks: dict) -> dict:
    phases = meta.get("phases")
    phases_ok = (isinstance(phases, list) and bool(phases)
                 and all(isinstance(ph, dict) and ph.get("ok") is True for ph in phases)
                 and _required_phases(head["kind"], head["hover_mode"]) <= {ph.get("name") for ph in phases}
                 and meta.get("interrupted") is not True)
    summary = dict(head, phases=phases, phases_ok=phases_ok,
                   result="PASS" if phases_ok and all(c["pass"] for c in checks.values()) else "FAIL",
                   checks=checks)
    tmp = run_dir / "summary.json.tmp"
    tmp.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(run_dir / "summary.json")
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Judge a SITL run directory and write summary.json.")
    kind = parser.add_mutually_exclusive_group(required=True)
    kind.add_argument("--probe", type=pathlib.Path, metavar="RUN_DIR",
                      help="probe run directory (log/sitl/<stamp>-<profile>-probe)")
    kind.add_argument("--hover", type=pathlib.Path, metavar="RUN_DIR",
                      help="hover run directory (log/sitl/<stamp>-<profile>-hover); needs --profile")
    kind.add_argument("--sweep", type=pathlib.Path, metavar="RUN_DIR",
                      help="altitude sweep run directory (log/sitl/<stamp>-gps-altitude_sweep); needs --profile gps")
    parser.add_argument("--profile", choices=("gps", "no_gps"), help="hover, sweep: the PX4 profile of the run")
    parser.add_argument("--spawn-yaw-rad", type=float, default=0.0,
                        help="hover, sweep: the spawn yaw of the run (ENU)")
    args = parser.parse_args(argv)
    run_dir = args.probe or args.hover or args.sweep
    if not run_dir.is_dir():
        print(f"sitl_checks: not a directory: {run_dir}", file=sys.stderr)
        return 2
    if args.probe:
        summary = evaluate_probe(run_dir)
    else:
        if args.profile is None:
            parser.error("--hover and --sweep need --profile")
        evaluate = evaluate_hover if args.hover else evaluate_sweep
        try:
            summary = evaluate(run_dir, args.profile, args.spawn_yaw_rad)
        except ValueError as exc:
            print(f"sitl_checks: {exc}", file=sys.stderr)
            return 2
    for name, check in summary["checks"].items():
        print(f"{'PASS' if check['pass'] else 'FAIL'}  {name}: {json.dumps(check['value'])}")
    print(f"{'PASS' if summary['phases_ok'] else 'FAIL'}  phases_ok: "
          f"{json.dumps([(ph.get('name'), ph.get('ok')) for ph in summary['phases'] or [] if isinstance(ph, dict)])}")
    print(f"SITL_{summary['kind'].upper()}_RESULT={summary['result']} ({run_dir / 'summary.json'})")
    return 0 if summary["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
