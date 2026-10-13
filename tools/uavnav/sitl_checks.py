"""Verdicts of SITL runs from the records a run leaves in its directory. Standard library only.

evaluate_probe(run_dir) judges the Task 3 probe of the SITL assumptions (D33 O20/O24, D29) and
writes run_dir/summary.json with seven checks: <mode>_accepted, cloud_layout, cloud_non_finite_ratio,
imu_rate, scan_stamp_vs_imu, sensor_time_identity, px4_time_identity. Every check fails when its
data is missing, empty or malformed (fail closed, SYSTEM_DESIGN §0): a check never passes on absent
evidence. The result is PASS only when every check passes and every probe phase was ok (phases_ok).

Records (written by tools/uavnav/sitl_probe.py; one JSON object per line):

    probe_meta.json               profile, hover_mode, target_height_m, ground_z_m,
                                  hover_window {start_ns, end_ns} (ROS /clock ns) or null, phases
    lidar_points.jsonl            arrival_ns, stamp_ns, newest_imu_stamp_ns, frame_id, point_step,
                                  fields [{name, offset, datatype, count}], n_points, n_nan, n_inf
                                  (newest_imu_stamp_ns is null for a cloud that arrived before any IMU)
    lidar_imu.jsonl               arrival_ns, stamp_ns
    clock.jsonl                   clock_ns
    vehicle_local_position.jsonl  arrival_ns, timestamp_us, timestamp_sample_us, xy_valid, z_valid,
                                  x, y, z, vz
    vehicle_status.jsonl          arrival_ns, timestamp_us, nav_state, arming_state

arrival_ns is the newest /clock value the probe had received when the message arrived.

CLI:

    python3 tools/uavnav/sitl_checks.py --probe RUN_DIR   -> prints summary.json; exit 0 PASS, 1 FAIL
"""
import argparse
import json
import math
import pathlib
import sys

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

    def __init__(self, run_dir: pathlib.Path):
        self.run_dir = run_dir
        self._cache = {}

    def jsonl(self, name: str) -> list:
        if name not in self._cache:
            try:
                self._cache[name] = _read_jsonl(self.run_dir / name)
            except MissingData as exc:
                self._cache[name] = exc
        value = self._cache[name]
        if isinstance(value, MissingData):
            raise value
        return value

    def meta(self) -> dict:
        if "meta" not in self._cache:
            path = self.run_dir / "probe_meta.json"
            try:
                data = json.loads(path.read_text(encoding="utf-8"), parse_constant=_reject_constant)
                self._cache["meta"] = data if isinstance(data, dict) else MissingData("probe_meta.json: not an object")
            except (OSError, ValueError) as exc:
                self._cache["meta"] = MissingData(f"probe_meta.json: {exc}")
        value = self._cache["meta"]
        if isinstance(value, MissingData):
            raise value
        return value


def _hover_mode(meta: dict) -> str:
    mode = meta.get("hover_mode", "posctl")
    if mode not in NAV_STATE:
        raise MissingData(f"probe_meta.json: unknown hover_mode {mode!r}")
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
    target = _field(meta, "target_height_m", float, "probe_meta.json")
    ground_z = _field(meta, "ground_z_m", float, "probe_meta.json")
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
    phases = meta.get("phases")
    phases_ok = (isinstance(phases, list) and bool(phases)
                 and all(isinstance(ph, dict) and ph.get("ok") is True for ph in phases)
                 and meta.get("interrupted") is not True)
    summary = {
        "kind": "probe",
        "run_dir": str(run_dir),
        "profile": meta.get("profile"),
        "hover_mode": mode,
        "phases": phases,
        "phases_ok": phases_ok,
        "result": "PASS" if phases_ok and all(c["pass"] for c in checks.values()) else "FAIL",
        "checks": checks,
    }
    tmp = run_dir / "summary.json.tmp"
    tmp.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(run_dir / "summary.json")
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Judge a SITL run directory and write summary.json.")
    parser.add_argument("--probe", required=True, type=pathlib.Path, metavar="RUN_DIR",
                        help="probe run directory (log/sitl/<stamp>-<profile>-probe)")
    args = parser.parse_args(argv)
    if not args.probe.is_dir():
        print(f"sitl_checks: not a directory: {args.probe}", file=sys.stderr)
        return 2
    summary = evaluate_probe(args.probe)
    for name, check in summary["checks"].items():
        print(f"{'PASS' if check['pass'] else 'FAIL'}  {name}: {json.dumps(check['value'])}")
    print(f"{'PASS' if summary['phases_ok'] else 'FAIL'}  phases_ok: "
          f"{json.dumps([(ph.get('name'), ph.get('ok')) for ph in summary['phases'] or [] if isinstance(ph, dict)])}")
    print(f"SITL_PROBE_RESULT={summary['result']} ({args.probe / 'summary.json'})")
    return 0 if summary["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
