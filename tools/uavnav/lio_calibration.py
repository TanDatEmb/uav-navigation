"""Degeneracy threshold calibration from a SITL altitude sweep (S1b Task 13; SYSTEM_DESIGN §3.2, D20, O6, F36).

Data (plan Task 13):
- per scan, the smallest eigenvalue of the translation and of the rotation block of the scan information matrix,
  from the "LioScan" events of the lio node (values translation_min_eigenvalue, rotation_min_eigenvalue);
- per 1 s window, the drift |dp_lio - dp_truth| of /lio/state against the Gazebo ground truth after T (the
  /alignment yaw maps lio_odom to PX4 NED; NED -> ENU is the ground-truth frame).

Threshold, per block: the smallest eigenvalue level L (on a log grid, GRID_PER_DECADE levels per decade) such that
the windows whose weakest scan is >= L drift <= max_drift_m in >= quantile of them, and every higher level with at
least min_support windows above it keeps that too (a bad band high up pushes the threshold above it). A threshold is
only recommended with support: >= min_support windows on each side, and the windows below it must actually fail
the quantile (a degenerate region exists). Otherwise InsufficientSupport (a ValueError) says why; no threshold is
ever invented from the lowest level seen. Scans and windows at or after the first LOST are excluded (in S1b a LOST
LIO stays RESTARTING, O25, and its /lio/state is IMU-only).

Limits of the method (stated in the report): a scan judged degenerate by the running thresholds is not applied to
the predictor (D31), so the drift of windows below the running pair is IMU-only by construction. A sweep can show
that a pair is too LOW (bad drift above it); it cannot show that a lower pair would be safe. In a sweep both blocks
fall together, so each block's threshold is confounded with the other (support["block_rank_correlation"]).

CLI:
    python3 -m tools.uavnav.lio_calibration RUN_DIR      (a log/sitl/<stamp>-gps-altitude_sweep directory)
        -> prints the per-step table and the recommendation, writes RUN_DIR/calibration.json;
           exit 0 a pair is supported, 3 insufficient support (the beta pair stays), 2 missing data or usage
"""
import argparse
import bisect
import json
import math
import pathlib
import sys

try:
    from tools.uavnav import events as event_log
    from tools.uavnav import sitl_checks
except ImportError:  # run as a script: tools/uavnav is sys.path[0]
    import events as event_log  # type: ignore[no-redef]
    import sitl_checks  # type: ignore[no-redef]

MissingData = sitl_checks.MissingData

MAX_DRIFT_M = 0.05          # plan Task 13: drift over a 1 s window <= 0.05 m ...
QUANTILE = 0.95             # ... in 95 % of the windows
WINDOW_S = 1.0
MIN_SUPPORT = 50            # windows on each side of a recommended threshold
GRID_PER_DECADE = 20        # log grid of candidate levels: one step is a factor 10^(1/20) = 1.122
BAND_MIN_WINDOWS = 10       # a one-step band below the threshold with this many windows can flag non-monotone
MAX_BRACKET_S = 0.1         # a position is interpolated only between samples at most 0.1 s apart
LIO_VALID = 1               # Alignment.VALID


class InsufficientSupport(ValueError):
    """The data cannot support a threshold (too few windows on a side, no degenerate region, no level keeps the
    quantile). The beta values stay."""


def _finite(*values) -> bool:
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values)


def _sensor_time_s(record: dict):
    t = record.get("values", {}).get("sensor_time_s")
    return float(t) if _finite(t) else record["t_ros_ns"] / 1e9  # SITL: sensor time = ROS time (D29)


def first_lost_s(events):
    """Sensor time of the first LIO StateTransition into LOST, or None."""
    times = [_sensor_time_s(r) for r in events
             if r.get("event") == "StateTransition" and r.get("state_after") == "LOST"]
    return min(times) if times else None


def scan_eigenvalues(events) -> list:
    """[(sensor_time_s, translation, rotation)] of the LioScan events that carry both eigenvalues as finite numbers,
    sorted by time; scans at or after the first LOST are excluded. A zero-information report keeps its 0.0 values
    (an unusable correction is the weakest scan, not a missing one)."""
    lost = first_lost_s(events)
    out = []
    for r in events:
        if r.get("event") != "LioScan":
            continue
        values = r.get("values", {})
        lt, lr = values.get("translation_min_eigenvalue"), values.get("rotation_min_eigenvalue")
        if not _finite(lt, lr):
            continue
        t = _sensor_time_s(r)
        if lost is not None and t >= lost:
            continue
        out.append((t, float(lt), float(lr)))
    return sorted(out)


def lio_in_truth_frame(lio, yaw_t_rad: float) -> list:
    """[(t, x, y, z)] of /lio/state positions (lio_odom FLU) expressed in the ground-truth frame (Gazebo ENU), up to
    a constant translation: FLU -> FRD, then T's yaw (FRD -> PX4 NED, alignment.hpp), then NED -> ENU."""
    c, s = math.cos(yaw_t_rad), math.sin(yaw_t_rad)
    out = []
    for t, x, y, z in lio:
        fx, fy, fz = x, -y, -z
        north, east, down = c * fx - s * fy, s * fx + c * fy, fz
        out.append((t, east, north, -down))
    return out


def _clean(samples) -> list:
    return sorted(tuple(float(v) for v in s[:4]) for s in samples if _finite(*s[:4]))


def _interpolate(samples, times, t, max_bracket_s):
    i = bisect.bisect_left(times, t)
    if i < len(times) and times[i] == t:
        return samples[i][1:]
    if i == 0 or i == len(times):
        return None
    (ta, *a), (tb, *b) = samples[i - 1], samples[i]
    if tb - ta > max_bracket_s:
        return None
    f = (t - ta) / (tb - ta)
    return tuple(pa + f * (pb - pa) for pa, pb in zip(a, b))


def drift_windows(lio, truth, window_s=WINDOW_S, max_bracket_s=MAX_BRACKET_S) -> list:
    """[(window end, drift m)] over consecutive windows of window_s from the first common time. lio and truth are
    [(t_s, x, y, z)] in one frame (lio_in_truth_frame); drift = |dp_lio - dp_truth| over the window (3-D). Non-finite
    samples are dropped; a window is skipped when an end is not bracketed by samples <= max_bracket_s apart."""
    lio, truth = _clean(lio), _clean(truth)
    if len(lio) < 2 or len(truth) < 2:
        return []
    lio_t, truth_t = [s[0] for s in lio], [s[0] for s in truth]
    t0, t_last = max(lio_t[0], truth_t[0]), min(lio_t[-1], truth_t[-1])
    out = []
    for k in range(1, int(math.floor((t_last - t0) / window_s + 1e-9)) + 1):
        start, end = t0 + (k - 1) * window_s, t0 + k * window_s
        ends = [_interpolate(lio, lio_t, start, max_bracket_s), _interpolate(lio, lio_t, end, max_bracket_s),
                _interpolate(truth, truth_t, start, max_bracket_s), _interpolate(truth, truth_t, end, max_bracket_s)]
        if any(p is None for p in ends):
            continue
        l0, l1, g0, g1 = ends
        out.append((end, math.dist([b - a for a, b in zip(l0, l1)], [b - a for a, b in zip(g0, g1)])))
    return out


def window_eigenvalues(scans, drift, window_s=WINDOW_S) -> list:
    """[(window end, translation, rotation, drift)]: each window carries the smallest eigenvalue of each block over
    the scans in (end - window_s, end]; a window without a scan is dropped."""
    scans = sorted(scans)
    times = [s[0] for s in scans]
    out = []
    for end, d in drift:
        lo, hi = bisect.bisect_right(times, end - window_s), bisect.bisect_right(times, end)
        if lo < hi:
            inside = scans[lo:hi]
            out.append((end, min(s[1] for s in inside), min(s[2] for s in inside), d))
    return out


def _check_args(max_drift_m, quantile, min_support):
    if not (_finite(max_drift_m) and max_drift_m > 0 and _finite(quantile) and 0 < quantile <= 1
            and isinstance(min_support, int) and min_support >= 1):
        raise ValueError(f"max_drift_m must be > 0, quantile in (0, 1], min_support >= 1: "
                         f"{max_drift_m!r} {quantile!r} {min_support!r}")


def _good_fraction(drifts, max_drift_m):
    return sum(1 for d in drifts if d <= max_drift_m) / len(drifts)


def block_threshold(pairs, max_drift_m=MAX_DRIFT_M, quantile=QUANTILE, min_support=MIN_SUPPORT) -> dict:
    """The threshold of one block from [(eigenvalue, drift)] windows (module docstring). Returns {"threshold",
    "above", "below", "good_fraction_above", "good_fraction_below", "monotone", "levels"}; raises
    InsufficientSupport. monotone is False when a one-step band below the threshold with >= BAND_MIN_WINDOWS windows
    keeps the quantile: the drift is not monotone in the eigenvalue and the threshold sits above a bad band."""
    _check_args(max_drift_m, quantile, min_support)
    pairs = [(float(l), float(d)) for l, d in pairs if _finite(l, d)]
    if not pairs:
        raise InsufficientSupport("insufficient support: no window with a finite eigenvalue and drift")
    overall = _good_fraction([d for _, d in pairs], max_drift_m)
    if overall >= quantile:
        raise InsufficientSupport(
            f"insufficient support: no degenerate region (drift <= {max_drift_m} m in {overall:.1%} of all "
            f"{len(pairs)} windows, so every level keeps {quantile:.0%} and the data cannot place a threshold)")
    positive = [l for l, _ in pairs if l > 0]
    if not positive:
        raise InsufficientSupport("insufficient support: no window with a positive eigenvalue")
    k_lo = math.floor(math.log10(min(positive)) * GRID_PER_DECADE)
    k_hi = math.ceil(math.log10(max(positive)) * GRID_PER_DECADE)
    levels = [10.0 ** (k / GRID_PER_DECADE) for k in range(k_lo, k_hi + 1)]
    judged = []  # (level, windows above, good fraction above, keeps the quantile or None when not judged)
    for level in levels:
        above = [d for l, d in pairs if l >= level]
        if len(above) < min_support:
            judged.append((level, len(above), None, None))
            continue
        fraction = _good_fraction(above, max_drift_m)
        judged.append((level, len(above), fraction, fraction >= quantile))
    highest_fail = max((level for level, _, _, ok in judged if ok is False), default=None)
    candidates = [row for row in judged if row[3] and (highest_fail is None or row[0] > highest_fail)]
    if not candidates:
        raise InsufficientSupport(
            f"insufficient support: no level keeps drift <= {max_drift_m} m in {quantile:.0%} of the >= "
            f"{min_support} windows above it (and at every higher level)")
    threshold, n_above, fraction_above, _ = min(candidates)
    below = [d for l, d in pairs if l < threshold]
    if len(below) < min_support:
        raise InsufficientSupport(f"insufficient support: {len(below)} window(s) below {threshold:.4g} "
                                  f"(need {min_support})")
    # The whole set fails the quantile and the side above keeps it, so the side below fails it: a degenerate region.
    fraction_below = _good_fraction(below, max_drift_m)
    monotone = True
    for low, high in zip(levels, levels[1:]):
        if high > threshold:
            break
        band = [d for l, d in pairs if low <= l < high]
        if len(band) >= BAND_MIN_WINDOWS and _good_fraction(band, max_drift_m) >= quantile:
            monotone = False
    return {"threshold": threshold, "above": n_above, "below": len(below), "good_fraction_above": fraction_above,
            "good_fraction_below": fraction_below, "monotone": monotone, "levels": len(levels)}


def _ranks(values):
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    i = 0
    while i < len(order):  # average ranks over ties
        j = i
        while j + 1 < len(order) and values[order[j + 1]] == values[order[i]]:
            j += 1
        for k in range(i, j + 1):
            ranks[order[k]] = (i + j) / 2.0
        i = j + 1
    return ranks


def rank_correlation(a, b):
    """Spearman rank correlation of two equal-length lists, or None when undefined."""
    if len(a) < 2:
        return None
    ra, rb = _ranks(a), _ranks(b)
    ma, mb = sum(ra) / len(ra), sum(rb) / len(rb)
    cov = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    va, vb = sum((x - ma) ** 2 for x in ra), sum((y - mb) ** 2 for y in rb)
    return cov / math.sqrt(va * vb) if va > 0 and vb > 0 else None


def recommend_thresholds(scans, drift, max_drift_m=MAX_DRIFT_M, quantile=QUANTILE, *, window_s=WINDOW_S,
                         lost_s=None, min_support=MIN_SUPPORT) -> dict:
    """{"translation", "rotation", "support"} from scans [(t, translation, rotation)] and drift windows
    [(end, drift)]. Windows ending after lost_s (the first LOST) are excluded. Raises InsufficientSupport naming the
    block when either block has no supported threshold, ValueError on bad arguments."""
    _check_args(max_drift_m, quantile, min_support)
    if not (_finite(window_s) and window_s > 0):
        raise ValueError(f"window_s must be > 0: {window_s!r}")
    finite_scans = [s for s in scans if _finite(*s)]
    usable, non_finite_windows, after_lost = [], 0, 0
    for end, d in drift:
        if not _finite(end, d):
            non_finite_windows += 1
        elif lost_s is not None and end > lost_s:
            after_lost += 1
        else:
            usable.append((end, d))
    windows = window_eigenvalues(finite_scans, usable, window_s)
    blocks = {}
    for name, index in (("translation", 1), ("rotation", 2)):
        try:
            blocks[name] = block_threshold([(w[index], w[3]) for w in windows], max_drift_m, quantile, min_support)
        except InsufficientSupport as exc:
            raise InsufficientSupport(f"{name}: {exc}") from None
    t, r = blocks["translation"]["threshold"], blocks["rotation"]["threshold"]
    healthy = [w[3] for w in windows if w[1] >= t and w[2] >= r]
    support = {"windows": len(windows), "without_scan": len(usable) - len(windows),
               "non_finite_windows": non_finite_windows, "non_finite_scans": len(scans) - len(finite_scans),
               "after_lost": after_lost, "translation": blocks["translation"], "rotation": blocks["rotation"],
               "joint_windows": len(healthy),
               "joint_good_fraction": _good_fraction(healthy, max_drift_m) if healthy else None,
               "block_rank_correlation": rank_correlation([w[1] for w in windows], [w[2] for w in windows]),
               "max_drift_m": max_drift_m, "quantile": quantile, "window_s": window_s, "min_support": min_support}
    return {"translation": t, "rotation": r, "support": support}


# --- a sweep run directory ------------------------------------------------------------------------------------

def _jsonl(run_dir: pathlib.Path, name: str) -> list:
    return sitl_checks._read_jsonl(run_dir / name)  # raises MissingData


def _events(run_dir: pathlib.Path) -> list:
    skipped = []
    try:
        records = event_log.load(run_dir / "events_lio.jsonl", skip_malformed=True, skipped=skipped)
    except OSError as exc:
        raise MissingData(f"missing events_lio.jsonl: {exc.strerror}") from None
    if skipped or not records:
        raise MissingData(f"events_lio.jsonl: {len(skipped)} malformed line(s), {len(records)} record(s)")
    return records


def _stats(values) -> dict:
    if not values:
        return {"n": 0}
    return {"n": len(values), "min": min(values), "p5": sitl_checks.percentile(values, 5),
            "p50": sitl_checks.percentile(values, 50), "p95": sitl_checks.percentile(values, 95), "max": max(values)}


def _circular_mean(angles):
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


def analyse_run(run_dir) -> dict:
    """Per-step distributions and the recommendation of an altitude sweep run. Raises MissingData."""
    run_dir = pathlib.Path(run_dir)
    try:
        meta = json.loads((run_dir / "hover_meta.json").read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise MissingData(f"hover_meta.json: {exc}") from None
    if not isinstance(meta, dict) or meta.get("pattern") != "altitude_sweep":
        raise MissingData("hover_meta.json: not an altitude_sweep run")
    steps = meta.get("sweep_steps")
    if not isinstance(steps, list) or not steps:
        raise MissingData("hover_meta.json: no sweep_steps")
    events = _events(run_dir)
    lost = first_lost_s(events)
    scans = scan_eigenvalues(events)
    states = [r for r in _jsonl(run_dir, "lio_state.jsonl") if _finite(r.get("stamp_ns"), r.get("x"), r.get("y"),
                                                                       r.get("z"))]
    if not states:
        raise MissingData("lio_state.jsonl: no finite sample")
    epoch = states[0].get("epoch")
    lio = [(r["stamp_ns"] / 1e9, r["x"], r["y"], r["z"]) for r in states if r.get("epoch") == epoch]
    yaws = [r["yaw_rad"] for r in _jsonl(run_dir, "alignment.jsonl")
            if r.get("state") == LIO_VALID and _finite(r.get("yaw_rad"), r.get("stamp_ns"))
            and (lost is None or r["stamp_ns"] / 1e9 < lost)]
    if not yaws:
        raise MissingData("alignment.jsonl: no VALID sample before the first LOST (T unknown)")
    yaw_t = _circular_mean(yaws)
    truth_records = _jsonl(run_dir, "ground_truth.jsonl")
    truth = [(r["stamp_ns"] / 1e9, r["x"], r["y"], r["z"]) for r in truth_records
             if _finite(r.get("stamp_ns"), r.get("x"), r.get("y"), r.get("z"))]
    windows = drift_windows(lio_in_truth_frame(lio, yaw_t), truth)
    scan_events = [r for r in events if r.get("event") == "LioScan"]
    gaps = [r for r in events if r.get("event") == "ImuGap"]
    table = []
    for step in steps:
        start, end = step.get("hold_start_ns"), step.get("hold_end_ns")
        if not _finite(start, end, step.get("target_m")):
            raise MissingData(f"sweep step without hold times: {step!r}")
        s, e = start / 1e9, end / 1e9
        in_hold = [x for x in scans if s <= x[0] < e]
        hold_events = [r for r in scan_events if s <= _sensor_time_s(r) < e]
        hold_windows = [d for t, d in windows if s + WINDOW_S <= t <= e and (lost is None or t <= lost)]
        altitude = [z for t, _, _, z in truth if s <= t < e]
        reasons = {}
        for r in hold_events:
            reasons[r["reason"]] = reasons.get(r["reason"], 0) + 1
        table.append({
            "target_m": step["target_m"], "hold_s": e - s, "after_lost": lost is not None and s >= lost,
            "truth_altitude_m": sum(altitude) / len(altitude) if altitude else None,
            "scans": len(in_hold), "scan_reasons": reasons,
            "degenerate_scans": sum(1 for r in hold_events if r["values"].get("degenerate") == 1.0),
            "translation": _stats([x[1] for x in in_hold]), "rotation": _stats([x[2] for x in in_hold]),
            "quality": _stats([r["values"]["quality"] for r in hold_events if _finite(r["values"].get("quality"))]),
            "drift_m": _stats(hold_windows),
            "imu_gap": sitl_checks._event_total([r for r in gaps if s <= _sensor_time_s(r) < e])})
    try:
        odometry_quality = [r["quality"] for r in _jsonl(run_dir, "lio_odometry.jsonl") if _finite(r.get("quality"))]
    except MissingData:
        odometry_quality = []
    result = {"run_dir": str(run_dir), "first_lost_s": lost, "yaw_t_rad": yaw_t,
              "yaw_t_spread_rad": max(abs(math.remainder(y - yaw_t, 2 * math.pi)) for y in yaws),
              "scans_with_eigenvalues": len(scans), "drift_windows": len(windows), "steps": table,
              "imu_gap_total": sitl_checks._event_total(gaps),
              "transitions": [f"{_sensor_time_s(r):.3f} {r['state_before']}->{r['state_after']} {r['reason']}"
                              for r in events if r.get("event") == "StateTransition"],
              "scan_reasons": {}, "odometry_quality": _stats(odometry_quality),
              "all_windows": {"translation": _stats([w[1] for w in window_eigenvalues(scans, windows)]),
                              "drift_m": _stats([d for t, d in windows if lost is None or t <= lost])}}
    for r in scan_events:
        result["scan_reasons"][r["reason"]] = result["scan_reasons"].get(r["reason"], 0) + 1
    blocks = {}
    for name, index in (("translation", 1), ("rotation", 2)):
        usable = [w for w in window_eigenvalues(scans, windows) if lost is None or w[0] <= lost]
        try:
            blocks[name] = block_threshold([(w[index], w[3]) for w in usable])
        except InsufficientSupport as exc:
            blocks[name] = {"unsupported": str(exc)}
    result["blocks"] = blocks
    try:
        result["recommendation"] = recommend_thresholds(scans, windows, lost_s=lost)
    except InsufficientSupport as exc:
        result["recommendation"] = None
        result["unsupported"] = str(exc)
    return result


def _fmt(stats, key):
    value = stats.get(key)
    return "-" if value is None else f"{value:.3g}"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Degeneracy threshold calibration from an altitude sweep run.")
    parser.add_argument("run_dir", type=pathlib.Path)
    args = parser.parse_args(argv)
    try:
        result = analyse_run(args.run_dir)
    except MissingData as exc:
        print(f"lio_calibration: missing data: {exc}", file=sys.stderr)
        return 2
    print(f"first LOST at {result['first_lost_s']} s; yaw_T {result['yaw_t_rad']:.4f} rad (spread "
          f"{result['yaw_t_spread_rad']:.4f}); {result['scans_with_eigenvalues']} scans with eigenvalues before it; "
          f"{result['drift_windows']} drift windows; ImuGap {result['imu_gap_total']}")
    print("target_m  alt_m  scans  degen  lt_p5     lt_p50    lr_p5     lr_p50    drift_p50 drift_p95 windows imu_gap")
    for row in result["steps"]:
        alt = "-" if row["truth_altitude_m"] is None else f"{row['truth_altitude_m']:.2f}"
        print(f"{row['target_m']:<9} {alt:<6} {row['scans']:<6} {row['degenerate_scans']:<6} "
              f"{_fmt(row['translation'], 'p5'):<9} {_fmt(row['translation'], 'p50'):<9} "
              f"{_fmt(row['rotation'], 'p5'):<9} {_fmt(row['rotation'], 'p50'):<9} "
              f"{_fmt(row['drift_m'], 'p50'):<9} {_fmt(row['drift_m'], 'p95'):<9} {row['drift_m']['n']:<7} "
              f"{row['imu_gap']}{'  (after LOST)' if row['after_lost'] else ''}")
    for line in result["transitions"]:
        print(f"  transition {line}")
    out = args.run_dir / "calibration.json"
    out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if result["recommendation"] is None:
        print(f"LIO_CALIBRATION=UNSUPPORTED ({result['unsupported']}); the beta pair stays ({out})")
        return 3
    rec = result["recommendation"]
    print(f"LIO_CALIBRATION=SUPPORTED translation {rec['translation']:.4g} rotation {rec['rotation']:.4g} ({out})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
