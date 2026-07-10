#!/usr/bin/env python3
"""Analyze uav-navigation SITL rosbag2 recording.

This tool reads the latest rosbag under log/sim/latest/rosbag/flight_data and
produces a concise report for Phase 1 milestones:

  M1 - Collision Prevention: /fmu/in/obstacle_distance rate and validity
    M2 - NED transform: /livox/world/cloud (fallback: /livox_processed_ned)
    M3 - Global map: /livox/map/global (fallback: /livox_map)
    M4 - Local planning: /livox/perception/scan_1d (fallback: /local_virtual_scan)
  M5 - Visual odometry: /fmu/in/vehicle_visual_odometry vs /fmu/out/vehicle_odometry RMSE

Usage:
    python3 tools/analyze_sitl.py [path/to/rosbag]

If no path is given, the latest session is used automatically.
"""

import json
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

M5_RMSE_THRESHOLD_M = 2.0


def select_topic(messages: Dict[str, List[Any]], candidates: List[str]) -> Tuple[str, List[Any]]:
    """Select the first topic that has data, preserving candidate priority."""
    for topic in candidates:
        topic_messages = messages.get(topic, [])
        if topic_messages:
            return topic, topic_messages

    # No data found: return canonical candidate with empty payload.
    return candidates[0], []


def find_latest_bag(workspace_dir: Path) -> Optional[Path]:
    """Return the most recent rosbag directory under log/sim/."""
    sim_dir = workspace_dir / "log" / "sim"
    latest_link = sim_dir / "latest"
    if latest_link.is_symlink():
        latest_session = latest_link.resolve()
        bag_dir = latest_session / "rosbag" / "flight_data"
        if bag_dir.exists():
            return bag_dir
    # Fallback: scan session directories
    if sim_dir.exists():
        sessions = sorted(
            (d for d in sim_dir.iterdir() if d.is_dir() and d.name.startswith("session_")),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        for session in sessions:
            bag_dir = session / "rosbag" / "flight_data"
            if bag_dir.exists():
                return bag_dir
    return None


def read_bag(bag_path: Path) -> Dict[str, List[Any]]:
    """Read a rosbag2 directory and return messages grouped by topic."""
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    except ImportError as exc:
        raise RuntimeError(
            "rosbag2_py is required. Install: sudo apt install ros-jazzy-rosbag2-py"
        ) from exc

    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python runtime is required. Source ROS 2 before running this script."
        ) from exc

    # Auto-detect storage plugin: mcap (default in Jazzy) or sqlite3.
    storage_id = "mcap"
    if not (bag_path / "flight_data_0.mcap").exists():
        storage_id = "sqlite3"

    storage_options = StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    msg_class_map: Dict[str, Any] = {}
    for topic_name, msg_type in type_map.items():
        try:
            msg_class_map[topic_name] = get_message(msg_type)
        except Exception:
            # Unsupported type is ignored.
            continue

    def deserialize(topic: str, data: bytes):
        msg_class = msg_class_map.get(topic)
        if msg_class is None:
            return None
        try:
            return deserialize_message(data, msg_class)
        except Exception:
            return None

    messages: Dict[str, List[Any]] = defaultdict(list)
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg = deserialize(topic, data)
        if msg is not None:
            messages[topic].append((timestamp, msg))

    return dict(messages)


def topic_rate(messages: List[Any]) -> Optional[float]:
    """Compute average publish rate from (timestamp, msg) list."""
    if len(messages) < 2:
        return None
    timestamps = [t for t, _ in messages]
    duration_s = (timestamps[-1] - timestamps[0]) * 1e-9
    if duration_s <= 0.0:
        return None
    return (len(messages) - 1) / duration_s


def analyze_obstacle_distance(messages: List[Any]) -> Dict[str, Any]:
    """Analyze /fmu/in/obstacle_distance messages."""
    if not messages:
        return {"status": "NO_DATA"}

    valid_bins = 0
    total_bins = 0
    min_dist_m = float("inf")
    max_dist_m = 0.0
    for _, msg in messages:
        for d in msg.distances:
            total_bins += 1
            if d != 0 and d != 65535:
                valid_bins += 1
                dist_m = d * 0.01
                min_dist_m = min(min_dist_m, dist_m)
                max_dist_m = max(max_dist_m, dist_m)

    return {
        "status": "OK",
        "count": len(messages),
        "rate_hz": topic_rate(messages),
        "valid_bins_ratio": valid_bins / total_bins if total_bins else 0.0,
        "min_distance_m": min_dist_m if math.isfinite(min_dist_m) else None,
        "max_distance_m": max_dist_m,
    }


def analyze_cloud(messages: List[Any], expected_frame: str) -> Dict[str, Any]:
    """Analyze a PointCloud2 topic."""
    if not messages:
        return {"status": "NO_DATA"}

    frames = set()
    point_counts = []
    for _, msg in messages:
        frames.add(msg.header.frame_id)
        point_counts.append(msg.width * msg.height)

    return {
        "status": "OK",
        "count": len(messages),
        "rate_hz": topic_rate(messages),
        "frames": sorted(frames),
        "frame_ok": expected_frame in frames,
        "avg_points": float(np.mean(point_counts)) if point_counts else 0.0,
        "min_points": int(np.min(point_counts)) if point_counts else 0,
        "max_points": int(np.max(point_counts)) if point_counts else 0,
    }


def analyze_laserscan(messages: List[Any], expected_frame: str) -> Dict[str, Any]:
    """Analyze a LaserScan topic."""
    if not messages:
        return {"status": "NO_DATA"}

    frames = set()
    beam_counts = []
    for _, msg in messages:
        frames.add(msg.header.frame_id)
        beam_counts.append(len(msg.ranges))

    return {
        "status": "OK",
        "count": len(messages),
        "rate_hz": topic_rate(messages),
        "frames": sorted(frames),
        "frame_ok": expected_frame in frames,
        "avg_beams": float(np.mean(beam_counts)) if beam_counts else 0.0,
        "min_beams": int(np.min(beam_counts)) if beam_counts else 0,
        "max_beams": int(np.max(beam_counts)) if beam_counts else 0,
    }


def analyze_voxel_map(messages: List[Any], topic_name: str) -> Dict[str, Any]:
    """Analyze voxel-map PointCloud2 messages.

    Production contract requires frame_id == "map_ned" (see
    docs/architecture.md Topic Contract). If the frame is not map_ned we
    still report raw metrics but mark the milestone as FAIL with a clear
    reason, so the analyzer never produces a false OK for debug mode.
    """
    if not messages:
        return {"status": "NO_DATA", "topic": topic_name}

    point_counts = []
    frames = set()
    for _, msg in messages:
        point_counts.append(msg.width * msg.height)
        frames.add(msg.header.frame_id)

    frame_id = messages[0][1].header.frame_id if messages else None
    frame_ok = frame_id == "map_ned"
    status = "OK" if frame_ok else "FAIL_DEBUG_MODE"

    result = {
        "status": status,
        "topic": topic_name,
        "count": len(messages),
        "rate_hz": topic_rate(messages),
        "frame_id": frame_id,
        "frame_ok": frame_ok,
        "latest_voxels": int(point_counts[-1]) if point_counts else 0,
        "max_voxels": int(np.max(point_counts)) if point_counts else 0,
        "avg_voxels": float(np.mean(point_counts)) if point_counts else 0.0,
    }
    if not frame_ok:
        result["reason"] = (
            f"{topic_name} frame_id must be 'map_ned' for production assessment. "
            "Re-run with MAP_INPUT_SOURCE=px4_full."
        )
    return result


def extract_pose(msg) -> Optional[np.ndarray]:
    """Extract [x, y, z] position from VehicleOdometry or Odometry message."""
    if hasattr(msg, "position"):
        # px4_msgs/VehicleOdometry: position is float[3]
        return np.array([float(msg.position[0]), float(msg.position[1]), float(msg.position[2])])
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        # nav_msgs/Odometry
        p = msg.pose.pose.position
        return np.array([float(p.x), float(p.y), float(p.z)])
    return None


def extract_sample_time_sec(record_timestamp_ns: int, msg: Any) -> float:
    """Extract sample time (sec), preferring PX4 timestamp_sample when available."""
    if hasattr(msg, "timestamp_sample"):
        ts_sample = int(msg.timestamp_sample)
        if ts_sample > 0:
            return ts_sample * 1e-6
    return record_timestamp_ns * 1e-9


def build_time_pose_arrays(messages: List[Any], use_sample_time: bool = True) -> Tuple[np.ndarray, np.ndarray]:
    """Convert [(record_ts_ns, msg), ...] into sorted unique time-pose arrays."""
    times: List[float] = []
    poses: List[np.ndarray] = []

    for record_t_ns, msg in messages:
        pose = extract_pose(msg)
        if pose is None or not np.isfinite(pose).all():
            continue
        if use_sample_time:
            times.append(extract_sample_time_sec(record_t_ns, msg))
        else:
            times.append(record_t_ns * 1e-9)
        poses.append(pose)

    if not times:
        return np.array([]), np.empty((0, 3))

    t_arr = np.asarray(times)
    p_arr = np.vstack(poses)

    order = np.argsort(t_arr)
    t_arr = t_arr[order]
    p_arr = p_arr[order]

    unique_t, unique_idx = np.unique(t_arr, return_index=True)
    unique_p = p_arr[unique_idx]

    return unique_t, unique_p


def analyze_visual_odometry(ev_messages: List[Any], px4_messages: List[Any]) -> Dict[str, Any]:
    """Compare /fmu/in/vehicle_visual_odometry vs /fmu/out/vehicle_odometry.

    In SITL the LIO-based EV pose is anchored to the PX4 odom pose via the
    translation-only alignment captured at startup, so the difference between
    EV and PX4 stays very small (~cm). In real flight, however, the PX4 EKF2
    itself drifts (especially with wind), so the same comparison is a *sensor
    cross-check*, not a measure of "ground-truth error". We therefore:

      1) compute the raw EV-vs-PX4 RMSE (sensor cross-check),
      2) report the relative-vs-stationary offset to help spot LIO drift,
      3) keep the hard pass/fail threshold for the cross-check, but make the
         expected behaviour explicit in the report.
    """
    if not ev_messages:
        return {"status": "NO_EV_DATA"}
    if not px4_messages:
        return {"status": "NO_PX4_DATA"}

    ev_times, ev_pos = build_time_pose_arrays(ev_messages, use_sample_time=True)
    px4_times, px4_pos = build_time_pose_arrays(px4_messages, use_sample_time=True)

    if ev_pos.shape[0] == 0 or px4_pos.shape[0] == 0:
        return {"status": "EMPTY_POSE"}

    overlap_mask = (ev_times >= px4_times[0]) & (ev_times <= px4_times[-1])
    if not np.any(overlap_mask):
        # Fallback for legacy logs where one topic used ROS-time while the
        # other used PX4 wall-clock in timestamp_sample.
        ev_times, ev_pos = build_time_pose_arrays(ev_messages, use_sample_time=False)
        px4_times, px4_pos = build_time_pose_arrays(px4_messages, use_sample_time=False)
        if ev_pos.shape[0] == 0 or px4_pos.shape[0] == 0:
            return {
                "status": "NO_TIME_OVERLAP",
                "ev_count": len(ev_messages),
                "px4_count": len(px4_messages),
                "reason": "No overlap in timestamp_sample domain and fallback arrays are empty",
            }

        overlap_mask = (ev_times >= px4_times[0]) & (ev_times <= px4_times[-1])
        if not np.any(overlap_mask):
            return {
                "status": "NO_TIME_OVERLAP",
                "ev_count": len(ev_messages),
                "px4_count": len(px4_messages),
                "reason": "No overlap in both timestamp_sample and bag record time domains",
            }

        time_basis = "bag record time (fallback due timestamp_sample mismatch)"
    else:
        time_basis = "timestamp_sample_us (fallback: bag record time)"

    eval_times = ev_times[overlap_mask]
    eval_ev_pos = ev_pos[overlap_mask]

    # Interpolate PX4 position at EV sample times.
    aligned_px4 = np.column_stack(
        [np.interp(eval_times, px4_times, px4_pos[:, axis]) for axis in range(3)]
    )

    errors = np.linalg.norm(eval_ev_pos - aligned_px4, axis=1)
    rmse_m = float(np.sqrt(np.mean(errors**2)))
    status = "OK" if rmse_m <= M5_RMSE_THRESHOLD_M else "FAIL_HIGH_RMSE"

    # Stationary-window RMSE: only the first 2 seconds of overlapping data
    # (or first 10 % of samples, whichever is larger) is treated as
    # "stationary" because the UAV is still on the ground. This isolates
    # LIO/EV noise from later PX4 EKF2 drift during flight.
    stationary_window_s = 2.0
    stationary_mask = eval_times <= (eval_times[0] + stationary_window_s)
    stationary_pairs = int(np.sum(stationary_mask))
    if stationary_pairs < 10:
        fallback_count = max(10, int(0.1 * len(eval_times)))
        stationary_mask = np.zeros_like(eval_times, dtype=bool)
        stationary_mask[:fallback_count] = True
        stationary_pairs = int(np.sum(stationary_mask))
    stationary_rmse_m = float(
        np.sqrt(np.mean(np.linalg.norm(eval_ev_pos[stationary_mask] - aligned_px4[stationary_mask], axis=1) ** 2))
    )

    # Per-second RMSE buckets: helps explain "RMSE ổn lúc đầu, về sau cao" hay
    # ngược lại. Output một mảng nhỏ thay vì full time-series để giữ report gọn.
    duration_s = float(eval_times[-1] - eval_times[0])
    bucket_s = max(duration_s / 10.0, 1.0)
    bucket_edges = np.arange(eval_times[0], eval_times[-1] + bucket_s, bucket_s)
    per_bucket_rmse: List[Dict[str, float]] = []
    for i in range(len(bucket_edges) - 1):
        m = (eval_times >= bucket_edges[i]) & (eval_times < bucket_edges[i + 1])
        n = int(np.sum(m))
        if n == 0:
            continue
        e = np.linalg.norm(eval_ev_pos[m] - aligned_px4[m], axis=1)
        per_bucket_rmse.append({
            "t_start_s": float(bucket_edges[i] - eval_times[0]),
            "t_end_s": float(bucket_edges[i + 1] - eval_times[0]),
            "samples": n,
            "rmse_m": float(np.sqrt(np.mean(e**2))),
        })

    result = {
        "status": status,
        "ev_count": len(ev_messages),
        "px4_count": len(px4_messages),
        "evaluated_pairs": int(eval_times.shape[0]),
        "ev_rate_hz": topic_rate(ev_messages),
        "px4_rate_hz": topic_rate(px4_messages),
        "time_basis": time_basis,
        "rmse_threshold_m": M5_RMSE_THRESHOLD_M,
        "rmse_m": rmse_m,
        "mean_error_m": float(np.mean(errors)),
        "max_error_m": float(np.max(errors)),
        "min_error_m": float(np.min(errors)),
        "stationary_rmse_m": stationary_rmse_m,
        "stationary_pairs": stationary_pairs,
        "per_bucket_rmse": per_bucket_rmse,
        "interpretation": (
            "ev_vs_px4 is a sensor cross-check, not a ground-truth RMSE. "
            "In SITL both share the same origin so RMSE stays small. "
            "In real flight PX4 EKF2 drifts; use stationary_rmse_m to "
            "judge LIO noise, and per_bucket_rmse to spot when the gap "
            "opens up after takeoff."
        ),
    }

    if status != "OK":
        result["reason"] = (
            f"RMSE={rmse_m:.3f}m exceeds threshold "
            f"{M5_RMSE_THRESHOLD_M:.3f}m"
        )

    return result


def main() -> int:
    workspace_dir = Path(__file__).resolve().parent.parent
    bag_path = Path(sys.argv[1]) if len(sys.argv) > 1 else None

    if bag_path is None:
        bag_path = find_latest_bag(workspace_dir)
        if bag_path is None:
            print("ERROR: No rosbag found. Run SITL first with scripts/sim_launch.sh", file=sys.stderr)
            return 1

    print(f"Analyzing rosbag: {bag_path}")
    messages = read_bag(bag_path)

    report: Dict[str, Any] = {
        "bag_path": str(bag_path),
        "topics_recorded": sorted(messages.keys()),
        "milestones": {},
    }

    # M1 - Collision Prevention (canonical topic, no legacy fallback needed).
    m1_messages = messages.get("/fmu/in/obstacle_distance", [])
    report["milestones"]["M1_collision_prevention"] = analyze_obstacle_distance(m1_messages)
    report["milestones"]["M1_collision_prevention"]["topic"] = "/fmu/in/obstacle_distance"

    # M2 - NED transform (canonical + legacy fallback)
    m2_topic, m2_messages = select_topic(
        messages, ["/livox/world/cloud", "/livox_processed_ned"]
    )
    report["milestones"]["M2_ned_transform"] = analyze_cloud(
        m2_messages, expected_frame="map_ned"
    )
    report["milestones"]["M2_ned_transform"]["topic"] = m2_topic

    # M3 - Global voxel map (canonical + legacy fallback)
    m3_topic, m3_messages = select_topic(
        messages, ["/livox/map/global", "/livox_map"]
    )
    report["milestones"]["M3_global_map"] = analyze_voxel_map(m3_messages, m3_topic)

    # M4 - Local virtual scan (canonical + legacy fallback)
    m4_topic, m4_messages = select_topic(
        messages, ["/livox/perception/scan_1d", "/local_virtual_scan"]
    )
    report["milestones"]["M4_local_virtual_scan"] = analyze_laserscan(
        m4_messages, expected_frame="aircraft"
    )
    report["milestones"]["M4_local_virtual_scan"]["topic"] = m4_topic

    # M5 - Visual odometry
    ev_messages = messages.get("/fmu/in/vehicle_visual_odometry", [])
    px4_messages = messages.get("/fmu/out/vehicle_odometry", [])
    report["milestones"]["M5_visual_odometry"] = analyze_visual_odometry(
        ev_messages, px4_messages
    )

    # Print and save report
    print(json.dumps(report, indent=2))

    report_path = bag_path.parent / "sitl_analysis_report.json"
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\nReport saved to: {report_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
