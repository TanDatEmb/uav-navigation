"""Read-only compact extraction from a ROS2 sqlite bag (ROS environment required).

Usage: source /opt/ros/jazzy/setup.bash; source <TARGET install>/setup.bash;
python3 extract_bag_events.py RUN_DIR OUTPUT_DIR

OUTPUT_DIR should be ignored/outside Git. Bag receive timestamp is the recorder's
timestamp, *not* the adapter callback receipt. ROS and PX4 boot clocks stay separate.
"""
import argparse
import csv
import json
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/lio/odometry_propagated": ("odom", ["bag_ns", "source_ns", "epoch", "sequence", "x", "y", "z", "vx", "vy", "vz"]),
    "/navigation/navigation_command": ("command", ["bag_ns", "source_ns", "valid_until_ns", "world_source_ns", "epoch", "mission", "waypoint", "request", "bundle", "sample", "continuation", "continuation_boundary_ns", "role", "status", "world_generation", "world_revision", "execution_authorization_steady_ns"]),
    "/navigation/goal": ("goal", ["bag_ns", "source_ns", "mission", "waypoint", "request", "route_revision", "route_progress_valid", "behavior"]),
    "/navigation/mode_status": ("status", ["bag_ns", "source_ns", "mission", "waypoint", "request", "state", "reason", "waypoint_accepted", "accepted_waypoint", "acceptance_error_m", "acceptance_speed_mps"]),
    "/fmu/out/vehicle_status_v1": ("px4_status", ["bag_ns", "px4_boot_ns", "nav_state", "executor_in_charge", "failsafe", "armed"]),
}


def ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def row(name, msg, bag_ns):
    if name == "odom":
        p = msg.odometry.pose.pose.position
        v = msg.odometry.twist.twist.linear
        return [bag_ns, ns(msg.odometry.header.stamp), msg.localization_epoch, msg.sequence, p.x, p.y, p.z, v.x, v.y, v.z]
    if name == "command":
        return [bag_ns, ns(msg.header.stamp), ns(msg.valid_until), ns(msg.world_observation_stamp), msg.localization_epoch, msg.mission_id, msg.waypoint_index, msg.request_id, msg.bundle_generation, msg.sample_id, int(msg.certified_main_continuation), msg.continuation_boundary_stamp_ns, msg.role, msg.status, msg.world_generation, msg.world_revision, msg.execution_authorization_steady_ns]
    if name == "goal":
        return [bag_ns, ns(msg.header.stamp), msg.mission_id, msg.waypoint_index, msg.request_id, msg.route.route_revision, msg.route.measured_progress_valid, msg.behavior]
    if name == "status":
        return [bag_ns, ns(msg.header.stamp), msg.mission_id, msg.waypoint_index, msg.request_id, msg.state, msg.reason, int(msg.waypoint_accepted), msg.accepted_waypoint_index, msg.acceptance_position_error_m, msg.acceptance_speed_mps]
    if name == "px4_status":
        return [bag_ns, int(msg.timestamp)*1000, msg.nav_state, msg.executor_in_charge, int(msg.failsafe), int(msg.arming_state)]
    raise AssertionError(name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("output_dir", type=Path)
    args = ap.parse_args()
    dbs = sorted((args.run_dir / "rosbag").glob("*.db3"))
    if not dbs:
        raise SystemExit("No sqlite bag; refuse incomplete extraction")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with sqlite3.connect(str(dbs[0])) as db:
        available = {topic: (tid, typ) for tid, topic, typ in db.execute("SELECT id,name,type FROM topics")}
        missing = sorted(set(TOPICS) - set(available))
        if missing:
            raise SystemExit(f"Missing required topics: {missing}")
        files = {}
        writers = {}
        counts = {}
        try:
            for topic, (name, columns) in TOPICS.items():
                f = (args.output_dir / f"{name}.csv").open("w", newline="")
                files[name] = f
                w = csv.writer(f, lineterminator="\n")
                w.writerow(columns)
                writers[name] = w
                counts[name] = 0
                tid, typ = available[topic]
                msg_type = get_message(typ)
                for bag_ns, data in db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
                    try:
                        decoded = deserialize_message(data, msg_type)
                    except Exception as exc:
                        raise RuntimeError(f"decode failed: topic={topic} type={typ} bag_ns={bag_ns} ordinal={counts[name]}") from exc
                    writers[name].writerow(row(name, decoded, bag_ns))
                    counts[name] += 1
        finally:
            for f in files.values(): f.close()
    (args.output_dir / "extraction_manifest.json").write_text(json.dumps({"run_dir":str(args.run_dir.resolve()), "bag_files":[str(p.resolve()) for p in dbs], "counts":counts},indent=2)+"\n")
    print(json.dumps(counts, sort_keys=True))


if __name__ == "__main__": main()
