"""Join audit-only crossing opportunities to recorded producer-side events.

All *_bag_ns are ROS bag recorder timestamps, never adapter receipt or mission
callback timestamps. Crossing replay evaluates each propagated odometry sample;
it does not reproduce MissionController's timer schedule.
"""
import csv
import json
from pathlib import Path
import sys


def rows(path):
    if not path.exists():
        return []
    with path.open() as f:
        return list(csv.DictReader(f))


def num(row, key):
    return int(row[key]) if row else ""


def main():
    raw_root, out_path = map(Path, sys.argv[1:3])
    fields = [
        "run", "crossing_data_source", "classification", "waypoint", "request", "epoch", "odom_sequence",
        "odom_prev_source_ns", "odom_current_source_ns", "crossing_observer_wall_ns",
        "crossing_error_m", "inside_ball", "speed_mps", "source_gap_s",
        "latest_compatible_command_bag_ns", "latest_command_source_ns",
        "latest_command_sample", "latest_command_bundle", "latest_command_continuation",
        "latest_command_valid_until_ns", "first_future_continuation_bag_ns",
        "first_future_continuation_source_ns", "first_future_continuation_sample",
        "mission_accept_bag_ns", "mission_accept_source_ns", "next_goal_bag_ns",
        "next_goal_source_ns", "adapter_receive_ns", "mission_callback_ns",
        "successor_activation_ns", "warning",
    ]
    output = []
    coverage = []
    for rd in sorted(raw_root.iterdir()):
        if not rd.name.startswith("external-mode-"):
            continue
        mf = rd / "extraction_manifest.json"
        if not mf.exists():
            continue
        manifest = json.loads(mf.read_text())
        commands = rows(rd / "command.csv")
        if manifest["counts"]["odom"] >= 100:
            crossing_data_source = "rosbag_recorder"
            crossings = rows(rd / "crossing_candidates.csv")
        else:
            crossing_data_source = "runner_monitor_wall_arrival"
            crossings = rows(rd / "crossing_candidates_monitor.csv")
        statuses = rows(rd / "status.csv")
        goals = rows(rd / "goal.csv")
        monitor_odom_count = max(0, sum(1 for _ in (rd / "odom_monitor.csv").open())-1) if (rd / "odom_monitor.csv").exists() else 0
        coverage.append({"run":rd.name, **manifest["counts"], "monitor_odom":monitor_odom_count, "selected_source":crossing_data_source, "crossing_opportunities":len(crossings)})
        for c in crossings:
            t = int(c["bag_ns"])
            wp, req, epoch = c["waypoint"], c["request"], c["epoch"]
            compatible = [x for x in commands if x["waypoint"] == wp and x["request"] == req and x["epoch"] == epoch]
            before = [x for x in compatible if int(x["bag_ns"]) <= t]
            latest = before[-1] if before else None
            future_ready = next((x for x in compatible if int(x["bag_ns"]) > t and x["continuation"] == "1"), None)
            accepted = next((x for x in statuses if int(x["bag_ns"]) >= t and x["waypoint_accepted"] == "1" and x["accepted_waypoint"] == wp), None)
            next_goal = next((x for x in goals if int(x["bag_ns"]) >= t and int(x["waypoint"]) > int(wp)), None)
            if latest and latest["continuation"] == "1" and int(latest["valid_until_ns"]) >= int(c["source_ns"]):
                classification = "CONTINUATION_PUBLISHED_BEFORE_CROSSING"
            elif latest and latest["continuation"] == "0" and future_ready:
                classification = "CROSSING_BEFORE_FUTURE_PUBLISHED_CONTINUATION_CANDIDATE"
            elif not latest and future_ready:
                classification = "NO_COMPATIBLE_COMMAND_BEFORE_CROSSING"
            else:
                classification = "NO_VALID_PUBLISHED_CONTINUATION_WITNESS"
            output.append({
                "run":rd.name, "crossing_data_source":crossing_data_source, "classification":classification,
                "waypoint":wp, "request":req, "epoch":epoch, "odom_sequence":c["sequence"],
                "odom_prev_source_ns":c["previous_source_ns"], "odom_current_source_ns":c["source_ns"],
                "crossing_observer_wall_ns":c["bag_ns"], "crossing_error_m":c["error_m"],
                "inside_ball":c["inside_ball"], "speed_mps":c["speed_mps"], "source_gap_s":c["gap_s"],
                "latest_compatible_command_bag_ns":num(latest,"bag_ns"),
                "latest_command_source_ns":num(latest,"source_ns"),
                "latest_command_sample":num(latest,"sample"), "latest_command_bundle":num(latest,"bundle"),
                "latest_command_continuation":latest["continuation"] if latest else "",
                "latest_command_valid_until_ns":num(latest,"valid_until_ns"),
                "first_future_continuation_bag_ns":num(future_ready,"bag_ns"),
                "first_future_continuation_source_ns":num(future_ready,"source_ns"),
                "first_future_continuation_sample":num(future_ready,"sample"),
                "mission_accept_bag_ns":num(accepted,"bag_ns"),
                "mission_accept_source_ns":num(accepted,"source_ns"),
                "next_goal_bag_ns":num(next_goal,"bag_ns"), "next_goal_source_ns":num(next_goal,"source_ns"),
                "adapter_receive_ns":"UNOBSERVABLE_WITH_CURRENT_TARGET",
                "mission_callback_ns":"UNOBSERVABLE_WITH_CURRENT_TARGET",
                "successor_activation_ns":"UNOBSERVABLE_WITH_CURRENT_TARGET",
                "warning":"ODOMETRY_SAMPLE_OPPORTUNITY_NOT_MISSION_CALLBACK;OBSERVER_WALL_NOT_ADAPTER_RECEIVE",
            })
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=fields,lineterminator="\n")
        w.writeheader(); w.writerows(output)
    with (out_path.parent / "E1_RUN_COVERAGE.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=["run","command","goal","odom","px4_status","status","monitor_odom","selected_source","crossing_opportunities"],lineterminator="\n")
        w.writeheader(); w.writerows(coverage)
    print(json.dumps({"runs":len(coverage),"crossing_opportunities":len(output),"classifications":{k:sum(x["classification"]==k for x in output) for k in sorted(set(x["classification"] for x in output))}},indent=2))


if __name__ == "__main__": main()
