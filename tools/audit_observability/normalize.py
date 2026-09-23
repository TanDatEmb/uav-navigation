#!/usr/bin/env python3
"""Normalize typed audit bags; never infer a missing event or clock bridge."""

import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path


EVENT_NAMES = {
    1: "MISSION_GATE", 2: "COMMAND_PUBLISH", 3: "COMMAND_RECEIVE",
    4: "HOLD_TRANSFER", 5: "WORLD_TRANSITION", 6: "LEASE_DISPOSITION",
    7: "QUEUE_ACCOUNTING",
}

COMMAND_KEY_FIELDS = (
    "localization_epoch", "goal_epoch", "mission_hash", "waypoint_index",
    "request_id", "bundle_generation", "sample_id",
)


def command_key(row):
    return tuple(int(row[name]) for name in COMMAND_KEY_FIELDS)


def mission_hash(value):
    result = 14695981039346656037
    for byte in value.encode("utf-8"):
        result = ((result ^ byte) * 1099511628211) & ((1 << 64) - 1)
    return result


def px4_input_use(message, observer_ns):
    """Read the existing diagnostic setpoint witness, without granting authority."""
    for status in message.status:
        if status.name != "navigation_external_mode/PX4_INPUT_SETPOINT":
            continue
        values = {item.key: item.value for item in status.values}
        if values.get("command_present") != "true":
            continue
        try:
            row = {name: int(values[name]) for name in COMMAND_KEY_FIELDS
                   if name != "mission_hash"}
            row["mission_hash"] = mission_hash(values["mission_id"])
            row["update_start_ros_ns"] = int(values["update_start_ros_ns"])
            row["update_end_ros_ns"] = int(values["update_end_ros_ns"])
            row["trace_sequence"] = int(values["trace_sequence"])
            row["trace_drop_count"] = int(values["trace_drop_count"])
        except (KeyError, ValueError):
            continue
        row["setpoint_boundary"] = values.get("setpoint_boundary", "UNKNOWN")
        row["bag_observer_ns"] = observer_ns
        yield row


def attach_first_use(pairs, uses, clock_proof=None):
    by_key = defaultdict(list)
    for use in uses:
        by_key[command_key(use)].append(use)
    for pair in pairs:
        eligible = sorted(by_key.get(pair["key"], []),
                          key=lambda use: use["update_start_ros_ns"])
        if clock_proof and pair["status"] == "PAIRED_ROS_CLOCK":
            eligible = [use for use in eligible
                        if use["update_start_ros_ns"] >= pair["receive_entry_ros_ns"]]
        if not eligible:
            pair["first_use"] = "NOT_OBSERVED"
            continue
        pair["first_use"] = "EXACT_KEY_OBSERVED"
        first = eligible[0]
        pair["first_use_boundary"] = first["setpoint_boundary"]
        pair["first_use_trace_sequence"] = first["trace_sequence"]
        pair["first_use_trace_drop_count"] = first["trace_drop_count"]
        if clock_proof and pair["status"] == "PAIRED_ROS_CLOCK":
            start = first["update_start_ros_ns"]
            receive = int(pair["receive_entry_ros_ns"])
            pair["ros_receive_to_first_use_ns"] = start - receive
    return pairs


def sequence_gaps(rows):
    """Return explicit gaps by process/producer; order is diagnostic sequence."""
    groups = defaultdict(list)
    for row in rows:
        groups[(int(row["producer_id"]), int(row["process_incarnation"]))].append(
            int(row["diagnostic_sequence"]))
    gaps = []
    for identity, seqs in groups.items():
        seqs.sort()
        if seqs and seqs[0] > 1:
            gaps.append(dict(producer_id=identity[0], process_incarnation=identity[1],
                             first_missing=1, last_missing=seqs[0] - 1))
        for before, after in zip(seqs, seqs[1:]):
            if after > before + 1:
                gaps.append(dict(producer_id=identity[0], process_incarnation=identity[1],
                                 first_missing=before + 1, last_missing=after - 1))
            elif after == before:
                gaps.append(dict(producer_id=identity[0], process_incarnation=identity[1],
                                 duplicate=after))
    return gaps


def final_drop_count(rows):
    latest = {}
    for row in rows:
        if int(row["event_type"]) != 7:
            continue
        identity = (int(row["producer_id"]), int(row["process_incarnation"]))
        seq = int(row["diagnostic_sequence"])
        if identity not in latest or seq > latest[identity][0]:
            latest[identity] = (seq, int(row["dropped_count"]))
    return sum(value for _, value in latest.values())


def correlate_commands(rows, clock_proof=None):
    """Pair only one-to-one exact keys. No cross-process steady subtraction."""
    published, received = defaultdict(list), defaultdict(list)
    for row in rows:
        event = int(row["event_type"])
        if event == 2:
            published[command_key(row)].append(row)
        elif event == 3:
            received[command_key(row)].append(row)
    pairs = []
    for key, publishers in published.items():
        receivers = received.get(key, [])
        if len(publishers) != 1 or len(receivers) != 1:
            pairs.append(dict(key=key, status="AMBIGUOUS_OR_UNPAIRED",
                              publish_count=len(publishers), receive_count=len(receivers)))
            continue
        p, r = publishers[0], receivers[0]
        row = dict(key=key, status="PAIRED_IDENTITY_ONLY", publish_count=1, receive_count=1,
                   publish_process_incarnation=p["process_incarnation"],
                   receive_process_incarnation=r["process_incarnation"],
                   receive_entry_ros_ns=int(r["ros_now_ns"]),
                   receive_outcome=int(r.get("outcome", 0)),
                   receive_reason=int(r.get("reason", 0)))
        if clock_proof:
            enter = int(r["ros_now_ns"]) - int(p["ros_now_ns"])
            returned = int(r["ros_now_ns"]) - int(p["ros_return_ns"])
            if min(int(p["ros_now_ns"]), int(p["ros_return_ns"]),
                   int(r["ros_now_ns"])) > 0 and enter >= 0:
                row.update(status="PAIRED_ROS_CLOCK", ros_publish_enter_to_receive_ns=enter,
                           ros_publish_return_to_receive_ns=returned,
                           clock_proof=clock_proof)
            else:
                row["status"] = "CLOCK_INVALID"
        pairs.append(row)
    return pairs


def load_bag(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                            output_serialization_format="cdr"))
    types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()}
    if "/navigation/audit_event" not in types:
        raise RuntimeError("audit topic absent; no event inference permitted")
    rows, uses, protocol = [], [], []
    while reader.has_next():
        topic, data, observer_ns = reader.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(data, types[topic])
        if topic == "/navigation/diagnostics":
            uses.extend(px4_input_use(msg, observer_ns))
            continue
        if topic.startswith("/fmu/"):
            protocol.append(dict(topic=topic, bag_observer_ns=observer_ns,
                                 message=message_to_ordereddict(msg)))
            continue
        if topic != "/navigation/audit_event":
            continue
        row = {name: getattr(msg, name) for name in msg.get_fields_and_field_types()}
        row["bag_observer_ns"] = observer_ns
        row["event_name"] = EVENT_NAMES.get(row["event_type"], "UNKNOWN")
        rows.append(row)
    return rows, uses, protocol


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--clock-proof", default=None,
                        help="documented common ROS clock proof ID; omit for identity-only pairs")
    args = parser.parse_args()
    rows, uses, protocol = load_bag(args.bag)
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "audit_events.jsonl").open("w") as output:
        for row in rows:
            output.write(json.dumps(row, sort_keys=True) + "\n")
    with (args.output / "px4_protocol_raw.jsonl").open("w") as output:
        for row in protocol:
            output.write(json.dumps(row, sort_keys=True) + "\n")
    with (args.output / "px4_first_use.jsonl").open("w") as output:
        for row in uses:
            output.write(json.dumps(row, sort_keys=True) + "\n")
    for event_type, filename in ((1, "O1_PASS_THROUGH_EVENTS.csv"),
                                 (2, "O2_PUBLISH_EVENTS.csv"),
                                 (3, "O2_RECEIVE_EVENTS.csv"),
                                 (4, "O3_HOLD_PROTOCOL_EVENTS.csv"),
                                 (5, "O4_WORLD_EVENTS.csv"),
                                 (6, "O4_LEASE_EVENTS.csv")):
        selected = [row for row in rows if row["event_type"] == event_type]
        if selected:
            with (args.output / filename).open("w", newline="") as output:
                writer = csv.DictWriter(output, fieldnames=selected[0].keys())
                writer.writeheader()
                writer.writerows(selected)
    gaps = sequence_gaps(rows)
    pairs = attach_first_use(correlate_commands(rows, args.clock_proof),
                             uses, args.clock_proof)
    if pairs:
        with (args.output / "O2_PUBLISH_RECEIVE.csv").open("w", newline="") as output:
            fields = sorted({field for pair in pairs for field in pair})
            writer = csv.DictWriter(output, fieldnames=fields)
            writer.writeheader()
            writer.writerows(pairs)
    (args.output / "trace_integrity.json").write_text(json.dumps({
        "event_counts": dict(Counter(row["event_name"] for row in rows)),
        "sequence_gaps": gaps,
        "queue_drops_observed": final_drop_count(rows),
        "clock_proof": args.clock_proof,
        "command_pairs": pairs,
        "px4_first_use_count": len(uses),
        "px4_protocol_event_counts": dict(Counter(row["topic"] for row in protocol)),
    }, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
