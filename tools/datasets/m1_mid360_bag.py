#!/usr/bin/env python3
"""Inspect and convert the official, unindexed Swarm-LIO2 ROS1 bag.

The original bag contains valid CHUNK and per-chunk IDXDATA records, but its
final connection/chunk-info index was never written. This reader deliberately
walks the chunk stream and fails on malformed records. It does not repair,
invent, or reorder sensor timestamps.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import struct
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import BinaryIO, Iterator

import numpy as np
from rosbags.rosbag1.reader import Header
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ROSBAG_MAGIC = b"#ROSBAG V2.0\n"
OP_MSGDATA = 2
OP_CHUNK = 5
OP_CONNECTION = 7
DATASET_ID = "swarm_lio2_mutual_avoidance_uav1"
OFFICIAL_PAGE = "https://github.com/hku-mars/Swarm-LIO2"
OFFICIAL_FOLDER = (
    "https://drive.google.com/drive/folders/"
    "17e-Fe5h3LApskJFN_Z0GEGu4ZoavBZsr"
)


def _u32(stream: BinaryIO) -> int:
    raw = stream.read(4)
    if len(raw) != 4:
        raise ValueError("truncated uint32")
    return struct.unpack("<I", raw)[0]


def _normalize_ros1_type(name: str) -> str:
    package, short = name.split("/", 1)
    return f"{package}/msg/{short}"


def _data_header(data: bytes) -> Header:
    return Header.read(io.BytesIO(struct.pack("<I", len(data)) + data))


@dataclass(frozen=True)
class ConnectionInfo:
    connection_id: int
    topic: str
    ros1_type: str
    normalized_type: str
    md5sum: str
    message_definition: str


@dataclass(frozen=True)
class RawMessage:
    connection: ConnectionInfo
    record_time_ns: int
    raw: bytes


class UnindexedRos1Bag:
    """Strict sequential reader for a ROS1 bag with a missing final index."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.connections: dict[int, ConnectionInfo] = {}
        self.chunk_count = 0

    def messages(self) -> Iterator[RawMessage]:
        with self.path.open("rb") as stream:
            if stream.readline() != ROSBAG_MAGIC:
                raise ValueError("not a ROSBAG V2.0 file")
            bag_header = Header.read(stream)
            declared_index = bag_header.get_uint64("index_pos")
            padding_size = _u32(stream)
            stream.seek(padding_size, io.SEEK_CUR)
            file_size = self.path.stat().st_size
            while stream.tell() < file_size:
                record_start = stream.tell()
                probe = stream.read(4)
                if not probe or probe == b"\0\0\0\0":
                    break
                stream.seek(record_start)
                outer = Header.read(stream)
                data_size = _u32(stream)
                data = stream.read(data_size)
                if len(data) != data_size:
                    raise ValueError(f"truncated outer record at {record_start}")
                if outer.get_uint8("op") != OP_CHUNK:
                    continue
                if outer.get_string("compression") != "none":
                    raise ValueError("this converter requires uncompressed chunks")
                if outer.get_uint32("size") != data_size:
                    raise ValueError("chunk uncompressed-size mismatch")
                self.chunk_count += 1
                chunk = io.BytesIO(data)
                while chunk.tell() < data_size:
                    inner_offset = chunk.tell()
                    probe = chunk.read(4)
                    if not probe or probe == b"\0\0\0\0":
                        break
                    chunk.seek(inner_offset)
                    inner = Header.read(chunk)
                    inner_size = _u32(chunk)
                    payload = chunk.read(inner_size)
                    if len(payload) != inner_size:
                        raise ValueError(
                            f"truncated chunk record at {record_start}:{inner_offset}"
                        )
                    operation = inner.get_uint8("op")
                    if operation == OP_CONNECTION:
                        metadata = _data_header(payload)
                        connection_id = inner.get_uint32("conn")
                        ros1_type = metadata.get_string("type")
                        info = ConnectionInfo(
                            connection_id=connection_id,
                            topic=inner.get_string("topic"),
                            ros1_type=ros1_type,
                            normalized_type=_normalize_ros1_type(ros1_type),
                            md5sum=metadata.get_string("md5sum"),
                            message_definition=metadata.get_string(
                                "message_definition"
                            ),
                        )
                        previous = self.connections.get(connection_id)
                        if previous is not None and previous != info:
                            raise ValueError("connection metadata changed in bag")
                        self.connections[connection_id] = info
                    elif operation == OP_MSGDATA:
                        connection_id = inner.get_uint32("conn")
                        if connection_id not in self.connections:
                            raise ValueError("message precedes connection metadata")
                        yield RawMessage(
                            self.connections[connection_id],
                            inner.get_time("time"),
                            payload,
                        )
            if declared_index >= file_size:
                raise ValueError("declared index is outside the bag")


def _typestore_for_bag(reader: UnindexedRos1Bag):
    store = get_typestore(Stores.ROS1_NOETIC)
    # Connections are learned while scanning. Registration is performed as
    # soon as a new definition is encountered by the caller.
    return store


def _register(store, connection: ConnectionInfo) -> None:
    if connection.normalized_type not in store.types:
        store.register(
            get_types_from_msg(
                connection.message_definition, connection.normalized_type
            )
        )


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def inspect_bag(bag: Path, output: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    reader = UnindexedRos1Bag(bag)
    store = _typestore_for_bag(reader)
    counts: Counter[str] = Counter()
    first_record: dict[str, int] = {}
    last_record: dict[str, int] = {}
    frames: dict[str, Counter[str]] = {}
    lidar_rows: list[dict] = []
    imu_times: list[int] = []
    lidar_type = ""
    imu_type = ""

    for raw in reader.messages():
        _register(store, raw.connection)
        topic = raw.connection.topic
        counts[topic] += 1
        first_record.setdefault(topic, raw.record_time_ns)
        last_record[topic] = raw.record_time_ns
        if topic == "/livox/lidar":
            lidar_type = raw.connection.ros1_type
            message = store.deserialize_ros1(
                raw.raw, raw.connection.normalized_type
            )
            if message.point_num != len(message.points):
                raise ValueError("LiDAR point_num differs from points length")
            offsets = [int(point.offset_time) for point in message.points]
            regressions = sum(
                right < left for left, right in zip(offsets, offsets[1:])
            )
            timebase = int(message.timebase)
            header_ns = _stamp_ns(message.header.stamp)
            lidar_rows.append(
                {
                    "record_time_ns": raw.record_time_ns,
                    "header_ns": header_ns,
                    "timebase_ns": timebase,
                    "point_count": len(offsets),
                    "offset_min_ns": min(offsets) if offsets else 0,
                    "offset_max_ns": max(offsets) if offsets else 0,
                    "offset_regressions": regressions,
                    "scan_start_ns": timebase,
                    "scan_end_ns": timebase + (max(offsets) if offsets else 0),
                    "frame_id": message.header.frame_id,
                }
            )
            frames.setdefault(topic, Counter())[message.header.frame_id] += 1
        elif topic == "/mavros/imu/data":
            imu_type = raw.connection.ros1_type
            message = store.deserialize_ros1(
                raw.raw, raw.connection.normalized_type
            )
            imu_time = _stamp_ns(message.header.stamp)
            imu_times.append(imu_time)
            frames.setdefault(topic, Counter())[message.header.frame_id] += 1

    if not lidar_rows or not imu_times:
        raise ValueError("required LiDAR or IMU topic is empty")
    imu_regressions = sum(
        right <= left for left, right in zip(imu_times, imu_times[1:])
    )
    imu_gaps = [
        right - left for left, right in zip(imu_times, imu_times[1:])
    ]
    bracket_failures = 0
    imu_index = 0
    scan_csv = output / "timestamp_scans.csv"
    with scan_csv.open("w", newline="") as stream:
        fieldnames = list(lidar_rows[0]) + [
            "imu_sample_count",
            "first_imu_ns",
            "last_imu_ns",
            "start_bracket",
            "end_bracket",
            "maximum_imu_gap_ns",
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for scan in lidar_rows:
            while (
                imu_index + 1 < len(imu_times)
                and imu_times[imu_index + 1] <= scan["scan_start_ns"]
            ):
                imu_index += 1
            begin = max(0, imu_index)
            end = begin
            while (
                end + 1 < len(imu_times)
                and imu_times[end] < scan["scan_end_ns"]
            ):
                end += 1
            selected = imu_times[begin : end + 1]
            start_bracket = bool(
                selected
                and selected[0] <= scan["scan_start_ns"]
                and selected[-1] >= scan["scan_start_ns"]
            )
            end_bracket = bool(
                selected
                and selected[0] <= scan["scan_end_ns"]
                and selected[-1] >= scan["scan_end_ns"]
            )
            if not (start_bracket and end_bracket):
                bracket_failures += 1
            gaps = [
                right - left for left, right in zip(selected, selected[1:])
            ]
            writer.writerow(
                scan
                | {
                    "imu_sample_count": len(selected),
                    "first_imu_ns": selected[0] if selected else "",
                    "last_imu_ns": selected[-1] if selected else "",
                    "start_bracket": int(start_bracket),
                    "end_bracket": int(end_bracket),
                    "maximum_imu_gap_ns": max(gaps) if gaps else 0,
                }
            )

    all_connections = sorted(
        reader.connections.values(), key=lambda item: item.connection_id
    )
    summary = {
        "dataset_id": DATASET_ID,
        "bag_format": "ROS1 bag v2.0 (final index missing; strict chunk scan)",
        "bag_size_bytes": bag.stat().st_size,
        "bag_sha256": hashlib.sha256(bag.read_bytes()).hexdigest(),
        "chunk_count_scanned": reader.chunk_count,
        "topics": [
            {
                "topic": connection.topic,
                "type": connection.ros1_type,
                "md5sum": connection.md5sum,
                "message_count": counts[connection.topic],
                "first_record_time_ns": first_record[connection.topic],
                "last_record_time_ns": last_record[connection.topic],
            }
            for connection in all_connections
        ],
        "lidar_topic": "/livox/lidar",
        "lidar_type": lidar_type,
        "imu_topic": "/mavros/imu/data",
        "imu_type": imu_type,
        "lidar_message_count": len(lidar_rows),
        "imu_message_count": len(imu_times),
        "lidar_start_ns": lidar_rows[0]["scan_start_ns"],
        "lidar_end_ns": lidar_rows[-1]["scan_end_ns"],
        "duration_s": (
            lidar_rows[-1]["scan_end_ns"] - lidar_rows[0]["scan_start_ns"]
        )
        / 1e9,
        "lidar_rate_hz": len(lidar_rows)
        / (
            (lidar_rows[-1]["scan_start_ns"] - lidar_rows[0]["scan_start_ns"])
            / 1e9
        ),
        "imu_rate_hz": len(imu_times)
        / ((imu_times[-1] - imu_times[0]) / 1e9),
        "point_count_min": min(row["point_count"] for row in lidar_rows),
        "point_count_max": max(row["point_count"] for row in lidar_rows),
        "point_count_mean": sum(row["point_count"] for row in lidar_rows)
        / len(lidar_rows),
        "scan_duration_min_ns": min(
            row["offset_max_ns"] for row in lidar_rows
        ),
        "scan_duration_max_ns": max(
            row["offset_max_ns"] for row in lidar_rows
        ),
        "point_offset_regression_count": sum(
            row["offset_regressions"] for row in lidar_rows
        ),
        "imu_timestamp_nonpositive_dt_count": imu_regressions,
        "maximum_imu_gap_ns": max(imu_gaps),
        "scan_bracket_failure_count": bracket_failures,
        "frame_ids": {
            topic: dict(counter) for topic, counter in frames.items()
        },
        "timestamp_policy": "timebase_authoritative",
        "header_timebase_delta_ns_min": min(
            row["header_ns"] - row["timebase_ns"] for row in lidar_rows
        ),
        "header_timebase_delta_ns_max": max(
            row["header_ns"] - row["timebase_ns"] for row in lidar_rows
        ),
    }
    (output / "topic_metadata.json").write_text(
        json.dumps(summary, indent=2) + "\n"
    )
    definitions = {
        connection.ros1_type: connection.message_definition
        for connection in all_connections
    }
    (output / "message_definitions.json").write_text(
        json.dumps(definitions, indent=2) + "\n"
    )
    (output / "dataset_inspection.md").write_text(
        "# Dataset inspection\n\n"
        f"- Dataset: `{DATASET_ID}`\n"
        f"- Original format: {summary['bag_format']}\n"
        f"- SHA-256: `{summary['bag_sha256']}`\n"
        f"- Duration: {summary['duration_s']:.3f} s\n"
        f"- LiDAR: `/livox/lidar`, `{lidar_type}`, "
        f"{len(lidar_rows)} messages, {summary['lidar_rate_hz']:.3f} Hz\n"
        f"- IMU: `/mavros/imu/data`, `{imu_type}`, "
        f"{len(imu_times)} messages, {summary['imu_rate_hz']:.3f} Hz\n"
        f"- Per-point field: `uint32 offset_time`, nanoseconds relative "
        f"to `uint64 timebase`\n"
        f"- Point offset regressions: "
        f"{summary['point_offset_regression_count']}\n"
        f"- Non-positive IMU dt: "
        f"{summary['imu_timestamp_nonpositive_dt_count']}\n"
        f"- Maximum IMU gap: {summary['maximum_imu_gap_ns']} ns\n"
        f"- Scans lacking a start/end IMU bracket: "
        f"{summary['scan_bracket_failure_count']}\n"
        f"- Timestamp policy: `{summary['timestamp_policy']}` because the "
        f"observed header/timebase delta is "
        f"[{summary['header_timebase_delta_ns_min']}, "
        f"{summary['header_timebase_delta_ns_max']}] ns.\n"
        "- Input frames: "
        f"`{json.dumps(summary['frame_ids'], sort_keys=True)}`\n"
        "- The missing final ROS1 index is preserved as a provenance fact; "
        "conversion reads valid chunks strictly and verifies every record.\n"
    )
    return summary


def _ros2_custom_types(store, root: Path) -> None:
    point = (root / "msg" / "CustomPoint.msg").read_text()
    message = (root / "msg" / "CustomMsg.msg").read_text()
    combined = (
        message
        + "\n================================================================================\n"
        + "MSG: livox_ros_driver2/CustomPoint\n"
        + point
    )
    store.register(
        get_types_from_msg(combined, "livox_ros_driver2/msg/CustomMsg")
    )


def convert_bag(bag: Path, output: Path, livox_source: Path) -> dict:
    output.parent.mkdir(parents=True, exist_ok=True)
    reader = UnindexedRos1Bag(bag)
    ros1 = get_typestore(Stores.ROS1_NOETIC)
    ros2 = get_typestore(Stores.ROS2_JAZZY)
    _ros2_custom_types(ros2, livox_source)
    custom_type = "livox_ros_driver2/msg/CustomMsg"
    custom_point = ros2.types["livox_ros_driver2/msg/CustomPoint"]
    custom_message = ros2.types[custom_type]
    header_type = ros2.types["std_msgs/msg/Header"]
    time_type = ros2.types["builtin_interfaces/msg/Time"]
    output_connections = {}
    before_counts: Counter[str] = Counter()
    after_counts: Counter[str] = Counter()
    first_time: dict[str, int] = {}
    last_time: dict[str, int] = {}
    point_count_before = 0
    point_count_after = 0

    with Writer(output, version=9, storage_plugin=StoragePlugin.SQLITE3) as writer:
        for raw in reader.messages():
            _register(ros1, raw.connection)
            if raw.connection.topic not in (
                "/livox/lidar",
                "/mavros/imu/data",
            ):
                continue
            before_counts[raw.connection.topic] += 1
            message = ros1.deserialize_ros1(
                raw.raw, raw.connection.normalized_type
            )
            if raw.connection.topic == "/livox/lidar":
                points = [
                    custom_point(
                        offset_time=int(point.offset_time),
                        x=float(point.x),
                        y=float(point.y),
                        z=float(point.z),
                        reflectivity=int(point.reflectivity),
                        tag=int(point.tag),
                        line=int(point.line),
                    )
                    for point in message.points
                ]
                point_count_before += len(message.points)
                converted = custom_message(
                    header=header_type(
                        stamp=time_type(
                            sec=int(message.header.stamp.sec),
                            nanosec=int(message.header.stamp.nanosec),
                        ),
                        frame_id=message.header.frame_id,
                    ),
                    timebase=int(message.timebase),
                    point_num=int(message.point_num),
                    lidar_id=int(message.lidar_id),
                    rsvd=np.asarray(message.rsvd, dtype=np.uint8),
                    points=points,
                )
                output_topic = "/livox/lidar"
                output_type = custom_type
                point_count_after += len(points)
            else:
                converted = message
                output_topic = "/mavros/imu/data"
                output_type = "sensor_msgs/msg/Imu"
            key = (output_topic, output_type)
            if key not in output_connections:
                output_connections[key] = writer.add_connection(
                    output_topic, output_type, typestore=ros2
                )
            serialized = ros2.serialize_cdr(converted, output_type)
            writer.write(
                output_connections[key], raw.record_time_ns, serialized
            )
            after_counts[output_topic] += 1
            first_time.setdefault(output_topic, raw.record_time_ns)
            last_time[output_topic] = raw.record_time_ns

    report = {
        "source_bag_sha256": hashlib.sha256(bag.read_bytes()).hexdigest(),
        "source_format": "ROS1 bag v2.0 with missing final index",
        "output_format": "ROS2 sqlite3 bag, CDR",
        "output_directory": str(output),
        "message_counts_before": dict(before_counts),
        "message_counts_after": dict(after_counts),
        "point_count_before": point_count_before,
        "point_count_after": point_count_after,
        "first_record_time_ns": first_time,
        "last_record_time_ns": last_time,
        "preserved_fields": [
            "header.stamp",
            "timebase",
            "point_num",
            "offset_time",
            "x",
            "y",
            "z",
            "reflectivity",
            "tag",
            "line",
            "IMU timestamp",
            "angular velocity",
            "linear acceleration",
            "message ordering",
        ],
    }
    if (
        before_counts != after_counts
        or point_count_before != point_count_after
    ):
        raise ValueError("conversion count verification failed")
    (output.parent / "conversion_report.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    (output.parent / "conversion_report.md").write_text(
        "# ROS1 to ROS2 conversion report\n\n"
        f"- Source SHA-256: `{report['source_bag_sha256']}`\n"
        f"- Messages before/after: `{dict(before_counts)}` / "
        f"`{dict(after_counts)}`\n"
        f"- Points before/after: {point_count_before} / {point_count_after}\n"
        "- Output LiDAR type: `livox_ros_driver2/msg/CustomMsg`\n"
        "- Output IMU type: `sensor_msgs/msg/Imu`\n"
        "- Point timing and message ordering were copied field-for-field; "
        "no PointCloud2 intermediate was used.\n"
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    inspect_parser = subparsers.add_parser("inspect")
    inspect_parser.add_argument("--bag", type=Path, required=True)
    inspect_parser.add_argument("--output", type=Path, required=True)
    convert_parser = subparsers.add_parser("convert")
    convert_parser.add_argument("--bag", type=Path, required=True)
    convert_parser.add_argument("--output", type=Path, required=True)
    convert_parser.add_argument("--livox-source", type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.command == "inspect":
        inspect_bag(arguments.bag, arguments.output)
    else:
        convert_bag(arguments.bag, arguments.output, arguments.livox_source)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
