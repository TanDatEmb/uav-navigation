#!/usr/bin/env python3
"""Export auditable M1 artifacts from the production-node output bag."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import struct
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader


def cloud_xyz(message) -> np.ndarray:
    fields = {field.name: field.offset for field in message.fields}
    if not {"x", "y", "z"} <= fields.keys():
        raise ValueError("PointCloud2 lacks x/y/z")
    endian = ">" if message.is_bigendian else "<"
    count = int(message.width) * int(message.height)
    result = np.empty((count, 3), dtype=np.float64)
    for index in range(count):
        base = index * int(message.point_step)
        result[index] = [
            struct.unpack_from(endian + "f", message.data, base + fields[axis])[0]
            for axis in ("x", "y", "z")
        ]
    return result[np.isfinite(result).all(axis=1)]


def write_pcd(path: Path, points: np.ndarray) -> None:
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
        "TYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(points)}\nDATA ascii\n"
    )
    with path.open("w") as stream:
        stream.write(header)
        np.savetxt(stream, points, fmt="%.7g")


def plot_map(points: np.ndarray, output: Path) -> None:
    views = (("xy", 0, 1), ("xz", 0, 2), ("yz", 1, 2))
    for name, first, second in views:
        fig, axis = plt.subplots(figsize=(8, 8))
        if len(points):
            axis.scatter(points[:, first], points[:, second], c=points[:, 2],
                         s=0.2, cmap="viridis")
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel(name[0] + " [m]")
        axis.set_ylabel(name[1] + " [m]")
        axis.set_title(f"M1 production local map: {name.upper()}")
        fig.tight_layout()
        fig.savefig(output / f"map_{name}.png", dpi=180)
        plt.close(fig)
    fig, axis = plt.subplots(figsize=(9, 8))
    if len(points):
        # Fixed orthographic/isometric projection avoids an optional mpl_toolkits
        # runtime dependency while retaining all three spatial coordinates.
        projected_x = points[:, 0] - 0.55 * points[:, 1]
        projected_y = points[:, 2] + 0.28 * (points[:, 0] + points[:, 1])
        axis.scatter(projected_x, projected_y, c=points[:, 2],
                     s=0.2, cmap="viridis")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("isometric horizontal [m]")
    axis.set_ylabel("isometric vertical [m]")
    axis.set_title("M1 production local map: fixed 3D perspective")
    fig.tight_layout()
    fig.savefig(output / "map_3d.png", dpi=180)
    plt.close(fig)


def export(bag: Path, output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    odometry = []
    diagnostics = []
    latest_transport = {}
    maps = []
    status_counts = Counter()
    with AnyReader([bag]) as reader:
        for connection, record_time, raw in reader.messages():
            message = reader.deserialize(raw, connection.msgtype)
            if connection.topic == "/lio/odometry_corrected":
                pose = message.pose.pose
                odometry.append([
                    message.header.stamp.sec * 1_000_000_000
                    + message.header.stamp.nanosec,
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w,
                    message.header.frame_id, message.child_frame_id,
                ])
            elif connection.topic == "/lio/local_map":
                maps.append(cloud_xyz(message))
            elif connection.topic == "/lio/diagnostics":
                status = message.status[0]
                values = {item.key: item.value for item in status.values}
                if status.name == "fast_lio/transport":
                    latest_transport = values
                    continue
                reason_key = (
                    "OVERLAPPING_LIDAR_INTERVAL"
                    if status.message.startswith("OVERLAPPING_LIDAR_INTERVAL ")
                    else status.message
                )
                status_counts[reason_key] += 1
                diagnostics.append({
                    "record_time_ns": record_time,
                    "level": status.level,
                    "reason": status.message,
                    **values,
                })
    columns = [
        "time_ns", "x", "y", "z", "qx", "qy", "qz", "qw",
        "frame_id", "child_frame_id",
    ]
    with (output / "trajectory.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        writer.writerows(odometry)
    with (output / "trajectory.tum").open("w") as stream:
        for row in odometry:
            stream.write(
                f"{row[0] / 1e9:.9f} " + " ".join(map(str, row[1:8])) + "\n"
            )
    diagnostic_columns = sorted(
        {key for row in diagnostics for key in row.keys()}
    )
    with (output / "diagnostics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=diagnostic_columns)
        writer.writeheader()
        writer.writerows(diagnostics)
    with (output / "state.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns[:8])
        for row in odometry:
            writer.writerow(row[:8])
    derived_csv = {
        "covariance.csv": (
            ["record_time_ns", "trace", "minimum_eigenvalue", "maximum_asymmetry"],
            ["covariance_trace", "covariance_minimum_eigenvalue",
             "covariance_maximum_asymmetry"],
        ),
        "corrections.csv": (
            ["record_time_ns", "status", "iteration_count",
             "final_increment_norm"],
            ["corrected_estimate_valid", "iteration_count",
             "final_increment_norm"],
        ),
        "residuals.csv": (
            ["record_time_ns", "accepted_count", "rms_m"],
            ["accepted_residual_count", "residual_rms_m"],
        ),
        "timing.csv": (
            ["record_time_ns", "prediction_us", "deskew_us",
             "preprocessing_us", "residual_build_us", "ikfom_update_us",
             "map_insert_crop_us", "snapshot_us", "total_processing_us"],
            ["prediction_us", "deskew_us", "preprocessing_us",
             "residual_build_us", "ikfom_update_us", "map_insert_crop_us",
             "snapshot_us", "total_processing_us"],
        ),
    }
    for name, (header, keys) in derived_csv.items():
        with (output / name).open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            for row in diagnostics:
                if all(key in row for key in keys):
                    writer.writerow([row["record_time_ns"]] +
                                    [row[key] for key in keys])
    points = maps[-1] if maps else np.empty((0, 3))
    write_pcd(output / "local_registration_map_final.pcd", points)
    write_pcd(output / "reference_map.pcd", np.empty((0, 3)))
    (output / "reference_trajectory.tum").write_text("")
    plot_map(points, output)
    fig, axis = plt.subplots(figsize=(8, 5))
    axis.text(0.5, 0.5, "Comparison unavailable\\nproduction tracking failed; ROS1 reference blocked",
              ha="center", va="center", transform=axis.transAxes)
    axis.set_axis_off()
    fig.tight_layout()
    fig.savefig(output / "trajectory_comparison.png", dpi=180)
    plt.close(fig)
    bounds = (
        {"min": points.min(axis=0).tolist(), "max": points.max(axis=0).tolist()}
        if len(points) else None
    )
    successful = status_counts["LIDAR_CORRECTION_CONVERGED"]
    rejected = len(diagnostics) - successful
    last_diagnostic = diagnostics[-1] if diagnostics else {}
    final_counters = {**last_diagnostic, **latest_transport}
    metadata = {
        "frame": "odom",
        "point_unit": "meter",
        "source_dataset": "swarm_lio2_mutual_avoidance_uav1",
        "extrinsic_T_imu_lidar": {
            "translation_m": [-0.019391, -0.000278, 0.080926],
            "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "state_convention": "odom to imu_link",
        "number_of_inserted_scans": successful,
        "number_of_rejected_scans": rejected,
        "correction_success_ratio": (
            successful /
            max(1, successful + status_counts["IKFOM_LIDAR_UPDATE_NOT_CONVERGED"])
        ),
        "number_of_points": len(points),
        "bounding_box": bounds,
        "voxel_size_m": 0.2,
        "map_backend": "upstream ikd-Tree",
        "config_path": final_counters.get("config_path"),
        "config_SHA256": final_counters.get("config_sha256"),
    }
    (output / "map_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    full_ingress = (
        final_counters.get("ros_received_imu_count") == "8000"
        and final_counters.get("ros_received_lidar_count") == "1384"
        and final_counters.get("core_accepted_imu_count") == "8000"
        and final_counters.get("core_accepted_lidar_count") == "1384"
    )
    finite_trajectory = all(
        math.isfinite(value) for row in odometry for value in row[1:8]
    )
    manifest = {
        "result": (
            "PASS"
            if successful > 100 and finite_trajectory and full_ingress
            else "FAIL"
        ),
        "production_output_bag": str(bag),
        "odometry_count": len(odometry),
        "diagnostic_count": len(diagnostics),
        "status_counts": dict(status_counts),
        "finite_trajectory": finite_trajectory,
        "full_ingress_and_core_acceptance": full_ingress,
        "ingress_counters": {
            key: final_counters.get(key)
            for key in (
                "ros_received_imu_count",
                "ros_received_lidar_count",
                "core_accepted_imu_count",
                "core_accepted_lidar_count",
                "ros_maximum_imu_gap_ns",
                "processing_queue_high_water_mark",
            )
        },
        "config_path": final_counters.get("config_path"),
        "config_SHA256": final_counters.get("config_sha256"),
        "processing_effectiveness": {
            key: final_counters.get(key)
            for key in (
                "raw_lidar_count",
                "buffer_accepted_lidar_count",
                "overlap_rejected_count",
                "missing_bracket_rejected_count",
                "invalid_timestamp_rejected_count",
                "synchronized_group_count",
                "correction_attempt_count",
                "correction_success_count",
                "correction_failure_count",
                "buffer_acceptance_ratio",
                "synchronization_ratio",
                "correction_success_ratio",
            )
        },
        "host": {
            "platform": platform.platform(),
            "cpu": platform.processor(),
            "logical_cores": __import__("os").cpu_count(),
        },
    }
    (output / "run_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    export(args.bag, args.output)


if __name__ == "__main__":
    main()
