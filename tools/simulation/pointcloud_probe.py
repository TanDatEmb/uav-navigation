#!/usr/bin/env python3
"""Bounded, layout-aware PointCloud2 XYZ quality probe."""
from __future__ import annotations

import math
import struct
from dataclasses import asdict, dataclass
from typing import Any

FLOAT32 = 7
FLOAT64 = 8


class PointCloudProbeError(ValueError):
    code = "POINTCLOUD_LAYOUT_INVALID"


class MissingXyzFields(PointCloudProbeError):
    code = "POINTCLOUD_XYZ_FIELD_MISSING"


@dataclass
class CloudQuality:
    width: int
    height: int
    point_step: int
    row_step: int
    total_points: int
    sampled_points: int
    is_dense: bool
    fields: list[dict[str, Any]]
    frame_id: str
    stamp_ns: int
    finite_xyz_count: int = 0
    nan_xyz_count: int = 0
    positive_inf_xyz_count: int = 0
    negative_inf_xyz_count: int = 0
    finite_ratio: float = 0.0
    zero_xyz_count: int = 0
    range_below_min_count: int = 0
    range_above_max_count: int = 0
    minimum_finite_range: float | None = None
    maximum_finite_range: float | None = None
    mean_finite_range: float | None = None
    density_contract_violation: bool = False

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _stamp_ns(message: Any) -> int:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    return 0 if stamp is None else int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _layout(message: Any) -> tuple[dict[str, tuple[int, str]], list[dict[str, Any]]]:
    fields: dict[str, tuple[int, str]] = {}
    description = []
    for field in message.fields:
        item = {"name": field.name, "offset": int(field.offset),
                "datatype": int(field.datatype), "count": int(field.count)}
        description.append(item)
        if field.name not in ("x", "y", "z"):
            continue
        if field.count < 1 or field.datatype not in (FLOAT32, FLOAT64):
            raise PointCloudProbeError(f"{field.name} must be scalar FLOAT32/FLOAT64")
        code = "f" if field.datatype == FLOAT32 else "d"
        size = 4 if code == "f" else 8
        if field.offset < 0 or field.offset + size > int(message.point_step):
            raise PointCloudProbeError(f"{field.name} exceeds point_step")
        fields[field.name] = (int(field.offset), code)
    if set(fields) != {"x", "y", "z"}:
        raise MissingXyzFields("PointCloud2 does not contain scalar x/y/z fields")
    return fields, description


def inspect_pointcloud(message: Any, maximum_points: int = 2048,
                       minimum_range_m: float = 0.10,
                       maximum_range_m: float = 40.0) -> CloudQuality:
    width, height = int(message.width), int(message.height)
    point_step, row_step = int(message.point_step), int(message.row_step)
    if width < 0 or height < 0 or point_step <= 0 or row_step < width * point_step:
        raise PointCloudProbeError("invalid dimensions or row stride")
    data = bytes(message.data)
    required = row_step * height
    if len(data) < required:
        raise PointCloudProbeError(f"data has {len(data)} bytes; needs {required}")
    layout, description = _layout(message)
    total = width * height
    count = min(max(0, int(maximum_points)), total)
    indices = [] if count == 0 else (
        [0] if count == 1 else
        [(i * (total - 1)) // (count - 1) for i in range(count)])
    endian = ">" if bool(message.is_bigendian) else "<"
    quality = CloudQuality(
        width, height, point_step, row_step, total, count, bool(message.is_dense),
        description, getattr(getattr(message, "header", None), "frame_id", ""),
        _stamp_ns(message),
    )
    finite_ranges: list[float] = []
    for index in indices:
        row, column = divmod(index, width)
        base = row * row_step + column * point_step
        xyz = [struct.unpack_from(endian + code, data, base + offset)[0]
               for offset, code in (layout[axis] for axis in ("x", "y", "z"))]
        if any(math.isnan(value) for value in xyz):
            quality.nan_xyz_count += 1
            continue
        if any(math.isinf(value) and value > 0 for value in xyz):
            quality.positive_inf_xyz_count += 1
            continue
        if any(math.isinf(value) and value < 0 for value in xyz):
            quality.negative_inf_xyz_count += 1
            continue
        quality.finite_xyz_count += 1
        distance = math.sqrt(sum(value * value for value in xyz))
        finite_ranges.append(distance)
        if distance == 0.0:
            quality.zero_xyz_count += 1
        if distance < minimum_range_m:
            quality.range_below_min_count += 1
        if distance > maximum_range_m:
            quality.range_above_max_count += 1
    quality.finite_ratio = quality.finite_xyz_count / count if count else 0.0
    if finite_ranges:
        quality.minimum_finite_range = min(finite_ranges)
        quality.maximum_finite_range = max(finite_ranges)
        quality.mean_finite_range = sum(finite_ranges) / len(finite_ranges)
    quality.density_contract_violation = bool(message.is_dense) and (
        quality.nan_xyz_count + quality.positive_inf_xyz_count +
        quality.negative_inf_xyz_count > 0)
    return quality
