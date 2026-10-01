"""Collision signed-distance helpers for configured obstacle geometry."""

from __future__ import annotations

import math
from typing import Sequence


def _local(point: Sequence[float], center: Sequence[float], rpy: Sequence[float]) -> tuple[float, float, float]:
    roll, pitch, yaw = (float(value) for value in rpy)
    dx, dy, dz = (float(point[i]) - float(center[i]) for i in range(3))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cp * cy * dx + cp * sy * dy - sp * dz,
        (sr * sp * cy - cr * sy) * dx + (sr * sp * sy + cr * cy) * dy + sr * cp * dz,
        (cr * sp * cy + sr * sy) * dx + (cr * sp * sy - sr * cy) * dy + cr * cp * dz,
    )


def box_signed_distance(point: Sequence[float], center: Sequence[float], half_extents: Sequence[float], rpy: Sequence[float] = (0.0, 0.0, 0.0)) -> float:
    """Exact signed distance to an RPY-oriented box (positive outside)."""
    local = _local(point, center, rpy)
    q = [abs(local[i]) - float(half_extents[i]) for i in range(3)]
    return math.sqrt(sum(max(value, 0.0) ** 2 for value in q)) + min(max(q), 0.0)


def cylinder_signed_distance(point: Sequence[float], center: Sequence[float], radius: float, half_height: float) -> float:
    """Signed distance to a vertical finite cylinder."""
    radial = math.hypot(float(point[0]) - float(center[0]), float(point[1]) - float(center[1])) - radius
    vertical = abs(float(point[2]) - float(center[2])) - half_height
    return math.hypot(max(radial, 0.0), max(vertical, 0.0)) + min(max(radial, vertical), 0.0)


def segment_min_signed_distance(start: Sequence[float], end: Sequence[float], signed_distance) -> float:
    """Minimum signed distance along a swept segment using convex 1-D search."""
    def at(t: float) -> float:
        point = [float(start[i]) + t * (float(end[i]) - float(start[i])) for i in range(3)]
        return signed_distance(point)

    lo, hi = 0.0, 1.0
    for _ in range(72):
        left = (2.0 * lo + hi) / 3.0
        right = (lo + 2.0 * hi) / 3.0
        if at(left) <= at(right):
            hi = right
        else:
            lo = left
    return min(at(0.0), at(1.0), at((lo + hi) / 2.0))
