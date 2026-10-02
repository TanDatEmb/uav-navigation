"""Shared runtime percentile definition used by judge and evidence tools."""

from __future__ import annotations

from typing import Sequence


def percentile(values: Sequence[float], fraction: float) -> float | None:
    """Return the acceptance judge's nearest-index percentile.

    The index is ``round((n - 1) * fraction)`` with Python's ties-to-even
    rounding, clamped to the available range. This preserves the existing
    acceptance-gate definition. NaN and infinity are not filtered; callers
    that require finite-only samples must filter them first.
    """
    if not values:
        return None
    ordered = sorted(values)
    bounded_fraction = max(0.0, min(1.0, float(fraction)))
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * bounded_fraction)))
    return ordered[index]
