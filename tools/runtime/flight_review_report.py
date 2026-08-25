#!/usr/bin/env python3
"""Internal HTML renderer for the single runtime report tool.

The existing runtime report is intentionally useful for debugging, but it is
too dense for a human assessment.  This generator keeps the raw artifacts
unchanged and produces a separate, self-contained report with a
summary-first layout inspired by PX4 Flight Review: clear verdicts, a small
number of interpretable plots, and raw evidence behind collapsed details.
"""

from __future__ import annotations

import html
import json
import math
import statistics
from pathlib import Path
from typing import Any, Iterable

from html_report import _analyze, _finite_number, _load, _point


NAVY = "#18324b"
BLUE = "#1f6feb"
TEAL = "#0f8b8d"
GREEN = "#16825d"
RED = "#c0392b"
ORANGE = "#b7791f"
PURPLE = "#7650a8"
SLATE = "#68778a"
GRID = "#dbe3ea"


def esc(value: Any) -> str:
    return html.escape(str(value), quote=True)


def finite(value: Any) -> float | None:
    return _finite_number(value)


def fmt(value: Any, digits: int = 2, suffix: str = "") -> str:
    number = finite(value)
    if number is None:
        return "—"
    return f"{number:.{digits}f}{suffix}"


def integer(value: Any) -> str:
    number = finite(value)
    return "—" if number is None else f"{int(round(number)):,}"


def percent(value: Any, digits: int = 1) -> str:
    number = finite(value)
    return "—" if number is None else f"{number * 100:.{digits}f}%"


def _series_values(points: list[tuple[float, float]]) -> list[float]:
    return [value for _, value in points if finite(value) is not None]


def _series_percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, round((len(ordered) - 1) * fraction))
    return ordered[index]


def _series_stats(values: list[float]) -> dict[str, Any]:
    return {
        "count": len(values),
        "mean": statistics.fmean(values) if values else None,
        "p50": _series_percentile(values, 0.50),
        "p95": _series_percentile(values, 0.95),
        "p99": _series_percentile(values, 0.99),
        "max": max(values) if values else None,
        "min": min(values) if values else None,
    }


def samples(values: Iterable[tuple[float, float]], limit: int = 280) -> list[tuple[float, float]]:
    ordered = [(float(x), float(y)) for x, y in values if math.isfinite(x) and math.isfinite(y)]
    if len(ordered) <= limit:
        return ordered
    step = (len(ordered) - 1) / (limit - 1)
    result = [ordered[round(index * step)] for index in range(limit)]
    return result


def nice_ticks(low: float, high: float, count: int = 5) -> list[float]:
    if not math.isfinite(low) or not math.isfinite(high) or high <= low:
        return [0.0, 1.0]
    step = (high - low) / max(1, count)
    magnitude = 10 ** math.floor(math.log10(step))
    normalized = step / magnitude
    if normalized <= 1:
        multiplier = 1
    elif normalized <= 2:
        multiplier = 2
    elif normalized <= 5:
        multiplier = 5
    else:
        multiplier = 10
    tick_step = multiplier * magnitude
    start = math.floor(low / tick_step) * tick_step
    end = math.ceil(high / tick_step) * tick_step
    values: list[float] = []
    current = start
    while current <= end + tick_step * 0.01 and len(values) < 20:
        values.append(current)
        current += tick_step
    return values or [low, high]


def tick_label(value: float) -> str:
    if abs(value) >= 100:
        return f"{value:.0f}"
    if abs(value) >= 10:
        return f"{value:.1f}"
    return f"{value:.2f}".rstrip("0").rstrip(".")


def line_chart(
    title: str,
    series: list[dict[str, Any]],
    y_label: str,
    x_label: str = "simulation time (s)",
    threshold: float | None = None,
    threshold_label: str | None = None,
    y_min: float | None = None,
    y_max: float | None = None,
    height: int = 300,
) -> str:
    plotted = [
        (label, samples(points), color, dash)
        for label, points, color, dash in (
            (item["label"], item.get("points", []), item["color"], item.get("dash", ""))
            for item in series
        )
        if points
    ]
    all_points = [point for _, points, _, _ in plotted for point in points]
    if not all_points:
        return '<div class="empty-chart">No samples recorded.</div>'
    min_t = min(point[0] for point in all_points)
    max_t = max(point[0] for point in all_points)
    if max_t <= min_t:
        max_t = min_t + 1.0
    data_min = min(point[1] for point in all_points)
    data_max = max(point[1] for point in all_points)
    allow_negative = (
        data_min < 0.0
        or (y_min is not None and y_min < 0.0)
        or (threshold is not None and threshold < 0.0)
    )
    axis_min = min(0.0, data_min) if y_min is None else y_min
    axis_max = max(0.0, data_max) if y_max is None else y_max
    if threshold is not None:
        axis_min = min(axis_min, threshold)
        axis_max = max(axis_max, threshold)
    if axis_max <= axis_min:
        axis_max = axis_min + 1.0
    padding = max((axis_max - axis_min) * 0.08, 1e-6)
    axis_min -= padding
    axis_max += padding
    if not allow_negative:
        axis_min = 0.0
    left, top, right, bottom = 70, 46, 24, 56
    width = 980
    plot_w = width - left - right
    plot_h = height - top - bottom
    y_ticks = nice_ticks(axis_min, axis_max, 5)
    axis_min = min(axis_min, y_ticks[0])
    axis_max = max(axis_max, y_ticks[-1])

    def point_xy(point: tuple[float, float]) -> tuple[float, float]:
        x = left + (point[0] - min_t) / (max_t - min_t) * plot_w
        y = top + (1.0 - (point[1] - axis_min) / (axis_max - axis_min)) * plot_h
        return x, y

    parts = [
        f'<div class="chart-card"><div class="chart-title">{esc(title)}</div>',
        f'<svg class="chart" viewBox="0 0 {width} {height}" role="img" aria-label="{esc(title)}">',
        f'<title>{esc(title)}</title><rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff"/>',
    ]
    for tick in y_ticks:
        y = top + (1.0 - (tick - axis_min) / (axis_max - axis_min)) * plot_h
        parts.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{width-right}" y2="{y:.1f}" stroke="{GRID}"/>'
            f'<text x="{left-10}" y="{y+4:.1f}" text-anchor="end" class="axis-label">{esc(tick_label(tick))}</text>'
        )
    x_ticks = [min_t + (max_t - min_t) * index / 5 for index in range(6)]
    for tick in x_ticks:
        x = left + (tick - min_t) / (max_t - min_t) * plot_w
        parts.append(
            f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{height-bottom}" stroke="{GRID}"/>'
            f'<text x="{x:.1f}" y="{height-bottom+20}" text-anchor="middle" class="axis-label">{esc(tick_label(tick))}</text>'
        )
    parts.extend([
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}" stroke="{NAVY}" stroke-width="1.2"/>',
        f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="{NAVY}" stroke-width="1.2"/>',
        f'<text x="16" y="{top-16}" class="axis-title">{esc(y_label)}</text>',
        f'<text x="{left + plot_w/2:.1f}" y="{height-10}" text-anchor="middle" class="axis-title">{esc(x_label)}</text>',
    ])
    if threshold is not None:
        y = top + (1.0 - (threshold - axis_min) / (axis_max - axis_min)) * plot_h
        label = threshold_label or f"limit {fmt(threshold)}"
        parts.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{width-right}" y2="{y:.1f}" stroke="{RED}" stroke-width="1.5" stroke-dasharray="7 5"/>'
            f'<text x="{width-right-4}" y="{y-6:.1f}" text-anchor="end" class="threshold-label">{esc(label)}</text>'
        )
    for label, points, color, dash in plotted:
        coordinates = " ".join(f"{point_xy(point)[0]:.1f},{point_xy(point)[1]:.1f}" for point in points)
        dash_attr = f' stroke-dasharray="{esc(dash)}"' if dash else ""
        parts.append(
            f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="2.5" stroke-linejoin="round" stroke-linecap="round"{dash_attr}/>'
        )
        end_x, end_y = point_xy(points[-1])
        parts.append(
            f'<circle cx="{end_x:.1f}" cy="{end_y:.1f}" r="3.5" fill="{color}"/>'
            f'<text x="{min(width-right-4, end_x+8):.1f}" y="{max(top+14, end_y-8):.1f}" class="end-label" fill="{color}">{esc(fmt(points[-1][1]))}</text>'
        )
    legend_x = left
    for label, _, color, _ in plotted:
        parts.append(
            f'<line x1="{legend_x}" y1="22" x2="{legend_x+20}" y2="22" stroke="{color}" stroke-width="3"/>'
            f'<text x="{legend_x+27}" y="26" class="legend-label">{esc(label)}</text>'
        )
        legend_x += 150 + min(120, len(label) * 4)
    parts.append('</svg></div>')
    return "".join(parts)


def map_svg(data: dict[str, Any]) -> str:
    actual = [item["position"] for item in data["ground_truth"]]
    waypoints = data["waypoints"]
    obstacles = data["metrics"].get("obstacles", [])
    all_points = actual + waypoints + [item["center"] for item in obstacles]
    if not all_points:
        return '<div class="empty-chart">No trajectory samples recorded.</div>'
    min_x = min(point[0] for point in all_points) - 3.0
    max_x = max(point[0] for point in all_points) + 3.0
    min_y = min(point[1] for point in all_points) - 3.0
    max_y = max(point[1] for point in all_points) + 3.0
    width, height = 980, 500
    left, top, right, bottom = 68, 38, 22, 54
    plot_w, plot_h = width - left - right, height - top - bottom
    scale = min(plot_w / max(1.0, max_x - min_x), plot_h / max(1.0, max_y - min_y))
    origin_x = left + (plot_w - (max_x - min_x) * scale) / 2
    origin_y = top + (plot_h - (max_y - min_y) * scale) / 2

    def transform(point: tuple[float, float, float]) -> tuple[float, float]:
        return origin_x + (point[0] - min_x) * scale, height - bottom - (point[1] - min_y) * scale - (plot_h - (max_y - min_y) * scale) / 2

    x_ticks = nice_ticks(min_x, max_x, 5)
    y_ticks = nice_ticks(min_y, max_y, 5)
    parts = [
        '<div class="chart-card"><div class="chart-title">Observed flight path and route geometry</div>',
        '<svg class="chart" viewBox="0 0 980 500" role="img" aria-label="2D flight path">',
        '<title>Observed flight path and route geometry</title><rect x="0" y="0" width="980" height="500" fill="#ffffff"/>',
    ]
    for tick in x_ticks:
        if tick < min_x - 1e-9 or tick > max_x + 1e-9:
            continue
        x, _ = transform((tick, min_y, 0.0))
        parts.append(
            f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{height-bottom}" stroke="{GRID}"/>'
            f'<text x="{x:.1f}" y="{height-bottom+20}" text-anchor="middle" class="axis-label">{esc(tick_label(tick))}</text>'
        )
    for tick in y_ticks:
        if tick < min_y - 1e-9 or tick > max_y + 1e-9:
            continue
        _, y = transform((min_x, tick, 0.0))
        parts.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{width-right}" y2="{y:.1f}" stroke="{GRID}"/>'
            f'<text x="{left-10}" y="{y+4:.1f}" text-anchor="end" class="axis-label">{esc(tick_label(tick))}</text>'
        )
    parts.extend([
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}" stroke="{NAVY}" stroke-width="1.2"/>',
        f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="{NAVY}" stroke-width="1.2"/>',
        f'<text x="{left + plot_w/2:.1f}" y="{height-10}" text-anchor="middle" class="axis-title">world X (m)</text>',
        f'<text x="16" y="{top-12}" class="axis-title">world Y (m)</text>',
    ])
    route_obstacles = set(data["metrics"].get("route_obstacles", []))
    for obstacle in obstacles:
        center = transform(obstacle["center"])
        is_route = obstacle["name"] in route_obstacles
        fill = "#e8897e" if is_route else "#cbd5df"
        stroke = RED if is_route else SLATE
        if obstacle["type"] == "cylinder":
            radius = float(obstacle.get("radius_m", 0.0)) * scale
            parts.append(
                f'<circle cx="{center[0]:.1f}" cy="{center[1]:.1f}" r="{radius:.1f}" fill="{fill}" fill-opacity="0.72" stroke="{stroke}" stroke-width="1.2"/>'
            )
        else:
            half = obstacle.get("half_extents", [0.0, 0.0, 0.0])
            top_left = transform((obstacle["center"][0] - half[0], obstacle["center"][1] + half[1], 0.0))
            bottom_right = transform((obstacle["center"][0] + half[0], obstacle["center"][1] - half[1], 0.0))
            parts.append(
                f'<rect x="{top_left[0]:.1f}" y="{top_left[1]:.1f}" width="{bottom_right[0]-top_left[0]:.1f}" height="{bottom_right[1]-top_left[1]:.1f}" fill="{fill}" fill-opacity="0.72" stroke="{stroke}"/>'
            )
        if is_route:
            parts.append(f'<text x="{center[0]+8:.1f}" y="{center[1]-8:.1f}" class="map-label">{esc(obstacle["name"])}</text>')
    actual_sampled = samples([(item["position"][0], item["position"][1]) for item in data["ground_truth"]], 500)
    actual_coords = " ".join(f"{transform((x, y, 0.0))[0]:.1f},{transform((x, y, 0.0))[1]:.1f}" for x, y in actual_sampled)
    if actual_coords:
        parts.append(f'<polyline points="{actual_coords}" fill="none" stroke="{BLUE}" stroke-width="3" stroke-linejoin="round" stroke-linecap="round"/>')
    records = data.get("trajectory_records", [])
    record_step = max(1, len(records) // 36)
    for index, record in enumerate(records):
        if index % record_step != 0 and index != len(records) - 1:
            continue
        points = []
        for value in record.get("position_points", []):
            if isinstance(value, (list, tuple)) and len(value) >= 3:
                try:
                    point = (float(value[0]), float(value[1]), float(value[2]))
                except (TypeError, ValueError):
                    continue
                if all(math.isfinite(item) for item in point):
                    points.append(point)
        if len(points) >= 2:
            coordinates = " ".join(f"{transform(point)[0]:.1f},{transform(point)[1]:.1f}" for point in points)
            parts.append(f'<polyline points="{coordinates}" fill="none" stroke="{TEAL}" stroke-opacity="0.36" stroke-width="1.2" stroke-dasharray="5 4"/>')
    for index, waypoint in enumerate(waypoints):
        x, y = transform(waypoint)
        parts.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="#f0a51a" stroke="#7b4e00" stroke-width="1.2"/>'
            f'<text x="{x+8:.1f}" y="{y-8:.1f}" class="map-label">WP{index}</text>'
        )
    parts.extend([
        f'<line x1="{left+12}" y1="22" x2="{left+32}" y2="22" stroke="{BLUE}" stroke-width="3"/><text x="{left+39}" y="26" class="legend-label">ground truth</text>',
        f'<line x1="{left+150}" y1="22" x2="{left+170}" y2="22" stroke="{TEAL}" stroke-opacity="0.6" stroke-width="2" stroke-dasharray="5 4"/><text x="{left+177}" y="26" class="legend-label">published plans</text>',
        f'<rect x="{left+310}" y="16" width="14" height="12" fill="#e8897e" stroke="{RED}"/><text x="{left+331}" y="26" class="legend-label">route obstacles</text>',
        f'<circle cx="{left+486}" cy="22" r="5" fill="#f0a51a" stroke="#7b4e00"/><text x="{left+499}" y="26" class="legend-label">waypoints</text>',
        '</svg></div>',
    ])
    return "".join(parts)


def comparison_bars(items: list[tuple[str, float | None, str, str]], title: str) -> str:
    valid = [(label, value, color, note) for label, value, color, note in items if value is not None]
    if not valid:
        return '<div class="empty-chart">No comparison data recorded.</div>'
    width, height = 980, 210
    left, top, right, bottom = 230, 30, 50, 38
    max_value = max(value for _, value, _, _ in valid) or 1.0
    max_value *= 1.15
    parts = [
        f'<div class="chart-card"><div class="chart-title">{esc(title)}</div>',
        f'<svg class="chart" viewBox="0 0 {width} {height}" role="img" aria-label="{esc(title)}">',
        f'<title>{esc(title)}</title><rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff"/>',
    ]
    for index in range(5):
        tick = max_value * index / 4
        x = left + tick / max_value * (width - left - right)
        parts.append(
            f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{height-bottom}" stroke="{GRID}"/>'
            f'<text x="{x:.1f}" y="{height-bottom+20}" text-anchor="middle" class="axis-label">{esc(tick_label(tick))}</text>'
        )
    bar_h = 30
    gap = 18
    for index, (label, value, color, note) in enumerate(valid):
        y = top + index * (bar_h + gap)
        bar_width = value / max_value * (width - left - right)
        parts.append(
            f'<text x="{left-12}" y="{y+21}" text-anchor="end" class="axis-label">{esc(label)}</text>'
            f'<rect x="{left}" y="{y}" width="{bar_width:.1f}" height="{bar_h}" rx="5" fill="{color}"/>'
            f'<text x="{min(width-right, left+bar_width+10):.1f}" y="{y+21}" class="bar-value">{esc(fmt(value, 1, " m"))}</text>'
            f'<text x="{left}" y="{y+bar_h+13}" class="bar-note">{esc(note)}</text>'
        )
    parts.append('</svg></div>')
    return "".join(parts)


def status_class(status: str) -> str:
    return {"PASS": "pass", "FAIL": "fail", "INCOMPLETE": "observe", "N/A": "na", "OBSERVE": "observe", "INFO": "info"}.get(status, "info")


def status_chip(status: str, label: str | None = None) -> str:
    return f'<span class="status {status_class(status)}">{esc(label or status)}</span>'


def _bool_value(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "yes", "1"}:
            return True
        if lowered in {"false", "no", "0"}:
            return False
    return None


def _gate_status(condition: bool | None) -> str:
    if condition is None:
        return "N/A"
    return "PASS" if condition else "FAIL"


def _evaluation(
    report: dict[str, Any],
    safety: dict[str, Any],
    acceptance: dict[str, Any],
    lio: dict[str, Any],
    px4: dict[str, Any],
    cross_p95: float | None,
    cross_limit: float | None,
    collision_count: float | None,
) -> dict[str, Any]:
    """Compute display gates from recorded evidence, never from presentation defaults."""
    outcome = str(safety.get("outcome") or "").upper()
    expected = str(acceptance.get("expected_outcome") or "complete").lower()
    if expected == "fail_closed":
        mission_condition = None if not outcome else outcome in {"PAUSED_SAFETY_STOP", "FAILED_COMPONENT"}
    else:
        complete = _bool_value(acceptance.get("mission_complete_observed"))
        mission_condition = None if complete is None else complete and outcome == "COMPLETE"

    waypoint_condition = _bool_value(acceptance.get("waypoint_acceptance_complete"))
    waypoint_reasons = acceptance.get("reasons", [])
    if (
        waypoint_condition is False
        and not acceptance.get("waypoint_acceptance_indices")
        and isinstance(waypoint_reasons, list)
        and any("unavailable" in str(reason).lower() for reason in waypoint_reasons)
    ):
        waypoint_condition = None
    cross_condition = (
        None if cross_p95 is None or cross_limit is None else cross_p95 <= cross_limit
    )
    collision_condition = None if collision_count is None else collision_count == 0.0

    lio_state = lio.get("state")
    lio_valid = _bool_value(lio.get("navigation_valid"))
    lio_condition = None if lio_state is None or lio_valid is None else str(lio_state) == "TRACKING" and lio_valid

    px4_keys = ("estimator_initialized", "local_position_valid", "local_velocity_valid")
    px4_values = [_bool_value(px4.get(key)) for key in px4_keys]
    px4_condition = None if any(value is None for value in px4_values) else all(px4_values)
    telemetry_verdict = str(report.get("verdict") or "").upper()
    runtime_contract_condition = (
        True if telemetry_verdict == "PASS"
        else False if telemetry_verdict == "FAIL"
        else None
    )

    gates = {
        "runtime_contract": _gate_status(runtime_contract_condition),
        "mission": _gate_status(mission_condition),
        "waypoint": _gate_status(waypoint_condition),
        "cross_track": _gate_status(cross_condition),
        "collision": _gate_status(collision_condition),
        "lio": _gate_status(lio_condition),
        "px4": _gate_status(px4_condition),
    }
    # An explicitly recorded temporary bypass is never a certification result,
    # even if mission/telemetry gates happen to pass.  Empty metadata preserves
    # legacy reports; malformed/non-empty metadata fails closed.
    bypasses = report.get("experimental_bypasses")
    bypass_active = bypasses not in (None, {})
    gates["temporary_bypass"] = _gate_status(False if bypass_active else True)
    required = list(gates.values())
    overall = "FAIL" if "FAIL" in required else "PASS" if all(item == "PASS" for item in required) else "INCOMPLETE"
    telemetry_verdict = telemetry_verdict or "N/A"
    return {"overall": overall, "telemetry_verdict": telemetry_verdict, "gates": gates}


def _timing_rows(report: dict[str, Any]) -> list[dict[str, Any]]:
    """Return only measured processing-time rows; absent telemetry stays absent."""
    rows: list[dict[str, Any]] = []

    def add_distribution(component: str, metric: str, value: Any, unit: str = "us") -> None:
        if not isinstance(value, dict):
            return
        measured = any(
            finite(value.get(key)) is not None
            for key in ("mean", "p50", "p95", "p99", "max", "maximum", "maximum_maintenance_us")
        )
        if not measured:
            return
        rows.append({
            "component": component,
            "metric": metric.replace("_", " "),
            "unit": unit,
            "mean": value.get("mean"),
            "p50": value.get("p50"),
            "p95": value.get("p95"),
            "p99": value.get("p99"),
            "max": value.get("max", value.get("maximum", value.get("maximum_maintenance_us"))),
            "count": value.get("sample_count", value.get("count")),
        })

    def add_timing_metrics(component: str, timing_metrics: dict[str, Any]) -> None:
        for metric, value in sorted(timing_metrics.items()):
            if not metric.endswith("_us") and not metric.endswith("_ms"):
                continue
            add_distribution(component, metric, value, "us" if metric.endswith("_us") else "ms")

    lio = report.get("lio", {}) if isinstance(report.get("lio"), dict) else {}
    add_distribution("LIO", "map maintenance", lio.get("map_maintenance"), "us")
    mapping = report.get("navigation_mapping", {}) if isinstance(report.get("navigation_mapping"), dict) else {}
    add_timing_metrics("ROG-Map", mapping.get("timing_distributions") or {})
    planning = report.get("planning", {}) if isinstance(report.get("planning"), dict) else {}
    add_timing_metrics("Planner", planning)
    external = report.get("external_mode", {}) if isinstance(report.get("external_mode"), dict) else {}
    add_distribution("External Mode", "fallback latency", external.get("fallback_latency_ms"), "ms")
    return rows


def _replay_point(value: Any) -> list[float] | None:
    point = _point(value)
    return list(point) if point is not None else None


def _replay_sampled(values: list[dict[str, Any]], limit: int) -> list[dict[str, Any]]:
    if len(values) <= limit:
        return values
    step = (len(values) - 1) / max(1, limit - 1)
    return [values[round(index * step)] for index in range(limit)]


def _replay_payload(data: dict[str, Any]) -> dict[str, Any]:
    """Build a compact timeline without embedding raw recorder streams."""
    ground_truth: list[dict[str, Any]] = []
    for item in data.get("ground_truth", []):
        position = _replay_point(item.get("position"))
        if position is None or finite(item.get("t")) is None:
            continue
        velocity = _replay_point(item.get("velocity"))
        speed = math.sqrt(sum(value * value for value in velocity)) if velocity else None
        ground_truth.append({
            "t": finite(item.get("t")),
            "p": position,
            "s": speed if speed is not None and math.isfinite(speed) else None,
        })
    ground_truth = _replay_sampled(ground_truth, 720)

    plans: list[dict[str, Any]] = []
    for item in data.get("trajectory_records", []):
        publish_time = finite(item.get("_publish_time_s"))
        if publish_time is None:
            continue
        positions = [
            point
            for point in (_replay_point(value) for value in item.get("position_points", []))
            if point is not None
        ]
        if not positions:
            continue
        positions = [
            positions[round(index * (len(positions) - 1) / max(1, min(48, len(positions)) - 1))]
            for index in range(min(48, len(positions)))
        ]
        role = int(finite(item.get("trajectory_role")) or 0)
        safety_kind = int(finite(item.get("safety_plan_kind")) or 0)
        plans.append({
            "t": publish_time,
            "duration": finite(item.get("duration_s")) or 0.0,
            "role": role,
            "role_label": "safety" if role == 1 else "nominal",
            "safety_kind": safety_kind,
            "waypoint": int(finite(item.get("waypoint_index")) or 0),
            "world_revision": int(finite(item.get("world_revision")) or 0),
            "points": positions,
        })
    plans.sort(key=lambda item: item["t"])

    planner: list[dict[str, Any]] = []
    for item in data.get("planning", []):
        timestamp = finite(item.get("t"))
        if timestamp is None:
            continue
        planner.append({
            "t": timestamp,
            "horizon": finite(item.get("planning_horizon_distance_m")),
            "known_free": finite(item.get("known_free_horizon_m")),
            "reason": str(item.get("replan_reason") or "—"),
            "plan_role": str(item.get("plan_role") or "—"),
            "safety_kind": str(item.get("safety_plan_kind") or "—"),
            "goal": [
                finite(item.get("effective_goal_x")),
                finite(item.get("effective_goal_y")),
                finite(item.get("effective_goal_z")),
            ],
        })
    planner.sort(key=lambda item: item["t"])

    obstacles = []
    bounds_points: list[list[float]] = [item["p"] for item in ground_truth]
    bounds_points.extend(list(point) for point in data.get("waypoints", []))
    for obstacle in data.get("metrics", {}).get("obstacles", []):
        center = _replay_point(obstacle.get("center"))
        if center is None:
            continue
        output = {"name": str(obstacle.get("name", "")), "type": str(obstacle.get("type", "")), "center": center}
        if obstacle.get("type") == "cylinder":
            output["radius"] = finite(obstacle.get("radius_m")) or 0.0
        else:
            half = [finite(value) or 0.0 for value in obstacle.get("half_extents", [0.0, 0.0, 0.0])]
            output["half"] = half
            bounds_points.extend([
                [center[0] - half[0], center[1] - half[1], center[2]],
                [center[0] + half[0], center[1] + half[1], center[2]],
            ])
        obstacles.append(output)
    for plan in plans:
        bounds_points.extend(plan["points"])
    if not bounds_points:
        bounds_points = [[0.0, 0.0, 0.0]]
    min_x = min(point[0] for point in bounds_points) - 3.0
    max_x = max(point[0] for point in bounds_points) + 3.0
    min_y = min(point[1] for point in bounds_points) - 3.0
    max_y = max(point[1] for point in bounds_points) + 3.0
    timestamps = [item["t"] for item in ground_truth + plans + planner]
    start = min(timestamps) if timestamps else 0.0
    end = max(timestamps) if timestamps else 1.0
    return {
        "ground_truth": ground_truth,
        "plans": plans,
        "planner": planner,
        "waypoints": [list(point) for point in data.get("waypoints", [])],
        "obstacles": obstacles,
        "route_obstacles": list(data.get("metrics", {}).get("route_obstacles", [])),
        "bounds": {"min_x": min_x, "max_x": max_x, "min_y": min_y, "max_y": max_y},
        "start": start,
        "end": end,
    }


_REPLAY_SCRIPT = r"""
(function () {
  const data = __REPLAY_DATA__;
  const svg = document.getElementById("replay-map");
  const slider = document.getElementById("replay-time");
  const output = document.getElementById("replay-time-value");
  const playButton = document.getElementById("replay-play");
  const resetButton = document.getElementById("replay-reset");
  const markerTrack = document.getElementById("replay-plan-events");
  const W = 980, H = 500, left = 68, top = 38, right = 22, bottom = 54;
  const plotW = W - left - right, plotH = H - top - bottom;
  const b = data.bounds;
  const scale = Math.min(plotW / Math.max(1, b.max_x - b.min_x), plotH / Math.max(1, b.max_y - b.min_y));
  const offsetX = left + (plotW - (b.max_x - b.min_x) * scale) / 2;
  const offsetY = top + (plotH - (b.max_y - b.min_y) * scale) / 2;
  let playing = false;
  let animationFrame = null;
  let lastFrame = null;
  let currentTime = data.start;
  const markers = [];

  function world(point) {
    return [offsetX + (point[0] - b.min_x) * scale,
      H - bottom - (point[1] - b.min_y) * scale - offsetY + top];
  }
  function svgNode(tag, attrs, parent) {
    const node = document.createElementNS("http://www.w3.org/2000/svg", tag);
    Object.entries(attrs || {}).forEach(([key, value]) => node.setAttribute(key, String(value)));
    if (parent) parent.appendChild(node);
    return node;
  }
  function svgText(parent, x, y, value, className, anchor) {
    const node = svgNode("text", {x, y, "class": className || "replay-svg-label", "text-anchor": anchor || "start"}, parent);
    node.textContent = value;
    return node;
  }
  function clear(node) { while (node.firstChild) node.removeChild(node.firstChild); }
  function fmt(value, digits) { return Number.isFinite(Number(value)) ? Number(value).toFixed(digits == null ? 2 : digits) : "—"; }
  function latestAt(list, time) {
    let low = 0, high = list.length - 1, answer = null;
    while (low <= high) {
      const mid = Math.floor((low + high) / 2);
      if (list[mid].t <= time + 1e-9) { answer = list[mid]; low = mid + 1; }
      else high = mid - 1;
    }
    return answer;
  }
  function nearestGround(time) {
    if (!data.ground_truth.length) return null;
    let low = 0, high = data.ground_truth.length - 1;
    while (low < high) {
      const mid = Math.floor((low + high) / 2);
      if (data.ground_truth[mid].t < time) low = mid + 1; else high = mid;
    }
    const next = data.ground_truth[low], previous = data.ground_truth[Math.max(0, low - 1)];
    return Math.abs(next.t - time) < Math.abs(previous.t - time) ? next : previous;
  }
  function polyline(parent, points, attrs) {
    if (!points || points.length < 2) return;
    svgNode("polyline", Object.assign({fill: "none", "stroke-linejoin": "round", "stroke-linecap": "round"}, attrs, {
      points: points.map(world).map(point => point.map(value => value.toFixed(1)).join(",")).join(" ")
    }), parent);
  }

  const staticLayer = svgNode("g", {}, svg);
  const trailLayer = svgNode("g", {}, svg);
  const activeLayer = svgNode("g", {}, svg);
  const vehicleLayer = svgNode("g", {}, svg);
  svg.setAttribute("viewBox", `0 0 ${W} ${H}`);

  function drawStatic() {
    svgNode("rect", {x: 0, y: 0, width: W, height: H, fill: "#ffffff"}, staticLayer);
    for (let i = 0; i <= 5; i += 1) {
      const x = left + plotW * i / 5;
      const value = b.min_x + (b.max_x - b.min_x) * i / 5;
      svgNode("line", {x1: x, y1: top, x2: x, y2: H - bottom, stroke: "#dbe3ea"}, staticLayer);
      svgText(staticLayer, x, H - bottom + 20, fmt(value, 1), "replay-svg-label", "middle");
      const y = top + plotH * i / 5;
      const yValue = b.max_y - (b.max_y - b.min_y) * i / 5;
      svgNode("line", {x1: left, y1: y, x2: W - right, y2: y, stroke: "#dbe3ea"}, staticLayer);
      svgText(staticLayer, left - 10, y + 4, fmt(yValue, 1), "replay-svg-label", "end");
    }
    svgNode("line", {x1: left, y1: top, x2: left, y2: H - bottom, stroke: "#18324b", "stroke-width": 1.2}, staticLayer);
    svgNode("line", {x1: left, y1: H - bottom, x2: W - right, y2: H - bottom, stroke: "#18324b", "stroke-width": 1.2}, staticLayer);
    svgText(staticLayer, left + plotW / 2, H - 10, "world X (m)", "replay-svg-title", "middle");
    svgText(staticLayer, 16, top - 12, "world Y (m)", "replay-svg-title", "start");
    const routeObstacles = new Set(data.route_obstacles || []);
    data.obstacles.forEach(obstacle => {
      const center = world(obstacle.center), isRoute = routeObstacles.has(obstacle.name);
      const fill = isRoute ? "#e8897e" : "#cbd5df", stroke = isRoute ? "#c0392b" : "#68778a";
      if (obstacle.type === "cylinder") {
        svgNode("circle", {cx: center[0], cy: center[1], r: (obstacle.radius || 0) * scale, fill, "fill-opacity": .72, stroke}, staticLayer);
      } else {
        const half = obstacle.half || [0, 0, 0];
        const a = world([obstacle.center[0] - half[0], obstacle.center[1] + half[1], 0]);
        const z = world([obstacle.center[0] + half[0], obstacle.center[1] - half[1], 0]);
        svgNode("rect", {x: a[0], y: a[1], width: z[0] - a[0], height: z[1] - a[1], fill, "fill-opacity": .72, stroke}, staticLayer);
      }
      if (isRoute) svgText(staticLayer, center[0] + 8, center[1] - 8, obstacle.name, "replay-map-label");
    });
    data.waypoints.forEach((point, index) => {
      const xy = world(point);
      svgNode("circle", {cx: xy[0], cy: xy[1], r: 5, fill: "#f0a51a", stroke: "#7b4e00"}, staticLayer);
      svgText(staticLayer, xy[0] + 8, xy[1] - 8, `WP${index}`, "replay-map-label");
    });
  }

  function updateStatus(time, ground, plan, planner) {
    output.textContent = `${fmt(time, 2)} s`;
    document.getElementById("replay-time-value-side").textContent = output.textContent;
    document.getElementById("replay-position").textContent = ground ? `${fmt(ground.p[0], 2)}, ${fmt(ground.p[1], 2)}, ${fmt(ground.p[2], 2)} m` : "—";
    document.getElementById("replay-speed").textContent = ground && ground.s != null ? `${fmt(ground.s, 2)} m/s` : "—";
    document.getElementById("replay-plan").textContent = plan ? `${plan.role_label} · published ${fmt(plan.t, 2)} s · ${fmt(plan.duration, 2)} s` : "No generated path yet";
    document.getElementById("replay-plan-meta").textContent = plan ? `WP${plan.waypoint} · world revision ${plan.world_revision} · safety kind ${plan.safety_kind}` : "—";
    document.getElementById("replay-planner").textContent = planner ? `${planner.reason} · horizon ${fmt(planner.horizon, 2)} m · known-free ${fmt(planner.known_free, 2)} m` : "No planner diagnostic yet";
  }

  function update(time) {
    currentTime = Math.max(data.start, Math.min(data.end, Number(time)));
    slider.value = currentTime;
    const ground = nearestGround(currentTime), plan = latestAt(data.plans, currentTime), planner = latestAt(data.planner, currentTime);
    clear(trailLayer); clear(activeLayer); clear(vehicleLayer);
    const trail = data.ground_truth.filter(item => item.t <= currentTime + 1e-9).map(item => item.p);
    polyline(trailLayer, trail, {stroke: "#1f6feb", "stroke-width": 3});
    if (plan) {
      const color = plan.role === 1 ? "#c0392b" : "#0f8b8d";
      polyline(activeLayer, plan.points, {stroke: color, "stroke-width": 3, "stroke-dasharray": "7 5", "stroke-opacity": .86});
      const end = world(plan.points[plan.points.length - 1]);
      svgNode("circle", {cx: end[0], cy: end[1], r: 5, fill: color, stroke: "#ffffff", "stroke-width": 2}, activeLayer);
      svgText(activeLayer, end[0] + 8, end[1] - 8, `${plan.role_label} path`, "replay-map-label");
    }
    if (ground) {
      const point = world(ground.p);
      svgNode("circle", {cx: point[0], cy: point[1], r: 7, fill: "#1f6feb", stroke: "#ffffff", "stroke-width": 2}, vehicleLayer);
      const previous = nearestGround(currentTime - .15);
      if (previous && previous !== ground) {
        const from = world(previous.p), dx = point[0] - from[0], dy = point[1] - from[1], length = Math.hypot(dx, dy);
        if (length > 1) svgNode("line", {x1: point[0], y1: point[1], x2: point[0] + dx / length * 16, y2: point[1] + dy / length * 16, stroke: "#1f6feb", "stroke-width": 3}, vehicleLayer);
      }
    }
    data.plans.forEach((item, index) => {
      if (markers[index]) markers[index].classList.toggle("active", item === plan);
    });
    updateStatus(currentTime, ground, plan, planner);
  }

  function animate(timestamp) {
    if (!playing) return;
    if (lastFrame == null) lastFrame = timestamp;
    currentTime += (timestamp - lastFrame) / 1000;
    lastFrame = timestamp;
    if (currentTime >= data.end) { currentTime = data.end; playing = false; playButton.textContent = "Play"; }
    update(currentTime);
    if (playing) animationFrame = requestAnimationFrame(animate);
  }
  function setPlaying(value) {
    playing = value;
    playButton.textContent = playing ? "Pause" : "Play";
    lastFrame = null;
    if (playing) animationFrame = requestAnimationFrame(animate);
    else if (animationFrame != null) cancelAnimationFrame(animationFrame);
  }
  data.plans.forEach(plan => {
    const marker = document.createElement("button");
    marker.type = "button";
    marker.className = `replay-marker ${plan.role === 1 ? "safety" : "nominal"}`;
    marker.style.left = `${(plan.t - data.start) / Math.max(1e-9, data.end - data.start) * 100}%`;
    marker.title = `${plan.role_label} path at ${fmt(plan.t, 2)} s`;
    marker.addEventListener("click", () => { setPlaying(false); update(plan.t); });
    markerTrack.appendChild(marker);
    markers.push(marker);
  });
  slider.min = data.start; slider.max = data.end; slider.step = .01; slider.value = data.start;
  slider.addEventListener("input", event => { setPlaying(false); update(event.target.value); });
  playButton.addEventListener("click", () => setPlaying(!playing));
  resetButton.addEventListener("click", () => { setPlaying(false); update(data.start); });
  drawStatic(); update(data.start);
})();
"""


def replay_section(data: dict[str, Any]) -> str:
    payload = _replay_payload(data)
    if not payload["ground_truth"]:
        return ""
    if payload["plans"]:
        replay_description = (
            "Scrub or play the recorded timeline. Blue is observed ground truth; "
            "the dashed line is the latest generated trajectory available at that time. "
            "Click a colored marker to jump to a path publication."
        )
    else:
        replay_description = (
            "Only observed ground truth is available in this recording; trajectory "
            "publication timestamps were not captured, so no generated path is inferred."
        )
    encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
    script = _REPLAY_SCRIPT.replace("__REPLAY_DATA__", encoded)
    return f"""
  <section id="flight-replay" class="replay-section">
    <h2>Flight replay · movement and generated paths</h2>
    <p class="small">{replay_description}</p>
    <div class="replay-toolbar">
      <button id="replay-play" type="button">Play</button><button id="replay-reset" type="button">Reset</button>
      <label class="replay-time-control" for="replay-time">simulation time <output id="replay-time-value">—</output><input id="replay-time" type="range" aria-label="Replay simulation time"></label>
    </div>
    <div class="replay-event-caption"><span>path publication timeline</span><span><i class="replay-swatch nominal"></i> nominal <i class="replay-swatch safety"></i> safety</span></div>
    <div id="replay-plan-events" class="replay-plan-events" aria-label="Trajectory publication events"></div>
    <div class="replay-grid">
      <div class="replay-map-card"><svg id="replay-map" class="replay-map" role="img" aria-label="Interactive 2D flight replay"><title>Interactive 2D flight replay</title></svg><div class="replay-legend"><span><i class="replay-swatch ground"></i> observed UAV</span><span><i class="replay-swatch nominal"></i> active nominal path</span><span><i class="replay-swatch safety"></i> active safety path</span></div></div>
      <aside class="replay-status"><h3>State at cursor</h3><dl><dt>Time</dt><dd id="replay-time-value-side">—</dd><dt>UAV position</dt><dd id="replay-position">—</dd><dt>UAV speed</dt><dd id="replay-speed">—</dd><dt>Active path</dt><dd id="replay-plan">—</dd><dt>Path metadata</dt><dd id="replay-plan-meta">—</dd><dt>Planner</dt><dd id="replay-planner">—</dd></dl><p class="small">The replay shows the latest published path, not a physics re-simulation. Path publication markers are the recorded planner/executor timeline.</p></aside>
    </div>
    <script>
      {script}
    </script>
  </section>
"""


def render(session: Path, output: Path) -> Path:
    data = _analyze(session)
    report = _load(session / "report.json", {})
    metrics = data["metrics"]
    # Preserve the existing machine-readable companion artifact while making
    # REPORT.html human-readable.  The HTML is a view, not a replacement for
    # the structured metrics contract.
    (session / "benchmark_metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    mission = metrics["mission"]
    tracking = metrics["tracking"]
    planning = metrics["planning"]
    safety = metrics["safety"]
    smoothness = metrics["smoothness"]
    acceptance = metrics["acceptance"]
    external = report.get("external_mode", {}) if isinstance(report, dict) else {}
    lio = report.get("lio", {}) if isinstance(report, dict) else {}
    px4 = report.get("px4", {}) if isinstance(report, dict) else {}

    # A report verdict is the aggregate telemetry verdict, not the mission's
    # observed outcome.  Keep the two separate so a missing scenario outcome
    # cannot be displayed as a real mission state.
    outcome = str(safety.get("outcome") or "N/A")
    accepted_indices = acceptance.get("waypoint_acceptance_indices")
    if not isinstance(accepted_indices, list):
        accepted_indices = []
    waypoint_count = int(mission.get("waypoint_count") or 0)
    accepted_count = len(accepted_indices)
    cross_track = tracking.get("cross_track_error_m", {})
    cross_p95 = finite(cross_track.get("p95"))
    cross_limit = finite(acceptance.get("max_cross_track_p95_m"))
    collision_count = finite(safety.get("collision_count"))
    min_clearance = finite(safety.get("minimum_collision_clearance_m"))
    known_free = planning.get("known_free_horizon_m", {})
    continuity = planning.get("continuity", {})
    safety_stop_ratio = finite(continuity.get("safety_stop_ratio"))
    trace = planning.get("rolling_bundle_trace", {})
    provenance = report.get("provenance", {}) if isinstance(report, dict) else {}
    manifest = provenance.get("manifest", {}) if isinstance(provenance, dict) else {}
    source_identity = manifest.get("source", {}) if isinstance(manifest, dict) else {}
    navigation_commit = (
        provenance.get("navigation_commit")
        if isinstance(provenance, dict) else None
    ) or source_identity.get("git_head") or "unknown"
    navigation_dirty = (
        provenance.get("navigation_dirty")
        if isinstance(provenance, dict) and "navigation_dirty" in provenance
        else source_identity.get("git_dirty")
    )
    trace_records = trace.get("records", []) if isinstance(trace, dict) else []
    zero_trace_fields: list[str] = []
    for field in ("horizon_arc_m", "horizon_start_arc_m", "horizon_end_arc_m", "splice_position_residual_m"):
        values = [finite(record.get(field)) for record in trace_records if isinstance(record, dict)]
        if values and all(value == 0.0 for value in values):
            zero_trace_fields.append(field)
    expected_outcome = str(acceptance.get("expected_outcome") or "complete").lower()

    failure_reasons: list[str] = []
    for source in (
        acceptance.get("reasons"),
        external.get("failures"),
        report.get("reasons"),
    ):
        if isinstance(source, list):
            for reason in source:
                text = str(reason).strip()
                if text and text not in failure_reasons:
                    failure_reasons.append(text)

    evaluation = _evaluation(
        report, safety, acceptance, lio, px4, cross_p95, cross_limit, collision_count
    )
    gates = evaluation["gates"]
    findings = [
        (gates["mission"], f"Mission outcome: {outcome or 'N/A'}; expected {expected_outcome}."),
        (gates["waypoint"], f"Waypoint acceptance: {accepted_count}/{waypoint_count}; only explicit acceptance events are counted."),
        (gates["cross_track"], f"Cross-track p95: {fmt(cross_p95, 2, ' m')} against limit {fmt(cross_limit, 2, ' m')}."),
        (gates["collision"], f"Collision safety: {fmt(collision_count, 0)} collisions; minimum clearance {fmt(min_clearance, 2, ' m')}."),
    ]
    if gates["mission"] == "PASS" and evaluation["overall"] != "PASS":
        findings.append((
            "OBSERVE",
            f"Mission outcome is PASS, but acceptance is {evaluation['overall']}; check quality gates below.",
        ))
    if failure_reasons:
        findings.append(("OBSERVE", "Failure context: " + "; ".join(failure_reasons[:2]) + ("; ..." if len(failure_reasons) > 2 else "")))
    if zero_trace_fields:
        findings.append(("OBSERVE", f"The rolling trace contains constant zero values for {', '.join(zero_trace_fields)}; those fields are treated as unpopulated, not as measured zero residuals."))

    path_points = data["ground_truth"]
    # Use the same segment-distance calculation as the benchmark generator,
    # but keep this file independent from the report HTML implementation.
    def segment_distance(point: tuple[float, float, float], start: tuple[float, float, float], end: tuple[float, float, float]) -> float:
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            return math.hypot(point[0] - start[0], point[1] - start[1])
        projection = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq
        projection = max(0.0, min(1.0, projection))
        return math.hypot(point[0] - (start[0] + projection * dx), point[1] - (start[1] + projection * dy))

    error_series = []
    speed_series = []
    velocity_series = {"vx": [], "vy": [], "vz": []}
    for item in path_points:
        if len(data["waypoints"]) >= 2:
            error_series.append((item["t"], min(
                segment_distance(item["position"], data["waypoints"][index], data["waypoints"][index + 1])
                for index in range(len(data["waypoints"]) - 1)
            )))
        velocity = item.get("velocity")
        if velocity:
            speed_series.append((item["t"], math.sqrt(sum(value * value for value in velocity))))
            for axis, index in zip(("vx", "vy", "vz"), range(3)):
                value = finite(velocity[index]) if len(velocity) > index else None
                if value is not None:
                    velocity_series[axis].append((item["t"], value))
    planning_horizon_series = [
        (item["t"], value)
        for item in data["planning"]
        if (value := finite(item.get("planning_horizon_distance_m"))) is not None
    ]
    known_horizon_series = [
        (item["t"], value)
        for item in data["planning"]
        if (value := finite(item.get("known_free_horizon_m"))) is not None
    ]
    setpoint_velocity_series = {"vx": [], "vy": [], "vz": []}
    setpoint_speed_series = []
    for item in data["trajectory_records"]:
        publish_time = finite(item.get("_publish_time_s"))
        velocities = [value for value in item.get("velocity_points", []) if isinstance(value, (list, tuple)) and len(value) >= 3]
        if publish_time is not None and velocities:
            try:
                velocity = [float(value) for value in velocities[0][:3]]
                for axis, index in zip(("vx", "vy", "vz"), range(3)):
                    setpoint_value = finite(velocity[index])
                    if setpoint_value is not None:
                        setpoint_velocity_series[axis].append((publish_time, setpoint_value))
                value = math.sqrt(sum(component * component for component in velocity))
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                setpoint_speed_series.append((publish_time, value))

    speed_series_stats = _series_stats(_series_values(speed_series))
    setpoint_speed_stats = _series_stats(_series_values(setpoint_speed_series))
    axis_stats = {axis: _series_stats([abs(value) for _, value in points]) for axis, points in velocity_series.items()}
    setpoint_axis_stats = {
        axis: _series_stats([abs(value) for _, value in points])
        for axis, points in setpoint_velocity_series.items()
    }

    progress_cells = []
    for index in range(waypoint_count):
        state = "accepted" if index in accepted_indices else "pending"
        progress_cells.append(
            f'<div class="waypoint {state}"><div class="waypoint-dot">{index}</div><div class="waypoint-label">WP{index}</div><div class="waypoint-state">{"accepted" if state == "accepted" else "not observed"}</div></div>'
        )
    progress_html = '<div class="waypoints">' + "<div class=\"waypoint-line\"></div>" + "".join(progress_cells) + "</div>"

    px4_observed = {"PASS": "valid", "FAIL": "not valid"}.get(gates["px4"], "N/A")
    telemetry_verdict = evaluation["telemetry_verdict"]
    telemetry_gate = "PASS" if telemetry_verdict == "PASS" else "INFO" if telemetry_verdict else "N/A"
    expected_outcome_text = f"expected {expected_outcome}"
    status_rows = [
        ("Mission outcome", outcome or "N/A", expected_outcome_text, gates["mission"]),
        ("Waypoint acceptance", f"{accepted_count}/{waypoint_count}", "explicit acceptance evidence only", gates["waypoint"]),
        ("Tracking cross-track p95", fmt(cross_p95, 2, " m"), f"≤ {fmt(cross_limit, 2, ' m')}", gates["cross_track"]),
        ("Collision safety", f"{fmt(collision_count, 0)} collisions", "0 collisions", gates["collision"]),
        ("LIO navigation state", str(lio.get("state") or "N/A"), "TRACKING + navigation_valid=true", gates["lio"]),
        ("PX4 estimator / local position", px4_observed, "valid", gates["px4"]),
        ("Telemetry verdict", telemetry_verdict, "informational", telemetry_gate),
        ("Planner safety-stop selection", percent(safety_stop_ratio), "diagnostic only", "OBSERVE" if safety_stop_ratio is not None else "N/A"),
    ]
    table_rows = "".join(
        f'<tr><td>{esc(label)}</td><td class="observed">{esc(observed)}</td><td>{esc(criterion)}</td><td>{status_chip(status)}</td></tr>'
        for label, observed, criterion, status in status_rows
    )
    failure_reasons_html = (
        f'<section><h2>Failure reasons</h2><ul>{"".join(f"<li>{esc(reason)}</li>" for reason in failure_reasons)}</ul></section>'
        if failure_reasons else ""
    )

    measured_axis_rows = "".join(
        f'<tr><td class="observed">|{axis.upper()}| measured</td>'
        f'<td>{fmt(stats["mean"], 2, " m/s")}</td>'
        f'<td>{fmt(stats["p50"], 2, " m/s")}</td>'
        f'<td>{fmt(stats["p95"], 2, " m/s")}</td>'
        f'<td>{fmt(stats["p99"], 2, " m/s")}</td>'
        f'<td>{fmt(stats["max"], 2, " m/s")}</td>'
        f'<td>{integer(stats["count"])}</td></tr>'
        for axis, stats in (
            ("vx", axis_stats["vx"]),
            ("vy", axis_stats["vy"]),
            ("vz", axis_stats["vz"]),
        )
    )
    setpoint_axis_rows = "".join(
        f'<tr><td class="observed">|{axis.upper()}| setpoint start</td>'
        f'<td>{fmt(stats["mean"], 2, " m/s")}</td>'
        f'<td>{fmt(stats["p50"], 2, " m/s")}</td>'
        f'<td>{fmt(stats["p95"], 2, " m/s")}</td>'
        f'<td>{fmt(stats["p99"], 2, " m/s")}</td>'
        f'<td>{fmt(stats["max"], 2, " m/s")}</td>'
        f'<td>{integer(stats["count"])}</td></tr>'
        for axis, stats in (
            ("vx", setpoint_axis_stats["vx"]),
            ("vy", setpoint_axis_stats["vy"]),
            ("vz", setpoint_axis_stats["vz"]),
        )
    )
    kinematics_rows = (
        f'{measured_axis_rows}{setpoint_axis_rows}'
        f'<tr><td class="observed">speed measured</td><td>{fmt(speed_series_stats["mean"], 2, " m/s")}</td>'
        f'<td>{fmt(speed_series_stats["p50"], 2, " m/s")}</td><td>{fmt(speed_series_stats["p95"], 2, " m/s")}</td>'
        f'<td>{fmt(speed_series_stats["p99"], 2, " m/s")}</td><td>{fmt(speed_series_stats["max"], 2, " m/s")}</td>'
        f'<td>{integer(speed_series_stats["count"])}</td></tr>'
        f'<tr><td class="observed">setpoint speed start</td><td>{fmt(setpoint_speed_stats["mean"], 2, " m/s")}</td>'
        f'<td>{fmt(setpoint_speed_stats["p50"], 2, " m/s")}</td><td>{fmt(setpoint_speed_stats["p95"], 2, " m/s")}</td>'
        f'<td>{fmt(setpoint_speed_stats["p99"], 2, " m/s")}</td><td>{fmt(setpoint_speed_stats["max"], 2, " m/s")}</td>'
        f'<td>{integer(setpoint_speed_stats["count"])}</td></tr>'
    )

    plot_html = "".join([
        map_svg(data),
        line_chart("Cross-track error", [{"label": "error", "points": error_series, "color": RED}], "distance error (m)", threshold=cross_limit, threshold_label=f"acceptance limit {fmt(cross_limit, 2, ' m')}"),
        line_chart("Velocity components", [{"label": "vx", "points": velocity_series["vx"], "color": BLUE}, {"label": "vy", "points": velocity_series["vy"], "color": TEAL}, {"label": "vz", "points": velocity_series["vz"], "color": ORANGE}], "velocity (m/s)"),
        line_chart("Speed magnitude", [{"label": "measured", "points": speed_series, "color": BLUE}, {"label": "setpoint start", "points": setpoint_speed_series, "color": TEAL, "dash": "6 4"}], "speed (m/s)"),
        line_chart("Available planning horizon", [{"label": "planning horizon", "points": planning_horizon_series, "color": PURPLE}, {"label": "known-free horizon", "points": known_horizon_series, "color": ORANGE, "dash": "6 4"}], "distance (m)"),
        comparison_bars([
            ("longest route leg", finite(mission.get("longest_leg_m")), RED, "route geometry"),
            ("max known-free horizon", finite(known_free.get("maximum")), ORANGE, "best observed free-space horizon"),
        ], "Route demand vs observed free-space horizon"),
    ])
    replay_html = replay_section(data)

    session_name = session.name
    raw_links = [
        ("report schema", "report.json"),
        ("scenario snapshot", "scenario.json"),
        ("sample stream", "samples.jsonl"),
        ("benchmark metrics", "benchmark_metrics.json"),
    ]
    links_html = " ".join(f'<a href="{esc(href)}">{esc(label)}</a>' for label, href in raw_links if (session / href).exists())
    trace_note = (
        f"{integer(trace.get('record_count'))} trace records were captured; "
        f"{integer(trace.get('complete_record_count'))} are complete and {integer(trace.get('partial_record_count'))} are partial. "
        + (f"The following fields are constant zero across all records: {', '.join(zero_trace_fields)}." if zero_trace_fields else "No constant-zero trace fields were found.")
    )
    interpretation = (
        f"Overall acceptance verdict: {evaluation['overall']}. "
        f"The estimator and PX4 validity signals are {gates['lio']} and {gates['px4']}. "
        f"The longest route leg is {fmt(mission.get('longest_leg_m'), 1, ' m')}; the maximum measured known-free horizon is {fmt(known_free.get('maximum'), 1, ' m')}. "
        "Unavailable planner measurements are shown as N/A and are not treated as zero."
    )
    timing_rows = _timing_rows(report)
    timing_html = "".join(
        f'<tr><td>{esc(row["component"])}</td><td>{esc(row["metric"])}</td><td>{esc(row["unit"])}</td>'
        f'<td>{fmt(row["mean"], 1)}</td><td>{fmt(row["p50"], 1)}</td><td>{fmt(row["p95"], 1)}</td>'
        f'<td>{fmt(row["p99"], 1)}</td><td>{fmt(row["max"], 1)}</td>'
        f'<td>{integer(row["count"])}</td></tr>'
        for row in timing_rows
    )
    if not timing_rows:
        timing_html = '<tr><td colspan="9">No processing-time telemetry was recorded.</td></tr>'
    kinematics_html = (
        '<section><h2>Kinematics summary</h2><p class="small">Velocity axis and speed metrics from measured trajectory and published speed setpoints.</p>'
        '<table class="evidence"><thead><tr><th>Metric</th><th>Mean</th><th>P50</th><th>P95</th><th>P99</th><th>Max</th><th>Samples</th></tr></thead>'
        f'<tbody>{kinematics_rows}</tbody></table></section>'
    )
    html_text = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Flight Review · {esc(session_name)}</title>
  <style>
    :root {{ --ink:#203348; --muted:#627386; --line:#dbe3ea; --paper:#ffffff; --canvas:#f2f5f8; --blue:{BLUE}; --red:{RED}; --green:{GREEN}; --orange:{ORANGE}; }}
    * {{ box-sizing:border-box; }}
    body {{ margin:0; background:var(--canvas); color:var(--ink); font-family:Inter,ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif; line-height:1.45; }}
    main {{ max-width:1180px; margin:0 auto; padding:28px 22px 56px; }}
    .hero {{ display:flex; justify-content:space-between; gap:24px; align-items:flex-start; margin-bottom:22px; }}
    h1 {{ margin:0 0 7px; font-size:30px; letter-spacing:-.03em; color:#102b45; }}
    h2 {{ margin:0 0 14px; font-size:20px; letter-spacing:-.015em; color:#102b45; }}
    h3 {{ margin:0 0 8px; font-size:15px; color:#29435d; }}
    p {{ margin:7px 0; color:var(--muted); }}
    .eyebrow {{ margin:0 0 5px; text-transform:uppercase; letter-spacing:.1em; font-size:11px; font-weight:800; color:var(--blue); }}
    .session {{ font-size:12px; color:var(--muted); word-break:break-all; }}
    .hero-right {{ display:flex; align-items:center; gap:12px; flex-shrink:0; }}
    .status {{ display:inline-flex; align-items:center; justify-content:center; min-width:68px; padding:4px 9px; border-radius:999px; font-size:11px; font-weight:800; letter-spacing:.06em; text-transform:uppercase; }}
    .status.pass {{ color:#126747; background:#dcf5e9; }} .status.fail {{ color:#9d2d22; background:#fde4e1; }} .status.na {{ color:#53677b; background:#e9eef2; }} .status.observe {{ color:#855b00; background:#fff0c2; }} .status.info {{ color:#365878; background:#e4edf7; }}
    .hero .status {{ font-size:13px; padding:8px 15px; min-width:86px; }}
    .card, section {{ background:var(--paper); border:1px solid var(--line); border-radius:12px; box-shadow:0 2px 8px rgba(28,54,78,.04); }}
    section {{ padding:20px; margin-top:18px; }}
    .lede {{ font-size:16px; color:#30485e; max-width:880px; }}
    .kpis {{ display:grid; grid-template-columns:repeat(5,minmax(0,1fr)); gap:12px; margin:20px 0; }}
    .kpi {{ padding:15px 16px; min-height:112px; }}
    .kpi-label {{ color:var(--muted); font-size:12px; font-weight:700; }}
    .kpi-value {{ margin-top:8px; font-size:25px; font-weight:800; letter-spacing:-.03em; color:#102b45; }}
    .kpi-sub {{ margin-top:3px; color:var(--muted); font-size:11px; }}
    .kpi.fail .kpi-value {{ color:var(--red); }} .kpi.pass .kpi-value {{ color:var(--green); }}
    .findings {{ display:grid; grid-template-columns:1fr 1fr; gap:9px 22px; }}
    .finding {{ display:flex; gap:10px; align-items:flex-start; padding:8px 0; border-bottom:1px solid #edf1f4; font-size:14px; }}
    .finding:last-child {{ border-bottom:0; }}
    .finding .status {{ flex-shrink:0; margin-top:1px; }}
    .waypoints {{ display:grid; grid-template-columns:repeat({max(1, waypoint_count)},1fr); gap:10px; position:relative; padding:6px 0 2px; }}
    .waypoint-line {{ position:absolute; top:22px; left:6%; right:6%; height:2px; background:#d9e1e8; }}
    .waypoint {{ position:relative; text-align:center; z-index:1; }}
    .waypoint-dot {{ width:32px; height:32px; margin:0 auto 6px; display:grid; place-items:center; border-radius:50%; border:2px solid #b9c7d3; background:#fff; color:var(--muted); font-size:12px; font-weight:800; }}
    .waypoint.accepted .waypoint-dot {{ border-color:var(--green); background:#dcf5e9; color:#126747; }}
    .waypoint-label {{ font-weight:800; font-size:12px; }} .waypoint-state {{ font-size:11px; color:var(--muted); }} .waypoint.accepted .waypoint-state {{ color:var(--green); font-weight:700; }}
    .charts {{ display:grid; grid-template-columns:1fr; gap:14px; }}
    .chart-card {{ border:1px solid #e3e9ee; border-radius:9px; background:#fff; padding:10px 10px 2px; overflow:hidden; }}
    .chart-card:first-child {{ grid-column:auto; }}
    .chart-title {{ margin:2px 5px 2px; font-weight:800; font-size:14px; color:#29435d; }}
    .chart {{ width:100%; height:auto; display:block; }}
    .axis-label {{ fill:#617284; font-size:11px; }} .axis-title {{ fill:#29435d; font-size:12px; font-weight:700; }} .legend-label {{ fill:#53677b; font-size:11px; }} .threshold-label {{ fill:{RED}; font-size:11px; font-weight:700; }} .end-label {{ font-size:11px; font-weight:800; }} .map-label {{ fill:#5a3f45; font-size:11px; font-weight:700; }} .bar-value {{ fill:#29435d; font-size:12px; font-weight:800; }} .bar-note {{ fill:#68778a; font-size:10px; }}
    .replay-toolbar {{ display:flex; align-items:center; gap:8px; flex-wrap:wrap; margin:12px 0 8px; }} .replay-toolbar button {{ border:1px solid #b9c9d6; border-radius:6px; background:#fff; color:#1f4f78; padding:7px 13px; font-weight:800; cursor:pointer; }} .replay-toolbar button:hover {{ background:#edf5fb; }} .replay-time-control {{ display:flex; align-items:center; gap:10px; flex:1; min-width:280px; color:#53677b; font-size:12px; font-weight:700; }} .replay-time-control input {{ flex:1; accent-color:{BLUE}; }} .replay-time-control output {{ min-width:62px; color:#18324b; font-variant-numeric:tabular-nums; }}
    .replay-event-caption {{ display:flex; justify-content:space-between; gap:12px; color:#68778a; font-size:11px; margin-top:8px; }} .replay-plan-events {{ height:18px; position:relative; margin:2px 3px 12px; border-bottom:1px solid #cdd8e1; background:linear-gradient(to bottom,transparent 0%,transparent 65%,#edf2f6 65%,#edf2f6 100%); }} .replay-marker {{ position:absolute; bottom:0; width:3px; height:11px; padding:0; border:0; cursor:pointer; transform:translateX(-1px); }} .replay-marker.nominal {{ background:{TEAL}; }} .replay-marker.safety {{ background:{RED}; }} .replay-marker.active {{ height:18px; width:5px; box-shadow:0 0 0 2px #f0a51a; z-index:2; }}
    .replay-grid {{ display:grid; grid-template-columns:minmax(0,1fr) 250px; gap:14px; align-items:start; }} .replay-map-card {{ min-width:0; border:1px solid #e3e9ee; border-radius:9px; background:#fff; padding:10px 10px 8px; overflow:hidden; }} .replay-map {{ width:100%; height:auto; display:block; }} .replay-status {{ border:1px solid #e3e9ee; border-radius:9px; background:#f8fafc; padding:14px; }} .replay-status h3 {{ margin-bottom:10px; }} .replay-status dl {{ margin:0; }} .replay-status dt {{ margin-top:9px; color:#68778a; font-size:11px; font-weight:800; text-transform:uppercase; letter-spacing:.05em; }} .replay-status dd {{ margin:2px 0 0; color:#1f3c56; font-size:13px; font-weight:700; overflow-wrap:anywhere; }} .replay-legend {{ display:flex; flex-wrap:wrap; gap:8px 18px; color:#53677b; font-size:11px; margin:5px 4px 0; }} .replay-swatch {{ display:inline-block; width:18px; height:3px; margin:0 5px 2px 0; vertical-align:middle; background:#1f6feb; }} .replay-swatch.nominal {{ background:{TEAL}; }} .replay-swatch.safety {{ background:{RED}; }} .replay-swatch.ground {{ background:{BLUE}; height:4px; }} .replay-svg-label {{ fill:#617284; font-size:11px; }} .replay-svg-title {{ fill:#29435d; font-size:12px; font-weight:700; }} .replay-map-label {{ fill:#5a3f45; font-size:11px; font-weight:700; }}
    .evidence {{ width:100%; border-collapse:collapse; font-size:13px; }} .evidence th,.evidence td {{ padding:10px 9px; border-bottom:1px solid #e5ebef; text-align:left; vertical-align:middle; }} .evidence th {{ color:var(--muted); font-size:11px; text-transform:uppercase; letter-spacing:.06em; }} .evidence .observed {{ font-weight:800; color:#1f3c56; }}
    .two-col {{ display:grid; grid-template-columns:1.1fr .9fr; gap:18px; align-items:start; }}
    .callout {{ padding:13px 15px; background:#f7fafc; border-left:4px solid var(--blue); border-radius:6px; }} .callout p {{ color:#385169; font-size:13px; }}
    details {{ margin-top:12px; border-top:1px solid var(--line); padding-top:12px; }} summary {{ cursor:pointer; font-weight:800; color:#2d5e8c; }}
    .raw-links {{ display:flex; flex-wrap:wrap; gap:8px 16px; margin-top:8px; }} a {{ color:#1b63a1; }} code {{ background:#eef3f7; padding:2px 5px; border-radius:4px; font-size:11px; }}
    .small {{ font-size:12px; color:var(--muted); }} footer {{ margin-top:18px; font-size:11px; color:#7b8996; }}
    @media (max-width:900px) {{ .kpis {{ grid-template-columns:repeat(3,minmax(0,1fr)); }} .two-col {{ grid-template-columns:1fr; }} .replay-grid {{ grid-template-columns:1fr; }} }}
    @media (max-width:600px) {{ main {{ padding:18px 12px 40px; }} .hero {{ display:block; }} .hero-right {{ margin-top:12px; }} .kpis {{ grid-template-columns:1fr 1fr; }} .findings {{ grid-template-columns:1fr; }} section {{ padding:14px; }} .evidence {{ font-size:12px; }} .evidence th:nth-child(3),.evidence td:nth-child(3) {{ display:none; }} }}
  </style>
</head>
<body>
<main>
  <header class="hero">
    <div><div class="eyebrow">PX4 / SITL flight review</div><h1>External Mode · {esc(metrics.get('mission', {}).get('waypoint_count', 0))}-waypoint mission</h1><p class="session">Session: {esc(session_name)} · {fmt(mission.get('duration_sim_s'), 1, ' s')} simulated · generated from recorded artifacts</p></div>
    <div class="hero-right">{status_chip(evaluation["overall"], evaluation["overall"])}</div>
  </header>

  <p class="lede">Acceptance is computed from explicit mission completion, waypoint acceptance, tracking, collision, LIO and PX4 evidence. Telemetry verdict: {esc(evaluation["telemetry_verdict"])}.</p>

  <div class="kpis">
    <div class="card kpi {status_class(gates['mission'])}"><div class="kpi-label">Mission outcome</div><div class="kpi-value">{esc(outcome or 'N/A')}</div><div class="kpi-sub">{status_chip(gates['mission'])} expected {esc(acceptance.get('expected_outcome') or 'complete')}</div></div>
    <div class="card kpi {status_class(gates['waypoint'])}"><div class="kpi-label">Waypoint progress</div><div class="kpi-value">{accepted_count}/{waypoint_count}</div><div class="kpi-sub">{status_chip(gates['waypoint'])} explicit acceptance events</div></div>
    <div class="card kpi {status_class(gates['cross_track'])}"><div class="kpi-label">Cross-track p95</div><div class="kpi-value">{fmt(cross_p95, 2, ' m')}</div><div class="kpi-sub">{status_chip(gates['cross_track'])} limit {fmt(cross_limit, 2, ' m')}</div></div>
    <div class="card kpi {status_class(gates['collision'])}"><div class="kpi-label">Collision safety</div><div class="kpi-value">{fmt(collision_count, 0)}</div><div class="kpi-sub">{status_chip(gates['collision'])} collisions · min clearance {fmt(min_clearance, 2, ' m')}</div></div>
    <div class="card kpi {status_class(gates['lio'])}"><div class="kpi-label">Localization</div><div class="kpi-value">{esc(lio.get('state') or 'N/A')}</div><div class="kpi-sub">{status_chip(gates['lio'])} navigation_valid = {esc(lio.get('navigation_valid'))}</div></div>
  </div>

  <section><h2>What the run says</h2><div class="findings">{"".join(f'<div class="finding">{status_chip(status)}<span>{esc(text)}</span></div>' for status, text in findings)}</div></section>

  <section><h2>Mission progress</h2><p class="small">Only explicit waypoint acceptance is counted. Goal publication is intentionally not shown as success.</p>{progress_html}</section>

  {replay_html}

  <section><h2>Acceptance gates</h2><table class="evidence"><thead><tr><th>Gate</th><th>Observed</th><th>Criterion / context</th><th>Status</th></tr></thead><tbody>{table_rows}</tbody></table></section>

  {failure_reasons_html}
  {kinematics_html}

  <section><h2>Measured processing time</h2><p class="small">Only runtime-supplied processing measurements are shown. Unit is reported per metric; missing telemetry is N/A, never treated as zero.</p><table class="evidence"><thead><tr><th>Component</th><th>Metric</th><th>Unit</th><th>Mean</th><th>P50</th><th>P95</th><th>P99</th><th>Max</th><th>Samples</th></tr></thead><tbody>{timing_html}</tbody></table></section>

  <section><h2>Flight overview</h2><p class="small">Plots are sampled for readability. Each plot has its own scale, units, labelled axes and legend; p95/limits remain visible in the cards above and in the gate table.</p><div class="charts">{plot_html}</div></section>

  <section><div class="two-col"><div><h2>Interpretation</h2><div class="callout"><p>{esc(interpretation)}</p></div></div><div><h2>Run context</h2><p class="small"><strong>Estimator:</strong> {esc(lio.get('state') or 'N/A')} · residual p95 {fmt(metrics.get('localization', {}).get('p95_position_residual_m'), 3, ' m')}.</p><p class="small"><strong>PX4:</strong> {esc(px4_observed)}; failsafe observed = {esc(external.get('failsafe_seen'))}.</p><p class="small"><strong>Planner:</strong> {integer(planning.get('diagnostic_sample_count'))} diagnostic samples · {integer(continuity.get('endpoint_change_count'))} endpoint changes · {integer(smoothness.get('handover_expired_count'))} expired handovers.</p></div></div></section>

  <section><h2>Evidence and raw artifacts</h2><p class="small">{esc(trace_note)}</p><details><summary>Source files</summary><div class="raw-links">{links_html}</div></details><details><summary>Provenance</summary><p class="small">Navigation commit: <code>{esc(navigation_commit)}</code> · dirty workspace: <code>{esc(navigation_dirty)}</code>.</p></details></section>

  <footer>Report purpose: human evaluation of one SITL run. Use the linked raw artifacts for debugging; do not use this page as a substitute for the full recorder output.</footer>
</main>
</body>
</html>
"""
    output.write_text(html_text, encoding="utf-8")
    return output
