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
    state_intervals: list[dict[str, Any]] | None = None,
) -> str:
    chart_id = "chart-" + "".join(
        character.lower() if character.isalnum() else "-"
        for character in title
    ).strip("-")
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
    left, right, bottom = 70, 24, 56
    width = 980
    # Keep the legend inside the SVG. Three fixed columns leave enough room
    # for the long frame-qualified labels (for example NED→ENU) and make a
    # fifth trace wrap to a second row instead of disappearing off-screen.
    legend_columns = 3
    legend_row_height = 22
    legend_rows = max(1, math.ceil(len(plotted) / legend_columns))
    top = 46 + (legend_rows - 1) * legend_row_height
    chart_height = height + (legend_rows - 1) * legend_row_height
    plot_w = width - left - right
    plot_h = chart_height - top - bottom
    y_ticks = nice_ticks(axis_min, axis_max, 5)
    axis_min = min(axis_min, y_ticks[0])
    axis_max = max(axis_max, y_ticks[-1])

    def point_xy(point: tuple[float, float]) -> tuple[float, float]:
        x = left + (point[0] - min_t) / (max_t - min_t) * plot_w
        y = top + (1.0 - (point[1] - axis_min) / (axis_max - axis_min)) * plot_h
        return x, y

    parts = [
        f'<div class="chart-card"><div class="chart-title">{esc(title)}</div>',
        f'<svg class="chart" viewBox="0 0 {width} {chart_height}" role="img" aria-label="{esc(title)}">',
        f'<title>{esc(title)}</title><rect x="0" y="0" width="{width}" height="{chart_height}" fill="#ffffff"/>',
    ]
    for interval in state_intervals or []:
        start = finite(interval.get("t_start"))
        end = finite(interval.get("t_end"))
        if start is None or end is None or end <= start:
            continue
        start = max(min_t, start)
        end = min(max_t, end)
        if end <= start:
            continue
        state = str(interval.get("state") or "unknown")
        color = {"normal": "#dff3e8", "safety": "#fde3e0", "unknown": "#eef2f5"}.get(state, "#eef2f5")
        x1 = left + (start - min_t) / (max_t - min_t) * plot_w
        x2 = left + (end - min_t) / (max_t - min_t) * plot_w
        parts.append(
            f'<rect x="{x1:.1f}" y="{top}" width="{max(0.5, x2-x1):.1f}" height="{plot_h}" fill="{color}" fill-opacity="0.72" data-state-band="{esc(state)}"><title>{esc(state)} path observed from {start:.2f}s to {end:.2f}s</title></rect>'
        )
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
            f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{chart_height-bottom}" stroke="{GRID}"/>'
            f'<text x="{x:.1f}" y="{chart_height-bottom+20}" text-anchor="middle" class="axis-label">{esc(tick_label(tick))}</text>'
        )
    parts.extend([
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{chart_height-bottom}" stroke="{NAVY}" stroke-width="1.2"/>',
        f'<line x1="{left}" y1="{chart_height-bottom}" x2="{width-right}" y2="{chart_height-bottom}" stroke="{NAVY}" stroke-width="1.2"/>',
        f'<text x="18" y="{top + plot_h/2:.1f}" text-anchor="middle" transform="rotate(-90 18 {top + plot_h/2:.1f})" class="axis-title">{esc(y_label)}</text>',
        f'<text x="{left + plot_w/2:.1f}" y="{chart_height-10}" text-anchor="middle" class="axis-title">{esc(x_label)}</text>',
    ])
    if threshold is not None:
        y = top + (1.0 - (threshold - axis_min) / (axis_max - axis_min)) * plot_h
        label = threshold_label or f"limit {fmt(threshold)}"
        parts.append(
            f'<line x1="{left}" y1="{y:.1f}" x2="{width-right}" y2="{y:.1f}" stroke="{RED}" stroke-width="1.5" stroke-dasharray="7 5"/>'
            f'<text x="{width-right-4}" y="{y-6:.1f}" text-anchor="end" class="threshold-label">{esc(label)}</text>'
        )
    for series_index, (label, points, color, dash) in enumerate(plotted):
        coordinates = " ".join(f"{point_xy(point)[0]:.1f},{point_xy(point)[1]:.1f}" for point in points)
        dash_attr = f' stroke-dasharray="{esc(dash)}"' if dash else ""
        series_id = f"{chart_id}-series-{series_index}"
        parts.append(
            f'<g id="{esc(series_id)}" class="chart-series"><polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="2.5" stroke-linejoin="round" stroke-linecap="round"{dash_attr}/>'
        )
        end_x, end_y = point_xy(points[-1])
        parts.append(
            f'<circle cx="{end_x:.1f}" cy="{end_y:.1f}" r="3.5" fill="{color}"/>'
            f'<text x="{min(width-right-4, end_x+8):.1f}" y="{max(top+14, end_y-8):.1f}" class="end-label" fill="{color}">{esc(fmt(points[-1][1]))}</text></g>'
        )
    legend_y = 22
    for index, (label, _, color, _) in enumerate(plotted):
        series_id = f"{chart_id}-series-{index}"
        legend_x = left + (index % legend_columns) * (plot_w / legend_columns)
        legend_row = index // legend_columns
        legend_y = 22 + legend_row * legend_row_height
        parts.append(
            f'<g class="chart-legend-toggle" data-target="{esc(series_id)}" tabindex="0" role="button" aria-pressed="true">'
            f'<line x1="{legend_x:.1f}" y1="{legend_y}" x2="{legend_x+20:.1f}" y2="{legend_y}" stroke="{color}" stroke-width="3"/>'
            f'<text x="{legend_x+27:.1f}" y="{legend_y+4}" class="legend-label">{esc(label)}</text></g>'
        )
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
    left, top, right, bottom = 68, 62, 22, 54
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
        f'<text x="{left + plot_w/2:.1f}" y="{height-10}" text-anchor="middle" class="axis-title">display ENU X = east (m)</text>',
        f'<text x="18" y="{top + plot_h/2:.1f}" text-anchor="middle" transform="rotate(-90 18 {top + plot_h/2:.1f})" class="axis-title">display ENU Y = north (m)</text>',
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
    actual_sampled = samples([(item["position"][0], item["position"][1]) for item in data["ground_truth"]], 500)
    actual_coords = " ".join(f"{transform((x, y, 0.0))[0]:.1f},{transform((x, y, 0.0))[1]:.1f}" for x, y in actual_sampled)
    if actual_coords:
        parts.append(f'<polyline points="{actual_coords}" fill="none" stroke="{BLUE}" stroke-width="3" stroke-linejoin="round" stroke-linecap="round"/>')
    heading, heading_source = _map_heading(data)
    if heading is not None and data["ground_truth"]:
        latest_xy = transform(tuple(data["ground_truth"][-1]["position"]))
        parts.append(
            f'<polygon points="{_arrow_polygon(latest_xy, heading)}" fill="{BLUE}" stroke="#ffffff" stroke-width="2">'
            f'<title>UAV heading: {math.degrees(heading):.1f}° ({esc(heading_source)})</title></polygon>'
        )
    trajectory_paths = data.get("observability", {}).get("trajectory_paths", [])
    if trajectory_paths:
        for path in trajectory_paths:
            points = [
                tuple(float(value) for value in point[:3])
                for point in path.get("points", [])
                if isinstance(point, (list, tuple)) and len(point) >= 3
                and all(finite(value) is not None for value in point[:3])
            ]
            if len(points) < 2:
                continue
            coordinates = " ".join(f"{transform(point)[0]:.1f},{transform(point)[1]:.1f}" for point in points)
            flag = int(finite(path.get("trajectory_flag")) or 0)
            color = RED if flag == 2 else TEAL
            dash = "5 4" if flag == 2 else ""
            dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
            label = "backup/safety" if flag == 2 else "main/nominal"
            parts.append(
                f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-opacity="0.68" stroke-width="2"{dash_attr}/>'
            )
    else:
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
        )
    obstacle_tags = "".join(
        f'<span class="map-tag {"route" if obstacle["name"] in route_obstacles else "obstacle"}">{esc(obstacle["name"])}</span>'
        for obstacle in obstacles
    ) or '<span class="map-tag muted">none</span>'
    waypoint_tags = "".join(f'<span class="map-tag waypoint">WP{index}</span>' for index in range(len(waypoints))) or '<span class="map-tag muted">none</span>'
    parts.extend([
        f'<line x1="{left+12}" y1="22" x2="{left+32}" y2="22" stroke="{BLUE}" stroke-width="3"/><text x="{left+39}" y="26" class="legend-label">ground truth</text>',
        f'<line x1="{left+150}" y1="22" x2="{left+170}" y2="22" stroke="{TEAL}" stroke-width="2"/><text x="{left+177}" y="26" class="legend-label">main / nominal</text>',
        f'<line x1="{left+310}" y1="22" x2="{left+330}" y2="22" stroke="{RED}" stroke-width="2" stroke-dasharray="5 4"/><text x="{left+337}" y="26" class="legend-label">backup / safety</text>',
        f'<rect x="{left+490}" y="16" width="14" height="12" fill="#e8897e" stroke="{RED}"/><text x="{left+511}" y="26" class="legend-label">route obstacles</text>',
        f'<circle cx="{left+666}" cy="22" r="5" fill="#f0a51a" stroke="#7b4e00"/><text x="{left+679}" y="26" class="legend-label">waypoints</text>',
        f'<polygon points="{left+12},38 {left+32},44 {left+12},50" fill="{BLUE}" stroke="#ffffff"/><text x="{left+39}" y="48" class="legend-label">UAV heading ({esc(heading_source)})</text>',
        '</svg>',
        f'<div class="map-annotations"><div><strong>Waypoints</strong> {waypoint_tags}</div><div><strong>Obstacles</strong> {obstacle_tags}</div><div class="small">Path role is encoded by line style/color; labels are listed here to keep the map readable.</div></div></div>',
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


def _safety_stop_status(observed: bool) -> str:
    """Report a safety action as evidence, never as mission acceptance."""
    return "OBSERVE" if observed else "N/A"


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
            "heading_rad": finite(item.get("heading_rad")),
        })
    ground_truth = _replay_sampled(ground_truth, 720)

    plans: list[dict[str, Any]] = []
    trajectory_paths = data.get("observability", {}).get("trajectory_paths", [])
    if trajectory_paths:
        for item in trajectory_paths:
            positions = [
                point for point in (_replay_point(value) for value in item.get("points", []))
                if point is not None
            ]
            if not positions:
                continue
            positions = [
                positions[round(index * (len(positions) - 1) / max(1, min(96, len(positions)) - 1))]
                for index in range(min(96, len(positions)))
            ]
            role = 1 if int(finite(item.get("trajectory_flag")) or 0) == 2 else 0
            start_time = finite(item.get("t_start"))
            end_time = finite(item.get("t_end"))
            if start_time is None:
                continue
            plans.append({
                "t": start_time,
                "duration": max(0.01, (end_time - start_time) if end_time is not None else 0.01),
                "role": role,
                "role_label": "safety / backup" if role == 1 else "nominal / main",
                "safety_kind": "recorded trajectory flag",
                "waypoint": int(finite(item.get("waypoint_index")) or 0),
                "world_revision": 0,
                "points": positions,
            })
    else:
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
                "role_label": "safety / backup" if role == 1 else "nominal / main",
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
    timestamps.extend(item["t"] for item in data.get("observability", {}).get("waypoint_events", []) if finite(item.get("t")) is not None)
    start = min(timestamps) if timestamps else 0.0
    end = max(timestamps) if timestamps else 1.0
    return {
        "ground_truth": ground_truth,
        "plans": plans,
        "planner": planner,
        "waypoints": [list(point) for point in data.get("waypoints", [])],
        "obstacles": obstacles,
        "route_obstacles": list(data.get("metrics", {}).get("route_obstacles", [])),
        "waypoint_events": [item for item in data.get("observability", {}).get("waypoint_events", []) if finite(item.get("t")) is not None],
        "vehicle_events": [item for item in data.get("observability", {}).get("vehicle_events", []) if finite(item.get("t")) is not None],
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
  function drawHeadingArrow(parent, point, heading) {
    if (!Number.isFinite(Number(heading))) return;
    const angle = Number(heading), dx = Math.cos(angle), dy = -Math.sin(angle);
    const px = -dy, py = dx, length = 24, width = 7;
    const tip = [point[0] + dx * length, point[1] + dy * length];
    const base = [point[0] - dx * length * .42, point[1] - dy * length * .42];
    const left = [base[0] + px * width, base[1] + py * width];
    const right = [base[0] - px * width, base[1] - py * width];
    const arrow = svgNode("polygon", {
      points: [tip, left, right].map(item => item.map(value => value.toFixed(1)).join(",")).join(" "),
      fill: "#1f6feb", stroke: "#ffffff", "stroke-width": 2
    }, parent);
    const title = svgNode("title", {}, arrow);
    title.textContent = `UAV heading ${((angle * 180 / Math.PI) + 360) % 360}° ENU yaw`;
    svgNode("circle", {cx: point[0], cy: point[1], r: 4, fill: "#1f6feb", stroke: "#ffffff", "stroke-width": 2}, parent);
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
    svgText(staticLayer, left + plotW / 2, H - 10, "display ENU X = east (m)", "replay-svg-title", "middle");
    const yTitle = svgText(staticLayer, 16, top + plotH / 2, "display ENU Y = north (m)", "replay-svg-title", "middle");
    yTitle.setAttribute("transform", `rotate(-90 16 ${top + plotH / 2})`);
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
    });
    data.waypoints.forEach((point, index) => {
      const xy = world(point);
      svgNode("circle", {cx: xy[0], cy: xy[1], r: 5, fill: "#f0a51a", stroke: "#7b4e00"}, staticLayer);
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
    const waypoint = latestAt(data.waypoint_events, time);
    const vehicle = latestAt(data.vehicle_events, time);
    document.getElementById("replay-waypoint").textContent = waypoint ? `WP${waypoint.waypoint_index} · ${waypoint.state_name || "—"} · accepted ${waypoint.waypoint_accepted ? "yes" : "no"}` : "—";
    document.getElementById("replay-acceptance").textContent = waypoint ? `WP${waypoint.accepted_waypoint_index} · ${fmt(waypoint.acceptance_position_error_m, 2)} m · ${fmt(waypoint.acceptance_speed_mps, 2)} m/s` : "—";
    document.getElementById("replay-vehicle-state").textContent = vehicle ? `nav=${vehicle.nav_state ?? "—"} · armed=${vehicle.arming_state ?? "—"} · failsafe=${vehicle.failsafe ?? "—"}` : "—";
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
      const previous = nearestGround(currentTime - .15);
      let heading = Number(ground.heading_rad);
      if (!Number.isFinite(heading) && previous && previous !== ground) {
        const from = world(previous.p), dx = point[0] - from[0], dy = point[1] - from[1];
        if (Math.hypot(dx, dy) > 1) heading = Math.atan2(-dy, dx);
      }
      drawHeadingArrow(vehicleLayer, point, heading);
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
      <div class="replay-map-card"><svg id="replay-map" class="replay-map" role="img" aria-label="Interactive 2D flight replay"><title>Interactive 2D flight replay</title></svg><div class="replay-legend"><span><i class="replay-swatch ground"></i> observed UAV path</span><span class="replay-heading-legend">➤</span><span>UAV heading (ENU yaw)</span><span><i class="replay-swatch nominal"></i> active nominal path</span><span><i class="replay-swatch safety"></i> active safety path</span></div></div>
      <aside class="replay-status"><h3>State at cursor</h3><dl><dt>Time</dt><dd id="replay-time-value-side">—</dd><dt>UAV position</dt><dd id="replay-position">—</dd><dt>UAV speed</dt><dd id="replay-speed">—</dd><dt>Vehicle state</dt><dd id="replay-vehicle-state">—</dd><dt>Waypoint state</dt><dd id="replay-waypoint">—</dd><dt>Acceptance</dt><dd id="replay-acceptance">—</dd><dt>Active path</dt><dd id="replay-plan">—</dd><dt>Path metadata</dt><dd id="replay-plan-meta">—</dd><dt>Planner</dt><dd id="replay-planner">—</dd></dl><p class="small">The replay shows the recorded command path and observed vehicle state at the cursor; it is not a physics re-simulation.</p></aside>
    </div>
    <script>
      {script}
    </script>
    </section>
"""


def _obs_points(observability: dict[str, Any], stream: str, vector: str, axis: str) -> list[tuple[float, float]]:
    rows = observability.get("streams", {}).get(stream, {}).get(f"{vector}_series", [])
    return [
        (float(item["t"]), float(item[axis]))
        for item in rows
        if isinstance(item, dict) and finite(item.get("t")) is not None and axis in item and finite(item.get(axis)) is not None
    ]


def _pva_points(observability: dict[str, Any], field: str, axis: int) -> list[tuple[float, float]]:
    result = []
    for item in observability.get("pva", []):
        value = item.get(field)
        if not isinstance(value, (list, tuple)) or len(value) <= axis:
            continue
        timestamp = finite(item.get("t"))
        component = finite(value[axis])
        if timestamp is not None and component is not None:
            result.append((timestamp, component))
    return result


def _px4_setpoint_points(observability: dict[str, Any], field: str, axis: int) -> list[tuple[float, float]]:
    result = []
    for item in observability.get("setpoints", []):
        value = item.get(field)
        timestamp = finite(item.get("t"))
        if timestamp is None or not isinstance(value, (list, tuple)) or len(value) < 3:
            continue
        converted = _point(value)
        if converted is None:
            continue
        # PX4 setpoint events are recorded in NED; the report uses ENU.
        enu = (converted[1], converted[0], -converted[2])
        if finite(enu[axis]) is not None:
            result.append((timestamp, enu[axis]))
    return result


def _map_heading(data: dict[str, Any]) -> tuple[float | None, str]:
    """Use recorded ENU yaw; fall back only to the measured track direction."""
    ground = data.get("ground_truth", [])
    if not ground:
        return None, "unavailable"
    latest = ground[-1]
    heading = finite(latest.get("heading_rad"))
    if heading is not None:
        return heading, "recorded ENU yaw"
    if len(ground) >= 2:
        previous = ground[-2]
        latest_position = latest.get("position", [])
        previous_position = previous.get("position", [])
        if len(latest_position) >= 2 and len(previous_position) >= 2:
            dx = finite(latest_position[0])
            dy = finite(latest_position[1])
            px = finite(previous_position[0])
            py = finite(previous_position[1])
            if None not in (dx, dy, px, py) and math.hypot(dx - px, dy - py) > 1e-6:
                return math.atan2(dy - py, dx - px), "measured track direction"
    return None, "unavailable"


def _arrow_polygon(center: tuple[float, float], heading: float, length: float = 24.0) -> str:
    """Create a QGC-like screen-space arrow for an ENU world yaw."""
    dx = math.cos(heading)
    dy = -math.sin(heading)  # SVG y grows downward; ENU y grows upward.
    px, py = -dy, dx
    tip = (center[0] + dx * length, center[1] + dy * length)
    base = (center[0] - dx * length * 0.42, center[1] - dy * length * 0.42)
    left = (base[0] + px * length * 0.28, base[1] + py * length * 0.28)
    right = (base[0] - px * length * 0.28, base[1] - py * length * 0.28)
    return " ".join(f"{x:.1f},{y:.1f}" for x, y in (tip, left, right))


def _stream_rows(observability: dict[str, Any]) -> str:
    labels = {
        "ground_truth_odometry": "Ground truth odometry",
        "propagated_odometry": "LIO propagated odometry",
        "corrected_odometry": "LIO corrected odometry",
        "external_odometry": "PX4 external (NED→ENU)",
        "px4_odometry": "PX4 odometry (NED→ENU)",
        "local_position": "PX4 local position (NED→ENU)",
    }
    rows = []
    for name, item in observability.get("streams", {}).items():
        label = labels.get(name, name)
        rows.append(
            f'<tr><td>{esc(label)}</td><td>{integer(item.get("count"))}</td>'
            f'<td>{fmt(item.get("mean_rate_hz"), 1, " Hz")}</td>'
            f'<td>{fmt(item.get("duration_s"), 2, " s")}</td>'
            f'<td>{fmt(item.get("p95_gap_ms"), 2, " ms")}</td>'
            f'<td>{fmt(item.get("max_gap_ms"), 2, " ms")}</td></tr>'
        )
    return "".join(rows) or '<tr><td colspan="6">No position/velocity telemetry recorded.</td></tr>'


def _coordinate_contract_rows(observability: dict[str, Any]) -> str:
    contract = observability.get("coordinate_contract", {})
    streams = contract.get("streams", {}) if isinstance(contract, dict) else {}
    labels = {
        "ground_truth_odometry": "Ground truth odometry",
        "propagated_odometry": "LIO propagated odometry",
        "corrected_odometry": "LIO corrected odometry",
        "external_odometry": "PX4 external odometry",
        "px4_odometry": "PX4 odometry",
        "local_position": "PX4 local position",
        "pva_command": "PVA command",
        "setpoint": "PX4 setpoint event",
    }
    rows = []
    for name in labels:
        item = streams.get(name, {})
        if not isinstance(item, dict):
            continue
        rows.append(
            f'<tr><td>{esc(labels[name])}</td><td>{esc(item.get("source_frame", "—"))}</td>'
            f'<td>{esc(item.get("transform", "—"))}</td><td>{esc(contract.get("display_frame", "ENU (x=east, y=north, z=up)"))}</td></tr>'
        )
    return "".join(rows) or '<tr><td colspan="4">Coordinate-frame metadata unavailable.</td></tr>'


def _axis_rows(observability: dict[str, Any]) -> str:
    labels = {
        "ground_truth_odometry": "Ground truth",
        "propagated_odometry": "LIO propagated",
        "corrected_odometry": "LIO corrected",
        "external_odometry": "PX4 external (NED→ENU)",
        "px4_odometry": "PX4 odometry (NED→ENU)",
        "local_position": "PX4 local position (NED→ENU)",
    }
    rows = []
    for name, item in observability.get("streams", {}).items():
        for axis in ("x", "y", "z"):
            stats = item.get("position_stats", {}).get(axis, {})
            rows.append(
                f'<tr><td>{esc(labels.get(name, name))}</td><td>position {axis}</td>'
                f'<td>{fmt(stats.get("mean"), 3, " m")}</td><td>{fmt(stats.get("p50"), 3, " m")}</td>'
                f'<td>{fmt(stats.get("p95"), 3, " m")}</td><td>{fmt(stats.get("maximum"), 3, " m")}</td>'
                f'<td>{integer(stats.get("count"))}</td></tr>'
            )
        for axis in ("vx", "vy", "vz"):
            stats = item.get("velocity_stats", {}).get(axis, {})
            rows.append(
                f'<tr><td>{esc(labels.get(name, name))}</td><td>{axis}</td>'
                f'<td>{fmt(stats.get("mean"), 3, " m/s")}</td><td>{fmt(stats.get("p50"), 3, " m/s")}</td>'
                f'<td>{fmt(stats.get("p95"), 3, " m/s")}</td><td>{fmt(stats.get("maximum"), 3, " m/s")}</td>'
                f'<td>{integer(stats.get("count"))}</td></tr>'
            )
    return "".join(rows) or '<tr><td colspan="7">No position or velocity axes recorded.</td></tr>'


def _diagnostic_health_rows(observability: dict[str, Any]) -> str:
    rows = []
    for item in observability.get("health", []):
        value = item.get("value")
        if isinstance(value, bool):
            observed = "true" if value else "false"
        elif isinstance(value, (int, float)):
            observed = fmt(value, 3) if not float(value).is_integer() else integer(value)
        else:
            observed = str(value) if value not in (None, "") else "—"
        rows.append(
            f'<tr><td>{esc(item.get("component"))}</td><td>{esc(item.get("source"))}</td>'
            f'<td>{esc(item.get("metric"))}</td><td class="observed">{esc(observed)}</td>'
            f'<td>{integer(item.get("count"))}</td></tr>'
        )
    return "".join(rows) or '<tr><td colspan="5">No LIO/planner diagnostic health fields recorded.</td></tr>'


def _diagnostic_timing_rows(observability: dict[str, Any]) -> str:
    rows = []
    for item in sorted(observability.get("timing", []), key=lambda row: (str(row.get("component")), str(row.get("metric")))):
        stats = item.get("stats", {})
        rows.append(
            f'<tr><td>{esc(item.get("component"))}</td><td>{esc(item.get("source"))}</td>'
            f'<td>{esc(item.get("metric"))}</td><td>{esc(item.get("unit"))}</td>'
            f'<td>{fmt(stats.get("mean"), 1)}</td><td>{fmt(stats.get("p50"), 1)}</td>'
            f'<td>{fmt(stats.get("p95"), 1)}</td><td>{fmt(stats.get("p99"), 1)}</td>'
            f'<td>{fmt(stats.get("maximum"), 1)}</td><td>{integer(item.get("count"))}</td>'
            f'<td>{integer(item.get("nonzero_count"))}</td></tr>'
        )
    return "".join(rows) or '<tr><td colspan="11">No LIO/planner processing telemetry recorded.</td></tr>'


_TIMING_PHASE_META: dict[str, dict[str, str]] = {
    "ros_pointcloud_decode_us": {"label": "ROS point-cloud decode", "group": "Input", "kind": "compute"},
    "observation_pair_wait_us": {"label": "Observation pair wait", "group": "Wait", "kind": "wait"},
    "mapping_callback_total_us": {"label": "Mapping callback total", "group": "Mapping", "kind": "total"},
    "world_snapshot_export_us": {"label": "World snapshot export", "group": "Mapping", "kind": "compute"},
    "rog_total_update_us": {"label": "ROG-Map update total", "group": "ROG-Map", "kind": "total"},
    "rog_raycast_us": {"label": "ROG raycast", "group": "ROG-Map", "kind": "compute"},
    "rog_probability_update_us": {"label": "ROG probability update", "group": "ROG-Map", "kind": "compute"},
    "rog_inflation_us": {"label": "ROG inflation", "group": "ROG-Map", "kind": "compute"},
    "rog_slide_us": {"label": "ROG slide", "group": "ROG-Map", "kind": "compute"},
    "planning_latency_ms": {"label": "Planner cycle total", "group": "Planner", "kind": "total"},
    "planning_scheduling_gap_us": {"label": "Planner scheduling gap", "group": "Planner", "kind": "wait"},
    "mapping_input_lock_wait_us": {"label": "Planner input-lock wait", "group": "Planner", "kind": "wait"},
    "exp_frontend_us": {"label": "Nominal trajectory frontend", "group": "Planner", "kind": "compute"},
    "exp_opt_us": {"label": "Nominal trajectory optimizer", "group": "Planner", "kind": "compute"},
    "backup_frontend_us": {"label": "Backup trajectory frontend", "group": "Planner", "kind": "compute"},
    "backup_opt_us": {"label": "Backup trajectory optimizer", "group": "Planner", "kind": "compute"},
    "scan_processing_p95_us": {"label": "LIO scan processing (reported p95)", "group": "LIO", "kind": "total"},
    "measurement_model_us": {"label": "LIO measurement model", "group": "LIO", "kind": "compute"},
    "ikfom_solver_only_us": {"label": "LIO IKFoM solver", "group": "LIO", "kind": "compute"},
    "map_maintenance_us": {"label": "LIO map maintenance", "group": "LIO", "kind": "compute"},
    "last_replay_runtime_us": {"label": "LIO propagation replay", "group": "LIO", "kind": "compute"},
    "maximum_replay_runtime_us": {"label": "LIO propagation replay maximum", "group": "LIO", "kind": "total"},
    "processing_lag_ns": {"label": "LIO transport processing lag", "group": "LIO", "kind": "wait"},
}


def _timing_value_to_us(value: Any, unit: Any) -> float | None:
    number = finite(value)
    if number is None:
        return None
    return number * {"us": 1.0, "ms": 1000.0, "ns": 0.001}.get(str(unit), 1.0)


def _timing_phase_rows(observability: dict[str, Any]) -> list[dict[str, Any]]:
    rows = []
    group_order = {"Wait": 0, "Input": 1, "Mapping": 2, "ROG-Map": 3, "Planner": 4, "LIO": 5}
    for item in observability.get("timing", []):
        if integer(item.get("nonzero_count")) == "—" or int(item.get("nonzero_count") or 0) <= 0:
            continue
        stats = item.get("stats", {})
        meta = _TIMING_PHASE_META.get(str(item.get("metric")), {
            "label": str(item.get("metric")), "group": str(item.get("component")), "kind": "observed",
        })
        converted = {
            key: _timing_value_to_us(stats.get(key), item.get("unit"))
            for key in ("mean", "p50", "p95", "p99", "maximum")
        }
        if converted["maximum"] is None:
            continue
        rows.append({
            **item,
            **meta,
            **converted,
            "metric": str(item.get("metric")),
        })
    return sorted(rows, key=lambda row: (group_order.get(str(row.get("group")), 9), -float(row.get("p95") or 0.0)))


def _timing_display(value_us: Any) -> str:
    value = finite(value_us)
    if value is None:
        return "—"
    if value >= 1000.0:
        return f"{value / 1000.0:.2f} ms"
    return f"{value:.0f} µs"


def _timing_row_map(observability: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(row.get("metric")): row for row in _timing_phase_rows(observability)}


def _timing_overview_cards(observability: dict[str, Any]) -> str:
    rows = _timing_row_map(observability)
    cards = [
        ("Mapping callback", "mapping_callback_total_us", "measured total", TEAL),
        ("Planner cycle", "planning_latency_ms", "measured total", PURPLE),
        ("LIO measurement phase", "measurement_model_us", "phase only; not LIO total", BLUE),
        ("Observation pair wait", "observation_pair_wait_us", "queue/synchronization wait", ORANGE),
    ]
    html_cards = []
    for label, metric, kind, color in cards:
        row = rows.get(metric)
        if row is None:
            value = "N/A"
            detail = "not recorded"
        else:
            value = _timing_display(row.get("p95"))
            detail = f"p50 {_timing_display(row.get('p50'))} · max {_timing_display(row.get('maximum'))} · n {integer(row.get('count'))}"
        html_cards.append(
            f'<div class="timing-kpi" style="border-top-color:{color}">'
            f'<div class="timing-kpi-label">{esc(label)}</div>'
            f'<div class="timing-kpi-value">{esc(value)}</div>'
            f'<div class="timing-kpi-kind">p95 · {esc(kind)}</div>'
            f'<div class="timing-kpi-detail">{esc(detail)}</div></div>'
        )
    return f'<div class="timing-kpis">{"".join(html_cards)}</div>'


def _timing_distribution_panel(group: str, rows: list[dict[str, Any]]) -> str:
    width = 700
    left, right, top, bottom = 200, 215, 36, 28
    row_height = 29
    height = top + row_height * len(rows) + bottom
    maximum = max(float(row.get("maximum") or 0.0) for row in rows)
    axis_max = max(1.0, maximum * 1.12)

    def x(value: Any) -> float:
        number = max(0.0, float(value or 0.0))
        return left + min(1.0, number / axis_max) * (width - left - right)

    group_colors = {"Wait": ORANGE, "Input": BLUE, "Mapping": TEAL, "ROG-Map": TEAL, "Planner": PURPLE, "LIO": BLUE}
    color = group_colors.get(group, SLATE)
    parts = [
        f'<div class="chart-card timing-panel"><div class="chart-title">{esc(group)}</div>',
        f'<svg class="chart timing-chart" viewBox="0 0 {width} {height}" role="img" aria-label="{esc(group)} timing distribution">',
        f'<title>{esc(group)} timing distribution</title><rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff"/>',
    ]
    for fraction in (0.0, 0.5, 1.0):
        value = axis_max * fraction
        tx = x(value)
        parts.append(
            f'<line x1="{tx:.1f}" y1="{top-8}" x2="{tx:.1f}" y2="{height-bottom}" stroke="{GRID}"/>'
            f'<text x="{tx:.1f}" y="{height-bottom+18}" text-anchor="middle" class="axis-label">{esc(_timing_display(value))}</text>'
        )
    for index, row in enumerate(rows):
        y = top + index * row_height + row_height / 2
        p50 = float(row.get("p50") or 0.0)
        p95 = float(row.get("p95") or 0.0)
        maximum_value = float(row.get("maximum") or 0.0)
        x50, x95, xmax = x(p50), x(p95), x(maximum_value)
        kind = str(row.get("kind") or "observed")
        stroke = color if kind != "wait" else ORANGE
        weight = "800" if kind == "total" else "400"
        parts.extend([
            f'<text x="{left-10}" y="{y+4:.1f}" text-anchor="end" class="axis-label" font-weight="{weight}">{esc(row["label"])}</text>',
            f'<line x1="{x50:.1f}" y1="{y:.1f}" x2="{x95:.1f}" y2="{y:.1f}" stroke="{stroke}" stroke-width="9" stroke-linecap="round" opacity="0.78"><title>{esc(row["label"])} p50–p95</title></line>',
            f'<line x1="{x95:.1f}" y1="{y:.1f}" x2="{xmax:.1f}" y2="{y:.1f}" stroke="#526578" stroke-width="2"><title>p95 to max</title></line>',
            f'<circle cx="{x50:.1f}" cy="{y:.1f}" r="4.2" fill="#18324b"><title>p50 {_timing_display(p50)}</title></circle>',
            f'<line x1="{xmax:.1f}" y1="{y-8:.1f}" x2="{xmax:.1f}" y2="{y+8:.1f}" stroke="#18324b" stroke-width="2"><title>max {_timing_display(maximum_value)}</title></line>',
            f'<text x="{width-right+8}" y="{y+4:.1f}" class="axis-label">{esc(_timing_display(p50))} / {esc(_timing_display(p95))} / {esc(_timing_display(maximum_value))} · n {integer(row.get("count"))}</text>',
        ])
    parts.extend([
        f'<line x1="{left}" y1="{top-8}" x2="{left}" y2="{height-bottom}" stroke="{NAVY}" stroke-width="1.2"/>',
        f'<text x="{left + (width-left-right)/2:.1f}" y="{height-7}" text-anchor="middle" class="axis-title">duration · linear scale</text>',
        '</svg></div>',
    ])
    return "".join(parts)


def _timing_distribution_chart(observability: dict[str, Any]) -> str:
    rows = _timing_phase_rows(observability)
    if not rows:
        return '<div class="empty-chart">No non-zero phase timing samples recorded.</div>'
    group_order = ["Wait", "Input", "Mapping", "ROG-Map", "Planner", "LIO"]
    panels = []
    for group in group_order:
        group_rows = [row for row in rows if str(row.get("group")) == group]
        if group_rows:
            panels.append(_timing_distribution_panel(group, group_rows))
    legend = (
        '<div class="timing-legend">'
        '<span><i class="timing-key compute"></i>compute</span>'
        '<span><i class="timing-key wait"></i>wait / queue</span>'
        '<span><i class="timing-key total"></i>aggregate total</span>'
        '<span>dot = p50 · thick segment = p50–p95 · whisker = p95–max</span>'
        '</div>'
    )
    return f'<div class="timing-panel-grid">{legend}{"".join(panels)}</div>'


def _timing_timeline_chart(observability: dict[str, Any]) -> str:
    wanted = (
        "observation_pair_wait_us",
        "mapping_callback_total_us",
        "rog_total_update_us",
        "planning_latency_ms",
        "measurement_model_us",
    )
    rows = [row for metric in wanted for row in [_timing_row_map(observability).get(metric)] if row]
    rows = [row for row in rows if len(row.get("series") or []) >= 2]
    if not rows:
        return '<div class="empty-chart">No timestamped timing samples recorded.</div>'
    width = 980
    left, right, top, bottom = 205, 120, 28, 34
    row_height = 58
    height = top + row_height * len(rows) + bottom
    all_times = [finite(point.get("t")) for row in rows for point in row.get("series", [])]
    all_times = [value for value in all_times if value is not None]
    time_min, time_max = min(all_times), max(all_times)
    time_span = max(1.0e-9, time_max - time_min)

    def x(value: Any) -> float:
        return left + (float(value) - time_min) / time_span * (width - left - right)

    parts = [
        '<div class="chart-card timing-timeline"><div class="chart-title">Observed duration over simulation time</div>',
        '<p class="small timing-caption">Each dot is a diagnostic sample; this shows when values spike, not the exact start/end interval of the phase.</p>',
        f'<svg class="chart" viewBox="0 0 {width} {height}" role="img" aria-label="Observed timing over simulation time">',
        f'<title>Observed timing over simulation time</title><rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff"/>',
    ]
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        tx = left + fraction * (width - left - right)
        value = time_min + fraction * time_span
        parts.append(
            f'<line x1="{tx:.1f}" y1="{top-5}" x2="{tx:.1f}" y2="{height-bottom}" stroke="{GRID}"/>'
            f'<text x="{tx:.1f}" y="{height-bottom+19}" text-anchor="middle" class="axis-label">{value:.1f}s</text>'
        )
    state_colors = {"normal": "#e6f5eb", "safety": "#fde8e5", "unknown": "#f1f3f5"}
    for interval in observability.get("state_intervals", []):
        start, end = finite(interval.get("t_start")), finite(interval.get("t_end"))
        if start is None or end is None:
            continue
        x1, x2 = max(left, x(start)), min(width-right, x(end))
        if x2 <= x1:
            continue
        parts.append(
            f'<rect x="{x1:.1f}" y="{top-5}" width="{x2-x1:.1f}" height="{height-top-bottom+5}" fill="{state_colors.get(str(interval.get("state")), "#f1f3f5")}" opacity="0.7"><title>{esc(interval.get("state"))} state</title></rect>'
        )
    group_colors = {"Input": ORANGE, "Mapping": TEAL, "ROG-Map": TEAL, "Planner": PURPLE, "LIO": BLUE}
    for index, row in enumerate(rows):
        y_top = top + index * row_height
        plot_top, plot_bottom = y_top + 8, y_top + row_height - 9
        series = [point for point in row.get("series", []) if finite(point.get("t")) is not None and finite(point.get("value")) is not None]
        max_value = max([float(point["value"]) for point in series] + [1.0])
        p50, p95 = float(row.get("p50") or 0.0), float(row.get("p95") or 0.0)
        def y(value: Any) -> float:
            return plot_bottom - min(1.0, max(0.0, float(value or 0.0) / max_value)) * (plot_bottom - plot_top)
        color = group_colors.get(str(row.get("group")), SLATE)
        parts.extend([
            f'<text x="{left-12}" y="{y_top+25}" text-anchor="end" class="axis-label">{esc(row["label"])}</text>',
            f'<line x1="{left}" y1="{y(p95):.1f}" x2="{width-right}" y2="{y(p95):.1f}" stroke="{color}" stroke-dasharray="4 3" opacity="0.55"/>',
            f'<text x="{width-right+8}" y="{y(p95)+4:.1f}" class="axis-label">p95 {_timing_display(p95)}</text>',
        ])
        for point in series:
            parts.append(
                f'<circle cx="{x(point["t"]):.1f}" cy="{y(point["value"]):.1f}" r="2.2" fill="{color}" opacity="0.62"><title>{esc(row["label"])} at {float(point["t"]):.2f}s: {_timing_display(float(point["value"]))}</title></circle>'
            )
    parts.extend([
        f'<line x1="{left}" y1="{top-5}" x2="{left}" y2="{height-bottom}" stroke="{NAVY}" stroke-width="1.2"/>',
        f'<text x="{left + (width-left-right)/2:.1f}" y="{height-7}" text-anchor="middle" class="axis-title">simulation time (s)</text>',
        '</svg></div>',
    ])
    return "".join(parts)


def _timing_execution_model() -> str:
    lanes = [
        ("LIO", (("measurement model", "compute"), ("IKFoM solver", "compute"), ("propagation", "compute"))),
        ("Mapping worker", (("point-cloud decode", "compute"), ("pair wait", "wait"), ("ROG-Map update", "total"), ("snapshot export", "unavailable"))),
        ("Planner cycle", (("pinned snapshot", "compute"), ("plan / replan", "compute"), ("publish command", "compute"))),
    ]
    parts = ['<div class="timing-flow"><div class="chart-title">Logical execution model</div>']
    for label, boxes in lanes:
        parts.append(f'<div class="timing-lane"><div class="timing-lane-label">{esc(label)}</div><div class="timing-lane-track">')
        for index, (name, kind) in enumerate(boxes):
            if index:
                parts.append('<span class="timing-arrow">→</span>')
            parts.append(f'<span class="timing-box {esc(kind)}">{esc(name)}</span>')
        parts.append('</div></div>')
    parts.append('<div class="timing-flow-note">Arrows are source-level sequence contracts. LIO, mapping and planner occupy independent worker lanes and may run concurrently; exact overlap requires start/end events, which this artifact does not record.</div></div>')
    return "".join(parts)


def _timing_relationship_rows() -> str:
    rows = [
        ("Input preparation", "decode → observation pair wait", "Decode precedes pair admission. Pair wait is synchronization/queue delay, not compute; it is not added to mapping callback total.", "observed + source contract"),
        ("Mapping callback", "ROG-Map update → snapshot export", "mapping_callback_total_us is the callback envelope. ROG-Map update and snapshot export are inside this envelope when both are emitted.", "source contract; export unavailable in this artifact"),
        ("ROG-Map update", "raycast + probability update + inflation + slide", "rog_total_update_us is an aggregate. Its subphases are reported separately; do not sum them with the aggregate.", "source contract"),
        ("Planner cycle", "planning latency / optimizer phases", "Planner runs from a pinned world snapshot in a separate cycle. Module phase timings are only comparable when emitted by the same build.", "planner total observed; optimizer phases unavailable here"),
        ("LIO estimator", "measurement model → IKFoM solver → propagation", "These are separate LIO diagnostics. The artifact has no single LIO end-to-end envelope, so their sum is not claimed as total LIO latency.", "observed; composition not certified"),
        ("Concurrency", "mapping worker || planner cycle || LIO callbacks", "Mapping worker and planner cycle are independently scheduled; timestamps show samples, not proof of exact overlap for each invocation.", "source contract; overlap not directly recorded"),
    ]
    return "".join(
        f'<tr><td>{esc(phase)}</td><td>{esc(sequence)}</td><td>{esc(meaning)}</td><td>{esc(evidence)}</td></tr>'
        for phase, sequence, meaning, evidence in rows
    )


def _waypoint_event_rows(observability: dict[str, Any]) -> str:
    rows = []
    for item in observability.get("waypoint_events", []):
        if item.get("kind") != "waypoint_accepted":
            continue
        rows.append(
            f'<tr><td>{fmt(item.get("t"), 2, " s")}</td><td>WP{esc(item.get("waypoint_index", "—"))}</td>'
            f'<td>{esc(item.get("state_name", "—"))}</td><td>{esc(item.get("reason_name", "—"))}</td>'
            f'<td>{"yes" if item.get("waypoint_accepted") else "no"}</td>'
            f'<td>{fmt(item.get("acceptance_position_error_m"), 3, " m")}</td>'
            f'<td>{fmt(item.get("acceptance_speed_mps"), 3, " m/s")}</td></tr>'
        )
    return "".join(rows) or '<tr><td colspan="7">No waypoint state transitions recorded.</td></tr>'


def render(session: Path, output: Path) -> Path:
    data = _analyze(session)
    report = _load(session / "report.json", {})
    metrics = data["metrics"]
    observability = data.get("observability", {})
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
    setpoint_velocity_series = {
        axis: _pva_points(observability, "velocity", index)
        for index, axis in enumerate(("vx", "vy", "vz"))
    }
    setpoint_speed_series = []
    for item in observability.get("pva", []):
        velocity = item.get("velocity")
        timestamp = finite(item.get("t"))
        if timestamp is None or not isinstance(velocity, (list, tuple)) or len(velocity) < 3:
            continue
        values = [finite(value) for value in velocity[:3]]
        if all(value is not None for value in values):
            setpoint_speed_series.append((timestamp, math.sqrt(sum(value * value for value in values if value is not None))))

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
    safety_stop_observed = bool(safety.get("safety_stop_observed"))
    safety_stop_detail = (
        f"{safety.get('safety_stop_reason_name') or 'SAFETY_STOP'}"
        f" → Hold in {fmt(safety.get('safety_to_handover_ms'), 1, ' ms')}"
        if safety_stop_observed else "no structured safety-stop event"
    )
    status_rows = [
        ("Mission outcome", outcome or "N/A", expected_outcome_text, gates["mission"]),
        ("Waypoint acceptance", f"{accepted_count}/{waypoint_count}", "explicit acceptance evidence only", gates["waypoint"]),
        ("Tracking cross-track p95", fmt(cross_p95, 2, " m"), f"≤ {fmt(cross_limit, 2, ' m')}", gates["cross_track"]),
        ("Collision safety", f"{fmt(collision_count, 0)} collisions", "0 collisions", gates["collision"]),
        ("Application safety stop", "observed" if safety_stop_observed else "not observed", safety_stop_detail, _safety_stop_status(safety_stop_observed)),
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
        f'<tr><td class="observed">|{axis.upper()}| PVA command</td>'
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
        f'<tr><td class="observed">PVA command speed</td><td>{fmt(setpoint_speed_stats["mean"], 2, " m/s")}</td>'
        f'<td>{fmt(setpoint_speed_stats["p50"], 2, " m/s")}</td><td>{fmt(setpoint_speed_stats["p95"], 2, " m/s")}</td>'
        f'<td>{fmt(setpoint_speed_stats["p99"], 2, " m/s")}</td><td>{fmt(setpoint_speed_stats["max"], 2, " m/s")}</td>'
        f'<td>{integer(setpoint_speed_stats["count"])}</td></tr>'
    )

    stream_labels = {
        "ground_truth_odometry": "ground truth",
        "propagated_odometry": "LIO propagated",
        "corrected_odometry": "LIO corrected",
        "external_odometry": "PX4 external (NED→ENU)",
        "px4_odometry": "PX4 odometry (NED→ENU)",
        "local_position": "PX4 local position (NED→ENU)",
    }
    stream_colors = {
        "ground_truth_odometry": BLUE,
        "propagated_odometry": TEAL,
        "corrected_odometry": PURPLE,
        "external_odometry": ORANGE,
        "px4_odometry": SLATE,
        "local_position": RED,
    }
    observed_streams = [
        name for name in (
            "ground_truth_odometry", "propagated_odometry", "corrected_odometry",
            "external_odometry", "px4_odometry", "local_position",
        ) if name in observability.get("streams", {})
    ]
    position_plot_html = "".join(
        line_chart(
            f"Position {axis.upper()} · display ENU ({({'x': 'east', 'y': 'north', 'z': 'up'})[axis]})",
            [
                *[{"label": stream_labels[name], "points": _obs_points(observability, name, "position", axis), "color": stream_colors[name]}
                  for name in observed_streams],
                {"label": "PX4 setpoint (NED→ENU)", "points": _px4_setpoint_points(observability, "position_ned", ("x", "y", "z").index(axis)), "color": RED, "dash": "6 4"},
            ],
            "position (m)",
            state_intervals=observability.get("state_intervals", []),
        )
        for axis in ("x", "y", "z")
    )
    velocity_plot_html = "".join(
        line_chart(
            f"Velocity {axis} · display ENU ({({'vx': 'east', 'vy': 'north', 'vz': 'up'})[axis]})",
            [
                *[{"label": stream_labels[name], "points": _obs_points(observability, name, "velocity", axis), "color": stream_colors[name]}
                  for name in observed_streams],
                {"label": "PVA command (LIO ENU)", "points": setpoint_velocity_series[axis], "color": RED, "dash": "6 4"},
                {"label": "PX4 setpoint (NED→ENU)", "points": _px4_setpoint_points(observability, "velocity_ned", ("vx", "vy", "vz").index(axis)), "color": PURPLE, "dash": "3 3"},
            ],
            "velocity (m/s)",
            state_intervals=observability.get("state_intervals", []),
        )
        for axis in ("vx", "vy", "vz")
    )
    stream_rows = _stream_rows(observability)
    coordinate_contract_rows = _coordinate_contract_rows(observability)
    axis_rows = _axis_rows(observability)
    diagnostic_health_rows = _diagnostic_health_rows(observability)
    diagnostic_timing_rows = _diagnostic_timing_rows(observability)
    timing_overview_cards = _timing_overview_cards(observability)
    timing_phase_chart = _timing_distribution_chart(observability)
    timing_timeline_chart = _timing_timeline_chart(observability)
    timing_execution_model = _timing_execution_model()
    timing_relationship_rows = _timing_relationship_rows()
    waypoint_event_rows = _waypoint_event_rows(observability)
    trajectory_paths = observability.get("trajectory_paths", [])
    main_path_count = sum(int(finite(item.get("trajectory_flag")) or 0) == 1 for item in trajectory_paths)
    backup_path_count = sum(int(finite(item.get("trajectory_flag")) or 0) == 2 for item in trajectory_paths)
    path_observation_note = (
        f"Recorded command paths: {main_path_count} main/nominal and {backup_path_count} backup/safety. "
        "A missing backup path means no backup command was observed in this run; it is not treated as a successful safety-path test."
    )
    state_intervals = observability.get("state_intervals", [])
    observed_states = {str(item.get("state")) for item in state_intervals}
    state_observation_note = (
        "Background bands are derived from accepted PositionCommand flags: "
        "green = normal/main, red = safety/backup. "
        f"Observed states: {', '.join(sorted(observed_states)) if observed_states else 'none'}; "
        "an absent safety band means no BACKUP command was recorded."
    )

    plot_html = "".join([
        map_svg(data),
        line_chart("Cross-track error", [{"label": "error", "points": error_series, "color": RED}], "distance error (m)", threshold=cross_limit, threshold_label=f"acceptance limit {fmt(cross_limit, 2, ' m')}", state_intervals=observability.get("state_intervals", [])),
        line_chart("Velocity components", [{"label": "vx", "points": velocity_series["vx"], "color": BLUE}, {"label": "vy", "points": velocity_series["vy"], "color": TEAL}, {"label": "vz", "points": velocity_series["vz"], "color": ORANGE}], "velocity (m/s)", state_intervals=observability.get("state_intervals", [])),
        line_chart("Speed magnitude", [{"label": "measured", "points": speed_series, "color": BLUE}, {"label": "PVA command", "points": setpoint_speed_series, "color": TEAL, "dash": "6 4"}], "speed (m/s)", state_intervals=observability.get("state_intervals", [])),
    ])
    replay_html = replay_section(data)

    session_name = session.name
    sim_duration_s = finite(mission.get("duration_sim_s"))
    wall_elapsed_s = finite(mission.get("wall_elapsed_s"))
    sim_samples = data.get("ground_truth", [])
    sim_start_s = finite(sim_samples[0].get("t")) if sim_samples else None
    sim_end_s = finite(sim_samples[-1].get("t")) if sim_samples else None
    sim_window = (
        f"t={fmt(sim_start_s, 2)}–{fmt(sim_end_s, 2)} s"
        if sim_start_s is not None and sim_end_s is not None else "window unavailable"
    )
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
    .map-annotations {{ display:flex; flex-wrap:wrap; gap:6px 16px; padding:7px 4px 5px; border-top:1px solid #edf1f4; color:#53677b; font-size:11px; }} .map-annotations > div {{ display:flex; align-items:center; flex-wrap:wrap; gap:5px; }} .map-annotations strong {{ color:#29435d; }} .map-tag {{ display:inline-block; padding:2px 6px; border:1px solid #c9d3dc; border-radius:999px; background:#f7fafc; color:#40566b; }} .map-tag.route {{ border-color:#e8897e; background:#fde8e5; color:#8f2e26; }} .map-tag.waypoint {{ border-color:#e2b35a; background:#fff5da; color:#7b4e00; }} .map-tag.muted {{ color:#8795a1; }}
    .state-note {{ display:flex; align-items:center; flex-wrap:wrap; gap:5px; }} .state-key {{ display:inline-block; width:18px; height:10px; margin-left:7px; border:1px solid #b8c5ce; border-radius:2px; vertical-align:middle; }} .state-key.normal {{ background:#dff3e8; }} .state-key.safety {{ background:#fde3e0; }}
    .timing-kpis {{ display:grid; grid-template-columns:repeat(4,minmax(0,1fr)); gap:10px; margin:14px 0; }} .timing-kpi {{ min-height:112px; padding:12px 13px; border:1px solid #e3e9ee; border-top:4px solid; border-radius:8px; background:#fbfdfe; }} .timing-kpi-label {{ color:#53677b; font-size:12px; font-weight:800; }} .timing-kpi-value {{ margin-top:6px; color:#102b45; font-size:22px; font-weight:800; font-variant-numeric:tabular-nums; }} .timing-kpi-kind,.timing-kpi-detail {{ color:#68778a; font-size:11px; }} .timing-kpi-detail {{ margin-top:5px; font-variant-numeric:tabular-nums; }}
    .timing-panel-grid {{ display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:10px; }} .timing-panel-grid > .timing-legend {{ grid-column:1/-1; }} .timing-panel {{ min-width:0; }} .timing-legend {{ display:flex; flex-wrap:wrap; gap:7px 16px; align-items:center; color:#53677b; font-size:11px; padding:2px 4px; }} .timing-key {{ width:18px; height:8px; display:inline-block; margin-right:5px; vertical-align:middle; border-radius:5px; background:{BLUE}; }} .timing-key.wait {{ background:{ORANGE}; }} .timing-key.total {{ height:10px; border:2px solid {TEAL}; background:#fff; }}
    .timing-caption {{ margin:2px 5px 8px; }} .timing-timeline {{ margin-top:10px; }} .timing-flow {{ margin-top:10px; padding:10px; border:1px solid #e3e9ee; border-radius:9px; background:#fbfdfe; }} .timing-lane {{ display:grid; grid-template-columns:112px minmax(0,1fr); gap:10px; align-items:center; margin-top:8px; }} .timing-lane-label {{ color:#29435d; font-size:12px; font-weight:800; text-align:right; }} .timing-lane-track {{ display:flex; align-items:center; flex-wrap:wrap; gap:5px; }} .timing-box {{ display:inline-block; padding:6px 9px; border:1px solid #9ab5ca; border-radius:6px; background:#eaf4fb; color:#234d70; font-size:11px; font-weight:700; }} .timing-box.wait {{ border-color:#e0b45b; background:#fff5dc; color:#7b4e00; }} .timing-box.total {{ border:2px solid {TEAL}; background:#e4f5f3; color:#126466; }} .timing-box.unavailable {{ border-style:dashed; background:#f2f4f6; color:#7b8996; }} .timing-arrow {{ color:#7c8d9d; font-weight:900; }} .timing-flow-note {{ margin:11px 0 0 122px; color:#68778a; font-size:11px; }}
    .chart-card:first-child {{ grid-column:auto; }}
    .chart-title {{ margin:2px 5px 2px; font-weight:800; font-size:14px; color:#29435d; }}
    .chart {{ width:100%; height:auto; display:block; }} .chart-legend-toggle {{ cursor:pointer; }} .chart-legend-toggle:focus {{ outline:2px solid {BLUE}; outline-offset:2px; }} .chart-legend-toggle.off {{ opacity:.32; }}
    .axis-label {{ fill:#617284; font-size:11px; }} .axis-title {{ fill:#29435d; font-size:12px; font-weight:700; }} .legend-label {{ fill:#53677b; font-size:11px; }} .threshold-label {{ fill:{RED}; font-size:11px; font-weight:700; }} .end-label {{ font-size:11px; font-weight:800; }} .map-label {{ fill:#5a3f45; font-size:11px; font-weight:700; }} .bar-value {{ fill:#29435d; font-size:12px; font-weight:800; }} .bar-note {{ fill:#68778a; font-size:10px; }}
    .replay-toolbar {{ display:flex; align-items:center; gap:8px; flex-wrap:wrap; margin:12px 0 8px; }} .replay-toolbar button {{ border:1px solid #b9c9d6; border-radius:6px; background:#fff; color:#1f4f78; padding:7px 13px; font-weight:800; cursor:pointer; }} .replay-toolbar button:hover {{ background:#edf5fb; }} .replay-time-control {{ display:flex; align-items:center; gap:10px; flex:1; min-width:280px; color:#53677b; font-size:12px; font-weight:700; }} .replay-time-control input {{ flex:1; accent-color:{BLUE}; }} .replay-time-control output {{ min-width:62px; color:#18324b; font-variant-numeric:tabular-nums; }}
    .replay-event-caption {{ display:flex; justify-content:space-between; gap:12px; color:#68778a; font-size:11px; margin-top:8px; }} .replay-plan-events {{ height:18px; position:relative; margin:2px 3px 12px; border-bottom:1px solid #cdd8e1; background:linear-gradient(to bottom,transparent 0%,transparent 65%,#edf2f6 65%,#edf2f6 100%); }} .replay-marker {{ position:absolute; bottom:0; width:3px; height:11px; padding:0; border:0; cursor:pointer; transform:translateX(-1px); }} .replay-marker.nominal {{ background:{TEAL}; }} .replay-marker.safety {{ background:{RED}; }} .replay-marker.active {{ height:18px; width:5px; box-shadow:0 0 0 2px #f0a51a; z-index:2; }}
    .replay-grid {{ display:grid; grid-template-columns:minmax(0,1fr) 250px; gap:14px; align-items:start; }} .replay-map-card {{ min-width:0; border:1px solid #e3e9ee; border-radius:9px; background:#fff; padding:10px 10px 8px; overflow:hidden; }} .replay-map {{ width:100%; height:auto; display:block; }} .replay-status {{ border:1px solid #e3e9ee; border-radius:9px; background:#f8fafc; padding:14px; }} .replay-status h3 {{ margin-bottom:10px; }} .replay-status dl {{ margin:0; }} .replay-status dt {{ margin-top:9px; color:#68778a; font-size:11px; font-weight:800; text-transform:uppercase; letter-spacing:.05em; }} .replay-status dd {{ margin:2px 0 0; color:#1f3c56; font-size:13px; font-weight:700; overflow-wrap:anywhere; }} .replay-legend {{ display:flex; flex-wrap:wrap; gap:8px 18px; color:#53677b; font-size:11px; margin:5px 4px 0; }} .replay-heading-legend {{ color:{BLUE}; font-size:18px; line-height:10px; font-weight:900; margin-left:4px; }} .replay-swatch {{ display:inline-block; width:18px; height:3px; margin:0 5px 2px 0; vertical-align:middle; background:#1f6feb; }} .replay-swatch.nominal {{ background:{TEAL}; }} .replay-swatch.safety {{ background:{RED}; }} .replay-swatch.ground {{ background:{BLUE}; height:4px; }} .replay-svg-label {{ fill:#617284; font-size:11px; }} .replay-svg-title {{ fill:#29435d; font-size:12px; font-weight:700; }} .replay-map-label {{ fill:#5a3f45; font-size:11px; font-weight:700; }}
    .evidence {{ width:100%; border-collapse:collapse; font-size:13px; }} .evidence th,.evidence td {{ padding:10px 9px; border-bottom:1px solid #e5ebef; text-align:left; vertical-align:middle; }} .evidence th {{ color:var(--muted); font-size:11px; text-transform:uppercase; letter-spacing:.06em; }} .evidence .observed {{ font-weight:800; color:#1f3c56; }}
    .two-col {{ display:grid; grid-template-columns:1.1fr .9fr; gap:18px; align-items:start; }}
    .callout {{ padding:13px 15px; background:#f7fafc; border-left:4px solid var(--blue); border-radius:6px; }} .callout p {{ color:#385169; font-size:13px; }}
    details {{ margin-top:12px; border-top:1px solid var(--line); padding-top:12px; }} summary {{ cursor:pointer; font-weight:800; color:#2d5e8c; }}
    .raw-links {{ display:flex; flex-wrap:wrap; gap:8px 16px; margin-top:8px; }} a {{ color:#1b63a1; }} code {{ background:#eef3f7; padding:2px 5px; border-radius:4px; font-size:11px; }}
    .small {{ font-size:12px; color:var(--muted); }} footer {{ margin-top:18px; font-size:11px; color:#7b8996; }}
    .run-log {{ display:flex; flex-wrap:wrap; gap:8px 18px; margin:0 0 14px; padding:10px 13px; border:1px solid #cfe0ec; border-left:4px solid {BLUE}; border-radius:7px; background:#f5faff; color:#29435d; font-size:12px; font-variant-numeric:tabular-nums; }} .run-log strong {{ color:#163b5b; }}
    @media (max-width:900px) {{ .kpis {{ grid-template-columns:repeat(3,minmax(0,1fr)); }} .timing-kpis {{ grid-template-columns:repeat(2,minmax(0,1fr)); }} .timing-panel-grid {{ grid-template-columns:1fr; }} .two-col {{ grid-template-columns:1fr; }} .replay-grid {{ grid-template-columns:1fr; }} }}
    @media (max-width:600px) {{ main {{ padding:18px 12px 40px; }} .hero {{ display:block; }} .hero-right {{ margin-top:12px; }} .kpis,.timing-kpis {{ grid-template-columns:1fr 1fr; }} .findings {{ grid-template-columns:1fr; }} section {{ padding:14px; }} .evidence {{ font-size:12px; }} .evidence th:nth-child(3),.evidence td:nth-child(3) {{ display:none; }} .timing-lane {{ display:block; }} .timing-lane-label {{ text-align:left; margin-bottom:4px; }} .timing-flow-note {{ margin-left:0; }} }}
  </style>
</head>
<body>
<main>
  <header class="hero">
    <div><div class="eyebrow">PX4 / SITL flight review</div><h1>External Mode · {esc(metrics.get('mission', {}).get('waypoint_count', 0))}-waypoint mission</h1><p class="session">Session: {esc(session_name)} · {fmt(mission.get('duration_sim_s'), 1, ' s')} simulated · generated from recorded artifacts</p></div>
    <div class="hero-right">{status_chip(evaluation["overall"], evaluation["overall"])}</div>
  </header>

  <div class="run-log" aria-label="Simulation runtime"><span><strong>Simulation runtime</strong> {fmt(sim_duration_s, 3, ' s')}</span><span><strong>Recorded telemetry window</strong> {esc(sim_window)}</span><span><strong>Wall elapsed</strong> {fmt(wall_elapsed_s, 3, ' s')}</span></div>

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

  <section><h2>Waypoint and command-state timeline</h2><p class="small">The replay cursor exposes the latest vehicle state, active waypoint, acceptance result, and active command path. This table is the durable waypoint evidence behind the replay.</p><table class="evidence"><thead><tr><th>Time</th><th>Waypoint</th><th>State</th><th>Reason</th><th>Accepted</th><th>Position error</th><th>Speed at acceptance</th></tr></thead><tbody>{waypoint_event_rows}</tbody></table><p class="small">{esc(path_observation_note)}</p></section>

  <section><h2>Coordinate-frame contract</h2><p class="small">All charts and map/replay geometry use one display frame: ENU, with <strong>x=east, y=north, z=up</strong>. PX4 vectors are converted before statistics and plotting with <code>[x,y,z]ENU = [y,x,-z]NED</code>; LIO, ground truth and PVA are already ENU.</p><table class="evidence"><thead><tr><th>Source</th><th>Recorded convention</th><th>Report transform</th><th>Display convention</th></tr></thead><tbody>{coordinate_contract_rows}</tbody></table></section>

  <section><h2>Telemetry coverage</h2><p class="small">Position and velocity streams below are already normalized to display ENU. PX4 external odometry, PX4 odometry, local position and setpoint events carry an explicit NED→ENU label in the charts. Gaps are based on consecutive accepted recorder samples.</p><table class="evidence"><thead><tr><th>Stream</th><th>Samples</th><th>Mean rate</th><th>Duration</th><th>Gap p95</th><th>Max gap</th></tr></thead><tbody>{stream_rows}</tbody></table><details><summary>Position XYZ and velocity XYZ statistics</summary><table class="evidence"><thead><tr><th>Stream</th><th>Axis</th><th>Mean</th><th>P50</th><th>P95</th><th>Max</th><th>Samples</th></tr></thead><tbody>{axis_rows}</tbody></table></details></section>

  <section><h2>LIO and planner diagnostics</h2><p class="small">These are explicit diagnostic fields from FAST-LIO and SUPER/ROG-Map. Boolean/counter fields show the latest observed value and diagnostic sample count; unavailable fields are omitted or shown as N/A.</p><table class="evidence"><thead><tr><th>Component</th><th>Source</th><th>Field</th><th>Latest observed</th><th>Samples</th></tr></thead><tbody>{diagnostic_health_rows}</tbody></table></section>

  {failure_reasons_html}
  {kinematics_html}

  <section><h2>Measured processing time · raw table</h2><p class="small">Detailed diagnostic values are retained here for audit. The visual summary below is the primary interpretation view.</p><details><summary>Show raw timing statistics</summary><table class="evidence"><thead><tr><th>Component</th><th>Source</th><th>Metric</th><th>Unit</th><th>Mean</th><th>P50</th><th>P95</th><th>P99</th><th>Max</th><th>Samples</th><th>Non-zero</th></tr></thead><tbody>{diagnostic_timing_rows}</tbody></table></details></section>

  <section><h2>Timing overview · spread and execution model</h2><p class="small">Read the cards first, then compare phases within each subsystem panel. A thick segment is p50–p95; the thin whisker is p95–max. Totals are not added to their child phases. The timeline uses diagnostic sample timestamps, not invented phase start/end times.</p>{timing_overview_cards}{timing_phase_chart}{timing_timeline_chart}{timing_execution_model}<details><summary>Show timing composition and evidence limits</summary><table class="evidence"><thead><tr><th>Phase group</th><th>Observed/source sequence</th><th>How to read the timing</th><th>Evidence status</th></tr></thead><tbody>{timing_relationship_rows}</tbody></table></details></section>

  <section><h2>Position, velocity and setpoint traces</h2><p class="small">These traces are the system-level supervision view: ground truth, LIO propagated/corrected odometry, PX4 odometry/local position and recorded PVA commands. If a stream is absent, the corresponding chart explicitly reports no samples.</p><p class="small state-note"><span class="state-key normal"></span>normal/main <span class="state-key safety"></span>safety/backup · {esc(state_observation_note)}</p><div class="charts">{position_plot_html}{velocity_plot_html}</div></section>

  <section><h2>Flight overview</h2><p class="small">Plots are sampled for readability. Each plot has its own scale, units, labelled axes and legend; p95/limits remain visible in the cards above and in the gate table.</p><div class="charts">{plot_html}</div></section>

  <section><div class="two-col"><div><h2>Interpretation</h2><div class="callout"><p>{esc(interpretation)}</p></div></div><div><h2>Run context</h2><p class="small"><strong>Estimator:</strong> {esc(lio.get('state') or 'N/A')} · residual p95 {fmt(metrics.get('localization', {}).get('p95_position_residual_m'), 3, ' m')}.</p><p class="small"><strong>PX4:</strong> {esc(px4_observed)}; failsafe observed = {esc(external.get('failsafe_seen'))}.</p><p class="small"><strong>Planner:</strong> {integer(planning.get('diagnostic_sample_count'))} diagnostic samples · {integer(continuity.get('endpoint_change_count'))} endpoint changes · {integer(smoothness.get('handover_expired_count'))} expired handovers.</p></div></div></section>

  <section><h2>Evidence and raw artifacts</h2><p class="small">{esc(trace_note)}</p><details><summary>Source files</summary><div class="raw-links">{links_html}</div></details><details><summary>Provenance</summary><p class="small">Navigation commit: <code>{esc(navigation_commit)}</code> · dirty workspace: <code>{esc(navigation_dirty)}</code>.</p></details></section>

  <footer>Report purpose: human evaluation of one SITL run. Use the linked raw artifacts for debugging; do not use this page as a substitute for the full recorder output.</footer>
</main>
<script>
  document.querySelectorAll('.chart-legend-toggle').forEach(function (legend) {{
    function toggle() {{
      const target = document.getElementById(legend.dataset.target);
      if (!target) return;
      const hidden = target.style.display === 'none';
      target.style.display = hidden ? '' : 'none';
      legend.classList.toggle('off', !hidden);
      legend.setAttribute('aria-pressed', hidden ? 'true' : 'false');
    }}
    legend.addEventListener('click', toggle);
    legend.addEventListener('keydown', function (event) {{
      if (event.key === 'Enter' || event.key === ' ') {{ event.preventDefault(); toggle(); }}
    }});
  }});
</script>
</body>
</html>
"""
    output.write_text(html_text, encoding="utf-8")
    return output
