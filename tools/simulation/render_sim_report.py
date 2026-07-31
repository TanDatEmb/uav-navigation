#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


def fmt(value, digits=3):
    if value is None:
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", required=True, type=Path)
    args = parser.parse_args()

    summary_path = args.session / "summary.json"
    if not summary_path.exists():
        print(f"No summary found: {summary_path}")
        return 1

    data = json.loads(summary_path.read_text(encoding="utf-8"))
    streams = data.get("streams", {})

    lines = [
        "# PX4 MID-360 Simulation Report",
        "",
        f"- Session: `{args.session}`",
        f"- Started: `{data.get('started_at', 'n/a')}`",
        f"- Duration: `{fmt(data.get('elapsed_s'))} s`",
        f"- Clock regressions: `{data.get('clock_regressions', 0)}`",
        f"- Diagnostic warnings: `{data.get('diagnostic_warnings', 0)}`",
        f"- Diagnostic errors: `{data.get('diagnostic_errors', 0)}`",
        "",
        "| Stream | Count | Rate Hz | Max gap s | P95 gap s | Stamp regressions | Invalid |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]

    for name in ("imu", "lidar", "odom", "registered", "local_map", "diagnostics"):
        item = streams.get(name, {})
        lines.append(
            f"| {name} | {item.get('count', 0)} | {fmt(item.get('rate_hz'))} | "
            f"{fmt(item.get('gap_max_s'))} | {fmt(item.get('gap_p95_s'))} | "
            f"{item.get('stamp_regressions', 0)} | {item.get('nonfinite_messages', 0)} |"
        )

    events = args.session / "events.log"
    lines += ["", "## Events", ""]
    if events.exists() and events.stat().st_size:
        lines.append("```text")
        lines.extend(events.read_text(encoding="utf-8").splitlines())
        lines.append("```")
    else:
        lines.append("No watchdog event was recorded.")

    report = args.session / "REPORT.md"
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
