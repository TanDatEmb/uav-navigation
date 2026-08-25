#!/usr/bin/env python3
"""Bounded, diagnostic-only Gazebo Transport and process observation.

Normal SITL never starts this helper. When explicitly enabled it subscribes
through the installed Gazebo Transport Python binding, so it does not create a
high-rate CLI echo process or serialize every native message. Only bounded gap
events and low-rate process/PSI samples are persisted.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
from typing import Any

HZ = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
PAGE_SIZE = os.sysconf("SC_PAGE_SIZE")
GAP_EVENT_CAPACITY = 1024


def _load_json(path: Path, default: Any) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return default


def _atomic_json(path: Path, payload: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def _message_time_ns(message: Any, field_name: str) -> int:
    value = message.get(field_name) if isinstance(message, dict) else getattr(message, field_name, None)
    if value is None:
        return 0
    if isinstance(value, dict):
        return int(value.get("sec", 0) or 0) * 1_000_000_000 + int(value.get("nsec", 0) or 0)
    return int(getattr(value, "sec", 0) or 0) * 1_000_000_000 + int(getattr(value, "nsec", 0) or 0)


def _process_stat(pid: int) -> dict[str, Any] | None:
    proc = Path("/proc") / str(pid)
    try:
        stat = (proc / "stat").read_text(encoding="utf-8")
        status = (proc / "status").read_text(encoding="utf-8")
        exe = os.readlink(proc / "exe")
        cmdline = (proc / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", errors="replace").strip()
    except OSError:
        return None
    closing = stat.rfind(")")
    if closing < 0:
        return None
    fields = stat[closing + 2:].split()
    if len(fields) < 22:
        return None
    status_values: dict[str, str] = {}
    for line in status.splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            status_values[key] = value.strip()
    try:
        result: dict[str, Any] = {
            "state": fields[0],
            "pgrp": int(fields[2]),
            "utime_s": int(fields[11]) / HZ,
            "stime_s": int(fields[12]) / HZ,
            "num_threads": int(fields[17]),
            "rss_bytes": int(fields[21]) * PAGE_SIZE,
            "voluntary_ctxt_switches": int(status_values.get("voluntary_ctxt_switches", "0")),
            "nonvoluntary_ctxt_switches": int(status_values.get("nonvoluntary_ctxt_switches", "0")),
            "exe": exe,
            "cmdline": cmdline,
        }
    except (TypeError, ValueError):
        return None
    try:
        sched = (proc / "schedstat").read_text(encoding="utf-8").split()
        if len(sched) >= 3:
            result["sched_runtime_ns"] = int(sched[0])
            result["sched_wait_ns"] = int(sched[1])
            result["sched_timeslices"] = int(sched[2])
    except (OSError, ValueError):
        pass
    return result


def _psi_snapshot() -> dict[str, str]:
    result: dict[str, str] = {}
    for name in ("cpu", "memory", "io"):
        try:
            result[name] = (Path("/proc/pressure") / name).read_text(encoding="utf-8").strip()
        except OSError:
            continue
    return result


def _registered_groups(session: Path) -> dict[int, set[str]]:
    registry = _load_json(session / "processes.json", {})
    records = registry.get("processes", []) if isinstance(registry, dict) else []
    groups: dict[int, set[str]] = {}
    for record in records:
        if not isinstance(record, dict):
            continue
        try:
            pgid = int(record["pgid"])
        except (KeyError, TypeError, ValueError):
            continue
        groups.setdefault(pgid, set()).add(str(record.get("role", "unknown")))
    return groups


def _process_samples(session: Path) -> list[dict[str, Any]]:
    groups = _registered_groups(session)
    if not groups:
        return []
    samples: list[dict[str, Any]] = []
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            pid = int(proc.name)
        except ValueError:
            continue
        stat = _process_stat(pid)
        if stat is None or stat.get("pgrp") not in groups or stat.get("state") == "Z":
            continue
        for role in sorted(groups[int(stat["pgrp"])]):
            samples.append({"role": role, "pid": pid, "pgid": int(stat["pgrp"]), "stat": stat})
    return samples


@dataclass
class _StreamState:
    kind: str
    last_arrival_ns: int = 0
    first_arrival_ns: int = 0
    last_source_ns: int = 0
    first_source_ns: int = 0
    last_iterations: int | None = None
    first_iterations: int | None = None
    count: int = 0
    max_gap_ns: int = 0
    gap_events: list[dict[str, Any]] = field(default_factory=list)
    gap_event_overflow: int = 0
    source_regressions: int = 0
    source_duplicates: int = 0
    real_time_factors: list[float] = field(default_factory=list)
    lock: threading.Lock = field(default_factory=threading.Lock)

    def record(self, *, arrival_ns: int, source_ns: int, iterations: int | None = None,
               real_time_factor: float | None = None, gap_budget_ns: int) -> None:
        with self.lock:
            if self.last_arrival_ns:
                gap_ns = arrival_ns - self.last_arrival_ns
                self.max_gap_ns = max(self.max_gap_ns, gap_ns)
                if gap_ns > gap_budget_ns:
                    event = {
                        "kind": "arrival_gap", "stream": self.kind,
                        "before_arrival_ns": self.last_arrival_ns,
                        "after_arrival_ns": arrival_ns,
                        "threshold_crossing_ns": self.last_arrival_ns + gap_budget_ns,
                        "gap_ns": gap_ns, "before_source_ns": self.last_source_ns,
                        "after_source_ns": source_ns,
                    }
                    if len(self.gap_events) < GAP_EVENT_CAPACITY:
                        self.gap_events.append(event)
                    else:
                        self.gap_event_overflow += 1
            if source_ns:
                if self.last_source_ns and source_ns < self.last_source_ns:
                    self.source_regressions += 1
                elif self.last_source_ns and source_ns == self.last_source_ns:
                    self.source_duplicates += 1
                if not self.first_source_ns:
                    self.first_source_ns = source_ns
                self.last_source_ns = source_ns
            if iterations is not None:
                if self.last_iterations is not None and iterations < self.last_iterations:
                    self.source_regressions += 1
                if self.first_iterations is None:
                    self.first_iterations = iterations
                self.last_iterations = iterations
            if real_time_factor is not None and math.isfinite(real_time_factor):
                if len(self.real_time_factors) < GAP_EVENT_CAPACITY:
                    self.real_time_factors.append(real_time_factor)
            if not self.first_arrival_ns:
                self.first_arrival_ns = arrival_ns
            self.last_arrival_ns = arrival_ns
            self.count += 1

    def finalize(self, now_ns: int, gap_budget_ns: int) -> None:
        with self.lock:
            if self.last_arrival_ns and now_ns - self.last_arrival_ns > gap_budget_ns:
                event = {
                    "kind": "arrival_gap", "stream": self.kind,
                    "before_arrival_ns": self.last_arrival_ns, "after_arrival_ns": now_ns,
                    "threshold_crossing_ns": self.last_arrival_ns + gap_budget_ns,
                    "gap_ns": now_ns - self.last_arrival_ns,
                    "before_source_ns": self.last_source_ns, "after_source_ns": 0,
                    "terminal": True,
                }
                if len(self.gap_events) < GAP_EVENT_CAPACITY:
                    self.gap_events.append(event)
                else:
                    self.gap_event_overflow += 1

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            factors = list(self.real_time_factors)
            return {
                "count": self.count,
                "first_arrival_ns": self.first_arrival_ns,
                "last_arrival_ns": self.last_arrival_ns,
                "first_source_ns": self.first_source_ns,
                "last_source_ns": self.last_source_ns,
                "first_iterations": self.first_iterations,
                "last_iterations": self.last_iterations,
                "maximum_arrival_gap_ms": self.max_gap_ns / 1e6,
                "arrival_gap_events": list(self.gap_events),
                "arrival_gap_event_overflow": self.gap_event_overflow,
                "source_regression_count": self.source_regressions,
                "source_duplicate_count": self.source_duplicates,
                "real_time_factor_min": min(factors) if factors else None,
                "real_time_factor_max": max(factors) if factors else None,
            }


def _summarize(samples: list[dict[str, Any]]) -> dict[str, Any]:
    """Summarize event/process rows for unit tests and old observer artifacts."""
    summary: dict[str, Any] = {"schema_version": 2, "gazebo_native": {}, "process_roles": {}, "psi_samples": 0}
    for row in samples:
        kind = row.get("kind")
        if kind == "process_sample":
            for process in row.get("processes", []):
                role = str(process.get("role", "unknown"))
                summary["process_roles"][role] = summary["process_roles"].get(role, 0) + 1
        elif kind == "psi_sample":
            summary["psi_samples"] += 1
    for stream in ("world_stats", "world_clock"):
        rows = [row for row in samples if row.get("stream") == stream or row.get("kind") == stream]
        source_field = "sim_time" if stream == "world_stats" else "sim"
        source_values = [
            _message_time_ns(row.get("payload", {}), source_field)
            if not isinstance(row.get("payload", {}).get(source_field), int)
            else int(row.get("payload", {}).get(source_field, 0))
            for row in rows
            if isinstance(row.get("payload"), dict)
        ]
        source_values = [value for value in source_values if value > 0]
        summary["gazebo_native"][stream] = {
            "samples": len(rows),
            "first_sim_time_ns": source_values[0] if source_values else 0,
            "last_sim_time_ns": source_values[-1] if source_values else 0,
            "arrival_gap_events": [row for row in samples if row.get("stream") == stream and row.get("kind") == "arrival_gap"],
        }
    return summary


def _run(args: argparse.Namespace, stop: threading.Event) -> int:
    try:
        import gz.transport13 as transport
        import gz.msgs10.clock_pb2 as clock_pb2
        import gz.msgs10.world_stats_pb2 as world_stats_pb2
    except ImportError as error:
        _atomic_json(args.session / "gazebo_native_summary.json", {"schema_version": 2, "status": "UNAVAILABLE", "error": str(error)})
        return 1
    args.session.mkdir(parents=True, exist_ok=True)
    stats_state = _StreamState("world_stats")
    clock_state = _StreamState("world_clock")
    gap_budget_ns = int(args.gap_budget_s * 1e9)
    node = transport.Node()

    def on_stats(message: Any) -> None:
        factor = float(getattr(message, "real_time_factor", 0.0) or 0.0)
        stats_state.record(arrival_ns=time.time_ns(), source_ns=_message_time_ns(message, "sim_time"),
                           iterations=int(getattr(message, "iterations", 0) or 0),
                           real_time_factor=factor if math.isfinite(factor) else None,
                           gap_budget_ns=gap_budget_ns)

    def on_clock(message: Any) -> None:
        clock_state.record(arrival_ns=time.time_ns(), source_ns=_message_time_ns(message, "sim"), gap_budget_ns=gap_budget_ns)

    stats_topic = f"/world/{args.world}/stats"
    clock_topic = f"/world/{args.world}/clock"
    node.subscribe(world_stats_pb2.WorldStatistics, stats_topic, on_stats)
    node.subscribe(clock_pb2.Clock, clock_topic, on_clock)
    samples_path = args.session / "gazebo_native_samples.jsonl"
    deadline = time.monotonic() + args.duration_s if args.duration_s > 0.0 else math.inf
    next_process_sample = 0.0
    with samples_path.open("a", encoding="utf-8") as output:
        while not stop.is_set() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_process_sample:
                output.write(json.dumps({"kind": "process_sample", "arrival_wall_ns": time.time_ns(), "processes": _process_samples(args.session)}, sort_keys=True) + "\n")
                output.write(json.dumps({"kind": "psi_sample", "arrival_wall_ns": time.time_ns(), "psi": _psi_snapshot()}, sort_keys=True) + "\n")
                output.flush()
                next_process_sample = now + args.process_period_s
            time.sleep(0.05)
    now_ns = time.time_ns()
    stats_state.finalize(now_ns, gap_budget_ns)
    clock_state.finalize(now_ns, gap_budget_ns)
    node.unsubscribe(stats_topic)
    node.unsubscribe(clock_topic)
    _atomic_json(args.session / "gazebo_native_summary.json", {
        "schema_version": 2, "status": "OK", "world": args.world,
        "gap_budget_s": args.gap_budget_s,
        "gazebo_native": {"world_stats": stats_state.snapshot(), "world_clock": clock_state.snapshot()},
        "verdict_owner": "diagnostic_only",
    })
    return 0


def run(args: argparse.Namespace) -> int:
    stop = threading.Event()
    def _stop(_signum: int, _frame: Any) -> None:
        stop.set()
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    return _run(args, stop)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--gz-command", default="gz", help="compatibility option; direct transport does not invoke it")
    parser.add_argument("--process-period-s", type=float, default=1.0)
    parser.add_argument("--duration-s", type=float, default=0.0)
    parser.add_argument("--gap-budget-s", type=float, default=0.5)
    args = parser.parse_args()
    if args.process_period_s <= 0.0 or not math.isfinite(args.process_period_s):
        raise ValueError("--process-period-s must be finite and positive")
    if args.duration_s < 0.0 or not math.isfinite(args.duration_s):
        raise ValueError("--duration-s must be finite and nonnegative")
    if args.gap_budget_s <= 0.0 or not math.isfinite(args.gap_budget_s):
        raise ValueError("--gap-budget-s must be finite and positive")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
