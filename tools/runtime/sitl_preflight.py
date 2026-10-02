"""Host checks used before a SITL session starts.

These checks protect evidence quality only.  They do not change planner,
controller, lease, freshness, or acceptance policy.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import subprocess
import threading
import time
from typing import Iterable


DEFAULT_LOADAVG_LIMIT = max(1.0, float(os.cpu_count() or 1) * 0.75)
_COMPETING_PROCESS = re.compile(r"(?:^|[\s/])(?:colcon|cmake|cc1plus|ninja|make)(?:$|[\s/])")


@dataclass(frozen=True)
class ProcessRow:
    pid: int
    command: str
    cpu_percent: float | None
    args: str


class PreflightViolation(RuntimeError):
    """Raised when host conditions make a SITL result non-evaluable."""

    def __init__(
        self,
        *,
        loadavg_1m: float,
        max_loadavg_1m: float,
        process_rows: Iterable[ProcessRow],
        active_runtime_sessions: Iterable[str],
    ) -> None:
        self.loadavg_1m = loadavg_1m
        self.max_loadavg_1m = max_loadavg_1m
        self.process_rows = tuple(process_rows)
        self.active_runtime_sessions = tuple(active_runtime_sessions)
        reasons: list[str] = []
        if loadavg_1m > max_loadavg_1m:
            reasons.append(
                f"loadavg_1m={loadavg_1m:.2f} exceeds limit={max_loadavg_1m:.2f}"
            )
        if self.process_rows:
            reasons.append(
                "competing build processes="
                + ",".join(f"{row.pid}:{row.command}" for row in self.process_rows)
            )
        if self.active_runtime_sessions:
            reasons.append(
                "active runtime sessions=" + "; ".join(self.active_runtime_sessions)
            )
        super().__init__("SITL_PREFLIGHT_BLOCKED: " + " | ".join(reasons))


def parse_process_table(text: str) -> list[ProcessRow]:
    """Parse ``ps -eo pid=,comm=,%cpu=,args=`` output."""
    rows: list[ProcessRow] = []
    for raw_line in text.splitlines():
        fields = raw_line.strip().split(None, 3)
        if len(fields) != 4:
            continue
        try:
            pid = int(fields[0])
            cpu = float(fields[2])
        except ValueError:
            continue
        rows.append(ProcessRow(pid, fields[1], cpu, fields[3]))
    return rows


def competing_processes(
    text: str,
    *,
    excluded_pids: set[int] | frozenset[int] = frozenset(),
) -> list[ProcessRow]:
    """Return active build commands, excluding this runner's ancestors."""
    result: list[ProcessRow] = []
    for row in parse_process_table(text):
        if row.pid in excluded_pids:
            continue
        command_line = f"{row.command} {row.args}"
        if _COMPETING_PROCESS.search(command_line):
            result.append(row)
    return result


def check_preflight(
    *,
    loadavg_1m: float,
    max_loadavg_1m: float,
    process_rows: Iterable[ProcessRow],
    excluded_pids: set[int] | frozenset[int],
    active_runtime_sessions: Iterable[str],
) -> PreflightViolation | None:
    """Return a structured violation, or ``None`` when the host is idle."""
    filtered = tuple(row for row in process_rows if row.pid not in excluded_pids)
    sessions = tuple(active_runtime_sessions)
    if loadavg_1m <= max_loadavg_1m and not filtered and not sessions:
        return None
    return PreflightViolation(
        loadavg_1m=loadavg_1m,
        max_loadavg_1m=max_loadavg_1m,
        process_rows=filtered,
        active_runtime_sessions=sessions,
    )


def current_process_ancestry() -> set[int]:
    """Return this process and its Linux parent chain for self-exclusion."""
    result: set[int] = set()
    pid = os.getpid()
    while pid > 1 and pid not in result:
        result.add(pid)
        try:
            fields = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()
            pid = int(fields[3])
        except (OSError, ValueError, IndexError):
            break
    return result


def read_process_table() -> str:
    result = subprocess.run(
        ["ps", "-eo", "pid=,comm=,%cpu=,args=", "--sort=-%cpu"],
        capture_output=True,
        text=True,
        check=False,
        timeout=5.0,
    )
    if result.returncode != 0:
        raise RuntimeError(f"ps preflight failed with exit {result.returncode}")
    return result.stdout


def read_uptime_s() -> float | None:
    try:
        return float(Path("/proc/uptime").read_text(encoding="utf-8").split()[0])
    except (OSError, ValueError, IndexError):
        return None


def snapshot_host() -> dict[str, object]:
    """Capture bounded host evidence for session metadata."""
    try:
        loadavg = float(os.getloadavg()[0])
    except (OSError, IndexError):
        loadavg = None
    rows = parse_process_table(read_process_table())
    return {
        "captured_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "captured_wall_ns": time.time_ns(),
        "uptime_s": read_uptime_s(),
        "loadavg_1m": loadavg,
        "top_cpu_processes": [
            {
                "pid": row.pid,
                "command": row.command,
                "cpu_percent": row.cpu_percent,
                "args": row.args[:240],
            }
            for row in rows[:5]
        ],
    }


class HostTelemetryRecorder:
    """Append bounded host/RTF snapshots to the session metadata."""

    def __init__(self, metadata_path: Path, rtf_path: Path, interval_s: float = 10.0) -> None:
        if interval_s <= 0.0:
            raise ValueError("telemetry interval must be positive")
        self.metadata_path = metadata_path
        self.rtf_path = rtf_path
        self.interval_s = interval_s
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._write_lock = threading.Lock()

    def _sample(self) -> dict[str, object]:
        sample = snapshot_host()
        try:
            rtf = json.loads(self.rtf_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            rtf = None
        if isinstance(rtf, dict) and isinstance(rtf.get("real_time_factor"), (int, float)):
            sample["real_time_factor"] = float(rtf["real_time_factor"])
            sample["real_time_factor_source"] = str(self.rtf_path.name)
        else:
            sample["real_time_factor"] = None
            sample["real_time_factor_source"] = "not_observed"
        return sample

    def _write_sample(self) -> None:
        sample = self._sample()
        with self._write_lock:
            try:
                metadata = json.loads(self.metadata_path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                return
            telemetry = metadata.setdefault("host_telemetry", {})
            telemetry["interval_s"] = self.interval_s
            samples = telemetry.setdefault("samples", [])
            if not isinstance(samples, list):
                samples = []
                telemetry["samples"] = samples
            samples.append(sample)
            rtf_values = telemetry.setdefault("rtf_gazebo", [])
            if sample.get("real_time_factor") is not None:
                rtf_values.append({
                    "captured_wall_ns": sample["captured_wall_ns"],
                    "real_time_factor": sample["real_time_factor"],
                })
            temporary = self.metadata_path.with_suffix(".telemetry.tmp")
            temporary.write_text(
                json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            os.replace(temporary, self.metadata_path)

    def _run(self) -> None:
        while not self._stop.wait(self.interval_s):
            self._write_sample()

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("host telemetry recorder already started")
        self._write_sample()
        self._thread = threading.Thread(
            target=self._run, name="sitl-host-telemetry", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self.interval_s + 1.0))
            self._thread = None
        self._write_sample()
