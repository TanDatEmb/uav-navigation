#!/usr/bin/env python3
"""Low-overhead Linux /proc process and host sampler."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except (OSError, PermissionError):
        return ""


def process_metrics(pid: int) -> dict[str, Any]:
    root = Path("/proc") / str(pid)
    stat = _read(root / "stat").split()
    status = {}
    for line in _read(root / "status").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            status[key] = value.strip()
    io = {}
    for line in _read(root / "io").splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            io[key] = int(value.strip())
    if len(stat) < 24:
        return {"pid": pid, "alive": False}
    page_kib = os.sysconf("SC_PAGE_SIZE") / 1024
    return {
        "pid": pid, "ppid": int(stat[3]), "pgid": int(stat[4]), "alive": True,
        "utime_ticks": int(stat[13]), "stime_ticks": int(stat[14]),
        "minor_faults": int(stat[9]), "major_faults": int(stat[11]),
        "vsz_bytes": int(stat[22]), "rss_kib": int(stat[23]) * page_kib,
        "thread_count": int(status.get("Threads", "0")),
        "voluntary_context_switches": int(status.get("voluntary_ctxt_switches", "0")),
        "involuntary_context_switches": int(status.get("nonvoluntary_ctxt_switches", "0")),
        "read_bytes": io.get("read_bytes", 0), "write_bytes": io.get("write_bytes", 0),
        "name": status.get("Name", ""), "command": _read(root / "cmdline").replace("\0", " ").strip(),
    }


def host_metrics() -> dict[str, Any]:
    mem = {}
    for line in _read(Path("/proc/meminfo")).splitlines():
        key, value = line.split(":", 1)
        mem[key] = int(value.strip().split()[0])
    cpu_line = _read(Path("/proc/stat")).splitlines()
    cpu = [int(value) for value in cpu_line[0].split()[1:]] if cpu_line else []
    load = _read(Path("/proc/loadavg")).split()
    return {
        "load_1": float(load[0]) if load else 0.0,
        "load_5": float(load[1]) if load else 0.0,
        "load_15": float(load[2]) if load else 0.0,
        "memory_available_kib": mem.get("MemAvailable", 0),
        "memory_total_kib": mem.get("MemTotal", 0),
        "swap_total_kib": mem.get("SwapTotal", 0),
        "swap_free_kib": mem.get("SwapFree", 0),
        "swap_used_kib": mem.get("SwapTotal", 0)-mem.get("SwapFree", 0),
        "cpu_user_ticks": sum(cpu[0:2]), "cpu_system_ticks": sum(cpu[2:3]),
        "cpu_idle_ticks": sum(cpu[3:5]), "cpu_iowait_ticks": cpu[4] if len(cpu) > 4 else 0,
        "cpu_total_ticks": sum(cpu),
    }


def cpu_percent(current: dict[str, Any], previous: dict[str, Any]) -> float:
    total = current.get("cpu_total_ticks", 0)-previous.get("cpu_total_ticks", 0)
    idle = current.get("cpu_idle_ticks", 0)-previous.get("cpu_idle_ticks", 0)
    return 0.0 if total <= 0 else 100.0 * max(0, total-idle) / total
