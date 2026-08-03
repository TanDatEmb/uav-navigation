#!/usr/bin/env python3
"""Sample one process and its children until the leader exits."""

import argparse
import csv
import os
import time
from pathlib import Path


def descendants(root: int) -> set[int]:
    result = {root}
    changed = True
    while changed:
        changed = False
        for entry in Path("/proc").glob("[0-9]*"):
            try:
                pid = int(entry.name)
                status = (entry / "status").read_text(encoding="utf-8")
                parent = next(int(line.split()[1]) for line in status.splitlines()
                              if line.startswith("PPid:"))
            except (OSError, StopIteration, ValueError):
                continue
            if parent in result and pid not in result:
                result.add(pid)
                changed = True
    return result


def sample(pid: int) -> tuple[float, int]:
    total_cpu = 0.0
    total_rss_kib = 0
    for child in descendants(pid):
        try:
            stat = (Path("/proc") / str(child) / "stat").read_text(encoding="utf-8")
            fields = stat.rsplit(") ", 1)[1].split()
            utime = int(fields[11])
            stime = int(fields[12])
            total_cpu += (utime + stime) / os.sysconf(os.sysconf_names["SC_CLK_TCK"])
            status = (Path("/proc") / str(child) / "status").read_text(encoding="utf-8")
            total_rss_kib += next(int(line.split()[1]) for line in status.splitlines()
                                  if line.startswith("VmRSS:"))
        except (OSError, StopIteration, ValueError, IndexError):
            continue
    return total_cpu, total_rss_kib


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pid", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["elapsed_s", "cpu_seconds_children", "rss_kib_children"])
        while os.path.exists(f"/proc/{args.pid}"):
            cpu, rss = sample(args.pid)
            writer.writerow([f"{time.monotonic() - start:.3f}", f"{cpu:.6f}", rss])
            stream.flush()
            time.sleep(max(0.1, args.interval))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
