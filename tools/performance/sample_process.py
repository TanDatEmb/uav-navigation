#!/usr/bin/env python3
"""Sample one process tree until the leader exits.

CPU percent follows psutil's process convention: 100% is one logical CPU.
"""

import argparse
import csv
import os
import time
from pathlib import Path

import psutil


def sample(process: psutil.Process) -> dict[str, float | int]:
    processes = [process]
    try:
        processes.extend(process.children(recursive=True))
    except psutil.Error:
        pass
    cpu = rss = vms = threads = voluntary = involuntary = 0
    cpu_times = 0.0
    live = 0
    for item in processes:
        try:
            cpu += item.cpu_percent(interval=None)
            memory = item.memory_info()
            rss += memory.rss
            vms += memory.vms
            threads += item.num_threads()
            context = item.num_ctx_switches()
            voluntary += context.voluntary
            involuntary += context.involuntary
            times = item.cpu_times()
            cpu_times += times.user + times.system
            live += 1
        except psutil.Error:
            continue
    return {
        "process_count": live,
        "cpu_percent_one_logical_core": cpu,
        "rss_bytes": rss,
        "vms_bytes": vms,
        "threads": threads,
        "voluntary_context_switches": voluntary,
        "involuntary_context_switches": involuntary,
        "cpu_seconds": cpu_times,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pid", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    process = psutil.Process(args.pid)
    sample(process)  # prime psutil's interval-based CPU measurement
    start = time.monotonic()
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "elapsed_s", "process_count", "cpu_percent_one_logical_core",
            "rss_bytes", "vms_bytes", "threads",
            "voluntary_context_switches", "involuntary_context_switches",
            "cpu_seconds",
        ])
        while os.path.exists(f"/proc/{args.pid}"):
            time.sleep(max(0.1, args.interval))
            try:
                values = sample(process)
            except psutil.NoSuchProcess:
                break
            writer.writerow([
                f"{time.monotonic() - start:.3f}",
                values["process_count"],
                f"{values['cpu_percent_one_logical_core']:.3f}",
                values["rss_bytes"], values["vms_bytes"], values["threads"],
                values["voluntary_context_switches"],
                values["involuntary_context_switches"],
                f"{values['cpu_seconds']:.6f}",
            ])
            stream.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
