#!/usr/bin/env python3
"""Sample one process tree until the leader exits.

CPU percent follows psutil's process convention: 100% is one logical CPU.
Process handles are cached by PID and creation time; missing/dead members are
reported as non-measured instead of being converted into valid zero load.
"""

import argparse
import csv
import os
import time
from pathlib import Path

import psutil


class CachedProcessSampler:
    def __init__(self, process: psutil.Process) -> None:
        self.root = process
        self.cache: dict[int, psutil.Process] = {process.pid: process}
        self.identities: dict[int, float] = {}
        self.primed: set[int] = set()

    def _members(self) -> list[psutil.Process]:
        discovered = [self.root]
        try:
            discovered.extend(self.root.children(recursive=True))
        except psutil.Error:
            pass
        members: list[psutil.Process] = []
        for candidate in discovered:
            try:
                identity = float(candidate.create_time())
                if self.identities.get(candidate.pid) != identity:
                    self.cache[candidate.pid] = candidate
                    self.identities[candidate.pid] = identity
                    self.primed.discard(candidate.pid)
                members.append(self.cache[candidate.pid])
            except psutil.Error:
                continue
        return members

    def prime(self) -> None:
        for process in self._members():
            try:
                process.cpu_percent(None)
                self.primed.add(process.pid)
            except psutil.Error:
                pass

    def sample(self) -> dict[str, float | int | str | None]:
        processes = self._members()
        cpu = rss = vms = threads = voluntary = involuntary = 0
        cpu_times = 0.0
        live = 0
        failed = 0
        for item in processes:
            try:
                if not item.is_running() or item.status() == psutil.STATUS_ZOMBIE:
                    failed += 1
                    continue
                if item.pid not in self.primed:
                    item.cpu_percent(None)
                    self.primed.add(item.pid)
                    failed += 1
                    continue
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
                failed += 1
        return {
            "sample_state": "measured" if live and not failed else ("partial" if live else "dead"),
            "process_count": live if live else None,
            "cpu_percent_one_logical_core": cpu if live else None,
            "rss_bytes": rss if live else None,
            "vms_bytes": vms if live else None,
            "threads": threads if live else None,
            "voluntary_context_switches": voluntary if live else None,
            "involuntary_context_switches": involuntary if live else None,
            "cpu_seconds": cpu_times if live else None,
        }


def sample(process: psutil.Process) -> dict[str, float | int | str | None]:
    """Compatibility helper for one isolated sample."""
    sampler = CachedProcessSampler(process)
    sampler.prime()
    return sampler.sample()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pid", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    process = psutil.Process(args.pid)
    sampler = CachedProcessSampler(process)
    sampler.prime()
    start = time.monotonic()
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "elapsed_s", "sample_state", "process_count", "cpu_percent_one_logical_core",
            "rss_bytes", "vms_bytes", "threads",
            "voluntary_context_switches", "involuntary_context_switches",
            "cpu_seconds",
        ])
        while os.path.exists(f"/proc/{args.pid}"):
            time.sleep(max(0.1, args.interval))
            try:
                values = sampler.sample()
            except psutil.NoSuchProcess:
                break
            writer.writerow([
                f"{time.monotonic() - start:.3f}",
                values["sample_state"],
                values["process_count"],
                values["cpu_percent_one_logical_core"],
                values["rss_bytes"], values["vms_bytes"], values["threads"],
                values["voluntary_context_switches"],
                values["involuntary_context_switches"],
                values["cpu_seconds"],
            ])
            stream.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
