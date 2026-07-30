#!/usr/bin/env python3
"""Clean registered ROS replay process groups left by interrupted runs."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = ROOT / ".artifacts" / "datasets"


def proc_stat(pid: int) -> tuple[str, int, int] | None:
    try:
        value = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    except (FileNotFoundError, ProcessLookupError):
        return None
    closing_parenthesis = value.rfind(")")
    if closing_parenthesis < 0:
        return None
    fields = value[closing_parenthesis + 2:].split()
    return fields[0], int(fields[2]), int(fields[19])


def process_group_members(process_group: int) -> list[int]:
    members = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        stat = proc_stat(int(entry.name))
        if stat is not None and stat[0] != "Z" and stat[1] == process_group:
            members.append(int(entry.name))
    return members


def command_line(pid: int) -> str:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        return ""
    return raw.replace(b"\0", b" ").decode(errors="replace").strip()


def registered_group_is_current(entry: dict[str, Any]) -> bool:
    pid = int(entry["pid"])
    process_group = int(entry["process_group"])
    members = process_group_members(process_group)
    if not members:
        return False
    leader_stat = proc_stat(pid)
    if leader_stat is not None:
        return (
            leader_stat[0] != "Z"
            and leader_stat[1] == process_group
            and leader_stat[2] == int(entry["start_ticks"])
        )
    expected = [str(part) for part in entry.get("command", [])]
    identifying_parts = [
        part for part in expected
        if str(ROOT) in part or str(ARTIFACTS) in part
    ]
    if not identifying_parts:
        return False
    member_commands = [command_line(member) for member in members]
    return any(
        any(part in command for part in identifying_parts)
        for command in member_commands
    )


def group_exists(process_group: int) -> bool:
    return bool(process_group_members(process_group))


def wait_for_group(process_group: int, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not group_exists(process_group):
            return True
        time.sleep(0.05)
    return not group_exists(process_group)


def stop_group(process_group: int) -> None:
    for stop_signal, timeout in (
        (signal.SIGINT, 3.0),
        (signal.SIGTERM, 2.0),
        (signal.SIGKILL, 1.0),
    ):
        if not group_exists(process_group):
            return
        try:
            os.killpg(process_group, stop_signal)
        except ProcessLookupError:
            return
        if wait_for_group(process_group, timeout):
            return
    if group_exists(process_group):
        raise RuntimeError(f"process group {process_group} survived cleanup")


def registry_paths() -> list[Path]:
    if not ARTIFACTS.is_dir():
        return []
    return sorted(ARTIFACTS.glob("*/*-replay-*/*process_groups.json"))


def cleanup(dry_run: bool = False) -> int:
    groups: dict[int, tuple[Path, dict[str, Any]]] = {}
    for registry in registry_paths():
        payload = json.loads(registry.read_text(encoding="utf-8"))
        for entry in payload.get("processes", []):
            if registered_group_is_current(entry):
                groups[int(entry["process_group"])] = (registry, entry)
    if not groups:
        print("no registered replay process groups are running")
        return 0
    for process_group, (registry, entry) in groups.items():
        role = entry.get("role", "unknown")
        print(
            f"{'would stop' if dry_run else 'stopping'} "
            f"pgid={process_group} role={role} registry={registry.parent}"
        )
        if not dry_run:
            stop_group(process_group)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dry-run", action="store_true",
        help="list owned replay process groups without signaling them",
    )
    args = parser.parse_args()
    return cleanup(args.dry_run)


if __name__ == "__main__":
    sys.exit(main())
