#!/usr/bin/env python3
"""Best-effort automatic diagnostic snapshot collector."""
from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import Any


def _capture(path: Path, command: list[str], timeout: float = 5.0) -> None:
    try:
        result = subprocess.run(command, capture_output=True, text=True,
                                timeout=timeout, check=False)
        path.write_text(result.stdout + result.stderr, encoding="utf-8")
    except Exception as error:
        path.write_text(f"capture_failed: {type(error).__name__}: {error}\n", encoding="utf-8")


def _tail(source: Path, destination: Path, lines: int) -> None:
    if not source.exists():
        destination.write_text("source log unavailable\n", encoding="utf-8")
        return
    destination.write_text("\n".join(
        source.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:])+"\n",
        encoding="utf-8")


def collect_snapshot(session: Path, event: dict[str, Any], auto_gdb: bool = False) -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    code = event["code"].replace("/", "_")
    target = session / "snapshots" / f"{stamp}-{code}"
    suffix = 1
    while target.exists():
        target = session / "snapshots" / f"{stamp}-{code}-{suffix}"
        suffix += 1
    target.mkdir(parents=True)
    (target/"event.json").write_text(json.dumps(event, indent=2)+"\n", encoding="utf-8")
    commands = {
        "ros_node_list.txt": ["ros2", "node", "list"],
        "ros_topic_list.txt": ["ros2", "topic", "list", "-t"],
        "ros_node_info_fast_lio.txt": ["ros2", "node", "info", "/fast_lio"],
        "ros_node_info_bridge.txt": ["ros2", "node", "info", "/px4_mid360_bridge"],
        "process_tree.txt": ["ps", "-e", "-o", "pid,ppid,pgid,%cpu,rss,nlwp,etime,args", "--forest"],
        "top_snapshot.txt": ["top", "-b", "-n", "1"],
        "memory_snapshot.txt": ["free", "-h"],
        "swap_snapshot.txt": ["swapon", "--show"],
        "disk_snapshot.txt": ["df", "-h", str(session)],
        "gazebo_topics.txt": ["gz", "topic", "-l"],
    }
    for filename, command in commands.items():
        _capture(target/filename, command)
    for source_name, output_name, count in [
        ("fast_lio.log", "last_500_fast_lio_log_lines.txt", 500),
        ("bridge.log", "last_200_bridge_log_lines.txt", 200),
        ("gazebo.log", "last_200_gazebo_log_lines.txt", 200),
    ]:
        _tail(session/"logs"/source_name, target/output_name, count)
    latest = session/"latest"
    for name in ("diagnostics.json", "stream_state.json", "pointcloud_state.json"):
        if (latest/name).exists():
            shutil.copy2(latest/name, target/("latest_"+name if name == "diagnostics.json" else name))
    status = {"gdb_snapshot_status": "disabled"}
    if auto_gdb:
        status["gdb_snapshot_status"] = "target_unavailable"
        for pidfile in (session/"pids").glob("*fast_lio*.pid"):
            pid = int(pidfile.read_text().strip())
            if os.path.exists(f"/proc/{pid}"):
                _capture(target/"gdb_backtrace.txt",
                         ["gdb", "-batch", "-p", str(pid), "-ex", "thread apply all bt"], 10)
                status["gdb_snapshot_status"] = "attempted"
                break
    (target/"snapshot_status.json").write_text(json.dumps(status, indent=2)+"\n")
    return target
