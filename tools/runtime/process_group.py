#!/usr/bin/env python3
"""Own and stop runtime processes without affecting unrelated processes."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any


def _start_ticks(pid: int) -> int | None:
    try:
        text = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    except OSError:
        return None
    closing = text.rfind(")")
    if closing < 0:
        return None
    fields = text[closing + 2 :].split()
    return int(fields[19]) if len(fields) > 19 else None


def _group_exists(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
    except (ProcessLookupError, PermissionError):
        return False
    # Linux keeps zombie members in a process group until its parent reaps
    # them.  They no longer represent live work and must not make cleanup
    # report a false failure.
    observed = False
    for entry in Path("/proc").glob("[0-9]*"):
        try:
            text = (entry / "stat").read_text(encoding="utf-8")
        except OSError:
            continue
        closing = text.rfind(")")
        if closing < 0:
            continue
        fields = text[closing + 2 :].split()
        if len(fields) <= 3:
            continue
        try:
            member_pgid = int(fields[3])
        except ValueError:
            continue
        if member_pgid == pgid:
            observed = True
            if fields[0] != "Z":
                return True
    if observed:
        return False
    # The process group can disappear between killpg(0) and the /proc scan.
    # No observed member is therefore a cleanly empty group, not a cleanup
    # failure.
    return False


def _atomic_json(path: Path, payload: Any) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


class Session:
    """A runtime session with process-group ownership and durable state."""

    def __init__(self, directory: Path) -> None:
        self.directory = directory.resolve()
        self.logs = self.directory / "logs"
        self.registry_path = self.directory / "processes.json"
        self.state_path = self.directory / "state.json"
        self.logs.mkdir(parents=True, exist_ok=True)
        self.directory.mkdir(parents=True, exist_ok=True)
        if not self.registry_path.exists():
            _atomic_json(self.registry_path, {"schema_version": 1, "processes": []})

    @classmethod
    def create(cls, root: Path, workflow: str) -> "Session":
        root = root.resolve()
        if root == Path("/") or root.name not in {"runtime", "artifacts", ".artifacts"}:
            raise ValueError(f"unsafe runtime artifact root: {root}")
        root.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
        directory = root / f"{workflow}-{stamp}-{os.getpid()}"
        suffix = 1
        while directory.exists():
            directory = root / f"{workflow}-{stamp}-{os.getpid()}-{suffix}"
            suffix += 1
        session = cls(directory)
        update_latest(root, directory)
        session.write_state({
            "workflow": workflow,
            "status": "STARTING",
            "created_at": time.time(),
            "owner_pid": os.getpid(),
        })
        return session

    @classmethod
    def from_path(cls, path: Path) -> "Session":
        resolved = path.resolve()
        if not resolved.is_dir():
            raise FileNotFoundError(f"runtime session does not exist: {resolved}")
        return cls(resolved)

    def write_state(self, values: dict[str, Any], **extra: Any) -> None:
        current: dict[str, Any] = {}
        if self.state_path.exists():
            try:
                current = json.loads(self.state_path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                current = {}
        current.update(values)
        current.update(extra)
        current["updated_at"] = time.time()
        _atomic_json(self.state_path, current)

    def state(self) -> dict[str, Any]:
        try:
            return json.loads(self.state_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}

    def _registry(self) -> dict[str, Any]:
        try:
            return json.loads(self.registry_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {"schema_version": 1, "processes": []}

    def start(
        self,
        role: str,
        command: list[str],
        *,
        cwd: Path,
        env: dict[str, str] | None = None,
        env_remove: set[str] | None = None,
    ) -> subprocess.Popen[Any]:
        log_path = self.logs / f"{role}.log"
        log = log_path.open("a", encoding="utf-8")
        child_env = os.environ.copy()
        # Python otherwise block-buffers stdout when it is redirected to a
        # session log, making a healthy scenario look silent until it exits.
        child_env.setdefault("PYTHONUNBUFFERED", "1")
        if env_remove:
            for key in env_remove:
                child_env.pop(key, None)
        if env:
            child_env.update(env)
        try:
            process = subprocess.Popen(
                command,
                cwd=str(cwd),
                env=child_env,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except BaseException:
            log.close()
            raise
        record = {
            "role": role,
            "pid": process.pid,
            "pgid": os.getpgid(process.pid),
            "start_ticks": _start_ticks(process.pid),
            "command": command,
            "log": str(log_path),
            "started_at": time.time(),
        }
        registry = self._registry()
        registry.setdefault("processes", []).append(record)
        _atomic_json(self.registry_path, registry)
        self.write_state({"last_started_role": role})
        # The file descriptor is inherited by the child; close the parent copy.
        log.close()
        return process

    def records(self) -> list[dict[str, Any]]:
        return list(self._registry().get("processes", []))

    def live_records(self) -> list[dict[str, Any]]:
        return [record for record in self.records() if _group_exists(int(record["pgid"]))]

    def stop(self, grace_s: float = 4.0) -> list[str]:
        """Stop only registered process groups and return cleanup failures."""
        failures: list[str] = []
        records = list(reversed(self.records()))
        monitors = [record for record in records if record.get("role") == "monitor"]
        records = monitors + [record for record in records if record.get("role") != "monitor"]
        for record in records:
            try:
                pgid = int(record["pgid"])
                pid = int(record["pid"])
            except (KeyError, TypeError, ValueError):
                failures.append(f"invalid process record: {record!r}")
                continue
            if not _group_exists(pgid):
                continue
            # A reused PID must never authorize killing a new process group.
            expected_ticks = record.get("start_ticks")
            actual_ticks = _start_ticks(pid)
            if expected_ticks is not None and actual_ticks not in {expected_ticks, None}:
                failures.append(f"process identity changed for {record.get('role', pgid)}")
                continue
            try:
                os.killpg(pgid, signal.SIGINT)
            except ProcessLookupError:
                continue
            deadline = time.monotonic() + grace_s
            while time.monotonic() < deadline and _group_exists(pgid):
                time.sleep(0.05)
            if _group_exists(pgid):
                try:
                    os.killpg(pgid, signal.SIGTERM)
                except ProcessLookupError:
                    continue
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline and _group_exists(pgid):
                    time.sleep(0.05)
            if _group_exists(pgid):
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    continue
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline and _group_exists(pgid):
                    time.sleep(0.05)
            if _group_exists(pgid):
                failures.append(f"process group remains: {record.get('role', pgid)} ({pgid})")
        if failures:
            self.write_state({"cleanup": "FAIL", "cleanup_failures": failures})
        else:
            self.write_state({"cleanup": "PASS", "cleanup_failures": []})
        return failures

    def mark_stopped(self, reason: str = "requested") -> None:
        self.write_state({"status": "STOPPED", "stop_reason": reason, "stopped": True})


def resolve_latest(root: Path) -> Path:
    latest = (root / "latest").resolve()
    if not latest.is_dir():
        raise FileNotFoundError(f"no runtime session under {root}")
    return latest


def update_latest(root: Path, directory: Path) -> None:
    """Atomically point ``latest`` at an existing session directory."""
    root = root.resolve()
    directory = directory.resolve()
    if directory.parent != root or not directory.is_dir():
        raise ValueError(f"latest target is not a runtime session under {root}: {directory}")
    latest = root / "latest"
    temporary = root / f".latest-{os.getpid()}.tmp"
    temporary.unlink(missing_ok=True)
    temporary.symlink_to(directory.name)
    temporary.replace(latest)


__all__ = ["Session", "resolve_latest", "update_latest"]
