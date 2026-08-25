"""Canonical Python and build/runtime lock contract.

The ROS 2 Jazzy installation in this workspace is built for the system
Python.  A virtualenv can still be useful for unrelated tooling, but it must
never be allowed to select the interpreter or installed overlay used by the
product build/SITL workflow.
"""

from __future__ import annotations

import errno
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any


CANONICAL_PYTHON = Path("/usr/bin/python3")


def canonical_python_error(
    executable: str | os.PathLike[str] | None = None,
    environment: dict[str, str] | None = None,
) -> str | None:
    """Return a deterministic error when a non-ROS Python is selected."""
    env = os.environ if environment is None else environment
    selected = Path(executable or sys.executable).resolve()
    expected = CANONICAL_PYTHON.resolve()
    if selected != expected:
        return (
            "canonical ROS workflow requires /usr/bin/python3; "
            f"selected {selected}. Deactivate the virtualenv and retry."
        )
    if env.get("VIRTUAL_ENV"):
        return (
            "canonical ROS workflow does not run inside VIRTUAL_ENV="
            f"{env['VIRTUAL_ENV']}; deactivate it and retry"
        )
    if env.get("PYTHONHOME"):
        return (
            "canonical ROS workflow does not accept PYTHONHOME="
            f"{env['PYTHONHOME']}; unset it and retry"
        )
    return None


def require_canonical_python(
    executable: str | os.PathLike[str] | None = None,
    environment: dict[str, str] | None = None,
) -> None:
    error = canonical_python_error(executable, environment)
    if error:
        raise RuntimeError(error)


def shared_artifact_root(root: Path) -> Path:
    override = os.environ.get("UAV_NAV_ARTIFACT_ROOT")
    if override:
        return Path(override).expanduser().resolve()
    try:
        result = subprocess.run(
            [
                "git",
                "-C",
                str(root),
                "rev-parse",
                "--path-format=absolute",
                "--git-common-dir",
            ],
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )
        common_dir = Path(result.stdout.strip()).resolve()
        if result.returncode == 0 and common_dir.name == ".git":
            return common_dir.parent / ".artifacts/runtime"
    except (OSError, subprocess.SubprocessError):
        pass
    return root / ".artifacts/runtime"


class BuildRuntimeBusyError(RuntimeError):
    """Raised when a build and a runtime session would overlap."""


class BuildRuntimeLock:
    """Shared/exclusive lock for the single canonical Release install.

    Runtime sessions take a shared lock; every build/test/check takes an
    exclusive lock.  This prevents a package-select build from replacing a
    library while an already-started session is loading the install overlay.
    """

    def __init__(self, root: Path, *, exclusive: bool) -> None:
        self.path = shared_artifact_root(root) / ".build-runtime.lock"
        self.exclusive = exclusive
        self._file: Any | None = None

    def _owner(self) -> str:
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return "owner metadata unavailable"
        if not isinstance(payload, dict):
            return "owner metadata unavailable"
        return f"pid={payload.get('pid', 'unknown')}, mode={payload.get('mode', 'unknown')}"

    def __enter__(self) -> "BuildRuntimeLock":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a+", encoding="utf-8")
        operation = fcntl.LOCK_EX if self.exclusive else fcntl.LOCK_SH
        try:
            fcntl.flock(self._file.fileno(), operation | fcntl.LOCK_NB)
        except OSError as error:
            self._file.close()
            self._file = None
            if error.errno in {errno.EACCES, errno.EAGAIN}:
                mode = "build" if self.exclusive else "runtime"
                raise BuildRuntimeBusyError(
                    f"cannot start {mode}: canonical build/runtime lock is held "
                    f"({self._owner()}); finish the other operation first"
                ) from error
            raise
        payload = {
            "pid": os.getpid(),
            "mode": "exclusive-build" if self.exclusive else "shared-runtime",
            "python": str(Path(sys.executable).resolve()),
        }
        self._file.seek(0)
        self._file.truncate()
        self._file.write(json.dumps(payload, sort_keys=True) + "\n")
        self._file.flush()
        os.fsync(self._file.fileno())
        return self

    def __exit__(self, _exception_type: Any, _exception: Any, _traceback: Any) -> None:
        if self._file is None:
            return
        try:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
        finally:
            self._file.close()
            self._file = None
