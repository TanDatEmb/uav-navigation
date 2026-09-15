"""Fail-closed identity contract between a product build and runtime evidence."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
from typing import Any, Iterable


SCHEMA_VERSION = 1
MANIFEST_NAME = ".uav_navigation_build_manifest.json"
CRITICAL_ARTIFACTS = (
    "fast_lio_ros/lib/fast_lio_ros/fast_lio_node",
    "navigation_runtime/lib/navigation_runtime/navigation_runtime_node",
    "navigation_runtime/lib/libnavigation_runtime_core.a",
    "navigation_planning_backend/lib/libnavigation_planning_backend.a",
    "px4_navigation_external_mode/lib/px4_navigation_external_mode/px4_navigation_external_mode_node",
    "px4_navigation_external_mode/lib/libpx4_navigation_external_mode_contract.a",
    "px4_odometry_bridge/lib/px4_odometry_bridge/px4_odometry_bridge_node",
    "px4_odometry_bridge/lib/px4_odometry_bridge/px4_odometry_bridge_external_node",
)
RUNTIME_SCRIPTS = (
    "tools/runtime/runtime_environment.py",
    "tools/runtime/runner.py",
    "tools/runtime/monitor.py",
    "tools/runtime/report.py",
    "tools/runtime/external_mode_scenario.py",
)
PRODUCT_RUNTIME_PREFIXES = (
    "livox_ros_driver2",
    "navigation_common",
    "navigation_contracts",
    "navigation_execution",
    "px4_msgs",
    "px4_ros2_cpp",
    "fast_lio_ros",
    "navigation_runtime",
    "navigation_planning",
    "navigation_planning_backend",
    "px4_navigation_external_mode",
    "px4_odometry_bridge",
    "navigation_bringup",
)


def _git(root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(root), *args], text=True, capture_output=True, check=False
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "git command failed")
    return result.stdout.strip()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def source_fingerprint(root: Path) -> dict[str, Any]:
    """Hash every tracked and untracked, non-ignored workspace input.

    Paths and contents are both covered. Gitignored build/artifact outputs are
    intentionally excluded, while an untracked product source is authoritative.
    """
    status = _git(root, "submodule", "status")
    submodule_lines = [line for line in status.splitlines() if line.strip()]
    submodule_paths = {
        fields[1]
        for line in submodule_lines
        if len(fields := line.lstrip(" +-U").split()) >= 2
    }
    raw = subprocess.run(
        ["git", "-C", str(root), "ls-files", "-z", "--cached", "--others", "--exclude-standard"],
        capture_output=True,
        check=True,
    ).stdout
    paths = sorted(item.decode("utf-8", "surrogateescape") for item in raw.split(b"\0") if item)
    digest = hashlib.sha256()
    file_count = 0
    nested_repositories: list[dict[str, Any]] = []
    for relative in paths:
        path = root / relative
        normalized = relative.rstrip("/")
        if normalized in submodule_paths:
            continue
        if not path.is_file():
            # Git reports an untracked nested repository as one directory
            # entry instead of enumerating its files.  Hash it recursively so
            # changing that dependency cannot preserve the parent identity.
            if path.is_dir() and not path.is_symlink():
                try:
                    nested_root = Path(
                        _git(path, "rev-parse", "--show-toplevel")
                    ).resolve()
                except RuntimeError:
                    continue
                if nested_root != path.resolve():
                    continue
                record = {
                    "path": normalized,
                    "source": source_fingerprint(path),
                }
                nested_repositories.append(record)
                digest.update(json.dumps(record, sort_keys=True).encode("utf-8"))
            continue
        encoded = relative.encode("utf-8", "surrogateescape")
        digest.update(len(encoded).to_bytes(8, "big"))
        digest.update(encoded)
        digest.update(bytes.fromhex(sha256_file(path)))
        file_count += 1
    submodules: list[dict[str, Any]] = []
    for line in submodule_lines:
        fields = line.lstrip(" +-U").split()
        if len(fields) < 2:
            continue
        relative = fields[1]
        subroot = root / relative
        sub_source = source_fingerprint(subroot)
        record = {
            "path": relative,
            **sub_source,
        }
        submodules.append(record)
        digest.update(json.dumps(record, sort_keys=True).encode("utf-8"))
    return {
        "sha256": digest.hexdigest(),
        "file_count": file_count,
        "git_head": _git(root, "rev-parse", "HEAD"),
        "git_dirty": bool(_git(root, "status", "--porcelain")),
        "submodules": submodules,
        "nested_repositories": nested_repositories,
    }


def _artifact_record(path: Path, root: Path) -> dict[str, Any]:
    resolved = path.resolve(strict=True)
    return {
        "path": str(path.relative_to(root)),
        "resolved_path": str(resolved),
        "size_bytes": resolved.stat().st_size,
        "mtime_ns": resolved.stat().st_mtime_ns,
        "sha256": sha256_file(resolved),
    }


def runtime_artifact_paths(root: Path, install: Path) -> list[Path]:
    """Resolve the launch-critical binaries and workspace runtime libraries.

    ROS message ABI and px4_ros2_cpp are dynamically loaded by the executable,
    so hashing only the top-level ELF would still permit a stale typesupport
    library. Include every installed product shared/static library, executable
    under a package lib directory, and installed Python launch/runtime script.
    """
    paths = {install / relative for relative in CRITICAL_ARTIFACTS}
    paths.update(root / relative for relative in RUNTIME_SCRIPTS)
    for package in PRODUCT_RUNTIME_PREFIXES:
        prefix = install / package
        if not prefix.exists():
            raise RuntimeError(f"product install prefix is missing: {prefix}")
        for path in prefix.rglob("*"):
            if not path.is_file():
                continue
            relative_parts = path.relative_to(prefix).parts
            installed_executable = len(relative_parts) >= 2 and relative_parts[0] == "lib" and os.access(path, os.X_OK)
            if path.suffix in {".so", ".a", ".py"} or installed_executable:
                paths.add(path)
    return sorted(paths, key=lambda value: str(value))


def create_manifest(
    root: Path,
    install: Path,
    *,
    mode: str,
    authoritative: bool,
    command: Iterable[str],
    build_started_wall_ns: int,
    source_before: dict[str, Any],
) -> dict[str, Any]:
    source_after = source_fingerprint(root)
    if source_after != source_before:
        raise RuntimeError("workspace source changed while the product build was running")
    artifacts = [
        _artifact_record(path, root) for path in runtime_artifact_paths(root, install)
    ]
    return {
        "schema_version": SCHEMA_VERSION,
        "authoritative": authoritative,
        "build_mode": mode,
        "build_command": list(command),
        "build_started_wall_ns": build_started_wall_ns,
        "build_finished_wall_ns": time.time_ns(),
        "source": source_after,
        "artifacts": artifacts,
    }


def write_manifest_atomic(install: Path, manifest: dict[str, Any]) -> Path:
    path = install / MANIFEST_NAME
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)
    return path


def validate_manifest(root: Path, install: Path) -> dict[str, Any]:
    path = install / MANIFEST_NAME
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise RuntimeError(f"authoritative Release build manifest unavailable: {path}: {error}") from error
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise RuntimeError("unsupported build manifest schema")
    if manifest.get("authoritative") is not True or manifest.get("build_mode") != "release":
        raise RuntimeError("runtime requires an authoritative full Release product build")
    current_source = source_fingerprint(root)
    if manifest.get("source") != current_source:
        raise RuntimeError("workspace source fingerprint differs from the installed product build")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, list) or not artifacts:
        raise RuntimeError("build manifest has no artifact identities")
    recorded_paths = {
        expected.get("path")
        for expected in artifacts
        if isinstance(expected, dict) and isinstance(expected.get("path"), str)
    }
    discovered_paths = {
        str(path.relative_to(root)) for path in runtime_artifact_paths(root, install)
    }
    if recorded_paths != discovered_paths:
        missing = sorted(discovered_paths - recorded_paths)
        stale = sorted(recorded_paths - discovered_paths)
        details: list[str] = []
        if missing:
            details.append("missing from manifest: " + ", ".join(missing[:5]))
        if stale:
            details.append("no longer discovered: " + ", ".join(stale[:5]))
        raise RuntimeError("build manifest artifact set mismatch (" + "; ".join(details) + ")")
    for expected in artifacts:
        if not isinstance(expected, dict) or not isinstance(expected.get("path"), str):
            raise RuntimeError("build manifest contains a malformed artifact record")
        actual = _artifact_record(root / expected["path"], root)
        for field in ("resolved_path", "size_bytes", "sha256"):
            if actual[field] != expected.get(field):
                raise RuntimeError(f"installed artifact identity mismatch: {expected['path']} ({field})")
    return {
        "status": "VALID",
        "validated_wall_ns": time.time_ns(),
        "manifest_path": str(path),
        "manifest_sha256": sha256_file(path),
        "manifest": manifest,
    }
