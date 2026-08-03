#!/usr/bin/env python3
"""Small, fail-closed provenance helpers for the P0.8 qualification."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
from typing import Any


PX4_MSGS = Path("src/external/px4_msgs")


def command(
    args: list[str], *, cwd: Path | None = None, check: bool = False
) -> str:
    result = subprocess.run(
        args, cwd=cwd, check=check, capture_output=True, text=True
    )
    return result.stdout.strip()


def sha256(path: Path) -> str:
    if not path.is_file():
        raise RuntimeError(f"required file does not exist: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_command(args: list[str], *, cwd: Path) -> str:
    result = subprocess.run(args, cwd=cwd, check=True, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    return hashlib.sha256(result.stdout).hexdigest()


def untracked_manifest(workspace: Path) -> list[dict[str, Any]]:
    result = subprocess.run(
        ["git", "ls-files", "--others", "--exclude-standard", "-z"],
        cwd=workspace, check=True, capture_output=True,
    )
    manifest: list[dict[str, Any]] = []
    for raw_path in result.stdout.split(b"\0"):
        if not raw_path:
            continue
        relative = raw_path.decode("utf-8", errors="surrogateescape")
        path = workspace / relative
        entry: dict[str, Any] = {"path": relative}
        if path.is_file():
            entry["sha256"] = sha256(path)
            entry["size_bytes"] = path.stat().st_size
        else:
            entry["kind"] = "non_regular"
        manifest.append(entry)
    return manifest


def git_tree_state(workspace: Path) -> dict[str, Any]:
    status = command(["git", "status", "--porcelain"], cwd=workspace)
    return {
        "dirty": bool(status),
        "status": status.splitlines(),
        "tracked_diff_sha256": sha256_command(["git", "diff", "--binary"], cwd=workspace),
        "staged_diff_sha256": sha256_command(
            ["git", "diff", "--cached", "--binary"], cwd=workspace),
        "untracked_manifest": untracked_manifest(workspace),
    }


def git_identity(workspace: Path) -> dict[str, Any]:
    full_sha = command(["git", "rev-parse", "HEAD"], cwd=workspace, check=True)
    short_sha = command(["git", "rev-parse", "--short", "HEAD"], cwd=workspace, check=True)
    branch = command(["git", "branch", "--show-current"], cwd=workspace)
    submodule = workspace / PX4_MSGS
    if not submodule.is_dir():
        raise RuntimeError(f"PX4 message submodule is missing: {submodule}")
    submodule_sha = command(["git", "rev-parse", "HEAD"], cwd=submodule, check=True)
    tree = git_tree_state(workspace)
    submodule_tree = git_tree_state(submodule)
    return {
        "full_sha": full_sha,
        "short_sha": short_sha,
        "branch": branch or "(detached)",
        "dirty": tree["dirty"],
        "status": tree["status"],
        "tracked_diff_sha256": tree["tracked_diff_sha256"],
        "staged_diff_sha256": tree["staged_diff_sha256"],
        "untracked_manifest": tree["untracked_manifest"],
        "submodules": {
            PX4_MSGS.as_posix(): {
                "sha": submodule_sha,
                **submodule_tree,
            }
        },
    }


def validate_provenance(
    provenance: dict[str, Any], *, policy: str = "qualification",
    expected_sha: str | None = None,
) -> dict[str, Any]:
    """Annotate provenance and enforce the selected replay policy.

    Development replay may run from a dirty worktree, but its artifact is
    explicitly ineligible for qualification. Qualification is fail-closed.
    """
    if policy not in {"development", "qualification"}:
        raise ValueError(f"unknown provenance policy: {policy}")
    git = provenance.get("git", provenance)
    submodules = git.get("submodules", {})
    submodule_dirty = any(bool(item.get("dirty")) for item in submodules.values())
    dirty = bool(git.get("dirty")) or submodule_dirty
    actual_sha = git.get("full_sha")
    sha_matches = expected_sha is None or actual_sha == expected_sha
    eligible = not dirty and sha_matches
    provenance["provenance_policy"] = policy
    provenance["acceptance_eligible"] = eligible
    provenance["qualification_clean"] = not dirty
    provenance["expected_sha"] = expected_sha
    provenance["expected_sha_matches"] = sha_matches
    if policy == "qualification" and dirty:
        raise RuntimeError("benchmark refused: qualification requires clean worktree and submodule")
    if expected_sha is not None and not sha_matches:
        raise RuntimeError(
            f"benchmark refused: expected HEAD {expected_sha}, actual {actual_sha}"
        )
    return provenance


def require_clean(workspace: Path, expected_sha: str | None = None) -> dict[str, Any]:
    identity = git_identity(workspace)
    validate_provenance(identity, policy="qualification", expected_sha=expected_sha)
    return identity


def find_binary(workspace: Path, relative: str | None = None) -> Path:
    candidates = []
    if relative:
        candidates.append(workspace / relative)
    candidates.extend([
        workspace / "install/fast_lio_ros/lib/fast_lio_ros/fast_lio_node",
        workspace / "install-safe/fast_lio_ros/lib/fast_lio_ros/fast_lio_node",
        workspace / "build/fast_lio_ros/fast_lio_node",
    ])
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise RuntimeError("fast_lio_node binary was not found")


def host_snapshot() -> dict[str, Any]:
    def optional(args: list[str]) -> str:
        try:
            return command(args)
        except (OSError, subprocess.SubprocessError):
            return "unavailable"

    cpu = optional(["lscpu"])
    memory = optional(["free", "-h"])
    power = optional(["cpupower", "frequency-info"])
    process_list = optional(
        ["ps", "-eo", "pid,comm,%cpu,%mem", "--sort=-%cpu"]
    )
    return {
        "kernel": platform.platform(),
        "uname": optional(["uname", "-a"]),
        "cpu": cpu,
        "memory": memory,
        "power_profile": power,
        "background_processes": "\n".join(process_list.splitlines()[:31]),
        "environment": {
            key: os.environ[key]
            for key in sorted(os.environ)
            if key in {"AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID"}
        },
    }


def build_snapshot(workspace: Path, binary: Path, config: Path) -> dict[str, Any]:
    compiler = shutil.which("c++") or "c++"
    return {
        "build_type": os.environ.get("CMAKE_BUILD_TYPE", "RelWithDebInfo"),
        "compiler": compiler,
        "compiler_version": optional_command([compiler, "--version"]),
        "binary_path": str(binary),
        "binary_sha256": sha256(binary),
        "configuration_path": str(config.resolve()),
        "configuration_sha256": sha256(config.resolve()),
        "workspace": str(workspace.resolve()),
    }


def optional_command(args: list[str]) -> str:
    try:
        return command(args)
    except (OSError, subprocess.SubprocessError):
        return "unavailable"


def make_provenance(
    workspace: Path,
    *,
    config: Path,
    dataset: str | None = None,
    rate: float | None = None,
    expected_sha: str | None = None,
    binary_relative: str | None = None,
    policy: str = "qualification",
) -> dict[str, Any]:
    identity = git_identity(workspace)
    binary = find_binary(workspace, binary_relative)
    if not config.is_file():
        raise RuntimeError(f"benchmark refused: config does not exist: {config}")
    result: dict[str, Any] = {
        "git": {
            "full_sha": identity["full_sha"],
            "short_sha": identity["short_sha"],
            "branch": identity["branch"],
            "dirty": identity["dirty"],
        },
        "submodules": identity["submodules"],
        "build": build_snapshot(workspace, binary, config),
        "configuration": {
            "path": str(config.resolve()),
            "sha256": sha256(config.resolve()),
        },
        "host": host_snapshot(),
    }
    validate_provenance(result, policy=policy, expected_sha=expected_sha)
    if dataset is not None:
        result["dataset"] = {"id": dataset, "rate": rate}
    return result


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)
