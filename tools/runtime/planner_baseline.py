#!/usr/bin/env python3
"""Capture an auditable planner baseline without changing planner behavior.

The output is diagnostic provenance only.  In particular, source YAML is
recorded as a source snapshot and is never labelled as a generated runtime
configuration unless the caller supplies that artifact explicitly.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIGS = (
    "src/runtime/navigation_runtime/config/planner.yaml",
    "config/runtime/common.yaml",
    "config/runtime/sim.yaml",
)
OUTPUT_NAMES = (
    "planner_baseline_manifest.json",
    "planner_baseline_delta.md",
    "planner_changed_files.csv",
    "generated_config_snapshot.yaml",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git(*args: str, check: bool = True) -> str:
    result = subprocess.run(
        ["git", "-C", str(ROOT), *args],
        capture_output=True,
        text=True,
        check=False,
    )
    if check and result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "git command failed")
    return result.stdout


def _command_version(command: str) -> str:
    result = subprocess.run(
        [command, "--version"], capture_output=True, text=True, check=False
    )
    line = (result.stdout or result.stderr).splitlines()
    return line[0] if result.returncode == 0 and line else "UNAVAILABLE"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _status_records() -> list[dict[str, str]]:
    records = []
    for line in _git("status", "--porcelain=v2", "--untracked-files=all").splitlines():
        if line.startswith(("1 ", "2 ")):
            # Porcelain v2 has fixed fields followed by a path that may
            # contain spaces.  Split only through the score field and use the
            # first path for renames/copies as the changed-path identity.
            fields = line.split(maxsplit=8)
            if len(fields) == 9:
                records.append(
                    {"status": fields[1], "path": fields[8].split("\t", 1)[0]}
                )
        elif line.startswith("? "):
            records.append({"status": "??", "path": line[2:]})
    return records


def classify_path(path: str) -> str:
    normalized = path.replace("\\", "/")
    if normalized.startswith("docs/"):
        return "docs"
    if "/test/" in normalized or normalized.startswith("tools/tests/") or normalized.startswith("tools/runtime/tests/"):
        return "tests"
    if normalized.startswith("tools/runtime/") or normalized.startswith("tools/"):
        return "observability_or_validation"
    if normalized.startswith("src/planning/"):
        return "planner"
    if normalized.startswith("src/mapping/"):
        return "mapping"
    if normalized.startswith("src/estimation/"):
        return "estimation"
    if normalized.startswith("src/runtime/") or normalized.startswith("src/external/"):
        return "runtime_or_integration"
    if normalized.startswith("config/") or normalized.startswith("launch/"):
        return "configuration"
    return "other"


def _diff_records(reference: str | None, candidate: str) -> tuple[list[dict[str, str]], str | None]:
    if not reference:
        return [], None
    result = subprocess.run(
        ["git", "-C", str(ROOT), "diff", "--name-status", f"{reference}..{candidate}"],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        return [], result.stderr.strip() or "git diff failed"
    records = []
    for line in result.stdout.splitlines():
        fields = line.split("\t")
        if len(fields) >= 2:
            records.append({"status": fields[0], "path": fields[-1]})
    return records, None


def _config_entries(configs: Iterable[str]) -> tuple[list[dict[str, Any]], list[str]]:
    entries: list[dict[str, Any]] = []
    errors: list[str] = []
    for raw in configs:
        path = (ROOT / raw).resolve()
        try:
            path.relative_to(ROOT.resolve())
        except ValueError:
            errors.append(f"config outside workspace: {raw}")
            continue
        if not path.is_file():
            errors.append(f"config missing: {raw}")
            continue
        entries.append(
            {
                "path": str(path.relative_to(ROOT)),
                "sha256": sha256_file(path),
                "content": path.read_text(encoding="utf-8"),
            }
        )
    return entries, errors


def _write_config_snapshot(path: Path, entries: list[dict[str, Any]], errors: list[str]) -> None:
    lines = [
        "schema_version: 1",
        "authority: source_snapshot_not_generated_runtime_config",
        f"captured_utc: {_utc_now()}",
        "files:",
    ]
    for entry in entries:
        lines.extend(
            [
                f"  - path: {entry['path']}",
                f"    sha256: {entry['sha256']}",
                "    content: |",
            ]
        )
        content = entry["content"].splitlines() or [""]
        lines.extend(f"      {line}" for line in content)
    if errors:
        lines.append("errors:")
        lines.extend(f"  - {error}" for error in errors)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_changed_csv(path: Path, records: list[dict[str, str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("status", "category", "path"))
        writer.writeheader()
        for record in records:
            writer.writerow(
                {
                    "status": record["status"],
                    "category": classify_path(record["path"]),
                    "path": record["path"],
                }
            )


def _write_delta(path: Path, reference: str | None, candidate: str, records: list[dict[str, str]], error: str | None) -> None:
    lines = [
        "# Planner baseline delta",
        "",
        f"- Reference: `{reference or 'NOT_SPECIFIED'}`",
        f"- Candidate: `{candidate}`",
        "- Scope: reference diff plus current dirty worktree; this file is not an acceptance verdict.",
        "",
    ]
    if error:
        lines.extend([f"- Diff status: `BLOCKED` ({error})", ""])
    else:
        lines.extend([f"- Changed paths: `{len(records)}`", "", "| Status | Category | Path |", "|---|---|---|"])
        lines.extend(
            f"| {record['status']} | {classify_path(record['path'])} | `{record['path']}` |"
            for record in records
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def capture(output_dir: Path, reference: str | None, candidate: str, configs: Iterable[str]) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    candidate_resolved = _git("rev-parse", candidate).strip()
    status = _status_records()
    reference_changed, diff_error = _diff_records(reference, candidate_resolved)
    changed = list(reference_changed)
    known_paths = {record["path"] for record in changed}
    for record in status:
        if record["path"] not in known_paths:
            changed.append(record)
    config_entries, config_errors = _config_entries(configs)
    install = ROOT / "install"
    build_validation: dict[str, Any]
    try:
        runtime_dir = str(Path(__file__).resolve().parent)
        if runtime_dir not in sys.path:
            sys.path.insert(0, runtime_dir)
        from build_provenance import validate_manifest

        build_validation = validate_manifest(ROOT, install)
    except Exception as error:  # provenance must expose, not hide, a blocker
        build_validation = {"status": "INVALID_OR_UNAVAILABLE", "error": str(error)}

    manifest: dict[str, Any] = {
        "schema_version": 1,
        "captured_utc": _utc_now(),
        "repository": {
            "root": str(ROOT),
            "head": _git("rev-parse", "HEAD").strip(),
            "branch": _git("branch", "--show-current").strip(),
            "tree": _git("rev-parse", "HEAD^{tree}").strip(),
            "status": status,
            "source_fingerprint": None,
        },
        "comparison": {
            "reference": reference,
            "candidate": candidate_resolved,
            "status": "BLOCKED" if diff_error else "RECORDED",
            "error": diff_error,
            "changed_file_count": len(changed),
            "reference_diff_file_count": len(reference_changed),
            "working_tree_file_count": len(status),
        },
        "toolchain": {
            "kernel": platform.release(),
            "machine": platform.machine(),
            "python": sys.version.split()[0],
            "cmake": _command_version("cmake"),
            "colcon": _command_version("colcon"),
            "gcc": _command_version("gcc"),
            "g++": _command_version("g++"),
            "ros_distro": os.environ.get("ROS_DISTRO", "UNSET"),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "UNSET"),
        },
        "build_provenance": build_validation,
        "config_snapshot": {
            "authority": "source_snapshot_not_generated_runtime_config",
            "files": [{"path": item["path"], "sha256": item["sha256"]} for item in config_entries],
            "errors": config_errors,
        },
        "artifacts": {name: str(output_dir / name) for name in OUTPUT_NAMES},
    }
    try:
        from build_provenance import source_fingerprint

        manifest["repository"]["source_fingerprint"] = source_fingerprint(ROOT)
    except Exception as error:
        manifest["repository"]["source_fingerprint"] = {"status": "UNAVAILABLE", "error": str(error)}

    _write_changed_csv(output_dir / "planner_changed_files.csv", changed)
    _write_delta(output_dir / "planner_baseline_delta.md", reference, candidate_resolved, changed, diff_error)
    _write_config_snapshot(output_dir / "generated_config_snapshot.yaml", config_entries, config_errors)
    manifest_path = output_dir / "planner_baseline_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--reference-sha")
    parser.add_argument("--candidate-sha", default="HEAD")
    parser.add_argument("--config", action="append", default=list(DEFAULT_CONFIGS))
    args = parser.parse_args()
    manifest = capture(args.output_dir, args.reference_sha, args.candidate_sha, args.config)
    print(json.dumps({"status": "RECORDED", "manifest": manifest["artifacts"]["planner_baseline_manifest.json"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
