#!/usr/bin/env python3
"""Synchronize and verify the pinned PX4 v1.17 ROS message dependency."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
from urllib.parse import urlparse

import yaml


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "dependencies/px4/px4_msgs.repos"
CHECKOUT = ROOT / "src/external/px4_msgs"
EXPECTED_REMOTE = "https://github.com/PX4/px4_msgs.git"
REQUIRED_FILES = (
    "msg/VehicleOdometry.msg",
    "msg/VehicleLocalPosition.msg",
    "msg/VehicleAttitude.msg",
    "msg/TimesyncStatus.msg",
)


def run(command: list[str], *, cwd: Path | None = None, input_text: str | None = None) -> subprocess.CompletedProcess[str]:
    print("+", " ".join(command), flush=True)
    return subprocess.run(
        command,
        cwd=cwd,
        input=input_text,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def lock() -> tuple[str, str]:
    manifest = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    entry = manifest["repositories"]["src/external/px4_msgs"]
    return str(entry["url"]), str(entry["version"])


def normalize_remote(value: str) -> str:
    if value.startswith("git@github.com:"):
        value = "https://github.com/" + value.removeprefix("git@github.com:")
    parsed = urlparse(value)
    path = parsed.path.removesuffix(".git").rstrip("/")
    return f"{parsed.scheme}://{parsed.netloc}{path}".lower()


def git(*args: str) -> str:
    result = run(["git", "-C", str(CHECKOUT), *args])
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "git command failed")
    return result.stdout.strip()


def verify() -> int:
    expected_remote, expected_sha = lock()
    print(f"dependency path: {CHECKOUT}")
    print(f"expected remote: {expected_remote}")
    print(f"expected SHA: {expected_sha}")
    if not CHECKOUT.is_dir() or not (CHECKOUT / ".git").exists():
        print("actual SHA: missing")
        print("working tree: missing")
        return 2
    actual_remote = git("remote", "get-url", "origin")
    actual_sha = git("rev-parse", "HEAD")
    dirty = bool(git("status", "--porcelain"))
    print(f"remote: {actual_remote}")
    print(f"actual SHA: {actual_sha}")
    print(f"working tree: {'dirty' if dirty else 'clean'}")
    errors: list[str] = []
    if normalize_remote(actual_remote) != normalize_remote(expected_remote):
        errors.append("remote URL mismatch")
    if actual_sha != expected_sha:
        errors.append("HEAD mismatch")
    if dirty:
        errors.append("working tree is dirty")
    for relative in REQUIRED_FILES:
        if not (CHECKOUT / relative).is_file():
            errors.append(f"missing required message: {relative}")
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 2
    print("PX4 v1.17 px4_msgs dependency: verified")
    return 0


def sync() -> int:
    expected_remote, expected_sha = lock()
    if not CHECKOUT.exists():
        result = run(["vcs", "import", "."], cwd=ROOT,
                     input_text=MANIFEST.read_text(encoding="utf-8"))
        if result.returncode != 0:
            print(result.stderr, file=sys.stderr)
            return result.returncode
        return verify()
    if not (CHECKOUT / ".git").exists():
        print(f"ERROR: {CHECKOUT} exists but is not a Git checkout", file=sys.stderr)
        return 2
    actual_remote = git("remote", "get-url", "origin")
    if normalize_remote(actual_remote) != normalize_remote(expected_remote):
        print("ERROR: refusing to mutate checkout with a different remote", file=sys.stderr)
        return 2
    if git("status", "--porcelain"):
        print("ERROR: refusing to update dirty px4_msgs checkout", file=sys.stderr)
        return 2
    if git("rev-parse", "HEAD") != expected_sha:
        fetch = run(["git", "-C", str(CHECKOUT), "fetch", "--no-tags", "origin", expected_sha])
        if fetch.returncode != 0:
            print(fetch.stderr, file=sys.stderr)
            return fetch.returncode
        checkout = run(["git", "-C", str(CHECKOUT), "checkout", "--detach", expected_sha])
        if checkout.returncode != 0:
            print(checkout.stderr, file=sys.stderr)
            return checkout.returncode
    return verify()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("operation", choices=("sync", "verify"))
    args = parser.parse_args()
    try:
        return sync() if args.operation == "sync" else verify()
    except (OSError, RuntimeError, KeyError, TypeError, yaml.YAMLError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
