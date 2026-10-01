#!/usr/bin/env python3
"""Static completeness checks for the WP-A2 as-is ICD.

The checks are deliberately source/manifest checks.  They do not claim a
runtime DDS graph, negotiated QoS, measured rate, or flight qualification.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "docs/refactor/WP-A2"
EXCLUDED = {"test", "tests", "external", "vendor", "build"}


def source_files(root: Path):
    for path in root.rglob("*"):
        if not path.is_file() or path.suffix not in {".cpp", ".hpp", ".cc", ".h"}:
            continue
        if any(part in EXCLUDED for part in path.parts):
            continue
        yield path


def scan_ros_calls() -> list[tuple[str, str]]:
    calls: list[tuple[str, str]] = []
    pattern = re.compile(r"create_(publisher|subscription|service|client)\s*<")
    for path in source_files(ROOT / "src"):
        rel = path.relative_to(ROOT).as_posix()
        for number, line in enumerate(path.read_text(errors="replace").splitlines(), 1):
            match = pattern.search(line)
            if match:
                calls.append((f"{rel}:{number}", match.group(1)))
    return calls


def topic_endpoint_locations(manifest: list[dict]) -> set[str]:
    locations: set[str] = set()
    for item in manifest:
        for role in ("publishers", "subscribers", "services", "clients"):
            for endpoint in item.get(role, []) or []:
                if isinstance(endpoint, dict) and isinstance(endpoint.get("at"), str):
                    locations.add(endpoint["at"])
    return locations


def message_declarations(path: Path) -> tuple[set[str], set[str]]:
    fields: set[str] = set()
    constants: set[str] = set()
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.replace("=", " = ").split()
        if len(parts) >= 4 and parts[2] == "=":
            constants.add(parts[1])
        elif len(parts) >= 2:
            fields.add(parts[1])
    return fields, constants


def diagnostic_literals() -> set[str]:
    names: set[str] = set()
    pattern = re.compile(
        r"['\"]((?:fast_lio|navigation_runtime|navigation_mapping|"
        r"navigation_external_mode|px4_odometry_bridge)/[A-Za-z0-9_.-]+)['\"]"
    )
    paths = list(source_files(ROOT / "src")) + list((ROOT / "tools/runtime").glob("*.py"))
    for path in paths:
        for line in path.read_text(errors="replace").splitlines():
            if "#include" in line:
                continue
            if not re.search(r"(?:status|status_name|name)\b", line):
                continue
            names.update(match.group(1) for match in pattern.finditer(line))
    return names


def main() -> int:
    errors: list[str] = []
    try:
        topics = yaml.safe_load((OUT / "icd_topics.yaml").read_text())
        messages = yaml.safe_load((OUT / "icd_msgs.yaml").read_text())
        diagnostics = yaml.safe_load((OUT / "diagnostic_channels.yaml").read_text())
    except Exception as exc:
        print(f"YAML_PARSE_ERROR: {exc}")
        return 1

    if not isinstance(topics, list):
        errors.append("icd_topics.yaml is not a list")
        topics = []
    locations = topic_endpoint_locations(topics)
    for location, kind in scan_ros_calls():
        if location not in locations:
            errors.append(f"missing {kind}: {location}")

    by_msg = {item.get("msg"): item for item in messages or [] if isinstance(item, dict)}
    msg_root = ROOT / "src/contracts/navigation_contracts/msg"
    for source in sorted(msg_root.glob("*.msg")):
        item = by_msg.get(source.stem)
        if item is None:
            errors.append(f"missing message entry: {source.stem}")
            continue
        declared, constants = message_declarations(source)
        listed_fields = {field.get("name") for field in item.get("fields", [])}
        listed_constants = {constant.get("name") for constant in item.get("constants", [])}
        for name in sorted(declared - listed_fields):
            errors.append(f"missing field {source.stem}.{name}")
        for name in sorted(constants - listed_constants):
            errors.append(f"missing constant {source.stem}.{name}")
        for name in sorted(listed_fields - declared):
            errors.append(f"extra field {source.stem}.{name}")

    channel_names = {
        item.get("name") for item in (diagnostics or {}).get("channels", [])
        if isinstance(item, dict)
    }
    for name in sorted(diagnostic_literals() - channel_names):
        errors.append(f"missing diagnostic channel: {name}")

    if errors:
        print("\n".join(errors))
        print(f"FAIL: {len(errors)} issue(s)")
        return 1
    print(
        "PASS: topic create-call coverage, 12 navigation_contracts messages, "
        f"and {len(channel_names)} diagnostic channels"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
