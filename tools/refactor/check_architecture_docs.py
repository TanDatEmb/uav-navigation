#!/usr/bin/env python3
"""Check the documented repository package layout against package.xml."""

from __future__ import annotations

import re
from pathlib import Path
import xml.etree.ElementTree as ET


def violations(root: Path) -> list[str]:
    packages: set[str] = set()
    for package in (root / "src").rglob("package.xml"):
        if "examples" in package.parts:
            continue
        try:
            node = ET.parse(package).getroot().find("name")
        except ET.ParseError as error:
            return [f"invalid package.xml {package}: {error}"]
        if node is not None and node.text:
            packages.add(node.text.strip())
    layout = root / "docs/architecture/repository_layout.md"
    text = layout.read_text(encoding="utf-8")
    code_blocks = re.findall(r"```(?:text)?\n(.*?)```", text, flags=re.DOTALL)
    documented_tokens = set(re.findall(r"(?m)^\s*(?:src/)?([A-Za-z0-9_]+)/", "\n".join(code_blocks)))
    documented = documented_tokens - {
        "src", "common", "contracts", "estimation", "mapping", "planning", "tools", "docs", "config",
        "px4_ros2_interface_lib",
        "execution", "external", "px4", "runtime",
    }
    return [*(f"package missing from repository_layout.md: {name}" for name in sorted(packages - documented)),
            *(f"repository_layout.md names unknown package: {name}" for name in sorted(documented - packages))]


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    failures = violations(root)
    for failure in failures:
        print(f"ARCHITECTURE_DOCS: FAIL: {failure}")
    if not failures:
        print("ARCHITECTURE_DOCS: PASS")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
