#!/usr/bin/env python3
"""Check the closed package dependency rules from ADR-018 E3/E6."""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
PACKAGE_RE = re.compile(r"<(?P<tag>depend|build_depend|exec_depend)>(?P<name>[^<]+)</")
DEP_CALL_RE = re.compile(
    r"(?:ament_target_dependencies|target_link_libraries)\s*\([^)]*?(?P<body>[^)]*)\)",
    re.DOTALL,
)


def package_name(package_xml: Path) -> str:
    match = re.search(r"<name>([^<]+)</name>", package_xml.read_text())
    return match.group(1).strip() if match else package_xml.parent.name


def package_dependencies() -> dict[str, set[str]]:
    result: dict[str, set[str]] = {}
    for package_xml in SRC.rglob("package.xml"):
        name = package_name(package_xml)
        result[name] = {
            match.group("name").strip()
            for match in PACKAGE_RE.finditer(package_xml.read_text())
        }
    return result


def cmake_dependencies() -> dict[str, set[str]]:
    result: dict[str, set[str]] = {}
    for cmake in SRC.rglob("CMakeLists.txt"):
        package = cmake.parent.name
        text = cmake.read_text()
        deps = set()
        for match in DEP_CALL_RE.finditer(text):
            deps.update(re.findall(r"\b[a-z][a-z0-9_]+\b", match.group("body")))
        result.setdefault(package, set()).update(deps)
    return result


def main() -> int:
    package_deps = package_dependencies()
    cmake_deps = cmake_dependencies()
    effective = {
        package: package_deps.get(package, set()) | cmake_deps.get(package, set())
        for package in set(package_deps) | set(cmake_deps)
    }
    violations: list[str] = []

    def forbid(source: str, targets: set[str], reason: str) -> None:
        for target in sorted(targets & effective.get(source, set())):
            violations.append(f"{source} -> {target}: {reason}")

    if "navigation_certifier" in effective:
        forbid("navigation_certifier", {"navigation_planning_backend"}, "certifier must not depend on planner backend")
    for source in ("navigation_execution", "navigation_mission", "navigation_planning"):
        forbid(source, {"navigation_planning_backend", "navigation_mapping", "rclcpp"}, "core contract must not depend on backend/mapping/ROS")
    for source in effective:
        if source.endswith(("_policy", "_core")):
            forbid(source, {"rclcpp"}, "policy/core must not depend on ROS")
    if "navigation_sitl_harness" in effective:
        for source, deps in effective.items():
            if "navigation_sitl_harness" in deps and source != "navigation_runtime":
                violations.append(f"{source} -> navigation_sitl_harness: only navigation_runtime may depend on harness")

    if violations:
        for violation in violations:
            print(f"DEPENDENCY_DIRECTION: FAIL: {violation}", file=sys.stderr)
        return 1
    print(f"DEPENDENCY_DIRECTION: PASS ({len(effective)} package/CMake units checked)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
