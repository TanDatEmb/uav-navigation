#!/usr/bin/env python3
"""Check the closed dependency-direction rules for the Wave 3 baseline.

The guard reads package.xml and CMake text only; it does not require ROS.  A
known B3 violation is still printed as a warning with its removal WP.  Any
new violation fails closed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_XML_RE = re.compile(
    r"<(?P<tag>depend|build_depend|exec_depend)\s*>\s*"
    r"(?P<name>[A-Za-z0-9][A-Za-z0-9_.-]*)\s*</(?P=tag)\s*>"
)
CMAKE_CALL_RE = re.compile(
    r"(?P<kind>target_link_libraries|ament_target_dependencies)\s*\(\s*"
    r"(?P<target>[^\s\)]+)(?P<body>.*?)\)",
    re.DOTALL,
)
TOKEN_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_:$+.-]*")
ACCESS_SPECIFIERS = {"PUBLIC", "PRIVATE", "INTERFACE"}

# E6 is deliberately closed.  These are current B3 findings, not a blanket
# exemption: the named W3-B3 move must remove them.
PACKAGE_RULES: dict[str, set[str]] = {
    "navigation_certifier": {"navigation_planning_backend"},
    "navigation_execution": {
        "navigation_planning_backend",
        "navigation_mapping",
        "navigation_world_model",
        "rclcpp",
    },
    "navigation_mission": {
        "navigation_planning_backend",
        "navigation_mapping",
        "rclcpp",
    },
    "navigation_planning": {
        "navigation_planning_backend",
        "navigation_mapping",
        "rclcpp",
    },
}

ALLOWED_VIOLATIONS: dict[tuple[str, str, str | None, str], dict[str, str]] = {
    ("package", "navigation_execution", None, "navigation_world_model"): {
        "finding": "V1",
        "wp": "W3-B3",
    },
    ("cmake", "navigation_execution", "navigation_execution", "navigation_world_model"): {
        "finding": "V1",
        "wp": "W3-B3",
    },
    ("cmake", "navigation_runtime", "navigation_runtime_core", "rclcpp"): {
        "finding": "A1",
        "wp": "W3-B3",
    },
}


@dataclass(frozen=True)
class DependencyUse:
    package: str
    target: str | None
    dependency: str
    source: Path
    line: int
    kind: str


@dataclass
class PackageInfo:
    name: str
    path: Path
    manifest_dependencies: set[str] = field(default_factory=set)
    cmake_uses: list[DependencyUse] = field(default_factory=list)


def _line_number(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def _package_name(path: Path) -> str | None:
    text = path.read_text(encoding="utf-8")
    match = re.search(r"<name\s*>\s*([^<\s]+)\s*</name\s*>", text)
    return match.group(1) if match else None


def _resolve_target(raw_target: str, package: str) -> str:
    return raw_target.replace("${PROJECT_NAME}", package)


def _known_dependency(token: str, known: set[str]) -> str | None:
    token = token.strip()
    if token in ACCESS_SPECIFIERS or token.startswith("${"):
        return None
    base = token.split("::", 1)[0]
    return base if base in known else None


def collect_packages(root: Path = ROOT) -> list[PackageInfo]:
    rows: list[PackageInfo] = []
    for manifest in sorted(root.glob("src/**/package.xml")):
        name = _package_name(manifest)
        if not name:
            continue
        text = manifest.read_text(encoding="utf-8")
        deps = {match.group("name") for match in PACKAGE_XML_RE.finditer(text)}
        rows.append(PackageInfo(name=name, path=manifest.parent, manifest_dependencies=deps))

    known = {row.name for row in rows} | {"rclcpp"}
    for row in rows:
        cmake = row.path / "CMakeLists.txt"
        if not cmake.is_file():
            continue
        text = cmake.read_text(encoding="utf-8")
        for match in CMAKE_CALL_RE.finditer(text):
            target = _resolve_target(match.group("target"), row.name)
            for token in TOKEN_RE.findall(match.group("body")):
                dependency = _known_dependency(token, known)
                if dependency is None:
                    continue
                row.cmake_uses.append(
                    DependencyUse(
                        package=row.name,
                        target=target,
                        dependency=dependency,
                        source=cmake,
                        line=_line_number(text, match.start()),
                        kind=match.group("kind"),
                    )
                )
    return rows


def _is_product_target(target: str | None) -> bool:
    return bool(target) and not target.startswith("test_")


def find_violations(root: Path = ROOT) -> list[DependencyUse]:
    rows = collect_packages(root)
    package_names = {row.name for row in rows}
    violations: list[DependencyUse] = []
    for row in rows:
        forbidden = set(PACKAGE_RULES.get(row.name, set()))
        if "navigation_sitl_harness" in package_names and row.name != "navigation_runtime":
            forbidden.add("navigation_sitl_harness")
        for dependency in sorted(row.manifest_dependencies & forbidden):
            violations.append(
                DependencyUse(
                    package=row.name,
                    target=None,
                    dependency=dependency,
                    source=row.path / "package.xml",
                    line=1,
                    kind="package.xml",
                )
            )
        for use in row.cmake_uses:
            if _is_product_target(use.target) and use.dependency in forbidden:
                violations.append(use)
            if (
                _is_product_target(use.target)
                and use.target.endswith(("_policy", "_core"))
                and use.dependency == "rclcpp"
            ):
                violations.append(use)
            if (
                "navigation_sitl_harness" in package_names
                and row.name != "navigation_runtime"
                and use.dependency == "navigation_sitl_harness"
            ):
                violations.append(use)
    return violations


def _key(use: DependencyUse) -> tuple[str, str, str | None, str]:
    source_kind = "package" if use.target is None else "cmake"
    return source_kind, use.package, use.target, use.dependency


def main() -> int:
    rows = collect_packages()
    violations = find_violations()
    unexpected: list[DependencyUse] = []
    allowed = 0
    for use in violations:
        exception = ALLOWED_VIOLATIONS.get(_key(use))
        location = f"{use.source.relative_to(ROOT)}:{use.line}"
        if exception:
            allowed += 1
            print(
                "DEPENDENCY_DIRECTION: WARNING "
                f"allowed B3 violation {use.package} -> {use.dependency} "
                f"({location}, target={use.target or 'manifest'}; "
                f"finding={exception['finding']}, remove_in={exception['wp']})"
            )
        else:
            unexpected.append(use)

    if unexpected:
        print("DEPENDENCY_DIRECTION: FAIL", file=sys.stderr)
        for use in unexpected:
            location = f"{use.source.relative_to(ROOT)}:{use.line}"
            print(
                f"- {use.package} target={use.target or 'manifest'} "
                f"depends on forbidden {use.dependency} ({location})",
                file=sys.stderr,
            )
        return 1

    print(
        "DEPENDENCY_DIRECTION: PASS "
        f"(packages={len(rows)}, allowed_baseline_violations={allowed})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
