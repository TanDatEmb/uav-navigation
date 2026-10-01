from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

import tools.check_dependency_direction as guard


class DependencyDirectionTests(unittest.TestCase):
    def write_fixture(self, package_name: str, manifest: str, cmake: str = "") -> Path:
        root = Path(tempfile.mkdtemp(prefix="dependency-guard-"))
        package = root / "src" / package_name
        package.mkdir(parents=True)
        (package / "package.xml").write_text(manifest, encoding="utf-8")
        (package / "CMakeLists.txt").write_text(cmake, encoding="utf-8")
        return root

    def test_manifest_forbidden_dependency_is_reported(self) -> None:
        root = self.write_fixture(
            "navigation_planning",
            "<package><name>navigation_planning</name>"
            "<depend>navigation_planning_backend</depend></package>",
        )
        violations = guard.find_violations(root)
        self.assertEqual(
            [(v.package, v.dependency, v.target) for v in violations],
            [("navigation_planning", "navigation_planning_backend", None)],
        )

    def test_core_target_ros_dependency_is_reported(self) -> None:
        root = self.write_fixture(
            "navigation_runtime",
            "<package><name>navigation_runtime</name></package>",
            "ament_target_dependencies(navigation_runtime_policy rclcpp)",
        )
        violations = guard.find_violations(root)
        self.assertEqual(
            [(v.target, v.dependency) for v in violations],
            [("navigation_runtime_policy", "rclcpp")],
        )

    def test_sitl_harness_is_only_allowed_from_runtime(self) -> None:
        root = Path(tempfile.mkdtemp(prefix="dependency-guard-"))
        for package_name, dependency in (
            ("navigation_sitl_harness", ""),
            ("navigation_runtime", "navigation_sitl_harness"),
            ("navigation_execution", "navigation_sitl_harness"),
        ):
            package = root / "src" / package_name
            package.mkdir(parents=True)
            content = f"<package><name>{package_name}</name>"
            if dependency:
                content += f"<depend>{dependency}</depend>"
            (package / "package.xml").write_text(content + "</package>", encoding="utf-8")
            (package / "CMakeLists.txt").write_text("", encoding="utf-8")
        violations = guard.find_violations(root)
        self.assertEqual(
            [(v.package, v.dependency) for v in violations],
            [("navigation_execution", "navigation_sitl_harness")],
        )


if __name__ == "__main__":
    unittest.main()
