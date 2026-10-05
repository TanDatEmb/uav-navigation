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

    def test_manifest_attributes_and_build_export_depend_are_checked(self) -> None:
        root = self.write_fixture(
            "navigation_planning",
            "<package><name>navigation_planning</name>"
            '<build_export_depend condition="x">navigation_mapping</build_export_depend>'
            "</package>",
        )
        violations = guard.find_violations(root)
        self.assertEqual(
            [(v.package, v.dependency, v.target) for v in violations],
            [("navigation_planning", "navigation_mapping", None)],
        )

    def test_cmake_variables_generators_and_comments_are_checked(self) -> None:
        root = self.write_fixture(
            "navigation_certifier",
            "<package><name>navigation_certifier</name></package>",
            "# target_link_libraries(fake rclcpp)\n"
            "target_link_libraries(certifier PRIVATE ${RCLCPP_TARGETS})\n"
            "ament_target_dependencies(certifier $<BUILD_INTERFACE:rclcpp::rclcpp>)\n",
        )
        violations = guard.find_violations(root)
        self.assertEqual(
            [(v.target, v.dependency) for v in violations],
            [("certifier", "rclcpp"), ("certifier", "rclcpp")],
        )

    def test_certifier_product_target_cannot_depend_on_rclcpp(self) -> None:
        root = self.write_fixture(
            "navigation_certifier",
            "<package><name>navigation_certifier</name></package>",
            "ament_target_dependencies(certifier rclcpp)",
        )
        violations = guard.find_violations(root)
        self.assertEqual(
            [(v.package, v.target, v.dependency) for v in violations],
            [("navigation_certifier", "certifier", "rclcpp")],
        )

    def test_diagnostics_subscription_is_allowlisted_with_adr021_owner(self) -> None:
        root = self.write_fixture(
            "px4_odometry_bridge",
            "<package><name>px4_odometry_bridge</name></package>",
        )
        source = root / "src" / "px4_odometry_bridge" / "src" / "bridge.cpp"
        source.parent.mkdir(parents=True)
        source.write_text(
            'auto sub = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(\n'
            '  "/lio/diagnostics", qos, callback);\n',
            encoding="utf-8",
        )
        violations = guard.find_violations(root)
        self.assertEqual(
            [(v.kind, v.package, v.dependency) for v in violations],
            [("topic", "px4_odometry_bridge", "/lio/diagnostics")],
        )
        self.assertEqual(
            guard.ALLOWED_VIOLATIONS[guard._key(violations[0])],
            {"finding": "V5/O1-02", "wp": "P6"},
        )

    def test_navigation_evidence_subscription_is_rejected(self) -> None:
        root = self.write_fixture(
            "navigation_runtime",
            "<package><name>navigation_runtime</name></package>",
        )
        source = root / "src" / "navigation_runtime" / "src" / "runtime.cpp"
        source.parent.mkdir(parents=True)
        source.write_text(
            'auto sub = create_subscription<navigation_evidence::msg::Record>(\n'
            '  "/navigation_evidence/record", qos, callback);\n',
            encoding="utf-8",
        )
        violations = guard.find_violations(root)
        self.assertEqual(
            [(v.kind, v.package, v.dependency) for v in violations],
            [("topic", "navigation_runtime", "/navigation_evidence/record")],
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
