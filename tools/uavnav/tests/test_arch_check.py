"""Tests for tools.uavnav.arch_check: evidence, deviations and dependency direction (pilot scenarios 4-6)."""
import datetime
import unittest

from tools.uavnav import arch_check
from tools.uavnav.tests.arch_fixture import Repo, dv_row

TODAY = datetime.date(2026, 10, 13)


def errors(repo, **kwargs):
    found = arch_check.run_checks(repo.root, repo.rules, today=TODAY, **kwargs)
    return [str(f) for f in found if f.level == "ERROR"]


class TraceabilityTest(unittest.TestCase):
    def setUp(self):
        self.repo = Repo()
        self.repo.write("src/core/uavnav_core/test/test_x.cpp", "TEST(Suite, Case) {}\n")
        self.sha = self.repo.commit("feat: add test")

    def tearDown(self):
        self.repo.close()

    def row(self, status, evidence):
        return f"| P1 | §3.1 | S1 | | {status} | {evidence} |\n"

    def test_designed_needs_no_evidence(self):
        self.repo.set_trace(self.row("DESIGNED", ""))
        self.assertEqual(errors(self.repo), [])

    def test_legacy_status_words_are_rejected(self):
        for word in ("todo", "doing", "done"):
            self.repo.set_trace(self.row(word, ""))
            self.assertTrue(any("is not one of" in e for e in errors(self.repo)), word)

    def test_component_verified_needs_an_existing_test(self):
        self.repo.set_trace(self.row("COMPONENT_VERIFIED", "`Suite.Case`"))
        self.assertEqual(errors(self.repo), [])
        self.repo.set_trace(self.row("COMPONENT_VERIFIED", "`Suite.Missing`"))
        self.assertTrue(any("not found in the sources" in e for e in errors(self.repo)))
        self.repo.set_trace(self.row("COMPONENT_VERIFIED", ""))
        self.assertTrue(any("COMPONENT_VERIFIED needs" in e for e in errors(self.repo)))

    def test_a_new_test_does_not_make_a_row_qualified(self):
        self.repo.set_trace(self.row("QUALIFIED", f"`Suite.Case`, `{self.sha[:8]}`"))
        found = errors(self.repo)
        self.assertTrue(any("QUALIFIED needs a commit and a `run:" in e for e in found), found)
        self.assertTrue(any("QUALIFIED needs `accept:" in e for e in found), found)

    def test_integrated_needs_a_run_and_qualified_needs_an_agreed_decision(self):
        self.repo.set_trace(self.row("INTEGRATED", f"`{self.sha[:8]}`, `run:20261013-gps-hover`"))
        self.assertEqual(errors(self.repo), [])
        self.repo.set_trace(self.row("QUALIFIED", f"`{self.sha[:8]}`, `run:x`, `accept:D2`"))
        self.assertTrue(any("QUALIFIED needs `accept:" in e for e in errors(self.repo)))
        self.repo.set_trace(self.row("QUALIFIED", f"`{self.sha[:8]}`, `run:x`, `accept:D1`"))
        self.assertEqual(errors(self.repo), [])

    def test_dangling_commit_and_unknown_decision_are_errors(self):
        self.repo.set_trace(self.row("IMPLEMENTED", "`deadbeef`"))
        self.assertTrue(any("commit deadbeef not found" in e for e in errors(self.repo)))
        self.repo.set_trace(f"| P1 D99 | §3.1 | S1 | | DESIGNED | |\n")
        self.assertTrue(any("D99" in e for e in errors(self.repo)))
        self.repo.set_trace(f"| P1 | §9.9 | S1 | | DESIGNED | |\n")
        self.assertTrue(any("§9.9" in e for e in errors(self.repo)))


class DeviationTest(unittest.TestCase):
    def setUp(self):
        self.repo = Repo()
        self.sha = self.repo.head()

    def tearDown(self):
        self.repo.close()

    def trace(self, dvs, rows=""):
        self.repo.set_trace(rows, dvs)

    def test_a_complete_open_deviation_passes(self):
        self.trace(dv_row())
        self.assertEqual(errors(self.repo), [])

    def test_open_deviation_needs_owner_trigger_and_plan(self):
        for kwargs, needle in (({"owner": ""}, "'Owner' is empty"), ({"review": ""}, "review date or trigger"),
                               ({"plan": ""}, "closing plan")):
            self.trace(dv_row(**kwargs))
            self.assertTrue(any(needle in e for e in errors(self.repo)), (kwargs, errors(self.repo)))

    def test_overdue_review_warns_locally_and_fails_in_ci(self):
        self.trace(dv_row(review="2026-01-01"))
        self.assertEqual(errors(self.repo), [])
        self.assertTrue(any("has passed" in e for e in errors(self.repo, strict_dates=True)))

    def test_closed_deviation_needs_existing_evidence(self):
        self.trace(dv_row(status="CLOSED", evidence=f"`{self.sha[:8]}`"))
        self.assertEqual(errors(self.repo), [])
        self.trace(dv_row(status="CLOSED", evidence="`deadbeef`"))
        self.assertTrue(any("commit deadbeef not found" in e for e in errors(self.repo)))
        self.trace(dv_row(status="CLOSED", evidence=""))
        self.assertTrue(any("needs evidence" in e for e in errors(self.repo)))

    def test_permanent_acceptance_needs_an_agreed_decision(self):
        self.trace(dv_row(kind="PERMANENT", status="ACCEPTED", plan="D2", evidence="`D2`"))
        self.assertTrue(any("permanent acceptance" in e for e in errors(self.repo)))
        self.trace(dv_row(kind="PERMANENT", status="ACCEPTED", plan="D1", evidence="`D1`"))
        self.assertEqual(errors(self.repo), [])

    def test_deviated_status_must_point_at_an_open_deviation(self):
        row = "| P1 | §3.1 | S1 | | DEVIATED | `DV1` |\n"
        self.trace(dv_row(), row)
        self.assertEqual(errors(self.repo), [])
        self.trace(dv_row(status="CLOSED", evidence=f"`{self.sha[:8]}`"), row)
        self.assertTrue(any("needs an OPEN deviation" in e for e in errors(self.repo)))


class DependencyTest(unittest.TestCase):
    PACKAGE = """<package format="3"><name>{name}</name>{deps}</package>"""

    def setUp(self):
        self.repo = Repo()
        self.repo.write("src/estimation/uavnav_lio_core/package.xml",
                        self.PACKAGE.format(name="uavnav_lio_core", deps="<depend>uavnav_core</depend>"))
        self.repo.write("src/core/uavnav_core/package.xml", self.PACKAGE.format(name="uavnav_core", deps=""))
        self.repo.commit("feat: packages")

    def tearDown(self):
        self.repo.close()

    def test_clean_tree_passes(self):
        self.assertEqual(errors(self.repo), [])

    def test_ros_include_in_a_ros_free_package_is_an_error(self):
        self.repo.write("src/estimation/uavnav_lio_core/src/x.cpp", '#include <rclcpp/rclcpp.hpp>\n')
        self.repo.commit("feat: ros leak")
        self.assertTrue(any("ROS include rclcpp/rclcpp.hpp" in e for e in errors(self.repo)))

    def test_ros_package_dependency_in_a_ros_free_package_is_an_error(self):
        self.repo.write("src/estimation/uavnav_lio_core/package.xml",
                        self.PACKAGE.format(name="uavnav_lio_core", deps="<depend>uavnav_core</depend><depend>rclcpp</depend>"))
        self.repo.commit("feat: ros dep")
        self.assertTrue(any("depends on rclcpp" in e for e in errors(self.repo)))

    def test_an_edge_outside_allowed_deps_is_an_error(self):
        self.repo.write("src/core/uavnav_core/package.xml",
                        self.PACKAGE.format(name="uavnav_core", deps="<depend>uavnav_lio_core</depend>"))
        self.repo.commit("feat: upward edge")
        self.assertTrue(any("uavnav_core depends on uavnav_lio_core" in e for e in errors(self.repo)))

    def test_raw_stdout_in_product_code_is_an_error(self):
        self.repo.write("src/estimation/uavnav_lio_core/src/y.cpp", 'void f() { std::cout << 1; }\n')
        self.repo.commit("feat: print")
        self.assertTrue(any("stdout/stderr" in e for e in errors(self.repo)))
        self.repo.write("src/estimation/uavnav_lio_core/src/y.cpp", '// std::cout in a comment\n')
        self.repo.commit("fix: remove print")
        self.assertEqual(errors(self.repo), [])


if __name__ == "__main__":
    unittest.main()
