"""Tests for tools.uavnav.arch_health: stress indicators trigger a review and a KEEP review silences them
(pilot scenarios 3 and 7)."""
import datetime
import unittest

from tools.uavnav import arch_health
from tools.uavnav.tests.arch_fixture import Repo

HPP = "src/estimation/uavnav_lio_core/include/uavnav/lio/state.hpp"
CPP = "src/estimation/uavnav_lio_core/src/other.cpp"


def analyse(repo):
    return arch_health.analyse(repo.root, repo.rules, repo.base, today=datetime.date(2026, 10, 13))


def triggers(repo, needle=""):
    return [t for t in analyse(repo)["triggers"] if needle in t]


class HealthTest(unittest.TestCase):
    def setUp(self):
        self.repo = Repo()

    def tearDown(self):
        self.repo.close()

    def add_flag(self, name, subject):
        text = (self.repo.root / HPP).read_text()
        i = text.rindex("};")
        self.repo.write(HPP, text[:i] + f"  bool {name}{{false}};\n" + text[i:])
        self.repo.commit(subject)

    def test_normal_development_raises_no_signal(self):
        self.repo.write(CPP, "int a = 1;\n")
        self.repo.commit("fix(lio): one bug")
        self.repo.write(CPP, "int a = 2;\n")
        self.repo.commit("feat(lio): grow")
        self.assertEqual(triggers(self.repo), [])

    def test_two_new_flags_stay_below_the_threshold_and_three_trigger_a_review(self):
        self.add_flag("is_recovering", "feat(lio): recovery")
        self.add_flag("reset_pending", "fix(lio): reset")
        self.assertEqual(triggers(self.repo, "S1 new bool"), [])
        self.add_flag("force_reinit", "fix(lio): reinit")
        found = triggers(self.repo, "S1 new bool")
        self.assertEqual(len(found), 1)
        self.assertIn("lio", found[0])

    def test_creating_a_file_is_not_stress_but_adding_a_state_later_is(self):
        self.repo.write("src/estimation/uavnav_lio_core/include/uavnav/lio/fsm.hpp", "enum class FooState { kA, kB };\n")
        self.repo.commit("feat(lio): fsm")
        self.assertEqual(triggers(self.repo, "state enumerators"), [])
        self.repo.write("src/estimation/uavnav_lio_core/include/uavnav/lio/fsm.hpp", "enum class FooState { kA, kB, kC };\n")
        self.repo.commit("fix(lio): c")
        self.assertEqual(len(triggers(self.repo, "state enumerators")), 1)

    def test_flags_in_a_newly_created_header_are_not_stress(self):
        body = "struct T {\n" + "".join(f"  bool f{i}{{false}};\n" for i in range(5)) + "};\n"
        self.repo.write("src/estimation/uavnav_lio_core/include/uavnav/lio/new.hpp", body)
        self.repo.commit("feat(lio): new component")
        self.assertEqual(triggers(self.repo, "S1 new bool"), [])

    def test_workaround_markers_count_in_product_code_but_not_in_the_governance_tools(self):
        for i in range(3):
            self.repo.write(f"tools/uavnav/arch_x{i}.py", "# workaround workaround\n")
        self.repo.commit("feat(tooling): tool strings")
        self.assertEqual(triggers(self.repo, "S2"), [])
        self.repo.write("src/estimation/uavnav_lio_core/src/w.cpp", "// workaround for now\n// hack\n")
        self.repo.commit("fix(lio): w")
        self.assertEqual(len(triggers(self.repo, "S2")), 1)

    def test_repeated_fixes_to_one_file_trigger(self):
        for i in range(5):
            self.repo.write(CPP, f"int a = {i + 10};\n")
            self.repo.commit(f"fix(lio): again {i}")
        self.assertEqual(len(triggers(self.repo, "S3")), 1)

    def test_a_keep_review_silences_the_reviewed_history_but_new_evidence_counts_again(self):
        for i in range(5):
            self.repo.write(CPP, f"int a = {i + 10};\n")
            self.repo.commit(f"fix(lio): again {i}")
        self.assertEqual(len(triggers(self.repo, "S3")), 1)
        self.repo.add_ar("AR1", "lio", self.repo.head(), "KEEP")
        self.repo.commit("docs: AR1 keep after review")
        self.assertEqual(triggers(self.repo), [])
        for i in range(5):
            self.repo.write(CPP, f"int a = {i + 100};\n")
            self.repo.commit(f"fix(lio): later {i}")
        self.assertEqual(len(triggers(self.repo, "S3")), 1)

    def test_a_review_of_one_subsystem_does_not_silence_another(self):
        for i in range(5):
            self.repo.write(CPP, f"int a = {i + 10};\n")
            self.repo.commit(f"fix(lio): again {i}")
        self.repo.add_ar("AR1", "px4_bridge", self.repo.head(), "KEEP")
        self.repo.commit("docs: AR1")
        self.assertEqual(len(triggers(self.repo, "S3")), 1)

    def test_open_deviation_past_its_date_and_three_open_deviations_trigger(self):
        from tools.uavnav.tests.arch_fixture import dv_row
        rows = dv_row("DV1", review="2026-01-01") + dv_row("DV2") + dv_row("DV3")
        self.repo.set_trace("", rows)
        self.repo.commit("docs: deviations")
        found = triggers(self.repo, "S5")
        self.assertEqual(len(found), 2, found)

    def test_a_status_regression_is_reported(self):
        self.repo.set_trace("| P1 | §3.1 | S1 | | IMPLEMENTED | `%s` |\n" % self.repo.head()[:8])
        self.repo.commit("docs: implemented")
        self.repo.set_trace("| P1 | §3.1 | S1 | | DESIGNED | |\n")
        self.repo.commit("docs: reopened")
        self.assertEqual(len(triggers(self.repo, "S8")), 1)


if __name__ == "__main__":
    unittest.main()
