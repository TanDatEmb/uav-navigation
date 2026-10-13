"""Tests for tools.uavnav.check_change: commit declarations (pilot scenarios 1, 2, 3 and 4)."""
import unittest

from tools.uavnav import check_change
from tools.uavnav.tests.arch_fixture import Repo

STATE_HPP = "src/estimation/uavnav_lio_core/include/uavnav/lio/state.hpp"
LIFECYCLE = "src/estimation/uavnav_lio_core/src/lifecycle.cpp"
MSG = "src/core/uavnav_interfaces/msg/LioState.msg"


def run(repo):
    found = check_change.check_range(repo.root, repo.rules, repo.base)
    return [str(f) for f in found if f.level == "ERROR"], [str(f) for f in found if f.level != "ERROR"]


class ChangeDeclarationTest(unittest.TestCase):
    def setUp(self):
        self.repo = Repo()
        self.repo.write(STATE_HPP, "enum class LioState { kA, kB };\nstruct S {\n};\n")
        self.repo.write(LIFECYCLE, "int x;\n")
        self.repo.write(MSG, "float64 x\n")
        self.repo.write("docs/architecture/SYSTEM_DESIGN.md", "# Design\n")
        self.repo.base = self.repo.commit("chore: set up")

    def tearDown(self):
        self.repo.close()

    def test_an_ordinary_bug_fix_needs_no_declaration(self):
        self.repo.write("src/estimation/uavnav_lio_core/src/other.cpp", "int a = 1;\n")
        self.repo.commit("fix(lio): off by one")
        self.assertEqual(run(self.repo), ([], []))

    def test_touching_an_invariant_file_needs_a_declaration_and_names_the_invariant(self):
        self.repo.write(LIFECYCLE, "int x = 2;\n")
        self.repo.commit("fix(lio): lifecycle edge")
        errs, _ = run(self.repo)
        self.assertTrue(any("invariant path" in e for e in errs), errs)
        self.repo.write(LIFECYCLE, "int x = 3;\n")
        self.repo.commit("fix(lio): lifecycle edge", "Arch-Level: 0\nDocs-Impact: none")
        errs, notes = run(self.repo)
        self.assertEqual(len(errs), 1)  # only the first, undeclared commit
        self.assertTrue(any("INV-LIO-1" in n for n in notes), notes)

    def test_an_interface_change_must_update_the_contract(self):
        self.repo.write(MSG, "float64 x\nfloat64 y\n")
        self.repo.commit("feat(interfaces): add y")
        self.assertTrue(any("contract path" in e for e in run(self.repo)[0]))

    def test_level_one_cannot_claim_no_docs_impact(self):
        self.repo.write(MSG, "float64 x\nfloat64 y\n")
        self.repo.commit("feat(interfaces): add y", "Arch-Level: 1\nDocs-Impact: none")
        self.assertTrue(any("needs Docs-Impact" in e for e in run(self.repo)[0]))

    def test_level_one_with_updated_docs_passes_and_a_missing_doc_is_an_error(self):
        self.repo.write(MSG, "float64 x\nfloat64 y\n")
        self.repo.write("docs/architecture/SYSTEM_DESIGN.md", "# Design\n\n§6.5 y added\n")
        self.repo.commit("feat(interfaces): add y", "Arch-Level: 1\nDocs-Impact: docs/architecture/SYSTEM_DESIGN.md")
        self.assertEqual(run(self.repo), ([], []))
        self.repo.write(MSG, "float64 x\nfloat64 y\nfloat64 z\n")
        self.repo.commit("feat(interfaces): add z", "Arch-Level: 1\nDocs-Impact: docs/nowhere.md")
        self.assertTrue(any("docs/nowhere.md, which does not exist" in e for e in run(self.repo)[0]))

    def test_pending_documentation_is_a_warning_not_an_error(self):
        self.repo.write(MSG, "float64 x\nfloat64 y\n")
        self.repo.commit("feat(interfaces): add y", "Arch-Level: 1\nDocs-Impact: pending:spec section after T10")
        errs, notes = run(self.repo)
        self.assertEqual(errs, [])
        self.assertTrue(any("documentation debt" in n for n in notes))

    def test_paths_followed_by_pending_text_with_commas_are_split_at_the_marker(self):
        self.repo.write(MSG, "float64 x\nfloat64 y\n")
        self.repo.write("docs/architecture/SYSTEM_DESIGN.md", "# Design\n\n§6.5 y\n")
        self.repo.commit("feat(interfaces): add y", "Arch-Level: 1\nDocs-Impact: docs/architecture/SYSTEM_DESIGN.md, "
                         "pending:TRACEABILITY row, O24 and the plan rule")
        errs, notes = run(self.repo)
        self.assertEqual(errs, [])
        self.assertTrue(any("pending:TRACEABILITY row, O24" in n for n in notes))
        self.repo.write(MSG, "float64 x\nfloat64 y\nfloat64 z\n")
        self.repo.commit("feat(interfaces): add z", "Arch-Level: 1\nDocs-Impact: docs/nowhere.md, pending:later")
        self.assertTrue(any("docs/nowhere.md, which does not exist" in e for e in run(self.repo)[0]))

    def test_a_new_state_enumerator_cannot_be_declared_below_level_two(self):
        self.repo.write(STATE_HPP, "enum class LioState { kA, kB, kC };\nstruct S {\n};\n")
        self.repo.commit("fix(lio): handle c", "Arch-Level: 1\nDocs-Impact: pending:x")
        self.assertTrue(any("adds a state enumerator" in e and "LioState::kC" in e for e in run(self.repo)[0]))

    def test_other_enums_and_test_files_are_not_state_models(self):
        self.repo.write(STATE_HPP, "enum class LioState { kA, kB };\nenum class Reason { kX, kY };\nstruct S {\n};\n")
        self.repo.write("src/estimation/uavnav_lio_core/test/test_fake.cpp", "enum class State { kA, kB };\n")
        self.repo.commit("feat(lio): reason", "Arch-Level: 0\nDocs-Impact: none")
        self.assertEqual(run(self.repo)[0], [])

    def test_level_two_needs_an_architecture_review_that_allows_it(self):
        self.repo.write(STATE_HPP, "enum class LioState { kA, kB, kC };\nstruct S {\n};\n")
        body = "Arch-Level: 2\nDocs-Impact: pending:x\nReview: AR1"
        self.repo.commit("refactor(lio): add c", body)
        self.assertTrue(any("needs `Review: AR<n>`" in e for e in run(self.repo)[0]))
        self.repo.add_ar("AR1", "lio", self.repo.head(), "KEEP")
        self.repo.commit("docs: AR1 keep")
        self.assertTrue(any("result is KEEP" in e for e in run(self.repo)[0]))
        self.repo.add_ar("AR2", "lio", self.repo.head(), "LIMITED_REFACTOR")
        self.repo.write(STATE_HPP, "enum class LioState { kA, kB, kC, kD };\nstruct S {\n};\n")
        self.repo.commit("refactor(lio): add d", "Arch-Level: 2\nDocs-Impact: pending:x\nReview: AR2")
        errs = run(self.repo)[0]
        self.assertFalse(any("AR2" in e for e in errs), errs)

    def test_level_three_needs_an_approved_review_and_an_agreed_decision(self):
        self.repo.add_ar("AR1", "lio", self.repo.head(), "LIMITED_REFACTOR")
        self.repo.commit("docs: AR1")
        self.repo.write(LIFECYCLE, "int x = 9;\n")
        self.repo.commit("feat(lio): move ownership", "Arch-Level: 3\nDocs-Impact: pending:x\nReview: AR1\nDecision: D1")
        self.assertTrue(any("needs APPROVED" in e for e in run(self.repo)[0]))
        self.repo.add_ar("AR3", "lio", self.repo.head(), "APPROVED")
        self.repo.commit("docs: AR3")
        self.repo.write(LIFECYCLE, "int x = 10;\n")
        self.repo.commit("feat(lio): move ownership", "Arch-Level: 3\nDocs-Impact: pending:x\nReview: AR3\nDecision: D2")
        self.assertTrue(any("with status AGREED" in e for e in run(self.repo)[0]))
        self.repo.write(LIFECYCLE, "int x = 11;\n")
        self.repo.commit("feat(lio): move ownership", "Arch-Level: 3\nDocs-Impact: pending:x\nReview: AR3\nDecision: D1")
        self.assertEqual(sum("AR3" in e or "Decision" in e for e in run(self.repo)[0]), 1)  # only the D2 commit

    def test_level_three_needs_the_owner_to_have_confirmed_the_review(self):
        self.repo.add_ar("AR5", "lio", self.repo.head(), "APPROVED", confirmed="chưa")
        self.repo.commit("docs: AR5 written by the executor")
        self.repo.write(LIFECYCLE, "int x = 77;\n")
        self.repo.commit("feat(lio): move ownership", "Arch-Level: 3\nDocs-Impact: pending:x\nReview: AR5\nDecision: D1")
        self.assertTrue(any("confirmed by the owner" in e for e in run(self.repo)[0]))

    def test_pr_body_must_repeat_the_declaration_at_least_as_high_as_commits(self):
        rules = self.repo.rules
        self.assertTrue(check_change.check_body(rules, "no lines here", []))
        self.assertEqual(check_change.check_body(rules, "Arch-Level: 1\nDocs-Impact: docs/x.md", [1, 0]), [])
        self.assertTrue(check_change.check_body(rules, "Arch-Level: 0\nDocs-Impact: none", [1]))
        self.assertTrue(check_change.check_body(rules, "Arch-Level: 7\nDocs-Impact: none", []))


if __name__ == "__main__":
    unittest.main()
