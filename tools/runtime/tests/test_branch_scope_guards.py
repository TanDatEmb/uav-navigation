"""Regression tests for explicit, pre-merge-only branch scope checks."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
GUARD_NAMES = (
    "check_qualification_evidence_scope.py",
    "check_software_qualification_scope.py",
)


def git(cwd: Path, *args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=cwd, text=True).strip()


class BranchScopeGuardTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.repo = Path(self.temp.name)
        (self.repo / "tools").mkdir()
        for name in GUARD_NAMES:
            (self.repo / "tools" / name).write_bytes((ROOT / "tools" / name).read_bytes())
        git(self.repo, "init", "-q", "--initial-branch=main")
        git(self.repo, "config", "user.name", "Scope Guard Test")
        git(self.repo, "config", "user.email", "scope-guard@example.invalid")
        self.branch = git(self.repo, "branch", "--show-current")
        (self.repo / "src").mkdir()
        (self.repo / "src" / "product.cpp").write_text("int product_value = 1;\n")
        (self.repo / "docs").mkdir()
        (self.repo / "docs" / "scope.md").write_text("baseline\n")
        git(self.repo, "add", ".")
        git(self.repo, "commit", "-qm", "baseline")
        self.base = git(self.repo, "rev-parse", "HEAD")

    def tearDown(self) -> None:
        self.temp.cleanup()

    def run_guard(self, name: str, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(self.repo / "tools" / name), *args],
            cwd=self.repo, text=True, capture_output=True, check=False,
        )

    def assert_scope_pass(self, name: str) -> None:
        result = self.run_guard(name, "--base", self.base)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_explicit_reachable_base_passes_when_product_source_is_unchanged(self) -> None:
        (self.repo / "docs" / "scope.md").write_text("updated documentation\n")
        git(self.repo, "add", "docs/scope.md")
        git(self.repo, "commit", "-qm", "documentation only")
        for name in GUARD_NAMES:
            with self.subTest(guard=name):
                self.assert_scope_pass(name)

    def test_explicit_reachable_base_fails_when_protected_source_changes(self) -> None:
        (self.repo / "src" / "product.cpp").write_text("int product_value = 2;\n")
        git(self.repo, "add", "src/product.cpp")
        git(self.repo, "commit", "-qm", "product change")
        for name in GUARD_NAMES:
            with self.subTest(guard=name):
                result = self.run_guard(name, "--base", self.base)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("src/product.cpp", result.stdout + result.stderr)

    def test_missing_commit_object_has_typed_failure(self) -> None:
        absent = "f" * 40
        for name in GUARD_NAMES:
            with self.subTest(guard=name):
                result = self.run_guard(name, "--base", absent)
                self.assertEqual(result.returncode, 2)
                self.assertIn("SCOPE_BASE_UNAVAILABLE", result.stdout)
                self.assertNotIn("Traceback", result.stderr)

    def test_invalid_revision_has_clean_failure(self) -> None:
        for name in GUARD_NAMES:
            with self.subTest(guard=name):
                result = self.run_guard(name, "--base", "not-a-commit")
                self.assertEqual(result.returncode, 2)
                self.assertIn("SCOPE_BASE_UNAVAILABLE", result.stdout)
                self.assertNotIn("Traceback", result.stderr)

    def test_non_ancestor_commit_is_rejected(self) -> None:
        git(self.repo, "checkout", "-qb", "other-line")
        (self.repo / "docs" / "scope.md").write_text("other line\n")
        git(self.repo, "add", "docs/scope.md")
        git(self.repo, "commit", "-qm", "other line")
        non_ancestor = git(self.repo, "rev-parse", "HEAD")
        git(self.repo, "checkout", "-q", self.branch)
        for name in GUARD_NAMES:
            with self.subTest(guard=name):
                result = self.run_guard(name, "--base", non_ancestor)
                self.assertEqual(result.returncode, 2)
                self.assertIn("SCOPE_BASE_INVALID", result.stdout)

    def test_base_is_required_by_cli(self) -> None:
        for name in GUARD_NAMES:
            with self.subTest(guard=name):
                result = self.run_guard(name)
                self.assertEqual(result.returncode, 2)
                self.assertIn("--base", result.stderr)

    def test_permanent_gate_registry_works_without_historical_baseline_object(self) -> None:
        gate = ROOT / "tools" / "runtime" / "pre_main_gate.py"
        source = gate.read_text()
        with tempfile.TemporaryDirectory() as shallow_dir:
            shallow = Path(shallow_dir)
            (shallow / "pre_main_gate.py").write_text(source)
            git(shallow, "init", "-q", "--initial-branch=main")
            git(shallow, "config", "user.name", "Shallow Gate Test")
            git(shallow, "config", "user.email", "shallow-gate@example.invalid")
            git(shallow, "add", "pre_main_gate.py")
            git(shallow, "commit", "-qm", "single shallow gate commit")
            old_base = "0b477638d21ce60cdb42ed85fb7c2d568bf500ed"
            missing = subprocess.run(
                ["git", "cat-file", "-e", f"{old_base}^{{commit}}"],
                cwd=shallow, capture_output=True, check=False,
            )
            self.assertNotEqual(missing.returncode, 0)
            spec = importlib.util.spec_from_file_location("pre_main_gate", shallow / "pre_main_gate.py")
            self.assertIsNotNone(spec)
            assert spec is not None and spec.loader is not None
            module = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = module
            try:
                spec.loader.exec_module(module)
                guards = module.static_guard_names()
            finally:
                sys.modules.pop(spec.name, None)
            self.assertNotIn(GUARD_NAMES[0], guards)
            self.assertNotIn(GUARD_NAMES[1], guards)
            self.assertNotIn(old_base, source)


if __name__ == "__main__":
    unittest.main()
