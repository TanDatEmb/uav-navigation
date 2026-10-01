from __future__ import annotations

import subprocess
import tempfile
import os
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
GATE = ROOT / "tools" / "gate.sh"


class GateScriptTest(unittest.TestCase):
    def _write_python_stub(self, directory: Path, failing_step: str | None = None) -> Path:
        stub = directory / "python-stub"
        failure = failing_step or ""
        stub.write_text(
            "#!/usr/bin/env bash\n"
            "if [[ \"$1\" == \"--version\" ]]; then exit 0; fi\n"
            f"if [[ -n \"{failure}\" && \"$1\" == *\"{failure}\"* ]]; then exit 7; fi\n"
            "exit 0\n",
            encoding="utf-8",
        )
        stub.chmod(0o755)
        return stub

    def _init_fixture_repo(self, directory: Path) -> None:
        subprocess.run(["git", "init", "-q"], cwd=directory, check=True)
        subprocess.run(["git", "config", "user.name", "gate"], cwd=directory, check=True)
        subprocess.run(
            ["git", "config", "user.email", "gate@example.invalid"],
            cwd=directory,
            check=True,
        )
        (directory / "seed.txt").write_text("seed\n", encoding="utf-8")
        subprocess.run(["git", "add", "seed.txt"], cwd=directory, check=True)
        subprocess.run(["git", "commit", "-qm", "fixture"], cwd=directory, check=True)

    def test_static_gate_rejects_each_validator_failure(self) -> None:
        validators = (
            "validate_runtime_safety_ledger.py",
            "check_mission_authority_cut.py",
            "check_citations.py",
            "check_dependency_direction.py",
        )
        for validator in validators:
            with self.subTest(validator=validator), tempfile.TemporaryDirectory() as directory:
                repo = Path(directory)
                self._init_fixture_repo(repo)
                gate_copy = repo / "tools" / "gate.sh"
                gate_copy.parent.mkdir()
                gate_copy.write_bytes(GATE.read_bytes())
                gate_copy.chmod(0o755)
                python_stub = self._write_python_stub(repo, validator)
                result = subprocess.run(
                    [str(gate_copy), "static"],
                    cwd=repo,
                    env={**os.environ, "PYTHON": str(python_stub)},
                    text=True,
                    capture_output=True,
                    check=False,
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("GATE_V3_RESULT=FAIL", result.stdout)

    def test_static_gate_passes_current_tree(self) -> None:
        result = subprocess.run(
            [str(GATE), "static"], cwd=ROOT, text=True, capture_output=True
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_static_gate_rejects_whitespace_error_in_git_repo(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            repo = Path(directory)
            (repo / "tools").mkdir()
            gate_copy = repo / "tools" / "gate.sh"
            gate_copy.write_bytes(GATE.read_bytes())
            gate_copy.chmod(0o755)
            subprocess.run(["git", "init", "-q"], cwd=repo, check=True)
            bad = repo / "bad.txt"
            bad.write_text("clean\n")
            subprocess.run(["git", "add", "."], cwd=repo, check=True)
            subprocess.run(
                [
                    "git",
                    "-c",
                    "user.name=gate",
                    "-c",
                    "user.email=gate@example.invalid",
                    "commit",
                    "-qm",
                    "fixture",
                ],
                cwd=repo,
                check=True,
            )
            bad.write_text("line with trailing spaces  \n")
            python_stub = self._write_python_stub(repo)
            result = subprocess.run(
                [str(gate_copy), "static"],
                cwd=repo,
                env={**os.environ, "PYTHON": str(python_stub)},
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("trailing whitespace", result.stdout + result.stderr)
            self.assertNotIn("GATE_V3_RESULT=PASS", result.stdout)
            self.assertIn("GATE_V3_RESULT=FAIL", result.stdout)

    def test_ros_gate_rejects_build_failure(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            repo = Path(directory)
            self._init_fixture_repo(repo)
            subprocess.run(
                ["git", "update-ref", "refs/remotes/origin/main", "HEAD"],
                cwd=repo,
                check=True,
            )
            gate_copy = repo / "tools" / "gate.sh"
            gate_copy.parent.mkdir()
            gate_copy.write_bytes(GATE.read_bytes())
            gate_copy.chmod(0o755)
            bin_dir = repo / "bin"
            bin_dir.mkdir()
            colcon = bin_dir / "colcon"
            colcon.write_text(
                "#!/usr/bin/env bash\n"
                "case \"$1\" in\n"
                "  list) echo 'demo src/demo' ;;\n"
                "  build) exit 9 ;;\n"
                "  *) exit 0 ;;\n"
                "esac\n",
                encoding="utf-8",
            )
            colcon.chmod(0o755)
            result = subprocess.run(
                [str(gate_copy), "ros"],
                cwd=repo,
                env={
                    **os.environ,
                    "PACKAGES": "demo",
                    "PATH": f"{bin_dir}:{os.environ['PATH']}",
                },
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("GATE_V3_RESULT=FAIL", result.stdout)


if __name__ == "__main__":
    unittest.main()
