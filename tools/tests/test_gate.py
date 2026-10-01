from __future__ import annotations

import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
GATE = ROOT / "tools" / "gate.sh"


class GateScriptTest(unittest.TestCase):
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
            result = subprocess.run(
                [str(gate_copy), "static"],
                cwd=repo,
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("trailing whitespace", result.stdout + result.stderr)
            self.assertNotIn("GATE_V3_RESULT=PASS", result.stdout)


if __name__ == "__main__":
    unittest.main()
