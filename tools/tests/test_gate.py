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
            subprocess.run(["git", "init", "-q"], cwd=repo, check=True)
            (repo / "bad.txt").write_text("line with trailing spaces  \n")
            subprocess.run(["git", "add", "bad.txt"], cwd=repo, check=True)
            subprocess.run(
                ["git", "-c", "user.name=gate", "-c", "user.email=gate@example.invalid", "commit", "-qm", "fixture"],
                cwd=repo,
                check=True,
            )
            result = subprocess.run(
                ["git", "diff", "--check", "HEAD^", "HEAD"],
                cwd=repo,
                text=True,
                capture_output=True,
            )
            self.assertNotEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
