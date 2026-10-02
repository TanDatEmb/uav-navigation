from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from tools.quality import check_quality


class QualityGateTest(unittest.TestCase):
    def test_timer_allowlist_accepts_current_and_rejects_new_product_timer(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            allowed = root / "src/pkg/src/allowed.cpp"
            new = root / "src/pkg/src/new.cpp"
            allowed.parent.mkdir(parents=True)
            allowed.write_text("void f() { node.create_wall_timer(period, cb); }\n", encoding="utf-8")
            new.write_text("void g() { node.create_wall_timer(period, cb); }\n", encoding="utf-8")
            allowlist = root / "allow.tsv"
            allowlist.write_text("src/pkg/src/allowed.cpp\tTEST-1\tW4-R4\n", encoding="utf-8")
            failures = check_quality.timer_violations(root, allowlist)
            self.assertEqual(len(failures), 1)
            self.assertIn("new.cpp:1", failures[0])

    def test_doc_layout_requires_each_active_package(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "src/pkg_a").mkdir(parents=True)
            (root / "src/pkg_a/package.xml").write_text(
                "<package format='3'><name>pkg_a</name></package>\n", encoding="utf-8"
            )
            (root / "src/pkg_b").mkdir(parents=True)
            (root / "src/pkg_b/package.xml").write_text(
                "<package format='3'><name>pkg_b</name></package>\n", encoding="utf-8"
            )
            layout = root / "layout.md"
            layout.write_text("```text\nsrc/pkg_a/\n```\n", encoding="utf-8")
            failures = check_quality.doc_layout_violations(root, layout)
            self.assertEqual(failures, ["package missing from repository_layout.md: pkg_b"])

    def test_lizard_ratchet_rejects_new_complex_function(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "src/pkg/new.cpp"
            source.parent.mkdir(parents=True)
            source.write_text("int f() { return 0; }\n", encoding="utf-8")
            baseline = root / "baseline.json"
            baseline.write_text(json.dumps({"schema": 1, "functions": {}}), encoding="utf-8")
            fake = root / "lizard"
            fake.write_text(
                "#!/usr/bin/env bash\n"
                "echo '1,16,1,0,1,loc,src/pkg/new.cpp,f,full,1,1'\n",
                encoding="utf-8",
            )
            fake.chmod(0o755)
            failures = check_quality.lizard_violations(root, baseline, str(fake), [source])
            self.assertEqual(len(failures), 1)
            self.assertIn("CCN>15", failures[0])


if __name__ == "__main__":
    unittest.main()
