import tempfile
import unittest
from pathlib import Path
import sys
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import planner_baseline


class PlannerBaselineTest(unittest.TestCase):
    def test_classify_path_preserves_safety_layer_boundaries(self) -> None:
        self.assertEqual(planner_baseline.classify_path("src/planning/foo.cpp"), "planner")
        self.assertEqual(planner_baseline.classify_path("src/mapping/foo.cpp"), "mapping")
        self.assertEqual(planner_baseline.classify_path("src/runtime/foo.cpp"), "runtime_or_integration")
        self.assertEqual(planner_baseline.classify_path("tools/runtime/report.py"), "observability_or_validation")
        self.assertEqual(planner_baseline.classify_path("src/planning/test/test.cpp"), "tests")

    def test_config_snapshot_is_explicitly_not_generated_runtime_config(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "snapshot.yaml"
            planner_baseline._write_config_snapshot(
                output,
                [{"path": "config.yaml", "sha256": "abc", "content": "a: 1\n"}],
                [],
            )
            content = output.read_text(encoding="utf-8")
            self.assertIn("authority: source_snapshot_not_generated_runtime_config", content)
            self.assertIn("path: config.yaml", content)
            self.assertIn("      a: 1", content)

    def test_changed_csv_includes_category(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "changed.csv"
            planner_baseline._write_changed_csv(
                output, [{"status": "M", "path": "src/planning/planner.cpp"}]
            )
            self.assertEqual(
                output.read_text(encoding="utf-8").splitlines(),
                ["status,category,path", "M,planner,src/planning/planner.cpp"],
            )

    def test_porcelain_v2_parser_keeps_modified_and_untracked_paths(self) -> None:
        with mock.patch.object(
            planner_baseline,
            "_git",
            return_value=(
                "1 .M N... 100644 100644 100644 "
                "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa "
                "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb "
                "src/planning/planner.cpp\n"
                "? tools/runtime/new_tool.py\n"
            ),
        ):
            self.assertEqual(
                planner_baseline._status_records(),
                [
                    {"status": ".M", "path": "src/planning/planner.cpp"},
                    {"status": "??", "path": "tools/runtime/new_tool.py"},
                ],
            )


if __name__ == "__main__":
    unittest.main()
