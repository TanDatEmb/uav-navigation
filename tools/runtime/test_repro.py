#!/usr/bin/env python3

import json
from pathlib import Path
import signal
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import repro
from repro import extract_estimator_state, run_iteration, sha256_path


class ReproducerTest(unittest.TestCase):
    def test_checksum_is_stable_and_names_sensitive(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "a").write_bytes(b"payload")
            first = sha256_path(root)
            self.assertEqual(first, sha256_path(root))
            (root / "a").rename(root / "b")
            self.assertNotEqual(first, sha256_path(root))

    def test_iteration_records_exit_runtime_and_rss(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "iteration"
            summary = run_iteration(
                3, output,
                [sys.executable, "-c", "print('ok')", "{output}"],
                {"mode": "test"}, diagnose=False,
            )
            self.assertEqual(summary["returncode"], 0)
            self.assertFalse(summary["crashed"])
            self.assertGreater(summary["runtime_ns"], 0)
            self.assertTrue((output / "run.json").is_file())
            self.assertTrue((output / "summary.json").is_file())
            self.assertTrue((output / "stdout.log").is_file())
            self.assertTrue((output / "stderr.log").is_file())
            self.assertTrue((output / "backtrace.txt").is_file())
            run = json.loads((output / "run.json").read_text())
            self.assertEqual(run["iteration"], 3)

    def test_signal_is_preserved(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "iteration"
            summary = run_iteration(
                1, output,
                [sys.executable, "-c",
                 "import os, signal; os.kill(os.getpid(), signal.SIGABRT)"],
                {"mode": "test"}, diagnose=False,
            )
            self.assertEqual(summary["signal"], signal.SIGABRT)
            self.assertEqual(summary["signal_name"], "SIGABRT")
            self.assertTrue(summary["crashed"])
            self.assertTrue((output / "core_metadata.json").is_file())

    def test_extracts_last_known_estimator_state(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            (output / "summary.json").write_text(
                json.dumps({
                    "raw_dataset_lidar_count": 7,
                    "successful_correction_count": 5,
                    "map_size_before_insert": 10,
                    "map_size_after_insert": 12,
                })
            )
            (output / "diagnostics.csv").write_text(
                "record_index,crop_performed,crop_removed_count,"
                "map_size_after_maintenance\n9,1,3,8\n"
            )
            state = extract_estimator_state(output)
            self.assertEqual(state["last_input_record"], 9)
            self.assertEqual(state["last_lidar_scan"], 7)
            self.assertEqual(state["map_size_after_insert"], 12)
            self.assertTrue(state["crop_prune_state"]["crop_performed"])

    def test_resolve_dataset_loads_catalog_yaml(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset = "fixture"
            catalog = root / "datasets" / "catalog"
            catalog.mkdir(parents=True)
            (catalog / f"{dataset}.yaml").write_text(
                "id: fixture\nsource:\n  type: rosbag2\n",
                encoding="utf-8",
            )
            config = (
                root
                / "src/navigation_estimator/fast_lio_ros/config/aist.yaml"
            )
            config.parent.mkdir(parents=True)
            config.write_text("fast_lio: {}\n", encoding="utf-8")
            data_root = root / "external"
            bag = data_root / "datasets" / dataset / "lio"
            bag.mkdir(parents=True)

            with mock.patch.object(repro, "ROOT", root), mock.patch.object(
                repro, "data_home", return_value=data_root
            ):
                resolved_bag, resolved_config = repro.resolve_dataset(dataset)

            self.assertEqual(resolved_bag, bag)
            self.assertEqual(resolved_config, config)


if __name__ == "__main__":
    unittest.main()
