#!/usr/bin/env python3

import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import data


class DataRegistryTest(unittest.TestCase):
    def test_catalog_is_valid_and_ids_are_unique(self) -> None:
        catalog = data.entries()
        self.assertIn("aist-mid360-drive", catalog)
        self.assertIn("m3dgr-mid360-dynamic01", catalog)

    def test_digest_and_blob_path_are_content_addressed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            payload = root / "payload"
            payload.write_bytes(b"dataset")
            expected = hashlib.sha256(b"dataset").hexdigest()
            self.assertEqual(data.digest(payload, "sha256"), expected)
            entry = {
                "download": {
                    "checksum": {"algorithm": "sha256", "value": expected}
                }
            }
            self.assertEqual(
                data.blob_path(root, entry),
                root / "archives" / f"sha256-{expected}",
            )

    def test_tracked_blob_guard_has_no_repository_violations(self) -> None:
        self.assertEqual(data.check_tracked_blobs(), [])

    def test_matrix_sets_and_actions_are_exact(self) -> None:
        self.assertEqual(
            data.matrix_datasets("runtime"),
            ("aist-mid360-drive", "m3dgr-mid360-dynamic01"),
        )
        self.assertEqual(
            data.matrix_datasets("all"),
            (
                "aist-mid360-drive",
                "local-mid360-static01",
                "local-mid360-yaw01",
                "m3dgr-mid360-dynamic01",
                "m3dgr-mid360-corridor02",
                "local-mid360-square01",
            ),
        )
        actions = data.matrix_actions(3, False)
        self.assertEqual(actions.count(("run",)), 4)
        self.assertIn(("replay", "--rate", "0.5"), actions)
        self.assertIn(("replay", "--rate", "1.0"), actions)
        self.assertNotIn(("replay", "--rate", "1.2"), actions)

    def test_identity_ground_truth_metrics_are_zero(self) -> None:
        samples = [
            (0, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)),
            (10, (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)),
        ]
        metrics = data.trajectory_metrics(samples, samples, 0)
        self.assertEqual(metrics["ate_translation_rmse_m"], 0.0)
        self.assertEqual(metrics["rpe_translation_rmse_m"], 0.0)
        self.assertEqual(metrics["rpe_rotation_rmse_rad"], 0.0)
        self.assertEqual(metrics["trajectory_coverage"], 1.0)
        self.assertFalse(metrics["thresholds_applied"])


if __name__ == "__main__":
    unittest.main()
