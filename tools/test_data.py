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
        self.assertEqual(set(catalog), {"aist-mid360-drive"})

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

    def test_percentile_uses_nearest_rank_index(self) -> None:
        self.assertEqual(data.percentile([4.0, 1.0, 3.0, 2.0], 0.5), 3.0)
        self.assertIsNone(data.percentile([], 0.95))


if __name__ == "__main__":
    unittest.main()
