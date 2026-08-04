import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import data


class DataRegistryTest(unittest.TestCase):
    def test_catalog_is_valid_and_ids_are_unique(self) -> None:
        self.assertEqual(set(data.entries()), {"aist-mid360-drive"})

    def test_digest_and_blob_path_are_content_addressed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            payload = root / "payload"
            payload.write_bytes(b"dataset")
            expected = hashlib.sha256(b"dataset").hexdigest()
            self.assertEqual(data.digest(payload, "sha256"), expected)
            entry = {"id": "fixture", "download": {"checksum": {"algorithm": "sha256", "value": expected}}}
            self.assertEqual(data.blob_path(root, entry), root / "archives" / f"sha256-{expected}")

    def test_tracked_blob_guard_has_no_repository_violations(self) -> None:
        self.assertEqual(data.check_tracked_blobs(), [])

    def test_frame_normalization_changes_only_header_frame_id(self) -> None:
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="legacy_sensor", stamp=(12, 34)),
            payload=b"serialized-payload",
        )
        source = data.normalize_message_frame(message, data.CANONICAL_LIDAR_FRAME)
        self.assertEqual(source, "legacy_sensor")
        self.assertEqual(message.header.frame_id, "livox_frame")
        self.assertEqual(message.header.stamp, (12, 34))
        self.assertEqual(message.payload, b"serialized-payload")

    def test_prepared_schema_rejects_old_provenance(self) -> None:
        with self.assertRaises(data.DataError):
            data.prepared_status_schema({"schema_version": 1})

    def test_runtime_dataset_config_is_canonical(self) -> None:
        self.assertEqual(data.config_path("aist-mid360-drive"), Path(data.ROOT) / "config/runtime/dataset.yaml")


if __name__ == "__main__":
    unittest.main()
