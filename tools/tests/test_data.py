#!/usr/bin/env python3

import hashlib
import csv
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
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

    def test_report_uses_explicit_stage_flags_and_excludes_rejections(self) -> None:
        fixture = """reason,synchronized,deskew_attempted,deskew_applied,correction_attempted,correction_succeeded,map_update_performed,accepted_residuals,residual_rms,iterations,total_processing_us,ikfom_update_us,map_insert_crop_us,map_maintenance_us,map_points
,1,1,1,1,1,1,12,0.2,3,100,40,20,5,10
OVERLAPPING_LIDAR_INTERVAL,0,0,0,0,0,0,0,0,0,0,0,0,0,10
MISSING_BRACKET,0,0,0,0,0,0,0,0,0,0,0,0,0,10
MAP_ONLY,1,1,0,0,0,1,0,0,0,30,0,7,2,12
"""
        rows = list(csv.DictReader(fixture.splitlines()))
        summary = {
            "synchronized_group_count": 2,
            "correction_attempt_count": 1,
            "successful_correction_count": 1,
            "failed_correction_count": 0,
            "overlap_rejected_count": 1,
            "missing_bracket_rejected_count": 1,
            "invalid_timestamp_rejected_count": 0,
            "raw_dataset_lidar_count": 4,
            "core_accepted_lidar_count": 4,
            "core_rejected_lidar_count": 0,
            "dataset_duration_seconds": 2,
            "effective_corrected_output_rate_hz": 0.5,
            "wall_runtime_us": 1000,
            "map_point_count": 12,
        }
        report = data.build_data_report(
            "fixture", Path("/tmp/fixture"), summary, rows
        )
        self.assertEqual(report["deskew_count"], 1)
        self.assertEqual(report["ikfom_update_us"]["sample_count"], 1)
        self.assertEqual(report["map_update_us"]["sample_count"], 2)
        self.assertEqual(report["total_processing_us"]["sample_count"], 2)
        self.assertEqual(
            report["rejection_reason_histogram"]["OVERLAPPING_LIDAR_INTERVAL"],
            1,
        )

    def test_report_rejects_inconsistent_correction_counts(self) -> None:
        rows = [{
            "reason": "", "synchronized": "1",
            "correction_attempted": "1", "correction_succeeded": "1",
            "deskew_applied": "1", "map_update_performed": "0",
            "accepted_residuals": "1", "residual_rms": "0.1",
            "iterations": "1", "total_processing_us": "1",
            "ikfom_update_us": "1", "map_insert_crop_us": "0",
            "map_maintenance_us": "0", "map_points": "1",
        }]
        summary = {
            "synchronized_group_count": 1,
            "correction_attempt_count": 2,
            "successful_correction_count": 1,
            "failed_correction_count": 1,
        }
        with self.assertRaises(data.DataError):
            data.build_data_report("fixture", Path("/tmp/fixture"), summary, rows)


if __name__ == "__main__":
    unittest.main()
