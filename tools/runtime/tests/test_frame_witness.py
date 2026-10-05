import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from frame_witness import build_frame_error_witness


class FrameWitnessProducerTests(unittest.TestCase):
    @staticmethod
    def _witness():
        return {
            "valid": True,
            "lio_localization_epoch": 7,
            "validity_scope": "localization_epoch",
            "provenance": "scenario_truth_frame_pair",
            "T_L_G": {
                "translation_lio_from_gazebo": [1.0, 0.0, 0.0],
                "rotation_matrix_lio_from_gazebo": [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0],
                ],
            },
        }

    def test_frame_error_uses_state_source_time_without_extrapolation(self):
        result = build_frame_error_witness(
            state_source_stamp_ns=1_500_000_000,
            localization_epoch=7,
            truth_frame_witness=self._witness(),
            lio_history=[
                {"source_stamp_ns": 1_000_000_000, "localization_epoch": 7,
                 "position": [0.5, 0.0, 0.0]},
                {"source_stamp_ns": 2_000_000_000, "localization_epoch": 7,
                 "position": [1.5, 0.0, 0.0]},
            ],
            truth_history=[
                {"source_stamp_ns": 1_000_000_000, "position": [0.0, 0.0, 0.0]},
                {"source_stamp_ns": 2_000_000_000, "position": [2.0, 0.0, 0.0]},
            ],
        )

        self.assertIsNotNone(result)
        self.assertEqual(result["frame_error_vector_m"], [1.0, 0.0, 0.0])
        self.assertEqual(result["frame_error_source_stamp_ns"], 1_500_000_000)
        self.assertEqual(result["frame_error_localization_epoch"], 7)
        self.assertEqual(result["frame_error_basis"], "truth_in_lio_frame_minus_lio")

    def test_frame_error_fails_closed_for_missing_or_out_of_range_evidence(self):
        base = dict(
            state_source_stamp_ns=1_500_000_000,
            localization_epoch=7,
            truth_frame_witness=self._witness(),
            lio_history=[{"source_stamp_ns": 1_000_000_000, "localization_epoch": 7,
                          "position": [0.0, 0.0, 0.0]}],
            truth_history=[{"source_stamp_ns": 1_000_000_000, "position": [0.0, 0.0, 0.0]}],
        )
        self.assertIsNone(build_frame_error_witness(**base))
        base["state_source_stamp_ns"] = 500_000_000
        self.assertIsNone(build_frame_error_witness(**base))
        base["state_source_stamp_ns"] = 1_000_000_000
        base["localization_epoch"] = 8
        self.assertIsNone(build_frame_error_witness(**base))


if __name__ == "__main__":
    unittest.main()
