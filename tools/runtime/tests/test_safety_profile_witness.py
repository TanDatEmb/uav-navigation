import sys
from pathlib import Path
import unittest

RUNTIME = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RUNTIME))

import runner  # noqa: E402


class SafetyProfileWitnessTests(unittest.TestCase):
    def test_parses_empty_mismatch_witness(self):
        witness = runner._parse_safety_profile_witness(
            "[INFO] SAFETY_PROFILE_WITNESS hash=0123456789abcdef keys=19 mismatches=[]\n",
            "mapping",
        )
        self.assertEqual(witness, {
            "hash": "0123456789abcdef", "keys": 19, "mismatches": []
        })

    def test_preserves_mismatches_without_failure(self):
        witness = runner._parse_safety_profile_witness(
            "SAFETY_PROFILE_WITNESS hash=0123456789abcdef keys=2 "
            "mismatches=[timing.solve_deadline_s=0.1/0.08,geometry.radius=0.3/0.25]",
            "mapping",
        )
        self.assertEqual(witness["mismatches"], [
            "timing.solve_deadline_s=0.1/0.08", "geometry.radius=0.3/0.25"
        ])

    def test_requires_exactly_one_witness(self):
        for log in ("", "SAFETY_PROFILE_WITNESS hash=0123456789abcdef keys=0 mismatches=[]\n"
                    "SAFETY_PROFILE_WITNESS hash=0123456789abcdef keys=0 mismatches=[]\n"):
            with self.subTest(log=log), self.assertRaises(ValueError):
                runner._parse_safety_profile_witness(log, "mapping")

    def test_records_profile_load_error_without_fabricating_a_witness(self):
        record = runner._parse_safety_profile_witness(
            "[ERROR] [t] [navigation_runtime_node]: "
            "SAFETY_PROFILE_LOAD_ERROR reason=unavailable\n",
            "mapping", "navigation_runtime_node",
        )
        self.assertEqual(record, {"status": "unavailable", "reason": "unavailable"})

    def test_selects_witness_by_ros_logger_when_launch_log_is_shared(self):
        log = (
            "[INFO] [t] [fast_lio]: SAFETY_PROFILE_WITNESS "
            "hash=aaaaaaaaaaaaaaaa keys=11 mismatches=[]\n"
            "[INFO] [t] [px4_external_odometry_bridge]: SAFETY_PROFILE_WITNESS "
            "hash=bbbbbbbbbbbbbbbb keys=6 mismatches=[]\n"
        )
        witness = runner._parse_safety_profile_witness(log, "lio", "fast_lio")
        self.assertEqual(witness["hash"], "aaaaaaaaaaaaaaaa")


if __name__ == "__main__":
    unittest.main()
