import tempfile
import unittest
from pathlib import Path

import yaml

from tools.safety_profile.profile import load_profile


ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "config/safety_profile/sitl_current_as_is.yaml"


class ProfileLoaderTest(unittest.TestCase):
    def test_load_computes_derived_values_and_stable_hash(self):
        profile = load_profile(SOURCE)
        second = load_profile(SOURCE)
        self.assertEqual(profile.sha256, second.sha256)
        self.assertEqual(profile.hash64, profile.sha256[:16])
        self.assertEqual(profile.values["geometry.planning_radius_sum_m"], 0.8)
        self.assertEqual(profile.values["timing.minimum_main_reserve_s"], 0.6)
        self.assertEqual(profile.values["envelope.max_yaw_acceleration_rad_s2"], 2.0)
        self.assertEqual(profile.values["envelope.min_thrust_acceleration_m_s2"], 6.0)
        self.assertEqual(profile.values["envelope.max_thrust_acceleration_m_s2"], 25.0)

    def test_missing_required_key_fails(self):
        document = yaml.safe_load(SOURCE.read_text(encoding="utf-8"))
        del document["timing"]["planner_period_s"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing.yaml"
            path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "planner_period_s"):
                load_profile(path)

    def test_out_of_domain_value_fails(self):
        document = yaml.safe_load(SOURCE.read_text(encoding="utf-8"))
        document["geometry"]["vehicle_radius_m"]["value"] = -0.1
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "invalid.yaml"
            path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "vehicle_radius_m"):
                load_profile(path)


if __name__ == "__main__":
    unittest.main()
