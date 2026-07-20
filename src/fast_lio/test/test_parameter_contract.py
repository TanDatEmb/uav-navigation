#!/usr/bin/env python3
"""Test parameter contract compliance for FAST-LIO YAML configs.

This test validates:
- YAML root is fast_lio/ros__parameters only
- No /** global parameters
- No legacy keys (lidar_input.profile, etc.)
- All 7 matching parameters declared
- Profile whitelist validity
"""

import unittest
from pathlib import Path
import yaml


class TestParameterContract(unittest.TestCase):
    """Validate FAST-LIO parameter contract."""

    @classmethod
    def setUpClass(cls):
        cls.config_dir = Path(__file__).parent.parent / "config"

    def _load_yaml(self, filename):
        """Load YAML file and return dict."""
        path = self.config_dir / filename
        with open(path, "r") as f:
            return yaml.safe_load(f)

    def test_yaml_root_is_fast_lio(self):
        """Only fast_lio/ros__parameters allowed at root."""
        for yaml_file in ["common.yaml", "mid360_custom.yaml", "mid360_pointcloud2.yaml", "simulation.yaml"]:
            with self.subTest(file=yaml_file):
                data = self._load_yaml(yaml_file)
                self.assertIn("fast_lio", data, f"{yaml_file}: Missing 'fast_lio' root")
                self.assertIn("ros__parameters", data["fast_lio"], f"{yaml_file}: Missing 'ros__parameters'")
                # No other keys at fast_lio level
                self.assertEqual(set(data.keys()), {"fast_lio"}, f"{yaml_file}: Extra root keys")

    def test_no_global_parameters(self):
        """No /** global parameter scope."""
        for yaml_file in ["common.yaml", "mid360_custom.yaml", "mid360_pointcloud2.yaml", "simulation.yaml"]:
            with self.subTest(file=yaml_file):
                path = self.config_dir / yaml_file
                with open(path, "r") as f:
                    content = f.read()
                self.assertNotIn("/**:", content, f"{yaml_file}: Contains forbidden '/**:' global scope")

    def test_no_legacy_keys(self):
        """No legacy lidar_input.profile."""
        for yaml_file in ["common.yaml", "mid360_custom.yaml", "mid360_pointcloud2.yaml", "simulation.yaml"]:
            with self.subTest(file=yaml_file):
                path = self.config_dir / yaml_file
                with open(path, "r") as f:
                    content = f.read()
                self.assertNotIn("lidar_input.profile", content, f"{yaml_file}: Contains legacy 'lidar_input.profile'")

    def test_common_has_required_keys(self):
        """common.yaml must have all required shared parameters."""
        data = self._load_yaml("common.yaml")
        params = data["fast_lio"]["ros__parameters"]

        # Required key groups
        required_prefixes = [
            "debug.",
            "synchronizer.",
            "noise.",
            "initialization.",
            "estimator.",
            "matching.",
            "measurement.",
            "extrinsic.",
        ]

        for prefix in required_prefixes:
            found = any(k.startswith(prefix) for k in params.keys())
            self.assertTrue(found, f"common.yaml: Missing key group '{prefix}'")

    def test_profiles_have_input_adapter(self):
        """Each profile must declare input.adapter."""
        profiles = [
            ("mid360_custom.yaml", "mid360_custom"),
            ("mid360_pointcloud2.yaml", "mid360_pointcloud2"),
            ("simulation.yaml", "sim_snapshot"),
        ]

        for yaml_file, expected_adapter in profiles:
            with self.subTest(file=yaml_file):
                data = self._load_yaml(yaml_file)
                params = data["fast_lio"]["ros__parameters"]
                self.assertIn("input.mode", params, f"{yaml_file}: Missing 'input.mode'")
                self.assertEqual(params["input.mode"], expected_adapter,
                                 f"{yaml_file}: Expected adapter '{expected_adapter}'")

    def test_profile_whitelist_valid(self):
        """Profile names must be in whitelist."""
        valid_profiles = {"sim", "mid360_custom", "mid360_pointcloud2"}
        yaml_files = {"sim": "simulation.yaml",
                      "mid360_custom": "mid360_custom.yaml",
                      "mid360_pointcloud2": "mid360_pointcloud2.yaml"}

        for profile, yaml_file in yaml_files.items():
            with self.subTest(profile=profile):
                self.assertIn(profile, valid_profiles)
                self.assertTrue((self.config_dir / yaml_file).exists(),
                                f"Profile '{profile}' maps to missing file '{yaml_file}'")

    def test_seven_matching_parameters_present(self):
        """All 7 previously ignored matching parameters must be declared."""
        data = self._load_yaml("common.yaml")
        params = data["fast_lio"]["ros__parameters"]

        required = [
            "matching.knn_count",
            "matching.min_plane_neighbors",
            "matching.max_neighbor_distance_m",
            "matching.max_plane_eigen_ratio",
            "matching.min_second_eigen_ratio",
            "matching.max_neighbor_plane_distance_m",
            "matching.max_point_plane_residual_m",
            "matching.min_effective_correspondences",
        ]

        for key in required:
            self.assertIn(key, params, f"common.yaml: Missing required matching param '{key}'")

    def test_numeric_values_reasonable(self):
        """Sanity check numeric values."""
        data = self._load_yaml("common.yaml")
        params = data["fast_lio"]["ros__parameters"]

        # Noise values positive
        self.assertGreater(params["noise.accelerometer"], 0.0)
        self.assertGreater(params["noise.gyroscope"], 0.0)

        # Iterations positive
        self.assertGreater(params["estimator.max_iterations"], 0)

        # Matching counts reasonable
        self.assertGreaterEqual(params["matching.min_plane_neighbors"], 3)
        self.assertGreaterEqual(params["matching.min_effective_correspondences"], 10)

    def test_backend_locked_to_ieskf(self):
        """Phase 1: estimator.backend must be 'ieskf'."""
        data = self._load_yaml("common.yaml")
        params = data["fast_lio"]["ros__parameters"]
        self.assertEqual(params["estimator.backend"], "ieskf",
                         "Phase 1: estimator.backend must be 'ieskf'")


class TestLaunchWhitelist(unittest.TestCase):
    """Validate launch file profile whitelist."""

    def test_whitelist_matches_files(self):
        """Whitelist profiles must exist as YAML files."""
        config_dir = Path(__file__).parent.parent / "config"

        whitelist = {
            "sim": "simulation.yaml",
            "mid360_custom": "mid360_custom.yaml",
            "mid360_pointcloud2": "mid360_pointcloud2.yaml",
        }

        for profile, yaml_file in whitelist.items():
            with self.subTest(profile=profile):
                self.assertTrue((config_dir / yaml_file).exists(),
                                f"Whitelist profile '{profile}' -> '{yaml_file}' not found")


if __name__ == "__main__":
    unittest.main()
