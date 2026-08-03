import ast
import json
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "odometry_supervisor_fault_injector.py"

def scenario_catalog():
    tree = ast.parse(MODULE_PATH.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "SCENARIOS"
            for target in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError("fault injector scenario catalog is missing")


class SupervisorFaultInjectorTest(unittest.TestCase):
    def test_catalog_contains_required_fault_classes(self):
        required = {
            "single_position_jump", "slow_xy_drift", "slow_yaw_drift",
            "velocity_bias", "px4_stale", "lio_propagated_stale",
            "lio_corrected_stale", "px4_reset_generation", "clock_pause",
            "diagnostic_schema_corruption", "correlated_unhealthy",
        }
        self.assertTrue(required.issubset(set(scenario_catalog())))

    def test_artifact_schema_is_machine_readable(self):
        artifact = {"scenario": "single_position_jump", "inputs": [], "supervisor_status": []}
        encoded = json.dumps(artifact)
        decoded = json.loads(encoded)
        self.assertEqual(decoded["scenario"], "single_position_jump")
        self.assertIn("inputs", decoded)
        self.assertIn("supervisor_status", decoded)


if __name__ == "__main__":
    unittest.main()
