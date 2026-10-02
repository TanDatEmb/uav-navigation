import csv
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / "config/safety_profile/sitl_current_as_is.yaml"
ORACLE = ROOT / "docs/refactor/WP-A6/constants.csv"


class ProfileSourceContractTest(unittest.TestCase):
    def test_profile_records_metadata_and_oracle_coverage(self):
        profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
        self.assertEqual(profile["profile"]["name"], "sitl_current_as_is")
        flattened = {}

        def visit(node, prefix=""):
            for key, value in node.items():
                path = f"{prefix}.{key}" if prefix else key
                if isinstance(value, dict) and "value" not in value:
                    visit(value, path)
                else:
                    for required in ("value", "unit", "qualification_status",
                                     "owners", "overlay", "sources",
                                     "superseded_sources"):
                        self.assertIn(required, value, f"{path}.{required}")
                    self.assertIn(value["qualification_status"],
                                  {"PROVISIONAL", "ACTIVE", "NOT_EVALUABLE"})
                    self.assertIn(value["overlay"], {"allowed", "forbidden"})
                    flattened[path] = value

        for section, values in profile.items():
            if section not in {"profile", "schema_version"}:
                visit(values, section)

        with ORACLE.open(encoding="utf-8", newline="") as stream:
            included = {row["proposed_name"] for row in csv.DictReader(stream)}
        represented = {entry.get("oracle_name", path.split(".")[-1])
                       for path, entry in flattened.items()}
        unresolved = set(profile["profile"].get("open_questions", []))
        excluded = {"numerical_roundoff_epsilons",
                    "world_observation_fault_duration_ms"}
        self.assertEqual((included - excluded) - represented, unresolved)
        self.assertEqual(unresolved, {"minimum_thrust_n", "maximum_thrust_n"})
        for forbidden in ("numerical_roundoff_epsilons",
                          "world_observation_fault_duration_ms"):
            self.assertNotIn(forbidden, represented)
            self.assertNotIn(forbidden, unresolved)


if __name__ == "__main__":
    unittest.main()
