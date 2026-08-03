from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "performance"))
import p0_8_provenance as provenance


def identity(*, dirty: bool, submodule_dirty: bool = False, sha: str = "abc") -> dict:
    return {
        "full_sha": sha,
        "short_sha": sha[:7],
        "branch": "test",
        "dirty": dirty,
        "status": [" M tracked.txt"] if dirty else [],
        "submodules": {
            "src/external/px4_msgs": {
                "sha": "px4",
                "dirty": submodule_dirty,
                "status": ["?? generated"] if submodule_dirty else [],
            }
        },
    }


class ProvenancePolicyTest(unittest.TestCase):
    def test_development_records_dirty_state_but_is_not_qualification_eligible(self):
        payload = {"git": identity(dirty=True, submodule_dirty=True)}
        result = provenance.validate_provenance(payload, policy="development")
        self.assertFalse(result["qualification_clean"])
        self.assertFalse(result["acceptance_eligible"])
        self.assertEqual(result["provenance_policy"], "development")

    def test_qualification_rejects_dirty_worktree(self):
        with self.assertRaises(RuntimeError):
            provenance.validate_provenance(
                {"git": identity(dirty=True)}, policy="qualification"
            )

    def test_expected_sha_is_enforced_when_supplied(self):
        clean = {"git": identity(dirty=False, sha="actual")}
        with self.assertRaises(RuntimeError):
            provenance.validate_provenance(
                clean, policy="development", expected_sha="expected"
            )
        result = provenance.validate_provenance(
            {"git": identity(dirty=False, sha="actual")},
            policy="qualification", expected_sha="actual",
        )
        self.assertTrue(result["acceptance_eligible"])


if __name__ == "__main__":
    unittest.main()
