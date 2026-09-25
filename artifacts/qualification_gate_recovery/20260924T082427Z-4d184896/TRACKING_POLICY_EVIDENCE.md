# Tracking policy evidence

`tools/runtime/evaluation.py:878-880,1342-1360` reads `tracking_coverage_policy` from the scenario/config; if absent it emits `TRACKING_COVERAGE_POLICY_UNAVAILABLE`. A1/A2/A3 scenario files lack the versioned policy. Source and test fixtures contain example numbers, not an approved `long_featured` tracking-off contract. No threshold was added in this branch. `qualification_eligible` must remain false until an approved policy is pinned with provenance and its requested/effective configuration is verified.
