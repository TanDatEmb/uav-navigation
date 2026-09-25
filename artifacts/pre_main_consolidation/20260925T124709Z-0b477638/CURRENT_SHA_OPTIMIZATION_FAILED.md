# Current-SHA exact OptimizationFailed regression

Session: `/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260925T144137-138251`. The focused harness returned `BLOCKED` for the whole mission; that aggregate must not be mistaken for failure of the injected seam. Reanalysis with ROS Jazzy and the built overlay verified one exact injection at the real planner-result boundary.

| Assertion | Result | Evidence |
|---|---|---|
| exact status `kOptimizationFailed` | PASS | original status captured, injected status `6` |
| injection arms/applies exactly once | PASS | semantic hot-handoff trigger |
| desired successor differs from active predecessor | PASS | active request 2, generation 6; desired request 3 |
| normal classifier returns `RetainCommittedCommand` | PASS | runtime classifier witness |
| HG-023 retained validator executes and passes | PASS | incumbent/world/state/anchor/suffix checks in trace |
| predecessor samples/admissions continue | PASS | four predecessor admissions after fault |
| publication/admission gaps stay below unchanged 100 ms lease | PASS | publication max 20.091 ms; adapter max 20.129 ms around fault |
| successor eventually admitted | PASS | 86.405 ms after fault |

A later PX4 Hold occurred about 52 seconds after injection, after generations and mission state had advanced to terminal STOP recovery. It followed an independent final bridge/tracking failure (`anchor_error=0.253 m`, projected 0.289 m, limit 0.250 m, relative anchor speed 0.441 m/s, fresh state). It was not caused by the injected result. The full mission did not complete; the fault-seam assertion is positive, whole-run completion assertion is negative. No safety rule was bypassed and no source behavior was changed for this fault.

Raw session directory and sizes/SHA256 are listed in `RAW_SESSION_MANIFEST.csv` and indexed by `EVIDENCE_INDEX.md`. This is current-source focused runtime evidence, not flight qualification.
