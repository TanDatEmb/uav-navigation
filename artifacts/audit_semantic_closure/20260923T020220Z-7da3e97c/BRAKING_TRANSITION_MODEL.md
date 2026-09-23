# Braking transition model

`tests/braking_model.py` explores an explicitly **hypothetical** two-stage reducer: `tracking`, `recoverable`, `committed`, `stopped`. A `nominal` event escapes `recoverable` only with certificate, continuity and lease; `committed` ignores late nominal/map events until `measured_stop`. Sample role remains orthogonal. The test checks invariant shape, not parity with TARGET.

| Transition | Required physical witness | Authority/certificate | Open issue |
|---|---|---|---|
| tracking→recoverable | measured P/V/A and current active brake sample | new brake starts continuously | exact trigger vs current adapter `Braking` |
| recoverable→tracking | current measured P/V/A, error and remaining stop margin | new latest-world candidate, continuous splice, live lease | representative evidence that planner can produce it |
| recoverable→committed | explicit safety commit | certified BACKUP/emergency/STOP identity | irreversible point across processes |
| committed→stopped | measured ≤0.15 m/s at certified endpoint | preserved suffix/stop identity | stop window/timeout ownership |
| stopped→tracking | fresh measured-state PlanFromRest | newly admitted MAIN | existing runtime allows this |
| any unsafe certification failure→PX4 transfer | no safe command | Hold protocol, not silent command absence | Blocker C/D |

`FACT_FROM_TARGET_CODE`: the repository product adapter calls `onNativeSafetyTrajectoryObserved`/`onNativeTrajectoryReady`, not `onTrajectory`, so the abstract two-layer mismatch test does **not** establish a reachable product mismatch. `INFERENCE`: if future rolling recovery is desired, it needs a pre-commit state with a continuity/world/lease proof; once runtime safety role is committed, existing one-way policy applies. Decision D remains for target design; no production variant should be implemented from this model.
