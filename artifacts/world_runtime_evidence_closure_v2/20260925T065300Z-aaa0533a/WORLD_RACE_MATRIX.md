# World race matrix

| Case | Evidence in this branch | Result | Evidence class |
|---|---|---|---|
| R1: E1 recertification returns after E2 activation | `TestExecutionAuthority.WorldRefreshForE1CannotMutateE2AfterBarrierSupersession` | Old callback is `kSuperseded`; E2 remains active and W10 remains published. | deterministic component / barrier |
| R2: P1 recertification returns after P2 staged replacement | `TestExecutionAuthority.WorldRefreshForP1CannotReplaceBarrierInstalledP2` | Old callback is `kSuperseded`; P2 and active predecessor remain. | deterministic component / barrier |
| R3: localization reset while old mapping callback is blocked | `NavigationRuntimeEpochReset.GoalAcceptedDuringDrainSurvivesOldMappingCallback` and `TerminalDuringDrainCannotBeResurrectedByNewWorld` in `test_navigation_runtime_shutdown.cpp` | Old world callback cannot reopen readiness or resurrect old terminal state; new epoch world is required before ready. | deterministic runtime-component barrier |
| R4: suspended E1 recertification returns after E2 cutover | `TestExecutionAuthority.SuspendedE1WorldRefreshCannotResumeAfterE2Cutover` | Old callback is `kSuperseded`; E2 remains active and available; W10 remains published. | deterministic component / barrier |
| Same source stamp, newer revision | `WorldSnapshotStore` tests in `test_world_snapshot_store.cpp` | Store allows a newer immutable revision at the same source stamp under the existing contract. | deterministic unit |
| W3 unsafe world revision | Existing exact candidate validation and owner transaction tests; no new runtime fault run. | Component coverage only; no integrated invalidation trace. | component/source |
| W7 source-time expiry | Existing temporal assessment and command suspension code; no isolated source fault. | End-to-end not run. | source only |
| W8 fresh world recovery after isolated stale stream | No isolated fault/recovery run. | Open. | not run |

The three new tests use latches to hold preparation outside the owner transaction while a competing execution mutation commits. They do not use sleeps to create ordering. The test helper executes the production `ExecutionAuthority` publication transaction path.
