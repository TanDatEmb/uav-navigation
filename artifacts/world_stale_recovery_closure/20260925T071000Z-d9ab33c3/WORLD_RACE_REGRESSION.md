# World race regression

Post-change six-package CTest run passed 37 CTest targets / 898 gtest cases, zero failed or skipped.

| Race | Test(s) | Result |
|---|---|---|
| R1: E1 recertification returns after E2 activates | `WorldRefreshForE1CannotMutateE2AfterBarrierSupersession` | PASS |
| R2: P1 recertification returns after P2 staged replacement | `WorldRefreshForP1CannotReplaceBarrierInstalledP2` | PASS |
| R3: localization reset while mapping callback is blocked | `GoalAcceptedDuringDrainSurvivesOldMappingCallback`, `TerminalDuringDrainCannotBeResurrectedByNewWorld` | PASS |
| R4: suspended E1 callback returns after E2 cutover | `SuspendedE1WorldRefreshCannotResumeAfterE2Cutover` | PASS |
| Unsafe fresh-world validation | `UnsafeFreshWorldRecertificationCannotResumeSuspendedExecution` | PASS |

CTest result XMLs are in the worktree build tree; the focused XML and package summary are listed in `TEST_EVIDENCE.md`. R3 is runtime-component/barrier evidence, not SITL.
