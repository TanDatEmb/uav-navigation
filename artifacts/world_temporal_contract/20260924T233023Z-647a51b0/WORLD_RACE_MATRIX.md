# World race matrix

This matrix records deterministic source/component coverage present in the pinned baseline and rerun by this branch's full CTest. It does not imply runtime fault or C0-SW transaction-witness coverage.

| Race / behavior | Current fence | Exact deterministic coverage | Result / gap |
|---|---|---|---|
| World publication cannot interleave a commit authorized against current world | `WorldSnapshotStore` publication gate serializes authorization and publication | `WorldSnapshotStore.PublicationCannotInterleaveAnAuthorizedCommit`; `WorldSnapshotStore.PublishAndFinalizeKeepsDependentStateOrdered` | PASS |
| Candidate validated on W10, then latest world advances before commit | exact latest identity authorization; disjoint-change path is explicit | `WorldSnapshotStore.WorldAdvanceAfterCandidateValidationCannotCommitCommand`; `WorldSnapshotStore.AllowsStaleCertificateWhenChangeProvenanceIsDisjoint` | PASS |
| E1 world refresh is superseded by a newer execution commit | execution timeline version and exact active/staged pointer checks return superseded/no-op | `ExecutionAuthority.SupersededWorldRefreshPreservesNewCommit`; `ExecutionAuthority.ActivationWinsAgainstStaleWorldRefresh`; `ExecutionAuthority.SnapshotSupersededDuringPreparationIsNoOp` | PASS |
| Active validation failure against current newer world | exact active record is revoked; retry/publication order is finalized | `ExecutionAuthority.PreparationFailureRevokesExactActiveAndRetriesWorld`; `ExecutionAuthority.RevokeWinsAndBlocksPendingReexposure` | PASS |
| Pending alone fails world validation | pending is dropped independently; active remains | `ExecutionAuthority.PendingPreparationFailureDoesNotRevokeActive`; `ExecutionAuthority.ActiveValidPendingInvalidKeepsActive` | PASS |
| Stale revoke after active replacement | revoke token must match exact active record | `ExecutionAuthority.StaleRevokePreservesReplacementActiveBundle` | PASS |
| Stale revoke after pending replacement | revoke token must match exact pending record | `ExecutionAuthority.StaleRevokePreservesNewPendingSuccessor`; `ExecutionAuthority.StaleRevokePreservesRecertifiedPendingSuccessor` | PASS |
| Stale sampled command after world refresh | exact sample pointer cannot authorize exposure after recertification | `ExecutionAuthority.WorldRecertificationRejectsOldSamplePointer`; `ExecutionAuthority.ExposureMustRecheckFreshnessAfterWaitingForStoreLock` | PASS |
| Fresh world resumes suspended exact execution only | exact generation and fresh-world recertification gate | `PlannerFsm.ResumesOnlyExactFreshWorldRecertifiedGeneration`; `ExecutionAuthority.RecertifiedSuspensionRetainsSafetyOwnership` | PASS at component level; no isolated runtime source-stale/recovery run |
| Localization reset while old callback is in flight | mapping actor epoch check, reset drain, and runtime reset owner lock | `MappingActorContract.ReconstructsBackendForNewLocalizationEpoch`; `NavigationRuntimeEpochReset.TerminalDuringDrainCannotBeResurrectedByNewWorld` | PASS for covered reset paths; no complete barrier matrix for every async world callback |
| Same generation revision/stamp edge cases | store identity monotonicity contract | new `WorldSnapshotStore.SameSourceTickRevisionAdvanceIsStoreLegal`; existing `RejectsNullInvalidAndNonMonotonicPublication` | PASS for equal stamp advance and general regression; detailed timestamp-regression subcases remain a coverage gap |
| New generation with restarted source timestamp | generation is a new source-time domain at store boundary | new `WorldSnapshotStore.GenerationAdvanceDefinesNewTimestampDomain` | PASS at store boundary; runtime MappingActor only increments generation on localization epoch change |
| Failed world-dependent finalizer | old immutable world remains published | new `WorldSnapshotStore.FailedDependentFinalizerKeepsPreviousWorldVisible` | PASS |

The tests above are unit/component evidence. This branch did not add all requested barrier cases for in-flight mapping reset, suspended E1 recovery racing E2, or P1 validation racing P2 replacement, and it did not run an isolated World Source Stale SITL fault. Those remain explicit gaps, not inferred passes.
