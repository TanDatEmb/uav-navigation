# Proposed execution authority owner — pre-implementation decision

Choose `navigation_execution::ExecutionAuthority` because the existing timeline already owns candidate/world admission and anchor lineage; recovery transitions are execution lifecycle policy and can move with it without a reverse package dependency. `navigation_runtime` remains responsible for desired mission/planning state, completion evidence, and outer safety coordination. `navigation_execution` will require the contract goal message only to store the full immutable active/staged goal payload; it will not own `MissionProgress` or a desired goal.

One mutex protects one state record:

```cpp
struct AdmissionContext { uint64_t localization_epoch; uint64_t goal_epoch; };
struct ActiveExecution { shared_ptr<const NavigationGoal> goal;
                         shared_ptr<const CandidateBundle> bundle; };
struct StagedExecution { shared_ptr<const NavigationGoal> goal;
                         shared_ptr<const CandidateBundle> bundle;
                         int64_t activation_ns; };
struct ExecutionLifecycle { Phase phase; RecoveryState recovery;
                            ExposureDisposition exposure;
                            SafetyOwnership safety;
                            RestartRequest restart; };
struct ExecutionAuthoritySnapshot { uint64_t version, active_lineage;
                                    optional<WorldIdentity> world;
                                    AdmissionContext admission;
                                    optional<ActiveExecution> active;
                                    optional<StagedExecution> staged;
                                    ExecutionLifecycle lifecycle; };
```

`AdmissionContext` is a monotonically advanced fence for stale planner results; it is *not* the active command epoch and carries no waypoint/mission payload. When desired N advances to N+1, the owner changes admission context and clears obsolete staged work, but retains active predecessor N while certified. Staging captures the full successor goal with the candidate. Activation swaps the staged record to active at `activation_ns` and updates lifecycle/lineage/version in the same mutex transaction. The goal and bundle must agree on request, epoch, and localization evidence; desired admission is validated by the runtime mission/planning boundary before the owner call.

`ExposureDisposition` must distinguish no command, available, suspended, failed. The failed state is absorbing until reset/new admitted execution according to the current rules. `SafetyOwnership` is independent of polynomial sample role; an active MAIN can already be safety suffix owned. `Phase` and `RecoveryState` remain orthogonal because terminal analytic hold can precede measured stop. `RestartRequest` survives STOPPED_HOLD samples.

Sampling captures immutable active record and version under the owner mutex, evaluates polynomial outside it, then final publication rechecks the exact active record/version, world identity, exposure, and external freshness/lease witnesses under the one execution mutex. The publish callback remains bounded. External `ExecutionStateFailureLatch`, WorldModel, MissionProgress, and PX4 adapter remain separate owners.

World recertification prepares candidate copies outside the owner mutex and conditionally swaps active and pending pointers while retaining their full goal payloads. Rejected pending never revokes valid active. Exact active revocation changes exposure/lifecycle in the same owner transaction. External planner-history finalization keeps its rollback semantics but no callback may exist solely to synchronize a second Episode owner.

State Admission Test: admission context survives desired transition before successor commit and fences late results; active goal payload is unavailable from `CandidateBundle`; staged goal payload must survive until activation; lifecycle phase/recovery/safety/restart/exposure preserve independent observations listed in `EXECUTION_STATE_REACHABILITY.md`; version/lineage/watermark protect distinct races; world identity binds recertification. Each is owned only by `ExecutionAuthority` and has an explicit mutation event. None copies MissionProgress's accepted gate or PX4-local authority.
