# Lifecycle enum cross-product — baseline `7e0b850`

The fields are independent in `ExecutionLifecycleState`; no enum is a complete
replacement for another. `ExecutionAuthority` is the owner and increments its
timeline version when lifecycle state changes (`execution_authority.hpp:936-939`).

## Combinations constructed by the current setters

| Phase | Recovery | Exposure | Safety ownership | Restart request | Evidence / status |
|---|---|---|---|---|---|
| `kInitialHold` | `kInitialHold` | `kUnavailable` | `kNominal` | `kNone` | Default/reset: `execution_lifecycle.hpp:9-66`; `execution_authority.hpp:181-186`. |
| `kTrackingMain` | `kTrackMain` | `kAvailable` | `kNominal` | `kNone` | Main commit path: `execution_authority.hpp:960-987`; recovery transition: `execution_recovery_state.hpp:47-91`. |
| `kTrackingBackup` | `kTrackBackup` | `kAvailable` | `kSafetySuffix` | `kNone` | Backup activation: `execution_authority.hpp:228-237`, `960-987`. |
| `kTrackingBackup` | `kEmergencyBrake` | `kAvailable` | `kSafetySuffix` | `kNone` | Emergency commit sets role-derived phase/safety and recovery event: `execution_authority.hpp:960-987`. |
| `kStoppedHold` | `kStoppedRecovery` | `kAvailable` | `kNominal` | `kNone` or `kFromRest` | Stop hold preserves the restart field: `execution_authority.hpp:289-296`; stop transition: `execution_recovery_state.hpp:74-84`. |
| `kPx4Hold` | `kPx4Hold` | `kFailed` | `kNominal` | `kNone` | Fail closed is an explicit tuple: `execution_authority.hpp:952-958`; `navigation_runtime_node.cpp:1878-1883`. |
| `kTrackingMain` | `kTrackMain` | `kAvailable` | `kNominal` | `kFromRest` | Constructible when `requestRestartFromRest` is applied to an active MAIN before the next solve: `execution_authority.hpp:258-263`; static/runtime reachability is `CONDITIONAL`, not measured in PX4. |
| `kTrackingBackup` | `kTrackMain` | `kAvailable` | `kSafetySuffix` | `kNone` | Constructible by `observeRetainedCommand` with a BACKUP/EMERGENCY-role bundle without a recovery event: `execution_authority.hpp:208-220`; no positive runtime trace in this WP (`NOT_MEASURED`). |

## Combinations rejected or not constructed by the owner

- `kPx4Hold` with any recovery other than `kPx4Hold`, or with `kAvailable`, is
  not constructed by `failClosedLifecycleLocked()` (`execution_authority.hpp:952-958`)
  and `transitionExecutionRecovery()` freezes `kPx4Hold` (`execution_recovery_state.hpp:52-58`).
- `kTrackingMain` with `kSafetySuffix` is not assigned by role-aware setters:
  MAIN sets nominal safety (`execution_authority.hpp:970-980`); suffix roles set
  backup phase/safety.
- `kStoppedHold` with `kSafetySuffix` is not assigned by `stoppedHold()`;
  that method explicitly sets nominal ownership (`execution_authority.hpp:289-295`).
- `kInitialHold` with `kAvailable` is not assigned by reset/default paths;
  availability requires an active identity and a committed/retained command.
- A recovery transition alone cannot authorize a command: `applyRecoveryEvent()`
  changes only `lifecycle_.recovery` (`execution_authority.hpp:274-284`), while
  exposure/phase/safety are changed by separate authority methods.

## Semantic overlap

| Pair | Overlap | Non-equivalence |
|---|---|---|
| `ExecutionPhase::kTrackingBackup` / `ExecutionRecoveryState::{kTrackBackup,kEmergencyBrake}` | Both describe safety-role tracking at the physical phase level. | `ExecutionPhase` has no emergency distinction; recovery does. `observeSampledSafetyRole()` sets phase backup while preserving the specific recovery event (`execution_authority.hpp:228-237`). |
| `ExecutionPhase::kStoppedHold` / `ExecutionRecoveryState::kStoppedRecovery` | Both indicate a stopped recovery/hold boundary. | Phase is physical publication state; recovery controls whether measured-state planning is allowed (`execution_recovery_state.hpp:47-57`). |
| `ExecutionPhase::kPx4Hold` / `ExecutionRecoveryState::kPx4Hold` | Both are fail-closed PX4 Hold labels. | Exposure still independently carries `kFailed`; safety/restart are separately normalized (`execution_authority.hpp:952-958`). |
| `ExecutionExposure::kAvailable` / `ExecutionSafetyOwnership::kNominal` | Often co-occurs for a normal MAIN command. | Availability is permission to expose the active command; nominal ownership is not a lease or freshness proof. |
| `ExecutionExposure::kSuspended` / `ExecutionSafetyOwnership::kSafetySuffix` | A stale world may suspend a currently safety-owned command. | Suspension is publication/exposure state; safety ownership remains a role fact. `publishCommand()` uses this distinction (`navigation_runtime_node.cpp:8598-8612`). |
| `ExecutionRestartRequest::kFromRest` / `ExecutionRecoveryState::kStoppedRecovery` | Normally paired after a stopped suffix that needs a new solve. | Restart is a one-shot request bit; recovery is the episode state. `stoppedHold()` deliberately preserves restart (`execution_authority.hpp:289-296`). |

No live PX4 combination census was run: `NOT_MEASURED` is retained for runtime
reachability claims. Component predicate tests are not flight qualification.
