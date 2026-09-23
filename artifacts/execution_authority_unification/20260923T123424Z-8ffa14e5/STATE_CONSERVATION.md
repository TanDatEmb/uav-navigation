# State conservation before mutation

Disposition is for one product `ExecutionAuthority` record. `DERIVE` means a read-only view, never another persistent writer. Source: `execution_episode.hpp:41-268`, `committed_bundle_store.hpp:79-833`, `navigation_runtime_node.hpp:472-622` at base `8ffa14e5`.

| Current field | Independent meaning | Disposition | Target |
|---|---|---|---|
| Episode `localization_epoch` | Fences episode observations | MERGE | typed admission context while no active; active bundle epoch otherwise |
| Episode `goal_epoch` | Desired admission scope | MERGE | `AdmissionContext.goal_epoch`, distinct from active epoch |
| Episode `request_id` | Desired request mirror used by renewal predicate | DERIVE | desired goal under input owner; candidate request checked at admission |
| Episode `active_command_goal_epoch` | Active epoch mirror | DELETE | active bundle epoch |
| Episode `active_command_request_id` | Active request mirror | DELETE | active bundle request |
| Episode `active_generation` | Active generation mirror | DELETE | active bundle generation |
| Episode `phase` | Sampled/physical presentation; can differ from recovery | KEEP | execution lifecycle phase, one owner |
| Episode `command_available` | Command exposure, independently suspended with active retained | MERGE | typed exposure disposition |
| Episode `failure_latched` | Terminal fail closed; excludes exposure | MERGE | typed exposure disposition `Failed` |
| Episode `safety_suffix_active` | Safety-owned suffix even while sample role MAIN | KEEP | typed safety ownership, independent of sampled role |
| Episode `restart_from_rest` | Temporal recovery request survives STOPPED_HOLD samples | KEEP | typed restart request inside lifecycle |
| Episode `recovery_state` | One-way recovery policy; orthogonal to phase | KEEP | execution lifecycle recovery |
| Timeline `active_goal_epoch_` | Candidate admission scope, *not* active identity | RENAME | `AdmissionContext.goal_epoch` |
| Timeline `timeline_version_` | Optimistic exact-timeline transaction fence | KEEP | authority version |
| Timeline `active_lineage_version_` | Predecessor anchor fence across replacement/revocation | KEEP | active lineage |
| Timeline `last_transaction_id_` | Reject older planner result | KEEP | admission transaction watermark |
| Timeline `world_identity_` | World publication/recertification transaction | KEEP | authority world identity |
| Timeline `committed_` | Immutable active candidate | MERGE | active record `{full goal, bundle}` |
| Timeline `pending_` | Immutable staged candidate | MERGE | staged record `{full goal, bundle, activation}` |
| Timeline `pending_activation_ns_` | Exact staged cutover time | MERGE | staged record activation stamp |
| Node `executing_goal_` | Full active goal payload | MERGE | active record goal; absent before first commit |
| Node `command_goal_epoch_` | Active execution epoch | DELETE | active record bundle epoch |

`ExecutionEpisode::mutex_` and `ExecutionTimelineStore::mutex_` must become one execution mutex. Outer `input`, localization, and command-lease transition locks remain orthogonal; they do not confer a second execution identity. `active_goal_` and `active_goal_epoch_` remain desired/planning facts and must never be repurposed as active identity.

Precondition to deletion: every former consumer of `executing_goal_`, `command_goal_epoch_`, and Episode identity fields must be mapped to an immutable active record or a deliberately distinct admission context. Compile success alone is not proof.
