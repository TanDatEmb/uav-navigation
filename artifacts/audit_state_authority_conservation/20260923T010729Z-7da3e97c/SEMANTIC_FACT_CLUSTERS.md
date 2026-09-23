# Semantic fact clusters

| Current fields/objects | Classification | Conservation result |
|---|---|---|
| `active_goal_`, `executing_goal_`, Episode desired/active IDs, store active bundle | desired intent and executing fact are independent; copied IDs may be cache | keep two facts, but one canonical desired record and one active bundle identity; prove Episode copies derivable before removing |
| `goal_epoch`, `request_id`, `bundle_generation`, `transaction_id`, `sample_id` | distinct semantic/protocol orderings | keep typed identities; use derived `PlanningContextId` only as comparison key |
| Store `committed_`, `pending_`, `pending_activation_ns_` | independent authority + future cutover | keep all three; no staged sample exposure |
| `ExecutionEpisodePhase`, `ExecutionRecoveryState`, `safety_suffix_active`, `command_available`, `failure_latched`, `restart_from_rest` | mixture of phase, recovery policy, temporal memory and derived gates | unresolved; enumerate reachable product states and replace only with a sum type that preserves stop/retry/latch semantics |
| `MissionControllerState`, `mission_terminal_`, `handover_requested_`, adapter recovery/suffix IDs | mission decision plus cross-process protocol cache/mirror | mission acceptance independent; adapter copies need explicit event/lease replacement proof |
| `hold_handover_pending_`, `hold_handover_in_flight_`, `px4_hold_confirmed_` | independent API operation versus external status | preserve distinction; tagged protocol state may replace booleans if retry/deactivation semantics survive |
| `route_progress_.state_`, `active_waypoint_index_`, previous measured position/time | measured history versus accepted policy gate | keep separately; monotonic scalar cannot reconstruct observed projection, reversal/time or policy acceptance |
| latest world pointer, bundle pinned/certified world, source/receive timestamps | independent evidence and age clocks | keep separate; no “fresh=true” mirror without expiry source |

`FACT_FROM_TARGET_CODE`: Episode hot retarget preserves active identity (`execution_episode.hpp:85-105`), Store has active/pending (`committed_bundle_store.hpp:821-829`), route projection stores segment and arc and handles geometric ties using previous progress (`route_progress.cpp:295-345`), Hold status is written by VehicleStatus (`navigation_mode_node.cpp:2846-2854`). `INFERENCE`: canonical target ownership and derivability candidates. `UNRESOLVED`: exact deletion/merge proof.
