# PX4 Hold ordering: source-derived counterexample

**P1 CONDITIONAL / SPECIFICATION_GAP.** `NavigationModeExecutor` distinguishes a Hold request, a scheduled mode operation, and `VehicleStatus` AUTO_LOITER observation. The code nevertheless clears `hold_handover_pending_` when the `scheduleMode` completion callback returns `Success` or `Deactivated`, even if `px4_hold_confirmed_` is still false. The periodic retry only runs while pending is true. This is a source-level path by which retry stops without a locally observed Hold status; whether it occurs in the target runtime, and whether the callback itself is an adequate authority witness, remain unverified.

| Event | pending | in flight | confirmed | Next timer action |
|---|---:|---:|---:|---|
| `schedulePx4Hold()` | true | true | false | waits for callback |
| `onPx4HoldHandoverCompleted(Success)` before status | false | false | false | `checkHoldHandover()` does nothing |
| No later AUTO_LOITER status | false | false | false | no retry or confirmation |

`FACT_FROM_TARGET_CODE`: `navigation_mode_node.cpp:2846-2863,2894-2937`. `FACT_FROM_PINNED_SUBMODULE`: `src/external/px4_ros2_interface_lib` gitlink `4a3370f084ac6f1ef001a4afa2b007845ffd0837`; `px4_ros2_cpp/src/components/mode_executor.cpp` blob `d8f23a6ccfb25d6f5943140cc44b1a85be44598d`, lines 225-260 and 484-519. `scheduleMode` waits for a VehicleCommand ACK before activating a `ModeCompleted` listener; the later completion callback is not equivalent to that ACK and is not a `VehicleStatus` AUTO_LOITER sample. The gitlink and blob are listed in `EVIDENCE_INDEX.md`.

The present code may intentionally treat `ModeCompleted(Success)` as final disposition and may rely on executor deactivation during handover. That policy is not specified by the current safety contract. A target tagged Hold protocol must define what `Success`, `Deactivated`, status loss, operator takeover and executor deactivation mean before merging any booleans. A paired PX4/SITL ordering trace should check that no state is reported confirmed without the status witness, and that a failed or unconfirmed handover has an explicit safe disposition. This artifact does not claim an observed PX4 failure.
