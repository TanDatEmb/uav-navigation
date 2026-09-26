# Pinned PX4 ROS 2 callback semantics

Source: submodule `src/external/px4_ros2_interface_lib`, SHA `4a3370f084ac6f1ef001a4afa2b007845ffd0837`.

- `ModeExecutorBase::scheduleMode(mode_id, callback)` constructs `VEHICLE_CMD_SET_NAV_STATE`, sends it synchronously, and waits for `VehicleCommandAck`. `Result::Success` at this point means the matching ACK result was `VEHICLE_CMD_RESULT_ACCEPTED`; only then does the library register a `ScheduledMode` and retain the callback (`px4_ros2_cpp/src/components/mode_executor.cpp:225-260`). It is not an observed nav-state transition.
- The retained callback fires later when a `ModeCompleted` message has `nav_state == scheduled mode`; its message `result` is forwarded. Or the callback fires as `Result::Deactivated` when the scheduled mode is canceled, including executor deactivation or a replacement `scheduleMode()` (`mode_executor.cpp:484-525`, `ModeExecutorBase::callOnDeactivate()` at `:105-117`).
- `VehicleStatus` is separately processed by `ModeExecutorBase::vehicleStatusUpdated()` and exposes the current `nav_state`, `executor_in_charge`, and `failsafe` fields. The product's `SharedSubscription` uses the same versioned `fmu/out/vehicle_status` topic.
- Pinned `px4_msgs` defines `NAVIGATION_STATE_AUTO_LOITER = 4` (`VehicleStatus.msg`). The external PX4 source maps that state to the Navigator Loiter mode (`src/modules/navigator/navigator_main.cpp:779`, `src/modules/navigator/loiter.cpp:46`).
- In the supplied external PX4 checkout, `ModeCompleted` is published by finite navigator modes (Mission, Land, Takeoff, RTL, VTOL Takeoff). No `mode_completed(AUTO_LOITER)` producer was found in the Navigator sources searched. Therefore a Hold scheduled-mode callback is not an expected confirmation path for persistent Loiter; current-state evidence must come from VehicleStatus.
- External PX4 files relevant to these semantics were read-only. Checkout dirty status is preserved in `BASE_PROVENANCE.md`; conclusions are tied to the recorded binary hash and checked-out source, not proof that the dirty checkout can reproduce that binary.

## Consequence for the product transition

`Result::Success` cannot be interpreted as "PX4 is currently in Hold" by the pinned library contract. `Result::Deactivated` specifically means the scheduled operation was canceled; it is not Hold confirmation. A confirmed Hold requires an actual VehicleStatus observation whose `nav_state` is AUTO_LOITER, correlated to the current handover/activation episode.
