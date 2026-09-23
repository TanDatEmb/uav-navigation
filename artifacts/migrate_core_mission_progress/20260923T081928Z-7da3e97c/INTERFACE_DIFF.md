# Interface diff

| Interface | Change | Owner and validation |
|---|---|---|
| `NavigationCommand.mode_activation_id` | Added PX4 External Mode activation binding. | Core writes at final exposure gate; adapter requires equality with its active activation. Existing 100 ms lease remains. |
| `NavigationCommandAdmission` | New exact accepted-command tuple `{mode activation, mission, localization, goal epoch, waypoint, request, bundle, sample}` plus adapter timestamp/frame. | Adapter emits only after local command commit; Core joins to issued command, current gate and fresh ACTIVE/airborne status. No acceptance policy in message. |
| `NavigationModeStatus.activation_id`, `.airborne` | Added local lifecycle observation. | Adapter owns; Core checks source/receive freshness and frame before mission activation or admission. |
| `NavigationMissionProgress` | New immutable GOAL/COMPLETE receipt with route/localization/gate and acceptance evidence. | Core writes; adapter uses only matching COMPLETE for PX4 mode completion. |
| `/navigation/mission_complete` | Publisher moves to Core. | Core publishes false at activation and true only after accepted final gate. |
| `/navigation/goal` | External ingress remains for no configured Core mission. | When Core mission is loaded, external goals are rejected; internal successor bypasses ROS roundtrip and uses the same validated transition. |
| `mission_file` launch argument | Routed only to `navigation_runtime.launch.py`. | PX4 launch and adapter no longer receive it. |

No PX4 firmware API, Hold callback, 100 ms adapter lease, 500 ms runtime state freshness or planner certificate shape changed. `NavigationModeStatus` waypoint acceptance fields on COMPLETE are read-only copies of Core's terminal receipt for existing runtime handover handling; ACTIVE status does not claim acceptance.
