# Clock-domain audit

## Quy tắc quan sát được

| phép so sánh | vế trái | vế phải | kết luận |
|---|---|---|---|
| Runtime state freshness | `now_ros_ns` từ `Node::get_clock()->now()` | `source_stamp_ros_ns` từ message | ROS–ROS; source age (`execution_state_freshness.hpp:28-48`) |
| Runtime receive freshness | `now_steady_ns` từ `steadyClockNowNanoseconds()` | `receive_stamp_steady_ns` | steady–steady; receive age (`execution_state_freshness.hpp:49-63`) |
| Mapping observation freshness | ROS `ros_clock->now()` | observation/source stamp ns | ROS–ROS (`navigation_runtime_node.cpp:1461-1508`) |
| World snapshot freshness | runtime `now_ns` | world observation stamp / identity | ROS–ROS (`navigation_runtime_node.cpp:3037-3047`, `8602-8612`) |
| Runtime command lease | `now()` / command timer ROS time | command `header.stamp` and `valid_until` | ROS–ROS (`navigation_command_contract.hpp:85-100`; `navigation_runtime_node.cpp:2765`) |
| Adapter command temporal lease | `callback_now` ROS time | command header/valid-until ROS time | ROS–ROS (`navigation_mode_node.cpp:1143-1159`) |
| Adapter command receive lease | `now.nanoseconds()` ROS time | `last_command_receive_ns_` captured from ROS clock | ROS–ROS (`navigation_mode_node.cpp:1300`, `2422-2439`) |
| Adapter odometry source freshness | adapter ROS now | odometry header source stamp ROS | ROS–ROS (`navigation_mode_node.cpp:1243-1249`) |
| Adapter odometry receive freshness | adapter steady now | `last_odometry_receive_steady_ns_` | steady–steady (`navigation_mode_node.cpp:1466-1509`, `2117-2123`) |
| Adapter health freshness | ROS now/source stamp plus steady now/receive stamp | corresponding typed health fields | paired ROS–ROS and steady–steady (`navigation_mode_node.cpp:923-930`, `2335-2340`) |
| Planner request deadline | `PlanningBudget::Clock::now()` and `steady_deadline_ns` | request deadline | `PlanningBudget::Clock` is steady for cancellation; exact alias should be confirmed from header; assignment at `navigation_runtime_node.cpp:5678-5682` |
| Planner `AbsoluteDeadline` | simulation deadline vs simulation `getSimTime()` | steady deadline vs `steady_clock::now()` | two independent comparisons, then conservative minimum; no wall/ROS subtraction (`absolute_deadline.hpp:18-55`) |
| Bridge diagnostics freshness | bridge ROS `now()` | producer diagnostic source stamp | ROS–ROS (`px4_external_odometry_bridge_node.cpp:206-213`) |
| Bridge timestamp conversion | source measurement timestamp and bridge ROS `now()` | timestamp converter's source/transport mapping | both are inputs to conversion; age is not subtracted across domains (`px4_external_odometry_bridge_node.cpp:286-300`) |
| Bridge geometric jump | current/previous propagated source stamps and pose | configured continuity state | source-time/source-pose domain; generation change clears latch (`px4_external_odometry_bridge_node.cpp:228-244`, `301-312`) |
| PX4 setpoint command stamp | adapter ROS now | command ROS header and validity interval | ROS–ROS (`navigation_mode_node.cpp:2432-2444`) |
| PX4 local-position alignment lease | adapter steady now | PX4 local-position receive steady timestamp | steady–steady (`navigation_mode_node.cpp:2117-2123`) |

`evaluateExecutionStateFreshness` intentionally accepts four timestamps but
compares them as two same-domain pairs (`execution_state_freshness.hpp:28-63`).
There is no direct `ROS_ns - steady_ns` arithmetic in that helper. A wall/steady
delay can nevertheless cause a receive-age failure while ROS source age remains
small; that is a deliberate fail-closed distinction.

## Wall timers trong mô phỏng

`create_wall_timer` is selected explicitly in the source. When launch passes
`use_sim_time=true` (launch propagation at `navigation_runtime.launch.py:17-28`,
`px4_external_mode.launch.py:14-24`, `fast_lio.launch.py:21-53`), these timer
callbacks remain host wall/steady scheduled; they are not gated by `/clock`.

| process/node | wall timer | source | `use_sim_time=true` status | implication |
|---|---|---|---|---|
| `navigation_runtime_node` | planning 100 ms | `navigation_runtime_node.cpp:1773-1775` | YES when launch argument is true | schedule cadence is wall, while key/freshness/command timestamps are ROS |
| `navigation_runtime_node` | command 20 ms | `navigation_runtime_node.cpp:1776-1778` | YES | 50 Hz sampling continues independently of a stopped `/clock`; command validity can still fail on ROS time |
| `navigation_runtime_node` | mission 50 ms | `navigation_runtime_node.cpp:1779-1783` | YES | mission polling is wall-paced; comparisons inside use ROS + steady pairs |
| `px4_navigation_external_mode` | boundary 50 ms | `navigation_mode_node.cpp:299-300` | YES when external-mode launch is sim-time | recovery boundary is wall-paced; recovery deadline value is ROS timestamp |
| `px4_navigation_external_mode` | Hold handover 50 ms | `navigation_mode_node.cpp:2599-2600` | YES | retry is wall-paced but retry eligibility is steady (`navigation_mode_node.cpp:2614-2620`) |
| `fast_lio` | transport diagnostics 1 s | `fast_lio_node.cpp:238-240` | YES under `fast_lio.launch.py` | diagnostic callback is wall-paced |
| `fast_lio` | estimator diagnostics 500 ms | `fast_lio_node.cpp:241-243` | YES | health publication cadence is wall-paced; source stamp remains ROS |
| `fast_lio` | initial-prior startup 10 ms | `fast_lio_node.cpp:244-247` | YES | startup polling is wall-paced |
| legacy `px4_odometry_bridge_node` | diagnostics 500 ms | `px4_odometry_bridge_node.cpp:144-155` | depends on launch/config; node reports `bridge_use_sim_time` at `px4_odometry_bridge_node.cpp:492` | external bridge executable has no timer; this is the alternative legacy bridge |

The external odometry bridge used by `fast_lio.launch.py` has no
`create_wall_timer`; it is callback-driven by `/lio/odometry_propagated` and
diagnostics (`px4_external_odometry_bridge_node.cpp:96-110`, `538-541`).

## Nhánh trộn clock cần theo dõi

1. Runtime command callback runs on a wall timer but compares command/state/world
   ROS timestamps and also uses steady receive age (`navigation_runtime_node.cpp:8590-8605`,
   `8891-8951`). Đây là mixed scheduling, not mixed subtraction.
2. Adapter `updateSetpoint` is framework-scheduled at 50 Hz (`navigation_mode_node.cpp:308`)
   while command validity uses ROS and receive/state leases use steady (`2063-2123`,
   `2422-2444`).
3. Planner combines simulation time for trajectory semantics with a steady hard
   cancellation boundary (`absolute_deadline.hpp:18-55`). This is fail-closed via
   `min(simulation_remaining, steady_remaining)`, not a unit conversion between
   unrelated clock epochs.
4. Diagnostic `/clock` gap thresholds in `tools/runtime/state_transport_analysis.py:45-51`
   are evidence-analysis thresholds, not runtime product gates. The pilot/cohort
   values in `timing_budget.csv` therefore must not be used as flight acceptance.
