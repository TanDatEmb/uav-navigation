# Lifecycle event catalog

| Phase | Recorded producer | Evidence consumer | Owner of fact | Identity currently present | Clock/order | Qualification role |
|---|---|---|---|---|---|---|
| `goal_issued` / `goal_observed` | Scenario goal publisher / topic subscriber | Event audit | Mission/desired intent | mission, waypoint, request; no planner cycle yet | ROS source plus observer sequence | Transport provenance, not a completed solve transaction |
| `request` planning cycle | Runtime planner diagnostic via scenario recorder | Reducer | Runtime planning | desired request/epoch, localization, cycle | diagnostic source stamp plus observer sequence | Required when joining export to solve |
| `export` | Runtime admission diagnostic via scenario recorder | Reducer | ExecutionAuthority admission | request, epoch, localization, cycle, candidate generation | diagnostic source stamp plus observer sequence | Required candidate owner witness |
| `activate` direct commit | Runtime admission disposition via scenario recorder | Reducer | ExecutionAuthority | candidate generation, cycle, request, epoch | diagnostic source stamp plus observer sequence | Required active transition witness |
| `activate` delayed | Runtime command-timer diagnostic via scenario recorder | Reducer | ExecutionAuthority | active generation/request/epoch, no planner cycle | diagnostic source stamp plus observer sequence | Must join exact export identity |
| `authorize` | NavigationExecutionDiagnostics | Reducer | ExecutionAuthority publication | sample, bundle generation, request, epoch, localization, world identity | command source and steady authorization stamps | Required for each published sample |
| `publish` | PX4 input diagnostic | Reducer | Adapter/PX4 boundary observer | sample, bundle generation, request, world identity, trace sequence | PX4 trace and observer order | Required downstream witness |
| `mode_activate` / `deactivate` | VehicleStatus observer | Mode/evaluator | PX4 boundary | mode activation instance; separate from bundle | VehicleStatus observer order | External authority, not bundle activation |

The recorder previously inherited missing lifecycle IDs from its most recent PVA/goal cache. That cache is unrelated to a new planner event under asynchronous interleaving. This branch removes the inheritance and adds a regression test. A goal-publisher event without a planner cycle remains unjoined rather than acquiring a fabricated cycle.
