# Completed terminal handover review

## Scope and diagnosis

This isolated review branch starts from `148753330625c3af76d3b8966c13e89bb6727c1c`
(`Serialize planner watchdog activity ownership`). The runtime already
validated the exact completed executing witness and suppressed terminal
comparison when the desired request differed. The remaining admission branch
requested `PlanFromRest` only when the desired position was outside its
completion tolerance. A newer same-mission desired goal coincident with the
completed predecessor could therefore remain without a replacement solve.

The baseline artifact
`/home/letandat/Dev/uav-navigation/.artifacts/runtime/external-mode-check-20260905T082033-554608`
records generation 4 starting at 37.896 s, duration 3.622254 s, ending at
41.518254 s, with request 5 arriving at 41.516 s. Source speeds immediately
after the endpoint were approximately 0.0258--0.0382 m/s, below the existing
0.15 m/s stationary gate. The desired request was coincident, so the distance
condition did not request a measured restart; after roughly three seconds of
drift, a restart attempt was blocked. This is runtime evidence for the
bounded lifecycle hole, not flight acceptance evidence.

## Implemented contract

The planning-cycle handover keeps the completed executing goal and checks the
old immutable bundle as a strict terminal witness: `kTerminalStop`,
`terminal_stop`, MAIN owner, finite declared endpoint, and endpoint role
contract. The already serialized completion witness supplies the exact
executing bundle/timeline identity; it is not relabelled for the desired goal.
Only a newer, valid, same-mission desired identity can consume this
predecessor completion to set the existing `restart_from_rest` request. The
next goal remains incomplete and must pass the existing fresh, stationary,
measured-state planner and execution admission gates. No additional mutable
terminal marker or distance exception is required for this admission.

The command publisher now scopes terminal-sample cancellation to the exact
completed bundle tuple while holding localization, input, and execution-lease
locks. `PlanningWorker::cancelActiveIfExecutionIdentity` holds its worker
mutex through the backend interrupt, preventing the old active job from
turning over to a new desired job between identity validation and cancellation.

## Required focused evidence

The regression must exercise the real runtime/worker dispatch boundary for:

- completed terminal predecessor followed by a same-mission coincident desired
  goal at measured speed 0.03 m/s, with a stopped-state admission request;
- replacement solve winning the race;
- stale timeline/witness, foreign mission, unfinished endpoint, moving state,
  and measured speed above 0.15 m/s remaining rejected or deferred;
- terminal publisher callback during a new solve leaving the new solve alive;
- old solve completion followed by new solve start around cancellation proving
  no new backend cancellation.

Boolean-only policy tests do not establish this dispatch contract. No SITL
acceptance is claimed here; integration and repeated representative evidence
remain owned by the central integration review.

## Before/after validation evidence

The clean runtime build used the read-only underlay
`/tmp/uav-navigation-reviewed-integration-install` and produced into
`/tmp/uav-navigation-completed-handover-clean-install2`; the source checkout
was this branch, with no main checkout or map pooling involved. The component
fixture publishes a dense 231-endpoint PointCloud2 volume through the public
`RegisteredScan` contract, and feeds propagated odometry from the public
command trajectory with the same measured velocity. It observes a real
`NavigationRuntimeNode` command stream.

The green run of
`NavigationRuntimeHandover.DispatchesNewStopAfterCoincidentCompletedPassThrough`
observed the old request 10 MAIN/BACKUP command, then the terminal completion
witness, followed by the log `desired request=11 changed`. The node logged
`restarting PlanFromRest` for request 11 and dispatched the successor MAIN
command. The successor target was 0.5 m beyond the measured stop position
with a 0.8 m acceptance radius, so it was in the desired goal ball while
retaining a finite route segment. The measured terminal velocity was 0.03
m/s; no pose teleport or gate relaxation was used.

Focused command:

```
source /opt/ros/jazzy/setup.bash
source /tmp/uav-navigation-reviewed-integration-install/setup.bash
source /tmp/uav-navigation-completed-handover-clean-install2/setup.bash
ctest --test-dir /tmp/uav-navigation-completed-handover-clean-build2/navigation_runtime \
  --output-on-failure -R 'test_planning_worker|test_navigation_runtime_shutdown'
```

Result: 2/2 tests passed, including the bounded worker turnover cancellation
interleaving and the real public-contract handover fixture. The raw watchdog
baseline remained the documented pre-fix evidence: artifact
`/tmp/uav-navigation-reviewed-integration/.artifacts/runtime/external-mode-check-20260905T111200-690529`
reported the same-request req4-to-5 failure, while the older runtime artifact
above records the coincident req5 arrival and subsequent drift. These are
diagnostic runtime artifacts; no SITL qualification claim follows from them.

The broader mission/execution debt remains outside this bounded change:
repeated representative flight evidence, PX4 transport freshness, moving
handover behavior, and the existing nonterminal planner failure paths still
need central integration review. The worker's pre-existing bare `submit()` and
`cancelActive()` paths retain their prior semantics; only the new terminal
identity path is serialized through the worker mutex and backend interrupt.
