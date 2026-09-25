# Remaining debt

1. Overall C0-SW eligibility is not closed: motion acceptance and tracking coverage policies are unavailable, and reference lineage mismatches. World evidence is resolved independently; evaluator policy was not broadened.
2. One nominal N2 run ended in a fail-closed terminal/recovery path at the final waypoint (`final_bridge_usable=false`); N1 and N3 completed but report-level validity/evidence was NOT_EVALUABLE or failed due Lidar freshness. This is not attributed to World evidence instrumentation and remains a separate product/runtime stability item.
3. 700 ms world-source loss leads to PX4 Hold as expected; automatic re-entry/restart after long stale is NEXT_PHASE.
4. Evidence writer does not expose event producer CPU time or queue high-water. Count/loss accounting for this capture was complete with 0 drops/errors and pending=0.
5. Historical A3 same-source-tick revision remains not evaluable; current live producer rejects equal/non-increasing source stamps.
6. PX4 closed-loop dynamics, flight performance, HITL, and qualification remain deferred; these SITL results are not flight qualification.
