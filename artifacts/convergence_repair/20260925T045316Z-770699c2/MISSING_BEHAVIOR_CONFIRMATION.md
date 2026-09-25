# Missing behavior confirmation

The audit pinned at `770699c2` identified exactly two absent behaviors. Source review confirmed both findings against the repair base before implementation:

1. **C1**: steady-cruise braking computed one closed-form `double` duration and used the numerical ULP helper to accept polynomial extrema. That helper correctly tolerates evaluation noise, but also allowed a representable duration rounded down below the physical acceleration or jerk bound.
2. **C2**: runtime captured an ExecutionAuthority-owned predecessor while constructing a request, but PlanningHistory carried only generation and velocity. The emergency bounded route-correction exception later inspected mutable `planner_warm_start_.snapshot()` instead of request-owned provenance.

The historical implementation branch was not merged and neither commit was cherry-picked. Canonical equivalents were implemented in separate reviewable commits.
