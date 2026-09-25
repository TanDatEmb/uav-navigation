# Performance and representation

- No full `CandidateBundle`, shared ownership, validator closure, or `std::function` was added to PlanningHistory.
- The predecessor witness is a value object: five `uint64_t` identity/generation scalars, kind/role enums, and an optional fixed-size `Eigen::Vector3d`. An exact ABI `sizeof(PlanningRequest)` before/after was not recorded; no allocation or dynamic payload was introduced by the witness itself.
- Steady correction maximum observed in pinned tests: one upward duration ULP. Configured cap: 32 corrections. Support correction maximum cap: 8 representable speed steps.
- No planner objective or timing budget changed. No performance gain is claimed.
