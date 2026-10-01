# ADR-014: Independent `nav_certifier` in `nav_core`; adapter keeps a local setpoint guard, not a certifier

**Status:** accepted (architecture), implementation in P3 (shadow first).

## Decision question

Does certification run only in `nav_core` before commit, or does the PX4
adapter also re-certify at the PX4 boundary?

## Decision

1. **All trajectory certification runs in `nav_core`**, in a new package
   `nav_certifier`. It depends only on tier-A contracts (`nav_plan_contract`,
   `nav_world_contract`, `nav_safety_profile`, `nav_core_types`) and **must not
   link `nav_planner`**. A CMake test asserts that dependency.
2. Certificates: continuous corridor-plane (HG-002/HG-013), dynamics and
   flatness against the role envelope (HG-004), swept world (HG-010, SAFE
   KNOWN_FREE / FAST explicit UNKNOWN, both reject OCCUPIED/OUT_OF_MAP),
   BACKUP/stop existence (HG-035), anchor continuity. The exact list and
   inputs are fixed by WP-A3/A4 plus the certificate inventory in P3's spec.
3. Where it runs:
   - planning lane, right after generation: `certify(bundle, pinned_world)`;
   - fast lane, on every world revision: recertify the active and staged
     bundles against the new snapshot (today `validateRetainedCommand`);
   - decision thread, at commit: **O(1) checks only**: the certificate refers
     to the latest world revision, the same bundle generation, localization
     epoch, goal epoch, lease and anchor. No geometry on the decision thread.
4. `CandidateBundle` becomes pure data. The planner may still run its own
   internal feasibility checks for search, but they grant no authority.
5. **The adapter does not certify trajectories.** It has no world model, and
   giving it one would duplicate `nav_world` across processes. The adapter keeps
   and formalizes a **`SetpointGuard`** in `px4_setpoint_core`: finite values,
   frame, monotonic identity, mode-activation id, command lease, state age,
   physical envelope from `nav_safety_profile`, tracking/anchor limit (HG-007).
   Guard limits are read from the same profile (fixes V6).

## Rationale

- RC2: today the component that generates a trajectory also decides whether it
  is safe. 21/22 historical reverts are in that component, so a solver change
  can silently weaken its own checker.
- `std::function` in the bundle (V4) makes certificates non-replayable. Data
  bundles can be recorded in bags (ADR-015) and certified offline, which is
  how P3 is qualified (shadow comparison).
- A world-aware certifier in the adapter would add a second map, a second
  snapshot identity and a cross-process consistency problem for no gain: a
  certified command still expires within 100 ms at the adapter lease.

## Consequences

- P3 runs old closure validation and `nav_certifier` side by side, logs every
  disagreement as typed evidence, and does not switch authority until there
  are 0 unexplained disagreements on the baseline corpus.
- Certification cost moves off the planning critical path only partly;
  `certify` p99 must fit inside the existing 80 ms solve budget (HG-001).
  Measure before switching; do not enlarge budgets.
