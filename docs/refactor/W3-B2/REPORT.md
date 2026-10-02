# W3-B2 — certifier MOVE checkpoint

## Verdict

`BLOCKED / REVIEW REQUEST`: the current `origin/main@cc13746` baseline does
not expose the certifier boundary required by W3-B2 as a mechanical file move.
No product or safety behavior was changed in this checkpoint.

## Evidence

- `trajectory_world_validator.hpp` and `route_regression_certificate.hpp`
  consume backend-owned `CandidateCommandBundle`/`CmdTraj` types.
- `trajectory_dynamics.hpp` includes backend-owned `traj_opt/config.hpp`.
- `backup_braking.hpp` mixes the requested stop witness types and reachability
  checks with BACKUP polynomial synthesis; the requested split needs an
  explicit pure interface rather than copying the whole header.
- `continuous_clearance.hpp` and `candidate_admission.hpp` can move only after
  their ownership and namespace consumers are updated together.

The listed dependencies are incompatible with a new `navigation_certifier`
package that may depend only on `navigation_planning`,
`navigation_world_model`, `navigation_common`, and Eigen. A plain `git mv`
would either leave a `navigation_planning_backend` include dependency or force
an undeclared header shim, both prohibited by ADR-018 E5/E6 and the W3-B2
prompt.

## Safety boundary

Planner authority remains unchanged. No validator, certificate, threshold,
UNKNOWN policy, or command path was modified. This checkpoint therefore has
no new bypass or ledger entry.

## Required next change

First introduce RED compile/unit contracts for pure certifier inputs and the
backend wrapper boundary; then extract the moved bodies one family at a time:

1. corridor and dynamics pure geometry interfaces;
2. world sweep and clearance views;
3. route/anchor continuity;
4. stop reachability witness separated from BACKUP synthesis;
5. package/CMake/dependency guard and moved tests.

The implementation must finish with `gate.sh all`, selected certifier/backend/
execution/runtime/world-model CTests, a color-moved hunk ledger, and the
dependency-direction guard. Until then B2 is not merge-ready.

## Verification

Inventory was read-only; no build or SITL was run. The checkpoint itself was
checked with `git diff --check`.
