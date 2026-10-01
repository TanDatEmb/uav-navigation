# W3-H3 open questions

## Review request

- Coordinator review is required for the H3 checkpoint. This worktree does
  not self-assign PASS, merge readiness, or flight/qualification authority.
- H3.1/H3.2 recorded RED tests before the production fix. H3.3 is inherited
  from the prior RED timeout and bounded GREEN regression; H3.6 is a refactor
  and has no behavior RED requirement.

## Replay

- `NOT_EVALUABLE`: no recorded bag/cache is present in this worktree. The
  catalog contains only the downloadable AIST manifest (about 517 MB); no
  external download was started. SITL was not requested and was not run.

## Cleanup

- No prompt-listed cleanup paths were identified beyond the live H3 report and
  this OQ file. No unrelated files or legacy safety records were deleted.

## Remaining safety boundaries

- No threshold, UNKNOWN policy, authority owner, lease, or PX4 behavior was
  changed. The current safety ledger remains authoritative and was validated.
