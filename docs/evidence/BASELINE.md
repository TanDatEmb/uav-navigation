# Baseline integration record

This record describes the repository migration, not flight qualification.
The baseline starts from old origin/main `4dc769c5ce9ca1b4fcf649fc4c935bfd6416f0d7`.
The source closure under review is old HEAD
`4749bd2`; its 293-commit ancestry is not being merged wholesale as PR #18.

## Preservation and disposition

The owner selected a fresh public repository with the same name, preserving
old code/history/WIP and GitHub metadata locally. The local package is
`/home/letandat/Dev/uav-navigation-backup-20261005/` and contains all-refs.bundle,
the original .git archive, source/WIP archive, retired documentation, refs,
checksums, restored-repository verification and GitHub PR/issues/settings.
Runtime artifacts are explicitly excluded from the code backup by owner request;
they remain in the original workspace. Do not assume this package preserves
sensor recordings, bags or replayable runtime evidence.

[Commit disposition](commit_disposition.csv) records every commit outside the
old main reachable from local branches and GitHub pull heads. KEEP applies only
to the reviewed source/test/tooling patch; it does not retain old checkpoint
authority or qualify the behavior. DUPLICATE requires patch equivalence.
DEFER remains work for ROADMAP R0, with exact SHA and paths in the local bundle.
REJECT records a reverted or unsuitable intermediate patch.

[Document mapping](documentation_manifest.csv) names exact retired paths,
content hashes, Git state and retained design section or historical backup.
A file removed from the active repository is recoverable from the package;
legacy line citations do not become citations into the new baseline.

## Interface changes selected for review

Relative to the old main, the candidate adds the CLEARANCE_EXCEEDED command-reason enum,
typed planning failure/rejection causes and braking/yaw/bias evidence fields.
PX4 bias subscription and frame-transform helpers are diagnostic-only; no new
continuous product transform authority is published. Old numeric diagnostic
codes must be interpreted with producer/source identity, not remapped blindly.
The public message/mission interfaces otherwise retain their existing owners.

The migration repairs COMPLETE-receipt handover failure reporting without
changing the three-attempt limit, stationary safety stream or PX4 mode-selection
owner. PlanningWorker test synchronization is a test-harness correction, not
runtime scheduling policy. Every safety correction remains unqualified until
its declared repeated runtime/recorded-data evidence exists.

## Evidence boundary

Fresh build/test/gate logs and independent review are retained in the local
package. The migration is accepted only after full-package gate PASS, backup
restore/checksum verification and public-tree review. New root-history mode
must run every product ROS package even with no source diff. The pre-existing
Makefile acceptance scope is 23 packages; 12 opt-in upstream example packages
are outside it. The separate 35-package diagnostic run exposed upstream
example lint failures and is retained as FAIL, not relabeled PASS.

This baseline retains open liveness/physical/frame/visibility/dependency debt.
A selected component fix can be implemented without being flight-qualified.
No gate threshold, UNKNOWN policy, command lease or runtime parameter is tuned
by the documentation cleanup. The new root commit is a packaging boundary,
not an assertion of mission or hardware acceptance.

The external `/lio/reset` service (923cdc1) and interior optimizer-duration
retry family (a98d131, 8ab34d6) are DEFERRED and their source hunks removed.
Reset publication fencing/topic-prior rearm and retry deadline/cancellation
proof are explicit R0 tasks. Existing worker/core reset helpers do not expose
that deferred service. No distribution or flight acceptance is inferred.

[Per-hunk disposition](hunk_disposition.csv) records historical inputs; the
migration safety correction patch in backup records final overrides. U1/U2
computations and background telemetry are unavailable; diagnostic producers and
raw artifact capture remain. Temporary patch/revert pairs are REJECTED.
Before replacement run `python3 tools/verify_baseline_migration.py --backup
/home/letandat/Dev/uav-navigation-backup-20261005` in the active workspace.
This checks code preservation/mapping/references; qualification debt is separate.

## Root-commit gate scope

The first root-mode gate run (56fefa7) failed the static whitespace check on
61 unmodified upstream and legacy files that a root diff presents as added.
Three first-party planning headers were normalized (whitespace only). The
upstream copies listed in safety record TB-006 are skipped by that one check;
px4_msgs, px4_ros2_interface_lib and all first-party paths remain covered.
