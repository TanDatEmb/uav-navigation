# Path-relative tracking implementation

Status: `IN_PROGRESS` — development characterization only.

The runtime now has a bounded local path-relative witness for an active MAIN
analytic bundle with an optional certified BACKUP suffix. It searches the
declared MAIN curve around
the execution state's source timestamp, selects a unique local closest point,
and reports:

- 3D path/contour error, cross-track error and vertical error;
- signed phase offset and measured progress rate;
- constant-velocity predicted path error and phase drift;
- projected/source timestamps and bounded analytic evaluation count.

The witness is used only outside the world/command owner locks. The final
retained-command transaction prepares the latest state outside owner locks,
then rechecks freshness, owner identity, lease age and localization epoch
before preserving MAIN. It does not retime the trajectory, extend the
executable lease, activate BACKUP, or alter PX4/PVA setpoints. Geometric lookback can use the declared MAIN prefix before
activation, while current/future execution remains lease-bound.

The unchanged `0.25 m` tracking budget is the local contour budget. The
existing absolute command-anchor cap is the base guard; a bounded
longitudinal allowance derived from the existing phase window and
reference/measured speed prevents expected along-path phase from being
mistaken for absolute divergence. Lateral/vertical divergence still uses the
base guard, and all world, dynamic, flatness, freshness and lifecycle gates
remain active. Ambiguous branches, MAIN/BACKUP seams, stale/reverse/nonfinite
input and out-of-window phase are rejected.

Component evidence is covered by `test_path_relative_tracking`; end-to-end
Q1 evidence is still required before this behavior is considered stable.
