# Runtime configuration truth — 2026-09-24

Owner: runtime runner, NavigationRuntimeNode, and PX4 External Mode adapter.
Scope: SITL tracking experiment selection and evidence provenance. This entry
revisits the [2026-09-09 zero-gate decision](archive/runtime_safety_legacy_full.md#2026-09-09---make-gps-on-558-and-zero-disabled-tracking-gates-the-run-defaults).

The former C++ loader treated `use_sim_time=true` as experiment enabled and
zero tracking coefficients as permission to suppress tracking-triggered braking
and fresh typed estimator-health response. The runner could request `off`, but
its generated ROS parameters contained only zero coefficients, so Core and
adapter executed the relaxed diagnostic policy. A matched 2026-09-24
`long_featured` run completed while the report was `NOT_EVALUABLE` for
`config_mismatch`; completion does not validate the disabled gates.

The new contract makes `tracking_experiment.mode` the explicit selector.
`off` disables the experiment regardless of `use_sim_time`; `relaxed` selects
the old suppression deliberately. Other experiment modes still require
`use_sim_time=true`. Both nodes log their effective policy. The runner checks
the exact requested/effective mode, enabled, suppression, and velocity-only
fields before mission start. An absent or mismatched witness fails setup.

Safety impact: tracking and typed-health responses are no longer silently
suppressed during a requested tracking-off SITL run. Such runs may fail earlier
for a real tracking/health condition; this is a safety response, not a mission
liveness regression to mask with a threshold change. Explicit `relaxed` runs
retain the historical increased collision risk and remain diagnostic only.
No tracking limit, command lease, world freshness, planner deadline, or PX4
Hold policy changes here.

Evidence: source comparison in
`artifacts/major_runtime_boundary_hardening/20260924T065002Z-68e718d4/PHASE_A_CONFIG_TRUTH.md`,
focused C++/Python contract tests, and three true tracking-off SITLs. All
three had matching live Core/adapter witnesses. Mission outcome was 2/3
COMPLETE; one run fail-closed on adapter navigation-odometry receive age
208.583 ms at the unchanged 200 ms boundary. Every evaluator report remained
FAIL/NOT_EVALUABLE for separate attribution/policy gaps. This is partial
Phase A evidence, not a nominal parity or qualification pass.
Removal condition: remove the diagnostic suppression mode only after an
independent product policy decision and representative evidence; never promote
it from a completed diagnostic mission alone.

Verification: `python3 tools/runtime/build.py --mode release build`,
`python3 tools/runtime/build.py --mode release test`,
`python3 -m unittest tools.runtime.tests.test_runtime_contract`,
`python3 tools/validate_runtime_safety_ledger.py`, `git diff --check`, and
three matched `long_featured` tracking-off SITL runs with both effective
configuration witnesses and evaluator output retained.
